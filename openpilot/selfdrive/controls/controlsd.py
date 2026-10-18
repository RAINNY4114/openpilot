#!/usr/bin/env python3
import math
from numbers import Number

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.lateral_clearance import LateralClearance
from openpilot.selfdrive.controls.lib.curve_lane_bias import CurveLaneBias
from openpilot.selfdrive.controls.lib.lane_trim_governor import LaneTrimGovernor
from openpilot.selfdrive.controls.lib.model_curv_governor import ModelCurvatureGovernor
from openpilot.selfdrive.controls.lib.lateral_planner_avoid import ObstacleAvoidance
from openpilot.selfdrive.controls.lib.ford_curve_controller import FordPlanLead
from openpilot.selfdrive.controls.lib.curve_bend import (bend_from_model, bend_deg_min,
                                                          bend_deg_min_lat, bend_weight)
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.aa_brake_apply import AABrakeApply
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['lateralDelay', 'vehicleParameters', 'lateralTorqueParameters', 'modelV2', 'selfdriveState',
                                   'extrinsicsCalibration', 'deviceMotion', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    # lane-centering trim + lateral clearance bias (barrier / adjacent large vehicle)
    # disabled by default -> zero behaviour change until /data/lateral_clearance.json enables it
    self.lateral_clearance = LateralClearance()

    # 弯道内圈偏移：直道居中、进弯后随曲率比例往内侧挪（默认开启，见 /data/curve_lane_bias.json）
    self.curve_lane_bias = CurveLaneBias()
    # 通用静止障碍避让（故障车/落石/锥桶）：>80km/h 自动关闭，交给紧急转向避让
    # disabled by default -> /data/lateral_planner_avoid.json 里 enabled=true 才生效
    self.lateral_planner_avoid = ObstacleAvoidance()
    # 三路叠加修正的统一整形器（低通 -> 速率限制 -> 幅值限制）
    self.trim_governor = LaneTrimGovernor()
    # 模型直出曲率(modelV2.action.desiredCurvature)的整型器：
    # 只压 0.2~0.5 Hz 的模型输出噪声（实测占转向功率 ~17%，即高速左右
    # 摇晃的指令源），真实道路曲率(0.03 Hz)与真实弯道/变道完全不受影响。
    self.model_curv_gov = ModelCurvatureGovernor()
    self.dp_ford_lead = FordPlanLead()

    # [AA_BRAKE_FALLBACK] 紧急刹车纵向落地通道。
    # 把 auto_avoidance.py 产出的 brake_request（经 modeld 镜像到
    # /data/params/d/AvoidBrakeReq）转成 a_target 的负向修正量。
    # disabled by default（/data/aa_brake_apply.json 里 enabled=true 才生效）
    self.aa_brake_apply = AABrakeApply()

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      self.LaC = LatControlCurvature(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    self.LaC = ControlsExt.initialize_lateral_control(self, self.LaC, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["extrinsicsCalibration"]:
      self.pose_calibrator.feed_extrinsics_calibration(self.sm['extrinsicsCalibration'])
    if self.sm.updated["deviceMotion"]:
      device_motion = Pose.from_device_motion(self.sm['deviceMotion'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_motion)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['vehicleParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['lateralTorqueParameters']
      if self.sm.all_checks(['lateralTorqueParameters']) and torque_params.useParams:
        self.LaC.update_torque_parameters(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)

    # ------------------------------------------------------------------
    # [AA_BRAKE_FALLBACK] 紧急刹车纵向落地
    #
    # auto_avoidance.py 在「无安全逃逸车道 + 风险已达 emergency_active」
    # 时会产出一个减速度请求，经 modeld 镜像到
    # /data/params/d/AvoidBrakeReq。本模块把它转成 a_target 的负向修正：
    #
    #     a_target_final = a_target_plan + delta_a      (delta_a <= 0)
    #
    # 安全边界（务必保留）：
    #   - delta_a 恒 <= 0：本通道**只能让目标更负（更接近刹车）**，
    #     绝不会让车加速，因此不存在"误加速"风险。
    #   - 仅当 CC.longActive 为真时介入；未使能纵向输出恒 0。
    #   - 模块内部默认关闭（config 缺失即 enabled=false），另有
    #     observe_only / 硬限幅 / 速率限制 / 请求超时守卫。
    #   - 整个调用包 try/except，任何异常都回落为不介入。
    #   - 结果再与 plan 的 aTarget 取 min，双保险。
    # ------------------------------------------------------------------
    _a_target_plan = float(long_plan.aTarget)
    try:
      _aa_delta_a = float(self.aa_brake_apply.update(v_ego=CS.vEgo, long_active=bool(CC.longActive)))
    except Exception:
      _aa_delta_a = 0.0
    if _aa_delta_a < 0.0:
      _a_target_plan = min(_a_target_plan, _a_target_plan + _aa_delta_a)

    actuators.accel = float(self.LoC.update(CC.longActive, CS, _a_target_plan, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    if self.sm.valid['lateralManeuverPlan']:
      new_desired_curvature = self.sm['lateralManeuverPlan'].desiredCurvature if CC.latActive else self.curvature
    else:
      new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    # dp_ford Tier 1: adaptive model-plan curvature lead. Default OFF (dp_ford_lead_enable).
    # Inserted BEFORE the lateral_clearance / curve_lane_bias trims so those
    # centering corrections are added at full strength afterwards, instead of
    # being scaled by (1 - b) along with the planner term.
    # 弯道闸门：bend_from_model 是纯函数，各进程各自用同一份 modelV2 调用，
    # 结果按构造一致，无需跨进程传状态。
    # 横向前瞻用独立的、更低的闸门（bend_deg_min_lat，默认 10 度），
    # 不再和纵向弯道减速共用 bend_deg_min（默认 45 度）——缓弯不需要减速，
    # 但横向确实需要提前打方向补 400 ms 延迟。
    # 闸门做成 0..1 软权重（smoothstep），避免阈值处给转向一个阶跃。
    # 计算异常时不闸门（保持旧行为），避免整体失效。
    try:
      _bend_deg = float(bend_from_model(model_v2, CS.vEgo))
      _bend_gate_lat = bend_deg_min_lat()
      _bend_ok = bool(_bend_deg >= _bend_gate_lat)
      _bend_w = bend_weight(_bend_deg, _bend_gate_lat)
    except Exception:
      _bend_ok = True
      _bend_w = 1.0
    new_desired_curvature = self.dp_ford_lead.apply(
      model_v2, CS.vEgo, new_desired_curvature, self.sm['lateralDelay'].lateralDelay,
      active=bool(CC.latActive), bend_ok=_bend_ok, bend_w=_bend_w)
    # 2026-10-18: 模型直出曲率整型。放在 FordPlanLead 之后、三路叠加修正之前，
    # 让"主项"和"修正项"都建立在同一个平滑过的曲率基础上。
    # 实测（seg8, >50km/h）：未整型前动作曲率 std=6.0e-4 1/m，其中 0.24/0.49 Hz
    # 分量是模型输出噪声，舵角 pp50=4.7°，指令->实际滞后 280 ms，即高速左右
    # 摇晃的指令源。整型只在小信号区生效，真实弯道/变道直通。
    try:
      _lane_change = (model_v2.meta.laneChangeState != LaneChangeState.off)
    except Exception:
      _lane_change = False
    new_desired_curvature = self.model_curv_gov.update(
      new_desired_curvature, CS.vEgo, bool(CC.latActive), bool(_lane_change), DT_CTRL)
    # 2026-10-18: 三路横向叠加修正先求和，再过统一整形器。
    #
    # 三路各自都是「车道中心误差 -> 曲率」的位置环，各带积分器和饱和，
    # 作用于同一个误差。并联后总积分增益翻倍、饱和点不一致，实车表现为
    # 车道内左右摇晃且越快越厉害。这里把三者合成一路，由 LaneTrimGovernor
    # 统一做 低通 -> 速率限制 -> 幅值限制（见 lane_trim_governor.py）。
    _trim_raw = 0.0
    # 2026-10-05: 传入 steering_pressed, 与 curve_lane_bias 的让权判据对齐
    _trim_raw += self.lateral_clearance.update(
      CC.latActive, model_v2, CS.vEgo, DT_CTRL,
      steering_pressed=bool(CS.steeringPressed))
    # 弯道内圈偏移：变道过程中不介入，避免跟变道抢方向
    _trim_raw += self.curve_lane_bias.update(
      CC.latActive, model_v2, CS.vEgo, DT_CTRL,
      lane_change_active=model_v2.meta.laneChangeState != LaneChangeState.off,
      steering_pressed=bool(CS.steeringPressed))
    # 通用静止障碍避让：返回曲率修正量（1/m）；变道中不介入，避免抢方向
    _trim_raw += self.lateral_planner_avoid.update(
      CC.latActive, model_v2, CS.vEgo, DT_CTRL, sm=self.sm,
      lane_change_active=model_v2.meta.laneChangeState != LaneChangeState.off)
    new_desired_curvature += self.trim_governor.update(
      _trim_raw, CS.vEgo, bool(CC.latActive), DT_CTRL)
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, lateral_output, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                     self.steer_limited_by_safety, self.desired_curvature,
                                                     self.calibrated_pose, curvature_limited, lat_delay)
    actuators.torque = float(steer)
    if self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      actuators.curvature = float(lateral_output)
    else:
      actuators.steeringAngleDeg = float(lateral_output)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.get_lat_active(self.sm):
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool(self.sm['driverMonitoringState'].noResponseForceDecel or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    # trigger the car's stock driver monitoring escalation
    CC.driverMonitoringEscalation = cs.forceDecel

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      cs.lateralControlState.curvatureState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      self.get_params_sp(self.sm)
      self.run_ext(self.sm, self.pm)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
