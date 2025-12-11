#!/usr/bin/env python3
import math
import os
import time
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from dragonpilot.selfdrive.controls.lib.acm import ACM
from dragonpilot.selfdrive.controls.lib.aem import AEM

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

class DPFlags:
  ACM = 1
  AEM = 2
  pass

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    # TODO remove mpc modes when TR released
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.acm = ACM()
    self.aem = AEM()
    # Lincoln/Ford 专用弯道限速状态
    self.curve_k_smooth = 0.0
    self.curve_active = False
    self.curve_v_target = None
    self.curve_exit_timer = 0.0
    self.last_curve_log_t = 0.0
    self.curve_log_dir = "/data/media/0/lincoln_curve_logs"

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def _apply_lincoln_curve_speed(self, sm, accel_clip, log_enabled: bool):
    """
    Ford/Lincoln 专用弯道减速（更稳健的前瞻 + 滞回）：
    - 取模型曲率在前方窗口内的最大值并平滑
    - 以舒适侧向加速度上限计算目标速度，带进入/退出滞回，避免出口抖动
    - 提前按照等效减速度收紧纵向上限（可能为负）实现预减速
    """
    model = sm['modelV2']
    car_state = sm['carState']
    v_ego = car_state.vEgo

    # 基础检查
    if len(model.position.x) != ModelConstants.IDX_N or len(model.orientationRate.z) != ModelConstants.IDX_N:
      if log_enabled:
        self._log_curve(reason="model_invalid")
      return accel_clip
    if v_ego < 1.0:
      if log_enabled:
        self._log_curve(reason="low_speed", v=v_ego)
      return accel_clip

    # 取 MPC 时间轴上的预测值
    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.velocity.x)
    turn_rates = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.orientationRate.z)
    positions = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.position.x)

    # 计算曲率，并截断到车辆可下发的信号范围
    curvatures = np.abs(turn_rates / np.clip(v_pred, 1.0, 100.0))
    curvatures = np.clip(curvatures, 0.0, 0.02)  # Ford 非 CAN FD 信号上限
    window_mask = positions <= 80.0  # 关注前方 80m
    if not np.any(window_mask):
      if log_enabled:
        self._log_curve(reason="no_window_points")
      return accel_clip

    k_window = curvatures[window_mask]
    pos_window = positions[window_mask]
    k_max = float(np.max(k_window))
    critical_idx = int(np.argmax(k_window))
    critical_distance = float(pos_window[critical_idx])
    # 原始模型曲率（未截断）与粗略置信度（std 或 0）
    k_window_raw = np.abs(turn_rates / np.clip(v_pred, 1.0, 100.0))[window_mask]
    k_model_max = float(np.max(k_window_raw)) if k_window_raw.size else 0.0
    try:
      orient_std = getattr(model.orientationRateStd, "z", [])
      k_std_window = np.abs(np.array(orient_std)) / np.clip(v_pred, 1.0, 100.0)
      k_model_std = float(np.max(k_std_window[window_mask])) if len(k_std_window) == len(v_pred) else 0.0
    except Exception:
      k_model_std = 0.0

    if k_max < 1e-4 or critical_distance < 5.0:
      if log_enabled:
        self._log_curve(reason="k_too_small", k_max=k_max, dist=critical_distance, v=v_ego)
      return accel_clip

    # 平滑曲率，进入/退出滞回，减弱抖动
    alpha = 0.3
    self.curve_k_smooth = alpha * k_max + (1 - alpha) * self.curve_k_smooth
    k_enter = 0.006  # 触发阈值（~半径 166 m）
    k_exit = k_enter * 0.6
    if self.curve_active:
      if self.curve_k_smooth < k_exit:
        # 退出滞回：计时 0.5s 再退出，避免出口抖动
        self.curve_exit_timer += self.dt
        if self.curve_exit_timer > 0.5:
          self.curve_active = False
          self.curve_exit_timer = 0.0
      else:
        self.curve_exit_timer = 0.0
    else:
      if self.curve_k_smooth >= k_enter:
        self.curve_active = True
        self.curve_exit_timer = 0.0

    if not self.curve_active:
      if log_enabled:
        self._log_curve(reason="below_enter", k=k_max, k_smooth=self.curve_k_smooth, v=v_ego)
      return accel_clip

    # 目标侧向加速度上限（舒适 1.6 m/s^2），带安全系数
    a_lat_limit = 1.6 * 0.9
    v_limit = math.sqrt(a_lat_limit / max(self.curve_k_smooth, 1e-4))

    # 已低于限速则不动作
    if v_ego <= v_limit * 1.05:
      if log_enabled:
        self._log_curve(reason="under_limit", v=v_ego, v_limit=v_limit, k=self.curve_k_smooth)
      return accel_clip

    # 等效减速度：a = (v_f^2 - v_i^2) / (2*d)
    required_decel = (v_limit ** 2 - v_ego ** 2) / max(2 * critical_distance, 1.0)
    required_decel = max(required_decel, -3.0)  # 不超过 -3 m/s^2，舒适范围

    # 收紧最大加速度（上限），促使 MPC 提前减速
    accel_clip[1] = min(accel_clip[1], required_decel)
    # 同步写入 MPC 约束，保证在关键距离前的时间步均受限
    for i in range(len(T_IDXS_MPC)):
      t = T_IDXS_MPC[i]
      distance_at_t = v_ego * t + 0.5 * required_decel * t**2
      if distance_at_t < critical_distance:
        self.mpc.params[i, 1] = min(self.mpc.params[i, 1], required_decel)
    mpc_a_max_min = float(np.min(self.mpc.params[:, 1]))

    if log_enabled:
      curv_cmd = getattr(sm['carControl'].actuators, "curvature", 0.0) if sm['carControl'].actuators is not None else 0.0
      accel_cmd = getattr(sm['carControl'].actuators, "accel", 0.0) if sm['carControl'].actuators is not None else 0.0
      curv_current = - (car_state.yawRate if hasattr(car_state, "yawRate") else 0.0) / max(v_ego, 0.1)
      steer_torque = getattr(car_state, "steeringTorque", 0.0)
      steer_fault_temp = getattr(car_state, "steerFaultTemporary", False)
      steer_fault_perm = getattr(car_state, "steerFaultPermanent", False)
      sensors_invalid = getattr(car_state, "vehicleSensorsInvalid", False)
      steer_override = getattr(car_state, "steeringPressed", False)
      ctrl_active = getattr(sm['controlsState'], "active", False)
      ctrl_long_active = getattr(sm['controlsState'], "longActive", False)
      ctrl_lat_active = getattr(sm['controlsState'], "latActive", False)
      ctrl_alert_type = getattr(sm['controlsState'], "alertType", 0)
      ctrl_alert_size = getattr(sm['controlsState'], "alertSize", 0)
      ctrl_alert_sound = getattr(sm['controlsState'], "alertSound", 0)
      cs_enabled = getattr(sm['carState'].cruiseState, "enabled", False)
      cs_available = getattr(sm['carState'].cruiseState, "available", False)
      cs_speed = getattr(sm['carState'].cruiseState, "speed", 0.0)
      self._log_curve({
        "reason": "active",
        "v_ego": v_ego,
        "v_limit": v_limit,
        "k_smooth": self.curve_k_smooth,
        "k_max": k_max,
        "k_model_max": k_model_max,
        "k_model_std": k_model_std,
        "distance": critical_distance,
        "required_decel": required_decel,
        "accel_clip_max": accel_clip[1],
        "yaw_rate": car_state.yawRate if hasattr(car_state, "yawRate") else 0.0,
        "steer_angle": car_state.steeringAngleDeg if hasattr(car_state, "steeringAngleDeg") else 0.0,
        "lat_active": getattr(sm['controlsState'], "latActive", False),
        "alerts": getattr(sm['controlsState'], "alertText1", "") if hasattr(sm['controlsState'], "alertText1") else "",
        "curv_cmd": curv_cmd,
        "curv_current": curv_current,
        "steer_pressed": steer_override,
        "gas_pressed": getattr(car_state, "gasPressed", False),
        "brake_pressed": getattr(car_state, "brakePressed", False),
        "accel_cmd": accel_cmd,
        "accel_actual": getattr(car_state, "aEgo", 0.0),
        "mpc_a_max_min": mpc_a_max_min,
        "v_desired": self.v_desired_filter.x,
        "v_plan0": self.v_desired_trajectory[0] if len(self.v_desired_trajectory) else 0.0,
        "steer_torque": steer_torque,
        "steer_fault_temp": steer_fault_temp,
        "steer_fault_perm": steer_fault_perm,
        "sensors_invalid": sensors_invalid,
        "ctrl_active": ctrl_active,
        "ctrl_long_active": ctrl_long_active,
        "ctrl_lat_active": ctrl_lat_active,
        "ctrl_alert_type": ctrl_alert_type,
        "ctrl_alert_size": ctrl_alert_size,
        "ctrl_alert_sound": ctrl_alert_sound,
        "cs_enabled": cs_enabled,
        "cs_available": cs_available,
        "cs_speed": cs_speed,
      })
    return accel_clip

  def _log_curve(self, data: dict = None, reason: str = "", force: bool = False, **kwargs):
    now = time.monotonic()
    if not force and now - self.last_curve_log_t < 0.5:
      return
    self.last_curve_log_t = now
    try:
      os.makedirs(self.curve_log_dir, exist_ok=True)
      ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
      if data is None:
        data = {}
      if kwargs:
        data.update(kwargs)
      if reason:
        data["reason"] = reason
      fields = [
        ts,
        data.get('reason', ''),
        f"{data.get('v_ego', 0):.2f}",
        f"{data.get('v_limit', 0):.2f}",
        f"{data.get('k_smooth', 0):.5f}",
        f"{data.get('k_max', 0):.5f}",
        f"{data.get('k_model_max',0):.5f}",
        f"{data.get('k_model_std',0):.5f}",
        f"{data.get('distance', 0):.1f}",
        f"{data.get('required_decel', 0):.2f}",
        f"{data.get('accel_clip_max', 0):.2f}",
        f"{data.get('mpc_a_max_min', 0):.2f}",
        f"{data.get('curv_cmd',0):.5f}",
        f"{data.get('curv_current',0):.5f}",
        f"{data.get('accel_cmd',0):.2f}",
        f"{data.get('accel_actual',0):.2f}",
        f"{data.get('v_desired',0):.2f}",
        f"{data.get('v_plan0',0):.2f}",
        f"{data.get('yaw_rate', 0):.3f}",
        f"{data.get('steer_angle', 0):.2f}",
        f"{data.get('steer_torque',0):.2f}",
        str(data.get('steer_pressed', False)),
        str(data.get('gas_pressed', False)),
        str(data.get('brake_pressed', False)),
        str(data.get('steer_fault_temp', False)),
        str(data.get('steer_fault_perm', False)),
        str(data.get('sensors_invalid', False)),
        str(data.get('lat_active', False)),
        str(data.get('ctrl_active', False)),
        str(data.get('ctrl_long_active', False)),
        str(data.get('ctrl_lat_active', False)),
        str(data.get('ctrl_alert_type', 0)),
        str(data.get('ctrl_alert_size', 0)),
        str(data.get('ctrl_alert_sound', 0)),
        str(data.get('cs_enabled', False)),
        str(data.get('cs_available', False)),
        f"{data.get('cs_speed', 0.0):.2f}",
        f"\"{data.get('alerts','')}\"",
      ]
      line = ",".join(fields) + "\n"
      fname = os.path.join(self.curve_log_dir, time.strftime("curve_%Y%m%d.log", time.localtime()))
      with open(fname, "a", encoding="utf-8") as f:
        f.write(line)
    except Exception as e:
      cloudlog.error(f"curve log failed: {e}")

  def update(self, sm, dp_flags = 0, lincoln_curve_speed: bool = False, lincoln_curve_log: bool = False):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if dp_flags & DPFlags.AEM:
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if mode == 'acc':
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if lincoln_curve_speed:
      accel_clip = self._apply_lincoln_curve_speed(sm, accel_clip, log_enabled=lincoln_curve_log)

    # 重要状态变化/告警即时记录（不受节流影响）
    if lincoln_curve_log:
      cs = sm['carState']
      ctrl = sm['controlsState']
      alert_text = getattr(ctrl, "alertText1", "") if hasattr(ctrl, "alertText1") else ""
      if (hasattr(ctrl, "latActive") and not ctrl.latActive) or alert_text:
        curv_current = - (cs.yawRate if hasattr(cs, "yawRate") else 0.0) / max(cs.vEgo, 0.1)
        self._log_curve({
          "reason": "alert_or_lat_off",
          "v_ego": cs.vEgo,
          "k_current": curv_current,
          "steer_torque": getattr(cs, "steeringTorque", 0.0),
          "steer_pressed": getattr(cs, "steeringPressed", False),
          "steer_fault_temp": getattr(cs, "steerFaultTemporary", False),
          "steer_fault_perm": getattr(cs, "steerFaultPermanent", False),
          "sensors_invalid": getattr(cs, "vehicleSensorsInvalid", False),
          "alerts": alert_text,
          "ctrl_active": getattr(ctrl, "active", False),
          "ctrl_long_active": getattr(ctrl, "longActive", False),
          "ctrl_lat_active": getattr(ctrl, "latActive", False),
          "ctrl_alert_type": getattr(ctrl, "alertType", 0),
          "ctrl_alert_size": getattr(ctrl, "alertSize", 0),
          "ctrl_alert_sound": getattr(ctrl, "alertSound", 0),
          "cs_enabled": getattr(cs.cruiseState, "enabled", False),
          "cs_available": getattr(cs.cruiseState, "available", False),
          "cs_speed": getattr(cs.cruiseState, "speed", 0.0),
        }, force=True)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    # ACM - Adaptive Coasting Module
    if dp_flags & DPFlags.ACM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
