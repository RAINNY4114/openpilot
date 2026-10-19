#!/usr/bin/env python3
import math
import time
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.selfdrive.controls.lib.ford_curve_speed import FordCurveController, _read_param_bool_direct
from openpilot.selfdrive.controls.lib.ford_curve_speed_fusion import FordCurveSpeedFusion
from openpilot.selfdrive.controls.lib.curve_speed_tsc import CurveSpeedTSC
from openpilot.selfdrive.controls.lib.nn import FordNNCurveControl  # [CHX_NN 20261019]
from openpilot.common.params import Params

A_CRUISE_MAX_VALS = [2.2, 1.6, 1.2, 0.9, 0.75, 0.62, 0.58, 0.50, 0.45,  0.40]
A_CRUISE_MAX_BP = [0.,  3,   5.,  7.,  11., 15.,  20.,  25.,  30.,  55.]
J_CRUISE_VALS = [2.2, 1.8, 1.4, 1.1, 0.95, 0.80, 0.70, 0.65, 0.60, 0.55]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt, CP=self.CP)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    # Ford/Lincoln dynamic curve speed controller
    self._ford_curve = FordCurveController(self.CP, dt)
    self._curve_speed_fusion = FordCurveSpeedFusion()
    self._params = Params()

    # 弯道减速（Ford 分支 V-TSC / M-TSC 移植）。
    # 只读 modelV2.orientationRate.z * velocity.x，不改神经网络链路。
    self._tsc = CurveSpeedTSC()
    self._tsc_active = False
    self._tsc_takeover = False

    # [CHX_NN_LONG 20261019] NN 弯道横向加速度预算腿（controlsd 进程：纵向从属，
    # 不学风格，每 30s 读一次 card 侧写好的 style.json）。mode<3 时 a_lat_budget=None，
    # 本腿完全静默。
    self.nn_curve = FordNNCurveControl(
      config={"style_enabled": False, "style_reload_period_s": 30.0},
    )

    # 对外发布的弯道减速真实值
    # -> cereal: LongitudinalPlanSP.curveSpeedTarget / curveSpeedActive
    self.curve_speed_target = 0.0
    self.curve_speed_active = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    # Ford/Lincoln dynamic curve speed control
    map_turn_limit_active = False
    map_a_target = 0.0
    if getattr(self.CP, "brand", "") == "ford":
      # 弯道减速：视觉(模型曲率) + 地图(OSM) 双源，**默认开启**。
      # 参数文件存在时以文件为准（写 "0" 即可关闭）；文件缺失时用代码默认值兜底，
      # 因为 manager 的 Params::clearAll() 会删掉所有未注册进已编译 params_keys.h 的 key。
      lincoln_curve_speed = _read_param_bool_direct("dp_lincoln_curve_speed", default=True)
      lincoln_osm_realtime_cruise = _read_param_bool_direct("dp_lincoln_osm_realtime_cruise", default=True)

      # TSC 接管时，旧的两条弯道腿让位，避免三套减速逻辑并联、
      # 各自按不同判据抢 v_cruise（这是"重新整理"的核心一步）。
      _tsc_cfg = self._tsc._cfg()
      self._tsc_takeover = bool(_tsc_cfg["mode"] > 0 and _tsc_cfg["replace"])
      if self._tsc_takeover:
        lincoln_curve_speed = False
        lincoln_osm_realtime_cruise = False

      # ---- 弯道减速（Ford 分支移植）----
      # 2026-10-18 调整顺序：TSC 移到旧腿**之前**。
      #
      # 原来 TSC 在 _ford_curve.update() 之后跑，拿到的 v_cruise 已经被旧腿压过，
      # 于是 TSC 要跟"已经被压过的目标"比 min —— 实车实测 active 只占 4.3%，
      # 比同样喂原始 v_cruise 的模拟(11.1%)低一倍多。旧腿的判据(OSM/几何)和
      # TSC(模型曲率)不是一回事，谁先算谁就抢到了"最低目标"的名义。
      # 换序后：TSC 拿驾驶员设定速度，旧腿再对 TSC 的结果取 min，
      # 三条弯道腿都还是 min 关系（v_cruise 只会更保守），但 active 标志
      # 和 HUD 上的弯道目标速度恢复为真。
      _gps = None
      if _tsc_cfg["mode"] >= 2 and 'liveLocationKalman' in sm.data:
        try:
          _llk = sm['liveLocationKalman']
          if _llk.gpsOK and _llk.positionGeodetic.valid:
            _gps = (float(_llk.positionGeodetic.value[0]),
                    float(_llk.positionGeodetic.value[1]))
        except Exception:
          _gps = None
      _tsc_enabled = bool(sm['selfdriveState'].enabled) and not long_control_off
      v_cruise, self._tsc_active = self._tsc.update(
        sm['modelV2'] if 'modelV2' in sm.data else None,
        v_ego, v_cruise,
        enabled=_tsc_enabled,
        left_blinker=bool(sm['carState'].leftBlinker),
        right_blinker=bool(sm['carState'].rightBlinker),
        gps=_gps)

      v_cruise, map_turn_limit_active, map_a_target = self._ford_curve.update(
        sm, v_ego, v_cruise, reset_state, long_control_off, sm['selfdriveState'].enabled,
        lincoln_curve_speed=lincoln_curve_speed,
        lincoln_osm_realtime_cruise=lincoln_osm_realtime_cruise)

      # [CHX_NN_LONG 20261019] NN 弯道预算腿：v = sqrt(a_lat_budget / |kappa|)，与其余
      # 弯道腿保持 min 关系（只会更保守）。a_lat_budget 只在 mode>=3 时非 None。
      try:
        _nn_kappa = float(sm['modelV2'].action.desiredCurvature) if 'modelV2' in sm.data else 0.0
        _nn_out = self.nn_curve.update(
          t=time.monotonic(),
          kappa=_nn_kappa,
          v_ego=float(v_ego),
          lateral_active=bool(sm['selfdriveState'].enabled),
          a_ego=float(sm['carState'].aEgo),
          gas=float(getattr(sm['carState'], "gas", 0.0) or 0.0),
          override=bool(sm['carState'].steeringPressed),
        )
        _nn_budget = _nn_out.get("a_lat_budget")
        if _nn_budget is not None:
          _nn_abs_k = abs(_nn_kappa)
          if _nn_abs_k > 1e-4:
            v_cruise = min(v_cruise, math.sqrt(_nn_budget / _nn_abs_k))
      except Exception:
        self.nn_curve.disabled = True
        self.nn_curve.disabled_reason = "planner_exception"

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)

    # Ford curve speed fusion: vision + map predictive curve speed management
    if (not getattr(self, "_tsc_takeover", False)
        and hasattr(self, '_curve_speed_fusion') and self._curve_speed_fusion is not None):
      try:
        # 注意：SubMaster 没有 __contains__/__iter__，`'modelV2' in sm` 会退化成
        # 旧式迭代 -> sm[0] -> KeyError(0)，且会被下面的 except 静默吞掉，
        # 导致本段（FordCurveSpeedFusion）从未真正执行过。必须查 sm.data。
        model_msg = sm['modelV2'] if 'modelV2' in sm.data else None
        v_ego_f = float(sm['carState'].vEgo) if 'carState' in sm.data else 0.0
        scc_vision = getattr(self, 'scc_vision', None)
        scc_map = getattr(self, 'scc_map', None)
        fusion_a_max = self._curve_speed_fusion.update(v_ego_f, model_msg, scc_vision=scc_vision, scc_map=scc_map)
        if fusion_a_max is not None:
          self.output_a_target = min(self.output_a_target, float(np.min(fusion_a_max)))
      except Exception:
        pass

    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    # Apply Ford/Lincoln curve decel
    if getattr(self.CP, "brand", "") == "ford":
      curve_decel = self._ford_curve.get_curve_decel(long_control_off)
      if curve_decel is not None:
        self.output_a_target = min(self.output_a_target, curve_decel)

      # TSC 弯道制动请求。
      # 巡航通道的 target_accel 被 A_CRUISE_MIN(-1.2 m/s^2) 钳住，仅靠降
      # v_cruise 无法在弯道前把速度降下来。这里把 TSC 算出的 -dv/2（上限
      # dp_curve_tsc_decel_max，默认 2.0 m/s^2）作为 a_target 上界。
      if getattr(self, "_tsc_active", False):
        _a_tsc = float(getattr(self._tsc, "a_target", 0.0))
        if _a_tsc < -1e-3:
          self.output_a_target = min(self.output_a_target, _a_tsc)
      if map_turn_limit_active and not long_control_off and map_a_target < -1e-3:
        self.output_a_target = min(self.output_a_target, float(map_a_target))

      # --- 汇总"真正在起作用的弯道目标车速"，供 HUD 显示 ---
      # 只发布此刻确实在施加约束的源；多个源同时有效时取最保守（最小）的那个。
      # 注意：这里发布的是控制器自己的值，HUD 不再自己重算，显示与控制天然一致。
      curve_target = 0.0
      curve_active = False

      # (0) TSC（Ford 分支移植，视觉/地图）——优先级最高的发布源
      if getattr(self, "_tsc_active", False):
        _tv = float(self._tsc.v_target)
        if _tv > 0.1:
          curve_target = _tv
          curve_active = True

      # (a) FordCurveSpeedFusion —— TSC 接管时跳过，否则取小
      if not getattr(self, "_tsc_takeover", False):
        fusion = self._curve_speed_fusion
        if fusion is not None and bool(getattr(fusion, "active", False)):
          t = float(getattr(fusion, "target_speed_ms", 0.0) or 0.0)
          if t > 0.1:
            curve_target = t if curve_target <= 0.1 else min(curve_target, t)
            curve_active = True

      if not long_control_off:
        # (b) FordCurveController 视觉腿（dp_lincoln_curve_speed，当前关闭）
        fc_v = getattr(self._ford_curve, "curve_v_target", None)
        if fc_v is not None and float(fc_v) > 0.1:
          fc_v = float(fc_v)
          curve_target = fc_v if curve_target <= 0.1 else min(curve_target, fc_v)
          curve_active = True
        # (c) FordCurveController 地图腿（dp_lincoln_osm_realtime_cruise，当前关闭）
        if map_turn_limit_active and float(self._ford_curve.map_v_target) > 0.1:
          mv = float(self._ford_curve.map_v_target)
          curve_target = mv if curve_target <= 0.1 else min(curve_target, mv)
          curve_active = True

      self.curve_speed_target = float(curve_target)
      self.curve_speed_active = bool(curve_active)
    else:
      self.curve_speed_target = 0.0
      self.curve_speed_active = False

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
