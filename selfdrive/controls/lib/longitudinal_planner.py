#!/usr/bin/env python3
import json
import math
import os
import time
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.params import Params
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

# Map Turn Speed Controller (mapd) constants (similar to FrogPilot/CP behavior)
_MAP_EARTH_RADIUS_M = 6373000.0
_MAP_TO_RADIANS = math.pi / 180.0
_MAP_TARGET_JERK = -0.6   # m/s^3
_MAP_TARGET_ACCEL = -1.2  # m/s^2
_MAP_TARGET_OFFSET_S = 1.0

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
    self.curve_a_target = 0.0
    self.last_curve_log_t = 0.0
    self.curve_log_dir = "/data/media/0/lincoln_curve_logs"
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")
    self._curve_cfg = {}
    self._curve_param_last = 0.0

    self._map_target_velocities_raw = None
    self._map_target_velocities = []
    self._map_v_target = 0.0
    self._map_a_target = 0.0
    self._map_turn_limit_active = False
    self._map_data_available = False
    self._map_points = 0
    self._map_min_dist_m = 0.0
    self._map_lock_lat = 0.0
    self._map_lock_lon = 0.0
    self._map_lock_v = 0.0
    self._curve_speed_source = 0  # 0:none, 1:vision, 2:map (published to longitudinalPlan for HUD)

  def _lincoln_curve_config(self):
    # 小环折加载，1s 内不重复读取参数
    now = time.monotonic()
    if now - self._curve_param_last < 1.0 and self._curve_cfg:
      return self._curve_cfg

    def _safe_int(name: str, default: int) -> int:
      try:
        val = self._params.get(name)
        return int(val) if val is not None else default
      except Exception:
        return default

    window_m = max(30, min(190, _safe_int("dp_lincoln_curve_window_m", 130)))
    k_enter_milli = max(2, min(20, _safe_int("dp_lincoln_curve_k_enter", 4)))  # 0.002~0.020
    k_enter = k_enter_milli * 1e-3
    k_exit = k_enter * 0.70
    # 固定舒适横向上限 1.0 m/s²（不再暴露给用户调节）
    a_lat = 1.0
    decel_cm = _safe_int("dp_lincoln_curve_decel", -320)                       # cm/s^2, negative
    decel_max = min(-0.5, max(-500, decel_cm) / 100.0)                         # clamp to [-5.0, -0.5]
    self._curve_cfg = {
      "window_m": float(window_m),
      "k_enter": float(k_enter),
      "k_exit": float(k_exit),
      "a_lat": float(a_lat),
      "decel_max": float(decel_max),
      "exit_h": 0.70,  # 固定退出滞回，移除可调
    }
    self._curve_param_last = now
    return self._curve_cfg

  def _map_distance_to_point(self, lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    ax = lat_a * _MAP_TO_RADIANS
    ay = lon_a * _MAP_TO_RADIANS
    bx = lat_b * _MAP_TO_RADIANS
    by = lon_b * _MAP_TO_RADIANS
    a = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1 - a)))
    return _MAP_EARTH_RADIUS_M * c

  def _map_target_velocities_list(self) -> list:
    raw = None
    try:
      raw = self._params_memory.get("MapTargetVelocities")
    except Exception:
      raw = None

    if not raw:
      try:
        raw = self._params.get("MapTargetVelocities")
      except Exception:
        raw = None

    if not raw:
      self._map_target_velocities_raw = None
      self._map_target_velocities = []
      return []

    if raw == self._map_target_velocities_raw:
      return self._map_target_velocities

    try:
      parsed = json.loads(raw)
      if not isinstance(parsed, list):
        parsed = []
    except Exception:
      parsed = []

    self._map_target_velocities_raw = raw
    self._map_target_velocities = parsed
    return parsed

  @staticmethod
  def _map_calculate_velocity(t: float, target_jerk: float, a_ego: float, v_ego: float) -> float:
    return v_ego + a_ego * t + (target_jerk / 2) * (t ** 2)

  @staticmethod
  def _map_calculate_distance(t: float, target_jerk: float, a_ego: float, v_ego: float) -> float:
    return t * v_ego + (a_ego / 2) * (t ** 2) + (target_jerk / 6) * (t ** 3)

  def _map_turn_target_speed(self, v_ego: float, a_ego: float, lat: float, lon: float, v_cruise: float) -> tuple[float, float]:
    target_velocities = self._map_target_velocities_list()
    if not target_velocities:
      return 0.0, 0.0

    min_dist = 1e9
    min_idx = 0
    distances: list[float] = []

    for i, target_velocity in enumerate(target_velocities):
      try:
        tlat = float(target_velocity.get("latitude", target_velocity.get("lat", 0.0)))
        tlon = float(target_velocity.get("longitude", target_velocity.get("lon", target_velocity.get("lng", 0.0))))
      except Exception:
        distances.append(1e9)
        continue

      d = self._map_distance_to_point(lat, lon, tlat, tlon)
      distances.append(d)
      if d < min_dist:
        min_dist = d
        min_idx = i

    self._map_min_dist_m = float(min_dist) if math.isfinite(min_dist) else 0.0

    forward_points = target_velocities[min_idx:]
    forward_distances = distances[min_idx:]

    # 1) In-curve hold: use the current segment's map speed cap so we don't accelerate inside long curves/S-curves
    #    even when v_ego is already below the cap.
    v_hold = 0.0
    if min_dist < 30.0 and forward_points:
      try:
        tv_here = float(forward_points[0].get("velocity", forward_points[0].get("speed", 0.0)))
      except Exception:
        tv_here = 0.0
      if tv_here > 70.0:  # likely km/h
        tv_here *= CV.KPH_TO_MS
      if math.isfinite(tv_here) and tv_here > 0.1 and tv_here < (v_cruise - 1e-3):
        v_hold = float(tv_here)

    # 2) Approach decel: find upcoming target speeds that require deceleration *now* (physics check).
    valid_velocities: list[tuple[float, float, float, float]] = []  # (target_v, dist_m, lat, lon)
    for i, target_velocity in enumerate(forward_points):
      try:
        tv = float(target_velocity.get("velocity", target_velocity.get("speed", 0.0)))
      except Exception:
        continue
      if tv > 70.0:  # likely km/h
        tv *= CV.KPH_TO_MS
      if not math.isfinite(tv) or tv <= 0.0:
        continue
      if tv >= v_ego - 1e-3:
        continue

      d = float(forward_distances[i])

      a_diff = (a_ego - _MAP_TARGET_ACCEL)
      accel_t = abs(a_diff / _MAP_TARGET_JERK) if abs(_MAP_TARGET_JERK) > 1e-6 else 0.0
      min_accel_v = self._map_calculate_velocity(accel_t, _MAP_TARGET_JERK, a_ego, v_ego)

      max_d = 0.0
      if tv > min_accel_v:
        qa = 0.5 * _MAP_TARGET_JERK
        qb = a_ego
        qc = v_ego - tv
        disc = qb * qb - 4 * qa * qc
        if disc < 0.0 or abs(qa) < 1e-9:
          continue
        sqrt_disc = math.sqrt(disc)
        t_a = (-qb - sqrt_disc) / (2 * qa)
        t_b = (-qb + sqrt_disc) / (2 * qa)
        t = t_a if t_a > 0.0 else t_b
        if t <= 0.0 or math.isnan(t) or math.isinf(t):
          continue
        max_d += self._map_calculate_distance(t, _MAP_TARGET_JERK, a_ego, v_ego)
      else:
        max_d += self._map_calculate_distance(accel_t, _MAP_TARGET_JERK, a_ego, v_ego)
        t = abs((min_accel_v - tv) / _MAP_TARGET_ACCEL) if abs(_MAP_TARGET_ACCEL) > 1e-6 else 0.0
        max_d += self._map_calculate_distance(t, 0.0, _MAP_TARGET_ACCEL, min_accel_v)

      if d < max_d + tv * _MAP_TARGET_OFFSET_S:
        try:
          tlat = float(target_velocity.get("latitude", target_velocity.get("lat", 0.0)))
          tlon = float(target_velocity.get("longitude", target_velocity.get("lon", target_velocity.get("lng", 0.0))))
        except Exception:
          tlat = 0.0
          tlon = 0.0
        valid_velocities.append((float(tv), d, float(tlat), float(tlon)))

    # 3) Lock map target until passed (prevents jitter/early release due to GPS/path noise).
    lock_d = None
    if self._map_lock_v > 0.1 and forward_points:
      for i, target_velocity in enumerate(forward_points):
        try:
          tlat = float(target_velocity.get("latitude", target_velocity.get("lat", 0.0)))
          tlon = float(target_velocity.get("longitude", target_velocity.get("lon", target_velocity.get("lng", 0.0))))
          tv = float(target_velocity.get("velocity", target_velocity.get("speed", 0.0)))
        except Exception:
          continue
        if tv > 70.0:
          tv *= CV.KPH_TO_MS
        if (abs(tlat - self._map_lock_lat) < 1e-6 and abs(tlon - self._map_lock_lon) < 1e-6 and
            abs(float(tv) - float(self._map_lock_v)) < 1e-3):
          lock_d = float(forward_distances[i])
          break

    v_target = 0.0
    d_target = 0.0
    if valid_velocities:
      cand_v, cand_d, cand_lat, cand_lon = min(valid_velocities, key=lambda x: x[0])
      v_target = float(cand_v)
      d_target = float(cand_d)
      self._map_lock_v = float(cand_v)
      self._map_lock_lat = float(cand_lat)
      self._map_lock_lon = float(cand_lon)
    elif lock_d is not None:
      v_target = float(self._map_lock_v)
      d_target = float(lock_d)
    else:
      # Clear stale lock
      self._map_lock_v = 0.0
      self._map_lock_lat = 0.0
      self._map_lock_lon = 0.0

    # If no decel target is active, fall back to "in-curve hold" cap.
    if v_target <= 0.1 and v_hold > 0.1:
      return float(v_hold), 0.0

    if v_target <= 0.1:
      return 0.0, 0.0

    # Only request decel when we're above the target speed; otherwise it's a pure speed cap.
    if v_target >= v_ego - 1e-3:
      return float(v_target), 0.0

    d_use = max(d_target - v_ego * _MAP_TARGET_OFFSET_S, 1.0)
    required_decel = (v_target ** 2 - v_ego ** 2) / (2.0 * d_use)
    required_decel = min(0.0, float(required_decel))

    # Always request *some* decel once map says it's time to slow down.
    # Cap by the same comfort limit as the vision curve controller.
    decel_cap = self._lincoln_curve_config().get("decel_max", -3.2)
    map_a_target = max(float(decel_cap), min(-0.30, required_decel))

    return float(v_target), float(map_a_target)

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

  def _apply_lincoln_curve_speed(self, sm, accel_clip, v_cruise: float, log_enabled: bool):
    """
    Ford/Lincoln 专用弯道减速（更稳健的前瞻 + 滞回）：
    - 取模型曲率在前方窗口内的最大值并平滑
    - 以舒适侧向加速度上限计算目标速度，带进入/退出滞回，避免出口抖动
    - 提前按照等效减速度收紧纵向上限（可能为负）实现预减速
    """
    model = sm['modelV2']
    car_state = sm['carState']
    v_ego = car_state.vEgo
    self.curve_a_target = 0.0

    # 基础检查
    if len(model.position.x) != ModelConstants.IDX_N or len(model.orientationRate.z) != ModelConstants.IDX_N:
      if log_enabled:
        self._log_curve(reason="model_invalid")
      self.curve_v_target = None
      return accel_clip, v_cruise
    if v_ego < 1.0:
      if log_enabled:
        self._log_curve(reason="low_speed", v=v_ego)
      self.curve_v_target = None
      return accel_clip, v_cruise

    # 取 MPC 时间轴上的预测值
    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.velocity.x)
    turn_rates = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.orientationRate.z)
    positions = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.position.x)

    cfg = self._lincoln_curve_config()

    # 计算曲率，并截断到车辆可下发的信号范围
    curvatures = np.abs(turn_rates / np.clip(v_pred, 1.0, 100.0))
    curvatures = np.clip(curvatures, 0.0, 0.02)  # Ford 非 CAN FD 信号上限
    window_mask = positions <= cfg["window_m"]  # 关注前方窗口
    if not np.any(window_mask):
      if log_enabled:
        self._log_curve(reason="no_window_points")
      self.curve_v_target = None
      return accel_clip, v_cruise

    k_window = curvatures[window_mask]
    pos_window = positions[window_mask]
    k_max = float(np.max(k_window))
    critical_idx = int(np.argmax(k_window))
    critical_distance = float(pos_window[critical_idx])
    k_p80 = float(np.quantile(k_window, 0.8)) if k_window.size else 0.0
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
      self.curve_v_target = None
      return accel_clip, v_cruise

    # 平滑曲率，进入/退出滞回，减弱抖动
    alpha = 0.6
    self.curve_k_smooth = alpha * k_max + (1 - alpha) * self.curve_k_smooth
    # 高速段适当降低触发阈值，加快响应
    speed_factor = np.interp(v_ego, [0., 25., 40.], [1.0, 0.9, 0.8])
    k_enter = cfg["k_enter"] * speed_factor
    k_exit = cfg["k_exit"] * speed_factor
    enter_now = np.any(k_window >= k_enter)

    if self.curve_active:
      if self.curve_k_smooth < k_exit:
        # 退出滞回：计时后再退出，避免出口抖动
        self.curve_exit_timer += self.dt
        if self.curve_exit_timer > cfg["exit_h"]:
          self.curve_active = False
          self.curve_exit_timer = 0.0
      else:
        self.curve_exit_timer = 0.0
    else:
      if enter_now or self.curve_k_smooth >= k_enter:
        self.curve_active = True
        self.curve_exit_timer = 0.0

    if not self.curve_active:
      if log_enabled:
        self._log_curve(reason="below_enter", k=k_max, k_smooth=self.curve_k_smooth, v=v_ego)
      self.curve_v_target = None
      self.curve_a_target = 0.0
      return accel_clip, v_cruise

    # 目标侧向加速度上限（舒适），带安全系数
    a_lat_limit = cfg["a_lat"]
    v_limit = math.sqrt(a_lat_limit / max(self.curve_k_smooth, 1e-4))

    # 只有当弯道目标速度会限制用户的巡航设定时才介入（与 HUD 图标条件一致）。
    if v_cruise <= 0.1 or v_limit >= v_cruise - 1e-3:
      if log_enabled:
        self._log_curve(reason="not_limiting_v_cruise", v=v_ego, v_limit=v_limit, v_cruise=v_cruise, k=self.curve_k_smooth)
      self.curve_v_target = None
      self.curve_a_target = 0.0
      return accel_clip, v_cruise

    # 初始化一个“弯道期间的 v_cruise 上限”，确保图标出现时立即开始缓慢减速
    if self.curve_v_target is None or not math.isfinite(self.curve_v_target):
      self.curve_v_target = min(v_cruise, max(v_limit, v_ego))

    # 选取触发距离：优先首个超过阈值的点，否则用最大值位置；附加累积弯度提前触发
    trigger_distance = critical_distance
    trigger_curv = k_max
    trigger_type = "max"
    trigger_mask = k_window >= k_enter
    if np.any(trigger_mask):
      first_idx = int(np.argmax(trigger_mask))
      trigger_distance = float(pos_window[first_idx])
      trigger_curv = float(k_window[first_idx])
      trigger_type = "first_k"
    # 累积弯度/航向变化触发：积分 |k|·ds，随速降低阈值
    if len(pos_window) > 1:
      ds = np.diff(pos_window, prepend=pos_window[0])
      acc_ds = np.cumsum(np.abs(k_window) * ds)
      bend_cum = float(acc_ds[-1])
    else:
      acc_ds = np.array([])
      bend_cum = 0.0
    bend_thresh = float(np.deg2rad(np.interp(v_ego, [0., 25., 40.], [5.0, 4.0, 3.0])))  # 随速减小
    if bend_cum >= bend_thresh and bend_thresh > 0.0 and acc_ds.size:
      idx_bend = int(np.argmax(acc_ds >= bend_thresh))
      trigger_distance = float(pos_window[idx_bend])
      trigger_curv = float(k_window[idx_bend])
      trigger_type = "bend"
    # 预留最小减速距离：车速 * 1s
    d_use = max(trigger_distance, v_ego * 1.5, 1.0)

    # 等效减速度：a = (v_f^2 - v_i^2) / (2*d_use)
    required_decel = 0.0
    decel_cap = cfg["decel_max"]
    if v_ego > v_limit + 1e-3:
      required_decel = (v_limit ** 2 - v_ego ** 2) / max(2 * d_use, 1.0)
      # 高速允许更大预刹（与配置取最保守值）
      if v_ego > 25.0:
        decel_cap = min(decel_cap, -3.5)
      if v_ego > 33.0:
        decel_cap = min(decel_cap, -4.0)
      required_decel = max(required_decel, decel_cap)
      self.curve_a_target = float(required_decel)

      # 让 v_cruise 上限按 required_decel 逐步下降，图标出现即开始“慢慢减速”
      self.curve_v_target = max(v_limit, self.curve_v_target + required_decel * self.dt)

    # 应用弯道目标速度上限
    v_cruise = min(v_cruise, float(self.curve_v_target))

    mpc_a_max_min = float("nan")

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
        "k_enter": k_enter,
        "k_exit": k_exit,
        "k_p80": k_p80,
        "distance": critical_distance,
        "trigger_distance": trigger_distance,
        "trigger_curv": trigger_curv,
        "trigger_type": trigger_type,
        "bend_cum": bend_cum,
        "bend_thresh": bend_thresh,
        "d_use": d_use,
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
        "a_lat_limit": a_lat_limit,
        "window_m": cfg["window_m"],
        "exit_h": cfg["exit_h"],
        "decel_setting": decel_cap,
        "v_map_target": getattr(self, "_map_v_target", 0.0),
        "map_a_target": getattr(self, "_map_a_target", 0.0),
        "map_active": getattr(self, "_map_turn_limit_active", False),
        "map_data": getattr(self, "_map_data_available", False),
        "map_points": getattr(self, "_map_points", 0),
        "map_min_dist": getattr(self, "_map_min_dist_m", 0.0),
      })
    return accel_clip, v_cruise

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
      headers = [
        "timestamp",
        "reason",
        "v_ego(m/s)",
        "v_limit(m/s)",
        "k_smooth",
        "k_max",
        "k_model_max",
        "k_model_std",
        "k_enter",
        "k_exit",
        "k_p80",
        "a_lat_limit(m/s^2)",
        "window_m",
        "exit_h(s)",
        "decel_setting(m/s^2)",
        "distance_to_peak(m)",
        "trigger_distance(m)",
        "trigger_curv",
        "trigger_type",
        "bend_cum(rad)",
        "bend_thresh(rad)",
        "d_use(m)",
        "required_decel(m/s^2)",
        "accel_clip_max(m/s^2)",
        "mpc_a_max_min(m/s^2)",
        "curv_cmd",
        "curv_current",
        "accel_cmd",
        "accel_actual",
        "v_desired",
        "v_plan0",
        "yaw_rate",
        "steer_angle",
        "steer_torque",
        "steer_pressed",
        "gas_pressed",
        "brake_pressed",
        "steer_fault_temp",
        "steer_fault_perm",
        "sensors_invalid",
        "lat_active",
        "ctrl_active",
        "ctrl_long_active",
        "ctrl_lat_active",
        "ctrl_alert_type",
        "ctrl_alert_size",
        "ctrl_alert_sound",
        "cs_enabled",
        "cs_available",
        "cs_speed",
        "alerts",
        "v_map_target(m/s)",
        "map_a_target(m/s^2)",
        "map_active",
        "map_data",
        "map_points",
        "map_min_dist(m)",
      ]
      values = [
        ts,
        data.get('reason', ''),
        f"{data.get('v_ego', 0):.2f}",
        f"{data.get('v_limit', 0):.2f}",
        f"{data.get('k_smooth', 0):.5f}",
        f"{data.get('k_max', 0):.5f}",
        f"{data.get('k_model_max',0):.5f}",
        f"{data.get('k_model_std',0):.5f}",
        f"{data.get('k_enter',0):.5f}",
        f"{data.get('k_exit',0):.5f}",
        f"{data.get('k_p80',0):.5f}",
        f"{data.get('a_lat_limit',0):.2f}",
        f"{data.get('window_m',0):.1f}",
        f"{data.get('exit_h',0):.2f}",
        f"{data.get('decel_setting',0):.2f}",
        f"{data.get('distance', 0):.1f}",
        f"{data.get('trigger_distance',0):.1f}",
        f"{data.get('trigger_curv',0):.5f}",
        data.get('trigger_type',""),
        f"{data.get('bend_cum',0):.5f}",
        f"{data.get('bend_thresh',0):.5f}",
        f"{data.get('d_use',0):.1f}",
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
        f"{data.get('v_map_target', 0.0):.2f}",
        f"{data.get('map_a_target', 0.0):.2f}",
        str(data.get('map_active', False)),
        str(data.get('map_data', False)),
        str(data.get('map_points', 0)),
        f"{data.get('map_min_dist', 0.0):.1f}",
      ]
      line = ",".join(values) + "\n"
      fname = os.path.join(self.curve_log_dir, time.strftime("curve_%Y%m%d.log", time.localtime()))
      write_header = not os.path.exists(fname) or os.path.getsize(fname) == 0
      with open(fname, "a", encoding="utf-8") as f:
        if write_header:
          f.write(",".join(headers) + "\n")
        f.write(line)
    except Exception as e:
      cloudlog.error(f"curve log failed: {e}")

  def update(self, sm, dp_flags = 0, lincoln_curve_speed: bool = False, lincoln_curve_log: bool = False,
             lincoln_osm_realtime_cruise: bool = False):
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

    # Map-based turn speed target (priority over vision curve limit when it actively limits v_cruise)
    self._map_v_target = 0.0
    self._map_a_target = 0.0
    self._map_turn_limit_active = False
    self._map_data_available = False
    self._map_points = 0
    self._map_min_dist_m = 0.0

    v_map_target = 0.0
    map_a_target = 0.0
    map_turn_limit_active = False
    map_data_available = False
    if lincoln_osm_realtime_cruise and v_cruise > 0.1 and getattr(sm['selfdriveState'], "enabled", False):
      lat_lon: tuple[float, float] | None = None
      for service in ("gpsLocationExternal", "gpsLocation"):
        if service not in sm.data:
          continue
        gps = sm[service]
        if not getattr(gps, "hasFix", False):
          continue
        try:
          lat = float(gps.latitude)
          lon = float(gps.longitude)
        except Exception:
          continue
        if not (math.isfinite(lat) and math.isfinite(lon)):
          continue
        lat_lon = (lat, lon)
        break

      if lat_lon is not None:
        try:
          v_map_target, map_a_target = self._map_turn_target_speed(v_ego, sm['carState'].aEgo, lat_lon[0], lat_lon[1], v_cruise)
          map_data_available = len(self._map_target_velocities) > 0
        except Exception:
          v_map_target = 0.0
          map_a_target = 0.0
          map_turn_limit_active = False
          map_data_available = False

    self._map_v_target = float(v_map_target)
    self._map_a_target = float(map_a_target)
    self._map_data_available = bool(map_data_available)
    self._map_points = int(len(self._map_target_velocities)) if map_data_available else 0
    self._map_turn_limit_active = bool(v_map_target > 0.1 and v_map_target < (v_cruise - 1e-3))

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
      accel_clip, v_cruise = self._apply_lincoln_curve_speed(sm, accel_clip, v_cruise, log_enabled=lincoln_curve_log)
    else:
      self.curve_v_target = None
      self.curve_a_target = 0.0

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

    # "active" means map is actually tightening the cruise target (vs only having data).
    map_turn_limit_active = v_map_target > 0.1 and v_map_target < (v_cruise - 1e-3)
    self._map_turn_limit_active = bool(map_turn_limit_active)

    if force_slow_decel:
      v_cruise = 0.0

    if lincoln_osm_realtime_cruise and map_turn_limit_active and v_cruise > 0.1 and getattr(sm['selfdriveState'], "enabled", False):
      v_cruise = min(v_cruise, v_map_target)

    # HUD source label: report which limiter is actively tightening the cruise target.
    # NOTE: Don't gate on `long_control_off`/`longActive` since Ford often runs stock ACC; users still want to know
    # whether the limiting comes from map or vision.
    if map_turn_limit_active:
      self._curve_speed_source = 2  # map
    elif self.curve_v_target is not None:
      self._curve_speed_source = 1  # vision
    else:
      self._curve_speed_source = 0

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

    # Ensure curve speed control actually requests decel when active.
    # Limiting v_cruise alone can be too soft depending on planner mode and cruise clipping.
    if lincoln_curve_speed and not long_control_off and self.curve_a_target < -1e-3:
      output_a_target = min(output_a_target, float(self.curve_a_target))

    if lincoln_osm_realtime_cruise and map_turn_limit_active and not long_control_off and map_a_target < -1e-3:
      output_a_target = min(output_a_target, float(map_a_target))

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
    longitudinalPlan.curveSpeedSource = int(getattr(self, "_curve_speed_source", 0))

    pm.send('longitudinalPlan', plan_send)
