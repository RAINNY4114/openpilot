#!/usr/bin/env python3
"""
Ford/Lincoln Dynamic Curve Speed Controller

Extracted from RAINNY4114/openpilot commit 264abad.
Provides vision-based and map-based curve speed control for Ford vehicles.

Integrates into the sunnypilot LongitudinalPlanner by modifying v_cruise
and accel_clip before MPC and cruise acceleration calculations.
"""

import json
import math
import os
import time
import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.curve_bend import bend_from_model, bend_deg_min
from openpilot.common.swaglog import cloudlog

def _read_param_direct(key, params_path="/dev/shm/params"):
  """Read a param directly from filesystem, bypassing C++ key validation.

  查找顺序（很重要）：
    1. <params_path>/<key>      老式内存参数 —— mapd 直接写的 GPSQualityOK 在这里
    2. <params_path>/d/<key>    新版内存参数 —— C++ Params 的 shm 目录；mapd 的
                                MapTargetVelocities / MapSpeedLimit 在这里
    3. /data/params/d/<key>     持久化磁盘参数 —— 直接写入的 dp_lincoln_* 在这里

  只写 1 和 3 会漏掉地图数据（原实现的 bug：MapTargetVelocities 实际在 /dev/shm/params/d，
  导致地图腿永远读不到数据）。
  """
  import os
  candidates = (
    # fork-owned dir first: it is not the params dir, so Params::clearAll()
    # (params.cc:208-224) never unlinks it.  See ford_curve_controller.py.
    os.path.join("/data/ford_params", key),
    os.path.join(params_path, key),
    os.path.join(params_path, "d", key),
    os.path.join("/data/params/d", key),
  )
  for path in candidates:
    try:
      with open(path, 'r') as f:
        return f.read().strip()
    except Exception:
      continue
  return None

def _read_param_bool_direct(key, params_path="/dev/shm/params", default=False):
  """Read a boolean param directly from filesystem.

  `default` is returned when the param file is missing (or empty), so callers can
  express a code-level default that survives Params::clearAll() wiping files.
  """
  val = _read_param_direct(key, params_path)
  if val:
    return val == "1" or val.lower() == "true"
  return bool(default)

# Map Turn Speed Controller (mapd) constants
_MAP_EARTH_RADIUS_M = 6373000.0
_MAP_TO_RADIANS = math.pi / 180.0
_MAP_TARGET_JERK = -0.6   # m/s^3
_MAP_TARGET_ACCEL = -1.2  # m/s^2
_MAP_TARGET_OFFSET_S = 1.0
_MAP_PREVIEW_BASE_TIME_S = 10.0
_MAP_PREVIEW_BASE_MIN_HORIZON_M = 80.0
_MAP_PREVIEW_BASE_MAX_HORIZON_M = 400.0
_MAP_PREVIEW_EXT_TIME_S = 20.0
_MAP_PREVIEW_EXT_MIN_HORIZON_M = 120.0
_MAP_PREVIEW_EXT_MAX_HORIZON_M = 800.0

# car.capnp GearShifter: unknown=0 park=1 drive=2 neutral=3 reverse=4 sport=5
# low=6 brake=7 eco=8 manumatic=9.
# Ford's selector position 4 ("Sport_DriveSport") is reported by carstate.py as
# GearShifter.sport.
_GEAR_SPORT = 5


class FordCurveController:
  """
  Ford/Lincoln dynamic curve speed controller.

  Features:
  - Vision-based curve speed control using model curvature prediction
  - Map-based curve speed control using offline OSM data (mapd)
  - GPS quality gating to prevent phantom braking
  - Rate-limited map speed cap (tighten fast, release slow)
  - Configurable parameters via Params
  """

  def __init__(self, CP, dt=DT_MDL):
    self.CP = CP
    self.dt = dt

    # Curve speed state (vision-based)
    self.curve_k_smooth = 0.0
    self.curve_active = False
    self.curve_v_target = None
    self.curve_exit_timer = 0.0
    self.curve_a_target = 0.0
    self.last_curve_log_t = 0.0
    self.curve_log_dir = "/data/media/0/lincoln_curve_logs"
    # Cumulative heading change (degrees) over the predicted path. This is the
    # "how much does the road actually turn" measure used to gate the curve
    # detection; shared verbatim with the HUD via curve_bend.py.
    self.curve_bend_deg = 0.0
    self.curve_bend_ok = False

    # Params
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")
    self._curve_cfg = {}
    self._curve_param_last = 0.0

    # S-gear (Sport) state, refreshed every frame from sm['carState'].
    self._gear_is_sport = False

    # Map turn speed state
    self._map_target_velocities_raw = None
    self._map_target_velocities = []
    self._map_v_target = 0.0
    self._map_a_target = 0.0
    self._map_a_target_filtered = 0.0
    self._map_turn_limit_active = False
    self._map_data_available = False
    self._map_points = 0
    self._map_min_dist_m = 0.0
    self._map_lock_lat = 0.0
    self._map_lock_lon = 0.0
    self._map_lock_v = 0.0
    self._map_preview_v = 0.0
    self._map_prev_lat = float("nan")
    self._map_prev_lon = float("nan")
    self._map_heading_x = 0.0
    self._map_heading_y = 0.0
    self._map_heading_valid = False
    self._map_v_cap = float("nan")

    # Output state
    self._curve_speed_source = 0  # 0:none, 1:vision, 2:map

  def _lincoln_curve_config(self):
    now = time.monotonic()
    if now - self._curve_param_last < 1.0 and self._curve_cfg:
      return self._curve_cfg

    def _safe_int(name: str, default: int) -> int:
      try:
        val = _read_param_direct(name, "/data/params/d")
        return int(val) if val is not None else default
      except Exception:
        return default

    window_m = max(30, min(190, _safe_int("dp_lincoln_curve_window_m", 130)))
    k_enter_milli = max(2, min(20, _safe_int("dp_lincoln_curve_k_enter", 4)))
    k_enter = k_enter_milli * 1e-3
    k_exit = k_enter * 0.70

    # Lateral-acceleration budget used to derive the curve speed.
    #
    # S-gear (Sport) profile: when `dp_ford_sport_enable` is set AND the car
    # reports GearShifter.sport, use a larger budget so the car carries more
    # speed through the curve. With the switch off this is exactly the previous
    # hardcoded 1.8, so behaviour is unchanged.
    a_lat = 1.8
    sport_on = bool(self._gear_is_sport) and _read_param_bool_direct("dp_ford_sport_enable")
    if sport_on:
      try:
        a_lat = float(_read_param_direct("dp_ford_sport_a_lat") or 2.2)
      except Exception:
        a_lat = 2.2
      a_lat = max(1.0, min(3.6, a_lat))

    decel_cm = _safe_int("dp_lincoln_curve_decel", -320)
    decel_max = min(-0.5, max(-500, decel_cm) / 100.0)
    self._curve_cfg = {
      "window_m": float(window_m),
      "k_enter": float(k_enter),
      "k_exit": float(k_exit),
      "a_lat": float(a_lat),
      "decel_max": float(decel_max),
      "exit_h": 0.70,
      "sport": sport_on,
    }
    self._curve_param_last = now
    return self._curve_cfg

  def _map_distance_to_point(self, lat_a, lon_a, lat_b, lon_b):
    ax = lat_a * _MAP_TO_RADIANS
    ay = lon_a * _MAP_TO_RADIANS
    bx = lat_b * _MAP_TO_RADIANS
    by = lon_b * _MAP_TO_RADIANS
    a = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1 - a)))
    return _MAP_EARTH_RADIUS_M * c

  @staticmethod
  def _map_local_xy(lat0, lon0, lat1, lon1):
    lat0_r = float(lat0) * _MAP_TO_RADIANS
    dlat = (float(lat1) - float(lat0)) * _MAP_TO_RADIANS
    dlon = (float(lon1) - float(lon0)) * _MAP_TO_RADIANS
    x = dlon * math.cos(lat0_r) * _MAP_EARTH_RADIUS_M
    y = dlat * _MAP_EARTH_RADIUS_M
    return float(x), float(y)

  def _map_target_velocities_list(self):
    raw = None
    try:
      raw = _read_param_direct("MapTargetVelocities")
    except Exception:
      raw = None
    if not raw:
      try:
        raw = _read_param_direct("MapTargetVelocities", "/data/params/d")
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
  def _map_calculate_velocity(t, target_jerk, a_ego, v_ego):
    return v_ego + a_ego * t + (target_jerk / 2) * (t ** 2)

  @staticmethod
  def _map_calculate_distance(t, target_jerk, a_ego, v_ego):
    return t * v_ego + (a_ego / 2) * (t ** 2) + (target_jerk / 6) * (t ** 3)

  def _map_turn_target_speed(self, v_ego, a_ego, lat, lon, v_cruise):
    target_velocities = self._map_target_velocities_list()
    if not target_velocities:
      return 0.0, 0.0

    min_dist = 1e9
    min_idx = 0
    distances = []
    ahead_flags = []

    for i, target_velocity in enumerate(target_velocities):
      try:
        tlat = float(target_velocity.get("latitude", target_velocity.get("lat", 0.0)))
        tlon = float(target_velocity.get("longitude", target_velocity.get("lon", target_velocity.get("lng", 0.0))))
      except Exception:
        distances.append(1e9)
        ahead_flags.append(False)
        continue

      ahead = True
      if self._map_heading_valid:
        try:
          dx, dy = self._map_local_xy(lat, lon, tlat, tlon)
          ahead = (dx * float(self._map_heading_x) + dy * float(self._map_heading_y)) >= 0.0
        except Exception:
          ahead = True

      if not ahead:
        distances.append(1e9)
        ahead_flags.append(False)
        continue

      d = self._map_distance_to_point(lat, lon, tlat, tlon)
      distances.append(d)
      ahead_flags.append(True)
      if d < min_dist:
        min_dist = d
        min_idx = i

    self._map_min_dist_m = float(min_dist) if math.isfinite(min_dist) else 0.0

    forward_points = target_velocities[min_idx:]
    forward_distances = distances[min_idx:]
    forward_ahead = ahead_flags[min_idx:]

    # 1) In-curve hold
    v_hold = 0.0
    if min_dist < 30.0 and forward_points and (not self._map_heading_valid or bool(forward_ahead[0])):
      try:
        tv_here = float(forward_points[0].get("velocity", forward_points[0].get("speed", 0.0)))
      except Exception:
        tv_here = 0.0
      if tv_here > 70.0:
        tv_here *= CV.KPH_TO_MS
      if math.isfinite(tv_here) and tv_here > 0.1 and tv_here < (v_cruise - 1e-3):
        v_hold = float(tv_here)

    # 2) Preview cap
    preview_ext_horizon = max(_MAP_PREVIEW_EXT_MIN_HORIZON_M, min(_MAP_PREVIEW_EXT_MAX_HORIZON_M, v_ego * _MAP_PREVIEW_EXT_TIME_S))
    v_preview_ext = 0.0
    d_preview_ext = 0.0

    # 3) Approach decel
    valid_velocities = []
    for i, target_velocity in enumerate(forward_points):
      if self._map_heading_valid and (not bool(forward_ahead[i])):
        continue
      try:
        tv = float(target_velocity.get("velocity", target_velocity.get("speed", 0.0)))
      except Exception:
        continue
      if tv > 70.0:
        tv *= CV.KPH_TO_MS
      if not math.isfinite(tv) or tv <= 0.0:
        continue
      d = float(forward_distances[i])
      if tv < (v_cruise - 1e-3) and d < preview_ext_horizon:
        if v_preview_ext <= 0.1 or tv < v_preview_ext:
          v_preview_ext = float(tv)
          d_preview_ext = float(d)

      if tv >= v_ego - 1e-3:
        continue

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

    # 4) Lock map target
    lock_d = None
    if self._map_lock_v > 0.1 and forward_points:
      for i, target_velocity in enumerate(forward_points):
        if self._map_heading_valid and (not bool(forward_ahead[i])):
          continue
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
      self._map_lock_v = 0.0
      self._map_lock_lat = 0.0
      self._map_lock_lon = 0.0

    v_preview = float(v_preview_ext) if v_preview_ext > 0.1 else 0.0

    if v_target <= 0.1 and v_hold > 0.1:
      self._map_preview_v = 0.0
      return float(v_hold), 0.0
    if v_target <= 0.1 and v_preview > 0.1:
      try:
        v_min = float(v_preview)
      except Exception:
        v_min = 0.0
      d_min = float(d_preview_ext) if math.isfinite(float(d_preview_ext)) else 0.0
      d_use = max(d_min - v_ego * _MAP_TARGET_OFFSET_S, 1.0)
      a_preview = max(0.1, abs(float(_MAP_TARGET_ACCEL)))
      v_cap = 0.0
      if v_min > 0.1 and math.isfinite(v_min):
        v_cap = math.sqrt(max(0.0, v_min * v_min + 2.0 * a_preview * d_use))
        if v_cruise > 0.1 and math.isfinite(v_cruise):
          v_cap = min(float(v_cruise), float(v_cap))
      if self._map_preview_v <= 0.1 or not math.isfinite(self._map_preview_v):
        self._map_preview_v = float(v_cap)
      elif math.isfinite(v_cap) and v_cap > 0.1:
        if v_cap < self._map_preview_v:
          self._map_preview_v = float(v_cap)
        else:
          tau_s = 2.0
          alpha = float(self.dt / (tau_s + self.dt)) if self.dt > 0.0 else 0.0
          self._map_preview_v = float(self._map_preview_v + (v_cap - self._map_preview_v) * alpha)
      return float(self._map_preview_v), 0.0

    self._map_preview_v = 0.0
    if v_target <= 0.1:
      return 0.0, 0.0

    if v_target >= v_ego - 1e-3:
      return float(v_target), 0.0

    if (float(v_ego) > 15.0) and (float(d_target) < 10.0):
      return 0.0, 0.0

    d_use = max(d_target - v_ego * _MAP_TARGET_OFFSET_S, 1.0)
    required_decel = (v_target ** 2 - v_ego ** 2) / (2.0 * d_use)
    required_decel = min(0.0, float(required_decel))

    decel_cap = self._lincoln_curve_config().get("decel_max", -3.2)
    dv = max(0.0, float(v_ego - v_target))
    min_decel = -0.30 * min(1.0, dv / 2.0)
    map_a_target = max(float(decel_cap), min(float(min_decel), required_decel))

    prev_a = float(self._map_a_target_filtered) if math.isfinite(float(self._map_a_target_filtered)) else 0.0
    raw_a = float(map_a_target)
    if raw_a < prev_a:
      max_jerk_down = 2.0
      step = float(max_jerk_down) * float(self.dt)
      filt_a = max(raw_a, prev_a - step)
    else:
      tau_s = 0.8
      alpha = float(self.dt / (tau_s + self.dt)) if self.dt > 0.0 else 1.0
      filt_a = prev_a + (raw_a - prev_a) * alpha

    self._map_a_target_filtered = float(filt_a)
    return float(v_target), float(self._map_a_target_filtered)

  def _apply_lincoln_curve_speed(self, sm, v_cruise):
    """
    Vision-based curve speed control using model curvature prediction.
    Returns modified v_cruise.
    """
    model = sm['modelV2']
    car_state = sm['carState']
    v_ego = car_state.vEgo
    self.curve_a_target = 0.0

    if len(model.position.x) != ModelConstants.IDX_N or len(model.orientationRate.z) != ModelConstants.IDX_N:
      self.curve_v_target = None
      return v_cruise
    if v_ego < 1.0:
      self.curve_v_target = None
      return v_cruise

    # --- 累计转角闸门 ---------------------------------------------------------
    # 闸门本身在 update() 里算好（视觉腿/地图腿共用同一个数），这里只做检查，
    # 保证"标志亮 ⇔ 检查开"；直接调用本函数（测试）时也不会绕过闸门。
    if not self.curve_bend_ok:
      self.curve_v_target = None
      self.curve_a_target = 0.0
      self.curve_active = False
      self.curve_exit_timer = 0.0
      self.curve_k_smooth = 0.0
      return v_cruise

    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.velocity.x)
    turn_rates = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.orientationRate.z)
    positions = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model.position.x)

    cfg = self._lincoln_curve_config()

    v_denom = np.clip(v_pred, max(1.0, v_ego * 0.7), 100.0)
    curvatures = np.abs(turn_rates / v_denom)
    curvatures = np.clip(curvatures, 0.0, 0.02)
    window_mask = positions <= cfg["window_m"]
    if not np.any(window_mask):
      self.curve_v_target = None
      return v_cruise

    k_window = curvatures[window_mask]
    pos_window = positions[window_mask]
    k_max = float(np.max(k_window))
    critical_idx = int(np.argmax(k_window))
    critical_distance = float(pos_window[critical_idx])

    if k_max < 1e-4 or critical_distance < 5.0:
      self.curve_v_target = None
      return v_cruise

    alpha = 0.6
    self.curve_k_smooth = alpha * k_max + (1 - alpha) * self.curve_k_smooth
    k_enter = cfg["k_enter"]
    k_exit = cfg["k_exit"]
    enter_now = np.any(k_window >= k_enter)

    if self.curve_active:
      if self.curve_k_smooth < k_exit:
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
      self.curve_v_target = None
      self.curve_a_target = 0.0
      return v_cruise

    a_lat_limit = cfg["a_lat"]
    v_limit = math.sqrt(a_lat_limit / max(self.curve_k_smooth, 1e-4))

    if v_cruise <= 0.1 or v_limit >= v_cruise - 1e-3:
      self.curve_v_target = None
      self.curve_a_target = 0.0
      return v_cruise

    if self.curve_v_target is None or not math.isfinite(self.curve_v_target):
      self.curve_v_target = min(v_cruise, max(v_limit, v_ego))

    trigger_distance = critical_distance
    trigger_mask = k_window >= k_enter
    if np.any(trigger_mask):
      first_idx = int(np.argmax(trigger_mask))
      trigger_distance = float(pos_window[first_idx])

    # Bend accumulation trigger
    if len(pos_window) > 1:
      ds = np.diff(pos_window, prepend=pos_window[0])
      acc_ds = np.cumsum(np.abs(k_window) * ds)
      bend_cum = float(acc_ds[-1])
    else:
      acc_ds = np.array([])
      bend_cum = 0.0
    bend_thresh = float(np.deg2rad(np.interp(v_ego, [0., 25., 40.], [5.0, 4.0, 3.0])))
    if bend_cum >= bend_thresh and bend_thresh > 0.0 and acc_ds.size:
      idx_bend = int(np.argmax(acc_ds >= bend_thresh))
      trigger_distance = float(pos_window[idx_bend])

    d_use = max(trigger_distance - v_ego * 0.7, 1.0)

    required_decel = 0.0
    decel_cap = cfg["decel_max"]
    if v_ego > v_limit + 1e-3:
      required_decel = (v_limit ** 2 - v_ego ** 2) / max(2 * d_use, 1.0)
      dv = max(0.0, float(v_ego - v_limit))
      min_decel = -0.30 * min(1.0, dv / 2.0)
      required_decel = min(float(required_decel), float(min_decel))
      required_decel = max(float(required_decel), float(decel_cap))
      self.curve_a_target = float(required_decel)
      self.curve_v_target = max(v_limit, self.curve_v_target + required_decel * self.dt)

    v_cruise = min(v_cruise, float(self.curve_v_target))
    return v_cruise

  def _update_map_heading(self, sm, v_ego, lat_lon, gps_for_heading):
    """Update GPS heading estimate for map point filtering."""
    if lat_lon is None:
      return

    try:
      lat = float(lat_lon[0])
      lon = float(lat_lon[1])
      heading_updated = False
      heading_x = float(self._map_heading_x)
      heading_y = float(self._map_heading_y)

      bearing = None
      if gps_for_heading is not None and float(v_ego) > 1.0:
        try:
          b = float(getattr(gps_for_heading, "bearingDeg", float("nan")))
          if math.isfinite(b):
            bearing = b
        except Exception:
          pass

        if bearing is None:
          try:
            vned = getattr(gps_for_heading, "vNED", None)
            if vned is not None and len(vned) >= 2:
              v_n = float(vned[0])
              v_e = float(vned[1])
              if math.isfinite(v_n) and math.isfinite(v_e) and (abs(v_n) + abs(v_e) > 0.2):
                bearing = (math.degrees(math.atan2(v_e, v_n)) + 360.0) % 360.0
          except Exception:
            pass

        if bearing is None:
          try:
            v_n = float(getattr(gps_for_heading, "vN", float("nan")))
            v_e = float(getattr(gps_for_heading, "vE", float("nan")))
            if math.isfinite(v_n) and math.isfinite(v_e) and (abs(v_n) + abs(v_e) > 0.2):
              bearing = (math.degrees(math.atan2(v_e, v_n)) + 360.0) % 360.0
          except Exception:
            pass

      if bearing is not None:
        br = float(bearing) * _MAP_TO_RADIANS
        hx = math.sin(br)
        hy = math.cos(br)
        if math.isfinite(hx) and math.isfinite(hy):
          heading_x, heading_y = float(hx), float(hy)
          heading_updated = True

      if not heading_updated and math.isfinite(float(self._map_prev_lat)) and math.isfinite(float(self._map_prev_lon)):
        dx, dy = self._map_local_xy(float(self._map_prev_lat), float(self._map_prev_lon), lat, lon)
        norm = math.hypot(dx, dy)
        if math.isfinite(norm) and norm > 2.0:
          heading_x, heading_y = float(dx / norm), float(dy / norm)
          heading_updated = True

      if heading_updated:
        self._map_heading_x = float(heading_x)
        self._map_heading_y = float(heading_y)
        self._map_heading_valid = True

      self._map_prev_lat = float(lat)
      self._map_prev_lon = float(lon)
    except Exception:
      pass

  def update(self, sm, v_ego, v_cruise, reset_state, long_control_off, enabled,
             lincoln_curve_speed=False, lincoln_osm_realtime_cruise=False):
    """
    Main update method for curve speed control.

    Args:
      sm: SubMaster
      v_ego: current ego speed (m/s)
      v_cruise: current cruise target speed (m/s)
      reset_state: whether to reset state (not engaged)
      long_control_off: whether longitudinal control is off
      enabled: whether selfdrive is enabled
      lincoln_curve_speed: enable vision-based curve speed control
      lincoln_osm_realtime_cruise: enable map-based curve speed control

    Returns:
      (v_cruise_modified, map_turn_limit_active, map_a_target)
    """
    # S-gear (Sport) state for the Sport profile in `_lincoln_curve_config()`.
    try:
      self._gear_is_sport = int(sm['carState'].gearShifter) == _GEAR_SPORT
    except Exception:
      self._gear_is_sport = False

    # --- 累计转角闸门：弯道标志出现 + 检查/减速 的唯一触发条件 ----------------
    # "弯道标志出现和检查设定在45度触发"：沿预测路径的累计航向变化 ∫|κ|ds
    # 达到 /data/ford_curve.json 里的 bend_deg_min（默认 45°）才认为前方是真正的弯道。
    # 视觉腿和地图腿共用这一个数，保证"标志亮 ⇔ 检查开 ⇔ 减速"三者完全一致。
    self.curve_bend_deg = 0.0
    self.curve_bend_ok = False
    try:
      self.curve_bend_deg = float(bend_from_model(sm['modelV2'], v_ego))
      self.curve_bend_ok = bool(self.curve_bend_deg >= bend_deg_min())
    except Exception:
      self.curve_bend_deg = 0.0
      self.curve_bend_ok = False

    # --- Map-based turn speed ---
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

    if lincoln_osm_realtime_cruise and self.curve_bend_ok and v_cruise > 0.1 and enabled:
      # GPS quality gating
      gps_quality_ok = True
      try:
        gps_quality_ok = _read_param_bool_direct("GPSQualityOK")
      except Exception:
        gps_quality_ok = True

      lat_lon = None
      gps_for_heading = None
      if gps_quality_ok:
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
          gps_for_heading = gps
          break

        if lat_lon is not None:
          self._update_map_heading(sm, v_ego, lat_lon, gps_for_heading)

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

    # --- Vision-based curve speed ---
    # 注意：不要在这里清 curve_bend_deg/ok —— 闸门是视觉腿和地图腿共用的，
    # 关闭视觉腿不等于关闭地图腿，闸门状态要继续对外可见（HUD / 日志）。
    if lincoln_curve_speed:
      v_cruise = self._apply_lincoln_curve_speed(sm, v_cruise)
    else:
      self.curve_v_target = None
      self.curve_a_target = 0.0

    # --- Map cap rate limiting ---
    map_turn_limit_active_raw = v_map_target > 0.1 and v_map_target < (v_cruise - 1e-3)
    map_turn_limit_active = bool(map_turn_limit_active_raw)

    if reset_state or (not lincoln_osm_realtime_cruise) or (not enabled):
      self._map_v_cap = float("nan")
      map_turn_limit_active = False
      self._map_turn_limit_active = False
    else:
      if not math.isfinite(float(self._map_v_cap)):
        self._map_v_cap = float(v_cruise)

      v_cruise_pre_cap = float(v_cruise)
      v_cap_target = float(v_cruise_pre_cap)
      if map_turn_limit_active_raw and float(v_map_target) > 0.1:
        v_cap_target = float(min(v_cruise_pre_cap, float(v_map_target)))

      down_rate = float(np.interp(abs(float(map_a_target)), [0.0, 0.3, 1.5, 3.0], [0.8, 1.2, 2.0, 2.5]))
      up_rate = 0.6
      down_step = down_rate * float(self.dt)
      up_step = up_rate * float(self.dt)

      if v_cap_target < float(self._map_v_cap):
        self._map_v_cap = max(v_cap_target, float(self._map_v_cap) - down_step)
      else:
        self._map_v_cap = min(v_cap_target, float(self._map_v_cap) + up_step)

      if v_cruise_pre_cap > 0.1:
        v_cruise = min(v_cruise_pre_cap, float(self._map_v_cap))

      map_turn_limit_active = bool(v_cruise_pre_cap > 0.1 and v_cruise > 0.1 and v_cruise < (v_cruise_pre_cap - 1e-3))
      self._map_turn_limit_active = bool(map_turn_limit_active)

    # --- Curve speed source for HUD ---
    if map_turn_limit_active:
      self._curve_speed_source = 2  # map
    elif self.curve_v_target is not None:
      self._curve_speed_source = 1  # vision
    else:
      self._curve_speed_source = 0

    return v_cruise, map_turn_limit_active, map_a_target

  def get_curve_decel(self, long_control_off):
    """Return curve decel target if active and longitudinal is engaged."""
    if self.curve_a_target < -1e-3 and not long_control_off:
      return float(self.curve_a_target)
    return None

  @property
  def curve_speed_source(self):
    return self._curve_speed_source

  @property
  def curve_bend_degrees(self):
    """Cumulative heading change (deg) over the predicted path, for HUD/logging."""
    return float(self.curve_bend_deg)

  @property
  def curve_bend_triggered(self):
    """True when the cumulative-turn-angle gate (bend_deg_min) is satisfied."""
    return bool(self.curve_bend_ok)

  @property
  def map_turn_limit_active(self):
    return self._map_turn_limit_active

  @property
  def map_v_target(self):
    return self._map_v_target
