#!/usr/bin/env python3
"""弯道减速：Ford 分支方法移植（FrogPilot 风格 V-TSC / M-TSC）。

移植来源
--------
RAINNY4114/openpilot 分支 `ford`（HEAD 376fb1f，2026-10-08）
  selfdrive/controls/lib/longitudinal_planner.py
  中的 _fp_calc_road_curvature / _fp_map_curvature / _fp_curve_config
  以及 update() 里 v_cruise 的三源取小逻辑。

设计原则（重要）
----------------
**只读神经网络输出，不替换、不旁路大模型链路。**

V-TSC 的输入是

    modelV2.orientationRate.z[i] * modelV2.velocity.x[i]

即 chestnut eGPU 大模型直接预测的「未来各时刻横向加速度」。本模块只做
几何反解：

    kappa = a_lat_pred / v^2
    v_safe = sqrt(A_lat * aggressiveness / (|kappa| * sensitivity))

不参与推理、不改模型输入输出、不影响 modeld 的调度与频率。

与设备原有实现的区别
--------------------
设备原先有 FordCurveSpeedFusion + FordCurveController 两条弯道腿（共 1200+
行）。本模块是 ford 分支那条更简单、更接近 FrogPilot 原版的实现，作为
单一弯道减速路径；旧路径可用参数回退（见 _cfg 的 replace）。

参数（全部从 /data/ford_params 直读，绕开 manager 的 Params::clearAll）
------------------------------------------------------------------------
    dp_curve_tsc            0=关  1=视觉(V-TSC)  2=视觉+地图(V/M-TSC)  默认 1
    dp_curve_tsc_replace    1=接管并关闭旧弯道腿  0=与旧腿并存取小      默认 1
    dp_curve_tsc_lat_a      目标横向加速度 m/s^2                      默认 2.0
    dp_curve_tsc_sens       弯道灵敏度（越大越保守）                    默认 2.0
    dp_curve_tsc_agg        过弯积极度（越大越快）                      默认 1.0
    dp_curve_tsc_detect     触发阈值 m/s^2                            默认 1.0
    dp_curve_tsc_decel_max  弯道最大减速度 m/s^2                       默认 2.0
    dp_curve_tsc_log        1=写 /data/media/0/tsc.log                默认 0
"""

import json
import math
import os
import time

import numpy as np

# ---------------------------------------------------------------------------
# 常量（与 ford 分支保持一致）
# ---------------------------------------------------------------------------
TARGET_LAT_A = 2.0          # _FP_TARGET_LAT_A
CURVE_SENSITIVITY = 2.0     # _FP_CURVE_SENSITIVITY
CURVE_DETECT_LAT_A = 1.0    # _FP_CURVE_DETECT_LAT_A
TURN_AGGRESSIVENESS = 1.0   # _FP_TURN_AGGRESSIVENESS
CRUISING_SPEED = 5.0        # _FP_CRUISING_SPEED

# 释放滞回：进入用 detect，退出用 detect*RELEASE。ford 分支没有这一层，
# 阈值附近来回穿越会让目标车速抖动，进而引起纵向点头。
RELEASE_RATIO = 0.80

_EARTH_RADIUS_M = 6378137.0
_TO_RAD = math.pi / 180.0

_PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
_MAP_PARAM_DIRS = ("/dev/shm/params", "/data/params/d")
_LOG_PATH = "/data/media/0/tsc.log"

_SRC_NONE = 0
_SRC_VISION = 1
_SRC_MAP = 2


def _read_direct(key, default=None, dirs=_PARAM_DIRS):
  """直读参数文件。返回 str 或 default。"""
  for d in dirs:
    try:
      with open(os.path.join(d, key), "r") as f:
        val = f.read().strip()
      if val:
        return val
    except (OSError, ValueError):
      continue
  return default


def _read_float(key, default):
  try:
    val = _read_direct(key)
    if val is None:
      return float(default)
    out = float(val)
    return out if math.isfinite(out) else float(default)
  except Exception:
    return float(default)


def _read_int(key, default):
  try:
    val = _read_direct(key)
    if val is None:
      return int(default)
    return int(float(val))
  except Exception:
    return int(default)


def _distance_m(lat_a, lon_a, lat_b, lon_b):
  ax = lat_a * _TO_RAD
  ay = lon_a * _TO_RAD
  bx = lat_b * _TO_RAD
  by = lon_b * _TO_RAD
  a = (math.sin((bx - ax) / 2.0) ** 2
       + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2.0) ** 2)
  c = 2.0 * math.atan2(math.sqrt(max(1e-12, a)), math.sqrt(max(1e-12, 1.0 - a)))
  return _EARTH_RADIUS_M * c


def _circumradius_curvature(p1, p2, p3):
  """三点外接圆曲率（1/m）。退化时返回 0。"""
  a = _distance_m(p2[0], p2[1], p3[0], p3[1])
  b = _distance_m(p1[0], p1[1], p3[0], p3[1])
  c = _distance_m(p1[0], p1[1], p2[0], p2[1])
  s = (a + b + c) / 2.0
  area_sq = s * (s - a) * (s - b) * (s - c)
  if area_sq <= 0.0:
    return 0.0
  area = math.sqrt(area_sq)
  if area <= 0.0:
    return 0.0
  radius = (a * b * c) / (4.0 * area)
  if radius <= 0.0:
    return 0.0
  return 1.0 / radius


class CurveSpeedTSC:
  """视觉 / 地图弯道目标车速。每个规划周期调用一次 update()。"""

  def __init__(self):
    self._cfg_cache = None
    self._cfg_t = 0.0

    self._map_raw = None
    self._map_points = []

    self._detected_last = False
    self._last_log_t = 0.0

    # 对外只读状态（供 planner 发布到 HUD / 日志）
    self.road_curvature = 0.0
    self.v_target = float("nan")
    self.a_target = 0.0
    self.source = _SRC_NONE
    self.active = False

  # ------------------------------------------------------------------
  # 配置
  # ------------------------------------------------------------------
  def _cfg(self):
    now = time.monotonic()
    if self._cfg_cache is not None and (now - self._cfg_t) < 1.0:
      return self._cfg_cache

    mode = _read_int("dp_curve_tsc", 1)
    replace = _read_int("dp_curve_tsc_replace", 1)
    lat_a = _read_float("dp_curve_tsc_lat_a", TARGET_LAT_A)
    sens = _read_float("dp_curve_tsc_sens", CURVE_SENSITIVITY)
    agg = _read_float("dp_curve_tsc_agg", TURN_AGGRESSIVENESS)
    detect = _read_float("dp_curve_tsc_detect", CURVE_DETECT_LAT_A)
    decel_max = _read_float("dp_curve_tsc_decel_max", 2.0)

    # 夹到合理范围，防止参数写错把车刹死或完全失效
    self._cfg_cache = {
      "mode": max(0, min(2, mode)),
      "replace": bool(replace),
      "lat_a": float(np.clip(lat_a, 0.8, 4.0)),
      "sens": float(np.clip(sens, 0.3, 5.0)),
      "agg": float(np.clip(agg, 0.3, 3.0)),
      "detect": float(np.clip(detect, 0.3, 3.0)),
      "decel_max": float(np.clip(decel_max, 0.5, 3.5)),
      "log": _read_int("dp_curve_tsc_log", 0) != 0,
    }
    self._cfg_t = now
    return self._cfg_cache

  # ------------------------------------------------------------------
  # 视觉腿：大模型预测横向加速度 -> 曲率
  # ------------------------------------------------------------------
  @staticmethod
  def curvature_from_model(model_msg, v_ego):
    """与 ford 分支 _fp_calc_road_curvature 等价。

    取整条预测轨迹上 |orientationRate.z * velocity.x| 最大的那一点，
    除以当前车速平方得到曲率。这样天然带上前视（模型 horizon）。
    """
    try:
      # capnp 的 _DynamicListReader 不能直接喂给 np.asarray(dtype=float)，
      # 会抛 TypeError 并被静默吞成"曲率恒为 0"。必须先落成 python list。
      orientation_rate = np.array(list(model_msg.orientationRate.z), dtype=float)
      velocity = np.array(list(model_msg.velocity.x), dtype=float)
    except Exception:
      return 0.0

    if orientation_rate.size == 0 or velocity.size == 0:
      return 0.0

    n = min(orientation_rate.size, velocity.size)
    lat_accel = orientation_rate[:n] * velocity[:n]
    lat_accel = lat_accel[np.isfinite(lat_accel)]
    if lat_accel.size == 0:
      return 0.0

    peak = float(lat_accel[int(np.argmax(np.abs(lat_accel)))])
    return peak / max(float(v_ego), 1.0) ** 2

  # ------------------------------------------------------------------
  # 地图腿
  # ------------------------------------------------------------------
  def _map_target_velocities(self):
    raw = None
    try:
      for d in _MAP_PARAM_DIRS:
        try:
          with open(os.path.join(d, "MapTargetVelocities"), "r") as f:
            raw = f.read().strip()
          break
        except OSError:
          continue
    except Exception:
      raw = None

    if not raw:
      self._map_raw = None
      self._map_points = []
      return []

    if raw == self._map_raw:
      return self._map_points

    try:
      parsed = json.loads(raw)
      if not isinstance(parsed, list):
        parsed = []
    except Exception:
      parsed = []

    self._map_raw = raw
    self._map_points = parsed
    return parsed

  @staticmethod
  def _parse_point(item):
    try:
      lat = float(item.get("latitude", item.get("lat", 0.0)))
      lon = float(item.get("longitude", item.get("lon", item.get("lng", 0.0))))
    except Exception:
      return None
    if not (math.isfinite(lat) and math.isfinite(lon)):
      return None
    return lat, lon

  def _map_curvature(self, lat, lon, v_ego, points):
    pts = []
    dists = []
    for item in points:
      p = self._parse_point(item)
      if p is None:
        continue
      pts.append(p)
      dists.append(_distance_m(lat, lon, p[0], p[1]))

    if len(pts) < 3:
      return 0.0

    # 从最近点开始向前找
    idx = int(np.argmin(dists))
    fwd_pts = pts[idx:]
    fwd_dist = dists[idx:]
    if len(fwd_pts) < 3:
      return 0.0

    lookahead = 10.0 * max(float(v_ego), 1.0)  # 约 10 s 前视
    acc = 0.0
    target = None
    for i, d in enumerate(fwd_dist):
      acc += float(d)
      if acc >= lookahead:
        target = i
        break

    if target is None or target <= 0 or target >= len(fwd_pts) - 1:
      return 0.0

    return _circumradius_curvature(fwd_pts[target - 1], fwd_pts[target], fwd_pts[target + 1])

  # ------------------------------------------------------------------
  # 主入口
  # ------------------------------------------------------------------
  def update(self, model_msg, v_ego, v_cruise, enabled, left_blinker=False,
             right_blinker=False, gps=None):
    """返回 (v_cruise_out, active)。

    v_cruise_out 已包含弯道约束（即 min(输入 v_cruise, 各腿目标)）。
    任何异常都回落到输入 v_cruise，保证不影响纵向控制。
    """
    v_cruise = float(v_cruise)
    self.active = False
    self.source = _SRC_NONE
    self.v_target = float("nan")

    try:
      cfg = self._cfg()
      mode = cfg["mode"]
      if mode <= 0 or not enabled or model_msg is None:
        self.road_curvature = 0.0
        self.a_target = 0.0
        self._detected_last = False
        return v_cruise, False

      if float(v_ego) <= CRUISING_SPEED:
        self.road_curvature = 0.0
        self.a_target = 0.0
        self._detected_last = False
        return v_cruise, False

      curv = self.curvature_from_model(model_msg, v_ego)
      self.road_curvature = float(curv)

      lat_pred = abs(curv) * max(float(v_ego), 1.0) ** 2
      detect = cfg["detect"]
      # 进入 / 退出用不同阈值，避免阈值附近抖动
      threshold = detect if self._detected_last else detect * RELEASE_RATIO
      detected = bool(lat_pred > threshold and not (left_blinker or right_blinker))
      self._detected_last = detected

      if not detected:
        self.v_target = float(v_cruise)
        self.a_target = 0.0
        self.source = _SRC_NONE
        return v_cruise, False

      v_best = v_cruise
      src = _SRC_NONE

      # --- 视觉腿 (V-TSC) ---
      denom = abs(curv) * cfg["sens"]
      if denom > 1e-9:
        v_vis = math.sqrt((cfg["lat_a"] * cfg["agg"]) / denom)
        v_vis = max(CRUISING_SPEED, float(v_vis))
        if v_vis < v_best:
          v_best = v_vis
          src = _SRC_VISION

      # --- 地图腿 (M-TSC) ---
      if mode >= 2 and gps is not None:
        try:
          pts = self._map_target_velocities()
          if pts:
            map_curv = self._map_curvature(gps[0], gps[1], v_ego, pts)
            denom_m = abs(map_curv) * cfg["sens"]
            if denom_m > 1e-9:
              v_map = math.sqrt((cfg["lat_a"] * cfg["agg"]) / denom_m)
              v_map = max(CRUISING_SPEED, float(v_map))
              if v_map < v_best:
                v_best = v_map
                src = _SRC_MAP
        except Exception:
          pass

      # 弯道制动请求。
      # ford 分支对应 cruise_min_accel = min(..., -dv/2)，用于突破巡航的
      # A_CRUISE_MIN(-1.2 m/s^2) 下限；本设备 MPC 不接受该参数，因此这里
      # 直接给出等价的减速度请求，由 planner 作为 a_target 的上界。
      dv = max(float(v_ego) - float(v_best), 0.0)
      if dv > 0.1:
        self.a_target = -float(min(dv / 2.0, cfg["decel_max"]))
      else:
        self.a_target = 0.0

      self.v_target = float(v_best)
      self.source = src
      self.active = src != _SRC_NONE

      if cfg["log"]:
        self._log(v_ego, v_cruise, curv, v_best, src)

      return float(v_best), bool(self.active)

    except Exception:
      self.road_curvature = 0.0
      self._detected_last = False
      self.v_target = float("nan")
      self.a_target = 0.0
      self.source = _SRC_NONE
      self.active = False
      return v_cruise, False

  # ------------------------------------------------------------------
  def _log(self, v_ego, v_cruise, curv, v_target, src):
    now = time.monotonic()
    if (now - self._last_log_t) < 0.5:
      return
    self._last_log_t = now
    try:
      with open(_LOG_PATH, "a", buffering=1) as f:
        f.write("%.3f,%.3f,%.3f,%.8f,%.3f,%d\n"
                % (now, float(v_ego), float(v_cruise), float(curv),
                   float(v_target), int(src)))
    except Exception:
      pass
