#!/usr/bin/env python3
"""cut-in 预减速风险评估（ford-chestnut 设备版）。

来源：carrot-wip 的 ``cutin_helpers.py`` / ``cutin_predecel.py``。

原实现有三处在本设备不可用，因此这里重写而非移植：

1. ``is_front_radar_cutin_enabled()`` 硬编码 ``car_brand == "hyundai"``，并依赖
   Hyundai 角雷达的 source / trackId 段约定（``CORNER_TRACK_ID_RANGES``）。
   → 本实现品牌无关，不再区分 front / corner 雷达。
2. 原实现用 ``RadarPoint.yvRel`` 直接取横向速度。设备 ``car.RadarPoint`` 只有
   ``trackId / dRel / yRel / vRel``，``yvRel`` 在 deprecated group 不可读。
   → 横向速度改为对 ``yRel`` 做时间最小二乘求斜率。
3. 原实现假定 trackId 跨帧稳定。设备实测 trackId 碎片化严重（146 帧 696 个
   distinct id），单靠 trackId 无法积累历史。
   → 采用「同 id 优先 + 最近邻兜底」的双轨关联，跨 renumber 延续轨迹。

额外融合（用户要求）：左右激光雷达（``amapNavi``）、左右盲点
（``carState.leftBlindspot/rightBlindspot``）、MR76 侧向占用
（``AOMr76GainFresh/AOMr76GainLane``）参与置信度修正，并记录在 ``source`` 位图里。

输出写入 ``RadarState.leadCutInRisk``（cereal/log.capnp 新增结构），由 plannerd
侧的 ``longitudinal_planner`` 消费，按 TTC 换算加速度上界做预减速。
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from typing import Any

CONFIG_PATH = "/data/cutin_risk.json"

# 与 modeld / card.py 一致的参数目录优先级（fork 的预编译 libparams_c.so 会
# 静默 unlink /data/params/d 下未注册的自定义键，所以 /data/ford_params 优先）。
PARAM_DIRS = ("/data/ford_params", "/dev/shm/params/d", "/data/params/d")

# --- leadCutInRisk.source 位定义 ---------------------------------------------
SRC_FRONT_RADAR = 1
SRC_VISION = 2
SRC_LEFT_LIDAR = 4
SRC_RIGHT_LIDAR = 8
SRC_LEFT_BSM = 16
SRC_RIGHT_BSM = 32
SRC_MR76_LEFT = 64
SRC_MR76_RIGHT = 128

RADAR_TO_CAMERA = 1.52  # 与 radard.py 保持一致

DEFAULTS: dict[str, Any] = {
    "enabled": True,
    # 0=关闭  1=仅分析（发布 leadCutInRisk，不做预减速）  2=分析+预减速
    "mode": 1,
    "log": True,
    "log_period_s": 5.0,
    "config_reload_period_s": 5.0,

    # --- 轨迹关联 -------------------------------------------------------------
    "hist_len": 14,                 # 0.6 s @ 20 Hz
    "min_hist_samples": 5,          # 0.25 s
    "max_dt_gap_s": 0.30,
    "max_frame_drel_jump_m": 3.0,   # CUTIN_MAX_FRAME_DREL_JUMP_M
    "max_frame_y_jump_m": 0.60,     # CUTIN_MAX_FRAME_Y_JUMP_M
    "max_frame_vrel_jump_mps": 2.5,
    "assoc_max_drel_m": 1.5,        # CUTIN_ASSOC_MAX_DREL
    "assoc_max_yrel_m": 0.75,       # CUTIN_ASSOC_MAX_YREL
    "assoc_max_vrel_mps": 2.5,      # CUTIN_ASSOC_MAX_VREL

    # --- 几何门限 -------------------------------------------------------------
    "min_drel_m": 5.0,              # FRONT_CUTIN_MIN_DREL_M
    "max_drel_m": 50.0,             # FRONT_CUTIN_MAX_DREL_M
    "max_abs_y_m": 7.0,             # FRONT_CUTIN_MAX_ABS_YREL_M
    "min_abs_y_m": 1.5,             # enter_min_abs_dpath
    "outer_abs_y_m": 4.5,
    "min_vlead_mps": 4.0,           # 只对运动车辆判定，排除静止目标
    "horizon_s": 1.5,               # CUTIN_LANE_MOTION_HORIZON_S
    # 预减速要「提前」，所以门限用双时间窗：近窗（horizon）或远窗
    # （horizon*mult）任一满足即放行。近窗只用于打分，避免远窗把置信度抬高。
    "future_lane_horizon_mult": 2.0,
    "min_inward_mps": 0.45,         # enter_min_inward_speed
    "min_centering_gain_m": 0.15,   # enter_centering_gain
    "min_future_in_lane_prob": 0.20,

    # --- 车道几何 -------------------------------------------------------------
    "lane_half_w_default": 1.75,
    "lane_half_w_min": 1.40,
    "lane_half_w_max": 2.20,
    "lane_soft_margin_m": 0.90,     # 半车宽，用于「将与本车道重叠」的软概率
    "lane_prob_thr": 0.30,

    # --- 置信度 ---------------------------------------------------------------
    "min_score": 0.35,
    "w_motion": 0.28,
    "w_lane": 0.26,
    "w_gain": 0.20,
    "w_range": 0.14,
    "w_closing": 0.12,
    "score_boost_same_side": 0.15,
    "score_penalty_opposite_side": 0.05,
    "score_scale_outer_no_ctx": 0.50,

    # --- 视觉腿（雷达腿不成立时才启用）-----------------------------------------
    "vision_enable": True,
    "vision_min_prob": 0.50,
    "vision_min_abs_y_m": 1.2,
    "vision_max_abs_y_m": 5.0,

    # --- 预减速（mode>=2 生效）------------------------------------------------
    # 比 status 门限高得多：影子态可以敏感（status），执行态必须保守。
    # carrot 原值 0.15 是它自己的分数标度，本实现归一化到 1.0，取 0.45。
    "predecel_min_score": 0.45,
    "predecel_min_ttc_s": 2.5,      # CUTIN_PREDECEL_MIN_TTC_S
    "predecel_max_ttc_s": 8.0,      # CUTIN_PREDECEL_MAX_TTC_S
    "predecel_min_accel": -0.65,    # CUTIN_PREDECEL_MIN_ACCEL_LIMIT
    "predecel_max_accel": -0.25,    # CUTIN_PREDECEL_MAX_ACCEL_LIMIT
    "predecel_jerk_step": 0.15,     # CUTIN_PREDECEL_MAX_JERK_STEP
}


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _read_param_num(key: str, default: float = 0.0) -> float:
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, key), "r") as f:
        return float(f.read().strip())
    except Exception:
      continue
  return default


def _slope(ts: list[float], ys: list[float]) -> float:
  """最小二乘斜率 dy/dt，对非均匀采样与 t 递减都成立。"""
  n = len(ts)
  if n < 2:
    return 0.0
  tm = sum(ts) / n
  ym = sum(ys) / n
  den = sum((t - tm) ** 2 for t in ts)
  if den <= 1e-9:
    return 0.0
  return sum((t - tm) * (y - ym) for t, y in zip(ts, ys)) / den


def _clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else (hi if x > hi else x)


def _empty_out() -> dict[str, Any]:
  return {
    "status": False, "radar": False, "score": 0.0,
    "dRel": 0.0, "yRel": 0.0, "vRel": 0.0, "ttc": 0.0, "inwardSpeed": 0.0,
    "source": 0, "trackId": -1, "ageFrames": 0, "stamp": 0,
  }


def _associate(prev: dict[int, tuple[float, float, float]],
               cur: dict[int, tuple[float, float, float]],
               max_d: float, max_y: float, max_v: float) -> dict[int, int]:
  """cur trackId -> prev trackId。同 id 且几何连续时优先，其余走最近邻。"""
  if not prev or not cur:
    return {}
  cands: list[tuple[float, int, int]] = []
  for cid, (cd, cy, cv) in cur.items():
    pd_py_pv = prev.get(cid)
    if pd_py_pv is not None:
      pd, py, pv = pd_py_pv
      if abs(cd - pd) <= max_d and abs(cy - py) <= max_y and abs(cv - pv) <= max_v:
        cands.append((-1.0, cid, cid))
    for pid, (pd, py, pv) in prev.items():
      if pid == cid:
        continue
      dd = abs(cd - pd)
      dy = abs(cy - py)
      dv = abs(cv - pv)
      if dd > max_d or dy > max_y or dv > max_v:
        continue
      cands.append((dd / max_d + dy / max_y + dv / max_v, cid, pid))
  cands.sort()
  assoc: dict[int, int] = {}
  used: set[int] = set()
  for _, cid, pid in cands:
    if cid in assoc or pid in used:
      continue
    assoc[cid] = pid
    used.add(pid)
  return assoc


def _interp_at(line: Any, d: float) -> float | None:
  """在 laneLine 的 (x, y) 上按 d 线性插值；x 必须单调。"""
  try:
    xs = line.x
    ys = line.y
    n = min(len(xs), len(ys))
    if n < 2:
      return None
    for i in range(1, n):
      x0 = float(xs[i - 1])
      x1 = float(xs[i])
      if x0 <= d <= x1 and x1 - x0 > 1e-6:
        y0 = float(ys[i - 1])
        y1 = float(ys[i])
        return y0 + (y1 - y0) * (d - x0) / (x1 - x0)
  except Exception:
    return None
  return None


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
def load_config(path: str = CONFIG_PATH) -> dict[str, Any]:
  cfg = dict(DEFAULTS)
  try:
    with open(path, "r") as f:
      user = json.load(f)
    if isinstance(user, dict):
      for k, v in user.items():
        if k in DEFAULTS:
          cfg[k] = v
  except Exception:
    pass
  return cfg


_CFG_CACHE: dict[str, Any] = {"cfg": None, "t": 0.0, "period": 5.0}


def load_config_cached(now: float | None = None, path: str = CONFIG_PATH) -> dict[str, Any]:
  """供 planner 侧低频读取（每 config_reload_period_s 一次）。"""
  now = time.monotonic() if now is None else now
  if _CFG_CACHE["cfg"] is None or (now - _CFG_CACHE["t"]) >= _CFG_CACHE["period"]:
    cfg = load_config(path)
    _CFG_CACHE["cfg"] = cfg
    _CFG_CACHE["t"] = now
    _CFG_CACHE["period"] = float(cfg.get("config_reload_period_s", 5.0))
  return _CFG_CACHE["cfg"]


# ---------------------------------------------------------------------------
# 估计器
# ---------------------------------------------------------------------------
class CutInRiskEstimator:
  def __init__(self, config_path: str = CONFIG_PATH):
    self.config_path = config_path
    self.cfg = load_config(config_path)
    self._cfg_t = time.monotonic()

    self._hist: dict[int, deque] = {}
    self._id2h: dict[int, int] = {}
    self._last_pts: dict[int, tuple[float, float, float]] = {}
    self._next_h = 1

    self._mr76 = (0, 0)
    self._mr76_t = -1e9
    self._last_log_t = -1e9
    self.last_out: dict[str, Any] = _empty_out()

  # -- 配置热重载 ------------------------------------------------------------
  def _maybe_reload(self, now: float) -> None:
    if now - self._cfg_t < float(self.cfg.get("config_reload_period_s", 5.0)):
      return
    self._cfg_t = now
    self.cfg = load_config(self.config_path)

  # -- 侧向上下文 ------------------------------------------------------------
  def _side_ctx(self, amap_msg: Any, car_state: Any, now: float) -> dict[str, bool]:
    left_lidar = right_lidar = False
    if amap_msg is not None:
      try:
        left_lidar = bool(amap_msg.leftLidarObstacle) or bool(amap_msg.leftBodyRadarObstacle)
        right_lidar = bool(amap_msg.rightLidarObstacle) or bool(amap_msg.rightBodyRadarObstacle)
      except Exception:
        pass

    left_bsm = bool(getattr(car_state, "leftBlindspot", False)) if car_state is not None else False
    right_bsm = bool(getattr(car_state, "rightBlindspot", False)) if car_state is not None else False

    if now - self._mr76_t > 1.0:
      self._mr76_t = now
      fresh = _read_param_num("AOMr76GainFresh", 0.0) != 0.0
      lane = int(_read_param_num("AOMr76GainLane", 0.0))
      # card.py: 十位 = 左，个位 = 右（见 _ao_publish_mr76_lane）
      self._mr76 = ((lane // 10) % 10, lane % 10) if fresh else (0, 0)

    return {
      "left_lidar": left_lidar, "right_lidar": right_lidar,
      "left_bsm": left_bsm, "right_bsm": right_bsm,
      "left_mr76": self._mr76[0] != 0, "right_mr76": self._mr76[1] != 0,
    }

  # -- 车道半宽 --------------------------------------------------------------
  def _lane_half_width(self, model_msg: Any, d: float) -> float:
    default = float(self.cfg["lane_half_w_default"])
    if model_msg is None:
      return default
    try:
      lines = model_msg.laneLines
      probs = model_msg.laneLineProbs
      if len(lines) < 3 or len(probs) < 3:
        return default
      if float(probs[1]) < float(self.cfg["lane_prob_thr"]):
        return default
      if float(probs[2]) < float(self.cfg["lane_prob_thr"]):
        return default
      yl = _interp_at(lines[1], d)
      yr = _interp_at(lines[2], d)
      if yl is None or yr is None:
        return default
      hw = 0.5 * (yl - yr)
      if not math.isfinite(hw) or hw <= 0.1:
        return default
      return _clip(hw, float(self.cfg["lane_half_w_min"]), float(self.cfg["lane_half_w_max"]))
    except Exception:
      return default

  # -- 单条轨迹打分 ----------------------------------------------------------
  def _score(self, *, d: float, y: float, v_rel: float, inward: float, v_ego: float,
             lane_hw: float, side: dict[str, bool]) -> dict[str, Any] | None:
    cfg = self.cfg
    v_lead = v_ego + v_rel

    if not (cfg["min_drel_m"] <= d <= cfg["max_drel_m"]):
      return None
    ay = abs(y)
    if ay > cfg["max_abs_y_m"] or ay < cfg["min_abs_y_m"]:
      return None
    if v_lead < cfg["min_vlead_mps"]:
      return None
    if inward < cfg["min_inward_mps"]:
      return None

    sign = 1.0 if y >= 0.0 else -1.0
    horizon = float(cfg["horizon_s"])
    mult = float(cfg["future_lane_horizon_mult"])
    y_future = y - sign * inward * horizon
    y_future_far = y - sign * inward * horizon * mult
    soft = max(0.1, lane_hw + float(cfg["lane_soft_margin_m"]))

    in_lane_now = _clip(1.0 - ay / soft, 0.0, 1.0)
    in_lane_near = _clip(1.0 - abs(y_future) / soft, 0.0, 1.0)
    in_lane_far = _clip(1.0 - abs(y_future_far) / soft, 0.0, 1.0)
    # 门限取两窗较大者（早触发）；打分只用近窗，避免远窗虚高置信度。
    in_lane_gate = max(in_lane_near, in_lane_far)
    gain = ay - abs(y_future)

    if in_lane_gate < cfg["min_future_in_lane_prob"]:
      return None
    if gain < cfg["min_centering_gain_m"]:
      return None

    s_motion = _clip(inward / 1.2, 0.0, 1.0)
    s_lane = in_lane_near
    s_gain = _clip(gain / 0.8, 0.0, 1.0)
    s_range = _clip((float(cfg["max_drel_m"]) - d) / max(1.0, float(cfg["max_drel_m"]) - 10.0), 0.0, 1.0)
    s_close = _clip(-v_rel / 3.0, 0.0, 1.0)

    score = (float(cfg["w_motion"]) * s_motion + float(cfg["w_lane"]) * s_lane
             + float(cfg["w_gain"]) * s_gain + float(cfg["w_range"]) * s_range
             + float(cfg["w_closing"]) * s_close)

    # --- 侧向上下文融合（激光雷达 / 盲点 / MR76）---
    src = 0
    left = sign > 0.0
    if left:
      if side["left_lidar"]:
        src |= SRC_LEFT_LIDAR
      if side["left_bsm"]:
        src |= SRC_LEFT_BSM
      if side["left_mr76"]:
        src |= SRC_MR76_LEFT
      same_side = bool(src)
      opposite = bool(side["right_lidar"] or side["right_bsm"] or side["right_mr76"])
    else:
      if side["right_lidar"]:
        src |= SRC_RIGHT_LIDAR
      if side["right_bsm"]:
        src |= SRC_RIGHT_BSM
      if side["right_mr76"]:
        src |= SRC_MR76_RIGHT
      same_side = bool(src)
      opposite = bool(side["left_lidar"] or side["left_bsm"] or side["left_mr76"])

    if same_side:
      score += float(cfg["score_boost_same_side"])
    elif opposite:
      score -= float(cfg["score_penalty_opposite_side"])
    if ay > float(cfg["outer_abs_y_m"]) and not same_side:
      score *= float(cfg["score_scale_outer_no_ctx"])

    # --- TTC ---
    if v_rel < -0.05:
      ttc = d / max(-v_rel, 0.1)
    else:
      ttc = d / max(v_ego - v_lead, 0.1)
    ttc = _clip(ttc, 0.0, 20.0)

    return {
      "score": _clip(score, 0.0, 1.0), "ttc": ttc,
      "in_lane_now": in_lane_now, "in_lane_future": in_lane_near,
      "in_lane_gate": in_lane_gate,
      "gain": gain, "source": src, "inward": inward, "y_future": y_future,
    }

  # -- 视觉腿 ----------------------------------------------------------------
  def _vision_leg(self, model_msg: Any, v_ego: float, side: dict[str, bool]) -> dict[str, Any] | None:
    cfg = self.cfg
    if not cfg["vision_enable"] or model_msg is None:
      return None
    try:
      leads = model_msg.leadsV3
      if len(leads) < 2:
        return None
      mv = float(model_msg.velocity.x[0]) if len(model_msg.velocity.x) else v_ego
    except Exception:
      return None

    best = None
    for i in range(2):
      try:
        ld = leads[i]
        if float(ld.prob) < float(cfg["vision_min_prob"]):
          continue
        n = min(len(ld.t), len(ld.x), len(ld.y), len(ld.v), 6)
        if n < 3:
          continue
        ts = [float(ld.t[k]) for k in range(n)]
        ys = [-float(ld.y[k]) for k in range(n)]   # 设备帧 y = -model y
        d = float(ld.x[0]) - RADAR_TO_CAMERA
        y = ys[0]
        v_rel = float(ld.v[0]) - mv
        rate = _slope(ts, ys)
      except Exception:
        continue
      if not all(math.isfinite(x) for x in (d, y, v_rel, rate)):
        continue
      if abs(y) < float(cfg["vision_min_abs_y_m"]) or abs(y) > float(cfg["vision_max_abs_y_m"]):
        continue
      inward = max(0.0, -math.copysign(1.0, y) * rate)
      lane_hw = self._lane_half_width(model_msg, max(d, 1.0))
      sc = self._score(d=d, y=y, v_rel=v_rel, inward=inward, v_ego=v_ego,
                       lane_hw=lane_hw, side=side)
      if sc is None:
        continue
      out = {
        "status": True, "radar": False, "score": sc["score"],
        "dRel": float(d), "yRel": float(y), "vRel": float(v_rel),
        "ttc": float(sc["ttc"]), "inwardSpeed": float(inward),
        "source": int(sc["source"]) | SRC_VISION,
        "trackId": -1, "ageFrames": int(cfg["min_hist_samples"]), "stamp": 0,
      }
      if best is None or out["score"] > best["score"]:
        best = out
    return best

  # -- 主入口 ----------------------------------------------------------------
  def update(self, points: Any, v_ego: float, model_msg: Any = None,
             amap_msg: Any = None, car_state: Any = None,
             now: float | None = None, stamp: int = 0) -> dict[str, Any]:
    now = time.monotonic() if now is None else now
    self._maybe_reload(now)
    out = _empty_out()
    out["stamp"] = int(stamp)

    if not self.cfg["enabled"] or int(self.cfg["mode"]) < 1:
      self.last_out = out
      return out

    # 1) 采集当前帧点
    cur: dict[int, tuple[float, float, float]] = {}
    try:
      for p in points:
        try:
          tid = int(p.trackId)
          d = float(p.dRel)
          y = float(p.yRel)
          v = float(p.vRel)
        except Exception:
          continue
        if tid == 0 or not all(math.isfinite(x) for x in (d, y, v)) or d <= 0.0:
          continue
        cur[tid] = (d, y, v)
    except Exception:
      cur = {}

    # 2) 关联 + 历史累积
    assoc = _associate(self._last_pts, cur,
                       float(self.cfg["assoc_max_drel_m"]),
                       float(self.cfg["assoc_max_yrel_m"]),
                       float(self.cfg["assoc_max_vrel_mps"]))
    hist: dict[int, deque] = {}
    tid_of: dict[int, int] = {}
    for tid, (d, y, v) in cur.items():
      prev_tid = assoc.get(tid)
      hkey = self._id2h.get(prev_tid) if prev_tid is not None else None
      if hkey is None:
        hkey = self._next_h
        self._next_h += 1
        h = deque(maxlen=int(self.cfg["hist_len"]))
      else:
        h = self._hist.get(hkey) or deque(maxlen=int(self.cfg["hist_len"]))
      if len(h):
        pt, pd, py, pv = h[-1]
        dt = now - pt
        if (dt > float(self.cfg["max_dt_gap_s"]) or dt <= 0.0
            or abs(d - pd) > float(self.cfg["max_frame_drel_jump_m"])
            or abs(y - py) > float(self.cfg["max_frame_y_jump_m"])
            or abs(v - pv) > float(self.cfg["max_frame_vrel_jump_mps"])):
          h.clear()
      h.append((now, d, y, v))
      hist[hkey] = h
      tid_of[hkey] = tid

    self._hist = hist
    self._id2h = {tid: hkey for hkey, tid in tid_of.items()}
    self._last_pts = cur

    # 3) 逐条轨迹打分
    side = self._side_ctx(amap_msg, car_state, now)
    best: dict[str, Any] | None = None
    min_samples = max(3, int(self.cfg["min_hist_samples"]))
    for hkey, h in hist.items():
      if len(h) < min_samples:
        continue
      d, y, v = h[-1][1], h[-1][2], h[-1][3]
      ts = [s[0] for s in h]
      ys = [s[2] for s in h]
      rate = _slope(ts, ys)
      inward = max(0.0, -math.copysign(1.0, y) * rate) if y != 0.0 else 0.0
      lane_hw = self._lane_half_width(model_msg, d)
      sc = self._score(d=d, y=y, v_rel=v, inward=inward, v_ego=v_ego,
                       lane_hw=lane_hw, side=side)
      if sc is None:
        continue
      cand = {
        "status": True, "radar": True, "score": sc["score"],
        "dRel": float(d), "yRel": float(y), "vRel": float(v),
        "ttc": float(sc["ttc"]), "inwardSpeed": float(inward),
        "source": int(sc["source"]) | SRC_FRONT_RADAR,
        "trackId": int(tid_of[hkey]), "ageFrames": len(h), "stamp": int(stamp),
      }
      if best is None or cand["score"] > best["score"]:
        best = cand

    # 4) 雷达腿不成立时退到视觉腿（雷达 trackId 碎片化时的兜底）
    if best is None or best["score"] < float(self.cfg["min_score"]):
      vis = self._vision_leg(model_msg, v_ego, side)
      if vis is not None and (best is None or vis["score"] > best["score"]):
        vis["stamp"] = int(stamp)
        best = vis

    # 5) 阈值裁决
    if best is not None and best["score"] >= float(self.cfg["min_score"]):
      out = best
    else:
      if best is not None:
        # 保留低分候选的距离/速度便于日志对账，但 status=False
        out["dRel"] = best["dRel"]
        out["yRel"] = best["yRel"]
        out["vRel"] = best["vRel"]
        out["score"] = best["score"]
        out["inwardSpeed"] = best["inwardSpeed"]
        out["source"] = best["source"]
        out["trackId"] = best["trackId"]
        out["ageFrames"] = best["ageFrames"]
      out["status"] = False

    out["stamp"] = int(stamp)
    self.last_out = out

    if self.cfg["log"] and (now - self._last_log_t) >= float(self.cfg["log_period_s"]):
      self._last_log_t = now
      print(
        f"[cutin] st={int(out['status'])} sc={out['score']:.2f} "
        f"d={out['dRel']:.1f} y={out['yRel']:.2f} vr={out['vRel']:.2f} "
        f"ttc={out['ttc']:.1f} in={out['inwardSpeed']:.2f} src={out['source']} "
        f"tid={out['trackId']} age={out['ageFrames']} ntr={len(cur)} mode={self.cfg['mode']}",
        flush=True,
      )

    return out


# ---------------------------------------------------------------------------
# 预减速（planner 侧）
# ---------------------------------------------------------------------------
def _risk_get(lead: Any, key: str, default: Any = None) -> Any:
  """leadCutInRisk 可能是 capnp reader，也可能是离线工具传的 dict。"""
  if isinstance(lead, dict):
    return lead.get(key, default)
  try:
    return getattr(lead, key)
  except Exception:
    return default


def get_cutin_predecel_accel_limit(radar_state: Any, cfg: dict[str, Any] | None = None) -> float | None:
  """从 radarState.leadCutInRisk 换算加速度上界；不成立时返回 None。"""
  cfg = load_config_cached() if cfg is None else cfg
  if not cfg.get("enabled") or int(cfg.get("mode", 1)) < 2:
    return None
  lead = getattr(radar_state, "leadCutInRisk", None)
  if lead is None and (isinstance(radar_state, dict) or hasattr(radar_state, "status")):
    # 兼容：离线工具/单测直接把 risk 本体（dict 或 namespace）传进来。
    # 真 RadarState 永远带 leadCutInRisk 字段（默认 status=False），走上面那条；
    # 旧 schema 的 reader 两者都不满足 -> 返回 None（fail-safe，不预减速）。
    lead = radar_state
  if lead is None:
    return None

  if not bool(_risk_get(lead, "status", False)) or not bool(_risk_get(lead, "radar", False)):
    return None
  try:
    score = float(_risk_get(lead, "score", 0.0))
    d_rel = float(_risk_get(lead, "dRel", 0.0))
    v_rel = float(_risk_get(lead, "vRel", 0.0))
  except Exception:
    return None
  if score < float(cfg["predecel_min_score"]):
    return None
  if not (math.isfinite(d_rel) and math.isfinite(v_rel)) or d_rel <= 0.0 or v_rel >= 0.0:
    return None

  ttc_lo = float(cfg["predecel_min_ttc_s"])
  ttc_hi = float(cfg["predecel_max_ttc_s"])
  ttc = _clip(d_rel / max(-v_rel, 0.1), ttc_lo, ttc_hi)
  fraction = (ttc - ttc_lo) / max(ttc_hi - ttc_lo, 1e-3)
  a_lo = float(cfg["predecel_min_accel"])
  a_hi = float(cfg["predecel_max_accel"])
  return a_lo + fraction * (a_hi - a_lo)


def apply_cutin_predecel_accel_limit(maximum_accel: float, a_desired: float,
                                     predecel_limit: float | None,
                                     cfg: dict[str, Any] | None = None) -> float:
  """把预减速上界并入加速度目标，并限制单帧阶跃（jerk）。"""
  cfg = load_config_cached() if cfg is None else cfg
  if predecel_limit is None:
    return float(maximum_accel)
  step = float(cfg.get("predecel_jerk_step", 0.15))
  requested = min(float(maximum_accel), float(predecel_limit))
  return max(requested, float(a_desired) - step)
