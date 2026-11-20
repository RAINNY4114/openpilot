#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Road-scenario enhancements borrowed from cpv9 (road_game.py + lane_planner_2.py),
re-implemented against this fork's actual available signals.

Design goals:
  - Fail-safe: any error / missing signal -> zero lateral offset, no accel cap.
  - Opt-in per feature via params (default OFF). Reuses the direct-read pattern
    from obstacle_avoid.py so no UnknownKeyName issues.
  - Independent from ObstacleAvoidPlanner: this module never modifies that code.

Signals used (all present in modeld_v2's SubMaster):
  - sm['radarState'].leadOne  (dRel, vRel, vLead, yRel)  -> oncoming/lead
  - sm['radarTracks'].points  (dRel, yRel, vRel)         -> static roadside obs
  - sm['amapNavi'].leftBlind / rightBlind (bitmask)      -> narrow-road hint
  - /dev/shm/mr76_targets.json (AUX targets [d,y,vr,cls])-> cone/ped/bike list
  - sm['carState'].vEgo, sm['carControl'].latActive

Features:
  1) static obstacle lane shrink  (RoadObsShrinkEnabled)
  2) two-wheeler / pedestrian independent left shift (RoadVruShiftEnabled)
  3) narrow-road oncoming slowdown (RoadNarrowEnabled)
"""

import json
import os
import time

import numpy as np

# --- param direct-read (same pattern as obstacle_avoid.py) ---
PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
AUX_PATH = "/dev/shm/mr76_targets.json"
AUX_STALE_SEC = 0.5
LOG_PATH = "/data/media/0/road_game_enhance.log"

def _read_num(key, default, cast=float):
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, key)) as f:
        raw = f.read().strip()
      return cast(float(raw))
    except Exception:
      continue
  return default

def _log(msg):
  try:
    with open(LOG_PATH, "a") as f:
      f.write(time.strftime("%m-%d %H:%M:%S ") + msg + "\n")
  except Exception:
    pass

# --- constants (from cpv9, tuned conservative) ---
OBS_MAX_LONG = 28.0       # cpv9 OBSTACLE_MAX_LONG_DIST
OBS_MIN_LONG = 0.5
LANE_SHRINK_MAX = 0.32    # cpv9 LANE_SHRINK_MAX_SIDE
SHRINK_PRE_SHIFT_MAX = 0.45  # cpv9 STATIC_OBS_PRE_SHIFT_MAX
VRU_SHIFT_MAX = 0.55      # cpv9 TW_AVOID_SHIFT
SMOOTH_ALPHA = 0.12
RATE_LIMIT_PER_FRAME = 0.02
PROFILE_SPAN_IDX = 4
ROADSIDE_LAT_MIN = 0.8    # |y| above this = roadside
VRU_LAT_MAX = 1.6         # two-wheeler considered in-path up to this |y|
NARROW_LANE_WIDTH_M = 3.0 # cpv9 narrow threshold
NARROW_SLOW_KPH = 15.0    # cpv9 narrow-road oncoming target
NORMAL_SLOW_KPH = 25.0    # cpv9 normal-road oncoming target
ONCOMING_MIN_DIST = 0.5
ONCOMING_MAX_DIST = 50.0  # cpv9 oncoming dist
ONCOMING_REL_SPEED = 0.0  # cpv9 relSpeed > 0 (closing)
BLIND_BOTH_MASK = 0b0101  # bit pattern used by OA for lidar wall
RELEASE_SEC = 1.5         # cpv9 hold time after oncoming disappears


class RoadGameEnhance:
  def __init__(self):
    self._profile = None
    self._n = 0
    self._last_oncoming_ts = 0.0
    self._last_log_ts = 0.0
    self.diag = {
      "enabled": False, "active": False, "why": "init",
      "shrink": 0.0, "vru_shift": 0.0, "accel_cap": None,
      "narrow": False, "oncoming": False, "n_obs": 0,
    }

  def _cfg(self):
    return {
      "shrink_on": _read_num("RoadObsShrinkEnabled", 0, int) != 0,
      "vru_on":    _read_num("RoadVruShiftEnabled", 1, int) != 0,
      "narrow_on": _read_num("RoadNarrowEnabled", 0, int) != 0,
    }

  def _read_aux(self):
    """Return AUX targets [(d, y, vr, cls), ...] from mr76d feed."""
    out = []
    try:
      with open(AUX_PATH) as f:
        raw = f.read()
      if not raw:
        return out
      data = json.loads(raw)
      ts = float(data.get("t", 0.0))
      if ts <= 0.0 or (time.time() - ts) > AUX_STALE_SEC:
        return out
      if data.get("fresh") is False:
        return out
      for item in data.get("targets", [])[:32]:
        try:
          d = float(item[0]); y = float(item[1])
          vr = float(item[2]) if len(item) > 2 else 0.0
          cls = item[3] if len(item) > 3 and isinstance(item[3], str) else ""
        except Exception:
          continue
        if OBS_MIN_LONG < d < OBS_MAX_LONG and abs(y) < 3.5:
          out.append((d, y, vr, cls))
    except Exception:
      pass
    return out

  @staticmethod
  def _radar_obs(sm):
    """Collect radarTracks roadside obstacles as (d, y, vr, cls)."""
    out = []
    try:
      if not (sm.seen.get("radarTracks", False) and sm.valid.get("radarTracks", False)):
        return out
      for p in sm["radarTracks"].points:
        d = float(p.dRel); y = float(p.yRel); vr = float(p.vRel)
        if OBS_MIN_LONG < d < OBS_MAX_LONG and abs(y) < 3.5:
          out.append((d, y, vr, ""))
    except Exception:
      pass
    return out

  @staticmethod
  def _narrow_hint(sm):
    """Narrow road hint from amapNavi blind bitmask (both sides blocked => narrow).
    Returns True if usable; False when signal absent (fail-safe: not narrow)."""
    try:
      if not (sm.seen.get("amapNavi", False) and sm.valid.get("amapNavi", False)):
        return False
      n = sm["amapNavi"]
      lb = int(n.leftBlind); rb = int(n.rightBlind)
      return bool((lb & BLIND_BOTH_MASK) and (rb & BLIND_BOTH_MASK))
    except Exception:
      return False

  def update(self, sm, t_idxs, v_ego, lat_active):
    """Return (lateral_offset_array_or_None, accel_cap_or_None)."""
    try:
      cfg = self._cfg()
      n = len(t_idxs)
      if self._profile is None or self._n != n:
        self._profile = np.zeros(n, dtype=np.float64)
        self._n = n
      self.diag["why"] = "run"
      self.diag["enabled"] = cfg["shrink_on"] or cfg["vru_on"] or cfg["narrow_on"]

      obs = self._read_aux() + self._radar_obs(sm)
      self.diag["n_obs"] = len(obs)

      target = np.zeros(n, dtype=np.float64)
      accel_cap = None

      # --- 1) static obstacle lane shrink (cpv9 update_obstacle_data) ---
      shrink_total = 0.0
      if cfg["shrink_on"]:
        for d, y, vr, cls in obs:
          if cls in ("cone", "pedestrian", "motorcycle", "bicycle", "oncoming"):
            continue  # handled separately / by VRU
          if abs(y) < ROADSIDE_LAT_MIN:
            continue
          # static-ish roadside object: low |vr|
          if abs(vr) > 2.0:
            continue
          dw = float(np.clip(1.0 - d / OBS_MAX_LONG, 0.0, 1.0))
          shrink = min(LANE_SHRINK_MAX, LANE_SHRINK_MAX * dw)
          push = -np.sign(y) * shrink * 0.5
          idx = int(np.clip(np.interp(d / max(v_ego, 2.0), t_idxs, np.arange(n)), 0, n - 1))
          i0 = max(0, idx - PROFILE_SPAN_IDX); i1 = min(n, idx + PROFILE_SPAN_IDX + 1)
          for i in range(i0, i1):
            w = 1.0 - abs(i - idx) / (PROFILE_SPAN_IDX + 1.0)
            target[i] += push * w
          shrink_total = max(shrink_total, shrink)
        target = np.clip(target, -SHRINK_PRE_SHIFT_MAX, SHRINK_PRE_SHIFT_MAX)

      # --- 2) two-wheeler / pedestrian independent left shift (cpv9 E5) ---
      vru_shift = 0.0
      if cfg["vru_on"]:
        for d, y, vr, cls in obs:
          if cls not in ("pedestrian", "motorcycle", "bicycle"):
            continue
          if not (-0.3 < y < VRU_LAT_MAX):  # right-side in-path VRU
            continue
          dw = float(np.clip(1.0 - d / OBS_MAX_LONG, 0.0, 1.0))
          vru_shift = min(vru_shift + VRU_SHIFT_MAX * dw, VRU_SHIFT_MAX)
        target -= vru_shift  # shift left, independent of lane offset

      # --- 3) narrow-road oncoming slowdown (cpv9 road_game) ---
      narrow = self._narrow_hint(sm)
      oncoming = False
      if cfg["narrow_on"]:
        now = time.time()
        try:
          if sm.seen.get("radarState", False) and sm.valid.get("radarState", False):
            lead = sm["radarState"].leadOne
            if getattr(lead, "present", False) or getattr(lead, "status", False):
              d = float(lead.dRel); vr = float(lead.vRel)
              if ONCOMING_MIN_DIST < d < ONCOMING_MAX_DIST and vr > ONCOMING_REL_SPEED:
                oncoming = True
                self._last_oncoming_ts = now
        except Exception:
          pass
        active = oncoming or (now - self._last_oncoming_ts < RELEASE_SEC)
        if active:
          vmax_kph = NARROW_SLOW_KPH if narrow else NORMAL_SLOW_KPH
          vmax = vmax_kph / 3.6
          if v_ego > vmax:
            # gentle decel cap: -1.5 m/s^2, never stronger
            accel_cap = max(-1.5, (vmax - v_ego) / 1.5)
          self.diag["active"] = True
        else:
          self.diag["active"] = False
        self.diag["narrow"] = narrow
        self.diag["oncoming"] = oncoming
      else:
        self.diag["active"] = False

      # --- smooth + rate-limit lateral profile (same as OA) ---
      if cfg["shrink_on"] or cfg["vru_on"]:
        smooth = SMOOTH_ALPHA * target + (1.0 - SMOOTH_ALPHA) * self._profile
        delta = np.clip(smooth - self._profile, -RATE_LIMIT_PER_FRAME, RATE_LIMIT_PER_FRAME)
        self._profile = np.clip(self._profile + delta, -SHRINK_PRE_SHIFT_MAX - VRU_SHIFT_MAX,
                                SHRINK_PRE_SHIFT_MAX + VRU_SHIFT_MAX)
      else:
        self._profile = self._profile * 0.85  # decay when all lateral features off

      self.diag["shrink"] = shrink_total
      self.diag["vru_shift"] = vru_shift
      self.diag["accel_cap"] = accel_cap

      # --- logging (1 Hz) ---
      now = time.monotonic()
      if now - self._last_log_ts >= 1.0:
        self._last_log_ts = now
        _log(f"[RGE] en={int(self.diag['enabled'])} act={int(self.diag['active'])} "
             f"n={len(obs)} shrink={shrink_total:.2f} vru={vru_shift:.2f} "
             f"narrow={int(narrow)} oncoming={int(oncoming)} cap={accel_cap}")

      lat_out = self._profile.copy() if (cfg["shrink_on"] or cfg["vru_on"]) else None
      return lat_out, accel_cap

    except Exception as e:
      self.diag["why"] = f"err:{type(e).__name__}"
      return None, None
