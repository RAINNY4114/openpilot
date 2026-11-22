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
  - AUX cls=oncoming / radarTracks vRel<-3.0 with |y|>1.0 -> true oncoming (NOT leadOne.vRel>0)
  - sm['radarTracks'].points  (dRel, yRel, vRel)         -> static roadside obs / oncoming geometry
  - sm['amapNavi'].leftBlind / rightBlind (bitmask)      -> narrow-road hint
  - /dev/shm/mr76_targets.json (AUX targets [d,y,vr,cls])-> cone/ped/bike list
  - sm['carState'].vEgo, sm['carControl'].latActive

Features:
  1) static obstacle lane shrink  (RoadObsShrinkEnabled)
  2) two-wheeler / pedestrian independent left shift (RoadVruShiftEnabled)
  3) risk-gated oncoming slowdown (RoadNarrowEnabled):
     brake only for narrow width / real collision / side-scrape risk.
     A well-separated oncoming car staying in its own lane never brakes.
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
NARROW_SLOW_KPH = 10.0    # narrow road (width insufficient): slow to 10 kph
COLLISION_SLOW_KPH = 15.0 # real head-on collision risk: slow to 15 kph
NORMAL_SLOW_KPH = 25.0    # cpv9 normal-road oncoming target
SCRAPE_SLOW_KPH = 20.0    # side-scrape risk: gentle deceleration
RELEASE_SEC = 1.5           # cpv9 hold time after risk disappears

# oncoming recognition (unchanged: true oncoming, not same-lane following car)
ONCOMING_MIN_DIST = 0.5
ONCOMING_MAX_DIST = 50.0    # cpv9 oncoming dist
ONCOMING_LAT_MIN = 1.0      # target must be in adjacent/opposite lane (|y| > this), NOT our lane
ONCOMING_LAT_MAX = 4.0      # beyond this is off-road clutter, not an oncoming vehicle
ONCOMING_VREL_MARGIN = 2.0  # radarTracks fallback: target abs speed (vRel+vEgo) < -margin => truly oncoming
ONCOMING_MIN_VEGO = 2.0     # only trust fallback while ego is moving (static objects have vRel=-vEgo)

# oncoming RISK thresholds (new): an oncoming car passing in its own lane is safe.
# Only when it intrudes laterally into our lane space does TTC/distance matter.
# Car body half-width ~0.9m, so two cars need >1.8m lateral center separation.
ONCOMING_SCRAPE_LAT = 1.6   # |y| below this = lateral intrusion / scrape risk (tunable)
ONCOMING_RISK_TTC = 3.0     # s, closing TTC below this while intruding = collision risk (tunable)
ONCOMING_NEAR_DIST = 12.0   # m, intruding target this close is risky even at low closing (tunable)
# Far targets have unreliable lateral precision (radar y jitter seen 1.3~2.8m at
# 40m in real logs). A distant oncoming car drifting across the |y| threshold
# must NOT trigger a hard brake -- that was a residual cause of frequent braking.
# Collision/scrape risk is only trusted inside this longitudinal range.
ONCOMING_RISK_MAX_DIST = 30.0  # m, ignore risk from targets farther than this (tunable)
BLIND_BOTH_MASK = 0b0101    # bit pattern used by OA for lidar wall


class RoadGameEnhance:
  def __init__(self):
    self._profile = None
    self._n = 0
    self._last_oncoming_ts = 0.0
    self._last_log_ts = 0.0
    self.diag = {
      "enabled": False, "active": False, "why": "init",
      "shrink": 0.0, "vru_shift": 0.0, "accel_cap": None,
      "narrow": False, "oncoming": False, "oncoming_src": "none",
      "risk_reason": "none", "risk_lat": None, "risk_ttc": None,
      "n_obs": 0,
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

  @staticmethod
  def _oncoming_targets(obs, v_ego):
    """Collect TRUE oncoming targets [(d, y, vr, cls)].

    Source priority: AUX cls=oncoming first; radarTracks vRel < -(vEgo+margin)
    fallback only for unclassified raw points in the adjacent-lane band.
    Never uses same-lane leadOne.vRel>0 -- a following car is not oncoming.
    """
    res = []
    for d, y, vr, cls in obs:
      if cls == "oncoming" and ONCOMING_MIN_DIST < d < ONCOMING_MAX_DIST and abs(y) > ONCOMING_LAT_MIN:
        res.append((d, y, vr, cls))
    if not res and v_ego > ONCOMING_MIN_VEGO:
      for d, y, vr, cls in obs:
        if cls != "":
          continue  # AUX already classified; fallback only on raw radarTracks points
        ay = abs(y)
        if (ONCOMING_MIN_DIST < d < ONCOMING_MAX_DIST and ONCOMING_LAT_MIN < ay < ONCOMING_LAT_MAX
            and vr < -(v_ego + ONCOMING_VREL_MARGIN)):
          res.append((d, y, vr, cls))
    return res

  @staticmethod
  def _oncoming_risk(targets, scrape_lat, risk_ttc, near_dist, risk_max_dist):
    """Risk level from oncoming geometry. Returns (level, min_lat, min_ttc).

    Key idea: an oncoming car well-separated laterally (|y| >= scrape_lat) is
    SAFE even with a tiny TTC -- the two vehicles simply pass side by side in
    separate lanes. Risk only exists when the target intrudes into our lane
    space (|y| < scrape_lat) AND is genuinely close/closing.
      - collision: intruding + within risk_max_dist + closing TTC < risk_ttc
      - scrape:    intruding + within near_dist (even if TTC undefined)
      - none:      laterally separated, too far, or not closing
    Remote targets (>risk_max_dist) are ignored: their lateral reading is noisy
    (real logs show y jitter 1.3~2.8m at 40m), so a distant car crossing the
    threshold must not cause a hard brake.
    """
    level = "none"
    min_lat = None
    min_ttc = None
    for d, y, vr, cls in targets:
      ay = abs(y)
      closing = -float(vr)
      ttc = (float(d) / closing) if closing > 0.01 else None
      if min_lat is None or ay < min_lat:
        min_lat = ay
      if ttc is not None and (min_ttc is None or ttc < min_ttc):
        min_ttc = ttc
      if ay >= scrape_lat or float(d) > risk_max_dist:
        continue  # laterally separated OR too far to trust: no risk
      if ttc is not None and ttc < risk_ttc:
        level = "collision"
      elif float(d) < near_dist and level != "collision":
        level = "scrape"
    return level, min_lat, min_ttc

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

      # --- 3) RISK-GATED oncoming slowdown (fixed judge) ---
      # Old behaviour braked on ANY true oncoming target (narrow=0 oncoming=1
      # cap=-1.5 in logs) -> frequent braking on normal two-lane meetings.
      # New behaviour: recognize oncoming, then brake ONLY if one of:
      #   (a) narrow road (nav blind bitmask) -> width insufficient to pass
      #   (b) collision risk: intruding |y| + short closing TTC
      #   (c) side-scrape risk: intruding |y| + close distance
      # A well-separated oncoming car in its own lane (|y| >= scrape_lat) is safe.
      narrow = self._narrow_hint(sm)
      oncoming = False
      oncoming_src = "none"
      risk_reason = "none"
      risk_lat = None
      risk_ttc = None
      if cfg["narrow_on"]:
        now = time.time()
        try:
          targets = self._oncoming_targets(obs, v_ego)
          oncoming = bool(targets)
          if oncoming:
            oncoming_src = "aux" if targets[0][3] == "oncoming" else "radar"
          scrape_lat = _read_num("OncomingScrapeLat", ONCOMING_SCRAPE_LAT, float)
          risk_ttc_cfg = _read_num("OncomingRiskTtc", ONCOMING_RISK_TTC, float)
          near_dist = _read_num("OncomingNearDist", ONCOMING_NEAR_DIST, float)
          risk_max_dist = _read_num("OncomingRiskMaxDist", ONCOMING_RISK_MAX_DIST, float)
          risk_level, risk_lat, risk_ttc = self._oncoming_risk(
            targets, scrape_lat, risk_ttc_cfg, near_dist, risk_max_dist)
          if oncoming and narrow:
            risk_reason = "narrow"
          elif oncoming and risk_level != "none":
            risk_reason = risk_level
          if risk_reason != "none":
            self._last_oncoming_ts = now
        except Exception:
          pass
        active = (risk_reason != "none") or (now - self._last_oncoming_ts < RELEASE_SEC)
        if active:
          # narrow (width insufficient) -> 10 kph; collision -> 15 kph;
          # scrape -> 20 kph; held after risk clears -> normal 25 kph
          if risk_reason == "narrow":
            vmax_kph = NARROW_SLOW_KPH
          elif risk_reason == "collision":
            vmax_kph = COLLISION_SLOW_KPH
          elif risk_reason == "scrape":
            vmax_kph = SCRAPE_SLOW_KPH
          else:
            vmax_kph = NORMAL_SLOW_KPH
          vmax = vmax_kph / 3.6
          if v_ego > vmax:
            accel_cap = max(-1.5, (vmax - v_ego) / 1.5)  # gentle, never stronger
          self.diag["active"] = True
        else:
          self.diag["active"] = False
        self.diag["narrow"] = narrow
        self.diag["oncoming"] = oncoming
        self.diag["oncoming_src"] = oncoming_src
      else:
        self.diag["active"] = False

      self.diag["risk_reason"] = risk_reason
      self.diag["risk_lat"] = risk_lat
      self.diag["risk_ttc"] = risk_ttc

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
        _lat = "na" if risk_lat is None else f"{risk_lat:.2f}"
        _ttc = "na" if risk_ttc is None else f"{risk_ttc:.2f}"
        _log(f"[RGE] en={int(self.diag['enabled'])} act={int(self.diag['active'])} "
             f"n={len(obs)} shrink={shrink_total:.2f} vru={vru_shift:.2f} "
             f"narrow={int(narrow)} oncoming={int(oncoming)} src={oncoming_src} "
             f"risk={risk_reason} lat={_lat} ttc={_ttc} cap={accel_cap}")

      lat_out = self._profile.copy() if (cfg["shrink_on"] or cfg["vru_on"]) else None
      return lat_out, accel_cap

    except Exception as e:
      self.diag["why"] = f"err:{type(e).__name__}"
      return None, None
