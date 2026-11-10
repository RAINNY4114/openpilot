#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time

import numpy as np

HALF_CAR_WIDTH = 0.90
MIN_SAFETY_MARGIN = 0.25
MAX_SAFETY_MARGIN = 0.60
CLOSE_DIST = 10.0
FAR_DIST = 30.0
VRU_EXTRA_MARGIN = 0.50

OBS_MAX_LONG = 40.0
OBS_MIN_LONG = 0.5
OBS_MAX_LAT = 3.2

OBS_HALF_WIDTH = {
  "pedestrian": 0.25,
  "motorcycle": 0.35,
  "bicycle": 0.30,
  "parked": 0.90,
  "static": 0.60,
  "oncoming": 0.90,
  "unknown": 0.60,
}

IN_PATH_HALF_WIDTH = 0.45
MIN_AVOID_DIST = 5.0

PROFILE_SPAN_IDX = 4
SMOOTH_ALPHA = 0.12
RATE_LIMIT_PER_FRAME = 0.02
EDGE_SAFETY = 0.30

PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
LOG_PATH = "/data/media/0/obstacle_avoid.log"
CFG_REFRESH_SEC = 1.0

AUX_PATH = "/dev/shm/mr76_targets.json"
AUX_STALE_SEC = 0.5
AUX_MAX_TARGETS = 32

LIDAR_WALL_Y = 1.75
LIDAR_WALL_GAP = 0.50

def _read_raw(key):
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, key)) as f:
        return f.read().strip()
    except Exception:
      continue
  return None

def _read_num(key, default, cast=float):
  raw = _read_raw(key)
  if not raw:
    return default
  try:
    return cast(float(raw))
  except (TypeError, ValueError):
    return default

def _log(msg):
  try:
    with open(LOG_PATH, "a") as f:
      f.write(time.strftime("%m-%d %H:%M:%S ") + msg + "\n")
  except Exception:
    pass

def _aux_fresh(data):
  if "fresh" not in data:
    return True
  v = data["fresh"]
  if v is None:
    return True
  if isinstance(v, str):
    return v.strip().lower() not in ("0", "false", "no", "off", "")
  try:
    return bool(v)
  except Exception:
    return True

class ObstacleAvoidPlanner:
  def __init__(self):
    self._cfg = {}
    self._cfg_ts = 0.0
    self._log_ts = 0.0
    self._profile = None
    self._n = 0
    self._lidar_seen = {"left": False, "right": False}
    self._aux_ok_ts = 0.0
    self._aux_n = 0
    self._aux_dead = False
    self.diag = {
      "enabled": False, "active": False, "n_obs": 0,
      "closest": float("inf"), "shift": 0.0,
      "conflict": False, "why": "init",
      "aux": 0, "aux_dead": False, "lidar": "-",
    }

  def _load_cfg(self):
    now = time.monotonic()
    if now - self._cfg_ts < CFG_REFRESH_SEC and self._cfg:
      return self._cfg
    self._cfg_ts = now
    self._cfg = {
      "enabled": _read_num("ObstacleAvoidEnabled", 0, int) != 0,
      "active": _read_num("ObstacleAvoidActive", 0, int) != 0,
      "max_shift": max(0.0, min(_read_num("ObstacleAvoidMaxShift", 0.90), 1.8)),
      "min_kph": _read_num("ObstacleAvoidMinKph", 8.0),
      "max_kph": _read_num("ObstacleAvoidMaxKph", 110.0),
      "vru_extra": max(0.0, min(_read_num("ObstacleAvoidVruExtra", VRU_EXTRA_MARGIN), 1.0)),
      "max_long": max(10.0, min(_read_num("ObstacleAvoidMaxLong", OBS_MAX_LONG), 60.0)),
    }
    return self._cfg

  @staticmethod
  def _cluster(points):
    clusters = []
    for d, y, vr in points:
      hit = None
      for c in clusters:
        if abs(c[0] - d) < 2.0 and abs(c[1] - y) < 1.0:
          hit = c
          break
      if hit is None:
        clusters.append([d, y, vr, 1])
      else:
        n = hit[3]
        hit[0] = (hit[0] * n + d) / (n + 1)
        hit[1] = (hit[1] * n + y) / (n + 1)
        hit[2] = (hit[2] * n + vr) / (n + 1)
        hit[3] = n + 1
    return [(c[0], c[1], c[2]) for c in clusters]

  @staticmethod
  def _classify(d, y, vr):
    if vr < -3.0:
      return "oncoming"
    if abs(vr) < 1.0:
      return "parked" if abs(y) > 1.0 else "static"
    if vr < 3.0 and abs(y) < 1.6:
      return "motorcycle"
    return "unknown"

  def _collect(self, sm, max_long):
    obs = []
    try:
      if sm.seen.get("radarTracks", False):
        pts = []
        for p in sm["radarTracks"].points:
          d = float(p.dRel)
          y = float(p.yRel)
          vr = float(p.vRel)
          if OBS_MIN_LONG < d < max_long and abs(y) < OBS_MAX_LAT:
            pts.append((d, y, vr))
        obs = [(d, y, vr, None) for (d, y, vr) in self._cluster(pts)]
    except Exception:
      obs = []

    try:
      if sm.seen.get("radarState", False):
        for ld in (sm["radarState"].leadOne, sm["radarState"].leadTwo):
          if not ld.status:
            continue
          d = float(ld.dRel)
          y = float(ld.yRel)
          vr = float(ld.vRel)
          if not (OBS_MIN_LONG < d < max_long and abs(y) < OBS_MAX_LAT):
            continue
          if any(abs(d - o[0]) < 2.0 and abs(y - o[1]) < 1.0 for o in obs):
            continue
          obs.append((d, y, vr, None))
    except Exception:
      pass
    return obs

  def _read_aux(self):
    self._aux_n = 0
    self._aux_dead = False
    try:
      with open(AUX_PATH) as f:
        raw = f.read()
      if not raw:
        return []
      data = json.loads(raw)
      ts = float(data.get("t", 0.0))
      if ts <= 0.0 or (time.time() - ts) > AUX_STALE_SEC:
        return []
      if not _aux_fresh(data):
        self._aux_dead = True
        return []
      out = []
      for item in data.get("targets", [])[:AUX_MAX_TARGETS]:
        try:
          d = float(item[0])
          y = float(item[1])
          vr = float(item[2]) if len(item) > 2 else 0.0
        except (TypeError, ValueError, IndexError):
          continue
        if not (OBS_MIN_LONG < d < OBS_MAX_LONG and abs(y) < OBS_MAX_LAT):
          continue
        cls = None
        if len(item) > 3 and isinstance(item[3], str) and item[3] in OBS_HALF_WIDTH:
          cls = item[3]
        out.append((d, y, vr, cls))
      self._aux_n = len(out)
      if out:
        self._aux_ok_ts = time.time()
      return out
    except Exception:
      return []

  def _lidar_walls(self, sm):
    walls = []
    try:
      if not (sm.seen.get("amapNavi", False) and sm.valid.get("amapNavi", False)):
        return walls
      n = sm["amapNavi"]
      lb = int(n.leftBlind)
      rb = int(n.rightBlind)
      if lb & 0b0101:
        self._lidar_seen["left"] = True
      if rb & 0b0101:
        self._lidar_seen["right"] = True
      if self._lidar_seen["left"] and (lb & 0b0101):
        walls.append((LIDAR_WALL_Y, LIDAR_WALL_GAP))
      if self._lidar_seen["right"] and (rb & 0b0101):
        walls.append((-LIDAR_WALL_Y, LIDAR_WALL_GAP))
    except Exception:
      pass
    return walls

  def _build_profile(self, obs, t_idxs, v_ego, max_shift, vru_extra, walls=()):
    n = len(t_idxs)
    profile = np.zeros(n, dtype=np.float64)
    if not obs and not walls:
      return profile, float("inf"), False

    envelope = []
    envelope.extend(walls)
    min_dist = float("inf")
    for d, y, vr, cls_hint in obs:
      min_dist = min(min_dist, d)
      cls = cls_hint or self._classify(d, y, vr)

      if d < CLOSE_DIST:
        margin = MAX_SAFETY_MARGIN
      else:
        t = np.clip((d - CLOSE_DIST) / (FAR_DIST - CLOSE_DIST), 0.0, 1.0)
        margin = MAX_SAFETY_MARGIN + (MIN_SAFETY_MARGIN - MAX_SAFETY_MARGIN) * t

      is_vru = cls in ("pedestrian", "bicycle", "motorcycle")
      gap = OBS_HALF_WIDTH.get(cls, 0.60) + HALF_CAR_WIDTH + margin + (vru_extra if is_vru else 0.0)

      if abs(y) < IN_PATH_HALF_WIDTH:
        continue
      if d < MIN_AVOID_DIST:
        continue

      envelope.append((y, gap))

      req = max(0.0, gap - abs(y))
      if req <= 0.0:
        continue
      push = -np.sign(y) * req

      t_obs = d / max(v_ego, 2.0)
      idx = int(np.clip(np.interp(t_obs, t_idxs, np.arange(n)), 0, n - 1))
      i0 = max(0, idx - PROFILE_SPAN_IDX)
      i1 = min(n, idx + PROFILE_SPAN_IDX + 1)
      for i in range(i0, i1):
        w = 1.0 - abs(i - idx) / (PROFILE_SPAN_IDX + 1.0)
        if push < 0:
          profile[i] = min(profile[i], push * w)
        else:
          profile[i] = max(profile[i], push * w)

    conflict = False
    if envelope:
      cand = np.linspace(-1.8, 1.8, 73)
      for i in range(n):
        near = envelope
        raw = float(profile[i])
        best_y, best_score, max_mc = raw, -1e9, -1e9
        for y in cand:
          mc = min(abs(o_y - y) - g for (o_y, g) in near)
          max_mc = max(max_mc, mc)
          score = -abs(y - raw) if mc >= 0.0 else mc - 0.01 * abs(y - raw)
          if score > best_score:
            best_score, best_y = score, y
        profile[i] = best_y
        if max_mc < 0.0:
          conflict = True

    profile = np.clip(profile, -max_shift, max_shift)
    return profile, min_dist, conflict

  def update(self, sm, plan, t_idxs, v_ego, lat_active):
    try:
      cfg = self._load_cfg()
      self.diag["enabled"] = cfg["enabled"]
      self.diag["active"] = cfg["active"]
      if not cfg["enabled"]:
        self.diag["why"] = "disabled"
        self._profile = None
        return None

      v_kph = v_ego * 3.6
      if not lat_active:
        self.diag["why"] = "lat_inactive"
        self._decay()
        return self._emit(cfg)
      if v_kph < cfg["min_kph"]:
        self.diag["why"] = "slow"
        self._decay()
        return self._emit(cfg)
      if v_kph > cfg["max_kph"]:
        self.diag["why"] = "fast"
        self._decay()
        return self._emit(cfg)

      n = len(t_idxs)
      if self._profile is None or self._n != n:
        self._profile = np.zeros(n, dtype=np.float64)
        self._n = n

      obs = self._collect(sm, cfg["max_long"])
      aux = self._read_aux()
      for a in aux:
        if any(abs(a[0] - o[0]) < 2.0 and abs(a[1] - o[1]) < 1.0 for o in obs):
          continue
        obs.append(a)
      walls = self._lidar_walls(sm)

      self.diag["n_obs"] = len(obs)
      self.diag["aux"] = self._aux_n
      self.diag["aux_dead"] = self._aux_dead
      self.diag["lidar"] = ("L" if self._lidar_seen["left"] else "-") + \
                           ("R" if self._lidar_seen["right"] else "-")
      target, min_dist, conflict = self._build_profile(
        obs, t_idxs, v_ego, cfg["max_shift"], cfg["vru_extra"], walls)
      self.diag["closest"] = min_dist
      self.diag["conflict"] = conflict

      smooth = SMOOTH_ALPHA * target + (1.0 - SMOOTH_ALPHA) * self._profile
      delta = np.clip(smooth - self._profile, -RATE_LIMIT_PER_FRAME, RATE_LIMIT_PER_FRAME)
      self._profile = np.clip(self._profile + delta, -cfg["max_shift"], cfg["max_shift"])

      self._profile = self._clamp_to_lane(sm, plan, self._profile)

      self.diag["shift"] = float(self._profile[0])
      self.diag["why"] = "run"
      return self._emit(cfg)
    except Exception as e:
      self.diag["why"] = f"err:{e}"
      return None

  def _decay(self):
    if self._profile is None:
      return
    self._profile = self._profile * 0.85
    if np.max(np.abs(self._profile)) < 1e-3:
      self._profile = None

  def _clamp_to_lane(self, sm, plan, profile):
    try:
      md = sm["modelV2"]
      y0 = float(plan[0, 1])
      left_bound = None
      right_bound = None
      if len(md.laneLines) >= 4:
        ly = float(np.array(md.laneLines[1].y)[0])
        ry = float(np.array(md.laneLines[2].y)[0])
        if md.laneLineProbs[1] > 0.5:
          left_bound = ly + EDGE_SAFETY
        if md.laneLineProbs[2] > 0.5:
          right_bound = ry - EDGE_SAFETY
      if left_bound is None and len(md.roadEdges) >= 2 and md.roadEdgeProbs[0] > 0.3:
        left_bound = float(np.array(md.roadEdges[0].y)[0]) + EDGE_SAFETY
      if right_bound is None and len(md.roadEdges) >= 2 and md.roadEdgeProbs[1] > 0.3:
        right_bound = float(np.array(md.roadEdges[1].y)[0]) - EDGE_SAFETY
      if left_bound is not None and right_bound is not None and left_bound < right_bound:
        lo = left_bound - y0
        hi = right_bound - y0
        profile = np.clip(profile, lo, hi)
    except Exception:
      pass
    return profile

  def _emit(self, cfg):
    now = time.monotonic()
    if now - self._log_ts >= 1.0:
      self._log_ts = now
      d = self.diag
      _log(f"[OA] why={d['why']} obs={d['n_obs']} aux={d['aux']} "
           f"auxdead={int(d['aux_dead'])} lidar={d['lidar']} "
           f"closest={d['closest']:.1f} shift={d['shift']:.2f} "
           f"conflict={int(d['conflict'])} active={int(cfg['active'])}")
    if not cfg["active"]:
      return None
    if self._profile is None:
      return None
    return self._profile.copy()
