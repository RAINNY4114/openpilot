#!/usr/bin/env python3

import os
import json
import math
import time
from typing import NamedTuple

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.controls.lib.auto_overtake import AutoOvertakeHelper, LANE_PREF_AUTO
from openpilot.selfdrive.controls.lib.auto_avoidance import AutoAvoidanceHelper

LaneChangeDirection = log.LaneChangeDirection

PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")

AO_LOG_PATH = "/data/media/0/auto_overtake.log"
AA_LOG_PATH = "/data/media/0/auto_avoidance.log"
LOG_MAX_BYTES = 5_000_000

SIDE_LEAD_DANGER_DIST = 5.0
# A side blind-spot target is dangerous when close, OR at moderate range but
# closing fast (a fast-approaching target in the zone is unsafe even a bit
# further out). Mirrors the device UI blind_spot_indicators definition.
SIDE_LEAD_CLOSING_DIST = 9.0
SIDE_LEAD_CLOSING_SPEED = 3.0  # m/s of |vRel| toward ego

MR76_AUX_PATH = "/dev/shm/mr76_targets.json"
MR76_AUX_STALE_SEC = 0.5
MR76_SIDE_LAT_MIN = 0.8
MR76_SIDE_DIST_MAX = 40.0

AA_CONFIRM_DELAY_SEC = 3.0
AA_BRIDGE_STABLE_SEC = 1.0
AA_LANE_CHANGE_MIN_KPH = 60.0

# carrot-wip-style radar evidence: current geometry must agree with a short
# measured history; no single-frame or extrapolated return can open the gate.
RADAR_MAX_GAP_S = 0.15
RADAR_MIN_SAMPLES = 3
RADAR_MIN_CONFIRM_S = 0.35
RADAR_PRED_D_ERR = 1.0
RADAR_PRED_Y_ERR = 0.30
RADAR_V_STEP_ERR = 1.5

# carrot-wip-style confident ego lane for lateral avoidance.
AA_LANE_PROB_MIN = 0.90
AA_LANE_STD_MAX = 0.25
AA_LANE_WIDTH_MIN = 2.5
AA_LANE_WIDTH_MAX = 4.3

CONFIRM_DELAY_SEC_DEFAULT = 3.0

AO_CFG_DEFAULTS = {
  "enabled": 0,
  "lane_preference": LANE_PREF_AUTO,
  "confirm_delay_sec": CONFIRM_DELAY_SEC_DEFAULT,
  "min_cruise_kph": 90.0,
  "lane_prob_min": 0.30,
  "require_lane_gain": 0,
}

AA_CFG_DEFAULTS = {
  "enabled": 0,
  "stable_mode": "ratio",
  "stable_ratio": 0.60,
  "stable_window_sec": 0.60,
  "obs_hold_sec": 0.45,
  "brake_gain": 1.0,
  "brake_max": 0.0,
  "brake_ttc_ref": 3.0,
}

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

def write_param(key, value):
  try:
    os.makedirs("/data/ford_params", exist_ok=True)
  except Exception:
    pass
  for d in ("/data/ford_params", "/dev/shm/params", "/data/params/d"):
    try:
      if not os.path.isdir(d):
        continue
      with open(os.path.join(d, str(key)), "w") as f:
        f.write(str(value))
      return True
    except Exception:
      continue
  return False

def read_cfg():
  ao = dict(AO_CFG_DEFAULTS)
  ao["enabled"] = _read_num("AutoOvertakeEnabled", ao["enabled"], int) != 0
  ao["lane_preference"] = _read_num("AutoOvertakeLanePref", ao["lane_preference"], int)
  ao["confirm_delay_sec"] = _read_num("AutoOvertakeConfirmSec", ao["confirm_delay_sec"])
  ao["min_cruise_kph"] = _read_num("AutoOvertakeMinCruiseKph", ao["min_cruise_kph"])
  ao["lane_prob_min"] = _read_num("AutoOvertakeLaneProbMin", ao["lane_prob_min"])
  ao["require_lane_gain"] = _read_num("AutoOvertakeRequireLaneGain", ao["require_lane_gain"], int) != 0

  aa = dict(AA_CFG_DEFAULTS)
  aa["enabled"] = _read_num("AutoAvoidanceEnabled", aa["enabled"], int) != 0

  try:
    raw = _read_raw("AvoidStableMode")
    if raw:
      m = str(raw).strip().lower()
      if m in ("strict", "ratio"):
        aa["stable_mode"] = m
  except Exception:
    pass

  for key, cfg_key, allow_zero in (("AvoidObsHold", "obs_hold_sec", True),
                                   ("AvoidBrakeMax", "brake_max", True),
                                   ("AvoidBrakeGain", "brake_gain", True),
                                   ("AvoidBrakeTtcRef", "brake_ttc_ref", False)):
    try:
      raw = _read_raw(key)
      if raw:
        v = float(raw)
        if v > 0.0 or (allow_zero and v >= 0.0):
          aa[cfg_key] = v
    except Exception:
      pass

  return ao, aa

def _read_mr76_aux():
  """Read mr76d's shared target list for the AO safety gate.

  Conservative rule: any qualifying side/rear target vetoes BOTH lanes.
  MR76 lateral sign is installation-dependent (Mr76LatSign), so refusing
  an overtake is safer than guessing the wrong side.
  """
  try:
    with open(MR76_AUX_PATH) as f:
      data = json.loads(f.read())
    age = max(0.0, time.time() - float(data.get("t", 0.0)))
    if age > MR76_AUX_STALE_SEC or not bool(data.get("fresh", False)):
      return False, 0, None
    blocked = False
    for item in data.get("targets", [])[:32]:
      try:
        d = float(item[0])
        y = float(item[1])
      except (TypeError, ValueError, IndexError):
        continue
      if 0.5 < d < MR76_SIDE_DIST_MAX and abs(y) > MR76_SIDE_LAT_MIN:
        blocked = True
        break
    return True, (11 if blocked else 0), age
  except Exception:
    return False, 0, None


def effective_bsm(sm, cs):
  """Blind-spot detection = OEM BSM (carState) + side radar targets.

  This is the same definition used by the device's UI blind_spot_indicators:
  an OEM blind-spot lamp OR a side radar target in the danger zone means the
  side is occupied. A side target is dangerous when within
  SIDE_LEAD_DANGER_DIST, or within SIDE_LEAD_CLOSING_DIST while closing fast.
  Returns (left, right) occupancy booleans. These are combined with the left/
  right lidar clearance in _update_aa so both sensors are checked together.
  """
  left = bool(cs.leftBlindspot)
  right = bool(cs.rightBlindspot)
  try:
    rs = sm['radarState']
    for present, drel, vrel, side in (
      (rs.leadLeft.present, rs.leadLeft.dRel, rs.leadLeft.vRel, "left"),
      (rs.leadRight.present, rs.leadRight.dRel, rs.leadRight.vRel, "right"),
    ):
      if not bool(present):
        continue
      try:
        d = float(drel)
        vr = float(vrel)
      except (TypeError, ValueError):
        continue
      closing = -vr  # vRel < 0 means the target is closing toward ego
      dangerous = (
        0.0 <= d < SIDE_LEAD_DANGER_DIST
        or (0.0 <= d < SIDE_LEAD_CLOSING_DIST and closing >= SIDE_LEAD_CLOSING_SPEED)
      )
      if dangerous:
        if side == "left":
          left = True
        else:
          right = True
  except Exception:
    pass
  return left, right

class _RadarObstacleEvidence:
  """Short-horizon in-path track confirmation, modeled on carrot-wip.

  Returns only a target whose current range/side/relative speed agree with a
  continuous measured history. No stale extrapolation, no single-point gate.
  """
  def __init__(self):
    self._tracks = {}

  def update(self, now, points):
    tracks = {}
    candidates = []
    for tid, d, y, vr in points:
      if not all(math.isfinite(float(v)) for v in (d, y, vr)):
        continue
      d, y, vr = float(d), float(y), float(vr)
      old = self._tracks.get(tid)
      dt = now - old[0] if old is not None else math.inf
      continuous = bool(
        old is not None and 0.0 < dt <= RADAR_MAX_GAP_S
        and abs(d - old[1] - old[3] * dt) <= RADAR_PRED_D_ERR
        and abs(y - old[2]) <= RADAR_PRED_Y_ERR
        and abs(vr - old[3]) <= RADAR_V_STEP_ERR
      )
      since = old[4] if continuous else now
      samples = old[5] + 1 if continuous else 1
      tracks[tid] = (now, d, y, vr, since, samples)
      if samples >= RADAR_MIN_SAMPLES and (now - since) >= RADAR_MIN_CONFIRM_S:
        candidates.append((d, y, vr, since, samples))
    self._tracks = tracks
    if len(candidates) != 1:
      return None
    return candidates[0]

class _ThrottledLog:
  def __init__(self, path, period=1.0):
    self.path = path
    self.period = period
    self._t = -1e9

  def write(self, text):
    now = time.monotonic()
    if now - self._t < self.period:
      return
    self._t = now
    try:
      if os.path.exists(self.path) and os.path.getsize(self.path) > LOG_MAX_BYTES:
        os.replace(self.path, self.path + ".1")
      with open(self.path, "a") as f:
        f.write(text + "\n")
    except Exception:
      pass

def _fmt(fields):
  return time.strftime("%H:%M:%S") + " " + " ".join(f"{k}={v}" for k, v in fields.items())

class NaviBits(NamedTuple):
  left_lidar_obstacle: bool
  right_lidar_obstacle: bool
  left_nav_blind: bool
  right_nav_blind: bool
  left_lane: bool
  right_lane: bool
  seen: bool

def decode_navi(sm):
  try:
    if not (sm.seen['amapNavi'] and sm.valid['amapNavi']):
      return NaviBits(False, False, False, False, False, False, False)
    n = sm['amapNavi']
    lb = int(n.leftBlind)
    rb = int(n.rightBlind)
    return NaviBits(
      left_lidar_obstacle=bool(lb & 0b0101),
      right_lidar_obstacle=bool(rb & 0b0101),
      left_nav_blind=bool(lb & 0b0010),
      right_nav_blind=bool(rb & 0b0010),
      left_lane=bool(lb & 0b1000),
      right_lane=bool(rb & 0b1000),
      seen=True,
    )
  except Exception:
    return NaviBits(False, False, False, False, False, False, False)

class LidarLatch:

  def __init__(self):
    self.left = False
    self.right = False

  def update(self, bits):
    if bits.left_lidar_obstacle:
      self.left = True
    if bits.right_lidar_obstacle:
      self.right = True

class BridgeOut(NamedTuple):
  direction: int
  brake_req: float
  confirm_delay_sec: float
  ao_state: dict
  aa_state: dict

_EMPTY_AO = {"mode": "-", "last_reason": "-"}
_EMPTY_AA = {"mode": "-", "last_reason": "-", "risk_level": 0}

class AoAaBridge:
  def __init__(self):
    self.ao = AutoOvertakeHelper()
    self.aa = AutoAvoidanceHelper()
    self.aa_radar = _RadarObstacleEvidence()
    self.lidar = LidarLatch()
    self.ao_log = _ThrottledLog(AO_LOG_PATH)
    self.aa_log = _ThrottledLog(AA_LOG_PATH)
    self._ao_cfg, self._aa_cfg = read_cfg()
    self._cfg_n = 0
    self._brake_pub = -1.0
    self._obs_hold = {"d": None, "y": None, "vr": None, "t": 0.0}
    self._aa_candidate = LaneChangeDirection.none
    self._aa_candidate_since = 0.0
    self._n = 0

  def _maybe_reload_cfg(self):
    self._cfg_n += 1
    if (self._cfg_n % 40) == 0:
      try:
        self._ao_cfg, self._aa_cfg = read_cfg()
      except Exception:
        pass

  def update(self, *, sm, dh, v_ego, modelv2, lat_active, left_edge, right_edge, is_rhd=False):
    self._n += 1
    self._maybe_reload_cfg()

    ao_cfg = self._ao_cfg
    aa_cfg = self._aa_cfg

    lp_left, lp_right = 1.0, 1.0
    try:
      probs = list(modelv2.laneLineProbs)
      lp_left = float(probs[0]) if len(probs) > 0 else 1.0
      lp_right = float(probs[3]) if len(probs) > 3 else 1.0
    except Exception:
      pass
    left_ok = (not bool(left_edge)) and lp_left >= ao_cfg["lane_prob_min"]
    right_ok = (not bool(right_edge)) and lp_right >= ao_cfg["lane_prob_min"]

    bits = decode_navi(sm)
    self.lidar.update(bits)
    navi_online = bool(bits.seen)
    lidar_avail_l = True if navi_online else None
    lidar_avail_r = True if navi_online else None
    lidar_valid_l = True if navi_online else None
    lidar_valid_r = True if navi_online else None
    lidar_age_l = 0.0 if navi_online else None
    lidar_age_r = 0.0 if navi_online else None
    lidar_obs_l = bool(bits.left_lidar_obstacle) if navi_online else None
    lidar_obs_r = bool(bits.right_lidar_obstacle) if navi_online else None
    lidar_free_l = (not bits.left_lidar_obstacle) if navi_online else None
    lidar_free_r = (not bits.right_lidar_obstacle) if navi_online else None

    param_fresh = _read_num("AOMr76Fresh", 0.0) != 0.0
    param_lane = int(_read_num("AOMr76Lane", 0.0))
    aux_fresh, aux_lane, aux_age = _read_mr76_aux()
    if param_fresh:
      mr76_fresh, mr76_lane, mr76_age = True, param_lane, 0.0
    elif aux_fresh:
      mr76_fresh, mr76_lane, mr76_age = True, aux_lane, aux_age
    else:
      mr76_fresh, mr76_lane, mr76_age = False, 0, None
    mr76_gain_fresh = _read_num("AOMr76GainFresh", 0.0) != 0.0
    mr76_gain_lane = int(_read_num("AOMr76GainLane", 0.0))

    ao_dir = self._update_ao(sm, dh, v_ego, ao_cfg, left_ok, right_ok, is_rhd,
                             lp_left, lp_right, bits,
                             lidar_avail_l, lidar_avail_r, lidar_valid_l, lidar_valid_r,
                             lidar_age_l, lidar_age_r, lidar_obs_l, lidar_obs_r,
                             lidar_free_l, lidar_free_r,
                             mr76_fresh, mr76_lane, mr76_gain_fresh, mr76_gain_lane, mr76_age)
    aa_dir, brake = self._update_aa(sm, dh, v_ego, aa_cfg, left_ok, right_ok, is_rhd, bits,
                                    lidar_avail_l, lidar_avail_r, lidar_valid_l, lidar_valid_r,
                                    lidar_age_l, lidar_age_r, lidar_free_l, lidar_free_r,
                                    mr76_gain_fresh, mr76_gain_lane)

    direction = aa_dir if aa_dir != LaneChangeDirection.none else ao_dir

    confirm = AA_CONFIRM_DELAY_SEC if aa_dir != LaneChangeDirection.none else float(ao_cfg["confirm_delay_sec"])
    return BridgeOut(
      direction=direction,
      brake_req=brake,
      confirm_delay_sec=float(confirm),
      ao_state=self.ao.get_state(),
      aa_state=self.aa.get_state(),
    )

  def _update_ao(self, sm, dh, v_ego, cfg, left_ok, right_ok, is_rhd,
                 lp_left, lp_right, bits,
                 l_avail, r_avail, l_valid, r_valid, l_age, r_age, l_obs, r_obs,
                 l_free, r_free,
                 mr76_fresh, mr76_lane, mr76_gain_fresh, mr76_gain_lane, mr76_age):
    fields = {"en": int(bool(cfg["enabled"]))}
    if not cfg["enabled"]:
      self.ao.reset()
      fields["why"] = "disabled"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

    if not sm.seen.get("radarState", False):
      self.ao.reset()
      fields["why"] = "no_radarstate"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

    try:
      cs = sm['carState']
      lead = sm['radarState'].leadOne
      v_cruise = float(cs.cruiseState.speed) if cs.cruiseState.enabled else 0.0
      v_lead = max(0.0, float(v_ego) + float(lead.vRel))

      lz, rz, rf = None, None, None
      try:
        packed = _read_raw("AOBsmZone")
        if packed:
          pv = int(float(packed))
          lz, rz = pv // 100, pv % 100
        f = _read_raw("AOBsmFault")
        if f:
          fv = int(float(f))
          rf = bool(fv % 10) or bool(fv // 10)
      except Exception:
        lz, rz, rf = None, None, None

      cs_left_bsm_effective, cs_right_bsm_effective = effective_bsm(sm, cs)

      mr76_l_occ = bool((mr76_lane // 10) % 10) if mr76_fresh else None
      mr76_r_occ = bool(mr76_lane % 10) if mr76_fresh else None
      tgt_l = bool((mr76_gain_lane // 10) % 10) if mr76_gain_fresh else None
      tgt_r = bool(mr76_gain_lane % 10) if mr76_gain_fresh else None

      ao_dir = self.ao.update(
        enabled=True,
        lc_state=dh.lane_change_state,
        v_ego=v_ego,
        v_cruise=v_cruise,
        lead_present=bool(lead.present),
        lead_d=float(lead.dRel),
        v_lead=v_lead,
        left_ok=left_ok,
        right_ok=right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(cs.leftBlinker or cs.rightBlinker),
        bsm_available=bool(sm.seen.get("carState", False)) and not bool(rf),
        left_bsm=cs_left_bsm_effective,
        right_bsm=cs_right_bsm_effective,
        left_rear_zone=lz,
        right_rear_zone=rz,
        rear_sensor_fault=rf,
        lane_preference=int(cfg["lane_preference"]),
        # Auto overtake min cruise speed is user-configurable via the Lincoln
        # settings page (60/70/80/90/100 km/h). The old >= 90 km/h hard floor
        # was removed on 2026-11-18 so every UI choice takes real effect.
        min_cruise_speed=float(cfg["min_cruise_kph"]) * CV.KPH_TO_MS,
        left_lane_prob=lp_left,
        right_lane_prob=lp_right,
        lane_prob_min=float(cfg["lane_prob_min"]),
        junction_dist=None,
        mr76_left_available=(True if mr76_fresh else False),
        mr76_right_available=(True if mr76_fresh else False),
        mr76_left_valid=(True if mr76_fresh else False),
        mr76_right_valid=(True if mr76_fresh else False),
        mr76_left_age=(mr76_age if mr76_fresh else None),
        mr76_right_age=(mr76_age if mr76_fresh else None),
        mr76_left_obstacle=mr76_l_occ,
        mr76_right_obstacle=mr76_r_occ,
        left_target_lead_present=tgt_l,
        right_target_lead_present=tgt_r,
        require_lane_gain=bool(cfg["require_lane_gain"]),
        left_lidar_available=l_avail,
        right_lidar_available=r_avail,
        left_lidar_valid=l_valid,
        right_lidar_valid=r_valid,
        left_lidar_age=l_age,
        right_lidar_age=r_age,
        left_lidar_obstacle=l_obs,
        right_lidar_obstacle=r_obs,
        left_lidar_free=l_free,
        right_lidar_free=r_free,
        # AA-style contract (2026-11-18): side clearance is gated by lidar + BSM.
        # MR76 is excluded from the hard gate (kept only as rear-traffic reference via
        # rear_zone), so an MR76 obstacle/stale reading cannot veto an otherwise
        # safe overtake. Lidar must be fresh+free; BSM must be available+clear.
        require_aux_sensors_mr76=False,
        require_aux_sensors_lidar=True,
      )

      st = self.ao.get_state()
      fields.update({
        "v": round(v_ego * 3.6, 1), "vc": round(v_cruise * 3.6, 1),
        "lp": int(bool(lead.present)), "d": round(float(lead.dRel), 1), "vl": round(v_lead, 1),
        "lok": int(left_ok), "rok": int(right_ok),
        "lbsm": int(bool(cs.leftBlindspot)), "rbsm": int(bool(cs.rightBlindspot)),
        "lp0": round(lp_left, 2), "lp3": round(lp_right, 2),
        "ao": int(ao_dir), "mode": st["mode"], "why": st["last_reason"],
        "dh": int(dh.lane_change_state), "dhd": int(dh.lane_change_direction),
        "cf": int(bool(st.get("lane_conf_L"))) * 2 + int(bool(st.get("lane_conf_R"))),
        "jb": int(bool(st.get("junction_ban"))),
        "llid": int(bool(self.lidar.left)) * 2 + int(bits.left_lidar_obstacle),
        "rlid": int(bool(self.lidar.right)) * 2 + int(bits.right_lidar_obstacle),
        "gf": int(bool(cfg["require_lane_gain"])),
        "gL": int(bool(st.get("gain_ok_L"))), "gR": int(bool(st.get("gain_ok_R"))),
        "gr": str(st.get("last_gain_reason") or "-"),
        "mrf": int(mr76_fresh), "mrl": int(mr76_lane),
        "mra": round(mr76_age, 3) if mr76_age is not None else -1,
        "nvs": int(bool(bits.seen)),
        "mgf": int(mr76_gain_fresh), "mgl": int(mr76_gain_lane),
        "nl": int(bits.left_lane) * 2 + int(bits.right_lane),
      })
      self.ao_log.write(_fmt(fields))
      return ao_dir

    except Exception:
      cloudlog.exception("auto overtake update failed")
      self.ao.reset()
      fields["why"] = "exception"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

  def _update_aa(self, sm, dh, v_ego, cfg, left_ok, right_ok, is_rhd, bits,
                 l_avail, r_avail, l_valid, r_valid, l_age, r_age, l_free, r_free,
                 mr76_gain_fresh, mr76_gain_lane):
    fields = {"en": int(bool(cfg["enabled"]))}
    if not cfg["enabled"]:
      self.aa.reset()
      self._clear_obs()
      fields["why"] = "disabled"
      self.aa_log.write(_fmt(fields))
      return LaneChangeDirection.none, 0.0

    try:
      cs = sm['carState']
      lead = sm['radarState'].leadOne if sm.seen.get("radarState", False) else None
      cs_left_bsm_effective, cs_right_bsm_effective = effective_bsm(sm, cs)

      now = time.monotonic()

      # carrot-wip-style confident ego lane: only a high-confidence, stable
      # current lane may be used for lateral avoidance. No lane-change state,
      # no extrapolation, no adjacent-lane borrowing.
      lane_conf = False
      try:
        md = sm['modelV2']
        probs = list(md.laneLineProbs)
        stds = list(md.laneLineStds)
        if len(probs) >= 3 and len(stds) >= 3:
          lane_conf = all(
            float(probs[i]) >= AA_LANE_PROB_MIN and 0.0 <= float(stds[i]) <= AA_LANE_STD_MAX
            for i in (1, 2)
          )
          if lane_conf and len(md.laneLines) >= 3:
            ly = list(md.laneLines[1].y)
            ry = list(md.laneLines[2].y)
            if ly and ry:
              width = abs(float(ry[0]) - float(ly[0]))
              lane_conf = AA_LANE_WIDTH_MIN <= width <= AA_LANE_WIDTH_MAX
          if lane_conf and int(getattr(md.meta.laneChangeState, "raw", 0)) != 0:
            lane_conf = False
      except Exception:
        lane_conf = False
      aa_left_ok = bool(left_ok and lane_conf)
      aa_right_ok = bool(right_ok and lane_conf)

      radar_pts = []
      if sm.seen.get("radarTracks", False):
        for p in sm['radarTracks'].points:
          d = float(p.dRel)
          y = float(p.yRel)
          vr = float(p.vRel)
          if 1.0 <= d <= 60.0 and abs(y) <= 1.0:
            radar_pts.append((int(p.trackId), d, y, vr))
      radar_evidence = self.aa_radar.update(now, radar_pts)

      obs_d, obs_vr, obs_in_path = None, None, False
      if radar_evidence is not None:
        obs_d, obs_vr, obs_in_path = radar_evidence[0], radar_evidence[2], True
        self._obs_hold["d"] = obs_d
        self._obs_hold["y"] = radar_evidence[1]
        self._obs_hold["vr"] = obs_vr
        self._obs_hold["t"] = now
      else:
        self._clear_obs()

      hold_s = float(cfg.get("obs_hold_sec", 0.0))

      # 2026-11-18 user directive: AA lateral safety is LIDAR-ONLY.
      #   - No MR76 rear-distance is added and MR76 never gates an AA steer.
      #   - Side/rear clearance comes from the left/right lidar channels
      #     decoded from amapNavi.leftBlind/rightBlind bit 0b0101.
      #   - SAFE  -> steer avoidance;  UNSAFE -> brake fallback to reduce impact.
      # The legacy MR76 gain-lane params are therefore NOT consumed by AA.
      # (They remain in the signature only because update() still passes them
      #  for AO; they are intentionally ignored here.)

      aa_dir, brake, _hazard, _off = self.aa.update(
        enabled=True,
        obstacle_in_path=obs_in_path,
        lc_state=dh.lane_change_state,
        v_ego=v_ego,
        left_ok=aa_left_ok,
        right_ok=aa_right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(cs.leftBlinker or cs.rightBlinker),
        bsm_available=True,
        left_bsm_blocked=cs_left_bsm_effective,
        right_bsm_blocked=cs_right_bsm_effective,
        obstacle_dist=obs_d,
        obstacle_rel_speed=obs_vr,
        lead_dist=float(lead.dRel) if lead is not None else None,
        lead_rel_speed=float(lead.vRel) if lead is not None else None,
        left_lidar_free=l_free,
        right_lidar_free=r_free,
        left_lidar_available=l_avail,
        right_lidar_available=r_avail,
        left_lidar_valid=l_valid,
        right_lidar_valid=r_valid,
        left_lidar_age=l_age,
        right_lidar_age=r_age,
        # AA uses lidar as the ONLY side/rear clearance source. Split the aux
        # switches so lidar is mandatory while MR76 is explicitly excluded.
        # With require_aux_sensors_lidar=True, any stale/blocked/missing lidar
        # side makes _escape_lane_safe() return False -> no steer -> brake.
        rear_sensing_present=False,
        mr76_left_available=False,
        mr76_right_available=False,
        mr76_left_valid=None,
        mr76_right_valid=None,
        mr76_left_age=None,
        mr76_right_age=None,
        require_aux_sensors=False,
        require_aux_sensors_mr76=False,
        require_aux_sensors_lidar=True,
        mr76_left_obstacle=None,
        mr76_right_obstacle=None,
        avoid_stable_mode=str(cfg["stable_mode"]),
        avoid_stable_ratio=float(cfg["stable_ratio"]),
        avoid_stable_window_sec=float(cfg["stable_window_sec"]),
        avoid_brake_gain=float(cfg["brake_gain"]),
        avoid_brake_max=float(cfg["brake_max"]),
        avoid_brake_ttc_ref=float(cfg["brake_ttc_ref"]),
      )

      st = self.aa.get_state()

      # Apply the risk gate BEFORE logging so the log never shows a
      # misleading non-emergency "dir=1" intent. An AA direction must be
      # backed by a real CRITICAL risk in THIS frame, even if the helper
      # state is stale.
      if aa_dir != LaneChangeDirection.none and int(st.get("risk_level", 0)) < 3:
        aa_dir = LaneChangeDirection.none

      # 2026-11-18: second, independent bridge gate. Even if the helper
      # emits a direction, require its own emergency flag, a confirmed
      # critical reason, sufficient speed, and a stable candidate for
      # AA_BRIDGE_STABLE_SEC. This prevents short sub-1 s AA bursts from
      # reaching DesireHelper between the 1 Hz log samples.
      bridge_ok = (
        aa_dir != LaneChangeDirection.none
        and bool(st.get("emergency_active", False))
        and str(st.get("mode")) == "avoiding"
        and str(st.get("last_reason")) in ("critical_confirmed", "emergency_escape_request")
        and v_ego * 3.6 >= AA_LANE_CHANGE_MIN_KPH
      )
      if bridge_ok:
        if self._aa_candidate != aa_dir:
          self._aa_candidate = aa_dir
          self._aa_candidate_since = now
        if (now - self._aa_candidate_since) < AA_BRIDGE_STABLE_SEC:
          aa_dir = LaneChangeDirection.none
      else:
        self._aa_candidate = LaneChangeDirection.none
        self._aa_candidate_since = 0.0
        aa_dir = LaneChangeDirection.none

      fields.update({
        "dir": int(aa_dir), "mode": st["mode"], "why": st["last_reason"],
        "risk": int(st["risk_level"]),
        "od": round(obs_d, 1) if obs_d is not None else -1,
        "ovr": round(obs_vr, 1) if obs_vr is not None else 0,
        "v": round(v_ego * 3.6, 1),
        "sm": str(cfg["stable_mode"]),
        "oh": round(float(cfg.get("obs_hold_sec", 0.0)), 2),
        "rev": int(radar_evidence is not None),
        "lc": int(bool(lane_conf)),
        "brk": round(float(brake), 3),
        "bmax": round(float(cfg.get("brake_max", 0.0)), 2),
        "egate": int(bool(st.get("emergency_active", False))),
        "astb": round(now - self._aa_candidate_since, 2) if self._aa_candidate != LaneChangeDirection.none else 0.0,
        "raux": 0,  # retired for AA: MR76 no longer gates steering
        "llid": int(bool(self.lidar.left)) * 2 + int(bits.left_lidar_obstacle),
        "rlid": int(bool(self.lidar.right)) * 2 + int(bits.right_lidar_obstacle),
        "lgate": 1,  # side/rear gate = lidar clearance + blind-spot (BSM) detection
        "lfr": int(bool(l_free)) if l_free is not None else -1,
        "rfr": int(bool(r_free)) if r_free is not None else -1,
        # BSM (OEM + side radar) and lidar are checked TOGETHER per side:
        # a side is clear only if lidar says free AND blind-spot is not occupied.
        "lbsm": int(bool(cs_left_bsm_effective)),
        "rbsm": int(bool(cs_right_bsm_effective)),
        "lclr": int(bool(l_free) and not cs_left_bsm_effective) if l_free is not None else 0,
        "rclr": int(bool(r_free) and not cs_right_bsm_effective) if r_free is not None else 0,
      })
      self.aa_log.write(_fmt(fields))

      brake = 0.0 if brake != brake else max(0.0, float(brake))
      self._publish_brake(brake)
      return aa_dir, brake

    except Exception:
      cloudlog.exception("auto avoidance update failed")
      self.aa.reset()
      self._clear_obs()
      fields["why"] = "exception"
      self.aa_log.write(_fmt(fields))
      self._publish_brake(0.0)
      return LaneChangeDirection.none, 0.0

  def _clear_obs(self):
    self._obs_hold["d"] = None
    self._obs_hold["y"] = None
    self._obs_hold["vr"] = None

  def _publish_brake(self, brake):
    try:
      changed = abs(brake - self._brake_pub) >= 0.01
      if changed or brake > 0.0:
        self._brake_pub = brake
        write_param("AvoidBrakeReq", f"{brake:.3f}:{time.monotonic():.3f}")
    except Exception:
      pass
