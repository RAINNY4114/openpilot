#!/usr/bin/env python3

import os
import json
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

MR76_AUX_PATH = "/dev/shm/mr76_targets.json"
MR76_AUX_STALE_SEC = 0.5
MR76_SIDE_LAT_MIN = 0.8
MR76_SIDE_DIST_MAX = 40.0

AA_CONFIRM_DELAY_SEC = 0.5

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
  left = bool(cs.leftBlindspot)
  right = bool(cs.rightBlindspot)
  try:
    rs = sm['radarState']
    if rs.leadLeft.present and float(rs.leadLeft.dRel) < SIDE_LEAD_DANGER_DIST:
      left = True
    if rs.leadRight.present and float(rs.leadRight.dRel) < SIDE_LEAD_DANGER_DIST:
      right = True
  except Exception:
    pass
  return left, right

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
    self.lidar = LidarLatch()
    self.ao_log = _ThrottledLog(AO_LOG_PATH)
    self.aa_log = _ThrottledLog(AA_LOG_PATH)
    self._ao_cfg, self._aa_cfg = read_cfg()
    self._cfg_n = 0
    self._brake_pub = -1.0
    self._obs_hold = {"d": None, "y": None, "vr": None, "t": 0.0}
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
        require_aux_sensors=True,
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

      obs_d, obs_vr, obs_in_path = None, None, False
      best = None
      if sm.seen.get("radarTracks", False):
        for p in sm['radarTracks'].points:
          d = float(p.dRel)
          y = float(p.yRel)
          vr = float(p.vRel)
          if d < 1.0 or d > 60.0:
            continue
          if abs(y) > 1.0:
            continue
          if best is None or d < best[0]:
            best = (d, y, vr)

      now = time.monotonic()
      hold_s = float(cfg.get("obs_hold_sec", 0.0))
      if best is not None:
        self._obs_hold["d"] = best[0]
        self._obs_hold["y"] = best[1]
        self._obs_hold["vr"] = best[2]
        self._obs_hold["t"] = now
        obs_d, obs_vr, obs_in_path = best[0], best[2], True
      elif hold_s > 0.0 and self._obs_hold["d"] is not None and (now - self._obs_hold["t"]) <= hold_s:
        obs_d = self._obs_hold["d"]
        obs_vr = self._obs_hold["vr"]
        obs_in_path = True
      else:
        self._clear_obs()

      aa_mr76_l = bool((mr76_gain_lane // 10) % 10) if mr76_gain_fresh else None
      aa_mr76_r = bool(mr76_gain_lane % 10) if mr76_gain_fresh else None

      aa_dir, brake, _hazard, _off = self.aa.update(
        enabled=True,
        obstacle_in_path=obs_in_path,
        lc_state=dh.lane_change_state,
        v_ego=v_ego,
        left_ok=left_ok,
        right_ok=right_ok,
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
        require_aux_sensors=False,
        mr76_left_obstacle=aa_mr76_l,
        mr76_right_obstacle=aa_mr76_r,
        avoid_stable_mode=str(cfg["stable_mode"]),
        avoid_stable_ratio=float(cfg["stable_ratio"]),
        avoid_stable_window_sec=float(cfg["stable_window_sec"]),
        avoid_brake_gain=float(cfg["brake_gain"]),
        avoid_brake_max=float(cfg["brake_max"]),
        avoid_brake_ttc_ref=float(cfg["brake_ttc_ref"]),
      )

      st = self.aa.get_state()
      fields.update({
        "dir": int(aa_dir), "mode": st["mode"], "why": st["last_reason"],
        "risk": int(st["risk_level"]),
        "od": round(obs_d, 1) if obs_d is not None else -1,
        "ovr": round(obs_vr, 1) if obs_vr is not None else 0,
        "v": round(v_ego * 3.6, 1),
        "sm": str(cfg["stable_mode"]),
        "oh": round(float(cfg.get("obs_hold_sec", 0.0)), 2),
        "brk": round(float(brake), 3),
        "bmax": round(float(cfg.get("brake_max", 0.0)), 2),
        "llid": int(bool(self.lidar.left)) * 2 + int(bits.left_lidar_obstacle),
        "rlid": int(bool(self.lidar.right)) * 2 + int(bits.right_lidar_obstacle),
        "amf": int(mr76_gain_fresh),
        "aml": int(bool(aa_mr76_l)), "amr": int(bool(aa_mr76_r)),
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
