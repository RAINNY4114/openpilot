#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ford / Lincoln Curve Controller
RAINNY4114 Ford curvature-control adaptation for sunnypilot.

Purpose
-------
Improve Ford / Lincoln non-CAN-FD EPS corner behavior by controlling
curvature request rate, curvature error, entry/hold/exit behavior and
driver-intervention reset.

Important
---------
This module ONLY modifies lateral curvature request.

It does NOT:
- control steering torque directly
- create CAN messages
- control longitudinal acceleration
- control radar
- control MR76
- control LiDAR
- control lane changes
- modify radarState
- modify longitudinalPlan

Design
------
Model curvature
      |
      +--> Human Turn Detection
      |
      +--> Curve Entry / Hold / Exit
      |
      +--> Ford current-curvature error limit
      |
      +--> Ford curvature-rate / unwind limit
      |
      +--> lateral acceleration limit
      |
      +--> hard curvature limit
      |
      +--> final output
"""

import math
import os
import time
from enum import Enum, auto

import numpy as np

from openpilot.common.params import Params


# ============================================================================
# Runtime-tunable parameter access — direct file I/O
#
# The compiled C++ params library on this device (`libparams_c.so`) is STALE:
# it does not contain this fork's custom keys, so `Params.get()/get_bool()`
# raises UnknownKeyName for them. Anything here that needs a *custom* key
# therefore reads the param file directly, exactly like
# `ford_curve_speed.py` and `sunnypilot/mapd/live_map_data/osm_map_data.py`.
#
# Read order:  /dev/shm/params/<key>  ->  /data/params/d/<key>  ->  built-in default
# A missing or unparsable file always falls back to the default, so an
# unconfigured device behaves exactly as before.
#
# Values are re-read at most every PARAM_REFRESH_SEC seconds, so tuning takes
# effect WITHOUT restarting controlsd.
# ============================================================================

# Read order: the fork-owned dir FIRST, then the volatile/persistent params dirs.
#
# `/data/ford_params` is a plain directory, NOT the params dir, so
# Params::clearAll() never touches it.  That is the whole point: params_keys.h
# cannot be recompiled on this device (no SConscript/SConstruct in the tree), so
# every dp_ford_* / dp_htd_* key written into /data/params/d is unlinked on the
# next manager start -- see params.cc:208-224.  Writing them here instead makes
# them survive.  Same approach as curve_bend.py, which keeps its config in
# /data/ford_curve.json for exactly this reason.
PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")


def _read_param_direct(key):
  """Return the raw string value of a param, or None. Never raises."""
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, key), "r") as f:
        val = f.read().strip()
      if val:
        return val
    except Exception:
      continue
  return None


def _read_float(key, default, lo=None, hi=None):
  """Read a float param, clamped to [lo, hi]. Falls back to `default`."""
  try:
    raw = _read_param_direct(key)
    if raw is None:
      return float(default)
    val = float(raw)
    if not math.isfinite(val):
      return float(default)
    if lo is not None:
      val = max(float(lo), val)
    if hi is not None:
      val = min(float(hi), val)
    return float(val)
  except Exception:
    return float(default)


def _read_bool(key, default):
  """Read a boolean param. Accepts '1' / 'true' (case-insensitive)."""
  try:
    raw = _read_param_direct(key)
    if raw is None:
      return bool(default)
    return raw == "1" or raw.lower() == "true"
  except Exception:
    return bool(default)


# ============================================================================
# Ford limits
# ============================================================================

MAX_CURVATURE = 0.02

# Ford EPS equivalent lateral acceleration limit.
#
# RAINNY's Ford controller uses ~2.2 m/s^2 for the CAN-FD path.
# For the Q3/non-CANFD path we keep the existing SP tuning at 2.5 m/s^2,
# while the curvature-rate limiter remains the primary EPS protection.
MAX_LATERAL_ACCEL = 2.5

MIN_SPEED = 8.0

# Current-curvature error allowed by Ford controller.
CURVATURE_ERROR = 0.002

# Above this speed, current-curvature blending is enabled.
CURRENT_CURVATURE_MIN_SPEED = 9.0


# ============================================================================
# Curve shaping
# ============================================================================

ENTRY_GAIN = 1.15
MID_GAIN = 1.10
EXIT_GAIN = 1.05

# Output filter.
#
# Smaller value = stronger filtering of the previous value.
# This is intentionally kept compatible with the user's previous controller.
CURVATURE_FILTER = 0.15


# ============================================================================
# Low-speed lateral stabilizer (dp_ford_ls_*)
#
# Purpose: remove the low-speed "hunting" (画龙) seen in jam / crawl driving.
#
# On this fork BOTH hard protections in this file are disabled by their speed
# gates at 12-26 km/h:
#   _current_curvature_limit : speed <= CURRENT_CURVATURE_MIN_SPEED (9.0 m/s)
#   _speed_limit             : speed <  MIN_SPEED (8.0 m/s)
# leaving only the Ford rate table (lo = 0.00050/frame = 0.01 /s) and a final
# IIR with alpha = 0.85 (tau ~ 0.33 s).  The model's own desired-curvature
# hunting (Std up to 2.8e-3, Range 1.06e-2) therefore reaches the wheel.
#
# The stabilizer below acts ONLY below `ls_max_speed` and is DEFAULT OFF.
# ============================================================================

LS_ENABLE_DEFAULT = False      # dp_ford_ls_enable
LS_MAX_SPEED = 8.0             # dp_ford_ls_max_speed   [m/s] 28.8 km/h
LS_TETHER_ERROR = 0.0012       # dp_ford_ls_tether_err   [1/m] anti-hunt band
LS_TETHER_LEAD = 6.0           # same-side lead multiplier (see _ls_tether)
LS_HUNT_WINDOW = 8.0           # [s]   excursion-history window for the hunt detector
LS_HUNT_REVERSALS = 2          # direction reversals inside the window -> hunting
LS_RATE_SCALE = 0.55           # dp_ford_ls_rate_scale   rate-table multiplier
LS_FILTER_HZ = 0.7             # dp_ford_ls_filter_hz    low-pass cutoff


# Anti-overshoot low-pass.
#
# NOTE: this controller is called at 20 Hz (CarControllerParams.STEER_STEP == 5
# at a 100 Hz CarController), so the real step is 0.05 s. The legacy hardcoded
# dt of 0.01 makes the effective time constant ~25 s instead of AO_TAU (~5 s),
# i.e. the anti-overshoot filter is 5x slower than its docstring claims.
#
# `dp_ford_ao_dt` defaults to the legacy 0.01 so behaviour is unchanged; set it
# to 0.05 on the device to get the intended AO_TAU.
AO_TAU = 5.0
AO_DT = 0.01


# ============================================================================
# S-gear (Sport) profile
#
# When `dp_ford_sport_enable` is set AND the car reports GearShifter.sport, the
# controller overlays these values on top of the normal tuning. This is the
# only place in the fork that consumes the Sport gear.
#
# The defaults are a deliberately modest step up from the base values, and
# every upper bound is clamped to what the panda safety layer will actually
# accept (`opendbc/safety/lateral.h::steer_curvature_cmd_checks` +
# `modes/ford.h`), so the profile can never produce frames the panda drops:
#
#   max_lat_accel   <= 3.6 m/s^2  (ISO_LATERAL_ACCEL + g * AVERAGE_ROAD_ROLL)
#   max_curvature   is NOT raised -- already at the 0.02 rad/m panda ceiling
#   curvature_error is NOT raised -- already at the 0.002 panda ceiling
#
# With the switch off (the default) the effective values are identical to the
# base ones, so behaviour is bit-for-bit unchanged.
# ============================================================================

SPORT_MAX_LATERAL_ACCEL = 3.0
SPORT_ENTRY_GAIN = 1.20
SPORT_MID_GAIN = 1.15
SPORT_EXIT_GAIN = 1.10
SPORT_RATE_SCALE = 1.25


# ============================================================================
# Human Turn Detection
# ============================================================================

PARAM_REFRESH_SEC = 2.0

HTD_MIN_SPEED_MS = 0.1
HTD_MAX_SPEED_MS = 9.72       # ~35 km/h

HTD_DEFAULT_ANGLE_THRESHOLD_DEG = 60.0
HTD_RELEASE_ANGLE_DEG = 20.0

HTD_TORQUE_START_NM = 2.0
HTD_TORQUE_RELEASE_NM = 0.6

HTD_RESUME_ANGLE_LOCK_DEG = 40.0

HTD_TRIGGER_CONFIRM_SEC = 0.10

HTD_MIN_RAMP_SEC = 0.50
HTD_MAX_RAMP_SEC = 1.00

# --- preturned criterion (hwh human_turn.py port) ---------------------------
# If the driver had already pre-rotated the wheel before the takeover read as a
# committed turn, hold automatic lateral control longer on release: the driver
# is mid-manoeuvre and is still unwinding. dp_htd_preturn_factor doubles the
# hold when set to 2.0; the default 1.0 leaves the existing timing untouched.
HTD_PRETURN_ANGLE_DEG = 45.0     # wheel must already be past this before trigger
HTD_PRETURN_DWELL_SEC = 0.40     # ... and held there for at least this long
HTD_PRETURN_FACTOR_DEFAULT = 1.0


# ============================================================================
# Runtime-tunable limits
# ============================================================================

class FordLimits:
  """Runtime-tunable mirror of the constants above.

  Every default equals the module constant, so with no param files present the
  behaviour is identical to the previously hardcoded version.

  The upper bounds are deliberately capped at what the panda safety layer will
  actually accept (`opendbc/safety/lateral.h::steer_curvature_cmd_checks` +
  `modes/ford.h`):

    max_curvature     <= 0.02 rad/m   (FORD_STEERING_LIMITS.max_curvature = 1250)
    max_lateral_accel <= 3.6  m/s^2   (ISO_LATERAL_ACCEL + EARTH_G * AVERAGE_ROAD_ROLL)

  Requesting more than that would only produce blocked CAN frames -- the panda
  would flag a violation and drop the message, so it is not a usable authority
  increase.
  """

  def __init__(self):
    self.last_read = 0.0

    # S-gear (Sport) state. Updated every frame by `set_sport()`.
    self.sport_active = False

    self.refresh(force=True)

  def refresh(self, force=False):
    now = time.monotonic()

    if not force and (now - self.last_read) < PARAM_REFRESH_SEC:
      return

    self.last_read = now

    # --- hard limits ---
    self.max_curvature = _read_float(
      "dp_ford_max_curvature", MAX_CURVATURE, 0.002, 0.02,
    )

    self.max_lateral_accel = _read_float(
      "dp_ford_max_lat_accel", MAX_LATERAL_ACCEL, 0.5, 3.6,
    )

    self.min_speed = _read_float(
      "dp_ford_min_speed", MIN_SPEED, 0.0, 20.0,
    )

    # --- current-curvature tether ---
    self.curvature_error = _read_float(
      "dp_ford_curvature_error", CURVATURE_ERROR, 0.0, 0.01,
    )

    self.cc_min_speed = _read_float(
      "dp_ford_cc_min_speed", CURRENT_CURVATURE_MIN_SPEED, 0.0, 30.0,
    )

    # --- curvature rate table, per 20 Hz frame ---
    self.windup_lo = _read_float("dp_ford_rate_windup_lo", 0.00050, 0.00005, 0.005)
    self.windup_hi = _read_float("dp_ford_rate_windup_hi", 0.00011, 0.00005, 0.005)
    self.unwind_lo = _read_float("dp_ford_rate_unwind_lo", 0.00055, 0.00005, 0.005)
    self.unwind_hi = _read_float("dp_ford_rate_unwind_hi", 0.00020, 0.00005, 0.005)

    # --- curve gains ---
    self.entry_gain = _read_float("dp_ford_entry_gain", ENTRY_GAIN, 0.5, 1.5)
    self.mid_gain = _read_float("dp_ford_mid_gain", MID_GAIN, 0.5, 1.5)
    self.exit_gain = _read_float("dp_ford_exit_gain", EXIT_GAIN, 0.5, 1.5)

    # --- anti-overshoot ---
    self.ao_tau = _read_float("dp_ford_ao_tau", AO_TAU, 0.1, 60.0)
    self.ao_dt = _read_float("dp_ford_ao_dt", AO_DT, 0.001, 0.2)

    # When set, restrict `_anti_overshoot()` to the curve EXIT phase, which is
    # the only phase its docstring claims to protect. Default 0 preserves the
    # legacy behaviour, where it also throttles ENTRY and HOLD and -- because
    # `dp_ford_ao_dt` defaults to 0.01 against a real 0.05 s step -- pins the
    # commanded curvature at roughly its 0.002 rad/m gate.
    self.ao_exit_only = _read_bool("dp_ford_ao_exit_only", False)
    self.ao_enable = _read_bool("dp_ford_ao_enable", True)

    # --- human turn detection ---
    self.htd_enabled = _read_bool("dp_htd_enabled", False)

    self.htd_angle_threshold_deg = _read_float(
      "dp_htd_turn_angle_threshold", HTD_DEFAULT_ANGLE_THRESHOLD_DEG, 20.0, 120.0,
    )

    # --- S-gear (Sport) profile ---
    self.sport_enable = _read_bool("dp_ford_sport_enable", False)

    self.sport_max_lateral_accel = _read_float(
      "dp_ford_sport_max_lat_accel", SPORT_MAX_LATERAL_ACCEL, 0.5, 3.6,
    )

    self.sport_entry_gain = _read_float(
      "dp_ford_sport_entry_gain", SPORT_ENTRY_GAIN, 0.5, 1.5,
    )

    self.sport_mid_gain = _read_float(
      "dp_ford_sport_mid_gain", SPORT_MID_GAIN, 0.5, 1.5,
    )

    self.sport_exit_gain = _read_float(
      "dp_ford_sport_exit_gain", SPORT_EXIT_GAIN, 0.5, 1.5,
    )

    self.sport_rate_scale = _read_float(
      "dp_ford_sport_rate_scale", SPORT_RATE_SCALE, 1.0, 3.0,
    )

    # --- low-speed lateral stabilizer (default OFF) ---
    self.ls_enable = _read_bool("dp_ford_ls_enable", LS_ENABLE_DEFAULT)

    self.ls_max_speed = _read_float(
      "dp_ford_ls_max_speed", LS_MAX_SPEED, 0.0, 15.0,
    )

    self.ls_tether_err = _read_float(
      "dp_ford_ls_tether_err", LS_TETHER_ERROR, 0.0, 0.01,
    )

    self.ls_rate_scale = _read_float(
      "dp_ford_ls_rate_scale", LS_RATE_SCALE, 0.05, 1.0,
    )

    self.ls_filter_hz = _read_float(
      "dp_ford_ls_filter_hz", LS_FILTER_HZ, 0.05, 5.0,
    )

    # Resolve the effective values = base, overlaid by Sport when enabled.
    self._refresh_sport()

  def set_sport(self, active):
    """Tell the limiter whether the car is in Sport.

    Called once per controller frame. Cheap: the overlay is only recomputed
    when the state actually flips.
    """
    active = bool(active)

    if active == self.sport_active:
      return

    self.sport_active = active

    self._refresh_sport()

  def _refresh_sport(self):
    """Effective values = base tuning, overlaid by the Sport profile.

    With `dp_ford_sport_enable` unset (default) these are identical to the base
    values, so the controller behaves exactly as before.
    """
    on = bool(self.sport_enable and self.sport_active)

    self.sport_on = on

    self.lat_accel_limit = (
      self.sport_max_lateral_accel if on else self.max_lateral_accel
    )

    self.gain_entry = self.sport_entry_gain if on else self.entry_gain
    self.gain_mid = self.sport_mid_gain if on else self.mid_gain
    self.gain_exit = self.sport_exit_gain if on else self.exit_gain

    self.rate_scale = self.sport_rate_scale if on else 1.0


def now_mono():
  """Monotonic seconds. Thin wrapper so the HTD additions stay readable."""
  return time.monotonic()


class HTDState(Enum):
  INACTIVE = auto()
  MANUAL_TURN = auto()
  RAMPING = auto()


class HumanTurnDetection:
  """
  Human Turn Detection.

  This is deliberately separated from curvature generation.

  Its only job is to determine whether the driver is actively making
  a large steering correction and whether automatic lateral control
  should remain suppressed temporarily.
  """

  def __init__(self):
    self.params = Params()
    self.last_params_read = 0.0

    self.enabled = False
    self.angle_threshold_deg = HTD_DEFAULT_ANGLE_THRESHOLD_DEG

    self.state = HTDState.INACTIVE

    self.state_change_time = 0.0
    self.trigger_start_time = 0.0

    self.last_angle_raw = 0.0
    self.last_torque_raw = 0.0

    self.last_angle = 0.0
    self.last_torque = 0.0

    self.last_pressed = False

    self.max_turn_angle = 0.0
    self.dynamic_delay = HTD_MIN_RAMP_SEC

    # preturned latch: set when the trigger fires with the wheel already
    # pre-rotated, consumed by the RAMPING branch to extend the hold.
    self.preturned = False
    self.preturn_since = 0.0
    self.preturn_factor = HTD_PRETURN_FACTOR_DEFAULT

  def _read_params(self):
    now = time.monotonic()

    if now - self.last_params_read < PARAM_REFRESH_SEC:
      return

    self.last_params_read = now

    # Direct file I/O: `dp_htd_*` are NOT present in the stale libparams_c.so,
    # so `Params.get_bool("dp_htd_enabled")` raised UnknownKeyName, the bare
    # `except` swallowed it, and HTD was therefore permanently disabled.
    self.enabled = _read_bool("dp_htd_enabled", False)

    self.angle_threshold_deg = _read_float(
      "dp_htd_turn_angle_threshold",
      HTD_DEFAULT_ANGLE_THRESHOLD_DEG,
      20.0,
      120.0,
    )

    self.preturn_factor = _read_float(
      "dp_htd_preturn_factor",
      HTD_PRETURN_FACTOR_DEFAULT,
      1.0,
      3.0,
    )

  def reset(self):
    self.state = HTDState.INACTIVE
    self.state_change_time = 0.0
    self.trigger_start_time = 0.0
    self.max_turn_angle = 0.0
    self.dynamic_delay = HTD_MIN_RAMP_SEC
    self.preturned = False
    self.preturn_since = 0.0

  def _transition(self, state):
    if state == self.state:
      return

    self.state = state
    self.state_change_time = time.monotonic()

  def _should_trigger(self):
    direction_match = (
      self.last_angle_raw * self.last_torque_raw > 0.0
    )

    condition = (
      self.last_pressed
      and direction_match
      and self.last_torque >= HTD_TORQUE_START_NM
      and self.last_angle >= self.angle_threshold_deg
    )

    now = time.monotonic()

    if condition:
      if self.trigger_start_time == 0.0:
        self.trigger_start_time = now
      elif now - self.trigger_start_time >= HTD_TRIGGER_CONFIRM_SEC:
        return True
    else:
      self.trigger_start_time = 0.0

    return False

  def _should_release(self):
    perfect_return = (
      self.last_torque <= HTD_TORQUE_RELEASE_NM
      and self.last_angle <= HTD_RELEASE_ANGLE_DEG
    )

    hands_off = not self.last_pressed

    if perfect_return or hands_off:
      self.trigger_start_time = 0.0
      return True

    return False

  def update(
      self,
      lat_active,
      cruise_enabled,
      steering_angle_deg,
      steering_torque_nm,
      v_ego,
      steering_pressed=False,
  ):
    """
    Returns:
      (allow_curve_control, state)
    """

    self._read_params()

    self.last_angle_raw = float(steering_angle_deg)
    self.last_torque_raw = float(steering_torque_nm)

    self.last_angle = abs(self.last_angle_raw)
    self.last_torque = abs(self.last_torque_raw)

    self.last_pressed = bool(steering_pressed)

    # --- preturn dwell tracking -------------------------------------------
    # Measured BEFORE the invalid gate so the timer runs on the same samples
    # the trigger sees. Cleared whenever the wheel drops back below the
    # preturn angle, so only a *sustained* pre-rotation counts.
    if abs(self.last_angle_raw) >= HTD_PRETURN_ANGLE_DEG:
      if self.preturn_since == 0.0:
        self.preturn_since = now_mono()
    else:
      self.preturn_since = 0.0

    invalid = (
      not self.enabled      
      or not lat_active
      or not (
        HTD_MIN_SPEED_MS
        <= v_ego
        <= HTD_MAX_SPEED_MS
      )
    )

    if invalid:
      self.reset()
      return True, self.state

    if self.state == HTDState.INACTIVE:

      if self._should_trigger():
        self.max_turn_angle = self.last_angle

        # hwh preturned criterion: was the wheel already pre-rotated and held
        # there long enough before this takeover read as a committed turn?
        self.preturned = (
          self.preturn_since > 0.0
          and (now_mono() - self.preturn_since) >= HTD_PRETURN_DWELL_SEC
        )

        self._transition(
          HTDState.MANUAL_TURN
        )

        return False, self.state

    elif self.state == HTDState.MANUAL_TURN:

      self.max_turn_angle = max(
        self.max_turn_angle,
        self.last_angle,
      )

      if self._should_release():

        delay = self.max_turn_angle / 270.0

        self.dynamic_delay = max(
          HTD_MIN_RAMP_SEC,
          min(delay, HTD_MAX_RAMP_SEC),
        )

        # preturned hold extension: no-op at the default factor of 1.0.
        if self.preturned and self.preturn_factor > 1.0:
          self.dynamic_delay = min(
            self.dynamic_delay * self.preturn_factor,
            HTD_MAX_RAMP_SEC * self.preturn_factor,
          )

        self._transition(
          HTDState.RAMPING
        )

      return False, self.state

    elif self.state == HTDState.RAMPING:

      # Driver immediately takes the wheel again.
      if self._should_trigger():

        self.trigger_start_time = 0.0

        self._transition(
          HTDState.MANUAL_TURN
        )

        return False, self.state

      elapsed = (
        time.monotonic()
        - self.state_change_time
      )

      if elapsed < self.dynamic_delay:
        return False, self.state

      # Do not resume automatic steering while the steering wheel
      # is still significantly displaced.
      if self.last_angle > HTD_RESUME_ANGLE_LOCK_DEG:
        return False, self.state

      self.max_turn_angle = 0.0

      self._transition(
        HTDState.INACTIVE
      )

      return True, self.state

    return True, self.state


# ============================================================================
# Ford Curve Controller
# ============================================================================

class FordCurveController:

  def __init__(self, CP=None):

    self.CP = CP

    self.params = Params()

    # Runtime-tunable limits (direct file I/O, hot-reloaded every 2 s).
    self.limits = FordLimits()

    self.last_curvature = 0.0

    self.last_requested_curvature = 0.0

    self.current_curvature = 0.0

    self.active = False

    # ------------------------------------------------------------------------
    # Curve phase
    # ------------------------------------------------------------------------

    self.curve_phase = "STRAIGHT"

    self.curve_entry_active = False
    self.curve_exit_active = False

    # ------------------------------------------------------------------------
    # Human turn
    # ------------------------------------------------------------------------

    self.htd = HumanTurnDetection()

    self.human_turn_active = False
    self.htd_ramping = False

    self.post_reset_ramp_active = False

    # ------------------------------------------------------------------------
    # Ford curvature state
    # ------------------------------------------------------------------------

    self.apply_curvature_last = 0.0

    # ------------------------------------------------------------------------
    # RAINNY style anti-overshoot state
    # ------------------------------------------------------------------------

    self.anti_overshoot_curvature_last = 0.0

    # ------------------------------------------------------------------------
    # Low-speed lateral stabilizer (dp_ford_ls_*)
    # ------------------------------------------------------------------------

    self.ls_filtered = 0.0
    self.ls_hunt_t = 0.0
    self.ls_hunt_hist = []
    self.ls_hunt_reversals = 0

  # ==========================================================================
  # Reset
  # ==========================================================================

  def reset(self):

    self.last_curvature = 0.0

    self.last_requested_curvature = 0.0

    self.ls_filtered = 0.0
    self.ls_hunt_t = 0.0
    self.ls_hunt_hist = []
    self.ls_hunt_reversals = 0

    self.current_curvature = 0.0

    self.active = False

    self.curve_phase = "STRAIGHT"

    self.curve_entry_active = False
    self.curve_exit_active = False

    self.human_turn_active = False
    self.htd_ramping = False

    self.post_reset_ramp_active = False

    self.apply_curvature_last = 0.0
    self.anti_overshoot_curvature_last = 0.0

    self.htd.reset()

  # ==========================================================================
  # Lateral acceleration limit
  # ==========================================================================

  # ==========================================================================
  # Low-speed lateral stabilizer (dp_ford_ls_*)
  # ==========================================================================

  def _ls_active(self, speed):
    """True when the stabilizer should act at this speed."""
    lim = self.limits
    return (
      bool(lim.ls_enable)
      and float(speed) < float(lim.ls_max_speed)
    )

  def _ls_hunting(self, curvature):
    """True when the incoming command is HUNTING rather than turning.

    A genuine low-speed turn holds ONE sign for many seconds (a whole
    intersection).  Hunting on this car is a SLOW saw at 6-8 s period, i.e.
    one direction reversal every 3-4 s.  Over an 8 s window that is >= 2
    reversals; a genuine turn has 0.

    NOTE: an earlier version counted sign flips inside a 2 s window with a
    threshold of 3 -- the period (6-8 s) is far longer than that, so the
    detector almost never fired and the stabilizer was inert.
    """
    s = 1 if curvature > 0.0008 else (-1 if curvature < -0.0008 else 0)

    now = getattr(self, 'ls_hunt_t', 0.0) + 0.05   # 20 Hz call cadence
    self.ls_hunt_t = now

    hist = getattr(self, 'ls_hunt_hist', None)
    if hist is None:
      hist = []
      self.ls_hunt_hist = hist
    hist.append((now, s))
    while hist and (now - hist[0][0]) > LS_HUNT_WINDOW:
      hist.pop(0)

    # collapse consecutive identical non-zero signs -> excursion sequence
    seq = []
    for (_, ss) in hist:
      if ss != 0 and (not seq or seq[-1] != ss):
        seq.append(ss)
    reversals = max(0, len(seq) - 1)

    self.ls_hunt_reversals = reversals
    return (reversals >= LS_HUNT_REVERSALS) and (len(seq) >= LS_HUNT_REVERSALS + 1)

  def _ls_tether(self, curvature, current_curvature, speed):
    """L1: suppress the low-speed command that OPPOSES the measured curvature.

    Reuses the RAINNY idea of _current_curvature_limit, but with its own
    (much lower) speed gate so it is active in the jam band where the stock
    gate (CURRENT_CURVATURE_MIN_SPEED = 9.0 m/s) has already switched it off.

    IMPORTANT -- why this is NOT a plain |cmd - measured| clip:
    a naive symmetric clip at 0.0012 broke REAL low-speed turns.  Offline
    replay of genuine turns (ls_replay.py --turn-only) showed the tracking
    gain (peak command / peak request) collapsing from ~0.99 to 0.33, i.e.
    2/3 of the steering needed for a crawl-speed turn was clipped away ->
    understeer / lane runout.

    The asymmetry that makes it safe:
      * command KEEPS the measured sign  -> it is a real turn (the command
        legitimately LEADS the measured value while the car is still
        turning in).  Allow a generous lead (tether_err * LS_TETHER_LEAD).
      * command FLIPS against the measured sign -> the model is hunting past
        centre; that half is what makes the wheel saw.  Clamp it tightly to
        the measured value +/- tether_err.
    """
    if not self._ls_active(speed):
      return curvature

    err = float(self.limits.ls_tether_err)
    meas = float(current_curvature)
    cmd = float(curvature)

    if cmd * meas >= 0.0:
      # SAME side as measured -> the car really is turning this way.
      # Do NOT cap the lead at all: capping it is exactly what understeers
      # (offline replay showed a 6x band still cost ~35% on mixed segments,
      # and a symmetric clip cost 66%).  Passing it through untouched keeps
      # the tracking gain at 1.000.
      return cmd

    # opposite side: hunting across centre -- clamp tightly
    return float(np.clip(cmd, meas - err, meas + err))

  def _ls_rate_scale_factor(self, speed):
    """L2: multiplier applied to the Ford rate table while hunting."""
    if not self._ls_active(speed):
      return 1.0
    if not bool(getattr(self, 'ls_hunt_reversals', 0) >= LS_HUNT_REVERSALS):
      return 1.0
    return float(self.limits.ls_rate_scale)

  def _ls_filter(self, curvature, speed):
    """L3: first-order low-pass, active only at low speed WHILE HUNTING.

    The wobble period is 6-8 s (~0.15 Hz); the default cutoff is 0.7 Hz,
    so a real lane change / turn at crawl speed is not slowed at all
    (the gate also excludes it outright), while the hunting is strongly
    attenuated.
    """
    lim = self.limits

    if not self._ls_active(speed) or not bool(getattr(self, 'ls_hunt_reversals', 0) >= LS_HUNT_REVERSALS):
      # keep the filter state tracking so re-entry does not kick
      self.ls_filtered = float(curvature)
      return curvature

    fc = max(float(lim.ls_filter_hz), 0.01)
    dt = 0.05                     # FordCurveController runs at 20 Hz
    alpha = 1.0 - math.exp(-2.0 * math.pi * fc * dt)

    self.ls_filtered = (
      alpha * float(curvature)
      + (1.0 - alpha) * self.ls_filtered
    )

    return float(self.ls_filtered)

  def _speed_limit(self, curvature, speed):

    lim = self.limits

    if speed < lim.min_speed:
      return curvature

    max_curvature = (
      lim.lat_accel_limit
      / max(speed * speed, 1.0)
    )

    return max(
      -max_curvature,
      min(
        curvature,
        max_curvature,
      ),
    )

  # ==========================================================================
  # Ford current curvature error limit
  # ==========================================================================

  def _current_curvature_limit(
      self,
      curvature,
      current_curvature,
      speed,
  ):
    """
    RAINNY Ford behavior.

    Above ~9 m/s, don't allow requested curvature to jump
    more than CURVATURE_ERROR away from the actual vehicle curvature.

    This prevents abrupt EPS changes and reduces high-speed ping-pong.
    """

    if speed <= self.limits.cc_min_speed:
      return curvature

    err = self.limits.curvature_error

    return float(
      np.clip(
        curvature,
        current_curvature - err,
        current_curvature + err,
      )
    )

  # ==========================================================================
  # Ford curvature rate limits
  # ==========================================================================

  def _ford_curvature_rate_limit(
      self,
      target,
      previous,
      speed,
      ramp_release=False,
  ):
    """
    Ford Q3/non-CANFD curvature rate limiter.

    These values follow the RAINNY Ford branch tuning:

      wind-up:
        5 m/s  -> 0.00050
        25 m/s -> 0.00011

      unwind:
        5 m/s  -> 0.00055
        25 m/s -> 0.00020

    Unwind is intentionally faster than wind-up.

    That is important for turn exit:
      entering a curve = controlled
      holding a curve  = stable
      exiting a curve  = release steering authority progressively
    """

    v = max(float(speed), 0.1)

    lim = self.limits

    # S-gear (Sport) scales the whole rate table; 1.0 when not in Sport.
    rate_scale = lim.rate_scale

    # L2: tighten the whole table below the low-speed gate.
    rate_scale = rate_scale * self._ls_rate_scale_factor(v)

    windup_rate = float(
      np.interp(
        v,
        [5.0, 25.0],
        [lim.windup_lo, lim.windup_hi],
      )
    ) * rate_scale

    unwind_rate = float(
      np.interp(
        v,
        [5.0, 25.0],
        [lim.unwind_lo, lim.unwind_hi],
      )
    ) * rate_scale

    delta = target - previous

    if delta > 0.0:

      max_delta = windup_rate

    else:

      max_delta = unwind_rate

      # During post-driver reset / curve exit, explicitly use the
      # faster Ford unwind path.
      if ramp_release:
        max_delta *= 1.15

    delta = float(
      np.clip(
        delta,
        -max_delta,
        max_delta,
      )
    )

    return previous + delta

  # ==========================================================================
  # Curve phase detection
  # ==========================================================================

  def _update_curve_phase(
      self,
      curvature,
      previous_curvature,
  ):

    abs_curvature = abs(curvature)
    abs_previous = abs(previous_curvature)

    # Straight
    if abs_curvature < 0.00035:

      self.curve_phase = "STRAIGHT"

      self.curve_entry_active = False
      self.curve_exit_active = False

      return

    # Entry
    if abs_curvature > abs_previous + 0.00008:

      self.curve_phase = "ENTRY"

      self.curve_entry_active = True
      self.curve_exit_active = False

      return

    # Exit
    if abs_curvature < abs_previous - 0.00008:

      self.curve_phase = "EXIT"

      self.curve_entry_active = False
      self.curve_exit_active = True

      return

    # Hold
    self.curve_phase = "HOLD"

    self.curve_entry_active = False
    self.curve_exit_active = False

  # ==========================================================================
  # Curve gain
  # ==========================================================================

  def _curve_gain(
      self,
      curvature,
      speed,
  ):

    if abs(curvature) <= 0.002:
      return curvature

    lim = self.limits

    if self.curve_phase == "ENTRY":

      gain = lim.gain_entry

    elif self.curve_phase == "HOLD":

      gain = lim.gain_mid

    elif self.curve_phase == "EXIT":

      # IMPORTANT:
      #
      # Do not boost curvature during exit.
      # Let Ford's unwind limiter release the wheel.
      gain = 1.0

    else:

      if speed < 15.0:
        gain = lim.gain_entry
      elif speed < 28.0:
        gain = lim.gain_mid
      else:
        gain = lim.gain_exit

    return curvature * gain

  # ==========================================================================
  # RAINNY anti overshoot
  # ==========================================================================

  def _anti_overshoot(
      self,
      curvature,
      speed,
  ):
    """
    Soft low-pass in lateral-acceleration domain.

    This is intentionally conservative for the MKX.

    It prevents the controller from carrying excessive curvature
    into the beginning of the exit phase.
    """

    speed = max(speed, 1.0)

    lat_accel = curvature * speed * speed

    last_lat_accel = (
      self.anti_overshoot_curvature_last
      * speed
      * speed
    )

    diff = 0.1

    if abs(lat_accel - last_lat_accel) < diff:
      lat_accel = last_lat_accel

    tau = self.limits.ao_tau

    dt = self.limits.ao_dt

    alpha = 1.0 - math.exp(
      -dt / tau
    )

    filtered_lat_accel = (
      alpha * lat_accel
      + (1.0 - alpha) * last_lat_accel
    )

    output = (
      filtered_lat_accel
      / max(speed * speed, 1.0)
    )

    self.anti_overshoot_curvature_last = output

    return float(output)

  # ==========================================================================
  # HTD ramp
  # ==========================================================================

  def _apply_htd_ramp(
      self,
      target,
      speed,
  ):
    """
    Slowly restore curvature after human steering.

    This is deliberately based on Ford unwind/rate limits instead
    of an arbitrary percentage multiplier.
    """

    self.apply_curvature_last = (
      self._ford_curvature_rate_limit(
        target,
        self.apply_curvature_last,
        speed,
        ramp_release=True,
      )
    )

    return self.apply_curvature_last

  # ==========================================================================
  # Main update
  # ==========================================================================

  def update(
      self,
      desired_curvature,
      v_ego,
      active=True,
      steering_angle_deg=0.0,
      steering_torque_nm=0.0,
      steering_pressed=False,
      lat_active=None,
      cruise_enabled=False,
      current_curvature=None,
      sport_gear=False,
  ):
    """
    Main Ford curvature controller.

    Required legacy arguments:
      desired_curvature
      v_ego
      active

    Optional Ford/HTD arguments:
      steering_angle_deg
      steering_torque_nm
      steering_pressed
      lat_active
      cruise_enabled
      current_curvature
      sport_gear        -- True when the car reports GearShifter.sport.
                           Only has an effect when `dp_ford_sport_enable` is set.

    Returns:
      final curvature
    """

    # S-gear (Sport) profile state. Kept current even while inactive, because
    # the gear can change at any time and the limiter is shared.
    self.limits.set_sport(sport_gear)

    if not active:

      self.reset()

      return 0.0

    self.active = True

    # Hot-reload the tunables (throttled to PARAM_REFRESH_SEC).
    self.limits.refresh()

    speed = max(
      float(v_ego),
      0.1,
    )

    if lat_active is None:
      lat_active = active

    # ------------------------------------------------------------------------
    # Current vehicle curvature
    # ------------------------------------------------------------------------

    if current_curvature is None:

      current_curvature = self.current_curvature

    else:

      current_curvature = float(
        current_curvature
      )

    self.current_curvature = current_curvature

    # ------------------------------------------------------------------------
    # Human Turn Detection
    # ------------------------------------------------------------------------

    curve_allowed, htd_state = self.htd.update(
      lat_active=bool(lat_active),
      cruise_enabled=bool(cruise_enabled),
      steering_angle_deg=float(
        steering_angle_deg
      ),
      steering_torque_nm=float(
        steering_torque_nm
      ),
      v_ego=speed,
      steering_pressed=bool(
        steering_pressed
      ),
    )

    previous_human_turn = self.human_turn_active

    self.human_turn_active = (
      htd_state == HTDState.MANUAL_TURN
    )

    self.htd_ramping = (
      htd_state == HTDState.RAMPING
    )

    # ------------------------------------------------------------------------
    # Driver actively steering
    # ------------------------------------------------------------------------

    if self.human_turn_active:

      self.post_reset_ramp_active = True

      self.anti_overshoot_curvature_last = 0.0

      self.apply_curvature_last = 0.0

      self.last_curvature = 0.0

      return 0.0

    # ------------------------------------------------------------------------
    # Driver has released steering wheel
    # ------------------------------------------------------------------------

    if previous_human_turn and self.htd_ramping:

      self.post_reset_ramp_active = True

    # ------------------------------------------------------------------------
    # Ramping after human turn
    # ------------------------------------------------------------------------

    if self.htd_ramping:

      # Do NOT instantly restore the previous curvature.
      #
      # The target starts from the current model curvature and is then
      # released through Ford's curvature-rate limiter.

      target = float(
        desired_curvature
      )

      target = max(
        -self.limits.max_curvature,
        min(
          self.limits.max_curvature,
          target,
        ),
      )

      output = self._apply_htd_ramp(
        target,
        speed,
      )

      self.last_curvature = output

      return float(output)

    # ------------------------------------------------------------------------
    # HTD fully released
    # ------------------------------------------------------------------------

    if htd_state == HTDState.INACTIVE:

      if self.post_reset_ramp_active:

        # The Ford rate limiter itself finishes the unwind.
        target = float(
          desired_curvature
        )

        output = self._ford_curvature_rate_limit(
          target,
          self.apply_curvature_last,
          speed,
          ramp_release=True,
        )

        self.apply_curvature_last = output

        # Once the target and actual command are sufficiently close,
        # leave post-reset mode.
        if abs(target - output) < 0.00015:

          self.post_reset_ramp_active = False

        self.last_curvature = output

        return float(output)

    # ------------------------------------------------------------------------
    # Requested curvature
    # ------------------------------------------------------------------------

    curvature = float(
      desired_curvature
    )

    self.last_requested_curvature = curvature

    # ------------------------------------------------------------------------
    # Detect curve phase
    # ------------------------------------------------------------------------

    self._update_curve_phase(
      curvature,
      self.last_curvature,
    )

    # ------------------------------------------------------------------------
    # Curve gain
    # ------------------------------------------------------------------------

    curvature = self._curve_gain(
      curvature,
      speed,
    )

    # ------------------------------------------------------------------------
    # Low-speed hunting stabilizer (L1 + hunt gate)
    #
    # Active only below dp_ford_ls_max_speed AND only while the incoming
    # command is actually hunting (sign flips inside the window).  A real
    # low-speed turn is left completely untouched, so the tracking gain
    # stays ~1.0 and there is no understeer.  Where the stock RAINNY gate
    # (CURRENT_CURVATURE_MIN_SPEED = 9.0 m/s) has already disabled its own
    # protection, this keeps the command near the MEASURED curvature.
    # ------------------------------------------------------------------------

    if self._ls_active(speed) and self._ls_hunting(curvature):
      curvature = self._ls_tether(
        curvature,
        current_curvature,
        speed,
      )

    # ------------------------------------------------------------------------
    # RAINNY current-curvature error protection
    # ------------------------------------------------------------------------

    curvature = self._current_curvature_limit(
      curvature,
      current_curvature,
      speed,
    )

    # ------------------------------------------------------------------------
    # Anti overshoot
    # ------------------------------------------------------------------------

    ao_phase_ok = (
      self.limits.ao_enable
      and (
        (not self.limits.ao_exit_only)
        or (self.curve_phase == "EXIT")
      )
    )

    if abs(curvature) > 0.002 and ao_phase_ok:

      curvature = self._anti_overshoot(
        curvature,
        speed,
      )

    else:

      self.anti_overshoot_curvature_last = (
        curvature
      )

    # ------------------------------------------------------------------------
    # Lateral acceleration protection
    # ------------------------------------------------------------------------

    curvature = self._speed_limit(
      curvature,
      speed,
    )

    # ------------------------------------------------------------------------
    # Hard curvature safety clamp
    # ------------------------------------------------------------------------

    curvature = max(
      -self.limits.max_curvature,
      min(
        self.limits.max_curvature,
        curvature,
      ),
    )

    # ------------------------------------------------------------------------
    # Ford curvature rate limit
    #
    # This is the most important difference from the original
    # simple SP controller.
    # ------------------------------------------------------------------------

    curvature = self._ford_curvature_rate_limit(
      curvature,
      self.apply_curvature_last,
      speed,
      ramp_release=(
        self.curve_phase == "EXIT"
      ),
    )

    self.apply_curvature_last = curvature

    # ------------------------------------------------------------------------
    # Final output filter
    #
    # Keep this mild because Ford rate limiting is already active.
    # ------------------------------------------------------------------------

    filtered = (
      self.last_curvature * CURVATURE_FILTER
      + curvature * (1.0 - CURVATURE_FILTER)
    )

    # Never exceed the Ford rate-limited value by filtering.
    filtered = self._ford_curvature_rate_limit(
      filtered,
      self.last_curvature,
      speed,
      ramp_release=(
        self.curve_phase == "EXIT"
      ),
    )

    # ------------------------------------------------------------------------
    # L3: low-speed hunting low-pass (gated on the hunt detector)
    # ------------------------------------------------------------------------

    filtered = self._ls_filter(filtered, speed)

    # Final clamp.
    filtered = max(
      -self.limits.max_curvature,
      min(
        self.limits.max_curvature,
        filtered,
      ),
    )

    self.last_curvature = float(
      filtered
    )

    return float(
      filtered
    )


# ============================================================================
# dp_ford Tier 1 -- adaptive model-plan curvature lead (VLT + entry/exit lead)
#
# Ported from hwh-kavin/openpilot sp-master-bp-260825 (curvature_lead.py and the
# VLT block of lateral_angle_ext.py). Everything below is inert unless
# dp_ford_lead_enable is set, and apply() can never raise into the caller.
# ============================================================================

_LEAD_DT_MDL = 0.05
_LEAD_MODEL_ACTION_EXTRA = _LEAD_DT_MDL + _LEAD_DT_MDL / 2.0   # 0.075 s

_LEAD_LOOKAHEAD_T_MIN = 0.25
_LEAD_LOOKAHEAD_T_MAX = 2.0
_LEAD_MIN_SPEED = 1.0

# Variable lookup time (VLT)
_VLT_T_EXTRA_MAX = 0.07
_VLT_V_LOW_MS = 25.0 * 0.44704
_VLT_V_HIGH_MS = 55.0 * 0.44704
_VLT_KAPPA_FULL = 0.005
_VLT_KAPPA_TAPER = 0.020
_VLT_ENTERING_FACTOR = 0.8

# liveDelay clamp. Lower bound is hwh's; upper bound is slightly above this car's
# measured 0.2822 s instead of hwh's 0.15 (see the patch docstring).
_LEAD_DELAY_LO = 0.10
_LEAD_DELAY_HI = 0.32

# Blend of the VLT-sampled model prediction with the planner value
_LEAD_BLEND_BASE = 0.40
_LEAD_BLEND_MIN = 0.10
_LEAD_BLEND_MAX = 0.60

# Entry lead: predicted peak |kappa| -> extra sample lead
_LEAD_KAPPA_BP = (0.0010, 0.0025, 0.0050)
_LEAD_KAPPA_T = (0.0, 0.18, 0.35)
_LEAD_RATIO_BP = (0.70, 1.00, 1.40)
_LEAD_RATIO_T = (0.0, 0.15, 0.35)
_LEAD_MAX_T = 0.40

# Exit lead: peak arrived early and the plan is clearly unwinding ahead
_LEAD_EXIT_T_PEAK_MAX = 0.55
_LEAD_EXIT_KAPPA_MIN = 0.0015
_LEAD_EXIT_KAPPA_DROP_MIN = 0.0008
_LEAD_EXIT_KAPPA_BP = (0.0025, 0.0050, 0.0090)
_LEAD_EXIT_KAPPA_T = (0.08, 0.15, 0.28)
_LEAD_MAX_EXIT_T = 0.32

_LEAD_DEPS = None


def _lead_deps():
  """Lazy import so a broken dependency cannot break module import."""
  global _LEAD_DEPS
  if _LEAD_DEPS is None:
    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
    _LEAD_DEPS = (ModelConstants, get_curvature_from_plan)
  return _LEAD_DEPS


class FordPlanLead:
  """Resample the model plan with an adaptive lead before it reaches the controller.

  Two complementary mechanisms, both from hwh's branch:
    * entry lead -- sample further ahead the harder the upcoming curve is, so the
      curvature command starts climbing earlier;
    * VLT blend  -- sample the plan at an adaptive lookup time and blend it with
      the planner value, with the weight cut back on exit so the model cannot
      drag out the unwind.

  `base_curvature` is NEVER replaced. It carries the lane-centering trim
  (curve_lane_bias / lateral_clearance) which does not exist in the model plan,
  so the model term only biases it, and b is capped at 0.60.

  Params (all read via direct file I/O, hot-reloaded every 2 s):
    dp_ford_lead_enable     bool   default False  -- master switch
    dp_ford_lead_blend      float  default 0.40   -- 0 disables the whole stage
    dp_ford_lead_delay_lo   float  default 0.10   -- liveDelay clamp lower bound
    dp_ford_lead_delay_hi   float  default 0.32   -- liveDelay clamp upper bound
    dp_ford_lead_extra_max  float  default 0.07   -- VLT extra lookahead ceiling
  """

  def __init__(self):
    self.enabled = False
    self.blend = _LEAD_BLEND_BASE
    self.delay_lo = _LEAD_DELAY_LO
    self.delay_hi = _LEAD_DELAY_HI
    self.extra_max = _VLT_T_EXTRA_MAX
    self._ticks = 0
    self.refresh()

  def refresh(self):
    self.enabled = _read_bool("dp_ford_lead_enable", False)
    self.blend = _read_float("dp_ford_lead_blend", _LEAD_BLEND_BASE, 0.0, 1.0)
    self.delay_lo = _read_float("dp_ford_lead_delay_lo", _LEAD_DELAY_LO, 0.0, 1.0)
    self.delay_hi = _read_float("dp_ford_lead_delay_hi", _LEAD_DELAY_HI, 0.0, 1.0)
    self.extra_max = _read_float("dp_ford_lead_extra_max", _VLT_T_EXTRA_MAX, 0.0, 0.30)
    if self.delay_hi < self.delay_lo:
      self.delay_hi = self.delay_lo

  def apply(self, model_v2, v_ego, base_curvature, lat_delay, active=True):
    """Return the lead-resampled curvature, or base_curvature untouched."""
    base = float(base_curvature)

    # Refresh BEFORE the enabled check. If the early return came first, a disabled
    # stage would skip the tick counter entirely, refresh() would never run, and
    # dp_ford_lead_enable could never turn the stage on without a full restart.
    self._ticks += 1
    if self._ticks >= 200:            # 2 s at the 100 Hz control loop
      self._ticks = 0
      self.refresh()

    if not self.enabled or not active:
      return base

    try:
      ModelConstants, get_curvature_from_plan = _lead_deps()
      v = max(float(v_ego), _LEAD_MIN_SPEED)

      t_all = ModelConstants.T_IDXS
      n = min(len(model_v2.orientation.z), len(model_v2.orientationRate.z), len(t_all))
      if n < 2:
        return base

      # capnp _DynamicListReader does NOT support slicing -- `z[:n]` raises
      # TypeError: an integer is required. Materialise with list() first, the
      # same way the lane-line readers are handled elsewhere.
      yaws = np.asarray(list(model_v2.orientation.z)[:n], dtype=float)
      yaw_rates = np.asarray(list(model_v2.orientationRate.z)[:n], dtype=float)
      t = np.asarray(t_all[:n], dtype=float)
      if len(model_v2.velocity.x) >= n:
        speed = np.maximum(np.asarray(list(model_v2.velocity.x)[:n], dtype=float), _LEAD_MIN_SPEED)
      else:
        speed = np.full(n, v, dtype=float)

      kappa = np.abs(yaw_rates) / speed
      mask = (t >= _LEAD_LOOKAHEAD_T_MIN) & (t <= _LEAD_LOOKAHEAD_T_MAX)
      kappa_peak = float(np.max(kappa[mask])) if bool(np.any(mask)) else float(np.max(kappa))
      if not math.isfinite(kappa_peak):
        return base

      delay = float(np.clip(float(lat_delay), self.delay_lo, self.delay_hi))
      t_base = delay + _LEAD_DT_MDL
      kappa_at_base = float(np.interp(t_base, t, kappa))
      entering = kappa_at_base > abs(base)

      # ---- extra lookahead above t_base -----------------------------------
      # hwh splits this into two schedulers: VLT (speed x kappa) for the blend,
      # and the entry-lead schedule (peak kappa / kappa limit) for the resample.
      # Both are the same physical quantity, so take the more aggressive one.
      speed_factor = float(np.interp(v, (_VLT_V_LOW_MS, _VLT_V_HIGH_MS), (1.0, 0.0)))
      if entering:
        vlt_extra = self.extra_max * _VLT_ENTERING_FACTOR * speed_factor
      else:
        kappa_factor = float(np.interp(abs(base), (_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER), (1.0, 0.0)))
        vlt_extra = self.extra_max * speed_factor * kappa_factor

      kappa_lim = min(0.02, 2.5 / (v * v))
      lead_t = float(np.interp(kappa_peak, _LEAD_KAPPA_BP, _LEAD_KAPPA_T))
      lead_t = max(lead_t, float(np.interp(kappa_peak / max(kappa_lim, 1e-6),
                                           _LEAD_RATIO_BP, _LEAD_RATIO_T)))
      lead_t = min(lead_t, _LEAD_MAX_T)
      extra = max(vlt_extra, lead_t)

      # ---- exit: peak arrived early and the plan is unwinding ahead --------
      exit_extra = 0.0
      if kappa_peak >= _LEAD_EXIT_KAPPA_MIN:
        t_w = t[mask]
        k_w = kappa[mask]
        if len(k_w):
          t_peak = float(t_w[int(np.argmax(k_w))])
          i_now = int(np.argmin(np.abs(t - 0.12)))
          i_fut = int(np.argmin(np.abs(t - min(t_peak + 0.35, float(t[n - 1])))))
          unwinding = (t_peak <= _LEAD_EXIT_T_PEAK_MAX and not entering
                       and (float(kappa[i_now]) - float(kappa[i_fut])) >= _LEAD_EXIT_KAPPA_DROP_MIN)
          if unwinding:
            exit_extra = min(float(np.interp(kappa_peak, _LEAD_EXIT_KAPPA_BP, _LEAD_EXIT_KAPPA_T)),
                             _LEAD_MAX_EXIT_T)

      model_term = float(get_curvature_from_plan(
        yaws, yaw_rates, t, v, max(delay + _LEAD_MODEL_ACTION_EXTRA + extra, 1e-3)))

      if exit_extra > 1e-4:
        cand = float(get_curvature_from_plan(
          yaws, yaw_rates, t, v,
          max(delay + _LEAD_MODEL_ACTION_EXTRA + max(extra, exit_extra), 1e-3)))
        # only accept the exit lead if it genuinely reduces the magnitude
        if abs(cand) + 1e-6 < abs(model_term):
          model_term = cand

      if not math.isfinite(model_term):
        return base

      # ---- blend, ANCHORED on the planner value ---------------------------
      # `base` carries the lane-centering trim (curve_lane_bias / lateral_clearance)
      # which does not exist in the model plan, so it must never be replaced --
      # only biased toward the lead-adjusted model term. b is capped at 0.60, so
      # the planner always keeps at least 40 % weight.
      b = float(np.clip(self.blend, 0.0, 1.0))
      if b <= 1e-6:
        return base
      b *= float(np.interp(v, (8.0, 22.0), (0.45, 1.0)))
      b *= float(np.interp(abs(base), (0.001, 0.010), (1.0, 0.55)))
      b = float(np.clip(b, _LEAD_BLEND_MIN, _LEAD_BLEND_MAX))
      if not entering:
        b *= 0.25              # exit bias: do not let the model hold the unwind
      out = model_term * b + base * (1.0 - b)
      if not math.isfinite(out):
        return base
      return float(out)
    except Exception as e:
      # Was a bare `except Exception: return base`, which silently disabled the
      # whole stage when the capnp slice bug above was present. Keep the safe
      # fallback, but surface the first few failures.
      n_err = getattr(self, "_err_logged", 0)
      if n_err < 5:
        self._err_logged = n_err + 1
        try:
          from openpilot.common.swaglog import cloudlog
          cloudlog.error(f"FordPlanLead: {type(e).__name__}: {e} (returning base)")
        except Exception:
          pass
      return base
