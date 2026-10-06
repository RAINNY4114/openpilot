#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ford / Lincoln Curve Controller
RAINNY4114 Ford curvature-control adaptation for sunnypilot.

Enhanced with BluePilot bp-7.0 angle control (path_angle / c1 signal):
  - Path angle computation: kappa * v_ego * curvature_factor
  - Variable lookup time (VLT): adaptive lookahead for curve entry/exit
  - PSCM saturation handling: rate-limit angle decrease near DBC limits
  - Soft ROC: speed-dependent path_angle rate-of-change limit

Purpose
-------
Improve Ford / Lincoln non-CAN-FD EPS corner behavior by controlling
curvature request rate, curvature error, entry/hold/exit behavior and
driver-intervention reset.

The path_angle (c1) signal provides additional heading authority to the
PSCM, especially beneficial for large turns and continuous curves where
curvature alone may saturate or unwind too early.
"""

import os
import math
import time
from enum import Enum, auto

import numpy as np

from openpilot.common.params import Params


# ============================================================================
# Ford limits
# ============================================================================

MAX_CURVATURE = 0.02

# Ford EPS equivalent lateral acceleration limit.
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
CURVATURE_FILTER = 0.15


# ============================================================================
# Path Angle (c1) Constants - Adapted from BluePilot bp-7.0
# ============================================================================

# PSCM short lookahead distance (d_ref) vs speed (m/s -> m)
_PSCM_DREF_SPEEDS_MS = (0.0, 4.17, 27.78, 41.67, 50.0, 55.56)
_PSCM_DREF_M = (0.5, 0.95, 1.4, 2.075, 2.75, 3.875)

# Variable lookup time (VLT): adapts model lookahead to speed and curve depth.
# Extra lookahead collapses toward zero at high speed (PSCM responds faster)
# and at large curvature (prevents importing "start unwinding" signal too early).
_VLT_T_EXTRA_MAX = 0.10              # max extra lookahead above t_base (s)
_VLT_V_LOW_MS = 25.0 * 0.44704       # 25 mph -- full extra lookahead at or below
_VLT_V_HIGH_MS = 55.0 * 0.44704      # 55 mph -- no extra lookahead at or above
_VLT_KAPPA_FULL = 0.005              # 1/m -- full extra lookahead below this curvature
_VLT_KAPPA_TAPER = 0.020             # 1/m -- no extra lookahead above this curvature

# PSCM saturation handling: rate-limit path_angle decrease near DBC limits.
# Prevents snap corrections when PSCM is released from authority limit.
# Scaled for 20Hz lateral tick cadence (STEER_STEP=5, DT_CTRL=0.01).
_PSCM_SAT_UNWIND_RATE = 0.02          # rad/call at 20Hz = 0.40 rad/s (23 deg/s)
FORD_DBC_PATH_ANGLE_MIN = -0.5        # rad
FORD_DBC_PATH_ANGLE_MAX = 0.5235      # rad (~30 degrees)

# Soft ROC on path_angle (rad/call at 20Hz).
# Scaled x5 from BluePilot 100Hz values to restore same real-world rate on 20Hz cadence.
# At 9-10 m/s: 63 deg/s, at 15 m/s: 49 deg/s, at 25 m/s: 10 deg/s
_SOFT_ROC_SPEEDS = [9., 10., 15., 25.]
_SOFT_ROC_RATES = [0.055, 0.055, 0.0425, 0.009]

# Path angle gains for CAN (non-CANFD) vehicles, including Lincoln Nautilus.
# Low-speed low-curvature gain is always 1.0 (no boost needed for gentle low-speed curves).
# Low-speed high-curvature gain is 1.30 (30% boost for sharp low-speed turns).
# High-speed low-curvature gain is 1.15 (15% boost for gentle highway curves).
# High-speed high-curvature gain is 1.05 (5% boost; high-speed large curves are
#   well-served by curvature alone, excessive path_angle could cause instability).
_PA_GAIN_LOW_CURV_HIGH_SPEED = 1.15
_PA_GAIN_HIGH_CURV_LOW_SPEED = 1.30
_PA_GAIN_HIGH_CURV_HIGH_SPEED = 1.05


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

  def _read_params(self):
    now = time.monotonic()

    if now - self.last_params_read < PARAM_REFRESH_SEC:
      return

    self.last_params_read = now

    try:
      value = self.params.get_bool("dp_htd_enabled")
      self.enabled = bool(value)
    except Exception:
      self.enabled = False

    try:
      value = self.params.get("dp_htd_turn_angle_threshold")

      if value is not None:
        if isinstance(value, bytes):
          value = value.decode("utf-8", errors="ignore")

        self.angle_threshold_deg = float(value)

    except Exception:
      self.angle_threshold_deg = HTD_DEFAULT_ANGLE_THRESHOLD_DEG

    self.angle_threshold_deg = max(
      20.0,
      min(self.angle_threshold_deg, 120.0),
    )

  def reset(self):
    self.state = HTDState.INACTIVE
    self.state_change_time = 0.0
    self.trigger_start_time = 0.0
    self.max_turn_angle = 0.0
    self.dynamic_delay = HTD_MIN_RAMP_SEC

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
    # Path angle (c1) state -- BluePilot bp-7.0 angle control
    # ------------------------------------------------------------------------

    self.path_angle = 0.0
    self.path_angle_last = 0.0

  # ==========================================================================
  # Reset
  # ==========================================================================

  def reset(self):

    self.last_curvature = 0.0

    self.last_requested_curvature = 0.0

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

    # Reset path angle state
    self.path_angle = 0.0
    self.path_angle_last = 0.0

    self.htd.reset()

  # ==========================================================================
  # Lateral acceleration limit
  # ==========================================================================

  def _speed_limit(self, curvature, speed):

    if speed < MIN_SPEED:
      return curvature

    max_curvature = (
      MAX_LATERAL_ACCEL
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

    if speed <= CURRENT_CURVATURE_MIN_SPEED:
      return curvature

    return float(
      np.clip(
        curvature,
        current_curvature - CURVATURE_ERROR,
        current_curvature + CURVATURE_ERROR,
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

    windup_rate = float(
      np.interp(
        v,
        [5.0, 25.0],
        [0.00050, 0.00011],
      )
    )

    unwind_rate = float(
      np.interp(
        v,
        [5.0, 25.0],
        [0.00055, 0.00020],
      )
    )

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

    if self.curve_phase == "ENTRY":

      gain = ENTRY_GAIN

    elif self.curve_phase == "HOLD":

      gain = MID_GAIN

    elif self.curve_phase == "EXIT":

      # IMPORTANT:
      #
      # Do not boost curvature during exit.
      # Let Ford's unwind limiter release the wheel.
      gain = 1.0

    else:

      if speed < 15.0:
        gain = ENTRY_GAIN
      elif speed < 28.0:
        gain = MID_GAIN
      else:
        gain = EXIT_GAIN

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

    tau = 5.0

    dt = 0.01

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
  # Path Angle (c1) computation -- BluePilot bp-7.0 angle control
  # ==========================================================================

  def _pscm_d_ref_m(self, v_ego):
    """
    PSCM short lookahead distance (d_ref) vs speed.

    Returns the PSCM's internal short lookahead distance in meters.
    This is used for VLT computation and as a reference for path_angle.
    """
    v = max(float(v_ego), 0.0)
    d = float(np.interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
    if v > _PSCM_DREF_SPEEDS_MS[-1]:
      # Doc: d_ref table ends at 3.875 m; cap at 5 m for high speed.
      d = min(5.0, d)
    return d

  def _compute_path_angle(self, curvature, v_ego):
    """
    Compute path_angle (c1) from curvature.

    Adapted from BluePilot bp-7.0 lateral_angle_ext.py.

    Formula: path_angle = kappa * v_ego * curvature_factor

    Where curvature_factor is a speed and curvature-magnitude-dependent
    gain that provides additional heading authority for large and
    continuous turns.

    The VLT (Variable Lookup Time) adjustment provides:
    - Full extra lookahead at low speed + curve entry (gradual pre-steering)
    - Tapered lookahead at high speed or large curvature (prevent early unwind)
    - Zero extra lookahead on curve exit (let planner unwind naturally)

    This is especially beneficial for:
    - Large turns: path_angle provides heading authority beyond curvature
    - Continuous turns: VLT prevents premature unwind between curves
    """
    # Speed-dependent gains
    low_gain = float(np.interp(
      v_ego, [13.5, 26.82],
      [1.0, _PA_GAIN_LOW_CURV_HIGH_SPEED]
    ))
    high_gain = float(np.interp(
      v_ego, [13.5, 26.82],
      [_PA_GAIN_HIGH_CURV_LOW_SPEED, _PA_GAIN_HIGH_CURV_HIGH_SPEED]
    ))

    # Curvature-dependent factor: boost for larger curvatures (bigger turns)
    curvature_factor = float(np.interp(
      abs(curvature), [0.0007, 0.001],
      [low_gain, high_gain]
    ))

    # Base path_angle = kappa * v_ego * curvature_factor
    path_angle = curvature * v_ego * curvature_factor

    # Variable lookup time (VLT): adapt lookahead to speed and curve phase
    speed_factor = float(np.interp(
      v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]
    ))

    if self.curve_phase == "ENTRY":
      # On curve entry: keep full extra lookahead for gradual pre-steering.
      # This is critical for large turns -- early pre-steering helps the PSCM
      # build up steering authority before the curve peak.
      kappa_factor = 1.0
    elif self.curve_phase == "EXIT":
      # On curve exit: no extra lookahead -- let the planner's natural unwind
      # dominate. This prevents the path_angle from holding the car in the
      # curve too long, which would cause overshoot on exit.
      kappa_factor = 0.0
    else:
      # HOLD or STRAIGHT: taper by curvature magnitude.
      # Small curvature (gentle curve): full extra lookahead.
      # Large curvature (sharp curve): no extra lookahead (PSCM already engaged).
      kappa_factor = float(np.interp(
        abs(curvature),
        [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER],
        [1.0, 0.0]
      ))

    extra_lookahead = _VLT_T_EXTRA_MAX * speed_factor * kappa_factor
    # Apply VLT as a multiplicative boost to path_angle
    path_angle *= (1.0 + extra_lookahead)

    return path_angle

  def _pscm_saturation_handle(self, path_angle, path_angle_last):
    """
    PSCM saturation handling.

    When path_angle is near DBC limits (PSCM at authority limit):
    - Block magnitude increases (can't steer more)
    - Rate-limit decreases to _PSCM_SAT_UNWIND_RATE (prevent snap on release)

    Without this, the desired angle drops rapidly at a sharp curve apex while
    the PSCM is physically pinned, causing a snap correction the moment the
    PSCM is released. This is a critical improvement for large turns.

    Adapted from BluePilot bp-7.0 lateral_angle_ext.py.
    """
    # Check if path_angle is near DBC limits (PSCM saturation proxy)
    dbc_sat = (
      path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
      path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90
    )

    if dbc_sat:
      last_mag = abs(path_angle_last)
      curr_mag = abs(path_angle)

      if curr_mag > last_mag:
        # Magnitude growing while saturated -- block (can't exceed PSCM authority)
        path_angle = path_angle_last
      elif last_mag - curr_mag > _PSCM_SAT_UNWIND_RATE:
        # Decreasing too fast -- rate-limit to prevent snap correction on release.
        # This holds the car slightly more in the curve during saturation, at the
        # cost of a smoother release transition.
        limited_mag = last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(limited_mag if path_angle_last >= 0 else -limited_mag)

    return path_angle

  def _soft_roc_limit(self, path_angle, path_angle_last, v_ego):
    """
    Soft rate-of-change (ROC) limit on path_angle.

    Speed-dependent ROC prevents abrupt path_angle changes, ensuring
    smooth transitions between curves and during curve entry/exit.

    Scaled for 20Hz lateral tick cadence (STEER_STEP=5, DT_CTRL=0.01).
    Values are x5 of BluePilot's 100Hz originals to restore same real-world rate.

    At 9-10 m/s: 63 deg/s (generous, for low-speed maneuvering)
    At 15 m/s:  49 deg/s (moderate)
    At 25 m/s:  10 deg/s (conservative, for highway stability)
    """
    roc = float(np.interp(v_ego, _SOFT_ROC_SPEEDS, _SOFT_ROC_RATES))
    return float(np.clip(
      path_angle,
      path_angle_last - roc,
      path_angle_last + roc
    ))

  def _finalize_path_angle(self, curvature, v_ego):
    """
    Compute and apply all path_angle processing.

    Called after curvature is finalized (rate-limited, filtered, clamped).
    This is the single entry point for path_angle computation.

    Pipeline:
    1. Compute path_angle from curvature (speed/curvature-dependent gain + VLT)
    2. PSCM saturation handling (rate-limit near DBC limits)
    3. Soft ROC limit (speed-dependent rate-of-change)
    4. Clamp to DBC limits
    """
    # Step 1: Compute path_angle from curvature
    path_angle = self._compute_path_angle(curvature, v_ego)

    # Step 2: PSCM saturation handling
    path_angle = self._pscm_saturation_handle(path_angle, self.path_angle_last)

    # Step 3: Soft ROC limit
    path_angle = self._soft_roc_limit(path_angle, self.path_angle_last, v_ego)

    # Step 4: Clamp to DBC limits
    path_angle = min(
      FORD_DBC_PATH_ANGLE_MAX,
      max(FORD_DBC_PATH_ANGLE_MIN, path_angle)
    )

    # Store state
    self.path_angle_last = path_angle
    self.path_angle = path_angle

    return path_angle

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
  ):
    """
    Main Ford curvature controller with path_angle (c1) computation.

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

    Returns:
      final curvature (float)

    Side effects:
      self.path_angle -- computed path_angle (c1) signal for the PSCM.
      CarController reads this attribute after calling update().
    """

    if not active:

      self.reset()

      return 0.0

    self.active = True

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

      # Reset path_angle during human turn override
      self.path_angle = 0.0
      self.path_angle_last = 0.0

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
        -MAX_CURVATURE,
        min(
          MAX_CURVATURE,
          target,
        )
      )

      output = self._apply_htd_ramp(
        target,
        speed,
      )

      self.last_curvature = output

      # Compute path_angle from the rate-limited curvature during HTD ramp.
      # This ensures path_angle ramps back in smoothly alongside curvature.
      self._finalize_path_angle(output, speed)

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

        # Compute path_angle from the rate-limited curvature during post-reset ramp.
        self._finalize_path_angle(output, speed)

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

    if abs(curvature) > 0.002:

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
      -MAX_CURVATURE,
      min(
        MAX_CURVATURE,
        curvature,
      )
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

    # Final clamp.
    filtered = max(
      -MAX_CURVATURE,
      min(
        MAX_CURVATURE,
        filtered,
      )
    )

    self.last_curvature = float(
      filtered
    )

    # ------------------------------------------------------------------------
    # Path Angle (c1) computation -- BluePilot bp-7.0 angle control
    #
    # Compute path_angle from the final filtered curvature.
    # This provides additional heading authority to the PSCM, especially
    # beneficial for large turns and continuous curves.
    #
    # The full pipeline:
    #   1. Compute path_angle from curvature (speed/curvature gain + VLT)
    #   2. PSCM saturation handling (rate-limit near DBC limits)
    #   3. Soft ROC limit (speed-dependent rate-of-change)
    #   4. Clamp to DBC limits [-0.5, 0.5235] rad
    # ------------------------------------------------------------------------

    self._finalize_path_angle(filtered, speed)

    return float(
      filtered
    )


# ============================================================================
# Adaptive model-plan lead (FordPlanLead)
#
# RESTORED 2026-10-06: the trimmed ford_curve_controller.py dropped this class
# while controlsd.py still imports it, which crashed controlsd with
#   ImportError: cannot import name 'FordPlanLead'
# and made cruise control unavailable.
#
# Ported from hwh-kavin/openpilot sp-master-bp-260825. Everything below is
# inert unless dp_ford_lead_enable is set, and apply() can never raise into
# the caller (all exceptions fall back to base_curvature).
# ============================================================================

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

# liveDelay clamp
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
      # TypeError: an integer is required. Materialise with list() first.
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
      n_err = getattr(self, "_err_logged", 0)
      if n_err < 5:
        self._err_logged = n_err + 1
        try:
          from openpilot.common.swaglog import cloudlog
          cloudlog.error(f"FordPlanLead: {type(e).__name__}: {e} (returning base)")
        except Exception:
          pass
      return base
