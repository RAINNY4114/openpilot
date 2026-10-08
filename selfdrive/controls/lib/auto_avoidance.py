#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Emergency Auto Avoidance

Purpose
-------

ONLY handle imminent collision avoidance.

This helper is NOT:

    - an overtaking helper
    - a normal lane-change helper
    - a comfort braking controller
    - a route planner
    - a lane keeping controller

The intended flow is:

    SceneUnderstanding
            |
            v
    emergency risk
            |
            v
    AutoAvoidanceHelper
            |
            +---- brake request
            |
            +---- emergency lane-change request
            |
            +---- short evasive lane offset

Normal overtaking remains the responsibility of
AutoOvertakeHelper.

Important
---------

Once an emergency direction has been selected, the direction
is latched until the emergency maneuver finishes.

The helper does not automatically return to the original lane.
"""

import time

from cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Basic limits
# ============================================================================

AUTO_AVOID_MIN_SPEED = 25.0 * CV.KPH_TO_MS


# ============================================================================
# Emergency thresholds
# ============================================================================

# TTC
TTC_WARNING_SEC = 5.0
TTC_URGENT_SEC = 3.5
TTC_CRITICAL_SEC = 2.0


# Distance
DIST_WARNING = 30.0
DIST_URGENT = 20.0
DIST_CRITICAL = 12.0


# Relative speed
CLOSING_SPEED_WARNING = 2.0
CLOSING_SPEED_URGENT = 4.0
CLOSING_SPEED_CRITICAL = 7.0


# ============================================================================
# Timing
# ============================================================================

# Critical emergency must not wait for normal lane stability.
CRITICAL_DIRECTION_WAIT_SEC = 0.05

# Urgent emergency gets only a very short confirmation.
URGENT_DIRECTION_WAIT_SEC = 0.15

# Once the obstacle has disappeared, keep emergency state briefly
# to avoid oscillation.
EMERGENCY_CLEAR_SEC = 0.30

# Prevent immediate retrigger after finishing an emergency maneuver.
EMERGENCY_COOLDOWN_SEC = 1.50

# Minimum time for the emergency maneuver to remain active.
MIN_EMERGENCY_ACTIVE_SEC = 0.40


# ============================================================================
# Rear traffic
# ============================================================================

# A close or rapidly approaching rear vehicle makes a side unsafe.
REAR_DIST_CRITICAL = 8.0
REAR_DIST_URGENT = 15.0

REAR_SPEED_CRITICAL = 15.0
REAR_SPEED_URGENT = 10.0


# ============================================================================
# Lane offset
# ============================================================================

# Emergency lateral offset is intentionally moderate.
# Actual steering remains controlled by the lane-change/controller layer.
EMERGENCY_OFFSET = 0.80


# ============================================================================
# Risk levels
# ============================================================================

RISK_NORMAL = 0
RISK_WARNING = 1
RISK_URGENT = 2
RISK_CRITICAL = 3


def _clamp(value, low, high):
  return max(low, min(high, value))


# ============================================================================
# TTC
# ============================================================================

def compute_ttc(lead_dist, lead_rel_speed):
  """
  Calculate time-to-collision using longitudinal relative speed.

  vRel convention:

      negative -> target is slower / ego is closing
      positive -> target is moving away
  """

  if lead_dist is None or lead_rel_speed is None:
    return None

  try:
    distance = float(lead_dist)
    v_rel = float(lead_rel_speed)
  except Exception:
    return None

  if distance <= 0.0:
    return 0.0

  closing_speed = max(-v_rel, 0.0)

  if closing_speed <= 0.1:
    return None

  return distance / closing_speed


# ============================================================================
# Adaptive emergency braking
# ============================================================================

def compute_emergency_brake(
    v_ego,
    lead_dist,
    lead_rel_speed,
    risk_level,
):
  """
  Produce a normalized emergency brake request.

  0.0:
      no emergency braking

  1.0:
      maximum emergency request

  This is deliberately not a normal cruise brake controller.
  """

  if risk_level == RISK_NORMAL:
    return 0.0

  try:
    v_ego = max(float(v_ego), 0.0)
  except Exception:
    return 0.0

  ttc = compute_ttc(
    lead_dist,
    lead_rel_speed,
  )

  # ------------------------------------------------------------------
  # Critical emergency
  # ------------------------------------------------------------------

  if risk_level == RISK_CRITICAL:

    if ttc is not None:

      if ttc <= 1.0:
        return 1.00

      if ttc <= TTC_CRITICAL_SEC:
        return _clamp(
          1.00 -
          (ttc - 1.0) /
          max(TTC_CRITICAL_SEC - 1.0, 0.1),
          0.75,
          1.00,
        )

    return 0.85

  # ------------------------------------------------------------------
  # Urgent
  # ------------------------------------------------------------------

  if risk_level == RISK_URGENT:

    if ttc is not None:

      if ttc <= TTC_CRITICAL_SEC:
        return 0.85

      if ttc <= TTC_URGENT_SEC:
        return _clamp(
          0.65 +
          (
            TTC_URGENT_SEC - ttc
          ) * 0.15,
          0.60,
          0.85,
        )

    return 0.55

  # ------------------------------------------------------------------
  # Warning
  # ------------------------------------------------------------------

  if risk_level == RISK_WARNING:

    return 0.25

  return 0.0


# ============================================================================
# Smooth emergency offset
# ============================================================================

def generate_smooth_evasive_path(
    length,
    target_offset,
):

  length = max(int(length), 2)

  path = []

  for i in range(length):

    t = i / float(length - 1)

    # smoothstep
    s = (
      3.0 * t * t
      -
      2.0 * t * t * t
    )

    path.append(
      target_offset * s
    )

  return path


# ============================================================================
# AutoAvoidanceHelper
# ============================================================================

class AutoAvoidanceHelper:

  # ========================================================================
  # Init
  # ========================================================================

  def __init__(self):
    self.reset()

  # ========================================================================
  # Reset
  # ========================================================================

  def reset(self):

    self._mode = "idle"

    self._risk_level = RISK_NORMAL

    self._out_dir = LaneChangeDirection.none

    self._started_t = None

    self._clear_since = None

    self._cooldown_until = 0.0

    self._last_lc_state = LaneChangeState.off

    self._left_ok_since = None
    self._right_ok_since = None

  # ========================================================================
  # Stable helper
  # ========================================================================

  @staticmethod
  def _stable(
      ok,
      since,
      now,
      duration):

    if not ok:
      return None, False

    if since is None:
      since = now

    return (
      since,
      (now - since) >= duration,
    )

  # ========================================================================
  # Opposite direction
  # ========================================================================

  @staticmethod
  def _opposite(direction):

    if direction == LaneChangeDirection.left:
      return LaneChangeDirection.right

    if direction == LaneChangeDirection.right:
      return LaneChangeDirection.left

    return LaneChangeDirection.none

  # ========================================================================
  # Rear vehicle blocking
  # ========================================================================

  @staticmethod
  def _rear_blocked(
      distance,
      speed,
      critical=False):

    if distance is not None:

      try:
        distance = float(distance)
      except Exception:
        distance = None

    if speed is not None:

      try:
        speed = float(speed)
      except Exception:
        speed = None

    if critical:

      if (
          distance is not None
          and
          distance < REAR_DIST_CRITICAL
      ):
        return True

      if (
          speed is not None
          and
          speed > REAR_SPEED_CRITICAL
      ):
        return True

      return False

    if (
        distance is not None
        and
        distance < REAR_DIST_URGENT
    ):
      return True

    if (
        speed is not None
        and
        speed > REAR_SPEED_URGENT
    ):
      return True

    return False

  # ========================================================================
  # Lane safe
  # ========================================================================

  @staticmethod
  def _lane_safe(
      direction,
      left_ok,
      right_ok):

    if direction == LaneChangeDirection.left:
      return bool(left_ok)

    if direction == LaneChangeDirection.right:
      return bool(right_ok)

    return False

  # ========================================================================
  # Select direction
  # ========================================================================

  @staticmethod
  def _pick_direction(
      left_ok,
      right_ok,
      is_rhd,
      prefer_dir):

    # Explicit preference wins.
    if (
        prefer_dir == LaneChangeDirection.left
        and
        left_ok
    ):
      return LaneChangeDirection.left

    if (
        prefer_dir == LaneChangeDirection.right
        and
        right_ok
    ):
      return LaneChangeDirection.right

    # Only one side available.
    if left_ok and not right_ok:
      return LaneChangeDirection.left

    if right_ok and not left_ok:
      return LaneChangeDirection.right

    if not left_ok and not right_ok:
      return LaneChangeDirection.none

    # Both sides available.
    #
    # Keep the same convention as the original implementation.
    if is_rhd:
      return LaneChangeDirection.right

    return LaneChangeDirection.left

  # ========================================================================
  # Risk classification
  # ========================================================================

  @staticmethod
  def _classify_risk(
      obstacle_in_path,
      is_pedestrian,
      is_cone,
      lead_dist,
      lead_rel_speed):

    ttc = compute_ttc(
      lead_dist,
      lead_rel_speed,
    )

    try:
      distance = (
        float(lead_dist)
        if lead_dist is not None
        else None
      )
    except Exception:
      distance = None

    try:
      closing_speed = max(
        -float(lead_rel_speed),
        0.0,
      ) if lead_rel_speed is not None else 0.0
    except Exception:
      closing_speed = 0.0

    # ------------------------------------------------------------------
    # CRITICAL
    # ------------------------------------------------------------------

    if obstacle_in_path:

      if (
          ttc is not None
          and
          ttc <= TTC_CRITICAL_SEC
      ):
        return RISK_CRITICAL

      if (
          distance is not None
          and
          distance <= DIST_CRITICAL
      ):
        return RISK_CRITICAL

      # Explicit obstacle indication itself is sufficient
      # to enter emergency handling.
      return RISK_CRITICAL

    if is_pedestrian:

      if (
          ttc is not None
          and
          ttc <= TTC_URGENT_SEC
      ):
        return RISK_CRITICAL

      if (
          distance is not None
          and
          distance <= DIST_URGENT
      ):
        return RISK_CRITICAL

    if is_cone:

      if (
          ttc is not None
          and
          ttc <= TTC_CRITICAL_SEC
      ):
        return RISK_CRITICAL

      if (
          distance is not None
          and
          distance <= DIST_CRITICAL
      ):
        return RISK_CRITICAL

    if (
        ttc is not None
        and
        ttc <= TTC_CRITICAL_SEC
    ):
      return RISK_CRITICAL

    if (
        distance is not None
        and
        distance <= DIST_CRITICAL
        and
        closing_speed >= CLOSING_SPEED_CRITICAL
    ):
      return RISK_CRITICAL

    # ------------------------------------------------------------------
    # URGENT
    # ------------------------------------------------------------------

    if (
        ttc is not None
        and
        ttc <= TTC_URGENT_SEC
    ):
      return RISK_URGENT

    if (
        distance is not None
        and
        distance <= DIST_URGENT
        and
        closing_speed >= CLOSING_SPEED_URGENT
    ):
      return RISK_URGENT

    if is_pedestrian and (
        distance is not None
        and
        distance <= DIST_WARNING
    ):
      return RISK_URGENT

    if is_cone and (
        distance is not None
        and
        distance <= DIST_WARNING
    ):
      return RISK_URGENT

    # ------------------------------------------------------------------
    # WARNING
    # ------------------------------------------------------------------

    if (
        ttc is not None
        and
        ttc <= TTC_WARNING_SEC
    ):
      return RISK_WARNING

    if (
        distance is not None
        and
        distance <= DIST_WARNING
        and
        closing_speed >= CLOSING_SPEED_WARNING
    ):
      return RISK_WARNING

    return RISK_NORMAL

  # ========================================================================
  # Update
  # ========================================================================

  def update(
      self,
      *,
      enabled,
      obstacle_in_path,
      lc_state,
      v_ego,
      left_ok,
      right_ok,
      is_rhd,
      manual_blinker,
      bsm_available,
      lead_dist=None,
      lead_rel_speed=None,
      is_pedestrian=False,
      is_cone=False,
      rear_left_dist=None,
      rear_left_speed=None,
      rear_right_dist=None,
      rear_right_speed=None,
      left_lidar_free=None,
      right_lidar_free=None,
      prefer_dir=LaneChangeDirection.none):

    now = time.monotonic()

    request = LaneChangeDirection.none
    brake = 0.0
    hazard = False
    lane_offset = 0.0

    # ======================================================================
    # Safety gates
    # ======================================================================

    if manual_blinker:

      self.reset()
      self._last_lc_state = lc_state

      return (
        request,
        brake,
        hazard,
        lane_offset,
      )

    if not enabled:

      self.reset()
      self._last_lc_state = lc_state

      return (
        request,
        brake,
        hazard,
        lane_offset,
      )

    if float(v_ego) < AUTO_AVOID_MIN_SPEED:

      self.reset()
      self._last_lc_state = lc_state

      return (
        request,
        brake,
        hazard,
        lane_offset,
      )

    # ======================================================================
    # Risk
    # ======================================================================

    risk = self._classify_risk(
      obstacle_in_path,
      is_pedestrian,
      is_cone,
      lead_dist,
      lead_rel_speed,
    )

    self._risk_level = risk

    emergency = (
      risk >= RISK_URGENT
    )

    critical = (
      risk >= RISK_CRITICAL
    )

    # ======================================================================
    # Lane availability
    # ======================================================================

    left_ok = bool(left_ok)
    right_ok = bool(right_ok)

    # Lidar is an additional veto.
    if left_lidar_free is not None:
      left_ok = (
        left_ok
        and
        bool(left_lidar_free)
      )

    if right_lidar_free is not None:
      right_ok = (
        right_ok
        and
        bool(right_lidar_free)
      )

    # ----------------------------------------------------------------------
    # Rear traffic
    #
    # In CRITICAL state we still try to avoid a dangerous rear collision,
    # but we do not require normal 0.6s lane stability.
    # ----------------------------------------------------------------------

    if self._rear_blocked(
        rear_left_dist,
        rear_left_speed,
        critical=critical,
    ):
      left_ok = False

    if self._rear_blocked(
        rear_right_dist,
        rear_right_speed,
        critical=critical,
    ):
      right_ok = False

    # ======================================================================
    # Lane stability
    # ======================================================================

    if critical:

      # Critical emergency:
      #
      # Do NOT wait 600 ms.
      #
      # Current lane information is used immediately.
      left_stable = left_ok
      right_stable = right_ok

      self._left_ok_since = (
        now if left_ok else None
      )

      self._right_ok_since = (
        now if right_ok else None
      )

    else:

      self._left_ok_since, left_stable = self._stable(
        left_ok,
        self._left_ok_since,
        now,
        URGENT_DIRECTION_WAIT_SEC,
      )

      self._right_ok_since, right_stable = self._stable(
        right_ok,
        self._right_ok_since,
        now,
        URGENT_DIRECTION_WAIT_SEC,
      )

    # ======================================================================
    # Brake
    # ======================================================================

    brake = compute_emergency_brake(
      v_ego,
      lead_dist,
      lead_rel_speed,
      risk,
    )

    # ======================================================================
    # Direction
    # ======================================================================

    # Once an emergency direction has been selected,
    # it MUST NOT change during the maneuver.
    if self._out_dir == LaneChangeDirection.none:

      self._out_dir = self._pick_direction(
        left_stable,
        right_stable,
        is_rhd,
        prefer_dir,
      )

    # ======================================================================
    # Lane-change completion detection
    # ======================================================================

    lc_finished = (
      self._last_lc_state ==
      LaneChangeState.laneChangeFinishing
      and
      lc_state ==
      LaneChangeState.off
    )

    # ======================================================================
    # State machine
    # ======================================================================

    # ----------------------------------------------------------------------
    # IDLE
    # ----------------------------------------------------------------------

    if self._mode == "idle":

      if (
          emergency
          and
          now >= self._cooldown_until):

        self._started_t = now

        self._mode = "emergency"

        # Re-evaluate direction immediately.
        self._out_dir = self._pick_direction(
          left_stable,
          right_stable,
          is_rhd,
          prefer_dir,
        )

    # ----------------------------------------------------------------------
    # EMERGENCY
    # ----------------------------------------------------------------------

    elif self._mode == "emergency":

      # Direction is latched.

      if self._out_dir != LaneChangeDirection.none:

        request = self._out_dir

      # --------------------------------------------------------------
      # If collision risk disappears before LC starts,
      # allow quick exit.
      # --------------------------------------------------------------

      if not emergency:

        if self._clear_since is None:
          self._clear_since = now

        clear_ok = (
          now - self._clear_since
          >= EMERGENCY_CLEAR_SEC
        )

        min_active_ok = (
          self._started_t is None
          or
          now - self._started_t
          >= MIN_EMERGENCY_ACTIVE_SEC
        )

        if clear_ok and min_active_ok:

          self._mode = "idle"

          self._cooldown_until = (
            now +
            EMERGENCY_COOLDOWN_SEC
          )

          self._out_dir = (
            LaneChangeDirection.none
          )

      else:

        self._clear_since = None

      # --------------------------------------------------------------
      # Lane change finished
      #
      # IMPORTANT:
      #
      # We DO NOT automatically return to original lane.
      # --------------------------------------------------------------

      if lc_finished:

        self._mode = "completed"

    # ----------------------------------------------------------------------
    # COMPLETED
    # ----------------------------------------------------------------------

    elif self._mode == "completed":

      # Keep emergency request off after the lane change is complete.
      request = LaneChangeDirection.none

      self._cooldown_until = (
        now +
        EMERGENCY_COOLDOWN_SEC
      )

      self._mode = "idle"

      self._out_dir = (
        LaneChangeDirection.none
      )

    # ----------------------------------------------------------------------
    # INVALID STATE
    # ----------------------------------------------------------------------

    else:

      self.reset()

    # ======================================================================
    # Emergency hazard
    # ======================================================================

    hazard = (
      risk >= RISK_URGENT
    )

    # ======================================================================
    # Short evasive lane offset
    # ======================================================================

    if (
        self._mode == "emergency"
        and
        self._out_dir != LaneChangeDirection.none
        and
        emergency
    ):

      if (
          self._out_dir ==
          LaneChangeDirection.left
      ):

        lane_offset = -EMERGENCY_OFFSET

      elif (
          self._out_dir ==
          LaneChangeDirection.right
      ):

        lane_offset = EMERGENCY_OFFSET

    # ======================================================================
    # Save state
    # ======================================================================

    self._last_lc_state = lc_state

    return (
      request,
      brake,
      hazard,
      lane_offset,
    )

  # ========================================================================
  # Diagnostics
  # ========================================================================

  def get_state(self):

    return {
      "mode": self._mode,
      "risk_level": self._risk_level,
      "out_dir": self._out_dir,
      "cooldown_until": self._cooldown_until,
    }