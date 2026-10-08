#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DP 10.2 Auto Overtake Helper
============================

Purpose
-------

Normal overtaking decision layer.

This module does NOT perform vehicle control.

It only determines whether a normal overtaking lane-change request
may be generated.

Architecture
------------

SceneUnderstanding
        |
        v
  scene / slow_vehicle
        |
        v
AutoOvertakeHelper
        |
        v
LaneChangeDirection
        |
        v
existing lane-change controller

Important
---------

This helper is NOT an emergency avoidance controller.

Emergency situations such as:

    - oncoming vehicle
    - static obstacle
    - pedestrian
    - cone
    - construction zone
    - very low TTC

must be handled by AutoAvoidanceHelper instead.

Normal overtaking is therefore deliberately conservative.
"""

import time

from cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Speed limits
# ============================================================================

OVERTAKE_MIN_SPEED = 65.0 * CV.KPH_TO_MS
OVERTAKE_MIN_CRUISE_SPEED = 75.0 * CV.KPH_TO_MS

# Minimum advantage of desired cruise speed over lead speed.
OVERTAKE_SPEED_DELTA = 2.0 * CV.KPH_TO_MS


# ============================================================================
# Lead vehicle stability
# ============================================================================

OVERTAKE_HEADWAY_MAX_S = 6.0
OVERTAKE_LEAD_STABLE_SEC = 0.50


# ============================================================================
# Lane change timing
# ============================================================================

PREPARE_BEFORE_LC_SEC = 0.30

RETURN_CLEAR_DELAY_SEC = 3.0
RETURN_MIN_TIME_AFTER_OUT_SEC = 3.0

OVERTAKE_COOLDOWN_SEC = 8.0

CLEAR_LANE_STABLE_SEC = 0.30


# ============================================================================
# Rear vehicle safety
# ============================================================================

REAR_DIST_TH = 20.0

# Relative speed of rear vehicle.
#
# This is deliberately conservative.
REAR_SPEED_TH = 12.0


# ============================================================================
# Overtake completion
# ============================================================================

PASSED_LEAD_DIST = 35.0
PASSED_LEAD_SPEED = 4.0


# ============================================================================
# Lane preference
# ============================================================================

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2


class AutoOvertakeHelper:

  # ==========================================================================
  # Initialization
  # ==========================================================================

  def __init__(self):
    self.reset()

  # ==========================================================================
  # Reset
  # ==========================================================================

  def reset(self):
    self._mode = "idle"

    self._out_dir = LaneChangeDirection.none
    self._return_dir = LaneChangeDirection.none

    self._cooldown_until = 0.0

    self._need_since = None
    self._prepare_since = None

    self._clear_since = None
    self._out_finished_t = None

    self._last_lc_state = LaneChangeState.off

    self._left_ok_since = None
    self._right_ok_since = None

    self._stay_in_fast_lane = False

    # Diagnostic state.
    self._last_reason = "idle"

  # ==========================================================================
  # Helpers
  # ==========================================================================

  @staticmethod
  def _stable(
      ok,
      since,
      now,
      stable_sec):

    if ok:
      if since is None:
        since = now

      return (
        since,
        (now - since) >= stable_sec,
      )

    return None, False

  @staticmethod
  def _opposite(direction):

    if direction == LaneChangeDirection.left:
      return LaneChangeDirection.right

    if direction == LaneChangeDirection.right:
      return LaneChangeDirection.left

    return LaneChangeDirection.none

  @staticmethod
  def _rear_blocked(
      distance,
      speed):

    if (
        distance is not None
        and distance > 0.0
        and distance < REAR_DIST_TH
    ):
      return True

    if (
        speed is not None
        and speed > REAR_SPEED_TH
    ):
      return True

    return False

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

  # ==========================================================================
  # Lane scoring
  # ==========================================================================

  @staticmethod
  def _lane_score(
      side,
      v_ego,
      v_lead,
      rear_dist,
      rear_speed,
      lane_free,
      bsm_blocked,
      is_highway,
      lane_preference,
      is_rhd):

    # ------------------------------------------------------------
    # Hard rejection
    # ------------------------------------------------------------

    if not lane_free:
      return -100.0

    if bsm_blocked:
      return -100.0

    score = 0.0

    # ------------------------------------------------------------
    # Rear vehicle safety
    # ------------------------------------------------------------

    if (
        rear_dist is not None
        and rear_dist > 0.0):

      if rear_dist < REAR_DIST_TH:
        score -= 40.0

      else:
        score += min(
          rear_dist / 10.0,
          5.0,
        )

    if (
        rear_speed is not None
        and rear_speed > 0.0):

      if rear_speed > REAR_SPEED_TH:
        score -= 35.0

      else:
        score += 2.0

    # ------------------------------------------------------------
    # Expected overtaking benefit
    # ------------------------------------------------------------

    if v_lead is not None:

      delta = (
        float(v_ego) -
        float(v_lead)
      )

      if delta > 0.0:
        score += min(
          delta * 1.5,
          15.0,
        )

    # ------------------------------------------------------------
    # Highway preference
    # ------------------------------------------------------------

    if is_highway:

      if lane_preference == LANE_PREF_KEEP_LEFT:

        score += (
          5.0 if side == "left"
          else -5.0
        )

      elif lane_preference == LANE_PREF_KEEP_RIGHT:

        score += (
          5.0 if side == "right"
          else -5.0
        )

      else:

        # Keep the vehicle's preferred overtaking
        # direction relatively stable.
        #
        # This is only a weak preference.
        if is_rhd:
          score += (
            2.0 if side == "right"
            else 0.0
          )
        else:
          score += (
            2.0 if side == "left"
            else 0.0
          )

    return score

  # ==========================================================================
  # Choose lane
  # ==========================================================================

  def _choose_lane(
      self,
      v_ego,
      v_lead,
      rear_left_dist,
      rear_left_speed,
      rear_right_dist,
      rear_right_speed,
      left_ok,
      right_ok,
      left_bsm,
      right_bsm,
      is_highway,
      lane_preference,
      is_rhd):

    left_score = self._lane_score(
      "left",
      v_ego,
      v_lead,
      rear_left_dist,
      rear_left_speed,
      left_ok,
      left_bsm,
      is_highway,
      lane_preference,
      is_rhd,
    )

    right_score = self._lane_score(
      "right",
      v_ego,
      v_lead,
      rear_right_dist,
      rear_right_speed,
      right_ok,
      right_bsm,
      is_highway,
      lane_preference,
      is_rhd,
    )

    # No safe lane.
    if (
        left_score <= -99.0
        and
        right_score <= -99.0
    ):
      return LaneChangeDirection.none

    # Scores too close.
    #
    # Do not randomly choose a side.
    if abs(left_score - right_score) < 2.0:

      if is_rhd and right_ok:
        return LaneChangeDirection.right

      if not is_rhd and left_ok:
        return LaneChangeDirection.left

      return LaneChangeDirection.none

    if left_score > right_score:
      return LaneChangeDirection.left

    return LaneChangeDirection.right

  # ==========================================================================
  # Determine whether overtaking is actually necessary
  # ==========================================================================

  def _need_overtake(
      self,
      now,
      lead_present,
      lead_d,
      v_lead,
      v_ego,
      v_cruise):

    if not lead_present:
      self._need_since = None
      return False

    if lead_d is None or lead_d <= 0.0:
      self._need_since = None
      return False

    if v_lead is None:
      self._need_since = None
      return False

    v_ego = float(v_ego)
    v_lead = float(v_lead)
    v_cruise = float(v_cruise)

    headway = (
      float(lead_d) /
      max(v_ego, 0.1)
    )

    closing_speed = (
      v_ego -
      v_lead
    )

    # ------------------------------------------------------------
    # Lead is sufficiently slower than desired speed.
    # ------------------------------------------------------------

    speed_need = (
      v_cruise -
      v_lead
    ) >= OVERTAKE_SPEED_DELTA

    # ------------------------------------------------------------
    # We are actually approaching it.
    # ------------------------------------------------------------

    closing_need = closing_speed > 0.5

    # ------------------------------------------------------------
    # Don't overtake a very distant object.
    # ------------------------------------------------------------

    distance_need = (
      headway <=
      OVERTAKE_HEADWAY_MAX_S
    )

    need = (
      speed_need
      and closing_need
      and distance_need
    )

    if not need:
      self._need_since = None
      return False

    if self._need_since is None:
      self._need_since = now

    return (
      now -
      self._need_since
    ) >= OVERTAKE_LEAD_STABLE_SEC

  # ==========================================================================
  # Lead passed
  # ==========================================================================

  @staticmethod
  def _lead_passed(
      lead_present,
      lead_d,
      v_ego,
      v_lead):

    if not lead_present:
      return True

    if lead_d is None:
      return True

    if lead_d > PASSED_LEAD_DIST:
      return True

    if v_lead is not None:

      if (
          float(v_ego) -
          float(v_lead)
      ) > PASSED_LEAD_SPEED:

        return True

    return False

  # ==========================================================================
  # Main update
  # ==========================================================================

  def update(
      self,
      *,
      enabled,
      lc_state,
      v_ego,
      v_cruise,

      # SceneUnderstanding inputs
      scene_type=None,
      slow_vehicle=False,
      oncoming=False,
      static_obstacle=False,
      pedestrian=False,
      construction=False,

      # Lead
      lead_present=False,
      lead_d=None,
      v_lead=None,

      # Lane state
      left_ok=False,
      right_ok=False,

      is_rhd=False,
      manual_blinker=False,

      # BSM
      bsm_available=True,

      rear_left_dist=None,
      rear_left_speed=None,
      rear_right_dist=None,
      rear_right_speed=None,

      # Lidar
      left_lidar_free=None,
      right_lidar_free=None,

      left_bsm=False,
      right_bsm=False,

      lane_preference=LANE_PREF_AUTO,

      min_cruise_speed=None):

    now = time.monotonic()

    request = LaneChangeDirection.none

    # ========================================================================
    # Immediate cancellation
    # ========================================================================

    if manual_blinker:

      self.reset()
      self._last_lc_state = lc_state
      self._last_reason = "manual_blinker"

      return request

    if not enabled:

      self.reset()
      self._last_lc_state = lc_state
      self._last_reason = "disabled"

      return request

    if not bsm_available:

      self.reset()
      self._last_lc_state = lc_state
      self._last_reason = "bsm_unavailable"

      return request

    # ========================================================================
    # Speed gate
    # ========================================================================

    if min_cruise_speed is None:
      min_cruise_speed = OVERTAKE_MIN_CRUISE_SPEED

    if float(v_ego) < OVERTAKE_MIN_SPEED:

      self.reset()
      self._last_lc_state = lc_state
      self._last_reason = "ego_speed_low"

      return request

    if float(v_cruise) < float(min_cruise_speed):

      self.reset()
      self._last_lc_state = lc_state
      self._last_reason = "cruise_speed_low"

      return request

    # ========================================================================
    # Scene safety gate
    #
    # This is the most important change.
    #
    # Normal overtaking must NEVER compete with emergency avoidance.
    # ========================================================================

    emergency_scene = (
      bool(oncoming)
      or bool(static_obstacle)
      or bool(pedestrian)
      or bool(construction)
    )

    if emergency_scene:

      self.reset()
      self._last_lc_state = lc_state

      self._last_reason = (
        "emergency_scene"
      )

      return request

    # ========================================================================
    # Explicit SceneUnderstanding gate
    #
    # If a scene classifier is available, only normal/slow vehicle scenes
    # can enter normal overtaking.
    # ========================================================================

    if scene_type is not None:

      allowed_scene = (
        scene_type == "normal"
        or
        scene_type == "slow_vehicle"
      )

      if not allowed_scene:

        self.reset()
        self._last_lc_state = lc_state

        self._last_reason = (
          "scene_not_allowed"
        )

        return request

    # ========================================================================
    # Rear vehicle filtering
    # ========================================================================

    if self._rear_blocked(
        rear_left_dist,
        rear_left_speed,
    ):
      left_ok = False

    if self._rear_blocked(
        rear_right_dist,
        rear_right_speed,
    ):
      right_ok = False

    # ========================================================================
    # Lidar confirmation
    #
    # IMPORTANT:
    #
    # Never overwrite lane state.
    #
    # Existing lane safety AND lidar safety must both be true.
    # ========================================================================

    if left_lidar_free is not None:

      left_ok = bool(
        left_ok and
        left_lidar_free
      )

    if right_lidar_free is not None:

      right_ok = bool(
        right_ok and
        right_lidar_free
      )

    # ========================================================================
    # BSM
    # ========================================================================

    if left_bsm:
      left_ok = False

    if right_bsm:
      right_ok = False

    # ========================================================================
    # Lane stability
    # ========================================================================

    self._left_ok_since, left_stable = (
      self._stable(
        bool(left_ok),
        self._left_ok_since,
        now,
        CLEAR_LANE_STABLE_SEC,
      )
    )

    self._right_ok_since, right_stable = (
      self._stable(
        bool(right_ok),
        self._right_ok_since,
        now,
        CLEAR_LANE_STABLE_SEC,
      )
    )

    # ========================================================================
    # Highway
    # ========================================================================

    is_highway = (
      float(v_ego) >
      22.0
    )

    # ========================================================================
    # Select lane
    # ========================================================================

    best_dir = self._choose_lane(
      v_ego,
      v_lead,

      rear_left_dist,
      rear_left_speed,

      rear_right_dist,
      rear_right_speed,

      left_stable,
      right_stable,

      left_bsm,
      right_bsm,

      is_highway,
      lane_preference,
      is_rhd,
    )

    # ========================================================================
    # Determine overtaking need
    # ========================================================================

    need_overtake = self._need_overtake(
      now,

      lead_present,
      lead_d,
      v_lead,

      v_ego,
      v_cruise,
    )

    # SceneUnderstanding slow vehicle should reinforce,
    # not override, physical lead validation.
    if not slow_vehicle:
      need_overtake = False

    # ========================================================================
    # Lane change completion
    # ========================================================================

    lc_finished = (
      self._last_lc_state ==
      LaneChangeState.laneChangeFinishing
      and
      lc_state ==
      LaneChangeState.off
    )

    # ========================================================================
    # State machine
    # ========================================================================

    # ------------------------------------------------------------------------
    # IDLE
    # ------------------------------------------------------------------------

    if self._mode == "idle":

      if (
          need_overtake
          and
          best_dir != LaneChangeDirection.none
          and
          now >= self._cooldown_until):

        self._out_dir = best_dir
        self._prepare_since = now

        self._mode = "preparing"

        self._last_reason = (
          "overtake_prepare"
        )

    # ------------------------------------------------------------------------
    # PREPARING
    # ------------------------------------------------------------------------

    elif self._mode == "preparing":

      # Lead no longer requires overtaking.
      if not need_overtake:

        self.reset()
        self._last_lc_state = lc_state
        self._last_reason = (
          "overtake_no_longer_needed"
        )

        return request

      # Selected lane became unsafe.
      if not self._lane_safe(
          self._out_dir,
          left_stable,
          right_stable):

        self._prepare_since = None

        self._last_reason = (
          "target_lane_not_safe"
        )

      else:

        if self._prepare_since is None:
          self._prepare_since = now

        if (
            now -
            self._prepare_since
            >= PREPARE_BEFORE_LC_SEC):

          self._mode = "changing_out"

          request = self._out_dir

          self._last_reason = (
            "overtake_lane_change"
          )

    # ------------------------------------------------------------------------
    # CHANGING OUT
    # ------------------------------------------------------------------------

    elif self._mode == "changing_out":

      request = self._out_dir

      # Do not dynamically change direction here.
      #
      # Once lane change has started, direction must remain latched.
      if lc_finished:

        self._mode = "waiting_return"

        self._out_finished_t = now
        self._clear_since = None

        self._last_reason = (
          "overtake_lane_change_finished"
        )

    # ------------------------------------------------------------------------
    # WAITING RETURN
    # ------------------------------------------------------------------------

    elif self._mode == "waiting_return":

      passed_lead = self._lead_passed(
        lead_present,
        lead_d,
        v_ego,
        v_lead,
      )

      return_dir = self._opposite(
        self._out_dir
      )

      return_safe = self._lane_safe(
        return_dir,
        left_stable,
        right_stable,
      )

      # ------------------------------------------------------------
      # Don't immediately return.
      # ------------------------------------------------------------

      min_wait_ok = (
        self._out_finished_t is None
        or
        now -
        self._out_finished_t
        >= RETURN_MIN_TIME_AFTER_OUT_SEC
      )

      if (
          passed_lead
          and
          return_safe
          and
          min_wait_ok):

        if self._clear_since is None:
          self._clear_since = now

        if (
            now -
            self._clear_since
            >= RETURN_CLEAR_DELAY_SEC):

          self._return_dir = return_dir

          self._mode = "changing_back"

          request = self._return_dir

          self._last_reason = (
            "return_to_original_lane"
          )

      else:

        self._clear_since = None

        # ----------------------------------------------------------
        # Stay in overtaking lane.
        # ----------------------------------------------------------

        self._stay_in_fast_lane = True

        self._last_reason = (
          "stay_in_overtaking_lane"
        )

    # ------------------------------------------------------------------------
    # CHANGING BACK
    # ------------------------------------------------------------------------

    elif self._mode == "changing_back":

      request = self._return_dir

      if lc_finished:

        self._mode = "idle"

        self._cooldown_until = (
          now +
          OVERTAKE_COOLDOWN_SEC
        )

        self._out_dir = (
          LaneChangeDirection.none
        )

        self._return_dir = (
          LaneChangeDirection.none
        )

        self._need_since = None
        self._prepare_since = None
        self._clear_since = None
        self._out_finished_t = None

        self._stay_in_fast_lane = False

        self._last_reason = (
          "overtake_complete"
        )

    # ------------------------------------------------------------------------
    # Unknown state
    # ------------------------------------------------------------------------

    else:

      self.reset()
      self._last_lc_state = lc_state

      self._last_reason = (
        "invalid_state"
      )

      return LaneChangeDirection.none

    # ========================================================================
    # Save LC state
    # ========================================================================

    self._last_lc_state = lc_state

    return request

  # ==========================================================================
  # Diagnostics
  # ==========================================================================

  def get_state(self):

    return {
      "mode":
        self._mode,

      "out_direction":
        self._out_dir,

      "return_direction":
        self._return_dir,

      "cooldown_until":
        self._cooldown_until,

      "stay_in_fast_lane":
        self._stay_in_fast_lane,

      "reason":
        self._last_reason,
    }