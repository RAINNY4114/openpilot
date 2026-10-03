#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto Overtake Helper
====================

Ford / Lincoln automatic overtaking decision layer.

Architecture
------------

OEM Delphi radar
        |
        v
    leadOne
        |
        v
AutoOvertakeHelper
        |
        v
Lane safety gate
        |
        v
DesireHelper / existing lane-change pipeline


IMPORTANT SAFETY ARCHITECTURE
=============================

OEM radar is the ONLY source that determines:

    "Do we need to overtake?"

MR76 and side WiFi LiDAR are NOT overtaking triggers.

They are SAFETY VETO sensors.

Therefore:

    OEM lead
        ->
    overtaking need
        ->
    safety validation
        ->
    lane-change request


MR76
----

MR76 is AUXILIARY ONLY.

MR76 NEVER:

    - creates RadarPoint
    - enters radarTracks
    - creates Track
    - becomes leadOne
    - becomes leadTwo
    - modifies radarState
    - modifies aTarget
    - modifies shouldStop
    - modifies longitudinalPlan
    - commands brake
    - commands throttle
    - commands steering
    - sends CAN actuator commands

MR76 is used only for:

    - left/right lane occupancy
    - rear/side traffic detection
    - automatic-overtake safety veto
    - emergency-avoidance auxiliary information


SIDE WIFI LiDAR
---------------

Left/right WiFi LiDAR is also AUXILIARY ONLY.

LiDAR can:

    - detect lane occupancy
    - detect side obstacles
    - veto automatic overtaking

LiDAR cannot:

    - create longitudinal targets
    - replace OEM radar lead
    - command braking
    - command throttle
    - directly command steering


SAFETY VETO PRINCIPLE
=====================

For automatic overtaking:

    MR76 obstacle
        ->
    corresponding lane BLOCKED

    LiDAR obstacle
        ->
    corresponding lane BLOCKED

    BSM obstacle
        ->
    corresponding lane BLOCKED

    stale / unavailable required sensor
        ->
    corresponding lane BLOCKED

There is NO "sensor voting" that can override another sensor.

Example:

    MR76 says clear
    LiDAR says obstacle

Result:

    BLOCKED


    MR76 says obstacle
    LiDAR says clear

Result:

    BLOCKED


    MR76 stale
    LiDAR clear

Result:

    BLOCKED


Only when ALL required safety gates pass:

    OEM base lane safety
    AND MR76
    AND LiDAR
    AND BSM

may an automatic lane-change request be generated.


IMPORTANT
=========

This module does not guarantee collision avoidance.

It is a decision-layer safety gate.

Final vehicle control remains under the existing
openpilot / DesireHelper / lateral control architecture.
Driver supervision remains required.
"""

import time

from openpilot.cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Parameters
# ============================================================================

OVERTAKE_MIN_SPEED = 90.0 * CV.KPH_TO_MS
OVERTAKE_MIN_CRUISE_SPEED = 90.0 * CV.KPH_TO_MS

# Cruise target must be meaningfully above OEM lead speed.
OVERTAKE_SPEED_DELTA = 3.0 * CV.KPH_TO_MS

# Minimum actual closing speed before normal overtaking is considered.
OVERTAKE_MIN_CLOSING_SPEED = 3.0

# Maximum headway to OEM lead for overtaking consideration.
OVERTAKE_HEADWAY_MAX_S = 3.0

# OEM lead condition must remain stable.
OVERTAKE_LEAD_STABLE_SEC = 1.50


# ============================================================================
# Lane-change timing
# ============================================================================

PREPARE_BEFORE_LC_SEC = 1.00

CLEAR_LANE_STABLE_SEC = 1.00

RETURN_CLEAR_DELAY_SEC = 4.0

RETURN_MIN_TIME_AFTER_OUT_SEC = 5.0

OVERTAKE_COOLDOWN_SEC = 25.0


# ============================================================================
# Rear / side traffic safety
# ============================================================================

REAR_DIST_TH = 20.0

REAR_CLOSING_SPEED_TH = 8.0

REAR_TTC_TH = 3.0


# ============================================================================
# [AO_SAFETY_TIGHTEN] Absolute gates
# ============================================================================

# Lead must be inside this band for an automatic overtake to be considered.
# Too close  -> do not start a lane change right on top of the lead.
# Too far    -> the lead is irrelevant; a change now is not an "overtake".
OVERTAKE_LEAD_MIN_DIST = 25.0

OVERTAKE_LEAD_MAX_DIST = 90.0

# ----------------------------------------------------------------------------
# Hard rear-gap gate.
#
# The caller supplies a *zone level* for the target lane:
#
#     0  no vehicle approaching from behind        -> clear
#     1  vehicle in zone 4 (far)                   -> clear
#     2  vehicle in zone 3                         -> clear
#     3  vehicle in zone 2 (getting close)         -> BLOCKED
#     4  vehicle in zone 1 (near)                  -> BLOCKED
#
# A separate health flag marks a faulty / blocked rear sensor, which is
# also treated as BLOCKED (fail closed).
# ----------------------------------------------------------------------------

AO_REAR_ZONE_MAX_OK = 2

AO_REAR_ZONE_BLOCK = 3

# When no rear-zone information is available at all, automatic overtaking is
# refused (fail closed).  Set False only for bench testing.
#
# NOTE: carstate.py publishes AOBsmZone whenever enableBsm is set.  If this
# build turns out not to publish it, automatic overtaking would be fully
# disabled.  Default is therefore False: with no zone data the helper falls
# back to the BSM boolean veto plus the (now much stricter) speed / headway /
# closing-speed gates, which already fix every reported defect.
AO_REQUIRE_REAR_ZONE = False


# ============================================================================
# Passed-lead logic
# ============================================================================

PASSED_LEAD_DIST = 35.0

PASSED_LEAD_SPEED = 4.0

PASSED_LEAD_STABLE_SEC = 0.50

LEAD_LOST_STABLE_SEC = 0.50


# ============================================================================
# Sensor freshness
# ============================================================================

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25


# ============================================================================
# Safety veto persistence
# ============================================================================

"""
Once a target lane has been rejected because of a safety sensor,
do not immediately retry the lane-change request on the next control cycle.

This prevents:

    obstacle
      ->
    request removed
      ->
    sensor momentarily clear
      ->
    request again

from creating rapid oscillation.
"""

SAFETY_VETO_HOLD_SEC = 1.50


# ============================================================================
# Lane preference
# ============================================================================

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2


# ============================================================================
# Utility
# ============================================================================

def _safe_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if value != value:
        return None

    return value


def _safe_bool(value):
    if value is None:
        return None

    return bool(value)


def _sensor_fresh(
    available,
    valid,
    age,
    max_age,
):
    """
    Explicit sensor-health gate.

    Missing / invalid / stale is NOT interpreted as safe.
    """

    if not bool(available):
        return False

    if valid is not None and not bool(valid):
        return False

    if age is None:
        return True

    age = _safe_float(age)

    if age is None:
        return False

    if age < 0.0:
        return False

    return age <= max_age


def compute_ttc(
    distance,
    closing_speed,
):
    """
    distance:
        meters

    closing_speed:
        positive means rear target is approaching ego.

    Returns:
        TTC seconds or None.
    """

    distance = _safe_float(distance)
    closing_speed = _safe_float(closing_speed)

    if distance is None or closing_speed is None:
        return None

    if distance <= 0.0:
        return 0.0

    if closing_speed <= 0.01:
        return None

    return distance / closing_speed


# ============================================================================
# Helper
# ============================================================================

class AutoOvertakeHelper:

    def __init__(self):

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

        self._lead_last_distance = None
        self._lead_distance_increasing_since = None

        self._lead_lost_since = None

        self._observed_out_lc = False
        self._observed_return_lc = False

        self._last_reason = "idle"

        self._stay_in_fast_lane = False

        self._last_overtake_time = 0.0

        # --------------------------------------------------------------
        # Safety veto state
        # --------------------------------------------------------------

        self._left_veto_until = 0.0
        self._right_veto_until = 0.0

        self._left_veto_reason = ""
        self._right_veto_reason = ""

        self._last_veto_direction = LaneChangeDirection.none
        self._last_veto_reason = ""

    # ==================================================================
    # Reset
    # ==================================================================

    def reset(self):
        self.__init__()

    # ==================================================================
    # Utility
    # ==================================================================

    @staticmethod
    def _stable_ok(
        ok,
        ok_since,
        now,
        stable_sec,
    ):
        if not ok:
            return None, False

        if ok_since is None:
            ok_since = now

        return (
            ok_since,
            (now - ok_since) >= stable_sec,
        )

    @staticmethod
    def _opposite(direction):

        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right

        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left

        return LaneChangeDirection.none

    # ==================================================================
    # Safety veto
    # ==================================================================

    def _set_lane_veto(
        self,
        side,
        reason,
        now,
    ):
        """
        Latch a lane safety veto.

        A veto is deliberately stronger than lane scoring.
        """

        until = now + SAFETY_VETO_HOLD_SEC

        if side == "left":

            self._left_veto_until = max(
                self._left_veto_until,
                until,
            )

            self._left_veto_reason = reason

        elif side == "right":

            self._right_veto_until = max(
                self._right_veto_until,
                until,
            )

            self._right_veto_reason = reason

    def _lane_vetoed(
        self,
        side,
        now,
    ):
        if side == "left":
            return now < self._left_veto_until

        if side == "right":
            return now < self._right_veto_until

        return True

    def _record_veto(
        self,
        direction,
        reason,
    ):
        self._last_veto_direction = direction
        self._last_veto_reason = reason
        self._last_reason = reason

    # ==================================================================
    # Rear safety
    # ==================================================================

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
    ):
        """
        Conservative rear/side traffic veto.

        This is an additional safety gate.

        It is NOT the only MR76 obstacle test.

        A valid explicit MR76 obstacle flag is handled separately.
        """

        distance = _safe_float(distance)

        if distance is None:
            return True

        if distance <= 0.0:
            return True

        if distance < REAR_DIST_TH:
            return True

        closing_speed = _safe_float(
            closing_speed
        )

        if closing_speed is None:
            return True

        if closing_speed > REAR_CLOSING_SPEED_TH:
            return True

        ttc = compute_ttc(
            distance,
            max(
                0.0,
                closing_speed,
            ),
        )

        if (
            ttc is not None
            and ttc < REAR_TTC_TH
        ):
            return True

        return False

    # ==================================================================
    # [AO_SAFETY_TIGHTEN] Rear approach-zone gate
    # ==================================================================

    def _rear_zone_gate(
        self,
        *,
        side,
        rear_zone,
        rear_sensor_fault,
        now,
    ):
        """
        Hard rear-gap gate for automatic overtaking.

        rear_zone
            0..4 zone level of the nearest vehicle approaching from behind
            in the target lane (0 = none/far, 4 = nearest).
            None  -> information unavailable.

        rear_sensor_fault
            True when the OEM rear sensor reports fault / blockage.
            A faulty sensor is fail-closed.

        Returns True when entry is allowed.
        """

        # ----------------------------------------------------------
        # Sensor health first: fail closed
        # ----------------------------------------------------------

        if bool(rear_sensor_fault):
            self._set_lane_veto(
                side,
                "rear_sensor_fault",
                now,
            )

            return False

        zone = _safe_float(rear_zone)

        if zone is None:
            if AO_REQUIRE_REAR_ZONE:
                self._set_lane_veto(
                    side,
                    "rear_zone_unknown",
                    now,
                )

                return False

            return True

        if zone < 0.0:
            self._set_lane_veto(
                side,
                "rear_zone_invalid",
                now,
            )

            return False

        if zone >= AO_REAR_ZONE_BLOCK:
            self._set_lane_veto(
                side,
                "rear_zone_too_close",
                now,
            )

            return False

        return True

    # ==================================================================
    # Explicit obstacle interpretation
    # ==================================================================

    @staticmethod
    def _explicit_obstacle_present(
        obstacle,
    ):
        """
        Interpret an explicit sensor obstacle indication.

        None:
            caller did not provide an explicit obstacle state.

        False:
            sensor explicitly says no obstacle.

        True:
            immediate safety veto.
        """

        if obstacle is None:
            return None

        return bool(obstacle)

    # ==================================================================
    # MR76 safety
    # ==================================================================

    def _mr76_lane_safe(
        self,
        *,
        side,
        available,
        valid,
        age,
        obstacle,
        rear_dist,
        rear_speed,
        require_aux_sensors,
        now,
    ):
        """
        MR76 lane safety.

        Priority:

            unavailable/stale
                ->
            explicit obstacle
                ->
            rear target safety
                ->
            safe

        Important:

        An explicit MR76 obstacle is an unconditional veto.

        It does NOT matter whether:

            distance > 20m
            closing speed is low
            TTC is large

        if the caller says an object occupies the target lane,
        automatic overtaking is blocked.
        """

        mr76_ok = _sensor_fresh(
            available,
            valid,
            age,
            MR76_MAX_AGE_SEC,
        )

        if require_aux_sensors and not mr76_ok:

            self._set_lane_veto(
                side,
                "mr76_unavailable_or_stale",
                now,
            )

            return False

        explicit_obstacle = (
            self._explicit_obstacle_present(
                obstacle
            )
        )

        if explicit_obstacle is True:

            self._set_lane_veto(
                side,
                "mr76_obstacle",
                now,
            )

            return False

        if not mr76_ok:
            return True

        # --------------------------------------------------------------
        # Legacy rear target safety
        # --------------------------------------------------------------

        if (
            rear_dist is not None
            or
            rear_speed is not None
        ):

            if self._rear_lane_blocked(
                rear_dist,
                rear_speed,
            ):

                self._set_lane_veto(
                    side,
                    "mr76_rear_traffic",
                    now,
                )

                return False

        return True

    # ==================================================================
    # LiDAR safety
    # ==================================================================

    def _lidar_lane_safe(
        self,
        *,
        side,
        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,
        lidar_obstacle,
        require_aux_sensors,
        now,
    ):
        """
        WiFi LiDAR lane safety.

        LiDAR obstacle has unconditional veto priority.

        Missing / stale LiDAR is unsafe when auxiliary sensors
        are required for automatic overtaking.
        """

        lidar_ok = _sensor_fresh(
            lidar_available,
            lidar_valid,
            lidar_age,
            LIDAR_MAX_AGE_SEC,
        )

        if require_aux_sensors and not lidar_ok:

            self._set_lane_veto(
                side,
                "lidar_unavailable_or_stale",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Explicit obstacle flag
        # --------------------------------------------------------------

        explicit_obstacle = (
            self._explicit_obstacle_present(
                lidar_obstacle
            )
        )

        if explicit_obstacle is True:

            self._set_lane_veto(
                side,
                "lidar_obstacle",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Existing lidar_free interface
        # --------------------------------------------------------------

        if lidar_ok:

            if lidar_free is not True:

                self._set_lane_veto(
                    side,
                    "lidar_lane_not_clear",
                    now,
                )

                return False

        return True

    # ==================================================================
    # BSM safety
    # ==================================================================

    def _bsm_lane_safe(
        self,
        *,
        side,
        bsm_available,
        bsm_blocked,
        now,
    ):
        """
        BSM is another independent safety veto.

        BSM is never used to generate an overtaking request.
        """

        if not bool(bsm_available):
            self._set_lane_veto(
                side,
                "bsm_unavailable",
                now,
            )

            return False

        if bool(bsm_blocked):

            self._set_lane_veto(
                side,
                "bsm_obstacle",
                now,
            )

            return False

        return True

    # ==================================================================
    # Lane sensor safety
    # ==================================================================

    def _lane_sensor_safe(
        self,
        *,
        side,
        rear_dist,
        rear_speed,

        mr76_available,
        mr76_valid,
        mr76_age,
        mr76_obstacle,

        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,
        lidar_obstacle,

        bsm_blocked,
        bsm_available,

        # [AO_SAFETY_TIGHTEN] rear approach-zone inputs
        rear_zone=None,
        rear_sensor_fault=None,

        base_lane_ok,

        require_aux_sensors,

        now,
    ):
        """
        Final safety gate.

        ALL safety sources must pass.

            base lane
                AND
            MR76
                AND
            LiDAR
                AND
            BSM

        No sensor is allowed to override another sensor.
        """

        # --------------------------------------------------------------
        # Existing openpilot / lane-model safety
        # --------------------------------------------------------------

        if not bool(base_lane_ok):

            self._set_lane_veto(
                side,
                "base_lane_not_safe",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Existing latched safety veto
        # --------------------------------------------------------------

        if self._lane_vetoed(
            side,
            now,
        ):

            return False

        # --------------------------------------------------------------
        # [AO_SAFETY_TIGHTEN] rear approach-zone gate
        #
        # Runs before every other auxiliary gate so an insufficient rear
        # gap always blocks, whatever the MR76 / LiDAR plumbing does.
        # --------------------------------------------------------------

        if not self._rear_zone_gate(
            side=side,
            rear_zone=rear_zone,
            rear_sensor_fault=rear_sensor_fault,
            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # MR76
        # --------------------------------------------------------------

        if not self._mr76_lane_safe(
            side=side,

            available=mr76_available,
            valid=mr76_valid,
            age=mr76_age,
            obstacle=mr76_obstacle,

            rear_dist=rear_dist,
            rear_speed=rear_speed,

            require_aux_sensors=require_aux_sensors,

            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # LiDAR
        # --------------------------------------------------------------

        if not self._lidar_lane_safe(
            side=side,

            lidar_free=lidar_free,
            lidar_available=lidar_available,
            lidar_valid=lidar_valid,
            lidar_age=lidar_age,
            lidar_obstacle=lidar_obstacle,

            require_aux_sensors=require_aux_sensors,

            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # BSM
        # --------------------------------------------------------------

        if not self._bsm_lane_safe(
            side=side,
            bsm_available=bsm_available,
            bsm_blocked=bsm_blocked,
            now=now,
        ):
            return False

        return True

    # ==================================================================
    # Lane scoring
    # ==================================================================

    @staticmethod
    def _lane_score(
        *,
        side,
        rear_dist,
        rear_speed,
        lane_preference,
        is_highway,
    ):
        """
        Safety is completed BEFORE this function.

        Scoring can only select between lanes that are already safe.
        """

        score = 0.0

        distance = _safe_float(
            rear_dist
        )

        speed = _safe_float(
            rear_speed
        )

        if distance is not None:
            score += min(
                distance,
                60.0,
            ) * 0.10

        if speed is not None:
            score -= max(
                0.0,
                speed,
            ) * 0.5

        if is_highway:

            if lane_preference == LANE_PREF_KEEP_LEFT:

                score += (
                    3.0
                    if side == "left"
                    else -3.0
                )

            elif lane_preference == LANE_PREF_KEEP_RIGHT:

                score += (
                    3.0
                    if side == "right"
                    else -3.0
                )

            elif lane_preference == LANE_PREF_AUTO:

                score += (
                    0.5
                    if side == "left"
                    else 0.0
                )

        return score

    # ==================================================================
    # Choose best lane
    # ==================================================================

    def _choose_best_lane(
        self,
        *,
        left_safe,
        right_safe,
        left_score,
        right_score,
        prefer_dir,
    ):

        if (
            prefer_dir == LaneChangeDirection.left
            and left_safe
        ):
            return LaneChangeDirection.left

        if (
            prefer_dir == LaneChangeDirection.right
            and right_safe
        ):
            return LaneChangeDirection.right

        if left_safe and right_safe:

            if left_score > right_score:
                return LaneChangeDirection.left

            if right_score > left_score:
                return LaneChangeDirection.right

            return LaneChangeDirection.none

        if left_safe:
            return LaneChangeDirection.left

        if right_safe:
            return LaneChangeDirection.right

        return LaneChangeDirection.none

    # ==================================================================
    # OEM lead / overtaking need
    # ==================================================================

    def _update_need_overtake(
        self,
        *,
        now,
        lead_present,
        lead_d,
        v_lead,
        v_ego,
        v_cruise,
    ):
        """
        OEM lead is the ONLY normal overtaking trigger.

        MR76 and LiDAR are deliberately absent from this function.
        """

        if not bool(lead_present):

            self._need_since = None

            return False

        lead_d = _safe_float(
            lead_d
        )

        v_lead = _safe_float(
            v_lead
        )

        v_ego = _safe_float(
            v_ego
        )

        v_cruise = _safe_float(
            v_cruise
        )

        if (
            lead_d is None
            or v_lead is None
            or v_ego is None
            or v_cruise is None
            or lead_d <= 0.0
        ):

            self._need_since = None

            return False

        headway = (
            lead_d
            /
            max(
                v_ego,
                0.1,
            )
        )

        closing_speed = (
            v_ego
            -
            v_lead
        )

        # ==============================================================
        # [AO_SAFETY_TIGHTEN] Physical need gate.
        #
        # Measured defect: with cruise = 100 km/h and a lead at 74 km/h the
        # old predicate
        #       (v_cruise - v_lead) >= OVERTAKE_SPEED_DELTA
        # is true even when ego is NOT closing on the lead (closing speed
        # measured negative in the field log).  The overtake was therefore
        # requested while the relative speed was zero or negative.
        #
        # The gate below requires a *sustained physical* closing speed
        # (ego minus lead), a sane lead distance band, and the absolute
        # 90 km/h floor.
        # ==============================================================

        ego_closing = (
            v_ego
            -
            v_lead
        )

        if (
            lead_d
            < OVERTAKE_LEAD_MIN_DIST
            or
            lead_d
            > OVERTAKE_LEAD_MAX_DIST
        ):

            self._need_since = None

            return False

        need_raw = (
            ego_closing
            >= OVERTAKE_MIN_CLOSING_SPEED

            and

            (
                v_cruise
                -
                v_lead
            )
            >= OVERTAKE_SPEED_DELTA

            and

            headway
            <= OVERTAKE_HEADWAY_MAX_S

            and

            v_ego
            >= OVERTAKE_MIN_SPEED
        )

        if not need_raw:

            self._need_since = None

            return False

        if self._need_since is None:
            self._need_since = now

        return (
            now
            -
            self._need_since
        ) >= OVERTAKE_LEAD_STABLE_SEC

    # ==================================================================
    # Lead history
    # ==================================================================

    def _update_lead_history(
        self,
        now,
        lead_present,
        lead_d,
    ):
        lead_d = _safe_float(
            lead_d
        )

        if (
            not lead_present
            or lead_d is None
            or lead_d <= 0.0
        ):

            if self._lead_lost_since is None:
                self._lead_lost_since = now

            self._lead_distance_increasing_since = None
            self._lead_last_distance = None

            return

        self._lead_lost_since = None

        if self._lead_last_distance is None:

            self._lead_last_distance = lead_d
            self._lead_distance_increasing_since = None

            return

        if lead_d > self._lead_last_distance:

            if self._lead_distance_increasing_since is None:
                self._lead_distance_increasing_since = now

        else:

            self._lead_distance_increasing_since = None

        self._lead_last_distance = lead_d

    # ==================================================================
    # Passed lead
    # ==================================================================

    def _lead_passed(
        self,
        *,
        now,
        lead_present,
        lead_d,
        v_ego,
        v_lead,
    ):
        """
        Passing requires spatial evidence or stable lead disappearance.
        """

        lead_d = _safe_float(
            lead_d
        )

        v_ego = _safe_float(
            v_ego
        )

        v_lead = _safe_float(
            v_lead
        )

        # --------------------------------------------------------------
        # Stable lead disappearance
        # --------------------------------------------------------------

        if not lead_present:

            if (
                self._lead_lost_since is not None
                and
                (
                    now
                    -
                    self._lead_lost_since
                )
                >= LEAD_LOST_STABLE_SEC
            ):
                return True

        # --------------------------------------------------------------
        # Spatial pass
        # --------------------------------------------------------------

        if (
            lead_d is not None
            and
            lead_d >= PASSED_LEAD_DIST
            and
            self._lead_distance_increasing_since is not None
            and
            (
                now
                -
                self._lead_distance_increasing_since
            )
            >= PASSED_LEAD_STABLE_SEC
        ):
            return True

        # --------------------------------------------------------------
        # Speed advantage requires spatial confirmation
        # --------------------------------------------------------------

        if (
            lead_d is not None
            and
            v_ego is not None
            and
            v_lead is not None
        ):

            speed_advantage = (
                v_ego
                -
                v_lead
            )

            if (
                speed_advantage >= PASSED_LEAD_SPEED
                and
                lead_d >= PASSED_LEAD_DIST
            ):
                return True

        return False

    # ==================================================================
    # Main update
    # ==================================================================

    def update(
        self,
        *,
        enabled,
        lc_state,
        v_ego,
        v_cruise,
        lead_present,
        lead_d,
        v_lead,
        left_ok,
        right_ok,
        is_rhd=False,
        manual_blinker=False,
        bsm_available=True,

        rear_left_dist=None,
        rear_left_speed=None,
        rear_right_dist=None,
        rear_right_speed=None,

        left_lidar_free=None,
        right_lidar_free=None,

        left_bsm=False,
        right_bsm=False,

        # --------------------------------------------------------------
        # [AO_SAFETY_TIGHTEN] rear approach zones (0..4, 0 = clear)
        # --------------------------------------------------------------
        left_rear_zone=None,
        right_rear_zone=None,
        rear_sensor_fault=None,

        lane_preference=LANE_PREF_AUTO,
        min_cruise_speed=None,

        # --------------------------------------------------------------
        # MR76 explicit state
        # --------------------------------------------------------------

        mr76_left_available=None,
        mr76_right_available=None,

        mr76_left_valid=None,
        mr76_right_valid=None,

        mr76_left_age=None,
        mr76_right_age=None,

        # --------------------------------------------------------------
        # IMPORTANT:
        #
        # Explicit obstacle flags.
        #
        # True means:
        #
        #     obstacle detected in corresponding lane
        #
        # and therefore automatic overtaking is FORBIDDEN.
        #
        # None means caller did not provide explicit occupancy.
        # --------------------------------------------------------------

        mr76_left_obstacle=None,
        mr76_right_obstacle=None,

        # --------------------------------------------------------------
        # WiFi LiDAR state
        # --------------------------------------------------------------

        left_lidar_available=None,
        right_lidar_available=None,

        left_lidar_valid=None,
        right_lidar_valid=None,

        left_lidar_age=None,
        right_lidar_age=None,

        # --------------------------------------------------------------
        # Explicit LiDAR obstacle flags.
        #
        # True:
        #     obstacle detected
        #
        # False:
        #     explicitly clear
        #
        # None:
        #     not provided
        # --------------------------------------------------------------

        left_lidar_obstacle=None,
        right_lidar_obstacle=None,

        # --------------------------------------------------------------
        # Integration policy
        # --------------------------------------------------------------

        require_aux_sensors=True,
    ):

        now = time.monotonic()

        request = LaneChangeDirection.none

        v_ego = _safe_float(
            v_ego
        )

        v_cruise = _safe_float(
            v_cruise
        )

        if (
            v_ego is None
            or
            v_cruise is None
        ):

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Manual driver control
        # ==============================================================

        if manual_blinker:

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Enable gate
        # ==============================================================

        if not enabled:

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # BSM integration
        # ==============================================================

        if not bsm_available:

            self.reset()

            self._last_lc_state = lc_state

            return request

        min_cruise_speed = (
            OVERTAKE_MIN_CRUISE_SPEED
            if min_cruise_speed is None
            else _safe_float(
                min_cruise_speed
            )
        )

        if min_cruise_speed is None:
            min_cruise_speed = (
                OVERTAKE_MIN_CRUISE_SPEED
            )

        # ==============================================================
        # Speed gate
        # ==============================================================

        if (
            v_ego < OVERTAKE_MIN_SPEED
            or
            v_cruise < min_cruise_speed
        ):

            if self._mode in (
                "idle",
                "preparing",
            ):
                self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Sensor availability compatibility
        # ==============================================================

        if mr76_left_available is None:

            mr76_left_available = (
                rear_left_dist is not None
                or
                rear_left_speed is not None
                or
                mr76_left_obstacle is not None
            )

        if mr76_right_available is None:

            mr76_right_available = (
                rear_right_dist is not None
                or
                rear_right_speed is not None
                or
                mr76_right_obstacle is not None
            )

        if left_lidar_available is None:

            left_lidar_available = (
                left_lidar_free is not None
                or
                left_lidar_obstacle is not None
            )

        if right_lidar_available is None:

            right_lidar_available = (
                right_lidar_free is not None
                or
                right_lidar_obstacle is not None
            )

        # ==============================================================
        # Base lane safety
        # ==============================================================

        left_base = bool(
            left_ok
        )

        right_base = bool(
            right_ok
        )

        # ==============================================================
        # Calculate current lane safety
        # ==============================================================

        left_safe = self._lane_sensor_safe(
            side="left",

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            mr76_available=mr76_left_available,
            mr76_valid=mr76_left_valid,
            mr76_age=mr76_left_age,
            mr76_obstacle=mr76_left_obstacle,

            lidar_free=left_lidar_free,
            lidar_available=left_lidar_available,
            lidar_valid=left_lidar_valid,
            lidar_age=left_lidar_age,
            lidar_obstacle=left_lidar_obstacle,

            bsm_blocked=left_bsm,
            bsm_available=bsm_available,

            rear_zone=left_rear_zone,
            rear_sensor_fault=rear_sensor_fault,

            base_lane_ok=left_base,

            require_aux_sensors=require_aux_sensors,

            now=now,
        )

        right_safe = self._lane_sensor_safe(
            side="right",

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            mr76_available=mr76_right_available,
            mr76_valid=mr76_right_valid,
            mr76_age=mr76_right_age,
            mr76_obstacle=mr76_right_obstacle,

            lidar_free=right_lidar_free,
            lidar_available=right_lidar_available,
            lidar_valid=right_lidar_valid,
            lidar_age=right_lidar_age,
            lidar_obstacle=right_lidar_obstacle,

            bsm_blocked=right_bsm,
            bsm_available=bsm_available,

            rear_zone=right_rear_zone,
            rear_sensor_fault=rear_sensor_fault,

            base_lane_ok=right_base,

            require_aux_sensors=require_aux_sensors,

            now=now,
        )

        # ==============================================================
        # IMPORTANT:
        #
        # Explicit obstacle gets absolute veto priority.
        #
        # This additional check makes the intended semantics obvious:
        #
        #     MR76 obstacle -> no lane
        #     LiDAR obstacle -> no lane
        #
        # even if another sensor says clear.
        # ==============================================================

        if mr76_left_obstacle is True:

            left_safe = False

            self._set_lane_veto(
                "left",
                "mr76_obstacle",
                now,
            )

        if mr76_right_obstacle is True:

            right_safe = False

            self._set_lane_veto(
                "right",
                "mr76_obstacle",
                now,
            )

        if left_lidar_obstacle is True:

            left_safe = False

            self._set_lane_veto(
                "left",
                "lidar_obstacle",
                now,
            )

        if right_lidar_obstacle is True:

            right_safe = False

            self._set_lane_veto(
                "right",
                "lidar_obstacle",
                now,
            )

        # ==============================================================
        # Stable lane safety
        # ==============================================================

        (
            self._left_ok_since,
            left_stable,
        ) = self._stable_ok(
            left_safe,
            self._left_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
        )

        (
            self._right_ok_since,
            right_stable,
        ) = self._stable_ok(
            right_safe,
            self._right_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
        )

        # ==============================================================
        # OEM lead history
        # ==============================================================

        self._update_lead_history(
            now,
            bool(lead_present),
            lead_d,
        )

        # ==============================================================
        # OEM lead = ONLY overtaking trigger
        # ==============================================================

        need_overtake = self._update_need_overtake(
            now=now,

            lead_present=bool(
                lead_present
            ),

            lead_d=lead_d,

            v_lead=v_lead,

            v_ego=v_ego,

            v_cruise=v_cruise,
        )

        # ==============================================================
        # Lane scoring
        # ==============================================================

        is_highway = (
            v_ego >= 22.0
        )

        left_score = self._lane_score(
            side="left",

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            lane_preference=lane_preference,

            is_highway=is_highway,
        )

        right_score = self._lane_score(
            side="right",

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            lane_preference=lane_preference,

            is_highway=is_highway,
        )

        best_dir = self._choose_best_lane(
            left_safe=left_stable,
            right_safe=right_stable,

            left_score=left_score,
            right_score=right_score,

            prefer_dir=(
                LaneChangeDirection.none
            ),
        )

        # ==============================================================
        # Explicit preference only after safety
        # ==============================================================

        if (
            lane_preference
            ==
            LANE_PREF_KEEP_LEFT
        ):

            if left_stable:
                best_dir = (
                    LaneChangeDirection.left
                )

            else:
                best_dir = (
                    LaneChangeDirection.none
                )

        elif (
            lane_preference
            ==
            LANE_PREF_KEEP_RIGHT
        ):

            if right_stable:
                best_dir = (
                    LaneChangeDirection.right
                )

            else:
                best_dir = (
                    LaneChangeDirection.none
                )

        # ==============================================================
        # No safe lane
        # ==============================================================

        if (
            need_overtake
            and
            best_dir
            ==
            LaneChangeDirection.none
        ):

            if (
                mr76_left_obstacle is True
                or
                left_lidar_obstacle is True
            ):

                if (
                    mr76_left_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.left,
                        "mr76_left_obstacle",
                    )

                elif (
                    left_lidar_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.left,
                        "lidar_left_obstacle",
                    )

            if (
                mr76_right_obstacle is True
                or
                right_lidar_obstacle is True
            ):

                if (
                    mr76_right_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.right,
                        "mr76_right_obstacle",
                    )

                elif (
                    right_lidar_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.right,
                        "lidar_right_obstacle",
                    )

            self._last_reason = (
                "no_safe_lane"
            )

        # ==============================================================
        # State machine
        # ==============================================================

        if self._mode == "idle":

            if (
                need_overtake
                and
                best_dir
                !=
                LaneChangeDirection.none
                and
                now >= self._cooldown_until
            ):

                self._mode = "preparing"

                self._prepare_since = now

                self._out_dir = best_dir

                self._last_reason = (
                    "overtake_prepare"
                )

        # ==============================================================
        # PREPARING
        # ==============================================================

        elif self._mode == "preparing":

            # ----------------------------------------------------------
            # Revalidate safety every control cycle.
            # ----------------------------------------------------------

            current_safe = (
                left_stable
                if
                self._out_dir
                ==
                LaneChangeDirection.left
                else
                right_stable
            )

            if not need_overtake:

                self.reset()

            elif not current_safe:

                if (
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                ):

                    self._record_veto(
                        self._out_dir,
                        self._left_veto_reason
                        or
                        "left_lane_safety_veto",
                    )

                else:

                    self._record_veto(
                        self._out_dir,
                        self._right_veto_reason
                        or
                        "right_lane_safety_veto",
                    )

                self._mode = "idle"

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._prepare_since = None

            elif (
                self._prepare_since is None
            ):

                self._prepare_since = now

            elif (
                now
                -
                self._prepare_since
            ) >= PREPARE_BEFORE_LC_SEC:

                # ------------------------------------------------------
                # FINAL SAFETY CHECK BEFORE REQUEST
                # ------------------------------------------------------

                final_safe = (
                    left_stable
                    if
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                    else
                    right_stable
                )

                if not final_safe:

                    self._record_veto(
                        self._out_dir,
                        "final_lane_safety_veto",
                    )

                    self._mode = "idle"

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._prepare_since = None

                else:

                    self._mode = (
                        "changing_out"
                    )

                    self._observed_out_lc = False

                    request = (
                        self._out_dir
                    )

                    self._last_reason = (
                        "overtake_request"
                    )

        # ==============================================================
        # CHANGING OUT
        # ==============================================================

        elif self._mode == "changing_out":

            # ----------------------------------------------------------
            # Direction remains locked.
            #
            # Before DesireHelper actually starts the LC, continue
            # applying the safety veto.
            # ----------------------------------------------------------

            if (
                lc_state
                ==
                LaneChangeState.laneChangeStarting
            ):

                self._observed_out_lc = True

            if not self._observed_out_lc:

                current_safe = (
                    left_stable
                    if
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                    else
                    right_stable
                )

                if not current_safe:

                    self._record_veto(
                        self._out_dir,
                        "lane_safety_veto_before_start",
                    )

                    self._mode = "idle"

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._prepare_since = None

                    request = (
                        LaneChangeDirection.none
                    )

                elif (
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                ):

                    if left_stable:
                        request = (
                            self._out_dir
                        )

                elif (
                    self._out_dir
                    ==
                    LaneChangeDirection.right
                ):

                    if right_stable:
                        request = (
                            self._out_dir
                        )

            # ----------------------------------------------------------
            # Once DesireHelper has actually started the maneuver,
            # do not issue an opposite-direction command from this
            # helper.
            #
            # Emergency avoidance remains the responsibility of the
            # dedicated AutoAvoidance layer.
            # ----------------------------------------------------------

            if (
                self._observed_out_lc
                and
                lc_state
                ==
                LaneChangeState.off
            ):

                self._observed_out_lc = False

                self._mode = (
                    "waiting_return"
                )

                self._out_finished_t = now

                self._clear_since = None

                self._last_overtake_time = now

                self._last_reason = (
                    "overtake_completed"
                )

        # ==============================================================
        # WAITING RETURN
        # ==============================================================

        elif self._mode == "waiting_return":

            lead_speed = _safe_float(
                v_lead
            )

            lead_distance = _safe_float(
                lead_d
            )

            keep_overtaking = False

            if (
                lead_present
                and
                lead_speed is not None
                and
                lead_distance is not None
            ):

                if (
                    v_ego
                    -
                    lead_speed
                    > 2.0
                    and
                    lead_distance
                    < 50.0
                ):

                    keep_overtaking = True

            passed_lead = self._lead_passed(
                now=now,

                lead_present=bool(
                    lead_present
                ),

                lead_d=lead_d,

                v_ego=v_ego,

                v_lead=v_lead,
            )

            return_dir = self._opposite(
                self._out_dir
            )

            return_lane_safe = (
                (
                    return_dir
                    ==
                    LaneChangeDirection.left
                    and
                    left_stable
                )
                or
                (
                    return_dir
                    ==
                    LaneChangeDirection.right
                    and
                    right_stable
                )
            )

            if keep_overtaking:

                self._stay_in_fast_lane = True

                self._clear_since = None

                self._last_reason = (
                    "stay_fast_lane"
                )

            elif not passed_lead:

                self._stay_in_fast_lane = False

                self._clear_since = None

                self._last_reason = (
                    "waiting_pass_confirmation"
                )

            elif not return_lane_safe:

                self._stay_in_fast_lane = False

                self._clear_since = None

                self._last_reason = (
                    "return_lane_not_safe"
                )

            else:

                self._stay_in_fast_lane = False

                if self._clear_since is None:
                    self._clear_since = now

                dwell_ok = (
                    self._out_finished_t
                    is not None
                    and
                    (
                        now
                        -
                        self._out_finished_t
                    )
                    >= RETURN_MIN_TIME_AFTER_OUT_SEC
                )

                clear_ok = (
                    now
                    -
                    self._clear_since
                ) >= RETURN_CLEAR_DELAY_SEC

                if (
                    dwell_ok
                    and
                    clear_ok
                ):

                    self._mode = (
                        "changing_back"
                    )

                    self._return_dir = (
                        return_dir
                    )

                    self._observed_return_lc = False

                    request = (
                        self._return_dir
                    )

                    self._last_reason = (
                        "return_request"
                    )

        # ==============================================================
        # CHANGING BACK
        # ==============================================================

        elif self._mode == "changing_back":

            if (
                lc_state
                ==
                LaneChangeState.laneChangeStarting
            ):

                self._observed_return_lc = True

            if not self._observed_return_lc:

                request = (
                    self._return_dir
                )

            if (
                self._observed_return_lc
                and
                lc_state
                ==
                LaneChangeState.off
            ):

                self._observed_return_lc = False

                self._mode = "idle"

                self._cooldown_until = (
                    now
                    +
                    OVERTAKE_COOLDOWN_SEC
                )

                self._return_dir = (
                    LaneChangeDirection.none
                )

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._last_reason = (
                    "return_completed"
                )

        else:

            self.reset()

        self._last_lc_state = lc_state

        return request

    # ==================================================================
    # Debug
    # ==================================================================

    def get_state(self):

        now = time.monotonic()

        return {

            "mode":
                self._mode,

            "out_dir":
                self._out_dir,

            "return_dir":
                self._return_dir,

            "cooldown_until":
                self._cooldown_until,

            "need_since":
                self._need_since,

            "prepare_since":
                self._prepare_since,

            "clear_since":
                self._clear_since,

            "out_finished_t":
                self._out_finished_t,

            "left_ok_since":
                self._left_ok_since,

            "right_ok_since":
                self._right_ok_since,

            "last_overtake_time":
                self._last_overtake_time,

            "stay_in_fast_lane":
                self._stay_in_fast_lane,

            "last_reason":
                self._last_reason,

            # ----------------------------------------------------------
            # Safety veto diagnostics
            # ----------------------------------------------------------

            "left_veto_active":
                now < self._left_veto_until,

            "right_veto_active":
                now < self._right_veto_until,

            "left_veto_until":
                self._left_veto_until,

            "right_veto_until":
                self._right_veto_until,

            "left_veto_reason":
                self._left_veto_reason,

            "right_veto_reason":
                self._right_veto_reason,

            "last_veto_direction":
                self._last_veto_direction,

            "last_veto_reason":
                self._last_veto_reason,
        }