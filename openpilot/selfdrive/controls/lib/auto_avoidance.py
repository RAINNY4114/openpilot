#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto Avoidance Helper
=====================

EMERGENCY LATERAL AVOIDANCE ONLY.

This module exists ONLY for genuine imminent collision avoidance.

It does NOT perform:

    - normal overtaking
    - normal lane change
    - cruise control
    - longitudinal planning
    - braking control
    - throttle control
    - radar lead control
    - direct steering actuator control
    - CAN transmission

MR76 is AUXILIARY ONLY.

MR76 MUST NEVER:

    - become leadOne
    - become leadTwo
    - replace OEM radar
    - modify OEM lead distance
    - modify OEM lead relative speed
    - modify longitudinalPlan
    - modify aTarget
    - modify shouldStop
    - generate brake command
    - directly command CAN
    - publish radarState


Emergency architecture
----------------------

OEM radar / filtered obstacle
              |
              v
      Emergency risk
              |
              v
    AutoAvoidanceHelper
              |
              +---- LaneChangeDirection
              |
              +---- hazard
              |
              +---- lane_offset diagnostic only
              |
              +---- brake_request = ALWAYS 0.0
              |
              v
         DesireHelper


MR76 + side LiDAR
-----------------

MR76 and side WiFi LiDAR are NOT longitudinal sensors here.

They are used to determine whether an escape lane is safe.

For emergency automatic avoidance:

    missing
    invalid
    stale

auxiliary sensor data is NOT treated as "clear".

The destination lane must have sufficient independent sensor
confirmation before an automatic escape request is generated.


Direction lock
--------------

Once an actual emergency lane change has started:

    - direction is locked
    - temporary sensor loss does not cause an opposite-direction command
    - the module never oscillates left/right

This module is a decision layer, not an actuator.
"""

import time

from openpilot.cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Parameters
# ============================================================================

AUTO_AVOID_MIN_SPEED = (
    25.0 * CV.KPH_TO_MS
)


# --------------------------------------------------------------------------
# Emergency risk
# --------------------------------------------------------------------------

TTC_WARNING = 5.0
TTC_URGENT = 3.5
TTC_CRITICAL = 2.0

DIST_WARNING = 30.0
DIST_URGENT = 20.0
DIST_CRITICAL = 12.0

CLOSING_WARNING = 2.0
CLOSING_URGENT = 4.0
CLOSING_CRITICAL = 7.0


# --------------------------------------------------------------------------
# Confirmation
# --------------------------------------------------------------------------

CRITICAL_CONFIRM_FRAMES = 3

CLEAR_LANE_STABLE_SEC = 0.30

MIN_ACTIVE_SEC = 0.40

AVOID_COOLDOWN_SEC = 1.5


# --------------------------------------------------------------------------
# Rear / side traffic
# --------------------------------------------------------------------------

REAR_DIST_TH = 8.0

REAR_CLOSING_SPEED_TH = 15.0

REAR_SECONDARY_DIST_TH = 15.0

REAR_SECONDARY_CLOSING_SPEED_TH = 10.0


# --------------------------------------------------------------------------
# Sensor freshness
# --------------------------------------------------------------------------

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25

# ---------------------------------------------------------------------------
# Auxiliary-sensor gating
#
# MR76 is physically present and healthy on this car; the side WiFi LiDAR is
# installed but NOT wired into openpilot, so it never publishes and must not be
# allowed to veto by its own absence.
#
#   require_aux_sensors (the historical master flag) is kept, but it now defers
#   to the two per-sensor flags below.  A caller that leaves everything at its
#   default gets: MR76 required, LiDAR optional.
#
# "LiDAR optional" means: if LiDAR data IS present it can still block an escape
# lane (fail-safe), it just cannot block by being absent.
# ---------------------------------------------------------------------------
AVOID_REQUIRE_MR76_DEFAULT = True
AVOID_REQUIRE_LIDAR_DEFAULT = False

# ---------------------------------------------------------------------------
# Escape-lane stability policy
#
# "strict"  : lane must be OK continuously for `stable_sec` (historical).
# "ratio"   : over a sliding window of `window_sec`, the fraction of OK frames
#             must be >= `ratio`.  Tolerates the BSM flicker seen in traffic.
#
# Default "strict" == unchanged behaviour.
# ---------------------------------------------------------------------------
AVOID_STABLE_MODE_DEFAULT = "strict"
AVOID_STABLE_RATIO_DEFAULT = 0.60
AVOID_STABLE_WINDOW_SEC_DEFAULT = 0.60

# Number of consecutive frames a sensor must report "blocked" before the
# blocker is honoured.  0 == off (historical behaviour).
AVOID_BSM_DEBOUNCE_DEFAULT = 0

# Whether the caller has any rear-side sensing at all.  When 0, the
# rear-distance gate cannot block (there is simply nothing to report), and BSM
# remains the side/rear veto.
AVOID_REAR_PRESENT_DEFAULT = 1

# ---------------------------------------------------------------------------
# Risk-input hardening
#
# A single-frame leadOne spike must not be able to start an emergency lane
# change.  These two knobs are OFF by default; turning them on only ever
# REDUCES the number of triggers.
#
#   avoid_lead_lpf_alpha : first-order IIR coefficient for (dRel, vRel).
#                          0.0 == disabled (raw values, historical).
#                          0.35 == ~3-frame time constant at 100 Hz.
#   avoid_urgent_confirm : consecutive URGENT frames required before the state
#                          machine may act.  0 == historical (immediate).
# ---------------------------------------------------------------------------
AVOID_LEAD_LPF_ALPHA_DEFAULT = 0.0
AVOID_URGENT_CONFIRM_DEFAULT = 0


# --------------------------------------------------------------------------
# Diagnostic lateral offset
# --------------------------------------------------------------------------

EMERGENCY_OFFSET = 0.8


# --------------------------------------------------------------------------
# Risk levels
# --------------------------------------------------------------------------

RISK_NORMAL = 0
RISK_WARNING = 1
RISK_URGENT = 2
RISK_CRITICAL = 3


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


def _sensor_fresh(
    available,
    valid,
    age,
    max_age,
):
    """
    Explicit sensor health check.

    False / None / stale is NOT safe.
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
    relative_speed,
):
    """
    relative_speed convention:

        positive:
            target moving away / less closing

        negative:
            target approaching ego

    Returns:
        TTC seconds, or None if not closing.
    """

    distance = _safe_float(distance)
    relative_speed = _safe_float(relative_speed)

    if distance is None or relative_speed is None:
        return None

    if distance <= 0.0:
        return 0.0

    closing_speed = -relative_speed

    if closing_speed <= 0.01:
        return None

    return distance / closing_speed


def generate_smooth_evasive_path(
    current_path_len,
    target_offset,
):
    """
    Diagnostic/path utility only.

    It does NOT directly command steering.
    """

    try:
        current_path_len = int(
            current_path_len
        )
    except (TypeError, ValueError):
        return []

    if current_path_len <= 0:
        return []

    try:
        target_offset = float(
            target_offset
        )
    except (TypeError, ValueError):
        target_offset = 0.0

    if current_path_len == 1:
        return [target_offset]

    path = []

    for i in range(current_path_len):
        t = i / float(
            current_path_len - 1
        )

        y = target_offset * (
            3.0 * t * t
            - 2.0 * t * t * t
        )

        path.append(y)

    return path


# ============================================================================
# Helper
# ============================================================================

class AutoAvoidanceHelper:

    def __init__(self):

        self._mode = "idle"

        self._out_dir = (
            LaneChangeDirection.none
        )

        self._cooldown_until = 0.0

        self._clear_since = None

        self._active_since = None

        self._emergency_since = None

        self._last_lc_state = (
            LaneChangeState.off
        )

        self._observed_lane_change = False

        self._left_ok_since = None
        self._right_ok_since = None

        self._critical_frames = 0

        # Risk-input hardening state (see AVOID_LEAD_LPF_ALPHA_DEFAULT).
        self._lead_lpf_d = None
        self._lead_lpf_v = None
        self._urgent_frames = 0

        self._risk_level = RISK_NORMAL

        self._last_reason = "idle"

    # ==================================================================
    # Reset
    # ==================================================================

    def reset(self):
        self.__init__()

    def _reset_stability_histories(self):
        self._left_ok_hist = []
        self._right_ok_hist = []
        self._bsm_l_run = 0
        self._bsm_r_run = 0

    def _reset_risk_filters(self):
        self._lead_lpf_d = None
        self._lead_lpf_v = None
        self._urgent_frames = 0

    # ==================================================================
    # Stable safety
    # ==================================================================

    @staticmethod
    def _stable_ok(
        ok,
        ok_since,
        now,
        stable_sec,
        history=None,
        mode=None,
        ratio=None,
        window_sec=None,
    ):
        """Escape-lane stabilisation.

        mode "strict" (default, historical): require `ok` to hold continuously
        for `stable_sec`.  Any False frame resets the timer.

        mode "ratio": keep a short history of (timestamp, ok) and require that
        at least `ratio` of the samples inside the last `window_sec` seconds are
        OK.  This tolerates sensor flicker while still rejecting a lane that is
        genuinely occupied.  `ok_since` is still maintained for reporting.
        """

        mode = AVOID_STABLE_MODE_DEFAULT if mode is None else str(mode)

        # ---------------- historical path ----------------

        if mode != "ratio" or history is None:
            if not ok:
                return None, False

            if ok_since is None:
                ok_since = now

            return (
                ok_since,
                (now - ok_since) >= stable_sec,
            )

        # ---------------- sliding-window ratio path ----------------

        if ratio is None:
            ratio = AVOID_STABLE_RATIO_DEFAULT

        if window_sec is None:
            window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        history.append((now, bool(ok)))

        # drop old samples
        cutoff = now - window_sec
        while history and history[0][0] < cutoff:
            history.pop(0)

        n = len(history)
        n_ok = sum(1 for _t, v in history if v)

        # do not judge until we have seen a meaningful slice of the window
        covered = (now - history[0][0]) if history else 0.0
        if n < 2 or covered < window_sec * 0.5:
            if not ok:
                return None, False
            if ok_since is None:
                ok_since = now
            return ok_since, False

        frac = n_ok / float(n)

        if not ok:
            ok_since = None
        elif ok_since is None:
            ok_since = now

        return (
            ok_since,
            frac >= float(ratio),
        )

    # ==================================================================
    # Opposite
    # ==================================================================

    @staticmethod
    def _opposite(direction):

        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right

        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left

        return LaneChangeDirection.none

    # ==================================================================
    # Rear traffic
    # ==================================================================

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
        rear_sensing_present=True,
    ):
        """
        Conservative emergency escape-lane veto.

        `rear_sensing_present=False` means the vehicle has no rear-side sensor
        at all, so the absence of a reading carries no information.  In that
        case this gate abstains (BSM still vetoes from the side/rear).
        """

        distance = _safe_float(distance)

        if distance is None:
            # No reading.  Block only if the caller claims to HAVE rear sensing
            # (then "no data" really is suspicious); otherwise abstain.
            return bool(rear_sensing_present)

        if distance <= 0.0:
            return True

        if distance < REAR_DIST_TH:
            return True

        closing_speed = _safe_float(
            closing_speed
        )

        if closing_speed is None:
            return True

        if (
            distance < REAR_SECONDARY_DIST_TH
            and
            closing_speed
            > REAR_SECONDARY_CLOSING_SPEED_TH
        ):
            return True

        if (
            distance < REAR_DIST_TH * 2.0
            and
            closing_speed
            > REAR_CLOSING_SPEED_TH
        ):
            return True

        return False

    # ==================================================================
    # Escape lane safety
    # ==================================================================

    def _escape_lane_safe(
        self,
        *,
        base_lane_ok,
        require_aux_sensors_mr76=None,
        require_aux_sensors_lidar=None,
        rear_sensing_present=None,

        rear_dist,
        rear_speed,

        mr76_available,
        mr76_valid,
        mr76_age,

        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,

        bsm_blocked,
        bsm_available,

        require_aux_sensors,
    ):
        """
        Emergency escape lane must pass all required gates.

        Base lane
            AND
        MR76
            AND
        LiDAR
            AND
        BSM
        """

        if not bool(base_lane_ok):
            return False

        # --------------------------------------------------------------
        # BSM
        # --------------------------------------------------------------

        if (
            bool(bsm_available)
            and
            bool(bsm_blocked)
        ):
            return False

        # --------------------------------------------------------------
        # MR76
        # --------------------------------------------------------------

        mr76_ok = _sensor_fresh(
            mr76_available,
            mr76_valid,
            mr76_age,
            MR76_MAX_AGE_SEC,
        )

        if require_aux_sensors_mr76 is None:
            require_aux_sensors_mr76 = (
                AVOID_REQUIRE_MR76_DEFAULT
                if require_aux_sensors is None
                else bool(require_aux_sensors)
            )

        # The rear-distance veto below is meaningless without rear sensing.
        # Callers that know their vehicle has none pass rear_sensing_present=0
        # (or leave dp_avoid_rear_present at 0) and the gate simply abstains.
        if not bool(rear_sensing_present):
            mr76_ok_for_rear = False
        else:
            mr76_ok_for_rear = mr76_ok

        if require_aux_sensors_mr76 and not mr76_ok:
            return False

        if mr76_ok_for_rear:
            if self._rear_lane_blocked(
                rear_dist,
                rear_speed,
                rear_sensing_present=rear_sensing_present,
            ):
                return False

        # --------------------------------------------------------------
        # LiDAR
        # --------------------------------------------------------------

        lidar_ok = _sensor_fresh(
            lidar_available,
            lidar_valid,
            lidar_age,
            LIDAR_MAX_AGE_SEC,
        )

        if require_aux_sensors_lidar is None:
            require_aux_sensors_lidar = (
                AVOID_REQUIRE_LIDAR_DEFAULT
                if require_aux_sensors is None
                else bool(require_aux_sensors)
            )

        # LiDAR is optional on this car: when there is no LiDAR data at all we
        # must NOT treat that as "lane blocked".  When data IS present, the
        # `lidar_free is not True` check below still applies as a fail-safe.
        if require_aux_sensors_lidar and not lidar_ok:
            return False

        if lidar_ok:
            if lidar_free is not True:
                return False

        return True

    # ==================================================================
    # Direction selection
    # ==================================================================

    @staticmethod
    def _pick_out_direction(
        left_ok,
        right_ok,
        is_rhd,
        prefer_dir=LaneChangeDirection.none,
    ):
        """
        Select ONLY from already-confirmed safe lanes.

        No direction is selected solely because of RHD/LHD.
        """

        if (
            prefer_dir == LaneChangeDirection.left
            and left_ok
        ):
            return LaneChangeDirection.left

        if (
            prefer_dir == LaneChangeDirection.right
            and right_ok
        ):
            return LaneChangeDirection.right

        if left_ok and right_ok:

            # Existing convention is only a tie breaker.
            if is_rhd:
                return LaneChangeDirection.right

            return LaneChangeDirection.left

        if left_ok:
            return LaneChangeDirection.left

        if right_ok:
            return LaneChangeDirection.right

        return LaneChangeDirection.none

    # ==================================================================
    # Risk classification
    # ==================================================================

    @staticmethod
    def _classify_risk(
        *,
        obstacle_in_path,
        obstacle_dist,
        obstacle_rel_speed,

        lead_dist,
        lead_rel_speed,

        is_pedestrian,
        is_cone,

        imminent_collision,
    ):
        """
        Risk classification ONLY.

        This function:

            - does not brake
            - does not modify OEM lead
            - does not create radar targets
        """

        if bool(imminent_collision):
            return RISK_CRITICAL

        # --------------------------------------------------------------
        # Filtered obstacle
        # --------------------------------------------------------------

        if bool(obstacle_in_path):

            distance = _safe_float(
                obstacle_dist
            )

            rel = _safe_float(
                obstacle_rel_speed
            )

            if (
                distance is not None
                and
                rel is not None
            ):

                ttc = compute_ttc(
                    distance,
                    rel,
                )

                if (
                    ttc is not None
                    and
                    ttc <= TTC_CRITICAL
                ):
                    return RISK_CRITICAL

                if (
                    ttc is not None
                    and
                    ttc <= TTC_URGENT
                ):
                    return RISK_URGENT

                closing = max(
                    0.0,
                    -rel,
                )

                if (
                    distance <= DIST_CRITICAL
                    and
                    closing >= CLOSING_CRITICAL
                ):
                    return RISK_CRITICAL

                if (
                    distance <= DIST_URGENT
                    and
                    closing >= CLOSING_URGENT
                ):
                    return RISK_URGENT

                if (
                    distance <= DIST_WARNING
                    and
                    closing >= CLOSING_WARNING
                ):
                    return RISK_WARNING

            # ----------------------------------------------------------
            # Pedestrian / cone
            #
            # Boolean classification alone is not enough to create
            # emergency steering.
            # ----------------------------------------------------------

            if (
                bool(is_pedestrian)
                or
                bool(is_cone)
            ):

                if (
                    distance is not None
                    and
                    distance <= DIST_URGENT
                ):
                    return RISK_URGENT

                return RISK_WARNING

            return RISK_WARNING

        # --------------------------------------------------------------
        # OEM lead emergency condition
        #
        # OEM lead is READ ONLY here.
        # --------------------------------------------------------------

        distance = _safe_float(
            lead_dist
        )

        rel = _safe_float(
            lead_rel_speed
        )

        if (
            distance is not None
            and
            rel is not None
        ):

            closing = max(
                0.0,
                -rel,
            )

            ttc = compute_ttc(
                distance,
                rel,
            )

            if (
                ttc is not None
                and
                ttc <= TTC_CRITICAL
                and
                closing >= CLOSING_CRITICAL
            ):
                return RISK_CRITICAL

            if (
                distance <= DIST_CRITICAL
                and
                closing >= CLOSING_CRITICAL
            ):
                return RISK_CRITICAL

            if (
                ttc is not None
                and
                ttc <= TTC_URGENT
                and
                closing >= CLOSING_URGENT
            ):
                return RISK_URGENT

            if (
                distance <= DIST_URGENT
                and
                closing >= CLOSING_URGENT
            ):
                return RISK_URGENT

            if (
                ttc is not None
                and
                ttc <= TTC_WARNING
                and
                closing >= CLOSING_WARNING
            ):
                return RISK_WARNING

        return RISK_NORMAL

    # ==================================================================
    # Main
    # ==================================================================

    def update(
        self,
        *,
        enabled,

        obstacle_in_path=False,

        lc_state=LaneChangeState.off,

        v_ego=0.0,

        left_ok=False,
        right_ok=False,

        is_rhd=False,

        manual_blinker=False,

        bsm_available=True,

        obstacle_dist=None,
        obstacle_rel_speed=None,

        lead_dist=None,
        lead_rel_speed=None,

        is_pedestrian=False,
        is_cone=False,

        imminent_collision=False,

        # --------------------------------------------------------------
        # Legacy rear interface
        # --------------------------------------------------------------

        rear_left_dist=None,
        rear_left_speed=None,
        rear_left_available=False,

        rear_right_dist=None,
        rear_right_speed=None,
        rear_right_available=False,

        # --------------------------------------------------------------
        # Legacy LiDAR interface
        # --------------------------------------------------------------

        left_lidar_free=None,
        right_lidar_free=None,

        left_bsm_blocked=False,
        right_bsm_blocked=False,

        prefer_dir=LaneChangeDirection.none,

        # --------------------------------------------------------------
        # Explicit MR76 interface
        # --------------------------------------------------------------

        mr76_left_available=None,
        mr76_right_available=None,

        mr76_left_valid=None,
        mr76_right_valid=None,

        mr76_left_age=None,
        mr76_right_age=None,

        # --------------------------------------------------------------
        # Explicit LiDAR interface
        # --------------------------------------------------------------

        left_lidar_available=None,
        right_lidar_available=None,

        left_lidar_valid=None,
        right_lidar_valid=None,

        left_lidar_age=None,
        right_lidar_age=None,

        # --------------------------------------------------------------
        # Integration policy
        # --------------------------------------------------------------

        require_aux_sensors=True,

        # Per-sensor overrides.  None == "follow require_aux_sensors".
        # Defaults chosen for this car: MR76 required, LiDAR optional.
        require_aux_sensors_mr76=None,
        require_aux_sensors_lidar=None,

        # Escape-lane stability policy.  None == module default (strict).
        avoid_stable_mode=None,
        avoid_stable_ratio=None,
        avoid_stable_window_sec=None,

        # BSM blocker debounce (frames).  None/0 == off.
        bsm_debounce_frames=None,

        # Rear-side sensing presence.  None == module default (present).
        rear_sensing_present=None,

        # Risk-input hardening.  None == module default (off).
        avoid_lead_lpf_alpha=None,
        avoid_urgent_confirm=None,
    ):

        now = time.monotonic()

        lane_request = (
            LaneChangeDirection.none
        )

        hazard = False

        lane_offset = 0.0

        # --------------------------------------------------------------
        # Validate ego speed
        # --------------------------------------------------------------

        v_ego = _safe_float(v_ego)

        if v_ego is None:
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Enable / speed gate
        # --------------------------------------------------------------

        if (
            not bool(enabled)
            or
            v_ego < AUTO_AVOID_MIN_SPEED
        ):
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Manual driver input
        #
        # Do not start an automatic emergency maneuver from idle while
        # the driver is manually commanding a lane change.
        # --------------------------------------------------------------

        if (
            bool(manual_blinker)
            and
            self._mode == "idle"
        ):
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Compatibility mapping
        # --------------------------------------------------------------

        if mr76_left_available is None:
            mr76_left_available = bool(
                rear_left_available
            )

        if mr76_right_available is None:
            mr76_right_available = bool(
                rear_right_available
            )

        if left_lidar_available is None:
            left_lidar_available = (
                left_lidar_free is not None
            )

        if right_lidar_available is None:
            right_lidar_available = (
                right_lidar_free is not None
            )

        # --------------------------------------------------------------
        # Escape-lane policy defaults
        # --------------------------------------------------------------

        if avoid_stable_mode is None:
            avoid_stable_mode = AVOID_STABLE_MODE_DEFAULT

        if avoid_stable_ratio is None:
            avoid_stable_ratio = AVOID_STABLE_RATIO_DEFAULT

        if avoid_stable_window_sec is None:
            avoid_stable_window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        if bsm_debounce_frames is None:
            bsm_debounce_frames = AVOID_BSM_DEBOUNCE_DEFAULT

        if rear_sensing_present is None:
            rear_sensing_present = bool(AVOID_REAR_PRESENT_DEFAULT)

        if not hasattr(self, "_left_ok_hist") or self._left_ok_hist is None:
            self._left_ok_hist = []

        if not hasattr(self, "_right_ok_hist") or self._right_ok_hist is None:
            self._right_ok_hist = []

        # --------------------------------------------------------------
        # BSM blocker debounce
        #
        # A single-frame BSM blip must not reset the escape-lane timer.
        # 0 frames == honour the raw flag (historical behaviour).
        # --------------------------------------------------------------

        self._bsm_l_run = getattr(self, "_bsm_l_run", 0)
        self._bsm_r_run = getattr(self, "_bsm_r_run", 0)

        if bool(left_bsm_blocked):
            self._bsm_l_run += 1
        else:
            self._bsm_l_run = 0

        if bool(right_bsm_blocked):
            self._bsm_r_run += 1
        else:
            self._bsm_r_run = 0

        if int(bsm_debounce_frames) > 0:
            left_bsm_blocked = (
                self._bsm_l_run >= int(bsm_debounce_frames)
            )
            right_bsm_blocked = (
                self._bsm_r_run >= int(bsm_debounce_frames)
            )

        # --------------------------------------------------------------
        # Base lane safety
        # --------------------------------------------------------------

        left_base = bool(left_ok)
        right_base = bool(right_ok)

        # --------------------------------------------------------------
        # Escape lane safety
        # --------------------------------------------------------------

        left_safe = self._escape_lane_safe(
            base_lane_ok=left_base,

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            mr76_available=mr76_left_available,
            mr76_valid=mr76_left_valid,
            mr76_age=mr76_left_age,

            lidar_free=left_lidar_free,
            lidar_available=left_lidar_available,
            lidar_valid=left_lidar_valid,
            lidar_age=left_lidar_age,

            bsm_blocked=left_bsm_blocked,
            bsm_available=bsm_available,

            require_aux_sensors=require_aux_sensors,
            require_aux_sensors_mr76=require_aux_sensors_mr76,
            require_aux_sensors_lidar=require_aux_sensors_lidar,
            rear_sensing_present=rear_sensing_present,
        )

        right_safe = self._escape_lane_safe(
            base_lane_ok=right_base,

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            mr76_available=mr76_right_available,
            mr76_valid=mr76_right_valid,
            mr76_age=mr76_right_age,

            lidar_free=right_lidar_free,
            lidar_available=right_lidar_available,
            lidar_valid=right_lidar_valid,
            lidar_age=right_lidar_age,

            bsm_blocked=right_bsm_blocked,
            bsm_available=bsm_available,

            require_aux_sensors=require_aux_sensors,
            require_aux_sensors_mr76=require_aux_sensors_mr76,
            require_aux_sensors_lidar=require_aux_sensors_lidar,
            rear_sensing_present=rear_sensing_present,
        )

        # --------------------------------------------------------------
        # Stable safety
        # --------------------------------------------------------------

        (
            self._left_ok_since,
            left_stable,
        ) = self._stable_ok(
            left_safe,
            self._left_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
            history=self._left_ok_hist,
            mode=avoid_stable_mode,
            ratio=avoid_stable_ratio,
            window_sec=avoid_stable_window_sec,
        )

        (
            self._right_ok_since,
            right_stable,
        ) = self._stable_ok(
            right_safe,
            self._right_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
            history=self._right_ok_hist,
            mode=avoid_stable_mode,
            ratio=avoid_stable_ratio,
            window_sec=avoid_stable_window_sec,
        )

        # --------------------------------------------------------------
        # Risk-input hardening
        #
        # (a) low-pass the lead measurements so a one-frame radar spike cannot
        #     be classified as urgent;
        # (b) optionally require the URGENT level to persist.
        # --------------------------------------------------------------

        if avoid_lead_lpf_alpha is None:
            avoid_lead_lpf_alpha = AVOID_LEAD_LPF_ALPHA_DEFAULT

        if avoid_urgent_confirm is None:
            avoid_urgent_confirm = AVOID_URGENT_CONFIRM_DEFAULT

        alpha = _safe_float(avoid_lead_lpf_alpha)
        if alpha is None or alpha <= 0.0 or alpha > 1.0:
            # disabled -- keep raw values, but still track nothing
            pass
        else:
            d_raw = _safe_float(lead_dist)
            v_raw = _safe_float(lead_rel_speed)

            if d_raw is None or v_raw is None:
                self._lead_lpf_d = None
                self._lead_lpf_v = None
            else:
                if self._lead_lpf_d is None:
                    self._lead_lpf_d = d_raw
                    self._lead_lpf_v = v_raw
                else:
                    self._lead_lpf_d = (
                        (1.0 - alpha) * self._lead_lpf_d + alpha * d_raw
                    )
                    self._lead_lpf_v = (
                        (1.0 - alpha) * self._lead_lpf_v + alpha * v_raw
                    )

                lead_dist = self._lead_lpf_d
                lead_rel_speed = self._lead_lpf_v

        # --------------------------------------------------------------
        # Risk
        # --------------------------------------------------------------

        risk = self._classify_risk(
            obstacle_in_path=bool(
                obstacle_in_path
            ),

            obstacle_dist=obstacle_dist,

            obstacle_rel_speed=obstacle_rel_speed,

            lead_dist=lead_dist,

            lead_rel_speed=lead_rel_speed,

            is_pedestrian=bool(
                is_pedestrian
            ),

            is_cone=bool(
                is_cone
            ),

            imminent_collision=bool(
                imminent_collision
            ),
        )

        self._risk_level = risk

        # --------------------------------------------------------------
        # Critical confirmation
        # --------------------------------------------------------------

        if risk == RISK_CRITICAL:
            self._critical_frames += 1
        else:
            self._critical_frames = 0

        critical_confirmed = (
            self._critical_frames
            >= CRITICAL_CONFIRM_FRAMES
        )

        # --------------------------------------------------------------
        # Urgent confirmation
        #
        # emergency_active needs CRITICAL_CONFIRM_FRAMES to confirm a
        # CRITICAL level, but URGENT used to fire with zero confirmation.
        # Make the URGENT requirement explicit and tunable (default 0 ==
        # historical immediate behaviour).
        # --------------------------------------------------------------

        if risk == RISK_URGENT:
            self._urgent_frames += 1
        else:
            self._urgent_frames = 0

        urgent_confirmed = (
            int(avoid_urgent_confirm) <= 0
            or
            self._urgent_frames >= int(avoid_urgent_confirm)
        )

        emergency_active = (
            (
                risk == RISK_URGENT
                and
                urgent_confirmed
            )
            or
            (
                risk == RISK_CRITICAL
                and
                critical_confirmed
            )
        )

        # --------------------------------------------------------------
        # Debug reason
        # --------------------------------------------------------------

        if risk == RISK_NORMAL:
            self._last_reason = "normal"

        elif risk == RISK_WARNING:
            self._last_reason = "warning_only"

        elif risk == RISK_URGENT:
            self._last_reason = "urgent"

        else:
            self._last_reason = (
                "critical_confirmed"
                if critical_confirmed
                else "critical_confirming"
            )

        # ==============================================================
        # STATE MACHINE
        # ==============================================================

        # --------------------------------------------------------------
        # IDLE
        # --------------------------------------------------------------

        if self._mode == "idle":

            if (
                emergency_active
                and
                now >= self._cooldown_until
            ):

                direction = (
                    self._pick_out_direction(
                        left_stable,
                        right_stable,
                        is_rhd,
                        prefer_dir,
                    )
                )

                self._out_dir = direction

                if (
                    direction
                    != LaneChangeDirection.none
                ):

                    self._mode = "avoiding"

                    self._active_since = now

                    self._emergency_since = now

                    self._observed_lane_change = False

                    lane_request = direction

                    hazard = True

                    self._last_reason = (
                        "emergency_escape_request"
                    )

                else:

                    self._last_reason = (
                        "emergency_no_safe_lane"
                    )

        # --------------------------------------------------------------
        # AVOIDING
        # --------------------------------------------------------------

        elif self._mode == "avoiding":

            hazard = True

            # ----------------------------------------------------------
            # BEFORE actual LC start:
            #
            # The locked direction may be cancelled if the destination
            # lane is no longer safe.
            #
            # We DO NOT automatically reverse direction after starting.
            # ----------------------------------------------------------

            if not self._observed_lane_change:

                if (
                    lc_state
                    == LaneChangeState.laneChangeStarting
                ):
                    self._observed_lane_change = True

                else:

                    direction_safe = (
                        (
                            self._out_dir
                            == LaneChangeDirection.left
                            and
                            left_stable
                        )
                        or
                        (
                            self._out_dir
                            == LaneChangeDirection.right
                            and
                            right_stable
                        )
                    )

                    if direction_safe:
                        lane_request = self._out_dir

                    else:
                        # Do not blindly switch sides.
                        #
                        # The original destination is unsafe.
                        # Re-evaluate both sides only before the actual
                        # maneuver has started.
                        opposite = self._opposite(
                            self._out_dir
                        )

                        opposite_safe = (
                            (
                                opposite
                                == LaneChangeDirection.left
                                and
                                left_stable
                            )
                            or
                            (
                                opposite
                                == LaneChangeDirection.right
                                and
                                right_stable
                            )
                        )

                        if opposite_safe:
                            self._out_dir = opposite

                            lane_request = opposite

                        else:
                            lane_request = (
                                LaneChangeDirection.none
                            )

                            self._last_reason = (
                                "escape_lane_lost"
                            )

            # ----------------------------------------------------------
            # AFTER actual LC start:
            #
            # Direction is LOCKED.
            # ----------------------------------------------------------

            else:

                lane_request = self._out_dir

            # ----------------------------------------------------------
            # Detect actual completion.
            # ----------------------------------------------------------

            if (
                self._observed_lane_change
                and
                lc_state == LaneChangeState.off
            ):

                self._observed_lane_change = False

                self._mode = "waiting_clear"

                self._clear_since = now

                self._last_reason = (
                    "emergency_lc_completed"
                )

        # --------------------------------------------------------------
        # WAITING CLEAR
        # --------------------------------------------------------------

        elif self._mode == "waiting_clear":

            hazard = False

            # ----------------------------------------------------------
            # Emergency returned immediately.
            # ----------------------------------------------------------

            if emergency_active:

                self._mode = "avoiding"

                self._clear_since = None

                hazard = True

                # IMPORTANT:
                #
                # Reuse locked direction only if it is still safe.
                #
                # Otherwise select a new safe escape direction.
                #

                direction_safe = (
                    (
                        self._out_dir
                        == LaneChangeDirection.left
                        and
                        left_stable
                    )
                    or
                    (
                        self._out_dir
                        == LaneChangeDirection.right
                        and
                        right_stable
                    )
                )

                if not direction_safe:

                    self._out_dir = (
                        self._pick_out_direction(
                            left_stable,
                            right_stable,
                            is_rhd,
                            prefer_dir,
                        )
                    )

                if (
                    self._out_dir
                    != LaneChangeDirection.none
                ):
                    lane_request = self._out_dir

                self._active_since = now

            else:

                if self._clear_since is None:
                    self._clear_since = now

                clear_time = (
                    now - self._clear_since
                )

                active_time_ok = (
                    self._active_since is None
                    or
                    (
                        now - self._active_since
                    ) >= MIN_ACTIVE_SEC
                )

                if (
                    clear_time >= 1.0
                    and
                    active_time_ok
                ):

                    self._mode = "cooldown"

                    self._cooldown_until = (
                        now + AVOID_COOLDOWN_SEC
                    )

                    self._clear_since = None

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._last_reason = (
                        "emergency_cleared"
                    )

        # --------------------------------------------------------------
        # COOLDOWN
        # --------------------------------------------------------------

        elif self._mode == "cooldown":

            if (
                emergency_active
                and
                now >= self._cooldown_until
            ):

                self._mode = "avoiding"

                self._active_since = now

                self._emergency_since = now

                self._out_dir = (
                    self._pick_out_direction(
                        left_stable,
                        right_stable,
                        is_rhd,
                        prefer_dir,
                    )
                )

                if (
                    self._out_dir
                    != LaneChangeDirection.none
                ):

                    lane_request = self._out_dir

                    hazard = True

            elif (
                now >= self._cooldown_until
            ):

                self._mode = "idle"

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._last_reason = (
                    "idle_after_cooldown"
                )

        else:

            self.reset()

        # --------------------------------------------------------------
        # Diagnostic lane offset
        #
        # IMPORTANT:
        #
        # This is NOT a steering command.
        # Caller must NOT directly convert this value into actuator CAN.
        # --------------------------------------------------------------

        if (
            emergency_active
            and
            self._mode == "avoiding"
        ):

            if (
                self._out_dir
                == LaneChangeDirection.left
            ):
                lane_offset = (
                    -EMERGENCY_OFFSET
                )

            elif (
                self._out_dir
                == LaneChangeDirection.right
            ):
                lane_offset = (
                    EMERGENCY_OFFSET
                )

        # --------------------------------------------------------------
        # ABSOLUTE LONGITUDINAL SAFETY BOUNDARY
        # --------------------------------------------------------------

        brake_request = 0.0

        self._last_lc_state = lc_state

        return (
            lane_request,
            brake_request,
            hazard,
            lane_offset,
        )

    # ==================================================================
    # Debug
    # ==================================================================

    def get_state(self):
        return {
            "mode": self._mode,
            "out_dir": self._out_dir,
            "cooldown_until": self._cooldown_until,
            "active_since": self._active_since,
            "emergency_since": self._emergency_since,
            "risk_level": self._risk_level,
            "critical_frames": self._critical_frames,
            "left_ok_since": self._left_ok_since,
            "right_ok_since": self._right_ok_since,
            "observed_lane_change": self._observed_lane_change,
            "last_reason": self._last_reason,
        }