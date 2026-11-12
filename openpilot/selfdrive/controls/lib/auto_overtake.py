#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time

from openpilot.cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

OVERTAKE_MIN_SPEED_CAP_MS = 90.0 * CV.KPH_TO_MS

OVERTAKE_MIN_SPEED_FLOOR_MS = 90.0 * CV.KPH_TO_MS

OVERTAKE_MIN_SPEED = OVERTAKE_MIN_SPEED_CAP_MS
OVERTAKE_MIN_CRUISE_SPEED = OVERTAKE_MIN_SPEED_CAP_MS

OVERTAKE_SPEED_DELTA = 1.0 * CV.KPH_TO_MS

OVERTAKE_MIN_CLOSING_SPEED = 1.0

OVERTAKE_HEADWAY_MAX_S = 5.0

OVERTAKE_LEAD_STABLE_SEC = 0.50

PREPARE_BEFORE_LC_SEC = 1.00

CLEAR_LANE_STABLE_SEC = 1.00

RETURN_CLEAR_DELAY_SEC = 4.0

RETURN_MIN_TIME_AFTER_OUT_SEC = 5.0

OVERTAKE_COOLDOWN_SEC = 25.0

REAR_DIST_TH = 20.0

REAR_CLOSING_SPEED_TH = 8.0

REAR_TTC_TH = 3.0

OVERTAKE_LEAD_MIN_DIST = 25.0

OVERTAKE_LEAD_MAX_DIST = 80.0

AO_GAIN_GATE_DEFAULT = True

AO_GAIN_MIN_DV_MS = 5.0 * CV.KPH_TO_MS

AO_GAIN_MIN_DIST_MARGIN_M = 20.0

AO_LEAD_JUMP_MAX_M = 15.0

AO_LEAD_JUMP_MAX_DT_SEC = 0.50

AO_LEAD_SETTLE_SEC = 0.50

AO_REAR_ZONE_MAX_OK = 2

AO_REAR_ZONE_BLOCK = 3

AO_REQUIRE_REAR_ZONE = False

AO_LANE_PROB_HARD_MIN = 0.30

AO_LANE_PROB_WITNESS = 0.55

AO_LANE_PROB_USE_PARAM = True

AO_LANE_CONF_STICKY_SEC = 0.30

AO_JUNCTION_BAN_DIST_M = 15.0

AO_JUNCTION_LANE_PROB = 0.30

AO_JUNCTION_CONFIRM_SEC = 0.60

AO_JUNCTION_MIN_SAMPLES = 6

AO_CONF_LOG_PATH = "/data/media/0/ao_conf.log"
AO_CONF_LOG_MAX_BYTES = 2_000_000
AO_CONF_LOG_PERIOD_SEC = 0.5

PASSED_LEAD_DIST = 35.0

PASSED_LEAD_SPEED = 4.0

PASSED_LEAD_STABLE_SEC = 0.50

LEAD_LOST_STABLE_SEC = 0.50

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25

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

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2

def _overtake_min_speed(v_cruise):
    v_cruise = _safe_float(v_cruise)

    if v_cruise is None or v_cruise <= 0.0:
        return OVERTAKE_MIN_SPEED_FLOOR_MS

    hooked = v_cruise - OVERTAKE_SPEED_DELTA

    if hooked < OVERTAKE_MIN_SPEED_FLOOR_MS:
        return OVERTAKE_MIN_SPEED_FLOOR_MS

    if hooked > OVERTAKE_MIN_SPEED_CAP_MS:
        return OVERTAKE_MIN_SPEED_CAP_MS

    return hooked

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

    distance = _safe_float(distance)
    closing_speed = _safe_float(closing_speed)

    if distance is None or closing_speed is None:
        return None

    if distance <= 0.0:
        return 0.0

    if closing_speed <= 0.01:
        return None

    return distance / closing_speed

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

        self._lead_hist_t = None
        self._lead_jump_until = 0.0
        self._lead_jump_count = 0

        self._observed_out_lc = False
        self._observed_return_lc = False

        self._last_reason = "idle"

        self._stay_in_fast_lane = False

        self._last_overtake_time = 0.0

        self._lane_conf_hist = {
            "left": [],
            "right": [],
        }

        self._lane_conf_raw = {
            "left": None,
            "right": None,
        }

        self._lane_conf_witness = AO_LANE_PROB_WITNESS

        self._lane_prob_min_current = None

        self._junction_since = None
        self._junction_source = None
        self._junction_samples = 0
        self._last_junction_ban = False

        self._conf_log_t = -1e9

        self._left_veto_until = 0.0
        self._right_veto_until = 0.0

        self._left_veto_reason = ""
        self._right_veto_reason = ""

        self._last_veto_direction = LaneChangeDirection.none
        self._last_veto_reason = ""

        self._gain_ok_left = False
        self._gain_ok_right = False

        self._gain_reason_left = "gain_unavailable"
        self._gain_reason_right = "gain_unavailable"

        self._last_gain_block_dir = LaneChangeDirection.none
        self._last_gain_reason = "gain_unavailable"

    def reset(self):
        self.__init__()

    def _speed_gate_abort(
        self,
        now,
        lc_state,
    ):

        if (
            (
                self._observed_out_lc
                or
                self._observed_return_lc
            )
            and
            lc_state != LaneChangeState.off
        ):

            self._last_reason = "speed_gate_hold_lc"

            return

        in_maneuver = self._mode in (
            "changing_out",
            "waiting_return",
            "changing_back",
        )

        self.reset()

        if in_maneuver:

            self._cooldown_until = (
                now
                + OVERTAKE_COOLDOWN_SEC
            )

            self._last_reason = "speed_gate_abort"

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

    def _set_lane_veto(
        self,
        side,
        reason,
        now,
    ):

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

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
    ):

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

    def _rear_zone_gate(
        self,
        *,
        side,
        rear_zone,
        rear_sensor_fault,
        now,
    ):

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

    @staticmethod
    def _explicit_obstacle_present(
        obstacle,
    ):

        if obstacle is None:
            return None

        return bool(obstacle)

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

        if available is None:
            if require_aux_sensors:
                self._set_lane_veto(
                    side,
                    "mr76_unavailable_or_stale",
                    now,
                )
                return False
            return True

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

        if lidar_available is None:
            if require_aux_sensors:
                self._set_lane_veto(
                    side,
                    "lidar_unavailable_or_stale",
                    now,
                )
                return False
            return True

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

        if lidar_ok:

            if lidar_free is not True:

                self._set_lane_veto(
                    side,
                    "lidar_lane_not_clear",
                    now,
                )

                return False

        return True

    def _bsm_lane_safe(
        self,
        *,
        side,
        bsm_available,
        bsm_blocked,
        now,
    ):

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

    def _lane_conf_threshold(
        self,
        lane_prob_min,
    ):

        threshold = AO_LANE_PROB_HARD_MIN

        pref = _safe_float(
            lane_prob_min
        )

        if (
            AO_LANE_PROB_USE_PARAM
            and
            pref is not None
            and
            pref > threshold
        ):
            threshold = pref

        return threshold

    def _update_lane_conf(
        self,
        now,
        left_lane_prob,
        right_lane_prob,
    ):

        threshold = self._lane_conf_threshold(
            self._lane_prob_min_current
        )

        witness = max(
            threshold,
            AO_LANE_PROB_WITNESS,
        )

        for key, prob in (
            ("left", left_lane_prob),
            ("right", right_lane_prob),
        ):

            value = _safe_float(
                prob
            )

            hist = self._lane_conf_hist[
                key
            ]

            if value is None:

                hist.append(
                    (
                        now,
                        None,
                    )
                )

            else:

                self._lane_conf_raw[
                    key
                ] = value

                hist.append(
                    (
                        now,
                        bool(
                            value >= threshold
                        ),
                    )
                )

            cutoff = (
                now
                - AO_LANE_CONF_STICKY_SEC
            )

            while (
                hist
                and
                hist[0][0] < cutoff
            ):
                hist.pop(0)

        self._lane_conf_witness = witness

    def _lane_conf_ok(
        self,
        side,
    ):

        hist = self._lane_conf_hist[
            side
        ]

        other = (
            "right"
            if side == "left"
            else
            "left"
        )

        seen = False
        all_ok = True

        for _, ok in hist:

            if ok is None:

                continue

            seen = True

            if not ok:

                all_ok = False

                break

        if seen and all_ok:

            return True

        raw_other = self._lane_conf_raw.get(
            other
        )

        witness = getattr(
            self,
            "_lane_conf_witness",
            AO_LANE_PROB_WITNESS,
        )

        if (
            raw_other is not None
            and
            raw_other >= witness
        ):

            return True

        return False

    def _lane_conf_raw_of(
        self,
        side,
    ):

        return self._lane_conf_raw.get(
            side
        )

    def _lane_conf_reason(
        self,
        side,
    ):

        return (
            "left_lane_conf_low"
            if side == "left"
            else
            "right_lane_conf_low"
        )

    def _update_junction_guess(
        self,
        now,
        left_lane_prob,
        right_lane_prob,
        junction_dist,
    ):

        lp_left = _safe_float(
            left_lane_prob
        )

        lp_right = _safe_float(
            right_lane_prob
        )

        left_weak = (
            lp_left is not None
            and
            lp_left < AO_JUNCTION_LANE_PROB
        )

        right_weak = (
            lp_right is not None
            and
            lp_right < AO_JUNCTION_LANE_PROB
        )

        d = _safe_float(
            junction_dist
        )

        if (
            d is not None
            and
            d <= AO_JUNCTION_BAN_DIST_M
        ):

            if self._junction_since is None:

                self._junction_since = now

            self._junction_source = "map"

            return

        if left_weak and right_weak:

            if self._junction_since is None:

                self._junction_since = now

                self._junction_samples = 0

            self._junction_samples += 1

            if (
                self._junction_source
                != "map"
            ):
                self._junction_source = "lane_dropout"

            return

        self._junction_since = None

        self._junction_source = None

        self._junction_samples = 0

    def _junction_ban_active(
        self,
        now,
        junction_dist,
    ):

        d = _safe_float(
            junction_dist
        )

        if (
            d is not None
            and
            d <= AO_JUNCTION_BAN_DIST_M
        ):

            return True

        if self._junction_since is None:

            return False

        if (
            now
            - self._junction_since
        ) < AO_JUNCTION_CONFIRM_SEC:

            return False

        return bool(
            self._junction_samples
            >= AO_JUNCTION_MIN_SAMPLES
        )

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

        rear_zone=None,
        rear_sensor_fault=None,

        base_lane_ok,

        require_aux_sensors,

        now,
    ):

        if not bool(base_lane_ok):

            self._set_lane_veto(
                side,
                "base_lane_not_safe",
                now,
            )

            return False

        if self._lane_vetoed(
            side,
            now,
        ):

            return False

        if not self._rear_zone_gate(
            side=side,
            rear_zone=rear_zone,
            rear_sensor_fault=rear_sensor_fault,
            now=now,
        ):
            return False

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

        if not self._bsm_lane_safe(
            side=side,
            bsm_available=bsm_available,
            bsm_blocked=bsm_blocked,
            now=now,
        ):
            return False

        return True

    @staticmethod
    def _lane_score(
        *,
        side,
        rear_dist,
        rear_speed,
        lane_preference,
        is_highway,
    ):

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

    @staticmethod
    def _lane_gain_ok(
        *,
        target_present,
        target_d,
        target_v,
        ego_lead_d,
        ego_lead_v,
    ):

        if target_present is None:

            return False, "gain_unavailable"

        if not bool(target_present):

            return True, "gain_clear"

        _d = _safe_float(
            target_d
        )

        _v = _safe_float(
            target_v
        )

        _ego_d = _safe_float(
            ego_lead_d
        )

        _ego_v = _safe_float(
            ego_lead_v
        )

        if (
            _d is not None
            and _ego_d is not None
            and _d
            >= (
                _ego_d
                + AO_GAIN_MIN_DIST_MARGIN_M
            )
        ):

            return True, "gain_farther"

        if (
            _v is not None
            and _ego_v is not None
            and (
                _v
                - _ego_v
            )
            >= AO_GAIN_MIN_DV_MS
        ):

            return True, "gain_faster"

        return False, "gain_no_benefit"

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

        if now < self._lead_jump_until:

            self._need_since = None

            return False

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
            >= _overtake_min_speed(v_cruise)
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
            self._lead_hist_t = None

            return

        self._lead_lost_since = None

        if self._lead_last_distance is None:

            self._lead_last_distance = lead_d
            self._lead_distance_increasing_since = None
            self._lead_hist_t = now

            return

        _dt = (
            None
            if self._lead_hist_t is None
            else (
                now
                - self._lead_hist_t
            )
        )

        if (
            _dt is not None
            and 0.0 < _dt <= AO_LEAD_JUMP_MAX_DT_SEC
            and abs(
                lead_d
                - self._lead_last_distance
            ) > AO_LEAD_JUMP_MAX_M
        ):

            self._lead_jump_until = (
                now
                + AO_LEAD_SETTLE_SEC
            )

            self._lead_jump_count += 1

        self._lead_hist_t = now

        if lead_d > self._lead_last_distance:

            if self._lead_distance_increasing_since is None:
                self._lead_distance_increasing_since = now

        else:

            self._lead_distance_increasing_since = None

        self._lead_last_distance = lead_d

    def _lead_passed(
        self,
        *,
        now,
        lead_present,
        lead_d,
        v_ego,
        v_lead,
    ):

        lead_d = _safe_float(
            lead_d
        )

        v_ego = _safe_float(
            v_ego
        )

        v_lead = _safe_float(
            v_lead
        )

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

        left_rear_zone=None,
        right_rear_zone=None,
        rear_sensor_fault=None,

        lane_preference=LANE_PREF_AUTO,
        min_cruise_speed=None,

        mr76_left_available=None,
        mr76_right_available=None,

        mr76_left_valid=None,
        mr76_right_valid=None,

        mr76_left_age=None,
        mr76_right_age=None,

        mr76_left_obstacle=None,
        mr76_right_obstacle=None,

        left_lidar_available=None,
        right_lidar_available=None,

        left_lidar_valid=None,
        right_lidar_valid=None,

        left_lidar_age=None,
        right_lidar_age=None,

        left_lidar_obstacle=None,
        right_lidar_obstacle=None,

        left_lane_prob=None,
        right_lane_prob=None,

        lane_prob_min=None,

        junction_dist=None,

        left_target_lead_present=None,
        right_target_lead_present=None,

        left_target_lead_d=None,
        right_target_lead_d=None,

        left_target_lead_v=None,
        right_target_lead_v=None,

        require_lane_gain=AO_GAIN_GATE_DEFAULT,

        require_aux_sensors=True,
    ):

        now = time.monotonic()

        request = LaneChangeDirection.none

        self._lane_prob_min_current = lane_prob_min

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

        if manual_blinker:

            self.reset()

            self._last_lc_state = lc_state

            return request

        if not enabled:

            self.reset()

            self._last_lc_state = lc_state

            return request

        if not bsm_available:

            self.reset()

            self._last_lc_state = lc_state

            return request

        _ao_default_min_cruise = _overtake_min_speed(v_cruise)

        min_cruise_speed = (
            _ao_default_min_cruise
            if min_cruise_speed is None
            else _safe_float(
                min_cruise_speed
            )
        )

        if min_cruise_speed is None:
            min_cruise_speed = (
                _ao_default_min_cruise
            )

        _ao_v_min = _overtake_min_speed(v_cruise)

        if (
            v_ego < _ao_v_min
            or
            v_cruise < min_cruise_speed
        ):

            self._speed_gate_abort(
                now,
                lc_state,
            )

            self._last_lc_state = lc_state

            return request

        if mr76_left_available is None:

            _mr76_left_any = (
                rear_left_dist is not None
                or
                rear_left_speed is not None
                or
                mr76_left_obstacle is not None
            )

            if _mr76_left_any:
                mr76_left_available = True

        if mr76_right_available is None:

            _mr76_right_any = (
                rear_right_dist is not None
                or
                rear_right_speed is not None
                or
                mr76_right_obstacle is not None
            )

            if _mr76_right_any:
                mr76_right_available = True

        if left_lidar_available is None:

            _lidar_left_any = (
                left_lidar_free is not None
                or
                left_lidar_obstacle is not None
            )

            if _lidar_left_any:
                left_lidar_available = True

        if right_lidar_available is None:

            _lidar_right_any = (
                right_lidar_free is not None
                or
                right_lidar_obstacle is not None
            )

            if _lidar_right_any:
                right_lidar_available = True

        left_base = bool(
            left_ok
        )

        right_base = bool(
            right_ok
        )

        self._update_lane_conf(
            now,
            left_lane_prob,
            right_lane_prob,
        )

        left_lane_conf_ok = self._lane_conf_ok(
            "left"
        )

        right_lane_conf_ok = self._lane_conf_ok(
            "right"
        )

        if not left_lane_conf_ok:

            left_base = False

        if not right_lane_conf_ok:

            right_base = False

        self._update_junction_guess(
            now,
            left_lane_prob,
            right_lane_prob,
            junction_dist,
        )

        junction_ban = self._junction_ban_active(
            now,
            junction_dist,
        )

        if junction_ban:

            left_base = False

            right_base = False

        self._last_junction_ban = bool(
            junction_ban
        )

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

        self._update_lead_history(
            now,
            bool(lead_present),
            lead_d,
        )

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

        (
            self._gain_ok_left,
            self._gain_reason_left,
        ) = self._lane_gain_ok(
            target_present=left_target_lead_present,
            target_d=left_target_lead_d,
            target_v=left_target_lead_v,
            ego_lead_d=lead_d,
            ego_lead_v=v_lead,
        )

        (
            self._gain_ok_right,
            self._gain_reason_right,
        ) = self._lane_gain_ok(
            target_present=right_target_lead_present,
            target_d=right_target_lead_d,
            target_v=right_target_lead_v,
            ego_lead_d=lead_d,
            ego_lead_v=v_lead,
        )

        if best_dir == LaneChangeDirection.left:

            self._last_gain_reason = self._gain_reason_left

            if require_lane_gain and not self._gain_ok_left:

                self._last_gain_block_dir = best_dir

                best_dir = LaneChangeDirection.none

        elif best_dir == LaneChangeDirection.right:

            self._last_gain_reason = self._gain_reason_right

            if require_lane_gain and not self._gain_ok_right:

                self._last_gain_block_dir = best_dir

                best_dir = LaneChangeDirection.none

        else:

            self._last_gain_block_dir = LaneChangeDirection.none

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

        elif self._mode == "preparing":

            current_safe = (
                left_stable
                if
                self._out_dir
                ==
                LaneChangeDirection.left
                else
                right_stable
            )

            current_gain_ok = (
                self._gain_ok_left
                if
                self._out_dir
                ==
                LaneChangeDirection.left
                else
                self._gain_ok_right
            )

            if not require_lane_gain:

                current_gain_ok = True

            if not need_overtake:

                self.reset()

            elif not current_gain_ok:

                self._mode = "idle"

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._prepare_since = None

                self._last_reason = (
                    "gain_gate_abort"
                )

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

        elif self._mode == "changing_out":

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

        self._write_conf_log(
            now,

            lane_prob_L=left_lane_prob,
            lane_prob_R=right_lane_prob,

            junction_dist=junction_dist,

            speed_mps=v_ego,
        )

        return request

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

            "lane_conf_L":
                self._lane_conf_ok("left"),

            "lane_conf_R":
                self._lane_conf_ok("right"),

            "lane_conf_threshold":
                self._lane_conf_threshold(
                    self._lane_prob_min_current
                ),

            "lane_conf_witness":
                self._lane_conf_witness,

            "lane_conf_L_last":
                self._lane_conf_last("left"),

            "lane_conf_R_last":
                self._lane_conf_last("right"),

            "lane_prob_L_raw":
                self._lane_conf_raw_of("left"),

            "lane_prob_R_raw":
                self._lane_conf_raw_of("right"),

            "junction_ban":
                self._last_junction_ban,

            "junction_source":
                self._junction_source,

            "junction_since":
                self._junction_since,

            "gain_ok_L":
                self._gain_ok_left,

            "gain_ok_R":
                self._gain_ok_right,

            "gain_reason_L":
                self._gain_reason_left,

            "gain_reason_R":
                self._gain_reason_right,

            "last_gain_reason":
                self._last_gain_reason,

            "last_gain_block_dir":
                self._last_gain_block_dir,

            "lead_jump_count":
                self._lead_jump_count,

            "lead_jump_active":
                now < self._lead_jump_until,

            "lead_jump_until":
                self._lead_jump_until,
        }

    def _lane_conf_last(
        self,
        side,
    ):

        hist = self._lane_conf_hist[
            side
        ]

        for _, ok in reversed(
            hist
        ):

            if ok is not None:

                return ok

        return None

    def _write_conf_log(
        self,
        now,
        *,
        lane_prob_L,
        lane_prob_R,
        junction_dist=None,
        speed_mps=None,
        mode=None,
    ):

        if (
            now
            - self._conf_log_t
        ) < AO_CONF_LOG_PERIOD_SEC:

            return

        self._conf_log_t = now

        try:

            try:

                if os.path.getsize(
                    AO_CONF_LOG_PATH
                ) > AO_CONF_LOG_MAX_BYTES:

                    os.replace(
                        AO_CONF_LOG_PATH,
                        AO_CONF_LOG_PATH
                        + ".1",
                    )

            except OSError:

                pass

            record = {
                "t":
                    round(
                        now,
                        3,
                    ),

                "lpL":
                    _safe_float(
                        lane_prob_L
                    ),

                "lpR":
                    _safe_float(
                        lane_prob_R
                    ),

                "jd":
                    _safe_float(
                        junction_dist
                    ),

                "v":
                    round(
                        float(speed_mps),
                        2,
                    )
                    if _safe_float(
                        speed_mps
                    )
                    is not None
                    else
                    None,

                "mode":
                    mode
                    if mode is not None
                    else
                    self._mode,

                "confL":
                    self._lane_conf_ok("left"),

                "confR":
                    self._lane_conf_ok("right"),

                "thr":
                    round(
                        self._lane_conf_threshold(
                            self._lane_prob_min_current
                        ),
                        3,
                    ),

                "wit":
                    round(
                        float(
                            self._lane_conf_witness
                        ),
                        3,
                    ),

                "jban":
                    self._last_junction_ban,

                "jsrc":
                    self._junction_source,

                "jns":
                    self._junction_samples,
            }

            with open(
                AO_CONF_LOG_PATH,
                "a",
                encoding="utf-8",
            ) as fp:

                fp.write(
                    json.dumps(
                        record,
                        ensure_ascii=False,
                    )
                    + "\n"
                )

        except Exception:

            pass