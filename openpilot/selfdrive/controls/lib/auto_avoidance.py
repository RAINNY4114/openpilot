#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

from openpilot.cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

AUTO_AVOID_MIN_SPEED = (
    25.0 * CV.KPH_TO_MS
)

TTC_WARNING = 5.0
TTC_URGENT = 3.5
TTC_CRITICAL = 2.0

DIST_WARNING = 30.0
DIST_URGENT = 20.0
DIST_CRITICAL = 12.0

CLOSING_WARNING = 2.0
CLOSING_URGENT = 4.0
CLOSING_CRITICAL = 7.0

TTC_CRITICAL_DIST_GUARD = 30.0
TTC_URGENT_DIST_GUARD = 40.0

CRITICAL_CONFIRM_FRAMES = 3

CLEAR_LANE_STABLE_SEC = 0.30

MIN_ACTIVE_SEC = 0.40

AVOID_COOLDOWN_SEC = 1.5

AVOID_ESC_ABORT_SEC = 2.0

REAR_DIST_TH = 8.0

REAR_CLOSING_SPEED_TH = 15.0

REAR_SECONDARY_DIST_TH = 15.0

REAR_SECONDARY_CLOSING_SPEED_TH = 10.0

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25

AVOID_REQUIRE_MR76_DEFAULT = True
AVOID_REQUIRE_LIDAR_DEFAULT = False

AVOID_STABLE_MODE_DEFAULT = "strict"
AVOID_STABLE_RATIO_DEFAULT = 0.60
AVOID_STABLE_WINDOW_SEC_DEFAULT = 0.60

AVOID_BSM_DEBOUNCE_DEFAULT = 0

AVOID_BRAKE_GAIN_DEFAULT = 1.0
AVOID_BRAKE_MAX_DEFAULT = 0.0
AVOID_BRAKE_TTC_REF_DEFAULT = 3.0

AVOID_REAR_PRESENT_DEFAULT = 1

AVOID_LATERAL_ENABLE_DEFAULT = 0
AVOID_LATERAL_MIN_SPEED_DEFAULT = 16.67
AVOID_LATERAL_CONFIRM_DEFAULT = 5
AVOID_LATERAL_CLOSING_MIN_DEFAULT = 0.0

AVOID_LEAD_LPF_ALPHA_DEFAULT = 0.0
AVOID_URGENT_CONFIRM_DEFAULT = 0

EMERGENCY_OFFSET = 0.8

RISK_NORMAL = 0
RISK_WARNING = 1
RISK_URGENT = 2
RISK_CRITICAL = 3

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

        self._lateral_left_frames = 0
        self._lateral_right_frames = 0
        self._lateral_active = False
        self._lateral_dir = 0

        self._critical_frames = 0

        self._lead_lpf_d = None
        self._lead_lpf_v = None
        self._urgent_frames = 0

        self._risk_level = RISK_NORMAL

        self._last_reason = "idle"

        self._esc_abort_since = None

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
        self._lateral_left_frames = 0
        self._lateral_right_frames = 0
        self._lateral_active = False
        self._lateral_dir = 0
        self._esc_abort_since = None

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

        mode = AVOID_STABLE_MODE_DEFAULT if mode is None else str(mode)

        if mode != "ratio" or history is None:
            if not ok:
                return None, False

            if ok_since is None:
                ok_since = now

            return (
                ok_since,
                (now - ok_since) >= stable_sec,
            )

        if ratio is None:
            ratio = AVOID_STABLE_RATIO_DEFAULT

        if window_sec is None:
            window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        history.append((now, bool(ok)))

        cutoff = now - window_sec
        while history and history[0][0] < cutoff:
            history.pop(0)

        n = len(history)
        n_ok = sum(1 for _t, v in history if v)

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

    @staticmethod
    def _opposite(direction):

        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right

        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left

        return LaneChangeDirection.none

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
        rear_sensing_present=True,
    ):

        distance = _safe_float(distance)

        if distance is None:
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

        mr76_obstacle=None,

        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,

        bsm_blocked,
        bsm_available,

        require_aux_sensors,
    ):

        if not bool(base_lane_ok):
            return False

        if (
            bool(bsm_available)
            and
            bool(bsm_blocked)
        ):
            return False

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

        if mr76_obstacle is True:
            return False

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

        if require_aux_sensors_lidar and not lidar_ok:
            return False

        if lidar_ok:
            if lidar_free is not True:
                return False

        return True

    @staticmethod
    def _pick_out_direction(
        left_ok,
        right_ok,
        is_rhd,
        prefer_dir=LaneChangeDirection.none,
    ):

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

            if is_rhd:
                return LaneChangeDirection.right

            return LaneChangeDirection.left

        if left_ok:
            return LaneChangeDirection.left

        if right_ok:
            return LaneChangeDirection.right

        return LaneChangeDirection.none

    @staticmethod
    def _calc_emergency_brake(
        *,
        obstacle_dist,
        obstacle_rel_speed,
        lead_dist,
        lead_rel_speed,
        v_ego,
        brake_gain=None,
        brake_max=None,
        brake_ttc_ref=None,
    ):

        if brake_gain is None:
            brake_gain = AVOID_BRAKE_GAIN_DEFAULT

        if brake_max is None:
            brake_max = AVOID_BRAKE_MAX_DEFAULT

        if brake_ttc_ref is None:
            brake_ttc_ref = AVOID_BRAKE_TTC_REF_DEFAULT

        brake_max = max(0.0, float(brake_max))
        if brake_max <= 0.0:
            return 0.0

        v = _safe_float(v_ego)
        if v is None or v <= 0.0:
            return 0.0

        distance = _safe_float(obstacle_dist)
        rel = _safe_float(obstacle_rel_speed)

        if distance is None or rel is None:
            distance = _safe_float(lead_dist)
            rel = _safe_float(lead_rel_speed)

        if distance is None or rel is None:
            return 0.0

        if distance <= 0.0:
            return float(brake_max)

        ttc = compute_ttc(distance, rel)
        if ttc is None:
            return 0.0

        ref = max(0.1, float(brake_ttc_ref))
        if ttc >= ref:
            return 0.0

        urgency = (ref - ttc) / ref
        urgency = max(0.0, min(1.0, urgency))

        req = float(brake_gain) * urgency * float(brake_max)

        if not math.isfinite(req):
            return 0.0

        return float(max(0.0, min(brake_max, req)))

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

        if bool(imminent_collision):
            return RISK_CRITICAL

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
                    and
                    distance <= TTC_CRITICAL_DIST_GUARD
                ):
                    return RISK_CRITICAL

                if (
                    ttc is not None
                    and
                    ttc <= TTC_URGENT
                    and
                    distance <= TTC_URGENT_DIST_GUARD
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

        rear_left_dist=None,
        rear_left_speed=None,
        rear_left_available=False,

        rear_right_dist=None,
        rear_right_speed=None,
        rear_right_available=False,

        left_lidar_free=None,
        right_lidar_free=None,

        left_bsm_blocked=False,
        right_bsm_blocked=False,

        prefer_dir=LaneChangeDirection.none,

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

        require_aux_sensors=True,

        require_aux_sensors_mr76=None,
        require_aux_sensors_lidar=None,

        avoid_stable_mode=None,
        avoid_stable_ratio=None,
        avoid_stable_window_sec=None,

        avoid_brake_gain=None,
        avoid_brake_max=None,
        avoid_brake_ttc_ref=None,

        bsm_debounce_frames=None,

        rear_sensing_present=None,

        avoid_lead_lpf_alpha=None,
        avoid_urgent_confirm=None,

        lateral_closing=None,
        avoid_lateral_enable=None,
        avoid_lateral_min_speed=None,
        avoid_lateral_confirm=None,
        avoid_lateral_closing_min=None,
    ):

        now = time.monotonic()

        lane_request = (
            LaneChangeDirection.none
        )

        hazard = False

        lane_offset = 0.0

        brake_request = 0.0

        v_ego = _safe_float(v_ego)

        if v_ego is None:
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

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

        if avoid_stable_mode is None:
            avoid_stable_mode = AVOID_STABLE_MODE_DEFAULT

        if avoid_stable_ratio is None:
            avoid_stable_ratio = AVOID_STABLE_RATIO_DEFAULT

        if avoid_stable_window_sec is None:
            avoid_stable_window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        if avoid_brake_gain is None:
            avoid_brake_gain = AVOID_BRAKE_GAIN_DEFAULT

        if avoid_brake_max is None:
            avoid_brake_max = AVOID_BRAKE_MAX_DEFAULT

        if avoid_brake_ttc_ref is None:
            avoid_brake_ttc_ref = AVOID_BRAKE_TTC_REF_DEFAULT

        if bsm_debounce_frames is None:
            bsm_debounce_frames = AVOID_BSM_DEBOUNCE_DEFAULT

        if rear_sensing_present is None:
            rear_sensing_present = bool(AVOID_REAR_PRESENT_DEFAULT)

        if not hasattr(self, "_left_ok_hist") or self._left_ok_hist is None:
            self._left_ok_hist = []

        if not hasattr(self, "_right_ok_hist") or self._right_ok_hist is None:
            self._right_ok_hist = []

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

        left_base = bool(left_ok)
        right_base = bool(right_ok)

        left_safe = self._escape_lane_safe(
            base_lane_ok=left_base,

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
            mr76_obstacle=mr76_right_obstacle,

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

        if avoid_lead_lpf_alpha is None:
            avoid_lead_lpf_alpha = AVOID_LEAD_LPF_ALPHA_DEFAULT

        if avoid_urgent_confirm is None:
            avoid_urgent_confirm = AVOID_URGENT_CONFIRM_DEFAULT

        alpha = _safe_float(avoid_lead_lpf_alpha)
        if alpha is None or alpha <= 0.0 or alpha > 1.0:
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

        if avoid_lateral_enable is None:
            avoid_lateral_enable = AVOID_LATERAL_ENABLE_DEFAULT

        if avoid_lateral_min_speed is None:
            avoid_lateral_min_speed = AVOID_LATERAL_MIN_SPEED_DEFAULT

        if avoid_lateral_confirm is None:
            avoid_lateral_confirm = AVOID_LATERAL_CONFIRM_DEFAULT

        if avoid_lateral_closing_min is None:
            avoid_lateral_closing_min = AVOID_LATERAL_CLOSING_MIN_DEFAULT

        lateral_active_now = False
        lateral_dir_now = 0

        if bool(avoid_lateral_enable):

            v_ok = (
                _safe_float(v_ego) is not None
                and _safe_float(v_ego) >= _safe_float(avoid_lateral_min_speed)
            )

            lateral_closing_v = _safe_float(lateral_closing)

            closing_ok = (
                _safe_float(avoid_lateral_closing_min) <= 0.0
                or (
                    lateral_closing_v is not None
                    and lateral_closing_v >= _safe_float(avoid_lateral_closing_min)
                )
            )

            bl = bool(left_bsm_blocked)
            br = bool(right_bsm_blocked)

            if bl:
                self._lateral_left_frames += 1
            else:
                self._lateral_left_frames = 0

            if br:
                self._lateral_right_frames += 1
            else:
                self._lateral_right_frames = 0

            need = max(1, int(avoid_lateral_confirm))

            if v_ok and closing_ok:
                if self._lateral_left_frames >= need:
                    lateral_active_now = True
                    lateral_dir_now = 1
                elif self._lateral_right_frames >= need:
                    lateral_active_now = True
                    lateral_dir_now = -1
        else:
            self._lateral_left_frames = 0
            self._lateral_right_frames = 0

        self._lateral_active = lateral_active_now
        self._lateral_dir = lateral_dir_now

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

        if risk == RISK_CRITICAL:
            self._critical_frames += 1
        else:
            self._critical_frames = 0

        critical_confirmed = (
            self._critical_frames
            >= CRITICAL_CONFIRM_FRAMES
        )

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
            or
            lateral_active_now
        )

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

                    if bool(emergency_active):
                        brake_request = self._calc_emergency_brake(
                            obstacle_dist=obstacle_dist,
                            obstacle_rel_speed=obstacle_rel_speed,
                            lead_dist=lead_dist,
                            lead_rel_speed=lead_rel_speed,
                            v_ego=v_ego,
                            brake_gain=avoid_brake_gain,
                            brake_max=avoid_brake_max,
                            brake_ttc_ref=avoid_brake_ttc_ref,
                        )
                        hazard = True

                    self._last_reason = (
                        "emergency_no_safe_lane"
                    )

        elif self._mode == "avoiding":

            hazard = True

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

                            if bool(emergency_active):
                                brake_request = self._calc_emergency_brake(
                                    obstacle_dist=obstacle_dist,
                                    obstacle_rel_speed=obstacle_rel_speed,
                                    lead_dist=lead_dist,
                                    lead_rel_speed=lead_rel_speed,
                                    v_ego=v_ego,
                                    brake_gain=avoid_brake_gain,
                                    brake_max=avoid_brake_max,
                                    brake_ttc_ref=avoid_brake_ttc_ref,
                                )
                                hazard = True

                if not emergency_active:

                    if self._esc_abort_since is None:
                        self._esc_abort_since = now

                    elif (
                        (now - self._esc_abort_since)
                        >= AVOID_ESC_ABORT_SEC
                    ):
                        self._mode = "cooldown"

                        self._cooldown_until = (
                            now + AVOID_COOLDOWN_SEC
                        )

                        self._esc_abort_since = None

                        self._out_dir = (
                            LaneChangeDirection.none
                        )

                        lane_request = (
                            LaneChangeDirection.none
                        )

                        self._last_reason = (
                            "escape_aborted_no_risk"
                        )

                else:
                    self._esc_abort_since = None

            else:

                lane_request = self._out_dir

            if (
                self._observed_lane_change
                and
                lc_state == LaneChangeState.off
            ):

                self._observed_lane_change = False

                self._mode = "waiting_clear"

                self._clear_since = now

                self._esc_abort_since = None

                self._last_reason = (
                    "emergency_lc_completed"
                )

        elif self._mode == "waiting_clear":

            hazard = False

            if emergency_active:

                self._mode = "avoiding"

                self._clear_since = None

                hazard = True

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

                    self._esc_abort_since = None

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._last_reason = (
                        "emergency_cleared"
                    )

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

                self._esc_abort_since = None

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._last_reason = (
                    "idle_after_cooldown"
                )

        else:

            self.reset()

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

        if not math.isfinite(float(brake_request)):
            brake_request = 0.0

        brake_request = float(max(0.0, min(float(avoid_brake_max), float(brake_request))))

        self._last_lc_state = lc_state

        return (
            lane_request,
            brake_request,
            hazard,
            lane_offset,
        )

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
            "lateral_active": self._lateral_active,
            "lateral_dir": self._lateral_dir,
        }