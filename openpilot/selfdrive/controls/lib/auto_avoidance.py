import time
from cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# Constants for lane-change avoidance behavior
AUTO_AVOID_MIN_SPEED = 30 * CV.KPH_TO_MS
SLOWDOWN_BEFORE_LC_SEC = 1.0
OBSTACLE_CLEAR_DELAY_SEC = 2.0
RETURN_MIN_TIME_AFTER_OUT_SEC = 3.0
AVOID_COOLDOWN_SEC = 8.0
CLEAR_LANE_STABLE_SEC = 0.6
AUTO_LC_REQUEST_TIMEOUT_SEC = 12.0

class AutoAvoidanceHelper:
    def __init__(self):
        # Initial state and attributes
        self._mode = "idle"  # Modes: idle | slowing | changing_out | waiting_return | changing_back
        self._out_dir = LaneChangeDirection.none
        self._return_dir = LaneChangeDirection.none
        self._cooldown_until = 0.0
        self._clear_since = None
        self._slow_since = None
        self._out_finished_t = None
        self._last_lc_state = LaneChangeState.off
        self._left_ok_since = None
        self._right_ok_since = None
        self._request_since = None

    @staticmethod
    def _pick_out_direction(left_ok, right_ok, is_rhd, prefer_dir):
        # Determine the direction for lane change
        if prefer_dir == LaneChangeDirection.left and left_ok:
            return LaneChangeDirection.left
        if prefer_dir == LaneChangeDirection.right and right_ok:
            return LaneChangeDirection.right
        if left_ok and right_ok:
            return LaneChangeDirection.right if is_rhd else LaneChangeDirection.left
        if left_ok:
            return LaneChangeDirection.left
        if right_ok:
            return LaneChangeDirection.right
        return LaneChangeDirection.none

    @staticmethod
    def _opposite(direction):
        # Get the opposite direction
        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right
        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left
        return LaneChangeDirection.none

    def reset(self):
        # Reset the avoidance system to idle state
        self._mode = "idle"
        self._out_dir = LaneChangeDirection.none
        self._return_dir = LaneChangeDirection.none
        self._cooldown_until = 0.0
        self._clear_since = None
        self._slow_since = None
        self._out_finished_t = None
        self._last_lc_state = LaneChangeState.off
        self._left_ok_since = None
        self._right_ok_since = None
        self._request_since = None

    @property
    def active(self):
        return self._mode != "idle"

    def abort_pending_lane_change(self):
        now = time.monotonic()
        if self._mode in ("slowing", "changing_out"):
            self.reset()
            self._cooldown_until = now + AVOID_COOLDOWN_SEC
        elif self._mode == "changing_back":
            self._mode = "waiting_return"
            self._clear_since = None
            self._request_since = None

    def _abort_with_cooldown(self, now):
        self.reset()
        self._cooldown_until = now + AVOID_COOLDOWN_SEC

    def _request_timed_out(self, now, lc_state):
        return (
            self._request_since is not None and
            lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and
            (now - self._request_since) >= AUTO_LC_REQUEST_TIMEOUT_SEC
        )

    @staticmethod
    def _stable_ok(ok, ok_since, now, stable_sec):
        if ok:
            if ok_since is None:
                ok_since = now
            stable = (now - ok_since) >= stable_sec
            return ok_since, stable
        return None, False

    def update(self, *, enabled, obstacle_in_path, lc_state, v_ego, left_ok, right_ok, is_rhd, manual_blinker,
               bsm_available, prefer_dir):
        now = time.monotonic()
        request = LaneChangeDirection.none

        # Ensure minimum requirements for avoidance to be enabled
        if not enabled or not bsm_available or v_ego < AUTO_AVOID_MIN_SPEED:
            self.reset()
            self._last_lc_state = lc_state
            return request

        # Don't start avoidance while the driver is manually signaling
        if manual_blinker and self._mode == "idle":
            self.reset()
            self._last_lc_state = lc_state
            return request

        # Stability checks for lane availability
        self._left_ok_since, left_stable = self._stable_ok(left_ok, self._left_ok_since, now, CLEAR_LANE_STABLE_SEC)
        self._right_ok_since, right_stable = self._stable_ok(right_ok, self._right_ok_since, now, CLEAR_LANE_STABLE_SEC)

        # Detect lane-change completion (finishing -> off)
        lc_finished = (self._last_lc_state == LaneChangeState.laneChangeFinishing and lc_state == LaneChangeState.off)

        if self._mode == "idle":
            self._out_dir = LaneChangeDirection.none
            self._return_dir = LaneChangeDirection.none
            self._clear_since = None
            self._slow_since = None
            self._out_finished_t = None
            if obstacle_in_path and lc_state == LaneChangeState.off and now >= self._cooldown_until:
                self._mode = "slowing"
                self._slow_since = now

        elif self._mode == "slowing":
            if not obstacle_in_path:
                self.reset()
            else:
                self._out_finished_t = None
                if self._out_dir == LaneChangeDirection.none:
                    self._out_dir = self._pick_out_direction(left_stable, right_stable, is_rhd, prefer_dir=prefer_dir)
                else:
                    if self._out_dir == LaneChangeDirection.left and (not left_stable) and right_stable:
                        self._out_dir = LaneChangeDirection.right
                    elif self._out_dir == LaneChangeDirection.right and (not right_stable) and left_stable:
                        self._out_dir = LaneChangeDirection.left
                if self._slow_since is not None and (now - self._slow_since) >= SLOWDOWN_BEFORE_LC_SEC:
                    dir_ok = (self._out_dir == LaneChangeDirection.left and left_stable) or (self._out_dir == LaneChangeDirection.right and right_stable)
                    if self._out_dir != LaneChangeDirection.none and dir_ok:
                        self._mode = "changing_out"
                        self._request_since = now
                        request = self._out_dir

        elif self._mode == "changing_out":
            # Hold request until the lane-change state machine starts moving (preLaneChange -> laneChangeStarting)
            dir_ok = (self._out_dir == LaneChangeDirection.left and left_stable) or (self._out_dir == LaneChangeDirection.right and right_stable)
            if not obstacle_in_path and lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
                self.reset()
                self._last_lc_state = lc_state
                return request
            if self._request_timed_out(now, lc_state):
                self._abort_with_cooldown(now)
                self._last_lc_state = lc_state
                return request
            if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and dir_ok:
                request = self._out_dir
            # If the request is canceled before starting, abort and cool down instead of flickering requests.
            if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
                if obstacle_in_path:
                    self._abort_with_cooldown(now)
                else:
                    self.reset()
            if lc_finished:
                self._mode = "waiting_return"
                self._clear_since = None
                self._out_finished_t = now
                self._request_since = None

        elif self._mode == "waiting_return":
            # Wait for obstacle clearance and return to the original lane
            if obstacle_in_path:
                self._clear_since = None
            else:
                if self._clear_since is None:
                    self._clear_since = now
                out_ok = self._out_finished_t is None or (now - self._out_finished_t) >= RETURN_MIN_TIME_AFTER_OUT_SEC
                if out_ok and (now - self._clear_since) >= OBSTACLE_CLEAR_DELAY_SEC:
                    self._return_dir = self._opposite(self._out_dir)
                    return_ok = (self._return_dir == LaneChangeDirection.left and left_stable) or (self._return_dir == LaneChangeDirection.right and right_stable)
                    if self._return_dir != LaneChangeDirection.none and return_ok:
                        self._mode = "changing_back"
                        self._request_since = now
                        request = self._return_dir

        elif self._mode == "changing_back":
            dir_ok = (self._return_dir == LaneChangeDirection.left and left_stable) or (self._return_dir == LaneChangeDirection.right and right_stable)
            if self._request_timed_out(now, lc_state):
                self._mode = "waiting_return"
                self._clear_since = None
                self._request_since = None
                self._last_lc_state = lc_state
                return request
            if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and dir_ok:
                request = self._return_dir
            # If return is canceled, go back to waiting_return and retry when stable again.
            if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
                self._mode = "waiting_return"
                self._clear_since = None
                self._request_since = None
            if lc_finished:
                self._mode = "idle"
                self._cooldown_until = now + AVOID_COOLDOWN_SEC
                self._request_since = None

        else:
            self.reset()

        self._last_lc_state = lc_state
        return request
