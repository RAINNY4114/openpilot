import time

from cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# Disable auto-lane-change avoidance at low speeds to reduce churn in city traffic.
AUTO_AVOID_MIN_SPEED = 30 * CV.KPH_TO_MS
SLOWDOWN_BEFORE_LC_SEC = 1.0
OBSTACLE_CLEAR_DELAY_SEC = 2.0
AVOID_COOLDOWN_SEC = 8.0
CLEAR_LANE_STABLE_SEC = 0.6


class AutoAvoidanceHelper:
  def __init__(self):
    self._mode: str = "idle"  # idle|slowing|changing_out|waiting_return|changing_back
    self._out_dir = LaneChangeDirection.none
    self._return_dir = LaneChangeDirection.none
    self._cooldown_until = 0.0
    self._clear_since: float | None = None
    self._slow_since: float | None = None
    self._last_lc_state = LaneChangeState.off
    self._left_ok_since: float | None = None
    self._right_ok_since: float | None = None

  @staticmethod
  def _pick_out_direction(left_ok: bool, right_ok: bool, is_rhd: bool) -> LaneChangeDirection:
    if left_ok and right_ok:
      return LaneChangeDirection.right if is_rhd else LaneChangeDirection.left
    if left_ok:
      return LaneChangeDirection.left
    if right_ok:
      return LaneChangeDirection.right
    return LaneChangeDirection.none

  @staticmethod
  def _opposite(direction: LaneChangeDirection) -> LaneChangeDirection:
    if direction == LaneChangeDirection.left:
      return LaneChangeDirection.right
    if direction == LaneChangeDirection.right:
      return LaneChangeDirection.left
    return LaneChangeDirection.none

  def reset(self) -> None:
    self._mode = "idle"
    self._out_dir = LaneChangeDirection.none
    self._return_dir = LaneChangeDirection.none
    self._cooldown_until = 0.0
    self._clear_since = None
    self._slow_since = None
    self._last_lc_state = LaneChangeState.off
    self._left_ok_since = None
    self._right_ok_since = None

  @staticmethod
  def _stable_ok(ok: bool, ok_since: float | None, now: float, stable_sec: float) -> tuple[float | None, bool]:
    if ok:
      if ok_since is None:
        ok_since = now
      stable = (now - ok_since) >= stable_sec
      return ok_since, stable
    return None, False

  def update(self, *, enabled: bool, obstacle_in_path: bool, lc_state: LaneChangeState, v_ego: float,
             left_ok: bool, right_ok: bool, is_rhd: bool, manual_blinker: bool, bsm_available: bool) -> LaneChangeDirection:
    now = time.monotonic()
    request = LaneChangeDirection.none

    # Gate hard requirements first.
    if not enabled or not bsm_available or v_ego < AUTO_AVOID_MIN_SPEED:
      self.reset()
      self._last_lc_state = lc_state
      return request

    # Don't start avoidance while the driver is explicitly signaling.
    #
    # NOTE: Some platforms inject/echo turn-signal state during auto lane changes (for exterior blinkers),
    # which would otherwise reset this helper mid-maneuver and prevent the "return to lane" behavior.
    if manual_blinker and self._mode == "idle":
      self.reset()
      self._last_lc_state = lc_state
      return request

    self._left_ok_since, left_stable = self._stable_ok(left_ok, self._left_ok_since, now, CLEAR_LANE_STABLE_SEC)
    self._right_ok_since, right_stable = self._stable_ok(right_ok, self._right_ok_since, now, CLEAR_LANE_STABLE_SEC)

    # Detect lane-change completion (finishing -> off)
    lc_finished = (self._last_lc_state == LaneChangeState.laneChangeFinishing and lc_state == LaneChangeState.off)

    if self._mode == "idle":
      self._out_dir = LaneChangeDirection.none
      self._return_dir = LaneChangeDirection.none
      self._clear_since = None
      self._slow_since = None
      if obstacle_in_path and now >= self._cooldown_until:
        self._mode = "slowing"
        self._slow_since = now

    elif self._mode == "slowing":
      if not obstacle_in_path:
        self.reset()
      else:
        if self._out_dir == LaneChangeDirection.none:
          self._out_dir = self._pick_out_direction(left_stable, right_stable, is_rhd)
        else:
          # If the chosen side becomes blocked, opportunistically switch to the other side (once) if stable.
          if self._out_dir == LaneChangeDirection.left and (not left_stable) and right_stable:
            self._out_dir = LaneChangeDirection.right
          elif self._out_dir == LaneChangeDirection.right and (not right_stable) and left_stable:
            self._out_dir = LaneChangeDirection.left
        if self._slow_since is not None and (now - self._slow_since) >= SLOWDOWN_BEFORE_LC_SEC:
          dir_ok = (self._out_dir == LaneChangeDirection.left and left_stable) or (self._out_dir == LaneChangeDirection.right and right_stable)
          if self._out_dir != LaneChangeDirection.none and dir_ok:
            self._mode = "changing_out"
            request = self._out_dir

    elif self._mode == "changing_out":
      # Hold request until the lane-change state machine starts moving (preLaneChange -> laneChangeStarting)
      dir_ok = (self._out_dir == LaneChangeDirection.left and left_stable) or (self._out_dir == LaneChangeDirection.right and right_stable)
      if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and dir_ok:
        request = self._out_dir
      # If the request is canceled before starting (preLaneChange -> off), re-enter slowing and try again.
      if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
        if obstacle_in_path:
          self._mode = "slowing"
          self._slow_since = now
          self._out_dir = LaneChangeDirection.none
        else:
          self.reset()
      if lc_finished:
        self._mode = "waiting_return"
        self._clear_since = None

    elif self._mode == "waiting_return":
      # Wait until the obstacle is clear for a while, then return to the original lane
      if obstacle_in_path:
        self._clear_since = None
      else:
        if self._clear_since is None:
          self._clear_since = now
        if (now - self._clear_since) >= OBSTACLE_CLEAR_DELAY_SEC:
          self._return_dir = self._opposite(self._out_dir)
          return_ok = (self._return_dir == LaneChangeDirection.left and left_stable) or (self._return_dir == LaneChangeDirection.right and right_stable)
          if self._return_dir != LaneChangeDirection.none and return_ok:
            self._mode = "changing_back"
            request = self._return_dir

    elif self._mode == "changing_back":
      dir_ok = (self._return_dir == LaneChangeDirection.left and left_stable) or (self._return_dir == LaneChangeDirection.right and right_stable)
      if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and dir_ok:
        request = self._return_dir
      # If return is canceled (preLaneChange -> off), go back to waiting_return and retry when stable again.
      if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
        self._mode = "waiting_return"
        self._clear_since = None
      if lc_finished:
        self._mode = "idle"
        self._cooldown_until = now + AVOID_COOLDOWN_SEC

    else:
      self.reset()

    self._last_lc_state = lc_state
    return request
