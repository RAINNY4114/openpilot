import time

from cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# Highway auto-overtake (experimental).
# This only injects a lane-change desire; longitudinal control remains unchanged (ACC/openpilot).
OVERTAKE_MIN_SPEED = 80 * CV.KPH_TO_MS
OVERTAKE_MIN_CRUISE_SPEED = 90 * CV.KPH_TO_MS
OVERTAKE_SPEED_DELTA = 15 * CV.KPH_TO_MS
OVERTAKE_HEADWAY_MAX_S = 2.8  # only consider overtake when we're close to being speed-limited
OVERTAKE_LEAD_STABLE_SEC = 1.0

PREPARE_BEFORE_LC_SEC = 0.6
RETURN_CLEAR_DELAY_SEC = 5.0
RETURN_MIN_TIME_AFTER_OUT_SEC = 4.0
OVERTAKE_COOLDOWN_SEC = 20.0
CLEAR_LANE_STABLE_SEC = 0.6

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2


class AutoOvertakeHelper:
  def __init__(self):
    self._mode: str = "idle"  # idle|preparing|changing_out|holding|waiting_return|changing_back
    self._out_dir = LaneChangeDirection.none
    self._return_dir = LaneChangeDirection.none
    self._cooldown_until = 0.0

    self._need_since: float | None = None
    self._prepare_since: float | None = None
    self._clear_since: float | None = None
    self._out_finished_t: float | None = None

    self._last_lc_state = LaneChangeState.off
    self._left_ok_since: float | None = None
    self._right_ok_since: float | None = None

  def reset(self) -> None:
    self._mode = "idle"
    self._out_dir = LaneChangeDirection.none
    self._return_dir = LaneChangeDirection.none
    self._need_since = None
    self._prepare_since = None
    self._clear_since = None
    self._out_finished_t = None
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

  @staticmethod
  def _opposite(direction: LaneChangeDirection) -> LaneChangeDirection:
    if direction == LaneChangeDirection.left:
      return LaneChangeDirection.right
    if direction == LaneChangeDirection.right:
      return LaneChangeDirection.left
    return LaneChangeDirection.none

  def _update_need_overtake(self, *, now: float, lead_present: bool, lead_d: float, v_lead: float,
                            v_ego: float, v_cruise: float) -> bool:
    need_raw = False
    if lead_present and lead_d > 0.0 and v_cruise >= OVERTAKE_MIN_CRUISE_SPEED and v_ego >= OVERTAKE_MIN_SPEED:
      headway = float(lead_d / max(v_ego, 0.1))
      if (v_cruise - v_lead) >= OVERTAKE_SPEED_DELTA and headway <= OVERTAKE_HEADWAY_MAX_S:
        need_raw = True

    if need_raw:
      if self._need_since is None:
        self._need_since = now
      return (now - self._need_since) >= OVERTAKE_LEAD_STABLE_SEC

    self._need_since = None
    return False

  def update(self, *, enabled: bool, lc_state: LaneChangeState, v_ego: float, v_cruise: float,
             lead_present: bool, lead_d: float, v_lead: float,
             left_ok: bool, right_ok: bool, is_rhd: bool, manual_blinker: bool, bsm_available: bool,
             lane_preference: int = LANE_PREF_AUTO) -> LaneChangeDirection:
    now = time.monotonic()
    request = LaneChangeDirection.none

    lane_preference = lane_preference if lane_preference in (LANE_PREF_AUTO, LANE_PREF_KEEP_LEFT, LANE_PREF_KEEP_RIGHT) else LANE_PREF_AUTO

    if (not enabled) or (not bsm_available):
      self.reset()
      self._last_lc_state = lc_state
      return request

    if v_ego < OVERTAKE_MIN_SPEED or v_cruise < OVERTAKE_MIN_CRUISE_SPEED:
      self.reset()
      self._last_lc_state = lc_state
      return request

    # Don't start overtake while the driver is explicitly signaling.
    if manual_blinker and self._mode == "idle":
      self.reset()
      self._last_lc_state = lc_state
      return request

    self._left_ok_since, left_stable = self._stable_ok(left_ok, self._left_ok_since, now, CLEAR_LANE_STABLE_SEC)
    self._right_ok_since, right_stable = self._stable_ok(right_ok, self._right_ok_since, now, CLEAR_LANE_STABLE_SEC)

    pass_dir = LaneChangeDirection.right if is_rhd else LaneChangeDirection.left
    pass_ok = right_stable if is_rhd else left_stable
    return_dir = self._opposite(pass_dir)
    return_ok = left_stable if is_rhd else right_stable

    # When set, "keep left/right" means: after a pass, prefer staying on that physical side (no auto return).
    stay_in_pass_lane = (lane_preference == LANE_PREF_KEEP_LEFT and not is_rhd) or (lane_preference == LANE_PREF_KEEP_RIGHT and is_rhd)

    need_overtake = self._update_need_overtake(
      now=now,
      lead_present=lead_present,
      lead_d=lead_d,
      v_lead=v_lead,
      v_ego=v_ego,
      v_cruise=v_cruise,
    )

    # Detect lane-change completion (finishing -> off)
    lc_finished = (self._last_lc_state == LaneChangeState.laneChangeFinishing and lc_state == LaneChangeState.off)

    if self._mode == "idle":
      self._out_dir = LaneChangeDirection.none
      self._return_dir = LaneChangeDirection.none
      self._prepare_since = None
      self._clear_since = None
      self._out_finished_t = None

      if need_overtake and pass_ok and now >= self._cooldown_until:
        self._mode = "preparing"
        self._prepare_since = now
        self._out_dir = pass_dir

    elif self._mode == "preparing":
      if not need_overtake:
        self.reset()
      else:
        # wait a short delay to avoid flicker-based triggers
        if self._prepare_since is not None and (now - self._prepare_since) >= PREPARE_BEFORE_LC_SEC:
          if pass_ok:
            self._mode = "changing_out"
            request = self._out_dir

    elif self._mode == "changing_out":
      if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and pass_ok:
        request = self._out_dir
      # If the request is canceled before starting (preLaneChange -> off), retry after a short prepare delay.
      if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
        if need_overtake:
          self._mode = "preparing"
          self._prepare_since = now
        else:
          self.reset()
      if lc_finished:
        self._mode = "holding" if stay_in_pass_lane else "waiting_return"
        self._out_finished_t = now
        self._clear_since = None

    elif self._mode == "holding":
      # Stay in the passing lane until user preference changes away from the passing side.
      if not stay_in_pass_lane:
        self._mode = "waiting_return"
        self._clear_since = None

    elif self._mode == "waiting_return":
      # Wait until not "need_overtake" for a while, then return to original lane.
      if need_overtake:
        self._clear_since = None
      else:
        if self._clear_since is None:
          self._clear_since = now
        out_ok = self._out_finished_t is None or (now - self._out_finished_t) >= RETURN_MIN_TIME_AFTER_OUT_SEC
        if out_ok and (now - self._clear_since) >= RETURN_CLEAR_DELAY_SEC and return_ok:
          self._mode = "changing_back"
          self._return_dir = return_dir
          request = self._return_dir

    elif self._mode == "changing_back":
      if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and return_ok:
        request = self._return_dir
      # If return is canceled (preLaneChange -> off), go back to waiting_return and retry when stable again.
      if self._last_lc_state == LaneChangeState.preLaneChange and lc_state == LaneChangeState.off and not lc_finished:
        self._mode = "waiting_return"
        self._clear_since = None
      if lc_finished:
        self._mode = "idle"
        self._cooldown_until = now + OVERTAKE_COOLDOWN_SEC

    else:
      self.reset()

    self._last_lc_state = lc_state
    return request
