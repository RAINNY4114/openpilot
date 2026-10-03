from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# ============================================================================
# [AUTO_OVERTAKE_WIRING]
#
# AutoOvertakeHelper (selfdrive/controls/lib/auto_overtake.py, driven from
# sunnypilot/modeld_v2/modeld.py) hands DesireHelper the LaneChangeDirection it
# wants to move to.  It is treated like a driver-issued turn signal, with three
# deliberate differences:
#
#   1. the manoeuvre only starts after AUTO_LC_CONFIRM_DELAY_SEC seconds.  The
#      turn signal is already commanded during that window (controlsd derives
#      CC.left/rightBlinker from laneChangeState), so the driver always gets a
#      visible, cancellable confirmation period.
#   2. the turn signal that WE commanded must not be mistaken for a driver
#      input.  controlsd turns the signal on from laneChangeState, so without
#      this the state machine would feed itself and the confirmation window
#      would be meaningless.  See `driver_blinker` below.
#   3. the ALC nudge timer must never start the manoeuvre early -- only the
#      confirmation window (or the driver) can.
# ============================================================================
AUTO_LC_CONFIRM_DELAY_SEC = 3.0

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    # [AUTO_OVERTAKE_WIRING] automatic lane-change request bookkeeping
    self.prev_auto_request = False
    self._auto_requested = False
    self._auto_dir = LaneChangeDirection.none
    self._auto_confirm_timer = 0.0
    self._auto_brake_blocked = False
    self._auto_blinker_release = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _clear_auto(self):
    # [AUTO_OVERTAKE_WIRING]
    self._auto_requested = False
    self._auto_dir = LaneChangeDirection.none
    self._auto_confirm_timer = 0.0
    self._auto_brake_blocked = False

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             auto_lane_change_direction=None, auto_confirm_delay_sec=None):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    # ------------------------------------------------------------------
    # [AUTO_OVERTAKE_WIRING] resolve the automatic lane-change request
    # ------------------------------------------------------------------
    auto_dir = auto_lane_change_direction
    blinker_dir = LaneChangeDirection.none
    if carstate.leftBlinker:
      blinker_dir = LaneChangeDirection.left
    elif carstate.rightBlinker:
      blinker_dir = LaneChangeDirection.right

    # `was_auto` is the previous frame's decision: it tells us whether the turn
    # signal we are looking at right now is the one we commanded ourselves.
    was_auto = self._auto_requested
    auto_owns_blinker = was_auto and blinker_dir == self._auto_dir
    driver_blinker = one_blinker and not auto_owns_blinker

    # The request survives the turn signal that we ourselves commanded
    # (blinker_dir == auto_dir), but dies as soon as the driver signals the
    # opposite direction.
    auto_request = (auto_dir in (LaneChangeDirection.left, LaneChangeDirection.right)
                    and blinker_dir in (LaneChangeDirection.none, auto_dir))

    confirm_delay = AUTO_LC_CONFIRM_DELAY_SEC if auto_confirm_delay_sec is None else float(auto_confirm_delay_sec)
    if confirm_delay < 0.0:
      confirm_delay = 0.0

    blinker_rising = driver_blinker and not self.prev_one_blinker
    auto_rising = auto_request and not self.prev_auto_request

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self._clear_auto()
    else:
      if self.lane_change_state == LaneChangeState.off and not self._auto_blinker_release \
         and (blinker_rising or auto_rising) and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # A driver blinker always outranks an automatic request.
        self._auto_requested = bool(auto_rising and not blinker_rising)
        self._auto_dir = auto_dir if self._auto_requested else LaneChangeDirection.none
        self._auto_confirm_timer = 0.0
        self._auto_brake_blocked = bool(self._auto_requested and carstate.brakePressed)
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self._auto_dir if self._auto_requested else self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if was_auto:
          if auto_request:
            # Update lane change direction
            self.lane_change_direction = self._auto_dir
          elif driver_blinker:
            # The driver took over the manoeuvre: keep it, but driven by him.
            self._clear_auto()
          else:
            # Request withdrawn and nothing else is holding the lane change.
            self._clear_auto()
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_timer = 0.0

        if self.lane_change_state == LaneChangeState.preLaneChange and not self._auto_requested:
          # Update lane change direction (driver-driven lane change)
          self.lane_change_direction = self.get_lane_change_direction(carstate)

        if self.lane_change_state == LaneChangeState.preLaneChange:
          torque_applied = carstate.steeringPressed and \
                           ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

          blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                                ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

          self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

          # [AUTO_OVERTAKE_WIRING] confirmation window.  Only frames with a
          # clear target lane count towards the delay; brake latches the
          # request off for good, a blind spot only pauses it.
          if self._auto_requested:
            if carstate.brakePressed:
              self._auto_brake_blocked = True
            if not blindspot_detected:
              self._auto_confirm_timer += DT_MDL
            if not self._auto_brake_blocked and not blindspot_detected and self._auto_confirm_timer >= confirm_delay:
              torque_applied = True

          # An automatic request is kept alive ONLY by the module -- never by
          # the turn signal we commanded ourselves.
          alive = True if self._auto_requested else (driver_blinker or auto_request)
          # ...and the ALC nudge timer may not start an automatic request early.
          alc_allowed = self.alc.auto_lane_change_allowed and not self._auto_requested

          if not alive or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_timer = 0.0
            self._clear_auto()
          elif (torque_applied or alc_allowed) and not blindspot_detected:
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          was_auto_here = self._auto_requested
          self._clear_auto()
          # An automatic lane change is exactly one lane: do not let the turn
          # signal that we commanded keep the state machine alive.
          if driver_blinker and not was_auto_here:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    # [AUTO_OVERTAKE_WIRING] after an automatic lane change our own turn signal
    # is still physically on for a moment; ignore it until it has cleared, or
    # the state machine would immediately start a second lane change.
    if was_auto and one_blinker:
      self._auto_blinker_release = True
    if not one_blinker:
      self._auto_blinker_release = False

    self.prev_one_blinker = driver_blinker and lateral_active
    self.prev_auto_request = auto_request and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
