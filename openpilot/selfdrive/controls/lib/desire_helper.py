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

BSD_DETECTED_HOLD_SEC = 1.5
BSD_EXIST_THRESHOLD = 3
SIDE_LEAD_DANGER_DIST = 5.0

AUTO_LC_CONFIRM_DELAY_SEC = 3.0

class ExistCounter:
  def __init__(self, threshold: int = BSD_EXIST_THRESHOLD):
    self.threshold = threshold
    self.cnt = 0
    self.state = False

  def update(self, detected: bool) -> None:
    if detected:
      self.cnt = min(self.cnt + 1, self.threshold)
    else:
      self.cnt = max(self.cnt - 1, 0)
    self.state = self.cnt >= self.threshold

  def reset(self) -> None:
    self.cnt = 0
    self.state = False

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class _AutoBlinkerProxy:
  __slots__ = ("_cs", "leftBlinker", "rightBlinker")

  def __init__(self, cs, direction):
    self._cs = cs
    self.leftBlinker = direction == LaneChangeDirection.left
    self.rightBlinker = direction == LaneChangeDirection.right

  def __getattr__(self, name):
    return getattr(self._cs, name)

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
    self.left_bsd_counter = ExistCounter()
    self.right_bsd_counter = ExistCounter()
    self.bsd_hold_timer = 0.0
    self.left_bsd_active = False
    self.right_bsd_active = False
    self._auto_active = False
    self._auto_confirm_timer = 0.0
    self._auto_release = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             auto_lane_change_direction=None, auto_confirm_delay_sec=None):
    raw_cs = carstate
    driver_blinker = bool(raw_cs.leftBlinker or raw_cs.rightBlinker)
    auto_dir = auto_lane_change_direction
    if auto_dir is None or auto_dir == LaneChangeDirection.none:
      self._auto_release = False
    auto_request = (auto_dir in (LaneChangeDirection.left, LaneChangeDirection.right)
                    and not driver_blinker and not self._auto_release)
    was_auto = self._auto_active
    self._auto_active = auto_request
    if auto_request:
      self._auto_confirm_timer += DT_MDL
    else:
      self._auto_confirm_timer = 0.0
    confirm_delay = AUTO_LC_CONFIRM_DELAY_SEC if auto_confirm_delay_sec is None else max(0.0, float(auto_confirm_delay_sec))
    if auto_request:
      try:
        carstate = _AutoBlinkerProxy(raw_cs, auto_dir)
      except Exception:
        carstate = raw_cs
        self._auto_active = False
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    self.lane_turn_controller.update_lane_turn(blindspot_left=raw_cs.leftBlindspot, blindspot_right=raw_cs.rightBlindspot,
                                               left_blinker=raw_cs.leftBlinker, right_blinker=raw_cs.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        raw_left_bsd = carstate.leftBlindspot
        raw_right_bsd = carstate.rightBlindspot
        self.left_bsd_counter.update(raw_left_bsd)
        self.right_bsd_counter.update(raw_right_bsd)
        any_bsd_raw = raw_left_bsd or raw_right_bsd
        if any_bsd_raw:
          self.bsd_hold_timer = BSD_DETECTED_HOLD_SEC
        else:
          self.bsd_hold_timer = max(0.0, self.bsd_hold_timer - DT_MDL)
        bsd_holding = self.bsd_hold_timer > 0.0
        self.left_bsd_active = self.left_bsd_counter.state or (raw_left_bsd and bsd_holding)
        self.right_bsd_active = self.right_bsd_counter.state or (raw_right_bsd and bsd_holding)
        blindspot_detected = ((self.left_bsd_active and self.lane_change_direction == LaneChangeDirection.left) or
                              (self.right_bsd_active and self.lane_change_direction == LaneChangeDirection.right))

        edge_detected = ((left_edge_detected and self.lane_change_direction == LaneChangeDirection.left) or
                         (right_edge_detected and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, edge_detected, carstate.brakePressed, torque_applied)

        auto_ready = (not self._auto_active) or \
                     (self._auto_confirm_timer >= confirm_delay and not raw_cs.brakePressed)
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or (self.alc.auto_lane_change_allowed and auto_ready)) and not self.alc.lane_change_blocked:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if was_auto:
            self._auto_release = True
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

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
