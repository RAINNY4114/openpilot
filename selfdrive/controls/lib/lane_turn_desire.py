from cereal import log

from openpilot.common.constants import CV
from openpilot.common.params import Params

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS


class LaneTurnController:
  def __init__(self):
    self.params = Params()
    self.turn_desire = log.Desire.none
    self.lane_turn_value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.param_read_counter = 0
    self.enabled = self.params.get_bool("LaneTurnDesire")

  def read_params(self) -> None:
    self.enabled = self.params.get_bool("LaneTurnDesire")
    value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.lane_turn_value = min(float(LANE_CHANGE_SPEED_MIN), value)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool,
                       v_ego: float) -> None:
    if not self.enabled:
      self.turn_desire = log.Desire.none
      return

    if left_blinker and not right_blinker and v_ego < self.lane_turn_value and not blindspot_left:
      self.turn_desire = log.Desire.turnLeft
    elif right_blinker and not left_blinker and v_ego < self.lane_turn_value and not blindspot_right:
      self.turn_desire = log.Desire.turnRight
    else:
      self.turn_desire = log.Desire.none

  def get_turn_desire(self) -> log.Desire:
    if not self.enabled:
      return log.Desire.none
    return self.turn_desire
