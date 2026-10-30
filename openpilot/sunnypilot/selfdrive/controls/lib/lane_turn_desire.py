"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom

from openpilot.common.constants import CV
from openpilot.common.params import Params

TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS


class LaneTurnController:
  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.turn_direction = TurnDirection.none
    self.params = Params()
    self.lane_turn_value = self._clamp_lane_turn_value(self.params.get("LaneTurnValue", return_default=True))
    self.param_read_counter = 0
    self.enabled = self.params.get_bool("LaneTurnDesire")

  @staticmethod
  def _clamp_lane_turn_value(value_mph) -> float:
    """`LaneTurnValue` 单位是 mph；转向意图的速度阈值不得超过 `LANE_CHANGE_SPEED_MIN`。

    `__init__` 与 `read_params()` 必须走同一条路径，否则两条路径会漂移。
    目前 `DesireHelper.update()` 里 `update_params()` 排在 `update_lane_turn()` 之前，
    且 `param_read_counter` 初值为 0 ⇒ 首次调用就会 `read_params()`，
    所以未截断的初值实际从未参与判断。但那只是调用顺序的巧合，
    一旦顺序变动就会在任意车速下误判转向意图 —— 这里把巧合变成不变量。
    """
    return min(float(LANE_CHANGE_SPEED_MIN), float(value_mph) * CV.MPH_TO_MS)

  def read_params(self):
    self.enabled = self.params.get_bool("LaneTurnDesire")
    self.lane_turn_value = self._clamp_lane_turn_value(self.params.get("LaneTurnValue", return_default=True))

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool, v_ego: float) -> None:
    if left_blinker and not right_blinker and v_ego < self.lane_turn_value and not blindspot_left:
      self.turn_direction = TurnDirection.turnLeft
    elif right_blinker and not left_blinker and v_ego < self.lane_turn_value and not blindspot_right:
      self.turn_direction = TurnDirection.turnRight
    else:
      self.turn_direction = TurnDirection.none

  def get_turn_direction(self):
    if not self.enabled:
      return TurnDirection.none
    return self.turn_direction
