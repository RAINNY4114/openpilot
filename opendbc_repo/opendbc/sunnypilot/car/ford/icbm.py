"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import fordcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTON_SIGNALS = {
  SendButtonState.increase: "CcAslButtnSetIncPress",
  SendButtonState.decrease: "CcAslButtnSetDecPress",
}

BUTTON_INTERVAL = 0.2


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton == SendButtonState.none:
      return can_sends

    button_signal = BUTTON_SIGNALS.get(self.ICBM.sendButton)
    if button_signal is None:
      return can_sends

    if (self.frame - self.last_button_frame) * DT_CTRL <= BUTTON_INTERVAL:
      return can_sends

    for bus in (CAN.camera, CAN.main):
      can_sends.append(fordcan.create_button_msg(packer, bus, CS.buttons_stock_values, icbm_button=button_signal))
    self.last_button_frame = self.frame

    return can_sends
