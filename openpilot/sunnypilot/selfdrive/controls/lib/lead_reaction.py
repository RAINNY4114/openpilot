"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lead Reaction Tuning: one global longitudinal-feel toggle that makes sunnypilot
respond quicker on both gas and brakes. When enabled it applies two effects
together:

- Lead reaction scales the lead's acceleration-decay time constant (aLeadTau)
  down, so the MPC projects the lead's braking (or pull-away) as persisting
  longer and engages the response earlier and more progressively instead of late
  and abrupt (5x stock).
- Launch response tightens the v_desired smoothing time constant used at takeoff,
  improving the tracking of the model's pull-off from a stop (4x stock). The
  planner blends this back to stock above a low speed so highway following is
  untouched.

Disabled reproduces stock behavior exactly.
"""
from openpilot.sunnypilot.selfdrive.controls.lib._long_params import read_bool, read_int, read_float
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

# when enabled, lead reaction is 5x stock and launch response is 4x stock
REACTIVITY = 5.0
LAUNCH_QUICKNESS = 4.0
STOCK_V_DESIRED_TAU = 2.0


class LeadReactionTuning:
  def __init__(self):
    self.frame = 0
    self.lead_reaction_factor = 1.0
    self.launch_tau = STOCK_V_DESIRED_TAU
    self.read_params()

  def read_params(self) -> None:
    enabled = read_bool("LeadReactionTuning", False)

    self.lead_reaction_factor = 1.0 / REACTIVITY if enabled else 1.0
    self.launch_tau = STOCK_V_DESIRED_TAU / LAUNCH_QUICKNESS if enabled else STOCK_V_DESIRED_TAU

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def get_lead_reaction_factor(self) -> float:
    # multiplier applied to the lead's aLeadTau in the MPC; 1.0 is stock
    return self.lead_reaction_factor

  def get_launch_tau(self) -> float:
    # target v_desired smoothing time constant at takeoff; stock is STOCK_V_DESIRED_TAU
    return self.launch_tau
