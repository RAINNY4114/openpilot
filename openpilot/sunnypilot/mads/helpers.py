"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE


MADS_NO_ACC_MAIN_BUTTON = ("rivian", "tesla")


class MadsSteeringModeOnBrake:
  REMAIN_ACTIVE = 0
  PAUSE = 1
  DISENGAGE = 2


def read_steering_mode_param(params: Params):
  return params.get("MadsSteeringMode", return_default=True)


def set_alternative_experience(CP: structs.CarParams, params: Params):
  if params.get_bool("Mads"):
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALKA
