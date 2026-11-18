"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Custom Personality: user-tunable Driving Personalities. When enabled, each
personality (Aggressive / Standard / Relaxed) gets its own three sliders,
and the active personality - switchable as always, including the steering
wheel distance button - uses its customized values:

- Acceleration: the maximum acceleration from a stop (m/s^2). The rest of the
  speed range blends between tested eco / stock / sport curve anchors to match.
- Jerk Value: how much jerk sunnypilot may use, as a multiplier of stock
  Standard's jerk weighting (the MPC cost factor is its inverse, replacing the
  personality's own factor). Lower responds earlier and smoother; higher
  responds later and firmer.
- Follow distance: the time gap to a lead vehicle, in seconds.

Slider defaults reproduce the stock personalities exactly. All values stay
within the panda safety limits for longitudinal control (ISO 15622 derived:
max 2.0 m/s^2 accel, max -3.5 m/s^2 braking).
"""
import numpy as np

from openpilot.cereal import log
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.sunnypilot.selfdrive.controls.lib._long_params import read_bool, read_int
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_jerk_factor, get_T_FOLLOW
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

PERSONALITY_NAMES = {
  int(log.LongitudinalPersonality.aggressive): "Aggressive",
  int(log.LongitudinalPersonality.standard): "Standard",
  int(log.LongitudinalPersonality.relaxed): "Relaxed",
}

# curve anchors; takeoff values 0.8 (eco), 1.6 (stock), 2.0 (sport)
             # MPH = [0.0,  11,  22,  45,  67,  89]
ANCHOR_A_BP =        [0.,   5., 10., 20., 30., 40.]
ANCHOR_A_ECO =       [0.8, 0.70, 0.55, 0.45, 0.35, 0.25]
ANCHOR_A_SPORT =     [2.0, 2.0, 2.00, 1.70, 1.30, 1.00]

ECO_TAKEOFF = ANCHOR_A_ECO[0]
STOCK_TAKEOFF = 1.6  # stock A_CRUISE_MAX at 0 m/s
SPORT_TAKEOFF = ANCHOR_A_SPORT[0]

# Jerk is exposed as a multiplier of stock Standard's jerk weighting: the MPC cost
# factor is its inverse. 1.0x matches stock Standard/Relaxed; 2.0x matches stock
# Aggressive (cost factor 0.5)
JERK_MULT_MIN, JERK_MULT_MAX = 0.5, 4.0

# Slider params are stored as ints: accel in 0.1 m/s^2 steps, jerk in 0.1x steps,
# follow in 0.05 s steps
ACCEL_PARAM_SCALE = 0.1
JERK_PARAM_SCALE = 0.1
FOLLOW_PARAM_SCALE = 0.05
T_FOLLOW_MIN, T_FOLLOW_MAX = 1.00, 2.50


def _personality_key(personality) -> int:
  # enums read from messages arrive as capnp DynamicEnum, which int() cannot cast
  # directly; its raw attribute holds the integer value
  return int(getattr(personality, "raw", personality))


class CustomLongitudinalTuning:
  def __init__(self):
    self.frame = 0
    self.enabled = False
    self.accel_takeoff: dict[int, float] = {}
    self.jerk_factor: dict[int, float] = {}
    self.t_follow: dict[int, float] = {}
    self.output_max_accel = ACCEL_MAX
    self.output_jerk_factor = 1.0
    self.output_t_follow = get_T_FOLLOW()
    self.read_params()

  # stock slider defaults (from fork params_keys.h), used when a key is absent
  _DEFAULTS = {
    "Aggressive": (16, 20, 25),
    "Standard":   (16, 10, 29),
    "Relaxed":    (16, 10, 35),
  }

  def read_params(self) -> None:
    self.enabled = read_bool("CustomPersonality", False)
    for p, name in PERSONALITY_NAMES.items():
      d_accel, d_jerk, d_follow = self._DEFAULTS.get(name, (16, 10, 29))
      accel = read_int(f"CustomPersonality{name}Accel", d_accel) * ACCEL_PARAM_SCALE
      self.accel_takeoff[p] = float(np.clip(accel, ECO_TAKEOFF, SPORT_TAKEOFF))
      jerk_mult = read_int(f"CustomPersonality{name}JerkMultiplier", d_jerk) * JERK_PARAM_SCALE
      self.jerk_factor[p] = 1.0 / float(np.clip(jerk_mult, JERK_MULT_MIN, JERK_MULT_MAX))
      follow = read_int(f"CustomPersonality{name}Follow", d_follow) * FOLLOW_PARAM_SCALE
      self.t_follow[p] = float(np.clip(follow, T_FOLLOW_MIN, T_FOLLOW_MAX))

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def get_max_accel(self, v_ego: float, default_max: float, personality) -> float:
    if not self.enabled:
      self.output_max_accel = float(default_max)
      return self.output_max_accel

    takeoff = self.accel_takeoff.get(_personality_key(personality), STOCK_TAKEOFF)

    # blend between the curve anchors so the takeoff accel matches the slider,
    # with the stock curve (default_max) as the middle anchor
    a_eco = float(np.interp(v_ego, ANCHOR_A_BP, ANCHOR_A_ECO))
    a_sport = float(np.interp(v_ego, ANCHOR_A_BP, ANCHOR_A_SPORT))
    a_stock = float(default_max)

    if takeoff <= STOCK_TAKEOFF:
      f = (takeoff - ECO_TAKEOFF) / (STOCK_TAKEOFF - ECO_TAKEOFF)
      self.output_max_accel = a_eco + f * (a_stock - a_eco)
    else:
      f = (takeoff - STOCK_TAKEOFF) / (SPORT_TAKEOFF - STOCK_TAKEOFF)
      self.output_max_accel = a_stock + f * (a_sport - a_stock)
    return self.output_max_accel

  def get_decel_jerk_factor(self, personality) -> float:
    base = get_jerk_factor(personality)
    if self.enabled:
      factor = self.jerk_factor.get(_personality_key(personality), base)
      self.output_jerk_factor = factor
      # returned as a multiplier on the personality's own factor, so the total equals the slider
      return factor / base
    self.output_jerk_factor = base
    return 1.0

  def get_t_follow(self, personality) -> float | None:
    if self.enabled:
      t_follow = self.t_follow.get(_personality_key(personality), get_T_FOLLOW(personality))
      self.output_t_follow = t_follow
      return t_follow
    self.output_t_follow = get_T_FOLLOW(personality)
    return None
