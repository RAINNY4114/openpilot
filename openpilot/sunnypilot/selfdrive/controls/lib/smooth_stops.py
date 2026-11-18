"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Smooth Stops: exponential-landing stops.

A human driver feathers the brake as the car settles; openpilot can carry
1+ m/s^2 of deceleration all the way into standstill, rocking the car on its
suspension. A velocity profile that "comes in for a smooth landing" is
exponential decay, produced by capping deceleration in proportion to speed:

  a_allowed = -(k * v + c)

k sets the landing's time constant and is the user-facing smoothness level;
c keeps the stop from asymptoting forever and is the residual deceleration at
standstill (the size of the remaining head-nod). Near the activation speed the
cap computes to ~full braking, so the approach can be as firm as the situation
demands and blends seamlessly into the landing.

Only applied when the planner's own trajectory comes to a stop, so braking for
corners and slowdowns is untouched, and bypassed whenever a lead is close so
braking authority is never reduced when the gap demands it.
"""
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.sunnypilot.selfdrive.controls.lib._long_params import read_bool
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

ACTIVATION_SPEED = 3.5  # m/s, cap is computed below this; near no-op at the top end
STOP_INTENT_SPEED = 0.5  # m/s, plan must reach below this to count as a stop
MIN_LEAD_DISTANCE = 5.0  # m, full braking authority when a lead is closer than this
LEAD_STOP_MARGIN = 4.0  # m, never block the braking required to stop this far behind the lead

SMOOTHNESS_K = 0.70  # [1/s], landing time constant (Softer)
SMOOTHNESS_C = 0.30  # [m/s^2], residual decel at standstill (Softer)



# settle-phase failsafe and smoothing
LINGER_SPEED = 0.5   # m/s (~1.1 mph), absolute backstop: arms clamp-to-stop as last resort
LINGER_TIME = 1.0    # s, how long to crawl before the clamp is allowed to finish the stop

# creep floor: below CREEP_FLOOR_SPEED, nudge a_target to at least -CREEP_FLOOR_DECEL.
# Prevents the car from coasting when the MPC is too gentle to overcome transmission
# creep torque — no sudden clamp, just guaranteed consistent deceleration.
CREEP_FLOOR_SPEED = 1.0   # m/s (~2.2 mph), below this enforce minimum commanded decel
CREEP_FLOOR_DECEL = 0.40  # m/s^2, minimum commanded decel at creep speeds

# progress watchdog: while the cap is limiting braking, the car must keep slowing.
# If speed stops decreasing, release the cap progressively until it does
STALL_TIME = 1.0    # s, no progress for this long starts releasing the cap
STALL_PROGRESS = 0.02  # m/s, minimum speed reduction to count as progress
STALL_RELEASE_RATE = 0.15  # m/s^2 of additional allowed braking per second of stall
SETTLE_SMOOTH_SPEED = 1.5  # m/s, jerk-limit the PID output below this
SETTLE_JERK_LIMIT = 2.5  # m/s^3


def read_smooth_stops_params(params=None) -> bool:
  return read_bool("SmoothStops", False)


class SmoothStops:
  def __init__(self):
    self.frame = 0
    self.enabled = False
    self.active = False
    self._v_min = float("inf")
    self._stall_frames = 0
    self.read_params()

  def _reset_watchdog(self) -> None:
    self._v_min = float("inf")
    self._stall_frames = 0

  def read_params(self) -> None:
    self.enabled = read_smooth_stops_params()

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def apply(self, a_target: float, v_ego: float, lead_one, plan_min_v: float) -> float:
    self.active = False

    if not self.enabled or a_target >= 0. or v_ego > ACTIVATION_SPEED:
      self._reset_watchdog()
      return a_target
    if plan_min_v > STOP_INTENT_SPEED:
      self._reset_watchdog()
      return a_target
    brake_floor = -(SMOOTHNESS_K * v_ego + SMOOTHNESS_C)

    _lead_present = bool(getattr(lead_one, 'present', False) or getattr(lead_one, 'status', False))
    if _lead_present:
      if lead_one.dRel < MIN_LEAD_DISTANCE:
        self._reset_watchdog()
        return a_target
      # enforce the gap: the cap may only soften braking that is genuinely surplus.
      # Never block the deceleration physically required to stop LEAD_STOP_MARGIN
      # behind the lead at the current closing speed - if the lead stops quickly,
      # the cap turns transparent and normal braking lands the stop at the margin
      closing = max(v_ego - lead_one.vLead, 0.0)
      gap_budget = max(lead_one.dRel - LEAD_STOP_MARGIN, 0.5)
      required = (closing ** 2) / (2.0 * gap_budget)
      brake_floor = max(min(brake_floor, -required), ACCEL_MIN)

    # progress watchdog: the cap must never hold speed. If the car has stopped
    # slowing while we limit braking, release the cap until it slows again
    if v_ego < self._v_min - STALL_PROGRESS:
      self._v_min = v_ego
      self._stall_frames = 0
    else:
      self._stall_frames += 1
    stalled_s = max(self._stall_frames * DT_MDL - STALL_TIME, 0.0)
    if stalled_s > 0.0:
      brake_floor = max(brake_floor - STALL_RELEASE_RATE * stalled_s, ACCEL_MIN)

    if a_target < brake_floor:
      self.active = True
      a_target = brake_floor

    # creep floor: if the MPC is too gentle below CREEP_FLOOR_SPEED, ensure at least
    # CREEP_FLOOR_DECEL of braking so the car consistently decelerates to a stop
    # rather than coasting against transmission creep torque.
    if v_ego < CREEP_FLOOR_SPEED:
      creep_limit = -CREEP_FLOOR_DECEL
      if a_target > creep_limit:
        self.active = True
        a_target = creep_limit

    return a_target


class SmoothStopsLongControl:
  """Controls-side companion to SmoothStops, running at 100 Hz in longcontrol.

  Stock longcontrol enters the stopping state while the car is still rolling,
  which has two harsh consequences: the open-loop ramp adds brake pressure all
  the way into standstill, and on cars like HKG the stopping state asserts a
  stop request on the CAN bus, letting the factory brake controller run its own
  clamp procedure while still moving - whichever engages first wins, making the
  end of the stop a per-stop lottery.

  Defer the stopping state until the car is actually stationary. While rolling,
  the PID stays in charge - closed loop on measured acceleration - tracking the
  planner's landing law all the way down, adapting to slope, brake lag, and
  creep torque. The clamp and the stop request then land on a stopped car,
  where they cannot be felt. Stock behavior whenever Smooth Stops is disabled.
  """

  def __init__(self):
    self.frame = 0
    self.enabled = False
    self.linger_frames = 0
    self.last_pid_output = 0.0

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.enabled = read_smooth_stops_params()
    self.frame += 1

  def defer_stopping(self, should_stop: bool, standstill: bool, v_ego: float) -> bool:
    if not self.enabled:
      self.linger_frames = 0
      return should_stop

    if not should_stop or standstill:
      self.linger_frames = 0
      return should_stop

    # the plan wants a stop and the car is crawling: if the light settle brake
    # never closes the last bit to standstill, stop deferring and let the clamp
    # finish the stop. The car must never keep rolling when it should be stopped.
    if v_ego < LINGER_SPEED:
      self.linger_frames += 1
      if self.linger_frames >= int(LINGER_TIME / DT_CTRL):
        return True
    else:
      self.linger_frames = 0

    return False

  def smooth_pid_output(self, output_accel: float, v_ego: float) -> float:
    # jerk-limit the command in the settle regime to remove low-speed PID dither
    if not self.enabled or v_ego > SETTLE_SMOOTH_SPEED:
      self.last_pid_output = output_accel
      return output_accel

    step = SETTLE_JERK_LIMIT * DT_CTRL
    output_accel = min(max(output_accel, self.last_pid_output - step), self.last_pid_output + step)
    self.last_pid_output = output_accel
    return output_accel
