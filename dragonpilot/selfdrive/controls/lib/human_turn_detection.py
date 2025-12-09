import os
import time
from enum import Enum, auto

from openpilot.common.params import Params


LOG_PATH = "/data/media/0/realdata/lincoln_debug.log"
PARAM_REFRESH_SEC = 2.0
MIN_SPEED_MS = 0.1


def _log(message: str) -> None:
  try:
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
      f.write(f"{time.time():.3f} {message}\n")
  except Exception:
    pass


class HTDState(Enum):
  INACTIVE = auto()
  MANUAL_TURN = auto()
  RAMPING = auto()


class HumanTurnDetection:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0
    self._enabled = True
    self._angle_threshold_deg = 90.0
    self._angle_release_deg = 8.0
    self._torque_start_nm = 1.2
    self._torque_release_nm = 0.6
    self._recovery_delay = 1.0

    self._state: HTDState = HTDState.INACTIVE
    self._state_change_time = 0.0
    self._last_angle = 0.0
    self._last_torque = 0.0

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    self._enabled = self._params.get_bool("dp_htd_enabled")
    self._angle_threshold_deg = self._get_float("dp_htd_turn_angle_threshold", 90.0)
    self._angle_release_deg = self._get_float("dp_htd_turn_angle_release_deg", 8.0)
    self._torque_start_nm = self._get_float("dp_htd_torque_start_nm", 1.2)
    self._torque_release_nm = self._get_float("dp_htd_torque_release_nm", 0.6)
    self._recovery_delay = self._get_float("dp_htd_recovery_delay", 1.0)

  def _transition(self, new_state: HTDState, reason: str) -> None:
    if new_state == self._state:
      return
    self._state = new_state
    self._state_change_time = time.monotonic()
    _log(f"HTD {new_state.name} reason={reason} angle={self._last_angle:.1f} torque={self._last_torque:.2f}")

  def update(self, lat_active: bool, steering_angle_deg: float, steering_torque_nm: float,
             v_ego: float) -> tuple[bool, HTDState]:
    self._read_params()

    self._last_angle = abs(steering_angle_deg)
    self._last_torque = abs(steering_torque_nm)

    if not self._enabled or not lat_active or v_ego < MIN_SPEED_MS:
      if self._state != HTDState.INACTIVE:
        self._transition(HTDState.INACTIVE, "disabled")
      return True, self._state

    if self._state == HTDState.INACTIVE:
      if self._should_trigger():
        self._transition(HTDState.MANUAL_TURN, "trigger")
        return False, self._state
      return True, self._state

    if self._state == HTDState.MANUAL_TURN:
      if self._should_release():
        self._transition(HTDState.RAMPING, "release")
      return False, self._state

    # RAMPING
    if self._should_trigger():
      self._transition(HTDState.MANUAL_TURN, "retrigger")
      return False, self._state
    if time.monotonic() - self._state_change_time >= self._recovery_delay:
      self._transition(HTDState.INACTIVE, "resume")
      return True, self._state
    return False, self._state

  def _should_trigger(self) -> bool:
    return self._last_torque >= self._torque_start_nm and self._last_angle >= self._angle_threshold_deg

  def _should_release(self) -> bool:
    return self._last_torque <= self._torque_release_nm and self._last_angle <= self._angle_release_deg

  def _get_float(self, key: str, default: float) -> float:
    try:
      val = self._params.get(key)
      if val is None:
        return default
      return float(val)
    except (TypeError, ValueError):
      return default
