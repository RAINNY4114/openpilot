import math
import numpy as np
import time
import wave


from openpilot.cereal import log, messaging, custom
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd
from openpilot.common.hardware import HARDWARE

from openpilot.sunnypilot.selfdrive.ui.quiet_mode import QuietMode

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
ALERT_RAMP_TIME = 4 # seconds to ramp to max volume for warningImmediate
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 26 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10

SOUNDS_DIR = BASEDIR + "/openpilot/selfdrive/assets/sounds/"

BSM_VOICE_ENABLE_PARAM = "dp_bsm_voice_enabled"
BSM_VOICE_INTERVAL_PARAM = "dp_bsm_voice_interval_sec"
BSM_VOICE_VOLUME_PARAM = "dp_bsm_voice_volume_pct"

BSM_VOICE_PARAM_PERIOD = 1.0

AO_ALERT_ENABLE_PARAM = "AOAlertEnabled"
AO_MANEUVER_VOICE_MIN_INTERVAL_S = 2.0
BSM_VOICE_DEFAULT_INTERVAL_S = 3.0
BSM_VOICE_MIN_INTERVAL_S = 1.0
BSM_VOICE_DEFAULT_VOLUME_PCT = 100

BSM_VOICE_PENDING_MAX_AGE = 5.0

BSM_VOICE_ALERT_GAIN = 4.0
BSM_VOICE_MAX_GAIN = 1.0
BSM_MIX_KNEE = 0.75

FORK_PARAM_DIR = "/data/ford_params"

def read_param_direct(key: str) -> str | None:
  for path in (f"{FORK_PARAM_DIR}/{key}", f"/dev/shm/params/{key}", f"/data/params/d/{key}"):
    try:
      with open(path) as f:
        return f.read().strip()
    except Exception:
      continue
  return None

def read_param_int(key: str, default: int) -> int:
  raw = read_param_direct(key)
  if not raw:
    return default
  try:
    return int(float(raw))
  except (TypeError, ValueError):
    return default

AudibleAlert = log.SelfdriveState.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert


sound_list_sp: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlertSP, file name, play count (none for infinite)
  AudibleAlertSP.promptSingleLow: ("prompt_single_low.wav", 1, MAX_VOLUME),
  AudibleAlertSP.promptSingleHigh: ("prompt_single_high.wav", 1, MAX_VOLUME),
}

sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("warning.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("warning.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("dm_warning.wav", None, MAX_VOLUME),

  AudibleAlert.preAlert: ("pre_alert.wav", 1, MAX_VOLUME),

  AudibleAlert.warningSoft: ("critical.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("dm_critical.wav", None, MAX_VOLUME),

  **sound_list_sp,
}

def _soft_limit(x: np.ndarray) -> np.ndarray:
  a = np.abs(x)
  if not np.any(a > BSM_MIX_KNEE):
    return x
  y = x.copy()
  m = a > BSM_MIX_KNEE
  head = 1.0 - BSM_MIX_KNEE
  y[m] = np.sign(x[m]) * (BSM_MIX_KNEE + head * np.tanh((a[m] - BSM_MIX_KNEE) / head))
  return y

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if (sm['selfdriveState'].enabled or sm['selfdriveStateSP'].mads.enabled) and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd(QuietMode):
  def __init__(self):
    super().__init__()

    self.load_sounds()
    self.load_bsm_voice_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.pending_stop = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    self.bsm_voice_enabled = True
    self.bsm_voice_interval = BSM_VOICE_DEFAULT_INTERVAL_S
    self.bsm_voice_volume = BSM_VOICE_DEFAULT_VOLUME_PCT / 100.0
    self._bsm_voice_last_param_check = 0.0
    self.bsm_voice_next_allowed = 0.0
    self.bsm_voice_playing = False
    self.bsm_voice_sound: np.ndarray | None = None
    self.bsm_voice_frame = 0
    self.bsm_voice_pending: dict[str, float] = {}
    self.bsm_voice_prev_left = False
    self.bsm_voice_prev_right = False

    self._ao_alert_enabled = True
    self._ao_prev_lc_state = int(log.LaneChangeState.off)
    self._ao_voice_next_allowed = 0.0

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      with wave.open(BASEDIR + "/openpilot/selfdrive/assets/sounds/" + filename, 'r') as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()
        self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def load_bsm_voice_sounds(self) -> None:
    self.bsm_voice_sounds: dict[str, np.ndarray] = {}

    for side in ("left", "right"):
      for filename in (f"{side}.wav", f"{side}-side.wav"):
        path = SOUNDS_DIR + filename
        try:
          with wave.open(path, "r") as wavefile:
            if (wavefile.getnchannels() != 1 or
                wavefile.getsampwidth() != 2 or
                wavefile.getframerate() != SAMPLE_RATE):
              raise ValueError(f"unsupported wav format: {path}")
            length = wavefile.getnframes()
            data = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)
          self.bsm_voice_sounds[side] = data
          break
        except FileNotFoundError:
          continue
        except Exception:
          cloudlog.exception(f"Failed loading blind-spot voice file: {path}")
          break

    for side in ("left", "right"):
      if side not in self.bsm_voice_sounds:
        cloudlog.warning(f"Missing blind-spot voice prompt for the {side} side")

  def _refresh_bsm_voice_params(self, now: float) -> None:
    if (now - self._bsm_voice_last_param_check) < BSM_VOICE_PARAM_PERIOD:
      return
    self._bsm_voice_last_param_check = now
    self.bsm_voice_enabled = read_param_int(BSM_VOICE_ENABLE_PARAM, 1) != 0
    self.bsm_voice_interval = max(
      BSM_VOICE_MIN_INTERVAL_S,
      float(read_param_int(BSM_VOICE_INTERVAL_PARAM, int(BSM_VOICE_DEFAULT_INTERVAL_S))),
    )
    self._ao_alert_enabled = read_param_int(AO_ALERT_ENABLE_PARAM, 1) != 0
    self.bsm_voice_volume = float(np.clip(
      read_param_int(BSM_VOICE_VOLUME_PARAM, BSM_VOICE_DEFAULT_VOLUME_PCT) / 100.0, 0.0, 1.0,
    ))
    if not self.bsm_voice_enabled:
      self.bsm_voice_pending.clear()
      if self.bsm_voice_playing:
        self.bsm_voice_playing = False
        self.bsm_voice_sound = None
        self.bsm_voice_frame = 0

  def _update_ao_maneuver_voice(self, sm, now: float) -> None:
    """Play a direction prompt at the auto-overtake *intent* stage.

    Mirrors openpilot-lz@ford: modelV2.meta.laneChangeState off ->
    preLaneChange, with no real blinker (auto lane changes inject a virtual
    one via _AutoBlinkerProxy, so carState stays clean). Gives the driver
    the full AutoOvertakeConfirmSec (3s) of lead time before steering.
    """
    if not self._ao_alert_enabled:
      self._ao_prev_lc_state = int(log.LaneChangeState.off)
      return
    try:
      if not sm.valid.get("modelV2", False) or not sm.alive.get("carState", False):
        return
      if not bool(getattr(sm["selfdriveState"], "enabled", False)):
        self._ao_prev_lc_state = int(log.LaneChangeState.off)
        return
      meta = sm["modelV2"].meta
      lc_state = int(getattr(meta, "laneChangeState", log.LaneChangeState.off))
      lc_dir = int(getattr(meta, "laneChangeDirection", log.LaneChangeDirection.none))
      cs = sm["carState"]
      manual_blinker = bool(getattr(cs, "leftBlinker", False)) != bool(getattr(cs, "rightBlinker", False))
      pre_started = (self._ao_prev_lc_state == int(log.LaneChangeState.off)
                     and lc_state == int(log.LaneChangeState.preLaneChange))
      if (pre_started and not manual_blinker
          and lc_dir in (int(log.LaneChangeDirection.left), int(log.LaneChangeDirection.right))
          and now >= self._ao_voice_next_allowed):
        side = "left" if lc_dir == int(log.LaneChangeDirection.left) else "right"
        if self._start_bsm_voice(side, now):
          self._ao_voice_next_allowed = now + AO_MANEUVER_VOICE_MIN_INTERVAL_S
      self._ao_prev_lc_state = lc_state
    except Exception:
      pass

  def _bsm_voice_can_start(self, now: float) -> bool:
    return (self.bsm_voice_enabled
            and now >= self.bsm_voice_next_allowed
            and self.current_alert == AudibleAlert.none
            and not self.bsm_voice_playing)

  def _start_bsm_voice(self, side: str, now: float) -> bool:
    if not self._bsm_voice_can_start(now):
      return False
    sound = self.bsm_voice_sounds.get(side)
    if sound is None or sound.size == 0:
      return False
    self.bsm_voice_sound = sound
    self.bsm_voice_frame = 0
    self.bsm_voice_playing = True
    self.bsm_voice_next_allowed = now + self.bsm_voice_interval
    return True

  def _request_bsm_voice(self, side: str, now: float) -> None:
    if self._start_bsm_voice(side, now):
      self.bsm_voice_pending.pop(side, None)
      return
    if not self.bsm_voice_enabled:
      return
    self.bsm_voice_pending[side] = now

  def _flush_pending_bsm_voice(self, now: float) -> None:
    for side in ("left", "right"):
      when = self.bsm_voice_pending.get(side)
      if when is None:
        continue
      if (now - when) > BSM_VOICE_PENDING_MAX_AGE:
        del self.bsm_voice_pending[side]
        continue
      if self._start_bsm_voice(side, now):
        del self.bsm_voice_pending[side]
        return

  def update_bsm_voice(self, car_state, now: float) -> None:
    if car_state is None:
      self.bsm_voice_prev_left = False
      self.bsm_voice_prev_right = False
      return

    left = bool(getattr(car_state, "leftBlindspot", False))
    right = bool(getattr(car_state, "rightBlindspot", False))

    if left and not self.bsm_voice_prev_left:
      self._request_bsm_voice("left", now)
    if right and not self.bsm_voice_prev_right:
      self._request_bsm_voice("right", now)

    self.bsm_voice_prev_left = left
    self.bsm_voice_prev_right = right

    self._flush_pending_bsm_voice(now)

  def _bsm_voice_gain(self) -> float:
    return min(self.current_volume * BSM_VOICE_ALERT_GAIN, BSM_VOICE_MAX_GAIN)

  def _bsm_voice_get_frames(self, frames: int) -> np.ndarray:
    if not self.bsm_voice_playing or self.bsm_voice_sound is None:
      return np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      self.bsm_voice_playing = False
      self.bsm_voice_sound = None
      self.bsm_voice_frame = 0
      return np.zeros(frames, dtype=np.float32)

    sound = self.bsm_voice_sound
    out = np.zeros(frames, dtype=np.float32)
    remaining = sound.shape[0] - self.bsm_voice_frame
    to_copy = min(frames, remaining)
    if to_copy > 0:
      out[:to_copy] = sound[self.bsm_voice_frame:self.bsm_voice_frame + to_copy]
      self.bsm_voice_frame += to_copy
    if self.bsm_voice_frame >= sound.shape[0]:
      self.bsm_voice_playing = False
      self.bsm_voice_sound = None
      self.bsm_voice_frame = 0
    return out * self.bsm_voice_volume

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.should_play_sound(self.current_alert):
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write
        current_sound_frame = self.current_sound_frame % len(sound_data)
        loops = self.current_sound_frame // len(sound_data)
        if self.pending_stop and current_sound_frame == 0:
          self.current_alert = AudibleAlert.none
          self.pending_stop = False
          break

    base_audio = ret * self.current_volume
    base_audio += self._bsm_voice_get_frames(frames) * self._bsm_voice_gain()
    return _soft_limit(base_audio)

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame >= len(self.loaded_sounds[self.current_alert])
    # let looping sounds finish the current loop instead of cutting off mid tone
    if new_alert == AudibleAlert.none and self.current_alert != AudibleAlert.none and sound_list[self.current_alert][1] is None:
      if current_alert_played_once:
        self.pending_stop = True
      else:
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0
      return
    self.pending_stop = False
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      if new_alert == AudibleAlert.warningImmediate:
        self.ramp_start_volume = self.current_volume
        self.ramp_start_time = time.monotonic()
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd
    micd.patch_sounddevice(sd)

    sm = messaging.SubMaster(['selfdriveState', 'selfdriveStateSP', 'soundPressure', 'carState', 'modelV2'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        self.load_param()

        now = time.monotonic()

        self._refresh_bsm_voice_params(now)
        self.update_bsm_voice(sm['carState'] if sm.alive['carState'] else None, now)
        self._update_ao_maneuver_voice(sm, now)

        if sm.updated['soundPressure']:
          if not self.bsm_voice_playing:
            self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          if self.current_alert == AudibleAlert.none and not self.bsm_voice_playing:
            self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

        self.get_audible_alert(sm)

        # Ramp up immediate warning sound over 4s
        if self.current_alert == AudibleAlert.warningImmediate:
          elapsed = time.monotonic() - self.ramp_start_time
          ramp_vol = float(np.interp(elapsed, [0, ALERT_RAMP_TIME], [self.ramp_start_volume, MAX_VOLUME]))
          self.current_volume = max(self.current_volume, ramp_vol)

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
