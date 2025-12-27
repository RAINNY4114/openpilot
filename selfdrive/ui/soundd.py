import math
import os
import numpy as np
import time
import wave

from cereal import car, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

from openpilot.system import micd
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 30 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  VOLUME_BASE = 10

AUTO_AVOID_CHIME_MIN_INTERVAL_S = 2.0
AUTO_AVOID_CHIME_STALE_TIMEOUT_S = 1.0
AUTO_AVOID_CHIME_SPEED_MIN = 10 * CV.MPH_TO_MS
HAZARD_CHIME_MIN_INTERVAL_S = 2.0

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),
}
if HARDWARE.get_device_type() == "tizi":
  sound_list.update({
    AudibleAlert.engage: ("engage_tizi.wav", 1, MAX_VOLUME),
    AudibleAlert.disengage: ("disengage_tizi.wav", 1, MAX_VOLUME),
  })

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self._params = Params()
    self.load_sounds()
    self.load_dp_voice_sounds()
    self.load_dp_avoid_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.selfdrive_timeout_alert = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    try:
      self._dp_dev_audible_alert_mode = int(self._params.get("dp_dev_audible_alert_mode") or 0)
    except Exception:
      self._dp_dev_audible_alert_mode = 0

    self.dp_voice_enabled = self._params.get_bool("dp_lincoln_bsm_voice_enabled")
    self.dp_voice_interval = max(1.0, float(self._params.get("dp_lincoln_bsm_voice_interval_sec") or 3))
    self.dp_voice_volume = np.clip(int(self._params.get("dp_lincoln_bsm_voice_volume_pct") or 100) / 100.0, 0.0, 1.0)
    self._dp_voice_last_param_check = 0.0
    self.dp_voice_next_allowed = 0.0
    self.dp_voice_playing = False
    self.dp_voice_sound: np.ndarray | None = None
    self.dp_voice_frame = 0
    self.dp_voice_prev_left = False
    self.dp_voice_prev_right = False

    # dp lincoln auto-avoid chime (plays alert_chime.wav once when avoidance triggers)
    self._dp_auto_avoid_enabled = self._params.get_bool("dp_lincoln_auto_avoid")
    self._dp_hazard_alert_enabled = self._params.get_bool("dp_lincoln_hazard_alert")
    self._dp_auto_avoid_last_param_check = 0.0
    self._dp_auto_avoid_prev_in_path = False
    self._dp_auto_avoid_last_det_t = 0.0
    self._dp_auto_avoid_chime_next_allowed = 0.0
    self._dp_auto_avoid_chime_playing = False
    self._dp_auto_avoid_chime_frame = 0

    # dp hazard chime (plays warning_immediate.wav once when a hazard is detected in-path)
    self._dp_hazard_prev_in_path = False
    self._dp_hazard_chime_next_allowed = 0.0
    self._dp_hazard_chime_playing = False
    self._dp_hazard_chime_frame = 0

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      with wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r') as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()
        self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def load_dp_voice_sounds(self):
    self.dp_voice_sounds: dict[str, np.ndarray] = {}
    base = os.path.join(BASEDIR, "selfdrive", "assets", "sounds")
    for side in ("left", "right"):
      path = os.path.join(base, f"{side}.wav")
      try:
        with wave.open(path, 'r') as wavefile:
          assert wavefile.getnchannels() == 1
          assert wavefile.getsampwidth() == 2
          assert wavefile.getframerate() == SAMPLE_RATE
          length = wavefile.getnframes()
          data = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)
          self.dp_voice_sounds[side] = data
      except FileNotFoundError:
        cloudlog.warning(f"Missing blindspot voice file: {path}")
      except Exception:
        cloudlog.exception(f"Failed loading blindspot voice file: {path}")

  def load_dp_avoid_sounds(self):
    self._dp_avoid_chime_sound: np.ndarray | None = None
    self._dp_hazard_chime_sound: np.ndarray | None = None

    base = os.path.join(BASEDIR, "selfdrive", "assets", "sounds")
    files = {
      "avoid": os.path.join(base, "alert_chime.wav"),
      "hazard": os.path.join(base, "warning_immediate.wav"),
    }
    for k, path in files.items():
      try:
        with wave.open(path, 'r') as wavefile:
          assert wavefile.getnchannels() == 1
          assert wavefile.getsampwidth() == 2
          assert wavefile.getframerate() == SAMPLE_RATE
          length = wavefile.getnframes()
          data = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)
          if k == "avoid":
            self._dp_avoid_chime_sound = data
          elif k == "hazard":
            self._dp_hazard_chime_sound = data
      except FileNotFoundError:
        cloudlog.warning(f"Missing {k} chime file: {path}")
      except Exception:
        cloudlog.exception(f"Failed loading {k} chime file: {path}")

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
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

      # dp - set vol to 0 instead
      if self._dp_dev_audible_alert_mode == 2 or (self._dp_dev_audible_alert_mode == 1 and self.current_alert in [AudibleAlert.engage, AudibleAlert.disengage]):
        self.current_volume = 0

    base_audio = ret * self.current_volume
    base_audio += self._dp_voice_get_frames(frames)
    base_audio += self._dp_auto_avoid_get_frames(frames)
    base_audio += self._dp_hazard_get_frames(frames)
    return np.clip(base_audio, -1.0, 1.0)

  def _refresh_dp_voice_params(self, now: float) -> None:
    if (now - self._dp_voice_last_param_check) < 1.0:
      return
    self._dp_voice_last_param_check = now
    try:
      self.dp_voice_enabled = self._params.get_bool("dp_lincoln_bsm_voice_enabled")
      self.dp_voice_interval = max(1.0, float(self._params.get("dp_lincoln_bsm_voice_interval_sec") or 3))
      self.dp_voice_volume = np.clip(int(self._params.get("dp_lincoln_bsm_voice_volume_pct") or 100) / 100.0, 0.0, 1.0)
      if not self.dp_voice_enabled and self.dp_voice_playing:
        self.dp_voice_playing = False
        self.dp_voice_sound = None
    except Exception:
      cloudlog.exception("Failed refreshing Lincoln BSM voice params")

  def _refresh_dp_auto_avoid_params(self, now: float) -> None:
    if (now - self._dp_auto_avoid_last_param_check) < 1.0:
      return
    self._dp_auto_avoid_last_param_check = now
    try:
      self._dp_auto_avoid_enabled = self._params.get_bool("dp_lincoln_auto_avoid")
      self._dp_hazard_alert_enabled = self._params.get_bool("dp_lincoln_hazard_alert")
    except Exception:
      cloudlog.exception("Failed refreshing Lincoln auto-avoid params")

  def _dp_voice_get_frames(self, frames: int) -> np.ndarray:
    if not self.dp_voice_playing or self.dp_voice_sound is None:
      return np.zeros(frames, dtype=np.float32)

    sound = self.dp_voice_sound
    out = np.zeros(frames, dtype=np.float32)
    remaining = sound.shape[0] - self.dp_voice_frame
    to_copy = min(frames, remaining)
    if to_copy > 0:
      out[:to_copy] = sound[self.dp_voice_frame:self.dp_voice_frame + to_copy]
      self.dp_voice_frame += to_copy
    if self.dp_voice_frame >= sound.shape[0]:
      self.dp_voice_playing = False
      self.dp_voice_sound = None
    return out * self.dp_voice_volume

  def _dp_auto_avoid_get_frames(self, frames: int) -> np.ndarray:
    if self.current_alert != AudibleAlert.none:
      self._dp_auto_avoid_chime_playing = False
      self._dp_auto_avoid_chime_frame = 0
      return np.zeros(frames, dtype=np.float32)

    if not self._dp_auto_avoid_chime_playing or self._dp_avoid_chime_sound is None:
      return np.zeros(frames, dtype=np.float32)

    sound = self._dp_avoid_chime_sound
    out = np.zeros(frames, dtype=np.float32)
    remaining = sound.shape[0] - self._dp_auto_avoid_chime_frame
    to_copy = min(frames, remaining)
    if to_copy > 0:
      out[:to_copy] = sound[self._dp_auto_avoid_chime_frame:self._dp_auto_avoid_chime_frame + to_copy]
      self._dp_auto_avoid_chime_frame += to_copy
    if self._dp_auto_avoid_chime_frame >= sound.shape[0]:
      self._dp_auto_avoid_chime_playing = False
      self._dp_auto_avoid_chime_frame = 0
    return out * self.current_volume

  def _dp_hazard_get_frames(self, frames: int) -> np.ndarray:
    if self.current_alert != AudibleAlert.none:
      self._dp_hazard_chime_playing = False
      self._dp_hazard_chime_frame = 0
      return np.zeros(frames, dtype=np.float32)

    if not self._dp_hazard_chime_playing or self._dp_hazard_chime_sound is None:
      return np.zeros(frames, dtype=np.float32)

    sound = self._dp_hazard_chime_sound
    out = np.zeros(frames, dtype=np.float32)
    remaining = sound.shape[0] - self._dp_hazard_chime_frame
    to_copy = min(frames, remaining)
    if to_copy > 0:
      out[:to_copy] = sound[self._dp_hazard_chime_frame:self._dp_hazard_chime_frame + to_copy]
      self._dp_hazard_chime_frame += to_copy
    if self._dp_hazard_chime_frame >= sound.shape[0]:
      self._dp_hazard_chime_playing = False
      self._dp_hazard_chime_frame = 0
    return out * self.current_volume

  def _maybe_start_dp_voice(self, side: str, now: float) -> None:
    if not self.dp_voice_enabled:
      return
    if now < self.dp_voice_next_allowed:
      return
    if self.current_alert != AudibleAlert.none or self.dp_voice_playing:
      return
    sound = self.dp_voice_sounds.get(side)
    if sound is None or sound.size == 0:
      return
    self.dp_voice_sound = sound
    self.dp_voice_frame = 0
    self.dp_voice_playing = True
    self.dp_voice_next_allowed = now + self.dp_voice_interval

  def _maybe_start_dp_auto_avoid_chime(self, now: float) -> None:
    if not self._dp_auto_avoid_enabled:
      return
    if now < self._dp_auto_avoid_chime_next_allowed:
      return
    if self.current_alert != AudibleAlert.none or self.dp_voice_playing or self._dp_auto_avoid_chime_playing:
      return
    if self._dp_avoid_chime_sound is None or self._dp_avoid_chime_sound.size == 0:
      return
    self._dp_auto_avoid_chime_playing = True
    self._dp_auto_avoid_chime_frame = 0
    self._dp_auto_avoid_chime_next_allowed = now + AUTO_AVOID_CHIME_MIN_INTERVAL_S

  def _maybe_start_dp_hazard_chime(self, now: float) -> None:
    if not self._dp_hazard_alert_enabled:
      return
    if now < self._dp_hazard_chime_next_allowed:
      return
    if self.current_alert != AudibleAlert.none or self.dp_voice_playing or self._dp_auto_avoid_chime_playing or self._dp_hazard_chime_playing:
      return
    if self._dp_hazard_chime_sound is None or self._dp_hazard_chime_sound.size == 0:
      return
    self._dp_hazard_chime_playing = True
    self._dp_hazard_chime_frame = 0
    self._dp_hazard_chime_next_allowed = now + HAZARD_CHIME_MIN_INTERVAL_S

  def _update_dp_voice_state(self, car_state, now: float) -> None:
    if car_state is None:
      return
    left = bool(getattr(car_state, "leftBlindspot", False))
    right = bool(getattr(car_state, "rightBlindspot", False))

    if left and not self.dp_voice_prev_left:
      self._maybe_start_dp_voice("left", now)
    if right and not self.dp_voice_prev_right:
      self._maybe_start_dp_voice("right", now)

    self.dp_voice_prev_left = left
    self.dp_voice_prev_right = right

  def _update_dp_auto_avoid_state(self, sm, now: float) -> None:
    in_path = False
    vehicle_in_path = False
    haz_in_path = False
    if sm.updated.get("customReservedRawData0", False):
      raw = sm["customReservedRawData0"].customReservedRawData0
      payload = decode_cone_detections(raw) if raw else None
      if payload is not None:
        in_path = bool(payload.get("inPath", False))
        vehicle_in_path = bool(payload.get("vehicleInPath", False))
        haz_in_path = bool(payload.get("hazInPath", False))
        self._dp_auto_avoid_last_det_t = now

    if (now - self._dp_auto_avoid_last_det_t) > AUTO_AVOID_CHIME_STALE_TIMEOUT_S:
      in_path = False
      vehicle_in_path = False
      haz_in_path = False

    bsm_available = bool(sm.valid.get("carParams", False) and sm["carParams"].enableBsm)
    manual_blinker = False
    v_ego = 0.0
    if sm.alive.get("carState", False):
      cs = sm["carState"]
      manual_blinker = cs.leftBlinker != cs.rightBlinker
      v_ego = float(cs.vEgo)

    should_avoid_chime = self._dp_auto_avoid_enabled and bsm_available and (not manual_blinker) and (v_ego >= AUTO_AVOID_CHIME_SPEED_MIN)
    should_hazard_chime = self._dp_hazard_alert_enabled and (not manual_blinker) and (v_ego >= AUTO_AVOID_CHIME_SPEED_MIN)

    obstacle_in_path = in_path or vehicle_in_path
    if should_avoid_chime and obstacle_in_path and not self._dp_auto_avoid_prev_in_path:
      self._maybe_start_dp_auto_avoid_chime(now)

    if should_hazard_chime and haz_in_path and not self._dp_hazard_prev_in_path:
      self._maybe_start_dp_hazard_chime(now)

    self._dp_auto_avoid_prev_in_path = obstacle_in_path
    self._dp_hazard_prev_in_path = haz_in_path

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame > len(self.loaded_sounds[self.current_alert])
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
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

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure', 'carState', 'carParams', 'customReservedRawData0'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)
        now = time.monotonic()

        if sm.updated['soundPressure'] and self.current_alert == AudibleAlert.none: # only update volume filter when not playing alert
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

        self.get_audible_alert(sm)
        self._refresh_dp_voice_params(now)
        self._refresh_dp_auto_avoid_params(now)
        if sm.alive['carState']:
          self._update_dp_voice_state(sm['carState'], now)
        self._update_dp_auto_avoid_state(sm, now)

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
