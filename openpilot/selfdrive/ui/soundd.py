import math
import numpy as np
import time
import wave

from openpilot.cereal import log, messaging, custom
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper

# Use the legacy retry implementation that was working on the old soundd.
from openpilot.common.retry import retry

from openpilot.common.swaglog import cloudlog
from openpilot.system import micd
from openpilot.common.hardware import HARDWARE

from openpilot.sunnypilot.selfdrive.ui.quiet_mode import QuietMode


# =============================================================================
# Audio configuration
# =============================================================================

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096  # approx 100 ms

MAX_VOLUME = 1.0
MIN_VOLUME = 0.1

ALERT_RAMP_TIME = 4  # seconds to ramp to max volume for warningImmediate
SELFDRIVE_STATE_TIMEOUT = 5  # seconds

FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 26
DB_SCALE = 30

VOLUME_BASE = 20


if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10


# =============================================================================
# C3X audio output device
# =============================================================================
#
# Tested on the current comma 3X:
#
# device 3:
#   hw:0,3
#   48000 Hz
#   OutputStream -> FAILED
#   ALSA error -22 (Invalid argument)
#
# device 6:
#   hw:0,6
#   48000 Hz
#   OutputStream -> SUCCESS
#
# default device:
#   31 ("default")
#   default samplerate = 44100 Hz
#
# soundd requires 48000 Hz, therefore do NOT use the PortAudio default
# output device on this C3X.
#
SOUND_OUTPUT_DEVICE = 6


# =============================================================================
# Audible alerts
# =============================================================================

AudibleAlert = log.SelfdriveState.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert


# =============================================================================
# Sound lists
# =============================================================================

sound_list_sp: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlertSP, file name, play count, volume
  AudibleAlertSP.promptSingleLow: ("prompt_single_low.wav", 1, MAX_VOLUME),
  AudibleAlertSP.promptSingleHigh: ("prompt_single_high.wav", 1, MAX_VOLUME),
}


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count, volume
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


# =============================================================================
# Selfdrive timeout handling
# =============================================================================

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time["selfdriveState"]

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if (
      (sm["selfdriveState"].enabled or sm["selfdriveStateSP"].mads.enabled)
      and
      (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10
    ):
      return True

  return False


# =============================================================================
# Soundd
# =============================================================================

class Soundd(QuietMode):
  def __init__(self):
    super().__init__()

    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.pending_stop = False

    self.spl_filter_weighted = FirstOrderFilter(
      0,
      2.5,
      FILTER_DT,
      initialized=False,
    )

  # ---------------------------------------------------------------------------
  # Load sounds
  # ---------------------------------------------------------------------------

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    sound_path = BASEDIR + "/openpilot/selfdrive/assets/sounds/"

    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      path = sound_path + filename

      with wave.open(path, "r") as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()

        self.loaded_sounds[sound] = (
          np.frombuffer(
            wavefile.readframes(length),
            dtype=np.int16,
          ).astype(np.float32) / (2**16 / 2)
        )

  # ---------------------------------------------------------------------------
  # Get sound data
  # ---------------------------------------------------------------------------

  def get_sound_data(self, frames):
    """
    Get 'frames' worth of data from the current alert sound.

    Sounds with play_count=None are looped indefinitely.

    Sounds with a finite play_count stop after the requested number of
    complete plays.

    A pending stop allows a looping alert to finish its current loop before
    stopping.
    """

    ret = np.zeros(frames, dtype=np.float32)

    if not self.should_play_sound(self.current_alert):
      return ret

    num_loops = sound_list[self.current_alert][1]
    sound_data = self.loaded_sounds[self.current_alert]

    written_frames = 0

    # Total frame position since this alert started.
    current_sound_frame = self.current_sound_frame

    while written_frames < frames:
      # Stop finite sounds after the requested number of complete plays.
      if num_loops is not None:
        if current_sound_frame >= len(sound_data) * num_loops:
          self.current_alert = AudibleAlert.none
          self.current_sound_frame = 0
          self.pending_stop = False
          break

      current_frame = current_sound_frame % len(sound_data)

      available_frames = len(sound_data) - current_frame
      frames_to_write = min(
        available_frames,
        frames - written_frames,
      )

      ret[
        written_frames:
        written_frames + frames_to_write
      ] = sound_data[
        current_frame:
        current_frame + frames_to_write
      ]

      written_frames += frames_to_write
      current_sound_frame += frames_to_write

      # If a looping sound was requested to stop, let the current loop
      # complete before stopping.
      if self.pending_stop and current_sound_frame % len(sound_data) == 0:
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0
        self.pending_stop = False
        break

    self.current_sound_frame = current_sound_frame

    return ret * self.current_volume

  # ---------------------------------------------------------------------------
  # PortAudio callback
  # ---------------------------------------------------------------------------

  def callback(
    self,
    data_out: np.ndarray,
    frames: int,
    time,
    status,
  ) -> None:
    if status:
      cloudlog.warning(
        f"soundd stream over/underflow: {status}"
      )

    data_out[:frames, 0] = self.get_sound_data(frames)

  # ---------------------------------------------------------------------------
  # Alert update
  # ---------------------------------------------------------------------------

  def update_alert(self, new_alert):
    current_alert_played_once = (
      self.current_alert == AudibleAlert.none
      or
      (
        self.current_alert in self.loaded_sounds
        and
        self.current_sound_frame >= len(
          self.loaded_sounds[self.current_alert]
        )
      )
    )

    # Let looping sounds finish the current loop instead of cutting off
    # mid-tone.
    if (
      new_alert == AudibleAlert.none
      and self.current_alert != AudibleAlert.none
      and sound_list[self.current_alert][1] is None
    ):
      if current_alert_played_once:
        self.pending_stop = True
      else:
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0

      return

    self.pending_stop = False

    if (
      self.current_alert != new_alert
      and
      (
        new_alert != AudibleAlert.none
        or
        current_alert_played_once
      )
    ):
      if new_alert == AudibleAlert.warningImmediate:
        self.ramp_start_volume = self.current_volume
        self.ramp_start_time = time.monotonic()

      self.current_alert = new_alert
      self.current_sound_frame = 0

  # ---------------------------------------------------------------------------
  # Get audible alert
  # ---------------------------------------------------------------------------

  def get_audible_alert(self, sm):
    if sm.updated["selfdriveState"]:
      new_alert = sm["selfdriveState"].alertSound.raw
      self.update_alert(new_alert)

    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True

    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  # ---------------------------------------------------------------------------
  # Volume
  # ---------------------------------------------------------------------------

  def calculate_volume(self, weighted_db):
    volume = (
      ((weighted_db - AMBIENT_DB) / DB_SCALE)
      * (MAX_VOLUME - MIN_VOLUME)
      + MIN_VOLUME
    )

    return math.pow(
      VOLUME_BASE,
      (
        np.clip(
          volume,
          MIN_VOLUME,
          MAX_VOLUME,
        ) - 1
      ),
    )

  # ---------------------------------------------------------------------------
  # Open PortAudio stream
  # ---------------------------------------------------------------------------

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    """
    Open the C3X audio output stream.

    IMPORTANT:
      Do not use the PortAudio default device.

    The tested C3X configuration is:

      device 6 = hw:0,6
      output channels = 2
      default samplerate = 48000 Hz

    Direct testing confirmed that device 6 successfully opens an
    OutputStream at 48000 Hz.
    """

    cloudlog.info(
      f"soundd: initializing PortAudio, "
      f"output device={SOUND_OUTPUT_DEVICE}"
    )

    # Reload sounddevice to reinitialize PortAudio.
    sd._terminate()
    sd._initialize()

    # Query the selected hardware device after PortAudio initialization.
    try:
      device_info = sd.query_devices(SOUND_OUTPUT_DEVICE)

      cloudlog.info(
        f"soundd: selected device {SOUND_OUTPUT_DEVICE}: "
        f"{device_info}"
      )

      if device_info["max_output_channels"] < 1:
        raise RuntimeError(
          f"soundd: device {SOUND_OUTPUT_DEVICE} has no output channels"
        )

      device_rate = float(device_info["default_samplerate"])

      cloudlog.info(
        f"soundd: device={SOUND_OUTPUT_DEVICE}, "
        f"device_rate={device_rate}, "
        f"requested_rate={SAMPLE_RATE}, "
        f"output_channels={device_info['max_output_channels']}"
      )

    except Exception as e:
      cloudlog.error(
        f"soundd: failed to query output device "
        f"{SOUND_OUTPUT_DEVICE}: {e}"
      )
      raise

    # Explicitly select device 6.
    #
    # Do NOT remove device=SOUND_OUTPUT_DEVICE.
    stream = sd.OutputStream(
      device=SOUND_OUTPUT_DEVICE,
      channels=1,
      samplerate=SAMPLE_RATE,
      callback=self.callback,
      blocksize=SAMPLE_BUFFER,
    )

    cloudlog.info(
      f"soundd: OutputStream opened successfully: "
      f"device={SOUND_OUTPUT_DEVICE}, "
      f"samplerate={SAMPLE_RATE}, "
      f"channels=1, "
      f"blocksize={SAMPLE_BUFFER}"
    )

    return stream

  # ---------------------------------------------------------------------------
  # Main sound thread
  # ---------------------------------------------------------------------------

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    # IMPORTANT:
    #
    # Do NOT call:
    #
    #   micd.patch_sounddevice(sd)
    #
    # Direct hardware testing showed:
    #
    #   device 3 -> ALSA -22
    #   device 6 -> SUCCESS
    #
    # Therefore soundd uses the normal sounddevice implementation and
    # explicitly selects device 6.

    sm = messaging.SubMaster(
      [
        "selfdriveState",
        "selfdriveStateSP",
        "soundPressure",
      ]
    )

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(
        f"soundd stream started: "
        f"{stream.samplerate=}, "
        f"{stream.channels=}, "
        f"{stream.dtype=}, "
        f"{stream.device=}, "
        f"{stream.blocksize=}"
      )

      while True:
        sm.update(0)

        self.load_param()

        # ---------------------------------------------------------------------
        # Freeze volume during alerts to avoid microphone feedback increasing
        # the volume.
        # ---------------------------------------------------------------------

        if sm.updated["soundPressure"]:
          self.spl_filter_weighted.update(
            sm["soundPressure"].soundPressureWeightedDb
          )

          if self.current_alert == AudibleAlert.none:
            self.current_volume = self.calculate_volume(
              float(self.spl_filter_weighted.x)
            )

        # ---------------------------------------------------------------------
        # Audible alert
        # ---------------------------------------------------------------------

        self.get_audible_alert(sm)

        # ---------------------------------------------------------------------
        # warningImmediate ramp
        # ---------------------------------------------------------------------

        if self.current_alert == AudibleAlert.warningImmediate:
          elapsed = time.monotonic() - self.ramp_start_time

          ramp_vol = float(
            np.interp(
              elapsed,
              [0, ALERT_RAMP_TIME],
              [
                self.ramp_start_volume,
                MAX_VOLUME,
              ],
            )
          )

          self.current_volume = max(
            self.current_volume,
            ramp_vol,
          )

        # ---------------------------------------------------------------------
        # Keep thread timing
        # ---------------------------------------------------------------------

        rk.keep_time()

        assert stream.active


# =============================================================================
# Main
# =============================================================================

def main():
  cloudlog.info(
    f"soundd: starting "
    f"(output_device={SOUND_OUTPUT_DEVICE}, "
    f"samplerate={SAMPLE_RATE})"
  )

  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
