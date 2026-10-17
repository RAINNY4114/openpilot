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


# ============================================================================
# Blind-spot voice prompts
#
# Reference implementation: the dragonpilot `ford` branch
# (https://github.com/RAINNY4114/openpilot/tree/ford), which plays
# "left.wav" / "right.wav" when the corresponding blind spot appears.
#
# Adapted here to this fork:
#   - both sides are watched independently (left and right are edge triggered
#     separately, so each side gets its own prompt)
#   - edge triggered: one prompt per appearance, never a repeating nag
#   - a minimum interval per prompt, so it cannot chatter
#   - mixed in as its own channel on top of the regular alert sounds, and it
#     yields immediately if a real alert starts
#   - one voice channel, so only one prompt can be in flight at a time.  If the
#     other side appears while a prompt is playing (or during its cooldown) it is
#     QUEUED rather than dropped, and announced as soon as the channel is free.
#     Without that, a car appearing on the right during the left prompt's
#     cooldown would be silently never announced.
#
# The prompt trigger is the FACTORY BLIND-SPOT SIGNAL ONLY:
#
#   left  = carState.leftBlindspot
#   right = carState.rightBlindspot
#
# Road test 2026-09-28: pulling the side LiDAR (amapNavi) straight into the
# trigger made the prompt announce "car there" nonstop.  The blind-spot check is
# therefore NOT augmented by the auxiliary sensors -- neither here nor in
# card.py.  `carState.leftBlindspot/rightBlindspot` is the OEM BSM (set by the
# car port), and that alone is what a "blind spot" prompt should follow.
# ============================================================================

BSM_VOICE_ENABLE_PARAM = "dp_bsm_voice_enabled"
BSM_VOICE_INTERVAL_PARAM = "dp_bsm_voice_interval_sec"
BSM_VOICE_VOLUME_PARAM = "dp_bsm_voice_volume_pct"

BSM_VOICE_PARAM_PERIOD = 1.0          # seconds between param re-reads
BSM_VOICE_DEFAULT_INTERVAL_S = 3.0    # minimum seconds between two prompts
BSM_VOICE_MIN_INTERVAL_S = 1.0
BSM_VOICE_DEFAULT_VOLUME_PCT = 100

# A prompt that could not start yet (channel busy / inside the cooldown) is
# queued and announced when the channel frees up.  It is dropped if it waited
# longer than this, so a stale announcement is never made about a car that has
# long since gone.
BSM_VOICE_PENDING_MAX_AGE = 5.0

# ---------------------------------------------------------------------------
# Prompt level
#
# The prompt used to be scaled only by bsm_voice_volume (default 1.0), so it
# played at FULL SCALE while every alert in sound_list is scaled by
# `current_volume`.  Measured on this car (vol_curve.py, route 1a--9):
# current_volume has a median of 0.140 and a mean of 0.195, so the prompt was
# playing ~14-17 dB LOUDER than any alert.  The cabin microphone confirms it:
# a blind-spot prompt raises soundPressureWeightedDb by 12-30 dB for exactly
# the length of the clip, with the alert channel silent throughout
# (bsm_tl.py / bsm_spl.py).  That over-drives the speaker.
#
# [NOTE] BSM_LEVEL_NOTE_2026_09_30
# The earlier claim that right.wav was "5.0 dB hotter and 1.6 dB denser
# ... and breaks up first" was WRONG, and the -5.0 dB correction it
# justified has been reverted (right.wav is the shipped file again).
#
# Plain RMS/LUFS is not what the driver hears.  right.wav is a darker
# clip -- spectral centroid 1588 Hz and 95% of its energy below 1917 Hz,
# against 1872 Hz / 4375 Hz for left.wav, with 10 dB less at 4-6 kHz --
# so at equal measured loudness it sounds weaker.
#
# Measured end to end through the speaker and the cabin microphone
# (bsm_power.py over routes --1..--42 of dongle 0000001b, both prompts
# played at a saturated gain of 1.000, ambient 43.1 vs 42.4 dB):
#
#     left  median Pp/Pa = 7.849
#     right median Pp/Pa = 2.764      -> the shipped right.wav is 4.53 dB
#                                        MORE acoustic power than left
#
# i.e. the two shipped files were already level-matched in the car and the
# -5.0 dB scaling made the right prompt ~4.5 dB too quiet -- exactly the
# asymmetry the driver reported.  The real over-drive was the missing
# volume law above, NOT the wav pair.  Do not re-level these files on a
# plain RMS/LUFS comparison.
#
# The prompt now follows the same volume law as the alert channel, with a
# deliberate lift so speech stays intelligible over a chime.
# ---------------------------------------------------------------------------
BSM_VOICE_ALERT_GAIN = 4.0   # +6 dB vs the alert channel (see the module docstring)
BSM_VOICE_MAX_GAIN = 1.0     # absolute ceiling, so a loud cabin cannot clip
BSM_MIX_KNEE = 0.75          # soft-limiter knee for the alert + prompt mix


# ============================================================================
# Automatic lane-change voice prompt (auto overtake)
#
# Plays "自动左变道，三秒后启动" / "自动右变道，三秒后启动" when the auto
# overtake starts a lane change, ~3 s before the car actually moves over.
#
# Trigger: the off -> preLaneChange edge of modelV2.meta.laneChangeState.
#
#   DesireHelper enters preLaneChange from either
#       (a) the driver's own blinker        -> blinker ON
#       (b) the auto-overtake request       -> blinker OFF
#   and for (b) it holds preLaneChange for AutoOvertakeConfirmSec (3.0 s)
#   before laneChangeStarting -- that is the "三秒后启动" window.
#
#   Measured on this car 2026-10-01:
#       route 28 seg 15  off->pre, L=False R=False, dir=right  -> AUTOMATIC
#       route 28 seg 24  off->pre, R=True                     -> DRIVER
#   so "both blinkers off at the off->pre edge" is the automatic test.
#
#   preLaneChange can collapse to a single frame when the driver applies
#   steering torque in the change direction (`torque_applied` short-circuits
#   the confirm window -- exactly what happened in seg 15).  laneChangeStarting
#   is therefore used as a once-only fallback trigger.
#
# The prompt is a voice channel shared with the blind-spot prompt, mixed on top
# of the alert channel with the same soft limiter.
# ============================================================================

LC_VOICE_ENABLE_PARAM = "dp_lc_voice_enabled"
LC_VOICE_VOLUME_PARAM = "dp_lc_voice_volume_pct"

LC_VOICE_PARAM_PERIOD = 1.0            # seconds between param re-reads
LC_VOICE_DEFAULT_VOLUME_PCT = 100
LC_VOICE_MIN_INTERVAL_S = 2.0          # never two announcements back to back
LC_VOICE_ALERT_GAIN = 4.0              # same +6 dB lift as the blind-spot prompt
LC_VOICE_MAX_GAIN = 1.0

LC_VOICE_FILES = {
  "left": "dp_lc_left.wav",
  "right": "dp_lc_right.wav",
}

# log.capnp LaneChangeState / LaneChangeDirection, as plain ints so no capnp
# enum import is needed here.
LC_STATE_OFF = 0
LC_STATE_PRE = 1
LC_STATE_STARTING = 2
LC_DIR_NONE = 0
LC_DIR_LEFT = 1
LC_DIR_RIGHT = 2

# Fork-owned param directory.  It is NOT the params dir, so
# Params::clearAll() (params.cc:208-224) never unlinks it -- see
# ford_curve_controller.py for the full explanation.
FORK_PARAM_DIR = "/data/ford_params"


def read_param_direct(key: str) -> str | None:
  """Read a param straight from disk.

  The prebuilt libparams_c.so in this fork does not know the custom dp_* keys,
  so Params() cannot be used for them.  Other control code in this fork reads
  the param files directly; follow the same convention here.
  """
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
  """C1-continuous soft limiter.

  The mix used to end in np.clip(..., -1.0, 1.0), which flat-tops the waveform
  the moment an alert and the blind-spot prompt overlap -- audible as 破音.
  Below the knee the response is exactly linear, so ordinary playback is
  bit-for-bit unchanged; above it the signal is compressed smoothly toward
  full scale, so an overshoot rounds off instead of clipping.
  """
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
    self.load_lc_voice_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.pending_stop = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    # blind-spot voice prompts
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

    # automatic lane-change voice prompt
    self.lc_voice_enabled = True
    self.lc_voice_volume = LC_VOICE_DEFAULT_VOLUME_PCT / 100.0
    self._lc_voice_last_param_check = 0.0
    self.lc_voice_next_allowed = 0.0
    self.lc_voice_playing = False
    self.lc_voice_sound: np.ndarray | None = None
    self.lc_voice_frame = 0
    self.lc_voice_issued = False
    self.lc_voice_prev_state = LC_STATE_OFF
    self.lc_voice_prev_dir = LC_DIR_NONE

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
    """Load the per-side blind-spot voice prompts.

    Prefers "<side>.wav" (dragonpilot's original name) and falls back to
    "<side>-side.wav".  A missing or malformed file only disables that side;
    it must never take soundd down.
    """
    self.bsm_voice_sounds: dict[str, np.ndarray] = {}

    for side in ("left", "right"):
      for filename in (f"{side}.wav", f"{side}-side.wav"):
        path = SOUNDS_DIR + filename
        try:
          with wave.open(path, 'r') as wavefile:
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
    self.bsm_voice_volume = float(np.clip(
      read_param_int(BSM_VOICE_VOLUME_PARAM, BSM_VOICE_DEFAULT_VOLUME_PCT) / 100.0, 0.0, 1.0,
    ))
    if not self.bsm_voice_enabled:
      self.bsm_voice_pending.clear()
      if self.bsm_voice_playing:
        self.bsm_voice_playing = False
        self.bsm_voice_sound = None
        self.bsm_voice_frame = 0

  def _bsm_voice_can_start(self, now: float) -> bool:
    return (self.bsm_voice_enabled
            and now >= self.bsm_voice_next_allowed
            and self.current_alert == AudibleAlert.none
            and not self.bsm_voice_playing
            and not self.lc_voice_playing)

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
    """Announce `side` now, or queue it for the next free slot.

    There is a single voice channel, so a side that appears while the other is
    still being announced must not simply be dropped -- it would never be
    announced at all.  Queue it instead.
    """
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
        return  # one prompt at a time; anything else waits for the next slot

  def update_bsm_voice(self, car_state, now: float) -> None:
    """Edge-triggered per-side blind-spot voice prompt.

    The trigger is the factory blind-spot signal only:

      left  = carState.leftBlindspot
      right = carState.rightBlindspot

    Left and right are watched independently, so a car appearing on the left
    announces the left side and one on the right announces the right side.  A
    prompt that cannot start yet (the other side is still playing, or the
    cooldown is still running) is queued rather than dropped, and announced as
    soon as the channel is free.
    """
    left = bool(getattr(car_state, "leftBlindspot", False))
    right = bool(getattr(car_state, "rightBlindspot", False))

    if left and not self.bsm_voice_prev_left:
      self._request_bsm_voice("left", now)
    if right and not self.bsm_voice_prev_right:
      self._request_bsm_voice("right", now)

    self.bsm_voice_prev_left = left
    self.bsm_voice_prev_right = right

    # announce anything that was queued while the channel was busy
    self._flush_pending_bsm_voice(now)


  def load_lc_voice_sounds(self) -> None:
    """Load the per-side automatic lane-change prompts.

    A missing or malformed file only disables that side; it must never take
    soundd down.
    """
    self.lc_voice_sounds: dict[str, np.ndarray] = {}

    for side, filename in LC_VOICE_FILES.items():
      path = SOUNDS_DIR + filename
      try:
        with wave.open(path, 'r') as wavefile:
          if (wavefile.getnchannels() != 1 or
              wavefile.getsampwidth() != 2 or
              wavefile.getframerate() != SAMPLE_RATE):
            raise ValueError(f"unsupported wav format: {path}")
          length = wavefile.getnframes()
          data = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)
        self.lc_voice_sounds[side] = data
      except FileNotFoundError:
        cloudlog.warning(f"Missing lane-change voice prompt: {path}")
      except Exception:
        cloudlog.exception(f"Failed loading lane-change voice file: {path}")

  def _refresh_lc_voice_params(self, now: float) -> None:
    if (now - self._lc_voice_last_param_check) < LC_VOICE_PARAM_PERIOD:
      return
    self._lc_voice_last_param_check = now
    self.lc_voice_enabled = read_param_int(LC_VOICE_ENABLE_PARAM, 1) != 0
    self.lc_voice_volume = float(np.clip(
      read_param_int(LC_VOICE_VOLUME_PARAM, LC_VOICE_DEFAULT_VOLUME_PCT) / 100.0, 0.0, 1.0,
    ))
    if not self.lc_voice_enabled and self.lc_voice_playing:
      self.lc_voice_playing = False
      self.lc_voice_sound = None
      self.lc_voice_frame = 0

  def _lc_voice_gain(self) -> float:
    return min(self.current_volume * LC_VOICE_ALERT_GAIN, LC_VOICE_MAX_GAIN)

  def _maybe_start_lc_voice(self, side: str, now: float) -> bool:
    """Start the announcement now, or report that it could not start."""
    if not self.lc_voice_enabled:
      return False
    if self.lc_voice_playing:
      return False
    if now < self.lc_voice_next_allowed:
      return False
    if self.current_alert != AudibleAlert.none:
      return False
    sound = self.lc_voice_sounds.get(side)
    if sound is None or sound.size == 0:
      return False

    # A lane change is imminent and the driver must be told, so this takes the
    # voice channel from a blind-spot prompt that is still in flight.  The
    # blind-spot side is not lost: its pending queue re-announces it later.
    if self.bsm_voice_playing:
      self.bsm_voice_playing = False
      self.bsm_voice_sound = None
      self.bsm_voice_frame = 0

    self.lc_voice_sound = sound
    self.lc_voice_frame = 0
    self.lc_voice_playing = True
    self.lc_voice_next_allowed = now + LC_VOICE_MIN_INTERVAL_S
    return True

  def update_lc_voice(self, sm, now: float) -> None:
    """Announce an AUTOMATIC lane change, once per manoeuvre."""
    if not self.lc_voice_enabled:
      return
    if not sm.valid.get('modelV2', False):
      return

    # This fork runs MADS, so `selfdriveState.enabled` can be False while the
    # lateral module is engaged.  Accept either, like check_selfdrive_timeout_alert.
    engaged = bool(sm.valid.get('selfdriveState', False)
                   and getattr(sm['selfdriveState'], 'enabled', False))
    if not engaged and sm.valid.get('selfdriveStateSP', False):
      engaged = bool(getattr(sm['selfdriveStateSP'].mads, 'enabled', False))
    if not engaged:
      return

    meta = sm['modelV2'].meta
    _st = getattr(meta, 'laneChangeState', LC_STATE_OFF)
    _dr = getattr(meta, 'laneChangeDirection', LC_DIR_NONE)
    lc_state = int(getattr(_st, 'raw', _st))
    lc_dir = int(getattr(_dr, 'raw', _dr))

    if lc_state == LC_STATE_OFF:
      # A new manoeuvre may be announced again once the state machine resets.
      self.lc_voice_issued = False

    if not self.lc_voice_issued and lc_dir in (LC_DIR_LEFT, LC_DIR_RIGHT):
      pre_started = (self.lc_voice_prev_state == LC_STATE_OFF and lc_state == LC_STATE_PRE)
      started = (self.lc_voice_prev_state != LC_STATE_STARTING and lc_state == LC_STATE_STARTING)

      if pre_started or started:
        # A driver lane change always has a blinker on; an automatic request
        # enters with both blinkers off.  Never announce the driver's own.
        manual_blinker = False
        if sm.valid.get('carState', False):
          cs = sm['carState']
          manual_blinker = bool(cs.leftBlinker != cs.rightBlinker)

        if not manual_blinker:
          side = "left" if lc_dir == LC_DIR_LEFT else "right"
          self.lc_voice_issued = self._maybe_start_lc_voice(side, now)

    self.lc_voice_prev_state = lc_state
    self.lc_voice_prev_dir = lc_dir

  def _lc_voice_get_frames(self, frames: int) -> np.ndarray:
    if not self.lc_voice_playing or self.lc_voice_sound is None:
      return np.zeros(frames, dtype=np.float32)

    # yield immediately to a real alert rather than talking over it
    if self.current_alert != AudibleAlert.none:
      self.lc_voice_playing = False
      self.lc_voice_sound = None
      self.lc_voice_frame = 0
      return np.zeros(frames, dtype=np.float32)

    sound = self.lc_voice_sound
    out = np.zeros(frames, dtype=np.float32)
    remaining = sound.shape[0] - self.lc_voice_frame
    to_copy = min(frames, remaining)
    if to_copy > 0:
      out[:to_copy] = sound[self.lc_voice_frame:self.lc_voice_frame + to_copy]
      self.lc_voice_frame += to_copy
    if self.lc_voice_frame >= sound.shape[0]:
      self.lc_voice_playing = False
      self.lc_voice_sound = None
      self.lc_voice_frame = 0
    return out * self.lc_voice_volume


  def _bsm_voice_gain(self) -> float:
    """Gain applied to the prompt, on top of bsm_voice_volume.

    Same volume law as the alert channel (`current_volume`), lifted by
    BSM_VOICE_ALERT_GAIN and capped by BSM_VOICE_MAX_GAIN.  bsm_voice_volume
    (the dp_bsm_voice_volume_pct param) stays as the user's relative trim.
    """
    return min(self.current_volume * BSM_VOICE_ALERT_GAIN, BSM_VOICE_MAX_GAIN)

  def _bsm_voice_get_frames(self, frames: int) -> np.ndarray:
    if not self.bsm_voice_playing or self.bsm_voice_sound is None:
      return np.zeros(frames, dtype=np.float32)

    # yield immediately to a real alert rather than talking over it
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

    # the blind-spot voice prompt is its own channel, mixed on top.
    # It obeys the same volume law as the alert channel -- see
    # BSM_VOICE_ALERT_GAIN for the measurements behind that.
    base_audio = ret * self.current_volume
    base_audio += self._bsm_voice_get_frames(frames) * self._bsm_voice_gain()
    base_audio += self._lc_voice_get_frames(frames) * self._lc_voice_gain()
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

        # blind-spot voice prompts (edge triggered per side)
        self._refresh_bsm_voice_params(now)
        self.update_bsm_voice(sm['carState'] if sm.alive['carState'] else None, now)

        # automatic lane-change voice prompt (auto overtake)
        self._refresh_lc_voice_params(now)
        self.update_lc_voice(sm, now)

        # freeze volume during alerts to avoid mic feedback increasing volume.
        # The blind-spot prompt is our own output as well, so it must be
        # excluded too: the cabin mic hears it and the controller would ramp
        # current_volume by ~12 dB over the length of the prompt (measured
        # cabin SPL rise 12-30 dB) and leave it elevated afterwards, making
        # the next alert play far too loud.  Holding the filter as well as the
        # volume keeps the pre-prompt value, so there is no step when the
        # prompt ends.
        if sm.updated['soundPressure']:
          if not self.bsm_voice_playing and not self.lc_voice_playing:
            self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          if (self.current_alert == AudibleAlert.none
              and not self.bsm_voice_playing and not self.lc_voice_playing):
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
