#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from functools import cache
import threading

from openpilot.cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog


# ============================================================================
# Audio configuration
# ============================================================================

RATE = 10

FFT_SAMPLES = 1600  # 100 ms at 16 kHz

REFERENCE_SPL = 2e-5  # newtons/m^2

SAMPLE_RATE = 16000

SAMPLE_BUFFER = 800  # 50 ms at 16 kHz


# ============================================================================
# C3X microphone input device
# ============================================================================
#
# Verified device enumeration on the target comma 3X:
#
#   device 3:
#       hw:0,3
#       input channels = 2
#       output channels = 2
#       default samplerate = 48000 Hz
#
#   device 6:
#       hw:0,6
#       input channels = 0
#       output channels = 2
#       default samplerate = 48000 Hz
#
#   device 31:
#       default
#       input channels = 128
#       output channels = 128
#       default samplerate = 44100 Hz
#
# micd requires an INPUT device.
#
# Therefore:
#
#   soundd -> device 6
#   micd   -> device 3
#
# Do NOT use device 6 for micd because it has zero input channels.
#

MIC_INPUT_DEVICE = 3


# ============================================================================
# sounddevice compatibility helper
# ============================================================================

def patch_sounddevice(sd):
  """
  Compatibility helper for sounddevice.

  Older versions of sounddevice used a custom array conversion path.
  Keep this function because existing micd code may depend on sd._array.

  This does NOT initialize or terminate PortAudio.
  """

  def sounddevice_array(buffer, channels, dtype):
    return np.frombuffer(
      buffer,
      dtype=dtype,
    ).reshape(
      -1,
      channels,
    )

  sd._array = sounddevice_array


# ============================================================================
# A-weighting filter
# ============================================================================

@cache
def get_a_weighting_filter():

  # Calculate the A-weighting filter.
  #
  # https://en.wikipedia.org/wiki/A-weighting

  freqs = np.fft.fftfreq(
    FFT_SAMPLES,
    d=1 / SAMPLE_RATE,
  )

  A = (
    12194 ** 2
    * freqs ** 4
    /
    (
      (freqs ** 2 + 20.6 ** 2)
      *
      (freqs ** 2 + 12194 ** 2)
      *
      np.sqrt(
        (freqs ** 2 + 107.7 ** 2)
        *
        (freqs ** 2 + 737.9 ** 2)
      )
    )
  )

  return A / np.max(A)


# ============================================================================
# SPL calculation
# ============================================================================

def calculate_spl(measurements):

  # RMS of amplitudes.
  sound_pressure = np.sqrt(
    np.mean(
      measurements ** 2
    )
  )

  if sound_pressure > 0:

    sound_pressure_level = (
      20
      *
      np.log10(
        sound_pressure
        /
        REFERENCE_SPL
      )
    )

  else:

    sound_pressure_level = 0

  return (
    sound_pressure,
    sound_pressure_level,
  )


# ============================================================================
# A-weighting
# ============================================================================

def apply_a_weighting(
  measurements: np.ndarray,
) -> np.ndarray:

  # Generate a Hanning window of the same length
  # as the audio measurements.

  measurements_windowed = (
    measurements
    *
    np.hanning(
      len(measurements)
    )
  )

  # Apply the A-weighting filter to the signal.

  return np.abs(
    np.fft.ifft(
      np.fft.fft(
        measurements_windowed
      )
      *
      get_a_weighting_filter()
    )
  )


# ============================================================================
# Mic daemon
# ============================================================================

class Mic:

  def __init__(self):

    self.rk = Ratekeeper(RATE)

    self.pm = messaging.PubMaster(
      [
        "soundPressure",
        "rawAudioData",
      ]
    )

    self.measurements = np.empty(
      0,
      dtype=np.float32,
    )

    self.sound_pressure = 0

    self.sound_pressure_weighted = 0

    self.sound_pressure_level_weighted = 0

    self.lock = threading.Lock()


  # ==========================================================================
  # Publish sound pressure
  # ==========================================================================

  def update(self):

    with self.lock:

      sound_pressure = (
        self.sound_pressure
      )

      sound_pressure_weighted = (
        self.sound_pressure_weighted
      )

      sound_pressure_level_weighted = (
        self.sound_pressure_level_weighted
      )


    msg = messaging.new_message(
      "soundPressure",
      valid=True,
    )

    msg.soundPressure.soundPressure = float(
      sound_pressure
    )

    msg.soundPressure.soundPressureWeighted = float(
      sound_pressure_weighted
    )

    msg.soundPressure.soundPressureWeightedDb = float(
      sound_pressure_level_weighted
    )

    self.pm.send(
      "soundPressure",
      msg,
    )

    self.rk.keep_time()


  # ==========================================================================
  # Audio callback
  # ==========================================================================

  def callback(
    self,
    indata,
    frames,
    time_info,
    status,
  ):
    """
    Using amplitude measurements, calculate an uncalibrated sound pressure
    and sound pressure level.

    Then apply A-weighting to the raw amplitudes and run the same calculations
    again.

    Logged A-weighted equivalents are rough approximations of human-perceived
    loudness.
    """

    if status:

      cloudlog.warning(
        f"micd input stream status: {status}"
      )


    # ------------------------------------------------------------------------
    # Validate incoming audio buffer
    # ------------------------------------------------------------------------

    if indata is None:

      cloudlog.warning(
        "micd: received empty input buffer"
      )

      return


    if len(indata) == 0:

      return


    # ------------------------------------------------------------------------
    # Use first microphone channel
    # ------------------------------------------------------------------------

    audio_data = indata[:, 0]


    # ------------------------------------------------------------------------
    # Publish raw audio
    # ------------------------------------------------------------------------

    msg = messaging.new_message(
      "rawAudioData",
      valid=True,
    )

    audio_data_int_16 = (
      audio_data
      * 32767
    ).astype(
      np.int16
    )

    msg.rawAudioData.data = (
      audio_data_int_16.tobytes()
    )

    msg.rawAudioData.sampleRate = (
      SAMPLE_RATE
    )

    self.pm.send(
      "rawAudioData",
      msg,
    )


    # ------------------------------------------------------------------------
    # Accumulate samples for SPL calculation
    # ------------------------------------------------------------------------

    with self.lock:

      self.measurements = np.concatenate(
        (
          self.measurements,
          audio_data,
        )
      )


      # Process complete FFT windows.

      while (
        self.measurements.size
        >= FFT_SAMPLES
      ):

        measurements = (
          self.measurements[
            :FFT_SAMPLES
          ]
        )


        # --------------------------------------------------------------
        # Unweighted SPL
        # --------------------------------------------------------------

        self.sound_pressure, _ = (
          calculate_spl(
            measurements
          )
        )


        # --------------------------------------------------------------
        # A-weighted SPL
        # --------------------------------------------------------------

        measurements_weighted = (
          apply_a_weighting(
            measurements
          )
        )


        (
          self.sound_pressure_weighted,
          self.sound_pressure_level_weighted,
        ) = calculate_spl(
          measurements_weighted
        )


        # --------------------------------------------------------------
        # Remove processed samples
        # --------------------------------------------------------------

        self.measurements = (
          self.measurements[
            FFT_SAMPLES:
          ]
        )


  # ==========================================================================
  # Create microphone input stream
  # ==========================================================================

  @retry(
    attempts=10,
    delay=3,
  )
  def get_stream(self, sd):

    # IMPORTANT:
    #
    # Do NOT call:
    #
    #     sd._terminate()
    #     sd._initialize()
    #
    # These are internal PortAudio functions and are unnecessary here.
    #
    # The C3X audio device is explicitly selected instead.
    #
    # Device 3:
    #
    #     hw:0,3
    #     input channels = 2
    #     default samplerate = 48000
    #
    # micd requests:
    #
    #     1 channel
    #     16000 Hz
    #
    # sounddevice/ALSA will negotiate the requested stream parameters.
    #

    cloudlog.info(
      "micd: opening input stream "
      f"device={MIC_INPUT_DEVICE}, "
      f"samplerate={SAMPLE_RATE}, "
      f"blocksize={SAMPLE_BUFFER}"
    )


    # ------------------------------------------------------------------------
    # Query the selected input device
    # ------------------------------------------------------------------------

    try:

      device_info = sd.query_devices(
        MIC_INPUT_DEVICE
      )

      cloudlog.info(
        "micd: input device "
        f"{MIC_INPUT_DEVICE}: "
        f"{device_info}"
      )


      input_channels = int(
        device_info.get(
          "max_input_channels",
          0,
        )
      )


      device_rate = device_info.get(
        "default_samplerate",
        None,
      )


      if input_channels <= 0:

        raise RuntimeError(
          "micd: selected device "
          f"{MIC_INPUT_DEVICE} has no "
          "input channels"
        )


      cloudlog.info(
        "micd: device="
        f"{MIC_INPUT_DEVICE}, "
        f"input_channels={input_channels}, "
        f"device_rate={device_rate}, "
        f"requested_rate={SAMPLE_RATE}"
      )


    except Exception as e:

      cloudlog.warning(
        "micd: failed to query input "
        f"device {MIC_INPUT_DEVICE}: {e}"
      )

      raise


    # ------------------------------------------------------------------------
    # Create InputStream
    # ------------------------------------------------------------------------

    stream = sd.InputStream(
      device=MIC_INPUT_DEVICE,
      channels=1,
      samplerate=SAMPLE_RATE,
      callback=self.callback,
      blocksize=SAMPLE_BUFFER,
    )


    cloudlog.info(
      "micd: InputStream created "
      f"device={MIC_INPUT_DEVICE}, "
      f"samplerate={SAMPLE_RATE}, "
      f"channels=1, "
      f"blocksize={SAMPLE_BUFFER}"
    )


    return stream


  # ==========================================================================
  # Main microphone thread
  # ==========================================================================

  def micd_thread(self):

    # sounddevice must be imported after forking processes.

    import sounddevice as sd


    cloudlog.info(
      "micd: sounddevice version="
      f"{getattr(sd, '__version__', 'unknown')}"
    )


    cloudlog.info(
      "micd: starting "
      f"(input_device={MIC_INPUT_DEVICE}, "
      f"samplerate={SAMPLE_RATE})"
    )


    # Keep the compatibility patch.
    #
    # This only provides the ndarray conversion helper.
    # It does NOT initialize/terminate PortAudio.

    patch_sounddevice(sd)


    # ------------------------------------------------------------------------
    # Open microphone stream
    # ------------------------------------------------------------------------

    with self.get_stream(sd) as stream:

      cloudlog.info(
        "micd stream started: "
        f"{stream.samplerate=}, "
        f"{stream.channels=}, "
        f"{stream.dtype=}, "
        f"{stream.device=}, "
        f"{stream.blocksize=}"
      )


      # ----------------------------------------------------------------------
      # Main loop
      # ----------------------------------------------------------------------

      while True:

        self.update()


# ============================================================================
# Main
# ============================================================================

def main():

  mic = Mic()

  mic.micd_thread()


if __name__ == "__main__":
  main()
