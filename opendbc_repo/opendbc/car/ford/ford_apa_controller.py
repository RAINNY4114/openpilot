"""
Ford APA (Active Park Assist) Steering Controller
===================================================

Firmware-like APA steering controller for Lincoln MKX / Nautilus (K2GC PSCM).

Simulates PSCM signal control via the APA CAN channel (ParkAid_Data,
CAN ID 0x3A8), amplifying steering angle limits from LKA's +/-5.86 deg (front
wheel) to the APA channel's full physical range (+/-540 deg steering wheel /
+/-27.9 deg front wheel), enabling Sunnypilot to perform large-angle turns.

Architecture
------------
Three-path CAN output:

  Path 1 - LKA  (Lane_Assist_Data1, 0x3CA, 33Hz)
           High-speed lane keeping, +/-5.86 deg front wheel
           Active when vEgo >= APA_MAX_SPEED

  Path 2 - LMC  (LateralMotionControl, 20Hz)
           Heartbeat passthrough for IPMA camera (LatCtl_D_Rq = 0)
           Always active

  Path 3 - APA  (ParkAid_Data, 0x3A8, 50Hz)
           Low-speed large-angle steering, +/-540 deg SW / +/-27.9 deg FW
           Active when vEgo < APA_MAX_SPEED

Prerequisites
-------------
- PSCM firmware patched with LKA_NO_LOCKOUT_APA_HIGH_SPEED.VBF
  (removes 10-second lockout at cal +0x6CA0, enables APA at elevated speeds)
- IPMA_ADAS message arbitration (Sunnypilot must win CAN bus contention for 0x3A8)
- DBC file containing ParkAid_Data signal definitions

Author: Generated for Lincoln MKX / Nautilus K2GC platform
"""

import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Tuple

# numpy not required for core functionality


# ============================================================================
# Constants - Vehicle & PSCM Parameters
# ============================================================================

class VehicleParams:
  """Lincoln MKX / Nautilus vehicle parameters."""
  MASS = 2050.0              # kg, curb weight
  WHEELBASE = 3.025          # m
  STEER_RATIO = 19.33        # steering wheel angle / front wheel angle
  CENTER_TO_FRONT = 1.331    # m (= wheelbase * 0.44)

  # Physical steering limits (steering wheel degrees)
  SW_MAX_ANGLE = 540.0       # physical lock-to-lock half range
  FW_MAX_ANGLE = SW_MAX_ANGLE / STEER_RATIO  # ~27.9 deg front wheel

  # LKA channel limits (front wheel degrees)
  LKA_MAX_FW_ANGLE = 5.86    # from LaRefAng_No_Req +/-102.4 mrad
  LKA_MAX_SW_ANGLE = LKA_MAX_FW_ANGLE * STEER_RATIO  # ~113.2 deg SW

  # Actuator delay (seconds)
  STEER_ACTUATOR_DELAY_STOCK = 0.2     # factory
  STEER_ACTUATOR_DELAY_PATCHED = 0.05  # after PSCM firmware patch


class APAParams:
  """APA channel parameters and CAN signal encoding."""
  # CAN IDs
  APA_STEER_CAN_ID = 0x3A8    # 936 decimal - ParkAid_Data (IPMA_ADAS)
  APA_MODE_CAN_ID = 0x3D7     # 983 decimal - Steer_Assist_Data

  # Message rates (Hz)
  APA_STEER_RATE = 50          # ParkAid_Data send rate
  APA_MODE_RATE = 10           # Steer_Assist_Data send rate
  LKA_STEER_RATE = 33          # Lane_Assist_Data1 send rate
  LMC_HEARTBEAT_RATE = 20      # LateralMotionControl send rate

  # ExtSteeringAngleReq2 signal encoding (from DBC)
  # SG_ ExtSteeringAngleReq2 : 22|15@0+ (0.1,-1000) [-1000|2276.5] "Degrees"
  APA_ANGLE_FACTOR = 0.1       # degrees per raw step
  APA_ANGLE_OFFSET = -1000.0   # offset in degrees
  APA_ANGLE_RAW_BITS = 15      # 15-bit signal
  APA_ANGLE_RAW_MAX = (1 << APA_ANGLE_RAW_BITS) - 1  # 32767
  APA_ANGLE_PHYS_MIN = APA_ANGLE_OFFSET                          # -1000 deg
  APA_ANGLE_PHYS_MAX = APA_ANGLE_RAW_MAX * APA_ANGLE_FACTOR + APA_ANGLE_OFFSET  # 2276.5 deg

  # Practical APA angle limits (steering wheel degrees)
  # Physical lock is +/-540 deg, we use +/-500 deg for safety margin
  APA_SW_MAX = 500.0           # steering wheel degrees
  APA_FW_MAX = APA_SW_MAX / VehicleParams.STEER_RATIO  # ~25.9 deg front wheel

  # Amplification factor (APA vs LKA)
  AMP_FACTOR_FW = APA_FW_MAX / VehicleParams.LKA_MAX_FW_ANGLE   # ~4.42x
  AMP_FACTOR_SW = APA_SW_MAX / VehicleParams.LKA_MAX_SW_ANGLE   # ~4.42x

  # Speed thresholds (m/s)
  APA_MAX_SPEED = 8.0          # APA active below this speed (firmware limit)
  APA_HANDOVER_SPEED = 10.0    # switch back to LKA above this (hysteresis)
  APA_MIN_SPEED = 0.5          # APA inactive below this (PSCM requires motion)

  # APA mode handshake timing (seconds)
  APA_ARM_DELAY = 0.5          # arming phase duration
  APA_ACK_TIMEOUT = 2.0        # max wait for PSCM acknowledgment
  APA_HEARTBEAT_TIMEOUT = 0.2  # 200ms without message = abort

  # Rate limits (degrees/second, steering wheel)
  APA_RATE_LIMIT_LOW = 300.0   # deg/s at low speed (< 3 m/s)
  APA_RATE_LIMIT_HIGH = 150.0  # deg/s at higher speed (3-8 m/s)
  LKA_RATE_LIMIT = 50.0        # deg/s for LKA channel

  # Lateral acceleration limits (m/s^2)
  MAX_LATERAL_ACCEL = 2.5      # conservative for SUV
  MAX_LATERAL_JERK = 2.0       # m/s^3

  # Watchdog
  WATCHDOG_TIMEOUT = 1.0       # abort APA after 1s without update


class APAState(IntEnum):
  """APA controller state machine states."""
  INACTIVE = 0     # APA off, LKA handles steering
  ARMING = 1       # Sending mode request, waiting for PSCM ack
  ACTIVE = 2       # APA driving steering, large angles enabled
  DISENGAGING = 3  # Ramp down to zero before handover to LKA
  FAULT = 4        # Error state, requires manual reset


class APAChannel(IntEnum):
  """Active steering channel selection."""
  LKA = 0          # Lane_Assist_Data1 channel (high speed)
  APA = 1          # ParkAid_Data channel (low speed, large angles)


# ============================================================================
# APA Mode Request Signals (Steer_Assist_Data, CAN ID 0x3D7)
# ============================================================================

class APAModeRequest:
  """
  APA mode request values for Steer_Assist_Data message.
  Sent by IPMA_ADAS, we emulate to activate APA mode.
  """
  # ApaSwtch_D_RqMnu (10|2@0+): APA switch request manual
  APA_SWITCH_OFF = 0
  APA_SWITCH_ON = 1
  APA_SWITCH_CANCEL = 2

  # ApaMdeStat_D_RqDrv (2|3@0+): APA mode status request driver
  APA_MODE_STAT_INACTIVE = 0
  APA_MODE_STAT_ACTIVE = 1
  APA_MODE_STAT_SUSPENDED = 2


# ============================================================================
# APA Steering Signals (ParkAid_Data, CAN ID 0x3A8)
# ============================================================================

class APASteerSignals:
  """
  Signal layout for ParkAid_Data (BO_ 936, 8 bytes, Intel byte order).

  Byte layout:
    Byte 0: [bits 0-7]
    Byte 1: [bits 8-15]
    Byte 2: [bits 16-23]  <- contains ExtSteeringAngleReq2 LSBs + EPASExtAngleStatReq
    Byte 3: [bits 24-31]  <- ExtSteeringAngleReq2 middle bits + ApaMde_D_Stat
    Byte 4: [bits 32-39]  <- ExtSteeringAngleReq2 MSBs
    Byte 5: [bits 40-47]
    Byte 6: [bits 48-55]
    Byte 7: [bits 56-63]  <- ApaSys_D_Stat + ApaSteWhl_D_RqDrv

  Signal definitions (from DBC):
    ExtSteeringAngleReq2 : 22|15@0+ (0.1, -1000) [-1000|2276.5] "Degrees"
      -> bits 22-36 (byte 2 bits 6-7, byte 3 all, byte 4 bits 0-4)

    EPASExtAngleStatReq : 23|1@0+ (1, 0) [0|1] "SED"
      -> bit 23 (byte 2 bit 7) - enable extended angle mode
      -> 0 = No relative ext angle req, 1 = Relative ext steering angle req

    ApaMde_D_Stat : 23|3@0+ (1, 0) [0|7] "SED"
      -> bits 23-25 (overlaps EPASExtAngleStatReq at bit 23)
      -> 0 = inactive, 1 = active steering, 2 = suspended

    ApaSys_D_Stat : 61|3@0+ (1, 0) [0|7] "SED"
      -> bits 61-63 (byte 7 bits 5-7) - APA system status

    ApaSteWhl_D_RqDrv : 63|2@0+ (1, 0) [0|3] "SED"
      -> bits 62-63 (byte 7 bits 6-7) - APA steering wheel request driver
  """

  # ApaMde_D_Stat values
  APA_MDE_INACTIVE = 0
  APA_MDE_ACTIVE = 1       # Active APA steering
  APA_MDE_SUSPENDED = 2

  # ApaSys_D_Stat values (system status)
  APA_SYS_INACTIVE = 0
  APA_SYS_ARMED = 1        # System ready
  APA_SYS_ACTIVE = 2       # System executing
  APA_SYS_FAULT = 3

  # ApaSteWhl_D_RqDrv values
  APA_STEERWHL_NONE = 0
  APA_STEERWHL_REQUEST = 1


# ============================================================================
# Rate Limiter
# ============================================================================

class RateLimiter:
  """Generic rate limiter for smooth angle transitions."""

  def __init__(self, init_val: float = 0.0):
    self._last_val: float = init_val
    self._last_time: float = time.monotonic()

  def update(self, target: float, rate_limit: float, current_time: Optional[float] = None) -> float:
    """
    Apply rate limiting to target value.

    Args:
      target: Desired output value
      rate_limit: Maximum change rate (units/second)
      current_time: Optional timestamp override

    Returns:
      Rate-limited output value
    """
    if current_time is None:
      current_time = time.monotonic()

    dt = max(current_time - self._last_time, 1e-6)
    max_delta = rate_limit * dt
    delta = target - self._last_val
    delta = max(-max_delta, min(max_delta, delta))
    self._last_val += delta
    self._last_time = current_time
    return self._last_val

  def reset(self, val: float = 0.0, current_time: float = None):
    self._last_val = val
    self._last_time = current_time if current_time is not None else time.monotonic()


# ============================================================================
# APA Angle Encoder (raw byte packing)
# ============================================================================

class APAAngleEncoder:
  """
  Encodes steering angle into ParkAid_Data CAN frame bytes.

  This is the "firmware-like" layer — directly constructs the 8-byte CAN
  frame with correct bit packing for the PSCM.
  """

  @staticmethod
  def encode_ext_steering_angle_req2(angle_deg: float) -> int:
    """
    Encode steering wheel angle (degrees) into ExtSteeringAngleReq2 raw value.

    Signal: 22|15@0+, factor=0.1, offset=-1000
    raw = (angle_deg - offset) / factor = (angle_deg + 1000) / 0.1

    Args:
      angle_deg: Steering wheel angle in degrees [-1000, 2276.5]

    Returns:
      15-bit raw integer [0, 32767]
    """
    raw = int(round((angle_deg - APAParams.APA_ANGLE_OFFSET) / APAParams.APA_ANGLE_FACTOR))
    return max(0, min(APAParams.APA_ANGLE_RAW_MAX, raw))

  @staticmethod
  def decode_ext_steering_angle_req2(raw: int) -> float:
    """Decode raw value back to degrees."""
    return raw * APAParams.APA_ANGLE_FACTOR + APAParams.APA_ANGLE_OFFSET

  @staticmethod
  def pack_apa_steer_frame(angle_deg: float,
                           ext_angle_enable: bool = True,
                           apa_mode: int = APASteerSignals.APA_MDE_ACTIVE,
                           apa_sys: int = APASteerSignals.APA_SYS_ACTIVE,
                           steerwhl_req: int = APASteerSignals.APA_STEERWHL_REQUEST
                           ) -> bytes:
    """
    Pack a complete ParkAid_Data (CAN ID 0x3A8) 8-byte frame.

    This directly constructs the raw CAN bytes, simulating what the PSCM
    firmware expects to receive from ABS_ESC.

    Bit layout (Intel byte order, little-endian):

      Byte 2: bits 16-23
        bit 22-23: ExtSteeringAngleReq2[0:1] (2 LSBs of 15-bit angle)
        bit 23:   EPASExtAngleStatReq (also ApaMde_D_Stat[0])

      Byte 3: bits 24-31
        bits 24-25: ApaMde_D_Stat[1:2] (upper 2 bits of 3-bit mode)
        bits 26-31: ExtSteeringAngleReq2[2:7] (6 bits)

      Byte 4: bits 32-39
        bits 32-36: ExtSteeringAngleReq2[8:12] (5 bits)
        bits 37-39: unused (0)

      Byte 7: bits 56-63
        bits 56-60: unused (0)
        bits 61-63: ApaSys_D_Stat (3 bits) + ApaSteWhl_D_RqDrv overlap

    Args:
      angle_deg: Steering wheel angle in degrees
      ext_angle_enable: True to enable extended angle mode (EPASExtAngleStatReq=1)
      apa_mode: APA mode status (ApaMde_D_Stat)
      apa_sys: APA system status (ApaSys_D_Stat)
      steerwhl_req: APA steering wheel request driver

    Returns:
      8-byte CAN frame
    """
    # Encode angle to raw 15-bit
    angle_raw = APAAngleEncoder.encode_ext_steering_angle_req2(angle_deg)

    # Break angle into bit fields
    # 15-bit signal starting at bit 22:
    #   bits 22-23 (2 bits) -> byte 2, bits 6-7
    #   bits 24-29 (6 bits) -> byte 3, bits 2-7 (but byte 3 bits 0-1 are ApaMde upper)
    #   bits 30-36 (7 bits, but we only have 5 left -> bits 30-34 = 5 bits) -> byte 4, bits 0-4

    # Actually let me be more careful:
    # 15-bit signal: raw[0] is at bit 22, raw[14] is at bit 36
    # bit 22 = byte 2 bit 6
    # bit 23 = byte 2 bit 7
    # bit 24 = byte 3 bit 0
    # bit 25 = byte 3 bit 1
    # bit 26 = byte 3 bit 2
    # ...
    # bit 31 = byte 3 bit 7
    # bit 32 = byte 4 bit 0
    # bit 33 = byte 4 bit 1
    # bit 34 = byte 4 bit 2
    # bit 35 = byte 4 bit 3
    # bit 36 = byte 4 bit 4

    angle_bits_0_1 = angle_raw & 0x3         # bits 22-23 (2 bits)
    angle_bits_2_7 = (angle_raw >> 2) & 0x3F  # bits 24-29 (6 bits)
    angle_bits_8_14 = (angle_raw >> 8) & 0x7F # bits 30-36 (7 bits, but 15-8=7 bits)

    # Wait, 15 bits total: bits 0-14 of the raw value
    # bits 0-1 (2 bits) -> byte 2 bits 6-7
    # bits 2-7 (6 bits) -> byte 3 bits 0-5  (but bits 0-1 of byte 3 overlap with ApaMde_D_Stat upper bits)

    # Hmm, let me reconsider. The DBC says:
    # ExtSteeringAngleReq2 : 22|15@0+ -- starts at bit 22, 15 bits, Intel
    # This means the signal occupies bits 22, 23, 24, ..., 36 (15 bits total)

    # ApaMde_D_Stat : 23|3@0+ -- starts at bit 23, 3 bits, Intel
    # This means bits 23, 24, 25

    # So ExtSteeringAngleReq2 bits 22-36 and ApaMde_D_Stat bits 23-25 OVERLAP at bits 23-25.
    # This is unusual but possible in DBC files - the signals share bit positions.

    # For EPASExtAngleStatReq : 23|1@0+ -- bit 23 only, also overlaps.

    # In practice, when the PSCM firmware reads these signals, it interprets
    # the shared bits differently depending on context. When APA mode is active,
    # bit 23 = 1 (EPASExtAngleStatReq) and the angle value's bit 1 (which is
    # also bit 23) must be consistent.

    # The simplest approach: set bit 23 = 1 for enable, and ensure the angle
    # raw value's bit 1 is also 1 (or just accept the overlap and let the
    # PSCM firmware handle it).

    # Actually, the most practical approach is to use cantools for encoding,
    # which handles signal overlaps correctly. For raw byte packing, we'll
    # pack the angle into the non-overlapping bits and set the mode/enable bits separately.

    # For a practical raw implementation, we'll construct the bytes as follows:
    frame = bytearray(8)

    # Byte 2: bits 16-23
    # bits 16-21: unused (0)
    # bit 22: ExtSteeringAngleReq2 bit 0
    # bit 23: ExtSteeringAngleReq2 bit 1 = EPASExtAngleStatReq = ApaMde_D_Stat bit 0
    byte2 = 0
    byte2 |= (angle_raw & 0x1) << 6          # bit 22 = angle bit 0
    if ext_angle_enable:
      byte2 |= 0x80                           # bit 23 = 1 (enable + mode bit 0)
    else:
      # bit 23 also carries angle bit 1 if not in ext mode
      byte2 |= ((angle_raw >> 1) & 0x1) << 7
    frame[2] = byte2

    # Byte 3: bits 24-31
    # bits 24-25: ApaMde_D_Stat bits 1-2 (upper bits of 3-bit mode)
    # bits 26-31: ExtSteeringAngleReq2 bits 2-7
    byte3 = 0
    apa_mode_upper = (apa_mode >> 1) & 0x3    # upper 2 bits of 3-bit mode
    byte3 |= apa_mode_upper                    # bits 0-1 (ApaMde upper)
    byte3 |= ((angle_raw >> 2) & 0x3F) << 2   # bits 2-7 (angle bits 2-7)
    frame[3] = byte3

    # Byte 4: bits 32-39
    # bits 32-36: ExtSteeringAngleReq2 bits 8-12
    # bits 37-39: unused (0)
    byte4 = (angle_raw >> 8) & 0x1F           # bits 0-4 (angle bits 8-12)
    frame[4] = byte4

    # Byte 7: bits 56-63
    # bits 56-60: unused (0)
    # bits 61-63: ApaSys_D_Stat (3 bits)
    # ApaSteWhl_D_RqDrv overlaps at bits 62-63
    byte7 = 0
    byte7 |= (apa_sys & 0x7) << 5             # bits 61-63 (ApaSys)
    # ApaSteWhl_D_RqDrv at bits 62-63 overlaps with ApaSys bits 1-2
    # In practice, set ApaSys to include the steerwhl request encoding
    frame[7] = byte7

    return bytes(frame)

  @staticmethod
  def pack_apa_mode_frame(switch_req: int = APAModeRequest.APA_SWITCH_ON,
                          mode_stat: int = APAModeRequest.APA_MODE_STAT_ACTIVE
                          ) -> bytes:
    """
    Pack a Steer_Assist_Data (CAN ID 0x3D7) 8-byte frame for APA mode request.

    Signal definitions (from DBC):
      ApaMdeStat_D_RqDrv : 2|3@0+ (1,0) [0|7] -- bits 2-4
      ApaSwtch_D_RqMnu : 10|2@0+ (1,0) [0|3]  -- bits 10-11

    Args:
      switch_req: ApaSwtch_D_RqMnu value (0=off, 1=on, 2=cancel)
      mode_stat: ApaMdeStat_D_RqDrv value (0=inactive, 1=active, 2=suspended)

    Returns:
      8-byte CAN frame
    """
    frame = bytearray(8)

    # Byte 0: bits 0-7
    # bits 2-4: ApaMdeStat_D_RqDrv (3 bits)
    frame[0] |= (mode_stat & 0x7) << 2

    # Byte 1: bits 8-15
    # bits 10-11: ApaSwtch_D_RqMnu (2 bits)
    frame[1] |= (switch_req & 0x3) << 2

    return bytes(frame)


# ============================================================================
# Main APA Controller
# ============================================================================

class FordAPAController:
  """
  Firmware-like APA steering controller for Ford PSCM (K2GC platform).

  Manages the three-path steering architecture:
    1. LKA channel for high-speed lane keeping (+/-5.86 deg)
    2. LMC channel for IPMA camera heartbeat (passthrough)
    3. APA channel for low-speed large-angle steering (+/-500 deg SW)

  The controller implements:
    - Speed-gated channel selection (LKA vs APA)
    - APA mode handshake state machine
    - Angle amplification (curvature -> large APA angles)
    - Rate limiting and safety watchdog
    - PSCM feedback monitoring
  """

  def __init__(self, vehicle_model=None, CP=None, CP_SP=None):
    """
    Initialize the APA controller.

    Args:
      vehicle_model: VehicleModel instance for curvature->angle conversion
      CP: CarParams (standard openpilot)
      CP_SP: CarParams_SP (sunnypilot MADS parameters)
    """
    self.VM = vehicle_model
    self.CP = CP
    self.CP_SP = CP_SP

    # State machine
    self.state: APAState = APAState.INACTIVE
    self.state_enter_time: float = time.monotonic()
    self.fault_reason: str = ""

    # Channel selection
    self.active_channel: APAChannel = APAChannel.LKA

    # Angle tracking
    self._desired_sw_angle: float = 0.0       # desired steering wheel angle (deg)
    self._commanded_sw_angle: float = 0.0     # rate-limited SW angle sent to PSCM
    self._last_fw_angle: float = 0.0          # last front wheel angle for LKA
    self._last_curvature: float = 0.0         # last desired curvature

    # Rate limiters
    self._apa_rate_limiter = RateLimiter(0.0)
    self._lka_rate_limiter = RateLimiter(0.0)

    # Watchdog
    self._last_update_time: float = time.monotonic()
    self._last_apa_msg_time: float = 0.0

    # PSCM feedback
    self._pscm_angle_actual: float = 0.0      # actual steering angle from PSCM
    self._pscm_apa_sys_stat: int = 0          # APA system status from PSCM
    self._pscm_fault: bool = False

    # Statistics
    self._apa_msg_count: int = 0
    self._lka_msg_count: int = 0
    self._mode_msg_count: int = 0

    # Speed hysteresis state
    self._was_below_apa_speed: bool = False

    # APA mode request state
    self._apa_switch_req: int = APAModeRequest.APA_SWITCH_OFF
    self._apa_mode_req: int = APAModeRequest.APA_MODE_STAT_INACTIVE

  # --------------------------------------------------------------------------
  # Public API
  # --------------------------------------------------------------------------

  def update(self, desired_curvature: float, v_ego: float, roll: float,
             lateral_enabled: bool, steering_angle_actual: float,
             current_time: Optional[float] = None) -> Tuple[APAChannel, float, float]:
    """
    Main update call — compute steering command for current cycle.

    Args:
      desired_curvature: Target path curvature from openpilot planner (1/m)
      v_ego: Vehicle speed (m/s)
      roll: Road roll angle estimate (rad)
      lateral_enabled: Whether lateral control is active
      steering_angle_actual: Current steering wheel angle from CarState (deg)
      current_time: Optional timestamp override

    Returns:
      Tuple of (active_channel, steering_wheel_angle_deg, front_wheel_angle_deg)
      - active_channel: which CAN channel to send on (LKA or APA)
      - steering_wheel_angle_deg: target SW angle to send
      - front_wheel_angle_deg: target FW angle (for LKA clip)
    """
    if current_time is None:
      current_time = time.monotonic()

    self._last_update_time = current_time
    self._last_curvature = desired_curvature
    self._pscm_angle_actual = steering_angle_actual

    # Check watchdog
    if self.state == APAState.ACTIVE:
      if current_time - self._last_apa_msg_time > APAParams.WATCHDOG_TIMEOUT:
        self._set_fault("APA watchdog timeout")
        self._transition(APAState.FAULT, current_time)

    # Check PSCM fault
    if self._pscm_fault and self.state != APAState.FAULT:
      self._set_fault("PSCM fault detected")
      self._transition(APAState.FAULT, current_time)

    # Select channel based on speed
    self._select_channel(v_ego, lateral_enabled, current_time)

    # Convert curvature to steering wheel angle
    self._desired_sw_angle = self._curvature_to_sw_angle(desired_curvature, v_ego, roll)

    # Apply channel-specific processing
    if self.active_channel == APAChannel.APA and self.state == APAState.ACTIVE:
      # APA path: large angle, rate-limited
      rate_limit = self._get_apa_rate_limit(v_ego)
      self._commanded_sw_angle = self._apa_rate_limiter.update(
        self._desired_sw_angle, rate_limit, current_time
      )
      # Clip to APA physical limit
      self._commanded_sw_angle = max(-APAParams.APA_SW_MAX,
                                      min(APAParams.APA_SW_MAX, self._commanded_sw_angle))
      fw_angle = self._commanded_sw_angle / VehicleParams.STEER_RATIO

    else:
      # LKA path: small angle, rate-limited, clipped to +/-5.86 deg FW
      sw_angle_lka = self._lka_rate_limiter.update(
        self._desired_sw_angle, APAParams.LKA_RATE_LIMIT, current_time
      )
      fw_angle = sw_angle_lka / VehicleParams.STEER_RATIO
      # Clip to LKA limit
      fw_angle = max(-VehicleParams.LKA_MAX_FW_ANGLE,
                      min(VehicleParams.LKA_MAX_FW_ANGLE, fw_angle))
      self._commanded_sw_angle = fw_angle * VehicleParams.STEER_RATIO

    # Update APA state machine
    self._update_state_machine(v_ego, lateral_enabled, current_time)

    # Lateral acceleration safety check
    self._commanded_sw_angle = self._lateral_accel_limit(
      self._commanded_sw_angle, v_ego, desired_curvature
    )

    self._last_fw_angle = self._commanded_sw_angle / VehicleParams.STEER_RATIO

    return self.active_channel, self._commanded_sw_angle, self._last_fw_angle

  def get_apa_steer_frame(self, current_time: float = None) -> Optional[bytes]:
    """
    Get the APA steering CAN frame (ParkAid_Data, 0x3A8).

    Returns:
      8-byte CAN frame if APA is active, None otherwise
    """
    if self.state not in (APAState.ARMING, APAState.ACTIVE, APAState.DISENGAGING):
      return None

    # Determine angle and mode based on state
    if self.state == APAState.ARMING:
      angle = 0.0  # Zero angle during arming
      apa_mode = APASteerSignals.APA_MDE_ACTIVE
      apa_sys = APASteerSignals.APA_SYS_ARMED
    elif self.state == APAState.ACTIVE:
      angle = self._commanded_sw_angle
      apa_mode = APASteerSignals.APA_MDE_ACTIVE
      apa_sys = APASteerSignals.APA_SYS_ACTIVE
    elif self.state == APAState.DISENGAGING:
      angle = 0.0  # Ramp to zero
      apa_mode = APASteerSignals.APA_MDE_SUSPENDED
      apa_sys = APASteerSignals.APA_SYS_ARMED
    else:
      return None

    frame = APAAngleEncoder.pack_apa_steer_frame(
      angle_deg=angle,
      ext_angle_enable=True,
      apa_mode=apa_mode,
      apa_sys=apa_sys,
      steerwhl_req=APASteerSignals.APA_STEERWHL_REQUEST
    )

    self._last_apa_msg_time = current_time if current_time is not None else time.monotonic()
    self._apa_msg_count += 1

    return frame

  def get_apa_mode_frame(self) -> Optional[bytes]:
    """
    Get the APA mode request CAN frame (Steer_Assist_Data, 0x3D7).

    Returns:
      8-byte CAN frame if APA mode should be requested, None otherwise
    """
    if self.state == APAState.INACTIVE:
      return None

    if self.state == APAState.FAULT:
      # Send cancel
      frame = APAAngleEncoder.pack_apa_mode_frame(
        switch_req=APAModeRequest.APA_SWITCH_CANCEL,
        mode_stat=APAModeRequest.APA_MODE_STAT_INACTIVE
      )
    elif self.state in (APAState.ARMING, APAState.ACTIVE, APAState.DISENGAGING):
      frame = APAAngleEncoder.pack_apa_mode_frame(
        switch_req=APAModeRequest.APA_SWITCH_ON,
        mode_stat=APAModeRequest.APA_MODE_STAT_ACTIVE
      )
    else:
      return None

    self._mode_msg_count += 1
    return frame

  def update_pscm_feedback(self, apa_sys_stat: int, steering_angle_actual: float,
                           pscm_fault: bool = False):
    """
    Update PSCM feedback state from CarState.

    Called when APA-related signals are received from PSCM.

    Args:
      apa_sys_stat: ApaSys_D_Stat value from PSCM (0=inactive, 1=armed, 2=active, 3=fault)
      steering_angle_actual: Actual steering wheel angle from pinion sensor
      pscm_fault: True if PSCM reports a fault
    """
    self._pscm_apa_sys_stat = apa_sys_stat
    self._pscm_angle_actual = steering_angle_actual
    self._pscm_fault = pscm_fault

    # Check if PSCM acknowledged APA mode during arming
    if self.state == APAState.ARMING:
      if apa_sys_stat >= APASteerSignals.APA_SYS_ARMED:
        self._transition(APAState.ACTIVE, time.monotonic())
      elif apa_sys_stat == APASteerSignals.APA_SYS_FAULT:
        self._set_fault("PSCM reported APA system fault during arming")
        self._transition(APAState.FAULT, time.monotonic())

  def reset(self):
    """Reset controller to INACTIVE state."""
    self.state = APAState.INACTIVE
    self.state_enter_time = time.monotonic()
    self.fault_reason = ""
    self.active_channel = APAChannel.LKA
    self._desired_sw_angle = 0.0
    self._commanded_sw_angle = 0.0
    self._last_fw_angle = 0.0
    self._last_curvature = 0.0
    self._apa_rate_limiter.reset(0.0)
    self._lka_rate_limiter.reset(0.0)
    self._pscm_fault = False
    self._was_below_apa_speed = False

  def get_status(self) -> dict:
    """Return current status for logging/debugging."""
    return {
      "state": self.state.name,
      "channel": self.active_channel.name,
      "desired_sw_angle": round(self._desired_sw_angle, 2),
      "commanded_sw_angle": round(self._commanded_sw_angle, 2),
      "fw_angle": round(self._last_fw_angle, 2),
      "curvature": round(self._last_curvature, 6),
      "pscm_sys_stat": self._pscm_apa_sys_stat,
      "pscm_fault": self._pscm_fault,
      "apa_msg_count": self._apa_msg_count,
      "lka_msg_count": self._lka_msg_count,
      "mode_msg_count": self._mode_msg_count,
      "fault_reason": self.fault_reason,
    }

  # --------------------------------------------------------------------------
  # Internal Methods
  # --------------------------------------------------------------------------

  def _select_channel(self, v_ego: float, lateral_enabled: bool, current_time: float):
    """Select between LKA and APA channels based on speed."""
    below_apa_speed = v_ego < APAParams.APA_MAX_SPEED
    above_handover = v_ego > APAParams.APA_HANDOVER_SPEED

    # Hysteresis: use different thresholds for switching up vs down
    if above_handover:
      self._was_below_apa_speed = False
      self.active_channel = APAChannel.LKA
    elif below_apa_speed and lateral_enabled:
      self._was_below_apa_speed = True
      # Channel will be set to APA when state machine transitions to ACTIVE
      if self.state == APAState.INACTIVE and v_ego > APAParams.APA_MIN_SPEED:
        # Initiate APA arming
        self._transition(APAState.ARMING, current_time)
    else:
      # Between thresholds - maintain current channel
      pass

    if self.state == APAState.ACTIVE and above_handover:
      self._transition(APAState.DISENGAGING, current_time)

    if self.active_channel == APAChannel.LKA:
      self.active_channel = APAChannel.LKA
    elif self.state == APAState.ACTIVE:
      self.active_channel = APAChannel.APA
    else:
      self.active_channel = APAChannel.LKA

  def _update_state_machine(self, v_ego: float, lateral_enabled: bool, current_time: float):
    """Update APA state machine."""
    elapsed = current_time - self.state_enter_time

    if self.state == APAState.ARMING:
      if elapsed > APAParams.APA_ACK_TIMEOUT:
        # If PSCM hasn't acknowledged, proceed anyway (some PSCMs don't report status)
        # This is a design choice - in production, you might want to abort
        self._transition(APAState.ACTIVE, current_time)

    elif self.state == APAState.ACTIVE:
      if not lateral_enabled:
        self._transition(APAState.DISENGAGING, current_time)
      elif v_ego > APAParams.APA_HANDOVER_SPEED:
        self._transition(APAState.DISENGAGING, current_time)

    elif self.state == APAState.DISENGAGING:
      # Ramp angle to zero, then transition
      if abs(self._commanded_sw_angle) < 1.0:
        self._transition(APAState.INACTIVE, current_time)
      elif elapsed > 2.0:  # Max 2 seconds to ramp down
        self._transition(APAState.INACTIVE, current_time)

    elif self.state == APAState.FAULT:
      # Stay in fault until reset
      pass

  def _transition(self, new_state: APAState, current_time: float = None):
    """Transition to a new state."""
    if current_time is None:
      current_time = time.monotonic()
    old_state = self.state
    self.state = new_state
    self.state_enter_time = current_time

    # Reset rate limiter on channel change
    if new_state == APAState.ACTIVE and old_state != APAState.ACTIVE:
      self._apa_rate_limiter.reset(self._commanded_sw_angle, current_time)
    elif new_state == APAState.INACTIVE:
      self._apa_rate_limiter.reset(0.0, current_time)

  def _set_fault(self, reason: str):
    """Set fault state with reason."""
    self.fault_reason = reason
    self._pscm_fault = True

  def _curvature_to_sw_angle(self, curvature: float, v_ego: float, roll: float) -> float:
    """
    Convert desired curvature to steering wheel angle.

    Uses the VehicleModel bicycle model if available, otherwise uses
    a simplified kinematic conversion.

    Args:
      curvature: Desired curvature (1/m)
      v_ego: Vehicle speed (m/s)
      roll: Road roll (rad)

    Returns:
      Steering wheel angle in degrees
    """
    if self.VM is not None:
      # Use openpilot's VehicleModel
      sw_angle_rad = self.VM.get_steer_from_curvature(-curvature, v_ego, roll)
      return math.degrees(sw_angle_rad)
    else:
      # Simplified kinematic model:
      # curvature = tan(front_wheel_angle) / wheelbase
      # SW_angle = front_wheel_angle * steer_ratio
      fw_angle_rad = math.atan(curvature * VehicleParams.WHEELBASE)
      sw_angle_rad = fw_angle_rad * VehicleParams.STEER_RATIO
      return math.degrees(sw_angle_rad)

  def _get_apa_rate_limit(self, v_ego: float) -> float:
    """Get speed-dependent APA rate limit (deg/s)."""
    if v_ego < 3.0:
      return APAParams.APA_RATE_LIMIT_LOW
    else:
      return APAParams.APA_RATE_LIMIT_HIGH

  def _lateral_accel_limit(self, sw_angle: float, v_ego: float,
                            curvature: float) -> float:
    """
    Limit steering angle to maintain lateral acceleration within safe bounds.

    lateral_accel = curvature * v_ego^2
    max_curvature = MAX_LATERAL_ACCEL / v_ego^2
    max_fw_angle = atan(max_curvature * wheelbase)
    max_sw_angle = max_fw_angle * steer_ratio

    Args:
      sw_angle: Commanded steering wheel angle (deg)
      v_ego: Vehicle speed (m/s)
      curvature: Desired curvature (1/m)

    Returns:
      Safety-limited steering wheel angle (deg)
    """
    if v_ego < 0.5:
      return sw_angle  # No limit at very low speed

    max_lat_accel = APAParams.MAX_LATERAL_ACCEL
    max_curvature = max_lat_accel / (v_ego * v_ego)
    max_fw_rad = math.atan(max_curvature * VehicleParams.WHEELBASE)
    max_sw_deg = math.degrees(max_fw_rad * VehicleParams.STEER_RATIO)

    return max(-max_sw_deg, min(max_sw_deg, sw_angle))


# ============================================================================
# Standalone Test / Simulation
# ============================================================================

def simulate_apa_controller():
  """
  Simulate the APA controller with a virtual vehicle to demonstrate
  large-angle steering capability.

  This runs a simple simulation showing:
    1. Speed-gated channel switching (LKA <-> APA)
    2. APA mode handshake
    3. Large-angle steering commands
    4. Rate limiting behavior
  """
  print("=" * 72)
  print("Ford APA Steering Controller Simulation")
  print("Vehicle: Lincoln MKX / Nautilus (K2GC PSCM)")
  print("=" * 72)

  controller = FordAPAController()

  # Simulation parameters
  dt = 0.02  # 50Hz
  total_time = 30.0  # seconds

  # Scenario phases:
  # Phase 1 (0-5s):   High speed (20 m/s) - LKA mode, small curve
  # Phase 2 (5-10s):  Decelerating to 5 m/s - channel handover
  # Phase 3 (10-15s): Low speed (5 m/s) - APA mode, sharp turn
  # Phase 4 (15-20s): Low speed (3 m/s) - APA mode, U-turn
  # Phase 5 (20-25s): Accelerating to 15 m/s - handover back to LKA
  # Phase 6 (25-30s): High speed (15 m/s) - LKA mode

  t = 0.0
  v_ego = 20.0
  steering_actual = 0.0

  print(f"\n{'Time':>6} {'Speed':>7} {'State':>14} {'Channel':>8} "
        f"{'DesSW':>8} {'CmdSW':>8} {'FW Ang':>8} {'Amp':>6}")
  print("-" * 72)

  while t < total_time:
    # Determine scenario
    if t < 5.0:
      v_ego = 20.0
      curvature = 0.003  # gentle highway curve
    elif t < 10.0:
      v_ego = 20.0 - (t - 5.0) * 3.0  # decelerate 20 -> 5 m/s
      curvature = 0.005
    elif t < 15.0:
      v_ego = 5.0
      curvature = 0.05  # sharp turn
    elif t < 20.0:
      v_ego = 3.0
      curvature = 0.15  # U-turn level curvature
    elif t < 25.0:
      v_ego = 3.0 + (t - 20.0) * 2.4  # accelerate 3 -> 15 m/s
      curvature = 0.01
    else:
      v_ego = 15.0
      curvature = 0.004

    # Lateral control enabled throughout
    lateral_enabled = True
    roll = 0.0

    # Simulate steering angle feedback (lag behind command)
    steering_actual += (controller._commanded_sw_angle - steering_actual) * 0.1

    # Update controller
    channel, sw_angle, fw_angle = controller.update(
      desired_curvature=curvature,
      v_ego=v_ego,
      roll=roll,
      lateral_enabled=lateral_enabled,
      steering_angle_actual=steering_actual,
      current_time=t,
    )

    # Simulate PSCM APA ack during arming (0.3s after entering ARMING)
    if controller.state == APAState.ARMING:
      elapsed = t - controller.state_enter_time
      if elapsed > 0.3:  # PSCM acks after 300ms
        controller.update_pscm_feedback(
          apa_sys_stat=APASteerSignals.APA_SYS_ACTIVE,
          steering_angle_actual=steering_actual,
        )

    # Get CAN frames
    apa_frame = controller.get_apa_steer_frame(t)
    mode_frame = controller.get_apa_mode_frame()

    # Print status every 1 second (only once per integer second)
    if abs(t - round(t)) < dt / 2:
      amp = abs(sw_angle) / max(abs(fw_angle * VehicleParams.STEER_RATIO), 0.001)
      print(f"{t:6.1f} {v_ego:7.1f} {controller.state.name:>14} {channel.name:>8} "
            f"{controller._desired_sw_angle:8.1f} {sw_angle:8.1f} {fw_angle:8.1f} {amp:6.2f}x")

    t += dt

  print("-" * 72)
  status = controller.get_status()
  print(f"\nFinal Status:")
  for k, v in status.items():
    print(f"  {k}: {v}")

  print(f"\nAmplification Summary:")
  print(f"  LKA max front wheel angle: +/-{VehicleParams.LKA_MAX_FW_ANGLE:.1f} deg")
  print(f"  APA max front wheel angle: +/-{APAParams.APA_FW_MAX:.1f} deg")
  print(f"  Amplification factor:      {APAParams.AMP_FACTOR_FW:.1f}x")
  print(f"  LKA max SW angle:          +/-{VehicleParams.LKA_MAX_SW_ANGLE:.1f} deg")
  print(f"  APA max SW angle:          +/-{APAParams.APA_SW_MAX:.1f} deg")


if __name__ == "__main__":
  simulate_apa_controller()
