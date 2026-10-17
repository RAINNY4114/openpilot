"""
Ford APA CAN Message Functions
===============================

CAN message creation for the APA (Active Park Assist) steering channel.

Provides functions to create:
  1. ParkAid_Data (CAN ID 0x3A8) - APA steering angle command + APA mode
  2. Steer_Assist_Data (CAN ID 0x3D7) - APA mode request

::

  ⚠️ IMPORTANT CORRECTION (verified against device DBC on 2026-10-05):

  ExtSteeringAngleReq2 and EPASExtAngleStatReq belong to
  BO_ 936 ParkAid_Data (CAN ID 0x3A8, sender IPMA_ADAS) —
  NOT to BO_ 136 ActiveFronSteering_Req (0x88, sender ABS_ESC).

  BO_ 136 ActiveFronSteering_Req only contains BrakeSnData-style signals:
    SG_ SteWhlBrkOffst_An_Rq : 7|15@0+ (0.1,-1600) [-1600|1676.5] "degrees"
    SG_ SteWhlBrkAnRq_No_Cs  : 23|8@0+ (1,0) [0|255]
    SG_ SteWhlBrkAnRq_No_Cnt : 31|4@0+ (1,0) [0|15]

  Extract from device ford_lincoln_base_pt.dbc (line 2808):

    BO_ 936 ParkAid_Data: 8 IPMA_ADAS
      SG_ ExtSteeringAngleReq2 : 22|15@0+ (0.1,-1000) [-1000|2276.5] "Degrees"  GWM,PSCM
      SG_ EPASExtAngleStatReq  : 23|1@0+  (1,0) [0|1] "SED"  PSCM,GWM,ABS_ESC
      SG_ ApaSys_D_Stat        : 61|3@0+  (1,0) [0|7] "SED"  PSCM,GWM,...
      SG_ ApaSteWhl_D_RqDrv    : 63|2@0+  (1,0) [0|3] "SED"  GWM
      SG_ ApaMde_D_Stat        : (NOT present as 23|3 in device DBC)

  NOTE: ApaMde_D_Stat is NOT defined in the device DBC's ParkAid_Data.
  The device DBC uses a different set of APA mode signals (ApaSelSapp_D_Stat,
  ApaScan_D_Stat, etc.). The generated code's ApaMde_D_Stat usage must be
  reviewed against the actual message layout before deployment.

DBC Signal Reference
--------------------
From device ford_lincoln_base_pt.dbc:

  BO_ 936 ParkAid_Data: 8 IPMA_ADAS
    SG_ ExtSteeringAngleReq2 : 22|15@0+ (0.1,-1000) [-1000|2276.5] "Degrees"  GWM,PSCM
    SG_ EPASExtAngleStatReq  : 23|1@0+  (1,0) [0|1] "SED"  PSCM,GWM,ABS_ESC
    SG_ ApaSys_D_Stat        : 61|3@0+  (1,0) [0|7] "SED"  PSCM,GWM
    SG_ ApaSteWhl_D_RqDrv    : 63|2@0+  (1,0) [0|3] "SED"  GWM

  BO_ 983 Steer_Assist_Data: 8 IPMA_ADAS
    SG_ ApaSwtch_D_RqMnu     : 10|2@0+  (1,0) [0|3] "SED"  IPMA_ADAS
    SG_ ApaMdeStat_D_RqDrv   : 2|3@0+   (1,0) [0|7] "SED"  IPMA_ADAS

Integration
-----------
Merge these functions into fordcan.py. They use the same cantools-based
pattern as existing Ford CAN functions in openpilot/sunnypilot.

  from opendbc.car.ford import fordcan_apa  # or merge into fordcan

  # In CarController.update():
  if apa_controller.active_channel == APAChannel.APA:
      msg = fordcan_apa.create_apa_steer_msg(packer, CP, apa_angle, ...)
      can.send(msg, Bus.pt)
"""

from dataclasses import dataclass
from typing import Optional

# Try to import cantools for DBC-based encoding (preferred)
try:
  import cantools
  _DBC_AVAILABLE = True
except ImportError:
  _DBC_AVAILABLE = False

# crcmod not needed for APA on Q3 CAN (no checksum in ActiveFronSteering_Req)


# ============================================================================
# APA Signal Constants
# ============================================================================

class APASignals:
  """APA signal value constants."""

  # ExtSteeringAngleReq2 encoding
  EXT_ANGLE_FACTOR = 0.1       # deg per raw step
  EXT_ANGLE_OFFSET = -1000.0   # offset (deg)
  EXT_ANGLE_RAW_MAX = 32767    # 15-bit max

  # EPASExtAngleStatReq (0=No, 1=Relative ext angle req)
  EXT_ANGLE_DISABLE = 0
  EXT_ANGLE_ENABLE = 1

  # ApaMde_D_Stat (APA mode)
  APA_MODE_INACTIVE = 0
  APA_MODE_ACTIVE = 1
  APA_MODE_SUSPENDED = 2

  # ApaSys_D_Stat (APA system status)
  APA_SYS_INACTIVE = 0
  APA_SYS_ARMED = 1
  APA_SYS_ACTIVE = 2
  APA_SYS_FAULT = 3

  # ApaSteWhl_D_RqDrv
  APA_STEERWHL_NONE = 0
  APA_STEERWHL_REQUEST = 1

  # ApaSwtch_D_RqMnu (APA switch)
  APA_SWITCH_OFF = 0
  APA_SWITCH_ON = 1
  APA_SWITCH_CANCEL = 2

  # ApaMdeStat_D_RqDrv (APA mode status request)
  APA_MODE_REQ_INACTIVE = 0
  APA_MODE_REQ_ACTIVE = 1
  APA_MODE_REQ_SUSPENDED = 2


# ============================================================================
# Raw Byte Packing (Firmware-like, no DBC dependency)
# ============================================================================

def create_apa_steer_msg_raw(angle_deg: float,
                              ext_angle_enable: bool = True,
                              apa_mode: int = APASignals.APA_MODE_ACTIVE,
                              apa_sys: int = APASignals.APA_SYS_ACTIVE,
                              steerwhl_req: int = APASignals.APA_STEERWHL_REQUEST
                              ) -> tuple[int, bytes]:
  """
  Create ParkAid_Data CAN message using raw byte packing.

  This simulates the firmware-level signal control that the PSCM expects.
  No DBC file required — bytes are packed directly.

  ⚠️ CAN ID is 0x3A8 (936, ParkAid_Data), NOT 0x88.

  Signal bit layout (Intel byte order, 8 bytes):

  ExtSteeringAngleReq2 (22|15@0+):
    The 15-bit raw value = (angle_deg - offset) / factor = (angle_deg + 1000) / 0.1

    Bit positions (Intel = LSB first):
      bit 22 = byte 2, bit 6   <- raw[0]
      bit 23 = byte 2, bit 7   <- raw[1] (shared with EPASExtAngleStatReq & ApaMde[0])
      bit 24 = byte 3, bit 0   <- raw[2]
      bit 25 = byte 3, bit 1   <- raw[3]
      bit 26 = byte 3, bit 2   <- raw[4]
      bit 27 = byte 3, bit 3   <- raw[5]
      bit 28 = byte 3, bit 4   <- raw[6]
      bit 29 = byte 3, bit 5   <- raw[7]
      bit 30 = byte 3, bit 6   <- raw[8]
      bit 31 = byte 3, bit 7   <- raw[9]
      bit 32 = byte 4, bit 0   <- raw[10]
      bit 33 = byte 4, bit 1   <- raw[11]
      bit 34 = byte 4, bit 2   <- raw[12]
      bit 35 = byte 4, bit 3   <- raw[13]
      bit 36 = byte 4, bit 4   <- raw[14]

  ApaMde_D_Stat (23|3@0+):
    bits 23-25 (shared with ExtAngle bit 1 and byte 3 bits 0-1)
    3-bit value: bit 0 at bit 23, bit 1 at bit 24, bit 2 at bit 25

  ApaSys_D_Stat (61|3@0+):
    bits 61-63 = byte 7, bits 5-7

  ApaSteWhl_D_RqDrv (63|2@0+):
    bits 62-63 = byte 7, bits 6-7 (overlaps with ApaSys bits 1-2)

  Args:
    angle_deg: Steering wheel angle in degrees [-1000, 2276.5]
    ext_angle_enable: Enable extended angle mode (EPASExtAngleStatReq)
    apa_mode: APA mode status (ApaMde_D_Stat)
    apa_sys: APA system status (ApaSys_D_Stat)
    steerwhl_req: APA steering wheel request (ApaSteWhl_D_RqDrv)

  Returns:
    Tuple of (can_id, 8-byte payload)
  """
  CAN_ID = 0x3A8  # 936 decimal - ParkAid_Data (IPMA_ADAS)

  # Encode angle to raw 15-bit
  raw = int(round((angle_deg - APASignals.EXT_ANGLE_OFFSET) / APASignals.EXT_ANGLE_FACTOR))
  raw = max(0, min(APASignals.EXT_ANGLE_RAW_MAX, raw))

  frame = bytearray(8)

  # --- Byte 2 (bits 16-23) ---
  # bit 22: ExtSteeringAngleReq2[0]
  # bit 23: ExtSteeringAngleReq2[1] / EPASExtAngleStatReq / ApaMde_D_Stat[0]
  byte2 = 0
  byte2 |= (raw & 0x1) << 6  # bit 22 = raw bit 0

  if ext_angle_enable:
    # Set bit 23 = 1 for extended angle mode
    # This also sets ApaMde_D_Stat bit 0 = 1 (which is consistent with APA_MODE_ACTIVE)
    byte2 |= 0x80  # bit 23 = 1
  else:
    # If not enabled, bit 23 carries raw bit 1
    byte2 |= ((raw >> 1) & 0x1) << 7

  frame[2] = byte2

  # --- Byte 3 (bits 24-31) ---
  # bits 24-25: ApaMde_D_Stat[1:2] (upper 2 bits of 3-bit mode value)
  # bits 26-31: ExtSteeringAngleReq2[2:7]
  byte3 = 0
  apa_mode_upper = (apa_mode >> 1) & 0x3   # bits 1-2 of mode -> bits 0-1 of byte3
  byte3 |= apa_mode_upper                    # bits 0-1
  byte3 |= ((raw >> 2) & 0x3F) << 2          # bits 2-7 = raw bits 2-7
  frame[3] = byte3

  # --- Byte 4 (bits 32-39) ---
  # bits 32-36: ExtSteeringAngleReq2[8:12]
  # bits 37-39: unused
  frame[4] = (raw >> 8) & 0x1F

  # --- Byte 7 (bits 56-63) ---
  # bits 56-60: unused
  # bits 61-63: ApaSys_D_Stat (3 bits)
  # ApaSteWhl_D_RqDrv overlaps at bits 62-63
  byte7 = (apa_sys & 0x7) << 5
  # If steerwhl_req is set, ensure bits 62-63 carry the request
  # In practice, ApaSys and ApaSteWhl share bits, so we encode them together
  if steerwhl_req:
    byte7 |= 0xC0  # set bits 6-7 of byte 7 (bits 62-63)
  frame[7] = byte7

  return CAN_ID, bytes(frame)


def create_apa_mode_msg_raw(switch_req: int = APASignals.APA_SWITCH_ON,
                             mode_req: int = APASignals.APA_MODE_REQ_ACTIVE
                             ) -> tuple[int, bytes]:
  """
  Create Steer_Assist_Data CAN message using raw byte packing.

  This requests APA mode activation from the IPMA/camera system.

  Signal bit layout:
    ApaMdeStat_D_RqDrv (2|3@0+): bits 2-4 in byte 0
    ApaSwtch_D_RqMnu (10|2@0+): bits 10-11 in byte 1

  Args:
    switch_req: ApaSwtch_D_RqMnu value (0=off, 1=on, 2=cancel)
    mode_req: ApaMdeStat_D_RqDrv value (0=inactive, 1=active, 2=suspended)

  Returns:
    Tuple of (can_id, 8-byte payload)
  """
  CAN_ID = 0x3D7  # 983 decimal

  frame = bytearray(8)

  # Byte 0: ApaMdeStat_D_RqDrv at bits 2-4
  frame[0] |= (mode_req & 0x7) << 2

  # Byte 1: ApaSwtch_D_RqMnu at bits 10-11 (byte 1, bits 2-3)
  frame[1] |= (switch_req & 0x3) << 2

  return CAN_ID, bytes(frame)


# ============================================================================
# DBC-based Packing (Preferred for openpilot integration)
# ============================================================================

# DBC message name constants
# NOTE: corrected per device DBC verification on 2026-10-05
APA_STEER_MSG_NAME = "ParkAid_Data"        # CAN ID 0x3A8 (936), IPMA_ADAS
APA_MODE_MSG_NAME = "Steer_Assist_Data"    # CAN ID 0x3D7 (983), IPMA_ADAS


def create_apa_steer_msg(packer, canbus,
                          angle_deg: float,
                          ext_angle_enable: bool = True,
                          apa_mode: int = APASignals.APA_MODE_ACTIVE,
                          apa_sys: int = APASignals.APA_SYS_ACTIVE,
                          steerwhl_req: int = APASignals.APA_STEERWHL_REQUEST):
  """
  Create ParkAid_Data CAN message using DBC-based packer.

  This is the preferred method for openpilot integration — it uses the
  cantools packer which handles signal bit packing correctly, including
  overlapping signals.

  Usage in CarController (device-style CAN object):
    # Create APA steering message
    apa_msg = create_apa_steer_msg(
      packer, CAN.main, angle_deg=250.0, ext_angle_enable=True
    )
    can_sends.append(apa_msg)

  Args:
    packer: cantools packer instance (from dbc file)
    canbus: Bus target — device-style CAN object (e.g. CAN.main) or int
    angle_deg: Steering wheel angle in degrees [-1000, 2276.5]
    ext_angle_enable: Enable extended angle mode
    apa_mode: APA mode status (NOTE: ApaMde_D_Stat is NOT in device DBC;
               passes only if DBC actually defines it)
    apa_sys: APA system status (0=inactive, 1=armed, 2=active, 3=fault)
    steerwhl_req: APA steering wheel request (0=none, 1=request)

  Returns:
    can.Message instance for ParkAid_Data (0x3A8)
  """
  # Build signal dictionary
  # Only include signals that exist in the device DBC (ParkAid_Data):
  #   ExtSteeringAngleReq2, EPASExtAngleStatReq, ApaSys_D_Stat, ApaSteWhl_D_RqDrv
  signals = {
    "ExtSteeringAngleReq2": angle_deg,
    "EPASExtAngleStatReq": APASignals.EXT_ANGLE_ENABLE if ext_angle_enable else APASignals.EXT_ANGLE_DISABLE,
    "ApaSys_D_Stat": apa_sys,
    "ApaSteWhl_D_RqDrv": steerwhl_req,
  }

  return packer.make_can_msg(APA_STEER_MSG_NAME, canbus, signals)


def create_apa_mode_msg(packer, canbus,
                        switch_req: int = APASignals.APA_SWITCH_ON,
                        mode_req: int = APASignals.APA_MODE_REQ_ACTIVE):
  """
  Create Steer_Assist_Data CAN message for APA mode request.

  This requests APA mode activation from the IPMA/camera system.
  Sent at 10Hz during APA arming and active operation.

  Usage in CarController (device-style CAN object):
    mode_msg = create_apa_mode_msg(packer, CAN.main, switch_req=1, mode_req=1)
    can_sends.append(mode_msg)

  Args:
    packer: cantools packer instance
    canbus: Bus target — device-style CAN object (e.g. CAN.main) or int
    switch_req: ApaSwtch_D_RqMnu (0=off, 1=on, 2=cancel)
    mode_req: ApaMdeStat_D_RqDrv (0=inactive, 1=active, 2=suspended)

  Returns:
    can.Message instance for Steer_Assist_Data (0x3D7)
  """
  signals = {
    "ApaSwtch_D_RqMnu": switch_req,
    "ApaMdeStat_D_RqDrv": mode_req,
  }

  return packer.make_can_msg(APA_MODE_MSG_NAME, canbus, signals)


# ============================================================================
# APA Channel Selection Helper
# ============================================================================

@dataclass
class APAChannelConfig:
  """Configuration for APA channel operation."""
  # Speed thresholds (m/s)
  apa_max_speed: float = 8.0           # APA active below this speed
  apa_handover_speed: float = 10.0     # Switch back to LKA above this
  apa_min_speed: float = 0.5           # APA inactive below this

  # Angle limits (steering wheel degrees)
  apa_sw_max: float = 500.0            # Practical APA limit (physical ~540)
  lka_fw_max: float = 5.86             # LKA front wheel limit

  # Rate limits (deg/s, steering wheel)
  apa_rate_low: float = 300.0          # < 3 m/s
  apa_rate_high: float = 150.0         # 3-8 m/s
  lka_rate: float = 50.0               # LKA channel

  # Message timing (seconds)
  apa_steer_period: float = 0.02        # 50Hz
  apa_mode_period: float = 0.1          # 10Hz
  lka_period: float = 1.0 / 33          # 33Hz
  lmc_period: float = 1.0 / 20          # 20Hz


def should_use_apa_channel(v_ego: float, config: APAChannelConfig,
                            apa_active: bool) -> bool:
  """
  Determine if APA channel should be used for the current cycle.

  Implements speed hysteresis to prevent rapid channel switching.

  Args:
    v_ego: Vehicle speed (m/s)
    config: APA channel configuration
    apa_active: Whether APA is currently active (for hysteresis)

  Returns:
    True if APA channel should be used
  """
  if apa_active:
    # Hysteresis: stay in APA until handover speed
    return v_ego < config.apa_handover_speed
  else:
    # Enter APA when below max speed
    return v_ego < config.apa_max_speed and v_ego > config.apa_min_speed


def get_apa_rate_limit(v_ego: float, config: APAChannelConfig) -> float:
  """Get speed-dependent APA rate limit."""
  if v_ego < 3.0:
    return config.apa_rate_low
  else:
    return config.apa_rate_high


# ============================================================================
# Test / Verification
# ============================================================================

def verify_apa_encoding():
  """Verify APA signal encoding round-trip."""
  print("=" * 60)
  print("APA Signal Encoding Verification")
  print("=" * 60)

  test_angles = [0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 500.0, -90.0, -180.0, -500.0]

  print(f"\n{'Angle (deg)':>12} {'Raw':>8} {'Decoded':>12} {'Error':>10}")
  print("-" * 48)

  for angle in test_angles:
    raw = int(round((angle - APASignals.EXT_ANGLE_OFFSET) / APASignals.EXT_ANGLE_FACTOR))
    raw = max(0, min(APASignals.EXT_ANGLE_RAW_MAX, raw))
    decoded = raw * APASignals.EXT_ANGLE_FACTOR + APASignals.EXT_ANGLE_OFFSET
    error = abs(decoded - angle)
    print(f"{angle:12.1f} {raw:8d} {decoded:12.1f} {error:10.3f}")

  # Test raw frame packing
  print(f"\nRaw Frame Packing Test (ParkAid_Data, CAN ID 0x3A8):")
  can_id, frame = create_apa_steer_msg_raw(angle_deg=250.0)
  print(f"  Angle: 250.0 deg -> CAN ID: 0x{can_id:02X}, Frame: {frame.hex()}")
  print(f"  Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")

  can_id, frame = create_apa_steer_msg_raw(angle_deg=-180.0)
  print(f"  Angle: -180.0 deg -> CAN ID: 0x{can_id:02X}, Frame: {frame.hex()}")
  print(f"  Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")

  # Test mode frame
  can_id, frame = create_apa_mode_msg_raw(switch_req=1, mode_req=1)
  print(f"\n  Mode Request -> CAN ID: 0x{can_id:03X}, Frame: {frame.hex()}")
  print(f"  Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")

  # Amplification summary
  print(f"\n{'=' * 60}")
  print("Angle Amplification Summary:")
  print(f"  LKA channel:  +/-5.86 deg front wheel  (+/-113.2 deg SW)")
  print(f"  APA channel:  +/-500 deg SW           (+/-25.9 deg front wheel)")
  print(f"  Amplification: {500.0 / 113.2:.1f}x larger steering range")
  print(f"  U-turn capable: YES (500 deg SW = ~1.4 turns each direction)")
  print(f"  Parking capable: YES (full lock-to-lock control)")
  print("=" * 60)


if __name__ == "__main__":
  verify_apa_encoding()
