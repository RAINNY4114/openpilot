#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ford / Lincoln Radar Interface
==============================

PRIMARY RADAR
=============

OEM Delphi ESR / MRR
        |
        v
    RadarData.points
        |
        v
    radarTracks
        |
        v
    RadarD
        |
        v
radarState.leadOne / leadTwo
        |
        v
longitudinal planning


AUXILIARY RADAR
===============

MR76
 CAN1
   |
   +-- 0x60A Status
   |
   +-- 0x60B ObjectData
   |
   v
u_radar.dbc
   |
   v
CANParser
   |
   v
self.mr76_objects
   |
   v
MR76SafetyState
   |
   +--------------------+
   |                    |
   v                    v
AutoOvertake       AutoAvoidance
  veto                 veto


IMPORTANT SAFETY BOUNDARY
=========================

MR76 is AUXILIARY ONLY.

MR76 NEVER enters:

    - self.points
    - self.pts
    - RadarData.points
    - radarTracks
    - radarState
    - leadOne
    - leadTwo
    - longitudinalPlan
    - aTarget
    - shouldStop
    - actuator control
    - CAN control

OEM Delphi ESR/MRR processing remains the primary radar path.

MR76 is only an auxiliary safety-information source.
"""


import os
import time

import numpy as np

from typing import cast
from collections import defaultdict
from math import cos, sin
from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase


# ============================================================================
# [P3] radar-bus loss must not raise a global canError
#
# The private radar bus (CanBus.radar) is an OPTIONAL sensor bus. Its CAN
# health used to be reported as `canError`, which selfdrived maps to
# EventName.canError = IMMEDIATE_DISABLE + NO_ENTRY. That hard-disabled
# openpilot AND blocked re-engagement for as long as the bus stayed degraded,
# which is exactly the reported "system exits / cannot re-engage / recovers
# after a while" behaviour.
#
# radard already falls back to the vision lead when no tracks are available,
# so radar loss is a degradation, not a vehicle-wide CAN failure.
#
# `libparams_c.so` on this device is stale (new keys raise UnknownKeyName), so
# the knob is read straight from the param files, like ford_curve_controller.py
# does. Hot reloaded every _RADAR_LOSS_REFRESH_SEC, never raises.
# ============================================================================

RADAR_LOSS_PARAM = "dp_radar_loss_no_disable"
RADAR_LOSS_PARAM_DIRS = ("/dev/shm/params", "/data/params/d")
_RADAR_LOSS_REFRESH_SEC = 2.0

_radar_loss_cache = {"t": -1e9, "v": False}


def _radar_loss_no_disable():
  """True when a radar-bus CAN dropout must NOT disable openpilot."""
  now = time.monotonic()
  if now - _radar_loss_cache["t"] < _RADAR_LOSS_REFRESH_SEC:
    return _radar_loss_cache["v"]

  val = False
  for d in RADAR_LOSS_PARAM_DIRS:
    try:
      with open(os.path.join(d, RADAR_LOSS_PARAM), "r") as f:
        raw = f.read().strip()
    except Exception:
      continue
    if raw:
      val = raw == "1" or raw.lower() == "true"
      break

  _radar_loss_cache["t"] = now
  _radar_loss_cache["v"] = val
  return val


# ============================================================================
# Ford / Lincoln OEM Delphi ESR radar
# ============================================================================

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))


# ============================================================================
# Ford / Lincoln OEM Delphi MRR radar
# ============================================================================

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x174

# ============================================================================
# [FIX] Standstill exemption for the OEM MRR scan-index check
# ============================================================================
#
# The factory Delphi MRR stops advancing CAN_SCAN_INDEX once the car is at or
# near standstill: it keeps transmitting at a rock-steady 33.3 Hz but the
# payload freezes (measured 2026-09-29 -- the freeze begins the instant vEgo
# drops below ~0.5 m/s and clears again once the car moves).
#
# The stock check counted that as a radar fault, setting
# radarUnavailableTemporary -> NO_ENTRY, which is what made openpilot exit and
# refuse to re-engage in stop-and-go traffic.
#
# Below this speed the sequence check is simply not applicable.
MRR_SCAN_MIN_VEGO = 1.5   # m/s (~5.4 km/h)
DELPHI_MRR_RADAR_MSG_COUNT = 64

DELPHI_MRR_RADAR_RANGE_COVERAGE = {
  0: 42,
  1: 164,
  2: 45,
  3: 175,
}

DELPHI_MRR_MIN_LONG_RANGE_DIST = 30

DELPHI_MRR_CLUSTER_THRESHOLD = 5


# ============================================================================
# MR76 AUXILIARY RADAR
# ============================================================================

"""
MR76 physical connection:

    C3X CAN1

Known IDs:

    0x201 = RadarState / configuration
    0x60A = Status
    0x60B = ObjectData

Only 0x60A and 0x60B are decoded.

0x201 is intentionally ignored.

DBC:

    /data/openpilot/opendbc_repo/opendbc/dbc/u_radar.dbc

IMPORTANT:

    MR76 never enters RadarData.
    MR76 never enters radarTracks.
    MR76 never becomes a longitudinal lead.
"""

MR76_BUS = 1                # MR76在panda bus1 (C3X CAN1)

# CANParser receives the DBC name, not the ".dbc" filename.
MR76_DBC = "u_radar"

MR76_RADAR_STATE_ID = 0x201
MR76_STATUS_ID = 0x60A
MR76_OBJECT_ID = 0x60B

MR76_STATUS_NAME = "Status"
MR76_OBJECT_NAME = "ObjectData"


# ============================================================================
# MR76 parser configuration
# ============================================================================

# CANParser liveness declaration.
#
# CANParser derives its stale-timeout as  10 / freq  seconds
# (opendbc/can/parser.py: state.timeout_threshold = (1e9 / freq) * 10) and
# marks the message invalid when the gap to the last frame exceeds it.
#
# The MR76 does NOT transmit at a fixed rate: measured on this car, Status
# (0x60A) runs at 0.32-16.5 Hz and ObjectData (0x60B) at 86-287 Hz, because the
# radar reallocates bandwidth with the number of tracked targets. Declaring 20
# (=> 0.50 s) made `can_valid` permanently False for Status.
#
# u_radar.dbc has no GenMsgCycleTime attributes, so there is no authoritative
# rate to read. NaN sets `ignore_alive=True` (see CANParser._add_message), which
# disables the liveness check rather than guessing. MR76 freshness is handled
# explicitly by MR76_OBJECT_TIMEOUT_SEC / MR76_STATUS_TIMEOUT_SEC below, and
# those are the values the consumer uses.
#
# NOTE: `mr76_rcp.can_valid` is not read anywhere today, so this changes no
# behaviour -- it removes a trap for future code.
MR76_STATUS_FREQ = float("nan")
MR76_OBJECT_FREQ = float("nan")


# ============================================================================
# MR76 physical sanity limits
# ============================================================================

MR76_MIN_DISTANCE = 0.0
MR76_MAX_DISTANCE = 150.0

MR76_MAX_YREL = 25.0

MR76_MAX_VREL = 80.0
MR76_MAX_VLAT = 50.0

MR76_VALID_CLASSES = frozenset({
  0,  # point
  1,  # vehicle
})


# ============================================================================
# MR76 freshness
# ============================================================================

MR76_OBJECT_TIMEOUT_SEC = 0.50
MR76_STATUS_TIMEOUT_SEC = 1.00

MR76_MAX_STALE_UPDATES = 10


# ============================================================================
# MR76 safety parameters
# ============================================================================

"""
These values only classify MR76 auxiliary safety information.

They DO NOT directly control:

    steering
    braking
    throttle
    CAN
    longitudinalPlan
    radarState
"""

MR76_SAFETY_MIN_DISTANCE = 1.0

# Negative VRel means the object is approaching ego.
MR76_MIN_CLOSING_SPEED = -2.0

MR76_EMERGENCY_TTC_SEC = 2.0

MR76_OVERTAKE_TTC_SEC = 4.0

# Continuous-frame confirmation.
MR76_CONFIRM_COUNT = 3

# Emergency can promote slightly faster than ordinary overtake veto.
MR76_EMERGENCY_CONFIRM_COUNT = 2

# Emergency clear hysteresis.
MR76_CLEAR_HOLD_SEC = 0.40


# ============================================================================
# MR76 adjacent-lane occupancy (auxiliary veto input)
# ============================================================================

"""
MR76 is a single front radar.  It reports every target as

    DistLong -> dRel   (metres, longitudinal, ego-relative)
    DistLat  -> yRel   (metres, lateral, NEGATIVE = left, POSITIVE = right)

The band below classifies a confirmed target as occupying the adjacent lane.
It is deliberately conservative and tunable, because it can only ever BLOCK a
lane change -- it never creates a RadarData point, a track or a lead.


CRITERIA v2 -- MEASURED, NOT GUESSED
====================================

The original rule was:

    keep if  (Class == 1)  OR  (|vRel| > MR76_LANE_MIN_CLOSING)

That rule is INVERTED in practice.  A stationary roadside object (guardrail,
lamp post, tree, sign, bridge pier) has

    vRel = 0 - vEgo  ->  |vRel| ~ vEgo ~ 27 m/s  >>  2.0

so it satisfied the second half of the OR and was always kept.  A genuine
parallel neighbour travelling at our own speed has vRel ~ 0, so it survived
only via the first half.  The rule therefore kept almost everything that
should have been rejected and rejected the one case it was written for.

Measured over 6 routes / 266,049 CAN batches / 1,568,057 MR76 targets
(107,492 driving batches with live MR76):

    rule                                        adjacent-lane occupancy
    ------------------------------------------------------------------
    original (OR)                                      77.34 %
    factory BSM (ground truth, side/rear)               10.54 %
    => original was 7.34x too eager

    85 % of the original hits were Class == 0 point targets,
    84 % were DynProp == stationary, 86 % had RCS < 0 dBsm,
    Top signatures were |yRel| 2-5 m, dRel 10-30 m, |vRel| 5-10 m/s --
    i.e. roadside clutter, never a vehicle.

CRITERIA v2 requires ALL of:

    - survived MR76_LANE_CONFIRM_COUNT consecutive frames
    - MR76_LANE_DIST_MIN <= dRel <= branch distance limit
    - MR76_LANE_YREL_MIN <= |yRel| <= MR76_LANE_YREL_MAX
    - Class == 1                       (a classified VEHICLE, not a point)
    - rcs >= MR76_LANE_MIN_RCS         (a real reflector)
    - |vLat| <= MR76_LANE_MAX_VLAT     (laterally stable, not sweeping past)

  then EITHER branch:

    A. moving vehicle (DynProp == 0 "moving")
         |vRel| <= MR76_LANE_MOVING_MAX_VREL   (parallel, not oncoming)
         dRel   <= MR76_LANE_MOVING_DIST_MAX

    B. stationary / stopped vehicle (DynProp in {1 "stationary", 6 "stopped"})
         dRel   <= MR76_LANE_STOPPED_DIST_MAX

Branch B exists because a broken-down car in the target lane is exactly the
case that must veto a lane change, and branch A alone would miss it.

Measured occupancy of the v2 rule on the same data:

    A only (moving)                                  7.31 %
    A | B                                            7.97 %
    B only                                           0.77 %  (552 extra
                                                              "occupied" batches
                                                              out of 107,492)

Branch B is cheap (0.66 pp) and adds 276 batches where the factory BSM agrees,
so it is kept.  Note that a stricter RCS gate for branch B (rcs >= 0) drops
every single BSM agreement -- real stopped vehicles in the adjacent lane are
seen at a grazing angle and never reach 0 dBsm -- so it must NOT be used.

Relationship to the factory BSM (do not confuse the two)
-------------------------------------------------------

    BSM   covers the side / REAR blind spot.
    MR76  covers the side / FRONT, roughly 5-60 m ahead.

v2 catches only ~13 % of BSM events, and 6,643 batches are "MR76 says occupied,
BSM says clear".  MR76 therefore SUPPLEMENTS BSM and can never REPLACE it.
The correct reading of this function is

    "is the adjacent lane occupied AHEAD of me?"

NOT "is my blind spot clear?".
"""

# Ignore targets essentially alongside the car; they are not overtake-relevant.
MR76_LANE_DIST_MIN = 5.0

# Confirmed vehicles only.  MR76 0x60B arrives in ~20 Hz bursts, so 10 frames
# is roughly 0.5 s of continuous track -- the same gate the replay evaluation
# used (age >= 0.5 s) and the point at which stationary hits stop changing.
MR76_LANE_CONFIRM_COUNT = 10

# Branch A: a moving vehicle ahead in the adjacent lane.  40 m is the furthest
# a parallel-moving vehicle was still reliably separable from clutter.
MR76_LANE_MOVING_DIST_MAX = 40.0

# Branch B: a stationary / stopped vehicle.  Kept out to 60 m because a stopped
# vehicle is a hard obstacle and closing speed is irrelevant.
MR76_LANE_STOPPED_DIST_MAX = 60.0

# Kept for backwards compatibility with any external reader of this module.
# The v2 rule uses the per-branch limits above instead.
MR76_LANE_DIST_MAX = MR76_LANE_STOPPED_DIST_MAX

# Inner edge of the adjacent lane.  Inside this the target is in our own lane.
MR76_LANE_YREL_MIN = 1.5

# Outer edge of the adjacent lane.
MR76_LANE_YREL_MAX = 5.5

# DynProp encoding (u_radar.dbc VAL_ 1547).
MR76_DYN_MOVING = 0
MR76_DYN_STATIONARY = 1
MR76_DYN_STOPPED = 6

MR76_LANE_DYN_STOPPED = frozenset({
  MR76_DYN_STATIONARY,
  MR76_DYN_STOPPED,
})

# Branch A: |vRel| of a vehicle travelling roughly at our own speed.
# Anything faster is oncoming (|vRel| ~ 2*vEgo) or a passing manoeuvre.
MR76_LANE_MOVING_MAX_VREL = 3.0

# Lateral relative motion.  A vehicle tracking alongside is laterally stable;
# a target sweeping across the lane (a real crossing hazard for the OTHER
# lane, or clutter being re-associated) is not.
MR76_LANE_MAX_VLAT = 1.0

# Minimum RCS, in dBsm.  Class-1 targets in this dataset have p50 = -5.0, so
# this keeps the real distribution and rejects the weak-clutter tail.
MR76_LANE_MIN_RCS = -5.0

# Retained for backwards compatibility with any external reader of this module.
# The v2 rule no longer uses a bare |vRel| threshold as a keep condition.
MR76_LANE_MIN_CLOSING = 2.0


# ============================================================================
# Data structures
# ============================================================================

@dataclass
class Cluster:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


@dataclass
class MR76Object:
  """
  Independent MR76 auxiliary target.

  This is NOT a RadarPoint.
  This is NOT a radar Track.
  """

  obj_id: int = 0

  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  vLat: float = 0.0

  dyn_prop: int = 0
  obj_class: int = 0
  rcs: float = 0.0

  last_seen: float = 0.0


@dataclass
class MR76SafetyState:
  """
  MR76 auxiliary safety information.

  No actuator command is contained here.

  Consumers:

      AutoOvertake
      AutoAvoidance

  This state MUST NOT be converted into:

      RadarPoint
      radarTrack
      leadOne
      leadTwo
      radarState
      longitudinalPlan
  """

  fresh: bool = False

  object_count: int = 0

  overtake_veto: bool = False

  emergency_veto: bool = False

  emergency_left: bool = False
  emergency_right: bool = False

  closest_distance: float = float("inf")

  closest_object_id: int = -1

  ttc: float = float("inf")

  confirmed_count: int = 0

  emergency_confirmed_count: int = 0

  timestamp: float = 0.0


# ============================================================================
# Point clustering helper
# ============================================================================

def cluster_points(
  pts_l: list[list[float]],
  pts2_l: list[list[float]],
  max_dist: float,
) -> list[int]:
  """
  Clusters a collection of points based on another collection.

  Returns:

    cluster index
    -1 when no point is close enough.
  """

  if not len(pts2_l):
    return []

  if not len(pts_l):
    return [-1] * len(pts2_l)

  max_dist_sq = max_dist ** 2

  pts = np.array(
    pts_l,
    dtype=float,
  )

  pts2 = np.array(
    pts2_l,
    dtype=float,
  )

  pts_norm_sq = np.sum(
    pts ** 2,
    axis=1,
  )

  pts2_norm_sq = np.sum(
    pts2 ** 2,
    axis=1,
  )

  dist_sq = (
    pts2_norm_sq[:, np.newaxis]
    + pts_norm_sq[np.newaxis, :]
    - 2 * np.dot(
      pts2,
      pts.T,
    )
  )

  dist_sq = np.maximum(
    dist_sq,
    0.0,
  )

  closest_clusters = np.argmin(
    dist_sq,
    axis=1,
  )

  closest_dist_sq = dist_sq[
    np.arange(len(pts2)),
    closest_clusters,
  ]

  cluster_idxs = np.where(
    closest_dist_sq < max_dist_sq,
    closest_clusters,
    -1,
  )

  return cast(
    list[int],
    cluster_idxs.tolist(),
  )


# ============================================================================
# OEM Delphi ESR parser
# ============================================================================

def _create_delphi_esr_radar_can_parser(
  CP,
) -> CANParser:

  msg_n = len(
    DELPHI_ESR_RADAR_MSGS
  )

  messages = list(
    zip(
      DELPHI_ESR_RADAR_MSGS,
      [20] * msg_n,
      strict=True,
    )
  )

  return CANParser(
    RADAR.DELPHI_ESR,
    messages,
    CanBus(CP).radar,
  )


# ============================================================================
# OEM Delphi MRR parser
# ============================================================================

def _create_delphi_mrr_radar_can_parser(
  CP,
) -> CANParser:

  messages = [
    (
      "MRR_Header_InformationDetections",
      33,
    ),
    (
      "MRR_Header_SensorCoverage",
      33,
    ),
  ]

  for i in range(
    1,
    DELPHI_MRR_RADAR_MSG_COUNT + 1,
  ):
    messages.append(
      (
        f"MRR_Detection_{i:03d}",
        33,
      )
    )

  return CANParser(
    RADAR.DELPHI_MRR,
    messages,
    CanBus(CP).radar,
  )


# ============================================================================
# MR76 parser
# ============================================================================

def _create_mr76_can_parser() -> CANParser:
  """
  Create an independent MR76 auxiliary parser.

  Physical DBC file:

    /data/openpilot/opendbc_repo/opendbc/dbc/u_radar.dbc

  DBC name supplied to CANParser:

    u_radar

  CAN bus:

    1

  Messages:

    0x60A -> Status
    0x60B -> ObjectData

  0x201 is intentionally not parsed.
  """

  messages = [
    (
      MR76_STATUS_NAME,
      MR76_STATUS_FREQ,
    ),
    (
      MR76_OBJECT_NAME,
      MR76_OBJECT_FREQ,
    ),
  ]

  return CANParser(
    MR76_DBC,
    messages,
    MR76_BUS,
  )


# ============================================================================
# Radar Interface
# ============================================================================

class RadarInterface(RadarInterfaceBase):

  def __init__(
    self,
    CP,
    CP_SP,
  ):

    super().__init__(
      CP,
      CP_SP,
    )

    # ========================================================================
    # PRIMARY OEM RADAR
    # ========================================================================

    self.points: list[list[float]] = []

    self.pts: dict[
      int,
      structs.RadarData.RadarPoint,
    ] = {}

    self.clusters: list[
      Cluster
    ] = []

    self.track_id = 0

    self.updated_messages = set()

    self.radar = DBC[
      CP.carFingerprint
    ].get(
      Bus.radar
    )

    self.scan_index_invalid_cnt = 0
    self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = 0

    # [FIX] vEgo, supplied by card.py, used by the standstill exemption below.
    # None (tests, replay, any caller that omits it) keeps stock behaviour.
    self.v_ego: float | None = None

    # ========================================================================
    # AUXILIARY MR76
    # ========================================================================

    self.mr76_objects: dict[
      int,
      MR76Object,
    ] = {}

    self.mr76_object_count = 0
    self.mr76_meas_count = 0

    self.mr76_updated = False
    self.mr76_stale = True
    self.mr76_stale_updates = 0

    self.mr76_rx_count = 0
    self.mr76_status_count = 0

    self.mr76_last_rx_time = 0.0
    self.mr76_last_status_time = 0.0

    # Diagnostic counter only.
    self.mr76_parse_error_count = 0

    # ========================================================================
    # MR76 safety state
    # ========================================================================

    self.mr76_safety = MR76SafetyState()

    self.mr76_confirm_counts: dict[
      int,
      int,
    ] = {}

    self.mr76_emergency_confirm_counts: dict[
      int,
      int,
    ] = {}

    self.mr76_emergency_since = 0.0

    # ========================================================================
    # Independent MR76 CANParser
    # ========================================================================

    self.mr76_rcp = (
      _create_mr76_can_parser()
    )

    # ========================================================================
    # OEM radar parser
    # ========================================================================

    if CP.radarUnavailable:

      self.rcp = None
      self.trigger_msg = None

    elif self.radar == RADAR.DELPHI_ESR:

      self.rcp = (
        _create_delphi_esr_radar_can_parser(
          CP
        )
      )

      self.trigger_msg = (
        DELPHI_ESR_RADAR_MSGS[-1]
      )

      self.valid_cnt = {
        key: 0
        for key in DELPHI_ESR_RADAR_MSGS
      }

    elif self.radar == RADAR.DELPHI_MRR:

      self.rcp = (
        _create_delphi_mrr_radar_can_parser(
          CP
        )
      )

      self.trigger_msg = (
        DELPHI_MRR_RADAR_HEADER_ADDR
      )

    elif self.radar == RADAR.MR76:

      # Compatibility only.
      #
      # MR76 is never a primary RadarData source.

      self.rcp = None
      self.trigger_msg = None

    else:

      raise ValueError(
        f"Unsupported radar: {self.radar}"
      )


  # ==========================================================================
  # Main update
  # ==========================================================================

  def update(
    self,
    can_strings,
    v_ego: float | None = None,
  ):
    """
    Main radar update.

    OEM:

        Delphi ESR/MRR
            |
            v
        RadarData.points

    AUX:

        MR76 CAN1
            |
            v
        mr76_objects
            |
            v
        MR76SafetyState

    MR76 never enters RadarData.
    """

    # [FIX] Stash vEgo for the standstill exemption in _update_delphi_mrr().
    self.v_ego = v_ego

    # ========================================================================
    # MR76 auxiliary path
    # ========================================================================

    self._update_mr76_aux(
      can_strings
    )

    # ========================================================================
    # No OEM parser
    # ========================================================================

    if self.rcp is None:

      if self.radar != RADAR.MR76:
        return super().update(None)

      # Explicit MR76-primary compatibility mode.
      #
      # No RadarData is generated from MR76.
      return None

    # ========================================================================
    # OEM radar parser
    # ========================================================================

    vls = self.rcp.update(
      can_strings
    )

    self.updated_messages.update(
      vls
    )

    if (
      self.trigger_msg
      not in self.updated_messages
    ):
      return None

    self.updated_messages.clear()

    ret = structs.RadarData()

    # ========================================================================
    # OEM CAN health
    # ========================================================================

    if not self.rcp.can_valid:
      # [P3] radar-bus loss must not raise a global canError
      #
      # Report radar degradation as radar degradation, so radard falls back to
      # the vision lead instead of openpilot being hard-disabled with NO_ENTRY.
      # When dp_radar_loss_no_disable is unset this is byte-for-byte the
      # original behaviour (ret.errors.canError = True).
      if _radar_loss_no_disable():
        ret.errors.radarUnavailableTemporary = True
        self.pts.clear()
        self.points.clear()
        self.clusters.clear()
      else:
        ret.errors.canError = True

    # ========================================================================
    # OEM radar decoding
    # ========================================================================

    if self.radar == RADAR.DELPHI_ESR:

      self._update_delphi_esr()

    elif self.radar == RADAR.DELPHI_MRR:

      updated = (
        self._update_delphi_mrr(
          ret
        )
      )

      if not updated:
        return None

    # ========================================================================
    # HARD SAFETY BOUNDARY
    # ========================================================================
    #
    # Only OEM radar points are published.
    #
    # MR76 is completely absent.
    # ========================================================================

    ret.points = list(
      self.pts.values()
    )

    return ret


  # ==========================================================================
  # MR76 auxiliary decoder
  # ==========================================================================

  def _update_mr76_aux(
    self,
    can_strings,
  ):
    """
    Decode MR76 CAN1 using u_radar.dbc.

    IMPORTANT:

      vl:
        latest decoded frame

      vl_all:
        all matching frames in this parser update

    0x60B ObjectData may occur multiple times in one CAN batch.

    Therefore:

        vl_all["ObjectData"]

    is used instead of:

        vl["ObjectData"]

    No measurement-cycle completion is required.

    NoOfObjects is diagnostic metadata only.
    MeasCount is diagnostic metadata only.
    """

    self.mr76_updated = False

    # ========================================================================
    # Feed the independent parser
    # ========================================================================

    try:

      updated_addrs = (
        self.mr76_rcp.update(
          can_strings
        )
      )

    except Exception:

      self.mr76_parse_error_count += 1

      # MR76 is auxiliary.
      #
      # Never allow auxiliary parser failure to stop OEM radar.
      return

    now = time.monotonic()

    got_status = (
      MR76_STATUS_ID
      in updated_addrs
    )

    got_object = (
      MR76_OBJECT_ID
      in updated_addrs
    )

    # ========================================================================
    # 0x60A Status
    # ========================================================================

    if got_status:

      try:

        status = self.mr76_rcp.vl[
          MR76_STATUS_NAME
        ]

        self.mr76_object_count = int(
          status.get(
            "NoOfObjects",
            0,
          )
        )

        self.mr76_meas_count = int(
          status.get(
            "MeasCount",
            0,
          )
        )

        self.mr76_status_count += 1
        self.mr76_last_status_time = now

      except (
        KeyError,
        TypeError,
        ValueError,
      ):

        self.mr76_parse_error_count += 1

    # ========================================================================
    # 0x60B ObjectData
    # ========================================================================

    if got_object:

      self.mr76_last_rx_time = now
      self.mr76_rx_count += 1

      try:

        all_objects = (
          self.mr76_rcp.vl_all[
            MR76_OBJECT_NAME
          ]
        )

        ids = all_objects.get(
          "ID",
          [],
        )

        d_rels = all_objects.get(
          "DistLong",
          [],
        )

        y_rels = all_objects.get(
          "DistLat",
          [],
        )

        v_rels = all_objects.get(
          "VRelLong",
          [],
        )

        v_lats = all_objects.get(
          "VRelLat",
          [],
        )

        dyn_props = all_objects.get(
          "DynProp",
          [],
        )

        classes = all_objects.get(
          "Class",
          [],
        )

        rcss = all_objects.get(
          "RCS",
          [],
        )

        # ==================================================================
        # All signal arrays correspond to the ObjectData frames received
        # during this parser update.
        # ==================================================================

        frame_count = min(
          len(ids),
          len(d_rels),
          len(y_rels),
          len(v_rels),
          len(v_lats),
          len(dyn_props),
          len(classes),
          len(rcss),
        )

        for i in range(
          frame_count
        ):

          try:

            obj_id = int(
              ids[i]
            )

            d_rel = float(
              d_rels[i]
            )

            y_rel = float(
              y_rels[i]
            )

            v_rel = float(
              v_rels[i]
            )

            v_lat = float(
              v_lats[i]
            )

            dyn_prop = int(
              dyn_props[i]
            )

            obj_class = int(
              classes[i]
            )

            rcs = float(
              rcss[i]
            )

          except (
            TypeError,
            ValueError,
            IndexError,
          ):

            continue

          # ================================================================
          # ID validation
          # ================================================================

          if obj_id <= 0:
            continue

          # ================================================================
          # Object class validation
          # ================================================================

          if (
            obj_class
            not in MR76_VALID_CLASSES
          ):
            continue

          # ================================================================
          # Numeric validation
          # ================================================================

          if not all(
            np.isfinite(x)
            for x in (
              d_rel,
              y_rel,
              v_rel,
              v_lat,
              rcs,
            )
          ):
            continue

          # ================================================================
          # Distance validation
          # ================================================================

          if (
            d_rel
            < MR76_MIN_DISTANCE
          ):
            continue

          if (
            d_rel
            > MR76_MAX_DISTANCE
          ):
            continue

          # ================================================================
          # Lateral validation
          # ================================================================

          if (
            abs(y_rel)
            > MR76_MAX_YREL
          ):
            continue

          # ================================================================
          # Relative velocity validation
          # ================================================================

          if (
            abs(v_rel)
            > MR76_MAX_VREL
          ):
            continue

          if (
            abs(v_lat)
            > MR76_MAX_VLAT
          ):
            continue

          # ================================================================
          # Existing object?
          # ================================================================

          old = self.mr76_objects.get(
            obj_id
          )

          # ================================================================
          # Update independent cache
          # ================================================================

          self.mr76_objects[
            obj_id
          ] = MR76Object(
            obj_id=obj_id,
            dRel=d_rel,
            yRel=y_rel,
            vRel=v_rel,
            vLat=v_lat,
            dyn_prop=dyn_prop,
            obj_class=obj_class,
            rcs=rcs,
            last_seen=now,
          )

          # ================================================================
          # Normal confirmation counter
          # ================================================================

          if old is None:

            self.mr76_confirm_counts[
              obj_id
            ] = 1

          else:

            self.mr76_confirm_counts[
              obj_id
            ] = (
              self.mr76_confirm_counts.get(
                obj_id,
                0,
              )
              + 1
            )

          # ================================================================
          # Emergency confirmation
          # ================================================================

          obj = self.mr76_objects[
            obj_id
          ]

          if self._mr76_emergency_candidate(
            obj
          ):

            self.mr76_emergency_confirm_counts[
              obj_id
            ] = (
              self.mr76_emergency_confirm_counts.get(
                obj_id,
                0,
              )
              + 1
            )

          else:

            self.mr76_emergency_confirm_counts[
              obj_id
            ] = 0

      except (
        KeyError,
        TypeError,
        ValueError,
      ):

        self.mr76_parse_error_count += 1

    # ========================================================================
    # Expire old objects
    # ========================================================================

    stale_ids = []

    for obj_id, obj in (
      self.mr76_objects.items()
    ):

      if (
        now
        - obj.last_seen
      ) > MR76_OBJECT_TIMEOUT_SEC:

        stale_ids.append(
          obj_id
        )

    for obj_id in stale_ids:

      self.mr76_objects.pop(
        obj_id,
        None,
      )

      self.mr76_confirm_counts.pop(
        obj_id,
        None,
      )

      self.mr76_emergency_confirm_counts.pop(
        obj_id,
        None,
      )

    # ========================================================================
    # Freshness
    # ========================================================================

    if (
      got_status
      or got_object
    ):

      self.mr76_updated = True
      self.mr76_stale_updates = 0

    else:

      self.mr76_stale_updates += 1

    # ========================================================================
    # Determine MR76 stale state
    # ========================================================================

    if self.mr76_objects:

      newest_rx = max(
        obj.last_seen
        for obj in self.mr76_objects.values()
      )

      self.mr76_stale = (
        now
        - newest_rx
      ) > MR76_OBJECT_TIMEOUT_SEC

    elif (
      self.mr76_last_rx_time > 0
      and
      (
        now
        - self.mr76_last_rx_time
      ) <= MR76_OBJECT_TIMEOUT_SEC
      and
      self.mr76_last_status_time > 0
      and
      (
        now
        - self.mr76_last_status_time
      ) <= MR76_STATUS_TIMEOUT_SEC
    ):

      # MR76 is alive AND its OBJECT stream (0x60B) is actually flowing,
      # but it currently reports no objects at all -> genuinely clear.
      #
      # [FIX] 原实现把"STATUS 流(0x60A)活着"单独当作 MR76 活着的证据：
      #
      #     elif last_status_time > 0 and now - last_status_time <= 1.0:
      #         mr76_stale = False
      #
      # 于是只要 0x60A 还在来、而 0x60B 停了（对象流死了），
      # 对象缓存会在 0.5 s 内逐个过期清空，随后本模块报告
      #     fresh = True, left = False, right = False
      # 即"目标车道空" —— 恰好是最危险的方向（否决静默失效）。
      #
      # 现在存活判定必须同时要求**对象流本身**新鲜。
      # 实测：0x60A 与 0x60B 在数据里总是一起出现，所以该改动
      # 在所有已观测工况下都是 no-op，只在上述病态场景下生效。
      self.mr76_stale = False

    else:

      self.mr76_stale = True

    # ========================================================================
    # Hard stale cleanup
    # ========================================================================

    if (
      self.mr76_stale_updates
      > MR76_MAX_STALE_UPDATES
    ):

      self.mr76_objects.clear()

      self.mr76_confirm_counts.clear()

      self.mr76_emergency_confirm_counts.clear()

      self.mr76_stale = True

    # ========================================================================
    # Safety state
    # ========================================================================

    self._update_mr76_safety_state(
      now
    )


  # ==========================================================================
  # MR76 emergency candidate
  # ==========================================================================

  def _mr76_emergency_candidate(
    self,
    obj: MR76Object,
  ) -> bool:
    """
    Classify an MR76 object as a potential emergency obstacle.

    This function NEVER commands the vehicle.
    """

    if (
      obj.dRel
      <= MR76_SAFETY_MIN_DISTANCE
    ):
      return True

    # Negative VRel means closing.
    if (
      obj.vRel
      >= MR76_MIN_CLOSING_SPEED
    ):
      return False

    closing_speed = -obj.vRel

    if closing_speed <= 0.1:
      return False

    ttc = (
      obj.dRel
      / closing_speed
    )

    return (
      ttc
      <= MR76_EMERGENCY_TTC_SEC
    )


  # ==========================================================================
  # MR76 TTC
  # ==========================================================================

  def _mr76_ttc(
    self,
    obj: MR76Object,
  ) -> float:
    """
    Calculate time-to-collision estimate from longitudinal relative motion.

    Positive finite result means closing.
    Infinity means no meaningful closing estimate.
    """

    if obj.dRel <= 0.0:
      return 0.0

    if obj.vRel >= -0.1:
      return float("inf")

    closing_speed = -obj.vRel

    if closing_speed <= 0.1:
      return float("inf")

    return (
      obj.dRel
      / closing_speed
    )


  # ==========================================================================
  # MR76 safety state
  # ==========================================================================

  def _update_mr76_safety_state(
    self,
    now: float,
  ):
    """
    Generate MR76 auxiliary safety state.

    No actuator operation is performed here.
    """

    state = MR76SafetyState(
      fresh=(
        not self.mr76_stale
      ),
      object_count=len(
        self.mr76_objects
      ),
      timestamp=now,
    )

    # ========================================================================
    # Stale sensor -> no safety object state
    # ========================================================================

    if self.mr76_stale:

      self.mr76_safety = state
      return

    # ========================================================================
    # Evaluate targets
    # ========================================================================

    emergency_candidates = []
    overtake_candidates = []

    for obj_id, obj in (
      self.mr76_objects.items()
    ):

      ttc = self._mr76_ttc(
        obj
      )

      # --------------------------------------------------------------
      # Closest object
      # --------------------------------------------------------------

      if (
        obj.dRel
        < state.closest_distance
      ):

        state.closest_distance = (
          obj.dRel
        )

        state.closest_object_id = (
          obj_id
        )

        state.ttc = ttc

      # --------------------------------------------------------------
      # Confirmation
      # --------------------------------------------------------------

      confirmed = (
        self.mr76_confirm_counts.get(
          obj_id,
          0,
        )
        >= MR76_CONFIRM_COUNT
      )

      emergency_confirmed = (
        self.mr76_emergency_confirm_counts.get(
          obj_id,
          0,
        )
        >= MR76_EMERGENCY_CONFIRM_COUNT
      )

      # --------------------------------------------------------------
      # Emergency candidate
      # --------------------------------------------------------------

      if (
        emergency_confirmed
        and
        self._mr76_emergency_candidate(
          obj
        )
      ):

        emergency_candidates.append(
          (
            obj,
            ttc,
          )
        )

      # --------------------------------------------------------------
      # Overtake veto candidate
      # --------------------------------------------------------------

      if confirmed:

        if (
          obj.dRel <= 30.0
          and
          (
            obj.obj_class == 1
            or
            abs(obj.vRel) > 2.0
          )
        ):

          overtake_candidates.append(
            obj
          )

    # ========================================================================
    # Emergency state
    # ========================================================================

    if emergency_candidates:

      emergency_obj, emergency_ttc = min(
        emergency_candidates,
        key=lambda x: x[1],
      )

      state.emergency_veto = True

      state.ttc = emergency_ttc

      state.emergency_confirmed_count = max(
        self.mr76_emergency_confirm_counts.values(),
        default=0,
      )

      # Negative yRel = object on left side.
      # Positive yRel = object on right side.
      #
      # These flags are only information for AutoAvoidance.
      if emergency_obj.yRel < -0.5:

        state.emergency_left = True

      elif emergency_obj.yRel > 0.5:

        state.emergency_right = True

      if (
        self.mr76_emergency_since
        <= 0.0
      ):

        self.mr76_emergency_since = (
          now
        )

    else:

      # ================================================================
      # Emergency clear hysteresis
      # ================================================================

      if (
        self.mr76_emergency_since
        > 0.0
        and
        (
          now
          -
          self.mr76_emergency_since
        )
        < MR76_CLEAR_HOLD_SEC
      ):

        state.emergency_veto = True

      else:

        self.mr76_emergency_since = 0.0

    # ========================================================================
    # AutoOvertake veto
    # ========================================================================

    if overtake_candidates:

      state.overtake_veto = True

      state.confirmed_count = max(
        (
          self.mr76_confirm_counts.get(
            obj.obj_id,
            0,
          )
          for obj in overtake_candidates
        ),
        default=0,
      )

    # ========================================================================
    # Store safety-only state
    # ========================================================================

    self.mr76_safety = state


  # ==========================================================================
  # OEM Delphi ESR
  # ==========================================================================

  def _update_delphi_esr(
    self,
  ):
    """
    OEM Ford/Lincoln Delphi ESR processing.

    MR76 is completely independent.
    """

    for ii in sorted(
      self.updated_messages
    ):

      cpt = self.rcp.vl[ii]

      if cpt['X_Rel'] > 0.00001:

        self.valid_cnt[ii] = 0
        self.valid_cnt[ii] += 1

      else:

        self.valid_cnt[ii] = max(
          self.valid_cnt[ii] - 1,
          0,
        )

      if self.valid_cnt[ii] > 0:

        if ii not in self.pts:

          self.pts[ii] = (
            structs.RadarData.RadarPoint()
          )

          self.pts[ii].trackId = (
            self.track_id
          )

          self.track_id += 1

        self.pts[ii].dRel = (
          cpt['X_Rel']
        )

        self.pts[ii].yRel = (
          cpt['X_Rel']
          * cpt['Angle']
          * CV.DEG_TO_RAD
        )

        self.pts[ii].vRel = (
          cpt['V_Rel']
        )

      else:

        if ii in self.pts:
          del self.pts[ii]


  # ==========================================================================
  # OEM Delphi MRR
  # ==========================================================================

  def _update_delphi_mrr(
    self,
    ret: structs.RadarData,
  ):
    """
    OEM Ford/Lincoln Delphi MRR processing.

    MR76 does not participate.

    Architecture:

        scan
          |
          v
        points
          |
          v
        cluster_points()
          |
          v
        clusters
          |
          v
        RadarPoint
    """

    headerScanIndex = int(
      self.rcp.vl[
        "MRR_Header_InformationDetections"
      ]['CAN_SCAN_INDEX']
    ) & 0b11

    # ========================================================================
    # Scan sequence health
    # ========================================================================

    # [FIX] Standstill exemption.
    #
    # At/near standstill the factory MRR legitimately stops cycling its scan
    # phases (it still transmits at 33.3 Hz, but the payload freezes). Counting
    # that as a fault raised radarUnavailableTemporary -> NO_ENTRY, so
    # openpilot exited and refused to re-engage in stop-and-go traffic.
    #
    # Below MRR_SCAN_MIN_VEGO the check does not apply: reset the counter and
    # re-baseline, so a standstill stall can never accumulate.
    _standstill = (
      self.v_ego is not None
      and self.v_ego < MRR_SCAN_MIN_VEGO
    )

    if _standstill:

      self.radar_unavailable_cnt = 0

    elif (
      (
        self.prev_headerScanIndex
        + 1
      ) % 4
      != headerScanIndex
    ):

      self.radar_unavailable_cnt += 1

    else:

      self.radar_unavailable_cnt = 0

    self.prev_headerScanIndex = (
      headerScanIndex
    )

    # ========================================================================
    # Radar unavailable
    # ========================================================================

    if (
      self.radar_unavailable_cnt
      >= 5
    ):

      self.pts.clear()
      self.points.clear()
      self.clusters.clear()

      ret.errors.radarUnavailableTemporary = (
        True
      )

      return True

    # ========================================================================
    # Only process scan phases 2 / 3
    # ========================================================================

    if headerScanIndex not in (
      2,
      3,
    ):

      return False

    # ========================================================================
    # Sensor coverage validation
    # ========================================================================

    expected_coverage = (
      DELPHI_MRR_RADAR_RANGE_COVERAGE[
        headerScanIndex
      ]
    )

    actual_coverage = int(
      self.rcp.vl[
        "MRR_Header_SensorCoverage"
      ]['CAN_RANGE_COVERAGE']
    )

    # [FIX] MRR_COVERAGE_STANDSTILL_GUARD
    #
    # While the MRR has halted its scan, CAN_RANGE_COVERAGE is frozen and is
    # compared against a table indexed by the frozen scan index, so the result
    # carries no information.  The scan-sequence exemption above also keeps
    # radar_unavailable_cnt at 0, which removes the early return that used to
    # protect this block, so it now runs on every frame instead of at most
    # four -- enough for scan_index_invalid_cnt to reach 5 and raise
    # wrongConfig.  wrongConfig maps to EventName.radarFault, the one branch
    # selfdrived deliberately does NOT gate, so unguarded it would bring back
    # the very disable this work removes.
    if (
      expected_coverage
      != actual_coverage
      and not _standstill
    ):

      self.scan_index_invalid_cnt += 1

    else:

      self.scan_index_invalid_cnt = 0

    if (
      self.scan_index_invalid_cnt
      >= 5
    ):

      ret.errors.wrongConfig = True

    # ========================================================================
    # Decode current MRR scan
    # ========================================================================

    for ii in range(
      1,
      DELPHI_MRR_RADAR_MSG_COUNT + 1,
    ):

      msg = self.rcp.vl[
        f"MRR_Detection_{ii:03d}"
      ]

      scanIndex = msg[
        f"CAN_SCAN_INDEX_2LSB_{ii:02d}"
      ]

      if scanIndex != headerScanIndex:
        continue

      valid = bool(
        msg[
          f"CAN_DET_VALID_LEVEL_{ii:02d}"
        ]
      )

      dist = msg[
        f"CAN_DET_RANGE_{ii:02d}"
      ]

      # ======================================================================
      # Long range filtering
      # ======================================================================

      if (
        scanIndex in (
          1,
          3,
        )
        and dist
        < DELPHI_MRR_MIN_LONG_RANGE_DIST
      ):

        valid = False

      if not valid:
        continue

      # ======================================================================
      # Position / velocity
      # ======================================================================

      azimuth = msg[
        f"CAN_DET_AZIMUTH_{ii:02d}"
      ]

      distRate = msg[
        f"CAN_DET_RANGE_RATE_{ii:02d}"
      ]

      dRel = (
        cos(azimuth)
        * dist
      )

      yRel = (
        -sin(azimuth)
        * dist
      )

      # Original implementation scaling retained.
      self.points.append([
        dRel,
        yRel * 2,
        distRate * 2,
      ])

    # ========================================================================
    # Publish only after scan mode 3
    # ========================================================================

    if headerScanIndex != 3:
      return False

    # ========================================================================
    # Cluster current scan cycle
    # ========================================================================

    prev_keys = [
      [
        p.dRel,
        p.yRel * 2,
        p.vRel * 2,
      ]
      for p in self.clusters
    ]

    labels = cluster_points(
      prev_keys,
      self.points,
      DELPHI_MRR_CLUSTER_THRESHOLD,
    )

    # ========================================================================
    # Group points by track ID
    # ========================================================================

    points_by_track_id = defaultdict(list)

    for idx, label in enumerate(
      labels
    ):

      if label != -1:

        points_by_track_id[
          self.clusters[label].trackId
        ].append(
          self.points[idx]
        )

      else:

        points_by_track_id[
          self.track_id
        ].append(
          self.points[idx]
        )

        self.track_id += 1

    # ========================================================================
    # Rebuild clusters
    # ========================================================================

    self.clusters = []

    for idx, (
      track_id,
      pts,
    ) in enumerate(
      points_by_track_id.items()
    ):

      dRel_values = [
        p[0]
        for p in pts
      ]

      min_dRel = min(
        dRel_values
      )

      dRel = (
        sum(dRel_values)
        / len(dRel_values)
      )

      yRel_values = [
        p[1]
        for p in pts
      ]

      yRel = (
        sum(yRel_values)
        / len(yRel_values)
        / 2
      )

      vRel_values = [
        p[2]
        for p in pts
      ]

      vRel = (
        sum(vRel_values)
        / len(vRel_values)
        / 2
      )

      self.clusters.append(
        Cluster(
          dRel=dRel,
          yRel=yRel,
          vRel=vRel,
          trackId=track_id,
        )
      )

      # ======================================================================
      # Reuse RadarPoint objects
      # ======================================================================

      if idx not in self.pts:

        self.pts[idx] = (
          structs.RadarData.RadarPoint()
        )

      self.pts[idx].dRel = (
        min_dRel
      )

      self.pts[idx].yRel = (
        yRel
      )

      self.pts[idx].vRel = (
        vRel
      )

      self.pts[idx].trackId = (
        track_id
      )

    # ========================================================================
    # Remove stale RadarPoints
    # ========================================================================

    for idx in range(
      len(points_by_track_id),
      len(self.pts),
    ):

      del self.pts[idx]

    # ========================================================================
    # Reset current MRR scan-cycle points
    # ========================================================================

    self.points = []

    return True


  # ==========================================================================
  # MR76 public API
  # ==========================================================================

  def get_mr76_objects(
    self,
  ) -> list[MR76Object]:
    """
    Return fresh MR76 auxiliary targets.

    These are NOT RadarData points.
    """

    if self.mr76_stale:
      return []

    return list(
      self.mr76_objects.values()
    )


  def get_mr76_object_dict(
    self,
  ) -> dict[int, MR76Object]:
    """
    Return a copy of the current MR76 cache.
    """

    if self.mr76_stale:
      return {}

    return dict(
      self.mr76_objects
    )


  def has_mr76_data(
    self,
  ) -> bool:
    """
    Whether MR76 currently has fresh targets.
    """

    return (
      self.mr76_updated
      and
      not self.mr76_stale
      and
      bool(
        self.mr76_objects
      )
    )


  def is_mr76_stale(
    self,
  ) -> bool:
    """
    Whether MR76 auxiliary data is stale.
    """

    return bool(
      self.mr76_stale
    )


  @staticmethod
  def _mr76_lane_vehicle(
    obj: MR76Object,
  ) -> str | None:
    """
    Classify an MR76 target as a vehicle occupying the adjacent lane.

    Returns:

      "moving"   branch A -- a vehicle travelling roughly alongside us
      "stopped"  branch B -- a stationary / stopped vehicle
      None       not an adjacent-lane vehicle

    See the CRITERIA v2 block above for the measured justification of every
    gate.  This is deliberately strict: a false "occupied" makes the car
    refuse a legitimate lane change, and the previous rule was 7.34x too
    eager because it treated roadside clutter as traffic.
    """

    # ------------------------------------------------------------------
    # A classified vehicle.  Class 0 is a bare point target and was 85 % of
    # the false hits.
    # ------------------------------------------------------------------

    if obj.obj_class != 1:
      return None

    # ------------------------------------------------------------------
    # Strong enough reflector to be metal, not clutter.
    # ------------------------------------------------------------------

    if obj.rcs < MR76_LANE_MIN_RCS:
      return None

    # ------------------------------------------------------------------
    # Laterally stable.
    # ------------------------------------------------------------------

    if abs(obj.vLat) > MR76_LANE_MAX_VLAT:
      return None

    # ------------------------------------------------------------------
    # Inside the adjacent lane, not our own lane and not the one beyond.
    # ------------------------------------------------------------------

    abs_y = abs(
      obj.yRel
    )

    if not (
      MR76_LANE_YREL_MIN
      <= abs_y
      <= MR76_LANE_YREL_MAX
    ):
      return None

    if obj.dRel < MR76_LANE_DIST_MIN:
      return None

    # ------------------------------------------------------------------
    # Branch A: moving vehicle, parallel to us.
    # ------------------------------------------------------------------

    if obj.dyn_prop == MR76_DYN_MOVING:

      if (
        abs(obj.vRel)
        > MR76_LANE_MOVING_MAX_VREL
      ):
        return None

      if (
        obj.dRel
        > MR76_LANE_MOVING_DIST_MAX
      ):
        return None

      return "moving"

    # ------------------------------------------------------------------
    # Branch B: stationary / stopped vehicle.
    #
    # No |vRel| gate here: a stopped vehicle in the target lane has
    # vRel ~ -vEgo, which is exactly why the old OR-rule could not tell it
    # apart from a guardrail.  What separates them is Class == 1 plus a
    # persistent track, both already required above.
    # ------------------------------------------------------------------

    if obj.dyn_prop in MR76_LANE_DYN_STOPPED:

      if (
        obj.dRel
        > MR76_LANE_STOPPED_DIST_MAX
      ):
        return None

      return "stopped"

    return None

  def get_mr76_lane_occupancy(
    self,
  ) -> dict:
    """
    Per-side adjacent-lane occupancy derived from confirmed MR76 targets.

    AUXILIARY SAFETY INFORMATION ONLY.

    Nothing returned here may be turned into a RadarData point, a radarTrack,
    leadOne / leadTwo, radarState, or any longitudinal target.  The only
    permitted use is a *veto*: blocking a lane change.

    Returns a plain dict, so callers outside this package do not need to
    import this module's dataclasses:

      {
        "left":   bool,  # something occupies the left adjacent lane
        "right":  bool,  # something occupies the right adjacent lane
        "fresh":  bool,  # MR76 data is currently usable at all
        "count":  int,   # confirmed targets that qualified
        "moving": int,   # of count, how many were branch A
        "stopped": int,  # of count, how many were branch B
      }

    When MR76 is stale / disconnected everything is False, so a missing
    auxiliary radar can never block anything by itself.

    NOTE ON SEMANTICS: this answers "is the adjacent lane occupied AHEAD of
    me?", NOT "is my blind spot clear?".  The factory BSM owns the side/rear
    blind spot.  Measured against BSM on 107,492 driving batches, the v2 rule
    catches ~13 % of BSM events, so MR76 supplements BSM and must never be
    used to replace it.
    """

    occupancy = {
      "left": False,
      "right": False,
      "fresh": not self.mr76_stale,
      "count": 0,
      "moving": 0,
      "stopped": 0,
    }

    if self.mr76_stale:
      return occupancy

    for obj_id, obj in self.mr76_objects.items():

      # Only trust targets that survived the confirmation counter.
      if (
        self.mr76_confirm_counts.get(
          obj_id,
          0,
        )
        < MR76_LANE_CONFIRM_COUNT
      ):
        continue

      kind = self._mr76_lane_vehicle(
        obj
      )

      if kind is None:
        continue

      occupancy["count"] += 1
      occupancy[kind] += 1

      # Negative yRel = object on the left.
      # Positive yRel = object on the right.
      if obj.yRel < 0.0:
        occupancy["left"] = True
      else:
        occupancy["right"] = True

    return occupancy


  def get_mr76_safety_state(
    self,
  ) -> MR76SafetyState:
    """
    Return an independent snapshot of MR76 safety information.

    Intended consumers:

      AutoOvertake
      AutoAvoidance

    This MUST NOT be used as a longitudinal radar source.
    """

    return MR76SafetyState(
      fresh=self.mr76_safety.fresh,

      object_count=(
        self.mr76_safety.object_count
      ),

      overtake_veto=(
        self.mr76_safety.overtake_veto
      ),

      emergency_veto=(
        self.mr76_safety.emergency_veto
      ),

      emergency_left=(
        self.mr76_safety.emergency_left
      ),

      emergency_right=(
        self.mr76_safety.emergency_right
      ),

      closest_distance=(
        self.mr76_safety.closest_distance
      ),

      closest_object_id=(
        self.mr76_safety.closest_object_id
      ),

      ttc=(
        self.mr76_safety.ttc
      ),

      confirmed_count=(
        self.mr76_safety.confirmed_count
      ),

      emergency_confirmed_count=(
        self.mr76_safety.emergency_confirmed_count
      ),

      timestamp=(
        self.mr76_safety.timestamp
      ),
    )


  def get_mr76_status(
    self,
  ) -> dict:
    """
    Return MR76 diagnostic information.

    This is auxiliary information only.
    """

    now = time.monotonic()

    newest_object_age = None

    if self.mr76_objects:

      newest_rx = max(
        obj.last_seen
        for obj in self.mr76_objects.values()
      )

      newest_object_age = max(
        0.0,
        now - newest_rx,
      )

    status_age = None

    if (
      self.mr76_last_status_time
      > 0
    ):

      status_age = max(
        0.0,
        now
        - self.mr76_last_status_time,
      )

    safety = (
      self.get_mr76_safety_state()
    )

    return {
      "object_count": int(
        self.mr76_object_count
      ),

      "meas_count": int(
        self.mr76_meas_count
      ),

      "rx_count": int(
        self.mr76_rx_count
      ),

      "status_count": int(
        self.mr76_status_count
      ),

      "active_objects": int(
        len(
          self.mr76_objects
        )
      ),

      "stale": bool(
        self.mr76_stale
      ),

      "stale_updates": int(
        self.mr76_stale_updates
      ),

      "parse_error_count": int(
        self.mr76_parse_error_count
      ),

      "newest_object_age": (
        newest_object_age
      ),

      "status_age": (
        status_age
      ),

      # Safety-only diagnostics.
      "overtake_veto": bool(
        safety.overtake_veto
      ),

      "emergency_veto": bool(
        safety.emergency_veto
      ),

      "emergency_left": bool(
        safety.emergency_left
      ),

      "emergency_right": bool(
        safety.emergency_right
      ),

      "closest_distance": (
        safety.closest_distance
      ),

      "closest_object_id": (
        safety.closest_object_id
      ),

      "ttc": (
        safety.ttc
      ),
    }


  def clear_mr76_data(
    self,
  ):
    """
    Clear all MR76 auxiliary data.

    OEM radar state is untouched.
    """

    self.mr76_objects.clear()

    self.mr76_confirm_counts.clear()

    self.mr76_emergency_confirm_counts.clear()

    self.mr76_object_count = 0
    self.mr76_meas_count = 0

    self.mr76_updated = False
    self.mr76_stale = True

    self.mr76_stale_updates = 0

    self.mr76_last_rx_time = 0.0
    self.mr76_last_status_time = 0.0

    self.mr76_emergency_since = 0.0

    self.mr76_safety = (
      MR76SafetyState()
    )