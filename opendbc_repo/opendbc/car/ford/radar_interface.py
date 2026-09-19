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
# Ford / Lincoln OEM Delphi ESR radar
# ============================================================================

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))


# ============================================================================
# Ford / Lincoln OEM Delphi MRR radar
# ============================================================================

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x174
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

MR76_BUS = 1

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

MR76_STATUS_FREQ = 20
MR76_OBJECT_FREQ = 20


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
      self.mr76_last_status_time > 0
      and
      (
        now
        - self.mr76_last_status_time
      ) <= MR76_STATUS_TIMEOUT_SEC
    ):

      # MR76 alive but currently sees no objects.
      self.mr76_stale = False

    elif (
      self.mr76_last_rx_time > 0
      and
      (
        now
        - self.mr76_last_rx_time
      ) <= MR76_OBJECT_TIMEOUT_SEC
    ):

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

    if (
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

    if (
      expected_coverage
      != actual_coverage
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
