import time
from collections import defaultdict
from dataclasses import dataclass
from math import cos, sin
from typing import cast

import numpy as np

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase


# ============================================================================
# Ford / Lincoln OEM Delphi radar
# ============================================================================

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_HEADER_INFO_ADDR = 0x170
DELPHI_MRR_RADAR_HEADER_COVERAGE_ADDR = 0x174

DELPHI_MRR_RADAR_MSG_COUNT = 64

# Kept as diagnostic/reference information only.
#
# IMPORTANT:
#   These values are NOT scan phases.
#   They are NOT used to determine scan sequence.
#   They are NOT used to generate radarUnavailableTemporary.
DELPHI_MRR_RADAR_RANGE_COVERAGE = {
  0: 42,
  1: 164,
  2: 45,
  3: 175,
}

DELPHI_MRR_CLUSTER_THRESHOLD = 5.0


# ============================================================================
# Smartmicro MR76 auxiliary radar
# ============================================================================

"""
MR76 physical connection:

    CAN1

u_radar.dbc:

    BO_ 513  RadarState: 8
    BO_ 1546 Status: 8
    BO_ 1547 ObjectData: 8

Only:

    0x60A Status
    0x60B ObjectData

are decoded.

0x201 RadarState is intentionally ignored.

MR76 is an AUXILIARY safety sensor.

It MUST NOT enter:

    - self.pts
    - self.points
    - self.clusters
    - RadarData.points
    - radarTracks
    - leadOne
    - leadTwo
    - longitudinalPlan
    - aTarget
    - shouldStop
    - actuator/control CAN messages

MR76 only produces:

    MR76Object cache
    MR76SafetyState
"""

MR76_RADAR_DBC = "u_radar"

MR76_BUS = 1

MR76_RADAR_STATE_CAN_ID = 0x201
MR76_STATUS_CAN_ID = 0x60A
MR76_OBJECT_DATA_CAN_ID = 0x60B

MR76_STATUS_MSG = "Status"
MR76_OBJECT_DATA_MSG = "ObjectData"


# ============================================================================
# MR76 sanity limits
# ============================================================================

# Actual forward safety target distance.
MR76_MIN_DISTANCE = 0.0
MR76_MAX_DISTANCE = 150.0

# DBC physical range is much wider, but auxiliary forward safety logic
# deliberately rejects absurd lateral values.
MR76_MAX_YREL = 25.0

MR76_MAX_VREL = 80.0
MR76_MAX_VLAT = 50.0

MR76_VALID_CLASSES = {0, 1}


# ============================================================================
# MR76 freshness
# ============================================================================

MR76_OBJECT_TIMEOUT = 0.50
MR76_STATUS_TIMEOUT = 1.00

# This counter is only an auxiliary diagnostic guard.
MR76_MAX_STALE_UPDATES = 10


# ============================================================================
# MR76 safety thresholds
# ============================================================================

# A target with extremely small distance is immediately safety relevant.
MR76_MIN_SAFETY_DISTANCE = 1.0

# Negative longitudinal relative velocity means closing.
MR76_MIN_CLOSING_SPEED = -2.0

# Emergency TTC.
MR76_EMERGENCY_TTC = 2.0

# Auxiliary overtake veto TTC.
MR76_OVERTAKE_TTC = 4.0

# Auxiliary overtake veto distance.
MR76_OVERTAKE_DISTANCE = 30.0

# Confirmation counts.
MR76_CONFIRM_COUNT = 3
MR76_EMERGENCY_CONFIRM_COUNT = 2

# Safety latch / clear hysteresis.
MR76_CLEAR_HOLD = 0.40


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
  Auxiliary Smartmicro MR76 target.

  This is deliberately NOT structs.RadarData.RadarPoint.
  """

  obj_id: int = 0
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  vLat: float = 0.0
  dyn_prop: int = 5
  obj_class: int = 0
  rcs: float = 0.0
  last_seen: float = 0.0


@dataclass
class MR76SafetyState:
  """
  Independent MR76 auxiliary safety result.

  This structure must never be converted to RadarData.RadarPoint.

  It is intended to be consumed by auxiliary safety features such as:

    AutoAvoidance
    AutoOvertake

  It does not authorize an action.
  It only exposes an auxiliary veto condition.
  """

  fresh: bool = False
  object_count: int = 0

  overtake_veto: bool = False

  emergency_veto: bool = False
  emergency_left: bool = False
  emergency_right: bool = False

  closest_distance: float = 0.0
  closest_id: int = -1

  ttc: float = float("inf")

  confirmed_count: int = 0
  emergency_confirmed_count: int = 0

  timestamp: float = 0.0


# ============================================================================
# OEM clustering helper
# ============================================================================

def cluster_points(
  pts_l: list[list[float]],
  pts2_l: list[list[float]],
  max_dist: float,
) -> list[int]:
  """
  Cluster pts2 against pts.

  Returns the closest cluster index for each pts2 item,
  or -1 when outside max_dist.
  """

  if not pts2_l:
    return []

  if not pts_l:
    return [-1] * len(pts2_l)

  max_dist_sq = max_dist ** 2

  pts = np.asarray(pts_l, dtype=float)
  pts2 = np.asarray(pts2_l, dtype=float)

  pts_norm_sq = np.sum(pts ** 2, axis=1)
  pts2_norm_sq = np.sum(pts2 ** 2, axis=1)

  dist_sq = (
    pts2_norm_sq[:, np.newaxis]
    + pts_norm_sq[np.newaxis, :]
    - 2 * np.dot(pts2, pts.T)
  )

  dist_sq = np.maximum(dist_sq, 0.0)

  closest_clusters = np.argmin(dist_sq, axis=1)

  closest_dist_sq = dist_sq[
    np.arange(len(pts2)),
    closest_clusters,
  ]

  cluster_idxs = np.where(
    closest_dist_sq < max_dist_sq,
    closest_clusters,
    -1,
  )

  return cast(list[int], cluster_idxs.tolist())


# ============================================================================
# CAN parser creation
# ============================================================================

def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  """
  OEM Delphi ESR parser.

  Current CANParser API:

      CANParser(dbc_name, [(message, frequency)], bus)
  """

  messages = [
    (addr, 20)
    for addr in DELPHI_ESR_RADAR_MSGS
  ]

  return CANParser(
    RADAR.DELPHI_ESR,
    messages,
    CanBus(CP).radar,
  )


def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  """
  OEM Delphi MRR parser.

  0x170:
      MRR_Header_InformationDetections

  0x174:
      MRR_Header_SensorCoverage

  64 detection messages:
      MRR_Detection_001 ... MRR_Detection_064
  """

  messages = [
    ("MRR_Header_InformationDetections", 33),
    ("MRR_Header_SensorCoverage", 33),
  ]

  for i in range(
    1,
    DELPHI_MRR_RADAR_MSG_COUNT + 1,
  ):
    messages.append(
      (f"MRR_Detection_{i:03d}", 33)
    )

  return CANParser(
    RADAR.DELPHI_MRR,
    messages,
    CanBus(CP).radar,
  )


def _create_mr76_radar_can_parser() -> CANParser:
  """
  Independent Smartmicro MR76 parser.

  IMPORTANT:

  MR76 is physically on CAN1.

  Therefore this parser intentionally uses:

      MR76_BUS = 1

  and does NOT use CanBus(CP).radar.

  This prevents the MR76 auxiliary bus from being accidentally coupled
  to the OEM Delphi radar bus configuration.
  """

  messages = [
    (MR76_STATUS_CAN_ID, 20),
    (MR76_OBJECT_DATA_CAN_ID, 20),
  ]

  return CANParser(
    MR76_RADAR_DBC,
    messages,
    MR76_BUS,
  )


# ============================================================================
# Radar interface
# ============================================================================

class RadarInterface(RadarInterfaceBase):

  def __init__(self, CP, CP_SP=None):
    super().__init__(CP)

    # ========================================================================
    # OEM radar state
    # ========================================================================

    self.points: list[list[float]] = []
    self.clusters: list[Cluster] = []

    self.track_id = 0

    self.updated_messages = set()

    self.scan_index_invalid_cnt = 0

    # Kept only for compatibility/debugging.
    #
    # This is NOT used as a scan phase state machine.
    self.prev_headerScanIndex = None

    # Compatibility with existing OEM radar code.
    self.radar_unavailable_cnt = 0

    self.radar = DBC[CP.carFingerprint].get(Bus.radar)

    self.mr76_rcp = None

    # ========================================================================
    # MR76 auxiliary state
    # ========================================================================

    self.mr76_objects: dict[int, MR76Object] = {}

    # Compatibility alias for existing code.
    self.mr76_points = self.mr76_objects

    self.mr76_object_count = 0
    self.mr76_measurement_count = 0
    self.mr76_interface_version = 0

    self.mr76_updated = False

    self.mr76_last_status_time = 0.0
    self.mr76_last_object_time = 0.0

    self.mr76_stale_updates = 0

    self.mr76_confirm_counts: dict[int, int] = {}

    self.mr76_emergency_latched_until = 0.0
    self.mr76_overtake_latched_until = 0.0

    self.mr76_safety_state = MR76SafetyState()

    # ========================================================================
    # OEM radar parser
    # ========================================================================

    if CP.radarUnavailable:
      self.rcp = None
      self.trigger_msg = None
      self.valid_cnt = {}

    elif self.radar == RADAR.DELPHI_ESR:

      self.rcp = _create_delphi_esr_radar_can_parser(CP)

      # CANParser.update() returns integer addresses.
      self.trigger_msg = DELPHI_ESR_RADAR_MSGS[-1]

      self.valid_cnt = {
        key: 0
        for key in DELPHI_ESR_RADAR_MSGS
      }

    elif self.radar == RADAR.DELPHI_MRR:

      self.rcp = _create_delphi_mrr_radar_can_parser(CP)

      # IMPORTANT:
      #
      # CANParser.update() returns addresses, not message names.
      #
      # Therefore this MUST be 0x170, not the string
      # "MRR_Header_InformationDetections".
      self.trigger_msg = DELPHI_MRR_RADAR_HEADER_INFO_ADDR

    else:
      raise ValueError(
        f"Unsupported radar: {self.radar}"
      )

    # ========================================================================
    # Independent MR76 parser
    # ========================================================================

    try:
      self.mr76_rcp = _create_mr76_radar_can_parser()

    except (
      KeyError,
      ValueError,
      FileNotFoundError,
      RuntimeError,
    ):
      # MR76 is optional.
      #
      # OEM Delphi radar MUST remain fully operational when the auxiliary
      # DBC/parser is unavailable.
      self.mr76_rcp = None

  # ==========================================================================
  # Public update
  # ==========================================================================

  def update(self, can_strings):

    # ------------------------------------------------------------------------
    # MR76 AUXILIARY PATH
    #
    # Completely independent from OEM radar.
    # ------------------------------------------------------------------------

    self._update_mr76(can_strings)

    # ------------------------------------------------------------------------
    # OEM RADAR PATH
    # ------------------------------------------------------------------------

    if self.rcp is None:
      return super().update(None)

    updated_addrs = self.rcp.update(can_strings)

    self.updated_messages.update(updated_addrs)

    if self.trigger_msg not in self.updated_messages:
      return None

    self.updated_messages.clear()

    ret = structs.RadarData()

    # OEM radar validity only.
    #
    # MR76 never changes this value.
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()

    elif self.radar == RADAR.DELPHI_MRR:
      if not self._update_delphi_mrr(ret):
        return None

    # ------------------------------------------------------------------------
    # CRITICAL ISOLATION BOUNDARY
    #
    # Only OEM Delphi points are exported.
    #
    # MR76 is NEVER appended here.
    # ------------------------------------------------------------------------

    ret.points = list(
      self.pts.values()
    )

    return ret

  # ==========================================================================
  # MR76 auxiliary parser
  # ==========================================================================

  def _get_can_timestamp(self, can_strings) -> float:
    """
    Return the latest CAN timestamp in seconds.

    CANParser timestamps are nanoseconds.
    """

    latest_ns = 0

    try:
      for entry in can_strings:
        if not entry:
          continue

        t = int(entry[0])

        if t > latest_ns:
          latest_ns = t

    except (
      TypeError,
      ValueError,
      IndexError,
    ):
      latest_ns = 0

    if latest_ns > 0:
      return latest_ns / 1e9

    return time.monotonic()

  def _update_mr76(self, can_strings):
    """
    Update MR76 auxiliary cache.

    Important:

      vl:
          latest message

      vl_all:
          ALL messages of the same address received during this
          parser.update() call.

    Because 0x60B represents ONE OBJECT PER CAN FRAME, this function
    deliberately reads:

        vl_all[0x60B]

    rather than:

        vl[0x60B]

    This prevents multiple MR76 objects from collapsing into only
    the last received object.
    """

    if self.mr76_rcp is None:
      return

    now = self._get_can_timestamp(can_strings)

    try:
      updated_addrs = self.mr76_rcp.update(can_strings)

    except (
      KeyError,
      ValueError,
      IndexError,
      TypeError,
    ):
      return

    if not updated_addrs:
      self._prune_mr76_objects(now)
      self._update_mr76_safety(now)
      return

    self.mr76_updated = True

    status_updated = (
      MR76_STATUS_CAN_ID in updated_addrs
    )

    object_updated = (
      MR76_OBJECT_DATA_CAN_ID in updated_addrs
    )

    # ========================================================================
    # 0x60A Status
    # ========================================================================

    if status_updated:

      try:
        state = self.mr76_rcp.vl[
          MR76_STATUS_CAN_ID
        ]

        self.mr76_object_count = int(
          state["NoOfObjects"]
        )

        self.mr76_measurement_count = int(
          state["MeasCount"]
        )

        self.mr76_interface_version = int(
          state["InterfaceVersion"]
        )

        self.mr76_last_status_time = now

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        pass

    # ========================================================================
    # 0x60B ObjectData
    # ========================================================================

    if object_updated:
      self._decode_mr76_objects(now)

    else:
      self.mr76_stale_updates += 1

    self._prune_mr76_objects(now)
    self._update_mr76_safety(now)

  # ==========================================================================
  # MR76 ObjectData decoder
  # ==========================================================================

  def _decode_mr76_objects(self, now: float):
    """
    Decode ALL 0x60B ObjectData messages from the current parser update.

    DBC physical scaling is already performed by CANParser.

    Therefore DO NOT manually apply:

        * 0.2
        * 0.25
        * 0.5
        * offsets

    The values obtained here are already physical values.
    """

    try:
      objects = self.mr76_rcp.vl_all[
        MR76_OBJECT_DATA_CAN_ID
      ]

    except KeyError:
      return

    ids = objects.get("ID", [])
    dists = objects.get("DistLong", [])
    lats = objects.get("DistLat", [])
    vrels = objects.get("VRelLong", [])
    vlats = objects.get("VRelLat", [])
    dyn_props = objects.get("DynProp", [])
    classes = objects.get("Class", [])
    rcss = objects.get("RCS", [])

    count = min(
      len(ids),
      len(dists),
      len(lats),
      len(vrels),
      len(vlats),
      len(dyn_props),
      len(classes),
      len(rcss),
    )

    if count <= 0:
      return

    self.mr76_last_object_time = now
    self.mr76_stale_updates = 0

    for i in range(count):

      try:
        obj_id = int(ids[i])

        dRel = float(dists[i])
        yRel = float(lats[i])
        vRel = float(vrels[i])
        vLat = float(vlats[i])

        dyn_prop = int(dyn_props[i])
        obj_class = int(classes[i])

        rcs = float(rcss[i])

      except (
        TypeError,
        ValueError,
        IndexError,
      ):
        continue

      # ----------------------------------------------------------------------
      # Finite-value checks
      # ----------------------------------------------------------------------

      if not np.isfinite(dRel):
        continue

      if not np.isfinite(yRel):
        continue

      if not np.isfinite(vRel):
        continue

      if not np.isfinite(vLat):
        continue

      if not np.isfinite(rcs):
        continue

      # ----------------------------------------------------------------------
      # ID sanity
      # ----------------------------------------------------------------------

      if not 0 <= obj_id <= 255:
        continue

      # ----------------------------------------------------------------------
      # Distance sanity
      # ----------------------------------------------------------------------

      if dRel < MR76_MIN_DISTANCE:
        continue

      if dRel > MR76_MAX_DISTANCE:
        continue

      # ----------------------------------------------------------------------
      # Lateral sanity
      # ----------------------------------------------------------------------

      if abs(yRel) > MR76_MAX_YREL:
        continue

      # ----------------------------------------------------------------------
      # Velocity sanity
      # ----------------------------------------------------------------------

      if abs(vRel) > MR76_MAX_VREL:
        continue

      if abs(vLat) > MR76_MAX_VLAT:
        continue

      # ----------------------------------------------------------------------
      # DBC enum sanity
      # ----------------------------------------------------------------------

      if dyn_prop < 0 or dyn_prop > 7:
        continue

      if obj_class not in MR76_VALID_CLASSES:
        continue

      # ----------------------------------------------------------------------
      # Update confirmation counter
      # ----------------------------------------------------------------------

      self.mr76_confirm_counts[obj_id] = min(
        self.mr76_confirm_counts.get(obj_id, 0) + 1,
        100,
      )

      # ----------------------------------------------------------------------
      # Store auxiliary object
      #
      # NEVER create RadarPoint here.
      # ----------------------------------------------------------------------

      self.mr76_objects[obj_id] = MR76Object(
        obj_id=obj_id,
        dRel=dRel,
        yRel=yRel,
        vRel=vRel,
        vLat=vLat,
        dyn_prop=dyn_prop,
        obj_class=obj_class,
        rcs=rcs,
        last_seen=now,
      )

  # ==========================================================================
  # MR76 freshness
  # ==========================================================================

  def _prune_mr76_objects(self, now: float):
    """
    Remove stale MR76 objects.

    No OEM radar state is touched here.
    """

    stale_ids = []

    for obj_id, obj in self.mr76_objects.items():
      if (
        now - obj.last_seen
        > MR76_OBJECT_TIMEOUT
      ):
        stale_ids.append(obj_id)

    for obj_id in stale_ids:
      self.mr76_objects.pop(
        obj_id,
        None,
      )

      self.mr76_confirm_counts.pop(
        obj_id,
        None,
      )

    # Status freshness is tracked independently.
    if (
      self.mr76_last_status_time > 0
      and now - self.mr76_last_status_time
      > MR76_STATUS_TIMEOUT
    ):
      self.mr76_object_count = 0

  # ==========================================================================
  # MR76 safety state
  # ==========================================================================

  def _update_mr76_safety(self, now: float):
    """
    Calculate the independent MR76 safety state.

    This function does NOT:

      - modify OEM radar points
      - modify radarState
      - modify lead targets
      - modify longitudinalPlan
      - send CAN
      - request braking

    It only creates a safety/veto state for higher-level auxiliary logic.
    """

    object_count = len(
      self.mr76_objects
    )

    object_fresh = object_count > 0

    status_fresh = (
      self.mr76_last_status_time > 0
      and (
        now - self.mr76_last_status_time
        <= MR76_STATUS_TIMEOUT
      )
    )

    # Object data itself is sufficient to regard MR76 as fresh.
    fresh = object_fresh

    closest_distance = float("inf")
    closest_id = -1
    minimum_ttc = float("inf")

    overtake_candidates = []
    emergency_candidates = []

    for obj_id, obj in self.mr76_objects.items():

      if obj.dRel < closest_distance:
        closest_distance = obj.dRel
        closest_id = obj_id

      # --------------------------------------------------------------
      # Closing target
      # --------------------------------------------------------------

      closing = (
        obj.vRel < MR76_MIN_CLOSING_SPEED
      )

      if closing:
        ttc = (
          obj.dRel / (-obj.vRel)
          if obj.vRel < 0
          else float("inf")
        )

        if ttc < minimum_ttc:
          minimum_ttc = ttc

      else:
        ttc = float("inf")

      confirmed = (
        self.mr76_confirm_counts.get(
          obj_id,
          0,
        )
        >= MR76_CONFIRM_COUNT
      )

      emergency_confirmed = (
        self.mr76_confirm_counts.get(
          obj_id,
          0,
        )
        >= MR76_EMERGENCY_CONFIRM_COUNT
      )

      # --------------------------------------------------------------
      # Emergency candidate
      #
      # Any valid class can be safety relevant.
      # --------------------------------------------------------------

      emergency = False

      if obj.dRel <= MR76_MIN_SAFETY_DISTANCE:
        emergency = True

      elif (
        closing
        and ttc <= MR76_EMERGENCY_TTC
      ):
        emergency = True

      if emergency and emergency_confirmed:
        emergency_candidates.append(obj)

      # --------------------------------------------------------------
      # Overtake veto candidate
      #
      # For overtaking, require a vehicle-class target.
      #
      # MR76 never authorizes overtake.
      # It can only veto.
      # --------------------------------------------------------------

      if (
        obj.obj_class == 1
        and obj.dRel <= MR76_OVERTAKE_DISTANCE
        and confirmed
      ):

        if closing:
          if ttc <= MR76_OVERTAKE_TTC:
            overtake_candidates.append(obj)

        elif obj.dyn_prop in (
          0,  # moving
          1,  # stationary
          6,  # stopped
        ):
          # A confirmed vehicle within the auxiliary veto distance
          # is retained as an overtake veto candidate.
          overtake_candidates.append(obj)

    # ========================================================================
    # Current emergency state
    # ========================================================================

    emergency_veto_now = (
      len(emergency_candidates) > 0
    )

    emergency_left_now = False
    emergency_right_now = False

    for obj in emergency_candidates:
      if obj.yRel < 0:
        emergency_left_now = True
      elif obj.yRel > 0:
        emergency_right_now = True

    # ========================================================================
    # Current overtake veto
    # ========================================================================

    overtake_veto_now = (
      len(overtake_candidates) > 0
    )

    # ========================================================================
    # Latching
    # ========================================================================

    if emergency_veto_now:
      self.mr76_emergency_latched_until = (
        now + MR76_CLEAR_HOLD
      )

    if overtake_veto_now:
      self.mr76_overtake_latched_until = (
        now + MR76_CLEAR_HOLD
      )

    emergency_veto = (
      emergency_veto_now
      or now < self.mr76_emergency_latched_until
    )

    overtake_veto = (
      overtake_veto_now
      or now < self.mr76_overtake_latched_until
    )

    # ========================================================================
    # Direction during latch
    #
    # When there is no currently active candidate, preserve direction from
    # the latest emergency candidates only while the emergency latch remains.
    # ========================================================================

    if not emergency_veto_now:
      emergency_left_now = False
      emergency_right_now = False

      for obj in self.mr76_objects.values():

        count = self.mr76_confirm_counts.get(
          obj.obj_id,
          0,
        )

        if count < MR76_EMERGENCY_CONFIRM_COUNT:
          continue

        if obj.yRel < 0:
          emergency_left_now = True

        elif obj.yRel > 0:
          emergency_right_now = True

    # ========================================================================
    # Confirmed counts
    # ========================================================================

    confirmed_count = 0
    emergency_confirmed_count = 0

    for obj_id in self.mr76_objects:

      count = self.mr76_confirm_counts.get(
        obj_id,
        0,
      )

      if count >= MR76_CONFIRM_COUNT:
        confirmed_count += 1

      if count >= MR76_EMERGENCY_CONFIRM_COUNT:
        emergency_confirmed_count += 1

    # ========================================================================
    # Closest distance normalization
    # ========================================================================

    if closest_id < 0:
      closest_distance_out = 0.0
    else:
      closest_distance_out = closest_distance

    self.mr76_safety_state = MR76SafetyState(
      fresh=fresh,
      object_count=object_count,

      overtake_veto=overtake_veto,

      emergency_veto=emergency_veto,
      emergency_left=emergency_left_now,
      emergency_right=emergency_right_now,

      closest_distance=closest_distance_out,
      closest_id=closest_id,

      ttc=minimum_ttc,

      confirmed_count=confirmed_count,
      emergency_confirmed_count=emergency_confirmed_count,

      timestamp=now,
    )

  # ==========================================================================
  # MR76 public APIs
  # ==========================================================================

  def get_mr76_objects(self) -> list[MR76Object]:
    """
    Return current MR76 auxiliary objects.

    This is never an OEM RadarData list.
    """

    now = time.monotonic()

    self._prune_mr76_objects(now)

    return list(
      self.mr76_objects.values()
    )

  def get_mr76_object_dict(self) -> dict[int, MR76Object]:
    """
    Return a copy of the current MR76 object cache.
    """

    now = time.monotonic()

    self._prune_mr76_objects(now)

    return dict(
      self.mr76_objects
    )

  def has_mr76_data(self) -> bool:
    """
    True when at least one MR76 object is currently fresh.
    """

    self._prune_mr76_objects(
      time.monotonic()
    )

    return bool(
      self.mr76_objects
    )

  def is_mr76_stale(self) -> bool:
    """
    True when MR76 object data is stale or absent.
    """

    now = time.monotonic()

    self._prune_mr76_objects(now)

    if not self.mr76_objects:
      return True

    return (
      self.mr76_last_object_time <= 0
      or (
        now - self.mr76_last_object_time
        > MR76_OBJECT_TIMEOUT
      )
    )

  def get_mr76_safety_state(self) -> MR76SafetyState:
    """
    Return the latest independent MR76 safety state.

    This state is auxiliary only.
    """

    now = time.monotonic()

    self._prune_mr76_objects(now)

    self._update_mr76_safety(now)

    return self.mr76_safety_state

  def get_mr76_status(self) -> dict:
    """
    Diagnostic MR76 status.

    This does not expose MR76 as RadarData.
    """

    now = time.monotonic()

    self._prune_mr76_objects(now)

    state = self.mr76_safety_state

    return {
      "available": self.mr76_rcp is not None,

      "fresh": state.fresh,

      "object_count": len(
        self.mr76_objects
      ),

      "status_object_count": (
        self.mr76_object_count
      ),

      "measurement_count": (
        self.mr76_measurement_count
      ),

      "interface_version": (
        self.mr76_interface_version
      ),

      "last_status_age": (
        now - self.mr76_last_status_time
        if self.mr76_last_status_time > 0
        else float("inf")
      ),

      "last_object_age": (
        now - self.mr76_last_object_time
        if self.mr76_last_object_time > 0
        else float("inf")
      ),

      "overtake_veto": (
        state.overtake_veto
      ),

      "emergency_veto": (
        state.emergency_veto
      ),

      "emergency_left": (
        state.emergency_left
      ),

      "emergency_right": (
        state.emergency_right
      ),

      "closest_distance": (
        state.closest_distance
      ),

      "closest_id": (
        state.closest_id
      ),

      "ttc": (
        state.ttc
      ),
    }

  def clear_mr76_data(self):
    """
    Clear only the auxiliary MR76 cache.

    OEM Delphi radar state is untouched.
    """

    self.mr76_objects.clear()
    self.mr76_confirm_counts.clear()

    self.mr76_object_count = 0
    self.mr76_measurement_count = 0
    self.mr76_interface_version = 0

    self.mr76_last_status_time = 0.0
    self.mr76_last_object_time = 0.0

    self.mr76_updated = False
    self.mr76_stale_updates = 0

    self.mr76_emergency_latched_until = 0.0
    self.mr76_overtake_latched_until = 0.0

    self.mr76_safety_state = (
      MR76SafetyState()
    )

  # ==========================================================================
  # OEM Delphi ESR
  # ==========================================================================

  def _update_delphi_esr(self):

    for ii in sorted(
      self.updated_messages
    ):

      cpt = self.rcp.vl[ii]

      if cpt["X_Rel"] > 0.00001:
        self.valid_cnt[ii] = 0

      if cpt["X_Rel"] > 0.00001:
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
          cpt["X_Rel"]
        )

        # In car frame's y axis,
        # left is positive.

        self.pts[ii].yRel = (
          cpt["X_Rel"]
          * cpt["Angle"]
          * CV.DEG_TO_RAD
        )

        self.pts[ii].vRel = (
          cpt["V_Rel"]
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
  ) -> bool:
    """
    Decode one OEM Delphi MRR header-triggered update.

    IMPORTANT MRR synchronization rule:

        CAN_SCAN_INDEX
            |
            +-- full 16-bit value is preserved
            |
            +-- only low 2 bits are used for matching
                    |
                    v
        CAN_SCAN_INDEX_2LSB_xx

    The low two bits are NOT interpreted as a four-state scan phase.

    There is NO requirement for:

        0 -> 1 -> 2 -> 3

    and no scan-sequence health failure is generated from them.

    0x174 CAN_RANGE_COVERAGE is diagnostic/reference data only.
    """

    # ========================================================================
    # 0x170 MRR header
    # ========================================================================

    try:
      header = self.rcp.vl[
        "MRR_Header_InformationDetections"
      ]

      header_scan_index = int(
        header["CAN_SCAN_INDEX"]
      )

    except (
      KeyError,
      TypeError,
      ValueError,
    ):
      return False

    # Keep complete 16-bit scan index.
    header_scan_index &= 0xFFFF

    # Only the low 2 bits are used to correlate detection slots.
    scan_index_2lsb = (
      header_scan_index & 0x3
    )

    # Store for diagnostics only.
    self.prev_headerScanIndex = (
      header_scan_index
    )

    # ========================================================================
    # 0x174 SensorCoverage
    #
    # Diagnostic only.
    #
    # DO NOT:
    #
    #   - map 42/164/45/175 to phase
    #   - reject a scan based on coverage
    #   - set radarUnavailableTemporary
    #   - set wrongConfig
    # ========================================================================

    coverage = None

    try:
      coverage = int(
        self.rcp.vl[
          "MRR_Header_SensorCoverage"
        ]["CAN_RANGE_COVERAGE"]
      )

    except (
      KeyError,
      TypeError,
      ValueError,
    ):
      coverage = None

    # Retain the value only for optional debugging.
    self.mrr_range_coverage = coverage

    # ========================================================================
    # Decode all 64 detection slots.
    #
    # IMPORTANT:
    #
    # Do not restrict this to scan modes 2 / 3.
    #
    # The actual synchronization rule is:
    #
    #     detection 2LSB == header CAN_SCAN_INDEX & 0x3
    # ========================================================================

    for ii in range(
      1,
      DELPHI_MRR_RADAR_MSG_COUNT + 1,
    ):

      msg_name = (
        f"MRR_Detection_{ii:03d}"
      )

      try:
        msg = self.rcp.vl[
          msg_name
        ]

        detection_scan_index = int(
          msg[
            f"CAN_SCAN_INDEX_2LSB_{ii:02d}"
          ]
        ) & 0x3

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        continue

      # ----------------------------------------------------------------------
      # Synchronize using only the low two bits.
      # ----------------------------------------------------------------------

      if (
        detection_scan_index
        != scan_index_2lsb
      ):
        continue

      # ----------------------------------------------------------------------
      # Detection validity.
      # ----------------------------------------------------------------------

      try:
        valid = bool(
          msg[
            f"CAN_DET_VALID_LEVEL_{ii:02d}"
          ]
        )

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        continue

      if not valid:
        continue

      # ----------------------------------------------------------------------
      # Detection range.
      #
      # No old phase-dependent 30 m filter is applied.
      # ----------------------------------------------------------------------

      try:
        dist = float(
          msg[
            f"CAN_DET_RANGE_{ii:02d}"
          ]
        )

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        continue

      if not np.isfinite(dist):
        continue

      if dist <= 0:
        continue

      # ----------------------------------------------------------------------
      # Azimuth
      # ----------------------------------------------------------------------

      try:
        azimuth = float(
          msg[
            f"CAN_DET_AZIMUTH_{ii:02d}"
          ]
        )

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        continue

      if not np.isfinite(azimuth):
        continue

      # ----------------------------------------------------------------------
      # Relative range rate
      # ----------------------------------------------------------------------

      try:
        dist_rate = float(
          msg[
            f"CAN_DET_RANGE_RATE_{ii:02d}"
          ]
        )

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        continue

      if not np.isfinite(dist_rate):
        continue

      # ----------------------------------------------------------------------
      # Convert polar measurement to vehicle frame.
      #
      # Preserve the existing OEM MRR internal x2 representation.
      # It is normalized during clustering below.
      # ----------------------------------------------------------------------

      dRel = (
        cos(azimuth)
        * dist
      )

      yRel = (
        -sin(azimuth)
        * dist
      )

      self.points.append([
        dRel,
        yRel * 2,
        dist_rate * 2,
      ])

    # ========================================================================
    # OEM MRR clustering
    #
    # Existing behavior is retained:
    #
    # complete publish boundary occurs on scan index low bits == 3.
    #
    # This is NOT treated as a four-phase validation sequence.
    #
    # It is only the existing cycle boundary used to publish clustered OEM
    # points.
    # ========================================================================

    if scan_index_2lsb != 3:
      return True

    # ========================================================================
    # Cluster current points against previous OEM clusters.
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

    points_by_track_id = defaultdict(list)

    for idx, label in enumerate(labels):

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
    # Rebuild OEM clusters.
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
      # OEM RadarPoint only.
      #
      # This is the ONLY place RadarPoint is created by the MRR path.
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
    # Remove old OEM points.
    # ========================================================================

    for idx in range(
      len(points_by_track_id),
      len(self.pts),
    ):
      del self.pts[idx]

    self.points = []

    return True
