import numpy as np

from typing import cast
from collections import defaultdict
from dataclasses import dataclass
from math import cos, sin

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, RADAR
from opendbc.car.interfaces import RadarInterfaceBase


# =============================================================================
# Ford Delphi ESR
# =============================================================================

DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))


# =============================================================================
# Ford Delphi MRR
# =============================================================================

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


# =============================================================================
# Smartmicro MR76
# =============================================================================

MR76_DBC = "u_radar"

MR76_RADAR_STATE_ID = 0x201
MR76_STATUS_ID = 0x60A
MR76_OBJECT_ID = 0x60B

MR76_RADAR_STATE_RATE = 10
MR76_STATUS_RATE = 10
MR76_OBJECT_RATE = 50


# -----------------------------------------------------------------------------
# MR76 validity
# -----------------------------------------------------------------------------
#
# Do NOT impose the previous artificial:
#
#   MAX_DISTANCE = 150 m
#   MAX_ANGLE = 45 deg
#   MAX_VREL = 80 m/s
#
# here.
#
# The purpose of this interface is to expose valid MR76 detections to the
# normal RadarData path. Target selection belongs to radard.
#
MR76_MIN_DISTANCE = 0.05


# -----------------------------------------------------------------------------
# MR76 object classes
# -----------------------------------------------------------------------------

MR76_CLASS_POINT = 0
MR76_CLASS_VEHICLE = 1


# -----------------------------------------------------------------------------
# MR76 DynProp
# -----------------------------------------------------------------------------

MR76_DYNPROP_MOVING = 0
MR76_DYNPROP_STATIONARY = 1
MR76_DYNPROP_ONCOMING = 2
MR76_DYNPROP_CROSSING_LEFT = 3
MR76_DYNPROP_CROSSING_RIGHT = 4
MR76_DYNPROP_UNKNOWN = 5
MR76_DYNPROP_STOPPED = 6


# =============================================================================
# Internal Delphi cluster representation
# =============================================================================

@dataclass
class Cluster:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


# =============================================================================
# Delphi clustering helper
# =============================================================================

def cluster_points(
  pts_l: list[list[float]],
  pts2_l: list[list[float]],
  max_dist: float,
) -> list[int]:
  """
  Match a new collection of points against the previous cluster centers.

  This is retained for the original Delphi MRR path.

  MR76 does NOT use this function because MR76 already supplies an object ID.
  """

  if not pts2_l:
    return []

  if not pts_l:
    return [-1] * len(pts2_l)

  max_dist_sq = max_dist ** 2

  pts = np.array(pts_l)
  pts2 = np.array(pts2_l)

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


# =============================================================================
# Delphi ESR parser
# =============================================================================

def _create_delphi_esr_radar_can_parser(CP) -> CANParser:
  msg_n = len(DELPHI_ESR_RADAR_MSGS)

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


# =============================================================================
# Delphi MRR parser
# =============================================================================

def _create_delphi_mrr_radar_can_parser(CP) -> CANParser:
  messages = [
    ("MRR_Header_InformationDetections", 33),
    ("MRR_Header_SensorCoverage", 33),
  ]

  for i in range(
    1,
    DELPHI_MRR_RADAR_MSG_COUNT + 1,
  ):
    msg = f"MRR_Detection_{i:03d}"
    messages.append((msg, 33))

  return CANParser(
    RADAR.DELPHI_MRR,
    messages,
    CanBus(CP).radar,
  )


# =============================================================================
# MR76 parser
# =============================================================================

def _create_mr76_radar_can_parser(CP) -> CANParser:
  """
  Native Smartmicro MR76 parser.

  Messages:

    0x201  RadarState
    0x60A  Status
    0x60B  ObjectData

  0x60B is a repeated CAN message.

  IMPORTANT:

  One CAN ID does NOT equal one radar target.

  The actual target number is contained in:

      ObjectData.ID

  Therefore the parser must expose all received 0x60B frames through
  vl_all rather than relying on vl[0x60B].
  """

  messages = [
    (MR76_RADAR_STATE_ID, MR76_RADAR_STATE_RATE),
    (MR76_STATUS_ID, MR76_STATUS_RATE),
    (MR76_OBJECT_ID, MR76_OBJECT_RATE),
  ]

  return CANParser(
    MR76_DBC,
    messages,
    CanBus(CP).radar,
  )


# =============================================================================
# RadarInterface
# =============================================================================

class RadarInterface(RadarInterfaceBase):
  """
  Ford/Lincoln radar interface.

  Supported:

    - Delphi ESR
    - Delphi MRR
    - Smartmicro MR76

  Design principle:

      RadarInterface
          |
          +---- OEM Delphi Radar
          |
          +---- MR76
                  |
                  +---- multiple RadarPoint objects
                          |
                          v
                       radard
                          |
                          +---- lead selection
                          |
                          +---- multi-target tracking
                          |
                          +---- liveTracks

  MR76 is NOT selected as lead here.

  This file only converts radar detections into standard RadarData.
  """

  def __init__(self, CP):
    super().__init__(CP)

    # -------------------------------------------------------------------------
    # Common state
    # -------------------------------------------------------------------------

    self.points: list[list[float]] = []
    self.clusters: list[Cluster] = []

    self.updated_messages = set()

    self.track_id = 0

    self.radar = DBC[CP.carFingerprint].get(Bus.radar)

    self.scan_index_invalid_cnt = 0
    self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = 0

    # -------------------------------------------------------------------------
    # MR76 diagnostics
    # -------------------------------------------------------------------------

    self.mr76_object_count = 0
    self.mr76_valid_object_count = 0
    self.mr76_update_count = 0
    self.mr76_last_ids: set[int] = set()

    # -------------------------------------------------------------------------
    # Radar parser initialization
    # -------------------------------------------------------------------------

    if CP.radarUnavailable:
      self.rcp = None
      self.trigger_msg = None
      self.valid_cnt = {}

    elif self.radar == RADAR.DELPHI_ESR:
      self.rcp = _create_delphi_esr_radar_can_parser(CP)

      self.trigger_msg = DELPHI_ESR_RADAR_MSGS[-1]

      self.valid_cnt = {
        key: 0
        for key in DELPHI_ESR_RADAR_MSGS
      }

    elif self.radar == RADAR.DELPHI_MRR:
      self.rcp = _create_delphi_mrr_radar_can_parser(CP)

      self.trigger_msg = DELPHI_MRR_RADAR_HEADER_ADDR

      self.valid_cnt = {}

    elif self.radar == RADAR.MR76:
      self.rcp = _create_mr76_radar_can_parser(CP)

      self.trigger_msg = MR76_OBJECT_ID

      self.valid_cnt = {}

    else:
      raise ValueError(
        f"Unsupported radar: {self.radar}"
      )

  # ===========================================================================
  # Main update
  # ===========================================================================

  def update(self, can_strings):
    """
    Update radar data.

    Returns:
      RadarData
      None when a complete radar update is not yet available.
    """

    if self.rcp is None:
      return super().update(None)

    # -------------------------------------------------------------------------
    # CAN parser
    # -------------------------------------------------------------------------

    vls = self.rcp.update(can_strings)

    self.updated_messages.update(vls)

    # -------------------------------------------------------------------------
    # MR76
    # -------------------------------------------------------------------------

    if self.radar == RADAR.MR76:
      return self._update_mr76(vls)

    # -------------------------------------------------------------------------
    # Delphi
    # -------------------------------------------------------------------------

    if self.trigger_msg not in self.updated_messages:
      return None

    self.updated_messages.clear()

    ret = structs.RadarData()

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()

    elif self.radar == RADAR.DELPHI_MRR:
      updated = self._update_delphi_mrr(ret)

      if not updated:
        return None

    ret.points = list(self.pts.values())

    return ret

  # ===========================================================================
  # Delphi ESR
  # ===========================================================================

  def _update_delphi_esr(self):
    for ii in sorted(self.updated_messages):
      cpt = self.rcp.vl[ii]

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] = 0

      if cpt['X_Rel'] > 0.00001:
        self.valid_cnt[ii] += 1
      else:
        self.valid_cnt[ii] = max(
          self.valid_cnt[ii] - 1,
          0,
        )

      if self.valid_cnt[ii] > 0:

        if ii not in self.pts:
          self.pts[ii] = structs.RadarData.RadarPoint()

          self.pts[ii].trackId = self.track_id

          self.track_id += 1

        self.pts[ii].dRel = cpt['X_Rel']

        self.pts[ii].yRel = (
          cpt['X_Rel']
          * cpt['Angle']
          * CV.DEG_TO_RAD
        )

        self.pts[ii].vRel = cpt['V_Rel']

        self.pts[ii].aRel = float('nan')

        self.pts[ii].yvRel = float('nan')

        self.pts[ii].measured = True

      else:

        if ii in self.pts:
          del self.pts[ii]

  # ===========================================================================
  # Delphi MRR
  # ===========================================================================

  def _update_delphi_mrr(
    self,
    ret: structs.RadarData,
  ):
    headerScanIndex = int(
      self.rcp.vl[
        "MRR_Header_InformationDetections"
      ]['CAN_SCAN_INDEX']
    ) & 0b11

    # -------------------------------------------------------------------------
    # Radar scan sequence
    # -------------------------------------------------------------------------

    if (
      (self.prev_headerScanIndex + 1) % 4
      != headerScanIndex
    ):
      self.radar_unavailable_cnt += 1
    else:
      self.radar_unavailable_cnt = 0

    self.prev_headerScanIndex = headerScanIndex

    # -------------------------------------------------------------------------
    # Radar unavailable
    # -------------------------------------------------------------------------

    if self.radar_unavailable_cnt >= 5:

      self.pts.clear()
      self.points.clear()
      self.clusters.clear()

      ret.errors.radarUnavailableTemporary = True

      return True

    if headerScanIndex not in (2, 3):
      return False

    # -------------------------------------------------------------------------
    # Sensor coverage
    # -------------------------------------------------------------------------

    if (
      DELPHI_MRR_RADAR_RANGE_COVERAGE[headerScanIndex]
      != int(
        self.rcp.vl[
          "MRR_Header_SensorCoverage"
        ]["CAN_RANGE_COVERAGE"]
      )
    ):
      self.scan_index_invalid_cnt += 1
    else:
      self.scan_index_invalid_cnt = 0

    if self.scan_index_invalid_cnt >= 5:
      ret.errors.wrongConfig = True

    # -------------------------------------------------------------------------
    # Read detections
    # -------------------------------------------------------------------------

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

      if (
        scanIndex in (1, 3)
        and dist < DELPHI_MRR_MIN_LONG_RANGE_DIST
      ):
        valid = False

      if not valid:
        continue

      azimuth = msg[
        f"CAN_DET_AZIMUTH_{ii:02d}"
      ]

      distRate = msg[
        f"CAN_DET_RANGE_RATE_{ii:02d}"
      ]

      dRel = cos(azimuth) * dist
      yRel = -sin(azimuth) * dist

      self.points.append([
        dRel,
        yRel * 2,
        distRate * 2,
      ])

    # -------------------------------------------------------------------------
    # MRR scan is complete
    # -------------------------------------------------------------------------

    if headerScanIndex != 3:
      return False

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

    self.clusters = []

    for idx, (track_id, pts) in enumerate(
      points_by_track_id.items()
    ):
      dRel_values = [
        p[0]
        for p in pts
      ]

      min_dRel = min(dRel_values)

      dRel = sum(dRel_values) / len(dRel_values)

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

      if idx not in self.pts:
        self.pts[idx] = (
          structs.RadarData.RadarPoint(
            measured=True,
            aRel=float('nan'),
            yvRel=float('nan'),
          )
        )

      self.pts[idx].dRel = min_dRel
      self.pts[idx].yRel = yRel
      self.pts[idx].vRel = vRel
      self.pts[idx].trackId = track_id

    for idx in range(
      len(points_by_track_id),
      len(self.pts),
    ):
      del self.pts[idx]

    self.points = []

    return True

  # ===========================================================================
  # MR76
  # ===========================================================================

  def _update_mr76(
    self,
    updated_messages,
  ):
    """
    Convert every MR76 ObjectData frame into a normal RadarPoint.

    IMPORTANT:

    0x60B is a repeated CAN message.

    It is NOT:

        one CAN ID = one target

    Instead:

        0x60B
          |
          +-- ID 1
          +-- ID 2
          +-- ID 3
          +-- ID 4
          +-- ...

    Therefore the implementation must use vl_all.

    No lead target selection is performed here.
    """

    if MR76_OBJECT_ID not in updated_messages:
      return None

    self.mr76_update_count += 1

    ret = structs.RadarData()

    # -------------------------------------------------------------------------
    # CAN validity
    # -------------------------------------------------------------------------

    if not self.rcp.can_valid:
      ret.errors.canError = True

    # -------------------------------------------------------------------------
    # Get all 0x60B frames
    # -------------------------------------------------------------------------

    vl_all = getattr(
      self.rcp,
      "vl_all",
      {},
    )

    object_frames = vl_all.get(
      MR76_OBJECT_ID,
      {},
    )

    if not object_frames:
      return None

    # -------------------------------------------------------------------------
    # Extract arrays
    # -------------------------------------------------------------------------

    ids = object_frames.get(
      "ID",
      [],
    )

    dist_long = object_frames.get(
      "DistLong",
      [],
    )

    dist_lat = object_frames.get(
      "DistLat",
      [],
    )

    v_rel_long = object_frames.get(
      "VRelLong",
      [],
    )

    v_rel_lat = object_frames.get(
      "VRelLat",
      [],
    )

    # These fields are deliberately retained for diagnostics and future
    # filtering, but are NOT used for target suppression here.
    dyn_prop = object_frames.get(
      "DynProp",
      [],
    )

    object_class = object_frames.get(
      "Class",
      [],
    )

    rcs = object_frames.get(
      "RCS",
      [],
    )

    # -------------------------------------------------------------------------
    # Defensive frame count
    # -------------------------------------------------------------------------

    arrays = [
      ids,
      dist_long,
      dist_lat,
      v_rel_long,
      v_rel_lat,
    ]

    if any(
      len(array) == 0
      for array in arrays
    ):
      return None

    frame_count = min(
      len(ids),
      len(dist_long),
      len(dist_lat),
      len(v_rel_long),
      len(v_rel_lat),
    )

    if frame_count <= 0:
      return None

    self.mr76_object_count = frame_count

    # -------------------------------------------------------------------------
    # Build new target set
    # -------------------------------------------------------------------------

    new_pts = {}

    current_ids: set[int] = set()

    for idx in range(frame_count):

      # -----------------------------------------------------------------------
      # Target ID
      # -----------------------------------------------------------------------

      try:
        track_id = int(ids[idx])
      except (
        TypeError,
        ValueError,
      ):
        continue

      if track_id < 0:
        continue

      if track_id > 255:
        continue

      # -----------------------------------------------------------------------
      # Radar values
      # -----------------------------------------------------------------------

      try:
        d_rel = float(
          dist_long[idx]
        )

        y_rel = float(
          dist_lat[idx]
        )

        v_rel = float(
          v_rel_long[idx]
        )

        yv_rel = float(
          v_rel_lat[idx]
        )

      except (
        TypeError,
        ValueError,
      ):
        continue

      # -----------------------------------------------------------------------
      # Validity
      # -----------------------------------------------------------------------

      if not np.isfinite(d_rel):
        continue

      if not np.isfinite(y_rel):
        continue

      if not np.isfinite(v_rel):
        continue

      if d_rel <= MR76_MIN_DISTANCE:
        continue

      # -----------------------------------------------------------------------
      # Duplicate ID protection
      # -----------------------------------------------------------------------
      #
      # Normally a target ID occurs only once in a scan.
      #
      # If malformed CAN traffic produces the same ID twice, keep the most
      # recent frame instead of creating duplicate track objects.
      #

      point = structs.RadarData.RadarPoint(
        measured=True,
        aRel=float('nan'),
        yvRel=(
          yv_rel
          if np.isfinite(yv_rel)
          else float('nan')
        ),
      )

      point.trackId = track_id
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.measured = True

      new_pts[track_id] = point

      current_ids.add(track_id)

      # -----------------------------------------------------------------------
      # Optional metadata.
      #
      # Do not filter using these values.
      # -----------------------------------------------------------------------

      if idx < len(dyn_prop):
        _dyn = dyn_prop[idx]
      else:
        _dyn = None

      if idx < len(object_class):
        _class = object_class[idx]
      else:
        _class = None

      if idx < len(rcs):
        _rcs = rcs[idx]
      else:
        _rcs = None

      # Explicitly consume variables to document that they are intentionally
      # not used for target suppression.
      del _dyn
      del _class
      del _rcs

    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------

    self.mr76_valid_object_count = len(
      new_pts
    )

    self.mr76_last_ids = current_ids

    # -------------------------------------------------------------------------
    # Replace current MR76 target set
    # -------------------------------------------------------------------------

    self.pts.clear()

    self.pts.update(
      new_pts
    )

    # -------------------------------------------------------------------------
    # Publish ALL MR76 targets
    # -------------------------------------------------------------------------

    ret.points = list(
      self.pts.values()
    )

    return ret