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


DELPHI_ESR_RADAR_MSGS = list(range(0x500, 0x540))

DELPHI_MRR_RADAR_START_ADDR = 0x120
DELPHI_MRR_RADAR_HEADER_ADDR = 0x174  # MRR_Header_SensorCoverage
DELPHI_MRR_RADAR_MSG_COUNT = 64

DELPHI_MRR_RADAR_RANGE_COVERAGE = {
  0: 42,
  1: 164,
  2: 45,
  3: 175,
}  # scan index to detection range (m)

DELPHI_MRR_MIN_LONG_RANGE_DIST = 30  # meters
DELPHI_MRR_CLUSTER_THRESHOLD = 5  # meters, lateral distance and relative velocity are weighted


# ============================================================================
# Smartmicro MR76 auxiliary radar
#
# MR76 DBC:
#   u_radar.dbc
#
# CAN messages:
#
#   0x201 = RadarState
#       Configuration / NVM / interface status
#
#   0x60A = Status
#       NoOfObjects
#       MeasCount
#       InterfaceVersion
#
#   0x60B = ObjectData
#       ID
#       DistLong
#       DistLat
#       VRelLong
#       VRelLat
#       DynProp
#       Class
#       RCS
#
# IMPORTANT:
#   MR76 is AUXILIARY ONLY.
#
#   It does NOT replace the OEM Delphi radar.
#   It does NOT enter self.pts.
#   It does NOT enter ret.points.
#   It does NOT become an OEM lead target.
#   It does NOT affect OEM radar availability.
#   It does NOT directly cause braking or deceleration.
# ============================================================================

MR76_RADAR_DBC = "u_radar"

MR76_STATUS_MSG = "Status"
MR76_OBJECT_DATA_MSG = "ObjectData"

MR76_STATUS_CAN_ID = 0x60A
MR76_OBJECT_DATA_CAN_ID = 0x60B

# The DBC itself defines the physical ranges.
# These limits are only sanity filters for the auxiliary cache.
MR76_MIN_DISTANCE = 0.5
MR76_MAX_DISTANCE = 150.0
MR76_MAX_LATERAL_DISTANCE = 204.8
MR76_MAX_RELATIVE_SPEED = 128.0


@dataclass
class Cluster:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


@dataclass
class MR76Point:
  """
  Auxiliary MR76 target.

  This is deliberately kept separate from
  structs.RadarData.RadarPoint.

  MR76Point must never be inserted into:
    - self.pts
    - ret.points

  Therefore MR76 cannot accidentally become an OEM
  radar target or OEM lead target.
  """
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  vLat: float = 0.0
  trackId: int = 0
  dynProp: int = 5
  objClass: int = 0
  rcs: float = 0.0


def cluster_points(
  pts_l: list[list[float]],
  pts2_l: list[list[float]],
  max_dist: float,
) -> list[int]:
  """
  Clusters a collection of points based on another collection of points.
  This is useful for correlating clusters through time.

  Points in pts2 not close enough to any point in pts are assigned -1.

  Args:
    pts_l: List of points to base the new clusters on
    pts2_l: List of points to cluster using pts
    max_dist: Max distance from cluster center to candidate point

  Returns:
    List of cluster indices for pts2 that correspond to pts
  """

  if not len(pts2_l):
    return []

  if not len(pts_l):
    return [-1] * len(pts2_l)

  max_dist_sq = max_dist ** 2

  pts = np.array(pts_l)
  pts2 = np.array(pts2_l)

  # Compute squared norms
  pts_norm_sq = np.sum(pts ** 2, axis=1)
  pts2_norm_sq = np.sum(pts2 ** 2, axis=1)

  # Compute squared Euclidean distances using the identity:
  #
  # dist_sq[i, j] =
  #   ||pts2[i]||^2 +
  #   ||pts[j]||^2 -
  #   2 * pts2[i] . pts[j]
  #
  dist_sq = (
    pts2_norm_sq[:, np.newaxis]
    + pts_norm_sq[np.newaxis, :]
    - 2 * np.dot(pts2, pts.T)
  )

  dist_sq = np.maximum(dist_sq, 0.0)

  # Find the closest cluster for each point and assign its index
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
    messages += [(msg, 33)]

  return CANParser(
    RADAR.DELPHI_MRR,
    messages,
    CanBus(CP).radar,
  )


def _create_mr76_radar_can_parser(CP) -> CANParser:
  """
  Create the independent Smartmicro MR76 CAN parser.

  MR76 uses u_radar.dbc.

  Actual DBC definitions:

    BO_ 1546 Status: 8 RADAR
    BO_ 1547 ObjectData: 8 RADAR

  Therefore:

    0x60A -> Status
    0x60B -> ObjectData

  Note:
    0x201 -> RadarState is a configuration/status frame and is
    intentionally NOT used for object detection here.
  """

  messages = [
    # 0x60A
    ("Status", 20),

    # 0x60B
    ("ObjectData", 20),
  ]

  return CANParser(
    MR76_RADAR_DBC,
    messages,
    CanBus(CP).radar,
  )


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    self.points: list[list[float]] = []
    self.clusters: list[Cluster] = []

    # ------------------------------------------------------------------------
    # MR76 auxiliary radar cache.
    #
    # IMPORTANT:
    #
    # These points are deliberately NOT self.pts.
    #
    # self.pts is reserved for the OEM Delphi radar.
    # ------------------------------------------------------------------------
    self.mr76_points: dict[int, MR76Point] = {}

    self.mr76_object_count = 0
    self.mr76_measurement_count = 0
    self.mr76_interface_version = 0

    self.mr76_updated = False

    self.updated_messages = set()

    # ------------------------------------------------------------------------
    # OEM radar selection remains unchanged.
    #
    # Ford/Lincoln continues to use Delphi ESR/MRR as the primary radar.
    # ------------------------------------------------------------------------
    self.radar = DBC[CP.carFingerprint].get(Bus.radar)

    self.scan_index_invalid_cnt = 0
    self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = 0

    # ------------------------------------------------------------------------
    # OEM radar parser
    # ------------------------------------------------------------------------
    self.mr76_rcp = None

    if CP.radarUnavailable:
      self.rcp = None

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

    else:
      raise ValueError(
        f"Unsupported radar: {self.radar}"
      )

    # ------------------------------------------------------------------------
    # MR76 auxiliary parser.
    #
    # MR76 is optional.
    #
    # If u_radar.dbc is unavailable or cannot be loaded, the OEM Delphi
    # radar MUST continue operating normally.
    # ------------------------------------------------------------------------
    try:
      self.mr76_rcp = _create_mr76_radar_can_parser(CP)

    except (
      KeyError,
      ValueError,
      FileNotFoundError,
    ):
      self.mr76_rcp = None

  def update(self, can_strings):
    # ------------------------------------------------------------------------
    # MR76 auxiliary radar
    #
    # This is intentionally performed independently from the OEM radar.
    #
    # Failure or malformed MR76 data must never affect:
    #
    #   - OEM radar availability
    #   - OEM radar canError
    #   - OEM radar points
    #   - ret.points
    # ------------------------------------------------------------------------
    self._update_mr76(can_strings)

    # ------------------------------------------------------------------------
    # Original OEM radar path
    # ------------------------------------------------------------------------
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)

    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    self.updated_messages.clear()

    ret = structs.RadarData()

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.radar == RADAR.DELPHI_ESR:
      self._update_delphi_esr()

    elif self.radar == RADAR.DELPHI_MRR:
      _update = self._update_delphi_mrr(ret)

      if not _update:
        return None

    # ------------------------------------------------------------------------
    # ONLY OEM Delphi radar points are published.
    #
    # MR76 NEVER enters ret.points.
    # ------------------------------------------------------------------------
    ret.points = list(
      self.pts.values()
    )

    return ret

  # ==========================================================================
  # MR76 auxiliary radar
  # ==========================================================================

  def _update_mr76(self, can_strings):
    """
    Update MR76 auxiliary target cache.

    This function intentionally does NOT touch:

      - self.pts
      - self.points
      - self.clusters
      - ret.points
      - radarUnavailableTemporary
      - OEM radar errors

    Therefore:

      bad MR76 frame
      missing MR76 frame
      single MR76 frame
      malformed MR76 frame

    cannot directly modify the OEM radar output.
    """

    if self.mr76_rcp is None:
      return

    try:
      vls = self.mr76_rcp.update(can_strings)

    except (
      KeyError,
      ValueError,
      IndexError,
      TypeError,
    ):
      return

    if not vls:
      return

    self.mr76_updated = True

    # ------------------------------------------------------------------------
    # 0x60A Status
    #
    # DBC:
    #
    # BO_ 1546 Status: 8 RADAR
    #
    #   NoOfObjects
    #   MeasCount
    #   InterfaceVersion
    # ------------------------------------------------------------------------
    if MR76_STATUS_MSG in vls:
      try:
        state = self.mr76_rcp.vl[
          MR76_STATUS_MSG
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

      except (
        KeyError,
        TypeError,
        ValueError,
      ):
        # Status corruption must not affect OEM radar.
        pass

    # ------------------------------------------------------------------------
    # 0x60B ObjectData
    #
    # DBC:
    #
    #   ID
    #   DistLong
    #   DistLat
    #   VRelLong
    #   VRelLat
    #   DynProp
    #   Class
    #   RCS
    # ------------------------------------------------------------------------
    if MR76_OBJECT_DATA_MSG not in vls:
      return

    try:
      msg = self.mr76_rcp.vl[
        MR76_OBJECT_DATA_MSG
      ]

      track_id = int(
        msg["ID"]
      )

      dRel = float(
        msg["DistLong"]
      )

      yRel = float(
        msg["DistLat"]
      )

      vRel = float(
        msg["VRelLong"]
      )

      vLat = float(
        msg["VRelLat"]
      )

      dynProp = int(
        msg["DynProp"]
      )

      objClass = int(
        msg["Class"]
      )

      rcs = float(
        msg["RCS"]
      )

    except (
      KeyError,
      TypeError,
      ValueError,
    ):
      return

    # ------------------------------------------------------------------------
    # Basic sanity filtering.
    #
    # This is ONLY data validation.
    #
    # It does NOT make any driving/control decision.
    # ------------------------------------------------------------------------

    if track_id < 0 or track_id > 255:
      return

    if not np.isfinite(dRel):
      return

    if not np.isfinite(yRel):
      return

    if not np.isfinite(vRel):
      return

    if not np.isfinite(vLat):
      return

    if not np.isfinite(rcs):
      return

    if dRel < MR76_MIN_DISTANCE:
      return

    if dRel > MR76_MAX_DISTANCE:
      return

    if abs(yRel) > MR76_MAX_LATERAL_DISTANCE:
      return

    if abs(vRel) > MR76_MAX_RELATIVE_SPEED:
      return

    # ------------------------------------------------------------------------
    # DBC-defined enumeration sanity.
    #
    # DynProp:
    #
    #   0 moving
    #   1 stationary
    #   2 oncoming
    #   3 crossing_left
    #   4 crossing_right
    #   5 unknown
    #   6 stopped
    #
    # Class:
    #
    #   0 point
    #   1 vehicle
    # ------------------------------------------------------------------------
    if dynProp < 0 or dynProp > 7:
      return

    if objClass < 0 or objClass > 3:
      return

    # ------------------------------------------------------------------------
    # Store latest valid MR76 object.
    #
    # IMPORTANT:
    #
    # This remains a private auxiliary cache.
    #
    # DO NOT convert this into:
    #
    #   structs.RadarData.RadarPoint
    #
    # DO NOT insert it into:
    #
    #   self.pts
    #
    # DO NOT insert it into:
    #
    #   ret.points
    # ------------------------------------------------------------------------
    self.mr76_points[track_id] = MR76Point(
      dRel=dRel,
      yRel=yRel,
      vRel=vRel,
      vLat=vLat,
      trackId=track_id,
      dynProp=dynProp,
      objClass=objClass,
      rcs=rcs,
    )

  # ==========================================================================
  # Original Delphi ESR implementation
  # ==========================================================================

  def _update_delphi_esr(self):
    for ii in sorted(
      self.updated_messages
    ):
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

      # Radar point only valid if there have been enough valid measurements
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

        # In car frame's y axis, left is positive.
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
  # Original Delphi MRR implementation
  # ==========================================================================

  def _update_delphi_mrr(
    self,
    ret: structs.RadarData,
  ):
    headerScanIndex = int(
      self.rcp.vl[
        "MRR_Header_InformationDetections"
      ]['CAN_SCAN_INDEX']
    ) & 0b11

    # In reverse, the radar continually sends the last messages.
    # Mark this as invalid.
    if (
      (self.prev_headerScanIndex + 1) % 4
      != headerScanIndex
    ):
      self.radar_unavailable_cnt += 1

    else:
      self.radar_unavailable_cnt = 0

    self.prev_headerScanIndex = (
      headerScanIndex
    )

    if self.radar_unavailable_cnt >= 5:
      self.pts.clear()
      self.points.clear()
      self.clusters.clear()

      ret.errors.radarUnavailableTemporary = True

      return True

    # Use points with Doppler coverage of +-60 m/s
    if headerScanIndex not in (2, 3):
      return False

    if (
      DELPHI_MRR_RADAR_RANGE_COVERAGE[
        headerScanIndex
      ]
      != int(
        self.rcp.vl[
          "MRR_Header_SensorCoverage"
        ]["CAN_RANGE_COVERAGE"]
      )
    ):
      self.scan_index_invalid_cnt += 1

    else:
      self.scan_index_invalid_cnt = 0

    # Rarely MRR_Header_InformationDetections can fail to send a message.
    # The scan index is skipped in this case.
    if self.scan_index_invalid_cnt >= 5:
      ret.errors.wrongConfig = True

    for ii in range(
      1,
      DELPHI_MRR_RADAR_MSG_COUNT + 1,
    ):
      msg = self.rcp.vl[
        f"MRR_Detection_{ii:03d}"
      ]

      # SCAN_INDEX rotates through 0..3 on each message
      # for different measurement modes.
      scanIndex = msg[
        f"CAN_SCAN_INDEX_2LSB_{ii:02d}"
      ]

      # Throw out old measurements.
      if scanIndex != headerScanIndex:
        continue

      valid = bool(
        msg[
          f"CAN_DET_VALID_LEVEL_{ii:02d}"
        ]
      )

      # Long range measurement mode is more sensitive
      # and can detect the road surface.
      dist = msg[
        f"CAN_DET_RANGE_{ii:02d}"
      ]

      if (
        scanIndex in (1, 3)
        and dist < DELPHI_MRR_MIN_LONG_RANGE_DIST
      ):
        valid = False

      if valid:
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

        self.points.append([
          dRel,
          yRel * 2,
          distRate * 2,
        ])

    # ------------------------------------------------------------------------
    # Cluster and publish using stored points once we've cycled
    # through all 4 scan modes.
    # ------------------------------------------------------------------------
    if headerScanIndex != 3:
      return False

    # Cluster points from this cycle against the centroids
    # from the previous cycle.
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

    self.clusters = []

    for idx, (
      track_id,
      pts,
    ) in enumerate(
      points_by_track_id.items()
    ):
      dRel = [
        p[0]
        for p in pts
      ]

      min_dRel = min(dRel)

      dRel = (
        sum(dRel)
        / len(dRel)
      )

      yRel = [
        p[1]
        for p in pts
      ]

      yRel = (
        sum(yRel)
        / len(yRel)
        / 2
      )

      vRel = [
        p[2]
        for p in pts
      ]

      vRel = (
        sum(vRel)
        / len(vRel)
        / 2
      )

      # FIXME: creating capnp RadarPoint and accessing attributes
      # are both expensive, so we store a dataclass and reuse
      # the RadarPoint.
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
          structs.RadarData.RadarPoint()
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
