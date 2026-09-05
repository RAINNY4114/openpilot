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
# MR76
# ============================================================================
#
# Smartmicro MR76 raw CAN.
#
# Bus: 1
#
# 0x201 RadarState
# 0x60A Status
# 0x60B ObjectData
#
# No u_radar.dbc is required.
#

MR76_BUS = 1

MR76_RADAR_STATE_ID = 0x201
MR76_STATUS_ID = 0x60A
MR76_OBJECT_ID = 0x60B


@dataclass
class Cluster:
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  trackId: int = 0


@dataclass
class MR76Object:
  obj_id: int = 0
  dRel: float = 0.0
  yRel: float = 0.0
  vRel: float = 0.0
  vLat: float = 0.0
  dyn_prop: int = 0
  obj_class: int = 0
  rcs: float = 0.0


def _get_mr76_signal(data: bytes, start_bit: int, length: int) -> int:
  """
  Decode a Motorola/big-endian DBC signal.

  The MR76 DBC uses @0 for the ObjectData signals.
  start_bit is the DBC MSB position.
  """

  value = 0

  bit = start_bit

  for _ in range(length):
    byte_index = bit // 8
    bit_index = bit % 8

    value = (value << 1) | ((data[byte_index] >> bit_index) & 1)

    if bit_index == 0:
      bit = (byte_index + 1) * 8 + 7
    else:
      bit -= 1

  return value


def _mr76_u8(data: bytes, start_bit: int, length: int) -> int:
  return _get_mr76_signal(data, start_bit, length)


def _mr76_object(data: bytes) -> MR76Object:
  """
  Decode MR76 ObjectData (0x60B).

  DBC:

    ID        : 7|8@0+
    DistLong  : 15|13@0+ (0.2,-500)
    DistLat   : 18|11@0+ (0.2,-204.6)
    VRelLong  : 39|10@0+ (0.25,-128)
    VRelLat   : 45|9@0+  (0.25,-64)
    DynProp   : 50|3@0+
    Class     : 52|2@0+
    RCS       : 63|8@0+ (0.5,-64)
  """

  return MR76Object(
    obj_id=_mr76_u8(data, 7, 8),

    dRel=(
      _mr76_u8(data, 15, 13) * 0.2
      - 500.0
    ),

    yRel=(
      _mr76_u8(data, 18, 11) * 0.2
      - 204.6
    ),

    vRel=(
      _mr76_u8(data, 39, 10) * 0.25
      - 128.0
    ),

    vLat=(
      _mr76_u8(data, 45, 9) * 0.25
      - 64.0
    ),

    dyn_prop=_mr76_u8(data, 50, 3),

    obj_class=_mr76_u8(data, 52, 2),

    rcs=(
      _mr76_u8(data, 63, 8) * 0.5
      - 64.0
    ),
  )


def cluster_points(
  pts_l: list[list[float]],
  pts2_l: list[list[float]],
  max_dist: float,
) -> list[int]:
  """
  Clusters a collection of points based on another collection of points.
  """

  if not len(pts2_l):
    return []

  if not len(pts_l):
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

  for i in range(1, DELPHI_MRR_RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    messages += [(msg, 33)]

  return CANParser(
    RADAR.DELPHI_MRR,
    messages,
    CanBus(CP).radar,
  )


class RadarInterface(RadarInterfaceBase):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    self.points: dict[int, structs.RadarData.RadarPoint] = {}
    self.clusters: list[Cluster] = []

    self.updated_messages = set()

    self.radar = DBC[CP.carFingerprint].get(Bus.radar)

    self.scan_index_invalid_cnt = 0
    self.radar_unavailable_cnt = 0
    self.prev_headerScanIndex = 0

    self.track_id = 0

    # MR76 state
    self.mr76_objects: dict[int, MR76Object] = {}
    self.mr76_object_count = 0
    self.mr76_meas_count = 0

    if CP.radarUnavailable:
      self.rcp = None

    elif self.radar == RADAR.MR76:
      # MR76 does not use CANParser.
      #
      # Raw CAN frames are decoded directly from can_strings.
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


  def update(self, can_strings):

    if self.radar == RADAR.MR76:
      return self._update_mr76(can_strings)

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

    ret.points = list(self.points.values())

    return ret


  # ==========================================================================
  # MR76 raw CAN decoder
  # ==========================================================================

  def _update_mr76(self, can_strings):

    ret = structs.RadarData()

    got_object = False

    for can in can_strings:

      # can is normally:
      #   (address, data, src)
      #
      # Ignore all buses except MR76 bus 1.

      try:
        address = can[0]
        data = can[1]
        src = can[2]
      except (IndexError, TypeError):
        continue

      if src != MR76_BUS:
        continue

      if not isinstance(data, bytes):
        data = bytes(data)

      if len(data) < 8:
        continue

      # --------------------------------------------------------------
      # RadarState 0x201
      # --------------------------------------------------------------

      if address == MR76_RADAR_STATE_ID:
        # Currently no control/configuration is required.
        #
        # Decode only the fields useful for diagnostics if needed.
        continue

      # --------------------------------------------------------------
      # Status 0x60A
      # --------------------------------------------------------------

      if address == MR76_STATUS_ID:

        self.mr76_object_count = _mr76_u8(
          data,
          7,
          8,
        )

        self.mr76_meas_count = _mr76_u8(
          data,
          15,
          16,
        )

        continue

      # --------------------------------------------------------------
      # ObjectData 0x60B
      # --------------------------------------------------------------

      if address != MR76_OBJECT_ID:
        continue

      obj = _mr76_object(data)

      # ID 0 is not a useful tracked object.
      if obj.obj_id == 0:
        continue

      # Invalid/out-of-range object.
      if obj.dRel < 0.0:
        continue

      # Keep the latest object by MR76 object ID.
      self.mr76_objects[obj.obj_id] = obj

      got_object = True

    if not got_object and not self.mr76_objects:
      return None

    # ------------------------------------------------------------------------
    # Convert MR76 objects to native RadarPoint objects.
    #
    # No lead selection is performed here.
    # All valid MR76 objects are returned.
    # ------------------------------------------------------------------------

    active_ids = set()

    for obj_id, obj in self.mr76_objects.items():

      active_ids.add(obj_id)

      if obj_id not in self.points:
        self.points[obj_id] = (
          structs.RadarData.RadarPoint()
        )

        self.points[obj_id].trackId = self.track_id
        self.track_id += 1

      point = self.points[obj_id]

      point.dRel = obj.dRel
      point.yRel = obj.yRel
      point.vRel = obj.vRel

    # Remove objects that have disappeared from the latest scan.
    #
    # ObjectData is streamed repeatedly, so keeping stale objects forever
    # would result in phantom targets.
    for obj_id in list(self.points.keys()):
      if obj_id not in active_ids:
        del self.points[obj_id]

    # Clear the object cache after publishing the current cycle.
    self.mr76_objects.clear()

    ret.points = list(self.points.values())

    return ret


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

        if ii not in self.points:
          self.points[ii] = (
            structs.RadarData.RadarPoint()
          )

          self.points[ii].trackId = self.track_id
          self.track_id += 1

        self.points[ii].dRel = cpt['X_Rel']

        self.points[ii].yRel = (
          cpt['X_Rel']
          * cpt['Angle']
          * CV.DEG_TO_RAD
        )

        self.points[ii].vRel = cpt['V_Rel']

      else:

        if ii in self.points:
          del self.points[ii]


  def _update_delphi_mrr(
    self,
    ret: structs.RadarData,
  ):

    headerScanIndex = int(
      self.rcp.vl[
        "MRR_Header_InformationDetections"
      ]['CAN_SCAN_INDEX']
    ) & 0b11

    if (
      self.prev_headerScanIndex + 1
    ) % 4 != headerScanIndex:

      self.radar_unavailable_cnt += 1

    else:
      self.radar_unavailable_cnt = 0

    self.prev_headerScanIndex = headerScanIndex

    if self.radar_unavailable_cnt >= 5:

      self.points.clear()
      self.clusters.clear()

      ret.errors.radarUnavailableTemporary = True

      return True

    if headerScanIndex not in (2, 3):
      return False

    if (
      DELPHI_MRR_RADAR_RANGE_COVERAGE[headerScanIndex]
      != int(
        self.rcp.vl[
          "MRR_Header_SensorCoverage"
        ]['CAN_RANGE_COVERAGE']
      )
    ):

      self.scan_index_invalid_cnt += 1

    else:
      self.scan_index_invalid_cnt = 0

    if self.scan_index_invalid_cnt >= 5:
      ret.errors.wrongConfig = True

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

      if valid:

        azimuth = msg[
          f"CAN_DET_AZIMUTH_{ii:02d}"
        ]

        distRate = msg[
          f"CAN_DET_RANGE_RATE_{ii:02d}"
        ]

        dRel = cos(azimuth) * dist
        yRel = -sin(azimuth) * dist

        self.mr76_objects.setdefault(
          ii,
          MR76Object(),
        )

        # Keep original MRR behavior through native points.
        self.mr76_objects[ii].dRel = dRel
        self.mr76_objects[ii].yRel = yRel * 2
        self.mr76_objects[ii].vRel = distRate * 2

    return True
