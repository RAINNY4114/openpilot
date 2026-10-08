#!/usr/bin/env python3

import math
import numpy as np

from collections import deque
from typing import Any

import capnp

from cereal import messaging, log, car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import (
  DT_MDL,
  Priority,
  config_realtime_process,
)
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# ============================================================================
# Original DragonPilot RadarD parameters
# ============================================================================

_LEAD_ACCEL_TAU = 1.5

SPEED, ACCEL = 0, 1

V_EGO_STATIONARY = 4.0

RADAR_TO_CENTER = 2.7
RADAR_TO_CAMERA = 1.52

LEAD_HOLD_TIME_S = 0.4
LEAD_HOLD_MAX_SPEED = 12.0


# ============================================================================
# MR76 auxiliary radar
#
# IMPORTANT:
#
# MR76 is auxiliary only.
#
# It NEVER:
#   - replaces OEM Ford/Delphi radar
#   - enters self.tracks
#   - becomes leadOne
#   - becomes leadTwo
#   - directly controls longitudinal acceleration
#   - modifies OEM radar points
#
# MR76 is intended for:
#   - auxiliary obstacle detection
#   - side object detection
#   - AutoAvoidance
#   - AutoOvertake
#   - future sensor fusion
#
# ============================================================================

MR76_ENABLE = True

# User hardware configuration:
# MR76 is connected to C3X CAN bus 1.
MR76_BUS = 1

# MR76 object protocol
MR76_STATUS_ID = 0x60A
MR76_OBJECT_ID = 0x60B


# ============================================================================
# MR76 safety limits
# ============================================================================

MR76_MIN_DISTANCE = 2.0
MR76_MAX_DISTANCE = 150.0

MR76_MAX_ANGLE_DEG = 45.0

# Minimum consecutive cycles before a target is considered confirmed.
MR76_MIN_CONFIRM_COUNT = 3

# --------------------------------------------------------------------------
# IMPORTANT:
#
# Previously this was 20.
#
# User requested:
#
#   "限制框选的目标不要超过5个"
#
# Therefore the auxiliary MR76 target list is limited to 5 targets.
#
# The closest valid targets are retained.
# --------------------------------------------------------------------------

MR76_MAX_TARGETS = 5

# Explicit display/output limit.
#
# Keep this separate so future code can retain more internal tracks if
# necessary without accidentally increasing the number of displayed targets.
MR76_MAX_DISPLAY_TARGETS = 5

# MR76 must NOT become lead by default.
MR76_LEAD_ENABLE = False

# Debug logging.
MR76_DEBUG = False

# Remove a target after this many consecutive missing cycles.
MR76_MISSING_CYCLES = 3


# ============================================================================
# MR76 CAN decoding
# ============================================================================

def _mr76_get_bits(
  data: bytes,
  start: int,
  length: int,
) -> int:
  """
  Extract a Motorola / big-endian CAN signal.

  DBC-style bit numbering:
    bit 0 = MSB of byte 0

  This helper intentionally operates on the first 8 bytes only.
  """

  if not data or len(data) < 8:
    return 0

  bits = ''.join(
    f'{byte:08b}'
    for byte in data[:8]
  )

  if start < 0:
    return 0

  if length <= 0:
    return 0

  if start + length > len(bits):
    return 0

  return int(
    bits[start:start + length],
    2,
  )


def _mr76_decode_60a(
  data: bytes,
) -> int:
  """
  MR76 0x60A Object List Status.

  Objects_NofObjects:
    start = 0
    length = 8
  """

  return _mr76_get_bits(
    data,
    0,
    8,
  )


def _mr76_decode_60b(
  data: bytes,
) -> dict[str, float | int]:
  """
  MR76 0x60B Object General Information.

  Signal definition used here:

    Object_ID
      start = 0
      length = 8

    Object_DistLong
      start = 19
      length = 13
      resolution = 0.2 m
      offset = -500 m

    Object_DistLat
      start = 24
      length = 11
      resolution = 0.2 m
      offset = -204.6 m

    Object_VrelLong
      start = 46
      length = 10
      resolution = 0.25 m/s
      offset = -128 m

    Object_DynProp
      start = 48
      length = 3

    Object_VrelLat
      start = 53
      length = 9
      resolution = 0.25 m/s
      offset = -64 m

    Object_RCS
      start = 56
      length = 8
      resolution = 0.5 dBm2
      offset = -64
  """

  if len(data) < 8:
    return {}

  object_id = _mr76_get_bits(
    data,
    0,
    8,
  )

  dist_long_raw = _mr76_get_bits(
    data,
    19,
    13,
  )

  dist_lat_raw = _mr76_get_bits(
    data,
    24,
    11,
  )

  vrel_long_raw = _mr76_get_bits(
    data,
    46,
    10,
  )

  dyn_prop = _mr76_get_bits(
    data,
    48,
    3,
  )

  vrel_lat_raw = _mr76_get_bits(
    data,
    53,
    9,
  )

  rcs_raw = _mr76_get_bits(
    data,
    56,
    8,
  )

  d_rel = (
    dist_long_raw * 0.2
    - 500.0
  )

  y_rel = (
    dist_lat_raw * 0.2
    - 204.6
  )

  v_rel = (
    vrel_long_raw * 0.25
    - 128.0
  )

  v_rel_lat = (
    vrel_lat_raw * 0.25
    - 64.0
  )

  rcs = (
    rcs_raw * 0.5
    - 64.0
  )

  return {
    "id": object_id,
    "dRel": d_rel,
    "yRel": y_rel,
    "vRel": v_rel,
    "vRelLat": v_rel_lat,
    "dynProp": dyn_prop,
    "rcs": rcs,
  }


# ============================================================================
# MR76 Track
# ============================================================================

class MR76Track:

  def __init__(
    self,
    identifier: int,
  ):
    self.identifier = identifier

    self.dRel = 0.0
    self.yRel = 0.0
    self.vRel = 0.0
    self.vRelLat = 0.0

    self.dynProp = 0
    self.rcs = 0.0

    self.cnt = 0
    self.missing_cycles = 0

  def update(
    self,
    d_rel: float,
    y_rel: float,
    v_rel: float,
    v_rel_lat: float,
    dyn_prop: int,
    rcs: float,
  ) -> None:

    self.dRel = float(d_rel)
    self.yRel = float(y_rel)
    self.vRel = float(v_rel)
    self.vRelLat = float(v_rel_lat)

    self.dynProp = int(dyn_prop)
    self.rcs = float(rcs)

    self.cnt += 1
    self.missing_cycles = 0

  @property
  def confirmed(self) -> bool:
    return (
      self.cnt >= MR76_MIN_CONFIRM_COUNT
    )

  @property
  def angle_deg(self) -> float:

    if abs(self.dRel) < 0.01:
      return (
        90.0
        if self.yRel >= 0
        else -90.0
      )

    return math.degrees(
      math.atan2(
        self.yRel,
        self.dRel,
      )
    )

  @property
  def is_moving(self) -> bool:
    """
    MR76 dynamic property.

    0 = moving
    1 = stationary
    2 = oncoming
    3 = stationary candidate
    4 = unknown
    5 = crossing stationary
    6 = crossing moving
    7 = stopped
    """

    return self.dynProp in (
      0,
      2,
      6,
    )

  def is_valid(self) -> bool:

    if not math.isfinite(
      self.dRel
    ):
      return False

    if not math.isfinite(
      self.yRel
    ):
      return False

    if not math.isfinite(
      self.vRel
    ):
      return False

    if not math.isfinite(
      self.vRelLat
    ):
      return False

    if self.dRel < MR76_MIN_DISTANCE:
      return False

    if self.dRel > MR76_MAX_DISTANCE:
      return False

    if (
      abs(self.angle_deg)
      > MR76_MAX_ANGLE_DEG
    ):
      return False

    return True

  def __str__(self):

    return (
      f"MR76 "
      f"id={self.identifier} "
      f"d={self.dRel:.1f} "
      f"y={self.yRel:.1f} "
      f"v={self.vRel:.1f} "
      f"vl={self.vRelLat:.1f} "
      f"angle={self.angle_deg:.1f} "
      f"dyn={self.dynProp} "
      f"cnt={self.cnt}"
    )


# ============================================================================
# MR76 Radar parser
# ============================================================================

class MR76Radar:

  def __init__(self):

    self.tracks: dict[
      int,
      MR76Track,
    ] = {}

    self.target_count = 0
    self.measurement_count = 0

    self._cycle_targets: dict[
      int,
      dict[str, float | int],
    ] = {}

    self._cycle_active = False

    self.last_valid_time = 0.0

    self.rx_count = 0
    self.object_count = 0

  # --------------------------------------------------------------------------
  # Start new MR76 measurement cycle
  # --------------------------------------------------------------------------

  def _start_cycle(
    self,
    target_count: int,
  ) -> None:

    if self._cycle_active:
      self._finish_cycle()

    self._cycle_targets = {}

    self.target_count = max(
      0,
      min(
        int(target_count),
        255,
      ),
    )

    self._cycle_active = True

    self.measurement_count += 1

  # --------------------------------------------------------------------------
  # Finish one complete MR76 object cycle
  # --------------------------------------------------------------------------

  def _finish_cycle(self) -> None:

    seen_ids = set()

    # ------------------------------------------------------------------------
    # Update received targets
    # ------------------------------------------------------------------------

    for object_id, obj in (
      self._cycle_targets.items()
    ):

      try:

        track = self.tracks.get(
          object_id
        )

        if track is None:

          track = MR76Track(
            object_id
          )

          self.tracks[
            object_id
          ] = track

        track.update(
          float(obj["dRel"]),
          float(obj["yRel"]),
          float(obj["vRel"]),
          float(obj["vRelLat"]),
          int(obj["dynProp"]),
          float(obj["rcs"]),
        )

        if track.is_valid():

          seen_ids.add(
            object_id
          )

        else:

          track.missing_cycles += 1

      except Exception:

        cloudlog.exception(
          "MR76 target update exception"
        )

    # ------------------------------------------------------------------------
    # Age missing targets
    # ------------------------------------------------------------------------

    for object_id in list(
      self.tracks.keys()
    ):

      if object_id not in seen_ids:

        track = self.tracks[
          object_id
        ]

        track.missing_cycles += 1

        if (
          track.missing_cycles
          >= MR76_MISSING_CYCLES
        ):

          self.tracks.pop(
            object_id,
            None,
          )

    # ------------------------------------------------------------------------
    # Remove invalid targets
    # ------------------------------------------------------------------------

    for object_id in list(
      self.tracks.keys()
    ):

      track = self.tracks[
        object_id
      ]

      if not track.is_valid():

        self.tracks.pop(
          object_id,
          None,
        )

    # ------------------------------------------------------------------------
    # IMPORTANT:
    #
    # Limit MR76 targets to 5.
    #
    # Selection priority:
    #
    #   1. confirmed target
    #   2. closer target
    #
    # This prevents an excessive number of boxes/targets from propagating
    # into future auxiliary-object consumers.
    # ------------------------------------------------------------------------

    if len(self.tracks) > MR76_MAX_TARGETS:

      ordered = sorted(
        self.tracks.values(),
        key=lambda t: (
          not t.confirmed,
          t.dRel,
        ),
      )

      keep_ids = {
        t.identifier
        for t in ordered[
          :MR76_MAX_TARGETS
        ]
      }

      for object_id in list(
        self.tracks.keys()
      ):

        if (
          object_id
          not in keep_ids
        ):

          self.tracks.pop(
            object_id,
            None,
          )

    self.object_count = len(
      self.tracks
    )

    # ------------------------------------------------------------------------
    # Debug
    # ------------------------------------------------------------------------

    if MR76_DEBUG:

      confirmed = [
        t
        for t in self.tracks.values()
        if (
          t.confirmed
          and t.is_valid()
        )
      ]

      confirmed = sorted(
        confirmed,
        key=lambda t: t.dRel,
      )

      if confirmed:

        cloudlog.warning(
          "[MR76] "
          f"cycle={self.measurement_count} "
          f"targets={len(confirmed)} "
          + " | ".join(
            str(t)
            for t in confirmed[
              :MR76_MAX_DISPLAY_TARGETS
            ]
          )
        )

    self._cycle_targets = {}

  # --------------------------------------------------------------------------
  # CAN update
  # --------------------------------------------------------------------------

  def update(
    self,
    can_data,
    current_time: float = 0.0,
  ) -> None:

    if not MR76_ENABLE:
      return

    if can_data is None:
      return

    for msg in can_data:

      try:

        address = int(
          msg.address
        )

        src = int(
          msg.src
        )

        # MR76 only exists on CAN1.
        if src != MR76_BUS:
          continue

        # --------------------------------------------------------------------
        # 0x60A
        # --------------------------------------------------------------------

        if address == MR76_STATUS_ID:

          data = bytes(
            msg.dat
          )

          if len(data) < 1:
            continue

          target_count = (
            _mr76_decode_60a(
              data
            )
          )

          self._start_cycle(
            target_count
          )

          self.rx_count += 1
          self.last_valid_time = (
            current_time
          )

        # --------------------------------------------------------------------
        # 0x60B
        # --------------------------------------------------------------------

        elif address == MR76_OBJECT_ID:

          data = bytes(
            msg.dat
          )

          if len(data) < 8:
            continue

          # Do not accept orphaned object frames.
          if not self._cycle_active:
            continue

          obj = _mr76_decode_60b(
            data
          )

          if not obj:
            continue

          object_id = int(
            obj["id"]
          )

          d_rel = float(
            obj["dRel"]
          )

          y_rel = float(
            obj["yRel"]
          )

          v_rel = float(
            obj["vRel"]
          )

          if not math.isfinite(
            d_rel
          ):
            continue

          if not math.isfinite(
            y_rel
          ):
            continue

          if not math.isfinite(
            v_rel
          ):
            continue

          if (
            d_rel < MR76_MIN_DISTANCE
            or
            d_rel > MR76_MAX_DISTANCE
          ):
            continue

          angle = math.degrees(
            math.atan2(
              y_rel,
              max(
                d_rel,
                0.01,
              ),
            )
          )

          if (
            abs(angle)
            > MR76_MAX_ANGLE_DEG
          ):
            continue

          # Same object ID appearing multiple times within one cycle:
          # keep the latest measurement.
          self._cycle_targets[
            object_id
          ] = obj

          self.rx_count += 1
          self.last_valid_time = (
            current_time
          )

      except Exception:

        # MR76 must NEVER be allowed to kill RadarD.
        cloudlog.exception(
          "MR76 CAN parse exception"
        )

  # --------------------------------------------------------------------------
  # Confirmed targets
  # --------------------------------------------------------------------------

  def get_confirmed_tracks(
    self,
  ) -> list[MR76Track]:

    tracks = [
      t
      for t in self.tracks.values()
      if (
        t.confirmed
        and t.is_valid()
      )
    ]

    return sorted(
      tracks,
      key=lambda t: t.dRel,
    )[
      :MR76_MAX_DISPLAY_TARGETS
    ]

  # --------------------------------------------------------------------------
  # Front targets
  # --------------------------------------------------------------------------

  def get_front_tracks(
    self,
  ) -> list[MR76Track]:

    tracks = [
      t
      for t in self.get_confirmed_tracks()
      if (
        t.dRel > MR76_MIN_DISTANCE
        and abs(t.yRel) < 3.5
      )
    ]

    return sorted(
      tracks,
      key=lambda t: t.dRel,
    )[
      :MR76_MAX_DISPLAY_TARGETS
    ]

  # --------------------------------------------------------------------------
  # Side targets
  # --------------------------------------------------------------------------

  def get_side_tracks(
    self,
  ) -> list[MR76Track]:

    tracks = [
      t
      for t in self.get_confirmed_tracks()
      if abs(t.yRel) >= 1.2
    ]

    return sorted(
      tracks,
      key=lambda t: t.dRel,
    )[
      :MR76_MAX_DISPLAY_TARGETS
    ]


# ============================================================================
# Original OEM Radar Kalman filter
# ============================================================================

class KalmanParams:

  def __init__(
    self,
    dt: float,
  ):

    assert (
      dt > .01
      and dt < .2
    ), (
      "Radar time step must be between "
      ".01s and .2s"
    )

    self.A = [
      [1.0, dt],
      [0.0, 1.0],
    ]

    self.C = [
      1.0,
      0.0,
    ]

    dts = [
      i * 0.01
      for i in range(1, 21)
    ]

    K0 = [
      0.12287673,
      0.14556536,
      0.16522756,
      0.18281627,
      0.1988689,
      0.21372394,
      0.22761098,
      0.24069424,
      0.253096,
      0.26491023,
      0.27621103,
      0.28705801,
      0.29750003,
      0.30757767,
      0.31732515,
      0.32677158,
      0.33594201,
      0.34485814,
      0.35353899,
      0.36200124,
    ]

    K1 = [
      0.29666309,
      0.29330885,
      0.29042818,
      0.28787125,
      0.28555364,
      0.28342219,
      0.28144091,
      0.27958406,
      0.27783249,
      0.27617149,
      0.27458948,
      0.27307714,
      0.27162685,
      0.27023228,
      0.26888809,
      0.26758976,
      0.26633338,
      0.26511557,
      0.26393339,
      0.26278425,
    ]

    self.K = [
      [
        np.interp(
          dt,
          dts,
          K0,
        )
      ],
      [
        np.interp(
          dt,
          dts,
          K1,
        )
      ],
    ]


# ============================================================================
# Original OEM Radar Track
# ============================================================================

class Track:

  def __init__(
    self,
    identifier: int,
    v_lead: float,
    kalman_params: KalmanParams,
  ):

    self.identifier = identifier
    self.cnt = 0

    self.aLeadTau = FirstOrderFilter(
      _LEAD_ACCEL_TAU,
      0.45,
      DT_MDL,
    )

    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K

    self.kf = KF1D(
      [[v_lead], [0.0]],
      self.K_A,
      self.K_C,
      self.K_K,
    )

  def update(
    self,
    d_rel: float,
    y_rel: float,
    v_rel: float,
    v_lead: float,
    measured: float,
  ):

    self.dRel = d_rel
    self.yRel = y_rel
    self.vRel = v_rel

    self.vLead = v_lead
    self.measured = measured

    if self.cnt > 0:
      self.kf.update(
        self.vLead
      )

    self.vLeadK = float(
      self.kf.x[SPEED][0]
    )

    self.aLeadK = float(
      self.kf.x[ACCEL][0]
    )

    if abs(self.aLeadK) < 0.5:

      self.aLeadTau.x = (
        _LEAD_ACCEL_TAU
      )

    else:

      self.aLeadTau.update(
        0.0
      )

    self.cnt += 1

  def get_RadarState(
    self,
    model_prob: float = 0.0,
  ):

    return {
      "dRel": float(
        self.dRel
      ),

      "yRel": float(
        self.yRel
      ),

      "vRel": float(
        self.vRel
      ),

      "vLead": float(
        self.vLead
      ),

      "vLeadK": float(
        self.vLeadK
      ),

      "aLeadK": float(
        self.aLeadK
      ),

      "aLeadTau": float(
        self.aLeadTau.x
      ),

      "status": True,

      "fcw": self.is_potential_fcw(
        model_prob
      ),

      "modelProb": model_prob,

      "radar": True,

      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(
    self,
    v_ego: float,
  ):

    return (
      abs(self.yRel) < 1.0
      and
      v_ego < V_EGO_STATIONARY
      and
      0.75 < self.dRel < 25
    )

  def is_potential_fcw(
    self,
    model_prob: float,
  ):

    return (
      model_prob > .9
    )

  def __str__(self):

    return (
      f"x: {self.dRel:4.1f} "
      f"y: {self.yRel:4.1f} "
      f"v: {self.vRel:4.1f} "
      f"a: {self.aLeadK:4.1f}"
    )


# ============================================================================
# Vision matching
# ============================================================================

def laplacian_pdf(
  x: float,
  mu: float,
  b: float,
):

  b = max(
    b,
    1e-4,
  )

  return math.exp(
    -abs(x - mu) / b
  )


def match_vision_to_track(
  v_ego: float,
  lead: capnp._DynamicStructReader,
  tracks: dict[int, Track],
):

  offset_vision_dist = (
    lead.x[0]
    - RADAR_TO_CAMERA
  )

  def prob(c):

    try:
      x_std = float(
        lead.xStd[0]
      )
    except Exception:
      x_std = 6.0

    try:
      y_std = float(
        lead.yStd[0]
      )
    except Exception:
      y_std = 1.0

    try:
      v_std = float(
        lead.vStd[0]
      )
    except Exception:
      v_std = 6.0

    x_std = float(
      np.clip(
        x_std,
        0.5,
        6.0,
      )
    )

    y_std = float(
      np.clip(
        y_std,
        0.2,
        1.2,
      )
    )

    v_std = float(
      np.clip(
        v_std,
        0.5,
        6.0,
      )
    )

    prob_d = laplacian_pdf(
      c.dRel,
      offset_vision_dist,
      x_std,
    )

    prob_y = laplacian_pdf(
      c.yRel,
      -lead.y[0],
      y_std,
    )

    prob_v = laplacian_pdf(
      c.vRel + v_ego,
      lead.v[0],
      v_std,
    )

    return (
      prob_d
      * prob_y
      * prob_v
    )

  if not tracks:
    return None

  track = max(
    tracks.values(),
    key=prob,
  )

  dist_sane = (
    abs(
      track.dRel
      - offset_vision_dist
    )
    <
    max(
      [
        offset_vision_dist * .25,
        5.0,
      ]
    )
  )

  vel_sane = (
    abs(
      track.vRel
      + v_ego
      - lead.v[0]
    ) < 10
    or
    (
      v_ego
      + track.vRel
      > 3
    )
  )

  if (
    dist_sane
    and vel_sane
  ):
    return track

  return None


# ============================================================================
# Vision-only RadarState
# ============================================================================

def get_RadarState_from_vision(
  lead_msg: capnp._DynamicStructReader,
  v_ego: float,
  model_v_ego: float,
):

  lead_v_rel_pred = (
    lead_msg.v[0]
    - model_v_ego
  )

  return {
    "dRel": float(
      lead_msg.x[0]
      - RADAR_TO_CAMERA
    ),

    "yRel": float(
      -lead_msg.y[0]
    ),

    "vRel": float(
      lead_v_rel_pred
    ),

    "vLead": float(
      v_ego
      + lead_v_rel_pred
    ),

    "vLeadK": float(
      v_ego
      + lead_v_rel_pred
    ),

    "aLeadK": float(
      lead_msg.a[0]
    ),

    "aLeadTau": 0.3,

    "fcw": False,

    "modelProb": float(
      lead_msg.prob
    ),

    "status": True,

    "radar": False,

    "radarTrackId": -1,
  }


# ============================================================================
# Lead selection
# ============================================================================

def get_lead(
  v_ego: float,
  ready: bool,
  tracks: dict[int, Track],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  low_speed_override: bool = True,
  match_prob_min: float = 0.5,
  vision_prob_min: float = 0.5,
  allow_radar_only: bool = False,
) -> dict[str, Any]:

  if (
    len(tracks) > 0
    and ready
    and lead_msg.prob > match_prob_min
  ):

    track = match_vision_to_track(
      v_ego,
      lead_msg,
      tracks,
    )

  else:

    track = None

  lead_dict = {
    "status": False
  }

  # --------------------------------------------------------------------------
  # OEM radar matched with vision
  # --------------------------------------------------------------------------

  if track is not None:

    lead_dict = (
      track.get_RadarState(
        lead_msg.prob
      )
    )

  # --------------------------------------------------------------------------
  # Vision fallback
  #
  # This is original behavior.
  # --------------------------------------------------------------------------

  elif (
    track is None
    and ready
    and lead_msg.prob > vision_prob_min
  ):

    lead_dict = (
      get_RadarState_from_vision(
        lead_msg,
        v_ego,
        model_v_ego,
      )
    )

  # --------------------------------------------------------------------------
  # Low-speed OEM radar override
  #
  # IMPORTANT:
  # Only OEM self.tracks are used.
  # MR76 cannot enter here.
  # --------------------------------------------------------------------------

  if low_speed_override:

    low_speed_tracks = [
      c
      for c in tracks.values()
      if c.potential_low_speed_lead(
        v_ego
      )
    ]

    if len(low_speed_tracks) > 0:

      closest_track = min(
        low_speed_tracks,
        key=lambda c: c.dRel,
      )

      if (
        not lead_dict["status"]
        or
        closest_track.dRel
        < lead_dict["dRel"]
      ):

        lead_dict = (
          closest_track.get_RadarState()
        )

  # --------------------------------------------------------------------------
  # Radar-only lead
  #
  # Explicitly disabled by RadarD.
  #
  # This is important for Ford/Lincoln because we want to avoid auxiliary
  # radar causing phantom braking.
  # --------------------------------------------------------------------------

  if (
    allow_radar_only
    and ready
    and not lead_dict["status"]
    and len(tracks) > 0
  ):

    radar_only_candidates = [
      t
      for t in tracks.values()
      if (
        t.measured
        and t.cnt >= 2
        and abs(t.yRel) < 1.1
        and (
          v_ego + t.vRel
        ) >= 2.0
        and
        6.0 < t.dRel < 130.0
      )
    ]

    if len(
      radar_only_candidates
    ) > 0:

      closest = min(
        radar_only_candidates,
        key=lambda t: t.dRel,
      )

      lead_dict = (
        closest.get_RadarState(
          model_prob=0.0
        )
      )

  return lead_dict


# ============================================================================
# RadarD
# ============================================================================

class RadarD:

  def __init__(
    self,
    delay: float = 0.0,
    CP: Any = None,
  ):

    self.current_time = 0.0

    self.CP = CP

    # ------------------------------------------------------------------------
    # OEM radar tracks
    #
    # DO NOT mix MR76 tracks into this dictionary.
    # ------------------------------------------------------------------------

    self.tracks: dict[
      int,
      Track,
    ] = {}

    self.kalman_params = (
      KalmanParams(
        DT_MDL
      )
    )

    # ------------------------------------------------------------------------
    # MR76 auxiliary radar
    # ------------------------------------------------------------------------

    self.mr76 = MR76Radar()

    self.mr76_front_tracks: list[
      MR76Track
    ] = []

    self.mr76_side_tracks: list[
      MR76Track
    ] = []

    # ------------------------------------------------------------------------
    # Ego vehicle state
    # ------------------------------------------------------------------------

    self.v_ego = 0.0

    self.v_ego_hist = deque(
      [0.0],
      maxlen=(
        int(
          round(
            delay / DT_MDL
          )
        )
        + 1
      ),
    )

    self.last_v_ego_frame = -1

    # ------------------------------------------------------------------------
    # RadarState
    # ------------------------------------------------------------------------

    self.radar_state = None

    self.radar_state_valid = False

    self.ready = False

    # ------------------------------------------------------------------------
    # Lead hold
    # ------------------------------------------------------------------------

    self._lead_one_last = None
    self._lead_one_last_t = 0.0

    self._lead_two_last = None
    self._lead_two_last_t = 0.0

  # --------------------------------------------------------------------------
  # Lead hold
  # --------------------------------------------------------------------------

  def _apply_lead_hold(
    self,
    lead_dict,
    last_lead,
    last_time,
  ):

    if lead_dict.get(
      "status",
      False,
    ):

      return (
        lead_dict,
        lead_dict,
        self.current_time,
      )

    if (
      self.v_ego <= LEAD_HOLD_MAX_SPEED
      and
      last_lead is not None
      and
      last_lead.get(
        "status",
        False,
      )
      and
      (
        self.current_time
        - last_time
      )
      <= LEAD_HOLD_TIME_S
    ):

      return (
        last_lead,
        last_lead,
        last_time,
      )

    return (
      lead_dict,
      last_lead,
      last_time,
    )

  # --------------------------------------------------------------------------
  # Main RadarD update
  # --------------------------------------------------------------------------

  def update(
    self,
    sm: messaging.SubMaster,
    rr: car.RadarData,
  ):

    self.ready = (
      sm.seen["modelV2"]
    )

    # ------------------------------------------------------------------------
    # Current timestamp
    # ------------------------------------------------------------------------

    try:

      self.current_time = (
        1e-9
        * max(
          sm.logMonoTime.values()
        )
      )

    except Exception:

      self.current_time = 0.0

    # ------------------------------------------------------------------------
    # Ego speed
    # ------------------------------------------------------------------------

    if (
      sm.recv_frame["carState"]
      != self.last_v_ego_frame
    ):

      self.v_ego = (
        sm["carState"].vEgo
      )

      self.v_ego_hist.append(
        self.v_ego
      )

      self.last_v_ego_frame = (
        sm.recv_frame[
          "carState"
        ]
      )

    # ------------------------------------------------------------------------
    # MR76 auxiliary radar
    #
    # IMPORTANT:
    #
    # Failure here can NEVER kill OEM RadarD.
    # ------------------------------------------------------------------------

    if MR76_ENABLE:

      try:

        self.mr76.update(
          sm["can"],
          self.current_time,
        )

        self.mr76_front_tracks = (
          self.mr76.get_front_tracks()
        )

        self.mr76_side_tracks = (
          self.mr76.get_side_tracks()
        )

      except Exception:

        cloudlog.exception(
          "MR76 auxiliary update exception"
        )

        self.mr76_front_tracks = []
        self.mr76_side_tracks = []

    # ------------------------------------------------------------------------
    # OEM radar points
    #
    # ONLY rr.points are allowed into self.tracks.
    # ------------------------------------------------------------------------

    ar_pts = {
      pt.trackId: [
        pt.dRel,
        pt.yRel,
        pt.vRel,
        pt.measured,
      ]
      for pt in rr.points
    }

    # ------------------------------------------------------------------------
    # Remove missing OEM radar points
    # ------------------------------------------------------------------------

    for ids in list(
      self.tracks.keys()
    ):

      if ids not in ar_pts:

        self.tracks.pop(
          ids,
          None,
        )

    # ------------------------------------------------------------------------
    # Update OEM radar tracks
    # ------------------------------------------------------------------------

    for ids in ar_pts:

      rpt = ar_pts[ids]

      # Align ego speed by radar delay.
      v_lead = (
        rpt[2]
        + self.v_ego_hist[0]
      )

      if ids not in self.tracks:

        self.tracks[ids] = (
          Track(
            ids,
            v_lead,
            self.kalman_params,
          )
        )

      self.tracks[
        ids
      ].update(
        rpt[0],
        rpt[1],
        rpt[2],
        v_lead,
        rpt[3],
      )

    # ------------------------------------------------------------------------
    # Build RadarState
    # ------------------------------------------------------------------------

    self.radar_state_valid = (
      sm.all_checks()
    )

    self.radar_state = (
      log.RadarState.new_message()
    )

    try:

      self.radar_state.mdMonoTime = (
        sm.logMonoTime[
          "modelV2"
        ]
      )

    except Exception:

      self.radar_state.mdMonoTime = 0

    try:

      self.radar_state.radarErrors = (
        rr.errors
      )

    except Exception:

      pass

    try:

      self.radar_state.carStateMonoTime = (
        sm.logMonoTime[
          "carState"
        ]
      )

    except Exception:

      self.radar_state.carStateMonoTime = 0

    # ------------------------------------------------------------------------
    # Model ego velocity
    # ------------------------------------------------------------------------

    if len(
      sm["modelV2"].velocity.x
    ):

      model_v_ego = (
        sm["modelV2"]
        .velocity.x[0]
      )

    else:

      model_v_ego = self.v_ego

    # ------------------------------------------------------------------------
    # Lead messages
    # ------------------------------------------------------------------------

    leads_v3 = (
      sm["modelV2"].leadsV3
    )

    if len(leads_v3) > 1:

      # ----------------------------------------------------------------------
      # Conservative Ford/Lincoln gating
      # ----------------------------------------------------------------------

      match_prob_min = 0.5

      vision_prob_min = 0.5

      # ----------------------------------------------------------------------
      # VERY IMPORTANT:
      #
      # Do NOT enable radar-only lead.
      #
      # This prevents MR76 / imperfect radar data from creating phantom
      # braking.
      # ----------------------------------------------------------------------

      allow_radar_only = False

      # ----------------------------------------------------------------------
      # Lead One
      # ----------------------------------------------------------------------

      lead_one = get_lead(
        self.v_ego,
        self.ready,
        self.tracks,
        leads_v3[0],
        model_v_ego,
        low_speed_override=True,
        match_prob_min=match_prob_min,
        vision_prob_min=vision_prob_min,
        allow_radar_only=allow_radar_only,
      )

      # ----------------------------------------------------------------------
      # Lead Two
      # ----------------------------------------------------------------------

      lead_two = get_lead(
        self.v_ego,
        self.ready,
        self.tracks,
        leads_v3[1],
        model_v_ego,
        low_speed_override=False,
        match_prob_min=match_prob_min,
        vision_prob_min=vision_prob_min,
        allow_radar_only=False,
      )

      # ----------------------------------------------------------------------
      # Lead hold
      # ----------------------------------------------------------------------

      (
        lead_one,
        self._lead_one_last,
        self._lead_one_last_t,
      ) = self._apply_lead_hold(
        lead_one,
        self._lead_one_last,
        self._lead_one_last_t,
      )

      (
        lead_two,
        self._lead_two_last,
        self._lead_two_last_t,
      ) = self._apply_lead_hold(
        lead_two,
        self._lead_two_last,
        self._lead_two_last_t,
      )

      # ----------------------------------------------------------------------
      # Publish ONLY OEM/vision lead information.
      #
      # MR76 is deliberately absent.
      # ----------------------------------------------------------------------

      self.radar_state.leadOne = (
        lead_one
      )

      self.radar_state.leadTwo = (
        lead_two
      )

  # --------------------------------------------------------------------------
  # Publish
  # --------------------------------------------------------------------------

  def publish(
    self,
    pm: messaging.PubMaster,
  ):

    if self.radar_state is None:
      return

    radar_msg = (
      messaging.new_message(
        "radarState"
      )
    )

    radar_msg.valid = (
      self.radar_state_valid
    )

    radar_msg.radarState = (
      self.radar_state
    )

    pm.send(
      "radarState",
      radar_msg,
    )


# ============================================================================
# Main
# ============================================================================

def main() -> None:

  # --------------------------------------------------------------------------
  # RadarD realtime process priority
  # --------------------------------------------------------------------------

  config_realtime_process(
    5,
    Priority.CTRL_LOW,
  )

  # --------------------------------------------------------------------------
  # Wait for CarParams
  # --------------------------------------------------------------------------

  cloudlog.info(
    "radard is waiting for CarParams"
  )

  CP = messaging.log_from_bytes(
    Params().get(
      "CarParams",
      block=True,
    ),
    car.CarParams,
  )

  cloudlog.info(
    "radard got CarParams"
  )

  # --------------------------------------------------------------------------
  # SubMaster
  #
  # Added:
  #
  #   can
  #
  # This is required for MR76 CAN1:
  #
  #   0x60A
  #   0x60B
  #
  # Existing:
  #
  #   modelV2
  #   carState
  #   liveTracks
  #
  # remain unchanged.
  # --------------------------------------------------------------------------

  sm = messaging.SubMaster(
    [
      "modelV2",
      "carState",
      "liveTracks",
      "can",
    ],
    poll="modelV2",
  )

  # --------------------------------------------------------------------------
  # Publisher
  # --------------------------------------------------------------------------

  pm = messaging.PubMaster(
    [
      "radarState"
    ]
  )

  # --------------------------------------------------------------------------
  # RadarD
  # --------------------------------------------------------------------------

  RD = RadarD(
    CP.radarDelay,
    CP,
  )

  # --------------------------------------------------------------------------
  # Main loop
  # --------------------------------------------------------------------------

  while True:

    sm.update()

    RD.update(
      sm,
      sm["liveTracks"],
    )

    RD.publish(
      pm
    )


# ============================================================================
# Entry point
# ============================================================================

if __name__ == "__main__":
  main()