#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MR76 Auxiliary Radar Parser

MR76:
  - Smartmicro MR76
  - CAN1
  - u_radar.dbc
  - 0x201 RadarState
  - 0x60A Status
  - 0x60B ObjectData

IMPORTANT
---------
This module is AUXILIARY ONLY.

It:
  - does NOT send CAN
  - does NOT modify CarControl
  - does NOT generate leadOne
  - does NOT generate leadTwo
  - does NOT replace OEM Ford/Delphi radar
  - does NOT publish radarState

It is intended to provide MR76 object information to:
  - SceneUnderstanding
  - AutoAvoidance
  - AutoOvertake
  - obstacle detection
  - side-object detection

The original u_radar.dbc is used directly.

No manual Motorola / Big-Endian bit decoder is used here.
All signal decoding is delegated to the system CANParser.
"""

from __future__ import annotations

import math
from typing import Any

from opendbc.can.parser import CANParser


# =============================================================================
# Configuration
# =============================================================================

MR76_DBC = "u_radar"

# MR76 is physically connected to C3X CAN1.
MR76_BUS = 1

# MR76 CAN IDs from the original u_radar.dbc.
MR76_RADAR_STATE_ID = 0x201
MR76_STATUS_ID = 0x60A
MR76_OBJECT_ID = 0x60B

# Parser frequencies.
MR76_RADAR_STATE_FREQ = 10
MR76_STATUS_FREQ = 10
MR76_OBJECT_FREQ = 50

# Safety limits.
MR76_MIN_DISTANCE = 2.0
MR76_MAX_DISTANCE = 150.0
MR76_MAX_ANGLE_DEG = 45.0

# Maximum number of objects exposed to the rest of the system.
MR76_MAX_TARGETS = 20

# A target must be seen repeatedly before being considered confirmed.
MR76_MIN_CONFIRM_COUNT = 3

# Number of completed object-list cycles allowed to miss a target.
MR76_MISSING_CYCLES = 3

# Debug logging disabled by default.
MR76_DEBUG = False


# =============================================================================
# Helper functions
# =============================================================================

def _finite(value: Any, default: float = 0.0) -> float:
  """
  Convert a value to finite float.

  CANParser normally already returns numeric values, but this prevents
  malformed values from propagating into SceneUnderstanding / RadarD.
  """
  try:
    value = float(value)
  except (TypeError, ValueError):
    return default

  if not math.isfinite(value):
    return default

  return value


def _int(value: Any, default: int = 0) -> int:
  """
  Safe integer conversion.
  """
  try:
    return int(value)
  except (TypeError, ValueError):
    return default


# =============================================================================
# MR76 Track
# =============================================================================

class MR76Track:

  def __init__(
    self,
    identifier: int,
  ):
    self.identifier = int(identifier)

    self.dRel = 0.0
    self.yRel = 0.0
    self.vRel = 0.0
    self.vRelLat = 0.0

    self.dynProp = 0
    self.object_class = 0
    self.rcs = 0.0

    self.cnt = 0
    self.missing_cycles = 0

  # ---------------------------------------------------------------------------
  # Update
  # ---------------------------------------------------------------------------

  def update(
    self,
    d_rel: float,
    y_rel: float,
    v_rel: float,
    v_rel_lat: float,
    dyn_prop: int,
    object_class: int,
    rcs: float,
  ) -> None:

    self.dRel = float(d_rel)
    self.yRel = float(y_rel)
    self.vRel = float(v_rel)
    self.vRelLat = float(v_rel_lat)

    self.dynProp = int(dyn_prop)
    self.object_class = int(object_class)
    self.rcs = float(rcs)

    self.cnt += 1
    self.missing_cycles = 0

  # ---------------------------------------------------------------------------
  # Confirmation
  # ---------------------------------------------------------------------------

  @property
  def confirmed(self) -> bool:
    return self.cnt >= MR76_MIN_CONFIRM_COUNT

  # ---------------------------------------------------------------------------
  # Angle
  # ---------------------------------------------------------------------------

  @property
  def angle_deg(self) -> float:

    if abs(self.dRel) < 0.01:
      return 90.0 if self.yRel >= 0.0 else -90.0

    return math.degrees(
      math.atan2(
        self.yRel,
        self.dRel,
      )
    )

  # ---------------------------------------------------------------------------
  # Motion state
  # ---------------------------------------------------------------------------

  @property
  def is_moving(self) -> bool:
    """
    According to the original u_radar.dbc:

      0 moving
      1 stationary
      2 oncoming
      3 crossing_left
      4 crossing_right
      5 unknown
      6 stopped
    """
    return self.dynProp in (0, 2, 6)

  # ---------------------------------------------------------------------------
  # Vehicle class
  # ---------------------------------------------------------------------------

  @property
  def is_vehicle(self) -> bool:
    """
    According to the original u_radar.dbc:

      0 point
      1 vehicle
    """
    return self.object_class == 1

  # ---------------------------------------------------------------------------
  # Validity
  # ---------------------------------------------------------------------------

  def is_valid(self) -> bool:

    if not math.isfinite(self.dRel):
      return False

    if not math.isfinite(self.yRel):
      return False

    if not math.isfinite(self.vRel):
      return False

    if not math.isfinite(self.vRelLat):
      return False

    if self.dRel < MR76_MIN_DISTANCE:
      return False

    if self.dRel > MR76_MAX_DISTANCE:
      return False

    if abs(self.angle_deg) > MR76_MAX_ANGLE_DEG:
      return False

    return True

  # ---------------------------------------------------------------------------
  # Dictionary representation
  # ---------------------------------------------------------------------------

  def as_dict(self) -> dict[str, Any]:

    return {
      "id": int(self.identifier),
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vRelLat": float(self.vRelLat),
      "dynProp": int(self.dynProp),
      "class": int(self.object_class),
      "rcs": float(self.rcs),
      "angle": float(self.angle_deg),
      "confirmed": bool(self.confirmed),
      "moving": bool(self.is_moving),
      "vehicle": bool(self.is_vehicle),
    }

  def __str__(self) -> str:

    return (
      f"MR76 "
      f"id={self.identifier} "
      f"d={self.dRel:.1f}m "
      f"y={self.yRel:.1f}m "
      f"v={self.vRel:.1f}m/s "
      f"vl={self.vRelLat:.1f}m/s "
      f"angle={self.angle_deg:.1f}deg "
      f"dyn={self.dynProp} "
      f"class={self.object_class} "
      f"cnt={self.cnt}"
    )


# =============================================================================
# MR76 parser
# =============================================================================

class MR76Parser:

  def __init__(
    self,
    dbc_name: str = MR76_DBC,
    bus: int = MR76_BUS,
  ):

    self.dbc_name = dbc_name
    self.bus = int(bus)

    # -------------------------------------------------------------------------
    # IMPORTANT:
    #
    # We use the original system CANParser.
    #
    # No manual CAN bit extraction.
    #
    # Signal definitions come directly from:
    #
    #   u_radar.dbc
    #
    # -------------------------------------------------------------------------

    self.parser = CANParser(
      dbc_name,
      [
        ("RadarState", MR76_RADAR_STATE_FREQ),
        ("Status", MR76_STATUS_FREQ),
        ("ObjectData", MR76_OBJECT_FREQ),
      ],
      self.bus,
    )

    # -------------------------------------------------------------------------
    # Persistent target tracks.
    # -------------------------------------------------------------------------

    self.tracks: dict[int, MR76Track] = {}

    # -------------------------------------------------------------------------
    # Current cycle.
    # -------------------------------------------------------------------------

    self._cycle_targets: dict[
      int,
      dict[str, Any],
    ] = {}

    self._cycle_active = False

    # -------------------------------------------------------------------------
    # Radar state / status.
    # -------------------------------------------------------------------------

    self.state: dict[str, Any] = {}
    self.status: dict[str, Any] = {}

    # -------------------------------------------------------------------------
    # Counters.
    # -------------------------------------------------------------------------

    self.frame_count = 0
    self.status_count = 0
    self.object_frame_count = 0
    self.measurement_count = 0

    self.object_count = 0
    self.confirmed_count = 0

    self.rx_count = 0

    # -------------------------------------------------------------------------
    # Last reported object count from 0x60A.
    # -------------------------------------------------------------------------

    self.expected_object_count = 0

    # -------------------------------------------------------------------------
    # Debug.
    # -------------------------------------------------------------------------

    self._last_debug_count = -1

  # ===========================================================================
  # Parser update
  # ===========================================================================

  def update(
    self,
    can_list,
  ) -> None:

    self.frame_count += 1

    if can_list is None:
      return

    try:
      self.parser.update(can_list)
    except Exception:
      # MR76 is auxiliary. Never allow it to kill RadarD.
      return

    # -------------------------------------------------------------------------
    # Read RadarState.
    # -------------------------------------------------------------------------

    try:
      self.state = dict(
        self.parser.vl.get(
          "RadarState",
          {},
        )
      )
    except Exception:
      self.state = {}

    # -------------------------------------------------------------------------
    # Read Status.
    # -------------------------------------------------------------------------

    try:
      self.status = dict(
        self.parser.vl.get(
          "Status",
          {},
        )
      )
    except Exception:
      self.status = {}

    # -------------------------------------------------------------------------
    # 0x60A:
    #
    # Status.NoOfObjects
    #
    # IMPORTANT:
    # This is the start of an MR76 object-list measurement.
    # -------------------------------------------------------------------------

    try:

      status = self.parser.vl.get(
        "Status",
        {},
      )

      if status:

        no_of_objects = _int(
          status.get(
            "NoOfObjects",
            0,
          )
        )

        self.expected_object_count = max(
          0,
          min(
            no_of_objects,
            255,
          ),
        )

        self.status_count += 1
        self.rx_count += 1

        self._start_cycle(
          self.expected_object_count
        )

    except Exception:
      pass

    # -------------------------------------------------------------------------
    # 0x60B:
    #
    # Read ALL available ObjectData frames.
    #
    # This is the critical part for multi-target support.
    # -------------------------------------------------------------------------

    objects = self._get_all_object_data()

    if objects:

      for obj in objects:

        self._process_object(
          obj
        )

    # -------------------------------------------------------------------------
    # If CANParser does not expose vl_all in this OpenPilot version,
    # fallback to its normal vl representation.
    #
    # This still gives us at least one target and prevents compatibility
    # problems across forks.
    # -------------------------------------------------------------------------

    if not objects:

      try:

        obj = self.parser.vl.get(
          "ObjectData",
          {},
        )

        if obj:

          self._process_object(
            obj
          )

      except Exception:
        pass

    # -------------------------------------------------------------------------
    # Finish the current measurement cycle.
    #
    # We deliberately finish after processing the current CANParser update.
    # -------------------------------------------------------------------------

    if self._cycle_active:

      self._finish_cycle()

  # ===========================================================================
  # Obtain all ObjectData messages
  # ===========================================================================

  def _get_all_object_data(
    self,
  ) -> list[dict[str, Any]]:

    result: list[
      dict[str, Any]
    ] = []

    # -------------------------------------------------------------------------
    # Preferred path:
    #
    # CANParser.vl_all
    #
    # Different OpenPilot generations expose this structure slightly
    # differently, so the code below accepts the common variants.
    # -------------------------------------------------------------------------

    try:

      vl_all = getattr(
        self.parser,
        "vl_all",
        None,
      )

      if vl_all is None:
        return result

      raw = None

      if isinstance(
        vl_all,
        dict,
      ):

        raw = vl_all.get(
          "ObjectData"
        )

        # Some versions use the numeric address.
        if raw is None:
          raw = vl_all.get(
            MR76_OBJECT_ID
          )

        if raw is None:
          raw = vl_all.get(
            str(MR76_OBJECT_ID)
          )

      if raw is None:
        return result

      # -----------------------------------------------------------------------
      # Variant A:
      #
      # [
      #   {"ID": ..., ...},
      #   {"ID": ..., ...},
      # ]
      # -----------------------------------------------------------------------

      if isinstance(
        raw,
        list,
      ):

        for item in raw:

          if isinstance(
            item,
            dict,
          ):

            result.append(
              dict(item)
            )

      # -----------------------------------------------------------------------
      # Variant B:
      #
      # {
      #   "ID": [1, 2, 3],
      #   "DistLong": [10, 20, 30],
      #   ...
      # }
      # -----------------------------------------------------------------------

      elif isinstance(
        raw,
        dict,
      ):

        result = (
          self._convert_signal_arrays(
            raw
          )
        )

    except Exception:
      return []

    return result

  # ===========================================================================
  # Convert CANParser signal-array representation
  # ===========================================================================

  def _convert_signal_arrays(
    self,
    raw: dict[str, Any],
  ) -> list[dict[str, Any]]:

    signal_names = (
      "ID",
      "DistLong",
      "DistLat",
      "VRelLong",
      "VRelLat",
      "DynProp",
      "Class",
      "RCS",
    )

    arrays: dict[
      str,
      list[Any],
    ] = {}

    max_len = 0

    for name in signal_names:

      value = raw.get(
        name
      )

      if isinstance(
        value,
        (list, tuple),
      ):

        arrays[name] = list(
          value
        )

        max_len = max(
          max_len,
          len(value),
        )

      elif value is not None:

        arrays[name] = [
          value
        ]

        max_len = max(
          max_len,
          1,
        )

    result = []

    for i in range(
      max_len
    ):

      obj: dict[str, Any] = {}

      for name in signal_names:

        values = arrays.get(
          name,
          []
        )

        if i < len(values):

          obj[name] = (
            values[i]
          )

      if obj:
        result.append(
          obj
        )

    return result

  # ===========================================================================
  # Start cycle
  # ===========================================================================

  def _start_cycle(
    self,
    expected_count: int,
  ) -> None:

    # If the previous cycle has not been explicitly completed, complete it
    # before starting the next one.
    if self._cycle_active:

      self._finish_cycle()

    self._cycle_targets = {}

    self.expected_object_count = max(
      0,
      min(
        int(expected_count),
        255,
      ),
    )

    self._cycle_active = True

    self.measurement_count += 1

  # ===========================================================================
  # Process one ObjectData frame
  # ===========================================================================

  def _process_object(
    self,
    obj: dict[str, Any],
  ) -> None:

    if not obj:
      return

    try:

      object_id = _int(
        obj.get(
          "ID",
          0,
        )
      )

      d_rel = _finite(
        obj.get(
          "DistLong",
          0.0,
        )
      )

      y_rel = _finite(
        obj.get(
          "DistLat",
          0.0,
        )
      )

      v_rel = _finite(
        obj.get(
          "VRelLong",
          0.0,
        )
      )

      v_rel_lat = _finite(
        obj.get(
          "VRelLat",
          0.0,
        )
      )

      dyn_prop = _int(
        obj.get(
          "DynProp",
          0,
        )
      )

      object_class = _int(
        obj.get(
          "Class",
          0,
        )
      )

      rcs = _finite(
        obj.get(
          "RCS",
          0.0,
        )
      )

      # -----------------------------------------------------------------------
      # Object ID 0 is normally not useful.
      # -----------------------------------------------------------------------

      if object_id < 0:
        return

      # -----------------------------------------------------------------------
      # Sanity checks.
      # -----------------------------------------------------------------------

      if not math.isfinite(
        d_rel
      ):
        return

      if not math.isfinite(
        y_rel
      ):
        return

      if not math.isfinite(
        v_rel
      ):
        return

      if not math.isfinite(
        v_rel_lat
      ):
        return

      if (
        d_rel < MR76_MIN_DISTANCE
        or d_rel > MR76_MAX_DISTANCE
      ):
        return

      angle = math.degrees(
        math.atan2(
          y_rel,
          max(
            d_rel,
            0.01,
          ),
        )
      )

      if abs(angle) > MR76_MAX_ANGLE_DEG:
        return

      # -----------------------------------------------------------------------
      # One ID = one target for the current measurement.
      # -----------------------------------------------------------------------

      self._cycle_targets[
        object_id
      ] = {
        "ID": object_id,
        "DistLong": d_rel,
        "DistLat": y_rel,
        "VRelLong": v_rel,
        "VRelLat": v_rel_lat,
        "DynProp": dyn_prop,
        "Class": object_class,
        "RCS": rcs,
      }

      self.object_frame_count += 1
      self.rx_count += 1

    except Exception:
      return

  # ===========================================================================
  # Finish cycle
  # ===========================================================================

  def _finish_cycle(
    self,
  ) -> None:

    seen_ids: set[int] = set()

    # -------------------------------------------------------------------------
    # Update / create tracks.
    # -------------------------------------------------------------------------

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
          d_rel=float(
            obj["DistLong"]
          ),
          y_rel=float(
            obj["DistLat"]
          ),
          v_rel=float(
            obj["VRelLong"]
          ),
          v_rel_lat=float(
            obj["VRelLat"]
          ),
          dyn_prop=int(
            obj["DynProp"]
          ),
          object_class=int(
            obj["Class"]
          ),
          rcs=float(
            obj["RCS"]
          ),
        )

        if track.is_valid():

          seen_ids.add(
            object_id
          )

      except Exception:
        pass

    # -------------------------------------------------------------------------
    # Age disappeared targets.
    # -------------------------------------------------------------------------

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

    # -------------------------------------------------------------------------
    # Limit maximum number of tracks.
    #
    # Prefer:
    #   1. confirmed
    #   2. nearest
    # -------------------------------------------------------------------------

    if len(
      self.tracks
    ) > MR76_MAX_TARGETS:

      ordered = sorted(
        self.tracks.values(),
        key=lambda t: (
          not t.confirmed,
          t.dRel,
        ),
      )

      keep_ids = {
        track.identifier
        for track in ordered[
          :MR76_MAX_TARGETS
        ]
      }

      for object_id in list(
        self.tracks.keys()
      ):

        if object_id not in keep_ids:

          self.tracks.pop(
            object_id,
            None,
          )

    # -------------------------------------------------------------------------
    # Counters.
    # -------------------------------------------------------------------------

    self.object_count = len(
      self.tracks
    )

    self.confirmed_count = len(
      self.get_confirmed_tracks()
    )

    self._cycle_targets = {}
    self._cycle_active = False

  # ===========================================================================
  # Confirmed targets
  # ===========================================================================

  def get_confirmed_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.tracks.values()
        if (
          track.confirmed
          and track.is_valid()
        )
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # All valid targets
  # ===========================================================================

  def get_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.tracks.values()
        if track.is_valid()
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # Front targets
  # ===========================================================================

  def get_front_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.get_confirmed_tracks()
        if (
          track.dRel > MR76_MIN_DISTANCE
          and abs(track.yRel) < 3.5
        )
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # Side targets
  # ===========================================================================

  def get_side_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.get_confirmed_tracks()
        if abs(track.yRel) >= 1.2
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # Left targets
  # ===========================================================================

  def get_left_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.get_confirmed_tracks()
        if track.yRel > 1.2
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # Right targets
  # ===========================================================================

  def get_right_tracks(
    self,
  ) -> list[MR76Track]:

    return sorted(
      [
        track
        for track in self.get_confirmed_tracks()
        if track.yRel < -1.2
      ],
      key=lambda t: t.dRel,
    )

  # ===========================================================================
  # Nearest front target
  # ===========================================================================

  def get_nearest_front(
    self,
  ) -> MR76Track | None:

    tracks = self.get_front_tracks()

    if not tracks:
      return None

    return tracks[0]

  # ===========================================================================
  # Object list
  # ===========================================================================

  def get_objects(
    self,
  ) -> list[dict[str, Any]]:

    return [
      track.as_dict()
      for track in self.get_tracks()
    ]

  # ===========================================================================
  # State
  # ===========================================================================

  def get_state(
    self,
  ) -> dict[str, Any]:

    return {
      "state": dict(
        self.state
      ),

      "status": dict(
        self.status
      ),

      "objects": self.get_objects(),

      "objectCount": int(
        self.object_count
      ),

      "confirmedCount": int(
        self.confirmed_count
      ),

      "expectedObjectCount": int(
        self.expected_object_count
      ),

      "measurementCount": int(
        self.measurement_count
      ),

      "objectFrameCount": int(
        self.object_frame_count
      ),

      "frames": int(
        self.frame_count
      ),

      "rxCount": int(
        self.rx_count
      ),
    }


# =============================================================================
# Compatibility alias
# =============================================================================
#
# Your previous code used:
#
#   MR76Parser
#
# Keep the name unchanged so existing imports do not need modification.
#

MR76RadarParser = MR76Parser


# =============================================================================
# Simple self-test helper
# =============================================================================

def create_mr76_parser() -> MR76Parser:
  """
  Convenience constructor.

  The default configuration is:
      DBC = u_radar
      BUS = 1
  """
  return MR76Parser(
    dbc_name=MR76_DBC,
    bus=MR76_BUS,
  )