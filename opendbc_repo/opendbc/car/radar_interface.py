"""
Copyright (c) 2025, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
- Commercial use (e.g. use in a product or service, or activity intended to
  generate revenue) is prohibited without explicit written permission from
  the copyright holder.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.can.parser import CANParser
from opendbc.car.structs import RadarData
from typing import List, Tuple

# car head to radar
DREL_OFFSET = -1.52


# typically max lane width is 3.7m
LANE_WIDTH = 3.8
LANE_WIDTH_HALF = LANE_WIDTH / 2

LANE_CENTER_MIN_LAT = 0.
LANE_CENTER_MAX_LAT = LANE_WIDTH_HALF
LANE_CENTER_MIN_DIST = 5.

LANE_SIDE_MIN_LAT = LANE_WIDTH_HALF
LANE_SIDE_MAX_LAT = LANE_WIDTH_HALF + LANE_WIDTH
LANE_SIDE_MIN_DIST = 10.


# lat distance, typically max lane width is 3.7m
MAX_LAT_DIST = 6.

# objects to ignore thats really close to the vehicle (after DREL_OFFSET applied)
MIN_DIST = 5.

# ignore oncoming objects
IGNORE_OBJ_STATE = 2

# ignore objects that we haven't seen for 5 secs
NOT_SEEN_INIT = 33


def _create_radar_parser():
  return CANParser(
    'u_radar',
    [
      ("Status", float('nan')),
      ("ObjectData", float('nan')),
    ],
    1,
  )


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.updated_messages = set()

    self.rcp = _create_radar_parser()

    self._pts_cache = dict()
    self._pts_not_seen = {key: 0 for key in range(255)}
    self._should_clear_cache = False

    # Keep a stable public RadarData point dictionary.
    #
    # card.py receives the RadarData returned by update() and publishes
    # that object as liveTracks. Do not publish cereal messages here.
    self.pts = dict()

  # called by card.py, 100hz
  def update(self, can_strings):
    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    # ----------------------------------------------------------------------
    # MR76 status message
    #
    # 0x60A indicates a new radar cycle. The following 0x60B ObjectData
    # frames contain the actual targets.
    # ----------------------------------------------------------------------
    if 1546 in self.updated_messages:
      self._should_clear_cache = True

    # ----------------------------------------------------------------------
    # MR76 ObjectData
    #
    # 0x60B is repeated for multiple targets. Therefore vl_all must be used
    # instead of vl[1547], otherwise only the last target is visible.
    # ----------------------------------------------------------------------
    if 1547 in self.updated_messages:
      object_data = self.rcp.vl_all.get('ObjectData', {})

      ids = object_data.get('ID', [])
      dist_longs = object_data.get('DistLong', [])
      dist_lats = object_data.get('DistLat', [])
      vrel_longs = object_data.get('VRelLong', [])
      vrel_lats = object_data.get('VRelLat', [])
      dyn_props = object_data.get('DynProp', [])
      obj_classes = object_data.get('Class', [])
      rcs_values = object_data.get('RCS', [])

      # All arrays should have the same number of entries.
      # Use the shortest one defensively.
      frame_count = min(
        len(ids),
        len(dist_longs),
        len(dist_lats),
        len(vrel_longs),
        len(vrel_lats),
        len(dyn_props),
        len(obj_classes),
        len(rcs_values),
      )

      # clean cache when we see a 0x60a then a 0x60b
      if self._should_clear_cache:
        self._pts_cache.clear()
        self._should_clear_cache = False

      for idx in range(frame_count):
        track_id = int(ids[idx])

        if track_id < 0 or track_id > 255:
          continue

        dist_long = float(dist_longs[idx])
        dist_lat = float(dist_lats[idx])
        vrel_long = float(vrel_longs[idx])
        vrel_lat = float(vrel_lats[idx])
        dyn_prop = int(dyn_props[idx])
        obj_class = int(obj_classes[idx])
        _rcs = float(rcs_values[idx])

        d_rel = dist_long + DREL_OFFSET
        y_rel = -dist_lat

        should_ignore = False

        # --------------------------------------------------------------
        # Ignore point
        # --------------------------------------------------------------
        if not should_ignore and obj_class == 0:
          should_ignore = True

        # --------------------------------------------------------------
        # Ignore oncoming objects
        # @todo remove this because it's always 0 ?
        # --------------------------------------------------------------
        if not should_ignore and dyn_prop == IGNORE_OBJ_STATE:
          should_ignore = True

        # --------------------------------------------------------------
        # Far away lane object, ignore
        # --------------------------------------------------------------
        if not should_ignore and abs(y_rel) > LANE_SIDE_MAX_LAT:
          should_ignore = True

        # --------------------------------------------------------------
        # Close object, ignore, use vision
        # --------------------------------------------------------------
        if (
          not should_ignore
          and LANE_CENTER_MIN_LAT < abs(y_rel) < LANE_CENTER_MAX_LAT
          and d_rel < LANE_CENTER_MIN_DIST
        ):
          should_ignore = True

        # --------------------------------------------------------------
        # Close side object, ignore, use vision
        # --------------------------------------------------------------
        if (
          not should_ignore
          and LANE_SIDE_MIN_LAT < abs(y_rel) < LANE_SIDE_MAX_LAT
          and d_rel < LANE_SIDE_MIN_DIST
        ):
          should_ignore = True

        # --------------------------------------------------------------
        # Invalid/empty radar target
        # --------------------------------------------------------------
        if d_rel <= MIN_DIST:
          should_ignore = True

        if not should_ignore and track_id not in self._pts_cache:
          self._pts_cache[track_id] = RadarData.RadarPoint()
          self._pts_cache[track_id].trackId = track_id

        if should_ignore:
          self._pts_not_seen[track_id] = -1
        else:
          self._pts_not_seen[track_id] = NOT_SEEN_INIT

          # init cache
          if track_id not in self._pts_cache:
            self._pts_cache[track_id] = RadarData.RadarPoint()
            self._pts_cache[track_id].trackId = track_id

          # add/update to cache
          point = self._pts_cache[track_id]

          point.trackId = track_id
          point.dRel = d_rel
          point.yRel = y_rel
          point.vRel = vrel_long
          point.yvRel = vrel_lat
          point.aRel = float('nan')
          point.measured = True

    self.updated_messages.clear()

    # ----------------------------------------------------------------------
    # Return RadarData periodically.
    #
    # card.py publishes this returned RadarData as liveTracks.
    # Keep this at the existing 1/3 rate to minimize CPU overhead.
    # ----------------------------------------------------------------------
    if self.frame % 3 == 0:
      keys_to_remove = [
        key for key in self.pts
        if key not in self._pts_cache
      ]

      for key in keys_to_remove:
        self._pts_not_seen[key] -= 1

        if self._pts_not_seen[key] <= 0:
          del self.pts[key]

      self.pts.update(self._pts_cache)

      ret = RadarData()

      if not self.rcp.can_valid:
        ret.errors.canError = True

      ret.points = list(self.pts.values())

      return ret

    return None