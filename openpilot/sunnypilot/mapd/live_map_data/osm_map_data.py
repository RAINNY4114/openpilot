"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData
from openpilot.sunnypilot.navd.helpers import Coordinate

# GPS quality gating thresholds (from Ford curve speed control commit 264abad).
# These help avoid wrong-road matches and phantom braking in dense environments.
_GPS_HACC_BAD_M = 50.0
_GPS_HACC_SOFT_M = 20.0
_GPS_HOLD_LAST_GOOD_S = 3.0
_GPS_HOLD_MAX_DIST_M = 15.0
_GPS_JUMP_SPEED_FACTOR = 2.0
_GPS_JUMP_SPEED_BONUS_MPS = 5.0

_EARTH_RADIUS_M = 6373000.0
_TO_RADIANS = math.pi / 180.0


def _safe_float(val) -> float | None:
  try:
    f = float(val)
  except Exception:
    return None
  return f if math.isfinite(f) else None


def _gps_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
  ax = float(lat_a) * _TO_RADIANS
  ay = float(lon_a) * _TO_RADIANS
  bx = float(lat_b) * _TO_RADIANS
  by = float(lon_b) * _TO_RADIANS
  a = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1 - a)))
  return _EARTH_RADIUS_M * c


def _gps_quality_ok(speed_mps: float, h_acc_m: float | None) -> bool:
  if h_acc_m is None:
    return True
  if h_acc_m > _GPS_HACC_BAD_M:
    return False
  if speed_mps > 8.0 and h_acc_m > _GPS_HACC_SOFT_M:
    return False
  return True


def _gps_jump_suspected(last_payload: dict[str, float] | None, last_t: float,
                        payload: dict[str, float], now: float,
                        speed_mps: float) -> bool:
  if last_payload is None or last_t <= 0.0:
    return False
  dt = float(now - last_t)
  if not math.isfinite(dt) or dt < 0.05:
    return False
  try:
    dist = _gps_distance_m(float(last_payload["latitude"]), float(last_payload["longitude"]),
                           float(payload["latitude"]), float(payload["longitude"]))
  except Exception:
    return False
  if not math.isfinite(dist) or dist < 0.0:
    return False
  implied_speed = dist / dt
  allowed_speed = max(20.0, float(speed_mps) * _GPS_JUMP_SPEED_FACTOR + _GPS_JUMP_SPEED_BONUS_MPS)
  return bool(implied_speed > allowed_speed)


class OsmMapData(BaseMapData):
  def __init__(self):
    super().__init__()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params

    # GPS quality monitoring state
    self._gps_sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
    self._gps_last_good: dict[str, float] | None = None
    self._gps_last_good_t: float = 0.0
    self._gps_last_written: dict[str, float] | None = None
    self._gps_last_written_t: float = 0.0
    self._gps_last_speed: float = 0.0

  def update_location(self) -> None:
    location = self.sm['liveLocationKalman']
    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])

    if self.last_position is None:
      return

    params = {
      "latitude": self.last_position.latitude,
      "longitude": self.last_position.longitude,
    }

    if self.last_bearing is not None:
      params['bearing'] = self.last_bearing

    self.mem_params.put("LastGPSPosition", json.dumps(params), block=True)

    # Update GPS quality for Ford curve speed control gating
    self._update_gps_quality()

  def _update_gps_quality(self) -> None:
    """Monitor raw GPS quality and write GPSQualityOK to memory params.

    This feeds the Ford/Lincoln curve speed controller's GPS quality gating,
    which disables map-based speed limits when GNSS is unstable (multipath,
    wrong-road matches, etc.) to prevent phantom braking.
    """
    self._gps_sm.update(0)

    now = time.monotonic()
    chosen = None
    gps_ok_out = False
    speed_mps = float(self._gps_last_speed)

    # Try to get a fresh GPS fix
    gps_data = None
    for service in ("gpsLocationExternal", "gpsLocation"):
      if not getattr(self._gps_sm, "alive", {}).get(service, False):
        continue
      gps = self._gps_sm[service]
      if getattr(gps, "hasFix", True) is False:
        continue

      lat = _safe_float(getattr(gps, "latitude", None))
      lon = _safe_float(getattr(gps, "longitude", None))
      if lat is None or lon is None:
        continue

      # Get speed
      spd = _safe_float(getattr(gps, "speed", None))
      if spd is None:
        try:
          vned = getattr(gps, "vNED", None)
          if vned is not None and len(vned) >= 2:
            v_n = _safe_float(vned[0])
            v_e = _safe_float(vned[1])
            if v_n is not None and v_e is not None:
              spd = float(math.hypot(v_n, v_e))
        except Exception:
          spd = None
      if spd is None:
        spd = 0.0

      h_acc = _safe_float(getattr(gps, "horizontalAccuracy", None))
      gps_data = ({
        "latitude": lat,
        "longitude": lon,
      }, float(spd), h_acc)
      break

    if gps_data is not None:
      payload, speed_mps, h_acc_m = gps_data
      self._gps_last_speed = float(speed_mps)

      base_ok = _gps_quality_ok(float(speed_mps), h_acc_m)
      if base_ok and _gps_jump_suspected(self._gps_last_written, float(self._gps_last_written_t),
                                          payload, float(now), float(speed_mps)):
        base_ok = False

      chosen = payload
      if base_ok:
        self._gps_last_good = dict(payload)
        self._gps_last_good_t = float(now)
        gps_ok_out = True
      else:
        clamped_speed = max(0.1, min(50.0, float(speed_mps)))
        hold_s = min(_GPS_HOLD_LAST_GOOD_S, max(0.5, _GPS_HOLD_MAX_DIST_M / clamped_speed))
        if self._gps_last_good is not None and (float(now) - float(self._gps_last_good_t)) < hold_s:
          chosen = self._gps_last_good
          gps_ok_out = True
        else:
          gps_ok_out = False
    else:
      # No fresh GPS fix; hold last good briefly
      if self._gps_last_good is not None:
        clamped_speed = max(0.1, min(50.0, float(speed_mps)))
        hold_s = min(_GPS_HOLD_LAST_GOOD_S, max(0.5, _GPS_HOLD_MAX_DIST_M / clamped_speed))
        chosen = self._gps_last_good
        gps_ok_out = bool((float(now) - float(self._gps_last_good_t)) < hold_s)

    # Write GPSQualityOK to memory params for ford_curve_speed.py
    # Direct file I/O - GPSQualityOK not in compiled C++ params key list
    try:
      with open("/dev/shm/params/GPSQualityOK", "w") as f:
        f.write("1" if gps_ok_out else "0")
    except Exception:
      pass

    if chosen is not None:
      self._gps_last_written = dict(chosen)
      self._gps_last_written_t = float(now)

  def get_current_speed_limit(self) -> float:
    return float(self.mem_params.get("MapSpeedLimit") or 0.0)

  def get_current_road_name(self) -> str:
    return str(self.mem_params.get("RoadName") or "")

  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    next_speed_limit_section_str = self.mem_params.get("NextMapSpeedLimit")
    next_speed_limit_section = next_speed_limit_section_str if next_speed_limit_section_str else {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0.0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = (self.last_position or Coordinate(0, 0)).distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance
