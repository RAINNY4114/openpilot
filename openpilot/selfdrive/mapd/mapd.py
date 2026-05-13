#!/usr/bin/env python3
import json
import math
import os
import socket
import stat
import subprocess
import threading
import time
import urllib.error
import urllib.request

from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# FrogPilot publishes a version pointer for mapd. v2 is JSON and is kept up-to-date.
VERSION_CHANNEL = "v2"
GITHUB_VERSION_URL = f"https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_{VERSION_CHANNEL}.json"
GITLAB_VERSION_URL = f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_{VERSION_CHANNEL}.json"
LEGACY_VERSION_URLS = (
  "https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_v1.txt",
  "https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_v1.txt",
)
FALLBACK_MAPD_VERSION = "v1.12.0"

MAPD_PATH = "/data/media/0/osm/mapd"
MAPD_VERSION_PATH = "/data/media/0/osm/mapd_version"
PARAMS_MEMORY_PATH = "/dev/shm/params"

# GPS quality gating (helps avoid wrong-road matches in dense/parallel-road environments).
# Keep conservative defaults and avoid new user-facing toggles.
_GPS_HACC_BAD_M = 50.0
_GPS_HACC_SOFT_M = 20.0
_GPS_HOLD_LAST_GOOD_S = 3.0
_GPS_HOLD_MAX_DIST_M = 15.0  # cap hold window by distance (≈0.5s @ 110km/h)
_GPS_JUMP_SPEED_FACTOR = 2.0
_GPS_JUMP_SPEED_BONUS_MPS = 5.0

_EARTH_RADIUS_M = 6373000.0
_TO_RADIANS = math.pi / 180.0

DOWNLOAD_URLS = [
  # Primary (openpilot-focused builds)
  "https://github.com/pfeiferj/openpilot-mapd/releases/download/{version}/mapd",
  # Mirror used by FrogPilot (public raw file)
  "https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Mapd/{version}",
  # Older fallback (kept for redundancy)
  "https://github.com/pfeiferj/mapd/releases/download/{version}/mapd",
]

def _read_local_version() -> str | None:
  try:
    with open(MAPD_VERSION_PATH, "r", encoding="utf-8") as f:
      version = f.read().strip()
      return version if version else None
  except FileNotFoundError:
    return None
  except Exception:
    cloudlog.exception("mapd: failed reading version file")
    return None


def _parse_remote_version(payload: bytes) -> str | None:
  try:
    text = payload.decode("utf-8", errors="ignore").strip()
  except Exception:
    return None

  if not text:
    return None

  if text.startswith("{"):
    try:
      obj = json.loads(text)
      version = obj.get("version")
      if isinstance(version, str) and version.strip():
        return version.strip()
    except Exception:
      pass

  # Legacy plain-text format: "vX.Y.Z"
  first_line = text.splitlines()[0].strip()
  return first_line if first_line else None


def _get_latest_version() -> str | None:
  for url in (GITHUB_VERSION_URL, GITLAB_VERSION_URL, *LEGACY_VERSION_URLS):
    try:
      with urllib.request.urlopen(url, timeout=10) as resp:
        version = _parse_remote_version(resp.read())
        if version:
          return version
    except (urllib.error.URLError, socket.timeout):
      continue
    except Exception:
      cloudlog.exception("mapd: failed to fetch latest version")
  return None


def _cleanup_leftovers() -> None:
  parent = os.path.dirname(MAPD_PATH)
  try:
    for name in os.listdir(parent):
      if not name.startswith("mapd"):
        continue
      if name in ("mapd", "mapd_version"):
        continue
      p = os.path.join(parent, name)
      try:
        if os.path.isfile(p):
          os.remove(p)
      except Exception:
        pass
  except FileNotFoundError:
    pass
  except Exception:
    cloudlog.exception("mapd: cleanup failed")


def _download(url: str, dst: str, version: str) -> None:
  os.makedirs(os.path.dirname(dst), exist_ok=True)
  tmp = f"{dst}.download"

  cloudlog.info(f"mapd: downloading {url} -> {dst}")
  with urllib.request.urlopen(url, timeout=30) as resp, open(tmp, "wb") as f:
    while True:
      chunk = resp.read(1024 * 1024)
      if not chunk:
        break
      f.write(chunk)
    f.flush()
    os.fsync(f.fileno())

  os.replace(tmp, dst)
  current_permissions = stat.S_IMODE(os.lstat(dst).st_mode)
  os.chmod(dst, current_permissions | stat.S_IEXEC)

  try:
    with open(MAPD_VERSION_PATH, "w", encoding="utf-8") as f:
      f.write(version)
      f.flush()
      os.fsync(f.fileno())
  except Exception:
    cloudlog.exception("mapd: failed writing version file")


def _ensure_mapd_binary(desired_version: str | None) -> bool:
  try:
    if os.path.exists(MAPD_PATH) and os.path.getsize(MAPD_PATH) > 0 and not desired_version:
      current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode)
      os.chmod(MAPD_PATH, current_permissions | stat.S_IEXEC)
      return True
  except Exception:
    pass

  try:
    local_version = _read_local_version()
    version_to_get = desired_version or local_version or FALLBACK_MAPD_VERSION
    if local_version == version_to_get and os.path.exists(MAPD_PATH) and os.path.getsize(MAPD_PATH) > 0:
      current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode)
      os.chmod(MAPD_PATH, current_permissions | stat.S_IEXEC)
      return True

    for url_tpl in DOWNLOAD_URLS:
      url = url_tpl.format(version=version_to_get)
      try:
        _download(url, MAPD_PATH, version_to_get)
        return True
      except (urllib.error.URLError, socket.timeout):
        continue
      except Exception:
        cloudlog.exception("mapd: download failed")
        continue
  except (urllib.error.URLError, socket.timeout):
    cloudlog.warning("mapd: download failed (no internet?)")
  except Exception:
    cloudlog.exception("mapd: download failed")
  return False


def _safe_float(val) -> float | None:
  try:
    f = float(val)
  except Exception:
    return None
  return f if math.isfinite(f) else None


def _bearing_from_vned(gps) -> float | None:
  try:
    vned = getattr(gps, "vNED", None)
    if vned is not None and len(vned) >= 2:
      v_n = _safe_float(vned[0])
      v_e = _safe_float(vned[1])
      if v_n is not None and v_e is not None and (abs(v_n) + abs(v_e) > 0.2):
        return (math.degrees(math.atan2(v_e, v_n)) + 360.0) % 360.0
  except Exception:
    pass

  # Legacy fields (some forks/devices expose vN/vE directly)
  try:
    v_n = _safe_float(getattr(gps, "vN", None))
    v_e = _safe_float(getattr(gps, "vE", None))
    if v_n is not None and v_e is not None and (abs(v_n) + abs(v_e) > 0.2):
      return (math.degrees(math.atan2(v_e, v_n)) + 360.0) % 360.0
  except Exception:
    pass

  return None


def _distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
  ax = float(lat_a) * _TO_RADIANS
  ay = float(lon_a) * _TO_RADIANS
  bx = float(lat_b) * _TO_RADIANS
  by = float(lon_b) * _TO_RADIANS
  a = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1 - a)))
  return _EARTH_RADIUS_M * c


def _gps_quality_ok(speed_mps: float, h_acc_m: float | None) -> bool:
  # If we have a horizontal accuracy estimate, use it. Many devices don't provide one.
  if h_acc_m is None:
    return True

  # Reject clearly bad positional accuracy (often wrong-road matches).
  if h_acc_m > _GPS_HACC_BAD_M:
    return False

  # At higher speeds require better quality; at low speed accuracy is naturally noisier.
  if speed_mps > 8.0 and h_acc_m > _GPS_HACC_SOFT_M:
    return False

  return True


def _gps_jump_suspected(last_payload: dict[str, float] | None, last_t: float, payload: dict[str, float],
                        now: float, speed_mps: float) -> bool:
  if last_payload is None or last_t <= 0.0:
    return False

  dt = float(now - last_t)
  if not math.isfinite(dt) or dt < 0.05:
    return False

  try:
    dist = _distance_m(float(last_payload["latitude"]), float(last_payload["longitude"]),
                       float(payload["latitude"]), float(payload["longitude"]))
  except Exception:
    return False

  if not math.isfinite(dist) or dist < 0.0:
    return False

  implied_speed = dist / dt
  allowed_speed = max(20.0, float(speed_mps) * _GPS_JUMP_SPEED_FACTOR + _GPS_JUMP_SPEED_BONUS_MPS)
  return bool(implied_speed > allowed_speed)


def _extract_gps_payload(sm: messaging.SubMaster) -> tuple[dict[str, float], float, float | None] | None:
  for service in ("gpsLocationExternal", "gpsLocation"):
    try:
      if not getattr(sm, "alive", {}).get(service, False):
        continue
      gps = sm[service]
      if getattr(gps, "hasFix", True) is False:
        continue

      lat = _safe_float(getattr(gps, "latitude", None))
      lon = _safe_float(getattr(gps, "longitude", None))
      if lat is None or lon is None:
        continue

      speed_mps = _safe_float(getattr(gps, "speed", None))
      if speed_mps is None:
        try:
          vned = getattr(gps, "vNED", None)
          if vned is not None and len(vned) >= 2:
            v_n = _safe_float(vned[0])
            v_e = _safe_float(vned[1])
            if v_n is not None and v_e is not None:
              speed_mps = float(math.hypot(v_n, v_e))
        except Exception:
          speed_mps = None
      if speed_mps is None:
        speed_mps = 0.0

      bearing_vned = _bearing_from_vned(gps)
      bearing_gps = _safe_float(getattr(gps, "bearingDeg", None))
      bearing = None
      if bearing_vned is not None and float(speed_mps) > 2.0:
        bearing = float(bearing_vned)
      elif bearing_gps is not None:
        bearing = float(bearing_gps)
      elif bearing_vned is not None:
        bearing = float(bearing_vned)
      else:
        bearing = 0.0

      h_acc_m = _safe_float(getattr(gps, "horizontalAccuracy", None))
      return ({
        "latitude": lat,
        "longitude": lon,
        "bearing": bearing,
      }, float(speed_mps), h_acc_m)
    except Exception:
      continue
  return None


def _gps_position_writer_thread(exit_event: threading.Event) -> None:
  params_memory = Params(PARAMS_MEMORY_PATH)
  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
  last_good_payload: dict[str, float] | None = None
  last_good_t = 0.0
  last_written_payload: dict[str, float] | None = None
  last_written_t = 0.0
  last_speed_mps = 0.0

  rk = Ratekeeper(2.0, print_delay_threshold=None)  # 2 Hz is enough for mapd
  while not exit_event.is_set():
    try:
      sm.update(0)
      res = _extract_gps_payload(sm)
      now = time.monotonic()
      chosen = None
      gps_ok_out = False

      speed_mps = float(last_speed_mps)
      if res is not None:
        payload, speed_mps, h_acc_m = res
        last_speed_mps = float(speed_mps)

        base_ok = _gps_quality_ok(float(speed_mps), h_acc_m)
        if base_ok and _gps_jump_suspected(last_written_payload, float(last_written_t), payload, float(now), float(speed_mps)):
          base_ok = False

        chosen = payload
        if base_ok:
          last_good_payload = dict(payload)
          last_good_t = float(now)
          gps_ok_out = True
        else:
          clamped_speed = max(0.1, min(50.0, float(speed_mps)))
          hold_s = min(_GPS_HOLD_LAST_GOOD_S, max(0.5, _GPS_HOLD_MAX_DIST_M / clamped_speed))
          if last_good_payload is not None and (float(now) - float(last_good_t)) < hold_s:
            chosen = last_good_payload
            gps_ok_out = True
          else:
            gps_ok_out = False
      else:
        # No new GPS fix; keep output stable briefly based on last known speed.
        if last_good_payload is not None:
          clamped_speed = max(0.1, min(50.0, float(speed_mps)))
          hold_s = min(_GPS_HOLD_LAST_GOOD_S, max(0.5, _GPS_HOLD_MAX_DIST_M / clamped_speed))
          chosen = last_good_payload
          gps_ok_out = bool((float(now) - float(last_good_t)) < hold_s)

      if chosen is not None:
        params_memory.put("LastGPSPosition", json.dumps(chosen, separators=(",", ":")))
        params_memory.put_bool("GPSQualityOK", bool(gps_ok_out))
        last_written_payload = dict(chosen)
        last_written_t = float(now)
      else:
        params_memory.put_bool("GPSQualityOK", False)
    except Exception:
      cloudlog.exception("mapd: failed updating LastGPSPosition")

    rk.keep_time()


def main() -> None:
  rk = Ratekeeper(0.2, print_delay_threshold=None)  # 5 Hz restart loop
  last_version_check_t = 0.0
  desired_version: str | None = None
  gps_exit = threading.Event()
  threading.Thread(target=_gps_position_writer_thread, args=(gps_exit,), daemon=True).start()

  while True:
    try:
      _cleanup_leftovers()
      now = time.monotonic()
      if now - last_version_check_t > 3600.0 or desired_version is None:
        desired_version = _get_latest_version() or desired_version
        last_version_check_t = now

      if not _ensure_mapd_binary(desired_version):
        time.sleep(10)
        rk.keep_time()
        continue

      cloudlog.info("mapd: starting")
      proc = subprocess.Popen([MAPD_PATH])
      proc.wait()
      cloudlog.warning(f"mapd: exited (code {proc.returncode})")
    except Exception:
      cloudlog.exception("mapd: crashed")

    time.sleep(1)
    rk.keep_time()


if __name__ == "__main__":
  main()
