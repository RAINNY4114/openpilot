#!/usr/bin/env python3
import json
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

VERSION_CHANNEL = "v1"
GITHUB_VERSION_URL = f"https://github.com/FrogAi/FrogPilot-Resources/raw/Versions/mapd_version_{VERSION_CHANNEL}.txt"
GITLAB_VERSION_URL = f"https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Versions/mapd_version_{VERSION_CHANNEL}.txt"
FALLBACK_MAPD_VERSION = "v1.9.1"

MAPD_PATH = "/data/media/0/osm/mapd"
MAPD_VERSION_PATH = "/data/media/0/osm/mapd_version"
PARAMS_MEMORY_PATH = "/dev/shm/params"

DOWNLOAD_URLS = [
  # Primary (works today, GitHub may redirect between repos)
  "https://github.com/pfeiferj/mapd/releases/download/{version}/mapd",
  "https://github.com/pfeiferj/openpilot-mapd/releases/download/{version}/mapd",
  # Mirror used by FrogPilot (public raw file)
  "https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/Mapd/{version}",
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


def _get_latest_version() -> str | None:
  for url in (GITHUB_VERSION_URL, GITLAB_VERSION_URL):
    try:
      with urllib.request.urlopen(url, timeout=10) as resp:
        version = resp.read().decode("utf-8").strip()
        if version:
          return version
    except (urllib.error.URLError, socket.timeout):
      continue
    except Exception:
      cloudlog.exception("mapd: failed to fetch latest version")
  return None


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


def _gps_position_writer_thread(exit_event: threading.Event) -> None:
  params_memory = Params(PARAMS_MEMORY_PATH)
  sm = messaging.SubMaster(["gpsLocationExternal"])

  rk = Ratekeeper(2.0, print_delay_threshold=None)  # 2 Hz is enough for mapd
  while not exit_event.is_set():
    try:
      sm.update(0)
      gps = sm["gpsLocationExternal"]
      if getattr(gps, "hasFix", False):
        payload = {
          "latitude": float(gps.latitude),
          "longitude": float(gps.longitude),
          "bearing": float(gps.bearingDeg),
        }
        params_memory.put("LastGPSPosition", json.dumps(payload, separators=(",", ":")))
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
