#!/usr/bin/env python3
import datetime
import json
import os
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

OSM_OFFLINE_DIR = "/data/media/0/osm/offline"
PARAMS_MEMORY_PATH = "/dev/shm/params"

SLEEP_IDLE_SEC = 10 * 60
SLEEP_WAIT_DOWNLOAD_SEC = 60


def _day_suffix(day: int) -> str:
  if day % 10 == 1 and day != 11:
    return "st"
  if day % 10 == 2 and day != 12:
    return "nd"
  if day % 10 == 3 and day != 13:
    return "rd"
  return "th"


def _format_current_date(now: datetime.datetime) -> str:
  suffix = _day_suffix(now.day)
  return now.strftime(f"%B {now.day}{suffix}, %Y")


def _offline_maps_exist() -> bool:
  try:
    for root, _, files in os.walk(OSM_OFFLINE_DIR):
      for name in files:
        _ = os.path.join(root, name)
        return True
  except Exception:
    return False
  return False


def _get_maps_selected(params: Params) -> dict | None:
  raw = params.get("MapsSelected")
  if not raw:
    return None
  try:
    obj = json.loads(raw)
  except Exception:
    return None

  if isinstance(obj, int):
    params.remove("MapsSelected")
    return None
  if not isinstance(obj, dict):
    return None

  nations = obj.get("nations") or []
  states = obj.get("states") or []
  if not (nations or states):
    return None

  obj.setdefault("nations", [])
  obj.setdefault("states", [])
  return obj


def _schedule_due(now: datetime.datetime, schedule: int) -> bool:
  if schedule == 1:  # Weekly
    return now.weekday() == 6  # Sunday
  if schedule == 2:  # Monthly
    return now.day == 1
  return False


def main() -> None:
  params = Params()
  params_memory = Params(PARAMS_MEMORY_PATH)

  while True:
    try:
      if params.get_bool("IsOnroad"):
        time.sleep(SLEEP_IDLE_SEC)
        continue

      now = datetime.datetime.now(datetime.timezone.utc).astimezone()
      schedule = int(params.get("PreferredSchedule") or 0)

      # 0 = Manually
      if schedule <= 0:
        time.sleep(SLEEP_IDLE_SEC)
        continue

      maps_selected = _get_maps_selected(params)
      if maps_selected is None:
        time.sleep(SLEEP_IDLE_SEC)
        continue

      maps_downloaded = _offline_maps_exist()
      if maps_downloaded and not _schedule_due(now, schedule):
        time.sleep(SLEEP_IDLE_SEC)
        continue

      todays_date = _format_current_date(now)
      if maps_downloaded and (params.get("LastMapsUpdate") == todays_date):
        time.sleep(SLEEP_IDLE_SEC)
        continue

      # If a download is already running, just wait.
      if params.get("OSMDownloadProgress"):
        time.sleep(SLEEP_WAIT_DOWNLOAD_SEC)
        continue

      if params_memory.get("OSMDownloadLocations"):
        time.sleep(SLEEP_WAIT_DOWNLOAD_SEC)
        continue

      cloudlog.info("maps_updater: starting scheduled offline maps download")
      params_memory.put("OSMDownloadLocations", json.dumps(maps_selected, separators=(",", ":")))

      # Wait for download to start (avoid race where progress isn't written yet).
      start_t = time.monotonic()
      while (params.get("OSMDownloadProgress") is None and
             params_memory.get("OSMDownloadLocations") and
             (time.monotonic() - start_t) < 5 * 60):
        time.sleep(5)

      # Wait until download finishes.
      while params.get("OSMDownloadProgress") is not None:
        time.sleep(SLEEP_WAIT_DOWNLOAD_SEC)

      # Some mapd builds may not clear this on completion.
      params_memory.remove("OSMDownloadLocations")

      if _offline_maps_exist():
        params.put_nonblocking("LastMapsUpdate", todays_date)
    except Exception:
      cloudlog.exception("maps_updater: error")
      time.sleep(60)


if __name__ == "__main__":
  main()
