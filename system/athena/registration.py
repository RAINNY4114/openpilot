#!/usr/bin/env python3
from pathlib import Path
import os
import hashlib
import time

import requests

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.hw import Paths


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

# Activation auth source (whitelist-first): local SN whitelist bypasses server;
# non-whitelisted devices must be authorized by server.
ACTIVATION_USE_SERVER = os.getenv("ACTIVATION_USE_SERVER", "1") == "1"
ACTIVATION_BASE_URL = os.getenv(
  "ACTIVATION_BASE_URL",
  os.getenv("REMOTE_AGENT_BASE_URL", "https://cp.zh182.cn"),
).rstrip("/")
ACTIVATION_TIMEOUT_SEC = max(float(os.getenv("ACTIVATION_TIMEOUT_SEC", "8")), 1.0)
ACTIVATION_SERVER_RETRY_WINDOW_SEC = max(float(os.getenv("ACTIVATION_SERVER_RETRY_WINDOW_SEC", "45")), 0.0)
ACTIVATION_SERVER_RETRY_INTERVAL_SEC = max(float(os.getenv("ACTIVATION_SERVER_RETRY_INTERVAL_SEC", "2")), 0.2)
ACTIVATION_LOCK_UNREGISTERED = os.getenv("ACTIVATION_LOCK_UNREGISTERED", "1") == "1"
ACTIVATION_RECHECK_INTERVAL_SEC = max(float(os.getenv("ACTIVATION_RECHECK_INTERVAL_SEC", "5")), 1.0)
ACTIVATION_CACHE_FILE = Path("/data/openpilot_cache/activation_authorized_serial")

_VALID_LICENSE_STATUS = {"authorized", "blocked", "expired", "pending"}


def _read_serial_whitelist() -> set[str]:
  repo_path = Path(BASEDIR) / "system" / "athena" / "serial_whitelist.txt"

  if not repo_path.is_file():
    return set()

  serials: set[str] = set()
  with open(repo_path) as f:
    for line in f:
      serial = line.strip()
      if serial:
        serials.add(serial)
  return serials


def _read_persist_dongle_id() -> str:
  dongle_id_path = Path(Paths.persist_root()) / "comma" / "dongle_id"
  if not dongle_id_path.is_file():
    return ""
  try:
    return dongle_id_path.read_text().strip()
  except Exception:
    return ""


def _read_cached_authorized_serial() -> str:
  if not ACTIVATION_CACHE_FILE.is_file():
    return ""
  try:
    return ACTIVATION_CACHE_FILE.read_text().strip()
  except Exception:
    return ""


def _write_cached_authorized_serial(serial: str) -> None:
  try:
    ACTIVATION_CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
    ACTIVATION_CACHE_FILE.write_text(serial + "\n")
  except Exception:
    pass


def _clear_cached_authorized_serial() -> None:
  try:
    if ACTIVATION_CACHE_FILE.is_file():
      ACTIVATION_CACHE_FILE.unlink()
  except Exception:
    pass


def _build_local_dongle_id(serial: str) -> str:
  # Keep it stable and non-reversible from UI perspective (avoid exposing raw serial as DongleId).
  digest = hashlib.sha256(serial.encode("utf-8", errors="ignore")).hexdigest()[:16]
  return f"local_{digest}"


def _resolve_authorized_dongle_id(params: Params, serial: str) -> str:
  existing = params.get("DongleId")
  if isinstance(existing, bytes):
    existing = existing.decode("utf-8", errors="ignore")
  elif existing is None:
    existing = ""
  else:
    existing = str(existing)

  # Preserve existing non-empty, non-unregistered, non-raw-serial DongleId.
  if existing and existing != UNREGISTERED_DONGLE_ID and existing != serial:
    return existing

  persist_dongle_id = _read_persist_dongle_id()
  if persist_dongle_id:
    return persist_dongle_id

  return _build_local_dongle_id(serial)


def _set_authorized(params: Params, serial: str) -> str:
  dongle_id = _resolve_authorized_dongle_id(params, serial)
  params.put("DongleId", dongle_id)
  set_offroad_alert("Offroad_UnregisteredHardware", False)
  return dongle_id


def _set_unregistered(params: Params, serial: str) -> str:
  params.put("DongleId", UNREGISTERED_DONGLE_ID)
  set_offroad_alert("Offroad_UnregisteredHardware", True, extra_text=serial)
  return UNREGISTERED_DONGLE_ID


def _check_server_license_status(serial: str, sw_version: str, spinner: Spinner | None) -> str | None:
  if not ACTIVATION_USE_SERVER or not ACTIVATION_BASE_URL:
    return None

  endpoint = f"{ACTIVATION_BASE_URL}/api/v1/device/register"
  payload = {
    "serial": serial,
    "device_fingerprint": serial,
    "sw_version": sw_version,
  }

  try:
    if spinner is not None:
      spinner.update(f"registering device - serial: {serial}, server auth check")

    resp = requests.post(endpoint, json=payload, timeout=ACTIVATION_TIMEOUT_SEC)
    resp.raise_for_status()
    data = resp.json()
    if not isinstance(data, dict):
      raise ValueError(f"invalid response payload: {type(data)}")

    status = str(data.get("license_status", "pending")).strip().lower()
    if status not in _VALID_LICENSE_STATUS:
      status = "pending"

    return status
  except Exception:
    return None


def _wait_until_authorized(params: Params, serial: str, sw_version: str, spinner: Spinner) -> str:
  while True:
    if serial in _read_serial_whitelist():
      return _set_authorized(params, serial)

    server_status = _check_server_license_status(serial, sw_version, spinner)
    if server_status == "authorized":
      _write_cached_authorized_serial(serial)
      return _set_authorized(params, serial)

    if server_status is not None:
      _clear_cached_authorized_serial()

    spinner.update(f"registering device - serial: {serial}, contact ZH")
    time.sleep(ACTIVATION_RECHECK_INTERVAL_SEC)


def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner: bool = False) -> str:
  params = Params()
  serial = HARDWARE.get_serial()
  sw_version_raw = params.get("Version")
  if isinstance(sw_version_raw, bytes):
    sw_version = sw_version_raw.decode("utf-8", errors="ignore")
  elif sw_version_raw is None:
    sw_version = ""
  else:
    sw_version = str(sw_version_raw)
  whitelist = _read_serial_whitelist()

  spinner: Spinner | None = Spinner() if show_spinner else None
  if spinner is not None:
    spinner.update(f"registering device - serial: {serial}")

  try:
    # 1) Offline cache first: previously authorized device can boot directly.
    if _read_cached_authorized_serial() == serial:
      if spinner is not None:
        spinner.update(f"registering device - serial: {serial}, offline authorized cache")
      return _set_authorized(params, serial)

    # 2) Local bypass path: if SN is whitelisted on device, authorize immediately.
    if serial in whitelist:
      if spinner is not None:
        spinner.update(f"registering device - serial: {serial}, whitelist bypass")
      return _set_authorized(params, serial)

    # 3) Non-whitelisted and cache-miss devices must be authorized by server.
    server_status: str | None = None
    if ACTIVATION_USE_SERVER and ACTIVATION_BASE_URL:
      deadline = time.monotonic() + ACTIVATION_SERVER_RETRY_WINDOW_SEC
      while True:
        server_status = _check_server_license_status(serial, sw_version, spinner)
        if server_status is not None:
          break

        now = time.monotonic()
        if now >= deadline:
          break

        if spinner is not None:
          remaining_sec = int(max(deadline - now, 0))
          spinner.update(f"registering device - serial: {serial}, waiting auth server ({remaining_sec}s)")

        sleep_sec = min(ACTIVATION_SERVER_RETRY_INTERVAL_SEC, max(deadline - now, 0))
        if sleep_sec > 0:
          time.sleep(sleep_sec)

    if server_status == "authorized":
      _write_cached_authorized_serial(serial)
      return _set_authorized(params, serial)

    # Server returned non-authorized status, clear cache.
    if server_status is not None:
      _clear_cached_authorized_serial()

    if spinner is not None:
      spinner.update(f"registering device - serial: {serial}, contact ZH")
    _set_unregistered(params, serial)

    if spinner is not None and ACTIVATION_LOCK_UNREGISTERED:
      return _wait_until_authorized(params, serial, sw_version, spinner)

    return UNREGISTERED_DONGLE_ID
  finally:
    if spinner is not None:
      spinner.close()


if __name__ == "__main__":
  print(register())
