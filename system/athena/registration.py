#!/usr/bin/env python3
from pathlib import Path
import os
import hashlib

import requests

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.hw import Paths


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

# Activation auth source (server-first): if server is unreachable, fallback to local SN whitelist.
ACTIVATION_USE_SERVER = os.getenv("ACTIVATION_USE_SERVER", "1") == "1"
ACTIVATION_BASE_URL = os.getenv(
  "ACTIVATION_BASE_URL",
  os.getenv("REMOTE_AGENT_BASE_URL", "https://cp.zh182.cn"),
).rstrip("/")
ACTIVATION_TIMEOUT_SEC = max(float(os.getenv("ACTIVATION_TIMEOUT_SEC", "8")), 1.0)

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
    # 1) Primary path: server authorization decides activation directly.
    server_status = _check_server_license_status(serial, sw_version, spinner)
    if server_status is not None:
      if server_status == "authorized":
        return _set_authorized(params, serial)
      return _set_unregistered(params, serial)

    # 2) Fallback path: server unreachable -> local whitelist fallback.
    if spinner is not None:
      spinner.update(f"registering device - serial: {serial}, server unavailable, fallback whitelist")

    if serial in whitelist:
      return _set_authorized(params, serial)

    return _set_unregistered(params, serial)
  finally:
    if spinner is not None:
      spinner.close()


if __name__ == "__main__":
  print(register())
