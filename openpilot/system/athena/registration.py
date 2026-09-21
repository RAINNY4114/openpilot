#!/usr/bin/env python3

import time
import json
import jwt

from typing import cast
from pathlib import Path
from datetime import datetime, timedelta, UTC

from openpilot.common.api import api_get, get_key_pair
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

REGISTRATION_TIMEOUT_S = 60.0
REGISTRATION_REQUEST_TIMEOUT_S = 15
REGISTRATION_MAX_BACKOFF_S = 15


def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  Register device with pilotauth.

  Important:
  - Reuse an existing DongleId when available.
  - Reuse /persist/comma/dongle_id when available.
  - Obtain C3X IMEI information through the current carrot
    HARDWARE API.
  - Do not block manager startup indefinitely if registration
    cannot be completed.
  """

  params = Params()

  dongle_id: str | None = params.get("DongleId")

  # ------------------------------------------------------------
  # Existing persistent DongleId
  # ------------------------------------------------------------
  persist_dongle_path = Path(
    Paths.persist_root() + "/comma/dongle_id"
  )

  if dongle_id is None and persist_dongle_path.is_file():
    try:
      with open(persist_dongle_path) as f:
        dongle_id = f.read().strip()

      if dongle_id == "":
        dongle_id = None

    except Exception:
      cloudlog.exception(
        f"failed to read persistent dongle id: {persist_dongle_path}"
      )

  # ------------------------------------------------------------
  # Create registration token
  # ------------------------------------------------------------
  jwt_algo, private_key, public_key = get_key_pair()

  if not public_key:
    dongle_id = UNREGISTERED_DONGLE_ID
    cloudlog.warning("missing public key")

  elif dongle_id is None:

    spinner = None

    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    # ----------------------------------------------------------
    # Get hardware serial
    # ----------------------------------------------------------
    serial = HARDWARE.get_serial()

    # ----------------------------------------------------------
    # Get IMEI(s)
    #
    # Keep the carrot/C3X hardware interface:
    #   get_imei(0)
    #   get_imei(1)
    #
    # This is intentionally not changed to the SP
    # get_imei() API.
    # ----------------------------------------------------------
    imei1: str | None = None
    imei2: str | None = None

    imei_start = time.monotonic()

    while imei1 is None and imei2 is None:
      try:
        imei1 = HARDWARE.get_imei(0)
      except Exception:
        cloudlog.exception(
          "Error getting IMEI1, trying again..."
        )

      try:
        imei2 = HARDWARE.get_imei(1)
      except Exception:
        cloudlog.exception(
          "Error getting IMEI2, trying again..."
        )

      # --------------------------------------------------------
      # Do not allow IMEI acquisition to block boot forever.
      # --------------------------------------------------------
      if time.monotonic() - imei_start > REGISTRATION_TIMEOUT_S:
        cloudlog.warning(
          "Timed out waiting for device IMEI, "
          "continuing as unregistered device"
        )

        if spinner is not None:
          spinner.update(
            f"registration timeout - serial: {serial}"
          )
          spinner.close()

        return UNREGISTERED_DONGLE_ID

      if imei1 is None and imei2 is None:
        time.sleep(1)

    if show_spinner and spinner is not None:
      spinner.update(
        f"registering device - serial: {serial}, "
        f"IMEI: ({imei1}, {imei2})"
      )

    # ----------------------------------------------------------
    # pilotauth registration
    # ----------------------------------------------------------
    registration_start = time.monotonic()
    backoff = 0

    while True:
      try:
        register_token = jwt.encode(
          {
            "register": True,
            "exp": datetime.now(UTC).replace(tzinfo=None)
                   + timedelta(hours=1),
          },
          cast(str, private_key),
          algorithm=jwt_algo,
        )

        cloudlog.info("getting pilotauth")

        resp = api_get(
          "v2/pilotauth/",
          method="POST",
          timeout=REGISTRATION_REQUEST_TIMEOUT_S,
          imei=imei1,
          imei2=imei2,
          serial=serial,
          public_key=public_key,
          register_token=register_token,
        )

        if resp.status_code in (402, 403):
          cloudlog.info(
            f"Unable to register device, got {resp.status_code}"
          )
          dongle_id = UNREGISTERED_DONGLE_ID

        else:
          dongleauth = json.loads(resp.text)
          dongle_id = dongleauth["dongle_id"]

        break

      except NotImplementedError:
        # Do not enter an endless retry loop when the JWT
        # dependency/API is not available.
        cloudlog.exception(
          "Registration dependency is not implemented"
        )

        if spinner is not None:
          spinner.close()

        return UNREGISTERED_DONGLE_ID

      except Exception:
        cloudlog.exception("failed to authenticate")

        backoff = min(
          backoff + 1,
          REGISTRATION_MAX_BACKOFF_S,
        )

        elapsed = time.monotonic() - registration_start

        # ------------------------------------------------------
        # SP-style boot safety:
        # never block manager startup indefinitely.
        # ------------------------------------------------------
        if elapsed >= REGISTRATION_TIMEOUT_S:
          cloudlog.warning(
            f"registration timed out after {elapsed:.1f}s, "
            "continuing as unregistered device"
          )

          dongle_id = UNREGISTERED_DONGLE_ID
          break

        if spinner is not None:
          spinner.update(
            f"registering device - serial: {serial}, "
            f"IMEI: ({imei1}, {imei2})"
          )

        time.sleep(backoff)

    if spinner is not None:
      spinner.close()

  # ------------------------------------------------------------
  # Save DongleId
  # ------------------------------------------------------------
  if dongle_id:
    params.put(
      "DongleId",
      dongle_id,
      block=True,
    )

    # Preserve SP behavior:
    # notify the UI when the device is unregistered.
    try:
      set_offroad_alert(
        "Offroad_UnregisteredHardware",
        (dongle_id == UNREGISTERED_DONGLE_ID) and not PC,
      )
    except Exception:
      cloudlog.exception(
        "failed to set Offroad_UnregisteredHardware alert"
      )

  return dongle_id


if __name__ == "__main__":
  print(register())
