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
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

# Hard timeout for the whole registration operation.
# This prevents manager.py from being blocked forever when IMEI or
# backend registration is unavailable.
REGISTRATION_TIMEOUT_S = 60.0
REGISTRATION_REQUEST_TIMEOUT_S = 15
REGISTRATION_MAX_BACKOFF_S = 15


def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  Keep the original carrot/fishop registration flow and hardware API,
  while adding a hard timeout so manager.py can continue booting when
  IMEI acquisition or backend registration is unavailable.

  All devices built since March 2024 come with all
  info stored in /persist/. This is kept around
  only for devices built before then.

  With a backend update to take serial number instead
  of dongle ID to some endpoints, this can be removed
  entirely.
  """
  params = Params()

  dongle_id: str | None = params.get("DongleId")
  if dongle_id is None and Path(Paths.persist_root() + "/comma/dongle_id").is_file():
    with open(Paths.persist_root() + "/comma/dongle_id") as f:
      dongle_id = f.read().strip()

  # Create registration token, in the future, this key will make JWTs directly
  jwt_algo, private_key, public_key = get_key_pair()

  if not public_key:
    dongle_id = UNREGISTERED_DONGLE_ID
    cloudlog.warning("missing public key")

  elif dongle_id is None:
    spinner = None
    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    serial = HARDWARE.get_serial()

    # Step 1: obtain IMEI(s), but never block manager startup forever.
    imei1: str | None = None
    imei2: str | None = None
    start_time = time.monotonic()

    while imei1 is None and imei2 is None:
      try:
        imei1, imei2 = HARDWARE.get_imei(0), HARDWARE.get_imei(1)
      except NotImplementedError:
        cloudlog.warning(
          "HARDWARE.get_imei is not implemented, continuing as unregistered"
        )
        break
      except Exception:
        cloudlog.exception("Error getting imei, trying again...")

      if imei1 is not None or imei2 is not None:
        break

      elapsed = time.monotonic() - start_time
      if elapsed >= REGISTRATION_TIMEOUT_S:
        cloudlog.warning(
          f"IMEI acquisition timed out after {elapsed:.1f}s, continuing as unregistered"
        )
        break

      if spinner is not None:
        spinner.update(
          f"registering device - serial: {serial}, IMEI: ({imei1}, {imei2})"
        )

      time.sleep(1)

    # No IMEI after the hard timeout: do not enter the network retry loop.
    if imei1 is None and imei2 is None:
      dongle_id = UNREGISTERED_DONGLE_ID

    # Step 2: register with pilotauth, with a hard overall timeout.
    if dongle_id is None:
      backoff = 0
      start_time = time.monotonic()

      while True:
        try:
          register_token = jwt.encode(
            {
              "register": True,
              "exp": datetime.now(UTC).replace(tzinfo=None) + timedelta(hours=1),
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
            cloudlog.info(f"Unable to register device, got {resp.status_code}")
            dongle_id = UNREGISTERED_DONGLE_ID
          else:
            dongleauth = json.loads(resp.text)
            dongle_id = dongleauth["dongle_id"]

          break

        except NotImplementedError:
          cloudlog.warning(
            "Device registration API is not implemented, continuing as unregistered"
          )
          dongle_id = UNREGISTERED_DONGLE_ID
          break

        except Exception:
          cloudlog.exception("failed to authenticate")
          backoff = min(backoff + 1, REGISTRATION_MAX_BACKOFF_S)

          elapsed = time.monotonic() - start_time
          if elapsed >= REGISTRATION_TIMEOUT_S:
            cloudlog.warning(
              f"Device registration timed out after {elapsed:.1f}s, "
              "continuing as unregistered"
            )
            dongle_id = UNREGISTERED_DONGLE_ID
            break

          if spinner is not None:
            spinner.update(
              f"registering device - serial: {serial}, "
              f"IMEI: ({imei1}, {imei2})"
            )

          remaining = max(0.0, REGISTRATION_TIMEOUT_S - elapsed)
          time.sleep(min(backoff, remaining))

    if spinner is not None:
      spinner.close()

  if dongle_id:
    # Blocking persistence ensures manager sees the value before continuing.
    params.put("DongleId", dongle_id, block=True)

    # Keep the original carrot behavior; do not change alert semantics here.
    # set_offroad_alert(
    #   "Offroad_UnofficialHardware",
    #   (dongle_id == UNREGISTERED_DONGLE_ID) and not PC,
    # )

  return dongle_id


if __name__ == "__main__":
  print(register())
