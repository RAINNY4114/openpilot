#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import usb1
import time
import signal
import subprocess
import threading

from panda import Panda, PandaDFU, PandaProtocolMismatch, FW_PATH
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import sudo_write
from openpilot.selfdrive.pandad.panda_helpers import connect_all_pandas, pandas_include_internal


def system_uptime() -> float:
  """Seconds since system boot, read from /proc/uptime."""
  try:
    with open("/proc/uptime") as f:
      return float(f.read().split()[0])
  except Exception:
    return 0.0


def get_expected_signature(panda: Panda) -> bytes:
  try:
    fn = os.path.join(FW_PATH, panda.get_mcu_type().config.app_fn)
    return Panda.get_signature_from_firmware(fn)
  except Exception:
    cloudlog.exception("Error computing expected signature")
    return b""

def flash_panda(panda_serial: str) -> Panda:
  try:
    panda = Panda(panda_serial)
  except PandaProtocolMismatch:
    cloudlog.warning("detected protocol mismatch, reflashing panda")
    HARDWARE.recover_internal_panda()
    raise

  try:
    fw_signature = get_expected_signature(panda)
    internal_panda = panda.is_internal()

    panda_version = "bootstub" if panda.bootstub else panda.get_version()
    panda_signature = b"" if panda.bootstub else panda.get_signature()
    cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

    if panda.bootstub or panda_signature != fw_signature:
      cloudlog.info("Panda firmware out of date, update required")
      panda.flash()
      cloudlog.info("Done flashing")

    if panda.bootstub:
      bootstub_version = panda.get_version()
      cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
      if internal_panda:
        HARDWARE.recover_internal_panda()
      panda.recover(reset=(not internal_panda))
      cloudlog.info("Done flashing bootstub")

    if panda.bootstub:
      cloudlog.info("Panda still not booting, exiting")
      raise AssertionError

    panda_signature = panda.get_signature()
    if panda_signature != fw_signature:
      cloudlog.info("Version mismatch after flashing, exiting")
      raise AssertionError

    return panda
  except Exception:
    panda.close()
    raise


def flash_all_pandas(panda_serials: list[str]) -> list[Panda]:
  return connect_all_pandas(panda_serials, flash_panda)


LTE_MODEM_PATH = "/sys/bus/usb/devices/1-1.1/authorized"


def disable_lte_modem() -> bool:
  """Deauthorize the internal LTE modem (USB 1-1.1) so it never configures with
  its 500mA request.  The internal hub's per-port budget is 100mA; the modem's
  request trips the hub's over-current protection, which repeatedly drops the
  internal panda off the USB bus during early boot.  Returns True if the modem
  was found and deauthorized."""
  try:
    with open(LTE_MODEM_PATH) as f:
      if f.read().strip() != "1":
        return False
    sudo_write("0", LTE_MODEM_PATH)
    return True
  except (FileNotFoundError, PermissionError, OSError):
    return False


def lte_modem_watchdog() -> None:
  """Early-boot watchdog: keep the internal LTE modem deauthorized during the
  first ~90s of boot so its 500mA config request cannot trip the hub's
  over-current protection and drop the panda off USB.  Runs in a background
  thread so it never delays panda setup."""
  try:
    while system_uptime() < 90:
      if disable_lte_modem():
        cloudlog.info("Internal LTE modem deauthorized (USB hub over-current protection)")
      time.sleep(0.5)
  except Exception:
    pass


def main() -> None:
  # signal pandad to close the relay and exit
  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    if process is not None:
      process.send_signal(signal.SIGINT)

  process = None
  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  # Deauthorize the internal LTE modem early in boot: its 500mA config request
  # trips the internal hub's over-current protection which repeatedly drops the
  # panda off USB during early boot (see disable_lte_modem).
  threading.Thread(target=lte_modem_watchdog, daemon=True).start()

  count = 0
  first_run = True
  params = Params()
  no_internal_panda_count = 0

  while not do_exit:
    pandas: list[Panda] = []
    try:
      count += 1
      cloudlog.event("pandad.flash_and_connect", count=count)
      params.remove("PandaSignatures")

      # Handle missing internal panda
      if no_internal_panda_count > 0:
        # Early-boot window: the internal USB hub (100mA/port) is over its power
        # budget because the LTE modem requests 500mA.  This trips the hub's
        # over-current protection and the internal panda drops off the bus
        # repeatedly for ~40s after boot.  Resetting the panda during this
        # window only re-triggers the protection (each re-enumeration trips it
        # again), so wait for the power to stabilize before resetting.
        if system_uptime() < 45:
          cloudlog.info(f"No pandas found {no_internal_panda_count} times during early boot, waiting for USB power to stabilize before resetting")
          time.sleep(5)
          # Do NOT reset during this window: resetting would re-trigger the hub
          # over-current protection and make the panda drop off repeatedly.
          # Fall through to the list below; if the panda becomes reachable
          # again (USB power stabilized) we connect immediately.
        else:
          if no_internal_panda_count == 3:
            cloudlog.info("No pandas found, putting internal panda into DFU")
            HARDWARE.recover_internal_panda()
          else:
            cloudlog.info("No pandas found, resetting internal panda")
            HARDWARE.reset_internal_panda()
          time.sleep(3)  # wait to come back up

      # Flash all Pandas in DFU mode
      dfu_serials = PandaDFU.list()
      if len(dfu_serials) > 0:
        for serial in dfu_serials:
          cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
          PandaDFU(serial).recover()
        time.sleep(1)

      panda_serials = Panda.list()
      if len(panda_serials) == 0:
        no_internal_panda_count += 1
        continue

      cloudlog.info(f"{len(panda_serials)} panda(s) found, connecting - {panda_serials}")

      # Flash every panda. C3 uses an internal DOS plus a USB red panda, while
      # C3X/C4 normally have a single internal panda.
      pandas = flash_all_pandas(panda_serials)

      # Ensure internal panda is present if expected
      if HARDWARE.has_internal_panda() and not pandas_include_internal(pandas):
        cloudlog.error("Internal panda is missing, trying again")
        no_internal_panda_count += 1
        continue
      no_internal_panda_count = 0

      panda_serials = [panda.get_usb_serial() for panda in pandas]

      # log panda fw versions
      params.put("PandaSignatures", b','.join(panda.get_signature() for panda in pandas))

      # check health for lost heartbeat
      for panda in pandas:
        health = panda.health()
        if health["heartbeat_lost"]:
          params.put_bool("PandaHeartbeatLost", True)
          cloudlog.event("heartbeat lost", deviceState=health, serial=panda.get_usb_serial())
        if health["som_reset_triggered"]:
          params.put_bool("PandaSomResetTriggered", True)
          cloudlog.event("panda.som_reset_triggered", health=health, serial=panda.get_usb_serial())

        if first_run:
          # reset pandas to ensure they're in a good state
          cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
          panda.reset(reconnect=True)
    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
      continue
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
      continue
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")
      continue
    finally:
      for panda in pandas:
        panda.close()

    first_run = False

    # run pandad with all connected serials as arguments
    os.environ['MANAGER_DAEMON'] = 'pandad'
    process = subprocess.Popen(["./pandad", *panda_serials], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
    process.wait()


if __name__ == "__main__":
  main()
