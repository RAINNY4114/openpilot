
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware.usb import controller, is_chestnut_usb_id, read_int, usb_devices

STABLE_SECONDS = 2.0
STABLE_THRESHOLD = 5.0

def _chestnut_portli() -> Path | None:
  for device in usb_devices():
    usb_id = (read_int(device / "idVendor", 16), read_int(device / "idProduct", 16))
    if is_chestnut_usb_id(*usb_id):
      ctrl = controller(device)
      if ctrl is not None and (ctrl / "portli").exists():
        return ctrl / "portli"
  return None

def wait_usbgpu_link(timeout: float = 30.0, threshold: float = STABLE_THRESHOLD) -> bool:
  portli = _chestnut_portli()
  if portli is None:
    cloudlog.warning("usbgpu_link: chestnut 不在总线上，跳过稳定性等待")
    return False

  t0 = time.monotonic()
  rate = 0.0
  while time.monotonic() - t0 < timeout:
    start = read_int(portli, 0)
    time.sleep(STABLE_SECONDS)
    rate = (read_int(portli, 0) - start) / STABLE_SECONDS
    if rate <= threshold:
      cloudlog.warning(f"usbgpu_link: stable ({rate:.0f} err/s) after {time.monotonic() - t0:.1f}s")
      return True
    cloudlog.warning(f"usbgpu_link: not stable, {rate:.0f} err/s")
  cloudlog.error(f"usbgpu_link: never stabilized in {timeout:.0f}s (last {rate:.0f} err/s)")
  return False
