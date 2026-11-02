"""[EGPU_LINK_LOADING 20261027]
usbgpu_link.py —— 等 eGPU 的 USB SuperSpeed 链路稳定后再加载大模型

来源：commaai/openpilot commit 24a9b6dae5 "profile usb gpu link stability (#38368)"。
适配：本分支 usb.py 用 CHESTNUT_USB_IDS / is_chestnut_usb_id()，没有上游的
      CHESTNUT_VENDOR_ID / CHESTNUT_PRODUCT_ID，故改成按 ID 集合判断。

为什么需要：ASM2464 的 SuperSpeed 链路刚枚举的几秒内会持续报 link error
（/sys/.../a600000.ssusb/portli 计数上涨）。此时直接开 AMD 设备会得到
`bulk IN 0x81 failed: Operation timed out`（2026-10-15 实测）。
"""

import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware.usb import controller, is_chestnut_usb_id, read_int, usb_devices

STABLE_SECONDS = 2.0
STABLE_THRESHOLD = 5.0  # link errors per second


def _chestnut_portli() -> Path | None:
  for device in usb_devices():
    usb_id = (read_int(device / "idVendor", 16), read_int(device / "idProduct", 16))
    if is_chestnut_usb_id(*usb_id):
      ctrl = controller(device)
      if ctrl is not None and (ctrl / "portli").exists():
        return ctrl / "portli"
  return None


def wait_usbgpu_link(timeout: float = 30.0, threshold: float = STABLE_THRESHOLD) -> bool:
  """等链路错误率降到 threshold 以下。

  返回 True=已稳定，False=超时或设备不在总线上。
  **永不抛异常** —— 调用点在加载路径上，抛错只会白吃重试预算。
  """
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
