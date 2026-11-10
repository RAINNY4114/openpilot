import os
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog

CHESTNUT_FW_VERSION = "8516b768"
CHESTNUT_USB_IDS = ((0xADD1, 0x0001), (0x3801, 0x0001))
CHESTNUT_ROM_USB_IDS = ((0x174C, 0x2464), (0x174C, 0x2463))
CHESTNUT_USB_PRODUCT = f"custom {CHESTNUT_FW_VERSION}-CLEAN"
USB_DEVICES_PATH = Path("/sys/bus/usb/devices")
TYPEC_CC_ORIENTATION_PATH = Path("/sys/class/power_supply/usb/typec_cc_orientation")
PRIMARY_USB_CONTROLLER = "a600000.ssusb"


def is_chestnut_usb_id(vendor_id: int, product_id: int, include_bootloader: bool = False) -> bool:
  ids = CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS if include_bootloader else CHESTNUT_USB_IDS
  return (vendor_id, product_id) in ids


def get_usb_topology() -> set[str]:
  try:
    return set(os.listdir(USB_DEVICES_PATH))
  except OSError:
    return set()


def read(path: Path) -> str | None:
  try:
    return path.read_text().strip()
  except OSError:
    return None


def read_int(path: Path, base: int = 10) -> int:
  try:
    return int(path.read_text(), base)
  except (OSError, ValueError, TypeError):
    return 0


def _extcon_usb_attached() -> bool | None:
  found = False
  try:
    states = list(Path('/sys/class/extcon').glob('*/state'))
  except OSError:
    return None
  for state in states:
    try:
      txt = state.read_text()
    except OSError:
      continue
    for line in txt.splitlines():
      key, _, val = line.partition('=')
      if key.strip() == 'USB':
        found = True
        if val.strip() == '1':
          return True
  return False if found else None

def cable_connected() -> bool:
  attached = _extcon_usb_attached()
  if attached is not None:
    return attached
  return read_int(TYPEC_CC_ORIENTATION_PATH) != 0


def usb_devices() -> list[Path]:
  try:
    devices = (d for d in USB_DEVICES_PATH.glob("*") if (d / "idVendor").exists())
    return sorted(devices, key=lambda p: p.name)
  except OSError:
    return []


def controller(device: Path) -> Path | None:
  try:
    return next((parent for parent in device.resolve().parents if parent.name.endswith(".ssusb")), None)
  except OSError:
    return None


def get_usb_state() -> list[dict]:
  devices = []
  typec_orientation = read_int(TYPEC_CC_ORIENTATION_PATH)
  for device in usb_devices():
    vendor_id = read_int(device / "idVendor", 16)
    product_id = read_int(device / "idProduct", 16)
    ctrl = controller(device)
    devices.append({
      "busnum": read_int(device / "busnum"),
      "devnum": read_int(device / "devnum"),
      "vendorId": vendor_id,
      "productId": product_id,
      "speedMbps": read_int(device / "speed"),
      "manufacturer": read(device / "manufacturer") or "",
      "product": read(device / "product") or "",
      "linkErrorCount": read_int(ctrl / "portli", 0) & 0xFFFF if ctrl is not None else 0,
      "usb3Lane": {1: "a", 2: "b"}.get(typec_orientation, "unknown") if ctrl is not None and ctrl.name == PRIMARY_USB_CONTROLLER else "unknown",
    })
  return devices


def set_usb_state(device_state, devices: list[dict]) -> None:
  entries = device_state.usbState.init('devices', len(devices))

  chestnut_present = False
  for entry, device in zip(entries, devices, strict=True):
    entry.busnum = device["busnum"]
    entry.devnum = device["devnum"]
    entry.vendorId = device["vendorId"]
    entry.productId = device["productId"]
    entry.speedMbps = device["speedMbps"]
    entry.manufacturer = device["manufacturer"]
    entry.product = device["product"]
    entry.linkErrorCount = device["linkErrorCount"]
    entry.usb3Lane = device.get("usb3Lane", "unknown")

    if is_chestnut_usb_id(entry.vendorId, entry.productId):
      chestnut_present = True

  device_state.chestnutPresent = chestnut_present
  device_state.usbState.connected = cable_connected()

CHESTNUT_PRE_POWER_FILE = Path("/data/ford_params/dp_chestnut_pre_power")
LTSSM_ADDR = 0xB450
LTSSM_TRAINED = 0x78
USB3_MIN_SPEED = 5000
LANE_A_MAX_LINK_ERR = 50

def _speed_mbps(device: Path) -> int:
  try:
    return int(float((device / "speed").read_text().strip()))
  except (OSError, ValueError, TypeError):
    return 0

def pre_power_enabled() -> bool:
  v = read(CHESTNUT_PRE_POWER_FILE)
  return (v or "").strip().lower() not in ("0", "false", "off", "no")

def chestnut_nodes() -> list[dict]:
  typec_orientation = read_int(TYPEC_CC_ORIENTATION_PATH)
  nodes = []
  for device in usb_devices():
    vendor_id = read_int(device / "idVendor", 16)
    product_id = read_int(device / "idProduct", 16)
    if not is_chestnut_usb_id(vendor_id, product_id, include_bootloader=True):
      continue
    ctrl = controller(device)
    link_err = read_int(ctrl / "portli", 0) & 0xFFFF if ctrl is not None else 0
    lane = {1: "a", 2: "b"}.get(typec_orientation, "unknown") if ctrl is not None and ctrl.name == PRIMARY_USB_CONTROLLER else "unknown"
    nodes.append({
      "sysfs": device,
      "vendor_id": vendor_id,
      "product_id": product_id,
      "bus": read_int(device / "busnum"),
      "dev": read_int(device / "devnum"),
      "speed": _speed_mbps(device),
      "product": read(device / "product") or "",
      "usb3Lane": lane,
      "linkErrorCount": link_err,
    })
  return nodes

def node_openable(bus: int, dev: int) -> bool:
  try:
    os.close(os.open(f"/dev/bus/usb/{bus:03d}/{dev:03d}", os.O_RDWR))
    return True
  except OSError:
    return False

def chestnut_ltssm(vendor_id: int, product_id: int) -> int:
  try:
    import usb1
    with usb1.USBContext() as ctx:
      handle = ctx.openByVendorIDAndProductID(vendor_id, product_id, skip_on_error=True)
      if handle is None:
        return -1
      try:
        return bytes(handle.controlRead(0xC0, 0xE4, LTSSM_ADDR, 0, 1, timeout=1000))[0]
      finally:
        try:
          handle.close()
        except Exception:
          pass
  except Exception:
    return -1

def chestnut_pre_power(vendor_id: int, product_id: int, force: bool = False) -> bool:
  try:
    import usb1
  except Exception as e:
    cloudlog.warning(f"[CHX_USB_ENUM] pre-power: usb1 不可用 ({e})")
    return False
  try:
    with usb1.USBContext() as ctx:
      handle = ctx.openByVendorIDAndProductID(vendor_id, product_id, skip_on_error=True)
      if handle is None:
        cloudlog.warning("[CHX_USB_ENUM] pre-power: 打不开设备（权限不足或已被占用）")
        return False
      try:
        ltssm = -1
        try:
          ltssm = bytes(handle.controlRead(0xC0, 0xE4, LTSSM_ADDR, 0, 1, timeout=1000))[0]
          cloudlog.warning(f"[CHX_USB_ENUM] pre-power: 上电前 LTSSM=0x{ltssm:02X}")
        except Exception as e:
          cloudlog.warning(f"[CHX_USB_ENUM] pre-power: 读 LTSSM 失败 ({e})，仍然尝试上电")
        if ltssm == LTSSM_TRAINED and not force:
          cloudlog.warning("[CHX_USB_ENUM] pre-power: 链路已训练，无需上电")
          return True
        if ltssm == LTSSM_TRAINED:
          cloudlog.warning("[CHX_USB_ENUM] pre-power: LTSSM=0x78 但 USB 仍在 USB2 侧，"
                           "强制发 0xF3 触发重枚举")
        handle.controlWrite(0x40, 0xF3, 1, 0, b"", timeout=5000)
        cloudlog.warning("[CHX_USB_ENUM] pre-power: 已发 0xF3(PCIe 上电)，等待重枚举到 USB3")
        return True
      finally:
        try:
          handle.close()
        except Exception:
          pass
  except Exception as e:
    cloudlog.warning(f"[CHX_USB_ENUM] pre-power 失败: {e}")
    return False

def _chestnut_usb_reset(vendor_id: int, product_id: int) -> bool:
  try:
    import usb1
  except Exception:
    return False
  try:
    with usb1.USBContext() as ctx:
      handle = ctx.openByVendorIDAndProductID(vendor_id, product_id, skip_on_error=True)
      if handle is None:
        return False
      try:
        handle.resetDevice()
        cloudlog.warning('[CHX_USB_ENUM] 0xF3 未生效，已执行 USB reset 强制重新枚举')
        return True
      finally:
        try:
          handle.close()
        except Exception:
          pass
  except Exception as e:
    cloudlog.warning(f'[CHX_USB_ENUM] USB reset 失败: {e}')
    return False

def chestnut_usb_node_ready(timeout: float = 60.0, power_timeout: float = 40.0,
                            poll: float = 0.5, min_speed: int = USB3_MIN_SPEED) -> bool:
  t0 = time.monotonic()
  seen_any = False
  powered = False
  reset_done = False

  while True:
    for node in chestnut_nodes():
      if not node_openable(node["bus"], node["dev"]):
        continue

      seen_any = True
      speed = node["speed"]
      ltssm = chestnut_ltssm(node["vendor_id"], node["product_id"]) if speed >= min_speed else -2

      if speed >= min_speed and ltssm in (-1, LTSSM_TRAINED):
        lane = node.get('usb3Lane', 'unknown')
        link_err = node.get('linkErrorCount', 0)
        if lane == 'a' and link_err > LANE_A_MAX_LINK_ERR and not reset_done:
          cloudlog.warning('[LANE_CHECK] lane=a linkErr=%d>%d, USB reset' % (link_err, LANE_A_MAX_LINK_ERR))
          _chestnut_usb_reset(node['vendor_id'], node['product_id'])
          reset_done = True
          time.sleep(2.0)
          continue
        if lane == 'a' and link_err > LANE_A_MAX_LINK_ERR and reset_done:
          cloudlog.warning('[LANE_CHECK] lane=a still unstable after reset, flip USB-C cable')
        if ltssm == LTSSM_TRAINED:
          cloudlog.warning('[CHX_USB_ENUM] USB3 trained bus=%03d/%03d speed=%d lane=%s linkErr=%d' % (node['bus'], node['dev'], speed, lane, link_err))
        else:
          cloudlog.warning('[CHX_USB_ENUM] USB3 LTSSM unreadable speed=%d lane=%s' % (speed, lane))
        return True

      if not powered:
        powered = True
        if pre_power_enabled():
          _lt = f"LTSSM=0x{ltssm & 0xFF:02X}" if ltssm >= 0 else "LTSSM 未读(USB2 侧)"
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut speed={speed}Mbps {_lt} -> 主动给 PCIe 上电")
          chestnut_pre_power(node["vendor_id"], node["product_id"],
                             force=speed < min_speed)
        else:
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut speed={speed}Mbps 需要上电，"
                           f"但 dp_chestnut_pre_power=0 已禁用")

    elapsed = time.monotonic() - t0

    if powered and not reset_done and elapsed > 15.0:
      reset_done = True
      for _n in chestnut_nodes():
        if _chestnut_usb_reset(_n['vendor_id'], _n['product_id']):
          break

    limit = power_timeout if powered else timeout
    if elapsed > limit:
      if not seen_any:
        cloudlog.warning(f"[CHX_USB_ENUM] {elapsed:.0f}s 内没看到可写的 chestnut 节点 —— "
                         f"检查线缆是否插好；节点权限由 system/egpu_usb_perm.sh 负责放开")
      else:
        cloudlog.warning(f"[CHX_USB_ENUM] 上电后 {elapsed:.0f}s 仍未回到 USB3 侧 -> 大模型跳过。"
                         f"检查线缆/接口是否支持 USB3/USB4，或跑 "
                         f"/usr/local/venv/bin/python3 /data/tools/chestnut_power_on.py 单独诊断")
      return False

    time.sleep(poll)
