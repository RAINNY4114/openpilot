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
  """qcom extcon 里当前的 USB 连接位。

  多个 extcon 都带 USB 属性（usb-pdphy / SMB2 / eud），只要有一条是 1 就算连上；
  全是 0 表示当前没有插任何 USB 设备。读不到 extcon 时返回 None 让调用方兜底。
  """
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
  # [port-egpu-usb] TYPEC_CC_ORIENTATION 是粘滞值：线拔掉后内核不清零
  # （实测拔掉底座后仍是 2）。只看它会让 CHESTNUT 分支永远成立 -> 每次 onroad 都要
  # 白等 wait_for_chestnut() 超时，然后置 ChestnutRetry=1，下次也不再自动重试。
  # 优先用 extcon 的 USB 连接位判断「现在到底插没插」。
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


# ---------------------------------------------------------------------------
# [CHX_USB_ENUM 20261027] chestnut (ASM2464) USB 枚举守卫
#
# 本机实测有两道坎，缺任意一道都表现为 "chestnut load failed" -> 静默回退小模型：
#
#   ① 权限：devtmpfs 以 0600 root:root 建节点，udev 要读完描述符才按
#      50-udev-default.rules 放宽成 0664。eGPU 与车机同时上电时 ASM2464 描述符
#      响应慢，这个窗口可达数十秒；modeld 以 comma 身份在 T+34s 就 libusb_open
#      -> EACCES。comma 在 root 组，0664 就够，所以由 system/egpu_usb_perm.sh
#      抢在 udev 之前 chmod 666。本模块只**实测**节点是否可写（O_RDWR，与
#      libusb_open 同一个 syscall），不做提权。
#
#   ② 上电时序：自定义固件启动时 PCIe 是关的，设备以 12 Mbps(USB1.1) 枚举。
#      只有收到 vendor 请求 0xF3 才给 PCIe 上电；链路训练完成后设备会**重新
#      枚举**到 USB3(5000 Mbps)。重枚举会让已打开的 libusb 句柄失效，所以上电
#      必须在 tinygrad 打开句柄之前完成 —— 放进 tinygrad 里做必然 "chestnut
#      load failed"（历史 8 次全这样、成功 0 次）。
#
# 判据来源（ford-chestnut-optimize/27 号报告）：/data/egpu_usb_perm.log 里
#   20 次 bus 004(USB3) 全部大模型成功；4 次 bus 003(USB2) 全部失败，无一例外。
#   所以「是否在 USB3 侧且链路已训练」是唯一有决定性的变量。
#
# 注意：12 Mbps 是**正常态**，不是故障。不要在 USB2 侧直接否决（旧
# [CHX_USB2_GATE] 就是 6 秒放弃，把 tinygrad 唯一能自愈的路径堵死了）。
# ---------------------------------------------------------------------------

CHESTNUT_PRE_POWER_FILE = Path("/data/ford_params/dp_chestnut_pre_power")
LTSSM_ADDR = 0xB450
LTSSM_TRAINED = 0x78
USB3_MIN_SPEED = 5000


def _speed_mbps(device: Path) -> int:
  try:
    return int(float((device / "speed").read_text().strip()))
  except (OSError, ValueError, TypeError):
    return 0


def pre_power_enabled() -> bool:
  """运行期开关，缺省开。往 dp_chestnut_pre_power 写 0/false/off/no 可关闭。"""
  v = read(CHESTNUT_PRE_POWER_FILE)
  return (v or "").strip().lower() not in ("0", "false", "off", "no")


def chestnut_nodes() -> list[dict]:
  """枚举 sysfs 里所有 chestnut 设备（含 ROM bootloader 的两组 ID）。"""
  nodes = []
  for device in usb_devices():
    vendor_id = read_int(device / "idVendor", 16)
    product_id = read_int(device / "idProduct", 16)
    if not is_chestnut_usb_id(vendor_id, product_id, include_bootloader=True):
      continue
    nodes.append({
      "sysfs": device,
      "vendor_id": vendor_id,
      "product_id": product_id,
      "bus": read_int(device / "busnum"),
      "dev": read_int(device / "devnum"),
      "speed": _speed_mbps(device),
      "product": read(device / "product") or "",
    })
  return nodes


def node_openable(bus: int, dev: int) -> bool:
  """实测节点是否可读写。判据与 libusb_open 同源，最贴近真实条件。"""
  try:
    os.close(os.open(f"/dev/bus/usb/{bus:03d}/{dev:03d}", os.O_RDWR))
    return True
  except OSError:
    return False


def chestnut_ltssm(vendor_id: int, product_id: int) -> int:
  """读 PCIe LTSSM（0x78 = 链路已训练）。读不到返回 -1。"""
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


def chestnut_pre_power(vendor_id: int, product_id: int) -> bool:
  """给 ASM2464 的 PCIe 上电（vendor 请求 0xF3）。返回 0xF3 是否已成功发出。

  返回 True 不代表链路已 up，调用方还要等重枚举。
  """
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
        if ltssm == LTSSM_TRAINED:
          cloudlog.warning("[CHX_USB_ENUM] pre-power: 链路已训练，无需上电")
          return True
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


def chestnut_usb_node_ready(timeout: float = 60.0, power_timeout: float = 40.0,
                            poll: float = 0.5, min_speed: int = USB3_MIN_SPEED) -> bool:
  """把 chestnut 带到「节点可写 + 在 USB3 侧 + 链路已训练」的健康态，再交给 tinygrad。

  timeout        等到第一次看见可写设备的上限。
  power_timeout  发过 0xF3 之后等重枚举到 USB3 的上限（实测窗口 3~20 s）。

  返回 True 才允许 tinygrad 打开设备；返回 False 时调用方必须明确放弃大模型并打日志，
  不要静默继续（静默继续只会得到一次 EACCES/EBUSY 的 "chestnut load failed"）。
  """
  t0 = time.monotonic()
  seen_any = False
  powered = False

  while True:
    for node in chestnut_nodes():
      if not node_openable(node["bus"], node["dev"]):
        continue

      seen_any = True
      speed = node["speed"]
      ltssm = chestnut_ltssm(node["vendor_id"], node["product_id"]) if speed >= min_speed else -2

      if speed >= min_speed and ltssm in (-1, LTSSM_TRAINED):
        if ltssm == LTSSM_TRAINED:
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut 在 USB3 侧且链路已训练 "
                           f"(bus={node['bus']:03d}/{node['dev']:03d} speed={speed}Mbps)，大模型路径就绪")
        else:
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut 在 USB3 侧但 LTSSM 读不到 "
                           f"(speed={speed}Mbps)，交给 tinygrad 判断")
        return True

      if not powered:
        powered = True
        if pre_power_enabled():
          _lt = f"LTSSM=0x{ltssm & 0xFF:02X}" if ltssm >= 0 else "LTSSM 未读(USB2 侧)"
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut speed={speed}Mbps {_lt} -> 主动给 PCIe 上电")
          chestnut_pre_power(node["vendor_id"], node["product_id"])
        else:
          cloudlog.warning(f"[CHX_USB_ENUM] chestnut speed={speed}Mbps 需要上电，"
                           f"但 dp_chestnut_pre_power=0 已禁用")

    elapsed = time.monotonic() - t0
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
