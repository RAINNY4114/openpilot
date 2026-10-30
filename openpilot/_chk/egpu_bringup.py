#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
egpu_bringup.py —— 绕过 modeld 门禁，手工把 chestnut eGPU 带到 USB3 就绪态。

【为什么需要它】
  sunnypilot/modeld_v2/modeld.py:179
      gate = pkl is not None and fmt == 'v2' and present
  本机 big_driving_tinygrad.pkl 是 legacy 容器（head = 41 84 0c 00 ...），
  fmt != 'v2'  =>  gate = False  =>  :480 `if CHESTNUT:` 不成立
  =>  load_big() 线程不创建  =>  :491 chestnut_usb_node_ready() 永不调用
  =>  common/hardware/usb.py:223 的 controlWrite(0x40, 0xF3, ...) 永不发出
  =>  PCIe 永不上电  =>  设备永远停在 USB2 12 Mbps
  =>  status.py:89 `slow_usb = offroad and len(devices)==1 and speedMbps<5000`
      => 橙色；再掉线 => 消失。

  而把 legacy pkl 转成 SPTGPLN1 的 sunnypilot/modeld_v2/convert_upstream_pkl.py:208
  要 `Device[Device.DEFAULT].renderer.target`，即必须 eGPU 已经能用。
  ==> 这是一个死锁。本脚本从「硬件侧」破环，不碰 pkl。

【用法（设备上，root）】
  sudo /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/egpu_bringup.py
  sudo ... egpu_bringup.py --stop-flash                 # 顺带建 /data/chestnut_no_flash
  sudo ... egpu_bringup.py --stop-flash --kill-flash    # 并杀掉正在跑的 flash.py
  sudo ... egpu_bringup.py --wait 60                    # 先等 60 s（留时间拔插线/断电）
  sudo ... egpu_bringup.py --timeout 120                # 总超时（默认 90 s）

【退出码】
  0  已在 USB3 侧（speed >= 5000）—— 链路 OK
  1  设备停在 ROM（174c:246x）—— 只能「设备 + 盒子两端同时断电 >= 30 s」
  2  在 USB2 侧但 0xF3 发不出去（节点不可写 / 被占用）
  3  发了 0xF3 但等不到 USB3 重枚举
  4  完全没看到任何 chestnut 设备（线/口/供电）
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

USB_DEVICES = Path("/sys/bus/usb/devices")
CHESTNUT_IDS = {(0xADD1, 0x0001), (0x3801, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}

LTSSM_ADDR = 0xB450
LTSSM_TRAINED = 0x78
USB3_MIN_SPEED = 5000

MARKER = Path("/data/chestnut_no_flash")

EXIT_OK = 0
EXIT_ROM = 1
EXIT_NO_PERM = 2
EXIT_NO_USB3 = 3
EXIT_NOTHING = 4


# ---------------------------------------------------------------- 基础设施

def say(tag, msg):
  print("%s [%-9s] %s" % (time.strftime("%H:%M:%S"), tag, msg), flush=True)


def read(path, default=""):
  try:
    return Path(path).read_text().strip()
  except (OSError, UnicodeDecodeError):
    return default


def read_int(path, base=10, default=-1):
  try:
    return int(read(path), base)
  except (OSError, ValueError):
    return default


def scan():
  """枚举 sysfs 里所有 chestnut / ROM 设备。返回 list[dict]。"""
  out = []
  try:
    entries = list(USB_DEVICES.iterdir())
  except OSError:
    return out
  for d in entries:
    vid = read_int(d / "idVendor", 16)
    pid = read_int(d / "idProduct", 16)
    if vid < 0 or pid < 0:
      continue
    key = (vid, pid)
    if key not in CHESTNUT_IDS and key not in ROM_IDS:
      continue
    try:
      speed = int(float(read(d / "speed", "0") or 0))
    except (OSError, ValueError):
      speed = 0
    out.append({
      "sysfs": d.name,
      "vid": vid, "pid": pid,
      "rom": key in ROM_IDS,
      "bus": read_int(d / "busnum"),
      "dev": read_int(d / "devnum"),
      "speed": speed,
      "product": read(d / "product"),
      "bcd": read(d / "version"),
      "serial": read(d / "serial"),
      "link_state": read(d / "link_state"),
      "mode": read(d / "mode"),
    })
  out.sort(key=lambda n: (n["rom"], n["bus"], n["dev"]))
  return out


def fingerprint(nodes):
  return tuple((n["vid"], n["pid"], n["bus"], n["dev"], n["speed"]) for n in nodes)


def describe(nodes):
  if not nodes:
    return "无 chestnut 设备"
  parts = []
  for n in nodes:
    kind = "ROM" if n["rom"] else "CHESTNUT"
    parts.append("%s %04x:%04x bus=%03d/%03d speed=%sMbps product=%r bcdUSB=%s%s" % (
      kind, n["vid"], n["pid"], n["bus"], n["dev"], n["speed"],
      n["product"], n["bcd"],
      (" link_state=%s" % n["link_state"]) if n["link_state"] else ""))
  return " | ".join(parts)


def node_openable(bus, dev):
  try:
    os.close(os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR))
    return True
  except OSError:
    return False


def ltssm(vid, pid):
  """读 PCIe LTSSM，0x78 = 已训练。读不到返回 -1。"""
  try:
    import usb1
  except Exception:
    return -1
  try:
    with usb1.USBContext() as ctx:
      h = ctx.openByVendorIDAndProductID(vid, pid, skip_on_error=True)
      if h is None:
        return -1
      try:
        return bytes(h.controlRead(0xC0, 0xE4, LTSSM_ADDR, 0, 1, timeout=1000))[0]
      finally:
        try:
          h.close()
        except Exception:
          pass
  except Exception:
    return -1


def pre_power(vid, pid):
  """发 vendor 请求 0xF3 给 ASM2464 的 PCIe 上电。返回 (ok, msg)。"""
  try:
    import usb1
  except Exception as e:
    return False, "usb1 不可用: %r" % (e,)
  try:
    with usb1.USBContext() as ctx:
      h = ctx.openByVendorIDAndProductID(vid, pid, skip_on_error=True)
      if h is None:
        return False, "打不开设备（权限不足或已被 tinygrad/flash.py 占用）"
      try:
        cur = -1
        try:
          cur = bytes(h.controlRead(0xC0, 0xE4, LTSSM_ADDR, 0, 1, timeout=1000))[0]
        except Exception as e:
          return False, "读 LTSSM 失败: %r" % (e,)
        if cur == LTSSM_TRAINED:
          return True, "LTSSM 已是 0x78，无需上电"
        h.controlWrite(0x40, 0xF3, 1, 0, b"", timeout=5000)
        return True, "已发 0xF3（上电前 LTSSM=0x%02X），等待重枚举到 USB3" % cur
      finally:
        try:
          h.close()
        except Exception:
          pass
  except Exception as e:
    return False, "0xF3 失败: %r" % (e,)


# ---------------------------------------------------------------- 辅助动作

def find_flash_procs():
  hits = []
  for p in Path("/proc").iterdir():
    if not p.name.isdigit():
      continue
    try:
      cmd = (p / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace")
    except OSError:
      continue
    if "chestnut/flash.py" in cmd and "grep" not in cmd:
      hits.append((int(p.name), cmd.strip()))
  return hits


def do_stop_flash(kill):
  try:
    MARKER.parent.mkdir(parents=True, exist_ok=True)
    MARKER.write_text("1\n")
    say("STOP-FLASH", "已写 %s —— hardwared.Chestnut.update() 的 NO_FLASH 门会拦住后续刷写" % MARKER)
  except OSError as e:
    say("STOP-FLASH", "写标记失败: %r" % (e,))
    return
  procs = find_flash_procs()
  if not procs:
    say("STOP-FLASH", "当前没有正在跑的 flash.py")
    return
  for pid, cmd in procs:
    say("STOP-FLASH", "在跑: pid=%d %s" % (pid, cmd[:110]))
  if not kill:
    say("STOP-FLASH", "默认不杀（可能正在写 SPI）。确认要停请加 --kill-flash")
    return
  for pid, _ in procs:
    try:
      os.kill(pid, 15)
      say("STOP-FLASH", "已 SIGTERM pid=%d" % pid)
    except OSError as e:
      say("STOP-FLASH", "SIGTERM pid=%d 失败: %r" % (pid, e))
  time.sleep(2)
  for pid, cmd in find_flash_procs():
    try:
      os.kill(pid, 9)
      say("STOP-FLASH", "已 SIGKILL pid=%d" % pid)
    except OSError as e:
      say("STOP-FLASH", "SIGKILL pid=%d 失败: %r" % (pid, e))


def dmesg_tail(n=40):
  try:
    out = subprocess.run(["dmesg"], capture_output=True, timeout=20,
                         encoding="utf-8", errors="replace").stdout or ""
  except Exception:
    return ""
  return "\n".join(out.splitlines()[-n:])


# ---------------------------------------------------------------- 主流程

def main():
  ap = argparse.ArgumentParser(description="把 chestnut eGPU 手工带到 USB3 就绪态（不依赖 pkl 格式）")
  ap.add_argument("--stop-flash", action="store_true", help="创建 /data/chestnut_no_flash")
  ap.add_argument("--kill-flash", action="store_true", help="同时杀掉正在跑的 flash.py（配合 --stop-flash）")
  ap.add_argument("--wait", type=float, default=0., help="开始前先等 N 秒")
  ap.add_argument("--timeout", type=float, default=90., help="总超时，默认 90 s")
  ap.add_argument("--power-timeout", type=float, default=40., help="发过 0xF3 后等重枚举的上限，默认 40 s")
  ap.add_argument("--no-power", action="store_true", help="只诊断，不发 0xF3")
  ap.add_argument("--poll", type=float, default=0.4)
  args = ap.parse_args()

  if os.geteuid() != 0:
    say("WARN", "非 root：读 link_state/mode 与打开 /dev/bus/usb 会失败，建议 sudo")

  if args.wait > 0:
    say("WAIT", "%.0f s 后开始（现在可以拔插线 / 断电）" % args.wait)
    time.sleep(args.wait)

  if args.stop_flash:
    do_stop_flash(args.kill_flash)

  running = find_flash_procs()
  if running:
    say("WARN", "flash.py 仍在跑（pid=%s）—— 它的 vbus_cycle() 会反复断 VBUS，"
                "枚举必然被反复打断。建议先 --stop-flash --kill-flash" %
        ",".join(str(p) for p, _ in running))

  say("SCAN", "开始枚举（总超时 %.0f s）" % args.timeout)
  t0 = time.monotonic()
  deadline = t0 + args.timeout
  last_fp = None
  last_say = 0.
  powered_at = None
  power_tried = False
  power_err = None
  saw_any = False
  saw_rom = False
  best_speed = 0

  while True:
    nodes = scan()
    fp = fingerprint(nodes)
    now = time.monotonic()

    if nodes:
      saw_any = True
      for n in nodes:
        if n["rom"]:
          saw_rom = True
        best_speed = max(best_speed, n["speed"])

    if fp != last_fp:
      say("CHANGE", describe(nodes))
      last_fp = fp
    elif now - last_say > 5.:
      say("IDLE", describe(nodes))
      last_say = now

    # ---- 成功判据：非 ROM 且 speed >= 5000
    good = [n for n in nodes if not n["rom"] and n["speed"] >= USB3_MIN_SPEED]
    if good:
      n = good[0]
      lt = ltssm(n["vid"], n["pid"])
      say("OK", "已在 USB3 侧：bus=%03d/%03d speed=%dMbps LTSSM=%s product=%r" % (
        n["bus"], n["dev"], n["speed"],
        ("0x%02X" % lt) if lt >= 0 else "读不到", n["product"]))
      if lt >= 0 and lt != LTSSM_TRAINED:
        say("WARN", "USB 侧已是 5000Mbps 但 LTSSM=0x%02X != 0x78，PCIe 仍未训练" % lt)
      say("DONE", "链路就绪，可以继续跑 pkl 转换 / 重启 modeld")
      return EXIT_OK

    # ---- 只有 ROM：不可能靠软件救
    non_rom = [n for n in nodes if not n["rom"]]
    if nodes and not non_rom:
      # 持续只看到 ROM 超过 6 s 就下结论（短暂出现在 ROM 是正常的中间态）
      if now - t0 > 6.:
        say("FAIL", "设备停在 ROM bootloader（174c:246x），固件没起来。")
        say("FAIL", "软件侧无解：请把「设备 + 底座」两端电源同时断开 >= 30 s，"
                    "先插好线，再一起上电。")
        return EXIT_ROM

    # ---- USB2 侧：尝试上电
    if non_rom and not power_tried and not args.no_power:
      n = non_rom[0]
      if not node_openable(n["bus"], n["dev"]):
        power_tried = True
        power_err = "节点 /dev/bus/usb/%03d/%03d 不可写" % (n["bus"], n["dev"])
        say("POWER", power_err + " —— 0xF3 发不出去")
      else:
        power_tried = True
        ok, msg = pre_power(n["vid"], n["pid"])
        say("POWER", msg)
        if not ok:
          power_err = msg
        else:
          powered_at = time.monotonic()

    if powered_at is not None and now - powered_at > args.power_timeout:
      say("FAIL", "发了 0xF3，%.0f s 内没回到 USB3 侧（最好只到 %dMbps）" % (
        now - powered_at, best_speed))
      say("FAIL", "下一步：拿一个普通 USB3 U 盘插同一个口做对照 —— 能跑 5000Mbps 说明"
                  "C3X 口没问题，问题在线/底座；也只能 480 说明口或供电有问题。")
      _dump()
      return EXIT_NO_USB3

    if now > deadline:
      if not saw_any:
        say("FAIL", "%.0f s 内没看到任何 chestnut / ROM 设备。" % (now - t0))
        say("FAIL", "查：底座 12V 电源是否插好；线是否插在 C3X 的 USB-C 数据口（不是充电口）；"
                    "线本身是否支持 USB3/USB4。")
        _dump()
        return EXIT_NOTHING
      if power_err:
        say("FAIL", "设备在 USB2 侧但上电失败：%s" % power_err)
        say("FAIL", "查：是否有别的进程占用（flash.py / tinygrad / modeld），或 sudo 是否生效。")
        _dump()
        return EXIT_NO_PERM
      say("FAIL", "超时。最后状态：%s" % describe(scan()))
      _dump()
      return EXIT_NO_USB3

    time.sleep(args.poll)


def _dump():
  tail = dmesg_tail(40)
  if tail:
    print("\n---- dmesg 尾部 ----", flush=True)
    print(tail, flush=True)
  print("\n---- 提示 ----", flush=True)
  print("绿->橙->消失 的代码级解释：status.py:89 slow_usb = 离车 and 1 个设备 and speed<5000", flush=True)
  print("  => 12Mbps 枚举成功那 1~2 秒 = 绿；写入 Offroad_ChestnutUsbSlow = 橙；掉线 = 消失", flush=True)


if __name__ == "__main__":
  sys.exit(main())
