#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vbus_hold.py —— 把 VBUS 钉在通电状态，给设备一段连续供电，然后看它到底能不能枚举。

【为什么需要它】
  实测有 3 个 flash.py 并存（hardwared 重启后旧 sudo 子进程变孤儿），
  每个都在跑 system/hardware/chestnut/flash.py:413 的 vbus_cycle()：
      vbus_write("0"); sleep(2); vbus_write("1"); sleep(5)
  三实例并集断电 ≈57%，稳定通电窗口可能碎片化到 <1 s。设备来不及完成
  hub_port_init 就又被断电，于是「VBUS 被切碎」和「硬件坏了」在观测上
  完全无法区分 —— 之前 40 s 零枚举就是这么来的。

  本脚本每 10 ms 把 VBUS 强制写回 "1"，把 flash.py 的断电动作在 10 ms 内抹掉，
  让这个问题变得可判：
      「给足连续供电，设备能不能枚举出来？」

【安全性】
  · 只写 VBUS="1"，从不写 "0" —— 只会「更少断电」，不会制造新的断电。
  · 若检测到设备已在线（可能在 ROM 写盘），默认**不杀** flash.py。
  · 若设备完全不在线，杀掉 flash.py 是安全的：打不开设备 ⇒ 不可能在 rom_write。

【用法（设备上，sudo）】
  sudo /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/vbus_hold.py
  sudo ... vbus_hold.py --seconds 60
  sudo ... vbus_hold.py --no-kill            # 只保活，不动 flash.py
  sudo ... vbus_hold.py --kill-flash         # 强制杀（设备在线时也杀，慎用）

【退出码】
  0  设备枚举成功（任意速度）—— 说明 VBUS 碎片化是元凶
  1  设备完全没出现 —— 硬件侧（12V / 线 / 设备状态）
  2  VBUS 节点不可写（需要 sudo 或 debugfs 未挂）
"""

import argparse
import glob
import os
import signal
import sys
import threading
import time

VBUS_PATH = "/sys/kernel/debug/regulator/smb2-vbus/enable"
CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}


def rd(path, dflt=""):
  try:
    return open(path).read().strip()
  except (OSError, UnicodeDecodeError):
    return dflt


def rdi(path, base=10, dflt=-1):
  try:
    return int(rd(path), base)
  except (OSError, ValueError):
    return dflt


def scan():
  out = []
  for d in sorted(glob.glob("/sys/bus/usb/devices/*")):
    vid, pid = rdi(d + "/idVendor", 16), rdi(d + "/idProduct", 16)
    if vid < 0 or pid < 0:
      continue
    key = (vid, pid)
    if key not in CHESTNUT_IDS and key not in ROM_IDS:
      continue
    try:
      speed = int(float(rd(d + "/speed", "0") or 0))
    except ValueError:
      speed = 0
    out.append({"name": os.path.basename(d), "id": "%04x:%04x" % key,
                "rom": key in ROM_IDS, "speed": speed,
                "bus": rdi(d + "/busnum"), "dev": rdi(d + "/devnum"),
                "product": rd(d + "/product"), "link_state": rd(d + "/link_state")})
  return out


def find_flash():
  hits = []
  try:
    names = os.listdir("/proc")
  except OSError:
    return hits
  for n in names:
    if not n.isdigit():
      continue
    try:
      cmd = open("/proc/%s/cmdline" % n, "rb").read().replace(b"\x00", b" ").decode(errors="replace")
    except OSError:
      continue
    if "chestnut/flash.py" in cmd:
      hits.append((int(n), cmd.strip()[:100]))
  return hits


def dmesg_tail(n=18):
  try:
    p = subprocess.run(["dmesg"], capture_output=True, timeout=15,
                       encoding="utf-8", errors="replace")
    return "\n".join((p.stdout or "").splitlines()[-n:])
  except Exception:
    return ""


class Holder(threading.Thread):
  """每 period 秒把 VBUS 写回 1。只写 1，从不写 0。"""

  def __init__(self, period):
    super().__init__(daemon=True)
    self.period = period
    self.stop = threading.Event()
    self.writes = 0
    self.error = None

  def run(self):
    while not self.stop.is_set():
      try:
        with open(VBUS_PATH, "w") as f:
          f.write("1")
        self.writes += 1
      except OSError as e:
        self.error = e
        return
      time.sleep(self.period)

  def release(self):
    self.stop.set()
    self.join(timeout=1.0)
    try:
      with open(VBUS_PATH, "w") as f:
        f.write("1")
    except OSError:
      pass


def describe(nodes):
  if not nodes:
    return "无设备"
  return " | ".join(
    "%s %s %s %dMbps bus=%03d/%03d %r%s" % (
      "ROM" if n["rom"] else "CHESTNUT", n["name"], n["id"], n["speed"],
      n["bus"], n["dev"], n["product"],
      (" link_state=" + n["link_state"]) if n["link_state"] else "")
    for n in nodes)


def main():
  ap = argparse.ArgumentParser(description="VBUS 保活，判断设备能否在连续供电下枚举")
  ap.add_argument("--seconds", type=float, default=40.)
  ap.add_argument("--hold", type=float, default=0.01, help="保活写周期，默认 10 ms")
  ap.add_argument("--sample", type=float, default=0.2)
  ap.add_argument("--kill-flash", action="store_true", help="设备在线时也强杀 flash.py（慎用）")
  ap.add_argument("--no-kill", action="store_true", help="完全不杀 flash.py，只保活")
  args = ap.parse_args()

  if getattr(os, "geteuid", lambda: 0)() != 0:
    print("  [WARN] 非 root，写 VBUS 会失败，建议 sudo")

  if not os.path.exists(VBUS_PATH):
    print("  [FAIL] 没有 VBUS 控制节点：%s" % VBUS_PATH)
    print("         先试 sudo mount -t debugfs none /sys/kernel/debug")
    return 2

  print("=" * 88)
  print(" VBUS 保活判决   %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
  print("=" * 88)

  before = scan()
  print("  起始枚举   = %s" % describe(before))

  procs = find_flash()
  print("  flash.py   = %s" % ("; ".join("pid=%d" % p for p, _ in procs) if procs else "未运行"))

  killed = 0
  if procs and not args.no_kill:
    if before and not args.kill_flash:
      print("  [SKIP] 设备已在线，可能在 ROM 写盘 —— 默认不杀 flash.py（只保活）。")
      print("         要强制杀加 --kill-flash")
    else:
      for pid, _ in procs:
        try:
          os.kill(pid, signal.SIGKILL)
          killed += 1
        except OSError:
          pass
      print("  [KILL] 设备不在线 ⇒ 不可能在 rom_write，已 SIGKILL %d 个 flash.py" % killed)
      time.sleep(1.5)

  h = Holder(args.hold)
  h.start()
  print("  VBUS 保活   = 每 %.0f ms 写回 \"1\"（只写 1，从不写 0）" % (args.hold * 1000))
  print("-" * 88)

  t0 = time.monotonic()
  prev_fp = None
  best = 0
  seen = 0
  first_seen = None
  try:
    while time.monotonic() - t0 < args.seconds:
      nodes = scan()
      fp = tuple((n["id"], n["bus"], n["dev"], n["speed"]) for n in nodes)
      if nodes:
        seen += 1
        if first_seen is None:
          first_seen = time.monotonic() - t0
        for n in nodes:
          best = max(best, n["speed"])
      if fp != prev_fp:
        prev_fp = fp
        print("  %s  %s" % (time.strftime("%H:%M:%S"), describe(nodes)))
      time.sleep(args.sample)
  except KeyboardInterrupt:
    print("\n  (手动中断)")
  finally:
    h.release()

  el = time.monotonic() - t0
  print("-" * 88)
  print("  观察 %.1f s，VBUS 写 %d 次%s" % (
    el, h.writes, ("，错误=%r" % (h.error,)) if h.error else ""))
  print("  设备出现 %d/%d 帧（%.0f%%），最高 %d Mbps%s" % (
    seen, max(1, int(el / args.sample)), 100.0 * seen / max(1, int(el / args.sample)), best,
    ("，首次 +%.1f s" % first_seen) if first_seen is not None else ""))

  tail = dmesg_tail(18)
  if tail:
    print("\n  ---- dmesg 尾部 ----")
    for line in tail.splitlines():
      print("  " + line)

  print("\n  " + "=" * 84)
  if seen == 0:
    print("  [结论] 连续供电 %.0f s，设备一次都没出现。" % el)
    print("         ⇒ 不是 VBUS 碎片化的问题，硬件侧：")
    print("           1. 底座 12V 电源是否插着（ASM2464 此时全靠 VBUS 供电，上电冲击最易欠压）")
    print("           2. 换一根 USB3.2 / USB4 全功能线")
    print("           3. 设备 + 底座两端同时断电 >= 30 s，先插好线再一起上电")
    rc = 1
  elif best >= 5000:
    print("  [结论] 设备以 %d Mbps 出现在 USB3 侧 —— 链路是好的。" % best)
    print("         ⇒ 之前的「零枚举」确实是 VBUS 被 flash.py 切碎导致。")
    print("         下一步：把 flash.py 彻底停掉（NO_FLASH 补丁），再跑 egpu_bringup.py。")
    rc = 0
  else:
    print("  [结论] 设备以 %d Mbps 出现（首次 +%.1f s）。" % (best, first_seen or 0))
    print("         ⇒ VBUS 碎片化就是元凶：给足连续供电它就回来了。")
    print("         下一步：sudo .../_chk/egpu_bringup.py --timeout 90  （发 0xF3 上 USB3）")
    rc = 0
  print("  " + "=" * 84)
  return rc


if __name__ == "__main__":
  sys.exit(main())
