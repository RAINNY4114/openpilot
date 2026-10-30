#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_egpu_cycle.py —— 重现「eGPU 图标 绿 -> 橙 -> 消失」的完整过程，并逐帧对齐代码判据。

【它到底在复现什么】
  离车（offroad）时，侧边栏 eGPU 图标的颜色完全由 selfdrive/ui/ui_state.py:226-231 决定：
      detected = sm["deviceState"].chestnutPresent
      chestnut_state = FAILED     if chestnut_alert            # 橙
                       READY      if detected and compiled     # 绿
                       UNCOMPILED if detected                  # 橙
                       DISCONNECTED                            # 无图标
  然后 sidebar.py:93-102 映射成图标：DISCONNECTED->不画；UNCOMPILED|FAILED->橙图；其余->绿图。

  所以「绿 -> 橙 -> 消失」只可能是：
      绿   = detected=True  + compiled=True + 无 Offroad_Chestnut* 告警
      橙   = 某个 Offroad_Chestnut* 告警被写入（离车时最常见的是 status.py:89 的 slow_usb）
      消失 = detected=False（设备掉线）+ 告警被删（devices 空 => slow_usb=False）

【节拍从哪来】
  system/hardware/chestnut/flash.py:413
      def vbus_cycle():
        vbus_write("0"); time.sleep(2)
        vbus_write("1"); time.sleep(5)
  2 + 5 = 7.0 s。本脚本用同一个 sysfs 节点人为复刻这个节拍，从而把现象变成可重复的。

【用法（设备上，sudo）】
  # A. 主动复现：自己驱动 VBUS 节拍（会先停掉 flash.py 的干扰？不会，请先 touch 标记）
  sudo /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/repro_egpu_cycle.py --cycles 6

  # B. 只观察：flash.py 正在自己制造节拍时，别跟它抢 VBUS
  sudo ... repro_egpu_cycle.py --observe-only --seconds 90

  # C. 只想看一帧当前状态
  sudo ... repro_egpu_cycle.py --once

【注意】
  · 必须在 offroad（熄火/停车）下跑，onroad 走的是 ui_state.py:234-246 另一套分支。
  · 只写 VBUS 断电 2 s，不碰固件、不碰 pkl，可重复。
  · 报告落盘到 _chk/repro_cycle_report.txt，可用 copyparty 直接读回。
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

VBUS_PATH = "/sys/kernel/debug/regulator/smb2-vbus/enable"
MODE_PATH = "/sys/devices/platform/soc/a600000.ssusb/mode"
PARAMS_DIR = "/data/params/d"
MODELS_DIR = "/data/openpilot/openpilot/selfdrive/modeld/models"

CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}

DEFAULT_OUT = "/data/openpilot/openpilot/_chk/repro_cycle_report.txt"


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


# ---------------------------------------------------------------- 硬件侧

def usb_nodes():
  """返回 (chestnut_nodes, rom_nodes)。chestnut_nodes 即 ui_state 的 `detected` 来源。"""
  chestnut, rom = [], []
  for d in sorted(glob.glob("/sys/bus/usb/devices/*")):
    vid = rdi(d + "/idVendor", 16)
    pid = rdi(d + "/idProduct", 16)
    if vid < 0 or pid < 0:
      continue
    key = (vid, pid)
    if key not in CHESTNUT_IDS and key not in ROM_IDS:
      continue
    try:
      speed = int(float(rd(d + "/speed", "0") or 0))
    except ValueError:
      speed = 0
    node = {
      "name": os.path.basename(d),
      "id": "%04x:%04x" % (vid, pid),
      "speed": speed,
      "bus": rdi(d + "/busnum"),
      "dev": rdi(d + "/devnum"),
      "product": rd(d + "/product"),
      "bcd": rd(d + "/version"),
      "link_state": rd(d + "/link_state"),
    }
    (chestnut if key in CHESTNUT_IDS else rom).append(node)
  return chestnut, rom


def compiled():
  """等价 selfdrive/modeld/helpers.py:chestnut_compiled()"""
  big = os.path.isfile(os.path.join(MODELS_DIR, "big_driving_tinygrad.pkl"))
  warps = all(os.path.isfile(os.path.join(MODELS_DIR, "big_driving_warp_%s_tinygrad.pkl" % s))
              for s in ("1344x760", "1928x1208"))
  return big and warps


def chestnut_alerts():
  """等价 ui_state.py:267 —— Params 里存在 Offroad_Chestnut* 键即为激活（remove 才删除）。"""
  out = {}
  try:
    names = os.listdir(PARAMS_DIR)
  except OSError:
    return out
  for n in names:
    if n.startswith("Offroad_Chestnut"):
      out[n] = rd(os.path.join(PARAMS_DIR, n))[:120]
  return out


# ---------------------------------------------------------------- UI 侧推演

def ui_state(chestnut, alerts, comp):
  """严格照 ui_state.py:226-231 的离车分支求值。返回 (state, 颜色说明)。"""
  detected = len(chestnut) > 0
  if alerts:
    return "FAILED", "ORANGE"
  if detected and comp:
    return "READY", "GREEN"
  if detected:
    return "UNCOMPILED", "ORANGE"
  return "DISCONNECTED", "NO-ICON"


def usb_desc(chestnut, rom):
  parts = []
  for n in chestnut:
    parts.append("CHESTNUT %s %dMbps bus=%03d/%03d %r bcd=%s%s" % (
      n["id"], n["speed"], n["bus"], n["dev"], n["product"], n["bcd"],
      (" link_state=" + n["link_state"]) if n["link_state"] else ""))
  for n in rom:
    parts.append("ROM %s %dMbps bus=%03d/%03d %r" % (
      n["id"], n["speed"], n["bus"], n["dev"], n["product"]))
  return " | ".join(parts) if parts else "-"


# ---------------------------------------------------------------- dmesg 对齐

class Dmesg:
  def __init__(self):
    self.last = None
    self.buf = []
    self._prime()

  def _read(self):
    try:
      p = subprocess.run(["dmesg"], capture_output=True, timeout=10,
                         encoding="utf-8", errors="replace")
      return (p.stdout or "").splitlines()
    except Exception:
      return []

  def _prime(self):
    lines = self._read()
    self.last = lines[-1] if lines else None
    self.buf = lines[-6:]

  def poll(self):
    lines = self._read()
    if not lines:
      return []
    if self.last is None or self.last not in lines:
      fresh = lines[-6:]
    else:
      fresh = lines[lines.index(self.last) + 1:]
    if fresh:
      self.last = fresh[-1]
      self.buf = (self.buf + fresh)[-12:]
    return [l for l in fresh if any(k in l for k in
            ("usb 3-1", "usb 4-1", "usb3-port", "usb4-port", "Cannot enable",
             "unable to enumerate", "new full-speed", "new SuperSpeed",
             "error -71", "usb-storage", "USB disconnect"))]


# ---------------------------------------------------------------- VBUS 驱动

def vbus_write(val):
  try:
    with open(VBUS_PATH, "w") as f:
      f.write(val)
    return True
  except OSError as e:
    return e


def ensure_debugfs():
  if os.path.exists(VBUS_PATH):
    return True
  try:
    if " /sys/kernel/debug " not in open("/proc/mounts").read():
      subprocess.run(["mount", "-t", "debugfs", "none", "/sys/kernel/debug"],
                     capture_output=True, timeout=15)
  except Exception:
    pass
  return os.path.exists(VBUS_PATH)


# ---------------------------------------------------------------- 主流程

def main():
  ap = argparse.ArgumentParser(description="重现 eGPU 图标 绿->橙->消失")
  ap.add_argument("--cycles", type=int, default=6, help="驱动多少个 VBUS 周期（默认 6）")
  ap.add_argument("--observe-only", action="store_true", help="不写 VBUS，只观察")
  ap.add_argument("--seconds", type=float, default=0., help="只观察模式下的时长，默认 cycles*7")
  ap.add_argument("--off", type=float, default=2.0, help="断电保持秒数，默认 2.0（同 flash.py）")
  ap.add_argument("--on", type=float, default=5.0, help="上电保持秒数，默认 5.0（同 flash.py）")
  ap.add_argument("--sample", type=float, default=0.2)
  ap.add_argument("--once", action="store_true", help="只打印一帧当前状态")
  ap.add_argument("--out", default=DEFAULT_OUT)
  args = ap.parse_args()

  period = args.off + args.on
  comp = compiled()

  print("=" * 92)
  print(" eGPU 图标复现   %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
  print("=" * 92)
  print("  模式             = %s" % ("OBSERVE-ONLY" if args.observe_only else "DRIVE-VBUS"))
  print("  a600000.ssusb/mode = %s  （要 host）" % rd(MODE_PATH, "?"))
  print("  chestnut_compiled  = %s  （%s）" % (comp, MODELS_DIR))
  print("  VBUS 控制节点      = %s 存在=%s" % (VBUS_PATH, os.path.exists(VBUS_PATH)))

  flash = []
  try:
    proc_names = os.listdir("/proc")
  except OSError:
    proc_names = []
  for name in proc_names:
    if name.isdigit():
      try:
        cmd = open("/proc/%s/cmdline" % name, "rb").read().replace(b"\x00", b" ").decode(errors="replace")
      except OSError:
        continue
      if "chestnut/flash.py" in cmd:
        flash.append((name, cmd.strip()[:80]))
  if flash:
    print("  ⚠ flash.py 在跑  = %s" % "; ".join("pid=%s %s" % f for f in flash))
    print("    它自己在每 7.0 s 断一次 VBUS，会和本脚本抢控制权。")
    print("    想自己驱动节拍：先 sudo touch /data/chestnut_no_flash 并等它退出。")
  else:
    print("  flash.py          = 未运行（好，没有竞争）")

  chestnut, rom = usb_nodes()
  alerts = chestnut_alerts()
  state, color = ui_state(chestnut, alerts, comp)
  print("\n  当前一帧：")
  print("    detected = %s   compiled = %s" % (len(chestnut) > 0, comp))
  print("    alerts   = %s" % (list(alerts) if alerts else "无"))
  print("    usb      = %s" % usb_desc(chestnut, rom))
  print("    => chestnut_state = %s  => 图标 %s" % (state, color))

  if args.once:
    print("=" * 92)
    return 0

  drive = not args.observe_only
  if drive and not ensure_debugfs():
    print("\n  [FAIL] 没有 VBUS 控制节点，无法自己驱动节拍。")
    print("         改跑 --observe-only，并让 flash.py 去制造节拍。")
    print("=" * 92)
    return 1

  total = args.seconds if args.observe_only else args.cycles * period
  print("\n  开始记录 %.1f s，采样 %.2f s%s" % (
    total, args.sample, "" if args.observe_only else "，节拍 %.1fs 断 / %.1fs 通" % (args.off, args.on)))
  print("-" * 92)
  print("   %-9s %-13s %-8s %-9s %-8s %s" % ("t", "STATE", "ICON", "detected", "alerts", "usb"))
  print("-" * 92)

  dm = Dmesg()
  t0 = time.monotonic()
  prev_sig = None
  cur_vbus = None
  timeline = []          # [(t, state, color, alerts_keys, speed)]
  dmesg_log = []

  while True:
    now = time.monotonic()
    el = now - t0
    if el > total:
      break

    # 驱动 VBUS：相位由绝对时间算出，保证采样不被阻塞
    if drive:
      phase = el % period
      want = "0" if phase < args.off else "1"
      if want != cur_vbus:
        err = vbus_write(want)
        cur_vbus = want
        if err is not True:
          print("   [FAIL] 写 VBUS=%s 失败: %r（需要 sudo）" % (want, err))
          drive = False

    chestnut, rom = usb_nodes()
    alerts = chestnut_alerts()
    state, color = ui_state(chestnut, alerts, comp)
    speed = max([n["speed"] for n in chestnut], default=0)

    sig = (state, tuple(sorted(alerts)))
    if sig != prev_sig:
      prev_sig = sig
      timeline.append((round(el, 2), state, color, tuple(sorted(alerts)), speed))
      print("   +%-8.2f %-13s %-8s %-9s %-8s %s" % (
        el, state, color, len(chestnut) > 0,
        ",".join(k.replace("Offroad_Chestnut", "") for k in alerts) or "-",
        usb_desc(chestnut, rom)[:80]))

    for line in dm.poll():
      dmesg_log.append((round(el, 2), line.strip()))
      print("   %9s %s" % ("  dmesg!", line.strip()[:110]))

    time.sleep(args.sample)

  if drive:
    vbus_write("1")

  # ---------------- 汇总
  print("-" * 92)
  print("  时间轴：")
  for t, st, col, al, sp in timeline:
    print("    +%7.2fs  %-13s %-8s %s" % (t, st, col,
          ("告警=" + ",".join(al)) if al else ("speed=%dMbps" % sp if sp else "")))

  dur = {}
  for i, (t, st, col, al, sp) in enumerate(timeline):
    end = timeline[i + 1][0] if i + 1 < len(timeline) else round(total, 2)
    dur[st] = dur.get(st, 0.) + (end - t)

  counts = {}
  for _, st, _, _, _ in timeline:
    counts[st] = counts.get(st, 0) + 1

  print("\n  统计：")
  for st in ("READY", "FAILED", "UNCOMPILED", "DISCONNECTED"):
    if st in counts:
      print("    %-13s 出现 %d 次，累计 %.2f s" % (st, counts[st], dur.get(st, 0.)))

  orange_by = {}
  for _, st, col, al, _ in timeline:
    if col == "ORANGE":
      for k in al:
        orange_by[k] = orange_by.get(k, 0) + 1
  if orange_by:
    print("    橙色触发者：%s" % ", ".join("%s x%d" % kv for kv in orange_by.items()))

  greens = [t for t, st, _, _, _ in timeline if st == "READY"]
  if len(greens) >= 2:
    gaps = [b - a for a, b in zip(greens, greens[1:])]
    print("    绿->绿 间隔：%s  （均值 %.2f s）" % (
      ", ".join("%.2f" % g for g in gaps), sum(gaps) / len(gaps)))

  verdict = ("已重现：绿(READY) -> 橙(FAILED, 触发者见上) -> 消失(DISCONNECTED)"
             if ("READY" in counts and "FAILED" in counts and "DISCONNECTED" in counts)
             else "本次没凑齐三段，看上面的时间轴判断卡在哪一段")
  print("\n  %s" % verdict)

  # ---------------- 落盘
  try:
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
      f.write("eGPU 图标复现报告  %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
      f.write("模式=%s cycles=%d off=%.1f on=%.1f compiled=%s\n" % (
        "observe" if args.observe_only else "drive", args.cycles, args.off, args.on, comp))
      f.write("\n[时间轴]\n")
      for t, st, col, al, sp in timeline:
        f.write("  +%7.2fs  %-13s %-8s %s\n" % (t, st, col,
                ("alerts=" + ",".join(al)) if al else ("speed=%d" % sp)))
      f.write("\n[统计]\n")
      for st in ("READY", "FAILED", "UNCOMPILED", "DISCONNECTED"):
        if st in counts:
          f.write("  %-13s x%d  %.2fs\n" % (st, counts[st], dur.get(st, 0.)))
      if orange_by:
        f.write("  橙色触发者: %s\n" % json.dumps(orange_by, ensure_ascii=False))
      f.write("\n[dmesg 对齐]\n")
      for t, line in dmesg_log:
        f.write("  +%7.2fs  %s\n" % (t, line))
      f.write("\n[结论] %s\n" % verdict)
    print("  报告已写 %s" % args.out)
  except OSError as e:
    print("  报告写入失败: %r" % (e,))

  print("=" * 92)
  return 0


if __name__ == "__main__":
  sys.exit(main())
