#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
usb_enum_diag.py —— USB 枚举层只读诊断。不写任何东西，10 秒内出全量快照。

【为什么需要它】
  dmesg 只告诉我们「失败了」，不告诉我们「失败在哪一层」。枚举失败至少五种成因，
  现象几乎一样，但修法完全不同：

    ① 端口过流 (over_current_count > 0)   -> 供电/线材短路，软件无解
    ② 端口被禁用 (disabled = 1)            -> 之前 Cannot enable 后被 xhci 关掉，
                                              必须清掉才能重试
    ③ 控制器 runtime PM 把口挂起           -> 写 power/control=on 可解
    ④ usb-storage / uas 抢绑               -> 解开才能给 tinygrad 独占
    ⑤ 权限 0600 root:root（udev 还没放宽） -> egpu_usb_perm.sh 的职责，本仓库已缺失
    ⑥ 描述符响应慢（ASM2464 已知特性）     -> 只能靠长时间连续供电 + 主动重枚举

  本脚本把 ①~⑤ 全部实测出来，⑥ 由 vbus_hold.py 负责。

【用法（设备上，sudo 只为了能读全 link_state / 设备节点权限）】
  sudo /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/usb_enum_diag.py
  sudo ... usb_enum_diag.py --out /data/openpilot/openpilot/_chk/usb_enum_diag.txt
"""

import argparse
import glob
import os
import subprocess
import time

CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}

PORT_ATTRS = ("connect_type", "disabled", "indicator", "over_current_count",
              "quirks", "link_state", "lane_count", "rx_lanes", "tx_lanes",
              "maxchild", "available_power", "usb2_lpm_l1_timeout")

PM_ATTRS = ("control", "runtime_status", "autosuspend_delay_ms", "runtime_usage")


def rd(path, dflt=None):
  try:
    return open(path).read().strip()
  except (OSError, UnicodeDecodeError):
    return dflt


def walk_files(base, names):
  out = []
  for n in names:
    p = os.path.join(base, n)
    if os.path.exists(p):
      v = rd(p)
      out.append((n, v))
  return out


class Report:
  def __init__(self):
    self.lines = []

  def h(self, title):
    self.lines.append("")
    self.lines.append("=" * 78)
    self.lines.append(" " + title)
    self.lines.append("=" * 78)

  def kv(self, k, v):
    self.lines.append("  %-30s %s" % (k, v))

  def raw(self, s):
    self.lines.append(s)

  def emit(self):
    text = "\n".join(self.lines)
    print(text, flush=True)
    return text


def sec_system(r):
  r.h("1. 系统")
  r.kv("时间", time.strftime("%Y-%m-%d %H:%M:%S"))
  r.kv("kernel", rd("/proc/sys/kernel/osrelease", "?"))
  r.kv("uptime", rd("/proc/uptime", "?").split()[0] + " s")
  r.kv("cmdline", rd("/proc/cmdline", "?"))
  r.kv("AGNOS", rd("/VERSION", "?") or rd("/etc/os-release", "?")[:60])


def sec_controller(r):
  r.h("2. USB 控制器 / xhci")
  base = "/sys/devices/platform/soc/a600000.ssusb"
  r.kv("a600000.ssusb 存在", os.path.isdir(base))
  for n in ("mode", "role", "state", "driver_override"):
    r.kv("  " + n, rd(os.path.join(base, n), "-"))
  pm = os.path.join(base, "power")
  for n, v in walk_files(pm, PM_ATTRS):
    r.kv("  power/" + n, v)

  r.raw("")
  r.raw("  xhci 模块参数：")
  modp = "/sys/module/xhci_hcd/parameters"
  if os.path.isdir(modp):
    for n in sorted(os.listdir(modp)):
      r.kv("    " + n, rd(os.path.join(modp, n)))
  else:
    r.raw("    （没有 xhci_hcd 模块目录 —— 可能编进内核）")

  r.raw("")
  r.raw("  PCI 上的 xhci 设备：")
  found = False
  for pci in sorted(glob.glob("/sys/bus/pci/devices/*")):
    cls = rd(pci + "/class", "")
    if not cls.startswith("0x0c03"):
      continue
    found = True
    r.kv("    " + os.path.basename(pci),
         "driver=%s power/control=%s irq=%s" % (
           os.path.basename(os.readlink(pci + "/driver")) if os.path.exists(pci + "/driver") else "无",
           rd(pci + "/power/control", "?"), rd(pci + "/irq", "?")))
  if not found:
    r.raw("    （没有 class 0x0c03 的 PCI 设备）")


def sec_ports(r):
  r.h("3. 端口状态（最关键：over_current_count / disabled）")
  ports = sorted(glob.glob("/sys/bus/usb/devices/usb*-*:*/usb*-port*"))
  if not ports:
    r.raw("  没找到任何 usbX-portN 目录（root hub 可能没注册）")
  for p in ports:
    r.raw("")
    r.raw("  %s" % p.replace("/sys/bus/usb/devices/", ""))
    for n, v in walk_files(p, PORT_ATTRS):
      flag = ""
      if n == "over_current_count" and v not in (None, "0"):
        flag = "   <== 过流！供电/线材问题"
      if n == "disabled" and v == "1":
        flag = "   <== 端口被禁用，必须清掉才能重试"
      if n == "connect_type" and v and v != "unknown":
        flag = "   <== 有东西接着"
      r.kv("    " + n, str(v) + flag)


def sec_devices(r):
  r.h("4. USB 设备树")
  devs = [d for d in sorted(glob.glob("/sys/bus/usb/devices/*")) if os.path.exists(d + "/idVendor")]
  if not devs:
    r.raw("  总线上没有任何设备（连 root hub 都没有？）")
  for d in devs:
    vid = rd(d + "/idVendor", "????")
    pid = rd(d + "/idProduct", "????")
    try:
      key = (int(vid, 16), int(pid, 16))
    except ValueError:
      key = (-1, -1)
    mark = ""
    if key in CHESTNUT_IDS:
      mark = "  <== CHESTNUT"
    elif key in ROM_IDS:
      mark = "  <== ROM bootloader"
    r.kv(os.path.basename(d), "%s:%s speed=%s ver=%s %r %s" % (
      vid, pid, rd(d + "/speed", "?"), rd(d + "/version", "?"),
      rd(d + "/product", ""), mark))
    for n, v in walk_files(d + "/power", PM_ATTRS):
      if v not in (None, ""):
        r.raw("        power/%-24s %s" % (n, v))
    perm = "/dev/bus/usb/%03d/%03d" % (
      int(rd(d + "/busnum", "0") or 0), int(rd(d + "/devnum", "0") or 0))
    if os.path.exists(perm):
      st = os.stat(perm)
      r.raw("        %-32s mode=%04o uid=%d gid=%d" % (
        perm, st.st_mode & 0o777, st.st_uid, st.st_gid))


def sec_drivers(r):
  r.h("5. 驱动绑定 / 抢占")
  for drv in ("usb-storage", "uas", "usbfs", "usbhid", "cdc_acm"):
    base = "/sys/bus/usb/drivers/" + drv
    if not os.path.isdir(base):
      r.kv(drv, "（驱动不存在）")
      continue
    bound = [n for n in sorted(os.listdir(base)) if ":" in n]
    r.kv(drv, "%d 个绑定: %s" % (len(bound), ", ".join(bound) if bound else "无"))

  r.raw("")
  r.raw("  模块：")
  try:
    for line in open("/proc/modules"):
      nm = line.split()[0]
      if any(k in nm for k in ("usb", "xhci", "dwc", "storage", "uas")):
        r.raw("    " + line.strip()[:100])
  except OSError:
    r.raw("    读不到 /proc/modules")


def sec_udev(r):
  r.h("6. 权限辅助脚本 / udev 规则（egpu_usb_perm.sh 已缺失）")
  for p in ("/data/openpilot/openpilot/system/egpu_usb_perm.sh",
            "/data/openpilot/openpilot/system/egpu_usb_diag.sh",
            "/data/tools/chestnut_power_on.py"):
    r.kv(p.replace("/data/openpilot/openpilot/", ""), "存在" if os.path.exists(p) else "缺失")
  rules = glob.glob("/etc/udev/rules.d/*") + glob.glob("/usr/lib/udev/rules.d/50-udev-default.rules")
  hits = []
  for f in rules:
    try:
      txt = open(f, errors="replace").read()
    except OSError:
      continue
    if "usb" in txt and ("MODE" in txt or "TAG" in txt):
      hits.append(os.path.basename(f))
  r.kv("含 usb+MODE 的 udev 规则", ", ".join(sorted(set(hits))) or "无")
  r.kv("当前用户", "uid=%d euid=%d" % (
    getattr(os, "getuid", lambda: -1)(), getattr(os, "geteuid", lambda: -1)()))


def sec_dmesg(r):
  r.h("7. dmesg 枚举指纹统计")
  try:
    p = subprocess.run(["dmesg"], capture_output=True, timeout=20,
                       encoding="utf-8", errors="replace")
    lines = (p.stdout or "").splitlines()
  except Exception as e:
    r.raw("  读 dmesg 失败: %r" % (e,))
    return
  keys = ("Cannot enable", "unable to enumerate", "new full-speed", "new high-speed",
          "new SuperSpeed", "error -71", "error -32", "error -110", "setup address",
          "attempt power cycle", "usb-storage", "over-current", "USB disconnect",
          "device not accepting address", "link state")
  counts = {k: 0 for k in keys}
  for ln in lines:
    for k in keys:
      if k in ln:
        counts[k] += 1
  for k in keys:
    r.kv(k, counts[k])
  r.raw("")
  r.raw("  最后 25 行：")
  for ln in lines[-25:]:
    r.raw("    " + ln.strip()[:120])


def main():
  ap = argparse.ArgumentParser(description="USB 枚举层只读诊断")
  ap.add_argument("--out", default="/data/openpilot/openpilot/_chk/usb_enum_diag.txt")
  args = ap.parse_args()

  r = Report()
  r.h("USB 枚举层诊断  (只读，不修改任何状态)")
  sec_system(r)
  sec_controller(r)
  sec_ports(r)
  sec_devices(r)
  sec_drivers(r)
  sec_udev(r)
  sec_dmesg(r)
  r.h("诊断结束")
  text = r.emit()

  try:
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
      f.write(text + "\n")
    print("\n已写 %s" % args.out, flush=True)
  except OSError as e:
    print("\n写报告失败: %r" % (e,), flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
