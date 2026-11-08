#!/usr/bin/env python3
"""check_egpu_link_v3.py —— eGPU(chestnut) USB 链路体检 v3

判据全部来自 2026-10-12~10-17 的实测，见：
  chestnut-egpu-link-broken-report.md §三/§五/§九/§十

  ✅ 健康：mode=host + link_state=U0 + 4-1/speed=5000 + bcdUSB 3.00/3.20 + Cannot enable=0
  ❌ 故障：usb3-port1/usb4-port1 "Cannot enable" → new full-speed(12M)
           + "invalid maxpacket 512, setting to 64"，link_state = Rx.Detect / Poll
  ★★ /sys/kernel/debug/a600000.dwc3/{link_state,mode} 必须 root 读，
     普通用户读到的是空串 —— 10-15 曾把「空」误判成异常。

v3 相对 10-15 的 check_egpu_link.sh 新增：
  * ROM bootloader 识别（174c:2464 / 174c:2463 ⇒ 固件没跑起来，另一条修复路）
  * "Cannot set link state" / "cannot disable (err=-32)" 计数（今天的新形态）
  * Type-C Sink connected 时刻（判「开机前插好」还是「开机后补插」）
  * 不 import openpilot，offroad 安全

用法：
  sudo python3 check_egpu_link_v3.py
  sudo python3 check_egpu_link_v3.py --dmesg-lines 4000
  python3 check_egpu_link_v3.py            # 非 root 也能跑，但 link_state 会标 UNKNOWN
"""

import argparse
import glob
import os
import re
import subprocess
import sys

DWC3_DEBUGFS = "/sys/kernel/debug/a600000.dwc3"
SSUSB_MODE = "/sys/devices/platform/soc/a600000.ssusb/mode"
USB_DEVICES = "/sys/bus/usb/devices"
POWER_SUPPLY = "/sys/class/power_supply"

CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}

# dmesg 指纹：(标签, 正则)
FINGERPRINTS = [
    ("bus3 重注册",        r"assigned bus number 3"),
    ("bus4 重注册",        r"assigned bus number 4"),
    ("Cannot enable",      r"Cannot enable"),
    ("Cannot set link state", r"Cannot set link state"),
    ("cannot disable",     r"cannot disable"),
    ("unable to enumerate", r"unable to enumerate USB device"),
    ("attempt power cycle", r"attempt power cycle"),
    ("new full-speed",     r"new full-speed USB device"),
    ("new SuperSpeed",     r"new SuperSpeed USB device"),
    ("invalid maxpacket",  r"invalid maxpacket"),
    ("QMP PHY TYPEC CTRL", r"QMP PHY: Update TYPEC CTRL"),
    ("Type-C Sink connected", r"Type-C Sink connected"),
    ("DWC3 low power",     r"DWC3 in low power mode"),
]

C_OK, C_BAD, C_WARN, C_DIM, C_END = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def color(txt, c):
    return f"{c}{txt}{C_END}" if sys.stdout.isatty() else txt


def is_root():
    return hasattr(os, "geteuid") and os.geteuid() == 0


def read_text(path):
    try:
        with open(path, "r", errors="replace") as f:
            return f.read().strip()
    except Exception:
        return None


def sudo_read(path):
    """root 直接读；否则用 sudo -n（非交互）。失败返回 None。"""
    if is_root():
        return read_text(path)
    if not os.path.exists(path):
        return None
    try:
        r = subprocess.run(["sudo", "-n", "cat", path],
                           capture_output=True, text=True, timeout=8)
        if r.returncode == 0:
            return r.stdout.strip()
    except Exception:
        pass
    return None


def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout if r.returncode == 0 else None
    except Exception:
        return None


def get_dmesg():
    for cmd in (["dmesg"], ["sudo", "-n", "dmesg"]):
        out = run(cmd, timeout=30)
        if out:
            return out
    return None


def parse_lsusb_ids():
    """返回 [(vid, pid, desc)]，读不到返回 None。"""
    out = run(["lsusb"])
    if out is None:
        return None
    devs = []
    for line in out.splitlines():
        m = re.search(r"ID\s+([0-9a-fA-F]{4}):([0-9a-fA-F]{4})\s*(.*)$", line)
        if m:
            devs.append((int(m.group(1), 16), int(m.group(2), 16), m.group(3).strip()))
    return devs


def usb_dev_nodes():
    """返回 /sys/bus/usb/devices 下的非 root-hub 设备节点名列表。"""
    names = []
    for p in sorted(glob.glob(os.path.join(USB_DEVICES, "*"))):
        name = os.path.basename(p)
        if "-" not in name or ":" in name:
            continue
        names.append(name)
    return names


def node_info(name):
    base = os.path.join(USB_DEVICES, name)
    return {
        "name": name,
        "speed": read_text(os.path.join(base, "speed")),
        "product": read_text(os.path.join(base, "product")),
        "vendor": read_text(os.path.join(base, "manufacturer")),
        "idVendor": read_text(os.path.join(base, "idVendor")),
        "idProduct": read_text(os.path.join(base, "idProduct")),
        "bcdUSB": read_text(os.path.join(base, "version")),
        "busnum": read_text(os.path.join(base, "busnum")),
        "devnum": read_text(os.path.join(base, "devnum")),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dmesg-lines", type=int, default=0,
                    help="只统计最近 N 行 dmesg（0=全部）")
    args = ap.parse_args()

    ok = True          # 链路总判定
    rom_mode = False   # 是否卡在 ROM bootloader

    print("=" * 62)
    print(" eGPU(chestnut) USB 链路体检 v3   —— 判据来自 2026-10-12~10-17")
    print("=" * 62)
    print(f"root={is_root()}   uptime={read_text('/proc/uptime')}")

    # ---------- 1) lsusb：先分清「链路坏」还是「卡 ROM」 ----------
    print("\n--- 1) lsusb：chestnut / ASM2464 ---")
    devs = parse_lsusb_ids()
    if devs is None:
        print(color("  (读不到 lsusb)", C_WARN))
    else:
        hits = [d for d in devs if (d[0], d[1]) in CHESTNUT_IDS or (d[0], d[1]) in ROM_IDS]
        if not hits:
            print(color("  ❌ 没枚举到 3801/add1/174c 任何一个", C_BAD))
            ok = False
        for vid, pid, desc in hits:
            if (vid, pid) in ROM_IDS:
                rom_mode = True
                print(color(f"  ⚠⚠ ROM BOOTLOADER: {vid:04x}:{pid:04x}  {desc}", C_BAD))
                print(color("     ⇒ 固件没跑起来，这不是链路问题！走 flash.py 恢复路径。", C_BAD))
            else:
                print(color(f"  ✅ 正常固件: {vid:04x}:{pid:04x}  {desc}", C_OK))
        others = [d for d in devs if d not in hits]
        print(color(f"  其他设备 {len(others)} 个: " +
                    ", ".join(f"{v:04x}:{p:04x}" for v, p, _ in others[:8]), C_DIM))

    # ---------- 2) USB 设备节点与速度 ----------
    print("\n--- 2) USB 设备节点 / 速度 ---")
    nodes = usb_dev_nodes()
    if not nodes:
        print(color("  ❌ 总线上没有任何下游设备（只有 root hub）", C_BAD))
        ok = False
    ss_node = None
    for name in nodes:
        info = node_info(name)
        speed = info["speed"] or "?"
        is_chest = (info["idVendor"], info["idProduct"]) in {
            (f"{v:04x}", f"{p:04x}") for v, p in (CHESTNUT_IDS | ROM_IDS)}
        mark = "★" if is_chest else " "
        print(f"  {mark} {name:<8} speed={speed:<6} bcdUSB={info['bcdUSB'] or '?':<6} "
              f"id={info['idVendor'] or '?'}:{info['idProduct'] or '?'} "
              f"product={info['product'] or '-'}")
        if is_chest and speed == "5000":
            ss_node = name

    print("\n--- 3) SuperSpeed 判定 ---")
    n41 = node_info("4-1")
    if os.path.exists(os.path.join(USB_DEVICES, "4-1")):
        sp = n41["speed"]
        if sp == "5000":
            print(color(f"  ✅ 4-1 存在，speed={sp}  bcdUSB={n41['bcdUSB']}", C_OK))
        else:
            print(color(f"  ❌ 4-1 存在但 speed={sp}（应为 5000）", C_BAD))
            ok = False
    else:
        print(color("  ❌ /sys/bus/usb/devices/4-1 不存在 —— SuperSpeed 链路没起来", C_BAD))
        ok = False

    # ---------- 4) dwc3 调试节点（必须 root） ----------
    print("\n--- 4) dwc3 调试节点（★必须 root，否则空串） ---")
    ls_path = os.path.join(DWC3_DEBUGFS, "link_state")
    md_path = os.path.join(DWC3_DEBUGFS, "mode")
    link_state = sudo_read(ls_path)
    dwc3_mode = sudo_read(md_path)
    sysfs_mode = read_text(SSUSB_MODE)

    if link_state is None:
        print(color(f"  link_state = UNKNOWN（读不到 {ls_path}）", C_WARN))
        print(color("     ⇒ 用 root 重跑本脚本，或手工: sudo cat " + ls_path, C_WARN))
        ok = False
    else:
        state_ok = (link_state == "U0")
        c = C_OK if state_ok else C_BAD
        print(color(f"  link_state = [{link_state}]   （应为 U0）", c))
        if not state_ok:
            print(color(f"     ⇒ 停在 {link_state}，SuperSpeed 链路训练未成功", C_BAD))
            ok = False
    print(f"  dwc3 mode  = [{dwc3_mode if dwc3_mode is not None else 'UNKNOWN'}]   （应为 host）")
    print(f"  ssusb mode = [{sysfs_mode if sysfs_mode is not None else 'UNKNOWN'}]"
          f"   {C_DIM if sys.stdout.isatty() else ''}"
          f"(仅控制器角色，不反映链路，别单独拿它判){C_END if sys.stdout.isatty() else ''}")

    # ---------- 5) dmesg 指纹 ----------
    print("\n--- 5) dmesg 指纹（健康 vs 故障，见 10-15 §三）---")
    dm = get_dmesg()
    if dm is None:
        print(color("  (读不到 dmesg，需要 root)", C_WARN))
    else:
        lines = dm.splitlines()
        if args.dmesg_lines > 0:
            lines = lines[-args.dmesg_lines:]
        counts = {}
        for label, pat in FINGERPRINTS:
            rx = re.compile(pat)
            counts[label] = sum(1 for ln in lines if rx.search(ln))
        print(f"  （统计范围：最近 {len(lines)} 行）")
        for label, _ in FINGERPRINTS:
            n = counts[label]
            if n == 0:
                continue
            bad = label in ("Cannot enable", "Cannot set link state", "cannot disable",
                            "unable to enumerate", "new full-speed", "invalid maxpacket")
            print(f"  {'❌' if bad else '  '} {label:<24} = {n}")

        # 关键三连判读
        if counts["Cannot enable"] > 0 or counts["Cannot set link state"] > 0:
            print(color("  ⇒ 出现 Cannot enable / Cannot set link state：端口 enable 阶段就失败", C_BAD))
        if counts["cannot disable"] > 0:
            print(color("  ⇒ 出现 cannot disable：xhci 无法复位端口（今天的新形态，比 10-15 更重）", C_BAD))
        if counts["new full-speed"] > 0:
            print(color("  ⇒ 掉到 full-speed(12 Mbps)：SS 训练失败后的降级", C_BAD))
        if counts["new SuperSpeed"] > 0 and counts["Cannot enable"] == 0:
            print(color("  ✅ 有 SuperSpeed 枚举且无 Cannot enable", C_OK))

        # 连接时机（10-15 的关键新线索）
        t = None
        for ln in lines:
            if "Type-C Sink connected" in ln:
                m = re.match(r"\[\s*([0-9.]+)\]", ln)
                if m:
                    t = float(m.group(1))
                    break
        print(f"\n  Type-C Sink connected 首次出现: "
              f"{f'{t:.2f} s' if t is not None else '未见'}")
        if t is not None:
            if t <= 15:
                print(color(f"     ✅ {t:.1f}s ≤ 15s ⇒ 开机前就插好了线（健康案例都在 2.5~11.8s）", C_OK))
            else:
                print(color(f"     ⚠ {t:.1f}s > 15s ⇒ 开机后才插的线（10-15 失败案例是 49s）", C_WARN))

    # ---------- 6) Type-C / 供电侧（只做记录，不作判据） ----------
    print("\n--- 6) Type-C / 供电侧（仅记录：10-15 已证明 VBUS 高低不是判据）---")
    for key in ("typec_cc_orientation", "typec_power_role", "typec_mode",
                "voltage_now", "pd_active", "real_type", "present"):
        v = read_text(os.path.join(POWER_SUPPLY, "usb", key))
        if v is not None:
            print(f"  usb/{key:<22} = {v}")
    v = read_text(os.path.join(POWER_SUPPLY, "dc", "online"))
    if v is not None:
        print(f"  dc/online{'':<14} = {v}   （0 = 没接车电，10-15 成功时也是 0）")

    # ---------- 7) 设备节点权限 ----------
    print("\n--- 7) /dev/bus/usb 节点权限（应为 0666）---")
    found = False
    for bus in ("003", "004"):
        d = f"/dev/bus/usb/{bus}"
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            p = os.path.join(d, f)
            try:
                st = os.stat(p)
                mode = oct(st.st_mode & 0o777)
            except Exception:
                continue
            mark = "✅" if mode == "0o666" else "⚠"
            print(f"  {mark} {p}  {mode}  uid={st.st_uid} gid={st.st_gid}")
            found = True
    if not found:
        print(color("  (003/004 下无节点)", C_DIM))

    # ---------- 8) 判定 ----------
    print("\n" + "=" * 62)
    if rom_mode:
        print(color("⚠ 设备卡在 ROM BOOTLOADER —— 这不是链路问题", C_BAD))
        print("  1) ls -l /data/chestnut_config/     # recover_from_rom() 需要 <nodename>.bin")
        print("  2) 两端断电 ≥30s 后重新上电，再看是否回到 3801:0001")
        print("  3) 仍为 174c:xxxx → 需要刷固件（链路不稳时刷 SPI 有风险，先确保 5000M）")
    elif ok:
        print(color("✅ SuperSpeed 链路合格。", C_OK))
        print("   下一步：offroad 下跑 egpu_smoke.py --run 验证 tinygrad 真能开 AMD 设备，")
        print("   然后点火跑一圈，确认 modelV2.big=True。")
    else:
        print(color("❌ 链路不合格。", C_BAD))
        print("   按 10-15 §九 验证过的顺序，别做无效动作：")
        print("   ★1 设备 + 盒子【两端同时】断电 ≥30 秒，再一起上电")
        print("      （拔插线 ❌ / 只给坞断电 10s ❌ / restart comma ❌ —— 都已证明无效）")
        print("    2 上电前先把线插好（成功案例 Type-C 都在 2.5~11.8s 建立）")
        print("    3 重跑本脚本，验收：link_state=U0 + 4-1/speed=5000 + Cannot enable=0")
        print("    4 仍失败 → 换确认支持 USB3.2 的线，【重做第 1 步】")
        print("    5 仍失败 → 普通 USB3 U 盘插同一口做对照，切开是「口坏」还是「线/坞坏」")
    print("=" * 62)
    return 0 if (ok and not rom_mode) else 1


if __name__ == "__main__":
    sys.exit(main())
