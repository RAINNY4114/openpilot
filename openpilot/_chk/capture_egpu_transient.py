#!/usr/bin/env python3
"""capture_egpu_transient.py —— 抓 eGPU「橙色闪一下就消失」的瞬态

为什么需要它：橙色图标只持续几秒，`check_egpu_link_v3.py` 是快照式体检，抓不到。
本脚本每 0.25 s 轮询一次，只打印**变化**，把整个生命周期记成时间线。

橙色（FAILED/UNCOMPILED）的代码来源（已核实）：
  selfdrive/ui/ui_state.py:225-232   offroad 分支
      detected = sm["deviceState"].chestnutPresent
      chestnut_state = FAILED if chestnut_alert else
                       READY if detected and chestnut_compiled else
                       UNCOMPILED if detected else DISCONNECTED
  selfdrive/ui/ui_state.py:267
      chestnut_alert = 任意 Offroad_Chestnut* 参数存在（severity>=0）
  ⇒ ★ 橙色与「设备是否在场」无关，只要 Offroad_Chestnut* 参数被写入就橙色

谁会写 Offroad_Chestnut*（system/hardware/chestnut/status.py:94-106）：
  Offroad_ChestnutUsbSlow       devices==1 且 speedMbps < 5000
  Offroad_ChestnutUpdateFailed  hardwared Chestnut.failed（刷固件 3 次全失败）
  Offroad_ChestnutNotDetected   仅 release-chestnut* 分支才可能（staging-chestnut 恒 False）
  ...

用法（必须 root 才能读 params 与 dwc3）：
  sudo python3 capture_egpu_transient.py --duration 120
  # 然后在窗口内插/拔显卡
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import time

USB_DEVICES = "/sys/bus/usb/devices"
PARAMS_DIR = "/data/params/d"
DWC3 = "/sys/kernel/debug/a600000.dwc3"

CHESTNUT_IDS = {"3801:0001", "add1:0001"}
ROM_IDS = {"174c:2464", "174c:2463"}
ALERT_PREFIX = "Offroad_Chestnut"


def read(path, default=None):
    try:
        with open(path, "r", errors="replace") as f:
            return f.read().strip()
    except Exception:
        return default


def usb_snapshot():
    """{devname: (speed, id, product)}，只收有 idVendor 的下游设备。"""
    out = {}
    for p in glob.glob(os.path.join(USB_DEVICES, "*")):
        name = os.path.basename(p)
        if "-" not in name or ":" in name:
            continue
        vid = read(os.path.join(p, "idVendor"))
        pid = read(os.path.join(p, "idProduct"))
        if not vid or not pid:
            continue
        out[name] = (read(os.path.join(p, "speed"), "?"),
                     f"{vid}:{pid}",
                     read(os.path.join(p, "product"), "-"))
    return out


def params_snapshot():
    """{param_name: 原始字节的可读表示}"""
    out = {}
    if not os.path.isdir(PARAMS_DIR):
        return None
    for p in glob.glob(os.path.join(PARAMS_DIR, ALERT_PREFIX + "*")):
        name = os.path.basename(p)
        try:
            raw = open(p, "rb").read()
        except Exception:
            raw = b""
        # openpilot Params 可能带 1 字节类型前缀；去掉不可打印首字节
        body = raw[1:] if raw and raw[0] < 0x20 else raw
        try:
            txt = body.decode("utf-8", "replace")
        except Exception:
            txt = repr(body)
        out[name] = txt[:80]
    return out


def flashing():
    """返回 flash.py 的进程数（0 = 没在刷）。"""
    try:
        r = subprocess.run(["pgrep", "-fc", "chestnut/flash.py"],
                           capture_output=True, text=True, timeout=5)
        return int(r.stdout.strip() or 0)
    except Exception:
        return -1


def link_state():
    v = read(os.path.join(DWC3, "link_state"))
    return v if v else None


def fmt_usb(d):
    if not d:
        return "（无）"
    parts = []
    for name, (speed, ident, prod) in sorted(d.items()):
        tag = "★" if ident in CHESTNUT_IDS else ("⚠ROM" if ident in ROM_IDS else "  ")
        parts.append(f"{tag}{name} {ident} {speed}M {prod}")
    return " | ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.,
                    help="总时长秒数（默认 120）")
    ap.add_argument("--interval", type=float, default=0.25,
                    help="轮询间隔秒数（默认 0.25）")
    args = ap.parse_args()

    t0 = time.monotonic()

    def el():
        return f"[{time.monotonic() - t0:7.2f}s]"

    print("=" * 70)
    print(" eGPU 瞬态捕获 —— 现在插/拔显卡，本脚本只打印变化")
    print(f" 时长 {args.duration:.0f}s  间隔 {args.interval}s")
    print("=" * 70)

    prev_usb = usb_snapshot()
    prev_par = params_snapshot()
    prev_flash = flashing()
    prev_ls = link_state()

    print(f"{el()} 起始 USB : {fmt_usb(prev_usb)}")
    print(f"{el()} 起始 link: {prev_ls!r}   flash进程: {prev_flash}")
    if prev_par is None:
        print(f"{el()} ⚠ 读不到 {PARAMS_DIR}（需要 root）")
    else:
        print(f"{el()} 起始告警: {prev_par if prev_par else '（无 Offroad_Chestnut* 参数）'}")
    print("-" * 70)

    events = []

    while time.monotonic() - t0 < args.duration:
        time.sleep(args.interval)

        cur_usb = usb_snapshot()
        if cur_usb != prev_usb:
            for name in sorted(set(cur_usb) - set(prev_usb)):
                sp, ident, prod = cur_usb[name]
                kind = "CHESTNUT" if ident in CHESTNUT_IDS else ("ROM" if ident in ROM_IDS else "其他")
                line = f"{el()} + {name:<6} {ident} speed={sp}M [{kind}] product={prod}"
                print(line)
                events.append(line)
            for name in sorted(set(prev_usb) - set(cur_usb)):
                sp, ident, prod = prev_usb[name]
                line = f"{el()} - {name:<6} {ident} 消失（之前在 {sp}M）"
                print(line)
                events.append(line)
            for name in sorted(set(cur_usb) & set(prev_usb)):
                if cur_usb[name] != prev_usb[name]:
                    line = f"{el()} ~ {name:<6} {prev_usb[name][0]}M → {cur_usb[name][0]}M"
                    print(line)
                    events.append(line)
            prev_usb = cur_usb

        cur_par = params_snapshot()
        if cur_par is not None and cur_par != prev_par:
            for k in sorted(set(cur_par) - set(prev_par or {})):
                line = f"{el()} ⚑ 告警写入 {k} = {cur_par[k]!r}   ⇒ UI 变橙色"
                print(line)
                events.append(line)
            for k in sorted(set(prev_par or {}) - set(cur_par)):
                line = f"{el()} ⚑ 告警清除 {k}   ⇒ 橙色消失"
                print(line)
                events.append(line)
            for k in sorted(set(cur_par) & set(prev_par or {})):
                if cur_par[k] != prev_par[k]:
                    line = f"{el()} ⚑ 告警变化 {k}: {prev_par[k]!r} → {cur_par[k]!r}"
                    print(line)
                    events.append(line)
            prev_par = cur_par

        cur_flash = flashing()
        if cur_flash != prev_flash:
            line = f"{el()} ▶ flash.py 进程数 {prev_flash} → {cur_flash}" + \
                   ("   ⚠⚠ 正在写 SPI！" if cur_flash > 0 else "   （刷写结束）")
            print(line)
            events.append(line)
            prev_flash = cur_flash

        cur_ls = link_state()
        if cur_ls != prev_ls:
            line = f"{el()} link_state {prev_ls!r} → {cur_ls!r}"
            print(line)
            events.append(line)
            prev_ls = cur_ls

    print("-" * 70)
    print(f"{el()} 结束 USB : {fmt_usb(usb_snapshot())}")
    print(f"{el()} 结束 link: {link_state()!r}   flash进程: {flashing()}")
    p = params_snapshot()
    print(f"{el()} 结束告警: {p if p else '（无）'}")

    print("\n" + "=" * 70)
    print(f" 共 {len(events)} 个事件")
    print("=" * 70)
    if not events:
        print(" ❌ 全程无任何变化。可能原因：")
        print("    ① 没在窗口内插拔  ② 显卡完全没供电  ③ 端口被彻底卡死（无任何枚举尝试）")
    else:
        # 给结论
        saw_chestnut = any("CHESTNUT" in e for e in events)
        saw_rom = any("[ROM]" in e for e in events)
        saw_slow = any("告警写入 Offroad_ChestnutUsbSlow" in e for e in events)
        saw_updfail = any("告警写入 Offroad_ChestnutUpdateFailed" in e for e in events)
        saw_flash = any("正在写 SPI" in e for e in events)

        print(" 判读：")
        if saw_chestnut:
            print("  ✅ 设备确实以 3801:0001 / add1:0001 身份枚举过（固件在跑）")
        if saw_rom:
            print("  ⚠ 出现过 174c:2464/2463 ⇒ ROM bootloader 模式（固件没跑起来）")
        if saw_slow:
            print("  ★ 写过 Offroad_ChestnutUsbSlow ⇒ 设备以 <5000 Mbps 枚举过 —— 这就是橙色的直接来源")
        if saw_updfail:
            print("  ★★ 写过 Offroad_ChestnutUpdateFailed ⇒ 自动刷固件 3 次全失败 —— 橙色来源之二")
        if saw_flash:
            print("  ⚠⚠ 检测到 flash.py 在跑 ⇒ 每次插上都在写 SPI，链路不稳时有变砖风险")
        if saw_chestnut and not saw_slow and not saw_updfail:
            print("  （枚举过但没触发告警，橙色另有来源，看上面时间线）")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
