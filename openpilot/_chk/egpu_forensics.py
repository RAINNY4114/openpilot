#!/usr/bin/env python3
"""egpu_forensics.py —— eGPU 一次性取证，报告写进 _chk/ 供远程直接读取

背景：橙色图标「闪现后消失」需要抓瞬态 + 全量现场。
本脚本做两件事：
  阶段 A（默认 45 s）：盯住 USB / 告警参数 / flash.py / link_state 的变化，
                      期间你拔掉再插上一次显卡即可。
  阶段 B：把当前全部现场（lsusb / sysfs / dwc3 / dmesg / params / swaglog /
          perm log / chestnut_config / pkl 清单 / 进程 / 分支）落盘。

输出（两者内容相同）：
  /data/openpilot/openpilot/_chk/egpu_forensics_<时间戳>.txt
  /data/openpilot/openpilot/_chk/egpu_forensics_latest.txt

用法（必须 root）：
  sudo python3 /data/openpilot/openpilot/_chk/egpu_forensics.py
  sudo python3 /data/openpilot/openpilot/_chk/egpu_forensics.py --watch 0    # 跳过观察，立即取证
  sudo python3 /data/openpilot/openpilot/_chk/egpu_forensics.py --watch 90   # 观察更久
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
SSUSB = "/sys/devices/platform/soc/a600000.ssusb"
POWER = "/sys/class/power_supply"
OUT_DIR = "/data/openpilot/openpilot/_chk"
REPO = "/data/openpilot/openpilot"
MODELS_DIR = os.path.join(REPO, "selfdrive/modeld/models")

CHESTNUT_IDS = {"3801:0001", "add1:0001"}
ROM_IDS = {"174c:2464", "174c:2463"}
ALERT_PREFIX = "Offroad_Chestnut"
EXTRA_PARAMS = ("ChestnutLoading", "ChestnutActive", "ChestnutRetry",
                "ModelRunnerTypeCache", "UpdaterTargetBranch")

BUFFER = []          # 报告正文


def emit(line=""):
    BUFFER.append(line)


def read(path, default=None):
    try:
        with open(path, "r", errors="replace") as f:
            return f.read().strip()
    except Exception:
        return default


def sh(cmd, timeout=25):
    """执行命令，返回 (rc, stdout+stderr)。编码一律按 utf-8 + replace，避免解码炸掉。"""
    try:
        r = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True,
                           text=True, encoding="utf-8", errors="replace", timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return -1, f"<执行失败: {e}>"


def sec(title):
    emit("")
    emit("=" * 72)
    emit(title)
    emit("=" * 72)


# ---------------- 快照函数 ----------------

def usb_snapshot():
    out = {}
    for p in glob.glob(os.path.join(USB_DEVICES, "*")):
        name = os.path.basename(p)
        if "-" not in name or ":" in name:
            continue
        vid = read(os.path.join(p, "idVendor"))
        pid = read(os.path.join(p, "idProduct"))
        if not vid or not pid:
            continue
        out[name] = (read(os.path.join(p, "speed"), "?"), f"{vid}:{pid}",
                     read(os.path.join(p, "product"), "-"))
    return out


def params_snapshot():
    out = {}
    if not os.path.isdir(PARAMS_DIR):
        return None
    for p in glob.glob(os.path.join(PARAMS_DIR, ALERT_PREFIX + "*")):
        try:
            raw = open(p, "rb").read()
        except Exception:
            raw = b""
        out[os.path.basename(p)] = raw
    return out


def flash_count():
    rc, out = sh("ps -eo args 2>/dev/null | grep -c '[c]hestnut/flash.py'")
    try:
        return int(out.strip().splitlines()[-1])
    except Exception:
        return -1


def link_state():
    return read(os.path.join(DWC3, "link_state"))


def fmt_usb(d):
    if not d:
        return "（无下游设备）"
    return " | ".join(f"{n} {i} {s}M {p}" for n, (s, i, p) in sorted(d.items()))


# ---------------- 阶段 A：观察 ----------------

def watch_phase(duration):
    sec(f"阶段 A —— 瞬态观察（{duration:.0f} s）")
    emit("★ 现在请：拔掉显卡 → 等 3 秒 → 重新插上（只做一次）")
    emit("")

    t0 = time.monotonic()
    prev_usb, prev_par = usb_snapshot(), params_snapshot()
    prev_flash, prev_ls = flash_count(), link_state()
    n_events = 0

    emit(f"[  0.00s] 起始 USB : {fmt_usb(prev_usb)}")
    emit(f"[  0.00s] 起始 link_state={prev_ls!r}  flash.py={prev_flash}")
    if prev_par is None:
        emit("[  0.00s] ⚠ 读不到 params 目录（需要 root）")
    else:
        emit(f"[  0.00s] 起始告警: {sorted(prev_par) if prev_par else '（无）'}")

    while time.monotonic() - t0 < duration:
        time.sleep(0.25)
        el = time.monotonic() - t0

        cur = usb_snapshot()
        if cur != prev_usb:
            for n in sorted(set(cur) - set(prev_usb)):
                sp, ident, prod = cur[n]
                kind = "CHESTNUT" if ident in CHESTNUT_IDS else ("ROM" if ident in ROM_IDS else "其他")
                emit(f"[{el:7.2f}s] + {n:<6} {ident} speed={sp}M [{kind}] product={prod}")
                n_events += 1
            for n in sorted(set(prev_usb) - set(cur)):
                emit(f"[{el:7.2f}s] - {n:<6} {prev_usb[n][1]} 消失（原 {prev_usb[n][0]}M）")
                n_events += 1
            for n in sorted(set(cur) & set(prev_usb)):
                if cur[n] != prev_usb[n]:
                    emit(f"[{el:7.2f}s] ~ {n:<6} {prev_usb[n][0]}M → {cur[n][0]}M")
                    n_events += 1
            prev_usb = cur

        cp = params_snapshot()
        if cp is not None and cp != prev_par:
            for k in sorted(set(cp) - set(prev_par or {})):
                body = cp[k][1:] if cp[k] and cp[k][0] < 0x20 else cp[k]
                emit(f"[{el:7.2f}s] ⚑ 告警写入 {k} = {body[:60]!r}   ⇒ UI 变橙色")
                n_events += 1
            for k in sorted(set(prev_par or {}) - set(cp)):
                emit(f"[{el:7.2f}s] ⚑ 告警清除 {k}   ⇒ 橙色消失")
                n_events += 1
            prev_par = cp

        cf = flash_count()
        if cf != prev_flash:
            emit(f"[{el:7.2f}s] ▶ flash.py {prev_flash} → {cf}" +
                 ("   ⚠⚠ 正在写 SPI" if cf > 0 else "   （结束）"))
            n_events += 1
            prev_flash = cf

        cl = link_state()
        if cl != prev_ls:
            emit(f"[{el:7.2f}s] link_state {prev_ls!r} → {cl!r}")
            n_events += 1
            prev_ls = cl

    emit("")
    emit(f"观察结束：共 {n_events} 个事件")
    emit(f"结束 USB : {fmt_usb(usb_snapshot())}")
    emit(f"结束 link_state={link_state()!r}  flash.py={flash_count()}")
    p = params_snapshot()
    emit(f"结束告警: {sorted(p) if p else '（无）'}")
    return n_events


# ---------------- 阶段 B：全量取证 ----------------

def dump_all():
    sec("B1. 身份 / 环境")
    emit(f"uptime        : {read('/proc/uptime')}")
    emit(f"boot_id       : {read('/proc/sys/kernel/random/boot_id')}")
    emit(f"date (UTC)    : {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())}")
    emit(f"root          : {os.geteuid() == 0 if hasattr(os, 'geteuid') else '?'}")
    emit("")
    emit(f"git branch    : {sh(f'git -C {REPO} rev-parse --abbrev-ref HEAD')[1].strip()}")
    emit(f"git HEAD      : {sh(f'git -C {REPO} log -1 --oneline')[1].strip()}")
    emit(f"CHESTNUT_FW   : {read(os.path.join(REPO, 'common/hardware/usb.py')) is not None}")

    # 直接从源码抓常量，最可靠
    usb_py = read(os.path.join(REPO, "common/hardware/usb.py"), "") or ""
    for line in usb_py.splitlines():
        if "CHESTNUT_FW_VERSION" in line or "CHESTNUT_USB_PRODUCT" in line:
            emit(f"  [usb.py] {line.strip()}")

    sec("B2. lsusb")
    rc, out = sh("lsusb")
    emit(out.strip() or "<读不到>")
    emit("")
    emit("--- lsusb -t ---")
    emit(sh("lsusb -t")[1].strip() or "<读不到>")
    emit("")
    for target in ("3801:0001", "add1:0001", "174c:2464", "174c:2463"):
        rc, out = sh(f"lsusb -v -d {target} 2>/dev/null | grep -iE "
                     f"'bcdUSB|idVendor|idProduct|iProduct|bDeviceClass|MaxPower|bmAttributes' | head -10")
        if out.strip():
            emit(f"--- {target} ---")
            emit(out.strip())

    sec("B3. /sys/bus/usb/devices 全量")
    for p in sorted(glob.glob(os.path.join(USB_DEVICES, "*"))):
        name = os.path.basename(p)
        if ":" in name:
            continue
        vid = read(os.path.join(p, "idVendor"))
        pid = read(os.path.join(p, "idProduct"))
        if not vid and not read(os.path.join(p, "speed")):
            continue
        info = {
            "speed": read(os.path.join(p, "speed")),
            "id": f"{vid}:{pid}" if vid else "-",
            "product": read(os.path.join(p, "product")),
            "version": read(os.path.join(p, "version")),
            "busnum": read(os.path.join(p, "busnum")),
            "devnum": read(os.path.join(p, "devnum")),
            "bDeviceClass": read(os.path.join(p, "bDeviceClass")),
        }
        tag = ""
        if info["id"] in CHESTNUT_IDS:
            tag = "  ★CHESTNUT"
        elif info["id"] in ROM_IDS:
            tag = "  ⚠ROM-BOOTLOADER"
        emit(f"{name:<10} {info}{tag}")

    sec("B4. dwc3 / ssusb 控制器")
    for node in ("mode", "link_state", "state", "maximum_speed", "is_otg"):
        v = read(os.path.join(DWC3, node))
        emit(f"debugfs/{node:<14} = {v!r}")
    for node in ("mode", "state", "power"):
        v = read(os.path.join(SSUSB, node))
        emit(f"ssusb/{node:<16} = {v!r}")
    emit("")
    emit("（★ 必须 root 才读得到 debugfs，普通用户读到空串）")

    sec("B5. dmesg 指纹")
    rc, dm = sh("dmesg")
    if rc != 0 or not dm.strip():
        emit("<dmesg 读不到，需要 root>")
    else:
        lines = dm.splitlines()
        emit(f"总行数 {len(lines)}")
        pats = [
            ("assigned bus number 3", "bus3 重注册"),
            ("assigned bus number 4", "bus4 重注册"),
            ("Cannot enable", "Cannot enable"),
            ("Cannot set link state", "Cannot set link state"),
            ("cannot disable", "cannot disable"),
            ("unable to enumerate USB device", "unable to enumerate"),
            ("attempt power cycle", "attempt power cycle"),
            ("new full-speed USB device", "new full-speed"),
            ("new SuperSpeed USB device", "new SuperSpeed"),
            ("invalid maxpacket", "invalid maxpacket"),
            ("QMP PHY: Update TYPEC CTRL", "QMP PHY TYPEC CTRL"),
            ("Type-C Sink connected", "Type-C Sink connected"),
            ("Type-C disconnect", "Type-C disconnect"),
            ("usb-storage", "usb-storage"),
            ("DWC3", "DWC3"),
        ]
        for pat, label in pats:
            n = sum(1 for ln in lines if pat in ln)
            if n:
                emit(f"  {label:<24} = {n}")
        emit("")
        emit("--- 最后 60 行含 usb/xhci/dwc3/QMP 的行 ---")
        sel = [ln for ln in lines if re.search(r"usb|xhci|dwc3|QMP|TYPEC", ln, re.I)]
        for ln in sel[-60:]:
            emit(ln)
        emit("")
        emit("--- Type-C Sink connected 首次出现时刻 ---")
        for ln in lines:
            if "Type-C Sink connected" in ln:
                emit(ln)
                break

    sec("B6. Params（告警 + chestnut 相关）")
    if not os.path.isdir(PARAMS_DIR):
        emit(f"<读不到 {PARAMS_DIR}>")
    else:
        keys = sorted(os.path.basename(p) for p in glob.glob(os.path.join(PARAMS_DIR, "*")))
        alerts = [k for k in keys if k.startswith(ALERT_PREFIX)]
        emit(f"Offroad_Chestnut* 参数（{len(alerts)} 个）:")
        for k in alerts:
            raw = open(os.path.join(PARAMS_DIR, k), "rb").read() if os.path.exists(
                os.path.join(PARAMS_DIR, k)) else b""
            body = raw[1:] if raw and raw[0] < 0x20 else raw
            emit(f"  {k:<32} raw={raw[:40]!r}  text={body[:60]!r}")
        if not alerts:
            emit("  （无 —— 说明当前没有告警，橙色不应出现）")
        emit("")
        for k in EXTRA_PARAMS:
            p = os.path.join(PARAMS_DIR, k)
            if os.path.exists(p):
                raw = open(p, "rb").read()
                body = raw[1:] if raw and raw[0] < 0x20 else raw
                emit(f"  {k:<32} text={body[:80]!r}")
            else:
                emit(f"  {k:<32} <不存在>")

    sec("B7. 进程")
    for name, pat in (("flash.py", "[c]hestnut/flash.py"),
                      ("modeld", "[m]odeld"),
                      ("hardwared", "[h]ardwared"),
                      ("ui", "[s]elfdrive.ui.ui")):
        rc, out = sh(f"ps -eo pid,etime,args 2>/dev/null | grep '{pat}'")
        emit(f"--- {name} ---")
        emit(out.strip() or "  （无）")

    sec("B8. swaglog 关键行（最近 3 个文件）")
    logs = sorted(glob.glob("/data/log/swaglog.*"))[-3:]
    if not logs:
        emit("<没有 swaglog>")
    for lf in logs:
        emit(f"--- {os.path.basename(lf)} ({os.path.getsize(lf)} B) ---")
        rc, out = sh(f"grep -aiE 'chestnut|eGPU|Cannot enable|flashing|firmware out of date|"
                     f"big model|combined pkl' {lf} | tail -25")
        emit(out.strip() or "  （无匹配）")

    sec("B9. /data/egpu_usb_perm.log（尾部）")
    for cand in ("/data/egpu_usb_perm.log", "/data/egpu_diag.jsonl", "/data/egpu_retry.json"):
        if os.path.exists(cand):
            emit(f"--- {cand} ({os.path.getsize(cand)} B) ---")
            emit(sh(f"tail -30 {cand}")[1].strip())
        else:
            emit(f"--- {cand} <不存在>")

    sec("B10. /data/chestnut_config（ROM 恢复依赖）")
    emit(sh("ls -la /data/chestnut_config/ 2>&1")[1].strip())

    sec("B11. 大模型 pkl 清单（chestnut_compiled 判据）")
    emit(f"MODELS_DIR = {MODELS_DIR}")
    emit(sh(f"ls -la {MODELS_DIR}/ 2>&1 | head -40")[1].strip())
    emit("")
    for need in ("big_driving_tinygrad.pkl",
                 "big_driving_warp_1344x760_tinygrad.pkl",
                 "big_driving_warp_1928x1208_tinygrad.pkl"):
        p = os.path.join(MODELS_DIR, need)
        if os.path.exists(p):
            with open(p, "rb") as f:
                head = f.read(16)
            emit(f"  ✅ {need:<42} {os.path.getsize(p):>12} B  head={head[:10].hex(' ')}")
        else:
            emit(f"  ❌ {need:<42} <缺失>")

    sec("B12. Type-C / 供电")
    for k in ("typec_cc_orientation", "typec_power_role", "typec_mode",
              "voltage_now", "pd_active", "real_type", "present", "current_now"):
        v = read(os.path.join(POWER, "usb", k))
        if v is not None:
            emit(f"usb/{k:<22} = {v}")
    emit(f"dc/online{'':<14} = {read(os.path.join(POWER, 'dc', 'online'))}")

    sec("B13. /dev/bus/usb 节点权限")
    emit(sh("ls -la /dev/bus/usb/003/ /dev/bus/usb/004/ 2>&1 | grep -v '^total'")[1].strip())


# ---------------- 主流程 ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=float, default=45.,
                    help="阶段 A 观察秒数（默认 45，0=跳过）")
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    if hasattr(os, "geteuid") and os.geteuid() != 0:
        print("⚠ 不是 root：params / dmesg / dwc3 会读不到，请用 sudo 重跑")

    t_start = time.strftime("%Y%m%d_%H%M%S")
    emit(f"eGPU 取证报告  生成时间 {time.strftime('%Y-%m-%d %H:%M:%S')}")
    emit(f"主机 {os.uname().nodename if hasattr(os, 'uname') else '?'}")

    if args.watch > 0:
        n = watch_phase(args.watch)
        emit("")
        emit(f"【观察阶段事件数 = {n}】")

    dump_all()

    text = "\n".join(BUFFER) + "\n"

    # 先打印到屏幕
    print(text)

    # 落盘
    try:
        os.makedirs(args.out_dir, exist_ok=True)
        p1 = os.path.join(args.out_dir, f"egpu_forensics_{t_start}.txt")
        p2 = os.path.join(args.out_dir, "egpu_forensics_latest.txt")
        for p in (p1, p2):
            with open(p, "w") as f:
                f.write(text)
        print("=" * 72)
        print(f"报告已写入:\n  {p1}\n  {p2}")
        print(f"共 {len(text)} 字节")
        print("=" * 72)
    except Exception as e:
        print(f"⚠ 落盘失败: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
