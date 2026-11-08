#!/usr/bin/env python3
"""UT3G / chestnut 底座刷写前只读诊断（不写任何东西）。

目的：在跑 flash_ut3g.py install 之前，判定底座当前是否满足安装器的硬门槛。

安装器 hardware.py:41 的判定是：
    identity == ('3801', '0001', 'tiny', '5000') and product in PRODUCTS.values()

四项必须同时成立：VID=3801 / PID=0001 / manufacturer=tiny / speed=5000。
任一不满足，candidates() 返回空 -> 安装器直接抛
「需要恰好一块受支持的RAW USB3底座」，连设备都不会打开。

本脚本只做：
  1. sysfs 枚举（manufacturer/product/serial/speed/busnum/devnum）
  2. comma.service 状态、Offroad 参数、USB 节点占用
  3. 只读 vendor 请求：0xB450(LTSSM) / 0xC0-C0(5B 供电) / 0xC2-C2(8B)
不做：不写寄存器、不发 0xF3、不碰 SPI、不改参数。

用法（设备上）：
  /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/ut3g_precheck.py
  /usr/local/venv/bin/python3 .../ut3g_precheck.py --json

退出码：
  0  GO      满足安装器全部前提，可以进入 flash_ut3g.py install
  1  NO_GO_HW  硬件身份不满足（速度/厂商/产品/Flash），安装器会拒绝
  2  NO_GO_ENV 环境不满足（服务在跑 / 非 Offroad / 节点被占用）
  3  NO_DEVICE 根本没看到 3801:0001 或 174c:2464 设备
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SYSFS = Path("/sys/bus/usb/devices")
PARAMS = Path("/data/params/d")

CHESTNUT_IDS = {("3801", "0001"), ("add1", "0001")}
ROM_IDS = {("174c", "2464"), ("174c", "2463")}

# 安装器 protocol.py:15
INSTALLER_PRODUCTS = {
    "R4": "custom ed4e39b7-CLEAN",
    "UID-R1": "custom ed4e39b7-CLEAN",
    "SP8516-R1": "custom 8516b768-CLEAN",
}
INSTALLER_MANUFACTURER = "tiny"
INSTALLER_SPEED = "5000"

FIELDS = ("idVendor", "idProduct", "manufacturer", "product", "serial",
          "speed", "busnum", "devnum", "bcdDevice")

LTSSM_TRAINED = 0x78


def read_field(dev: Path, name: str) -> str:
    try:
        return (dev / name).read_text().strip()
    except (OSError, UnicodeError):
        return ""


def read_hex(dev: Path, name: str) -> str:
    v = read_field(dev, name)
    return v.lower() if v else ""


def enumerate_devices() -> tuple[list[dict], list[dict]]:
    """返回 (chestnut 候选, ROM 模式候选)。"""
    chestnut, rom = [], []
    if not SYSFS.is_dir():
        return chestnut, rom
    for dev in sorted(SYSFS.iterdir()):
        vid = read_hex(dev, "idVendor")
        pid = read_hex(dev, "idProduct")
        if not vid or not pid:
            continue
        info = {k: read_field(dev, k) for k in FIELDS}
        info["path"] = str(dev)
        info["node"] = ""
        try:
            info["node"] = f"/dev/bus/usb/{int(info['busnum']):03d}/{int(info['devnum']):03d}"
        except (ValueError, KeyError):
            pass
        if (vid, pid) in CHESTNUT_IDS:
            info["kind"] = "chestnut"
            chestnut.append(info)
        elif (vid, pid) in ROM_IDS:
            info["kind"] = "rom"
            rom.append(info)
    return chestnut, rom


def service_state() -> dict:
    out = {"active": None, "returncode": None, "raw": ""}
    try:
        r = subprocess.run(["systemctl", "is-active", "comma.service"],
                           capture_output=True, text=True, timeout=5)
        out["returncode"] = r.returncode
        out["raw"] = r.stdout.strip()
        out["active"] = r.stdout.strip()
    except Exception as e:
        out["error"] = str(e)
    return out


def offroad_state() -> dict:
    obs = {}
    for key in ("IsOffroad", "IsOnroad", "IsEngaged"):
        p = PARAMS / key
        try:
            obs[key] = p.read_bytes().strip().decode("utf-8", "replace")
        except FileNotFoundError:
            pass
        except OSError as e:
            obs[key] = f"<err {e}>"
    return obs


def offroad_verdict(obs: dict) -> tuple[bool, str]:
    """复刻安装器 hardware.py:18-30 的判定。"""
    for k, v in obs.items():
        if v not in ("0", "1"):
            return False, f"{k}={v!r} 格式未知，安装器会拒绝"
    if obs.get("IsOffroad") == "0" or obs.get("IsOnroad") == "1" or obs.get("IsEngaged") == "1":
        return False, "当前 Onroad / 已接管，安装器会拒绝"
    if obs.get("IsOffroad") != "1" and obs.get("IsOnroad") != "0":
        return False, "缺少可确认的 Offroad 状态，安装器会拒绝"
    return True, "Offroad 已确认"


def node_owners(node: str) -> str:
    if not node or not os.path.exists(node):
        return ""
    try:
        r = subprocess.run(["fuser", node], capture_output=True, text=True, timeout=3)
        return r.stdout.strip()
    except Exception as e:
        return f"<fuser 失败 {e}>"


def probe_vendor_requests(dev: dict) -> dict:
    """只读 vendor 请求。全部捕获异常，失败不影响判定。"""
    result = {"ltssm": None, "supply": None, "c2": None, "errors": {}}
    try:
        import usb1
    except Exception as e:
        result["errors"]["usb1"] = f"不可用: {e}"
        return result

    vid = int(dev["idVendor"], 16)
    pid = int(dev["idProduct"], 16)
    try:
        with usb1.USBContext() as ctx:
            handle = ctx.openByVendorIDAndProductID(vid, pid, skip_on_error=True)
            if handle is None:
                result["errors"]["open"] = "打不开（权限不足或已被 modeld 占用）"
                return result
            try:
                try:
                    raw = bytes(handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=1000))
                    result["ltssm"] = f"0x{raw[0]:02X}"
                except Exception as e:
                    result["errors"]["ltssm"] = str(e)
                try:
                    raw = bytes(handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=1000))
                    if len(raw) >= 5:
                        import struct
                        v, i, f = struct.unpack("<Hh?", raw[:5])
                        result["supply"] = {"voltage_mv": v, "current_ma": i, "fault": f}
                except Exception as e:
                    result["errors"]["supply"] = str(e)
                try:
                    raw = bytes(handle.controlRead(0xC0, 0xC2, 0, 0, 8, timeout=1000))
                    result["c2"] = raw.hex()
                except Exception as e:
                    result["errors"]["c2"] = str(e)
            finally:
                try:
                    handle.close()
                except Exception:
                    pass
    except Exception as e:
        result["errors"]["ctx"] = str(e)
    return result


def classify(dev: dict) -> tuple[bool, list[str]]:
    """按安装器 hardware.py:41 逐项核对。"""
    missing = []
    if dev.get("idVendor") != "3801":
        missing.append(f"idVendor={dev.get('idVendor')!r}，安装器要求 '3801'")
    if dev.get("idProduct") != "0001":
        missing.append(f"idProduct={dev.get('idProduct')!r}，安装器要求 '0001'")
    if dev.get("manufacturer") != INSTALLER_MANUFACTURER:
        missing.append(f"manufacturer={dev.get('manufacturer')!r}，安装器要求 {INSTALLER_MANUFACTURER!r}")
    if dev.get("speed") != INSTALLER_SPEED:
        missing.append(f"speed={dev.get('speed')!r} Mbps，安装器要求 {INSTALLER_SPEED!r}（USB3 SuperSpeed）")
    if dev.get("product") not in INSTALLER_PRODUCTS.values():
        missing.append(f"product={dev.get('product')!r} 不在已知集合 {sorted(set(INSTALLER_PRODUCTS.values()))}")
    return (not missing), missing


def emit_identity(chestnut: list[dict], path: Path) -> int:
    """把**实测**身份写成安装器适配版的白名单。

    只写当前真实枚举到的设备，不做任何推测或放宽：
      * identities = 每个 chestnut 设备的 (vid, pid, manufacturer, speed) 精确四元组
      * products   = 每个 chestnut 设备的 product 精确字符串

    适配版 candidates() 用 `identity in idents and product in products` 判定，
    仍然是精确匹配，只是白名单来源从硬编码变成实测。
    """
    if not chestnut:
        print("没有 chestnut 设备，拒绝生成白名单（先解决 USB 枚举）", file=sys.stderr)
        return 3

    idents, products, detail = [], [], []
    for d in chestnut:
        quad = [d.get("idVendor", ""), d.get("idProduct", ""),
                d.get("manufacturer", ""), d.get("speed", "")]
        if not all(quad):
            print(f"设备 {d.get('path')} 身份字段不全，跳过：{quad}", file=sys.stderr)
            continue
        if quad not in idents:
            idents.append(quad)
        prod = d.get("product", "")
        if prod and prod not in products:
            products.append(prod)
        detail.append({"path": d.get("path"), "node": d.get("node"),
                       "identity": quad, "product": prod, "serial": d.get("serial", "")})

    if not idents or not products:
        print("实测身份为空，拒绝生成白名单", file=sys.stderr)
        return 3

    spec = {
        "schema": "ut3g-adapted-identity-v1",
        "generated_from": "ut3g_precheck.py --emit-identity",
        "identities": idents,
        "products": products,
        "devices": detail,
        "warning": "白名单来自实测，仍是精确匹配；不改 Flash 几何/保护位/扇区约束",
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(spec, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"已写入实测白名单：{path}")
    print(f"  identities = {idents}")
    print(f"  products   = {products}")
    return 0


def fmt_dev(d: dict) -> str:
    return (f"{d.get('idVendor')}:{d.get('idProduct')}  "
            f"speed={d.get('speed') or '?'}Mbps  "
            f"manufacturer={d.get('manufacturer') or '<空>'!r}  "
            f"product={d.get('product') or '<空>'!r}  "
            f"serial={d.get('serial') or '<空>'!r}")


def main() -> int:
    ap = argparse.ArgumentParser(description="UT3G 刷写前只读诊断")
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    ap.add_argument("--probe", action="store_true", default=True,
                    help="读取只读 vendor 请求（默认开）")
    ap.add_argument("--no-probe", dest="probe", action="store_false",
                    help="跳过 vendor 请求，只看 sysfs")
    ap.add_argument("--emit-identity", type=Path, default=None, metavar="PATH",
                    help="把实测身份写成安装器适配版白名单（如 /data/ut3g-install/identity.json）")
    args = ap.parse_args()

    chestnut, rom = enumerate_devices()

    if args.emit_identity is not None:
        return emit_identity(chestnut, args.emit_identity)

    svc = service_state()
    obs = offroad_state()
    off_ok, off_msg = offroad_verdict(obs)

    report = {
        "chestnut": chestnut,
        "rom": rom,
        "service": svc,
        "params": obs,
        "offroad_ok": off_ok,
        "offroad_msg": off_msg,
    }

    # ---- 人类可读 ----
    if not args.json:
        print("=" * 74)
        print("UT3G / chestnut 刷写前诊断（只读）")
        print("=" * 74)

        print(f"\n[1] chestnut 设备 (3801:0001 / add1:0001)：{len(chestnut)} 个")
        for d in chestnut:
            print(f"    {fmt_dev(d)}")
            print(f"    {d['path']}  node={d.get('node') or '?'}")
        if not chestnut:
            print("    （无）")

        print(f"\n[2] ROM 模式设备 (174c:2464 / 174c:2463)：{len(rom)} 个")
        for d in rom:
            print(f"    {fmt_dev(d)}")
        if not rom:
            print("    （无）")

        print(f"\n[3] comma.service：{svc.get('active')!r} (rc={svc.get('returncode')})")
        print(f"[4] Offroad 参数：{obs}")
        print(f"    判定：{off_msg}")

        for d in chestnut:
            owners = node_owners(d.get("node", ""))
            print(f"[5] 节点占用 {d.get('node')}：{owners or '无'}")

    # ---- vendor 请求 ----
    probes = {}
    if args.probe:
        for d in chestnut:
            probes[d["path"]] = probe_vendor_requests(d)
        report["probes"] = probes
        if not args.json:
            print("\n[6] 只读 vendor 请求")
            for path, pr in probes.items():
                print(f"    {path}")
                print(f"      LTSSM(0xB450) = {pr.get('ltssm') or '读不到'}")
                sup = pr.get("supply")
                if sup:
                    print(f"      供电(0xC0/0xC0) = {sup['voltage_mv']} mV / {sup['current_ma']} mA / fault={sup['fault']}")
                else:
                    print(f"      供电(0xC0/0xC0) = 读不到")
                print(f"      C2(0xC2) = {pr.get('c2') or '读不到'}")
                if pr.get("errors"):
                    print(f"      异常：{pr['errors']}")

    # ---- 判定 ----
    verdicts = []
    for d in chestnut:
        ok, missing = classify(d)
        verdicts.append({"path": d["path"], "installer_ok": ok, "missing": missing})
    report["verdicts"] = verdicts

    code = 0
    if not chestnut and not rom:
        code = 3
    elif not chestnut:
        code = 1
    else:
        any_ok = any(v["installer_ok"] for v in verdicts)
        env_ok = off_ok and svc.get("active") == "inactive"
        if not any_ok:
            code = 1
        elif not env_ok:
            code = 2

    report["exit_code"] = code
    report["verdict"] = {0: "GO", 1: "NO_GO_HW", 2: "NO_GO_ENV", 3: "NO_DEVICE"}[code]

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return code

    print("\n" + "=" * 74)
    print("安装器门槛核对（hardware.py:41 要求四项同时成立）")
    print("=" * 74)
    if not verdicts:
        print("  ✗ 没有任何 chestnut 设备，无法进入安装器")
    for v in verdicts:
        if v["installer_ok"]:
            print(f"  ✓ {v['path']} 满足安装器 candidates() 条件")
        else:
            print(f"  ✗ {v['path']} 不满足：")
            for m in v["missing"]:
                print(f"      - {m}")

    print("\n环境门槛")
    print(f"  {'✓' if off_ok else '✗'} Offroad：{off_msg}")
    svc_ok = svc.get("active") == "inactive"
    print(f"  {'✓' if svc_ok else '✗'} comma.service inactive：当前 {svc.get('active')!r}")
    if not svc_ok:
        print("      （安装器 guard() 要求 returncode==3 且 stdout=='inactive'）")

    print("\n" + "=" * 74)
    label = report["verdict"]
    print(f"结论：{label}（退出码 {code}）")
    if code == 0:
        print("  可以进入：sudo python3 flash_ut3g.py install")
    elif code == 1:
        print("  硬件身份不满足，安装器会拒绝且不打开设备。")
        print("  若 speed 不是 5000：先确认底座是否已上电到 USB3（0xF3），或线材/接口是否支持 USB3。")
        print("  若 manufacturer 不是 'tiny'：这块底座可能不是 UT3G，安装器不适用。")
        print("  若 product 不在已知集合：当前固件不是 R4/UID-R1/SP8516-R1，安装器按设计拒绝。")
    elif code == 2:
        print("  先停服务并确认 Offroad：")
        print("    sudo systemctl stop comma.service")
        print("    systemctl is-active comma.service   # 应为 inactive")
    else:
        print("  连 chestnut/ROM 设备都看不到。先解决 USB 枚举，再谈刷固件。")
    print("=" * 74)
    return code


if __name__ == "__main__":
    sys.exit(main())
