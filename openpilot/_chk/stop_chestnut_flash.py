#!/usr/bin/env python3
"""stop_chestnut_flash.py —— 停掉 hardwared 的自动刷固件（它正在和枚举打架）

## 为什么必须停

2026-10-27 19:35 的取证报告（_chk/egpu_forensics_latest.txt）实测：

```
--- flash.py ---
5228  01:35  sudo /usr/local/venv/bin/python3 .../chestnut/flash.py 8516b768
5232  01:35  sudo /usr/local/venv/bin/python3 .../chestnut/flash.py 8516b768
5233  01:35  /usr/local/venv/bin/python3 .../chestnut/flash.py 8516b768
```

两个 flash.py 实例同时在跑（已 95 s，`FLASH_BUDGET=600`）。

而 `flash.py` 干的事就是：
  - `vbus_cycle()`：`VBUS=0` → sleep 2 → `VBUS=1` → sleep 5
  - 操作 `PM_PATHS = ("/sys/bus/platform/devices/a600000.ssusb", "/sys/bus/usb/devices/usb4")`
  - 把设备切进 ROM 模式（`174c:2464`，mass storage）来写 SPI

dmesg 同期（1761 行）：
```
bus4 重注册 = 1        Cannot enable = 60      unable to enumerate = 14
attempt power cycle = 14   new full-speed = 14   usb-storage = 3
```
循环节奏（每 ~3.8 s 一轮）：
```
[ 66.9] usb usb3-port1: Cannot enable. Maybe the USB cable is bad?
[ 67.8] usb usb3-port1: attempt power cycle
[ 69.9] usb usb3-port1: unable to enumerate USB device
[ 70.0] usb 3-1: new full-speed USB device number 46 using xhci-hcd
[ 70.2] usb 3-1: device descriptor read/all, error -71        ← EPROTO
...
[112.0] usb 3-1: new full-speed USB device number 82
[112.2] usb 3-1: not running at top speed; connect to a high speed hub
[112.2] usb 3-1: can't set config #1, error -71
[112.2] usb 3-1: USB disconnect, device number 82
```

**结论：设备每次刚以 12 Mbps 枚举出来，就被 flash.py 的 VBUS/USB 复位打断，
描述符读不出来（error -71），随即 disconnect。**
UI 上「橙色出现一会儿就消失」= 那个 12 Mbps 的短暂窗口 + 告警参数被写又删。

## 为什么不能只靠「pin 固件版本」

`hardwared.Chestnut.mismatch` 用的是 `include_bootloader=True`：
```python
self.mismatch = any(is_chestnut_usb_id(d["vendorId"], d["productId"], include_bootloader=True) and
                    d["product"] != CHESTNUT_USB_PRODUCT for d in usb_state)
```
设备若在 ROM 模式（`174c:2464`，product `USB 3.2 PCIe TinyEnclosure`），
pin 成 `custom ed4e39b7-CLEAN` 之后**仍然不相等** ⇒ 照样刷。
所以需要一个**与 product 无关的硬开关**。

## 做法

1. 新建 `/data/chestnut_no_flash` 标记文件
2. `hardwared.py` 的 `Chestnut.update()` 里加一道门：标记文件存在就直接 return
   （`self.mismatch` 仍会算，保留诊断信息；`failed` 因为 `thread is None` 恒 False，
    不会误报 `Offroad_ChestnutUpdateFailed`）
3. `usb.py` 的 `CHESTNUT_FW_VERSION` pin 成设备实际值（默认 ed4e39b7），
   让 `chestnut_present()` 的严格相等判断能通过
4. **不杀正在跑的 flash.py** —— 它可能正在写 SPI，中途 kill 有变砖风险。
   加门之后不会再起新的，等现有实例自己退出（`FLASH_BUDGET=600 s`，最多约 10 分钟）。
   `--kill` 是显式选项，会二次确认风险。

用法：
  sudo python3 stop_chestnut_flash.py --check
  sudo python3 stop_chestnut_flash.py --apply                  # 默认 pin 到 ed4e39b7
  sudo python3 stop_chestnut_flash.py --apply --pin xxxx      # 指定值
  sudo python3 stop_chestnut_flash.py --revert
"""

import argparse
import os
import py_compile
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DEFAULT_REPO = Path("/data/openpilot/openpilot")
DEFAULT_BACKUP = Path("/data/chestnut_flash_stop_backup")
NO_FLASH_PATH = Path("/data/chestnut_no_flash")   # 默认值；--marker 可覆盖（测试用）

USB_REL = "common/hardware/usb.py"
HW_REL = "system/hardware/hardwared.py"

MARK = "[NO_FLASH 20261027]"
DEFAULT_PIN = "ed4e39b7"

USB_ANCHOR = 'CHESTNUT_FW_VERSION = "8516b768"\n'
HW_ANCHOR = '    if not self.mismatch:\n      self.flashed = False\n      return\n'

HW_GATE = (
    '    # ' + MARK + ' —— 硬开关：标记文件存在就不刷固件。\n'
    '    # 设备卡在 ROM（174c:2464）时 product 永远 != CHESTNUT_USB_PRODUCT，\n'
    '    # 光 pin 固件版本停不下来，所以这里用与 product 无关的开关。\n'
    '    if os.path.exists("/data/chestnut_no_flash"):\n'
    '      self.flashed = False\n'
    '      return\n'
    + HW_ANCHOR
)


def _read(p: Path) -> str:
    try:
        return p.read_text(errors="replace")
    except OSError:
        return ""


def _chown(p: Path, uid: int, gid: int) -> None:
    try:
        os.chown(p, uid, gid)
    except (AttributeError, OSError):
        pass


def sh(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=timeout)
        return (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return f"<失败: {e}>"


def running_flashers() -> list[str]:
    out = sh("ps -eo pid,etime,args 2>/dev/null | grep '[c]hestnut/flash.py'")
    return [ln.strip() for ln in out.splitlines() if "flash.py" in ln]


def device_products() -> list[tuple[str, str, str]]:
    """从 sysfs 读当前 USB 上 chestnut / ROM 设备：(节点, id, product)"""
    res = []
    base = Path("/sys/bus/usb/devices")
    try:
        nodes = sorted(base.glob("*"))
    except OSError:
        return res
    for d in nodes:
        if "-" not in d.name or ":" in d.name:
            continue
        vid = _read(d / "idVendor")
        pid = _read(d / "idProduct")
        if not vid or not pid:
            continue
        ident = f"{vid}:{pid}"
        if ident.lower() in ("3801:0001", "add1:0001", "174c:2464", "174c:2463"):
            res.append((d.name, ident, _read(d / "product") or "-"))
    return res


def current_pin(repo: Path) -> str | None:
    for ln in _read(repo / USB_REL).splitlines():
        if ln.startswith("CHESTNUT_FW_VERSION"):
            try:
                return ln.split('"')[1]
            except IndexError:
                return None
    return None


def state(repo: Path, marker: Path = NO_FLASH_PATH) -> dict:
    hw = _read(repo / HW_REL)
    return {
        "pin": current_pin(repo),
        "gate": MARK in hw,
        "marker": marker.exists(),
        "hw_anchor": HW_ANCHOR in hw,
        "usb_anchor": USB_ANCHOR in _read(repo / USB_REL),
    }


def do_check(repo: Path, marker: Path = NO_FLASH_PATH) -> int:
    s = state(repo, marker)
    print("== 当前状态 ==")
    print(f"  repo                      : {repo}")
    print(f"  CHESTNUT_FW_VERSION       : {s['pin']}")
    print(f"  hardwared 硬开关（代码）  : {s['gate']}")
    print(f"  标记文件 {marker} : {s['marker']}")
    print()
    print("== 总线上现在有什么 ==")
    prods = device_products()
    if not prods:
        print("  （没有任何 chestnut / ROM 设备 —— 描述符读不出来时也会这样）")
    for name, ident, prod in prods:
        kind = "ROM-BOOTLOADER" if ident.lower().startswith("174c") else "CHESTNUT"
        print(f"  {name:<6} {ident} [{kind}] product={prod}")
    print()
    print("== 正在跑的 flash.py ==")
    fl = running_flashers()
    if not fl:
        print("  （无）")
    for ln in fl:
        print("  " + ln)
    print()
    if fl:
        print("  ⚠ 有 flash.py 在跑 —— 它会 vbus_cycle() + unbind usb4，正在打断枚举。")
    if s["gate"] and s["marker"] and not fl:
        print("  ✅ 已止血：硬开关在、标记文件在、没有刷固件进程")
        return 0
    print("\n  需要处理：--apply")
    return 1


def do_apply(repo: Path, backup: Path, pin: str, kill: bool,
             marker: Path = NO_FLASH_PATH) -> int:
    s = state(repo, marker)
    usb = repo / USB_REL
    hw = repo / HW_REL

    jobs = []
    if not s["gate"]:
        if not s["hw_anchor"]:
            print(f"[FAIL] {HW_REL} 找不到锚点 `if not self.mismatch:` —— 版本不匹配，不硬套")
            return 1
        jobs.append((hw, HW_ANCHOR, HW_GATE, "hardwared 加刷固件硬开关"))
    if s["pin"] != pin:
        if not s["usb_anchor"] and s["pin"] is None:
            print(f"[FAIL] {USB_REL} 找不到 CHESTNUT_FW_VERSION，无法 pin")
            return 1
        old = f'CHESTNUT_FW_VERSION = "{s["pin"]}"\n' if s["pin"] else USB_ANCHOR
        jobs.append((usb, old, f'CHESTNUT_FW_VERSION = "{pin}"\n', f"固件版本 pin 成 {pin}"))

    if not jobs:
        print("\n代码已是目标状态，无需改动。")
        if not marker.exists():
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.write_text(f"{MARK} 存在此文件即禁止 hardwared 自动刷固件。\n"
                              f"删除即可恢复。创建于 {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            print(f"  [OK] 补建标记文件 {marker}")
        else:
            print(f"  [OK] 标记文件已存在 {marker}")
        fl = running_flashers()
        print(f"\n== 正在跑的 flash.py：{len(fl)} 个 ==")
        for ln in fl:
            print("  " + ln)
        if fl and not kill:
            print("  ★ 不主动 kill；硬开关已在，不会再起新的，等它自己退出（≤600 s）")
        elif fl and kill:
            for ln in fl:
                pid = ln.split()[0]
                if pid.isdigit():
                    sh(f"sudo kill -TERM {pid} 2>/dev/null || kill -TERM {pid} 2>/dev/null")
            time.sleep(2)
            print(f"  剩余：{len(running_flashers())} 个")
        return 0

    # 备份
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bdir = backup / stamp
    bdir.mkdir(parents=True, exist_ok=True)
    print(f"== 备份 → {bdir} ==")
    for p in {j[0] for j in jobs}:
        bak = bdir / (p.name + ".bak")
        shutil.copy2(p, bak)
        print(f"  [OK] {bak} ({bak.stat().st_size} B)")

    # 标记文件（先建，这样即使后面写代码失败，下一次 hardwared 重启也不会刷）
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(f"{MARK} 存在此文件即禁止 hardwared 自动刷固件。\n"
                             f"删除即可恢复。创建于 {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    print(f"  [OK] 标记文件 {marker}")

    # 写代码
    print("\n== 写入 ==")
    for p, old, new, desc in jobs:
        st = p.stat()
        txt = p.read_text(errors="replace")
        if old not in txt:
            print(f"  [FAIL] {desc}：锚点丢失")
            continue
        p.write_text(txt.replace(old, new, 1), encoding="utf-8", newline="")
        _chown(p, st.st_uid, st.st_gid)
        os.chmod(p, st.st_mode & 0o777)
        print(f"  [OK] {desc}")

    # 语法校验
    print("\n== 语法校验 ==")
    ok = True
    for p in {j[0] for j in jobs} | {usb, hw}:
        try:
            py_compile.compile(str(p), cfile=tempfile.mktemp(suffix=".pyc"), doraise=True)
            print(f"  [OK] {p}")
        except py_compile.PyCompileError as e:
            print(f"  [FAIL] {p}\n{e}")
            ok = False
    if not ok:
        print("\n⚠ 语法校验失败 —— 立刻 --revert")
        return 1

    # 处理正在跑的 flash.py
    fl = running_flashers()
    print(f"\n== 正在跑的 flash.py：{len(fl)} 个 ==")
    for ln in fl:
        print("  " + ln)
    if fl:
        if kill:
            print("\n⚠ --kill 已指定：正在终止 flash.py")
            print("   注意：若它正写到一半，SPI 可能留下半成品。")
            for ln in fl:
                pid = ln.split()[0]
                if pid.isdigit():
                    sh(f"sudo kill -TERM {pid} 2>/dev/null || kill -TERM {pid} 2>/dev/null")
            time.sleep(2)
            left = running_flashers()
            print(f"   剩余：{len(left)} 个")
        else:
            print("\n  ★ 不主动 kill（可能正在写 SPI，中途终止有变砖风险）。")
            print("    已经加了硬开关 ⇒ 不会再起新的；等现有实例自己退出")
            print("    （flash.py 的 FLASH_BUDGET=600 s，最多约 10 分钟）。")
            print("    之后用 `ps -eo pid,etime,args | grep '[c]hestnut/flash.py'` 复查。")

    print("\n== 复查 ==")
    s2 = state(repo, marker)
    print(f"  pin={s2['pin']}  硬开关={s2['gate']}  标记文件={s2['marker']}")
    print("\n下一步：")
    print("  1) 等现有 flash.py 退出（或已 --kill）")
    print("  2) sudo systemctl restart comma")
    print("  3) sudo python3 _chk/check_egpu_link_v3.py   # 看 link_state / 4-1 speed")
    print("\n⚠ 这只是停止刷固件造成的干扰；USB 链路本身若仍卡死，")
    print("   还需「设备 + 盒子两端同时断电 ≥30 s」（10-15 唯一验证有效的手段）。")
    return 0


def do_revert(repo: Path, backup: Path, marker: Path = NO_FLASH_PATH) -> int:
    if marker.exists():
        marker.unlink()
        print(f"  [OK] 删除 {marker}")

    if backup.is_dir():
        baks = sorted(backup.glob("*/"))
        if baks:
            bdir = baks[-1]
            print(f"== 从 {bdir} 回滚 ==")
            for bak in bdir.glob("*.py.bak"):
                name = bak.name[: -len(".bak")]
                for rel in (USB_REL, HW_REL):
                    p = repo / rel
                    if p.name == name and p.is_file():
                        st = p.stat()
                        shutil.copy2(bak, p)
                        _chown(p, st.st_uid, st.st_gid)
                        print(f"  [OK] {p}")

    print("\n== 复查 ==")
    s = state(repo, marker)
    print(f"  pin={s['pin']}  硬开关={s['gate']}  标记文件={s['marker']}")
    print("\n下一步：sudo systemctl restart comma")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    ap.add_argument("--pin", default=DEFAULT_PIN, help=f"pin 的固件版本（默认 {DEFAULT_PIN}）")
    ap.add_argument("--kill", action="store_true", help="同时终止正在跑的 flash.py（有风险）")
    ap.add_argument("--repo", default=str(DEFAULT_REPO))
    ap.add_argument("--backup-root", default=str(DEFAULT_BACKUP))
    ap.add_argument("--marker", default=str(NO_FLASH_PATH), help="硬开关标记文件路径（测试用）")
    args = ap.parse_args()

    if hasattr(os, "geteuid") and os.geteuid() != 0:
        print("⚠ 不是 root：写 /data 与系统文件可能失败，请 sudo")

    n = sum([args.check, args.apply, args.revert])
    if n != 1:
        ap.error("必须且只能指定 --check / --apply / --revert 之一")

    repo = Path(args.repo)
    backup = Path(args.backup_root)
    marker = Path(args.marker)

    if args.check:
        return do_check(repo, marker)
    if args.apply:
        return do_apply(repo, backup, args.pin, args.kill, marker)
    return do_revert(repo, backup, marker)


if __name__ == "__main__":
    sys.exit(main())
