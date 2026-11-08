#!/usr/bin/env python3
"""patch_egpu_link_loading.py —— 按 chx2025 + commaai 的做法修 eGPU「枚举后的加载」

来源与依据（全部实测核对过，不是猜的）：

  [A] commaai/openpilot 24a9b6dae5 "profile usb gpu link stability (#38368)"
      （经 onemiless/openpilot 镜像取到）
      新增 selfdrive/modeld/usbgpu_link.py：读 /sys/.../a600000.ssusb/portli
      的链路错误计数，等错误率降到 5/s 以下再加载。
      本分支**没有这个文件** ⇒ 需要补。

  [B] commaai/openpilot cf8371c2c5 "add usb link err counter (#38338)"
      usb.py 加 controller() + linkErrorCount。
      本分支**已经合入**（usb.py:54 controller / usb.py:76 linkErrorCount）⇒ 不用动。

  [C] chx2025/openpilot 分支 egpubak0917 @ a8d3ee86ddb9
      tinygrad_repo/tinygrad/runtime/support/usb.py 里 CustomASM24Controller 的本地补丁：
        - _wait_link_up()：轮询 LTSSM(0xB450) 直到 0x78，而不是只读一次
        - 仍不 up ⇒ set_pcie_power(False) → 等 1.5s → True → 再等 15s
        - 链路刚 up 后 settle 3.0s（断电重上电过）或 0.5s 才允许 MMIO
      上游 tinygrad 原版（已核对 raw 源码）是：
          ltssm = self.read(0xB450, 1)[0]
          if ltssm != 0x78: self.set_pcie_power(True)
          ltssm = self.read(0xB450, 1)[0]
          if ltssm != 0x78: raise RuntimeError(...)
      ⇒ 只读一次，必然读到中间态；且 up 后立刻 MMIO 会
        `bulk IN 0x81 failed: Operation timed out`（2026-10-15 实测过这个错）。

★ 适用范围（必须看清）：
   这三处修的是「**USB 已枚举、PCIe 链路/MMIO 阶段**」的失败。
   它**修不了** `usb4-port1: Cannot enable` / `cannot disable (err=-32)` 那种
   xhci 端口状态机卡死 —— 那种只能「设备 + 盒子两端同时断电 ≥30 s」。
   顺序：先让设备枚举出来，再让链路稳定，再谈加载。

用法：
  sudo python3 patch_egpu_link_loading.py --check
  sudo python3 patch_egpu_link_loading.py --apply
  sudo python3 patch_egpu_link_loading.py --revert
  # 可加 --repo / --tg / --backup-root 覆盖默认路径
"""

import argparse
import os
import py_compile
import shutil
import sys
import tempfile
import time
from pathlib import Path

DEFAULT_REPO = Path("/data/openpilot/openpilot")
DEFAULT_TG = Path("/data/openpilot/tinygrad_repo/tinygrad/runtime/support/usb.py")
DEFAULT_BACKUP = Path("/data/egpu_link_loading_backup")

LINK_REL = "selfdrive/modeld/usbgpu_link.py"
V2_REL = "sunnypilot/modeld_v2/modeld.py"

MARK = "[EGPU_LINK_LOADING 20261027]"

# ---------------------------------------------------------------- [A] 新文件

LINK_PY = '''"""{mark}
usbgpu_link.py —— 等 eGPU 的 USB SuperSpeed 链路稳定后再加载大模型

来源：commaai/openpilot commit 24a9b6dae5 "profile usb gpu link stability (#38368)"。
适配：本分支 usb.py 用 CHESTNUT_USB_IDS / is_chestnut_usb_id()，没有上游的
      CHESTNUT_VENDOR_ID / CHESTNUT_PRODUCT_ID，故改成按 ID 集合判断。

为什么需要：ASM2464 的 SuperSpeed 链路刚枚举的几秒内会持续报 link error
（/sys/.../a600000.ssusb/portli 计数上涨）。此时直接开 AMD 设备会得到
`bulk IN 0x81 failed: Operation timed out`（2026-10-15 实测）。
"""

import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware.usb import controller, is_chestnut_usb_id, read_int, usb_devices

STABLE_SECONDS = 2.0
STABLE_THRESHOLD = 5.0  # link errors per second


def _chestnut_portli() -> Path | None:
  for device in usb_devices():
    usb_id = (read_int(device / "idVendor", 16), read_int(device / "idProduct", 16))
    if is_chestnut_usb_id(*usb_id):
      ctrl = controller(device)
      if ctrl is not None and (ctrl / "portli").exists():
        return ctrl / "portli"
  return None


def wait_usbgpu_link(timeout: float = 30.0, threshold: float = STABLE_THRESHOLD) -> bool:
  """等链路错误率降到 threshold 以下。

  返回 True=已稳定，False=超时或设备不在总线上。
  **永不抛异常** —— 调用点在加载路径上，抛错只会白吃重试预算。
  """
  portli = _chestnut_portli()
  if portli is None:
    cloudlog.warning("usbgpu_link: chestnut 不在总线上，跳过稳定性等待")
    return False

  t0 = time.monotonic()
  rate = 0.0
  while time.monotonic() - t0 < timeout:
    start = read_int(portli, 0)
    time.sleep(STABLE_SECONDS)
    rate = (read_int(portli, 0) - start) / STABLE_SECONDS
    if rate <= threshold:
      cloudlog.warning(f"usbgpu_link: stable ({rate:.0f} err/s) after {time.monotonic() - t0:.1f}s")
      return True
    cloudlog.warning(f"usbgpu_link: not stable, {rate:.0f} err/s")
  cloudlog.error(f"usbgpu_link: never stabilized in {timeout:.0f}s (last {rate:.0f} err/s)")
  return False
'''.replace("{mark}", MARK)

# ------------------------------------------------- [C] tinygrad 枚举/链路修复

# 上游 tinygrad 原版（raw 源码核对，缩进原样）
TG_OLD = (
    '    ltssm = self.read(0xB450, 1)[0]\n'
    '    if ltssm != 0x78: self.set_pcie_power(True)\n'
    '    ltssm = self.read(0xB450, 1)[0]\n'
    '    if ltssm != 0x78: raise RuntimeError(f"PCIe link not up (LTSSM=0x{ltssm:02X}), custom firmware not ready")\n'
)

# chx2025 egpubak0917 的做法 + 标记
TG_NEW = (
    '    # ' + MARK + ' —— 见 chx2025/openpilot egpubak0917\n'
    '    # 原实现上电后立刻复读，必然读到中间态(0x01/0x00)，导致 eGPU 永远起不来。\n'
    '    ltssm = self.read(0xB450, 1)[0]\n'
    '    did_cycle = False\n'
    '    if ltssm != 0x78:\n'
    '      self.set_pcie_power(True)\n'
    '      ltssm = self._wait_link_up(3.0)\n'
    '      if ltssm != 0x78:\n'
    '        print(f"[usb] LTSSM=0x{ltssm:02X} after power-on, cycling PCIe power ...", flush=True)\n'
    '        self.set_pcie_power(False)\n'
    '        time.sleep(1.5)\n'
    '        self.set_pcie_power(True)\n'
    '        ltssm = self._wait_link_up(15.0)\n'
    '        did_cycle = True\n'
    '    if ltssm != 0x78: raise RuntimeError(f"PCIe link not up (LTSSM=0x{ltssm:02X}), custom firmware not ready")\n'
    '    # 链路刚训练完就访问 GPU MMIO 会 bulk IN 超时，需要等 GPU 就绪。\n'
    '    settle = 3.0 if did_cycle else 0.5\n'
    '    if DEBUG >= 1: print(f"[usb] LTSSM=0x78, settling {settle}s before MMIO access", flush=True)\n'
    '    time.sleep(settle)\n'
)

# _wait_link_up 方法，插在 class CustomASM24Controller: 之后
TG_CLASS_ANCHOR = 'class CustomASM24Controller:\n'
TG_METHOD = (
    '  def _wait_link_up(self, timeout: float, interval: float = 0.1) -> int:\n'
    '    # ' + MARK + ' —— 轮询等待 PCIe 链路训练完成，而不是只读一次。\n'
    '    t0 = time.monotonic()\n'
    '    ltssm = self.read(0xB450, 1)[0]\n'
    '    while ltssm != 0x78 and time.monotonic() - t0 < timeout:\n'
    '      time.sleep(interval)\n'
    '      ltssm = self.read(0xB450, 1)[0]\n'
    '    return ltssm\n'
    '\n'
)

# --------------------------------------------------------- [A] 挂进 v2 loader

V2_IMPORT_ANCHOR = 'from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present, modeld_pkl_path\n'
V2_IMPORT_NEW = V2_IMPORT_ANCHOR + 'from openpilot.selfdrive.modeld.usbgpu_link import wait_usbgpu_link  # ' + MARK + '\n'

V2_CALL_ANCHOR = '        m = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=True)\n'
V2_CALL_NEW = (
    '        # ' + MARK + ' —— 等链路错误率降下来再开 GPU，避免 bulk IN 0x81 超时。\n'
    '        # timeout 取 6 s：BIG_MODEL_TIMEOUT 要留给守卫 + 真正的加载。\n'
    '        wait_usbgpu_link(timeout=6.0)\n'
    + V2_CALL_ANCHOR
)


def _read(p: Path) -> str:
    try:
        return p.read_text(errors="replace")
    except OSError:
        return ""


def _chown(p: Path, uid: int, gid: int) -> None:
    # Windows 没有 os.chown；Linux 上权限不足也可能失败 —— 都不应让补丁中断
    try:
        os.chown(p, uid, gid)
    except (AttributeError, OSError):
        pass


def state(repo: Path, tg: Path) -> dict:
    link = repo / LINK_REL
    v2 = repo / V2_REL
    ltxt = _read(link)
    vtxt = _read(v2)
    ttxt = _read(tg)
    return {
        "link_file": link.is_file() and MARK in ltxt,
        "tg_method": "_wait_link_up" in ttxt,
        "tg_init": MARK in ttxt,
        "tg_stock_anchor": TG_OLD in ttxt,
        "v2_import": "usbgpu_link import wait_usbgpu_link" in vtxt,
        "v2_call": "wait_usbgpu_link(timeout=6.0)" in vtxt,
        "v2_import_anchor": V2_IMPORT_ANCHOR in vtxt,
        "v2_call_anchor": V2_CALL_ANCHOR in vtxt,
        "tg_exists": tg.is_file(),
    }


def do_check(repo: Path, tg: Path) -> int:
    s = state(repo, tg)
    print("== 状态 ==")
    print(f"  repo                : {repo}")
    print(f"  tinygrad usb.py     : {tg}   (存在={s['tg_exists']})")
    print()
    print(f"  [A] usbgpu_link.py 新建     : {s['link_file']}")
    print(f"  [A] v2 loader 加 import     : {s['v2_import']}")
    print(f"  [A] v2 loader 加调用        : {s['v2_call']}")
    print(f"  [C] tinygrad _wait_link_up  : {s['tg_method']}")
    print(f"  [C] tinygrad __init__ 改完  : {s['tg_init']}")
    print()
    if not s["tg_exists"]:
        print("  ⚠ 找不到 tinygrad usb.py，[C] 无法应用（用 --tg 指定）")
    elif not s["tg_method"] and not s["tg_stock_anchor"]:
        print("  ⚠ tinygrad usb.py 既没有补丁也没有原版锚点 —— 版本不匹配，[C] 会拒绝应用")

    pending = []
    if not s["link_file"]:
        pending.append("A-新建 usbgpu_link.py")
    if not s["v2_import"] or not s["v2_call"]:
        pending.append("A-挂进 v2 loader")
    if s["tg_exists"] and not s["tg_init"]:
        pending.append("C-tinygrad 链路等待")

    if pending:
        print("  待应用：" + "、".join(pending))
        return 1
    print("  全部已应用")
    return 0


def do_apply(repo: Path, tg: Path, backup_root: Path) -> int:
    s = state(repo, tg)
    jobs = []   # (path, old, new, desc, is_new_file)

    link = repo / LINK_REL
    v2 = repo / V2_REL

    if not s["link_file"]:
        if link.exists():
            print(f"[SKIP] {LINK_REL} 已存在但不是本补丁写的，不覆盖")
        else:
            jobs.append((link, "", LINK_PY, "新建 usbgpu_link.py", True))

    if not s["v2_import"]:
        if not s["v2_import_anchor"]:
            print(f"[FAIL] {V2_REL} 找不到 import 锚点，跳过挂载")
        else:
            jobs.append((v2, V2_IMPORT_ANCHOR, V2_IMPORT_NEW, "v2 loader 加 import", False))

    if not s["v2_call"]:
        if not s["v2_call_anchor"]:
            print(f"[FAIL] {V2_REL} 找不到 ModelState 调用锚点，跳过挂载")
        else:
            jobs.append((v2, V2_CALL_ANCHOR, V2_CALL_NEW, "v2 loader 加 wait_usbgpu_link", False))

    if s["tg_exists"] and not s["tg_init"]:
        if not s["tg_stock_anchor"]:
            print(f"[FAIL] {tg} 找不到上游原版锚点（版本不匹配），跳过")
            print("       → 请把该文件发我，或确认 tinygrad 版本")
        else:
            jobs.append((tg, TG_OLD, TG_NEW, "tinygrad __init__ 链路等待", False))
            if not s["tg_method"]:
                jobs.append((tg, TG_CLASS_ANCHOR, TG_CLASS_ANCHOR + TG_METHOD,
                             "tinygrad 新增 _wait_link_up", False))

    if not jobs:
        print("\n没有需要应用的改动")
        return 1

    # 备份
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bdir = backup_root / stamp
    bdir.mkdir(parents=True, exist_ok=True)
    print(f"\n== 备份 → {bdir} ==")
    originals = {}
    for p, _o, _n, _d, is_new in jobs:
        if is_new or not p.is_file():
            originals[p] = None
            print(f"  [新文件] {p}")
            continue
        if p not in originals:
            bak = bdir / (p.name + ".bak")
            shutil.copy2(p, bak)
            originals[p] = p.read_bytes()
            print(f"  [OK] {bak}  ({bak.stat().st_size} B)")

    # 写入
    print("\n== 写入 ==")
    for p, old, new, desc, is_new in jobs:
        if is_new:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(new, encoding="utf-8", newline="")
            print(f"  [OK] {desc} → {p}")
            continue
        st = p.stat()
        txt = p.read_text(errors="replace")
        if old not in txt:
            print(f"  [FAIL] {desc}：锚点丢失，跳过")
            continue
        p.write_text(txt.replace(old, new, 1), encoding="utf-8", newline="")
        _chown(p, st.st_uid, st.st_gid)
        os.chmod(p, st.st_mode & 0o777)
        print(f"  [OK] {desc}")

    # 语法校验
    print("\n== 语法校验 ==")
    ok = True
    for p in {j[0] for j in jobs}:
        try:
            py_compile.compile(str(p), cfile=tempfile.mktemp(suffix=".pyc"), doraise=True)
            print(f"  [OK] {p}")
        except py_compile.PyCompileError as e:
            print(f"  [FAIL] {p}\n{e}")
            ok = False

    if not ok:
        print("\n⚠ 语法校验失败，建议立刻 --revert")
        return 1

    print("\n== 复查 ==")
    s2 = state(repo, tg)
    print(f"  usbgpu_link.py : {s2['link_file']}")
    print(f"  v2 import/call : {s2['v2_import']} / {s2['v2_call']}")
    print(f"  tinygrad 方法  : {s2['tg_method']}")
    print(f"  tinygrad init  : {s2['tg_init']}")
    print("\n下一步：sudo systemctl restart comma")
    print("⚠ 这只修「枚举之后的链路/MMIO」，修不了 usb4-port1 Cannot enable。")
    return 0


def do_revert(repo: Path, tg: Path, backup_root: Path) -> int:
    if not backup_root.is_dir():
        print(f"[FAIL] 没有备份目录 {backup_root}")
        return 1
    baks = sorted(backup_root.glob("*/"))
    if not baks:
        print(f"[FAIL] {backup_root} 下没有备份")
        return 1
    bdir = baks[-1]
    print(f"== 从 {bdir} 回滚 ==")

    # 1) 恢复 .bak
    for bak in bdir.glob("*.py.bak"):
        name = bak.name[: -len(".bak")]
        for p in (repo / V2_REL, tg):
            if p.name == name and p.is_file():
                st = p.stat()
                shutil.copy2(bak, p)
                _chown(p, st.st_uid, st.st_gid)
                print(f"  [OK] {p}")

    # 2) 删掉本补丁新建的文件
    link = repo / LINK_REL
    if link.is_file() and MARK in _read(link):
        link.unlink()
        print(f"  [OK] 删除 {link}")

    print("\n== 复查 ==")
    s = state(repo, tg)
    print(f"  usbgpu_link.py : {s['link_file']}")
    print(f"  v2 import/call : {s['v2_import']} / {s['v2_call']}")
    print(f"  tinygrad 方法  : {s['tg_method']}")
    print(f"  tinygrad init  : {s['tg_init']}")
    print("\n下一步：sudo systemctl restart comma")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    ap.add_argument("--repo", default=str(DEFAULT_REPO))
    ap.add_argument("--tg", default=str(DEFAULT_TG))
    ap.add_argument("--backup-root", default=str(DEFAULT_BACKUP))
    args = ap.parse_args()

    repo = Path(args.repo)
    tg = Path(args.tg)
    backup = Path(args.backup_root)

    if hasattr(os, "geteuid") and os.geteuid() != 0:
        print("⚠ 不是 root：写 tinygrad_repo 可能失败，建议 sudo")

    n = sum([args.check, args.apply, args.revert])
    if n != 1:
        ap.error("必须且只能指定 --check / --apply / --revert 之一")

    if args.check:
        return do_check(repo, tg)
    if args.apply:
        return do_apply(repo, tg, backup)
    return do_revert(repo, tg, backup)


if __name__ == "__main__":
    sys.exit(main())
