#!/usr/bin/env python3
"""修 eGPU 认不出来的根因：固件版本串判据过严。

背景
----
`openpilot/selfdrive/modeld/helpers.py:chestnut_present()` 用的是**完全相等**：
    if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
而 `CHESTNUT_USB_PRODUCT = f"custom {CHESTNUT_FW_VERSION}-CLEAN"`，
`CHESTNUT_FW_VERSION` 在两个分支里不一样：

    ford-chestnut    : "ed4e39b7"   <- 10-21 那台 eGPU 的实测值，当时大模型能加载
    staging-chestnut : "8516b768"   <- 10-26 换分支后的要求

所以换到 staging-chestnut 之后，本机 eGPU 的 product 串（custom ed4e39b7-CLEAN）
永远不等于期望值 ⇒ chestnut_present() 恒 False ⇒
  · deviceState.chestnutPresent = False ⇒ UI 不显示 eGPU
  · 门控靠 cable_connected() 兜底放行 ⇒ load_big() 起来
  · 但 wait_for_chestnut() 只等 chestnut_present() ⇒ 白等 10 s 抛 TimeoutError
  · ChestnutActive = False ⇒ UI 橙色（LOADING）闪一下就没

两种修法
--------
  --mode relax（默认，推荐）改 helpers.py 的判据为「custom 前缀 + -CLEAN 后缀」。
        不再依赖具体固件号，以后升级/更换 eGPU 固件都不会失效；
        仍要求 -CLEAN 后缀，能排除非官方固件。
  --mode pin   只把 usb.py 的 CHESTNUT_FW_VERSION 钉成指定值（默认 ed4e39b7）。
        改动最小（一行），但换固件就要再改一次。

用法（设备上）
--------------
  sudo python3 /data/openpilot/openpilot/_chk/patch_chx_fw_match.py --check
  sudo python3 /data/openpilot/openpilot/_chk/patch_chx_fw_match.py --apply
  sudo python3 /data/openpilot/openpilot/_chk/patch_chx_fw_match.py --apply --mode pin
  sudo python3 /data/openpilot/openpilot/_chk/patch_chx_fw_match.py --revert
"""
import argparse
import os
import py_compile
import shutil
import sys
import tempfile
import time
from pathlib import Path

REPO = Path('/data/openpilot/openpilot')
HELPERS_REL = 'selfdrive/modeld/helpers.py'
USB_REL = 'common/hardware/usb.py'
BACKUP_ROOT = Path('/data/chx_fw_match_backup')
SKIP_USB = False

MARK = '[CHX_FW_ANY 20261027]'

HELPERS_OLD = '      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:\n'
HELPERS_NEW = (
  f'      # ---- {MARK} ----\n'
  '      # 上游是 product == CHESTNUT_USB_PRODUCT（完全相等），而 CHESTNUT_FW_VERSION\n'
  '      # 在 ford-chestnut 是 ed4e39b7、staging-chestnut 是 8516b768。本机 eGPU 固件是\n'
  '      # ed4e39b7（10-21 实测，当时大模型能加载），换分支后就被判成「不存在」。\n'
  '      # 固件构建号不该决定硬件在不在，这里放宽成「custom 前缀 + -CLEAN 后缀」，\n'
  '      # 仍然排除非官方固件。\n'
  '      if is_chestnut_usb_id(*usb_id) and product.startswith("custom ") and product.endswith("-CLEAN"):\n'
)

PIN_OLD = 'CHESTNUT_FW_VERSION = "8516b768"\n'


def _chown(path: Path, uid: int, gid: int) -> None:
  fn = getattr(os, 'chown', None)
  if fn is None:
    return
  try:
    fn(path, uid, gid)
  except (PermissionError, OSError, NotImplementedError):
    pass


def scan_usb() -> list[dict]:
  """不 import openpilot，直接读 sysfs 看 eGPU 真实身份。"""
  ids = {(0xADD1, 0x0001), (0x3801, 0x0001)}
  rom = {(0x174C, 0x2464), (0x174C, 0x2463)}
  out = []
  root = Path('/sys/bus/usb/devices')
  if not root.is_dir():
    return out
  for name in sorted(os.listdir(root)):
    if ':' in name:
      continue
    d = root / name
    try:
      vid = int((d / 'idVendor').read_text().strip(), 16)
      pid = int((d / 'idProduct').read_text().strip(), 16)
    except (OSError, ValueError):
      continue
    kind = 'chestnut' if (vid, pid) in ids else 'rom' if (vid, pid) in rom else None
    if kind is None:
      continue
    out.append({
      'name': name, 'vid': vid, 'pid': pid, 'kind': kind,
      'product': (d / 'product').read_text(errors='replace').strip() if (d / 'product').exists() else '',
      'speed': (d / 'speed').read_text().strip() if (d / 'speed').exists() else '?',
    })
  return out


def print_usb() -> None:
  if SKIP_USB:
    return
  print('== 0. 当前 eGPU 真实身份（sysfs）==')
  devs = scan_usb()
  if not devs:
    print('  [WARN] sysfs 里没有 chestnut / ASM2464 设备 —— eGPU 没枚举，改判据也救不了')
    print('         先跑 diag_egpu_usb.py 查线缆/供电/控制器模式')
    return
  for d in devs:
    tag = 'OK  ' if d['kind'] == 'chestnut' else 'WARN'
    print(f"  [{tag}] {d['name']:8} {d['vid']:04x}:{d['pid']:04x} {d['speed']:>6}Mbps "
          f"product={d['product']!r}  ({'正常设备' if d['kind'] == 'chestnut' else 'ROM bootloader'})")
  good = [d for d in devs if d['kind'] == 'chestnut']
  if not good:
    print('  [WARN] 只有 ROM bootloader（174C:2464/2463），正常设备 3801:0001 没枚举')
    print('         ⇒ 卡在固件加载阶段，改判据救不了；先看 diag_egpu_usb.py 的供电/线缆段')
    return
  for d in good:
    p = d['product']
    if p.startswith('custom ') and p.endswith('-CLEAN'):
      print(f"  ⇒ {d['name']} 的 product={p!r}，放宽判据后 chestnut_present() 会变成 True")
      print('     （当前严格比较要求 product == \'custom 8516b768-CLEAN\'）')
    else:
      print(f"  [WARN] {d['name']} product={p!r} 不是 'custom *-CLEAN' 形态")
      print('         ⇒ 非官方/半刷固件，relax 模式也不会认；需要用 pin 模式钉实际值')
    if d['speed'] not in ('?', '') and float(d['speed']) < 5000:
      print(f"  [WARN] {d['name']} 速率只有 {d['speed']}Mbps（<5000），走的是 USB2，带宽不够跑大模型")


def state(root: Path) -> dict:
  h = (root / HELPERS_REL).read_text(errors='replace') if (root / HELPERS_REL).is_file() else ''
  u = (root / USB_REL).read_text(errors='replace') if (root / USB_REL).is_file() else ''
  return {
    'relax': MARK in h,
    'strict': HELPERS_OLD in h,
    'pin': 'CHESTNUT_FW_VERSION = "ed4e39b7"' in u,
    'pin_stock': PIN_OLD in u,
  }


def do_check(root: Path) -> int:
  print_usb()
  print('\n== 1. 补丁状态 ==')
  s = state(root)
  print(f"  relax（helpers.py 放宽判据） : {s['relax']}")
  print(f"  严格比较仍在                 : {s['strict']}")
  print(f"  pin（usb.py 钉 ed4e39b7）    : {s['pin']}")
  if s['relax'] and s['pin']:
    print('  WARNING  两种修法都打了，冗余；建议只留 relax')
  pending = not (s['relax'] or s['pin'])
  print('  NEEDS PATCH' if pending else '  PATCHED')
  return 1 if pending else 0


def do_apply(root: Path, mode: str, pin_value: str) -> int:
  print_usb()
  s = state(root)
  if s['relax'] or s['pin']:
    print(f"\n[FAIL] 已打过补丁（relax={s['relax']} pin={s['pin']}），先 --revert")
    return 1

  jobs = []
  if mode == 'relax':
    p = root / HELPERS_REL
    txt = p.read_text(errors='replace')
    if HELPERS_OLD not in txt:
      print(f'\n[FAIL] {HELPERS_REL} 里找不到锚点：\n  {HELPERS_OLD.strip()}')
      return 1
    jobs.append((p, HELPERS_OLD, HELPERS_NEW, 'chestnut_present() 判据放宽为 custom*-CLEAN'))
  else:
    p = root / USB_REL
    txt = p.read_text(errors='replace')
    if PIN_OLD not in txt:
      print(f'\n[FAIL] {USB_REL} 里找不到 `{PIN_OLD.strip()}`')
      return 1
    jobs.append((p, PIN_OLD, f'CHESTNUT_FW_VERSION = "{pin_value}"\n', f'固件版本钉成 {pin_value}'))

  print('\n== 2. 备份 ==')
  stamp = time.strftime('%Y%m%d_%H%M%S')
  bdir = BACKUP_ROOT / stamp
  bdir.mkdir(parents=True, exist_ok=True)
  originals = {}
  for p, _old, _new, _desc in jobs:
    bak = bdir / (p.name + '.bak')
    shutil.copy2(p, bak)
    originals[p] = p.read_bytes()
    print(f'  [OK] {bak}  ({bak.stat().st_size} B)')

  print('\n== 3. 写入 ==')
  for p, old, new, desc in jobs:
    st = p.stat()
    txt = p.read_text(errors='replace')
    p.write_text(txt.replace(old, new, 1), encoding='utf-8', newline='')
    _chown(p, st.st_uid, st.st_gid)
    os.chmod(p, st.st_mode & 0o777)
    print(f'  [OK] {p.name}: {desc}')

  print('\n== 4. 语法校验 ==')
  bad = False
  for p, *_ in jobs:
    try:
      py_compile.compile(str(p), cfile=tempfile.mktemp(suffix='.pyc'), doraise=True)
      print(f'  [OK] {p.name} py_compile 通过')
    except py_compile.PyCompileError as e:
      print(f'  [FAIL] {p.name} 语法错误，整批回滚\n{e}')
      bad = True
      break
  if bad:
    for p, data in originals.items():
      p.write_bytes(data)
    print('  已回滚')
    return 1

  print('\n== 5. 结果 ==')
  s2 = state(root)
  print(f"  relax={s2['relax']}  pin={s2['pin']}")
  print('\n下一步：sudo systemctl restart comma')
  print('然后跑 diag_bigmodel_log.py 看这次走到哪一步：')
  print('  · 10 s 后仍 TimeoutError  -> eGPU 不在 USB3 侧或没枚举，不是判据问题')
  print('  · ValueError not a plain tinygrad model -> pkl 格式（legacy）要转换')
  print('  · 34~37 s 成功 -> 大模型起来了')
  return 0


def do_revert(root: Path, backup_root: Path) -> int:
  if not backup_root.is_dir():
    print(f'[FAIL] 没有备份目录 {backup_root}')
    return 1
  baks = sorted(backup_root.glob('*/'))
  if not baks:
    print(f'[FAIL] {backup_root} 下没有备份')
    return 1
  bdir = baks[-1]
  print(f'== 从 {bdir} 回滚 ==')
  n = 0
  for bak in bdir.glob('*.py.bak'):
    name = bak.name[:-len('.bak')]
    for rel in (HELPERS_REL, USB_REL):
      if Path(rel).name == name:
        tgt = root / rel
        if tgt.is_file():
          st = tgt.stat()
          shutil.copy2(bak, tgt)
          _chown(tgt, st.st_uid, st.st_gid)
        else:
          shutil.copy2(bak, tgt)
        print(f'  [OK] {rel}')
        n += 1
  if not n:
    print('  (没有可回滚的文件)')
    return 1
  for rel in (HELPERS_REL, USB_REL):
    p = root / rel
    if p.is_file():
      try:
        py_compile.compile(str(p), cfile=tempfile.mktemp(suffix='.pyc'), doraise=True)
        print(f'  [OK] {rel} py_compile 通过')
      except py_compile.PyCompileError as e:
        print(f'  [FAIL] {rel}\n{e}')
        return 1
  print('\n下一步：sudo systemctl restart comma')
  return 0


def main() -> int:
  global REPO, BACKUP_ROOT, SKIP_USB
  ap = argparse.ArgumentParser(description='放宽 eGPU 固件版本判据')
  ap.add_argument('--check', action='store_true')
  ap.add_argument('--apply', action='store_true')
  ap.add_argument('--revert', action='store_true')
  ap.add_argument('--mode', choices=('relax', 'pin'), default='relax')
  ap.add_argument('--pin-value', default='ed4e39b7', help='--mode pin 时钉的固件号')
  ap.add_argument('--repo', default=str(REPO), help='仓库根（仅测试用）')
  ap.add_argument('--backup-root', default=str(BACKUP_ROOT), help='备份目录（仅测试用）')
  ap.add_argument('--no-usb', action='store_true', help='跳过 sysfs 扫描（仅测试用）')
  args = ap.parse_args()

  REPO = Path(args.repo)
  BACKUP_ROOT = Path(args.backup_root)
  SKIP_USB = args.no_usb
  print(f'仓库 {REPO}\n备份 {BACKUP_ROOT}\n')
  if args.check:
    return do_check(REPO)
  if args.apply:
    return do_apply(REPO, args.mode, args.pin_value)
  if args.revert:
    return do_revert(REPO, BACKUP_ROOT)
  ap.print_help()
  return 2


if __name__ == '__main__':
  sys.exit(main())
