#!/usr/bin/env python3
"""把设备上的大模型加载器恢复成上游 staging-chestnut 原文。

上游：RAINNY4114/openpilot @ 3e9ddb6a71af4b7b0d9709ef1808f666b66553a2
      branch staging-chestnut, commit msg "sunnypilot v2026.003.000"
文件：openpilot/sunnypilot/modeld_v2/modeld.py   523 行 / 23538 B

只动这一个文件。理由：
  - openpilot/selfdrive/modeld/modeld.py 与本文件的本地差异是 **amapNavi 接线**
    （订阅 amapNavi + DH.update(navi, ...) 签名），属用户自有改动，不是大模型代码
  - openpilot/common/hardware/usb.py 本地被扩展成超集（+8094 B），
    上游 modeld.py 只用到 cable_connected（扩展版照样有），恢复它没有收益、只有风险
  - openpilot/selfdrive/modeld/egpu_log.py 是本地新增文件，上游 404；
    恢复后不再被引用，留着无害，本脚本不删

用法（设备上）：
  sudo python3 /data/openpilot/openpilot/_chk/restore_big_model.py --check
  sudo python3 /data/openpilot/openpilot/_chk/restore_big_model.py --apply
  sudo python3 /data/openpilot/openpilot/_chk/restore_big_model.py --revert
"""
import argparse
import hashlib
import os
import py_compile
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path('/data/openpilot/openpilot')
TARGET_REL = 'sunnypilot/modeld_v2/modeld.py'
TARGET = REPO / TARGET_REL
BACKUP_ROOT = Path('/data/bigmodel_restore_backup')
DEFAULT_SRC = Path(__file__).resolve().parent / 'upstream_modeld_v2_modeld.py'
UPSTREAM_COMMIT = '3e9ddb6a71af4b7b0d9709ef1808f666b66553a2'
UPSTREAM_SHA = '1ae7564c4c0e5a220c3add12183831d6fdf1bdecff0fb46d0138409ca6a649b0'
UPSTREAM_SIZE = 23538
UPSTREAM_LINES = 523


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  return h.hexdigest()


def stat_line(path: Path) -> str:
  try:
    st = path.stat()
  except OSError as e:
    return f'(不存在: {e})'
  with open(path, 'rb') as f:
    data = f.read()
  return (f'{len(data)} B / {data.count(b"\n")} 行 / '
          f'{oct(st.st_mode & 0o777)} uid={st.st_uid} gid={st.st_gid}')


def _chown(path: Path, uid: int, gid: int) -> None:
  """保留原属主，避免 sudo 跑完变成 root:root 让 comma 用户改不动。

  Windows 上 os 模块根本没有 chown，所以要 getattr 兜住。
  """
  fn = getattr(os, 'chown', None)
  if fn is None:
    return
  try:
    fn(path, uid, gid)
  except (PermissionError, OSError, NotImplementedError):
    pass


def git_status() -> str:
  try:
    r = subprocess.run(['git', '-C', str(REPO), 'status', '--porcelain', '--', TARGET_REL],
                       capture_output=True, text=True, timeout=20)
    if r.returncode != 0:
      return f'(git 返回 {r.returncode}: {(r.stderr or "").strip()[:90]})'
    out = (r.stdout or '').strip()
    return out if out else '(无改动)'
  except Exception as e:
    return f'(git 不可用: {e})'


def git_head() -> str:
  try:
    r = subprocess.run(['git', '-C', str(REPO), 'log', '-1', '--oneline'],
                       capture_output=True, text=True, timeout=20)
    if r.returncode != 0:
      return f'(git 返回 {r.returncode}: {(r.stderr or "").strip()[:90]})'
    return (r.stdout or '').strip() or '(空)'
  except Exception as e:
    return f'(git 不可用: {e})'


def verify_src(src: Path) -> bool:
  if not src.is_file():
    print(f'  [FAIL] 上游原文不存在: {src}')
    print('         需要先把 upstream_modeld_v2_modeld.py 上传到同目录')
    return False
  size = src.stat().st_size
  digest = sha256(src)
  ok = size == UPSTREAM_SIZE and digest == UPSTREAM_SHA
  print(f'  [{"OK" if ok else "FAIL"}] 上游原文 {size} B  sha256={digest[:16]}...')
  if not ok:
    print(f'         期望 {UPSTREAM_SIZE} B / {UPSTREAM_SHA[:16]}...')
    print('         文件不完整或被改动，拒绝使用（避免写入半截源码把 modeld 打死）')
  return ok


def do_check(src: Path) -> int:
  print('== 1. 上游原文 ==')
  src_ok = verify_src(src)

  print('\n== 2. 设备当前文件 ==')
  if not TARGET.is_file():
    print(f'  [FAIL] 目标不存在: {TARGET}')
    return 1
  cur = sha256(TARGET)
  print(f'  路径   {TARGET}')
  print(f'  大小   {stat_line(TARGET)}')
  print(f'  sha256 {cur}')

  print('\n== 3. 判定 ==')
  if cur == UPSTREAM_SHA:
    print('  [OK] 已经是上游原文，无需恢复')
    state = 'restored'
  else:
    print('  [--] 与上游不一致，需要恢复')
    state = 'modified'

  print('\n== 4. git 视角 ==')
  print(f'  HEAD     {git_head()}')
  print(f'  status   {git_status()}')

  print('\n== 5. 备份 ==')
  baks = sorted(BACKUP_ROOT.glob('*/modeld_v2_modeld.py.bak')) if BACKUP_ROOT.is_dir() else []
  if baks:
    for b in baks[-5:]:
      print(f'  {b}  ({b.stat().st_size} B)')
    print(f'  共 {len(baks)} 份，--revert 用最新那份')
  else:
    print('  (还没有备份)')

  print()
  if not src_ok:
    print('结论：上游原文缺失/损坏，--apply 会被拒绝')
    return 1
  if state == 'restored':
    print('结论：已是上游原文')
    return 0
  print('结论：可执行 --apply 恢复（会先备份，py_compile 失败自动回滚）')
  return 1


def do_apply(src: Path) -> int:
  print('== 1. 校验上游原文 ==')
  if not verify_src(src):
    return 1
  if not TARGET.is_file():
    print(f'  [FAIL] 目标不存在: {TARGET}')
    return 1

  cur = sha256(TARGET)
  if cur == UPSTREAM_SHA:
    print('\n[OK] 已经是上游原文，跳过')
    return 0

  print('\n== 2. 备份当前文件 ==')
  stamp = time.strftime('%Y%m%d_%H%M%S')
  bdir = BACKUP_ROOT / stamp
  bdir.mkdir(parents=True, exist_ok=True)
  bak = bdir / 'modeld_v2_modeld.py.bak'
  shutil.copy2(TARGET, bak)
  if sha256(bak) != cur:
    print(f'  [FAIL] 备份校验失败: {bak}')
    return 1
  print(f'  [OK] {bak}  ({bak.stat().st_size} B, sha256={cur[:16]}...)')

  print('\n== 3. 写入上游原文 ==')
  st = TARGET.stat()
  original = TARGET.read_bytes()
  try:
    TARGET.write_bytes(src.read_bytes())
    # 保留原属主与权限，否则 sudo 跑会把文件写成 root:root，comma 用户改不动
    _chown(TARGET, st.st_uid, st.st_gid)
    os.chmod(TARGET, st.st_mode & 0o777)
  except OSError as e:
    TARGET.write_bytes(original)
    print(f'  [FAIL] 写入失败，已还原: {e}')
    return 1

  new = sha256(TARGET)
  print(f'  [{"OK" if new == UPSTREAM_SHA else "FAIL"}] 写入后 sha256={new[:16]}...')

  print('\n== 4. 语法校验 ==')
  try:
    py_compile.compile(str(TARGET), cfile=tempfile.mktemp(suffix='.pyc'), doraise=True)
    print('  [OK] py_compile 通过')
  except py_compile.PyCompileError as e:
    print(f'  [FAIL] py_compile 失败，回滚\n{e}')
    TARGET.write_bytes(original)
    _chown(TARGET, st.st_uid, st.st_gid)
    return 1

  print('\n== 5. 结果 ==')
  print(f'  {TARGET}')
  print(f'  {stat_line(TARGET)}')
  print(f'  备份: {bak}')
  print('\n下一步：sudo systemctl restart comma')
  print('注意：上游门控是 `_find_driving_pkl(get_active_bundle(chestnut=True)) is not None`，')
  print('      本机 bundle 恒为 None ⇒ 恢复后大模型仍不会加载，且上游没有 [eGPU] 日志，')
  print('      只会看到 `models loaded in ~0.8s`（小模型）。这是上游设计，不是恢复失败。')
  return 0


def do_revert() -> int:
  if not BACKUP_ROOT.is_dir():
    print(f'[FAIL] 没有备份目录 {BACKUP_ROOT}')
    return 1
  baks = sorted(BACKUP_ROOT.glob('*/modeld_v2_modeld.py.bak'))
  if not baks:
    print(f'[FAIL] {BACKUP_ROOT} 下没有备份')
    return 1
  bak = baks[-1]
  print(f'== 从备份恢复 ==\n  {bak}  ({bak.stat().st_size} B)')
  if TARGET.is_file():
    st = TARGET.stat()
    shutil.copy2(bak, TARGET)
    _chown(TARGET, st.st_uid, st.st_gid)
  else:
    shutil.copy2(bak, TARGET)
  print(f'  [OK] 已恢复，sha256={sha256(TARGET)[:16]}...')
  try:
    py_compile.compile(str(TARGET), cfile=tempfile.mktemp(suffix='.pyc'), doraise=True)
    print('  [OK] py_compile 通过')
  except py_compile.PyCompileError as e:
    print(f'  [FAIL] py_compile 失败\n{e}')
    return 1
  print('\n下一步：sudo systemctl restart comma')
  return 0


def main() -> int:
  global TARGET, BACKUP_ROOT, REPO
  ap = argparse.ArgumentParser(description='恢复上游 staging-chestnut 的大模型加载器')
  ap.add_argument('--check', action='store_true', help='只读比对，不写文件')
  ap.add_argument('--apply', action='store_true', help='备份并写入上游原文')
  ap.add_argument('--revert', action='store_true', help='从最近备份恢复')
  ap.add_argument('--src', default=str(DEFAULT_SRC), help='上游原文路径')
  ap.add_argument('--repo', default=str(REPO), help='仓库根（默认 /data/openpilot/openpilot，仅测试用）')
  ap.add_argument('--backup-dir', default=str(BACKUP_ROOT), help='备份根目录（仅测试用）')
  args = ap.parse_args()

  REPO = Path(args.repo)
  TARGET = REPO / TARGET_REL
  BACKUP_ROOT = Path(args.backup_dir)

  if not (args.check or args.apply or args.revert):
    ap.print_help()
    return 2

  print(f'目标 {TARGET}')
  print(f'上游 RAINNY4114/openpilot @ {UPSTREAM_COMMIT[:10]} (staging-chestnut)\n')

  if args.check:
    return do_check(Path(args.src))
  if args.apply:
    return do_apply(Path(args.src))
  return do_revert()


if __name__ == '__main__':
  sys.exit(main())
