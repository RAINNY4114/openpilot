#!/usr/bin/env python3
"""离车（offroad）自检：eGPU 底座能不能被认出来、能不能真正跑起来。

为什么要单独一个脚本
--------------------
`modeld` / `modeld_tinygrad` 的启动条件是 `and_(only_onroad, ...)`（process_config.py:127/178），
所以停车时它们根本不启动，日志里永远看不到大模型那一段。想在离车状态下验证 eGPU，
必须绕过 modeld，自己开一次设备。

本脚本做四件事：
  1. 读 sysfs：eGPU 到底在不在总线上、product 串是什么、速率多少、LTSSM 是否训练完成
  2. 调 openpilot 的三个判据：chestnut_present() / chestnut_compiled() / cable_connected()
  3. 可选 `--prepare`：调 chestnut_usb_node_ready()，该上电的上电、该等枚举的等枚举
  4. 真正跑一次 tinygrad：矩阵乘正确性 + 显存读回带宽（这一步才算「正常运行」）

为什么用子进程探多个设备串
--------------------------
`modeld_v2` 用的是 `DEV='AMD'`（modeld.py:287，KFD 走 amdgpu），
stock `load_oob()` 用的是 `DEV='USB+AMD:LLVM'`（selfdrive/modeld/helpers.py:17，libusb 直连）。
两条路不一样，而且 tinygrad 初始化失败时可能直接 abort/卡死，所以在**子进程**里逐个试，
带超时，崩了也不影响其它候选。

用法（设备上，必须 offroad；sudo 是为了 params 与 USB 节点权限）
--------------------------------------------------------------
  sudo /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/egpu_smoke.py
  sudo /usr/local/venv/bin/python3 .../egpu_smoke.py --prepare
  sudo /usr/local/venv/bin/python3 .../egpu_smoke.py --run
  sudo /usr/local/venv/bin/python3 .../egpu_smoke.py --run --dev AMD
  sudo /usr/local/venv/bin/python3 .../egpu_smoke.py --run --dev USB+AMD:LLVM --gmmu 0

把 pkl 转成 SPTGPLN1（这一步也需要 eGPU，所以放在 --run 通过之后）：
  sudo /usr/local/venv/bin/python3 .../egpu_smoke.py --convert \
       /data/openpilot/openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl \
       /data/tmp/big_driving_tinygrad.new.pkl
"""
import argparse
import json
import os
import subprocess
import sys
import time
import traceback

OP_ROOT = '/data/openpilot'
OP_INNER = '/data/openpilot/openpilot'
VENV_PY = '/usr/local/venv/bin/python3'
MODELS_DIR = os.path.join(OP_INNER, 'selfdrive/modeld/models')

CANDIDATE_DEVS = ('AMD', 'USB+AMD:LLVM', 'AMD:LLVM')

# modeld 在两个入口都设了这三个（stock modeld.py:9-12 / modeld_v2 modeld.py:12）
BASE_ENV = {
  'GMMU': '0',                 # chestnut fast loading
  'AM_POWER_LIMIT': '100',
  'AM_PERF_LEVEL': 'high',
}


def _setup_path():
  for p in (OP_INNER, OP_ROOT):
    if p not in sys.path:
      sys.path.insert(0, p)


def hr(t):
  print('\n' + '=' * 78)
  print(f' {t}')
  print('=' * 78)


def modeld_running():
  out = []
  try:
    for name in os.listdir('/proc'):
      if not name.isdigit():
        continue
      try:
        cmd = open(f'/proc/{name}/cmdline', 'rb').read().replace(b'\x00', b' ').decode(errors='replace')
      except OSError:
        continue
      if 'modeld' in cmd and 'egpu_smoke' not in cmd:
        out.append((name, cmd.strip()[:90]))
  except OSError:
    pass
  return out


# ---------------------------------------------------------------- 子进程探针
def probe(dev: str) -> int:
  """在独立进程里真正跑一次 tinygrad。父进程用 subprocess 调它。"""
  for k, v in BASE_ENV.items():
    os.environ.setdefault(k, v)
  os.environ['DEV'] = dev

  def emit(ok, **kw):
    print('PROBE_RESULT ' + json.dumps({'ok': ok, 'dev': dev, **kw}, ensure_ascii=False, default=str))

  try:
    t0 = time.monotonic()
    import numpy as np
    from tinygrad import Device, Tensor, dtypes
    emit_import = round(time.monotonic() - t0, 2)
  except Exception as e:
    emit(False, stage='import', err=f'{type(e).__name__}: {e}', tb=traceback.format_exc()[-800:])
    return 1

  # 设备枚举
  try:
    devs = []
    try:
      from tinygrad.device import Device as _D
      devs = sorted(getattr(_D, '_devices', {}) or {})
    except Exception:
      devs = []
  except Exception:
    devs = []

  # 1) 矩阵乘正确性
  try:
    rng = np.random.default_rng(0)
    na = rng.random((512, 512), dtype=np.float32)
    nb = rng.random((512, 512), dtype=np.float32)
    ref = na @ nb

    ta, tb = Tensor(na), Tensor(nb)
    tc = (ta @ tb).realize()
    nc = tc.numpy()
    err = float(np.abs(nc - ref).max())
    rel = float(err / (np.abs(ref).max() + 1e-9))

    t1 = time.monotonic()
    tc2 = (Tensor(na) @ Tensor(nb)).realize()
    tc2.numpy()
    matmul_s = round(time.monotonic() - t1, 3)
  except Exception as e:
    emit(False, stage='matmul', err=f'{type(e).__name__}: {e}', tb=traceback.format_exc()[-1200:])
    return 1

  # 2) 显存读回带宽（判断是不是真的走在 USB3 x4 上）
  bw = {}
  try:
    for mb in (64, 256):
      n = mb * 1024 * 1024 // 4
      t = Tensor.zeros(n, dtype=dtypes.float32).realize()
      t2 = time.monotonic()
      _ = t.numpy()
      dt = time.monotonic() - t2
      bw[f'read_{mb}MB'] = round(mb / 1024 / dt, 2) if dt > 0 else None
      del t, _
  except Exception as e:
    bw['err'] = f'{type(e).__name__}: {e}'[:200]

  emit(True, import_s=emit_import, matmul_err=err, matmul_rel=rel,
       matmul_s=matmul_s, bw_gbps=bw, devices=devs[:8])
  return 0 if rel < 1e-3 else 2


def run_probes(devs, timeout):
  hr(f'4. tinygrad 实测（每个设备串一个子进程，超时 {timeout}s）')
  results = []
  for dev in devs:
    print(f'\n  -- DEV={dev} --')
    t0 = time.monotonic()
    try:
      p = subprocess.run([sys.executable, os.path.abspath(__file__), '--probe', dev],
                         capture_output=True, text=True, timeout=timeout)
      out, err = p.stdout, p.stderr
      rc = p.returncode
    except subprocess.TimeoutExpired as e:
      out = (e.stdout or b'').decode(errors='replace') if isinstance(e.stdout, bytes) else (e.stdout or '')
      err = f'TIMEOUT after {timeout}s'
      rc = -9
    dt = round(time.monotonic() - t0, 1)

    res = None
    for line in out.splitlines():
      if line.startswith('PROBE_RESULT '):
        try:
          res = json.loads(line[len('PROBE_RESULT '):])
        except Exception:
          pass
    tail = '\n'.join((out + err).strip().splitlines()[-12:])
    if res is None:
      print(f'  [FAIL] rc={rc} {dt}s 没拿到结果')
      if tail:
        print('         ' + tail.replace('\n', '\n         '))
      results.append({'dev': dev, 'ok': False, 'rc': rc, 'err': tail[:400]})
      continue

    if not res.get('ok'):
      print(f"  [FAIL] rc={rc} {dt}s stage={res.get('stage')} {res.get('err')}")
      tb = res.get('tb') or ''
      if tb:
        print('         ' + tb.strip().replace('\n', '\n         '))
      results.append(res)
      continue

    print(f"  [OK  ] rc={rc} {dt}s import={res['import_s']}s matmul={res['matmul_s']}s")
    print(f"         512x512 误差 max={res['matmul_err']:.3e} rel={res['matmul_rel']:.3e}"
          f"{'  <-- 数值不对' if res['matmul_rel'] >= 1e-3 else ''}")
    bw = res.get('bw_gbps') or {}
    if bw.get('err'):
      print(f"         带宽测不出：{bw['err']}")
    else:
      r64 = bw.get('read_64MB')
      r256 = bw.get('read_256MB')
      print(f"         读回带宽 64MB={r64} GB/s  256MB={r256} GB/s")
      if r64 is not None and r64 < 0.5:
        print('         注：<0.5 GB/s 说明多半掉到 USB2 或走了慢路径')
    results.append(res)
  return results


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--probe', metavar='DEV', help='（内部用）在子进程里实测指定设备串')
  ap.add_argument('--run', action='store_true', help='真正跑 tinygrad 自检')
  ap.add_argument('--prepare', action='store_true', help='先调 chestnut_usb_node_ready 做上电/等枚举')
  ap.add_argument('--dev', default=None, help='只测这一个设备串（默认依次试 AMD / USB+AMD:LLVM / AMD:LLVM）')
  ap.add_argument('--timeout', type=int, default=120, help='单个设备串的实测超时（秒）')
  ap.add_argument('--gmmu', default=None, help='覆盖 GMMU 环境变量（默认 0，与 modeld 一致）')
  ap.add_argument('--convert', nargs=2, metavar=('IN', 'OUT'), help='调 convert_upstream_pkl.py 转 pkl')
  args = ap.parse_args()

  if args.probe:
    return probe(args.probe)
  if args.gmmu is not None:
    BASE_ENV['GMMU'] = str(args.gmmu)

  _setup_path()

  hr(f'eGPU 离车自检   {time.strftime("%Y-%m-%d %H:%M:%S")}')

  # ---- 0. 先确认没有 modeld 在抢设备
  print('\n== 0. 前置检查 ==')
  procs = modeld_running()
  if procs:
    for pid, cmd in procs:
      print(f'  [FAIL] modeld 正在运行 pid={pid} {cmd}')
    print('  ⇒ 车在 onroad（或 modeld 还没退）。它会独占 eGPU，这里再开只会 EBUSY。')
    print('  ⇒ 先停车/熄火，或 systemctl stop comma 后再跑。')
    if args.run or args.convert:
      return 1
  else:
    print('  [OK  ] 没有 modeld 进程在跑（offroad），eGPU 空闲')

  # ---- 1. sysfs 身份
  print('\n== 1. sysfs 里的 eGPU ==')
  usb_mod = None
  try:
    from openpilot.common.hardware.usb import (
      CHESTNUT_USB_PRODUCT, chestnut_ltssm, chestnut_nodes, is_chestnut_usb_id, node_openable,
    )
    usb_mod = 'ok'
  except Exception as e:
    print(f'  [WARN] import openpilot.common.hardware.usb 失败：{type(e).__name__}: {e}')

  if usb_mod:
    print(f'  期望 product 串 = {CHESTNUT_USB_PRODUCT!r}')
    try:
      nodes = chestnut_nodes()
    except Exception as e:
      nodes = []
      print(f'  [WARN] chestnut_nodes() 抛错：{e}')
    if not nodes:
      print('  [FAIL] sysfs 里没有 chestnut(3801:0001 / ADD1:0001) 或 ROM(174C:246x) 设备')
      print('         ⇒ 物理层：线缆/底座供电/控制器 mode。先跑 _chk/diag_egpu_usb.py 看细节')
    for nd in nodes:
      try:
        openable = node_openable(nd['bus'], nd['dev'])
      except Exception:
        openable = None
      try:
        ltssm = chestnut_ltssm(nd['vendor_id'], nd['product_id'])
      except Exception:
        ltssm = None
      lt = f'0x{ltssm:02X}' if isinstance(ltssm, int) and ltssm >= 0 else str(ltssm)
      kind = 'chestnut' if is_chestnut_usb_id(nd['vendor_id'], nd['product_id']) else 'ROM/bootloader'
      print(f"  {nd.get('bus'):>3}/{nd.get('dev'):<3} {nd['vendor_id']:04x}:{nd['product_id']:04x} "
            f"{nd.get('speed'):>6}Mbps LTSSM={lt:<6} 可写={openable} {kind}")
      prod = nd.get('product')
      if prod is not None:
        mark = 'OK' if prod == CHESTNUT_USB_PRODUCT else 'MISMATCH'
        print(f"        product={prod!r}  [{mark}]")

  # ---- 2. openpilot 的三个判据
  print('\n== 2. openpilot 判据 ==')
  try:
    from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present
    from openpilot.common.hardware.usb import cable_connected
    present = chestnut_present()
    compiled = chestnut_compiled()
    cable = cable_connected()
    print(f'  chestnut_present()  = {present}   （USB 上有 3801:0001 且 product 完全相等）')
    print(f'  chestnut_compiled() = {compiled}  （models 目录里三个 big pkl 齐不齐）')
    print(f'  cable_connected()   = {cable}     （USB-C CC 方向，插充电器也算 True）')
    if not present and cable:
      print('  ⇒ 线插着但认不出设备：product 串不匹配 或 没枚举到 3801:0001')
      print('     先跑 _chk/patch_chx_fw_match.py --check（它会打印真实 product 串）')
    if not compiled:
      print('  ⇒ 三个 big pkl 不齐，即使 eGPU 好了大模型也没得加载')
  except Exception as e:
    print(f'  [WARN] 判据求值失败：{type(e).__name__}: {e}')

  # ---- 3. USB 准备
  if args.prepare:
    hr('3. USB 上电 / 等枚举（chestnut_usb_node_ready）')
    try:
      from openpilot.common.hardware.usb import chestnut_usb_node_ready
      t0 = time.monotonic()
      ok = chestnut_usb_node_ready(timeout=60.0, power_timeout=40.0)
      print(f'  结果 = {ok}  耗时 {time.monotonic() - t0:.1f}s')
      if not ok:
        print('  ⇒ 没等到「节点可写 + USB3 + 链路训练」。看上面 [CHX_USB_ENUM] 日志的判定：')
        print('     · 从没看到可写节点  -> 线缆/权限（system/egpu_usb_perm.sh）')
        print('     · 上电后仍在 USB2   -> 线材不支持 USB3/USB4，或插错口')
    except Exception as e:
      print(f'  [FAIL] {type(e).__name__}: {e}')
      traceback.print_exc()

  # ---- 4. 真正跑
  probe_results = []
  if args.run:
    devs = [args.dev] if args.dev else list(CANDIDATE_DEVS)
    probe_results = run_probes(devs, args.timeout)
    hr('5. 判读')
    good = [r for r in probe_results if r.get('ok') and r.get('matmul_rel', 1) < 1e-3]
    if good:
      best = good[0]
      print(f" [结论] eGPU 可用，设备串 = {best['dev']}")
      print(f"        matmul 512x512 正确（rel={best['matmul_rel']:.2e}），"
            f"读回带宽 {best.get('bw_gbps')}")
      print('        ⇒ 离车验证通过。可以上车了；上车后 modeld_tinygrad 会用 DEV=AMD')
      print('        ⇒ 若 pkl 还是 legacy，先 --convert 转成 SPTGPLN1 再上车，否则大模型仍会被 ValueError 拒掉')
    elif any(r.get('ok') for r in probe_results):
      print(' [结论] 能打开设备但数值不对，看上面的 rel 误差')
    else:
      print(' [结论] 所有候选设备串都打不开 eGPU')
      for r in probe_results:
        print(f"        {r['dev']:14} stage={r.get('stage')} {str(r.get('err'))[:120]}")
      print('        ⇒ 按错误类型分流：')
      print('          PermissionError/EACCES  -> 节点权限，跑 system/egpu_usb_perm.sh 或 --prepare')
      print('          EBUSY/Resource busy    -> 接口被占，重插 eGPU 后立刻跑')
      print('          LIBUSB_ERROR_NO_DEVICE -> 掉线，查供电')
      print('          No device / not found  -> 根本没枚举，回到第 1 节')
    if not args.convert:
      return 0

  # ---- convert
  if args.convert:
    src, dst = args.convert
    hr('4. 转换 pkl 到 SPTGPLN1')
    conv = os.path.join(OP_INNER, 'sunnypilot/modeld_v2/convert_upstream_pkl.py')
    if not os.path.isfile(src):
      print(f'  [FAIL] 找不到 {src}')
      return 1
    with open(src, 'rb') as f:
      head = f.read(16)
    fmt = 'v2(SPTGPLN1)' if head[:8] == b'SPTGPLN1' else 'legacy' if len(head) >= 9 and head[8:9] == b'\x80' else 'pickle' if head[:1] == b'\x80' else 'unknown'
    print(f'  源文件 {src}  {os.path.getsize(src)} B  头 {head[:12].hex(" ")}  判定 {fmt}')
    if fmt.startswith('v2'):
      print('  ⇒ 已经是 SPTGPLN1，不用转')
      return 0
    if not os.path.isfile(conv):
      print(f'  [FAIL] 找不到 {conv}')
      return 1
    print(f'  执行：{VENV_PY} {conv} {src} {dst}')
    env = dict(os.environ, **BASE_ENV)
    t0 = time.monotonic()
    rc = subprocess.call([VENV_PY, conv, src, dst], env=env)
    print(f'  退出码 {rc}  耗时 {time.monotonic() - t0:.1f}s')
    if rc == 0 and os.path.isfile(dst):
      with open(dst, 'rb') as f:
        nh = f.read(16)
      print(f'  产物 {dst}  {os.path.getsize(dst)} B  头 {nh[:12].hex(" ")}'
            f'  {"OK SPTGPLN1" if nh[:8] == b"SPTGPLN1" else "仍不是 SPTGPLN1"}')
    return rc

  if not args.run and not args.convert:
    print('\n（只做了检测。加 --prepare 做上电准备，加 --run 真正跑 tinygrad）')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
