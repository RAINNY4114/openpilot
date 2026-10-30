#!/usr/bin/env python3
"""只读诊断：chestnut（eGPU）大模型为什么没有进入加载。

判据全部复刻自 sunnypilot/modeld_v2/modeld.py 的 _resolve_chestnut_gate()，
但**故意不导入那个模块** —— 它 import tinygrad 会打开 USB 设备，
可能抢走正在运行的 modeld 的 eGPU。

用法（设备上）：
  /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/diag_egpu_load.py
  /usr/local/venv/bin/python3 .../diag_egpu_load.py --log-lines 60
"""
import argparse
import glob
import json
import os
import struct
import sys
import time

sys.path.insert(0, '/data/openpilot')
sys.path.insert(0, '/data/openpilot/openpilot')

PKL_MAGIC = b'SPTGPLN1'
PARAM_DIRS = ('/data/ford_params', '/dev/shm/params', '/data/params/d')
SWAGLOG_GLOBS = ('/data/log/swaglog.*',)
EGPU_DIAG = '/data/egpu_diag.jsonl'

# 只看这些精确串，别用裸 chestnut（会被 UI 纹理、models_manager 刷屏）
LOG_KEYS = (
  'models loaded in',
  'loading combined pkl',
  'chestnut load failed',
  '[eGPU]',
  '[CHX_USB_ENUM]',
  '[CHX_PRE_POWER]',
  '[CHESTNUT_GATE',
  'Resource busy',
  'usb node not accessible',
  'not a plain tinygrad model',
)

ok = fail = warn = 0


def line(tag, msg, extra=''):
  global ok, fail, warn
  if tag == 'OK':
    ok += 1
  elif tag == 'FAIL':
    fail += 1
  else:
    warn += 1
  print(f'  [{tag:4}] {msg}' + (f'  --  {extra}' if extra else ''))


def read_param(key):
  for d in PARAM_DIRS:
    p = os.path.join(d, key)
    if os.path.isfile(p):
      try:
        return open(p, 'rb').read().decode('utf-8', 'replace').strip(), p
      except OSError:
        pass
  return None, None


def pkl_format(path):
  """与 modeld.py:_pkl_format 完全同构。"""
  try:
    with open(path, 'rb') as f:
      head = f.read(16)
  except OSError:
    return 'missing'
  if head[:8] == PKL_MAGIC:
    return 'v2'
  if head[:1] == b'\x80':
    return 'pickle'
  if len(head) >= 9 and head[8:9] == b'\x80':
    return 'legacy'
  return 'unknown'


def fmt_note(fmt):
  return {
    'v2': 'SPTGPLN1 容器，modeld_v2 能读',
    'legacy': 'load_oob 容器（旧 selfdrive/modeld 用），modeld_v2 读不了',
    'pickle': '裸 pickle，modeld_v2 读不了',
    'missing': '文件不存在',
    'unknown': '无法识别',
  }.get(fmt, fmt)


def sec_gate():
  print('\n===== 1. CHESTNUT 门控（逐项求值，顺序与源码一致）=====')
  try:
    from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present, modeld_pkl_path
    from openpilot.common.hardware.usb import cable_connected
    from openpilot.sunnypilot.models.helpers import get_active_bundle
    from openpilot.common.hardware.hw import Paths
  except Exception as e:
    line('FAIL', '导入失败，无法复现门控', repr(e))
    return None

  # --- bundle ---
  bundle = None
  try:
    bundle = get_active_bundle(chestnut=True)
  except Exception as e:
    line('WARN', 'get_active_bundle(chestnut=True) 抛异常', repr(e))
  if bundle is None:
    line('WARN', 'chestnut bundle 为 None -> pkl 只能走 fallback（MODELS_DIR）',
         'ModelManager_ActiveBundleChestnut 未设置或模型清单拉不到')
  else:
    line('OK', 'chestnut bundle 存在', f'generation={getattr(bundle, "generation", None)} models={len(bundle.models)}')

  # --- pkl 路径 ---
  pkl, src = None, None
  if (ov := os.environ.get('COMBINED_MODEL_PKL')) and os.path.exists(ov):
    pkl, src = ov, 'override'
  if pkl is None and bundle is not None and bundle.models:
    p = os.path.join(Paths.model_root(), bundle.models[0].artifact.fileName)
    if os.path.exists(p):
      pkl, src = p, 'bundle'
    else:
      line('WARN', 'bundle 里的 pkl 在 model_root 下不存在', p)
  if pkl is None:
    try:
      if chestnut_compiled():
        p = str(modeld_pkl_path(chestnut=True))
        if os.path.exists(p):
          pkl, src = p, 'fallback'
    except Exception as e:
      line('WARN', 'chestnut_compiled() 抛异常', repr(e))

  fmt = pkl_format(pkl) if pkl else 'missing'
  if pkl:
    sz = os.path.getsize(pkl) / 1048576
    line('OK' if fmt == 'v2' else 'FAIL', f'pkl 格式 = {fmt}', f'{fmt_note(fmt)} | {sz:.1f} MiB')
    print(f'         path = {pkl}')
    print(f'         src  = {src}')
  else:
    line('FAIL', 'pkl 路径为 None（bundle 无、fallback 也没找到）')

  # --- 硬件 ---
  try:
    present = chestnut_present()
    line('OK' if present else 'FAIL', f'chestnut_present() = {present}',
         'USB 上有 3801:0001 且 product 匹配' if present else 'eGPU 未枚举或 product 串不匹配')
  except Exception as e:
    present = False
    line('FAIL', 'chestnut_present() 抛异常', repr(e))
  try:
    cable = cable_connected()
    line('WARN', f'cable_connected() = {cable}', '只反映 USB-C CC 方向，插充电器也是 True，不参与门控')
  except Exception:
    cable = None
  try:
    compiled = chestnut_compiled()
    line('OK' if compiled else 'FAIL', f'chestnut_compiled() = {compiled}',
         'big_driving + 两个 warp pkl 都在' if compiled else '缺 pkl')
  except Exception as e:
    compiled = False
    line('FAIL', 'chestnut_compiled() 抛异常', repr(e))

  # --- 门控结论 ---
  gate = pkl is not None and fmt == 'v2' and present
  print()
  if gate:
    line('OK', 'CHESTNUT 门控 = True -> load_big() 线程会被创建')
  else:
    why = []
    if pkl is None:
      why.append('pkl 路径为 None')
    elif fmt != 'v2':
      why.append(f'pkl 格式是 {fmt} 不是 v2')
    if not present:
      why.append('chestnut_present() = False')
    line('FAIL', 'CHESTNUT 门控 = False -> **load_big() 线程根本不会创建**', ' + '.join(why))
  return gate


def sec_usb():
  print('\n===== 2. eGPU USB 枚举现状 =====')
  try:
    from openpilot.common.hardware.usb import (CHESTNUT_USB_PRODUCT, LTSSM_TRAINED, USB3_MIN_SPEED,
                                               chestnut_ltssm, chestnut_nodes, node_openable,
                                               pre_power_enabled)
  except Exception as e:
    line('FAIL', '导入 usb 模块失败', repr(e))
    return
  line('OK' if pre_power_enabled() else 'WARN', f'pre_power_enabled() = {pre_power_enabled()}',
       'dp_chestnut_pre_power 控制是否允许发 0xF3 上电')
  nodes = []
  try:
    nodes = chestnut_nodes()
  except Exception as e:
    line('WARN', 'chestnut_nodes() 抛异常', repr(e))
  if not nodes:
    line('FAIL', '没有枚举到 chestnut USB 节点', '查线缆 / 底座 12V / 是否只走 USB2')
  for n in nodes:
    bus, dev = n['bus'], n['dev']
    op = False
    try:
      op = node_openable(bus, dev)
    except Exception:
      pass
    ltssm = -2
    try:
      if n['speed'] >= USB3_MIN_SPEED:
        ltssm = chestnut_ltssm(n['vendor_id'], n['product_id'])
    except Exception:
      pass
    usb3 = n['speed'] >= USB3_MIN_SPEED
    trained = ltssm == LTSSM_TRAINED
    tag = 'OK' if (usb3 and (trained or ltssm == -1)) else 'FAIL'
    line(tag, f'bus={bus:03d}/{dev:03d} speed={n["speed"]}Mbps',
         f'vid:pid={n["vendor_id"]:04x}:{n["product_id"]:04x} openable={op} '
         f'USB3={usb3} LTSSM={("0x%02X" % (ltssm & 0xFF)) if ltssm >= 0 else ltssm} '
         f'{"(已训练)" if trained else ""}')
  print(f'         product 期望串 = {CHESTNUT_USB_PRODUCT!r}')


def sec_params():
  print('\n===== 3. 相关参数 =====')
  for k in ('ChestnutLoading', 'ChestnutActive', 'ChestnutRetry',
            'ModelManager_ActiveBundleChestnut', 'ModelManager_ActiveBundle',
            'dp_chestnut_pre_power', 'DisableUpdates'):
    v, p = read_param(k)
    if v is None:
      line('WARN', f'{k} 未设置')
    else:
      short = v if len(v) <= 90 else v[:90] + '...'
      line('OK', f'{k} = {short}', p)


def sec_diag_jsonl(n):
  print(f'\n===== 4. {EGPU_DIAG} =====')
  if not os.path.exists(EGPU_DIAG):
    line('WARN', '文件不存在', '说明本 boot 里 modeld 还没走到 egpu() 埋点，或已轮转')
    return
  try:
    with open(EGPU_DIAG) as f:
      rows = f.readlines()
  except OSError as e:
    line('FAIL', '读取失败', repr(e))
    return
  line('OK', f'共 {len(rows)} 行，显示最后 {n} 行')
  for r in rows[-n:]:
    try:
      d = json.loads(r)
    except Exception:
      print('    ' + r.rstrip()[:200])
      continue
    ev = d.pop('ev', '?')
    boot = str(d.pop('boot', ''))[:8]
    ts = d.pop('ts', None)
    when = time.strftime('%m-%d %H:%M:%S', time.localtime(ts)) if isinstance(ts, (int, float)) else ''
    body = ' '.join(f'{k}={v}' for k, v in d.items() if k not in ('pid',))
    print(f'    {when} boot={boot} {ev:22} {body}'[:240])


def sec_swaglog(n):
  print(f'\n===== 5. swaglog 关键行（最近 {n} 条）=====')
  files = []
  for g in SWAGLOG_GLOBS:
    files += glob.glob(g)
  files = [f for f in files if os.path.isfile(f)]
  files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
  if not files:
    line('FAIL', '找不到 swaglog')
    return
  hits = []
  for path in files[:4]:
    try:
      with open(path, errors='replace') as f:
        for raw in f:
          if not any(k in raw for k in LOG_KEYS):
            continue
          try:
            d = json.loads(raw)
          except Exception:
            hits.append((os.path.getmtime(path), path, raw.rstrip()[:200]))
            continue
          msg = str(d.get('msg', ''))
          if not any(k in msg for k in LOG_KEYS):
            continue
          ts = d.get('time') or d.get('created') or os.path.getmtime(path)
          hits.append((ts, os.path.basename(path), f'{d.get("level", "")} {d.get("filename", "")} | {msg}'))
    except OSError:
      continue
  hits.sort(key=lambda x: x[0])
  if not hits:
    line('WARN', '没有命中任何关键行')
    return
  line('OK', f'命中 {len(hits)} 条')
  for ts, fn, msg in hits[-n:]:
    when = time.strftime('%m-%d %H:%M:%S', time.localtime(ts)) if isinstance(ts, (int, float)) else '?'
    print(f'    {when} [{fn}] {msg}'[:240])


def sec_pkls():
  print('\n===== 6. 所有候选 pkl 的格式 =====')
  try:
    from openpilot.selfdrive.modeld.helpers import MODELS_DIR
  except Exception as e:
    line('FAIL', '拿不到 MODELS_DIR', repr(e))
    return
  found = False
  for p in sorted(glob.glob(str(MODELS_DIR / '*.pkl'))):
    found = True
    f = pkl_format(p)
    sz = os.path.getsize(p) / 1048576
    tag = 'OK' if f == 'v2' else 'WARN'
    line(tag, f'{os.path.basename(p):46} {f:8} {sz:8.1f} MiB')
  if not found:
    line('FAIL', f'{MODELS_DIR} 下没有任何 pkl')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--log-lines', type=int, default=30)
  ap.add_argument('--diag-lines', type=int, default=25)
  args = ap.parse_args()

  print('=' * 78)
  print(' chestnut 大模型「没进入加载」只读诊断')
  print(f' {time.strftime("%Y-%m-%d %H:%M:%S")}  uptime={open("/proc/uptime").read().split()[0]}s')
  print('=' * 78)

  gate = sec_gate()
  sec_usb()
  sec_params()
  sec_diag_jsonl(args.diag_lines)
  sec_swaglog(args.log_lines)
  sec_pkls()

  print('\n' + '=' * 78)
  print(f' 汇总：OK {ok} / WARN {warn} / FAIL {fail}')
  if gate is False:
    print(' 判定：门控为假 ⇒ load_big() 从未创建 ⇒ 大模型「没有进入加载」')
    print(' 下一步看第 1 节哪一项 FAIL；若是 pkl 格式，用')
    print('   /usr/local/venv/bin/python3 sunnypilot/modeld_v2/convert_upstream_pkl.py <legacy.pkl> <out.pkl>')
    print(' 转成 SPTGPLN1 容器')
  elif gate is True:
    print(' 判定：门控为真 ⇒ 加载器已进入，失败点在 load_big() 内部（看第 4/5 节）')
  print('=' * 78)
  return 0 if fail == 0 else 1


if __name__ == '__main__':
  sys.exit(main())
