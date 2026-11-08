#!/usr/bin/env python3
"""只读：从 swaglog 里捞出大模型（chestnut）加载全过程，定位失败点。

swaglog 是 NDJSON（每行一个 JSON），所以直接按行 json.loads，不要走 cereal_log。

用法（设备上）：
  python3 /data/openpilot/openpilot/_chk/diag_bigmodel_log.py
  python3 .../diag_bigmodel_log.py --files 6 --tail 60
"""
import argparse
import glob
import json
import os
import sys
import time

GLOBS = ('/data/log/swaglog.*',)

# 只要这些精确串，别用裸 "chestnut"（UI 纹理、models_manager 会刷屏）
KEYS = (
  'chestnut',
  'models loaded in',
  'big model',
  'loading combined pkl',
  'ChestnutLoading',
  'ChestnutActive',
  'ChestnutRetry',
  'not a plain tinygrad model',
  'Resource busy',
  'did not enumerate',
)

# 命中这些就整段打印（含 traceback）
VERBOSE = ('chestnut load failed', 'big model load failed', 'Traceback', 'did not enumerate')


def ts(v):
  try:
    return time.strftime('%m-%d %H:%M:%S', time.localtime(float(v)))
  except Exception:
    return '?'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--files', type=int, default=4, help='最多看最近 N 个 swaglog')
  ap.add_argument('--tail', type=int, default=40, help='最多打印 N 条')
  ap.add_argument('--glob', default=GLOBS[0], help='日志通配（仅测试用）')
  args = ap.parse_args()

  files = []
  for g in (args.glob,):
    files += glob.glob(g)
  files = [f for f in files if os.path.isfile(f)]
  files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
  if not files:
    print(f'[FAIL] 找不到 swaglog: {args.glob}')
    return 1

  print('=' * 78)
  print(f' 大模型加载日志提取   {time.strftime("%Y-%m-%d %H:%M:%S")}')
  print(f' 扫描最近 {min(args.files, len(files))} 个文件（共 {len(files)} 个）')
  print('=' * 78)
  for p in files[:args.files]:
    print(f'  {p}  {os.path.getsize(p) / 1048576:.1f} MiB  {ts(os.path.getmtime(p))}')

  hits = []
  for path in files[:args.files]:
    try:
      with open(path, errors='replace') as f:
        for raw in f:
          if not any(k in raw for k in KEYS):
            continue
          try:
            d = json.loads(raw)
          except Exception:
            hits.append((os.path.getmtime(path), os.path.basename(path), '?', '?', raw.rstrip()[:400]))
            continue
          msg = str(d.get('msg', ''))
          if not any(k in msg for k in KEYS):
            continue
          hits.append((
            d.get('time') or d.get('created') or os.path.getmtime(path),
            os.path.basename(path),
            str(d.get('level', '')),
            str(d.get('filename') or d.get('daemon') or ''),
            msg,
          ))
    except OSError:
      continue

  if not hits:
    print('\n[WARN] 一条都没命中。可能：modeld 从没走到大模型分支，或日志已轮转。')
    return 1

  hits.sort(key=lambda x: float(x[0]) if str(x[0]).replace('.', '').isdigit() else 0)
  print(f'\n===== 命中 {len(hits)} 条，显示最后 {min(args.tail, len(hits))} 条 =====')
  for t, fn, lvl, src, msg in hits[-args.tail:]:
    body = msg.replace('\n', '\n      ')
    print(f'  {ts(t)} [{fn}] {lvl:8} {src:16} | {body[:400]}')

  # 关键结论
  print('\n' + '=' * 78)
  print(' 判读')
  joined = '\n'.join(h[4] for h in hits)
  loaded = [h for h in hits if 'models loaded in' in h[4]]
  if 'did not enumerate' in joined:
    print(' [结论] chestnut_present() 一直为假 ⇒ wait_for_chestnut 10s 超时。')
    print('        说明 eGPU 的 product 串不等于 "custom 8516b768-CLEAN"，或不在 USB3 侧。')
    print('        下一步：python3 /data/openpilot/openpilot/_chk/diag_egpu_usb.py')
  elif 'not a plain tinygrad model' in joined:
    print(' [结论] pkl 是 legacy/裸 pickle，不是 SPTGPLN1 容器 ⇒ modeld_v2 读不了。')
    print('        下一步：用 sunnypilot/modeld_v2/convert_upstream_pkl.py 转换。')
  elif 'Resource busy' in joined:
    print(' [结论] libusb_set_configuration EBUSY ⇒ 接口被别的驱动/进程占着。')
    print('        下一步：查 /data/egpu_usb_perm.log 的 unbind 次数，重插 eGPU。')
  elif 'chestnut load failed' in joined:
    print(' [结论] load_big 抛异常但没匹配到已知模式，看上面 Traceback 原文。')
  elif loaded:
    last = loaded[-1][4]
    print(f' [结论] 最近一次启动：{last[:120]}')
    print('        耗时 34~37s 量级 = 大模型成功；0.8~1.2s = 直接小模型（门控为假）；')
    print('        60s 左右 = BIG_MODEL_TIMEOUT 超时。')
  else:
    print(' [结论] 只有零星 chestnut 行，没有加载记录 —— 门控可能为假。')
  print('=' * 78)
  return 0


if __name__ == '__main__':
  sys.exit(main())
