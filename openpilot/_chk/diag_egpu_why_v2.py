#!/usr/bin/env python3
"""只读：回答「大模型为什么没加载」—— 不依赖 swaglog 也能出结论。

上一版 diag_bigmodel_log.py 只翻 swaglog，结果一条没命中，因为：
  · process_config.py:178 `modeld_tinygrad` 的启动条件是 `and_(only_onroad, is_tinygrad_model)`
    ⇒ **车不在 onroad 状态时 modeld 根本不启动**，swaglog 里当然没有任何 chestnut 行；
  · swaglog 按重启轮转，最新几个文件可能只有十几 KB。

所以这里改成四路取证的顺序：
  1. 进程在不在（决定「门控有没有被求值」）
  2. Params 真值表（ChestnutLoading / ChestnutActive / ChestnutRetry 是 modeld 亲手写的，
     且用 mtime 与本次 boot 时刻比较，能排除上次开机残留）
  3. /data/egpu_diag.jsonl（egpu_log.py 的结构化记录，比 swaglog 干净得多）
  4. swaglog（覆盖情况 + 命中行）
  5. eGPU 在 sysfs 里的真实身份（product 串 / 速率）

不 import 任何 openpilot 模块，不碰 tinygrad，offroad 时也能安全跑。

用法（设备上，需要 sudo 才读得到 /data/params/d）：
  sudo python3 /data/openpilot/openpilot/_chk/diag_egpu_why.py
  sudo python3 .../diag_egpu_why.py --swaglog-files 12 --tail 60
"""
import argparse
import glob
import json
import os
import time

PARAMS_DIR = '/data/params/d'
DIAG_PATH = '/data/egpu_diag.jsonl'
LOG_DIR = '/data/log'
PROC_DIR = '/proc'

# modeld 亲手写的三个键：读它们比翻日志可靠
PARAM_KEYS = (
  'ChestnutLoading',
  'ChestnutActive',
  'ChestnutRetry',
  'ModelRunnerTypeCache',
  'OnroadCycleRequested',
)

# 判断 onroad 的旁证：这些进程也只在 onroad 时存在
ONROAD_PROCS = ('modeld', 'modeld_tinygrad', 'selfdrived', 'controlsd', 'card', 'plannerd', 'calibrationd')

# swaglog 里值得看的串
KEYS = (
  '[eGPU]',
  'chestnut',
  'modeld init',
  'models loaded in',
  'big model',
  'loading combined pkl',
  'not a plain tinygrad model',
  'Resource busy',
  'did not enumerate',
  'chestnut flash',
  'firmware out of date',
  'flashing',
)

# hardwared 的 ChestnutStatus 会把诊断结论写成 Offroad_Chestnut* 参数
# （system/hardware/chestnut/status.py:95-106），这是最直接的「为什么橙色」答案
ALERT_PREFIX = 'Offroad_Chestnut'

VERBOSE = ('chestnut load failed', 'big model load failed', 'Traceback', 'did not enumerate')


def ts(v):
  try:
    return time.strftime('%m-%d %H:%M:%S', time.localtime(float(v)))
  except Exception:
    return '?'


def human(n):
  if n >= 1048576:
    return f'{n / 1048576:.1f} MiB'
  if n >= 1024:
    return f'{n / 1024:.0f} KiB'
  return f'{n} B'


def boot_info(proc_dir):
  try:
    up = float(open(os.path.join(proc_dir, 'uptime')).read().split()[0])
  except (OSError, ValueError, IndexError):
    return None, None
  boot_epoch = time.time() - up
  bid = '?'
  try:
    bid = open(os.path.join(proc_dir, 'sys/kernel/random/boot_id')).read().strip()[:8]
  except OSError:
    pass
  return boot_epoch, (up, bid)


def scan_procs(proc_dir):
  """返回 {进程名: [pid, ...]}，只看 ONROAD_PROCS。"""
  found = {}
  try:
    names = os.listdir(proc_dir)
  except OSError:
    return found
  for name in names:
    if not name.isdigit():
      continue
    try:
      raw = open(os.path.join(proc_dir, name, 'cmdline'), 'rb').read()
    except OSError:
      continue
    if not raw:
      continue
    cmd = raw.replace(b'\x00', b' ').decode(errors='replace').strip()
    for key in ONROAD_PROCS:
      if key not in cmd:
        continue
      # modeld 是 modeld_tinygrad 的子串，这里按 argv 特征区分：
      #   modeld_tinygrad 的 cmdline 含 "modeld_v2/modeld.py"（setproctitle 后直接是 modeld_tinygrad）
      if key == 'modeld' and ('modeld_tinygrad' in cmd or 'modeld_v2' in cmd):
        continue
      found.setdefault(key, []).append((int(name), cmd[:90]))
  return found


def read_params(params_dir, boot_epoch):
  out = {}
  for key in PARAM_KEYS:
    p = os.path.join(params_dir, key)
    if not os.path.exists(p):
      out[key] = {'exists': False}
      continue
    try:
      raw = open(p, 'rb').read()
      mt = os.path.getmtime(p)
    except OSError as e:
      out[key] = {'exists': True, 'err': repr(e)}
      continue
    if raw in (b'1', b'\x01'):
      val = 'True'
    elif raw in (b'0', b'\x00'):
      val = 'False'
    elif not raw:
      val = '(空)'
    else:
      val = raw[:40].decode(errors='replace')
    out[key] = {
      'exists': True, 'raw': raw[:16], 'val': val, 'mtime': mt,
      'stale': (boot_epoch is not None and mt < boot_epoch),
    }
  return out


def read_alerts(params_dir, boot_epoch):
  """读 /data/params/d/Offroad_Chestnut* —— hardwared 亲手写的诊断结论。"""
  out = []
  try:
    names = sorted(os.listdir(params_dir))
  except OSError:
    return out
  for name in names:
    if not name.startswith(ALERT_PREFIX):
      continue
    p = os.path.join(params_dir, name)
    try:
      raw = open(p, 'rb').read()
      mt = os.path.getmtime(p)
    except OSError:
      continue
    txt = raw.decode(errors='replace')
    out.append({'key': name, 'raw': txt, 'mtime': mt,
                'stale': (boot_epoch is not None and mt < boot_epoch)})
  return out


def read_diag(diag_path, tail):
  if not os.path.isfile(diag_path):
    return None, []
  recs = []
  try:
    with open(diag_path, errors='replace') as f:
      for line in f:
        line = line.strip()
        if not line:
          continue
        body = line[len('[eGPU] '):] if line.startswith('[eGPU] ') else line
        try:
          recs.append(json.loads(body))
        except Exception:
          recs.append({'event': '(unparsable)', 'raw': line[:160]})
  except OSError:
    return None, []
  return os.path.getsize(diag_path), recs[-tail:]


def scan_swaglog(log_dir, want, tail):
  files = sorted(glob.glob(os.path.join(log_dir, 'swaglog.*')),
                 key=lambda p: os.path.getmtime(p), reverse=True)
  info = []
  for p in files[:want]:
    try:
      info.append((p, os.path.getsize(p), os.path.getmtime(p)))
    except OSError:
      pass
  hits = []
  for p, _sz, _mt in info:
    try:
      with open(p, errors='replace') as f:
        for raw in f:
          if not any(k in raw for k in KEYS):
            continue
          try:
            d = json.loads(raw)
          except Exception:
            hits.append((_mt, os.path.basename(p), '?', '?', raw.rstrip()[:300]))
            continue
          msg = str(d.get('msg', ''))
          if not any(k in msg for k in KEYS):
            continue
          hits.append((d.get('time') or d.get('created') or _mt,
                       os.path.basename(p), str(d.get('level', '')),
                       str(d.get('filename') or d.get('daemon') or ''), msg))
    except OSError:
      continue
  hits.sort(key=lambda x: float(x[0]) if str(x[0]).replace('.', '').isdigit() else 0)
  return len(files), info, hits[-tail:]


def scan_usb():
  ids = {(0xADD1, 0x0001), (0x3801, 0x0001)}
  rom = {(0x174C, 0x2464), (0x174C, 0x2463)}
  out = []
  root = '/sys/bus/usb/devices'
  if not os.path.isdir(root):
    return out
  for name in sorted(os.listdir(root)):
    if ':' in name:
      continue
    d = os.path.join(root, name)
    try:
      vid = int(open(os.path.join(d, 'idVendor')).read().strip(), 16)
      pid = int(open(os.path.join(d, 'idProduct')).read().strip(), 16)
    except (OSError, ValueError):
      continue
    kind = 'chestnut' if (vid, pid) in ids else 'rom' if (vid, pid) in rom else None
    if kind is None:
      continue
    prod = ''
    for f in ('product', 'manufacturer'):
      try:
        prod += open(os.path.join(d, f)).read().strip() + ' '
      except OSError:
        pass
    try:
      speed = open(os.path.join(d, 'speed')).read().strip()
    except OSError:
      speed = '?'
    out.append({'name': name, 'vid': vid, 'pid': pid, 'kind': kind,
                'product': prod.strip(), 'speed': speed})
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--params-dir', default=PARAMS_DIR)
  ap.add_argument('--diag-path', default=DIAG_PATH)
  ap.add_argument('--log-dir', default=LOG_DIR)
  ap.add_argument('--proc-dir', default=PROC_DIR)
  ap.add_argument('--swaglog-files', type=int, default=8, help='看最近 N 个 swaglog')
  ap.add_argument('--tail', type=int, default=40)
  args = ap.parse_args()

  print('=' * 78)
  print(f' eGPU 大模型「为什么没加载」只读诊断   {time.strftime("%Y-%m-%d %H:%M:%S")}')
  print('=' * 78)

  # ---- 0. 环境
  boot_epoch, extra = boot_info(args.proc_dir)
  print('\n== 0. 环境 ==')
  if extra:
    up, bid = extra
    print(f'  uptime={up:.0f}s  boot={ts(boot_epoch)}  boot_id={bid}')
  else:
    print('  [WARN] 读不到 /proc/uptime')

  # ---- 1. 进程
  print('\n== 1. 进程（决定门控有没有被求值）==')
  procs = scan_procs(args.proc_dir)
  if not procs:
    print('  [WARN] modeld / selfdrived / controlsd / card 一个都没有')
  for key in ONROAD_PROCS:
    if key in procs:
      for pid, cmd in procs[key][:3]:
        print(f'  [OK  ] {key:16} pid={pid:<7} {cmd}')
  onroad = any(k in procs for k in ('modeld', 'modeld_tinygrad', 'selfdrived', 'controlsd'))
  if 'modeld_tinygrad' in procs:
    print('  ⇒ modeld_tinygrad 在跑，门控已被求值（大模型分支的入口在 main() 开头）')
  elif onroad:
    print('  ⇒ 在 onroad，但 modeld_tinygrad 没跑 ⇒ is_tinygrad_model 为假（runner 不是 tinygrad）')
  else:
    print('  ⇒ 不在 onroad ⇒ process_config.py:178 的 and_(only_onroad, ...) 直接拦住，')
    print('     modeld 从未启动，swaglog 里不会有任何 chestnut 行。这是上一版日志空手而归的原因。')

  # ---- 2. Params
  print('\n== 2. Params 真值表（modeld 亲手写的）==')
  pm = read_params(args.params_dir, boot_epoch)
  for key in PARAM_KEYS:
    v = pm[key]
    if not v['exists']:
      print(f'  [ -- ] {key:22} 不存在')
      continue
    if 'err' in v:
      print(f'  [WARN] {key:22} 读取失败 {v["err"]}')
      continue
    tag = 'STALE' if v['stale'] else ' ok  '
    print(f'  [{tag}] {key:22} = {v["val"]:8} mtime={ts(v["mtime"])}  raw={v["raw"]!r}')
  print('  解读：')
  print('    ChestnutLoading=True  且 ChestnutActive 不存在 ⇒ 正在加载大模型（卡在 load_big 里）')
  print('    ChestnutLoading=False 且 ChestnutActive=True  ⇒ 大模型已生效')
  print('    ChestnutLoading=False 且 ChestnutActive=False ⇒ 加载失败（ChestnutRetry 说明是否在重试）')
  print('    ChestnutActive 不存在且 Loading=False         ⇒ 门控为假，load_big() 从未创建')
  if any(pm.get(k, {}).get('stale') for k in PARAM_KEYS):
    print('  [注意] 标 STALE 的值来自上一次开机，不能当本次结论')

  # ---- 2b. hardwared 的 chestnut 告警（橙色图标的直接来源）
  print('\n== 2b. Offroad_Chestnut* 告警（hardwared 写的诊断结论）==')
  alerts = read_alerts(args.params_dir, boot_epoch)
  if not alerts:
    print('  (没有任何 Offroad_Chestnut* 参数 ⇒ 当前没有 chestnut 相关告警)')
  for a in alerts:
    tag = 'STALE' if a['stale'] else ' ok  '
    print(f"  [{tag}] {a['key']:32} mtime={ts(a['mtime'])}  {a['raw'][:80]!r}")
  print('  对照 system/hardware/chestnut/status.py:')
  print('    NotDetected     -> 插着但枚举不到（或 usb_failed）')
  print('    UsbSlow         -> 枚举到了但速率 < 5000Mbps')
  print('    PcieUnavailable -> PCIe 链路没训练起来 / 12V 供电掉了')
  print('    UpdateFailed    -> 固件刷写失败（hardwared 自动 flash 3 次都没成功）')
  print('    ModelError      -> 大模型加载失败')
  print('    Branch          -> 当前渠道需要切到 chestnut 分支')

  # ---- 3. egpu_diag.jsonl
  print(f'\n== 3. {args.diag_path} ==')
  size, recs = read_diag(args.diag_path, args.tail)
  gate_rec = None
  if size is None:
    print('  [WARN] 文件不存在 ⇒ egpu_log.egpu() 一次都没被调用过')
  else:
    print(f'  大小 {human(size)}，显示最后 {len(recs)} 条：')
    for r in recs:
      ev = str(r.get('event', '?'))
      body = {k: v for k, v in r.items() if k not in ('event', 'ts', 'time', 'boot', 'boot_id', 'pid')}
      line = ' '.join(f'{k}={v}' for k, v in body.items())
      print(f'    {ts(r.get("ts") or r.get("time") or 0)} {ev:22} {line}'[:230])
      if ev == 'gate':
        gate_rec = r
    print('  说明：gate 记录里的 chestnut= 就是门控最终值；load_start/load_ok/load_fail 是 load_big 的进出')

  # ---- 4. swaglog
  print(f'\n== 4. swaglog（{args.log_dir}）==')
  total, info, hits = scan_swaglog(args.log_dir, args.swaglog_files, args.tail)
  print(f'  共 {total} 个文件，看最近 {len(info)} 个：')
  for p, sz, mt in info:
    print(f'    {os.path.basename(p):28} {human(sz):>9}  {ts(mt)}')
  if hits:
    print(f'\n  命中 {len(hits)} 条（已按时间排序，显示最后 {len(hits)} 条）：')
    for t, fn, lvl, src, msg in hits:
      body = msg.replace('\n', '\n      ')
      print(f'    {ts(t)} [{fn}] {lvl:8} {src:14} | {body[:300]}')
  else:
    print('  一条都没命中')

  # ---- 5. USB
  print('\n== 5. eGPU 在 sysfs 里的真实身份 ==')
  usb = scan_usb()
  if not usb:
    print('  [WARN] 没有 3801:0001 / ADD1:0001 / 174C:246x 设备 ⇒ eGPU 完全没枚举')
  for d in usb:
    print(f"  {d['name']:8} {d['vid']:04x}:{d['pid']:04x} {d['speed']:>7}Mbps "
          f"{d['kind']:8} {d['product']!r}")

  # ---- 6. 判读
  print('\n' + '=' * 78)
  print(' 判读')
  alert_keys = {a['key'] for a in alerts if not a['stale']}
  if alert_keys:
    print(f" [告警] 当前存在：{', '.join(sorted(alert_keys))}")
    print('        UI 的橙色 eGPU 图标就是 chestnut_alert 为真时画的（sidebar.py:99-100，')
    print('        UNCOMPILED/FAILED -> chestnut_orange.png；LOADING 用的是默认图，不是橙色）')
    if any(k.endswith('UpdateFailed') for k in alert_keys):
      print(' [结论] 固件刷写失败。hardwared 检测到 product 串与 CHESTNUT_FW_VERSION 不符，')
      print('        自动跑了 3 次 flash.py 都没成功 ⇒ 设备留在旧固件上 ⇒ chestnut_present() 恒假。')
      print('        两条路：① 让 USB 链路稳定后重试刷写（先修线/供电）')
      print('                ② 跳过刷写，直接用旧固件：_chk/patch_chx_fw_match.py --apply')
    elif any(k.endswith('PcieUnavailable') for k in alert_keys):
      print(' [结论] PCIe 链路没训练 / 12V 掉了 —— 查底座电源与显卡是否插紧')
    elif any(k.endswith('UsbSlow') for k in alert_keys):
      print(' [结论] 枚举到了但速率 < 5000Mbps —— 线材问题，换 USB3.2/USB4 全功能线')
    elif any(k.endswith('NotDetected') for k in alert_keys):
      print(' [结论] 插着但枚举不到 —— 线/供电/接口')
  joined = '\n'.join(h[4] for h in hits)
  # swaglog 里也可能有 [eGPU] 记录（egpu() 同时 cloudlog），jsonl 被清掉时靠它兜底
  sw_gate = None
  for h in hits:
    msg = h[4]
    if '[eGPU]' not in msg:
      continue
    body = msg[msg.index('[eGPU]') + len('[eGPU]'):].strip()
    try:
      r = json.loads(body)
    except Exception:
      continue
    if r.get('event') == 'gate':
      sw_gate = r
  if gate_rec is None and sw_gate is not None:
    gate_rec = sw_gate
    print(' （egpu_diag.jsonl 没有 gate，以下用 swaglog 里的 [eGPU] 记录判读）')
  if not onroad:
    print(' [结论] 车不在 onroad，modeld 没启动 —— 这次重启不可能产生任何大模型日志。')
    print('        想在停车状态下验证门控，跑（不抢 eGPU）：')
    print('          /usr/local/venv/bin/python3 /data/openpilot/openpilot/_chk/diag_egpu_load.py')
    print('        想验证真实加载，只能开出去（onroad 才会拉起 modeld_tinygrad）。')
  elif gate_rec is not None:
    if str(gate_rec.get('chestnut')).lower() in ('false', '0'):
      bad = []
      if 'pkl' in gate_rec and str(gate_rec.get('pkl')) in ('None', '', 'missing'):
        bad.append('pkl(找不到模型文件)')
      if 'pkl_fmt' in gate_rec and str(gate_rec.get('pkl_fmt')) not in ('v2',):
        bad.append(f'pkl_fmt={gate_rec.get("pkl_fmt")}(要求 SPTGPLN1/v2)')
      if 'present' in gate_rec and str(gate_rec.get('present')).lower() in ('false', '0'):
        bad.append('present(chestnut_present() 为假)')
      print(f' [结论] egpu_diag 里 gate.chestnut=False')
      for b in bad or ['(字段都对不上，看下面原始行)']:
        print(f'        · 失效判据：{b}')
      print(f'        原始：{json.dumps(gate_rec, ensure_ascii=False)[:400]}')
      if any('pkl_fmt' in b for b in bad):
        print('        ⇒ pkl 格式要转换：sunnypilot/modeld_v2/convert_upstream_pkl.py <in.pkl> <out.pkl>')
      if any('present' in b for b in bad):
        print('        ⇒ 跑 _chk/patch_chx_fw_match.py --check 看 product 串，或 _chk/diag_egpu_usb.py')
    else:
      print(' [结论] gate.chestnut=True ⇒ 加载器已进入，失败点在 load_big 内部，看 load_fail 的 err')
  elif 'not a plain tinygrad model' in joined:
    print(' [结论] pkl 是 legacy/裸 pickle，不是 SPTGPLN1 容器 ⇒ modeld_v2 读不了')
  elif 'did not enumerate' in joined:
    print(' [结论] wait_for_chestnut 10s 超时 ⇒ chestnut_present() 恒假')
  elif 'Resource busy' in joined:
    print(' [结论] libusb_set_configuration EBUSY ⇒ 接口被占，重插 eGPU')
  elif hits:
    print(' [结论] 有零星 chestnut 行，但没抓到 gate / load 记录，看第 4 节原文')
  elif size is not None:
    print(' [结论] 在 onroad 且 modeld 在跑，但 egpu_diag 与 swaglog 都没有 gate 记录')
    print('        ⇒ 大概率 modeld 在 _resolve_chestnut_gate() 之前就退出了（看第 4 节末尾）')
  else:
    print(' [结论] 在 onroad 但既无 egpu_diag 也无 swaglog 记录 ⇒ modeld 可能刚崩，')
    print('        用 tmux 抓屏最快：sudo -u comma tmux capture-pane -p -t comma -S -60')
  print('=' * 78)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
