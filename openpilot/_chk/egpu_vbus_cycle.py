#!/usr/bin/env python3
"""用软件给 eGPU 底座断一次电（等价于拔插），然后盯重枚举。

为什么用这个而不是让用户拔线
----------------------------
`system/hardware/chestnut/flash.py:24` 里有现成的软件 VBUS 控制：
    VBUS_PATH = /sys/kernel/debug/regulator/smb2-vbus/enable
    def vbus_cycle(): vbus_write("0"); sleep(2); vbus_write("1"); sleep(5)
`flash.py` 自己就靠它在 ROM 恢复流程里强制重枚举。10-21 那次「链路卡死」的恢复方法
（设备+盒子一起冷启动 ≥30s）本质上也是同一件事，只是要动手。

现在的症状是 `usb4-port1: Cannot enable. Maybe the USB cable is bad?` 热循环，
端口连设备节点都建不出来 ⇒ 物理拔插之前，先试一次 VBUS 断电，成本低且可重复。

用法（设备上，必须 sudo，且必须 offroad）
----------------------------------------
  sudo python3 /data/openpilot/openpilot/_chk/egpu_vbus_cycle.py
  sudo python3 .../egpu_vbus_cycle.py --apply
  sudo python3 .../egpu_vbus_cycle.py --apply --watch 40
"""
import argparse
import glob
import os
import subprocess
import time

VBUS_PATH = '/sys/kernel/debug/regulator/smb2-vbus/enable'
MODE_PATH = '/sys/devices/platform/soc/a600000.ssusb/mode'
CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}


def rd(p, dflt='?'):
  try:
    return open(p).read().strip()
  except OSError:
    return dflt


def debugfs_mounted():
  try:
    for line in open('/proc/mounts'):
      if ' /sys/kernel/debug ' in line and 'debugfs' in line:
        return True
  except OSError:
    pass
  return False


def devices():
  out = {}
  for d in sorted(glob.glob('/sys/bus/usb/devices/*')):
    name = os.path.basename(d)
    try:
      vid = int(rd(d + '/idVendor', ''), 16)
      pid = int(rd(d + '/idProduct', ''), 16)
    except ValueError:
      continue
    if (vid, pid) not in CHESTNUT_IDS and (vid, pid) not in ROM_IDS:
      continue
    out[name] = {
      'id': f'{vid:04x}:{pid:04x}',
      'kind': 'CHESTNUT' if (vid, pid) in CHESTNUT_IDS else 'ROM',
      'speed': rd(d + '/speed'),
      'product': rd(d + '/product', ''),
    }
  return out


def show(tag, devs):
  if not devs:
    print(f'  {tag} (总线上没有 chestnut / ASM2464)')
    return
  for k, v in devs.items():
    print(f"  {tag} {k:8} {v['id']} {v['kind']:8} {v['speed']:>7}Mbps {v['product']!r}")


def dmesg_new(last):
  try:
    p = subprocess.run(['dmesg'], capture_output=True, text=True, timeout=10)
    lines = p.stdout.splitlines()
  except Exception:
    return last, []
  if last is None:
    return (lines[-1] if lines else None), []
  try:
    i = lines.index(last)
  except ValueError:
    i = max(0, len(lines) - 8) - 1
  fresh = lines[i + 1:]
  return (fresh[-1] if fresh else last), fresh


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--apply', action='store_true', help='真的断电循环（默认只看状态）')
  ap.add_argument('--watch', type=int, default=25, help='断电后观察多少秒')
  ap.add_argument('--off', type=float, default=2.0, help='断电保持秒数')
  ap.add_argument('--on', type=float, default=5.0, help='上电后先等多少秒')
  ap.add_argument('--vbus', default=VBUS_PATH, help='（仅测试用）VBUS 控制路径')
  args = ap.parse_args()

  print('=' * 78)
  print(f' eGPU VBUS 断电循环   {time.strftime("%H:%M:%S")}')
  print('=' * 78)

  procs = []
  try:
    for name in os.listdir('/proc'):
      if name.isdigit():
        cmd = open(f'/proc/{name}/cmdline', 'rb').read().replace(b'\x00', b' ').decode(errors='replace')
        if 'modeld' in cmd and 'egpu_vbus' not in cmd:
          procs.append((name, cmd.strip()[:70]))
  except OSError:
    pass
  if procs:
    for pid, cmd in procs:
      print(f'  [FAIL] modeld 在跑 pid={pid} {cmd}')
    print('  ⇒ 会抢 eGPU，先停车/熄火再跑')
    return 1
  print('  [OK  ] 没有 modeld 在跑')

  print(f'\n  a600000.ssusb/mode = {rd(MODE_PATH)}   （要 host）')
  print(f'  debugfs 挂载        = {debugfs_mounted()}')
  print(f'  VBUS 控制           = {args.vbus}  存在={os.path.exists(args.vbus)}'
        + (f'  当前={rd(args.vbus)}' if os.path.exists(args.vbus) else ''))

  print('\n  当前枚举：')
  show('·', devices())

  if not os.path.exists(args.vbus):
    print('\n  [FAIL] 没有 VBUS 控制节点，软件断电这条路走不通。')
    if not debugfs_mounted():
      print('         先试：sudo mount -t debugfs none /sys/kernel/debug')
    print('         退回物理办法：设备 + 底座一起断电 ≥30s（10-15/10-21 的结论）')
    return 1

  if not args.apply:
    print('\n（只看了状态。加 --apply 执行 断电 %.1fs → 上电 → 观察 %ds）' % (args.off, args.watch))
    return 0

  print(f'\n  → 断电 {args.off:.1f}s')
  try:
    with open(args.vbus, 'w') as f:
      f.write('0')
  except OSError as e:
    print(f'  [FAIL] 写 0 失败：{e}  ⇒ 需要 sudo')
    return 1
  time.sleep(args.off)

  print('  → 上电')
  try:
    with open(args.vbus, 'w') as f:
      f.write('1')
  except OSError as e:
    print(f'  [FAIL] 写 1 失败：{e}')
    return 1
  time.sleep(args.on)

  print(f'\n  观察 {args.watch}s（只看变化）')
  last_dmesg = None
  last_dmesg, _ = dmesg_new(last_dmesg)
  prev = devices()
  show('起始', prev)
  t0 = time.monotonic()
  max_speed = 0.0
  saw_chestnut = False
  while time.monotonic() - t0 < args.watch:
    time.sleep(0.4)
    cur = devices()
    for n in sorted(set(cur) - set(prev)):
      v = cur[n]
      print(f"  {time.strftime('%H:%M:%S')} + {n:8} {v['id']} {v['kind']:8} "
            f"{v['speed']:>7}Mbps {v['product']!r}")
    for n in sorted(set(prev) - set(cur)):
      print(f"  {time.strftime('%H:%M:%S')} - {n} 消失")
    for n in sorted(set(cur) & set(prev)):
      if cur[n] != prev[n]:
        print(f"  {time.strftime('%H:%M:%S')} ~ {n} {prev[n]} -> {cur[n]}")
    for n, v in cur.items():
      try:
        max_speed = max(max_speed, float(v['speed']))
      except ValueError:
        pass
      if v['kind'] == 'CHESTNUT':
        saw_chestnut = True
    last_dmesg, fresh = dmesg_new(last_dmesg)
    for line in fresh:
      if any(k in line for k in ('usb', 'xhci', 'Cannot', 'link state', 'SuperSpeed')):
        print(f"  {time.strftime('%H:%M:%S')} ! {line.strip()[:140]}")
    prev = cur

  print('\n' + '=' * 78)
  if saw_chestnut and max_speed >= 5000:
    print(f' [OK] 断电循环后回到 USB3（{max_speed:.0f}Mbps）⇒ 链路能训练，接下来看 product 串')
  elif prev:
    print(f' [WARN] 设备出现了但最高只有 {max_speed:.0f}Mbps —— 断电没解决，换线/查 12V')
  else:
    print(' [FAIL] 断电循环后仍然一个设备都没出现。')
    print('        · 端口仍在 Cannot enable -> 电气层：线材缺 SuperSpeed 对 / 底座 12V 没接 / 接口坏')
    print('        · 只能物理处置：设备与底座一起断电 ≥30s，再换一根 USB3.2/USB4 全功能线')
  print('=' * 78)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
