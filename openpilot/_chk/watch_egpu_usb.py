#!/usr/bin/env python3
"""盯着 USB 总线，看重插 eGPU 时到底发生了什么（只读，不 import openpilot）。

为什么要它
----------
`diag_egpu_usb.py` 给的是某一瞬间的快照。但 eGPU 的问题往往是**时序性**的：
先以 12Mbps 出现 -> 被 usb-storage 抓走 -> 发 0xF3 -> 掉线 -> 重枚举到 5000Mbps。
快照只能看到某一帧，看不出是哪一步断的。

这个脚本每 0.3s 扫一次 /sys/bus/usb/devices，并且把 dmesg 的新增行也打出来，
所以「插上之后 3 秒内发生了什么」能被完整记录下来。

用法（设备上，sudo 才读得到 dmesg 和 /dev/bus/usb）
-------------------------------------------------
  sudo python3 /data/openpilot/openpilot/_chk/watch_egpu_usb.py
  sudo python3 .../watch_egpu_usb.py --seconds 180
  sudo python3 .../watch_egpu_usb.py --power      # 看到设备在 USB2 就发 0xF3 上电

输出约定
--------
  + 设备出现      - 设备消失      ~ 设备属性变化      ! 内核报错
"""
import argparse
import glob
import os
import subprocess
import sys
import time

CHESTNUT_IDS = {(0x3801, 0x0001), (0xADD1, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}
USB3_MIN = 5000

ERR_KEYS = ('Cannot enable', 'Cannot set link state', 'cannot disable', 'unable to enumerate',
            'device descriptor read', 'device not accepting address', 'reset SuperSpeed',
            'over-current', 'Cannot reset', 'error -')

FIELDS = ('idVendor', 'idProduct', 'speed', 'product', 'bConfigurationValue')


def rd(path):
  try:
    return open(path).read().strip()
  except OSError:
    return ''


def snapshot(sys_root='/sys/bus/usb/devices'):
  """{sysfs 名: {字段: 值}}，只收 chestnut / ROM / 以及 root hub 下的子设备。"""
  out = {}
  for d in sorted(glob.glob(os.path.join(sys_root, '*'))):
    name = os.path.basename(d)
    vid_s, pid_s = rd(d + '/idVendor'), rd(d + '/idProduct')
    if not vid_s or not pid_s:
      continue
    try:
      vid, pid = int(vid_s, 16), int(pid_s, 16)
    except ValueError:
      continue
    # 只关心：chestnut / ROM，以及 USB3 root hub 上挂的东西（能看出掉速）
    interesting = (vid, pid) in CHESTNUT_IDS or (vid, pid) in ROM_IDS or name.startswith('4-') or name.startswith('3-')
    if not interesting:
      continue
    out[name] = {
      'idVendor': vid_s, 'idProduct': pid_s,
      'speed': rd(d + '/speed'),
      'product': rd(d + '/product'),
      'bConfigurationValue': rd(d + '/bConfigurationValue'),
      'kind': 'CHESTNUT' if (vid, pid) in CHESTNUT_IDS else 'ROM' if (vid, pid) in ROM_IDS else '-',
    }
  return out


def describe(name, v):
  sp = v.get('speed') or '?'
  try:
    slow = ' [掉速!]' if float(sp) < USB3_MIN else ''
  except ValueError:
    slow = ''
  return (f"{name:8} {v['idVendor']}:{v['idProduct']} {v['kind']:8} "
          f"{sp:>7}Mbps{slow} cfg={v.get('bConfigurationValue') or '?'} {v.get('product')!r}")


def dmesg_all():
  try:
    p = subprocess.run(['dmesg'], capture_output=True, text=True, timeout=10)
    return p.stdout.splitlines()
  except Exception:
    return []


class DmesgTail:
  def __init__(self):
    self.last = None

  def new_lines(self):
    lines = dmesg_all()
    if not lines:
      return []
    if self.last is None:
      self.last = lines[-1] if lines else None
      return []
    try:
      idx = lines.index(self.last)
    except ValueError:
      idx = max(0, len(lines) - 12) - 1
    fresh = lines[idx + 1:]
    if fresh:
      self.last = fresh[-1]
    return fresh


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--seconds', type=int, default=120, help='监测时长（默认 120s）')
  ap.add_argument('--interval', type=float, default=0.3, help='sysfs 轮询间隔')
  ap.add_argument('--power', action='store_true', help='看到 chestnut 在 USB2 就发 0xF3 上电')
  ap.add_argument('--all-dmesg', action='store_true', help='打印 dmesg 所有新增行，不只 USB 相关')
  ap.add_argument('--sys-root', default='/sys/bus/usb/devices', help='sysfs 根（仅测试用）')
  args = ap.parse_args()

  print('=' * 78)
  print(f' eGPU 重插监测   {time.strftime("%H:%M:%S")}   时长 {args.seconds}s')
  print(' 现在去插/重插 eGPU（先确认底座 12V 已通电）')
  print('=' * 78)

  dt = DmesgTail()
  dt.new_lines()  # 吃掉历史
  prev = snapshot(args.sys_root)
  if prev:
    print('\n起始状态：')
    for k, v in prev.items():
      print('  ' + describe(k, v))
  else:
    print('\n起始状态：USB3/USB2 端口上没有任何设备')

  max_speed = 0.0
  seen_chestnut = False
  seen_rom = False
  powered = False
  t0 = time.monotonic()
  last_err = 0.0

  try:
    while time.monotonic() - t0 < args.seconds:
      time.sleep(args.interval)
      cur = snapshot(args.sys_root)

      for name in sorted(set(cur) - set(prev)):
        v = cur[name]
        print(f'  {time.strftime("%H:%M:%S")} + {describe(name, v)}')
      for name in sorted(set(prev) - set(cur)):
        print(f'  {time.strftime("%H:%M:%S")} - {name} 消失')
      for name in sorted(set(cur) & set(prev)):
        if cur[name] != prev[name]:
          chg = {k: (prev[name].get(k), cur[name].get(k)) for k in FIELDS
                 if prev[name].get(k) != cur[name].get(k)}
          print(f'  {time.strftime("%H:%M:%S")} ~ {name} 变化 {chg}')

      for name, v in cur.items():
        try:
          max_speed = max(max_speed, float(v['speed']))
        except (ValueError, KeyError):
          pass
        if v['kind'] == 'CHESTNUT':
          seen_chestnut = True
        if v['kind'] == 'ROM':
          seen_rom = True

      # 可选：主动上电
      if args.power and not powered and any(v['kind'] == 'CHESTNUT' and v['speed'] and
                                           float(v['speed'] or 0) < USB3_MIN for v in cur.values()):
        powered = True
        print(f'  {time.strftime("%H:%M:%S")} ! 设备在 USB2 侧，发 0xF3 请求 PCIe 上电')
        try:
          sys.path.insert(0, '/data/openpilot/openpilot')
          sys.path.insert(0, '/data/openpilot')
          from openpilot.common.hardware.usb import chestnut_pre_power, pre_power_enabled
          if not pre_power_enabled():
            print('           [WARN] dp_chestnut_pre_power=0，已禁用，跳过')
          else:
            for name, v in cur.items():
              if v['kind'] == 'CHESTNUT':
                ok = chestnut_pre_power(int(v['idVendor'], 16), int(v['idProduct'], 16))
                print(f'           chestnut_pre_power() = {ok}')
                break
        except Exception as e:
          print(f'           [WARN] 上电调用失败：{type(e).__name__}: {e}')

      # 内核报错
      now = time.monotonic()
      for line in dt.new_lines():
        if args.all_dmesg or any(k in line for k in ERR_KEYS):
          if now - last_err > 0.5:  # 别让热循环刷屏
            print(f'  {time.strftime("%H:%M:%S")} ! {line.strip()[:150]}')
            last_err = now

      prev = cur
  except KeyboardInterrupt:
    print('\n  (手动中断)')

  # ---- 判读
  print('\n' + '=' * 78)
  print(f' 结果：最高速率 {max_speed:.0f}Mbps  chestnut={"见过" if seen_chestnut else "没见过"} '
        f'ROM={"见过" if seen_rom else "没见过"}')
  if seen_chestnut and max_speed >= USB3_MIN:
    print(' [OK] 设备上过 USB3 侧 ⇒ 链路能训练成功。下一步看 product 串是否等于期望值，')
    print('      然后跑 egpu_smoke.py --run 验证真能算。')
  elif seen_chestnut or seen_rom:
    print(' [FAIL] 设备出现过，但只到 USB2（或 ROM 模式），USB3 链路始终没训练起来。')
    print('        · 12/480Mbps 一直是这个数 -> 线材缺 SuperSpeed 差分对，换 USB3.2/USB4 全功能线')
    print('        · 底座 12V 没插 -> ASM2464 供电不足，插上适配器')
    print('        · 发过 0xF3 也没回到 5000 -> 加 --power 再试一次，看是不是上电命令没生效')
  else:
    print(' [FAIL] 整个监测窗口内总线上没出现过 chestnut / ASM2464。')
    print('        · dmesg 有 "Cannot enable / Cannot set link state" -> 电气层就不通：换线、查供电')
    print('        · dmesg 干净无报错 -> 线没插到、或插的是充电口/从模式口')
    print('        · 反复出现 over-current -> 底座或显卡短路保护，拔掉重来')
  print('=' * 78)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
