#!/usr/bin/env python3
"""只读诊断：eGPU（chestnut）为什么没有被识别 / 没有显示。

不 import openpilot，纯读 /sys 与 /dev ⇒ openpilot 没起来、modeld 崩了也能跑。
系统 python3 就够（不需要 venv 里的 numpy）：

  python3 /data/openpilot/openpilot/_chk/diag_egpu_usb.py
  python3 .../diag_egpu_usb.py --dmesg 40      # 多看几条 dmesg

判据来源（与上游 staging-chestnut 一致）：
  CHESTNUT_USB_IDS     = (0xADD1,0x0001), (0x3801,0x0001)
  CHESTNUT_ROM_USB_IDS = (0x174C,0x2464), (0x174C,0x2463)   # ASM2464 ROM bootloader
  CHESTNUT_USB_PRODUCT = "custom 8516b768-CLEAN"
  USB3_MIN_SPEED       = 5000 (Mbps)
  LTSSM_TRAINED        = 0x78
"""
import argparse
import glob
import os
import subprocess
import sys
from pathlib import Path

USB_ROOT = Path('/sys/bus/usb/devices')
CHESTNUT_IDS = {(0xADD1, 0x0001), (0x3801, 0x0001)}
ROM_IDS = {(0x174C, 0x2464), (0x174C, 0x2463)}
EXPECT_PRODUCT = 'custom 8516b768-CLEAN'
USB3_MIN_SPEED = 5000
LTSSM_TRAINED = 0x78
PERM_LOG = '/data/egpu_usb_perm.log'

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


def rd(path, default=''):
  try:
    return Path(path).read_text(errors='replace').strip()
  except OSError:
    return default


def rdi(path, base=10, default=None):
  v = rd(path)
  if not v:
    return default
  try:
    return int(v, base)
  except ValueError:
    return default


def list_usb():
  """返回所有真实 USB 设备（跳过 1-1:1.0 这类接口节点）。"""
  out = []
  if not USB_ROOT.is_dir():
    return out
  for name in sorted(os.listdir(USB_ROOT)):
    if ':' in name:
      continue
    d = USB_ROOT / name
    vid = rdi(d / 'idVendor', 16)
    pid = rdi(d / 'idProduct', 16)
    if vid is None or pid is None:
      continue
    drivers = []
    try:
      for ifname in sorted(os.listdir(d)):
        if ':' not in ifname:
          continue
        drv = d / ifname / 'driver'
        try:
          drivers.append(f'{ifname}->{os.path.basename(os.readlink(drv))}')
        except OSError:
          drivers.append(f'{ifname}->(未绑定)')
    except OSError:
      pass
    out.append({
      'name': name,
      'vid': vid,
      'pid': pid,
      'product': rd(d / 'product'),
      'manufacturer': rd(d / 'manufacturer'),
      'serial': rd(d / 'serial'),
      'speed': rdi(d / 'speed', default=-1),
      'busnum': rdi(d / 'busnum'),
      'devnum': rdi(d / 'devnum'),
      'version': rd(d / 'version'),
      'maxpower': rd(d / 'bMaxPower'),
      'config': rd(d / 'bConfigurationValue'),
      'authorized': rd(d / 'authorized'),
      'num_if': rdi(d / 'bNumInterfaces'),
      'drivers': drivers,
      'sysfs': str(d),
    })
  return out


def sec_controller():
  print('\n===== 1. USB 控制器模式 =====')
  p = Path('/sys/devices/platform/soc/a600000.ssusb/mode')
  mode = rd(p, '(读不到)')
  tag = 'OK' if mode == 'host' else 'FAIL'
  line(tag, f'a600000.ssusb/mode = {mode}', 'host 才能识别外接设备；none 表示主控没切到 host 模式')
  cc = rd('/sys/class/power_supply/usb/typec_cc_orientation', '(读不到)')
  line('WARN', f'typec_cc_orientation = {cc}', '只反映线缆方向，插充电器也是非空，不能当 eGPU 就绪')


def sec_devices(devs):
  print('\n===== 2. USB 总线上的全部设备 =====')
  if not devs:
    line('FAIL', '一个 USB 设备都没读到（/sys/bus/usb/devices 为空）')
    return
  print(f'  {"sysfs":14} {"vid:pid":11} {"速":>6}  {"bus/dev":>9}  {"product":30} 驱动')
  for d in devs:
    mark = ''
    if (d['vid'], d['pid']) in CHESTNUT_IDS:
      mark = '  <== CHESTNUT'
    elif (d['vid'], d['pid']) in ROM_IDS:
      mark = '  <== ASM2464 ROM(未加载固件)'
    prod = (d['product'] or '(无)')[:30]
    print(f'  {d["name"]:14} {d["vid"]:04x}:{d["pid"]:04x} {d["speed"]:>6}  '
          f'{d["busnum"]:>3}/{d["devnum"]:<3}  {prod:30} {",".join(d["drivers"])}{mark}')


def sec_verdict(devs):
  print('\n===== 3. 判定 =====')
  chest = [d for d in devs if (d['vid'], d['pid']) in CHESTNUT_IDS]
  rom = [d for d in devs if (d['vid'], d['pid']) in ROM_IDS]

  if not chest and not rom:
    line('FAIL', '总线上没有 chestnut / ASM2464 设备',
         'eGPU 完全没枚举 —— 查 USB-C 线、底座 12V 供电、插头是否插到底')
    print('\n  ⇒ 这一层就断了，跟 pkl 格式、bundle、门控全都无关。')
    print('    先让设备出现在总线里，再看后面几步。')
    return

  if rom and not chest:
    line('FAIL', f'只有 ROM bootloader（{rom[0]["vid"]:04x}:{rom[0]["pid"]:04x}）',
         'ASM2464 固件没加载起来；chestnut_present() 只认 0xADD1/0x3801，这里必然 False')
    print('\n  ⇒ 需要让底座重新加载固件：断电重插，或换一根支持 USB3 的线。')
    return

  d = chest[0]
  line('OK', f'找到 chestnut：{d["vid"]:04x}:{d["pid"]:04x} bus={d["busnum"]:03d}/{d["devnum"]:03d}',
       f'sysfs={d["sysfs"]}')

  # product 串
  prod = d['product']
  if prod == EXPECT_PRODUCT:
    line('OK', f'product 串匹配 {prod!r}')
  else:
    line('FAIL', f'product 串不匹配：实际 {prod!r}', f'期望 {EXPECT_PRODUCT!r}')
    print('         chestnut_present() 要求完全相等 ⇒ 这里不匹配就是「没显示 eGPU」的直接原因。')
    print('         串里的 8 位十六进制是 ASM2464 固件版本号。实际值不同 = 固件不是这个版本，')
    print('         要么升级 eGPU 固件，要么把代码里的 CHESTNUT_FW_VERSION 改成实际值。')

  # 速率 / USB3
  sp = d['speed']
  if sp >= USB3_MIN_SPEED:
    line('OK', f'链路速率 {sp} Mbps（USB3 侧）')
  else:
    line('FAIL', f'链路速率 {sp} Mbps < {USB3_MIN_SPEED}（在 USB2 侧）',
         '大模型要 PCIe over USB3；USB2 侧带不动 762 MB 模型')

  # 配置状态
  if d['config'] in ('1', '2', '3'):
    line('OK', f'bConfigurationValue = {d["config"]}（已配置）')
  else:
    line('FAIL', f'bConfigurationValue = {d["config"] or "(空)"}（未配置）',
         '未配置时 libusb_set_configuration 容易报 Resource busy')

  # 接口驱动占用（EBUSY 的真凶）
  bound = [x for x in d['drivers'] if '(未绑定)' not in x]
  if bound:
    line('WARN', f'{len(bound)} 个接口已绑定驱动', ', '.join(bound))
    print('         10-21 定案：任一接口有驱动绑定，内核 devio.c:proc_setconfig 就返回 -EBUSY。')
    print('         egpu_usb_perm.sh 会解绑非 usbfs 的驱动，但 usbfs 被显式跳过。')
  else:
    line('OK', '所有接口都未绑定驱动（tinygrad 可以独占）')

  # 节点权限
  node = f'/dev/bus/usb/{d["busnum"]:03d}/{d["devnum"]:03d}'
  if os.path.exists(node):
    try:
      os.close(os.open(node, os.O_RDWR))
      line('OK', f'{node} 可读写')
    except OSError as e:
      line('FAIL', f'{node} 打不开：{e}', '权限问题，由 system/egpu_usb_perm.sh 负责放开')
  else:
    line('FAIL', f'{node} 不存在')

  # LTSSM
  ltssm = None
  try:
    import usb1
    with usb1.USBContext() as ctx:
      h = ctx.openByVendorIDAndProductID(d['vid'], d['pid'], skip_on_error=True)
      if h is not None:
        try:
          ltssm = bytes(h.controlRead(0xC0, 0xE4, 0x03, 0, 1, timeout=1000))[0]
        finally:
          h.close()
  except Exception as e:
    line('WARN', 'LTSSM 读不到', repr(e))
  if ltssm is not None:
    tag = 'OK' if ltssm == LTSSM_TRAINED else 'FAIL'
    line(tag, f'PCIe LTSSM = 0x{ltssm:02X}' + ('（链路已训练）' if ltssm == LTSSM_TRAINED else ''),
         '不是 0x78 说明 PCIe 侧没训练起来（10-23 定案：也可能是过期快照，需配合枚举日志看）')


def sec_perm_log(n):
  print(f'\n===== 4. {PERM_LOG} =====')
  if not os.path.exists(PERM_LOG):
    line('WARN', '文件不存在', 'egpu_usb_perm.sh 可能没在跑')
    return
  try:
    with open(PERM_LOG, errors='replace') as f:
      rows = f.readlines()
  except OSError as e:
    line('FAIL', '读取失败', repr(e))
    return
  unbind = sum(1 for r in rows if 'unbind' in r)
  renum = sum(1 for r in rows if 'was: 600 root:root' in r)
  line('OK' if renum <= 2 else 'WARN', f'{len(rows)} 行，其中 unbind {unbind} 次、重枚举 {renum} 次',
       '重枚举次数多 = 枚举风暴，加载必然 EBUSY' if renum > 2 else '')
  for r in rows[-n:]:
    print('    ' + r.rstrip()[:190])


def sec_dmesg(n):
  print(f'\n===== 5. dmesg 里的 USB 记录（最近 {n} 条）=====')
  try:
    r = subprocess.run(['dmesg'], capture_output=True, text=True, timeout=20)
    if r.returncode != 0:
      line('WARN', f'dmesg 返回 {r.returncode}', (r.stderr or '').strip()[:120])
      return
    rows = [x for x in r.stdout.splitlines()
            if any(k in x.lower() for k in ('usb', 'asm24', 'superspeed', 'xhci'))]
    if not rows:
      line('WARN', '没有 USB 相关 dmesg')
      return
    line('OK', f'命中 {len(rows)} 条')
    for x in rows[-n:]:
      print('    ' + x.strip()[:190])
  except Exception as e:
    line('WARN', 'dmesg 不可用', repr(e))


def main():
  global USB_ROOT
  ap = argparse.ArgumentParser()
  ap.add_argument('--dmesg', type=int, default=20)
  ap.add_argument('--perm-log', type=int, default=10)
  ap.add_argument('--root', default=str(USB_ROOT), help='sysfs 根（仅测试用）')
  args = ap.parse_args()

  USB_ROOT = Path(args.root)

  print('=' * 78)
  print(' eGPU / chestnut USB 层只读诊断（不依赖 openpilot）')
  try:
    up = Path('/proc/uptime').read_text().split()[0]
  except OSError:
    up = '?'
  print(f' {up}s uptime   expect product = {EXPECT_PRODUCT!r}')
  print('=' * 78)

  sec_controller()
  devs = list_usb()
  sec_devices(devs)
  sec_verdict(devs)
  sec_perm_log(args.perm_log)
  sec_dmesg(args.dmesg)

  print('\n' + '=' * 78)
  print(f' 汇总：OK {ok} / WARN {warn} / FAIL {fail}')
  print('=' * 78)
  return 0 if fail == 0 else 1


if __name__ == '__main__':
  sys.exit(main())
