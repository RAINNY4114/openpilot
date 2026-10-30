#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
restore_sp0920xl.py —— 按 chx2025/openpilot 的 sp0920xl 分支，恢复「大模型识别与枚举」相关代码。

【上游基线】
  repo   chx2025/openpilot
  branch sp0920xl
  commit a06366ce9246962f313380e4ebe15955e720e7a5   (2026-10-26)
  实测大小：common/hardware/usb.py 3186 B
            selfdrive/modeld/helpers.py 1573 B
            sunnypilot/modeld_v2/modeld.py 23538 B
            system/hardware/hardwared.py 23504 B   ← 与设备完全一致，未改过
            system/hardware/chestnut/status.py 5112 B  ← 一致
            system/hardware/chestnut/monitoring.py 2036 B ← 一致

【为什么不是整体覆盖】
  设备上的 sunnypilot/modeld_v2/modeld.py 是 35218 B，除大模型改动外还混着车机功能：
      [AUTO_OVERTAKE_WIRING] / [AUTO_AVOIDANCE_WIRING]
      SubMaster 里多订阅了 amapNavi / radarState / radarTracks
  整体覆盖上游会把这三项一起丢掉。所以本脚本只恢复「大模型识别与枚举」相关的两处。

【改动 1】sunnypilot/modeld_v2/modeld.py 的 gate
  上游 modeld_v2/modeld.py:262
      CHESTNUT = _find_driving_pkl(get_active_bundle(chestnut=True)) is not None \
                 and (chestnut_present() or cable_connected())
  即**不把 pkl 格式塞进门禁**。
  本仓库写成了 gate = pkl is not None and fmt == 'v2' and present，
  后果是 fmt 不对时整块 `if CHESTNUT:` 被跳过 —— 而
  chestnut_usb_node_ready()（全设备唯一发 0xF3 给 PCIe 上电的地方）就在里面：
      pkl 不转换 -> 不上电 -> eGPU 不可用 -> 不能转换   死锁
  改为 gate = pkl is not None and present
  （保留 present 而不采用上游的 cable_connected()：后者插个充电器也返回 True）

【改动 2】common/hardware/usb.py 的 CHESTNUT_FW_VERSION
  上游是 "8516b768"，但本机 eGPU 固件是 "ed4e39b7"（10-21 实测）。
  这个常量有 4 个消费点，pin 对它比逐处 relax 更彻底：
      helpers.py:chestnut_present()   monitoring.py:35   status.py:36
      hardwared.py:71 的 mismatch（就是刷固件的源头）
  ⇒ pin 之后不再刷固件、chestnutState 恢复发布、firmware_ok 恢复。
  注意：helpers.py 的 [CHX_FW_ANY] relax 保留不动 —— 万一 pin 错了它还能兜住。

【用法（设备上）】
  sudo python3 /data/openpilot/openpilot/_chk/restore_sp0920xl.py --check
  sudo python3 /data/openpilot/openpilot/_chk/restore_sp0920xl.py --apply
  sudo python3 /data/openpilot/openpilot/_chk/restore_sp0920xl.py --revert
  sudo python3 /data/openpilot/openpilot/_chk/restore_sp0920xl.py --apply --no-pin

  改完必须 sudo systemctl restart comma 才生效（内存里还是旧代码）。

【注意】
  恢复 gate 只解决「死锁」。当前更前置的阻塞是 USB 枚举本身失败
  （dmesg: Device not responding to setup address），那需要先让设备上线。
"""

import argparse
import os
import shutil
import sys

DEFAULT_REPO = "/data/openpilot/openpilot"
MARK = "[RESTORE_SP0920XL 20261027]"

MODELD_REL = "sunnypilot/modeld_v2/modeld.py"
USB_REL = "common/hardware/usb.py"

GATE_OLD = "  gate = pkl is not None and fmt == 'v2' and present"

GATE_NEW = (
  "  # " + MARK + " 恢复上游 sp0920xl 的门禁语义。\n"
  "  # 上游 modeld_v2/modeld.py:262 是\n"
  "  #   _find_driving_pkl(get_active_bundle(chestnut=True)) is not None and (chestnut_present() or cable_connected())\n"
  "  # 即**不把 pkl 格式塞进门禁**。本仓库曾写成 fmt == 'v2' and present，\n"
  "  # 后果是 fmt 不对时整块 `if CHESTNUT:` 被跳过，而 chestnut_usb_node_ready()\n"
  "  # （全设备唯一发 0xF3 给 PCIe 上电的地方）就在里面：\n"
  "  #   pkl 不转换 -> 不上电 -> eGPU 不可用 -> 不能转换   死锁。\n"
  "  # 这里去掉 fmt 校验，只保留 present（不用上游的 cable_connected()：插充电器也是 True）。\n"
  "  # 格式不对时 load_big 仍会跑到 ModelState 才失败，但那时 0xF3 已经发出、链路已起。\n"
  "  # 注意：上面「提前拦住可以省掉一次重试预算」那段理由已被本改动取代 ——\n"
  "  # 代价是格式不对时多烧一次重试预算，换来链路能起来，这个交换是划算的。\n"
  "  gate = pkl is not None and present"
)

FW_OLD = 'CHESTNUT_FW_VERSION = "8516b768"'
FW_NEW = ('CHESTNUT_FW_VERSION = "ed4e39b7"  '
          '# ' + MARK + ' pin 成设备实际固件号（10-21 实测）；'
          '上游 sp0920xl 是 8516b768')

BACKUP_SUFFIX = ".bak_restore_sp0920xl"


def read(path):
  with open(path, "rb") as f:
    return f.read().decode("utf-8")


def write(path, text):
  st = os.stat(path)
  with open(path, "wb") as f:
    f.write(text.encode("utf-8"))
  try:
    os.chmod(path, st.st_mode)
    if hasattr(os, "chown"):
      os.chown(path, st.st_uid, st.st_gid)
  except (OSError, AttributeError):
    pass


def find_marker(text, marker):
  return [i + 1 for i, ln in enumerate(text.splitlines()) if marker in ln]


class Job:
  def __init__(self, name, rel, old, new, repo, optional=False):
    self.name = name
    self.rel = rel
    self.path = os.path.join(repo, rel)
    self.old = old
    self.new = new
    self.optional = optional

  def state(self):
    """返回 'already' / 'pending' / 'missing' / 'nofile'"""
    if not os.path.isfile(self.path):
      return "nofile"
    text = read(self.path)
    if self.new.splitlines()[-1].strip() and self.new.splitlines()[-1].strip() in text \
       and MARK in text:
      return "already"
    if self.old in text:
      return "pending"
    return "missing"

  def apply(self):
    text = read(self.path)
    if self.old not in text:
      raise RuntimeError("锚点未命中: %r" % self.old)
    if text.count(self.old) != 1:
      raise RuntimeError("锚点出现 %d 次，拒绝改动" % text.count(self.old))
    bak = self.path + BACKUP_SUFFIX
    if not os.path.exists(bak):
      shutil.copy2(self.path, bak)
    write(self.path, text.replace(self.old, self.new))

  def revert(self):
    bak = self.path + BACKUP_SUFFIX
    if not os.path.exists(bak):
      raise RuntimeError("没有备份 %s" % bak)
    shutil.copy2(bak, self.path)
    os.remove(bak)


def build_jobs(repo, do_pin):
  jobs = [Job("gate 恢复上游语义", MODELD_REL, GATE_OLD, GATE_NEW, repo)]
  if do_pin:
    jobs.append(Job("pin 固件版本", USB_REL, FW_OLD, FW_NEW, repo))
  return jobs


def show_state(jobs, repo):
  ok = True
  print("  仓库: %s" % repo)
  print("")
  for j in jobs:
    st = j.state()
    icon = {"already": "OK  ", "pending": "TODO", "missing": "ERR ", "nofile": "ERR "}[st]
    print("  [%s] %-22s %s" % (icon, j.name, j.rel))
    if st == "already":
      print("         已应用")
    elif st == "pending":
      print("         待应用")
    elif st == "missing":
      print("         锚点未命中 —— 文件可能被改过，先人工核对")
      ok = False
    else:
      print("         文件不存在")
      ok = False
  return ok


def py_compile(paths):
  """语法检查。编译产物写到临时目录，避免在仓库里生成 __pycache__。"""
  import py_compile as pc
  import tempfile
  for p in paths:
    try:
      with tempfile.TemporaryDirectory() as td:
        pc.compile(p, cfile=os.path.join(td, "chk.pyc"), doraise=True)
    except Exception as e:
      return False, "%s: %r" % (os.path.basename(p), e)
  return True, ""


def main():
  ap = argparse.ArgumentParser(description="按 sp0920xl 恢复大模型识别与枚举代码")
  ap.add_argument("--check", action="store_true")
  ap.add_argument("--apply", action="store_true")
  ap.add_argument("--revert", action="store_true")
  ap.add_argument("--no-pin", action="store_true", help="只恢复 gate，不改固件版本号")
  ap.add_argument("--repo", default=DEFAULT_REPO)
  args = ap.parse_args()

  if not (args.check or args.apply or args.revert):
    args.check = True

  jobs = build_jobs(args.repo, not args.no_pin)

  print("=" * 82)
  print(" sp0920xl 大模型代码恢复")
  print("=" * 82)

  if args.check:
    ok = show_state(jobs, args.repo)
    print("")
    print("  " + ("全部就绪" if ok else "有问题，见上"))
    print("=" * 82)
    return 0 if ok else 1

  if args.revert:
    for j in jobs:
      try:
        j.revert()
        print("  [REVERT] %s" % j.rel)
      except Exception as e:
        print("  [SKIP  ] %s: %s" % (j.rel, e))
    ok, err = py_compile([j.path for j in jobs if os.path.isfile(j.path)])
    print("\n  语法检查: %s" % ("通过" if ok else err))
    print("\n  记得 sudo systemctl restart comma")
    print("=" * 82)
    return 0 if ok else 1

  changed = []
  for j in jobs:
    st = j.state()
    if st == "already":
      print("  [SKIP  ] %-22s 已应用" % j.name)
      continue
    if st != "pending":
      print("  [FAIL  ] %-22s 状态=%s" % (j.name, st))
      print("=" * 82)
      return 1
    try:
      j.apply()
      changed.append(j)
      print("  [APPLY ] %-22s %s" % (j.name, j.rel))
    except Exception as e:
      print("  [FAIL  ] %-22s %s" % (j.name, e))
      print("=" * 82)
      return 1

  ok, err = py_compile([j.path for j in jobs if os.path.isfile(j.path)])
  print("\n  语法检查: %s" % ("通过" if ok else err))
  if not ok:
    print("  回滚...")
    for j in changed:
      try:
        j.revert()
      except Exception:
        pass
    print("  已回滚，未生效")
    print("=" * 82)
    return 1

  print("")
  print("  下一步：sudo systemctl restart comma")
  print("  然后确认：grep -n 'RESTORE_SP0920XL' %s" % jobs[0].path)
  print("")
  print("  注意：本脚本只解决「fmt 把 0xF3 关掉」这个死锁。")
  print("        当前 dmesg 仍是 Device not responding to setup address，")
  print("        说明 USB 枚举本身还没通 —— 那需要先让设备上线，gate 才有机会执行。")
  print("=" * 82)
  return 0


if __name__ == "__main__":
  sys.exit(main())
