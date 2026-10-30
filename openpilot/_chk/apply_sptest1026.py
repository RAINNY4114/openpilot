#!/usr/bin/env python3
"""按 sptest1026 分支移植 eGPU(chestnut) 修复到本机 staging-chestnut。

背景（来自 sptest1026 的 hardwared.py 原始注释）：
  2026-10-26 14:26，hardwared 在 offroad 时把**官方 chestnut 固件**写进了本机的
  第三方 ASM2464 底座（returncode=0、镜像校验通过），**刷完底座就不再枚举**。
  这才是「绿->橙->消失」的真正根因；此前关于 12V 欠压 / 线材 / VBUS 碎片的推断
  都只是次要因素。

本脚本移植 sptest1026 的 7 处改动，全部是「防止再次刷写 + 按真实条件识别设备」，
不包含任何会改变控制行为的改动：

  P0  hardwared.py   flash() 无条件 return + update() 直接 mismatch=False   ★砖机根因
  P1  flash.py       flash_chestnut() 加 CHESTNUT_FLASH_UNLOCK 解锁门        ★双保险
  P2  helpers.py     chestnut_present() 只按 VID/PID + wait_for_chestnut(60)
  P3  usb.py         新增 _extcon_usb_attached()，cable_connected() 改用 extcon
  P4  status.py      三方底座无 INA231，powered/power_lost 不误报
  P5  modeld.py      gate 去掉 fmt 校验(解除 0xF3 死锁) + BIG_MODEL_TIMEOUT=600
                     + AM_POWER_LIMIT 缺省 100
  P6  system/egpu_usb_perm.sh   建 0666 权限守护脚本（可选）

刻意**不**做的事：
  * 不整文件覆盖 modeld.py。设备版 35218B 含 egpu_log / 0xF3 上电守卫 / ao_aa_bridge
    接线，上游版 25517B 没有。只替换 gate 与超时，保留诊断与上电能力。
  * 不整文件覆盖 usb.py。设备版尾部有 [CHX_USB_ENUM] 枚举守卫 200 行（含 0xF3 上电），
    上游版没有。只插入 extcon 函数并改 cable_connected()。
  * 不动 launch_env.sh（AM_POWER_LIMIT 已由 modeld.py setdefault 覆盖；
    SKIP_TINYGRAD_COMPILE / TINYGRAD_CACHE_DIR 若与现有缓存不匹配会让大模型
    直接加载失败，风险大于收益）。

用法（在设备上，需要 root）：
  sudo /usr/local/venv/bin/python3 /data/tmp/apply_sptest1026.py --check
  sudo /usr/local/venv/bin/python3 /data/tmp/apply_sptest1026.py --apply
  sudo /usr/local/venv/bin/python3 /data/tmp/apply_sptest1026.py --revert
  sudo /usr/local/venv/bin/python3 /data/tmp/apply_sptest1026.py --status

--apply 之后必须重启进程才生效：
  sudo systemctl restart comma          # 或整机重启
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import time

DEFAULT_REPO = "/data/openpilot/openpilot"
DEFAULT_BACKUP = "/data/tmp/sptest1026_backup"

# 可在运行时通过 --backup-root 覆盖。之所以做成可变量而不是直接写常量：
# 在 Windows 上跑仿真时，"/data/tmp/..." 会被解析成 "C:\\data\\tmp\\..."，
# 在用户机器上凭空建目录（2026-10-27 实际踩过一次）。
_BACKUP_ROOT = DEFAULT_BACKUP

# ---------------------------------------------------------------------------
# 补丁定义
#
# 每项: (probe, old, new)
#   probe  新内容里的独特片段，用于判定「已打过」（幂等）
#   old    设备上必须**恰好出现一次**的原文
#   new    替换后的内容
# ---------------------------------------------------------------------------

P0_HARDWARED = [
    (
        "chestnut flash SKIPPED: third-party dock protection",
        '''  def flash(self) -> None:
    ret = subprocess.run(["sudo", sys.executable, os.path.join(BASEDIR, "openpilot/system/hardware/chestnut/flash.py"), CHESTNUT_FW_VERSION],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    cloudlog.event("chestnut flash done", returncode=ret.returncode, output=ret.stdout[-1000:], error=ret.returncode != 0)
    self.flashed = ret.returncode == 0
''',
        '''  def flash(self) -> None:
    # [flash-lock] 双保险：第三方底座绝不自动刷官方固件。即使 update() 短路被
    # 意外回退，这里也无条件拒绝，杜绝任何自动刷写路径。
    cloudlog.warning("chestnut flash SKIPPED: third-party dock protection (flash disabled)")
    self.flashed = False
    return
''',
    ),
    (
        "self.mismatch = False\n    self.flashed = False\n    return\n    if not self.mismatch:",
        '''  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    self.mismatch = any(is_chestnut_usb_id(d["vendorId"], d["productId"], include_bootloader=True) and
                        d["product"] != CHESTNUT_USB_PRODUCT for d in usb_state)
    if not self.mismatch:
''',
        '''  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    # [port-egpu-usb] 底座是第三方 ASM2464：product 串永远不会等于 comma 的
    # 官方版本号（实测 custom ed4e39b7-CLEAN）。原来这里会在 offroad 时把官方固件
    # 写进第三方底座（2026-10-26 14:26 实际发生：returncode=0、镜像校验通过），
    # 刷完底座就不再枚举。而且判据带 include_bootloader=True，ROM 模式
    # (174c:2464) 也会被判成 mismatch 再刷一次，恢复时会被反复打断。
    # 官方固件只适用于官方底座：这里一律不自动刷，需要时手动跑
    #   sudo python3 openpilot/system/hardware/chestnut/flash.py <target-hash>
    self.mismatch = False
    self.flashed = False
    return
    if not self.mismatch:
''',
    ),
    (
        "is_unsupported_combo = False",
        '''    is_unsupported_combo = COMMA_HARDWARE and HARDWARE.get_device_type() == "tici" and build_metadata.channel_type != "tici"
''',
        '''    is_unsupported_combo = False  # [port-egpu-usb] 去 tici 限制
''',
    ),
]

P1_FLASH = [
    (
        "CHESTNUT_FLASH_UNLOCK",
        '''def flash_chestnut(expected_version=None, force=False):
  global _deadline

  image = FIRMWARE_PATH.read_bytes()
''',
        '''def flash_chestnut(expected_version=None, force=False):
  global _deadline

  # [flash-lock] 彻底锁死：本设备底座是第三方 ASM2464，绝不自动刷官方固件。
  # 2026-10-26 hardwared 误刷官方固件致第三方底座砖机；此后即便 --force 也必须
  # 显式设置环境变量 CHESTNUT_FLASH_UNLOCK=<magic> 才允许，杜绝任何自动/半自动路径。
  _unlock = os.environ.get("CHESTNUT_FLASH_UNLOCK", "")
  if _unlock != "ALLOW-THIRD-PARTY-DOCK-FLASH":
    raise RuntimeError("chestnut flashing is DISABLED on this device (third-party dock). "
                       "Refusing to flash. To override, set CHESTNUT_FLASH_UNLOCK explicitly.")

  image = FIRMWARE_PATH.read_bytes()
''',
    ),
]

P2_HELPERS = [
    (
        "if is_chestnut_usb_id(*usb_id):",
        '''      product = (d / "product").read_text().strip()
      # ---- [CHX_FW_ANY 20261027] ----
      # 上游是 product == CHESTNUT_USB_PRODUCT（完全相等），而 CHESTNUT_FW_VERSION
      # 在 ford-chestnut 是 ed4e39b7、staging-chestnut 是 8516b768。本机 eGPU 固件是
      # ed4e39b7（10-21 实测，当时大模型能加载），换分支后就被判成「不存在」。
      # 固件构建号不该决定硬件在不在，这里放宽成「custom 前缀 + -CLEAN 后缀」，
      # 仍然排除非官方固件。
      if is_chestnut_usb_id(*usb_id) and product.startswith("custom ") and product.endswith("-CLEAN"):
''',
        '''      # [port-egpu-usb] 只按 VID/PID(3801:0001 / ADD1:0001) 识别。
      # 原实现还要求 product == "custom 8516b768-CLEAN"（官方固件 hash），
      # 第三方 ASM2464 底座报的是别的 hash（实测 custom ed4e39b7-CLEAN）
      # -> 永远 False -> wait_for_chestnut 必超时 -> 大模型永远不加载。
      if is_chestnut_usb_id(*usb_id):
''',
    ),
    (
        "def wait_for_chestnut(timeout: float = 60.)",
        '''def wait_for_chestnut(timeout: float = 10.) -> None:
''',
        '''def wait_for_chestnut(timeout: float = 60.) -> None:  # [port-egpu-usb] 第三方底座描述符响应慢
''',
    ),
]

P3_USB = [
    (
        "_extcon_usb_attached",
        '''def cable_connected() -> bool:
  return read_int(TYPEC_CC_ORIENTATION_PATH) != 0
''',
        '''def _extcon_usb_attached() -> bool | None:
  """qcom extcon 里当前的 USB 连接位。

  多个 extcon 都带 USB 属性（usb-pdphy / SMB2 / eud），只要有一条是 1 就算连上；
  全是 0 表示当前没有插任何 USB 设备。读不到 extcon 时返回 None 让调用方兜底。
  """
  found = False
  try:
    states = list(Path('/sys/class/extcon').glob('*/state'))
  except OSError:
    return None
  for state in states:
    try:
      txt = state.read_text()
    except OSError:
      continue
    for line in txt.splitlines():
      key, _, val = line.partition('=')
      if key.strip() == 'USB':
        found = True
        if val.strip() == '1':
          return True
  return False if found else None


def cable_connected() -> bool:
  # [port-egpu-usb] TYPEC_CC_ORIENTATION 是粘滞值：线拔掉后内核不清零
  # （实测拔掉底座后仍是 2）。只看它会让 CHESTNUT 分支永远成立 -> 每次 onroad 都要
  # 白等 wait_for_chestnut() 超时，然后置 ChestnutRetry=1，下次也不再自动重试。
  # 优先用 extcon 的 USB 连接位判断「现在到底插没插」。
  attached = _extcon_usb_attached()
  if attached is not None:
    return attached
  return read_int(TYPEC_CC_ORIENTATION_PATH) != 0
''',
    ),
]

P4_STATUS = [
    (
        "powered = True",
        '''      powered = state.supplyVoltage >= CHESTNUT_POWERED_VOLTAGE
      power_lost = state.supplyFault or not powered
''',
        '''      # [port-egpu-usb] 三方底座无 INA231 供电遥测，不因 supplyVoltage/supplyFault 误报供电断开
      powered = True
      power_lost = False
''',
    ),
]

P5_MODELD = [
    (
        "os.environ.setdefault('AM_POWER_LIMIT'",
        """os.environ['GMMU'] = '0'
""",
        """os.environ['GMMU'] = '0'
os.environ.setdefault('AM_POWER_LIMIT', '100')  # [port-egpu-usb] chestnut eGPU PPT 上限，降低峰值电流
""",
    ),
    (
        "BIG_MODEL_TIMEOUT = 600",
        """BIG_MODEL_TIMEOUT = 150
""",
        """BIG_MODEL_TIMEOUT = 600  # [port-egpu-usb] 上游值：762MB pkl 冷启动 + 重枚举余量
""",
    ),
    (
        "gate = pkl is not None and present",
        """  # 另外加了 pkl 格式校验：modeld_v2 只认 SPTGPLN1 容器，老 pickle 会在 load_model()
  # 里抛 ValueError，提前拦住可以省掉一次重试预算和一次 onroad cycle。
  gate = pkl is not None and fmt == 'v2' and present
""",
        """  # [port-egpu-usb] fmt 校验已从门禁里移除。
  # 原写法把「格式不对就别浪费重试预算」和「PCIe 上电」写成了同一个条件：
  # fmt 不是 SPTGPLN1 时 gate=False -> load_big() 线程根本不创建 ->
  # chestnut_usb_node_ready() 永远不执行 -> 0xF3 永远不发 -> 底座停在 12Mbps ->
  # 格式永远修不好。完整死锁。
  # 实测 /data/egpu_diag.jsonl：CHX_USB_ENUM 0 次 / pre-power 0 次 / load_start 0 次，
  # 而 gate_pkl_bad_fmt 5 次 —— 诊断清楚知道格式不对，然后把自己关掉了。
  # 格式问题由 convert_upstream_pkl.py 单独解决，仍记录在 egpu('gate', pkl_fmt=...)
  # 与 gate_pkl_bad_fmt 事件里，只是不再充当硬件门禁。
  gate = pkl is not None and present
""",
    ),
]

# 文件相对 inner repo 的路径
FILES = {
    "P0": ("system/hardware/hardwared.py", P0_HARDWARED),
    "P1": ("system/hardware/chestnut/flash.py", P1_FLASH),
    "P2": ("selfdrive/modeld/helpers.py", P2_HELPERS),
    "P3": ("common/hardware/usb.py", P3_USB),
    "P4": ("system/hardware/chestnut/status.py", P4_STATUS),
    "P5": ("sunnypilot/modeld_v2/modeld.py", P5_MODELD),
}

ORDER = ["P0", "P1", "P2", "P3", "P4", "P5"]

TITLES = {
    "P0": "hardwared.py 锁死刷写(砖机根因)",
    "P1": "flash.py CHESTNUT_FLASH_UNLOCK 解锁门",
    "P2": "helpers.py 只按 VID/PID + 60s 超时",
    "P3": "usb.py extcon 判线",
    "P4": "status.py 供电遥测不误报",
    "P5": "modeld.py 解除 fmt 死锁 + 600s 超时",
}

EGPU_USB_PERM_SH = r"""#!/bin/bash
# [port-egpu-usb] chestnut(ASM2464) USB 节点权限守护。
#
# devtmpfs 以 0600 root:root 建节点，udev 要读完描述符才放宽成 0664。
# eGPU 与车机同时上电时 ASM2464 描述符响应慢，这个窗口可达数十秒；
# modeld 以 comma 身份 libusb_open -> EACCES。comma 在 root 组，
# 所以这里抢在 udev 之前把节点 chmod 666。
#
# 由 launch_env.sh 以 setsid 拉起，单实例运行，日志 /data/egpu_usb_perm.log
set -u

SELF="$(readlink -f "$0")"
PIDFILE=/data/egpu_usb_perm.pid
LOG=/data/egpu_usb_perm.log

# 自提权
if [ "$(id -u)" -ne 0 ]; then
  exec sudo -n "$SELF" "$@"
fi

# 单实例锁
if [ -f "$PIDFILE" ]; then
  OLD=$(cat "$PIDFILE" 2>/dev/null || echo 0)
  if [ -n "$OLD" ] && [ "$OLD" != "$$" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "$(date -u +%FT%TZ) already running pid=$OLD, exit" >>"$LOG"
    exit 0
  fi
fi
echo $$ >"$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT INT TERM

echo "$(date -u +%FT%TZ) egpu_usb_perm start pid=$$" >>"$LOG"

while true; do
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    v=$(cat "$d/idVendor" 2>/dev/null)
    [ "$v" = "3801" ] || [ "$v" = "add1" ] || continue
    b=$(cat "$d/busnum" 2>/dev/null)
    n=$(cat "$d/devnum" 2>/dev/null)
    [ -n "$b" ] && [ -n "$n" ] || continue
    node=$(printf "/dev/bus/usb/%03d/%03d" "$b" "$n")
    if [ -e "$node" ]; then
      cur=$(stat -c '%a' "$node" 2>/dev/null)
      if [ "$cur" != "666" ]; then
        chmod 666 "$node" 2>/dev/null && \
          echo "$(date -u +%FT%TZ) chmod 666 $node (was $cur) speed=$(cat "$d/speed" 2>/dev/null)" >>"$LOG"
      fi
    fi
  done
  sleep 0.5
done
"""


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------

def md5(path: str) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_text(path: str) -> str:
    # newline='' 必须显式给：文本模式默认做 universal-newline 转换，
    # 在 Windows 上读会把 \r\n 折叠成 \n、写又会把 \n 展开成 \r\n，
    # 结果整份文件行尾被改写（2026-10-27 实测 diff 变成全文件重写）。
    # 源码文件的行尾必须原样保留。
    with open(path, "r", encoding="utf-8", newline="") as f:
        return f.read()


def write_text(path: str, text: str) -> None:
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)


def syntax_ok(path: str, tempdir: str) -> tuple[bool, str]:
    """编译检查，pyc 写到 tempdir，避免污染仓库 __pycache__。"""
    import py_compile
    cfile = os.path.join(tempdir, os.path.basename(path) + "c")
    try:
        py_compile.compile(path, cfile=cfile, doraise=True)
        return True, ""
    except py_compile.PyCompileError as e:
        return False, str(e)


def backup_path(rel: str) -> str:
    return os.path.join(_BACKUP_ROOT, rel.replace("/", "__") + ".orig")


# ---------------------------------------------------------------------------
# 计划 / 应用
# ---------------------------------------------------------------------------

def _fit_eol(s: str, use_crlf: bool) -> str:
    """把补丁字面量的行尾对齐到目标文件的行尾。

    本脚本自身在 Windows 上是 CRLF，设备源码是 LF。Python tokenizer 会把源码行尾
    归一化成 \\n，所以字面量里通常是 \\n；但一旦有人在 CRLF 环境下编辑过本脚本，
    字面量可能带 \\r\\n。统一先归一再按目标展开，避免「补丁打上了但行尾混排」。
    """
    s = s.replace("\r\n", "\n")
    return s.replace("\n", "\r\n") if use_crlf else s


def plan_file(repo: str, key: str) -> dict:
    rel, pairs = FILES[key]
    path = os.path.join(repo, rel)
    result = {"key": key, "rel": rel, "path": path, "exists": os.path.exists(path),
              "actions": [], "errors": []}
    if not result["exists"]:
        result["errors"].append(f"文件不存在: {path}")
        return result

    text = read_text(path)
    crlf = text.count("\r\n")
    use_crlf = crlf > (text.count("\n") - crlf)
    for idx, (probe, old, new) in enumerate(pairs):
        tag = f"{key}.{idx + 1}"
        old = _fit_eol(old, use_crlf)
        new = _fit_eol(new, use_crlf)
        n_old = text.count(old)
        n_probe = text.count(probe)

        # 幂等判定必须放在最前：插入型补丁（P5.1 GMMU / P3.1 cable_connected）的
        # new 完整包含 old，打完以后 old 仍然存在，只看 n_old 会重复插入。
        # 用「完整 new 是否已作为子串存在」作为唯一权威判据。
        if new in text:
            result["actions"].append((tag, "SKIP", None, None))
        elif n_old == 1:
            result["actions"].append((tag, "PATCH", old, new))
            text = text.replace(old, new, 1)
        elif n_old == 0 and n_probe >= 1:
            result["actions"].append((tag, "SKIP", None, None))
        elif n_old == 0 and n_probe == 0:
            result["errors"].append(
                f"{tag}: 锚点未命中，且未发现已打补丁痕迹 —— 该文件版本可能不同")
        else:
            result["errors"].append(
                f"{tag}: 锚点出现 {n_old} 次（必须恰好 1 次），拒绝修改")
    return result


def cmd_check(repo: str) -> int:
    print("=" * 78)
    print(f"仓库: {repo}")
    print("=" * 78)
    bad = 0
    total_patch = 0
    for key in ORDER:
        r = plan_file(repo, key)
        print(f"\n[{key}] {TITLES[key]}")
        print(f"      {r['path']}")
        if not r["exists"]:
            print("      ✗ 文件不存在")
            bad += 1
            continue
        for tag, act, _o, _n in r["actions"]:
            print(f"      {'•' if act == 'PATCH' else '✓'} {tag}: "
                  f"{'待打补丁' if act == 'PATCH' else '已是新版本，跳过'}")
            if act == "PATCH":
                total_patch += 1
        for e in r["errors"]:
            print(f"      ✗ {e}")
            bad += 1

    print("\n" + "=" * 78)
    if bad:
        print(f"结论: 有 {bad} 个问题，不要 apply")
    elif total_patch == 0:
        print("结论: 全部已是 sptest1026 版本，无需修改")
    else:
        print(f"结论: 可打 {total_patch} 处补丁，锚点全部唯一命中")
    print("=" * 78)
    return 0 if bad == 0 else 1


def cmd_apply(repo: str, with_perm: bool) -> int:
    # 先全量预检，任何一处不通过就整体不动
    plans = []
    for key in ORDER:
        r = plan_file(repo, key)
        if r["errors"]:
            print(f"✗ [{key}] {TITLES[key]}")
            for e in r["errors"]:
                print(f"    {e}")
            print("\n预检失败，未做任何修改。")
            return 1
        plans.append(r)

    os.makedirs(_BACKUP_ROOT, exist_ok=True)
    tempdir = os.path.join(_BACKUP_ROOT, "_pyc")
    os.makedirs(tempdir, exist_ok=True)

    applied = []
    print("=" * 78)
    for r in plans:
        patches = [a for a in r["actions"] if a[1] == "PATCH"]
        if not patches:
            print(f"[{r['key']}] {TITLES[r['key']]}: 已是最新，跳过")
            continue

        # 备份
        bp = backup_path(r["rel"])
        if not os.path.exists(bp):
            shutil.copy2(r["path"], bp)
        before = md5(r["path"])

        text = read_text(r["path"])
        for tag, _act, old, new in patches:
            assert text.count(old) == 1, f"{tag} 锚点在应用时不再唯一"
            text = text.replace(old, new, 1)
        write_text(r["path"], text)

        ok, err = syntax_ok(r["path"], tempdir)
        if not ok:
            shutil.copy2(bp, r["path"])
            print(f"✗ [{r['key']}] 语法检查失败，已回滚: {err}")
            return 1

        after = md5(r["path"])
        applied.append((r["key"], r["path"], bp, before, after, len(patches)))
        print(f"✓ [{r['key']}] {TITLES[r['key']]}: {len(patches)} 处")
        print(f"    {r['path']}")
        print(f"    {before} -> {after}")

    if with_perm:
        perm = os.path.join(os.path.dirname(repo), "system", "egpu_usb_perm.sh")
        os.makedirs(os.path.dirname(perm), exist_ok=True)
        existed = os.path.exists(perm)
        if existed and "port-egpu-usb" not in read_text(perm):
            bp = backup_path("system/egpu_usb_perm.sh")
            if not os.path.exists(bp):
                shutil.copy2(perm, bp)
            print(f"! {perm} 已存在且不是本版本，已备份到 {bp} 后覆盖")
        write_text(perm, EGPU_USB_PERM_SH)
        os.chmod(perm, 0o755)
        print(f"✓ [P6] egpu_usb_perm.sh 已写入 {'(覆盖)' if existed else '(新建)'}")
        print(f"    {perm}")

    print("=" * 78)
    if not applied and not with_perm:
        print("没有需要修改的内容。")
        return 0

    print(f"\n备份目录: {_BACKUP_ROOT}")
    print("\n生效方式（任选其一）:")
    print("  sudo systemctl restart comma")
    print("  sudo reboot")
    print("\n重启后验证:")
    print("  # 1. 确认不会再刷固件")
    print("  grep -c 'chestnut flash SKIPPED' /data/log/swaglog.* 2>/dev/null | tail -1")
    print("  # 2. 确认门禁已打开（应看到 load_start / CHX_USB_ENUM）")
    print("  grep -E 'gate|load_start|CHX_USB_ENUM' /data/egpu_diag.jsonl | tail -20")
    print("  # 3. 确认节点权限守护在跑")
    print("  pgrep -af egpu_usb_perm.sh; tail -5 /data/egpu_usb_perm.log")
    return 0


def cmd_revert(repo: str) -> int:
    if not os.path.isdir(_BACKUP_ROOT):
        print(f"备份目录不存在: {_BACKUP_ROOT}，无可回滚内容")
        return 1

    print("=" * 78)
    done = 0
    for key in ORDER:
        rel, _ = FILES[key]
        bp = backup_path(rel)
        if not os.path.exists(bp):
            print(f"· [{key}] 无备份，跳过: {rel}")
            continue
        dst = os.path.join(repo, rel)
        cur = md5(dst) if os.path.exists(dst) else "(missing)"
        shutil.copy2(bp, dst)
        restored = md5(dst)
        print(f"✓ [{key}] {rel}")
        print(f"    {cur} -> {restored}  (备份 {md5(bp)})")
        if restored != md5(bp):
            print("    ✗ 回滚后 md5 与备份不一致")
            return 1
        done += 1

    perm = os.path.join(os.path.dirname(repo), "system", "egpu_usb_perm.sh")
    bp = backup_path("system/egpu_usb_perm.sh")
    if os.path.exists(bp):
        shutil.copy2(bp, perm)
        print(f"✓ [P6] egpu_usb_perm.sh 已还原")
    elif os.path.exists(perm):
        os.remove(perm)
        print(f"✓ [P6] egpu_usb_perm.sh 已删除（原本不存在）")

    print("=" * 78)
    print(f"已回滚 {done} 个文件。记得重启: sudo systemctl restart comma")
    return 0


def cmd_status(repo: str) -> int:
    print("=" * 78)
    print(f"仓库: {repo}")
    print("=" * 78)
    for key in ORDER:
        r = plan_file(repo, key)
        if not r["exists"]:
            print(f"[{key}] ✗ 文件不存在: {r['path']}")
            continue
        n_patch = sum(1 for a in r["actions"] if a[1] == "PATCH")
        n_skip = sum(1 for a in r["actions"] if a[1] == "SKIP")
        state = "已移植" if n_patch == 0 and n_skip else ("部分移植" if n_skip else "未移植")
        print(f"[{key}] {state:6s} patch={n_patch} done={n_skip}  {r['rel']}")
        for e in r["errors"]:
            print(f"        ✗ {e}")

    perm = os.path.join(os.path.dirname(repo), "system", "egpu_usb_perm.sh")
    print(f"\n[P6] egpu_usb_perm.sh: {'存在' if os.path.exists(perm) else '不存在'}  {perm}")

    # 运行态
    print("\n--- 运行态 ---")
    for label, cmd in (
        ("flash.py 进程", ["pgrep", "-af", "chestnut/flash.py"]),
        ("perm.sh 进程", ["pgrep", "-af", "egpu_usb_perm.sh"]),
        ("hardwared", ["pgrep", "-af", "system/hardware/hardwared.py"]),
    ):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
        except Exception as e:
            out = f"(查询失败 {e})"
        print(f"{label}: {out if out else '无'}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="按 sptest1026 移植 eGPU(chestnut) 修复",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--repo", default=DEFAULT_REPO, help=f"inner repo 根 (默认 {DEFAULT_REPO})")
    ap.add_argument("--backup-root", default=DEFAULT_BACKUP,
                    help=f"备份目录 (默认 {DEFAULT_BACKUP})")
    ap.add_argument("--check", action="store_true", help="只预检，不修改")
    ap.add_argument("--apply", action="store_true", help="应用补丁")
    ap.add_argument("--revert", action="store_true", help="从备份回滚")
    ap.add_argument("--status", action="store_true", help="查看移植状态 + 运行态")
    ap.add_argument("--no-perm", action="store_true",
                    help="不安装 system/egpu_usb_perm.sh（默认安装）")
    args = ap.parse_args()

    if not any((args.check, args.apply, args.revert, args.status)):
        ap.print_help()
        return 2

    if getattr(os, "geteuid", lambda: 0)() != 0:
        print("警告: 当前不是 root，写 openpilot/ 目录可能失败", file=sys.stderr)

    repo = os.path.abspath(args.repo)
    if not os.path.isdir(repo):
        print(f"仓库目录不存在: {repo}", file=sys.stderr)
        return 2

    global _BACKUP_ROOT
    _BACKUP_ROOT = os.path.abspath(args.backup_root)

    if args.check:
        return cmd_check(repo)
    if args.apply:
        return cmd_apply(repo, with_perm=not args.no_perm)
    if args.revert:
        return cmd_revert(repo)
    return cmd_status(repo)


if __name__ == "__main__":
    sys.exit(main())
