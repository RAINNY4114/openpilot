"""egpu_log.py — [eGPU] 结构化诊断（P1-2 / P2-1，2026-10-27）

设计约束（照 egpucp 的教训）：
  1. 只写 **stderr** 与独立文件，绝不进 worker 的 stdout —— stdout 是 tinygrad 的协议通道。
  2. 任何异常都被吞掉：诊断代码永远不能把 modeld 带崩。
  3. 一条记录一行 JSON，便于 `grep '\\[eGPU\\]' /data/egpu_diag.jsonl | jq`。

为什么需要它：本机大模型失败时**一条日志都没有**（`get_active_bundle(chestnut=True)`
返回 None 导致整条分支短路），只能事后靠 `chestnutPresent` 猜。有了 `gate` 记录，
一眼就能看出卡在 bundle 还是卡在硬件。
"""

import json
import os
import sys
import time

DIAG_PATH = os.environ.get('EGPU_DIAG_PATH', '/data/egpu_diag.jsonl')
DIAG_MAX_BYTES = 256 * 1024
RETRY_PATH = os.environ.get('EGPU_RETRY_PATH', '/data/egpu_retry.json')

# 每 boot 最多请求几次 onroad cycle 来重试大模型（egpucp 用 3，照抄）
RETRY_MAX = 3
# 第 1/2/3 次重试的最小间隔（秒）。由 threading.Timer 兑现，不阻塞帧循环。
RETRY_BACKOFF = (10.0, 30.0, 60.0)

# ---------------------------------------------------------------------------
# libusb 错误码分类
#
# 关键教训（egpucp）：旧分类器只认 -1，把 +1 = LIBUSB_TRANSFER_ERROR 当成
# 「数值失败」永久拒绝重试。**正数来自 USB 传输层、负数来自 libusb 调用层**，
# 必须分开看，否则会把「偶发一次传输错误」升级成「设备永久不可用」。
# 来源：autogen/libusb.py:204（libusb_transfer_status）与 libusb.h（libusb_error）。
# ---------------------------------------------------------------------------
_TRANSFER = {
  0: 'OK',
  1: 'TRANSFER_ERROR',
  2: 'TRANSFER_TIMED_OUT',
  3: 'TRANSFER_CANCELLED',
  4: 'TRANSFER_STALL',
  5: 'TRANSFER_NO_DEVICE',
  6: 'TRANSFER_OVERFLOW',
}
_USB = {
  -1: 'IO',
  -2: 'INVALID_PARAM',
  -3: 'ACCESS',
  -4: 'NO_DEVICE',
  -5: 'NOT_FOUND',
  -6: 'BUSY',
  -7: 'TIMEOUT',
  -8: 'OVERFLOW',
  -9: 'PIPE',
  -10: 'INTERRUPTED',
  -11: 'NO_MEM',
  -12: 'NOT_SUPPORTED',
  -99: 'OTHER',
}


def classify_error(raw: int) -> str:
  """把 error_state 的原始整数翻译成可读类别。"""
  if raw == 0:
    return 'ok'
  if raw > 0:
    return 'transfer:' + _TRANSFER.get(raw, f'unknown_{raw}')
  return 'usb:' + _USB.get(raw, f'unknown_{raw}')


def boot_id() -> str:
  """一次开机一个 id。用它给重试预算分段，跨 modeld 重启保留、换 boot 自动归零。"""
  try:
    with open('/proc/sys/kernel/random/boot_id') as f:
      return f.read().strip()
  except OSError:
    return 'unknown'


def egpu(event: str, **fields) -> dict:
  """记一条结构化诊断。永远不抛异常。"""
  rec = {'ts': round(time.time(), 3), 'boot': boot_id()[:8], 'ev': event}
  rec.update(fields)
  try:
    line = '[eGPU] ' + json.dumps(rec, ensure_ascii=False, sort_keys=True, default=str)
  except Exception:
    line = f'[eGPU] {{"ev": "{event}", "serialize": "failed"}}'
  try:
    print(line, file=sys.stderr, flush=True)
  except Exception:
    pass
  _append(line)
  return rec


def _append(line: str) -> None:
  try:
    if os.path.exists(DIAG_PATH) and os.path.getsize(DIAG_PATH) > DIAG_MAX_BYTES:
      os.replace(DIAG_PATH, DIAG_PATH + '.1')
    with open(DIAG_PATH, 'a') as f:
      f.write(line + '\n')
  except Exception:
    pass


def usb_snapshot() -> dict:
  """一次 USB 拓扑快照。

  用途是把「零枚举」和「枚举了但传输出错」分开：
    零枚举      -> nodes == [] 且 devices 里没有 chestnut 的 vendorId
    在 USB2 侧  -> nodes 非空但 speed < 5000
    传输出错    -> nodes 在 USB3 侧，但 error_state != 0
  """
  try:
    # ⚠️ chestnut_present 在 selfdrive/modeld/helpers.py，**不在** common/hardware/usb.py
    from openpilot.common.hardware.usb import cable_connected, chestnut_nodes, get_usb_state
    from openpilot.selfdrive.modeld.helpers import chestnut_present
    nodes = [{k: v for k, v in n.items() if k != 'sysfs'} for n in chestnut_nodes()]
    return {
      'present': chestnut_present(),
      'cable': cable_connected(),
      'nodes': nodes,
      'devices': get_usb_state(),
    }
  except Exception as e:
    return {'error': repr(e)}


def read_error_state(dev: str) -> tuple[int, str]:
  """读 tinygrad Device 的 error_state[0]。读不到返回 (-1000, 'unavailable:...')。"""
  try:
    from tinygrad.device import Device
    raw = int(Device[dev].error_state.host.view(fmt='q')[0])
    return raw, classify_error(raw)
  except Exception as e:
    return -1000, f'unavailable:{e.__class__.__name__}'


# ---------------------------------------------------------------------------
# 重试预算（P0-2）
#
# 原实现只有 `params.put_bool("ChestnutRetry", True)`，而该键带
# CLEAR_ON_MANAGER_START|CLEAR_ON_IGNITION_ON（params_keys.h:140）——
# 也就是说**每次 manager 启动就会被清掉**，所以「一次失败永久放弃」的准确说法是
# 「同一个 manager 生命周期内只重试一次」，跨 manager 重启则无上限、也无退避。
#
# 这里换成按 boot_id 记次：上限 3 次、10/30/60 s 退避、换 boot 自动归零。
# 用文件而不是 Params，是因为 Params 会被 CLEAR_ON_* 清掉，且新增键必须重编
# libparams_c.so（踩过：不重编 -> UnknownKeyName -> 卡 comma logo）。
# ---------------------------------------------------------------------------


def _load_retry() -> dict:
  try:
    with open(RETRY_PATH) as f:
      st = json.load(f)
    if isinstance(st, dict) and st.get('boot') == boot_id():
      return st
  except Exception:
    pass
  return {'boot': boot_id(), 'count': 0, 'last_ts': 0.0}


def _save_retry(st: dict) -> None:
  try:
    tmp = RETRY_PATH + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(st, f)
    os.replace(tmp, RETRY_PATH)
  except Exception:
    pass


def retry_budget() -> tuple[dict, bool, float]:
  """返回 (state, allowed, backoff)。allowed=False 表示本 boot 预算已用尽。"""
  st = _load_retry()
  idx = int(st.get('count', 0))
  if idx >= RETRY_MAX:
    return st, False, 0.0
  return st, True, RETRY_BACKOFF[min(idx, len(RETRY_BACKOFF) - 1)]


def consume_retry(st: dict) -> dict:
  st['count'] = int(st.get('count', 0)) + 1
  st['last_ts'] = time.time()
  _save_retry(st)
  return st


def reset_retry() -> None:
  """大模型加载成功时调用：本 boot 的失败计数清零，下次失败还能重试。"""
  _save_retry({'boot': boot_id(), 'count': 0, 'last_ts': 0.0})
