
import json
import os
import sys
import time

DIAG_PATH = os.environ.get('EGPU_DIAG_PATH', '/data/egpu_diag.jsonl')
DIAG_MAX_BYTES = 256 * 1024
RETRY_PATH = os.environ.get('EGPU_RETRY_PATH', '/data/egpu_retry.json')

RETRY_MAX = 1
RETRY_BACKOFF = (10.0, 30.0, 60.0)

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
  if raw == 0:
    return 'ok'
  if raw > 0:
    return 'transfer:' + _TRANSFER.get(raw, f'unknown_{raw}')
  return 'usb:' + _USB.get(raw, f'unknown_{raw}')

def boot_id() -> str:
  try:
    with open('/proc/sys/kernel/random/boot_id') as f:
      return f.read().strip()
  except OSError:
    return 'unknown'

def egpu(event: str, **fields) -> dict:
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
  try:
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
  try:
    from tinygrad.device import Device
    raw = int(Device[dev].error_state.host.view(fmt='q')[0])
    return raw, classify_error(raw)
  except Exception as e:
    return -1000, f'unavailable:{e.__class__.__name__}'

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
  _save_retry({'boot': boot_id(), 'count': 0, 'last_ts': 0.0})

def pcie_ltssm() -> int | None:
  try:
    from openpilot.common.hardware.usb import chestnut_ltssm, chestnut_nodes
    for node in chestnut_nodes():
      value = chestnut_ltssm(node["vendor_id"], node["product_id"])
      if value >= 0:
        return value
  except Exception:
    pass

  fd = None
  try:
    import ctypes
    import fcntl
    from openpilot.system.hardware.chestnut.flash import Ctrl, USBDEVFS_CONTROL, find_chestnut, open_device
    path, _, _ = find_chestnut()
    if path is None:
      return None
    fd = open_device(path)
    buf = (ctypes.c_ubyte * 1)()
    fcntl.ioctl(fd, USBDEVFS_CONTROL,
                Ctrl(0xC0, 0xE4, 0xB450, 0, 1, 1000, ctypes.cast(buf, ctypes.c_void_p)))
    return int(buf[0])
  except Exception:
    return None
  finally:
    if fd is not None:
      try:
        os.close(fd)
      except Exception:
        pass

LINK_ERR_UNSTABLE = 10

def link_err() -> int | None:
  try:
    from openpilot.common.hardware.usb import get_usb_state
    for dev in get_usb_state():
      if int(dev.get("vendorId", 0)) in (0x3801, 0xADD1):
        return int(dev.get("linkErrorCount", 0))
  except Exception:
    pass
  return None

def link_err_delta(before: int | None, after: int | None) -> int:
  if before is None or after is None:
    return 0
  return (after - before) & 0xFFFF
