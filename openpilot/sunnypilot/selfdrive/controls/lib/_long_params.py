"""Direct file-backed param reader for longitudinal enhancements.

Bypasses Params() key validation (libparams_c.so) so new keys need no rebuild.
Priority mirrors the device convention: /data/ford_params > /dev/shm/params > /data/params/d.
"""
import os

PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")


def read_raw(key):
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, str(key))) as f:
        return f.read().strip()
    except Exception:
      continue
  return None


def read_bool(key, default=False):
  raw = read_raw(key)
  if raw is None:
    return default
  return raw.strip().lower() in ("1", "true", "yes", "on")


def read_int(key, default=0):
  raw = read_raw(key)
  if raw is None:
    return default
  try:
    return int(float(raw))
  except (TypeError, ValueError):
    return default


def read_float(key, default=0.0):
  raw = read_raw(key)
  if raw is None:
    return default
  try:
    return float(raw)
  except (TypeError, ValueError):
    return default


def write_param(key, value):
  try:
    os.makedirs("/data/ford_params", exist_ok=True)
  except Exception:
    pass
  for d in PARAM_DIRS:
    try:
      if not os.path.isdir(d):
        continue
      with open(os.path.join(d, str(key)), "w") as f:
        f.write(str(value))
      return True
    except Exception:
      continue
  return False
