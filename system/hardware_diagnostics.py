#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

HARDWARE_DIAGNOSTIC_LOG_PATH = os.getenv("HARDWARE_DIAGNOSTIC_LOG_PATH", "/data/community/crashes/error.log")
HARDWARE_DIAGNOSTIC_ENABLED = os.getenv("HARDWARE_DIAGNOSTIC_ENABLED", "0").lower() in ("1", "true", "yes", "on")


def _int_env(name: str, default: int) -> int:
  try:
    return int(os.getenv(name, str(default)))
  except ValueError:
    return default


HARDWARE_DIAGNOSTIC_MAX_BYTES = _int_env("HARDWARE_DIAGNOSTIC_MAX_BYTES", 60000)

_LOG_LOCK = threading.Lock()
_LAST_WRITE_TIME: dict[str, float] = {}


def _format_value(value: Any) -> str:
  try:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, default=str)
  except Exception:
    return str(value)


def _format_block(source: str, issue: str, details: dict[str, Any] | None) -> str:
  lines = [
    "",
    f"=== Hardware Diagnostic ({datetime.now().strftime('%Y-%m-%d %H:%M:%S')}) ===",
    f"source: {source}",
    f"issue: {issue}",
  ]

  if details:
    for key in sorted(details):
      lines.append(f"{key}: {_format_value(details[key])}")

  return "\n".join(lines) + "\n"


def _trim_log_if_needed(path: Path) -> None:
  if HARDWARE_DIAGNOSTIC_MAX_BYTES <= 0:
    return
  try:
    size = path.stat().st_size
  except FileNotFoundError:
    return

  if size <= HARDWARE_DIAGNOSTIC_MAX_BYTES:
    return

  keep_bytes = max(HARDWARE_DIAGNOSTIC_MAX_BYTES - 512, 1024)
  with path.open("rb") as f:
    f.seek(max(0, size - keep_bytes))
    tail = f.read()

  marker = (
    f"=== Hardware Diagnostic Log Trimmed ({datetime.now().strftime('%Y-%m-%d %H:%M:%S')}) ===\n"
    f"Kept the latest {len(tail)} bytes from {size} bytes so Developer/Server error upload stays useful.\n"
  ).encode("utf-8", errors="replace")
  path.write_bytes(marker + tail)


def append_hardware_diagnostic(source: str, issue: str, details: dict[str, Any] | None = None, *,
                               dedupe_key: str | None = None, min_interval_sec: float = 60.0,
                               log_path: str = HARDWARE_DIAGNOSTIC_LOG_PATH) -> bool:
  if not HARDWARE_DIAGNOSTIC_ENABLED:
    return False

  key = dedupe_key or f"{source}:{issue}"
  now = time.monotonic()

  with _LOG_LOCK:
    last_write_time = _LAST_WRITE_TIME.get(key)
    if last_write_time is not None and now - last_write_time < min_interval_sec:
      return False

    try:
      path = Path(log_path)
      path.parent.mkdir(parents=True, exist_ok=True)
      with path.open("a", encoding="utf-8", errors="replace") as f:
        f.write(_format_block(source, issue, details))
      _trim_log_if_needed(path)
      _LAST_WRITE_TIME[key] = now
      return True
    except Exception:
      return False
