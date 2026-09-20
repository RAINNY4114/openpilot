"""Big-model DOWNLOAD progress badge, drawn under the eGPU badge.

Two stacked lines: the artifact type on top ("ONNX" / "PKL" / "FW") and the
percentage below ("45%"). Uses the same font size and the same box width as the
eGPU badge, so only the height grows to fit the second line.

Scope is deliberately narrow: only download progress is shown, because that is the
only phase with a real numeric source (downloaded_bytes / total_bytes in
status.json). Loading the model onto the GPU has no numeric progress anywhere in
openpilot - modeld only exposes UsbGpuLoading / UsbGpuActive booleans - so nothing
is drawn for that phase rather than inventing a fake percentage.

The label comes from status.json's "detail" field: big_model reports the ONNX
download itself and leaves detail unset, precompiled_fetch reports the
precompiled model with detail="pkl", and firmware.py reports the firmware blobs
with detail="firmware".

This lives in its own module so hud_renderer.py only needs a single call, keeping
upstream merges into that file conflict-free. Geometry and colors mirror
hud_renderer's eGPU badge and are duplicated here to avoid a circular import
(hud_renderer imports this module).
"""
from __future__ import annotations

import os
import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

# --- geometry mirrored from hud_renderer's eGPU badge ---
_BORDER_SIZE = 30
_BUTTON_SIZE = 192
_RIGHT_MARGIN = 24
_EGPU_FONT_SIZE = 38
_EGPU_PAD_X, _EGPU_PAD_Y = 18, 8
_EGPU_TOP = 24

# Same font size as the eGPU badge. The text is split over two lines instead of one
# long "ONNX 45%" precisely so the box can stay as narrow as the eGPU badge does.
_LINE_GAP = 4        # vertical space between the two lines
_GAP = 12            # space between the eGPU badge and this one

# --- colors mirrored from hud_renderer.Colors ---
_YELLOW = rl.Color(255, 255, 0, 210)
_BG = rl.Color(0, 0, 0, 150)

# Only this state carries a real byte counter. "checking" / "verifying" /
# "compiling" have no meaningful progress, and "compiled" / "ready" /
# "waiting_for_ignition" / "error" are terminal states with nothing to report.
_DOWNLOAD_STATES = {"downloading"}

# States in which everything has been downloaded already.
_FINAL_STATES = {"compiled", "ready"}

# "verifying" also means the artifact is on disk and committed - the chunked installer
# writes the manifest (and the reboot flag) before this phase - so the reboot prompt
# belongs there too. Without it the badge went blank for the ~1 minute the load check
# takes, right after the download reached 100%.
_PROMPT_STATES = _FINAL_STATES | {"verifying"}

# status.json's "detail" field -> first-line label. An unset or unknown detail
# means the ONNX download, which big_model reports without any detail field.
_LABELS = {
  "pkl": "PKL",
  "firmware": "FW",
  # The chunked delivery is the same artifact type (the model pickle), just split into
  # 45 MB pieces, so it gets the same label.
  "chunked model": "PKL",
}


def read_status() -> dict | None:
  """Validated big-model status, or None when unavailable."""
  try:
    from openpilot.selfdrive.modeld.big_model import model_cache_dir
    from openpilot.selfdrive.modeld.big_model_status import read_big_model_status
    return read_big_model_status(model_cache_dir())
  except Exception:
    return None


# A download interrupted by a reboot leaves status.json saying "downloading" even
# though nothing is transferring until the background downloader starts again.
# Scanning /proc (no fork) at most this often keeps that check cheap.
_DOWNLOADER_MARKER = "--ensure-if-egpu"
_DL_CHECK_INTERVAL = 2.0
_dl_check = {"at": -1e9, "running": False}

# The status file is rewritten for every ~1 MB transferred, so a file that is still being
# updated is a second, delivery-agnostic liveness signal. Needed because the chunked set is
# downloaded inside manager.py (the boot build), which the /proc scan above cannot see.
_STALE_AFTER = 12.0


def _recently_updated(status: dict) -> bool:
  try:
    return time.time() - float(status.get("updated_at") or 0.0) < _STALE_AFTER
  except (TypeError, ValueError):
    return False


def _downloader_running() -> bool:
  """Is the background big-model downloader actually alive right now?"""
  now = time.monotonic()
  if now - _dl_check["at"] < _DL_CHECK_INTERVAL:
    return _dl_check["running"]

  running = False
  try:
    for entry in os.listdir("/proc"):
      if not entry.isdigit():
        continue
      try:
        with open(f"/proc/{entry}/cmdline", "rb") as f:
          cmd = f.read().decode("utf-8", "ignore")
      except OSError:
        continue
      if "modeld.big_model" in cmd and _DOWNLOADER_MARKER in cmd:
        running = True
        break
  except OSError:
    running = False

  _dl_check["at"] = now
  _dl_check["running"] = running
  return running


def _needs_reboot() -> bool:
  """A download finished but the big model still is not running.

  precompiled_fetch touches REBOOT_FLAG in /tmp (a tmpfs) once the download
  completes, so the flag cannot survive a reboot or a power cycle - which is
  exactly how the prompt stops showing afterwards.
  """
  try:
    from openpilot.selfdrive.modeld.precompiled_fetch import REBOOT_FLAG
    if not REBOOT_FLAG.exists():
      return False
  except Exception:
    return False
  try:
    from openpilot.common.params import Params
    params = Params()
    # Still loading counts as fine: modeld is already bringing the model up, so no
    # reboot is needed. Without this the prompt would flash on every boot for the
    # tens of seconds it takes to load, telling the user to reboot for nothing.
    if params.get_bool("UsbGpuLoading"):
      return False
    return not params.get_bool("UsbGpuActive")
  except Exception:
    return False


def badge_style(status: dict, frac: float) -> tuple[list[str], rl.Color] | None:
  """Return (lines, color) to draw, or None to draw nothing.

  Two lines (artifact + percentage) while downloading; a single "REBOOT" line
  once the download is done but the model has not started yet.
  """
  state = status.get("state") or ""
  if state in _DOWNLOAD_STATES:
    # big_model writes the ONNX progress with no detail field; precompiled_fetch
    # writes detail="pkl" and firmware.py writes detail="firmware".
    label = _LABELS.get(status.get("detail") or "", "ONNX")
    return [label, f"{int(max(0.0, min(1.0, frac)) * 100)}%"], _YELLOW
  if state in _PROMPT_STATES and _needs_reboot():
    return ["REBOOT"], _YELLOW
  return None


def draw(rect: rl.Rectangle, font) -> None:
  """Draw the download progress badge below the eGPU badge.

  Never raises: a decoration must not take down the whole HUD.
  """
  try:
    _draw_impl(rect, font)
  except Exception as exc:
    from openpilot.common.swaglog import cloudlog
    cloudlog.warning(f"model download badge skipped: {exc}")


def _draw_impl(rect: rl.Rectangle, font) -> None:
  # Same visibility rule as the eGPU badge, so the two always appear together.
  if not (ui_state.usbgpu_present or ui_state.usbgpu_active or
          ui_state.usbgpu_loading or ui_state.usbgpu_startup_failed):
    return

  status = read_status()
  if status is None:
    return

  # A reboot mid-download leaves status.json on "downloading" while nothing is
  # transferring yet. Do not present that as live progress: require either the background
  # downloader to be alive or the status file to be still updating.
  if (status.get("state") or "") in _DOWNLOAD_STATES and not (_downloader_running() or _recently_updated(status)):
    return

  total = status.get("total_bytes") or 0
  done_bytes = status.get("downloaded_bytes") or 0
  # No early return on total<=0: the REBOOT prompt carries no byte counter.
  frac = max(0.0, min(1.0, done_bytes / float(total))) if total > 0 else 0.0
  styled = badge_style(status, frac)
  if styled is None:
    return
  lines, color = styled

  egpu_text_size = measure_text_cached(font, "eGPU", _EGPU_FONT_SIZE)
  line_sizes = [measure_text_cached(font, line, _EGPU_FONT_SIZE) for line in lines]

  # Same width as the eGPU badge, so swapping "45%" for "100%" never resizes it.
  width = max([egpu_text_size.x] + [s.x for s in line_sizes]) + _EGPU_PAD_X * 2
  line_h = max(s.y for s in line_sizes)
  height = line_h * len(lines) + _LINE_GAP * (len(lines) - 1) + _EGPU_PAD_Y * 2

  # Sit right below the eGPU badge: its top plus its height plus a small gap.
  y = rect.y + _EGPU_TOP + (egpu_text_size.y + _EGPU_PAD_Y * 2) + _GAP

  badge = rl.Rectangle(
    rect.x + rect.width - _BORDER_SIZE - _BUTTON_SIZE - width - _RIGHT_MARGIN,
    y,
    width,
    height,
  )
  rl.draw_rectangle_rounded(badge, 0.35, 8, _BG)
  rl.draw_rectangle_rounded_lines_ex(badge, 0.35, 8, 3, color)

  # Center every line horizontally: the box matches the eGPU badge's width and is
  # therefore wider than short labels such as "45%".
  text_y = badge.y + _EGPU_PAD_Y
  for line, size in zip(lines, line_sizes):
    rl.draw_text_ex(font, line, rl.Vector2(badge.x + (width - size.x) / 2, text_y),
                    _EGPU_FONT_SIZE, 0, color)
    text_y += line_h + _LINE_GAP
