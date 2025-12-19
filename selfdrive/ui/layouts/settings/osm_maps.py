import json
import math
import os
import re
import subprocess
import shutil
import time

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import (
  button_item,
  simple_item,
  text_item,
  toggle_item,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller

OSM_OFFLINE_DIR = "/data/media/0/osm/offline"


class OSMMapsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")
    self._size_cache_bytes = 0
    self._size_cache_t = 0.0
    self._prev_download_active = self._download_active()
    self._download_start_t: float | None = None
    self._download_start_done = 0
    self._download_start_bytes = 0
    self._download_rate_bps = 0.0
    self._download_last_sample_t = 0.0
    self._download_last_sample_bytes = 0

    self._ensure_default_maps_selected()

    self._traffic_item = text_item(
      title=lambda: tr("Download traffic"),
      value=lambda: self._traffic_text(),
      description=lambda: tr("Approximate downloaded data and speed while maps are downloading."),
    )
    self._traffic_item.set_visible(lambda: self._download_active())

    self._eta_item = text_item(
      title=lambda: tr("Download ETA"),
      value=lambda: self._eta_text(),
    )
    self._eta_item.set_visible(lambda: self._download_active())

    self._elapsed_item = text_item(
      title=lambda: tr("Time elapsed"),
      value=lambda: self._elapsed_text(),
    )
    self._elapsed_item.set_visible(lambda: self._download_active())

    self._scroller = Scroller([
      simple_item(title=lambda: tr("### Offline Maps (China) ###")),
      toggle_item(
        title=lambda: tr("Realtime cruise mode"),
        description=lambda: tr("Use offline maps for real-time cruise assistance (no navigation). Requires maps downloaded."),
        initial_state=self._params.get_bool("dp_lincoln_osm_realtime_cruise"),
        callback=lambda val: self._params.put_bool("dp_lincoln_osm_realtime_cruise", val),
        enabled=lambda: self._has_maps(),
      ),
      text_item(
        title=lambda: tr("Offline maps size"),
        value=lambda: self._format_bytes(self._local_bytes()),
        description=lambda: tr("Stored under /data/media/0/osm/offline."),
      ),
      text_item(
        title=lambda: tr("Download status"),
        value=lambda: self._status_text(),
      ),
      self._traffic_item,
      self._eta_item,
      self._elapsed_item,
      button_item(
        title=lambda: tr("Download offline maps"),
        button_text=lambda: tr("DOWNLOAD"),
        description=lambda: tr("Download China offline maps (large download, WiFi recommended)."),
        callback=self._on_download,
        enabled=lambda: not self._download_active(),
      ),
      button_item(
        title=lambda: tr("Cancel download"),
        button_text=lambda: tr("CANCEL"),
        description=lambda: tr("Stop the current download (keeps partial file for resume)."),
        callback=self._on_cancel,
        enabled=lambda: self._download_active(),
      ),
      button_item(
        title=lambda: tr("Remove offline maps"),
        button_text=lambda: tr("REMOVE"),
        description=lambda: tr("Delete downloaded offline maps to free storage."),
        callback=self._on_remove_prompt,
        enabled=lambda: (not self._download_active()) and self._has_any_files(),
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._update_state()
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()

  @staticmethod
  def _default_maps_selected() -> str:
    return json.dumps({"states": [], "nations": ["CN"]}, separators=(",", ":"))

  def _ensure_default_maps_selected(self) -> None:
    selected = self._params.get("MapsSelected")
    if not selected:
      self._params.put("MapsSelected", self._default_maps_selected())

  def _download_active(self) -> bool:
    return bool(self._params_memory.get("OSMDownloadLocations"))

  def _progress(self) -> dict:
    raw = None
    try:
      raw = self._params_memory.get("OSMDownloadProgress")
    except Exception:
      raw = None

    if not raw:
      raw = self._params.get("OSMDownloadProgress")
    if not raw:
      return {}
    try:
      return json.loads(raw)
    except Exception:
      # Fallback: be tolerant of non-JSON payloads (match FrogPilot's regex approach)
      m = re.search(r'"total_files"\s*:\s*(\d+).*"downloaded_files"\s*:\s*(\d+)', raw)
      if not m:
        return {}
      return {
        "total_files": int(m.group(1)),
        "downloaded_files": int(m.group(2)),
      }

  def _has_any_files(self) -> bool:
    return os.path.isdir(OSM_OFFLINE_DIR)

  def _has_maps(self) -> bool:
    return self._local_bytes() > 0

  def _local_bytes(self) -> int:
    now = time.monotonic()
    if now - self._size_cache_t < 1.0:
      return self._size_cache_bytes

    total = 0
    try:
      for root, _, files in os.walk(OSM_OFFLINE_DIR):
        for name in files:
          p = os.path.join(root, name)
          try:
            total += os.path.getsize(p)
          except Exception:
            pass
    except Exception:
      pass

    self._size_cache_bytes = total
    self._size_cache_t = now
    return total

  @staticmethod
  def _format_bytes(num_bytes: int) -> str:
    if num_bytes <= 0:
      return "0 B"
    units = ["B", "KB", "MB", "GB", "TB"]
    i = int(min(len(units) - 1, math.floor(math.log(num_bytes, 1024))))
    value = num_bytes / (1024 ** i)
    return f"{value:.2f} {units[i]}"

  @staticmethod
  def _format_rate(bytes_per_sec: float) -> str:
    bytes_per_sec = max(0.0, float(bytes_per_sec))
    if bytes_per_sec <= 0.0:
      return "0 KB/s"
    units = ["B/s", "KB/s", "MB/s", "GB/s"]
    i = 0
    value = bytes_per_sec
    while value >= 1024.0 and i < (len(units) - 1):
      value /= 1024.0
      i += 1
    return f"{value:.2f} {units[i]}"

  @staticmethod
  def _format_duration(seconds: int) -> str:
    seconds = max(0, int(seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"

  def _traffic_text(self) -> str:
    if not self._download_active() or self._download_start_t is None:
      return ""
    downloaded = max(0, self._local_bytes() - int(self._download_start_bytes))
    return f"{self._format_bytes(downloaded)} ({self._format_rate(self._download_rate_bps)})"

  def _status_text(self) -> str:
    p = self._progress()
    if self._download_active():
      total = int(p.get("total_files", 0) or 0)
      done = int(p.get("downloaded_files", 0) or 0)
      if total > 0:
        pct = int(round(100.0 * min(done, total) / total))
        if done >= total and self._has_maps():
          # Some mapd builds may not clear OSMDownloadLocations on completion.
          self._params_memory.remove("OSMDownloadLocations")
          self._params.remove("OSMDownloadProgress")
          return tr("Ready")
        return tr("Downloading: {done}/{total} ({pct}%)").format(done=done, total=total, pct=pct)
      return tr("Downloading...")

    if p.get("status") == "error":
      msg = p.get("message") or tr("Unknown error")
      return tr("Error: {}").format(msg)
    if self._has_maps():
      return tr("Ready")
    return tr("Not downloaded")

  def _eta_text(self) -> str:
    p = self._progress()
    total = int(p.get("total_files", 0) or 0)
    done = int(p.get("downloaded_files", 0) or 0)
    if total <= 0 or done <= 0 or self._download_start_t is None:
      return ""
    elapsed = max(0.1, time.monotonic() - self._download_start_t)
    rate = (done - self._download_start_done) / elapsed
    if rate <= 1e-3:
      return ""
    remaining = max(0, total - done) / rate
    return self._format_duration(int(remaining))

  def _elapsed_text(self) -> str:
    if self._download_start_t is None:
      return ""
    return self._format_duration(int(time.monotonic() - self._download_start_t))

  def _on_download(self):
    if self._download_active():
      return
    self._ensure_default_maps_selected()
    self._params.remove("OSMDownloadProgress")
    self._params_memory.put("OSMDownloadLocations", self._params.get("MapsSelected") or self._default_maps_selected())

  def _on_cancel(self):
    self._params_memory.remove("OSMDownloadLocations")
    try:
      subprocess.run(["pkill", "mapd"], check=False)
    except Exception:
      pass

  def _on_remove_prompt(self):
    dialog = ConfirmDialog(tr("Are you sure you want to delete all downloaded offline maps?"), tr("Remove"))
    gui_app.set_modal_overlay(dialog, callback=self._on_remove_confirmed)

  def _on_remove_confirmed(self, result: int):
    if result != DialogResult.CONFIRM:
      return
    self._params_memory.remove("OSMDownloadLocations")
    try:
      shutil.rmtree(OSM_OFFLINE_DIR, ignore_errors=True)
    except Exception:
      pass
    try:
      subprocess.run(["pkill", "mapd"], check=False)
    except Exception:
      pass
    self._params.remove("OSMDownloadProgress")

  def _update_state(self):
    now = time.monotonic()
    active = self._download_active()
    if active and self._download_start_t is None:
      p = self._progress()
      self._download_start_t = now
      self._download_start_done = int(p.get("downloaded_files", 0) or 0)
      self._download_start_bytes = self._local_bytes()
      self._download_last_sample_t = now
      self._download_last_sample_bytes = self._download_start_bytes

    if active and not self._prev_download_active:
      p = self._progress()
      self._download_start_t = now
      self._download_start_done = int(p.get("downloaded_files", 0) or 0)
      self._download_start_bytes = self._local_bytes()
      self._download_last_sample_t = now
      self._download_last_sample_bytes = self._download_start_bytes
      self._download_rate_bps = 0.0

    if active and self._download_start_t is not None:
      # Estimate download throughput using local disk growth (1s cached).
      current_bytes = self._local_bytes()
      if now - self._download_last_sample_t >= 1.0:
        delta_bytes = max(0, current_bytes - self._download_last_sample_bytes)
        dt = max(0.001, now - self._download_last_sample_t)
        self._download_rate_bps = delta_bytes / dt
        self._download_last_sample_t = now
        self._download_last_sample_bytes = current_bytes

    if self._prev_download_active and not active:
      self._download_start_t = None
      self._download_start_done = 0
      self._download_start_bytes = 0
      self._download_rate_bps = 0.0
      self._download_last_sample_t = 0.0
      self._download_last_sample_bytes = 0
    self._prev_download_active = active
