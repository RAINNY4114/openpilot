import os
import subprocess
import sys
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import alert_dialog
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.list_view import (
  button_item,
  dual_button_item,
  simple_item,
  spin_button_item,
  text_item,
  toggle_item,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller

DEFAULT_DEST = "NAS@192.168.50.200:/volume1/openpilot"
DEFAULT_PORT = "22"
DEFAULT_KEY = ""
DEFAULT_STATUS = "Waiting for action"
NAS_SCRIPT = os.path.join(BASEDIR, "selfdrive", "ui", "tools", "lincoln_media_manager.py")


class LincolnLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._keyboard = Keyboard()
    self._nas_pending_steps: list[str] = []
    self._nas_action_inflight = False

    self._nas_summary()  # ensure defaults written

    self._scroller = Scroller([
      simple_item(title=lambda: tr("### Lincoln Blindspot Voice Alerts ###")),
      toggle_item(
        title=lambda: tr("Blindspot Voice Alert"),
        description=lambda: tr("Play left/right voice prompts when blindspot sensors detect a vehicle."),
        initial_state=self._params.get_bool("dp_lincoln_bsm_voice_enabled"),
        callback=lambda val: self._params.put_bool("dp_lincoln_bsm_voice_enabled", val),
      ),
      spin_button_item(
        title=lambda: tr("Voice repeat interval"),
        description=lambda: tr("Minimum seconds between consecutive blindspot alerts."),
        initial_value=int(self._params.get("dp_lincoln_bsm_voice_interval_sec") or 3),
        callback=lambda val: self._params.put("dp_lincoln_bsm_voice_interval_sec", int(val)),
        min_val=1,
        max_val=10,
        step=1,
        suffix=tr(" sec"),
      ),
      spin_button_item(
        title=lambda: tr("Voice volume"),
        description=lambda: tr("Playback volume for blindspot alerts (percentage)."),
        initial_value=int(self._params.get("dp_lincoln_bsm_voice_volume_pct") or 100),
        callback=lambda val: self._params.put("dp_lincoln_bsm_voice_volume_pct", int(val)),
        min_val=20,
        max_val=100,
        step=5,
        suffix=tr(" %"),
      ),
      button_item(
        title=lambda: tr("NAS (Synology) configuration"),
        button_text=lambda: tr("Edit"),
        description=lambda: self._nas_overview(),
        callback=self._on_nas_configure,
      ),
      text_item(
        title=lambda: tr("NAS status"),
        value=lambda: self._nas_status(),
        description=lambda: tr("Last result of NAS upload/delete operations."),
      ),
      dual_button_item(
        left_text=lambda: tr("Upload recordings"),
        right_text=lambda: tr("Delete local recordings"),
        left_callback=self._on_nas_upload,
        right_callback=self._on_nas_delete,
        description=lambda: tr("Upload recordings from /data/media/0/realdata to NAS via SCP, or delete them locally. Files are read-only to other users (0600) by default."),
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()

  # --- Blindspot helpers remain handled by params ---

  # --- NAS helpers ---

  def _nas_overview(self) -> str:
    destination = self._nas_summary()
    port = self._params.get("NasSshPort") or DEFAULT_PORT
    key = self._params.get("NasSshKey") or ""
    key_display = key if key else tr("System default (copy custom keys to /data/openpilot/ssh/ and enter full path)")
    return tr("Destination: {dest} · Port: {port} · Key: {key}").format(dest=destination, port=port, key=key_display)

  def _nas_status(self) -> str:
    status = self._params.get("LincolnNASLastResult")
    if not status:
      status = tr(DEFAULT_STATUS)
      self._params.put("LincolnNASLastResult", status)
    return status

  def _nas_summary(self) -> str:
    dest = self._params.get("NasSshDest") or DEFAULT_DEST
    port = self._params.get("NasSshPort") or DEFAULT_PORT
    key = self._params.get("NasSshKey")
    if key is None:
      key = DEFAULT_KEY
    self._params.put("NasSshDest", dest)
    self._params.put("NasSshPort", port)
    self._params.put("NasSshKey", key)

    user, host, remote_path = self._parse_dest(dest)
    self._params.put("LincolnNASAddress", host)
    self._params.put("LincolnNASUsername", user)
    self._params.put("LincolnNASPassword", "")
    formatted = f"{host}:{remote_path}" if remote_path else host
    return f"{formatted} (user: {user})"

  @staticmethod
  def _parse_dest(dest: str) -> tuple[str, str, str]:
    user = "NAS"
    host = "unknown"
    remote_path = ""
    if ":" in dest:
      target, remote_path = dest.split(":", 1)
    else:
      target = dest
    if "@" in target:
      user, host = target.split("@", 1)
    else:
      host = target
    remote_path = remote_path or "/"
    return user or "NAS", host or "unknown", remote_path

  def _on_nas_configure(self):
    self._nas_pending_steps = ["dest", "port", "key"]
    self._show_next_nas_keyboard()

  def _show_next_nas_keyboard(self):
    if not self._nas_pending_steps:
      gui_app.set_modal_overlay(None)
      return

    step = self._nas_pending_steps.pop(0)
    titles = {
      "dest": tr("Edit NAS destination"),
      "port": tr("Edit NAS SSH port"),
      "key": tr("Edit NAS SSH key path"),
    }
    subtitles = {
      "dest": tr("Use format user@host:/volume/path"),
      "port": tr("Default 22 if empty."),
      "key": tr("Leave empty for system default. For custom keys copy the private key to /data/openpilot/ssh/ and enter the full path (e.g. /data/openpilot/ssh/id_nas)."),
    }
    current_values = {
      "dest": self._params.get("NasSshDest") or DEFAULT_DEST,
      "port": self._params.get("NasSshPort") or DEFAULT_PORT,
      "key": self._params.get("NasSshKey") or "",
    }

    self._keyboard.reset(min_text_size=1 if step != "key" else 0)
    self._keyboard.set_title(titles[step], subtitles[step])
    self._keyboard.set_text(current_values[step])
    gui_app.set_modal_overlay(self._keyboard, callback=lambda result, step=step: self._on_nas_keyboard_done(step, result))

  def _on_nas_keyboard_done(self, step: str, result: DialogResult):
    if result != DialogResult.CONFIRM:
      self._nas_pending_steps.clear()
      return

    text = self._keyboard.text.strip()
    if step == "dest":
      if not self._validate_dest(text):
        gui_app.set_modal_overlay(alert_dialog(tr("Invalid destination format. Use user@host:/volume/path.")))
        self._nas_pending_steps.insert(0, "dest")
        return
      self._params.put("NasSshDest", text)
    elif step == "port":
      if not text:
        text = DEFAULT_PORT
      if not text.isdigit():
        gui_app.set_modal_overlay(alert_dialog(tr("Port must be a number.")))
        self._nas_pending_steps.insert(0, "port")
        return
      self._params.put("NasSshPort", text)
    elif step == "key":
      self._params.put("NasSshKey", text)

    self._nas_summary()
    if self._nas_pending_steps:
      self._show_next_nas_keyboard()

  @staticmethod
  def _validate_dest(dest: str) -> bool:
    return bool(dest) and "@" in dest and ":" in dest

  def _on_nas_upload(self):
    self._run_nas_command(["--upload"])

  def _on_nas_delete(self):
    self._run_nas_command(["--delete"])

  def _run_nas_command(self, extra_args: list[str]):
    if self._nas_action_inflight:
      gui_app.set_modal_overlay(alert_dialog(tr("NAS task already running.")))
      return

    if not os.path.exists(NAS_SCRIPT):
      self._params.put("LincolnNASLastResult", tr("NAS helper script missing."))
      return

    self._nas_action_inflight = True
    self._params.put("LincolnNASLastResult", tr("Processing NAS request..."))

    def _worker():
      try:
        subprocess.run([sys.executable, NAS_SCRIPT] + extra_args, check=True)
      except subprocess.CalledProcessError as e:
        self._params.put("LincolnNASLastResult", tr("NAS command failed (code {}).").format(e.returncode))
      except Exception as e:
        self._params.put("LincolnNASLastResult", tr("NAS command failed: {}").format(e))
      finally:
        self._nas_action_inflight = False

    threading.Thread(target=_worker, daemon=True).start()
