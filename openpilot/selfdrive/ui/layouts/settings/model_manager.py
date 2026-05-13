from __future__ import annotations

import re
import time
from collections.abc import Callable
from pathlib import Path

import pyray as rl

from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.confirm_dialog import alert_dialog, ConfirmDialog
from openpilot.system.ui.widgets.list_view import ListItem, BaseSpinBoxAction, button_item, toggle_item
from openpilot.system.ui.widgets.progress_bar import progress_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.tree_dialog import TreeOptionDialog, TreeNode, TreeFolder


class ParamSpinBoxAction(BaseSpinBoxAction):
  def __init__(self, params: Params, param_key: str, min_val: float, max_val: float, step: float,
               label_callback: Callable[[float], str], enabled: bool | Callable[[], bool] = True, width: int = 500):
    self._params = params
    self._param_key = param_key
    self._min_val = min_val
    self._max_val = max_val
    self._step = step
    self._label_callback = label_callback
    self._value = self._read_param()
    super().__init__(callback=None, enabled=enabled, width=width)

  def _read_param(self) -> float:
    raw = self._params.get(self._param_key, return_default=True)
    try:
      value = float(raw)
    except (TypeError, ValueError):
      value = self._min_val
    return max(self._min_val, min(self._max_val, value))

  def sync_from_params(self) -> None:
    self._value = self._read_param()

  def set_step(self, step: float) -> None:
    self._step = step

  def _set_value(self, value: float) -> None:
    value = max(self._min_val, min(self._max_val, value))
    if value != self._value:
      self._value = value
      self._params.put(self._param_key, value)

  def _on_minus(self):
    self._set_value(self._value - self._step)

  def _on_plus(self):
    self._set_value(self._value + self._step)

  def _get_minus_enabled(self) -> bool:
    return self._value > self._min_val + 1e-6

  def _get_plus_enabled(self) -> bool:
    return self._value < self._max_val - 1e-6

  def _get_display_text(self) -> str:
    if self._label_callback:
      return self._label_callback(self._value)
    return f"{self._value:.2f}"


class ModelManagerLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self.model_manager = None
    self.download_status = None
    self.prev_download_status = None
    self.model_dialog = None
    self.last_cache_calc_time = 0.0

    self._initialize_items()
    self._scroller = Scroller(self.items, line_separator=True, spacing=0)

  def _initialize_items(self) -> None:
    self.current_model_item = button_item(tr("Current Model"), tr("SELECT"), "", self._handle_current_model_clicked)

    self.supercombo_label = progress_item(tr("Driving Model"))
    self.vision_label = progress_item(tr("Vision Model"))
    self.policy_label = progress_item(tr("Policy Model"))

    self.refresh_item = button_item(
      tr("Refresh Model List"),
      tr("REFRESH"),
      "",
      lambda: (self._params.put("ModelManager_LastSyncTime", 0),
               gui_app.set_modal_overlay(alert_dialog(tr("Fetching Latest Models")))),
    )

    self.clear_cache_item = button_item(
      tr("Clear Model Cache"),
      tr("CLEAR"),
      "",
      self._clear_cache,
    )
    self.clear_cache_item.action_item.set_value(f"{self._calculate_cache_size():.2f} MB")

    self.cancel_download_item = button_item(tr("Cancel Download"), tr("Cancel"), "", self._cancel_download)

    self._lane_turn_action = ParamSpinBoxAction(
      self._params,
      "LaneTurnValue",
      5.0,
      20.0,
      1.0,
      self._format_lane_turn_value,
    )
    self.lane_turn_value_control = ListItem(
      title=tr("Adjust Lane Turn Speed"),
      description=tr("Set the maximum speed for lane turn desires. Default is 19 mph."),
      action_item=self._lane_turn_action,
    )

    self.lane_turn_desire_toggle = toggle_item(
      tr("Use Lane Turn Desires"),
      tr("If you're driving at 20 mph (32 km/h) or below and have your blinker on, the car will plan a turn at the nearest drivable path."),
      callback=lambda val: self._params.put_bool("LaneTurnDesire", val),
    )

    self._lagd_delay_action = ParamSpinBoxAction(
      self._params,
      "LagdToggleDelay",
      0.05,
      0.50,
      0.01,
      lambda v: f"{v:.2f}s",
    )
    self.delay_control = ListItem(
      title=tr("Adjust Software Delay"),
      description=tr("Adjust the software delay when Live Learning Steer Delay is toggled off. The default software delay value is 0.2"),
      action_item=self._lagd_delay_action,
    )

    self.lagd_toggle = toggle_item(
      tr("Live Learning Steer Delay"),
      "",
      callback=lambda val: self._params.put_bool("LagdToggle", val),
    )

    self.items = [
      self.current_model_item,
      self.cancel_download_item,
      self.supercombo_label,
      self.vision_label,
      self.policy_label,
      self.refresh_item,
      self.clear_cache_item,
      self.lane_turn_desire_toggle,
      self.lane_turn_value_control,
      self.lagd_toggle,
      self.delay_control,
    ]

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def _format_lane_turn_value(self, value: float) -> str:
    if ui_state.is_metric:
      return f"{int(round(value * CV.MPH_TO_KPH))} km/h"
    return f"{int(round(value))} mph"

  def _is_downloading(self) -> bool:
    return (self.model_manager and self.model_manager.selectedBundle and
            self.model_manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.downloading)

  def _calculate_cache_size(self) -> float:
    cache_size = 0.0
    model_root = Path(Paths.model_root())
    if model_root.exists():
      for entry in model_root.iterdir():
        if entry.is_file():
          try:
            cache_size += entry.stat().st_size
          except OSError:
            pass
    return cache_size / (1024**2)

  def _clear_cache(self) -> None:
    def _callback(response):
      if response == DialogResult.CONFIRM:
        self._params.put_bool("ModelManager_ClearCache", True)
        self.clear_cache_item.action_item.set_value(f"{self._calculate_cache_size():.2f} MB")

    gui_app.set_modal_overlay(ConfirmDialog(tr("This will delete ALL downloaded models from the cache except the currently active model. Are you sure?"),
                                            tr("Clear Cache")), callback=_callback)

  def _cancel_download(self) -> None:
    self._params.remove("ModelManager_DownloadIndex")

  def _handle_bundle_download_progress(self) -> None:
    labels = {
      custom.ModelManagerSP.Model.Type.supercombo: self.supercombo_label,
      custom.ModelManagerSP.Model.Type.vision: self.vision_label,
      custom.ModelManagerSP.Model.Type.policy: self.policy_label,
    }
    for label in labels.values():
      label.set_visible(False)
    self.cancel_download_item.set_visible(False)

    if not self.model_manager or (not self.model_manager.selectedBundle and not self.model_manager.activeBundle):
      return

    bundle = self.model_manager.selectedBundle if self._is_downloading() or (
      self.model_manager.selectedBundle and self.model_manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.failed
    ) else self.model_manager.activeBundle
    if not bundle:
      return

    self.download_status = bundle.status
    status_changed = self.prev_download_status != self.download_status
    self.prev_download_status = self.download_status

    self.cancel_download_item.set_visible(bool(self.model_manager.selectedBundle) and self._params.get("ModelManager_DownloadIndex") is not None)

    if (current_time := time.monotonic()) - self.last_cache_calc_time > 0.5:
      self.last_cache_calc_time = current_time
      self.clear_cache_item.action_item.set_value(f"{self._calculate_cache_size():.2f} MB")

    if self.download_status == custom.ModelManagerSP.DownloadStatus.downloading:
      device.reset_interactive_timeout(300)

    for model in bundle.models:
      model_type = getattr(model.type, 'raw', model.type)
      if label := labels.get(model_type):
        label.set_visible(True)
        progress = model.artifact.downloadProgress
        text = f"pending - {bundle.displayName}"
        show = False
        color = rl.GRAY
        if progress.status == custom.ModelManagerSP.DownloadStatus.downloading:
          text = f"{int(progress.progress)}% - {bundle.displayName}"
          show = True
        elif progress.status in (custom.ModelManagerSP.DownloadStatus.downloaded, custom.ModelManagerSP.DownloadStatus.cached):
          status_text = tr("from cache") if progress.status == custom.ModelManagerSP.DownloadStatus.cached else tr("downloaded")
          text = f"{bundle.displayName} - {status_text if status_changed else tr('ready')}"
          color = rl.Color(28, 101, 186, 255)
        elif progress.status == custom.ModelManagerSP.DownloadStatus.failed:
          text = f"{tr('download failed')} - {bundle.displayName}"
          color = rl.RED
        label.action_item.update(progress.progress, text, show, color)

  def _show_reset_params_dialog(self) -> None:
    def _callback(response):
      if response == DialogResult.CONFIRM:
        self._params.remove("CalibrationParams")
        self._params.remove("LiveTorqueParameters")
    msg = tr("Model download has started in the background. We suggest resetting calibration. Would you like to do that now?")
    gui_app.set_modal_overlay(ConfirmDialog(msg, tr("Reset Calibration")), callback=_callback)

  def _on_model_selected(self, result) -> None:
    if result != DialogResult.CONFIRM:
      return
    selected_ref = self.model_dialog.selection_ref
    if selected_ref == "Default":
      self._params.remove("ModelManager_ActiveBundle")
      self._params.put("ModelRunnerTypeCache", int(custom.ModelManagerSP.Runner.stock))
      self._show_reset_params_dialog()
    elif self.model_manager:
      selected_bundle = next((bundle for bundle in self.model_manager.availableBundles if bundle.ref == selected_ref), None)
      if selected_bundle:
        self._params.put("ModelManager_DownloadIndex", selected_bundle.index)
        if self.model_manager.activeBundle and selected_bundle.generation != self.model_manager.activeBundle.generation:
          self._show_reset_params_dialog()
    self.model_dialog = None

  @staticmethod
  def _bundle_to_node(bundle):
    return TreeNode(bundle.ref, {'display_name': bundle.displayName, 'short_name': bundle.internalName})

  def _get_folders(self, favorites):
    bundles = self.model_manager.availableBundles
    folders = {}
    for bundle in bundles:
      folder_name = ""
      for ov_ride in bundle.overrides:
        if ov_ride.key == "folder":
          folder_name = ov_ride.value
          break
      folders.setdefault(folder_name, []).append(bundle)

    folders_list = [TreeFolder("", [TreeNode("Default", {'display_name': tr("Default Model"), 'short_name': "Default"})])]
    for folder, folder_bundles in sorted(folders.items(), key=lambda x: max((bundle.index for bundle in x[1]), default=-1), reverse=True):
      folder_bundles.sort(key=lambda bundle: bundle.index, reverse=True)
      name = folder
      if folder_bundles and (m := re.search(r'\\(([^)]*)\\)[^(]*$', folder_bundles[0].displayName)):
        name = f"{folder} - (Updated: {m.group(1)})"
      folders_list.append(TreeFolder(name, [self._bundle_to_node(bundle) for bundle in folder_bundles]))

    if favorites and (fav_bundles := [bundle for bundle in bundles if bundle.ref in favorites]):
      folders_list.insert(1, TreeFolder(tr("Favorites"), [self._bundle_to_node(bundle) for bundle in fav_bundles]))
    return folders_list

  def _handle_current_model_clicked(self) -> None:
    favs = self._params.get("ModelManager_Favs")
    favorites = set(favs.split(';')) if favs else set()
    folders_list = self._get_folders(favorites)

    active_ref = self.model_manager.activeBundle.ref if self.model_manager and self.model_manager.activeBundle else "Default"
    self.model_dialog = TreeOptionDialog(tr("Select a Model"), folders_list, active_ref, "ModelManager_Favs",
                                         get_folders_fn=self._get_folders, on_exit=self._on_model_selected)
    gui_app.set_modal_overlay(self.model_dialog, callback=self._on_model_selected)

  def _update_lagd_description(self, lagd_toggle: bool) -> None:
    desc = tr("Enable this for the car to learn and adapt its steering response time. Disable to use a fixed steering response time. Keeping this on provides the stock openpilot experience.")
    if lagd_toggle:
      try:
        live_delay = ui_state.sm["liveDelay"].lateralDelay
        desc += f"<br>{tr('Live Steer Delay:')} {live_delay:.3f} s"
      except Exception:
        pass
    elif ui_state.CP:
      sw = float(self._params.get("LagdToggleDelay", return_default=True))
      cp = ui_state.CP.steerActuatorDelay
      desc += f"<br>{tr('Actuator Delay:')} {cp:.2f} s + {tr('Software Delay:')} {sw:.2f} s = {tr('Total Delay:')} {cp + sw:.2f} s"
    self.lagd_toggle.set_description(desc)

  def _update_state(self) -> None:
    advanced_controls = self._params.get_bool("ShowAdvancedControls")
    turn_desire = self._params.get_bool("LaneTurnDesire")
    live_delay = self._params.get_bool("LagdToggle")

    self.lane_turn_desire_toggle.action_item.set_state(turn_desire)
    self.lane_turn_value_control.set_visible(turn_desire and advanced_controls)
    self.lagd_toggle.action_item.set_state(live_delay)
    self.delay_control.set_visible(not live_delay and advanced_controls)

    lane_turn_step = 1.0 / CV.MPH_TO_KPH if ui_state.is_metric else 1.0
    self._lane_turn_action.set_step(lane_turn_step)
    self._lane_turn_action.sync_from_params()
    self._lagd_delay_action.sync_from_params()

    self._update_lagd_description(live_delay)
    self.model_manager = ui_state.sm["modelManagerSP"]
    self._handle_bundle_download_progress()

    active_name = tr("Default Model")
    if self.model_manager and self.model_manager.activeBundle and self.model_manager.activeBundle.ref:
      active_name = self.model_manager.activeBundle.internalName or self.model_manager.activeBundle.displayName or active_name
    self.current_model_item.action_item.set_value(active_name)

    if not ui_state.is_offroad():
      self.current_model_item.action_item.set_enabled(False)
      self.current_model_item.set_description(tr("Only available when vehicle is off, or always offroad mode is on"))
    else:
      self.current_model_item.action_item.set_enabled(True)
      self.current_model_item.set_description("")
