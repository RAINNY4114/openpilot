import os

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import (
  multiple_button_item,
  simple_item,
  double_spin_button_item,
  spin_button_item,
  toggle_item,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller

OSM_OFFLINE_DIR = "/data/media/0/osm/offline"


class LincolnLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    self._normalize_curve_method()

    self._curve_method_setting = multiple_button_item(
      title=lambda: tr("Curve Detection Method"),
      description=lambda: tr("How curves are detected. <b>Map-Based</b> uses downloaded map data to identify curves and determine the appropriate speed in which to handle them at, while <b>Vision</b> relies solely on the driving model. <b>Map + Vision</b> uses both and applies the safer (lower) target speed."),
      buttons=[lambda: tr("Map Based"), lambda: tr("Vision"), lambda: tr("Map + Vision")],
      selected_index=self._curve_method_index(),
      button_width=255,
      callback=self._on_curve_method_selected,
    )

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
      simple_item(title=lambda: tr("### Human Turn Detection ###")),
      toggle_item(
        title=lambda: tr("Enable Human Turn Detection"),
        description=lambda: tr("Automatically pause steering when the driver applies large manual steering input, then smoothly resume."),
        initial_state=self._params.get_bool("dp_htd_enabled"),
        callback=lambda val: self._params.put_bool("dp_htd_enabled", val),
      ),
      spin_button_item(
        title=lambda: tr("Trigger angle"),
        description=lambda: tr("Driver steering angle that triggers HTD (degrees)."),
        initial_value=self._get_param_int("dp_htd_turn_angle_threshold", 90),
        callback=lambda val: self._params.put("dp_htd_turn_angle_threshold", int(val)),
        min_val=30,
        max_val=120,
        step=1,
        suffix=tr(" °"),
      ),
      
      simple_item(title=lambda: tr("### Curve Speed Control ###")),
      toggle_item(
        title=lambda: tr("Curve Speed Control"),
        description=lambda: tr("Automatically slow down for upcoming curves using downloaded maps or the driving model."),
        initial_state=self._params.get_bool("CurveSpeedControl"),
        callback=lambda val: self._params.put_bool("CurveSpeedControl", val),
      ),
      self._curve_method_setting,
      spin_button_item(
        title=lambda: tr("Curve Detection Sensitivity"),
        description=lambda: tr("How sensitive openpilot is when detecting curves. Higher values trigger earlier responses at the risk of triggering too often, while lower values increase confidence at the risk of triggering too infrequently."),
        initial_value=self._get_param_int("CurveSensitivity", 200),
        callback=lambda val: self._params.put("CurveSensitivity", int(val)),
        min_val=50,
        max_val=200,
        step=5,
        suffix=tr(" %"),
      ),
      spin_button_item(
        title=lambda: tr("Curve Speed Aggressiveness"),
        description=lambda: tr("How aggressive openpilot is when navigating through curves. Higher values result in faster turns but may reduce comfort or stability, while lower values result in slower, smoother turns at the risk of being overly cautious."),
        initial_value=self._get_param_int("TurnAggressiveness", 100),
        callback=lambda val: self._params.put("TurnAggressiveness", int(val)),
        min_val=50,
        max_val=200,
        step=5,
        suffix=tr(" %"),
      ),
      simple_item(title=lambda: tr("### Follow-Coast (Traffic) ###")),
      toggle_item(
        title=lambda: tr("Follow-Coast (Traffic)"),
        description=lambda: tr("At low speeds when following a lead, suppress very gentle braking if the lead is pulling away to smooth stop-and-go."),
        initial_state=self._params.get_bool("dp_lincoln_follow_coast"),
        callback=lambda val: self._params.put_bool("dp_lincoln_follow_coast", val),
      ),
      simple_item(title=lambda: tr("### Following & Stopping ###")),
      double_spin_button_item(
        title=lambda: tr("Stop distance (standstill)"),
        description=lambda: tr("Target gap to the lead vehicle when coming to a stop (stop-and-go / red lights). Lower = closer; higher = more buffer."),
        initial_value=self._get_param_float("dp_lincoln_stop_distance_m", 4.0),
        callback=lambda val: self._params.put("dp_lincoln_stop_distance_m", float(val)),
        min_val=3.0,
        max_val=8.0,
        step=0.5,
        decimals=1,
        suffix=tr(" m"),
      ),
      simple_item(title=lambda: tr("### Obstacle Avoidance (Experimental) ###")),
      toggle_item(
        title=lambda: tr("Cone Detection (Experimental)"),
        description=lambda: tr("Detect traffic cones ahead and publish results for UI/debug and future features."),
        initial_state=self._params.get_bool("dp_lat_cone_detection"),
        callback=lambda val: self._params.put_bool("dp_lat_cone_detection", val),
      ),
      toggle_item(
        title=lambda: tr("Auto avoidance"),
        description=lambda: tr("When cones/vehicles are detected in-path, automatically slow down then initiate a lane change to pass, and return when clear. Pedestrians trigger a stop (no auto lane change). Experimental and requires blindspot sensors."),
        initial_state=self._params.get_bool("dp_lincoln_auto_avoid"),
        callback=lambda val: self._params.put_bool("dp_lincoln_auto_avoid", val),
      ),
      toggle_item(
        title=lambda: tr("Auto overtake"),
        description=lambda: tr("Highway-only: when a slower lead vehicle is detected ahead and the passing lane is clear, automatically initiate a lane change to pass, and return when clear. Experimental and requires blindspot sensors."),
        initial_state=self._params.get_bool("dp_lincoln_auto_overtake"),
        callback=lambda val: self._params.put_bool("dp_lincoln_auto_overtake", val),
      ),
      spin_button_item(
        title=lambda: tr("Auto lane-change min speed"),
        description=lambda: tr("Minimum cruise set speed required for auto overtake (km/h)."),
        initial_value=self._get_param_int("dp_lincoln_auto_overtake_min_cruise_kph", 90),
        callback=lambda val: self._params.put("dp_lincoln_auto_overtake_min_cruise_kph", int(val)),
        min_val=60,
        max_val=140,
        step=1,
        suffix=tr(" km/h"),
      ),
      spin_button_item(
        title=lambda: tr("Auto lane-change confirm delay"),
        description=lambda: tr("Seconds to wait after voice + auto blinker before starting an automatic lane change."),
        initial_value=self._get_param_int("dp_lincoln_auto_lc_confirm_delay_sec", 3),
        callback=lambda val: self._params.put("dp_lincoln_auto_lc_confirm_delay_sec", int(val)),
        min_val=0,
        max_val=10,
        step=1,
        suffix=tr(" sec"),
      ),
      double_spin_button_item(
        title=lambda: tr("Auto LC edge clearance"),
        description=lambda: tr("Block automatic lane changes when the road-edge is too close to the current lane boundary (manual signal not affected)."),
        initial_value=self._get_param_float("dp_lincoln_auto_lc_edge_clearance_m", 0.6),
        callback=lambda val: self._params.put("dp_lincoln_auto_lc_edge_clearance_m", f"{val:.1f}"),
        min_val=0.3,
        max_val=2.0,
        step=0.1,
        decimals=1,
        suffix=tr(" m"),
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._normalize_curve_method()
    self._curve_method_setting.action_item.set_selected_button(self._curve_method_index())
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()

  def _has_offline_maps(self) -> bool:
    try:
      if not os.path.isdir(OSM_OFFLINE_DIR):
        return False
      with os.scandir(OSM_OFFLINE_DIR) as it:
        return any(True for _ in it)
    except Exception:
      return False

  def _normalize_curve_method(self) -> None:
    map_enabled = bool(self._params.get_bool("MapTurnControl"))
    vision_enabled = bool(self._params.get_bool("VisionTurnControl"))
    has_maps = self._has_offline_maps()

    if not has_maps and map_enabled:
      self._params.put_bool("MapTurnControl", False)
      self._params.put_bool("VisionTurnControl", True)
      return

    if not map_enabled and not vision_enabled:
      if has_maps:
        self._params.put_bool("MapTurnControl", True)
        self._params.put_bool("VisionTurnControl", False)
      else:
        self._params.put_bool("MapTurnControl", False)
        self._params.put_bool("VisionTurnControl", True)

  def _curve_method_index(self) -> int:
    map_enabled = bool(self._params.get_bool("MapTurnControl"))
    vision_enabled = bool(self._params.get_bool("VisionTurnControl"))
    if map_enabled and vision_enabled:
      return 2
    if map_enabled and not vision_enabled:
      return 0
    if vision_enabled and not map_enabled:
      return 1
    return 0 if self._has_offline_maps() else 1

  def _on_curve_method_selected(self, index: int) -> None:
    if index == 0:
      if not self._has_offline_maps():
        dlg = ConfirmDialog(tr("The <b>Map Based</b> options are only available when some <b>Map Data</b> has been downloaded!"),
                            tr("OK"), cancel_text="", rich=True)
        gui_app.set_modal_overlay(dlg)
        self._curve_method_setting.action_item.set_selected_button(1)
        self._params.put_bool("MapTurnControl", False)
        self._params.put_bool("VisionTurnControl", True)
        return
      self._params.put_bool("MapTurnControl", True)
      self._params.put_bool("VisionTurnControl", False)
    elif index == 1:
      self._params.put_bool("MapTurnControl", False)
      self._params.put_bool("VisionTurnControl", True)
    else:
      if not self._has_offline_maps():
        dlg = ConfirmDialog(tr("The <b>Map Based</b> options are only available when some <b>Map Data</b> has been downloaded!"),
                            tr("OK"), cancel_text="", rich=True)
        gui_app.set_modal_overlay(dlg)
        self._curve_method_setting.action_item.set_selected_button(1)
        self._params.put_bool("MapTurnControl", False)
        self._params.put_bool("VisionTurnControl", True)
        return
      self._params.put_bool("MapTurnControl", True)
      self._params.put_bool("VisionTurnControl", True)

  @staticmethod
  def _safe_int(val: bytes | str | None, default: int) -> int:
    if not val:
      return default
    try:
      return int(val)
    except (TypeError, ValueError):
      return default

  @staticmethod
  def _safe_float(val: bytes | str | None, default: float) -> float:
    if not val:
      return default
    try:
      if isinstance(val, bytes):
        val = val.decode("utf-8", errors="ignore")
      return float(val)
    except (TypeError, ValueError):
      return default

  def _get_param_int(self, key: str, default: int) -> int:
    return self._safe_int(self._params.get(key), default)

  def _get_param_float(self, key: str, default: float) -> float:
    return self._safe_float(self._params.get(key), default)
