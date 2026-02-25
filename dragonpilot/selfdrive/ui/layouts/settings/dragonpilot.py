import os
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item, simple_item, button_item, spin_button_item, double_spin_button_item, text_spin_button_item
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog

LITE = os.getenv("LITE") is not None

class DragonpilotLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._scroller: Scroller | None = None
    self._has_long_ctrl = False
    self._has_radar_unavailable = False

    self._toggles = {}

    if ui_state.CP is not None:
      self._has_long_ctrl = ui_state.CP.openpilotLongitudinalControl
      self._has_radar_unavailable = ui_state.CP.radarUnavailable
      match ui_state.CP.brand:
        case "toyota":
          self._toyota_toggles()
        case "volkswagen":
          self._vag_toggles()
        case "mazda":
          self._mazda_toggles()

    self._lat_toggles()
    self._lon_toggles()
    self._ui_toggles()
    self._device_toggles()

    self._reset_dp_conf_btn = button_item(
      lambda: tr("Reset DP Settings"),
      lambda: tr("RESET"),
      lambda: tr("Reset dragonpilot settings to default and restart the device."),
      callback=self._reset_dp_conf)

    self._toggles['btn_reset_dp_conf'] = self._reset_dp_conf_btn

    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

  def _toyota_toggles(self):
    self._toggles["title_toyota"] = simple_item(title=lambda: tr("### Toyota / Lexus ###"))

    self._toggles["dp_toyota_door_auto_lock_unlock"] = toggle_item(
      title=lambda: tr("Door Auto Lock/Unlock"),
      description=lambda: tr("Enable openpilot to auto-lock doors above 20 km/h and auto-unlock when shifting to Park."),
      initial_state=self._params.get_bool("dp_toyota_door_auto_lock_unlock"),
      callback=lambda val: self._params.put_bool("dp_toyota_door_auto_lock_unlock", val),
    )

    self._toggles["dp_toyota_tss1_sng"] = toggle_item(
      title=lambda: tr("Enable TSS1 SnG Mod"),
      description=lambda: "",
      initial_state=self._params.get_bool("dp_toyota_tss1_sng"),
      callback=lambda val: self._params.put_bool("dp_toyota_tss1_sng", val),
    )

    self._toggles["dp_toyota_stock_lon"] = toggle_item(
      title=lambda: tr("Use Stock Longitudinal Control"),
      description=lambda: "",
      initial_state=self._params.get_bool("dp_toyota_stock_lon"),
      callback=lambda val: self._params.put_bool("dp_toyota_stock_lon", val),
    )

  def _vag_toggles(self):
    self._toggles["title_vag"] = simple_item(title=lambda: tr("### VAG ###"))

    self._toggles["dp_vag_a0_sng"] = toggle_item(
      title=lambda: tr("MQB A0 SnG Mod"),
      description=lambda: "",
      initial_state=self._params.get_bool("dp_vag_a0_sng"),
      callback=lambda val: self._params.put_bool("dp_vag_a0_sng", val),
    )

    self._toggles["dp_vag_pq_steering_patch"] = toggle_item(
      title=lambda: tr("PQ Steering Patch"),
      description=lambda: "",
      initial_state=self._params.get_bool("dp_vag_pq_steering_patch"),
      callback=lambda val: self._params.put_bool("dp_vag_pq_steering_patch", val),
    )

    self._toggles["dp_vag_avoid_eps_lockout"] = toggle_item(
      title=lambda: tr("Avoid EPS Lockout"),
      description=lambda: "",
      initial_state=self._params.get_bool("dp_vag_avoid_eps_lockout"),
      callback=lambda val: self._params.put_bool("dp_vag_avoid_eps_lockout", val),
    )

  def _mazda_toggles(self):
    self._toggles["title_mazda"] = simple_item(title=lambda: tr("### Mazda ###"))

  def _lat_toggles(self):
    self._toggles["title_lat"] = simple_item(title=lambda: tr("### Lateral ###"))

    self._toggles["dp_lat_alka"] = toggle_item(
      title=lambda: tr("Always-on Lane Keeping Assist (ALKA)"),
      description=lambda: tr("Allows openpilot to always steer to keep the car in its lane."),
      initial_state=self._params.get_bool("dp_lat_alka"),
      callback=lambda val: self._params.put_bool("dp_lat_alka", val),
    )

    self._toggles['dp_lat_lca_speed'] = spin_button_item(
      title=lambda: tr('Lane Change Assist At:'),
      description=lambda: tr("Off = Disable LCA.<br>1 mph = 1.2 km/h."),
      initial_value=int(self._params.get("dp_lat_lca_speed") or 20),
      callback=self._on_dp_lat_lca_speed_change,
      min_val=0,
      max_val=100,
      step=5,
      suffix=tr(" mph"),
      special_value_text=tr("Off"),
    )

    self._toggles['dp_lat_lca_auto_sec'] = double_spin_button_item(
      title=lambda: "+ " + tr('Auto Lane Change after:'),
      description=lambda: tr("Off = Disable Auto Lane Change."),
      initial_value=int(self._params.get("dp_lat_lca_auto_sec") or 0.0),
      callback=lambda val: self._params.put("dp_lat_lca_auto_sec", float(val)),
      min_val=0.0,
      max_val=5.0,
      step=0.5,
      suffix=tr(" sec"),
      special_value_text=tr("Off"),
      # enabled=int(self._params.get("dp_lat_lca_speed") or 20) > 0
    )

    self._toggles['dp_lat_road_edge_detection'] = toggle_item(
      title=lambda: tr('Road Edge Detection (RED)'),
      description=lambda: tr("Block lane change assist when the system detects the road edge.<br>NOTE: This will show 'Car Detected in Blindspot' warning."),
      initial_state=self._params.get_bool("dp_lat_road_edge_detection"),
      callback=lambda val: self._params.put_bool("dp_lat_road_edge_detection", val),
    )

  def _lon_toggles(self):
    self._toggles["title_lon"] = simple_item(title=lambda: tr("### Longitudinal ###"))

    self._toggles['dp_lon_ext_radar'] = toggle_item(
      title=lambda: tr("Use External Radar"),
      description=lambda: tr("See https://github.com/eFiniLan/openpilot-ext-radar-addon for more information."),
      initial_state=self._params.get_bool("dp_lon_ext_radar"),
      callback=lambda val: self._params.put_bool("dp_lon_ext_radar", val),
    )


    self._toggles["dp_lon_aem"] = toggle_item(
      title=lambda: tr("Adaptive Experimental Mode (AEM)"),
      description=lambda: tr("Adaptive mode switcher between ACC and Blended based on driving context."),
      initial_state=self._params.get_bool("dp_lon_aem"),
      callback=lambda val: self._params.put_bool("dp_lon_aem", val),
    )

  def _ui_toggles(self):
    self._toggles["title_ui"] = simple_item(title=lambda: tr("### UI ###"))

    self._toggles["dp_ui_hide_hud_speed_kph"] = spin_button_item(
      title=lambda: tr("Hide HUD When Moves above:"),
      description=lambda: tr("To prevent screen burn-in, hide Speed, MAX Speed, and Steering/DM Icons when the car moves.<br>Off = Stock Behavior<br>1 km/h = 0.6 mph"),
      initial_value=int(self._params.get("dp_ui_hide_hud_speed_kph") or 0),
      callback=lambda val: self._params.put("dp_ui_hide_hud_speed_kph", val),
      suffix=tr(" km/h"),
      min_val=0,
      max_val=120,
      step=5,
      special_value_text=tr("Off"),
    )

    self._toggles["dp_ui_rainbow"] = toggle_item(
      title=lambda: tr("Rainbow Driving Path like Tesla"),
      description=lambda: tr("Why not?"),
      initial_state=self._params.get_bool("dp_ui_rainbow"),
      callback=lambda val: self._params.put_bool("dp_ui_rainbow", val),
    )

    self._toggles["dp_ui_lead"] = text_spin_button_item(
      title=lambda: tr("Display Lead Stats"),
      description=lambda: tr("Display the statistics of lead car and/or radar tracking points.<br>Lead: Lead stats only<br>Radar: Radar tracking point stats only<br>All: Lead and Radar stats<br>NOTE: Radar option only works on certain vehicle models."),
      initial_index=int(self._params.get("dp_ui_lead") or 0),
      callback=lambda val: self._params.put("dp_ui_lead", val),
      options=["Off", "Lead", "Radar", "All"]
    )

  def _device_toggles(self):
    self._toggles["title_dev"] = simple_item(title=lambda: tr("### Device ###"))

    if LITE:
      self._toggles["dp_dev_is_rhd"] = toggle_item(
        title=lambda: tr("Enable Right-Hand Drive Mode"),
        description=lambda: tr("Allow openpilot to obey right-hand traffic conventions on right driver seat."),
        initial_state=self._params.get_bool("dp_dev_is_rhd"),
        callback=lambda val: self._params.put_bool("dp_dev_is_rhd", val),
      )

      self._toggles["dp_dev_monitoring_disabled"] = toggle_item(
        title=lambda: tr("Disable Driver Monitoring"),
        description=lambda: tr("USE AT YOUR OWN RISK."),
        initial_state=self._params.get_bool("dp_dev_monitoring_disabled"),
        callback=lambda val: self._params.put_bool("dp_dev_monitoring_disabled", val),
      )

      self._toggles["dp_dev_beep"] = toggle_item(
        title=lambda: tr("Enable Beep (Warning)"),
        description=lambda: tr("Use Buzzer for audiable alerts."),
        initial_state=self._params.get_bool("dp_dev_beep"),
        callback=lambda val: self._params.put_bool("dp_dev_beep", val),
      )

    self._toggles["dp_ui_display_mode"] = text_spin_button_item(
      title=lambda: tr("Display Mode"),
      callback=lambda val: self._params.put("dp_ui_display_mode", val),
      options=["Std.", "MAIN+", "OP+", "MAIN-", "OP-"],
      initial_index=int(self._params.get("dp_ui_display_mode") or 0),
      description=lambda: tr("Std.: Stock behavior.<br>MAIN+: ACC MAIN on = Display ON.<br>OP+: OP enabled = Display ON.<br>MAIN-: ACC MAIN on = Display OFF<br>OP-: OP enabled = Display OFF."),
    )

    if "LITE" not in os.environ:
      self._toggles["dp_dev_audible_alert_mode"] = text_spin_button_item(
        title=lambda: tr("Audible Alert"),
        description=lambda: tr("Std.: Stock behaviour.<br>Warning: Only emits sound when there is a warning.<br>Off: Does not emit any sound at all."),
        initial_index=int(self._params.get("dp_dev_audible_alert_mode") or 0),
        callback=lambda val: self._params.put("dp_dev_audible_alert_mode", val),
        options=["Std.", "Warning", "Off"],
      )

    self._toggles["dp_dev_auto_shutdown_in"] = spin_button_item(
      title=lambda: tr("Auto Shutdown After"),
      description=lambda: tr("0 mins = Immediately"),
      initial_value=int(self._params.get("dp_dev_auto_shutdown_in") or -5),
      callback=lambda val: self._params.put("dp_dev_auto_shutdown_in", val),
      min_val=-5,
      max_val=300,
      step=5,
      suffix=tr(" mins"),
      special_value_text=tr("Off"),
    )

    self._toggles["dp_dev_dashy"] = text_spin_button_item(
      title=lambda: tr("dashy"),
      description=lambda: tr("dashy - dragonpilot's all-in-one system hub for you.<br><br>Visit http://<device_ip>:5088 to access.<br><br>Off - Turn off dashy completely.<br>File: File Manager only.<br>All: File Manager + Live Stream."),
      initial_index=int(self._params.get("dp_dev_dashy") or 0),
      callback=lambda val: self._params.put("dp_dev_dashy", val),
      options=[tr("Off"), tr("File"), tr("All")],
    )

    self._toggles["dp_dev_delay_loggerd"] = spin_button_item(
      title=lambda: tr("Delay Starting Loggerd for:"),
      description=lambda: tr("Delays the startup of loggerd and its related processes when the device goes on-road.<br>This prevents the initial moments of a drive from being recorded, protecting location privacy at the start of a trip."),
      initial_value=int(self._params.get("dp_dev_delay_loggerd") or 0),
      callback=lambda val: self._params.put("dp_dev_delay_loggerd", val),
      min_val=0,
      max_val=300,
      step=5,
      suffix=tr(" secs"),
      special_value_text=tr("Off"),
    )

    self._toggles["dp_dev_disable_connect"] = toggle_item(
      title=lambda: tr("Disable Comma Connect"),
      description=lambda: tr("Disable Comma connect service if you do not wish to upload / being tracked by the service."),
      initial_state=self._params.get_bool("dp_dev_disable_connect"),
      callback=lambda val: self._params.put_bool("dp_dev_disable_connect", val),
    )

  def _reset_dp_conf(self):
    def reset_dp_conf(result: int):
      # Check engaged again in case it changed while the dialog was open
      if result != DialogResult.CONFIRM:
        return
      self._params.put_bool("dp_dev_reset_conf", True)
      self._params.put_bool("DoReboot", True)

    dialog = ConfirmDialog(tr("Are you sure you want to reset ALL DP SETTINGS to default?"), tr("Reset"))
    gui_app.set_modal_overlay(dialog, callback=reset_dp_conf)

  def show_event(self):
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

  def _render(self, rect):
    self._scroller.render(rect)

  def _on_dp_lat_lca_speed_change(self, val):
    self._params.put("dp_lat_lca_speed", int(val))
    self._toggles['dp_lat_lca_auto_sec'].action_item.set_enabled(val > 0)
