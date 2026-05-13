from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, double_spin_button_item, multiple_button_item, simple_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


class BPLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    self._scroller = Scroller([
      simple_item(title=lambda: tr("### Visuals ###")),
      toggle_item(
        title=lambda: tr("Show Blindspot Overlay"),
        description=lambda: tr("Display red overlay when vehicle is detected in blindspot."),
        initial_state=self._params.get_bool("ShowBlindspotOverlay"),
        callback=lambda val: self._params.put_bool("ShowBlindspotOverlay", val),
      ),
      toggle_item(
        title=lambda: tr("Show Brake Status"),
        description=lambda: tr("Display speed setpoint in red when vehicle is braking."),
        initial_state=self._params.get_bool("ShowBrakeStatus"),
        callback=lambda val: self._params.put_bool("ShowBrakeStatus", val),
      ),
      toggle_item(
        title=lambda: tr("Show Confidence Ball"),
        description=lambda: tr("Display the confidence ball on the left side of the driving view."),
        initial_state=self._params.get_bool("BPShowConfidenceBall"),
        callback=lambda val: self._params.put_bool("BPShowConfidenceBall", val),
      ),
      toggle_item(
        title=lambda: tr("Animate Steering Wheel"),
        description=lambda: tr("Rotate the steering wheel icon to match the current steering angle."),
        initial_state=self._params.get_bool("BPAnimateSteeringWheel"),
        callback=lambda val: self._params.put_bool("BPAnimateSteeringWheel", val),
      ),
      toggle_item(
        title=lambda: tr("Show Radar Lead Overlay (Ford ACC)"),
        description=lambda: tr("Display boxed lead vehicle info when using Ford stock ACC."),
        initial_state=self._params.get_bool("FordPrefShowRadarLeadOverlay"),
        callback=lambda val: self._params.put_bool("FordPrefShowRadarLeadOverlay", val),
      ),
      multiple_button_item(
        title=lambda: tr("Radar Overlay Size"),
        description=lambda: tr("Set the size of the radar lead overlay boxes."),
        buttons=[lambda: tr("Small"), lambda: tr("Medium"), lambda: tr("Large")],
        selected_index=self._get_param_int("FordPrefRadarOverlaySize", 1),
        button_width=180,
        callback=lambda idx: self._params.put("FordPrefRadarOverlaySize", int(idx)),
      ),
      simple_item(title=lambda: tr("### Longitudinal Tuning ###")),
      toggle_item(
        title=lambda: tr("Bypass BP Longitudinal Control"),
        description=lambda: tr("Use stock longitudinal logic instead of BluePilot TTC/coasting tuning."),
        initial_state=self._params.get_bool("disable_BP_long_UI"),
        callback=lambda val: self._params.put_bool("disable_BP_long_UI", val),
      ),
      toggle_item(
        title=lambda: tr("Disable Downhill Compensation"),
        description=lambda: tr("Disable pitch-based brake/gas compensation when going downhill."),
        initial_state=self._params.get_bool("disable_downhill_comp_UI"),
        callback=lambda val: self._params.put_bool("disable_downhill_comp_UI", val),
      ),
      simple_item(title=lambda: tr("### Lateral Tuning ###")),
      toggle_item(
        title=lambda: tr("Disable BP Lateral Control"),
        description=lambda: tr("Disable BluePilot lateral control."),
        initial_state=self._params.get_bool("disable_BP_lat_UI"),
        callback=lambda val: self._params.put_bool("disable_BP_lat_UI", val),
      ),
      toggle_item(
        title=lambda: tr("Disable Lane Change Under Speed"),
        description=lambda: tr("Pause lateral control when blinker is on and below minimum speed."),
        initial_state=self._params.get_bool("BlinkerPauseLaneChange"),
        callback=lambda val: self._params.put_bool("BlinkerPauseLaneChange", val),
      ),
      toggle_item(
        title=lambda: tr("Enable Lane Positioning"),
        description=lambda: tr("Enable custom lane positioning controls."),
        initial_state=self._params.get_bool("enable_lane_positioning"),
        callback=lambda val: self._params.put_bool("enable_lane_positioning", val),
      ),
      double_spin_button_item(
        title=lambda: tr("In-Lane Offset"),
        description=lambda: tr("Adjust the in-lane offset (-0.5 to 0.5)."),
        initial_value=self._get_param_float("custom_path_offset", 0.0),
        callback=lambda val: self._params.put("custom_path_offset", float(val)),
        min_val=-0.5,
        max_val=0.5,
        step=0.05,
        decimals=2,
        enabled=lambda: self._params.get_bool("enable_lane_positioning"),
      ),
      toggle_item(
        title=lambda: tr("Enable Lanefull Mode"),
        description=lambda: tr("Enable lanefull mode for lane positioning."),
        initial_state=self._params.get_bool("enable_lane_full_mode"),
        callback=lambda val: self._params.put_bool("enable_lane_full_mode", val),
        enabled=lambda: self._params.get_bool("enable_lane_positioning"),
      ),
      toggle_item(
        title=lambda: tr("Use Custom Tuning Profile"),
        description=lambda: tr("Enable custom tuning profile settings."),
        initial_state=self._get_param_int("custom_profile", 0) != 0,
        callback=lambda val: self._params.put("custom_profile", int(val)),
      ),
      double_spin_button_item(
        title=lambda: tr("Predicted Curvature Blend Ratio High"),
        description=lambda: tr("Adjust the high curvature blend ratio (0.0-1.0)."),
        initial_value=self._get_param_float("pc_blend_ratio_high_C_UI", 0.4),
        callback=lambda val: self._params.put("pc_blend_ratio_high_C_UI", float(val)),
        min_val=0.0,
        max_val=1.0,
        step=0.05,
        decimals=2,
        enabled=lambda: self._get_param_int("custom_profile", 0) != 0,
      ),
      double_spin_button_item(
        title=lambda: tr("Predicted Curvature Blend Ratio Low"),
        description=lambda: tr("Adjust the low curvature blend ratio (0.0-1.0)."),
        initial_value=self._get_param_float("pc_blend_ratio_low_C_UI", 0.4),
        callback=lambda val: self._params.put("pc_blend_ratio_low_C_UI", float(val)),
        min_val=0.0,
        max_val=1.0,
        step=0.05,
        decimals=2,
        enabled=lambda: self._get_param_int("custom_profile", 0) != 0,
      ),
      double_spin_button_item(
        title=lambda: tr("Low Curvature PID Gain"),
        description=lambda: tr("Adjust the low curvature PID gain (0.0-5.0)."),
        initial_value=self._get_param_float("LC_PID_gain_UI", 3.0),
        callback=lambda val: self._params.put("LC_PID_gain_UI", float(val)),
        min_val=0.0,
        max_val=5.0,
        step=0.1,
        decimals=1,
        enabled=lambda: self._get_param_int("custom_profile", 0) != 0,
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()

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

  def _get_param_float(self, key: str, default: float) -> float:
    return self._safe_float(self._params.get(key), default)

  @staticmethod
  def _safe_int(val: bytes | str | None, default: int) -> int:
    if not val:
      return default
    try:
      if isinstance(val, bytes):
        val = val.decode("utf-8", errors="ignore")
      return int(val)
    except (TypeError, ValueError):
      return default

  def _get_param_int(self, key: str, default: int) -> int:
    return self._safe_int(self._params.get(key), default)
