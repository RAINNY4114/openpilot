"""
Lincoln-specific settings layout (ford-lincoln-ui-20261118).

Adapted from openpilot-lz/ford lincoln.py to the device's sunnypilot UI
framework.  Only parameters that are REALLY read by the device code are
exposed here (verified by source cross-reference on 2026-11-18):

  - AutoOvertakeEnabled        (ao_aa_bridge.py)
  - AutoAvoidanceEnabled       (ao_aa_bridge.py)
  - AutoOvertakeMinCruiseKph   (ao_aa_bridge.py)
  - AutoOvertakeConfirmSec     (ao_aa_bridge.py)

Deliberately NOT exposed (kept always-on at their current values):
  - dp_bsm_voice_enabled / _interval_sec / _volume_pct   (soundd.py)
  - AOAlertEnabled                                        (soundd.py)

All lz options whose params do not exist on this device were left out
(Cone Detection, Human Turn Detection, Curve Speed Control, Follow-Coast,
Stop distance, Auto LC edge clearance).

IMPORTANT — two device-specific constraints handled here:
  1. Param read priority: /data/ford_params > /dev/shm/params > /data/params/d
     (first existing file wins).  These keys live in /data/ford_params, while
     a plain Params() writes /data/params/d; a standard param= binding would be
     silently overridden.  We use a direct multi-directory helper.
  2. These keys are NOT registered in params_keys.h, so Params.get() raises
     UnknownKeyName.  The stock option_item_sp calls Params.get(param) and
     would crash the whole UI on startup ("stuck at comma logo").  Numeric
     items therefore use a self-contained DirectOptionAction that never
     touches Params.
"""
import os

import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos, TextAlignment, TextAlignmentVertical
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.sunnypilot.lib.styles import style
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  toggle_item_sp,
  ListItemSP,
  LineSeparatorSP,
)
from openpilot.system.ui.widgets.list_view import ItemAction
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget

# Same priority order as ao_aa_bridge.PARAM_DIRS and soundd.read_param_direct.
_PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")


def _read_param(key, default=None):
  for d in _PARAM_DIRS:
    try:
      with open(os.path.join(d, key)) as f:
        return f.read().strip()
    except Exception:
      continue
  return default


def _write_param(key, value):
  for d in _PARAM_DIRS:
    try:
      if not os.path.isdir(d):
        continue
      with open(os.path.join(d, key), "w") as f:
        f.write(str(value))
    except Exception:
      continue


def _get_int(key, default):
  try:
    return int(float(_read_param(key, default)))
  except (TypeError, ValueError):
    return default


def _get_float(key, default):
  try:
    return float(_read_param(key, default))
  except (TypeError, ValueError):
    return default


def _get_bool(key, default=False):
  raw = _read_param(key)
  if raw is None:
    return default
  return str(raw).strip() in ("1", "true", "True")


# ---- Stepper geometry (mirrors sunnypilot OptionControlSP) ----
_STEP_BTN_W = 150
_CONTROL_H = 150
_STEP_LABEL_W = 300
_STEP_GAP = 25
_STEP_VALUE_FONT = 50
_STEP_BTN_FONT = 60


class DirectOptionAction(ItemAction):
  """A -/+ stepper backed purely by _read_param/_write_param (no Params, no
  check_key).  Value is scaled: stored = display / scale.  label_callback
  receives the display (integer) value."""
  def __init__(self, key: str, min_value: int, max_value: int, step: int,
               scale: float, label_callback: Callable[[int], str] | None = None):
    total_w = _STEP_BTN_W * 2 + _STEP_LABEL_W + _STEP_GAP * 2
    super().__init__(width=total_w)
    self.key = key
    self.min_value = min_value
    self.max_value = max_value
    self.step = step
    self.scale = scale
    self.label_callback = label_callback
    self.value = min_value
    self._minus = False
    self._plus = False
    self._font = gui_app.font(FontWeight.MEDIUM)
    self._minus_rect = rl.Rectangle(0, 0, 0, 0)
    self._plus_rect = rl.Rectangle(0, 0, 0, 0)
    self._label_rect = rl.Rectangle(0, 0, 0, 0)

  def seed(self, value: int):
    self.value = max(self.min_value, min(self.max_value, int(value)))

  def _display(self) -> str:
    if self.label_callback:
      return self.label_callback(self.value)
    return str(self.value)

  def _commit(self):
    _write_param(self.key, f"{self.value / self.scale:.2f}".rstrip("0").rstrip("."))

  def _button(self, rect: rl.Rectangle, text: str, enabled: bool):
    mouse = rl.get_mouse_position()
    pressed = (enabled and rl.check_collision_point_rec(mouse, rect)
               and self._touch_valid() and rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT))
    color = style.ITEM_TEXT_COLOR if enabled else style.ITEM_DISABLED_TEXT_COLOR
    if enabled and pressed:
      rl.draw_rectangle_rounded(rect, 0.2, 20, style.OPTION_CONTROL_BTN_PRESSED)
    ts = measure_text_cached(self._font, text, _STEP_BTN_FONT)
    rl.draw_text_ex(self._font, text,
                    rl.Vector2(rect.x + (rect.width - ts.x) / 2, rect.y + (rect.height - ts.y) / 2),
                    _STEP_BTN_FONT, 0, color)

  def _render(self, rect: rl.Rectangle):
    if self._rect.width == 0 or self._rect.height == 0 or not self.is_visible:
      return
    control_w = _STEP_BTN_W * 2 + _STEP_LABEL_W + _STEP_GAP * 2
    start_x = self._rect.x + self._rect.width - control_w
    y = rect.y + (rect.height - _CONTROL_H) / 2
    container = rl.Rectangle(start_x, y, control_w, _CONTROL_H)
    rl.draw_rectangle_rounded(container, 0.2, 20, style.OPTION_CONTROL_CONTAINER_BG)
    self._minus_rect = rl.Rectangle(container.x, y, _STEP_BTN_W, _CONTROL_H)
    label_x = container.x + _STEP_BTN_W + _STEP_GAP
    self._label_rect = rl.Rectangle(label_x, y, _STEP_LABEL_W, _CONTROL_H)
    self._plus_rect = rl.Rectangle(label_x + _STEP_LABEL_W + _STEP_GAP, y, _STEP_BTN_W, _CONTROL_H)
    self._minus = self.enabled and self.value > self.min_value
    self._plus = self.enabled and self.value < self.max_value
    self._button(self._minus_rect, "-", self._minus)
    # value label
    text = self._display()
    color = style.ITEM_TEXT_COLOR if self.enabled else style.ITEM_DISABLED_TEXT_COLOR
    ts = measure_text_cached(self._font, text, _STEP_VALUE_FONT)
    rl.draw_text_ex(self._font, text,
                    rl.Vector2(self._label_rect.x + (self._label_rect.width - ts.x) / 2,
                               self._label_rect.y + (self._label_rect.height - ts.y) / 2),
                    _STEP_VALUE_FONT, 0, color)
    self._button(self._plus_rect, "+", self._plus)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._minus and rl.check_collision_point_rec(mouse_pos, self._minus_rect):
      self.value = max(self.min_value, self.value - self.step)
      self._commit()
    elif self._plus and rl.check_collision_point_rec(mouse_pos, self._plus_rect):
      self.value = min(self.max_value, self.value + self.step)
      self._commit()


def _stepper_item(title, description, key, min_value, max_value, step, scale, label_callback):
  action = DirectOptionAction(key, min_value, max_value, step, scale, label_callback)
  return ListItemSP(title=title, description=description, action_item=action, inline=True), action


class LincolnLayout(Widget):
  def __init__(self):
    super().__init__()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=False, spacing=0)

  def _initialize_items(self):
    # ---- Auto overtake / avoidance (ao_aa_bridge.py) ----
    self._ao_toggle = toggle_item_sp(
      title=lambda: tr("Auto overtake"),
      description=lambda: tr(
        "Highway-only: when a slower lead vehicle is detected ahead and the "
        "passing lane is clear, automatically change lanes to pass and return "
        "when clear. Requires blindspot sensors."
      ),
      initial_state=_get_bool("AutoOvertakeEnabled"),
      callback=lambda val: _write_param("AutoOvertakeEnabled", 1 if val else 0),
    )
    self._aa_toggle = toggle_item_sp(
      title=lambda: tr("Auto avoidance"),
      description=lambda: tr(
        "When an obstacle is detected in-path, slow down then initiate a lane "
        "change to pass, and return when clear. Emergency-only. Requires "
        "blindspot sensors."
      ),
      initial_state=_get_bool("AutoAvoidanceEnabled"),
      callback=lambda val: _write_param("AutoAvoidanceEnabled", 1 if val else 0),
    )
    # scale=100 means value is stored value/100 (e.g. 90.0 -> 9000 display units).
    self._ao_min_speed, self._ao_min_speed_action = _stepper_item(
      title=lambda: tr("Auto lane-change min speed"),
      description=lambda: tr(
        "Minimum cruise set speed required for auto overtake. "
        "Selectable in 10 km/h steps from 60 to 100 km/h."
      ),
      key="AutoOvertakeMinCruiseKph",
      min_value=6000, max_value=10000, step=1000, scale=100.0,
      label_callback=lambda v: f"{v / 100.0:.0f} km/h",
    )
    self._ao_confirm_delay, self._ao_confirm_delay_action = _stepper_item(
      title=lambda: tr("Auto lane-change confirm delay"),
      description=lambda: tr(
        "Seconds to wait after voice + auto blinker before starting an "
        "automatic lane change."
      ),
      key="AutoOvertakeConfirmSec",
      min_value=0, max_value=1000, step=50, scale=100.0,
      label_callback=lambda v: f"{v / 100.0:.1f} s",
    )

    # ---- Obstacle avoidance (experimental) ----
    # Section header (plain text, no action widget).
    self._oa_header = ListItemSP(title=lambda: tr("### Obstacle Avoidance (Experimental) ###"))
    # Obstacle avoidance: reuses the real ObstacleAvoidEnabled param consumed by
    # obstacle_avoid.py (radarTracks + MR76 AUX -> lateral path offset).
    self._oa_toggle = toggle_item_sp(
      title=lambda: tr("Obstacle Avoidance"),
      description=lambda: tr(
        "Experimental: steer around obstacles (vehicles, pedestrians, road "
        "debris) detected by radar by adding a smooth lateral offset to the "
        "planned path."
      ),
      initial_state=_get_bool("ObstacleAvoidEnabled"),
      callback=lambda val: _write_param("ObstacleAvoidEnabled", 1 if val else 0),
    )
    # Cone detection (experimental): opt-in flag for classifying low-RCS,
    # stationary, road-side radar targets as traffic cones. No camera used.
    self._cone_toggle = toggle_item_sp(
      title=lambda: tr("Cone Detection (Experimental)"),
      description=lambda: tr(
        "Experimental: treat low-reflectivity stationary road-side radar "
        "targets as traffic cones and include them in obstacle avoidance. "
        "Uses the existing radar stream only (no camera)."
      ),
      initial_state=_get_bool("ConeDetectionEnabled"),
      callback=lambda val: _write_param("ConeDetectionEnabled", 1 if val else 0),
    )

    # ---- Longitudinal enhancements (SpysyWeeb) ----
    self._long_header = ListItemSP(title=lambda: tr("### Longitudinal Enhancements ###"))
    self._smooth_stops_toggle = toggle_item_sp(
      title=lambda: tr("Smooth Stops"),
      description=lambda: tr("Exponential brake landing for a smooth, imperceptible stop. Braking fades near standstill; off = stock."),
      initial_state=_get_bool("SmoothStops"),
      callback=lambda val: _write_param("SmoothStops", 1 if val else 0),
    )
    self._lead_reaction_toggle = toggle_item_sp(
      title=lambda: tr("Lead Reaction Tuning"),
      description=lambda: tr("Quicker response to lead pull-away and braking; launch tracks the lead from a stop."),
      initial_state=_get_bool("LeadReactionTuning"),
      callback=lambda val: _write_param("LeadReactionTuning", 1 if val else 0),
    )
    self._custom_personality_toggle = toggle_item_sp(
      title=lambda: tr("Custom Driving Style"),
      description=lambda: tr("Enable per-personality Acceleration / Jerk / Follow sliders. Off uses stock personalities."),
      initial_state=_get_bool("CustomPersonality"),
      callback=lambda val: _write_param("CustomPersonality", 1 if val else 0),
    )
    self._accel_item, self._accel_action = _stepper_item(
      title=lambda: tr("Acceleration"),
      description=lambda: tr("Maximum acceleration from a stop (m/s^2). Stock 1.6."),
      key="CustomPersonalityStandardAccel",
      min_value=8, max_value=20, step=1, scale=10.0,
      label_callback=lambda v: f"{v / 10.0:.1f} m/s^2",
    )
    self._jerk_item, self._jerk_action = _stepper_item(
      title=lambda: tr("Jerk Value"),
      description=lambda: tr("Acceleration smoothness multiplier. Lower = smoother/earlier."),
      key="CustomPersonalityStandardJerkMultiplier",
      min_value=5, max_value=40, step=1, scale=10.0,
      label_callback=lambda v: f"{v / 10.0:.1f}x",
    )
    self._follow_item, self._follow_action = _stepper_item(
      title=lambda: tr("Follow Distance"),
      description=lambda: tr("Time gap to lead vehicle in seconds. Smaller = closer."),
      key="CustomPersonalityStandardFollow",
      min_value=20, max_value=50, step=1, scale=20.0,
      label_callback=lambda v: f"{v / 20.0:.2f} s",
    )

    self._seed_values()

    return [
      LineSeparatorSP(40),
      self._ao_toggle,
      self._aa_toggle,
      self._ao_min_speed,
      self._ao_confirm_delay,
      LineSeparatorSP(40),
      self._oa_header,
      self._oa_toggle,
      self._cone_toggle,
      LineSeparatorSP(40),
      self._long_header,
      self._smooth_stops_toggle,
      self._lead_reaction_toggle,
      self._custom_personality_toggle,
      self._accel_item,
      self._jerk_item,
      self._follow_item,
    ]

  def _seed_values(self):
    self._ao_min_speed_action.seed(int(_get_float("AutoOvertakeMinCruiseKph", 90.0) * 100))
    self._ao_confirm_delay_action.seed(int(_get_float("AutoOvertakeConfirmSec", 3.0) * 100))
    self._accel_action.seed(int(_get_float("CustomPersonalityStandardAccel", 1.6) * 10))
    self._jerk_action.seed(int(_get_float("CustomPersonalityStandardJerkMultiplier", 1.0) * 10))
    self._follow_action.seed(int(_get_float("CustomPersonalityStandardFollow", 1.45) * 20))

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._ao_toggle.action_item.set_state(_get_bool("AutoOvertakeEnabled"))
    self._aa_toggle.action_item.set_state(_get_bool("AutoAvoidanceEnabled"))
    self._oa_toggle.action_item.set_state(_get_bool("ObstacleAvoidEnabled"))
    self._cone_toggle.action_item.set_state(_get_bool("ConeDetectionEnabled"))
    self._seed_values()
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()
