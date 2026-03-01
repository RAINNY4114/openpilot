from importlib.resources import as_file
import math
import pyray as rl
import time
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.onroad.lane_pref_button import LanePrefButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import FONT_DIR, FONT_SCALE, font_fallback, gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from dragonpilot.selfdrive.ui.onroad.torque_bar import TorqueBar

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371  # km to mile
CRUISE_DISABLED_CHAR = '–'


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  white: rl.Color = rl.WHITE
  disengaged: rl.Color = rl.Color(145, 155, 149, 255)
  override: rl.Color = rl.Color(145, 155, 149, 255)  # Added
  engaged: rl.Color = rl.Color(128, 216, 166, 255)
  disengaged_bg: rl.Color = rl.Color(0, 0, 0, 153)
  override_bg: rl.Color = rl.Color(145, 155, 149, 204)
  engaged_bg: rl.Color = rl.Color(128, 216, 166, 204)
  blue: rl.Color = rl.Color(0, 122, 255, 255)
  blue_translucent: rl.Color = rl.Color(0, 122, 255, 166)
  grey: rl.Color = rl.Color(166, 166, 166, 255)
  dark_grey: rl.Color = rl.Color(114, 114, 114, 255)
  black_translucent: rl.Color = rl.Color(0, 0, 0, 166)
  white_translucent: rl.Color = rl.Color(255, 255, 255, 200)
  border_translucent: rl.Color = rl.Color(255, 255, 255, 75)
  header_gradient_start: rl.Color = rl.Color(0, 0, 0, 114)
  header_gradient_end: rl.Color = rl.BLANK


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


@dataclass
class _DynamicFontCacheEntry:
  font: rl.Font
  last_used_t: float


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self._a_ego: float = 0.0
    self._standstill: bool = False
    self.v_ego_cluster_seen: bool = False
    self._set_speed_rect: rl.Rectangle | None = None

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)
    self._lane_pref_button_size: int = int(UI_CONFIG.button_size * 0.85)
    self._lane_pref_button: LanePrefButton = LanePrefButton(self._lane_pref_button_size)

    self._torque_bar = TorqueBar()

    # NOTE: Prefer explicit L/R assets instead of texture mirroring; some devices/drivers don't render negative src_rect reliably.
    self._curve_speed_icon_l: rl.Texture = gui_app.texture("icons/curve_speed.png", UI_CONFIG.button_size, UI_CONFIG.button_size)
    self._curve_speed_icon_r: rl.Texture = gui_app.texture("icons/curveR_speed.png", UI_CONFIG.button_size, UI_CONFIG.button_size)
    # Pedal indicator icons (FrogPilot-style). Always shown onroad (no toggle).
    self._brake_pedal_icon: rl.Texture = gui_app.texture(
      "icons/brake_pedal.png", UI_CONFIG.button_size, UI_CONFIG.button_size
    )
    self._gas_pedal_icon: rl.Texture = gui_app.texture(
      "icons/gas_pedal.png", UI_CONFIG.button_size, UI_CONFIG.button_size
    )
    self._curve_speed_str: str = ""
    self._curve_dist_str: str = ""
    self._curve_state_str: str = ""
    self._curve_speed_font = None
    self._curve_dist_font = None
    self._curve_speed_flip: bool = False
    self._curve_show: bool = False
    self._curve_k_smooth: float = 0.0
    self._curve_active: bool = False
    self._curve_exit_timer: float = 0.0
    self._curve_last_update_t: float = time.monotonic()
    self._curve_flip_candidate: bool | None = None
    self._curve_flip_candidate_t: float = 0.0
    self._curve_font_cache: dict[tuple[int, ...], _DynamicFontCacheEntry] = {}

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self._curve_show = False
      self._curve_active = False
      self._curve_exit_timer = 0.0
      self._curve_speed_str = ""
      self._curve_dist_str = ""
      self._curve_state_str = ""
      self._curve_speed_font = None
      self._curve_dist_font = None
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    self._a_ego = float(car_state.aEgo)
    self._standstill = bool(car_state.standstill)

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    self._update_curve_speed_widget()

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.header_gradient_start,
      COLORS.header_gradient_end,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)
      self._draw_curve_speed_control()

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))
    self._draw_pedal_icons(rect, button_x, button_y)

    # Lane preference button: left side, between MAX set-speed box (top-left)
    # and the driver monitoring icon (bottom-left).
    max_box_bottom_y = rect.y + 45 + UI_CONFIG.set_speed_height
    if self._set_speed_rect is not None:
      max_box_bottom_y = float(self._set_speed_rect.y + self._set_speed_rect.height)

    dm_box_top_y = rect.y + rect.height - (UI_CONFIG.border_size + UI_CONFIG.button_size)
    pref_center_y = (max_box_bottom_y + dm_box_top_y) / 2.0
    pref_y = float(pref_center_y - self._lane_pref_button_size / 2.0)

    # Align X with the driver monitoring icon center for consistent left-column layout.
    dm_center_x = rect.x + UI_CONFIG.border_size + UI_CONFIG.button_size / 2.0
    pref_x = float(dm_center_x - self._lane_pref_button_size / 2.0)

    self._lane_pref_button.render(rl.Rectangle(pref_x, pref_y, self._lane_pref_button_size, self._lane_pref_button_size))

    if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
      self._torque_bar.render(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed or self._lane_pref_button.is_pressed

  def _draw_pedal_icons(self, rect: rl.Rectangle, exp_button_x: float, exp_button_y: float) -> None:
    # FrogPilot-style pedals: opacity reflects +/- acceleration (aEgo).
    # This is a UI hint only; it is not a physical pedal position indicator.
    sm = ui_state.sm
    if sm.recv_frame.get("modelV2", 0) < ui_state.started_frame:
      return

    a_ego = float(self._a_ego)
    standstill = bool(self._standstill)

    brake_opacity = 0.25
    gas_opacity = 0.25
    if standstill:
      brake_opacity = 1.0
    else:
      if a_ego < -0.25:
        brake_opacity = max(0.25, abs(a_ego))
      gas_opacity = max(0.25, a_ego)

    brake_opacity = float(max(0.0, min(1.0, brake_opacity)))
    gas_opacity = float(max(0.0, min(1.0, gas_opacity)))

    if self._set_speed_rect is not None:
      start_x = float(self._set_speed_rect.x)
      start_y = float(self._set_speed_rect.y + self._set_speed_rect.height + UI_CONFIG.border_size / 2)
    else:
      set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
      start_x = float(rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) / 2)
      start_y = float(rect.y + 45 + UI_CONFIG.set_speed_height + UI_CONFIG.border_size / 2)

    # Avoid overlapping the lane-preference button on small screens.
    max_box_bottom_y = float(rect.y + 45 + UI_CONFIG.set_speed_height)
    if self._set_speed_rect is not None:
      max_box_bottom_y = float(self._set_speed_rect.y + self._set_speed_rect.height)
    dm_box_top_y = float(rect.y + rect.height - (UI_CONFIG.border_size + UI_CONFIG.button_size))
    pref_center_y = (max_box_bottom_y + dm_box_top_y) / 2.0
    pref_y = float(pref_center_y - self._lane_pref_button_size / 2.0)

    brake_w = float(self._brake_pedal_icon.width)
    brake_h = float(self._brake_pedal_icon.height)
    gas_w = float(self._gas_pedal_icon.width)
    gas_h = float(self._gas_pedal_icon.height)

    if start_y + max(brake_h, gas_h) > pref_y - UI_CONFIG.border_size / 2:
      start_y = float(pref_y - max(brake_h, gas_h) - UI_CONFIG.border_size / 2)
    src_brake = rl.Rectangle(0, 0, brake_w, brake_h)
    dst_brake = rl.Rectangle(start_x, start_y, brake_w, brake_h)
    tint_brake = rl.Color(255, 255, 255, int(round(255 * brake_opacity)))
    rl.draw_texture_pro(self._brake_pedal_icon, src_brake, dst_brake, rl.Vector2(0, 0), 0.0, tint_brake)

    src_gas = rl.Rectangle(0, 0, gas_w, gas_h)
    dst_gas = rl.Rectangle(start_x + UI_CONFIG.button_size / 2.0, start_y, gas_w, gas_h)
    tint_gas = rl.Color(255, 255, 255, int(round(255 * gas_opacity)))
    rl.draw_texture_pro(self._gas_pedal_icon, src_gas, dst_gas, rl.Vector2(0, 0), 0.0, tint_gas)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    self._set_speed_rect = set_speed_rect
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.black_translucent)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.border_translucent)

    max_color = COLORS.grey
    set_speed_color = COLORS.dark_grey
    if self.is_cruise_set:
      set_speed_color = COLORS.white
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.engaged
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.disengaged
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.override

    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.white)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.white_translucent)

  @staticmethod
  def _interp(x: float, xp: tuple[float, ...], fp: tuple[float, ...]) -> float:
    if x <= xp[0]:
      return fp[0]
    for i in range(1, len(xp)):
      if x <= xp[i]:
        x0, x1 = xp[i - 1], xp[i]
        y0, y1 = fp[i - 1], fp[i]
        if x1 == x0:
          return y1
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return fp[-1]

  @staticmethod
  def _safe_int_param(key: str, default: int) -> int:
    try:
      raw = ui_state.params.get(key)
      if raw is None:
        return default
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")
      raw_s = str(raw).strip()
      return int(raw_s) if raw_s else default
    except Exception:
      return default

  def _update_curve_icon_direction(self, k_signed_at_max: float, k_max: float, now: float, *, force: bool = False) -> None:
    # Avoid rapid L/R flipping when curvature sign is noisy near zero.
    k_dir_deadband = 2e-4
    hold_s = 0.25

    try:
      k_signed = float(k_signed_at_max)
      k_abs = float(k_max)
    except Exception:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if (not math.isfinite(k_signed)) or (not math.isfinite(k_abs)) or k_abs < k_dir_deadband:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    new_flip = bool(k_signed >= 0.0)
    if force:
      self._curve_speed_flip = new_flip
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if new_flip == self._curve_speed_flip:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if self._curve_flip_candidate != new_flip:
      self._curve_flip_candidate = new_flip
      self._curve_flip_candidate_t = float(now)
      return

    if (now - float(self._curve_flip_candidate_t)) >= hold_s:
      self._curve_speed_flip = new_flip
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0

  def _update_curve_speed_widget(self) -> None:
    now = time.monotonic()
    dt = max(0.0, min(0.2, now - self._curve_last_update_t))
    self._curve_last_update_t = now

    prev_show = bool(self._curve_show)
    self._curve_show = False
    self._curve_speed_str = ""
    self._curve_dist_str = ""
    self._curve_state_str = ""
    self._curve_speed_font = None
    self._curve_dist_font = None
    if not prev_show:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0

    sm = ui_state.sm
    try:
      curve_speed_enabled = ui_state.params.get_bool("CurveSpeedControl")
      show_curve_speed = ui_state.params.get_bool("ShowCSCStatus")
      if not (curve_speed_enabled and show_curve_speed):
        self._curve_active = False
        self._curve_exit_timer = 0.0
        return
    except Exception:
      return

    if not self.is_cruise_set:
      self._curve_active = False
      self._curve_exit_timer = 0.0
      return

    if sm.recv_frame["modelV2"] < ui_state.started_frame:
      self._curve_active = False
      self._curve_exit_timer = 0.0
      return

    model = sm["modelV2"]
    long_plan = sm["longitudinalPlan"]
    car_state = sm["carState"]
    v_ego = float(getattr(car_state, "vEgo", 0.0))
    if not math.isfinite(v_ego) or v_ego < 1.0:
      self._curve_active = False
      self._curve_exit_timer = 0.0
      return

    positions = getattr(getattr(model, "position", None), "x", []) or []
    v_preds = getattr(getattr(model, "velocity", None), "x", []) or []
    turn_rates = getattr(getattr(model, "orientationRate", None), "z", []) or []
    if not positions or len(positions) != len(v_preds) or len(v_preds) != len(turn_rates):
      self._curve_active = False
      self._curve_exit_timer = 0.0
      return

    # If map curve speed is actively limiting cruise, show the curve widget even when the vision curvature
    # is too mild/too far to trigger the vision-only heuristic.
    src_val = 0
    try:
      src = getattr(long_plan, "curveSpeedSource", None)
      raw = getattr(src, "raw", None)
      if raw is not None:
        src_val = int(raw)
      else:
        src_val = int(src)
    except Exception:
      src_val = 0

    try:
      speeds = list(getattr(long_plan, "speeds", []) or [])
    except Exception:
      speeds = []

    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    speed_unit = tr("km/h") if ui_state.is_metric else tr("mph")

    if src_val in (1, 2) and speeds:
      try:
        v_min = float(min(float(v) for v in speeds if math.isfinite(float(v)) and float(v) > 0.0))
      except Exception:
        v_min = 0.0

      curve_target = None
      try:
        v_target = float(getattr(long_plan, "vTargetDEPRECATED", 0.0))
        if math.isfinite(v_target) and v_target > 0.0:
          curve_target = v_target
      except Exception:
        curve_target = None

      if curve_target is None:
        curve_target = v_min

      v_target_disp = float(curve_target) * speed_conversion
      if math.isfinite(v_target_disp) and v_target_disp > 0.0 and v_target_disp < self.set_speed:
        # Estimate distance-to-target within the published horizon.
        t = list(ModelConstants.T_IDXS[:len(speeds)])
        dist_to_min = 0.0
        try:
          idx_min = int(min(range(len(speeds)), key=lambda i: float(speeds[i])))
        except Exception:
          idx_min = 0

        idx_target = None
        for i, v in enumerate(speeds):
          try:
            if float(v) <= float(curve_target) + 0.05:
              idx_target = i
              break
          except Exception:
            continue
        if idx_target is None:
          idx_target = idx_min

        for i in range(1, min(len(speeds), len(t))):
          dt_i = float(t[i] - t[i - 1])
          if dt_i <= 0.0:
            continue
          v0 = float(speeds[i - 1])
          v1 = float(speeds[i])
          dist_to_min += 0.5 * (v0 + v1) * dt_i
          if i >= idx_target:
            break

        # Use vision sign to pick L/R icon if available.
        k_signed_at_max = 0.0
        k_max = 0.0
        try:
          for v_pred, turn_rate in zip(v_preds, turn_rates, strict=True):
            v_pred_f = float(v_pred)
            turn_rate_f = float(turn_rate)
            if not math.isfinite(v_pred_f) or not math.isfinite(turn_rate_f):
              continue
            v_pred_f = max(max(1.0, v_ego * 0.7), min(100.0, v_pred_f))
            k_signed = turn_rate_f / v_pred_f
            k = abs(k_signed)
            if k > k_max:
              k_max = k
              k_signed_at_max = k_signed
        except Exception:
          k_signed_at_max = 0.0
          k_max = 0.0

        source_label = "地图融合" if src_val == 2 else "视觉"
        self._curve_speed_str = f"目标 {round(v_target_disp)} {speed_unit}"
        self._update_curve_icon_direction(k_signed_at_max, k_max, now, force=not prev_show)
        base = f"前方弯道 {max(0, int(round(dist_to_min)))} m"
        self._curve_state_str = source_label
        self._curve_dist_str = f"{base} · {source_label}"

        try:
          base_font = font_fallback(self._font_medium)
          if self._font_has_missing_glyphs(base_font, self._curve_dist_str):
            self._curve_dist_font = self._get_dynamic_unifont_font(self._curve_dist_str)
        except Exception:
          self._curve_dist_font = None

        try:
          base_font = font_fallback(self._font_bold)
          if self._font_has_missing_glyphs(base_font, self._curve_speed_str):
            self._curve_speed_font = self._get_dynamic_unifont_font(self._curve_speed_str)
        except Exception:
          self._curve_speed_font = None

        self._curve_show = True
        return

  def _draw_curve_speed_control(self) -> None:
    if not self._curve_show or self._set_speed_rect is None:
      return

    widget_size = int(UI_CONFIG.button_size * 1.25)
    x = self._set_speed_rect.x + self._set_speed_rect.width + UI_CONFIG.border_size
    y = self._set_speed_rect.y

    speed_text_size = 50
    dist_text_size = 34
    if self._curve_speed_font is not None:
      speed_metrics = self._measure_text_ex_no_fallback(self._curve_speed_font, self._curve_speed_str, speed_text_size)
    else:
      speed_metrics = measure_text_cached(self._font_bold, self._curve_speed_str, speed_text_size)

    if self._curve_dist_font is not None:
      dist_metrics = self._measure_text_ex_no_fallback(self._curve_dist_font, self._curve_dist_str, dist_text_size)
    else:
      dist_metrics = measure_text_cached(self._font_medium, self._curve_dist_str, dist_text_size)

    padding_x = 20.0
    # Keep a stable base width so the widget doesn't "jitter" as numbers update, but always
    # grow if the rendered text would overflow the background.
    min_width = float(widget_size) * 2.0
    csc_width = max(min_width, float(max(speed_metrics.x, dist_metrics.x)) + 2 * padding_x)

    icon = self._curve_speed_icon_l
    if self._curve_speed_flip and getattr(self._curve_speed_icon_r, "id", 0) != 0:
      icon = self._curve_speed_icon_r

    src_rect = rl.Rectangle(0, 0, float(icon.width), float(icon.height))
    icon_x = x + (widget_size - icon.width) / 2
    icon_y = y + (widget_size - icon.height) / 2
    dest_rect = rl.Rectangle(icon_x, icon_y, float(icon.width), float(icon.height))
    rl.draw_texture_pro(icon, src_rect, dest_rect, rl.Vector2(0, 0), 0.0, COLORS.white)

    csc_rect = rl.Rectangle(x, y + widget_size + 10, csc_width, 100.0)
    rl.draw_rectangle_rounded(csc_rect, 0.35, 10, COLORS.blue_translucent)
    rl.draw_rectangle_rounded_lines_ex(csc_rect, 0.35, 10, 10, COLORS.blue)

    total_h = dist_metrics.y + 6 + speed_metrics.y
    start_y = csc_rect.y + (csc_rect.height - total_h) / 2
    dist_pos = rl.Vector2(csc_rect.x + padding_x, start_y)
    speed_pos = rl.Vector2(csc_rect.x + padding_x, start_y + dist_metrics.y + 6)
    if self._curve_dist_font is not None:
      self._draw_text_ex_no_fallback(self._curve_dist_font, self._curve_dist_str, dist_pos, dist_text_size, 0, COLORS.white)
    else:
      rl.draw_text_ex(self._font_medium, self._curve_dist_str, dist_pos, dist_text_size, 0, COLORS.white)

    if self._curve_speed_font is not None:
      self._draw_text_ex_no_fallback(self._curve_speed_font, self._curve_speed_str, speed_pos, speed_text_size, 0, COLORS.white)
    else:
      rl.draw_text_ex(self._font_bold, self._curve_speed_str, speed_pos, speed_text_size, 0, COLORS.white)

  @staticmethod
  def _measure_text_ex_no_fallback(font: rl.Font, text: str, font_size: int, spacing: float = 0) -> rl.Vector2:
    try:
      return rl.measure_text_ex(font, text, font_size * FONT_SCALE, spacing)  # noqa: TID251
    except Exception:
      return rl.Vector2(0, 0)

  @staticmethod
  def _draw_text_ex_no_fallback(font: rl.Font, text: str, position: rl.Vector2, font_size: int, spacing: float, tint: rl.Color) -> None:
    # application.py patches rl.draw_text_ex to always apply font_fallback().
    # Use the original function so we can explicitly draw with a dynamic font.
    if hasattr(rl, "_orig_draw_text_ex"):
      rl._orig_draw_text_ex(font, text, position, font_size * FONT_SCALE, spacing, tint)
    else:
      rl.draw_text_ex(font, text, position, font_size, spacing, tint)

  @staticmethod
  def _font_has_missing_glyphs(font: rl.Font, text: str) -> bool:
    try:
      q = ord("?")
      q_idx = rl.get_glyph_index(font, q)
      for ch in text:
        cp = ord(ch)
        if cp == q:
          continue
        idx = rl.get_glyph_index(font, cp)
        if idx != q_idx:
          continue
        gi = rl.get_glyph_info(font, cp)
        if getattr(gi, "value", q) == q:
          return True
    except Exception:
      return False
    return False

  def _get_dynamic_unifont_font(self, text: str) -> "rl.Font | None":
    codepoints = tuple(sorted({ord(c) for c in text}))
    if not codepoints:
      return None

    now = time.monotonic()
    entry = self._curve_font_cache.get(codepoints)
    if entry is not None:
      entry.last_used_t = now
      return entry.font

    try:
      with as_file(FONT_DIR) as font_dir:
        font_path = font_dir / "unifont.otf"
        if not font_path.exists():
          return None

        cp_buffer = rl.ffi.new("int[]", list(codepoints))
        cp_ptr = rl.ffi.cast("int *", cp_buffer)
        # Match the atlas generation default (UNIFONT_SIZE=64) for good downscaled clarity.
        font = rl.load_font_ex(font_path.as_posix(), 64, cp_ptr, len(codepoints))

      if getattr(font, "glyphCount", 0) <= 0 or getattr(font, "texture", None) is None or font.texture.id == 0:
        try:
          rl.unload_font(font)
        except Exception:
          pass
        return None

      rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    except Exception:
      return None

    self._curve_font_cache[codepoints] = _DynamicFontCacheEntry(font=font, last_used_t=now)
    self._prune_curve_font_cache(max_entries=8)
    return font

  def _prune_curve_font_cache(self, max_entries: int) -> None:
    if len(self._curve_font_cache) <= max_entries:
      return

    items = sorted(self._curve_font_cache.items(), key=lambda kv: kv[1].last_used_t)
    for key, entry in items[: max(0, len(items) - max_entries)]:
      try:
        rl.unload_font(entry.font)
      except Exception:
        pass
      self._curve_font_cache.pop(key, None)
