import math
import pyray as rl
import time
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from dragonpilot.selfdrive.ui.onroad.torque_bar import TorqueBar

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
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


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._set_speed_rect: rl.Rectangle | None = None

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    self._torque_bar = TorqueBar()

    # NOTE: Prefer explicit L/R assets instead of texture mirroring; some devices/drivers don't render negative src_rect reliably.
    self._curve_speed_icon_l: rl.Texture = gui_app.texture("icons/curve_speed.png", UI_CONFIG.button_size, UI_CONFIG.button_size)
    self._curve_speed_icon_r: rl.Texture = gui_app.texture("icons/curveR_speed.png", UI_CONFIG.button_size, UI_CONFIG.button_size)
    self._curve_speed_str: str = ""
    self._curve_dist_str: str = ""
    self._curve_state_str: str = ""
    self._curve_speed_flip: bool = False
    self._curve_show: bool = False
    self._curve_k_smooth: float = 0.0
    self._curve_active: bool = False
    self._curve_exit_timer: float = 0.0
    self._curve_last_update_t: float = time.monotonic()

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
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

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

    if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
      self._torque_bar.render(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

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

  def _update_curve_speed_widget(self) -> None:
    now = time.monotonic()
    dt = max(0.0, min(0.2, now - self._curve_last_update_t))
    self._curve_last_update_t = now

    self._curve_show = False
    self._curve_speed_str = ""
    self._curve_dist_str = ""
    self._curve_state_str = ""

    sm = ui_state.sm
    try:
      car_params = sm["carParams"]
      if getattr(car_params, "brand", "") != "ford":
        self._curve_active = False
        self._curve_exit_timer = 0.0
        return
    except Exception:
      return

    try:
      if not ui_state.params.get_bool("dp_lincoln_curve_speed"):
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

    window_m = max(30, min(190, self._safe_int_param("dp_lincoln_curve_window_m", 130)))
    k_enter_milli = max(2, min(20, self._safe_int_param("dp_lincoln_curve_k_enter", 4)))
    k_enter = (k_enter_milli / 1000.0) * self._interp(v_ego, (0.0, 25.0, 40.0), (1.0, 0.9, 0.8))
    k_exit = k_enter * 0.70

    k_max = 0.0
    k_signed_at_max = 0.0
    dist_at_max = 0.0
    dist_at_enter = None
    for pos, v_pred, turn_rate in zip(positions, v_preds, turn_rates):
      try:
        pos_f = float(pos)
        if not math.isfinite(pos_f) or pos_f > window_m:
          continue
        v_pred_f = float(v_pred)
        turn_rate_f = float(turn_rate)
        if not math.isfinite(v_pred_f) or not math.isfinite(turn_rate_f):
          continue
      except Exception:
        continue

      v_pred_f = max(1.0, min(100.0, v_pred_f))
      k_signed = turn_rate_f / v_pred_f
      k = abs(k_signed)
      k = min(k, 0.02)
      if dist_at_enter is None and k >= k_enter:
        dist_at_enter = pos_f
      if k > k_max:
        k_max = k
        k_signed_at_max = k_signed
        dist_at_max = pos_f

    if k_max < 1e-4:
      self._curve_active = False
      self._curve_exit_timer = 0.0
      self._curve_k_smooth = 0.0
      return

    alpha = 0.6
    self._curve_k_smooth = alpha * k_max + (1.0 - alpha) * self._curve_k_smooth

    enter_now = k_max >= k_enter
    if self._curve_active:
      if self._curve_k_smooth < k_exit:
        self._curve_exit_timer += dt
        if self._curve_exit_timer > 0.70:
          self._curve_active = False
          self._curve_exit_timer = 0.0
      else:
        self._curve_exit_timer = 0.0
    else:
      if enter_now or self._curve_k_smooth >= k_enter:
        self._curve_active = True
        self._curve_exit_timer = 0.0

    if not self._curve_active:
      return

    v_limit = math.sqrt(1.0 / max(self._curve_k_smooth, 1e-4))
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    v_limit_disp = v_limit * speed_conversion
    if not math.isfinite(v_limit_disp) or v_limit_disp <= 0.0:
      return

    # Only show when curve target would limit the user's set speed.
    if v_limit_disp >= self.set_speed:
      return

    display_speed = min(self.speed, v_limit_disp)
    speed_unit = tr("km/h") if ui_state.is_metric else tr("mph")
    self._curve_speed_str = f"目标 {round(display_speed)} {speed_unit}"
    self._curve_speed_flip = k_signed_at_max >= 0.0

    dist_m = float(dist_at_enter if dist_at_enter is not None else dist_at_max)
    dist = dist_m
    unit = "m"

    decel_active = False
    try:
      a_ego = float(getattr(car_state, "aEgo", 0.0))
      if math.isfinite(a_ego) and a_ego < -0.30:
        decel_active = True
    except Exception:
      pass
    if not decel_active:
      try:
        if getattr(sm, "valid", {}).get("carOutput", False):
          brake_cmd = float(sm["carOutput"].actuatorsOutput.brake)
          if math.isfinite(brake_cmd) and brake_cmd >= 0.05:
            decel_active = True
      except Exception:
        pass

    self._curve_state_str = "减速中" if decel_active else "准备减速"
    self._curve_dist_str = f"前方弯道 {max(0, int(round(dist)))} {unit} · {self._curve_state_str}"
    self._curve_show = True

  def _draw_curve_speed_control(self) -> None:
    if not self._curve_show or self._set_speed_rect is None:
      return

    widget_size = int(UI_CONFIG.button_size * 1.25)
    x = self._set_speed_rect.x + self._set_speed_rect.width + UI_CONFIG.border_size
    y = self._set_speed_rect.y

    icon = self._curve_speed_icon_l
    if self._curve_speed_flip and getattr(self._curve_speed_icon_r, "id", 0) != 0:
      icon = self._curve_speed_icon_r

    src_rect = rl.Rectangle(0, 0, float(icon.width), float(icon.height))
    icon_x = x + (widget_size - icon.width) / 2
    icon_y = y + (widget_size - icon.height) / 2
    dest_rect = rl.Rectangle(icon_x, icon_y, float(icon.width), float(icon.height))
    rl.draw_texture_pro(icon, src_rect, dest_rect, rl.Vector2(0, 0), 0.0, COLORS.white)

    csc_rect = rl.Rectangle(x, y + widget_size + 10, float(widget_size), 100.0)
    rl.draw_rectangle_rounded(csc_rect, 0.35, 10, COLORS.blue_translucent)
    rl.draw_rectangle_rounded_lines_ex(csc_rect, 0.35, 10, 10, COLORS.blue)

    speed_text_size = 50
    dist_text_size = 34
    speed_metrics = measure_text_cached(self._font_bold, self._curve_speed_str, speed_text_size)
    dist_metrics = measure_text_cached(self._font_medium, self._curve_dist_str, dist_text_size)
    total_h = dist_metrics.y + 6 + speed_metrics.y
    start_y = csc_rect.y + (csc_rect.height - total_h) / 2
    dist_pos = rl.Vector2(csc_rect.x + 20, start_y)
    speed_pos = rl.Vector2(csc_rect.x + 20, start_y + dist_metrics.y + 6)
    rl.draw_text_ex(self._font_medium, self._curve_dist_str, dist_pos, dist_text_size, 0, COLORS.white)
    rl.draw_text_ex(self._font_bold, self._curve_speed_str, speed_pos, speed_text_size, 0, COLORS.white)
