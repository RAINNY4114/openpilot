import math
import os
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib import curve_bend
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

# 弯道相关参数与纵向控制器（ford_curve_speed.py）共用同一批 param 文件，
# 这里直接读文件，避免 UI 进程 import 整个纵向控制栈。
PARAMS_DIR = "/data/params/d"
CURVE_WINDOW_M_DEFAULT = 130
CURVE_K_ENTER_MILLI_DEFAULT = 4
CURVE_A_LAT_DEFAULT = 1.8

# --- HUD 弯道标志的迟滞参数 -------------------------------------------------
# 前三个与控制器 ford_curve_speed.py::_lincoln_curve_config() 严格一致，
# 这样"亮色 <=> 控制器真的在减速"；最后一个只用于显示防抖。
CURVE_K_EXIT_RATIO = 0.70    # 控制器: k_exit = k_enter * 0.70
CURVE_ACT_EXIT_H_S = 0.70    # 控制器: exit_h = 0.70 s
CURVE_EXIT_HOLD_S = 0.70     # 显示防抖：显示闸门刚消失时保持最后一帧
# 方向迟滞（移植自 RAINNY4114/openpilot @ ford 的 _update_curve_icon_direction）
CURVE_DIR_DEADBAND = 2e-4    # 曲率死区 (1/m)：低于此值不判方向，避免在 0 附近乱翻
CURVE_DIR_HOLD_S = 0.25      # 新方向必须保持这么久才允许翻转


def _read_param_int_file(key: str, default: int) -> int:
  try:
    with open(os.path.join(PARAMS_DIR, key), "r") as f:
      raw = f.read().strip()
    return int(raw) if raw else int(default)
  except Exception:
    return int(default)


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
  curve_speed: int = 50
  curve_dist: int = 34


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  # Curve-deceleration widget (active = actually limiting the speed)
  BLUE = rl.Color(0, 122, 255, 255)
  BLUE_TRANSLUCENT = rl.Color(0, 122, 255, 166)
  CURVE_IDLE_BG = rl.Color(0, 0, 0, 166)
  CURVE_IDLE_BORDER = rl.Color(166, 166, 166, 200)


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

    # ---- Curve-deceleration widget (ported from RAINNY4114/openpilot @ ford) ----
    # 弯道标志 + 弯道减速信息。左右两个方向用各自的图标，不做纹理镜像
    # （部分设备/驱动对负 src_rect 支持不好）。
    try:
      self._curve_icon_l: rl.Texture = gui_app.texture("icons/curve_speed.png",
                                                       UI_CONFIG.button_size, UI_CONFIG.button_size)
      self._curve_icon_r: rl.Texture = gui_app.texture("icons/curveR_speed.png",
                                                       UI_CONFIG.button_size, UI_CONFIG.button_size)
    except Exception:
      self._curve_icon_l = None
      self._curve_icon_r = None

    self._curve_show: bool = False
    self._curve_active: bool = False
    self._curve_bend_deg: float = 0.0
    self._curve_k_smooth: float = 0.0
    self._curve_flip: bool = False
    self._curve_speed_str: str = ""
    self._curve_dist_str: str = ""
    self._curve_exit_timer: float = 0.0
    self._curve_act_timer: float = 0.0
    self._curve_last_update_t: float = time.monotonic()
    self._curve_flip_candidate: bool | None = None
    self._curve_flip_candidate_t: float = 0.0

  # ------------------------------------------------------------------ state
  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self._reset_curve_widget()
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
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

    self._update_curve_widget()

  def _reset_curve_widget(self) -> None:
    self._curve_show = False
    self._curve_active = False
    self._curve_bend_deg = 0.0
    self._curve_speed_str = ""
    self._curve_dist_str = ""
    # 与控制器 ford_curve_speed.py 的 `if not self.curve_bend_ok:` 分支一致：
    # 闸门关闭时把平滑量与锁存一起清零，避免重新亮起时被旧值带偏。
    self._curve_k_smooth = 0.0
    self._curve_exit_timer = 0.0
    self._curve_act_timer = 0.0
    self._curve_flip_candidate = None
    self._curve_flip_candidate_t = 0.0

  def _update_curve_widget(self) -> None:
    """弯道标志 + 目标车速。

    设计参照 RAINNY4114/openpilot @ ford 的 hud_renderer.py
    （`_update_curve_speed_widget` / `_update_curve_icon_direction`）。
    参考实现的触发语义是「**弯道限速真的在起作用**」—— 它直接读控制器发布的
    `longitudinalPlan.curveSpeedSource` / `speeds`，所以显示与控制不可能不一致。

    本车做不到这一点：cereal/log.capnp 里没有弯道字段，UI 读不到控制器发布的
    弯道目标车速，只能自己用 curve_bend 重算。于是有两处必须修正：

    1. **45 度闸门在高速上几乎不可达。**
       控制侧 `bend_deg_min`（默认 45 度）是"要不要弯道减速"的闸门，改它会直接
       改变刹车（最大 -3.2 m/s^2），所以不能动。但它被同时当成显示闸门就太高了：
         实测 116 条路线累计转角峰值 >=20 度占 50%、>=30 度占 26%、>=45 度仅 16%
         （102 条高速路线里只有 4 条曾达到 45 度）
         80 条路线 96000 帧中 has_curve 仅 10.95% 为真
       => 这就是"转弯时弯道标志从未触发"。
       现在显示闸门改用独立的 `bend_deg_min_display`（默认 20 度，同样热加载），
       控制闸门 `bend_deg_min` 完全不动 —— **刹车行为不变**。

    2. **原 UI 少了控制器的锁存（latch）。**
       控制器的有效闸门不是 `has_curve` 本身，而是
           `curve_bend_ok AND curve_active`
       其中 `curve_active` 是锁存的：窗口内出现 |k| >= k_enter 就置位，
       直到 `curve_k_smooth < k_exit`(=0.70*k_enter) 持续 `exit_h`(=0.70s) 才释放。
       原 UI 每帧重新判瞬时 `has_curve`，于是控制器仍在减速、但 k 已降到 k_enter
       以下的退出窗口里，标志会先灭掉。现在 UI 复刻同一个锁存。

    显示分两级：
      * 暗色 (idle)   = 前方有值得注意的弯（bend_deg >= 显示闸门），但控制器不减速
      * 亮色 (active) = 控制器确实在按弯道限速减速（复刻的锁存为真）
    """
    now = time.monotonic()
    dt = max(0.0, min(0.2, now - self._curve_last_update_t))
    self._curve_last_update_t = now

    sm = ui_state.sm

    # ---- 硬闸门：立刻隐藏，不做迟滞 ----
    if not self.is_cruise_set:
      self._reset_curve_widget()
      return
    if sm.recv_frame["modelV2"] < ui_state.started_frame:
      self._reset_curve_widget()
      return

    try:
      model = sm["modelV2"]
      v_ego = float(sm["carState"].vEgo)
    except Exception:
      self._reset_curve_widget()
      return
    if not math.isfinite(v_ego) or v_ego < 1.0:
      self._reset_curve_widget()
      return

    k_enter = _read_param_int_file("dp_lincoln_curve_k_enter",
                                   CURVE_K_ENTER_MILLI_DEFAULT) * 1e-3
    k_enter = max(2e-3, min(20e-3, k_enter))
    window_m = float(max(30, min(190, _read_param_int_file("dp_lincoln_curve_window_m",
                                                          CURVE_WINDOW_M_DEFAULT))))

    summary = curve_bend.curve_summary_from_model(model, v_ego, k_enter, window_m)
    bend_deg = float(summary["bend_deg"])
    self._curve_bend_deg = bend_deg

    k_max = float(summary["k_max"])
    if not math.isfinite(k_max) or k_max < 1e-4:
      k_max = 0.0

    # 与 ford_curve_speed.py 相同的平滑（alpha=0.6），让显示贴近实际控制量
    self._curve_k_smooth = 0.6 * k_max + 0.4 * self._curve_k_smooth

    # ---- 复刻控制器的锁存 curve_active（亮色 <=> 控制器真的在减速）----
    if bend_deg >= curve_bend.bend_deg_min():
      if self._curve_active:
        if self._curve_k_smooth < k_enter * CURVE_K_EXIT_RATIO:
          self._curve_act_timer += dt
          if self._curve_act_timer > CURVE_ACT_EXIT_H_S:
            self._curve_active = False
            self._curve_act_timer = 0.0
        else:
          self._curve_act_timer = 0.0
      elif bool(summary["has_curve"]) or self._curve_k_smooth >= k_enter:
        self._curve_active = True
        self._curve_act_timer = 0.0
    else:
      # 控制器在 `not curve_bend_ok` 时也是这么清的
      self._curve_active = False
      self._curve_act_timer = 0.0

    # ---- 显示闸门（独立于控制闸门）----
    if bend_deg < curve_bend.bend_deg_min_display():
      # 显示防抖：闸门刚消失时保留最后一帧，避免在阈值附近闪烁
      if self._curve_show:
        self._curve_exit_timer += dt
        if self._curve_exit_timer < CURVE_EXIT_HOLD_S:
          return
      self._reset_curve_widget()
      return

    self._curve_exit_timer = 0.0

    k_smooth = max(self._curve_k_smooth, 1e-4)
    a_lat = float(CURVE_A_LAT_DEFAULT)
    v_limit = math.sqrt(a_lat / k_smooth)

    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    speed_unit = tr("km/h") if ui_state.is_metric else tr("mph")
    set_speed_ms = self.set_speed / speed_conversion

    v_target_disp = min(v_limit, set_speed_ms) * speed_conversion
    if not math.isfinite(v_target_disp) or v_target_disp <= 0.0:
      self._reset_curve_widget()
      return

    # 方向迟滞：首次显示时直接给定，避免先闪一下再翻
    self._update_curve_icon_direction(float(summary["k_signed"]), k_max, now,
                                      force=not self._curve_show)

    dist = float(summary["dist_m"])
    if dist < 15.0:
      self._curve_dist_str = tr("CURVE") + f" {bend_deg:.0f}\u00b0 | " + tr("NOW")
    else:
      self._curve_dist_str = tr("CURVE") + f" {bend_deg:.0f}\u00b0 | {dist:.0f} m"
    self._curve_speed_str = f"{v_target_disp:.0f} {speed_unit}"
    self._curve_show = True

  def _update_curve_icon_direction(self, k_signed: float, k_max: float, now: float,
                                   *, force: bool = False) -> None:
    """左右方向的迟滞 —— 移植自 RAINNY4114/openpilot @ ford。

    曲率在 0 附近抖动时，直接取符号会让图标左右乱闪。这里：
      * 曲率低于 CURVE_DIR_DEADBAND 时不判方向（保持上一次）；
      * 新方向必须持续 CURVE_DIR_HOLD_S 才允许翻转；
      * 标志刚出现时 force=True，直接给定方向（否则会先显示一个错的方向再翻）。
    """
    try:
      k_s = float(k_signed)
      k_a = float(k_max)
    except Exception:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if (not math.isfinite(k_s)) or (not math.isfinite(k_a)) or k_a < CURVE_DIR_DEADBAND:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    new_flip = bool(k_s >= 0.0)

    if force:
      self._curve_flip = new_flip
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if new_flip == self._curve_flip:
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0
      return

    if self._curve_flip_candidate != new_flip:
      self._curve_flip_candidate = new_flip
      self._curve_flip_candidate_t = float(now)
      return

    if (now - float(self._curve_flip_candidate_t)) >= CURVE_DIR_HOLD_S:
      self._curve_flip = new_flip
      self._curve_flip_candidate = None
      self._curve_flip_candidate_t = 0.0

  # ------------------------------------------------------------------ render
  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_curve_widget()

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    self._set_speed_rect = set_speed_rect
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

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

  def _draw_curve_widget(self) -> None:
    """弯道标志（黄色菱形弯道牌）+ 累计转角 / 距离 / 目标车速。

    active（蓝框）= 弯道真的在限速；idle（灰框）= 判定为弯道但不需要减速。
    """
    if not self._curve_show or self._set_speed_rect is None:
      return

    widget_size = int(UI_CONFIG.button_size * 1.25)
    x = self._set_speed_rect.x + self._set_speed_rect.width + UI_CONFIG.border_size
    y = self._set_speed_rect.y

    # --- 图标 ---
    icon = self._curve_icon_r if self._curve_flip else self._curve_icon_l
    if icon is not None and getattr(icon, "id", 0) != 0:
      src_rect = rl.Rectangle(0, 0, float(icon.width), float(icon.height))
      icon_x = x + (widget_size - icon.width) / 2
      icon_y = y + (widget_size - icon.height) / 2
      dest_rect = rl.Rectangle(icon_x, icon_y, float(icon.width), float(icon.height))
      rl.draw_texture_pro(icon, src_rect, dest_rect, rl.Vector2(0, 0), 0.0, COLORS.WHITE)

    # --- 信息框 ---
    speed_metrics = measure_text_cached(self._font_bold, self._curve_speed_str, FONT_SIZES.curve_speed)
    dist_metrics = measure_text_cached(self._font_medium, self._curve_dist_str, FONT_SIZES.curve_dist)

    padding_x = 20.0
    min_width = float(widget_size) * 2.0
    box_width = max(min_width, float(max(speed_metrics.x, dist_metrics.x)) + 2 * padding_x)

    if self._curve_active:
      bg = COLORS.BLUE_TRANSLUCENT
      border = COLORS.BLUE
    else:
      bg = COLORS.CURVE_IDLE_BG
      border = COLORS.CURVE_IDLE_BORDER

    box_rect = rl.Rectangle(x, y + widget_size + 10, box_width, 100.0)
    rl.draw_rectangle_rounded(box_rect, 0.35, 10, bg)
    rl.draw_rectangle_rounded_lines_ex(box_rect, 0.35, 10, 10, border)

    total_h = dist_metrics.y + 6 + speed_metrics.y
    start_y = box_rect.y + (box_rect.height - total_h) / 2
    rl.draw_text_ex(self._font_medium, self._curve_dist_str,
                    rl.Vector2(box_rect.x + padding_x, start_y),
                    FONT_SIZES.curve_dist, 0, COLORS.WHITE)
    rl.draw_text_ex(self._font_bold, self._curve_speed_str,
                    rl.Vector2(box_rect.x + padding_x, start_y + dist_metrics.y + 6),
                    FONT_SIZES.curve_speed, 0, COLORS.WHITE)

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
