import json
import math
import os
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib import curve_bend
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'


# ============================================================================
# HUD curve sign  [CURVE_SIGN_20261026]
#
# Ported from RAINNY4114/openpilot @ ford-chestnut.
#
# The curve params are shared with the longitudinal controller, so they are
# read straight off disk here instead of importing the whole control stack
# into the UI process.  /data/ford_params comes first: it is NOT the params
# dir, so Params::clearAll() (params.cc:208-224) never unlinks it.
# ============================================================================
PARAMS_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
CURVE_WINDOW_M_DEFAULT = 130
CURVE_K_ENTER_MILLI_DEFAULT = 4
# [CURVE_MAP_20261027] 地图弯道的 1 Hz 落盘位置，供离线核对（不含任何个人数据，
# 只有角度/曲率/车速/来源标记）。
CURVE_MAP_DIAG_PATH = "/data/curve_map_state.json"

# Hysteresis.  The first two match ford_curve_speed.py::_lincoln_curve_config()
# exactly, so "bright icon <=> the controller really is slowing down".  The
# last one is display-only debounce.
CURVE_K_EXIT_RATIO = 0.70    # controller: k_exit = k_enter * 0.70
CURVE_ACT_EXIT_H_S = 0.70    # controller: exit_h = 0.70 s
CURVE_EXIT_HOLD_S = 0.70     # display: hold the last frame when the gate closes
# Direction hysteresis: curvature jitters around 0, so the raw sign would make
# the icon flip left/right.  Below the deadband no direction is inferred; a new
# direction must persist before it is allowed to flip.
CURVE_DIR_DEADBAND = 2e-4    # curvature deadband (1/m)
CURVE_DIR_HOLD_S = 0.25      # a new direction must hold this long to flip


def _read_param_int_file(key: str, default: int) -> int:
  """Read a param file directly (libparams_c.so does not know the dp_* keys)."""
  for _d in PARAMS_DIRS:
    try:
      with open(os.path.join(_d, key), "r") as f:
        raw = f.read().strip()
      if raw:
        return int(raw)
    except Exception:
      continue
  return int(default)


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204


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
  # Curve widget (bright = the controller really is limiting speed)
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

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._set_speed_rect: rl.Rectangle | None = None

    # ---- HUD curve sign  [CURVE_SIGN_20261026] ----
    # Each direction has its own texture instead of mirroring one: some
    # devices/drivers handle a negative src_rect badly.
    try:
      self._curve_icon_l: rl.Texture = gui_app.texture("icons/curve_speed.png",
                                                       UI_CONFIG.button_size, UI_CONFIG.button_size)
      self._curve_icon_r: rl.Texture = gui_app.texture("icons/curveR_speed.png",
                                                       UI_CONFIG.button_size, UI_CONFIG.button_size)
    except Exception:
      self._curve_icon_l = None
      self._curve_icon_r = None

    self._curve_show: bool = False
    # [CURVE_DRAW_DIAG 20261027] 上一帧到底有没有真的画出来，以及卡在哪一步。
    # 会被 _write_curve_map_diag 以 1 Hz 写进 /data/curve_map_state.json
    # 的 drawn_prev / skip 字段 —— 否则"标志不显示"只能靠猜。
    self._curve_drawn: bool = False
    self._curve_skip: str = ""
    self._curve_active: bool = False
    self._curve_ctrl_active: bool = False
    self._curve_bend_deg: float = 0.0
    self._curve_k_smooth: float = 0.0
    self._curve_flip: bool = False
    self._curve_speed_str: str = ""
    self._curve_dist_str: str = ""
    self._curve_has_speed: bool = False
    self._curve_exit_timer: float = 0.0
    self._curve_act_timer: float = 0.0
    self._curve_last_update_t: float = time.monotonic()
    self._curve_flip_candidate: bool | None = None
    self._curve_flip_candidate_t: float = 0.0

    # ---- map curve  [CURVE_MAP_20261027] ----
    self._curve_map_deg: float = 0.0
    self._curve_map_dist: float = 0.0
    self._curve_map_ok: bool = False
    self._curve_map_src: str | None = None
    self._curve_diag_t: float = 0.0

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
    self._curve_ctrl_active = False
    self._curve_bend_deg = 0.0
    self._curve_speed_str = ""
    self._curve_dist_str = ""
    self._curve_has_speed = False
    self._curve_map_deg = 0.0
    self._curve_map_dist = 0.0
    self._curve_map_ok = False
    self._curve_map_src = None
    # Mirrors the controller's `if not self.curve_bend_ok:` branch: when the
    # gate closes the smoothed curvature and the latch are cleared together, so
    # the next appearance is not biased by a stale value.
    self._curve_k_smooth = 0.0
    self._curve_exit_timer = 0.0
    self._curve_act_timer = 0.0
    self._curve_flip_candidate = None
    self._curve_flip_candidate_t = 0.0

  def _update_curve_widget(self) -> None:
    """Curve sign + bend geometry.

    Two gates, deliberately separate:

      * control gate  `curve_bend.bend_deg_min()`  (default 45 deg)
        decides whether the car actually slows down.  Changing it changes
        braking (up to -3.2 m/s^2), so the display never touches it.
      * display gate  `curve_bend.bend_deg_min_display()`  (default 12 deg)
        decides whether the icon appears.  Much lower, so gentle highway
        curves get a hint without braking.  Measured over 116 routes, only
        15.5% ever reached 45 deg, and just 4 of the 102 highway routes did --
        sharing the 45 deg gate is why the sign "never triggered" before.

    Two display levels:
      * idle (grey border)  = a curve worth noticing, controller not slowing
      * active (blue border) = the replicated controller latch is engaged
    """
    now = time.monotonic()
    dt = max(0.0, min(0.2, now - self._curve_last_update_t))
    self._curve_last_update_t = now

    sm = ui_state.sm

    # ---- hard gates: hide immediately, no hysteresis ----
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

    # same smoothing as ford_curve_speed.py (alpha=0.6) so the display tracks
    # the quantity the controller actually acts on
    self._curve_k_smooth = 0.6 * k_max + 0.4 * self._curve_k_smooth

    # ---- replicate the controller latch (bright <=> really slowing) ----
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
      # the controller clears it the same way when `not curve_bend_ok`
      self._curve_active = False
      self._curve_act_timer = 0.0

    # ---- is the curve-speed controller actually limiting? ----
    # In the reference branch "bright <=> the controller is slowing down" holds
    # because ford_curve_speed.py publishes longitudinalPlanSP.curveSpeedActive.
    # This branch has no such controller, so nothing publishes it and that
    # equivalence does NOT hold.  The replicated latch above is kept for when the
    # controller lands, but the bright state is gated on the controller actually
    # reporting active -- otherwise the sign would claim a deceleration that is
    # not happening.
    self._curve_ctrl_active = False
    try:
      self._curve_ctrl_active = bool(sm["longitudinalPlanSP"].curveSpeedActive)
    except Exception:
      self._curve_ctrl_active = False

    # ---- map curve  [CURVE_MAP_20261027] ----
    # κ(s) 来自 mapd 写进 /dev/shm/params 的 MapCurvatures（中国 OSM 离线库），
    # 与视觉侧同一个物理量 ∫|κ|ds，所以两边可以直接对比。
    map_sum = curve_bend.map_curve_summary(v_ego, curve_bend.map_bend_horizon_m(), k_enter)
    map_deg = float(map_sum["bend_deg"])
    map_dist = float(map_sum["dist_m"])
    self._curve_map_deg = map_deg
    self._curve_map_dist = map_dist
    self._curve_map_ok = map_sum["src"] is not None
    self._curve_map_src = map_sum["src"]
    self._write_curve_map_diag(map_sum, bend_deg, k_enter)

    # ---- display gate (vision OR map, independent of the control gate) ----
    display_gate = curve_bend.bend_deg_min_display()
    vis_ok = bend_deg >= display_gate
    map_ok = self._curve_map_ok and map_deg >= display_gate
    if not (vis_ok or map_ok):
      # hold the last frame briefly so it does not flicker at the threshold
      if self._curve_show:
        self._curve_exit_timer += dt
        if self._curve_exit_timer < CURVE_EXIT_HOLD_S:
          return
      self._reset_curve_widget()
      return

    self._curve_exit_timer = 0.0

    speed_unit = tr("km/h") if ui_state.is_metric else tr("mph")

    # direction hysteresis; force on first show so it cannot flash the wrong way.
    # 方向永远由视觉 κ 的符号决定：MapCurvatures 里的 curvature 是否带符号
    # 在各版 mapd 之间不一致，用它定方向会让图标随机左右翻。
    self._update_curve_icon_direction(float(summary["k_signed"]), k_max, now,
                                      force=not self._curve_show)

    def _line(tag: str, deg: float, dist_m: float) -> str:
      if dist_m < 15.0:
        return f"{tag} {deg:.0f}\u00b0 | " + tr("NOW")
      return f"{tag} {deg:.0f}\u00b0 | {dist_m:.0f} m"

    # ---- line 1: 视觉弯角；视觉没到显示门限时让地图顶上 ----
    if vis_ok:
      self._curve_dist_str = _line(tr("CURVE"), bend_deg, float(summary["dist_m"]))
    else:
      self._curve_dist_str = _line(tr("MAP"), map_deg, map_dist)

    # ---- line 2: 真实车速（地图角度并存时并排）----
    # 这里显示的是**当前真实车速**（carState，与大号速度数字同源），
    # 不是任何控制器的目标车速 —— 本分支没有弯道减速控制器，
    # 显示"目标车速"就是在骗人。
    speed_str = f"{self.speed:.0f} {speed_unit}"
    if vis_ok and map_ok:
      self._curve_speed_str = f"{tr('MAP')} {map_deg:.0f}\u00b0 | {speed_str}"
    else:
      self._curve_speed_str = speed_str
    self._curve_has_speed = True

    self._curve_show = True

  def _write_curve_map_diag(self, map_sum: dict, vis_deg: float, k_enter: float) -> None:
    """把地图弯道的原始状态按 1 Hz 落盘（/data/curve_map_state.json）。

    停车时 mapd 的输出是空的，只有行驶中才有值。没有这个文件，事后就分不清
    "地图角度没显示"到底是没数据、还是算错了 —— 所以留一条 1 Hz 的痕迹。
    show_prev 是上一帧的标志显示状态（本函数在显示闸门之前调用）。
    """
    now = time.monotonic()
    if (now - self._curve_diag_t) < 1.0:
      return
    self._curve_diag_t = now
    try:
      age = map_sum.get("age_s")
      payload = {
        "t": round(time.time(), 3),
        "src": map_sum.get("src"),
        "age_s": None if age is None else round(float(age), 2),
        "n": int(map_sum.get("n", 0) or 0),
        "map_bend_deg": round(float(map_sum.get("bend_deg", 0.0)), 1),
        "map_k_max": round(float(map_sum.get("k_max", 0.0)), 5),
        "map_k_signed": round(float(map_sum.get("k_signed", 0.0)), 5),
        "map_dist_m": round(float(map_sum.get("dist_m", 0.0)), 1),
        "vis_bend_deg": round(float(vis_deg), 1),
        "k_enter": round(float(k_enter), 5),
        "v_kph": round(float(self.speed), 1),
        "show_prev": bool(self._curve_show),
        # 上一帧实际有没有画到屏幕上，以及没画的原因（not_shown / no_rect）
        "drawn_prev": bool(getattr(self, "_curve_drawn", False)),
        "skip": getattr(self, "_curve_skip", ""),
        "rect_ok": self._set_speed_rect is not None,
        "use_map_bend": curve_bend.use_map_bend(),
        "horizon_m": curve_bend.map_bend_horizon_m(),
      }
      tmp = CURVE_MAP_DIAG_PATH + ".tmp"
      with open(tmp, "w") as f:
        json.dump(payload, f)
      os.replace(tmp, CURVE_MAP_DIAG_PATH)
    except Exception:
      pass

  def _update_curve_icon_direction(self, k_signed: float, k_max: float, now: float,
                                   *, force: bool = False) -> None:
    """Left/right hysteresis (ported from RAINNY4114/openpilot @ ford).

    Curvature jitters around 0, so taking the raw sign would make the icon flip
    left/right.  Below CURVE_DIR_DEADBAND no direction is inferred; a new
    direction must persist for CURVE_DIR_HOLD_S before it may flip; and when the
    sign first appears force=True sets the direction outright, otherwise it would
    show one direction and then flip.
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
    """Curve sign (yellow bend plate) + cumulative turn angle / distance.

    bright (blue border) = the curve-speed controller really is limiting speed;
    idle (grey border)   = judged a curve but nothing is being limited.
    Bright additionally requires the controller to report active, which cannot
    happen on this branch yet (no ford_curve_speed.py), so today it stays idle.
    """
    if not self._curve_show:
      self._curve_drawn = False
      self._curve_skip = "not_shown"
      return
    if self._set_speed_rect is None:
      # 定位锚点是 MAX 速度框的 rect。sunnypilot 的 HudRendererSP 重写了
      # _draw_set_speed 却漏掉 `self._set_speed_rect = set_speed_rect`
      # ⇒ 这里恒为 None ⇒ 弯道标志算了但永远画不出来（2026-10-27 实测）。
      self._curve_drawn = False
      self._curve_skip = "no_rect"
      return
    self._curve_drawn = True
    self._curve_skip = ""

    widget_size = int(UI_CONFIG.button_size * 1.25)
    x = self._set_speed_rect.x + self._set_speed_rect.width + UI_CONFIG.border_size
    y = self._set_speed_rect.y

    # --- icon ---
    icon = self._curve_icon_r if self._curve_flip else self._curve_icon_l
    if icon is not None and getattr(icon, "id", 0) != 0:
      src_rect = rl.Rectangle(0, 0, float(icon.width), float(icon.height))
      icon_x = x + (widget_size - icon.width) / 2
      icon_y = y + (widget_size - icon.height) / 2
      dest_rect = rl.Rectangle(icon_x, icon_y, float(icon.width), float(icon.height))
      rl.draw_texture_pro(icon, src_rect, dest_rect, rl.Vector2(0, 0), 0.0, COLORS.WHITE)

    # --- info box ---
    dist_metrics = measure_text_cached(self._font_medium, self._curve_dist_str, FONT_SIZES.curve_dist)
    if self._curve_has_speed:
      speed_metrics = measure_text_cached(self._font_bold, self._curve_speed_str, FONT_SIZES.curve_speed)
      speed_w, speed_h = float(speed_metrics.x), float(speed_metrics.y)
    else:
      speed_w, speed_h = 0.0, 0.0

    padding_x = 20.0
    min_width = float(widget_size) * 2.0
    box_width = max(min_width, float(max(speed_w, dist_metrics.x)) + 2 * padding_x)

    if self._curve_active and self._curve_ctrl_active:
      bg = COLORS.BLUE_TRANSLUCENT
      border = COLORS.BLUE
    else:
      bg = COLORS.CURVE_IDLE_BG
      border = COLORS.CURVE_IDLE_BORDER

    box_height = 100.0 if self._curve_has_speed else 72.0
    box_rect = rl.Rectangle(x, y + widget_size + 10, box_width, box_height)
    rl.draw_rectangle_rounded(box_rect, 0.35, 10, bg)
    rl.draw_rectangle_rounded_lines_ex(box_rect, 0.35, 10, 10, border)

    total_h = dist_metrics.y + (6 + speed_h if self._curve_has_speed else 0.0)
    start_y = box_rect.y + (box_rect.height - total_h) / 2
    rl.draw_text_ex(self._font_medium, self._curve_dist_str,
                    rl.Vector2(box_rect.x + padding_x, start_y),
                    FONT_SIZES.curve_dist, 0, COLORS.WHITE)
    if self._curve_has_speed:
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
