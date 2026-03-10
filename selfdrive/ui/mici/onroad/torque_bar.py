import math
import time
from functools import wraps
from collections import OrderedDict

import numpy as np
import pyray as rl
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE, MAX_LATERAL_ACCEL_NO_ROLL

# TODO: arc_bar_pts doesn't consider rounded end caps part of the angle span
TORQUE_ANGLE_SPAN = 12.7

DEBUG = False
BASE_UI_WIDTH = 536
BASE_UI_HEIGHT = 240


def quantized_lru_cache(maxsize=128):
  def decorator(func):
    cache = OrderedDict()
    @wraps(func)
    def wrapper(cx, cy, r_mid, thickness, a0_deg, a1_deg, **kwargs):
      # Quantize inputs: balanced for smoothness vs cache effectiveness
      key = (round(cx), round(cy), round(r_mid),
             round(thickness),           # 1px precision for smoother height transitions
             round(a0_deg * 10) / 10,    # 0.1° precision for smoother angle transitions
             round(a1_deg * 10) / 10,
             tuple(sorted(kwargs.items())))

      if key in cache:
        cache.move_to_end(key)
      else:
        if len(cache) >= maxsize:
          cache.popitem(last=False)

        result = func(cx, cy, r_mid, thickness, a0_deg, a1_deg, **kwargs)
        cache[key] = result
      return cache[key]
    return wrapper
  return decorator


@quantized_lru_cache(maxsize=256)
def arc_bar_pts(cx: float, cy: float,
                r_mid: float, thickness: float,
                a0_deg: float, a1_deg: float,
                *, max_points: int = 100, cap_segs: int = 10,
                cap_radius: float = 7, px_per_seg: float = 2.0) -> np.ndarray:
  """Return Nx2 np.float32 points for a single closed polygon (rounded thick arc)."""

  def get_cap(left: bool, a_deg: float):
    # end cap at a1: center (a1), sweep a1→a1+180 (skip endpoints to avoid dupes)
    # quarter arc (outer corner) at a1 with fixed pixel radius cap_radius

    nx, ny = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))  # outward normal
    tx, ty = -ny, nx  # tangent (CCW)

    mx, my = cx + nx * r_mid, cy + ny * r_mid  # mid-point at a1
    if DEBUG:
      rl.draw_circle(int(mx), int(my), 4, rl.PURPLE)

    ex = mx + nx * (half - cap_radius)
    ey = my + ny * (half - cap_radius)

    if DEBUG:
      rl.draw_circle(int(ex), int(ey), 2, rl.WHITE)

    # sweep 90° in the local (t,n) frame: from outer edge toward inside
    if not left:
      alpha = np.deg2rad(np.linspace(90, 0, cap_segs + 2))[1:-1]
    else:
      alpha = np.deg2rad(np.linspace(180, 90, cap_segs + 2))[1:-1]
    cap_end = np.c_[ex + np.cos(alpha) * cap_radius * tx + np.sin(alpha) * cap_radius * nx,
                    ey + np.cos(alpha) * cap_radius * ty + np.sin(alpha) * cap_radius * ny]

    # bottom quarter (inner corner) at a1
    ex2 = mx + nx * (-half + cap_radius)
    ey2 = my + ny * (-half + cap_radius)
    if DEBUG:
      rl.draw_circle(int(ex2), int(ey2), 2, rl.WHITE)

    if not left:
      alpha2 = np.deg2rad(np.linspace(0, -90, cap_segs + 1))[:-1]  # include 0 once, exclude -90
    else:
      alpha2 = np.deg2rad(np.linspace(90 - 90 - 90, 0 - 90 - 90, cap_segs + 1))[:-1]
    cap_end_bot = np.c_[ex2 + np.cos(alpha2) * cap_radius * tx + np.sin(alpha2) * cap_radius * nx,
                        ey2 + np.cos(alpha2) * cap_radius * ty + np.sin(alpha2) * cap_radius * ny]

    # append to the top quarter
    if not left:
      cap_end = np.vstack((cap_end, cap_end_bot))
    else:
      cap_end = np.vstack((cap_end_bot, cap_end))

    return cap_end

  if a1_deg < a0_deg:
    a0_deg, a1_deg = a1_deg, a0_deg
  half = thickness * 0.5

  cap_radius = min(cap_radius, half)

  span = max(1e-3, a1_deg - a0_deg)

  # pick arc segment count from arc length, clamp to shader points[] budget
  arc_len = r_mid * math.radians(span)
  arc_segs = max(6, int(arc_len / px_per_seg))
  max_arc = (max_points - (4 * cap_segs + 3)) // 2
  arc_segs = max(6, min(arc_segs, max_arc))

  # outer arc a0→a1
  ang_o = np.deg2rad(np.linspace(a0_deg, a1_deg, arc_segs + 1))
  outer = np.c_[cx + np.cos(ang_o) * (r_mid + half),
                cy + np.sin(ang_o) * (r_mid + half)]

  # end cap at a1
  cap_end = get_cap(False, a1_deg)

  # inner arc a1→a0
  ang_i = np.deg2rad(np.linspace(a1_deg, a0_deg, arc_segs + 1))
  inner = np.c_[cx + np.cos(ang_i) * (r_mid - half),
                cy + np.sin(ang_i) * (r_mid - half)]

  # start cap at a0
  cap_start = get_cap(True, a0_deg)

  pts = np.vstack((outer, cap_end, inner, cap_start, outer[:1])).astype(np.float32)

  if DEBUG:
    n = len(pts)
    idx = int(time.monotonic() * 12) % max(1, n)  # speed: 12 pts/sec
    for i, (x, y) in enumerate(pts):
      j = (i - idx) % n  # rotate the gradient
      t = j / n
      color = rl.Color(255, int(255 * (1 - t)), int(255 * t), 255)
      rl.draw_circle(int(x), int(y), 2, color)

  return pts


class TorqueBar(Widget):
  def __init__(self, demo: bool = False):
    super().__init__()
    self._demo = demo
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._curve_intensity_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._torque_line_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._angle_mode = False
    self._scale = 1.0

  def update_filter(self, value: float):
    """Update the torque filter value (for demo mode)."""
    self._torque_filter.update(value)

  def _update_state(self):
    if self._demo:
      return

    sm = ui_state.sm
    controls_state = sm['controlsState']
    if controls_state.lateralControlState.which() == 'angleState':
      self._angle_mode = True

      # 1) Direction (sign) from desired curvature
      try:
        desired_curv = float(getattr(controls_state, "desiredCurvature", 0.0))
      except Exception:
        desired_curv = 0.0
      curv_sign = 1.0 if desired_curv >= 0.0 else -1.0

      # 2) Confidence from lane lines + road edges
      confidence = 0.0
      try:
        if sm.valid.get("modelV2", False):
          model = sm["modelV2"]
          lane_probs = list(getattr(model, "laneLineProbs", []) or [])
          road_edge_stds = list(getattr(model, "roadEdgeStds", []) or [])
          lane_conf = float(min(lane_probs[1], lane_probs[2])) if len(lane_probs) >= 3 else 0.0
          edge_conf = float(np.clip(1.0 - max(road_edge_stds), 0.0, 1.0)) if len(road_edge_stds) >= 1 else 0.0
          confidence = float(np.clip(0.7 * lane_conf + 0.3 * edge_conf, 0.0, 1.0))
      except Exception:
        confidence = 0.0

      # 3) Curvature magnitude (intensity)
      bar_mag = 0.0
      try:
        car_state = sm["carState"]
        live_parameters = sm["liveParameters"]
        v_ego = max(float(getattr(car_state, "vEgo", 0.0)), 1.0)
        roll = float(getattr(live_parameters, "roll", 0.0))
        max_lat_accel = float(MAX_LATERAL_ACCEL_NO_ROLL + roll * ACCELERATION_DUE_TO_GRAVITY)
        curv_limit = min(float(MAX_CURVATURE), max_lat_accel / max(v_ego * v_ego, 1e-6))
        curv_ratio = abs(desired_curv) / max(curv_limit, 1e-6)
        bar_mag = float(np.clip(curv_ratio, 0.0, 1.0))
      except Exception:
        bar_mag = 0.0

      self._torque_filter.update(curv_sign * confidence)
      self._curve_intensity_filter.update(bar_mag)
    else:
      self._angle_mode = False
      self._torque_filter.update(-sm['carOutput'].actuatorsOutput.torque)
      self._curve_intensity_filter.update(abs(self._torque_filter.x))

  def _render(self, rect: rl.Rectangle) -> None:
    # scale for screen size (C3 big UI vs small UI)
    base_scale = 1.0
    if rect.width > 0 and rect.height > 0:
      base_scale = min(rect.width / BASE_UI_WIDTH, rect.height / BASE_UI_HEIGHT)
    scale = self._scale * base_scale

    bar_mag = float(self._curve_intensity_filter.x if self._angle_mode else abs(self._torque_filter.x))

    # adjust y pos with torque/curvature magnitude
    torque_line_offset = np.interp(bar_mag, [0.5, 1], [22 * scale, 26 * scale])
    torque_line_height = np.interp(bar_mag, [0.5, 1], [14 * scale, 56 * scale])

    # animate alpha and angle span
    if not self._demo:
      alpha_target = 1.0 if (ui_state.status != UIStatus.DISENGAGED or ui_state.dp_alka_active) else 0.0
      self._torque_line_alpha_filter.update(alpha_target)
    else:
      self._torque_line_alpha_filter.update(1.0)

    # draw curved line polygon torque bar
    torque_line_radius = 1200 * scale
    top_angle = -90
    torque_bg_angle_span = self._torque_line_alpha_filter.x * TORQUE_ANGLE_SPAN
    torque_start_angle = top_angle - torque_bg_angle_span / 2
    torque_end_angle = top_angle + torque_bg_angle_span / 2
    # centerline radius & center (you already have these values)
    mid_r = torque_line_radius + torque_line_height / 2

    cx = rect.x + rect.width / 2 + 8 * scale  # offset to right of camera feed
    cy = rect.y + rect.height + torque_line_radius - torque_line_offset

    # draw bg torque indicator line
    bg_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, torque_start_angle, torque_end_angle, cap_radius=7 * scale)

    # draw torque indicator line
    a0s = top_angle
    a1s = a0s + torque_bg_angle_span / 2 * self._torque_filter.x
    sl_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, a0s, a1s, cap_radius=7 * scale)

    if self._angle_mode:
      confidence = abs(float(self._torque_filter.x))
      if confidence >= 0.7:
        base_color = rl.Color(0, 255, 120, 255)
      elif confidence >= 0.4:
        base_color = rl.Color(255, 200, 0, 255)
      else:
        base_color = rl.Color(255, 80, 80, 255)

      alpha_scale = (0.35 + 0.65 * bar_mag) * self._torque_line_alpha_filter.x
      bg_alpha = int(255 * alpha_scale * 0.35)
      fg_alpha = int(255 * alpha_scale)

      draw_polygon(rect, bg_pts, color=rl.Color(base_color.r, base_color.g, base_color.b, bg_alpha))
      draw_polygon(rect, sl_pts, color=rl.Color(base_color.r, base_color.g, base_color.b, fg_alpha))
    else:
      torque_line_bg_alpha = np.interp(abs(self._torque_filter.x), [0.5, 1.0], [0.25, 0.5])
      torque_line_bg_color = rl.Color(255, 255, 255, int(255 * torque_line_bg_alpha * self._torque_line_alpha_filter.x))
      if ui_state.status != UIStatus.ENGAGED and not self._demo:
        torque_line_bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * self._torque_line_alpha_filter.x))

      draw_polygon(rect, bg_pts, color=torque_line_bg_color)

      # draw beautiful gradient from center to 65% of the bg torque bar width
      start_grad_pt = cx / rect.width
      if self._torque_filter.x < 0:
        end_grad_pt = (cx * (1 - 0.65) + (min(bg_pts[:, 0]) * 0.65)) / rect.width
      else:
        end_grad_pt = (cx * (1 - 0.65) + (max(bg_pts[:, 0]) * 0.65)) / rect.width

      # fade to orange as we approach max torque
      start_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
        rl.Color(255, 200, 0, int(255 * self._torque_line_alpha_filter.x)),  # yellow
        max(0, abs(self._torque_filter.x) - 0.75) * 4,
      )
      end_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
        rl.Color(255, 115, 0, int(255 * self._torque_line_alpha_filter.x)),  # orange
        max(0, abs(self._torque_filter.x) - 0.75) * 4,
      )

      if ui_state.status != UIStatus.ENGAGED and not self._demo:
        start_color = end_color = rl.Color(255, 255, 255, int(255 * 0.35 * self._torque_line_alpha_filter.x))

      gradient = Gradient(
        start=(start_grad_pt, 0),
        end=(end_grad_pt, 0),
        colors=[
          start_color,
          end_color,
        ],
        stops=[0.0, 1.0],
      )

      draw_polygon(rect, sl_pts, gradient=gradient)

    # draw center torque bar dot
    if abs(self._torque_filter.x) < 0.5:
      dot_y = rect.y + rect.height - torque_line_offset - torque_line_height / 2
      rl.draw_circle(int(cx), int(dot_y), int(10 * scale) // 2,
                     rl.Color(182, 182, 182, int(255 * 0.9 * self._torque_line_alpha_filter.x)))
