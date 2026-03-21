import numpy as np
import pyray as rl
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE, MAX_LATERAL_ACCEL_NO_ROLL

# TODO: arc_bar_pts doesn't consider rounded end caps part of the angle span
TORQUE_ANGLE_SPAN = 12.7

DEBUG = False
BASE_UI_WIDTH = 536
BASE_UI_HEIGHT = 240


class TorqueBar(Widget):
  def __init__(self, demo: bool = False):
    super().__init__()
    self._demo = demo
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._curve_intensity_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._confidence_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
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

      # For angle-mode platforms like Ford/Lincoln, the bar should read like
      # lateral control intent, not lane confidence. Use desired curvature
      # magnitude for the left/right span, and keep confidence for color/alpha.
      self._torque_filter.update(curv_sign * bar_mag)
      self._curve_intensity_filter.update(bar_mag)
      self._confidence_filter.update(confidence)
    else:
      self._angle_mode = False
      self._confidence_filter.update(0.0)
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
    # centerline radius & center
    mid_r = torque_line_radius + torque_line_height / 2

    cx = rect.x + rect.width / 2 + (8 * scale)
    cy = rect.y + rect.height + torque_line_radius - torque_line_offset

    # SCALED: pass cap_radius explicitly so the corners round properly
    scaled_cap_radius = 7 * scale

    # draw bg torque indicator line
    bg_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, torque_start_angle, torque_end_angle,
                         cap_radius=scaled_cap_radius)

    # draw torque indicator line
    a0s = top_angle
    a1s = a0s + torque_bg_angle_span / 2 * self._torque_filter.x
    sl_pts = arc_bar_pts(cx, cy, mid_r, torque_line_height, a0s, a1s,
                         cap_radius=scaled_cap_radius)
    if self._angle_mode:
      confidence = float(np.clip(self._confidence_filter.x, 0.0, 1.0))
      if confidence >= 0.7:
        base_color = rl.Color(0, 255, 120, 255)
      elif confidence >= 0.4:
        base_color = rl.Color(255, 200, 0, 255)
      else:
        base_color = rl.Color(255, 80, 80, 255)

      confidence_alpha = np.interp(confidence, [0.0, 1.0], [0.35, 1.0])
      magnitude_alpha = np.interp(bar_mag, [0.0, 1.0], [0.30, 1.0])
      alpha_scale = confidence_alpha * magnitude_alpha * self._torque_line_alpha_filter.x
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
