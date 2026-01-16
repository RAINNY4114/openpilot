import colorsys
import math
import numpy as np
import pyray as rl
from cereal import messaging, car
from dataclasses import dataclass, field
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]

# dp
DP_RAINBOW_SCROLL_SPEED_FACTOR = 20.0
DP_RAINBOW_NUM_REPEATS = 3
DP_RAINBOW_ALPHA = 128
DP_RAINBOW_GRADIENT_SAMPLES = 20
DP_RAINBOW_HUE_SECTORS = 6

# Lincoln HUD enhancements
# NOTE: This is a radar/model-projected occupancy box (not pixel-perfect vision bbox).
# Tune it to "feel" like a vehicle outline without becoming huge at close range.
DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_NEAR = 0.90
DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_FAR = 1.10
DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_NEAR = 0.70
DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_FAR = 0.62
DP_LINCOLN_LEAD_BOX_SMOOTHING_FAR = 0.85
DP_LINCOLN_LEAD_BOX_SMOOTHING_NEAR = 0.55
DP_LINCOLN_LEAD_EFFECT_START_M = 130.0
DP_LINCOLN_LEAD_EFFECT_FULL_M = 30.0
DP_LINCOLN_LEAD_EFFECT_TTC_START_S = 5.0
DP_LINCOLN_LEAD_EFFECT_TTC_FULL_S = 2.5
DP_LINCOLN_LEAD_COLOR_FAR = rl.Color(255, 215, 0, 255)
DP_LINCOLN_LEAD_COLOR_NEAR = rl.Color(255, 60, 0, 255)


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  fill_alpha: int = 0
  # dp
  d_rel: float = 0.0
  x: float = 0.0
  y: float = 0.0
  sz: float = 0.0
  v_lead: float = 0.0

@dataclass
class DpUiLeadMode:
  off = 0
  lead = 1
  radar = 2
  all = 3


class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._curve_path_filter = FirstOrderFilter(0.0, 1.0, 1 / gui_app.target_fps)
    self._curve_path_alert = 0.0
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

    # dp
    self._dp_ui_rainbow_rotation = 0.0
    self._dp_ui_rainbow_gradient = None

    # Lincoln HUD enhancements
    self._lead_box_valid = False
    self._lead_box_alpha = 0.0
    self._lead_box_x = 0.0
    self._lead_box_y = 0.0
    self._lead_box_w = 0.0
    self._lead_box_h = 0.0
    self._lead_box_intensity = 0.0

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    # Check if data is up-to-date
    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode

    live_calib = sm['liveCalibration']
    self._path_offset_z = live_calib.height[0] if live_calib.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = radar_state is not None and (self._longitudinal_control or ui_state.dp_lincoln_hud_enhanced)

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        v_ego = float(sm["carState"].vEgo) if sm.valid.get("carState", False) else 0.0
        self._update_leads(radar_state, model, v_ego, path_x_array)
      self._transform_dirty = False

    # dp - draw live tracks before everything
    if ui_state.dp_ui_lead in [DpUiLeadMode.radar, DpUiLeadMode.all] and sm.valid['liveTracks']:
      self._draw_live_tracks(sm)

    # Draw elements
    self._update_curve_path_alert(sm)
    self._draw_lane_lines()
    self._draw_path(sm)

    if render_lead_indicator and radar_state:
      self._draw_lead_indicator()

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, model, v_ego: float, path_x_array):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    self._update_lead_box(radar_state.leadOne if radar_state is not None else None, model, v_ego, path_x_array)

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        v_lead = float(getattr(lead_data, "vLead", 0.0))
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, v_lead, point, self._rect)

  def _update_lead_box(self, radar_lead_one, model, v_ego: float, path_x_array):
    if not ui_state.dp_lincoln_hud_enhanced:
      self._lead_box_valid = False
      self._lead_box_alpha = 0.0
      self._lead_box_intensity = 0.0
      return

    lead_available = False
    d_rel = 0.0
    y_rel = 0.0
    v_rel = 0.0

    if radar_lead_one is not None and getattr(radar_lead_one, "status", False):
      lead_available = True
      d_rel = float(radar_lead_one.dRel)
      y_rel = float(radar_lead_one.yRel)
      v_rel = float(getattr(radar_lead_one, "vRel", 0.0))
    else:
      # Fallback to model lead when radar lead is missing/unreliable (e.g., radarless or brief detections).
      try:
        model_lead = model.leadsV3[0]
        prob = float(model_lead.prob)
        x0 = float(model_lead.x[0]) if len(model_lead.x) > 0 else float("nan")
        y0 = float(model_lead.y[0]) if len(model_lead.y) > 0 else float("nan")
        v0 = float(model_lead.v[0]) if len(model_lead.v) > 0 else float("nan")
      except Exception:
        prob, x0, y0, v0 = 0.0, float("nan"), float("nan"), float("nan")

      if prob >= 0.55 and np.isfinite(x0) and np.isfinite(y0) and x0 > 0.0:
        lead_available = True
        d_rel = x0
        # Model lead y is in device frame (left-positive). Radar lead yRel is the opposite, so invert here
        # to keep the existing -y_rel mapping below.
        y_rel = -y0
        v_rel = v0 - float(v_ego)

    if not lead_available:
      self._lead_box_intensity = 0.0
      self._lead_box_alpha = max(0.0, self._lead_box_alpha - 0.20)
      if self._lead_box_alpha <= 0.0:
        self._lead_box_valid = False
      return

    if not (d_rel > 0.0 and np.isfinite(d_rel) and np.isfinite(y_rel)):
      return

    dist_intensity = float(np.clip(np.interp(d_rel, [DP_LINCOLN_LEAD_EFFECT_FULL_M, DP_LINCOLN_LEAD_EFFECT_START_M], [1.0, 0.0]), 0.0, 1.0))
    ttc_intensity = 0.0
    if v_rel < -0.1:
      ttc = d_rel / max(0.1, -v_rel)
      ttc_intensity = float(np.clip(np.interp(ttc, [DP_LINCOLN_LEAD_EFFECT_TTC_FULL_S, DP_LINCOLN_LEAD_EFFECT_TTC_START_S], [1.0, 0.0]), 0.0, 1.0))
    intensity = max(dist_intensity, ttc_intensity)

    # Hide until we're meaningfully close/closing; fade out smoothly when far
    if intensity <= 0.01:
      self._lead_box_intensity = 0.0
      self._lead_box_alpha = max(0.0, self._lead_box_alpha - 0.10)
      if self._lead_box_alpha <= 0.0:
        self._lead_box_valid = False
      return

    idx = self._get_path_length_idx(path_x_array, d_rel)
    z_on_path = float(self._path_offset_z)
    if 0 <= idx < len(self._path.raw_points):
      z_on_path += float(self._path.raw_points[idx, 2])

    # Make the box narrower when near so it doesn't cover adjacent vehicles
    half_width_m = float(np.interp(d_rel, [DP_LINCOLN_LEAD_EFFECT_FULL_M, DP_LINCOLN_LEAD_EFFECT_START_M],
                                   [DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_NEAR, DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_FAR]))

    left_pt = self._map_to_screen(d_rel, -y_rel - half_width_m, z_on_path)
    right_pt = self._map_to_screen(d_rel, -y_rel + half_width_m, z_on_path)
    if left_pt is None or right_pt is None:
      return

    width_px = abs(float(right_pt[0]) - float(left_pt[0]))
    if width_px < 8.0 or not np.isfinite(width_px):
      return

    center_x = (float(left_pt[0]) + float(right_pt[0])) / 2.0
    bottom_y = (float(left_pt[1]) + float(right_pt[1])) / 2.0
    height_ratio = float(np.interp(d_rel, [DP_LINCOLN_LEAD_EFFECT_FULL_M, DP_LINCOLN_LEAD_EFFECT_START_M],
                                   [DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_NEAR, DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_FAR]))
    height_px = width_px * height_ratio

    margin_px = max(6.0, width_px * 0.03)
    x = center_x - width_px / 2.0 - margin_px
    y = bottom_y - height_px
    w = width_px + 2.0 * margin_px
    h = height_px

    # Clamp to viewport
    x = float(np.clip(x, self._rect.x, self._rect.x + self._rect.width - w))
    y = float(np.clip(y, self._rect.y, self._rect.y + self._rect.height - h))

    # Make updates more responsive when the lead is close/closing
    smoothing = float(np.interp(intensity, [0.0, 1.0], [DP_LINCOLN_LEAD_BOX_SMOOTHING_FAR, DP_LINCOLN_LEAD_BOX_SMOOTHING_NEAR]))

    if self._lead_box_valid:
      a = smoothing
      self._lead_box_x = self._lead_box_x * a + x * (1.0 - a)
      self._lead_box_y = self._lead_box_y * a + y * (1.0 - a)
      self._lead_box_w = self._lead_box_w * a + w * (1.0 - a)
      self._lead_box_h = self._lead_box_h * a + h * (1.0 - a)
    else:
      self._lead_box_valid = True
      self._lead_box_x, self._lead_box_y, self._lead_box_w, self._lead_box_h = x, y, w, h

    self._lead_box_intensity = intensity
    self._lead_box_alpha = min(1.0, self._lead_box_alpha + 0.25)

  def _draw_lead_box(self):
    if not (ui_state.dp_lincoln_hud_enhanced and self._lead_box_valid) or self._lead_box_alpha <= 0.01:
      return

    t = float(np.clip(self._lead_box_intensity, 0.0, 1.0))
    r = int(DP_LINCOLN_LEAD_COLOR_FAR.r + t * (DP_LINCOLN_LEAD_COLOR_NEAR.r - DP_LINCOLN_LEAD_COLOR_FAR.r))
    g = int(DP_LINCOLN_LEAD_COLOR_FAR.g + t * (DP_LINCOLN_LEAD_COLOR_NEAR.g - DP_LINCOLN_LEAD_COLOR_FAR.g))
    b = int(DP_LINCOLN_LEAD_COLOR_FAR.b + t * (DP_LINCOLN_LEAD_COLOR_NEAR.b - DP_LINCOLN_LEAD_COLOR_FAR.b))

    alpha = int(np.clip(self._lead_box_alpha * (110.0 + 145.0 * t), 0, 255))
    color = rl.Color(r, g, b, alpha)
    rect = rl.Rectangle(self._lead_box_x, self._lead_box_y, self._lead_box_w, self._lead_box_h)

    thickness = 3
    rl.draw_rectangle_rounded_lines_ex(rect, 0.10, 8, thickness, color)

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    for i, lane_line in enumerate(self._lane_lines):
      if ui_state.dp_lincoln_hud_enhanced:
        base = 0.055 if i in (1, 2) else 0.050
        line_width = base + 0.015 * self._lane_line_probs[i]
      else:
        line_width = 0.025 * self._lane_line_probs[i]
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, line_width, 0.0, max_idx, max_distance
      )

    # Update road edges using raw points
    road_edge_width = 0.08 if ui_state.dp_lincoln_hud_enhanced else 0.025
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, road_edge_width, 0.0, max_idx, max_distance)

    # Update path using raw points
    if lead and lead.status:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, 0.9, self._path_offset_z, max_idx, max_distance, allow_invert=False
    )

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=segment_colors,
      stops=gradient_stops,
    )

  def _update_lead_vehicle(self, d_rel, v_rel, v_lead, point, rect):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha), d_rel=d_rel, x=x, y=y, sz=sz, v_lead=v_lead)

  def _update_curve_path_alert(self, sm) -> None:
    target = 0.0

    if not ui_state.started:
      self._curve_path_alert = float(self._curve_path_filter.update(0.0))
      return

    try:
      if not ui_state.params.get_bool("CurveSpeedControl"):
        self._curve_path_alert = float(self._curve_path_filter.update(0.0))
        return
    except Exception:
      self._curve_path_alert = float(self._curve_path_filter.update(0.0))
      return

    if sm.recv_frame["longitudinalPlan"] < ui_state.started_frame:
      self._curve_path_alert = float(self._curve_path_filter.update(0.0))
      return

    long_plan = sm["longitudinalPlan"]
    src_val = 0
    try:
      src = getattr(long_plan, "curveSpeedSource", None)
      raw = getattr(src, "raw", None)
      src_val = int(raw if raw is not None else src)
    except Exception:
      src_val = 0

    if src_val in (1, 2):
      target = 1.0
      try:
        speeds = list(getattr(long_plan, "speeds", []) or [])
      except Exception:
        speeds = []

      v_ego = float(getattr(sm["carState"], "vEgo", 0.0))
      if speeds and math.isfinite(v_ego) and v_ego > 1.0:
        try:
          v_min = float(min(float(v) for v in speeds if math.isfinite(float(v)) and float(v) > 0.0))
        except Exception:
          v_min = 0.0
        if v_min > 0.0:
          drop_ratio = (v_ego - v_min) / max(v_ego * 0.35, 1.0)
          target = float(np.clip(drop_ratio, 0.0, 1.0))
      target = max(target, 0.25)

    self._curve_path_alert = float(self._curve_path_filter.update(target))

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      prob = float(self._lane_line_probs[i])
      if ui_state.dp_lincoln_hud_enhanced:
        alpha = float(np.clip(prob * 1.35, 0.0, 1.0))
        if prob < 0.05:
          alpha = 0.0
        else:
          alpha = max(alpha, 0.40)
      else:
        alpha = float(np.clip(prob, 0.0, 0.7))

      is_primary_lane_boundary = i in (1, 2)
      if ui_state.dp_lincoln_hud_enhanced:
        if is_primary_lane_boundary:
          alpha = max(alpha, 0.80 if ui_state.status == UIStatus.ENGAGED else 0.55)
          color = rl.Color(0, 255, 80, int(alpha * 255))
        else:
          alpha = max(alpha, 0.55 if ui_state.status == UIStatus.ENGAGED else 0.45)
          color = rl.Color(0, 220, 70, int(alpha * 255))
      else:
        color = rl.Color(255, 255, 255, int(alpha * 255))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      confidence = float(np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0))
      if ui_state.dp_lincoln_hud_enhanced:
        alpha = max(0.45, confidence ** 0.5)
        color = rl.Color(255, 0, 0, int(alpha * 255))
      else:
        alpha = confidence
        color = rl.Color(255, 0, 0, int(alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    if ui_state.dp_ui_rainbow:
      v_ego = sm['carState'].vEgo
      self._update_rainbow_gradient(v_ego)
      if self._dp_ui_rainbow_gradient:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._dp_ui_rainbow_gradient)
      return

    try:
      allow_throttle = bool(getattr(sm['longitudinalPlan'], "allowThrottle", True))
    except Exception:
      allow_throttle = True
    allow_throttle = allow_throttle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if self._experimental_mode:
      # Draw with acceleration coloring
      if len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )
      draw_polygon(self._rect, self._path.projected_points, gradient=gradient)

      curve_t = float(self._curve_path_alert)
      if curve_t > 0.001:
        top_alpha = int(30 + curve_t * 140)
        curve_overlay = Gradient(
          start=(0.0, 1.0),
          end=(0.0, 0.0),
          colors=[rl.Color(255, 60, 0, 0), rl.Color(255, 60, 0, top_alpha)],
          stops=[0.0, 1.0],
        )
        draw_polygon(self._rect, self._path.projected_points, gradient=curve_overlay)

      # Lincoln HUD enhancements: tint the road band as we approach lead, then restore to green when clear
      if ui_state.dp_lincoln_hud_enhanced and self._lead_box_intensity > 0.01:
        t = float(np.clip(self._lead_box_intensity, 0.0, 1.0))
        r = int(DP_LINCOLN_LEAD_COLOR_FAR.r + t * (DP_LINCOLN_LEAD_COLOR_NEAR.r - DP_LINCOLN_LEAD_COLOR_FAR.r))
        g = int(DP_LINCOLN_LEAD_COLOR_FAR.g + t * (DP_LINCOLN_LEAD_COLOR_NEAR.g - DP_LINCOLN_LEAD_COLOR_FAR.g))
        b = int(DP_LINCOLN_LEAD_COLOR_FAR.b + t * (DP_LINCOLN_LEAD_COLOR_NEAR.b - DP_LINCOLN_LEAD_COLOR_FAR.b))
        top_alpha = int(20 + t * 120)
        overlay = Gradient(
          start=(0.0, 1.0),
          end=(0.0, 0.0),
          colors=[rl.Color(r, g, b, 0), rl.Color(r, g, b, top_alpha)],
          stops=[0.0, 1.0],
        )
        draw_polygon(self._rect, self._path.projected_points, gradient=overlay)

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    self._draw_lead_box()
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

      # Lincoln HUD: force-show only distance + lead speed (no TTC, no extra numbers)
      if ui_state.dp_lincoln_hud_enhanced:
        start_y = lead.y
        dist_str = f"{lead.d_rel:.1f}m" if ui_state.is_metric else f"{lead.d_rel * 3.28084:.1f}ft"
        v_lead = max(0.0, float(getattr(lead, "v_lead", 0.0)))
        speed_str = f"{v_lead * 3.6:.0f}km/h" if ui_state.is_metric else f"{v_lead * 2.23694:.0f}mph"
        self._dp_paint_centered_lead_text(dist_str, 56, lead.x, start_y + lead.sz)
        self._dp_paint_centered_lead_text(speed_str, 48, lead.x, start_y + lead.sz + 40)
      else:
        show_distance = ui_state.dp_ui_lead in [DpUiLeadMode.lead, DpUiLeadMode.all]
        show_ttc = ui_state.dp_ui_lead in [DpUiLeadMode.lead, DpUiLeadMode.all]

        if show_distance or show_ttc:
          start_y = lead.y

          if show_distance:
            dist_str = f"{lead.d_rel:.1f}m" if ui_state.is_metric else f"{lead.d_rel * 3.28084:.1f}ft"
            self._dp_paint_centered_lead_text(dist_str, 56, lead.x, start_y + lead.sz)

          if show_ttc:
            car_state = ui_state.sm['carState']
            ttc = (lead.d_rel / car_state.vEgo) if car_state.vEgo > 0 else float("NaN")
            if ttc < 5.:
              ttc_str = f"{ttc:.1f}s"
              self._dp_paint_centered_lead_text(ttc_str, 80, lead.x, start_y + lead.sz + 40)

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_distance: float) -> int:
    """Get the index corresponding to the given path distance"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_distance)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, max_distance: float, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]

    # Interpolate around max_idx so path end is smooth (max_distance is always >= p0.x)
    if 0 < max_idx < line.shape[0] - 1:
      p0 = line[max_idx]
      p1 = line[max_idx + 1]
      x0, x1 = p0[0], p1[0]
      interp_y = np.interp(max_distance, [x0, x1], [p0[1], p1[1]])
      interp_z = np.interp(max_distance, [x0, x1], [p0[2], p1[2]])
      interp_point = np.array([max_distance, interp_y, interp_z], dtype=points.dtype)
      points = np.concatenate((points, interp_point[None, :]), axis=0)

    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]

  def _update_rainbow_gradient(self, v_ego):
    # Scroll speed
    rotation_speed = max(0.01, v_ego) / gui_app.target_fps / DP_RAINBOW_SCROLL_SPEED_FACTOR
    self._dp_ui_rainbow_rotation += rotation_speed
    if self._dp_ui_rainbow_rotation > 1.0:
      self._dp_ui_rainbow_rotation -= 1.0

    gradient_stops = np.linspace(0, 1, DP_RAINBOW_GRADIENT_SAMPLES)

    hues = (gradient_stops * DP_RAINBOW_NUM_REPEATS + self._dp_ui_rainbow_rotation) % 1.0

    # Vectorized hsv_to_rgb
    i = np.floor(hues * DP_RAINBOW_HUE_SECTORS).astype(np.uint8)
    f = hues * DP_RAINBOW_HUE_SECTORS - i
    q = 1 - f
    t = f

    i %= DP_RAINBOW_HUE_SECTORS

    rgb = np.zeros((hues.shape[0], 3))

    masks = [i == j for j in range(DP_RAINBOW_HUE_SECTORS)]

    rgb[masks[0], 0] = 1
    rgb[masks[0], 1] = t[masks[0]]

    rgb[masks[1], 0] = q[masks[1]]
    rgb[masks[1], 1] = 1

    rgb[masks[2], 1] = 1
    rgb[masks[2], 2] = t[masks[2]]

    rgb[masks[3], 1] = q[masks[3]]
    rgb[masks[3], 2] = 1

    rgb[masks[4], 0] = t[masks[4]]
    rgb[masks[4], 2] = 1

    rgb[masks[5], 0] = 1
    rgb[masks[5], 2] = q[masks[5]]

    rgb_int = (rgb * 255).astype(np.uint8)

    colors = [rl.Color(r, g, b, DP_RAINBOW_ALPHA) for r, g, b in rgb_int]

    self._dp_ui_rainbow_gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=colors,
      stops=gradient_stops.tolist(),
    )

  def _dp_paint_centered_lead_text(self, text, size, x, y):
    font = gui_app.font(FontWeight.NORMAL)
    text_width = measure_text_cached(font, text, size).x
    text_x = x - text_width / 2
    rl.draw_text_ex(font, text, rl.Vector2(text_x, y), size, 0, rl.WHITE)

  def _draw_live_tracks(self, sm):
    font = gui_app.font(FontWeight.NORMAL)
    live_tracks = sm['liveTracks']
    font_size = 40
    line_height = 40

    for point in live_tracks.points:
      d_rel = point.dRel
      y_rel = point.yRel
      v_rel = point.vRel

      z_on_path = self._path_offset_z
      if d_rel >= 0 and self._path.raw_points.shape[0] > 0:
        path_x = self._path.raw_points[:, 0]
        path_z = self._path.raw_points[:, 2]
        idx = self._get_path_length_idx(path_x, d_rel)
        if idx < len(path_z):
          z_on_path += path_z[idx]

      screen_pos = self._map_to_screen(d_rel, -y_rel, z_on_path)
      if screen_pos:
        sx, sy = int(screen_pos[0]), int(screen_pos[1])

        rl.draw_circle(sx, sy, 10, rl.Color(255, 0, 0, 200))

        if ui_state.is_metric:
          dist_unit, speed_unit = "m", "m/s"
          d_rel_str = f"{d_rel:.2f}"
          y_rel_str = f"{y_rel:.2f}"
          v_rel_str = f"{v_rel:.2f}"
        else:
          dist_unit, speed_unit = "ft", "mph"
          d_rel_str = f"{d_rel * 3.28084:.2f}"
          y_rel_str = f"{y_rel * 3.28084:.2f}"
          v_rel_str = f"{v_rel * 2.23694:.2f}"

        info_text = (
          f"ID: {point.trackId}\n"
          f"d: {d_rel_str} {dist_unit}\n"
          f"y: {y_rel_str} {dist_unit}\n"
          f"dV: {v_rel_str} {speed_unit}"
        )
        lines = info_text.split('\n')
        text_x = sx + 15
        text_y = sy - 20

        for i, line in enumerate(lines):
          rl.draw_text_ex(font, line, rl.Vector2(text_x, text_y + i * line_height), font_size, 0, rl.WHITE)
