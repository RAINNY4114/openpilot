import math
from dataclasses import dataclass
from importlib.resources import as_file
import os
import shutil
import threading
import time
import numpy as np
import pyray as rl
from cereal import log, messaging
from msgq.visionipc import VisionStreamType
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import FONT_DIR, FONT_SCALE, font_fallback, gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

OpState = log.SelfdriveState.OpenpilotState
CALIBRATED = log.LiveCalibrationData.Status.calibrated
ROAD_CAM = VisionStreamType.VISION_STREAM_ROAD
WIDE_CAM = VisionStreamType.VISION_STREAM_WIDE_ROAD
DEFAULT_DEVICE_CAMERA = DEVICE_CAMERAS["tici", "ar0231"]

LITE = os.getenv("LITE") is not None

BORDER_COLORS = {
  UIStatus.DISENGAGED: rl.Color(0x12, 0x28, 0x39, 0xFF),  # Blue for disengaged state
  UIStatus.OVERRIDE: rl.Color(0x89, 0x92, 0x8D, 0xFF),  # Gray for override state
  UIStatus.ENGAGED: rl.Color(0x16, 0x7F, 0x40, 0xFF),  # Green for engaged state
  UIStatus.ALKA: rl.Color(0x22, 0xa0, 0xdc, 0xf1), # Blue for ALKA state
}

WIDE_CAM_MAX_SPEED = 10.0  # m/s (22 mph)
ROAD_CAM_MIN_SPEED = 15.0  # m/s (34 mph)
INF_POINT = np.array([1000.0, 0.0, 0.0])

# dp
DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)
DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)
DP_INDICATOR_COLOR_BSM = rl.Color(255, 255, 0, 255)
DP_INDICATOR_COLOR_BLINKER = rl.Color(0, 255, 0, 255)
DP_INDICATOR_COLOR_BSM_ENHANCED = rl.Color(255, 0, 0, 255)
DP_INDICATOR_COLOR_BLINKER_ENHANCED = rl.Color(255, 255, 0, 255)
DP_DECEL_BAR_MIN_MS2 = 0.25
DP_DECEL_BAR_MAX_MS2 = 3.0
DP_HARD_BRAKE_DECEL_MS2 = 3.5
DP_HARD_BRAKE_BRAKE_CMD = 0.7
DP_HARD_BRAKE_FLASH_HZ = 4.0
PERF_DIRECTION_LABELS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
PERF_FONT_SIZE = 32
PERF_PADDING = 12
PERF_MARGIN_BOTTOM = UI_BORDER_SIZE // 2
PERF_ITEM_GAP = 140
PERF_BG_COLOR = rl.Color(0, 0, 0, 120)


@dataclass
class _DynamicFontCacheEntry:
  font: rl.Font
  last_used_t: float


class AugmentedRoadView(CameraView):
  def __init__(self, stream_type: VisionStreamType = VisionStreamType.VISION_STREAM_ROAD):
    super().__init__("camerad", stream_type)
    self._set_placeholder_color(BORDER_COLORS[UIStatus.DISENGAGED])

    self.device_camera: DeviceCameraConfig | None = None
    self.view_from_calib = view_frame_from_device_frame.copy()
    self.view_from_wide_calib = view_frame_from_device_frame.copy()

    self._matrix_cache_key = (0, 0.0, 0.0, stream_type)
    self._cached_matrix: np.ndarray | None = None
    self._content_rect = rl.Rectangle()

    self.model_renderer = ModelRenderer()
    self._hud_renderer = HudRenderer()
    self.alert_renderer = AlertRenderer()
    self.driver_state_renderer = DriverStateRenderer()

    # DP border indicator
    self._dp_indicator_show_left = False
    self._dp_indicator_show_right = False
    self._dp_indicator_count_left = 0
    self._dp_indicator_count_right = 0
    self._dp_indicator_color_left = rl.Color(0, 0, 0, 0)
    self._dp_indicator_color_right = rl.Color(0, 0, 0, 0)

    # debug
    self._pm = messaging.PubMaster(['uiDebug'])

    # Lincoln perf overlay
    self._perf_font = gui_app.font(FontWeight.MEDIUM)
    self._perf_stats: dict[str, str] = {"cpu_temp": "N/A", "mem_usage": "N/A", "disk_free": "N/A"}
    self._perf_lock = threading.Lock()
    self._perf_running = True
    self._perf_thread = threading.Thread(target=self._perf_update_loop, daemon=True)
    self._perf_thread.start()
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")
    self._road_loc_cache = "--"
    self._road_loc_cache_t = 0.0
    self._road_name_last = ""
    self._road_name_last_t = 0.0
    self._road_font_cache: dict[tuple[int, ...], _DynamicFontCacheEntry] = {}

    # Lincoln HUD enhancements
    self._hud_brake_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)

  def _render(self, rect):
    # Only render when system is started to avoid invalid data access
    start_draw = time.monotonic()
    if not ui_state.started:
      return

    self._update_dp_indicator_states(ui_state.sm)
    self._switch_stream_if_needed(ui_state.sm)

    # Update calibration before rendering
    self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      rect.x + UI_BORDER_SIZE,
      rect.y + UI_BORDER_SIZE,
      rect.width - 2 * UI_BORDER_SIZE,
      rect.height - 2 * UI_BORDER_SIZE,
    )

    # Enable scissor mode to clip all rendering within content rectangle boundaries
    # This creates a rendering viewport that prevents graphics from drawing outside the border
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    # Render the base camera view
    super()._render(rect)

    hide_hud = False
    if ui_state.dp_ui_hide_hud_speed_ms > 0. and ui_state.sm['carState'].vEgo > ui_state.dp_ui_hide_hud_speed_ms:
      hide_hud = True

    # Draw all UI overlays
    self.model_renderer.render(self._content_rect)
    self._draw_hud_enhancements()
    if not hide_hud:
      self._hud_renderer.render(self._content_rect)
    self.alert_renderer.render(self._content_rect)
    if not LITE:
      if not hide_hud:
        self.driver_state_renderer.render(self._content_rect)

    self._draw_performance_info()

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds

    # End clipping region
    rl.end_scissor_mode()

    # Draw colored border based on driving state
    self._draw_border(rect)

    # publish uiDebug
    msg = messaging.new_message('uiDebug')
    msg.uiDebug.drawTimeMillis = (time.monotonic() - start_draw) * 1000
    self._pm.send('uiDebug', msg)

  def _handle_mouse_press(self, _):
    if not self._hud_renderer.user_interacting() and self._click_callback is not None:
      self._click_callback()

  def _handle_mouse_release(self, _):
    # We only call click callback on press if not interacting with HUD
    pass

  def _draw_border(self, rect: rl.Rectangle):
    rl.draw_rectangle_lines_ex(rect, UI_BORDER_SIZE, rl.BLACK)
    border_roundness = 0.12
    border_color = BORDER_COLORS.get(ui_state.status, BORDER_COLORS[UIStatus.DISENGAGED])
    if ui_state.dp_alka_active and ui_state.status == UIStatus.DISENGAGED:
      border_color = BORDER_COLORS[UIStatus.ALKA]
    base_border_color = border_color

    # Lincoln HUD enhancements: brake intensity colors the whole border (FrogPilot-style "whole frame" cue)
    if ui_state.dp_lincoln_hud_enhanced:
      sm = ui_state.sm
      if sm.alive.get("carState", False):
        cs = sm["carState"]
        brake_pressed = bool(getattr(cs, "brakePressed", False))

        # 1) Actual deceleration (covers stock ACC braking too)
        a_ego = float(getattr(cs, "aEgo", 0.0))
        decel = max(0.0, -a_ego)  # m/s^2
        decel_intensity = float(np.interp(decel, [DP_DECEL_BAR_MIN_MS2, DP_DECEL_BAR_MAX_MS2], [0.0, 1.0]))

        # 2) Commanded brake (covers OP longitudinal brake actuation)
        brake_cmd = 0.0
        if sm.valid.get("carOutput", False):
          brake_cmd = float(sm["carOutput"].actuatorsOutput.brake)
        brake_intensity = float(np.interp(brake_cmd, [0.02, 0.6], [0.0, 1.0]))

        intensity_raw = max(decel_intensity, brake_intensity)
        if brake_pressed:
          intensity_raw = max(intensity_raw, 0.20)
        intensity = float(np.clip(self._hud_brake_filter.update(intensity_raw), 0.0, 1.0))

        if intensity > 0.02:
          hard_brake_pred = False
          if sm.alive.get("modelV2", False):
            hard_brake_pred = bool(sm["modelV2"].meta.hardBrakePredicted)

          hard_brake = hard_brake_pred or (decel >= DP_HARD_BRAKE_DECEL_MS2) or (brake_cmd >= DP_HARD_BRAKE_BRAKE_CMD)
          if hard_brake:
            flash_on = (time.monotonic() * DP_HARD_BRAKE_FLASH_HZ) % 1.0 < 0.5
            border_color = rl.Color(255, 0, 0, 255) if flash_on else base_border_color
          else:
            t = intensity
            border_color = rl.Color(
              int(base_border_color.r + t * (255 - base_border_color.r)),
              int(base_border_color.g + t * (0 - base_border_color.g)),
              int(base_border_color.b + t * (0 - base_border_color.b)),
              base_border_color.a,
            )

    border_rect = rl.Rectangle(rect.x + UI_BORDER_SIZE, rect.y + UI_BORDER_SIZE,
                               rect.width - 2 * UI_BORDER_SIZE, rect.height - 2 * UI_BORDER_SIZE)
    rl.draw_rectangle_rounded_lines_ex(border_rect, border_roundness, 10, UI_BORDER_SIZE, border_color)

    # dp - Side indicators
    indicator_y = int(rect.y+4*UI_BORDER_SIZE)
    indicator_height = int(rect.height-8*UI_BORDER_SIZE)
    if self._dp_indicator_show_left:
      rl.draw_rectangle(int(rect.x), indicator_y, UI_BORDER_SIZE, indicator_height, self._dp_indicator_color_left)
    if self._dp_indicator_show_right:
      rl.draw_rectangle(int(rect.x + rect.width-UI_BORDER_SIZE), indicator_y, UI_BORDER_SIZE, indicator_height, self._dp_indicator_color_right)

  def _switch_stream_if_needed(self, sm):
    if sm['selfdriveState'].experimentalMode and WIDE_CAM in self.available_streams:
      v_ego = sm['carState'].vEgo
      if v_ego < WIDE_CAM_MAX_SPEED:
        target = WIDE_CAM
      elif v_ego > ROAD_CAM_MIN_SPEED:
        target = ROAD_CAM
      else:
        # Hysteresis zone - keep current stream
        target = self.stream_type
    else:
      target = ROAD_CAM

    if self.stream_type != target:
      self.switch_stream(target)

  def _update_calibration(self):
    # Update device camera if not already set
    sm = ui_state.sm
    if not self.device_camera and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      self.device_camera = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]

    # Check if live calibration data is available and valid
    if not (sm.updated["liveCalibration"] and sm.valid['liveCalibration']):
      return

    calib = sm['liveCalibration']
    if len(calib.rpyCalib) != 3 or calib.calStatus != CALIBRATED:
      return

    # Update view_from_calib matrix
    device_from_calib = rot_from_euler(calib.rpyCalib)
    self.view_from_calib = view_frame_from_device_frame @ device_from_calib

    # Update wide calibration if available
    if hasattr(calib, 'wideFromDeviceEuler') and len(calib.wideFromDeviceEuler) == 3:
      wide_from_device = rot_from_euler(calib.wideFromDeviceEuler)
      self.view_from_wide_calib = view_frame_from_device_frame @ wide_from_device @ device_from_calib

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    # Check if we can use cached matrix
    cache_key = (
      ui_state.sm.recv_frame['liveCalibration'],
      self._content_rect.width,
      self._content_rect.height,
      self.stream_type
    )
    if cache_key == self._matrix_cache_key and self._cached_matrix is not None:
      return self._cached_matrix

    # Get camera configuration
    device_camera = self.device_camera or DEFAULT_DEVICE_CAMERA
    is_wide_camera = self.stream_type == WIDE_CAM
    intrinsic = device_camera.ecam.intrinsics if is_wide_camera else device_camera.fcam.intrinsics
    calibration = self.view_from_wide_calib if is_wide_camera else self.view_from_calib
    zoom = 2.0 if is_wide_camera else 1.1

    # Calculate transforms for vanishing point
    calib_transform = intrinsic @ calibration
    kep = calib_transform @ INF_POINT

    # Calculate center points and dimensions
    x, y = self._content_rect.x, self._content_rect.y
    w, h = self._content_rect.width, self._content_rect.height
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]

    # Calculate max allowed offsets with margins
    margin = 5
    max_x_offset = cx * zoom - w / 2 - margin
    max_y_offset = cy * zoom - h / 2 - margin

    # Calculate and clamp offsets to prevent out-of-bounds issues
    try:
      if abs(kep[2]) > 1e-6:
        x_offset = np.clip((kep[0] / kep[2] - cx) * zoom, -max_x_offset, max_x_offset)
        y_offset = np.clip((kep[1] / kep[2] - cy) * zoom, -max_y_offset, max_y_offset)
      else:
        x_offset, y_offset = 0, 0
    except (ZeroDivisionError, OverflowError):
      x_offset, y_offset = 0, 0

    # Cache the computed transformation matrix to avoid recalculations
    self._matrix_cache_key = cache_key
    self._cached_matrix = np.array([
      [zoom * 2 * cx / w, 0, -x_offset / w * 2],
      [0, zoom * 2 * cy / h, -y_offset / h * 2],
      [0, 0, 1.0]
    ])

    video_transform = np.array([
      [zoom, 0.0, (w / 2 + x - x_offset) - (cx * zoom)],
      [0.0, zoom, (h / 2 + y - y_offset) - (cy * zoom)],
      [0.0, 0.0, 1.0]
    ])
    self.model_renderer.set_transform(video_transform @ calib_transform)

    return self._cached_matrix

  def _update_dp_indicator_side_state(self, blinker_state, bsm_state, show_prev, count_prev):
    if not blinker_state and not bsm_state:
      return False, 0, rl.Color(0, 0, 0, 0)

    count = count_prev + 1
    show = True
    color = rl.Color(0, 0, 0, 0)

    if ui_state.dp_lincoln_hud_enhanced:
      # Enhanced logic: blinker = yellow flash, blindspot = red flash, both = red fast flash
      if bsm_state:
        blink_rate = DP_INDICATOR_BLINK_RATE_FAST if blinker_state else DP_INDICATOR_BLINK_RATE_STD
        show = (count // blink_rate) % 2 == 0
        color = DP_INDICATOR_COLOR_BSM_ENHANCED
      elif blinker_state:
        blink_rate = DP_INDICATOR_BLINK_RATE_STD
        show = (count // blink_rate) % 2 == 0
        color = DP_INDICATOR_COLOR_BLINKER_ENHANCED
      else:
        show = False
    else:
      if bsm_state and blinker_state:
        show = (count // DP_INDICATOR_BLINK_RATE_FAST) % 2 == 0
        color = DP_INDICATOR_COLOR_BSM
      elif blinker_state:
        show = (count // DP_INDICATOR_BLINK_RATE_STD) % 2 == 0
        color = DP_INDICATOR_COLOR_BLINKER
      elif bsm_state:
        show = True
        color = DP_INDICATOR_COLOR_BSM
      else:
        show = False

    return show, count, color

  def _update_dp_indicator_states(self, sm):
    cs = sm['carState']
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left = \
      self._update_dp_indicator_side_state(cs.leftBlinker, cs.leftBlindspot,
                                           self._dp_indicator_show_left, self._dp_indicator_count_left)
    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right = \
      self._update_dp_indicator_side_state(cs.rightBlinker, cs.rightBlindspot,
                                           self._dp_indicator_show_right, self._dp_indicator_count_right)

  def _draw_hud_enhancements(self) -> None:
    if not ui_state.dp_lincoln_hud_enhanced:
      return

    sm = ui_state.sm
    if not sm.alive.get("carState", False):
      return

    rect = self._content_rect
    if rect.width <= 0 or rect.height <= 0:
      return

    cs = sm["carState"]

    # FrogPilot-style blindspot "wall" (drawn in the adjacent lane polygon)
    if cs.leftBlindspot:
      self._draw_hud_fp_blindspot_wall(rect=rect, is_left=True)
    elif cs.leftBlinker:
      # Keep the original (simple) blinker intent highlight when no blindspot is present
      self._draw_hud_enhanced_side_zone(
        rect=rect,
        is_left=True,
        blinker=True,
        blindspot=False,
        show=self._dp_indicator_show_left,
        color=self._dp_indicator_color_left,
      )

    if cs.rightBlindspot:
      self._draw_hud_fp_blindspot_wall(rect=rect, is_left=False)
    elif cs.rightBlinker:
      self._draw_hud_enhanced_side_zone(
        rect=rect,
        is_left=False,
        blinker=True,
        blindspot=False,
        show=self._dp_indicator_show_right,
        color=self._dp_indicator_color_right,
      )

  @staticmethod
  def _fp_calculate_lane_width(lane: np.ndarray, current_lane: np.ndarray, road_edge: np.ndarray | None = None) -> float:
    if lane.size == 0 or current_lane.size == 0:
      return 0.0

    try:
      current_x = current_lane[:, 0]
      current_y = current_lane[:, 1]
      lane_y_interp = np.interp(current_x, lane[:, 0], lane[:, 1])
      distance_to_lane = float(np.mean(np.abs(current_y - lane_y_interp)))

      if road_edge is None or road_edge.size == 0:
        return distance_to_lane

      road_edge_y_interp = np.interp(current_x, road_edge[:, 0], road_edge[:, 1])
      distance_to_road_edge = float(np.mean(np.abs(current_y - road_edge_y_interp)))
      if distance_to_road_edge < distance_to_lane:
        return 0.0

      return distance_to_lane
    except Exception:
      return 0.0

  def _fp_adjacent_lane_centerline(self, lane: np.ndarray, current_lane: np.ndarray) -> np.ndarray:
    if lane.size == 0 or current_lane.size == 0:
      return np.empty((0, 3), dtype=np.float32)

    try:
      x = current_lane[:, 0]
      lane_y = np.interp(x, lane[:, 0], lane[:, 1])
      lane_z = np.interp(x, lane[:, 0], lane[:, 2])
      center_y = 0.5 * (lane_y + current_lane[:, 1])
      center_z = 0.5 * (lane_z + current_lane[:, 2])
      return np.column_stack((x, center_y, center_z)).astype(np.float32)
    except Exception:
      return np.empty((0, 3), dtype=np.float32)

  def _fp_get_adjacent_lane_polygon(self, rect: rl.Rectangle, is_left: bool) -> np.ndarray:
    mr = self.model_renderer
    if not hasattr(mr, "_lane_lines") or not hasattr(mr, "_road_edges"):
      return np.empty((0, 2), dtype=np.float32)

    try:
      # Model lane lines are ordered like upstream:
      # 0: left far, 1: left near (ego boundary), 2: right near, 3: right far
      if is_left:
        outer = mr._lane_lines[0].raw_points
        inner = mr._lane_lines[1].raw_points
        road_edge = mr._road_edges[0].raw_points if len(mr._road_edges) > 0 else None
      else:
        outer = mr._lane_lines[3].raw_points
        inner = mr._lane_lines[2].raw_points
        road_edge = mr._road_edges[1].raw_points if len(mr._road_edges) > 1 else None

      lane_width = self._fp_calculate_lane_width(outer, inner, road_edge)
      if lane_width <= 0.0:
        return np.empty((0, 2), dtype=np.float32)

      centerline = self._fp_adjacent_lane_centerline(outer, inner)
      if centerline.shape[0] < 2:
        return np.empty((0, 2), dtype=np.float32)

      # Match FrogPilot's max draw distance logic (10-100 m, optionally shortened when a lead is present)
      path_raw = getattr(mr, "_path", None)
      path_x = np.empty((0,), dtype=np.float32)
      if path_raw is not None and getattr(path_raw, "raw_points", np.empty((0, 3))).size:
        path_x = path_raw.raw_points[:, 0]

      if path_x.size:
        max_distance = float(np.clip(float(path_x[-1]), 10.0, 100.0))
      else:
        max_distance = float(np.clip(float(centerline[-1, 0]), 10.0, 100.0))

      sm = ui_state.sm
      if sm.valid.get("radarState", False):
        lead_one = sm["radarState"].leadOne
        if getattr(lead_one, "status", False):
          lead_d = float(lead_one.dRel) * 2.0
          max_distance = float(np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance))

      max_idx = mr._get_path_length_idx(path_x if path_x.size else centerline[:, 0], max_distance)
      return mr._map_line_to_polygon(centerline, lane_width / 2.0, 0.0, max_idx, max_distance, allow_invert=False)
    except Exception:
      return np.empty((0, 2), dtype=np.float32)

  def _draw_hud_fp_blindspot_wall(self, rect: rl.Rectangle, is_left: bool) -> None:
    poly = self._fp_get_adjacent_lane_polygon(rect, is_left=is_left)
    if poly.size == 0:
      return

    # 1:1 FrogPilot color: HSL(0°, 0.75, 0.5) with alpha 0.6/0.4/0.2
    r, g, b = 223, 32, 32
    gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[
        # `shader_polygon` maps t=0 at gradientEnd (top) and t=1 at gradientStart (bottom),
        # so order colors from top->bottom to match FrogPilot's bottom-heavy gradient.
        rl.Color(r, g, b, int(0.20 * 255)),
        rl.Color(r, g, b, int(0.40 * 255)),
        rl.Color(r, g, b, int(0.60 * 255)),
      ],
      stops=[0.0, 0.5, 1.0],
    )
    draw_polygon(rect, poly, gradient=gradient)

  @staticmethod
  def _draw_hud_enhanced_side_zone(rect: rl.Rectangle, is_left: bool, blinker: bool, blindspot: bool,
                                  show: bool, color: rl.Color) -> None:
    if not (blinker or blindspot) or not show:
      return

    inset = rect.width * 0.01
    top_y = rect.y + min(320.0, rect.height * 0.35)
    bottom_y = rect.y + rect.height
    slant = rect.height * 0.10

    top_w = rect.width * 0.10
    bottom_w = rect.width * 0.16

    alpha = 70
    if blindspot and blinker:
      alpha = 180
    elif blindspot:
      alpha = 140

    # FrogPilot-style: red/yellow semi-transparent vertical gradient band
    a0 = int(np.clip(alpha, 0, 255))
    a1 = int(np.clip(alpha * 0.70, 0, 255))
    a2 = int(np.clip(alpha * 0.25, 0, 255))
    gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[
        rl.Color(color.r, color.g, color.b, a2),
        rl.Color(color.r, color.g, color.b, a1),
        rl.Color(color.r, color.g, color.b, a0),
      ],
      stops=[0.0, 0.55, 1.0],
    )

    if is_left:
      x_outer = rect.x + inset
      points = [
        (x_outer, bottom_y),
        (x_outer, top_y),
        (x_outer + top_w, top_y + slant),
        (x_outer + bottom_w, bottom_y),
      ]
    else:
      x_outer = rect.x + rect.width - inset
      points = [
        (x_outer, bottom_y),
        (x_outer, top_y),
        (x_outer - top_w, top_y + slant),
        (x_outer - bottom_w, bottom_y),
      ]

    draw_polygon(rect, np.array(points, dtype=np.float32), gradient=gradient)

  def _draw_performance_info(self) -> None:
    if not ui_state.dp_lincoln_perf_info_enabled:
      return

    rect = self._content_rect
    if rect.width <= 0 or rect.height <= 0:
      return

    stats = self._get_perf_stats()
    curvature_text, steering_text, torque_text = self._get_curvature_steer_torque()
    direction_text = self._get_direction_label()
    road_loc_text = self._get_road_location_text()
    control_text = self._get_control_state_text()
    mem_usage = stats.get("mem_usage", "N/A")
    cpu_temp = stats.get("cpu_temp", "N/A")

    base_items = [
      f"{tr('Curvature')} {curvature_text}/{steering_text}/{torque_text}",
      f"{tr('Direction')} {direction_text}",
      f"{tr('Road')} {road_loc_text}",
      f"{tr('Control')} {control_text}",
      f"{tr('Memory')} {mem_usage}",
      f"{tr('CPU Temp')} {cpu_temp}",
    ]

    road_item_idx = 2
    max_width = max(rect.width - 40, 0)

    items = list(base_items)
    road_label = f"{tr('Road')} "
    road_value = road_loc_text if road_loc_text else "--"

    road_item_font: rl.Font | None = None
    measurements: list[rl.Vector2] = []
    gap_count = max(0, len(items) - 1)
    gap = 0.0
    desired_width = 0.0

    for _ in range(2):
      items[road_item_idx] = f"{tr('Road')} {road_value}"

      road_item_font = None
      if road_value not in ("", "--"):
        base_font = font_fallback(self._perf_font)
        if self._font_has_missing_glyphs(base_font, items[road_item_idx]):
          road_item_font = self._get_dynamic_unifont_font(items[road_item_idx])

      measurements = []
      for idx, text in enumerate(items):
        if idx == road_item_idx and road_item_font is not None:
          measurements.append(self._measure_text_ex_no_fallback(road_item_font, text, PERF_FONT_SIZE))
        else:
          measurements.append(measure_text_cached(self._perf_font, text, PERF_FONT_SIZE))

      total_text_width = sum(size.x for size in measurements)
      gap = float(PERF_ITEM_GAP if gap_count else 0.0)
      if gap_count > 0 and max_width > 0:
        gap = min(gap, max(0.0, (max_width - 2 * PERF_PADDING - total_text_width) / gap_count))
      desired_width = total_text_width + 2 * PERF_PADDING + gap * gap_count

      if max_width <= 0 or desired_width <= max_width or road_value in ("", "--"):
        break

      width_without_road = total_text_width - measurements[road_item_idx].x
      if road_item_font is not None:
        label_width = self._measure_text_ex_no_fallback(road_item_font, road_label, PERF_FONT_SIZE).x
        measure_value_width = lambda t: self._measure_text_ex_no_fallback(road_item_font, t, PERF_FONT_SIZE).x
      else:
        label_width = measure_text_cached(self._perf_font, road_label, PERF_FONT_SIZE).x
        measure_value_width = lambda t: measure_text_cached(self._perf_font, t, PERF_FONT_SIZE).x

      available_for_road_item = max_width - 2 * PERF_PADDING - gap * gap_count - width_without_road
      available_for_value = available_for_road_item - label_width
      shortened = self._ellipsize_text_to_width(road_value, available_for_value, measure_value_width)
      road_value = shortened if shortened else "--"

    bar_width = min(desired_width, max_width) if max_width > 0 else desired_width
    bar_height = PERF_FONT_SIZE + 2 * PERF_PADDING

    bar_x = rect.x + (rect.width - bar_width) / 2
    bar_y = rect.y + rect.height - bar_height - PERF_MARGIN_BOTTOM
    minimum_y = rect.y + PERF_MARGIN_BOTTOM
    if bar_y < minimum_y:
      bar_y = minimum_y

    rl.draw_rectangle_rounded(
      rl.Rectangle(bar_x, bar_y, bar_width, bar_height),
      0.2,
      8,
      PERF_BG_COLOR,
    )

    cursor_x = bar_x + PERF_PADDING
    text_y = bar_y + PERF_PADDING
    for idx, (text, measurement) in enumerate(zip(items, measurements)):
      if idx == road_item_idx and road_item_font is not None:
        self._draw_text_ex_no_fallback(road_item_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, rl.WHITE)
      else:
        rl.draw_text_ex(self._perf_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, rl.WHITE)
      cursor_x += measurement.x + gap

  @staticmethod
  def _ellipsize_text_to_width(text: str, max_width: float, measure_width, ellipsis: str = "...") -> str:
    if max_width <= 0:
      return ""
    if not text:
      return text
    if measure_width(text) <= max_width:
      return text
    if measure_width(ellipsis) > max_width:
      return ""

    lo, hi = 0, len(text)
    best = ""
    while lo <= hi:
      mid = (lo + hi) // 2
      candidate = text[:mid] + ellipsis
      if measure_width(candidate) <= max_width:
        best = candidate
        lo = mid + 1
      else:
        hi = mid - 1
    return best

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
    entry = self._road_font_cache.get(codepoints)
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

    self._road_font_cache[codepoints] = _DynamicFontCacheEntry(font=font, last_used_t=now)
    self._prune_road_font_cache(max_entries=8)
    return font

  def _prune_road_font_cache(self, max_entries: int) -> None:
    if len(self._road_font_cache) <= max_entries:
      return

    items = sorted(self._road_font_cache.items(), key=lambda kv: kv[1].last_used_t)
    for key, entry in items[: max(0, len(items) - max_entries)]:
      try:
        rl.unload_font(entry.font)
      except Exception:
        pass
      self._road_font_cache.pop(key, None)

  def _get_curvature_steer_torque(self) -> tuple[str, str, str]:
    curvature_text = "--"
    steering_text = "--"
    torque_text = "--"
    sm = ui_state.sm
    try:
      if getattr(sm, "alive", {}).get("controlsState", False):
        curvature = sm['controlsState'].curvature
        if math.isfinite(curvature):
          curvature_text = f"{abs(curvature):.3f}"
      if getattr(sm, "alive", {}).get("carState", False):
        car_state = sm['carState']
        steering = car_state.steeringAngleDeg
        if math.isfinite(steering):
          steering_text = f"{steering:.1f}°"
        torque = car_state.steeringTorque
        if math.isfinite(torque):
          torque_text = f"{torque:.1f}Nm"
    except Exception:
      pass
    return curvature_text, steering_text, torque_text

  def _get_road_location_text(self) -> str:
    now = time.monotonic()
    if now - self._road_loc_cache_t < 0.5:
      return self._road_loc_cache

    # Prefer mapd-provided road name (OSM matched); fall back to GPS coordinates.
    road_name = ""
    for params in (getattr(self, "_params_memory", None), getattr(self, "_params", None)):
      if params is None:
        continue
      try:
        rn = params.get("RoadName")
        if rn:
          road_name = rn.strip()
          if road_name:
            break
      except Exception:
        continue

    if road_name:
      max_chars = 22
      if len(road_name) > max_chars:
        road_name = road_name[:max_chars - 3] + "..."
      self._road_name_last = road_name
      self._road_name_last_t = now
      self._road_loc_cache = road_name
      self._road_loc_cache_t = now
      return road_name

    # mapd can briefly output an empty road name during GPS jitter or process restarts.
    # Keep the last valid match to avoid flickering to raw coordinates, especially when stopped.
    sm = ui_state.sm
    hold_s = 10.0
    try:
      if getattr(sm, "alive", {}).get("carState", False):
        car_state = sm["carState"]
        if getattr(car_state, "standstill", False):
          hold_s = 3600.0
        else:
          v = getattr(car_state, "vEgo", None)
          if v is not None:
            v_ego = float(v)
            if math.isfinite(v_ego) and v_ego < 1.0:
              hold_s = 60.0
    except Exception:
      pass

    if self._road_name_last and (now - self._road_name_last_t) < hold_s:
      self._road_loc_cache = self._road_name_last
      self._road_loc_cache_t = now
      return self._road_loc_cache

    lat = None
    lon = None
    for service in ("gpsLocationExternal", "gpsLocation"):
      try:
        if getattr(sm, "alive", {}).get(service, False):
          msg = sm[service]
          if hasattr(msg, "hasFix") and not getattr(msg, "hasFix", False):
            continue
          lat_v = getattr(msg, "latitude", None)
          lon_v = getattr(msg, "longitude", None)
          if lat_v is None or lon_v is None:
            continue
          lat_f = float(lat_v)
          lon_f = float(lon_v)
          if math.isfinite(lat_f) and math.isfinite(lon_f):
            lat = lat_f
            lon = lon_f
            break
      except Exception:
        continue

    self._road_loc_cache = f"{lat:.5f},{lon:.5f}" if lat is not None and lon is not None else "--"
    self._road_loc_cache_t = now
    return self._road_loc_cache

  def _get_direction_label(self) -> str:
    sm = ui_state.sm
    try:
      def bearing_to_label(bearing_deg: float):
        if math.isfinite(bearing_deg):
          idx = int(((bearing_deg % 360) + 22.5) // 45) % len(PERF_DIRECTION_LABELS)
          return tr(PERF_DIRECTION_LABELS[idx])
        return None

      def extract_label(msg):
        if not getattr(msg, "hasFix", True):
          return None
        bearing_deg = getattr(msg, "bearingDeg", float("nan"))
        if not math.isfinite(bearing_deg):
          vn = getattr(msg, "vN", None)
          ve = getattr(msg, "vE", None)
          if vn is not None and ve is not None:
            bearing_deg = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0
        return bearing_to_label(bearing_deg)

      for service in ("gpsLocationExternal", "gpsLocation"):
        if getattr(sm, "alive", {}).get(service, False):
          label = extract_label(sm[service])
          if label is not None:
            return label
      # Fallback: livePose velocity向量
      if getattr(sm, "alive", {}).get("livePose", False):
        lp = sm['livePose']
        vel = getattr(lp, "velocityDevice", None)
        if vel is not None and getattr(vel, "valid", False):
          vn = getattr(vel, "x", float("nan"))
          ve = getattr(vel, "y", float("nan"))
          if math.isfinite(vn) and math.isfinite(ve) and (abs(vn) + abs(ve) > 0.1):
            bearing_deg = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0
            label = bearing_to_label(bearing_deg)
            if label is not None:
              return label
    except Exception:
      pass
    return "---"

  def _get_control_state_text(self) -> str:
    status = ui_state.status
    if status == UIStatus.ENGAGED:
      return tr("Auto control")
    return tr("Manual control")

  def _get_perf_stats(self) -> dict[str, str]:
    with self._perf_lock:
      return dict(self._perf_stats)

  def _perf_update_loop(self) -> None:
    time.sleep(5)
    while self._perf_running:
      stats = {
        "cpu_temp": self._read_cpu_temp(),
        "mem_usage": self._read_mem_usage(),
        "disk_free": self._read_disk_free(),
      }
      with self._perf_lock:
        self._perf_stats.update(stats)
      for _ in range(10):
        if not self._perf_running:
          return
        time.sleep(0.1)

  @staticmethod
  def _read_cpu_temp() -> str:
    path = "/sys/class/thermal/thermal_zone0/temp"
    try:
      with open(path) as f:
        temp_c = int(f.read().strip()) / 1000.0
        return f"{temp_c:.0f}°C"
    except Exception:
      return "N/A"

  @staticmethod
  def _read_mem_usage() -> str:
    try:
      total_kb = None
      available_kb = None
      with open("/proc/meminfo") as f:
        for line in f:
          if line.startswith("MemTotal:"):
            total_kb = float(line.split()[1])
          elif line.startswith("MemAvailable:"):
            available_kb = float(line.split()[1])
          if total_kb is not None and available_kb is not None:
            break
      if total_kb and available_kb:
        used_pct = (total_kb - available_kb) / total_kb * 100.0
        used_pct = min(max(used_pct, 0.0), 100.0)
        return f"{used_pct:.0f}%"
    except Exception:
      pass
    return "N/A"

  @staticmethod
  def _read_disk_free() -> str:
    try:
      usage = shutil.disk_usage("/data")
      free_gb = usage.free / (1024 ** 3)
      if free_gb >= 1.0:
        return f"{free_gb:.1f}GB"
      free_mb = usage.free / (1024 ** 2)
      return f"{free_mb:.0f}MB"
    except Exception:
      return "N/A"

  def close(self) -> None:
    self._perf_running = False
    if hasattr(self, "_perf_thread") and self._perf_thread and self._perf_thread.is_alive():
      self._perf_thread.join(timeout=1.0)
    super().close()

if __name__ == "__main__":
  gui_app.init_window("OnRoad Camera View")
  road_camera_view = AugmentedRoadView(ROAD_CAM)
  print("***press space to switch camera view***")
  try:
    for _ in gui_app.render():
      ui_state.update()
      if rl.is_key_released(rl.KeyboardKey.KEY_SPACE):
        if WIDE_CAM in road_camera_view.available_streams:
          stream = ROAD_CAM if road_camera_view.stream_type == WIDE_CAM else WIDE_CAM
          road_camera_view.switch_stream(stream)
      road_camera_view.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  finally:
    road_camera_view.close()
