from __future__ import annotations

import math
import json
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
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections
from openpilot.selfdrive.modeld.lane_occupancy import compute_lane_occupancy
from openpilot.selfdrive.ui.onroad.object_tracker import ObjectTracker

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
DP_INDICATOR_BLINKER_ONLY_HZ = 1.0
DP_INDICATOR_BSM_ONLY_HZ = 3.3
DP_INDICATOR_BSM_WITH_BLINKER_HZ = 5.0
DP_INDICATOR_COLOR_BSM_ENHANCED = rl.Color(255, 0, 0, 255)
DP_INDICATOR_COLOR_BLINKER_ENHANCED = rl.Color(255, 255, 0, 255)
DP_BSD_ICON_TEXTURE_SIZE = 256
DP_BSD_ICON_WIDTH = 256.0
DP_BSD_ICON_HEIGHT = 256.0
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
DET_STALE_TIMEOUT_S = 2.0
DET_COLOR_CONE = rl.Color(255, 149, 0, 220)
DET_COLOR_PERSON = rl.Color(0, 170, 255, 220)
DET_COLOR_VEHICLE = rl.Color(255, 60, 60, 220)
DET_COLOR_SIGN = rl.Color(255, 220, 0, 220)
DET_COLOR_ANIMAL = rl.Color(180, 120, 255, 220)
DET_COLOR_OTHER = rl.Color(220, 220, 220, 200)
DET_LABEL_COLOR = rl.WHITE
DET_LABEL_BG_ALPHA = 180
DET_SHOW_TRACK_ID = bool(int(os.getenv("DP_DET_SHOW_TRACK_ID", "0")))
DET_SHOW_DISTANCE = bool(int(os.getenv("DP_DET_SHOW_DISTANCE", "0")))
DET_DRAW_ALL = bool(int(os.getenv("DP_DET_DRAW_ALL", "0")))
DET_VEHICLE_CLASS_IDS = {1, 2, 3, 5, 7}  # bicycle/car/moto/bus/truck
DET_DRAW_CLASSES = {0, 80} | DET_VEHICLE_CLASS_IDS

# Adjacent-lane occupancy visualization (uses coned bboxes + model lane lines).
# This is a conservative "block" cue for the driver, not a guarantee that a lane is clear.
AUTO_LC_UI_TARGET_LANE_TGAP_S = float(os.getenv("DP_LINCOLN_AUTO_LC_TGAP_S", "2.0"))
AUTO_LC_UI_TARGET_LANE_MIN_DIST_M = float(os.getenv("DP_LINCOLN_AUTO_LC_MIN_DIST_M", "25.0"))
AUTO_LC_UI_TARGET_LANE_MAX_DIST_M = float(os.getenv("DP_LINCOLN_AUTO_LC_MAX_DIST_M", "90.0"))
AUTO_LC_UI_LANE_LINE_PROB_MIN = float(os.getenv("DP_LINCOLN_AUTO_LC_LANE_LINE_PROB_MIN", "0.30"))
AUTO_LC_UI_LANE_MARGIN_M = float(os.getenv("DP_LINCOLN_AUTO_LC_LANE_MARGIN_M", "-0.20"))
AUTO_LC_UI_SIDE_CLOSE_DIST_M = float(os.getenv("DP_LINCOLN_AUTO_LC_SIDE_CLOSE_DIST_M", "15.0"))
DET_COLOR_LANE_BLOCK = rl.Color(255, 149, 0, 255)  # orange

# COCO-80 plus `traffic cone (80)` used by `Cone_YOLO11n`.
YOLO_CLASS_NAMES = (
  "person",
  "bicycle",
  "car",
  "motorcycle",
  "airplane",
  "bus",
  "train",
  "truck",
  "boat",
  "traffic light",
  "fire hydrant",
  "stop sign",
  "parking meter",
  "bench",
  "bird",
  "cat",
  "dog",
  "horse",
  "sheep",
  "cow",
  "elephant",
  "bear",
  "zebra",
  "giraffe",
  "backpack",
  "umbrella",
  "handbag",
  "tie",
  "suitcase",
  "frisbee",
  "skis",
  "snowboard",
  "sports ball",
  "kite",
  "baseball bat",
  "baseball glove",
  "skateboard",
  "surfboard",
  "tennis racket",
  "bottle",
  "wine glass",
  "cup",
  "fork",
  "knife",
  "spoon",
  "bowl",
  "banana",
  "apple",
  "sandwich",
  "orange",
  "broccoli",
  "carrot",
  "hot dog",
  "pizza",
  "donut",
  "cake",
  "chair",
  "couch",
  "potted plant",
  "bed",
  "dining table",
  "toilet",
  "tv",
  "laptop",
  "mouse",
  "remote",
  "keyboard",
  "cell phone",
  "microwave",
  "oven",
  "toaster",
  "sink",
  "refrigerator",
  "book",
  "clock",
  "vase",
  "scissors",
  "teddy bear",
  "hair drier",
  "toothbrush",
  "traffic cone",
)


def _det_label_and_color(cls: int, score: float) -> tuple[str, rl.Color]:
  cls = int(cls)
  name = f"ID{cls}"
  if cls == 0:
    name = "PED"
  elif cls == 80:
    name = "CONE"
  elif 0 <= cls < len(YOLO_CLASS_NAMES):
    name = YOLO_CLASS_NAMES[cls].upper()

  if cls == 0:
    color = DET_COLOR_PERSON
  elif cls == 80:
    color = DET_COLOR_CONE
  elif cls in (1, 2, 3, 4, 5, 6, 7, 8):
    color = DET_COLOR_VEHICLE
  elif cls in (9, 10, 11, 12):
    color = DET_COLOR_SIGN
  elif 14 <= cls <= 23:
    color = DET_COLOR_ANIMAL
  else:
    color = DET_COLOR_OTHER

  return name, color


def _det_obj_height_m(cls: int) -> float:
  cls = int(cls)
  if cls == 0:
    return 1.7
  if cls == 80:
    return 0.7
  if cls in DET_VEHICLE_CLASS_IDS:
    return 1.5
  return 0.0


def _lane_y_at_x(lane_x: np.ndarray, lane_y: np.ndarray, x_m: float) -> float | None:
  if lane_x.size < 2 or lane_y.size < 2 or lane_x.size != lane_y.size:
    return None
  x0 = float(lane_x[0])
  x1 = float(lane_x[-1])
  if not (x0 <= float(x_m) <= x1):
    return None
  return float(np.interp(float(x_m), lane_x, lane_y))


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
    self._dp_indicator_started_at_left: float | None = None
    self._dp_indicator_started_at_right: float | None = None
    self._txt_bsd_left: rl.Texture = gui_app.texture("icons/bsd_l.png", DP_BSD_ICON_TEXTURE_SIZE, DP_BSD_ICON_TEXTURE_SIZE)
    self._txt_bsd_right: rl.Texture = gui_app.texture("icons/bsd_r.png", DP_BSD_ICON_TEXTURE_SIZE, DP_BSD_ICON_TEXTURE_SIZE)

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
    self._road_name_candidate = ""
    self._road_name_candidate_t = 0.0
    self._road_gps_last = ""
    self._road_gps_last_t = 0.0
    self._road_font_cache: dict[tuple[int, ...], _DynamicFontCacheEntry] = {}
    self._map_tv_raw_cache: str | None = None
    self._map_tv_points_cache: int = 0
    self._map_tv_cache_t = 0.0

    # Lincoln HUD enhancements
    self._hud_brake_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)

    # Object detections (coned -> customReservedRawData0)
    self._det_payload: dict | None = None
    self._det_last_update_t = 0.0
    self._det_img_w = 0
    self._det_img_h = 0
    self._det_tracker = ObjectTracker()
    self._det_tracker_uses_sof_time = False

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
    self._draw_performance_info()
    if not hide_hud:
      self._hud_renderer.render(self._content_rect)
    self.alert_renderer.render(self._content_rect)
    if not LITE:
      if not hide_hud:
        self.driver_state_renderer.render(self._content_rect)

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds
    self._draw_object_detections(rect)

    # End clipping region
    rl.end_scissor_mode()

    # Draw colored border based on driving state
    self._draw_border(rect)

    # publish uiDebug
    msg = messaging.new_message('uiDebug')
    msg.uiDebug.drawTimeMillis = (time.monotonic() - start_draw) * 1000
    self._pm.send('uiDebug', msg)

  def _get_camera_dst_rect(self, rect: rl.Rectangle) -> rl.Rectangle | None:
    if self.frame is None:
      return None

    transform = self._calc_frame_matrix(rect)
    scale_x = rect.width * float(transform[0, 0])  # zx
    scale_y = rect.height * float(transform[1, 1])  # zy

    x_offset = rect.x + (rect.width - scale_x) / 2.0
    y_offset = rect.y + (rect.height - scale_y) / 2.0

    x_offset += float(transform[0, 2]) * rect.width / 2.0
    y_offset += float(transform[1, 2]) * rect.height / 2.0

    return rl.Rectangle(float(x_offset), float(y_offset), float(scale_x), float(scale_y))

  def _draw_object_detections(self, rect: rl.Rectangle) -> None:
    if not self._params.get_bool("dp_lat_cone_detection"):
      return
    sm = ui_state.sm
    det_ts_sof_ns = 0
    det_t_s: float | None = None
    if sm.updated.get("customReservedRawData0", False):
      try:
        raw = sm["customReservedRawData0"]
        payload = decode_cone_detections(raw) if raw else None
        if payload is not None:
          self._det_payload = payload
          self._det_last_update_t = time.monotonic()
          det_ts_sof_ns = int(self._det_payload.get("timestampSof", 0) or 0)
          if det_ts_sof_ns > 0:
            det_t_s = float(det_ts_sof_ns) * 1e-9
          img_w = int(self._det_payload.get("imgW", 0) or 0)
          img_h = int(self._det_payload.get("imgH", 0) or 0)
          if img_w > 0 and img_h > 0 and (img_w != self._det_img_w or img_h != self._det_img_h):
            self._det_img_w = img_w
            self._det_img_h = img_h
            self._det_tracker.reset()

          objs = self._det_payload.get("objsR", []) or []
          if not objs:
            objs = self._det_payload.get("objs", []) or []

          # Backward/robustness: if the refined/raw streams are empty but `cones` is present,
          # surface cones in the HUD by promoting them into the object stream.
          try:
            have_cone = any(isinstance(o, dict) and int(o.get("c", -1)) == 80 for o in objs)
            cones = self._det_payload.get("cones", []) or []
            if (not have_cone) and isinstance(cones, list):
              for c in cones:
                if not isinstance(c, dict):
                  continue
                objs.append({
                  "c": 80,
                  "x1": float(c.get("x1", 0.0)),
                  "y1": float(c.get("y1", 0.0)),
                  "x2": float(c.get("x2", 0.0)),
                  "y2": float(c.get("y2", 0.0)),
                  "s": float(c.get("s", 0.0)),
                })
          except Exception:
            pass
          if not DET_DRAW_ALL:
            try:
              objs = [o for o in objs if isinstance(o, dict) and int(o.get("c", -1)) in DET_DRAW_CLASSES]
            except Exception:
              objs = []
          self._det_tracker_uses_sof_time = det_t_s is not None
          self._det_tracker.update(objs=objs, now=det_t_s if det_t_s is not None else self._det_last_update_t)
      except Exception:
        self._det_payload = None
        self._det_last_update_t = 0.0
        self._det_img_w = 0
        self._det_img_h = 0
        self._det_tracker.reset()
        self._det_tracker_uses_sof_time = False

    if self._det_payload is None:
      return

    draw_now = time.monotonic()
    if draw_now - self._det_last_update_t > DET_STALE_TIMEOUT_S:
      self._det_payload = None
      self._det_img_w = 0
      self._det_img_h = 0
      self._det_tracker.reset()
      self._det_tracker_uses_sof_time = False
      return

    cam_dst = self._get_camera_dst_rect(rect)
    if cam_dst is None:
      return

    img_w = int(self._det_payload.get("imgW", 0) or 0)
    img_h = int(self._det_payload.get("imgH", 0) or 0)
    if img_w <= 0 or img_h <= 0:
      return

    focal_length_px_det = 0.0
    try:
      focal_length_px_det = float(self._det_payload.get("focalLengthPx", 0.0) or 0.0)
    except Exception:
      focal_length_px_det = 0.0
    focal_length_px = float(focal_length_px_det) if DET_SHOW_DISTANCE else 0.0

    cs = sm["carState"] if sm.alive.get("carState", False) else None

    lane_left_x = np.empty((0,), dtype=np.float32)
    lane_left_y = np.empty((0,), dtype=np.float32)
    lane_right_x = np.empty((0,), dtype=np.float32)
    lane_right_y = np.empty((0,), dtype=np.float32)
    lane_lines_ok = False
    try:
      if sm.valid.get("modelV2", False):
        model = sm["modelV2"]
        lane_lines = getattr(model, "laneLines", [])
        lane_probs = getattr(model, "laneLineProbs", [])
        if len(lane_lines) >= 3 and len(lane_probs) >= 3:
          if float(lane_probs[1]) >= AUTO_LC_UI_LANE_LINE_PROB_MIN and float(lane_probs[2]) >= AUTO_LC_UI_LANE_LINE_PROB_MIN:
            lane_left_x = np.asarray(lane_lines[1].x, dtype=np.float32)
            lane_left_y = np.asarray(lane_lines[1].y, dtype=np.float32)
            lane_right_x = np.asarray(lane_lines[2].x, dtype=np.float32)
            lane_right_y = np.asarray(lane_lines[2].y, dtype=np.float32)

            if lane_left_x.size >= 2 and float(lane_left_x[0]) > float(lane_left_x[-1]):
              lane_left_x = lane_left_x[::-1]
              lane_left_y = lane_left_y[::-1]
            if lane_right_x.size >= 2 and float(lane_right_x[0]) > float(lane_right_x[-1]):
              lane_right_x = lane_right_x[::-1]
              lane_right_y = lane_right_y[::-1]

            lane_lines_ok = bool(lane_left_x.size >= 2 and lane_right_x.size >= 2 and lane_left_x.size == lane_left_y.size and lane_right_x.size == lane_right_y.size)
    except Exception:
      lane_lines_ok = False

    # Adjacent-lane hazard marking (vision-only).
    # Draw an orange FrogPilot-style wall in the adjacent lane polygon when coned+lane-lines estimate
    # an occupied target lane close enough to block a lane change.
    try:
      if cs is not None and lane_lines_ok and focal_length_px_det > 1.0:
        objs_raw = (self._det_payload.get("objsR", None) or self._det_payload.get("objs", [])) or []
        if isinstance(objs_raw, list):
          occ = compute_lane_occupancy(
            objs=objs_raw,
            img_w=img_w,
            img_h=img_h,
            focal_length_px=focal_length_px_det,
            lane_left_x=lane_left_x,
            lane_left_y=lane_left_y,
            lane_right_x=lane_right_x,
            lane_right_y=lane_right_y,
            lane_margin_m=AUTO_LC_UI_LANE_MARGIN_M,
            side_close_dist_m=AUTO_LC_UI_SIDE_CLOSE_DIST_M,
            score_min_vehicle=0.25,
          )

          v_ego = float(getattr(cs, "vEgo", 0.0))
          target_lane_block_dist_m = max(
            AUTO_LC_UI_TARGET_LANE_MIN_DIST_M,
            min(AUTO_LC_UI_TARGET_LANE_MAX_DIST_M, v_ego * AUTO_LC_UI_TARGET_LANE_TGAP_S),
          )

          left_block = (occ.left_min_dist_m > 0.1 and occ.left_min_dist_m < target_lane_block_dist_m) or bool(occ.left_side_close)
          right_block = (occ.right_min_dist_m > 0.1 and occ.right_min_dist_m < target_lane_block_dist_m) or bool(occ.right_side_close)

          # Avoid drawing over the true BSM wall.
          left_bsm = bool(getattr(cs, "leftBlindspot", False))
          right_bsm = bool(getattr(cs, "rightBlindspot", False))

          if left_block and not left_bsm:
            d = occ.left_min_dist_m if occ.left_min_dist_m > 0.1 else target_lane_block_dist_m
            intensity = float(np.clip((target_lane_block_dist_m - d) / max(target_lane_block_dist_m, 1.0), 0.25, 1.0))
            self._draw_hud_fp_lane_block_wall(rect=self._content_rect, is_left=True, intensity=intensity)
          if right_block and not right_bsm:
            d = occ.right_min_dist_m if occ.right_min_dist_m > 0.1 else target_lane_block_dist_m
            intensity = float(np.clip((target_lane_block_dist_m - d) / max(target_lane_block_dist_m, 1.0), 0.25, 1.0))
            self._draw_hud_fp_lane_block_wall(rect=self._content_rect, is_left=False, intensity=intensity)
    except Exception:
      pass

    # Align tracking to the currently displayed camera frame timestamp, otherwise boxes will lag by
    # the detector pipeline latency (coned runs asynchronously at low Hz).
    draw_t_s = draw_now
    if self._det_tracker_uses_sof_time:
      try:
        frame_ts_sof_ns = int(getattr(self.frame, "timestamp_sof", 0) or 0) if self.frame is not None else 0
        if frame_ts_sof_ns > 0:
          draw_t_s = float(frame_ts_sof_ns) * 1e-9
      except Exception:
        draw_t_s = draw_now

    tracks = self._det_tracker.get_tracked(now=draw_t_s)
    if not tracks:
      return

    font = gui_app.font(FontWeight.MEDIUM)
    spacing = 1.0

    for o in tracks:
      cls = int(o.cls)
      x1 = float(max(0.0, min(float(img_w), o.x1)))
      y1 = float(max(0.0, min(float(img_h), o.y1)))
      x2 = float(max(0.0, min(float(img_w), o.x2)))
      y2 = float(max(0.0, min(float(img_h), o.y2)))
      score = float(o.score)
      missed = int(o.missed)

      if x2 <= x1 or y2 <= y1:
        continue

      dist_m: float | None = None
      lane_dir: int | None = None  # left=1, ego=0, right=-1
      obj_h_m = _det_obj_height_m(cls)
      if focal_length_px_det > 1.0 and obj_h_m > 0.1:
        h_px = float(max(1.0, y2 - y1))
        d = (float(focal_length_px_det) * float(obj_h_m)) / h_px
        if np.isfinite(d):
          dist_m = float(max(0.0, min(250.0, d)))

        if dist_m is not None and lane_lines_ok:
          cx = 0.5 * (x1 + x2)
          y_m = -((float(cx) - float(img_w) * 0.5) * float(dist_m) / max(float(focal_length_px_det), 1.0))
          if np.isfinite(y_m):
            y_left = _lane_y_at_x(lane_left_x, lane_left_y, dist_m)
            y_right = _lane_y_at_x(lane_right_x, lane_right_y, dist_m)
            if y_left is not None and y_right is not None:
              left_boundary = max(float(y_left), float(y_right))
              right_boundary = min(float(y_left), float(y_right))
              margin = float(AUTO_LC_UI_LANE_MARGIN_M)
              if y_m > (left_boundary + margin):
                lane_dir = 1
              elif y_m < (right_boundary - margin):
                lane_dir = -1
              else:
                lane_dir = 0

      label, base_color = _det_label_and_color(cls, score)
      if DET_SHOW_TRACK_ID:
        label = f"{label} #{int(o.track_id)}"
      if DET_SHOW_DISTANCE and dist_m is not None and focal_length_px > 1.0:
        label = f"{label} {dist_m:.0f}m"

      sx1 = cam_dst.x + (x1 / img_w) * cam_dst.width
      sy1 = cam_dst.y + (y1 / img_h) * cam_dst.height
      sx2 = cam_dst.x + (x2 / img_w) * cam_dst.width
      sy2 = cam_dst.y + (y2 / img_h) * cam_dst.height

      w = sx2 - sx1
      h = sy2 - sy1
      if w < 2.0 or h < 2.0:
        continue

      # Size-adaptive styling
      min_dim = float(min(w, h))
      thickness = int(max(1.0, min(4.0, min_dim / 90.0)))
      font_size = int(max(14.0, min(22.0, min_dim / 8.0)))

      missed_fade = float(max(0.35, 1.0 - 0.14 * missed))
      score_fade = 0.70
      try:
        if np.isfinite(score):
          score_fade = float(np.interp(score, [0.10, 0.60], [0.55, 1.0]))
      except Exception:
        score_fade = 0.70

      lane_fade = 0.85
      if lane_dir == 0:
        lane_fade = 1.0
      elif lane_dir in (-1, 1):
        lane_fade = 0.92

      fade = float(max(0.25, min(1.0, missed_fade * score_fade * lane_fade)))
      color = rl.Color(base_color.r, base_color.g, base_color.b, int(base_color.a * fade))

      box = rl.Rectangle(float(sx1), float(sy1), float(w), float(h))
      if cls in DET_VEHICLE_CLASS_IDS:
        self._draw_det_corner_box(box=box, thickness=float(thickness), color=color)
        if lane_dir in (-1, 1):
          self._draw_det_lane_arrow(box=box, lane_dir=int(lane_dir), color=color)
      elif cls == 0:
        self._draw_det_person_icon(box=box, thickness=float(thickness), color=color)
      elif cls == 80:
        self._draw_det_cone_icon(box=box, thickness=float(thickness), color=color)
      else:
        rl.draw_rectangle_lines_ex(box, thickness, color)

      # Labels get noisy on tiny boxes; keep HUD readable.
      show_label = (cls in DET_VEHICLE_CLASS_IDS) or DET_SHOW_TRACK_ID or (DET_SHOW_DISTANCE and dist_m is not None)
      if show_label and min_dim >= 35.0:
        label_size = measure_text_cached(font, label, font_size, spacing)
        pad = 4
        bg = rl.Color(color.r, color.g, color.b, DET_LABEL_BG_ALPHA)
        label_rect = rl.Rectangle(box.x, max(cam_dst.y, box.y - (label_size.y + pad * 2)), label_size.x + pad * 2, label_size.y + pad * 2)
        rl.draw_rectangle_rec(label_rect, bg)
        rl.draw_text_ex(font, label, rl.Vector2(label_rect.x + pad, label_rect.y + pad), font_size, spacing, DET_LABEL_COLOR)

  @staticmethod
  def _draw_det_corner_box(*, box: rl.Rectangle, thickness: float, color: rl.Color) -> None:
    w = float(box.width)
    h = float(box.height)
    if w <= 2.0 or h <= 2.0:
      return

    x1 = float(box.x)
    y1 = float(box.y)
    x2 = x1 + w
    y2 = y1 + h

    min_dim = float(min(w, h))
    corner = float(np.clip(min_dim * 0.30, 8.0, 28.0))
    corner = float(min(corner, w * 0.45, h * 0.45))
    t = float(max(1.0, thickness))

    # top-left
    rl.draw_line_ex(rl.Vector2(x1, y1), rl.Vector2(x1 + corner, y1), t, color)
    rl.draw_line_ex(rl.Vector2(x1, y1), rl.Vector2(x1, y1 + corner), t, color)
    # top-right
    rl.draw_line_ex(rl.Vector2(x2 - corner, y1), rl.Vector2(x2, y1), t, color)
    rl.draw_line_ex(rl.Vector2(x2, y1), rl.Vector2(x2, y1 + corner), t, color)
    # bottom-left
    rl.draw_line_ex(rl.Vector2(x1, y2 - corner), rl.Vector2(x1, y2), t, color)
    rl.draw_line_ex(rl.Vector2(x1, y2), rl.Vector2(x1 + corner, y2), t, color)
    # bottom-right
    rl.draw_line_ex(rl.Vector2(x2, y2 - corner), rl.Vector2(x2, y2), t, color)
    rl.draw_line_ex(rl.Vector2(x2 - corner, y2), rl.Vector2(x2, y2), t, color)

  @staticmethod
  def _draw_det_lane_arrow(*, box: rl.Rectangle, lane_dir: int, color: rl.Color) -> None:
    if int(lane_dir) not in (-1, 1):
      return

    w = float(box.width)
    h = float(box.height)
    if w <= 4.0 or h <= 4.0:
      return

    min_dim = float(min(w, h))
    size = float(np.clip(min_dim * 0.22, 8.0, 20.0))
    cx = float(box.x + w * 0.5)
    y = float(box.y + size * 0.10)

    if int(lane_dir) == 1:  # left
      tip = rl.Vector2(cx - size * 0.65, y + size * 0.50)
      b1 = rl.Vector2(cx + size * 0.65, y + size * 0.10)
      b2 = rl.Vector2(cx + size * 0.65, y + size * 0.90)
    else:  # right
      tip = rl.Vector2(cx + size * 0.65, y + size * 0.50)
      b1 = rl.Vector2(cx - size * 0.65, y + size * 0.10)
      b2 = rl.Vector2(cx - size * 0.65, y + size * 0.90)

    a = int(np.clip(max(100, int(color.a)), 0, 255))
    fill = rl.Color(color.r, color.g, color.b, a)
    rl.draw_triangle(tip, b1, b2, fill)

  @staticmethod
  def _draw_det_cone_icon(*, box: rl.Rectangle, thickness: float, color: rl.Color) -> None:
    w = float(box.width)
    h = float(box.height)
    if w <= 2.0 or h <= 2.0:
      return

    min_dim = float(min(w, h))
    size = float(np.clip(min_dim * 0.55, 10.0, 26.0))
    cx = float(box.x + w * 0.5)
    cy = float(box.y + h)
    t = float(max(1.0, thickness))

    p1 = rl.Vector2(cx, cy - size)
    p2 = rl.Vector2(cx - size * 0.60, cy)
    p3 = rl.Vector2(cx + size * 0.60, cy)

    fill = rl.Color(color.r, color.g, color.b, int(color.a * 0.45))
    rl.draw_triangle(p1, p2, p3, fill)
    rl.draw_line_ex(p1, p2, t, color)
    rl.draw_line_ex(p2, p3, t, color)
    rl.draw_line_ex(p3, p1, t, color)
    rl.draw_line_ex(rl.Vector2(cx - size * 0.75, cy), rl.Vector2(cx + size * 0.75, cy), t, color)

  @staticmethod
  def _draw_det_person_icon(*, box: rl.Rectangle, thickness: float, color: rl.Color) -> None:
    w = float(box.width)
    h = float(box.height)
    if w <= 2.0 or h <= 2.0:
      return

    min_dim = float(min(w, h))
    size = float(np.clip(min_dim * 0.70, 14.0, 34.0))
    cx = float(box.x + w * 0.5)
    cy = float(box.y + h)
    t = float(max(1.0, thickness))

    head_r = float(size * 0.16)
    head_cy = float(cy - size * 0.78)
    head_fill = rl.Color(color.r, color.g, color.b, int(color.a * 0.25))
    rl.draw_circle(int(cx), int(head_cy), head_r, head_fill)
    rl.draw_circle_lines(int(cx), int(head_cy), head_r, color)

    body_top = rl.Vector2(cx, float(cy - size * 0.62))
    body_mid = rl.Vector2(cx, float(cy - size * 0.28))
    rl.draw_line_ex(body_top, body_mid, t, color)

    rl.draw_line_ex(
      rl.Vector2(float(cx - size * 0.22), float(cy - size * 0.50)),
      rl.Vector2(float(cx + size * 0.22), float(cy - size * 0.50)),
      t,
      color,
    )
    rl.draw_line_ex(body_mid, rl.Vector2(float(cx - size * 0.18), cy), t, color)
    rl.draw_line_ex(body_mid, rl.Vector2(float(cx + size * 0.18), cy), t, color)

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

    # Brake intensity colors the whole border (FrogPilot-style "whole frame" cue)
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
    left_blindspot = False
    right_blindspot = False
    if sm.alive.get("carState", False):
      left_blindspot = bool(getattr(sm["carState"], "leftBlindspot", False))
      right_blindspot = bool(getattr(sm["carState"], "rightBlindspot", False))

    if self._dp_indicator_show_left:
      rl.draw_rectangle(int(rect.x), indicator_y, UI_BORDER_SIZE, indicator_height, self._dp_indicator_color_left)
      if left_blindspot:
        self._draw_dp_bsd_icon(rect, is_left=True)
    if self._dp_indicator_show_right:
      rl.draw_rectangle(int(rect.x + rect.width-UI_BORDER_SIZE), indicator_y, UI_BORDER_SIZE, indicator_height, self._dp_indicator_color_right)
      if right_blindspot:
        self._draw_dp_bsd_icon(rect, is_left=False)

  def _draw_dp_bsd_icon(self, rect: rl.Rectangle, *, is_left: bool) -> None:
    texture = self._txt_bsd_left if is_left else self._txt_bsd_right
    if getattr(texture, "id", 0) == 0:
      return

    icon_w = DP_BSD_ICON_WIDTH
    icon_h = DP_BSD_ICON_HEIGHT
    inset_x = float(UI_BORDER_SIZE) + 8.0
    # These two assets are 256x256, but the visible car/waves occupy only a small inner-side region.
    # Keep the display at original 256x256 size while offsetting the full texture so the visible icon
    # still sits at the inner side of each blindspot bar and stays vertically centered.
    visible_x = 1.0 if is_left else 184.0
    visible_y = 17.0
    visible_w = 71.0
    visible_h = 73.0

    if is_left:
      visible_left = float(rect.x + inset_x)
      icon_x = visible_left - visible_x
    else:
      visible_right = float(rect.x + rect.width - inset_x)
      icon_x = visible_right - (visible_x + visible_w)

    visible_center_y = float(rect.y + rect.height * 0.5)
    icon_y = visible_center_y - (visible_y + visible_h * 0.5)

    src_rect = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
    dst_rect = rl.Rectangle(icon_x, icon_y, icon_w, icon_h)
    rl.draw_texture_pro(texture, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)

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

  @staticmethod
  def _indicator_phase_show(elapsed_s: float, hz: float) -> bool:
    if hz <= 0.0:
      return True

    half_period_s = 0.5 / hz
    if half_period_s <= 0.0:
      return True

    return int(elapsed_s / half_period_s) % 2 == 0

  def _update_dp_indicator_side_state(self, blinker_state, bsm_state, show_prev, count_prev, started_at_prev):
    if not blinker_state and not bsm_state:
      return False, 0, rl.Color(0, 0, 0, 0), None

    now = time.monotonic()
    started_at = now if started_at_prev is None else started_at_prev
    elapsed_s = max(0.0, now - started_at)
    count = count_prev + 1
    show = True
    color = rl.Color(0, 0, 0, 0)

    # blinker = yellow flash, blindspot = red flash, both = red fast flash
    if bsm_state:
      hz = DP_INDICATOR_BSM_WITH_BLINKER_HZ if blinker_state else DP_INDICATOR_BSM_ONLY_HZ
      show = self._indicator_phase_show(elapsed_s, hz)
      color = DP_INDICATOR_COLOR_BSM_ENHANCED
    elif blinker_state:
      show = self._indicator_phase_show(elapsed_s, DP_INDICATOR_BLINKER_ONLY_HZ)
      color = DP_INDICATOR_COLOR_BLINKER_ENHANCED
    else:
      show = False

    return show, count, color, started_at

  def _update_dp_indicator_states(self, sm):
    cs = sm['carState']
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left, self._dp_indicator_started_at_left = \
      self._update_dp_indicator_side_state(cs.leftBlinker, cs.leftBlindspot,
                                           self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_started_at_left)
    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right, self._dp_indicator_started_at_right = \
      self._update_dp_indicator_side_state(cs.rightBlinker, cs.rightBlindspot,
                                           self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_started_at_right)

  def _draw_hud_enhancements(self) -> None:
    sm = ui_state.sm
    if not sm.alive.get("carState", False):
      return

    rect = self._content_rect
    if rect.width <= 0 or rect.height <= 0:
      return

    cs = sm["carState"]

    # FrogPilot-style blindspot "wall" (drawn in the adjacent lane polygon)
    if cs.leftBlindspot:
      drew = self._draw_hud_fp_blindspot_wall(rect=rect, is_left=True)
      # Removed: internal trapezoid blinker band

    if cs.rightBlindspot:
      drew = self._draw_hud_fp_blindspot_wall(rect=rect, is_left=False)
      # Removed: internal trapezoid blinker band

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

  def _draw_hud_fp_blindspot_wall(self, rect: rl.Rectangle, is_left: bool) -> bool:
    poly = self._fp_get_adjacent_lane_polygon(rect, is_left=is_left)
    if poly.size == 0:
      return False

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
    return True

  def _draw_hud_fp_lane_block_wall(self, rect: rl.Rectangle, is_left: bool, *, intensity: float) -> None:
    poly = self._fp_get_adjacent_lane_polygon(rect, is_left=is_left)
    if poly.size == 0:
      return

    intensity = float(np.clip(float(intensity), 0.0, 1.0))
    r, g, b = int(DET_COLOR_LANE_BLOCK.r), int(DET_COLOR_LANE_BLOCK.g), int(DET_COLOR_LANE_BLOCK.b)

    # Bottom-heavy orange gradient, scaled by intensity.
    a_top = int(np.clip((0.10 + 0.12 * intensity) * 255.0, 0.0, 255.0))
    a_mid = int(np.clip((0.20 + 0.22 * intensity) * 255.0, 0.0, 255.0))
    a_bot = int(np.clip((0.32 + 0.36 * intensity) * 255.0, 0.0, 255.0))
    gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[
        rl.Color(r, g, b, a_top),
        rl.Color(r, g, b, a_mid),
        rl.Color(r, g, b, a_bot),
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
    rect = self._content_rect
    if rect.width <= 0 or rect.height <= 0:
      return

    sm = ui_state.sm
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
    # Keep this bar away from the side HUD elements; too-wide bars can "cut" other UI overlays.
    max_width = max(min(rect.width - 40.0, rect.width * 0.98), 0.0)
    perf_font = font_fallback(self._perf_font)

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
        base_font = perf_font
        if self._font_has_missing_glyphs(base_font, items[road_item_idx]):
          road_item_font = self._get_dynamic_unifont_font(items[road_item_idx])

      measurements = []
      for idx, text in enumerate(items):
        if idx == road_item_idx and road_item_font is not None:
          measurements.append(self._measure_text_ex_no_fallback(road_item_font, text, PERF_FONT_SIZE))
        else:
          measurements.append(measure_text_cached(perf_font, text, PERF_FONT_SIZE))

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
        def measure_value_width(t: str, font: rl.Font = road_item_font) -> float:
          return self._measure_text_ex_no_fallback(font, t, PERF_FONT_SIZE).x
      else:
        label_width = measure_text_cached(perf_font, road_label, PERF_FONT_SIZE).x
        def measure_value_width(t: str) -> float:
          return measure_text_cached(perf_font, t, PERF_FONT_SIZE).x

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
    # Temporarily clip text to the bar bounds to prevent overlap with other HUD elements,
    # then restore the content scissor rectangle for subsequent renderers.
    try:
      rl.begin_scissor_mode(int(bar_x), int(bar_y), int(bar_width), int(bar_height))
    except Exception:
      pass
    for idx, (text, measurement) in enumerate(zip(items, measurements, strict=True)):
      if idx == road_item_idx and road_item_font is not None:
        self._draw_text_ex_no_fallback(road_item_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, rl.WHITE)
      else:
        rl.draw_text_ex(perf_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, rl.WHITE)
      cursor_x += measurement.x + gap
    try:
      rl.begin_scissor_mode(
        int(self._content_rect.x),
        int(self._content_rect.y),
        int(self._content_rect.width),
        int(self._content_rect.height)
      )
    except Exception:
      pass

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

  def _get_dynamic_unifont_font(self, text: str) -> rl.Font | None:
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

  def _get_map_target_velocities_points(self) -> int | None:
    """
    Returns the number of map target-velocity points currently available from mapd.

    This is a lightweight, HUD-friendly summary to help diagnose whether "地图融合" is
    working without collecting logs.
    """
    now = time.monotonic()
    if (now - float(self._map_tv_cache_t)) < 0.5:
      return int(self._map_tv_points_cache)

    raw = None
    for params in (getattr(self, "_params_memory", None), getattr(self, "_params", None)):
      if params is None:
        continue
      try:
        raw = params.get("MapTargetVelocities")
      except Exception:
        raw = None
      if raw:
        break

    if not raw:
      self._map_tv_raw_cache = None
      self._map_tv_points_cache = 0
      self._map_tv_cache_t = float(now)
      return 0

    if raw != self._map_tv_raw_cache:
      points = 0
      try:
        parsed = json.loads(raw)
        if isinstance(parsed, list):
          points = len(parsed)
      except Exception:
        points = 0
      self._map_tv_raw_cache = raw
      self._map_tv_points_cache = int(points)

    self._map_tv_cache_t = float(now)
    return int(self._map_tv_points_cache)

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

    gps_quality_ok = True
    for params in (getattr(self, "_params_memory", None), getattr(self, "_params", None)):
      if params is None:
        continue
      try:
        gps_quality_ok = bool(params.get_bool("GPSQualityOK"))
        break
      except Exception:
        continue

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
      # Keep the full road name; the renderer will ellipsize to width as needed.
      # Guard against pathological values from mapd to avoid excessive measurements.
      road_name = " ".join(road_name.split())
      if len(road_name) > 120:
        road_name = road_name[:117] + "..."

      # When GNSS is unstable (multipath), map matching can briefly jump to parallel roads.
      # Gate changes and hold the last stable RoadName to avoid flicker/wrong-road display.
      if not gps_quality_ok and self._road_name_last:
        road_name = ""
      elif self._road_name_last and road_name != self._road_name_last:
        debounce_s = 1.0
        if self._road_name_candidate != road_name:
          self._road_name_candidate = road_name
          self._road_name_candidate_t = now
          road_name = ""
        elif (now - self._road_name_candidate_t) < debounce_s:
          road_name = ""
        else:
          self._road_name_candidate = ""
          self._road_name_candidate_t = 0.0
      else:
        self._road_name_candidate = ""
        self._road_name_candidate_t = 0.0

      if road_name:
        self._road_name_last = road_name
        self._road_name_last_t = now
        self._road_loc_cache = road_name
        self._road_loc_cache_t = now
        return road_name

    # mapd can briefly output an empty road name during GPS jitter or process restarts.
    # Keep the last valid match to avoid flickering to raw coordinates, especially when stopped.
    sm = ui_state.sm
    hold_s = 30.0
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

    if lat is not None and lon is not None:
      gps_text = f"{lat:.5f},{lon:.5f}"
      self._road_gps_last = gps_text
      self._road_gps_last_t = now
      self._road_loc_cache = gps_text
    elif self._road_gps_last and (now - self._road_gps_last_t) < hold_s:
      # GPS can briefly drop fix; keep the last known coordinates for a short time to avoid "--" flicker.
      self._road_loc_cache = self._road_gps_last
    else:
      self._road_loc_cache = "--"
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
