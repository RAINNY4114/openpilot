from __future__ import annotations

import numpy as np
import pyray as rl
import time
from cereal import car, log
from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.mici.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.mici.onroad.confidence_ball import ConfidenceBall
from openpilot.selfdrive.ui.mici.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import FontWeight, gui_app, MousePos, MouseEvent
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import BounceFilter
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.ui.onroad import augmented_road_view as det_hud
from openpilot.selfdrive.ui.onroad.object_tracker import ObjectTracker
from enum import IntEnum

OpState = log.SelfdriveState.OpenpilotState
CALIBRATED = log.LiveCalibrationData.Status.calibrated
ROAD_CAM = VisionStreamType.VISION_STREAM_ROAD
WIDE_CAM = VisionStreamType.VISION_STREAM_WIDE_ROAD
DEFAULT_DEVICE_CAMERA = DEVICE_CAMERAS["tici", "ar0231"]


class BookmarkState(IntEnum):
  HIDDEN = 0
  DRAGGING = 1
  TRIGGERED = 2

WIDE_CAM_MAX_SPEED = 5.0  # m/s (10 mph)
ROAD_CAM_MIN_SPEED = 10  # m/s (25 mph)

CAM_Y_OFFSET = 20


class BookmarkIcon(Widget):
  PEEK_THRESHOLD = 50  # If icon peeks out this much, snap it fully visible
  FULL_VISIBLE_OFFSET = 200  # How far onscreen when fully visible
  HIDDEN_OFFSET = -50  # How far offscreen when hidden

  def __init__(self, bookmark_callback):
    super().__init__()
    self._bookmark_callback = bookmark_callback
    self._icon = gui_app.texture("icons_mici/onroad/bookmark.png", 180, 180)
    self._offset_filter = BounceFilter(0.0, 0.1, 1 / gui_app.target_fps)

    # State
    self._interacting = False
    self._state = BookmarkState.HIDDEN
    self._swipe_start_x = 0.0
    self._swipe_current_x = 0.0
    self._is_swiping = False
    self._is_swiping_left: bool = False
    self._triggered_time: float = 0.0

  def is_swiping_left(self) -> bool:
    """Check if currently swiping left (for scroller to disable)."""
    return self._is_swiping_left

  def interacting(self):
    interacting, self._interacting = self._interacting, False
    return interacting

  def _update_state(self):
    if self._state == BookmarkState.DRAGGING:
      # Allow pulling past activated position with rubber band effect
      swipe_offset = self._swipe_start_x - self._swipe_current_x
      swipe_offset = min(swipe_offset, self.FULL_VISIBLE_OFFSET + 50)
      self._offset_filter.update(swipe_offset)

    elif self._state == BookmarkState.TRIGGERED:
      # Continue animating to fully visible
      self._offset_filter.update(self.FULL_VISIBLE_OFFSET)
      # Stay in TRIGGERED state for 1 second
      if rl.get_time() - self._triggered_time >= 1.5:
        self._state = BookmarkState.HIDDEN

    elif self._state == BookmarkState.HIDDEN:
      self._offset_filter.update(self.HIDDEN_OFFSET)

      if self._offset_filter.x < 1e-3:
        self._interacting = False

  def _handle_mouse_event(self, mouse_event: MouseEvent):
    if not ui_state.started:
      return

    if mouse_event.left_pressed:
      # Store relative position within widget
      self._swipe_start_x = mouse_event.pos.x
      self._swipe_current_x = mouse_event.pos.x
      self._is_swiping = True
      self._is_swiping_left = False
      self._state = BookmarkState.DRAGGING

    elif mouse_event.left_down and self._is_swiping:
      self._swipe_current_x = mouse_event.pos.x
      swipe_offset = self._swipe_start_x - self._swipe_current_x
      self._is_swiping_left = swipe_offset > 0
      if self._is_swiping_left:
        self._interacting = True

    elif mouse_event.left_released:
      if self._is_swiping:
        swipe_distance = self._swipe_start_x - self._swipe_current_x

        # If peeking past threshold, transition to animating to fully visible and bookmark
        if swipe_distance > self.PEEK_THRESHOLD:
          self._state = BookmarkState.TRIGGERED
          self._triggered_time = rl.get_time()
          self._bookmark_callback()
        else:
          # Otherwise, transition back to hidden
          self._state = BookmarkState.HIDDEN

        # Reset swipe state
        self._is_swiping = False
        self._is_swiping_left = False

  def _render(self, _):
    """Render the bookmark icon."""
    if self._offset_filter.x > 0:
      icon_x = self.rect.x + self.rect.width - round(self._offset_filter.x)
      icon_y = self.rect.y + (self.rect.height - self._icon.height) / 2  # Vertically centered
      rl.draw_texture(self._icon, int(icon_x), int(icon_y), rl.WHITE)


class AugmentedRoadView(CameraView):
  def __init__(self, bookmark_callback=None, stream_type: VisionStreamType = VisionStreamType.VISION_STREAM_ROAD):
    super().__init__("camerad", stream_type)
    self._bookmark_callback = bookmark_callback
    self._set_placeholder_color(rl.BLACK)

    self.device_camera: DeviceCameraConfig | None = None
    self.view_from_calib = view_frame_from_device_frame.copy()
    self.view_from_wide_calib = view_frame_from_device_frame.copy()

    self._last_calib_time: float = 0
    self._last_rect_dims = (0.0, 0.0)
    self._last_stream_type = stream_type
    self._cached_matrix: np.ndarray | None = None
    self._content_rect = rl.Rectangle()
    self._last_click_time = 0.0

    # Bookmark icon with swipe gesture
    self._bookmark_icon = BookmarkIcon(bookmark_callback)

    self._model_renderer = ModelRenderer()
    self._hud_renderer = HudRenderer()
    self._alert_renderer = AlertRenderer()
    self._driver_state_renderer = DriverStateRenderer()
    self._confidence_ball = ConfidenceBall()
    self._offroad_label = UnifiedLabel("start the car to\nuse openpilot", 54, FontWeight.DISPLAY,
                                       text_color=rl.Color(255, 255, 255, int(255 * 0.9)),
                                       alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
                                       alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE)

    self._fade_texture = gui_app.texture("icons_mici/onroad/onroad_fade.png")
    self._params = Params()

    # Object detections (coned -> customReservedRawData0), shared with the big HUD path.
    self._det_payload: dict | None = None
    self._det_last_update_t = 0.0
    self._det_img_w = 0
    self._det_img_h = 0
    self._det_tracker = ObjectTracker()
    self._det_tracker_uses_sof_time = False

  def is_swiping_left(self) -> bool:
    """Check if currently swiping left (for scroller to disable)."""
    return self._bookmark_icon.is_swiping_left()

  def _update_state(self):
    super()._update_state()

    # update offroad label
    if ui_state.panda_type == log.PandaState.PandaType.unknown:
      self._offroad_label.set_text("system booting")
    else:
      self._offroad_label.set_text("start the car to\nuse openpilot")

  def _handle_mouse_release(self, mouse_pos: MousePos):
    # Don't trigger click callback if bookmark was triggered
    if not self._bookmark_icon.interacting():
      super()._handle_mouse_release(mouse_pos)

  def _render(self, _):
    self._switch_stream_if_needed(ui_state.sm)

    # Update calibration before rendering
    self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      self.rect.x,
      self.rect.y,
      self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.height,
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
    super()._render(self._content_rect)

    # Draw all UI overlays
    self._model_renderer.render(self._content_rect)

    # Fade out bottom of overlays for looks
    rl.draw_texture_ex(self._fade_texture, rl.Vector2(self._content_rect.x, self._content_rect.y), 0.0, 1.0, rl.WHITE)

    alert_to_render, not_animating_out = self._alert_renderer.will_render()

    # Hide DMoji when disengaged unless AlwaysOnDM is enabled
    should_draw_dmoji = (not self._hud_renderer.drawing_top_icons() and ui_state.is_onroad() and
                         (ui_state.status != UIStatus.DISENGAGED or ui_state.always_on_dm))
    self._driver_state_renderer.set_should_draw(should_draw_dmoji)
    self._driver_state_renderer.set_position(self._rect.x + 16, self._rect.y + 10)
    self._driver_state_renderer.render()

    self._hud_renderer.set_can_draw_top_icons(alert_to_render is None)
    self._hud_renderer.set_wheel_critical_icon(alert_to_render is not None and not not_animating_out and
                                               alert_to_render.visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired)
    # TODO: have alert renderer draw offroad mici label below
    if ui_state.started:
      self._alert_renderer.render(self._content_rect)
    self._hud_renderer.render(self._content_rect)
    self._draw_object_detections(self._content_rect)

    # Draw fake rounded border
    rl.draw_rectangle_rounded_lines_ex(self._content_rect, 0.2 * 1.02, 10, 50, rl.BLACK)

    # End clipping region
    rl.end_scissor_mode()

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds
    self._confidence_ball.render(self.rect)

    self._bookmark_icon.render(self.rect)

    # Draw darkened background and text if not onroad
    if not ui_state.started:
      rl.draw_rectangle(int(self.rect.x), int(self.rect.y), int(self.rect.width), int(self.rect.height), rl.Color(0, 0, 0, 175))
      self._offroad_label.render(self._content_rect)

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
    # Get camera configuration
    # TODO: cache with vEgo?
    calib_time = ui_state.sm.recv_frame['liveCalibration']
    current_dims = (self._content_rect.width, self._content_rect.height)
    device_camera = self.device_camera or DEFAULT_DEVICE_CAMERA
    is_wide_camera = self.stream_type == WIDE_CAM
    intrinsic = device_camera.ecam.intrinsics if is_wide_camera else device_camera.fcam.intrinsics
    calibration = self.view_from_wide_calib if is_wide_camera else self.view_from_calib
    if is_wide_camera:
      zoom = 0.7 * 1.5
    else:
      zoom = np.interp(ui_state.sm['carState'].vEgo, [10, 30], [0.8, 1.0])

    # Calculate transforms for vanishing point
    inf_point = np.array([1000.0, 0.0, 0.0])
    calib_transform = intrinsic @ calibration
    kep = calib_transform @ inf_point

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
        y_offset = np.clip((kep[1] / kep[2] - cy) * zoom + CAM_Y_OFFSET, -max_y_offset, max_y_offset)
      else:
        x_offset, y_offset = 0, 0
    except (ZeroDivisionError, OverflowError):
      x_offset, y_offset = 0, 0

    # Cache the computed transformation matrix to avoid recalculations
    self._last_calib_time = calib_time
    self._last_rect_dims = current_dims
    self._last_stream_type = self.stream_type
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
    self._model_renderer.set_transform(video_transform @ calib_transform)

    return self._cached_matrix

  def _get_camera_dst_rect(self, rect: rl.Rectangle) -> rl.Rectangle | None:
    if self.frame is None:
      return None

    transform = self._calc_frame_matrix(rect)
    scale_x = rect.width * float(transform[0, 0])
    scale_y = rect.height * float(transform[1, 1])

    x_offset = rect.x + (rect.width - scale_x) / 2.0
    y_offset = rect.y + (rect.height - scale_y) / 2.0
    x_offset += float(transform[0, 2]) * rect.width / 2.0
    y_offset += float(transform[1, 2]) * rect.height / 2.0

    return rl.Rectangle(float(x_offset), float(y_offset), float(scale_x), float(scale_y))

  def _draw_object_detections(self, rect: rl.Rectangle) -> None:
    if not det_hud._det_features_enabled(self._params):
      return

    sm = ui_state.sm
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

          objs = det_hud._det_payload_objects(self._det_payload, prefer_refined=True)
          if not det_hud.DET_DRAW_ALL:
            objs = [o for o in objs if isinstance(o, dict) and int(o.get("c", -1)) in det_hud.DET_DRAW_CLASSES]
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
    if draw_now - self._det_last_update_t > det_hud.DET_STALE_TIMEOUT_S:
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

    focal_length_px = 0.0
    try:
      focal_length_px = float(self._det_payload.get("focalLengthPx", 0.0) or 0.0)
    except Exception:
      focal_length_px = 0.0

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
          if float(lane_probs[1]) >= det_hud.AUTO_LC_UI_LANE_LINE_PROB_MIN and float(lane_probs[2]) >= det_hud.AUTO_LC_UI_LANE_LINE_PROB_MIN:
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
            lane_lines_ok = bool(lane_left_x.size >= 2 and lane_right_x.size >= 2 and
                                 lane_left_x.size == lane_left_y.size and lane_right_x.size == lane_right_y.size)
    except Exception:
      lane_lines_ok = False

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

    for o in tracks:
      cls = int(o.cls)
      x1 = float(max(0.0, min(float(img_w), o.x1)))
      y1 = float(max(0.0, min(float(img_h), o.y1)))
      x2 = float(max(0.0, min(float(img_w), o.x2)))
      y2 = float(max(0.0, min(float(img_h), o.y2)))
      if x2 <= x1 or y2 <= y1:
        continue

      lane_dir: int | None = None
      obj_h_m = det_hud._det_obj_height_m(cls)
      if focal_length_px > 1.0 and obj_h_m > 0.1 and lane_lines_ok:
        h_px = float(max(1.0, y2 - y1))
        dist_m = float(max(0.0, min(250.0, (float(focal_length_px) * float(obj_h_m)) / h_px)))
        cx = 0.5 * (x1 + x2)
        y_m = (float(cx) - float(img_w) * 0.5) * dist_m / max(float(focal_length_px), 1.0)
        if np.isfinite(y_m):
          y_left = det_hud._lane_y_at_x(lane_left_x, lane_left_y, dist_m)
          y_right = det_hud._lane_y_at_x(lane_right_x, lane_right_y, dist_m)
          if y_left is not None and y_right is not None:
            left_boundary = min(float(y_left), float(y_right))
            right_boundary = max(float(y_left), float(y_right))
            margin = float(det_hud.AUTO_LC_UI_LANE_MARGIN_M)
            if y_m < (left_boundary - margin):
              lane_dir = 1
            elif y_m > (right_boundary + margin):
              lane_dir = -1
            else:
              lane_dir = 0

      sx1 = cam_dst.x + (x1 / img_w) * cam_dst.width
      sy1 = cam_dst.y + (y1 / img_h) * cam_dst.height
      sx2 = cam_dst.x + (x2 / img_w) * cam_dst.width
      sy2 = cam_dst.y + (y2 / img_h) * cam_dst.height
      w = sx2 - sx1
      h = sy2 - sy1
      if w < 2.0 or h < 2.0:
        continue

      _, base_color = det_hud._det_label_and_color(cls, float(o.score))
      min_dim = float(min(w, h))
      thickness = int(max(1.0, min(4.0, min_dim / 90.0)))
      fade = float(max(0.35, 1.0 - 0.14 * int(o.missed)))
      color = rl.Color(base_color.r, base_color.g, base_color.b, int(base_color.a * fade))
      box = rl.Rectangle(float(sx1), float(sy1), float(w), float(h))

      if cls in det_hud.DET_VEHICLE_CLASS_IDS:
        det_hud.AugmentedRoadView._draw_det_corner_box(box=box, thickness=float(thickness), color=color)
        if lane_dir in (-1, 1):
          det_hud.AugmentedRoadView._draw_det_lane_arrow(box=box, lane_dir=int(lane_dir), color=color)
      elif cls == 0:
        det_hud.AugmentedRoadView._draw_det_person_icon(box=box, thickness=float(thickness), color=color)
      elif cls == 80:
        det_hud.AugmentedRoadView._draw_det_cone_icon(box=box, thickness=float(thickness), color=color)
      else:
        rl.draw_rectangle_lines_ex(box, thickness, color)


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
