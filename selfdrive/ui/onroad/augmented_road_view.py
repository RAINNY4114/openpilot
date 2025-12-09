import math
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
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

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
PERF_DIRECTION_LABELS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
PERF_FONT_SIZE = 32
PERF_PADDING = 12
PERF_MARGIN_BOTTOM = 24
PERF_ITEM_GAP = 140
PERF_BG_COLOR = rl.Color(0, 0, 0, 160)


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
    show = show_prev
    count = count_prev
    color = rl.Color(0, 0, 0, 0)

    if not blinker_state and not bsm_state:
      show = False
      count = 0
    else:
      count += 1

    if bsm_state and blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_FAST == 0 else show
      color = DP_INDICATOR_COLOR_BSM
    elif blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_STD == 0 else show
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

  def _draw_performance_info(self) -> None:
    if not ui_state.dp_lincoln_perf_info_enabled:
      return

    rect = self._content_rect
    if rect.width <= 0 or rect.height <= 0:
      return

    stats = self._get_perf_stats()
    curvature_text, steering_text, torque_text = self._get_curvature_steer_torque()
    direction_text = self._get_direction_label()
    control_text = self._get_control_state_text()
    mem_usage = stats.get("mem_usage", "N/A")
    cpu_temp = stats.get("cpu_temp", "N/A")

    items = [
      f"{tr('Curvature')} {curvature_text}/{steering_text}/{torque_text}",
      f"{tr('Direction')} {direction_text}",
      f"{tr('Control')} {control_text}",
      f"{tr('Memory')} {mem_usage}",
      f"{tr('CPU Temp')} {cpu_temp}",
    ]

    measurements = [measure_text_cached(self._perf_font, text, PERF_FONT_SIZE) for text in items]
    total_text_width = sum(size.x for size in measurements)
    gap_count = max(0, len(items) - 1)
    gap = float(PERF_ITEM_GAP if gap_count else 0)
    desired_width = total_text_width + 2 * PERF_PADDING + gap * gap_count
    max_width = max(rect.width - 40, 0)
    if gap_count > 0 and max_width > 0 and desired_width > max_width:
      gap = max(20.0, (max_width - 2 * PERF_PADDING - total_text_width) / gap_count)
      desired_width = total_text_width + 2 * PERF_PADDING + gap * gap_count
    bar_width = desired_width
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
    for text, measurement in zip(items, measurements):
      rl.draw_text_ex(self._perf_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, rl.WHITE)
      cursor_x += measurement.x + gap

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

  def _get_direction_label(self) -> str:
    sm = ui_state.sm
    try:
      if getattr(sm, "alive", {}).get("gpsLocationExternal", False):
        gps = sm['gpsLocationExternal']
        if getattr(gps, "hasFix", True):
          bearing = getattr(gps, "bearingDeg", float("nan"))
          if math.isfinite(bearing):
            index = int(((bearing % 360) + 22.5) // 45) % len(PERF_DIRECTION_LABELS)
            return tr(PERF_DIRECTION_LABELS[index])
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
