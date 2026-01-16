#!/usr/bin/env python3
import json
import math
import time
import numpy as np

from cereal import log
import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from dragonpilot.selfdrive.controls.lib.acm import ACM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_VALS_FORD = [1.2, 1.0, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
OBSTACLE_DET_STALE_TIMEOUT_S = 1.0
OBSTACLE_SLOW_SPEED_MPH = 12.0
OBSTACLE_STOP_DELAY_S = 3.0
OBSTACLE_APPROACH_SPEED_MPH = 35.0
OBSTACLE_METRIC_START = 0.15
OBSTACLE_METRIC_FULL = 0.75
OBSTACLE_V_CAP_DOWN_RATE = 8.0  # m/s^2 (rate-limit on reducing v_cap)
OBSTACLE_V_CAP_UP_RATE = 2.0    # m/s^2 (rate-limit on releasing v_cap)

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# Map Turn Speed Controller constants (FrogPilot-style)
_MAP_EARTH_RADIUS_M = 6373000.0
_MAP_TO_RADIANS = math.pi / 180.0

# FrogPilot-style curve speed control defaults
_FP_TARGET_LAT_A = 2.0
_FP_CURVE_SENSITIVITY = 1.0
_FP_TURN_AGGRESSIVENESS = 1.0
_FP_CRUISING_SPEED = 5.0
_FP_MTSCC_CURVATURE_CHECK = True

class DPFlags:
  ACM = 1
  AEM = 2
  pass

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_max_accel_for_car(v_ego, CP):
  if getattr(CP, "brand", "") == "ford":
    return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS_FORD)
  return get_max_accel(v_ego)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt, CP=CP)
    # TODO remove mpc modes when TR released
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.acm = ACM()
    self.aem = AEM()
    # Curve speed control (FrogPilot-style)
    self.curve_v_target = None
    self._curve_speed_source = 0  # 0:none, 1:vision, 2:map (published to longitudinalPlan for HUD)
    self._fp_map_target = float("nan")
    self._fp_vision_target = float("nan")
    self._fp_road_curvature = 0.0
    self._fp_road_curvature_detected = False
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")
    self._fp_curve_cfg = {}
    self._fp_curve_param_last = 0.0

    self._map_target_velocities_raw = None
    self._map_target_velocities = []
    self._map_v_target = 0.0
    self._map_a_target = 0.0
    self._map_turn_limit_active = False
    self._map_data_available = False
    self._map_points = 0
    self._map_min_dist_m = 0.0
    self._obstacle_last_det_t = 0.0
    self._obstacle_cone_in_path = False
    self._obstacle_person_in_path = False
    self._obstacle_vehicle_in_path = False
    self._obstacle_metric = 0.0
    self._haz_metric = 0.0
    self._obstacle_present_prev = False
    self._obstacle_present_since: float | None = None
    self._obstacle_v_cap = float("nan")

  def _fp_curve_config(self):
    now = time.monotonic()
    if now - self._fp_curve_param_last < 1.0 and self._fp_curve_cfg:
      return self._fp_curve_cfg

    def _safe_int(name: str, default: int) -> int:
      try:
        val = self._params.get(name)
        return int(val) if val is not None else default
      except Exception:
        return default

    def _safe_bool(name: str, default: bool) -> bool:
      try:
        val = self._params.get_bool(name)
        return bool(val)
      except Exception:
        return default

    curve_sensitivity = _safe_int("CurveSensitivity", int(_FP_CURVE_SENSITIVITY * 100)) / 100.0
    if not math.isfinite(curve_sensitivity) or curve_sensitivity <= 0.0:
      curve_sensitivity = float(_FP_CURVE_SENSITIVITY)

    turn_aggressiveness = _safe_int("TurnAggressiveness", int(_FP_TURN_AGGRESSIVENESS * 100)) / 100.0
    if not math.isfinite(turn_aggressiveness) or turn_aggressiveness <= 0.0:
      turn_aggressiveness = float(_FP_TURN_AGGRESSIVENESS)

    map_enabled = _safe_bool("MapTurnControl", True)
    vision_enabled = _safe_bool("VisionTurnControl", True)
    mtsc_curvature_check = _safe_bool("MTSCCurvatureCheck", True)

    self._fp_curve_cfg = {
      "curve_sensitivity": float(curve_sensitivity),
      "turn_aggressiveness": float(turn_aggressiveness),
      "map_enabled": bool(map_enabled),
      "vision_enabled": bool(vision_enabled),
      "mtsc_curvature_check": bool(mtsc_curvature_check),
    }
    self._fp_curve_param_last = now
    return self._fp_curve_cfg

  def _map_distance_to_point(self, lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    ax = lat_a * _MAP_TO_RADIANS
    ay = lon_a * _MAP_TO_RADIANS
    bx = lat_b * _MAP_TO_RADIANS
    by = lon_b * _MAP_TO_RADIANS
    a = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1 - a)))
    return _MAP_EARTH_RADIUS_M * c

  def _map_target_velocities_list(self) -> list:
    raw = None
    try:
      raw = self._params_memory.get("MapTargetVelocities")
    except Exception:
      raw = None

    if not raw:
      self._map_target_velocities_raw = None
      self._map_target_velocities = []
      return []

    if raw == self._map_target_velocities_raw:
      return self._map_target_velocities

    try:
      parsed = json.loads(raw)
      if not isinstance(parsed, list):
        parsed = []
    except Exception:
      parsed = []

    self._map_target_velocities_raw = raw
    self._map_target_velocities = parsed
    return parsed

  @staticmethod
  def _fp_parse_map_point(target_velocity) -> tuple[float, float] | None:
    try:
      tlat = float(target_velocity.get("latitude", target_velocity.get("lat", 0.0)))
      tlon = float(target_velocity.get("longitude", target_velocity.get("lon", target_velocity.get("lng", 0.0))))
    except Exception:
      return None
    if not (math.isfinite(tlat) and math.isfinite(tlon)):
      return None
    return tlat, tlon

  @staticmethod
  def _fp_calc_road_curvature(model_msg, v_ego: float) -> float:
    try:
      orientation_rate = np.array(model_msg.orientationRate.z)
      velocity = np.array(model_msg.velocity.x)
    except Exception:
      return 0.0
    if orientation_rate.size == 0 or velocity.size == 0:
      return 0.0
    try:
      max_pred_lat_acc = max(np.max(orientation_rate * velocity), np.min(orientation_rate * velocity), key=abs)
    except Exception:
      return 0.0
    return float(max_pred_lat_acc / max(v_ego, 1.0) ** 2)

  def _fp_calc_curvature(self, p1: tuple[float, float], p2: tuple[float, float], p3: tuple[float, float]) -> float:
    side_a = self._map_distance_to_point(p2[0], p2[1], p3[0], p3[1])
    side_b = self._map_distance_to_point(p1[0], p1[1], p3[0], p3[1])
    side_c = self._map_distance_to_point(p1[0], p1[1], p2[0], p2[1])

    s = (side_a + side_b + side_c) / 2.0
    area_squared = s * (s - side_a) * (s - side_b) * (s - side_c)
    if area_squared <= 0.0:
      return 0.0
    area = math.sqrt(area_squared)
    if area <= 0.0:
      return 0.0

    radius = (side_a * side_b * side_c) / (4.0 * area)
    if radius <= 0.0:
      return 0.0
    return 1.0 / radius

  def _fp_map_curvature(self, lat: float, lon: float, v_ego: float, target_velocities: list) -> float:
    if not target_velocities:
      self._map_min_dist_m = 0.0
      return 1e-6

    points: list[tuple[float, float]] = []
    distances: list[float] = []
    min_idx = 0
    min_dist = 1e9
    for target_velocity in target_velocities:
      point = self._fp_parse_map_point(target_velocity)
      if point is None:
        continue
      tlat, tlon = point
      points.append((tlat, tlon))
      d = self._map_distance_to_point(lat, lon, tlat, tlon)
      distances.append(d)
      if d < min_dist:
        min_dist = d
        min_idx = len(points) - 1

    self._map_min_dist_m = float(min_dist) if math.isfinite(min_dist) else 0.0
    if len(points) < 3:
      return 1e-6

    forward_distances = distances[min_idx:]
    forward_points = points[min_idx:]
    if not forward_distances or len(forward_points) < 3:
      return 1e-6

    lookahead_dist = float(ModelConstants.T_IDXS[-1]) * float(v_ego)
    cumulative_distance = 0.0
    target_idx = None
    for i, distance in enumerate(forward_distances):
      cumulative_distance += float(distance)
      if cumulative_distance >= lookahead_dist:
        target_idx = i
        break

    if target_idx is None or target_idx == 0 or target_idx >= len(forward_points) - 1:
      return 1e-6

    p1 = forward_points[target_idx - 1]
    p2 = forward_points[target_idx]
    p3 = forward_points[target_idx + 1]
    return max(self._fp_calc_curvature(p1, p2, p3), 1e-6)

  def update(self, sm, dp_flags = 0, curve_speed_control: bool = False):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if dp_flags & DPFlags.AEM:
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    now = time.monotonic()

    # Obstacle detections (from coned via customReservedRawData0).
    if sm.updated.get("customReservedRawData0", False):
      try:
        raw = sm["customReservedRawData0"]
        payload = decode_cone_detections(raw) if raw else None
        if payload is not None:
          self._obstacle_cone_in_path = bool(payload.get("inPath", False))
          self._obstacle_person_in_path = bool(payload.get("personInPath", False))
          self._obstacle_vehicle_in_path = bool(payload.get("vehicleInPath", False))
          self._obstacle_metric = float(payload.get("obstacleMetric", 0.0) or 0.0)
          self._haz_metric = float(payload.get("hazMetric", 0.0) or 0.0)
          self._obstacle_last_det_t = now
      except Exception:
        pass

    if (now - self._obstacle_last_det_t) > OBSTACLE_DET_STALE_TIMEOUT_S:
      self._obstacle_cone_in_path = False
      self._obstacle_person_in_path = False
      self._obstacle_vehicle_in_path = False
      self._obstacle_metric = 0.0
      self._haz_metric = 0.0

    obstacle_present = self._obstacle_cone_in_path or self._obstacle_person_in_path or self._obstacle_vehicle_in_path
    if obstacle_present and not self._obstacle_present_prev:
      self._obstacle_present_since = now
    if not obstacle_present:
      self._obstacle_present_since = None
    self._obstacle_present_prev = obstacle_present

    controls_enabled = bool(getattr(sm['controlsState'], "enabled", False))
    curve_speed_control = bool(curve_speed_control)

    v_cruise_cluster = float(v_cruise)
    try:
      v_cruise_cluster = float(max(float(sm['controlsState'].vCruiseCluster) * CV.KPH_TO_MS, v_cruise))
    except Exception:
      v_cruise_cluster = float(v_cruise)
    v_cruise_diff = float(v_cruise_cluster - v_cruise)

    gps_position = None
    gps_msg = None
    try:
      gps_ext = sm["gpsLocationExternal"] if sm.valid.get("gpsLocationExternal", False) else None
      gps_int = sm["gpsLocation"] if sm.valid.get("gpsLocation", False) else None
      if gps_ext is not None and getattr(gps_ext, "hasFix", False):
        gps_msg = gps_ext
      elif gps_int is not None and getattr(gps_int, "hasFix", False):
        gps_msg = gps_int
    except Exception:
      gps_msg = None

    if gps_msg is not None:
      try:
        lat = float(gps_msg.latitude)
        lon = float(gps_msg.longitude)
        bearing = float(getattr(gps_msg, "bearingDeg", 0.0))
        if math.isfinite(lat) and math.isfinite(lon):
          if not math.isfinite(bearing):
            bearing = 0.0
          gps_position = {
            "latitude": lat,
            "longitude": lon,
            "bearing": bearing,
          }
      except Exception:
        gps_position = None

    if gps_position is not None:
      try:
        self._params_memory.put("LastGPSPosition", json.dumps(gps_position))
      except Exception:
        pass
    else:
      try:
        self._params_memory.remove("LastGPSPosition")
      except Exception:
        pass

    curve_sensitivity = float(_FP_CURVE_SENSITIVITY)
    turn_aggressiveness = float(_FP_TURN_AGGRESSIVENESS)
    fp_map_enabled = False
    fp_vision_enabled = False
    fp_mtsc_check = bool(_FP_MTSCC_CURVATURE_CHECK)
    if curve_speed_control:
      fp_cfg = self._fp_curve_config()
      curve_sensitivity = float(fp_cfg.get("curve_sensitivity", curve_sensitivity))
      turn_aggressiveness = float(fp_cfg.get("turn_aggressiveness", turn_aggressiveness))
      fp_map_enabled = bool(fp_cfg.get("map_enabled", False))
      fp_vision_enabled = bool(fp_cfg.get("vision_enabled", False))
      fp_mtsc_check = bool(fp_cfg.get("mtsc_curvature_check", fp_mtsc_check))

    road_curvature = 0.0
    road_curvature_detected = False
    if curve_speed_control:
      road_curvature = self._fp_calc_road_curvature(sm['modelV2'], v_ego)
      self._fp_road_curvature = float(road_curvature)
      try:
        road_curvature_detected = (1.0 / abs(road_curvature)) ** 0.5 < v_ego > _FP_CRUISING_SPEED
      except Exception:
        road_curvature_detected = False
      road_curvature_detected = bool(road_curvature_detected and not (sm['carState'].leftBlinker or sm['carState'].rightBlinker))
      self._fp_road_curvature_detected = bool(road_curvature_detected)
    else:
      self._fp_road_curvature = 0.0
      self._fp_road_curvature_detected = False

    target_velocities = []
    map_data_available = False
    if curve_speed_control and fp_map_enabled:
      target_velocities = self._map_target_velocities_list()
      map_data_available = bool(target_velocities)
    self._map_data_available = bool(map_data_available)
    self._map_points = len(target_velocities) if map_data_available else 0

    map_curv = 1e-6
    if map_data_available and gps_position is not None:
      map_curv = self._fp_map_curvature(gps_position["latitude"], gps_position["longitude"], v_ego, target_velocities)
    else:
      self._map_min_dist_m = 0.0

    if not math.isfinite(float(self._fp_map_target)):
      self._fp_map_target = float(v_cruise)
    if not math.isfinite(float(self._fp_vision_target)):
      self._fp_vision_target = float(v_cruise)

    if curve_speed_control and fp_map_enabled and controls_enabled and v_ego > _FP_CRUISING_SPEED:
      mtsc_active = self._fp_map_target < v_cruise
      if road_curvature_detected and mtsc_active:
        self._fp_map_target = float(self._fp_map_target)
      elif not road_curvature_detected and fp_mtsc_check:
        self._fp_map_target = float(v_cruise)
      else:
        map_speed = math.sqrt((_FP_TARGET_LAT_A * turn_aggressiveness) / (map_curv * curve_sensitivity))
        self._fp_map_target = max(_FP_CRUISING_SPEED, float(map_speed))
    else:
      self._fp_map_target = float(v_cruise)

    if curve_speed_control and fp_vision_enabled and controls_enabled and road_curvature_detected and v_ego > _FP_CRUISING_SPEED:
      vtsc_speed = math.sqrt((_FP_TARGET_LAT_A * turn_aggressiveness) / (abs(road_curvature) * curve_sensitivity))
      self._fp_vision_target = max(_FP_CRUISING_SPEED, float(vtsc_speed))
    else:
      self._fp_vision_target = float(v_cruise)

    v_cruise_pre_curve = float(v_cruise)
    if curve_speed_control:
      targets = [self._fp_map_target, self._fp_vision_target, v_cruise_pre_curve]
      v_cruise = min([target if target > _FP_CRUISING_SPEED else v_cruise_pre_curve for target in targets])

    map_turn_limit_active = bool(curve_speed_control and self._fp_map_target > 0.1 and self._fp_map_target < (v_cruise_pre_curve - 1e-3))
    vision_turn_limit_active = bool(curve_speed_control and self._fp_vision_target > 0.1 and self._fp_vision_target < (v_cruise_pre_curve - 1e-3))

    self._fp_map_target = float(self._fp_map_target + v_cruise_diff)
    self._fp_vision_target = float(self._fp_vision_target + v_cruise_diff)

    self._map_v_target = float(self._fp_map_target)
    self._map_a_target = 0.0
    self._map_turn_limit_active = bool(map_turn_limit_active)
    self.curve_v_target = float(self._fp_vision_target) if vision_turn_limit_active else None

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if mode == 'acc':
      max_accel = float(get_max_accel_for_car(v_ego, self.CP))

      # Ford/Lincoln comfort: when following a lead at shorter headways, cap max accel to avoid abrupt tip-in
      # (often felt as high RPM / delayed upshifts after a lead slows down).
      try:
        lead_one = sm['radarState'].leadOne
        if getattr(self.CP, "brand", "") == "ford" and bool(lead_one.status) and v_ego > 3.0:
          d_rel = float(lead_one.dRel)
          if math.isfinite(d_rel) and d_rel > 0.0:
            headway_s = d_rel / max(v_ego, 0.1)
            follow_cap = float(np.interp(headway_s, [0.8, 1.2, 1.8, 2.6], [0.30, 0.45, 0.65, max_accel]))
            max_accel = float(min(max_accel, follow_cap))
      except Exception:
        pass

      accel_clip = [ACCEL_MIN, max_accel]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    dp_auto_avoid = self._params.get_bool("dp_lincoln_auto_avoid")

    # Auto obstacle slowdown (experimental):
    # - cones/vehicles: progressively cap cruise speed, then attempt an auto lane change (handled in modeld).
    # - pedestrians: stop (no auto lane change).
    if dp_auto_avoid and obstacle_present and getattr(sm['selfdriveState'], "enabled", False):
      accel_clip[1] = min(accel_clip[1], 0.0)  # prevent accelerating towards the obstacle

      v_cap_target = 0.0
      if not self._obstacle_person_in_path:
        bsm_available = bool(getattr(self.CP, "enableBsm", False))
        if bsm_available:
          m = float(max(0.0, min(1.0, self._obstacle_metric)))
          if m <= OBSTACLE_METRIC_START:
            v_cap_target = float(OBSTACLE_APPROACH_SPEED_MPH * CV.MPH_TO_MS)
          elif m >= OBSTACLE_METRIC_FULL:
            v_cap_target = float(OBSTACLE_SLOW_SPEED_MPH * CV.MPH_TO_MS)
          else:
            alpha = (m - OBSTACLE_METRIC_START) / max(1e-3, (OBSTACLE_METRIC_FULL - OBSTACLE_METRIC_START))
            v_cap_target_mph = (1.0 - alpha) * OBSTACLE_APPROACH_SPEED_MPH + alpha * OBSTACLE_SLOW_SPEED_MPH
            v_cap_target = float(v_cap_target_mph * CV.MPH_TO_MS)

          # If we failed to start a lane change for a while, stop as a last-resort to avoid collision.
          if self._obstacle_present_since is not None and (now - self._obstacle_present_since) >= OBSTACLE_STOP_DELAY_S:
            lane_change_state = int(getattr(sm['modelV2'].meta, "laneChangeState", log.LaneChangeState.off))
            if lane_change_state in (int(log.LaneChangeState.off), int(log.LaneChangeState.preLaneChange)):
              v_cap_target = 0.0

      # Rate limit to avoid step changes in cruise cap (tighten fast, release slower).
      if not math.isfinite(self._obstacle_v_cap):
        self._obstacle_v_cap = float(v_cruise)

      if v_cap_target <= 1e-3:
        self._obstacle_v_cap = 0.0
      else:
        down_step = float(OBSTACLE_V_CAP_DOWN_RATE * self.dt)
        up_step = float(OBSTACLE_V_CAP_UP_RATE * self.dt)
        if v_cap_target < self._obstacle_v_cap:
          self._obstacle_v_cap = max(float(v_cap_target), float(self._obstacle_v_cap - down_step))
        else:
          self._obstacle_v_cap = min(float(v_cap_target), float(self._obstacle_v_cap + up_step))

      v_cruise = min(v_cruise, float(self._obstacle_v_cap))
    else:
      # Reset limiter state when disabled or no obstacle (prevents stale cap when toggling quickly).
      self._obstacle_v_cap = float("nan")

    # HUD source label: report which limiter is actively tightening the cruise target.
    # NOTE: Don't gate on `long_control_off`/`longActive` since Ford often runs stock ACC; users still want to know
    # whether the limiting comes from map or vision.
    if map_turn_limit_active:
      self._curve_speed_source = 2  # map
    elif self.curve_v_target is not None:
      self._curve_speed_source = 1  # vision
    else:
      self._curve_speed_source = 0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    # ACM - Adaptive Coasting Module
    if dp_flags & DPFlags.ACM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    output_a_target_clipped = float(np.clip(output_a_target, accel_clip[0], accel_clip[1]))

    # Ford/Lincoln comfort: limit positive jerk to reduce abrupt tip-in (high RPM / delayed upshifts),
    # while keeping decel response unmodified for safety.
    if getattr(self.CP, "brand", "") == "ford" and (not reset_state) and (not long_control_off):
      if math.isfinite(self.output_a_target) and output_a_target_clipped > self.output_a_target:
        max_jerk_up = float(np.interp(v_ego, [0.0, 5.0, 15.0, 30.0], [2.0, 1.5, 1.0, 0.8]))
        max_step = max_jerk_up * float(self.dt)
        output_a_target_clipped = float(min(output_a_target_clipped, self.output_a_target + max_step))

    self.output_a_target = output_a_target_clipped
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.curveSpeedSource = int(getattr(self, "_curve_speed_source", 0))

    pm.send('longitudinalPlan', plan_send)
