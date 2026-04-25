import math
from collections import deque
import cereal.messaging as messaging
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN FD limits:
# Limit to average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll raises lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2
HUMAN_TURN_STEERING_ANGLE_DEG = 45.0


def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active,
                                                  CarControllerParams.get_angle_limits(CP))

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))

  return apply_curvature


def apply_ford_curvature_limits_bp(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  max_curvature = 1.0

  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)
    max_curvature = abs(current_curvature) + CarControllerParams.CURVATURE_ERROR

  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active,
                                                 CarControllerParams.get_angle_limits(CP))

  angle_limits = CarControllerParams.get_angle_limits(CP)
  steer_up = apply_curvature_last * apply_curvature >= 0. and abs(apply_curvature) > abs(apply_curvature_last)
  rate_limits = angle_limits.ANGLE_RATE_LIMIT_UP if steer_up else angle_limits.ANGLE_RATE_LIMIT_DOWN
  std_steer_angle_rate_limit = float(np.interp(v_ego_raw, rate_limits[0], rate_limits[1]))
  std_steer_angle_limit = abs(apply_curvature_last) + abs(std_steer_angle_rate_limit)
  max_curvature = min(max_curvature, std_steer_angle_limit)

  if CP.flags & FordFlags.CANFD:
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))
    max_curvature = min(max_curvature, abs(curvature_accel_limit))

  return float(apply_curvature), float(max_curvature)


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = Params()
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.enable_human_turn_detection = True
    self.human_turn = False
    self.post_reset_ramp_active = False
    self.reset_steering_last = False

    # Auto-turn-signal latch (for auto-requested lane changes)
    self._auto_blinker_dir = 0  # 0=off, 1=left, 2=right
    self._prev_cc_blinker_dir = 0

    # BluePilot longitudinal defaults
    self.MAX_URBAN_SPEED_MPH = 45.0
    self.following_accel_ROC = 0.002
    self.brake_actuate_target = -0.14
    self.brake_actuate_release = -0.06
    self.precharge_actuate_target = -0.12
    self.precharge_actuate_release = -0.06
    self.op_brake_actuate_last = False
    self.bpSpeedAllow = False
    self.bp_gas_last = 0.0
    self.bp_accel_last = 0.0
    self.disable_BP_long_UI = False
    self.disable_downhill_comp_UI = False
    self.sm = messaging.SubMaster(['modelV2', 'radarState'])

    # BluePilot lateral defaults
    self.precision_type = 1
    self.disable_BP_lat_UI = False
    self.enable_lane_positioning = False
    self.enable_lane_full_mode = False
    self.custom_profile = 0
    self.lane_change_factor_bp = [4.4, 40.23]
    self.lane_change_factor_low = 0.95
    self.lane_change_factor_high = 0.85
    self.pc_blend_ratio_low = 0.40
    self.pc_blend_ratio_high = 0.40
    self.pc_blend_ratio_low_C_UI = 0.40
    self.pc_blend_ratio_high_C_UI = 0.40
    self.pc_blend_ratio_bp = [0.0, 0.001]
    self.curvature_lookup_time = 0.2
    self.curvature_rate_delta_t = 0.3
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / 0.05)))
    self.curvature_rate_speed_bp = [0.0, 14.5, 15.5]
    self.curvature_rate_speed_v = [1.0, 1.0, 0.0]
    self.curvature_rate_PC_bp = [0.0, 0.008, 0.01]
    self.curvature_rate_PC_v = [0.0, 0.0, 1.0]
    self.large_curve_factor_bp = [0.001, 0.02]
    self.large_curve_factor_v = [1.0, 0.80]
    self.custom_path_offset = 0.0
    self.path_offset_lookup_time = 0.2
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.LC_PID_gain_UI = 3.0
    self.LC_PID_controller = PIDController(k_p=0.25, k_i=0.05, rate=20)
    self.LC_PID_speed_bp = [0.0, 9.0, 15.0]
    self.LC_PID_speed_v = [0.0, 0.0, 1.0]
    self.LC_path_angle_ROC_bp = [5, 15, 25]
    self.LC_path_angle_ROC_v = [0.003, 0.0015, 0.002]
    self.LC_path_angle_reset_counter = 0
    self.LC_path_angle_reset_duration = 1.5
    self.path_angle_max = 0.5
    self.path_offset_max = 2.0
    self.curvature_max = 0.02
    self.curvature_rate_max = 0.001023
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.path_angle_deque = deque(maxlen=3)
    self.lane_change = False
    self.lane_change_last = False
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.pre_lane_change_values = {
      'path_angle': 0.0,
      'path_offset': 0.0,
      'desired_curvature_rate': 0.0,
    }
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125
    self.max_curvature_rate_change = 0.0001
    self.model = None
    self.VM = VehicleModel(self.CP)
    self._bp_lat_error_logged = False

  def _update_human_turn_detection_enabled(self) -> None:
    enabled = self.params.get("enable_human_turn_detection")
    if enabled is not None:
      self.enable_human_turn_detection = enabled != b"0"
      return

    legacy_enabled = self.params.get("dp_htd_enabled")
    if legacy_enabled is not None:
      self.enable_human_turn_detection = legacy_enabled != b"0"
      self.params.put_bool("enable_human_turn_detection", self.enable_human_turn_detection)
      return

    self.enable_human_turn_detection = True

  def _update_bp_long_params(self) -> None:
    self.disable_BP_long_UI = self.params.get_bool("disable_BP_long_UI")
    self.disable_downhill_comp_UI = self.params.get_bool("disable_downhill_comp_UI")

  def _safe_float_param(self, key: str, default: float) -> float:
    try:
      raw = self.params.get(key)
      if raw is None:
        return default
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")
      return float(raw)
    except Exception:
      return default

  def _safe_int_param(self, key: str, default: int) -> int:
    try:
      raw = self.params.get(key)
      if raw is None:
        return default
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")
      return int(raw)
    except Exception:
      return default

  def _update_bp_lat_params(self) -> None:
    self.disable_BP_lat_UI = self.params.get_bool("disable_BP_lat_UI")
    self.pc_blend_ratio_high_C_UI = self._safe_float_param("pc_blend_ratio_high_C_UI", 0.4)
    self.pc_blend_ratio_low_C_UI = self._safe_float_param("pc_blend_ratio_low_C_UI", 0.4)
    self.enable_lane_positioning = self.params.get_bool("enable_lane_positioning")
    self.custom_path_offset = self._safe_float_param("custom_path_offset", 0.0)
    self.enable_lane_full_mode = self.params.get_bool("enable_lane_full_mode")
    self.custom_profile = self._safe_int_param("custom_profile", 0)
    self.LC_PID_gain_UI = self._safe_float_param("LC_PID_gain_UI", 3.0)

  def handle_post_lane_change_transition(self, path_angle, path_offset, desired_curvature_rate):
    if self.lane_change_last and not self.lane_change:
      self.post_lane_change_active = True
      self.post_lane_change_timer = 0
      self.pre_lane_change_values = {
        'path_angle': 0.0,
        'path_offset': 0.0,
        'desired_curvature_rate': 0.0,
      }

    self.lane_change_last = self.lane_change

    if self.post_lane_change_active:
      self.post_lane_change_timer += 1

      new_path_angle = np.clip(
        path_angle,
        self.pre_lane_change_values['path_angle'] - self.max_path_angle_change,
        self.pre_lane_change_values['path_angle'] + self.max_path_angle_change,
      )
      new_path_offset = np.clip(
        path_offset,
        self.pre_lane_change_values['path_offset'] - self.max_path_offset_change,
        self.pre_lane_change_values['path_offset'] + self.max_path_offset_change,
      )
      new_curvature_rate = np.clip(
        desired_curvature_rate,
        self.pre_lane_change_values['desired_curvature_rate'] - self.max_curvature_rate_change,
        self.pre_lane_change_values['desired_curvature_rate'] + self.max_curvature_rate_change,
      )

      self.pre_lane_change_values = {
        'path_angle': float(new_path_angle),
        'path_offset': float(new_path_offset),
        'desired_curvature_rate': float(new_curvature_rate),
      }

      if self.post_lane_change_timer >= 160:
        self.post_lane_change_active = False

      return float(new_path_angle), float(new_path_offset), float(new_curvature_rate)

    return float(path_angle), float(path_offset), float(desired_curvature_rate)

  def calculate_lateral_uncertainty(self, requested_curvature, apply_curvature, max_curvature):
    max_curvature = np.clip(max_curvature, apply_curvature, self.curvature_max)
    if abs(max_curvature) < 1e-6:
      return 0.0
    return float(requested_curvature / max_curvature)

  def _bp_model_ready(self) -> bool:
    try:
      model = self.model
      return model is not None and \
        len(model.orientationRate.z) >= 17 and \
        len(model.position.y) >= 5 and \
        len(model.laneLines) >= 3 and \
        len(model.laneLines[1].y) > 0 and \
        len(model.laneLines[2].y) > 0 and \
        len(model.laneLineProbs) >= 3
    except Exception:
      return False

  def _run_stock_lateral(self, CC, CS, actuators):
    can_sends = []
    self.human_turn = self.enable_human_turn_detection and CS.out.steeringPressed and \
                      abs(CS.out.steeringAngleDeg) > HUMAN_TURN_STEERING_ANGLE_DEG
    reset_steering = self.human_turn or CS.out.vEgoRaw < 0.1

    if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14) and not reset_steering:
      self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
      requested_curvature = self.anti_overshoot_curvature_last
    else:
      if reset_steering:
        self.anti_overshoot_curvature_last = 0.0
      requested_curvature = actuators.curvature

    if reset_steering:
      requested_curvature = 0.0
      self.apply_curvature_last = 0.0
      self.post_reset_ramp_active = False
    else:
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
      if self.reset_steering_last:
        self.post_reset_ramp_active = True
        self.apply_curvature_last = 0.0

      if self.post_reset_ramp_active:
        self.apply_curvature_last = apply_std_steer_angle_limits(
          requested_curvature,
          self.apply_curvature_last,
          CS.out.vEgoRaw,
          0.0,
          CC.latActive,
          CarControllerParams.get_angle_limits(self.CP),
        )

        curvature_error = abs(requested_curvature - self.apply_curvature_last)
        curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)
        if curvature_error < curvature_threshold:
          self.post_reset_ramp_active = False
      else:
        self.apply_curvature_last = apply_ford_curvature_limits(
          requested_curvature, self.apply_curvature_last, current_curvature,
          CS.out.vEgoRaw, 0.0, CC.latActive, self.CP,
        )

    self.reset_steering_last = reset_steering

    if self.CP.flags & FordFlags.CANFD:
      mode = 1 if CC.latActive else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0, 0, 0.0, 0.0,
                                                   -self.apply_curvature_last, 0.0, counter))
    else:
      can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0, 0, 0.0, 0.0,
                                                  -self.apply_curvature_last, 0.0))

    self.curvature_rate_deque.clear()
    self.path_angle_deque.clear()
    self.LC_PID_controller.reset()
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate_last = 0.0
    return can_sends

  def _run_bp_lateral(self, CC, CS, actuators):
    can_sends = []
    self.precision_type = 1
    steering_pressed = CS.out.steeringPressed
    steering_angle_deg = CS.out.steeringAngleDeg

    if self.custom_profile == 1:
      pc_blend_ratio_v = [self.pc_blend_ratio_low_C_UI, self.pc_blend_ratio_high_C_UI]
    else:
      pc_blend_ratio_v = [self.pc_blend_ratio_low, self.pc_blend_ratio_high]

    current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
    desired_curvature = actuators.curvature

    curvatures = np.array(self.model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
    model_t = ModelConstants.T_IDXS[:len(curvatures)]
    predicted_curvature = float(np.interp(self.curvature_lookup_time, model_t, curvatures))

    pc_blend_ratio = float(np.interp(abs(desired_curvature), self.pc_blend_ratio_bp, pc_blend_ratio_v))
    requested_curvature = (predicted_curvature * pc_blend_ratio) + (desired_curvature * (1.0 - pc_blend_ratio))

    lane_change_state = int(self.model.meta.laneChangeState)
    lane_change_direction = int(self.model.meta.laneChangeDirection)
    self.lane_change = lane_change_state in (1, 2, 3)
    lane_change_factor = float(np.interp(CS.out.vEgoRaw, self.lane_change_factor_bp,
                                         [self.lane_change_factor_low, self.lane_change_factor_high]))

    if self.lane_change and lane_change_direction == 1 and requested_curvature < 0.0:
      requested_curvature *= lane_change_factor
      self.precision_type = 0
    elif self.lane_change and lane_change_direction == 2 and requested_curvature > 0.0:
      requested_curvature *= lane_change_factor
      self.precision_type = 0

    self.human_turn = steering_pressed and abs(steering_angle_deg) > HUMAN_TURN_STEERING_ANGLE_DEG
    reset_steering = ((self.human_turn and self.enable_human_turn_detection) or (CS.out.vEgoRaw < 0.1))

    if reset_steering:
      requested_curvature = 0.0

    apply_curvature, max_curvature = apply_ford_curvature_limits_bp(
      requested_curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw, 0.0, CC.latActive, self.CP,
    )

    if reset_steering:
      apply_curvature = 0.0
      self.post_reset_ramp_active = False
    else:
      if self.reset_steering_last and not reset_steering:
        self.post_reset_ramp_active = True
        self.apply_curvature_last = 0.0

    if self.post_reset_ramp_active:
      apply_curvature = apply_std_steer_angle_limits(
        requested_curvature, self.apply_curvature_last, CS.out.vEgoRaw, 0.0, CC.latActive,
        CarControllerParams.get_angle_limits(self.CP),
      )
      curvature_error = abs(requested_curvature - apply_curvature)
      curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)
      if curvature_error < curvature_threshold:
        self.post_reset_ramp_active = False

    self.reset_steering_last = bool(reset_steering)

    self.curvature_rate_deque.append(predicted_curvature)
    if len(self.curvature_rate_deque) > 1:
      if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen:
        delta_t = self.curvature_rate_delta_t
      else:
        delta_t = (len(self.curvature_rate_deque) - 1) * 0.05
      desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
    else:
      desired_curvature_rate = 0.0

    desired_curvature_rate *= float(np.interp(abs(predicted_curvature), self.curvature_rate_PC_bp, self.curvature_rate_PC_v))
    desired_curvature_rate *= float(np.interp(CS.out.vEgoRaw, self.curvature_rate_speed_bp, self.curvature_rate_speed_v))
    desired_curvature_rate *= float(np.interp(abs(requested_curvature), self.large_curve_factor_bp, self.large_curve_factor_v))
    if self.lane_change:
      desired_curvature_rate = 0.0

    position_t = ModelConstants.T_IDXS[:len(self.model.position.y)]
    path_offset_position = float(np.interp(self.path_offset_lookup_time, position_t, self.model.position.y))
    path_offset_lanelines = float((self.model.laneLines[1].y[0] + self.model.laneLines[2].y[0]) / 2.0)
    laneline_width = float(self.model.laneLines[2].y[0] + (-self.model.laneLines[1].y[0]))
    laneline_width_tolerance = float(np.interp(laneline_width, [3.75, 4.25], [0.81, 0.59]))

    laneline_confidence = min(float(self.model.laneLineProbs[1]), float(self.model.laneLineProbs[2]), laneline_width_tolerance)
    if not self.enable_lane_full_mode:
      laneline_confidence = 0.0

    laneline_path_offset_scale = float(np.interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0]))
    path_offset = (path_offset_position * (1.0 - laneline_path_offset_scale) +
                   path_offset_lanelines * laneline_path_offset_scale) + self.custom_path_offset
    if self.lane_change:
      path_offset = 0.0

    path_offset_error = path_offset * (self.LC_PID_gain_UI / 100.0)
    path_offset_error_adj = path_offset_error * float(np.interp(CS.out.vEgoRaw, self.LC_PID_speed_bp, self.LC_PID_speed_v))
    if not self.enable_lane_positioning:
      path_offset_error_adj = 0.0

    path_angle = float(self.LC_PID_controller.update(path_offset_error_adj))
    if not self.enable_lane_positioning or reset_steering:
      path_angle = 0.0

    path_angle_roc = float(np.interp(abs(CS.out.vEgoRaw), self.LC_path_angle_ROC_bp, self.LC_path_angle_ROC_v))
    path_angle = float(np.clip(path_angle, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc))

    if steering_pressed:
      self.LC_path_angle_reset_counter += 1
    else:
      self.LC_path_angle_reset_counter = 0
    if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20:
      self.LC_PID_controller.reset()

    path_angle, path_offset, desired_curvature_rate = self.handle_post_lane_change_transition(
      path_angle, path_offset, desired_curvature_rate,
    )

    if reset_steering:
      path_angle = 0.0

    apply_curvature = float(np.clip(apply_curvature, -self.curvature_max, self.curvature_max))
    desired_curvature_rate = float(np.clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max))
    path_offset = float(np.clip(path_offset, -self.path_offset_max, self.path_offset_max))
    path_angle = float(np.clip(path_angle, -self.path_angle_max, self.path_angle_max))

    path_offset = 0.0
    if reset_steering:
      ramp_type = 3
      self.path_angle_deque.clear()
      self.LC_PID_controller.reset()
    else:
      ramp_type = 2

    self.apply_curvature_last = apply_curvature
    self.curvature_rate_last = desired_curvature_rate
    self.path_offset_last = path_offset
    self.path_angle_last = path_angle

    if self.CP.flags & FordFlags.CANFD:
      mode = 1 if CC.latActive else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, ramp_type, self.precision_type,
                                                   -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate, counter))
    else:
      can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, ramp_type, self.precision_type,
                                                  -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate))

    return can_sends

  def update(self, CC, CS, now_nanos):
    can_sends = []
    self.sm.update(0)
    self.model = self.sm['modelV2'] if self.sm.valid.get('modelV2', False) else None
    self._update_human_turn_detection_enabled()
    self._update_bp_long_params()
    self._update_bp_lat_params()

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # Latch whether this lane-change blinker request is "auto" (driver stalk was not active at start).
    cc_blinker_dir = 0
    if CC.leftBlinker and not CC.rightBlinker:
      cc_blinker_dir = 1
    elif CC.rightBlinker and not CC.leftBlinker:
      cc_blinker_dir = 2

    if cc_blinker_dir != self._prev_cc_blinker_dir:
      if cc_blinker_dir == 0:
        self._auto_blinker_dir = 0
      elif self._prev_cc_blinker_dir == 0:
        driver_signaling = bool(CS.out.leftBlinker or CS.out.rightBlinker)
        self._auto_blinker_dir = 0 if driver_signaling else cc_blinker_dir
    self._prev_cc_blinker_dir = cc_blinker_dir

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    # Auto blinker (exterior) during auto-requested lane changes.
    if self._auto_blinker_dir != 0 and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_turn_signal_msg(self.packer, self.CAN.main, CS.buttons_stock_values, self._auto_blinker_dir))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      use_bp_lateral = CC.latActive and not self.disable_BP_lat_UI and self._bp_model_ready()
      if use_bp_lateral:
        try:
          can_sends.extend(self._run_bp_lateral(CC, CS, actuators))
        except Exception:
          if not self._bp_lat_error_logged:
            cloudlog.exception("BP lateral fallback to stock")
            self._bp_lat_error_logged = True
          can_sends.extend(self._run_stock_lateral(CC, CS, actuators))
      else:
        can_sends.extend(self._run_stock_lateral(CC, CS, actuators))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      v_ego_mph = CS.out.vEgo * 2.23694
      op_accel = actuators.accel
      op_gas = op_accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        op_accel = apply_creep_compensation(op_accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        op_accel = max(op_accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      op_accel = float(np.clip(op_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      op_gas = float(np.clip(op_gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or op_gas < CarControllerParams.MIN_GAS:
        op_gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      if self.disable_downhill_comp_UI and accel_due_to_pitch < 0.0:
        accel_due_to_pitch = 0.0

      accel_pitch_compensated = op_accel + accel_due_to_pitch
      op_brake_actuate = self.op_brake_actuate_last
      if accel_pitch_compensated > self.brake_actuate_release or not CC.longActive:
        op_brake_actuate = False
      elif accel_pitch_compensated < self.brake_actuate_target:
        op_brake_actuate = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      target_speed = V_CRUISE_MAX

      bp_speed_too_slow = v_ego_mph < self.MAX_URBAN_SPEED_MPH
      bp_speed_high_enough = v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5
      if bp_speed_high_enough:
        self.bpSpeedAllow = True
      if bp_speed_too_slow:
        self.bpSpeedAllow = False

      lead = None
      v_rel = 0.0
      v_lead = 0.0
      lead_time_sec = 999.0
      if self.sm.valid.get('radarState', False):
        rs = self.sm['radarState']
        lead = getattr(rs, 'leadOne', None)
        if lead is not None and getattr(lead, 'status', 0) != 1:
          lead = None
        if lead:
          d_rel = float(getattr(lead, 'dRel', 0.0))
          v_rel = float(getattr(lead, 'vRel', 0.0))
          v_lead = float(getattr(lead, 'vLead', 0.0))
          if d_rel > 0.0:
            lead_time_sec = d_rel / max(CS.out.vEgo, 0.5)
      lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))
      v_lead_mph = v_lead * 2.23694

      ttc_sec = 120.0
      if lead:
        d_rel = float(getattr(lead, 'dRel', 0.0))
        if d_rel > 0.0 and v_rel < 0.0:
          ttc_sec = d_rel / (-v_rel)
        else:
          ttc_sec = 60.0
      ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

      max_follow_gas = op_gas
      min_follow_gas = op_gas
      max_follow_accel = op_accel
      min_follow_accel = op_accel
      bp_brake_actuate = False
      bp_precharge_actuate = False

      gaining = lead is not None and v_rel < -0.1
      pacing = lead is not None and abs(v_rel) <= 0.1
      trailing = lead is not None and v_rel > 0.1

      if gaining:
        if lead_time_sec < 1.5:
          max_follow_gas = 0.0
          min_follow_gas = 0.0
      elif pacing:
        max_follow_gas = 0.2 + accel_due_to_pitch
        min_follow_gas = 0.0
      elif lead is None:
        max_follow_accel = 0.0
        min_follow_accel = 0.0

      bp_gas = float(np.clip(op_gas, min_follow_gas, max_follow_gas))
      bp_accel = float(np.clip(op_accel, min_follow_accel, max_follow_accel))

      if ttc_sec > 8.0 and lead_time_sec > 0.5:
        bp_accel = float(np.clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999.0))

      if bp_accel < self.brake_actuate_target:
        bp_brake_actuate = True
      if bp_accel > self.brake_actuate_release:
        bp_brake_actuate = False
      if bp_accel < self.precharge_actuate_target:
        bp_precharge_actuate = True
      if bp_accel > self.precharge_actuate_release:
        bp_precharge_actuate = False

      apply_bp_long = (not self.disable_BP_long_UI) and self.bpSpeedAllow and \
                      (not CS.out.gasPressed) and (not CS.out.brakePressed) and \
                      (lead is None or v_lead_mph > 40.0)

      if apply_bp_long and CC.longActive:
        accel = bp_accel
        gas = bp_gas
        brake_actuate = bp_brake_actuate
        precharge_actuate = bp_precharge_actuate
      else:
        accel = op_accel
        gas = op_gas
        brake_actuate = op_brake_actuate
        precharge_actuate = op_brake_actuate

      self.bp_gas_last = bp_gas
      self.bp_accel_last = bp_accel

      if brake_actuate:
        gas = CarControllerParams.INACTIVE_GAS

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if gas != CarControllerParams.INACTIVE_GAS:
        gas = float(np.clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
      accel_pred_send = CarControllerParams.INACTIVE_GAS

      can_sends.append(fordcan.create_acc_msg(
        self.packer, self.CAN, CC.longActive, gas, accel, accel_pred_send, stopping,
        brake_actuate, precharge_actuate, v_ego_kph=target_speed
      ))

      self.accel = accel
      self.gas = gas
      self.op_brake_actuate_last = op_brake_actuate

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
