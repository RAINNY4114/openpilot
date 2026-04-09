import math
from collections import deque

from cereal import log
import cereal.messaging as messaging
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
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

DEFAULT_CUSTOM_PATH_OFFSET = 0.0
DEFAULT_PC_BLEND_RATIO_LOW = 0.40
DEFAULT_PC_BLEND_RATIO_HIGH = 0.40
DEFAULT_LC_PID_GAIN = 3.0
DEFAULT_LANE_CHANGE_FACTOR_HIGH = 0.85
TUNING_EPS = 1e-6


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
    self.sm = messaging.SubMaster(["modelV2"])
    self.model = None

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.enable_human_turn_detection = True
    self.human_turn = False
    self.post_reset_ramp_active = False
    self.reset_steering_last = False

    # BP-derived Ford/Lincoln lateral tuning exposed in the Lincoln panel.
    self.custom_path_offset = DEFAULT_CUSTOM_PATH_OFFSET
    self.pc_blend_ratio_low = DEFAULT_PC_BLEND_RATIO_LOW
    self.pc_blend_ratio_high = DEFAULT_PC_BLEND_RATIO_HIGH
    self.lc_pid_gain = DEFAULT_LC_PID_GAIN
    self.lane_change_factor_high = DEFAULT_LANE_CHANGE_FACTOR_HIGH
    self._lincoln_curvature_blend_active = False
    self._lincoln_lane_positioning_active = False
    self._lincoln_lane_change_smoothing_active = False
    self.lane_change_factor_low = 0.95
    self.pc_blend_ratio_bp = [0.0, 0.001]
    self.lane_change_factor_bp = [4.4, 40.23]
    self.curvature_lookup_time = 0.2
    self.path_offset_lookup_time = 0.2
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.lc_pid_speed_bp = [0.0, 9.0, 15.0]
    self.lc_pid_speed_v = [0.0, 0.0, 1.0]
    self.lc_path_angle_roc_bp = [5, 15, 25]
    self.lc_path_angle_roc_v = [0.003, 0.0015, 0.002]
    self.lc_pid_controller = PIDController(k_p=0.25, k_i=0.05, rate=20)
    self.lc_path_angle_reset_counter = 0
    self.lc_path_angle_reset_duration = 1.5
    self.path_angle_last = 0.0
    self.path_angle_max = 0.5

    # Auto-turn-signal latch (for auto-requested lane changes)
    self._auto_blinker_dir = 0  # 0=off, 1=left, 2=right
    self._prev_cc_blinker_dir = 0

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

  def _get_float_param(self, key: str, default: float) -> float:
    raw = self.params.get(key)
    if not raw:
      return default
    try:
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")
      return float(raw)
    except (TypeError, ValueError):
      return default

  def _update_lincoln_lateral_tuning(self) -> None:
    self.custom_path_offset = float(np.clip(
      self._get_float_param("custom_path_offset", DEFAULT_CUSTOM_PATH_OFFSET), -0.5, 0.5,
    ))
    self.pc_blend_ratio_low = float(np.clip(
      self._get_float_param("pc_blend_ratio_low_C_UI", DEFAULT_PC_BLEND_RATIO_LOW), 0.0, 1.0,
    ))
    self.pc_blend_ratio_high = float(np.clip(
      self._get_float_param("pc_blend_ratio_high_C_UI", DEFAULT_PC_BLEND_RATIO_HIGH), 0.0, 1.0,
    ))
    self.lc_pid_gain = float(np.clip(
      self._get_float_param("LC_PID_gain_UI", DEFAULT_LC_PID_GAIN), 0.0, 5.0,
    ))
    self.lane_change_factor_high = float(np.clip(
      self._get_float_param("lane_change_factor_high", DEFAULT_LANE_CHANGE_FACTOR_HIGH), 0.5, 1.0,
    ))
    self._lincoln_curvature_blend_active = (
      abs(self.pc_blend_ratio_low - DEFAULT_PC_BLEND_RATIO_LOW) > TUNING_EPS or
      abs(self.pc_blend_ratio_high - DEFAULT_PC_BLEND_RATIO_HIGH) > TUNING_EPS
    )
    self._lincoln_lane_positioning_active = (
      abs(self.custom_path_offset - DEFAULT_CUSTOM_PATH_OFFSET) > TUNING_EPS or
      abs(self.lc_pid_gain - DEFAULT_LC_PID_GAIN) > TUNING_EPS
    )
    self._lincoln_lane_change_smoothing_active = (
      abs(self.lane_change_factor_high - DEFAULT_LANE_CHANGE_FACTOR_HIGH) > TUNING_EPS
    )

  def _lincoln_requested_curvature(self, desired_curvature: float, v_ego_raw: float) -> tuple[float, bool]:
    if self.model is None:
      return desired_curvature, False

    try:
      orientation_rate = list(getattr(self.model.orientationRate, "z", []))
      if len(orientation_rate) >= len(ModelConstants.T_IDXS):
        curvatures = np.array(orientation_rate) / max(0.01, v_ego_raw)
        predicted_curvature = float(np.interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures))
      else:
        predicted_curvature = 0.0
    except Exception:
      predicted_curvature = 0.0

    if self._lincoln_curvature_blend_active:
      blend_ratio = float(np.interp(abs(desired_curvature), self.pc_blend_ratio_bp,
                                    [self.pc_blend_ratio_low, self.pc_blend_ratio_high]))
      requested_curvature = (predicted_curvature * blend_ratio) + (desired_curvature * (1.0 - blend_ratio))
    else:
      requested_curvature = desired_curvature

    meta = getattr(self.model, "meta", None)
    lane_change_state = getattr(meta, "laneChangeState", log.LaneChangeState.off)
    lane_change_direction = getattr(meta, "laneChangeDirection", log.LaneChangeDirection.none)
    lane_change_active = lane_change_state in (
      log.LaneChangeState.preLaneChange,
      log.LaneChangeState.laneChangeStarting,
      log.LaneChangeState.laneChangeFinishing,
    )

    if lane_change_active and self._lincoln_lane_change_smoothing_active:
      lane_change_factor = float(np.interp(v_ego_raw, self.lane_change_factor_bp,
                                           [self.lane_change_factor_low, self.lane_change_factor_high]))
      if lane_change_direction == log.LaneChangeDirection.left and requested_curvature < 0.0:
        requested_curvature *= lane_change_factor
      elif lane_change_direction == log.LaneChangeDirection.right and requested_curvature > 0.0:
        requested_curvature *= lane_change_factor

    return requested_curvature, lane_change_active

  def _lincoln_path_angle_cmd(self, CS, lane_change_active: bool, reset_steering: bool) -> float:
    if self.model is None:
      return 0.0
    if not self._lincoln_lane_positioning_active:
      self.lc_pid_controller.reset()
      self.lc_path_angle_reset_counter = 0
      self.path_angle_last = 0.0
      return 0.0

    try:
      lane_lines = list(getattr(self.model, "laneLines", []))
      lane_line_probs = list(getattr(self.model, "laneLineProbs", []))
      position_y = list(getattr(getattr(self.model, "position", None), "y", []))
      if len(lane_lines) < 3 or len(lane_line_probs) < 3 or len(position_y) < len(ModelConstants.T_IDXS):
        return 0.0

      left_y = list(getattr(lane_lines[1], "y", []))
      right_y = list(getattr(lane_lines[2], "y", []))
      if len(left_y) == 0 or len(right_y) == 0:
        return 0.0

      path_offset_position = float(np.interp(self.path_offset_lookup_time, ModelConstants.T_IDXS, position_y))
      path_offset_lanelines = (left_y[0] + right_y[0]) / 2.0
      laneline_width = right_y[0] + (-left_y[0])
      laneline_width_tolerance = float(np.interp(laneline_width, [3.75, 4.25], [0.81, 0.59]))
      laneline_confidence = min(lane_line_probs[1], lane_line_probs[2], laneline_width_tolerance)
      laneline_path_offset_scale = float(np.interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0]))

      path_offset = (path_offset_position * (1.0 - laneline_path_offset_scale) +
                     (path_offset_lanelines * laneline_path_offset_scale)) + self.custom_path_offset
      if lane_change_active:
        path_offset = 0.0

      path_offset_error = path_offset * (self.lc_pid_gain / 100.0)
      lc_pid_speed_factor = float(np.interp(CS.out.vEgoRaw, self.lc_pid_speed_bp, self.lc_pid_speed_v))
      path_angle = float(self.lc_pid_controller.update(path_offset_error * lc_pid_speed_factor))

      if reset_steering:
        path_angle = 0.0

      path_angle_roc = float(np.interp(abs(CS.out.vEgoRaw), self.lc_path_angle_roc_bp, self.lc_path_angle_roc_v))
      path_angle = float(np.clip(path_angle, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc))

      if CS.out.steeringPressed:
        self.lc_path_angle_reset_counter += 1
      else:
        self.lc_path_angle_reset_counter = 0
      if self.lc_path_angle_reset_counter > self.lc_path_angle_reset_duration * 20:
        self.lc_pid_controller.reset()

      path_angle = float(np.clip(path_angle, -self.path_angle_max, self.path_angle_max))
      self.path_angle_last = path_angle
      return path_angle
    except Exception:
      return 0.0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    self.sm.update(0)
    if self.sm.updated["modelV2"]:
      self.model = self.sm["modelV2"]
    self._update_human_turn_detection_enabled()
    self._update_lincoln_lateral_tuning()

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
      path_angle = 0.0
      self.human_turn = self.enable_human_turn_detection and CS.out.steeringPressed and \
                        abs(CS.out.steeringAngleDeg) > HUMAN_TURN_STEERING_ANGLE_DEG
      reset_steering = self.human_turn or CS.out.vEgoRaw < 0.1
      lane_change_active = False

      # Bronco and some other cars consistently overshoot curv requests
      # Apply some deadzone + smoothing convergence to avoid oscillations
      if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14) and not reset_steering:
        desired_curvature = float(actuators.curvature)
        requested_curvature, lane_change_active = self._lincoln_requested_curvature(desired_curvature, CS.out.vEgoRaw)
        self.anti_overshoot_curvature_last = anti_overshoot(requested_curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
        requested_curvature = self.anti_overshoot_curvature_last
      else:
        if reset_steering:
          self.anti_overshoot_curvature_last = 0.0
        desired_curvature = float(actuators.curvature)
        requested_curvature, lane_change_active = self._lincoln_requested_curvature(desired_curvature, CS.out.vEgoRaw)

      # apply rate limits, curvature error limit, and clip to signal range
      if reset_steering:
        requested_curvature = 0.0
        self.apply_curvature_last = 0.0
        self.post_reset_ramp_active = False
        self.lc_pid_controller.reset()
        self.lc_path_angle_reset_counter = 0
        self.path_angle_last = 0.0
        path_angle = 0.0
      else:
        path_angle = self._lincoln_path_angle_cmd(CS, lane_change_active, reset_steering)
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        if self.reset_steering_last:
          self.post_reset_ramp_active = True
          self.apply_curvature_last = 0.0

        if self.post_reset_ramp_active:
          self.apply_curvature_last = apply_std_steer_angle_limits(
            requested_curvature,
            self.apply_curvature_last,
            CS.out.vEgoRaw,
            0.,
            CC.latActive,
            CarControllerParams.get_angle_limits(self.CP),
          )

          curvature_error = abs(requested_curvature - self.apply_curvature_last)
          curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)
          if curvature_error < curvature_threshold:
            self.post_reset_ramp_active = False
          path_angle = 0.0
        else:
          self.apply_curvature_last = apply_ford_curvature_limits(requested_curvature, self.apply_curvature_last, current_curvature,
                                                                  CS.out.vEgoRaw, 0., CC.latActive, self.CP)

      self.reset_steering_last = reset_steering

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02m/s^2)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., -path_angle, -self.apply_curvature_last, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., -path_angle, -self.apply_curvature_last, 0.))

      if not CC.latActive:
        self.lc_pid_controller.reset()
        self.lc_path_angle_reset_counter = 0
        self.path_angle_last = 0.0

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      # Release brake request as soon as we are no longer asking for decel.
      # This prevents lingering brake precharge/drag during mild accel.
      if accel_pitch_compensated >= 0.0 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

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
