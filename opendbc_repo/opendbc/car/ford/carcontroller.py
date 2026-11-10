import math
import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX

try:
  from openpilot.cereal import messaging as _cereal_messaging
  from openpilot.selfdrive.modeld.constants import ModelConstants
  _BP_MODEL_AVAILABLE = True
except Exception:
  _cereal_messaging = None
  ModelConstants = None
  _BP_MODEL_AVAILABLE = False

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

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


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


_PARAM_DIR = "/data/ford_params"
_SHADOW_LOG = "/data/media/0/path_angle_shadow.log"
_SHADOW_LOG_MAX_BYTES = 2_000_000

_DT_MDL = 0.05
_VLT_T_EXTRA_MAX = 0.05
_VLT_V_LOW_MS = 25.0 * 0.44704
_VLT_V_HIGH_MS = 55.0 * 0.44704
_VLT_KAPPA_FULL = 0.005
_VLT_KAPPA_TAPER = 0.020
_EXIT_BLEND_SCALE = 0.25
_EXIT_FALLING_DK = 0.010

_HUMAN_TURN_ANGLE_DEG = 45.0
_HUMAN_TURN_HOLD_S = 1.5
_HUMAN_TURN_HOLD_PRETURNED_S = 3.0

class HumanTurnDetector:

  def __init__(self):
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False

  def update(self, enabled: bool, steering_pressed: bool, steering_angle_deg: float) -> bool:
    self._active_last = self.active
    if steering_pressed and not self._pressed_last:
      self._press_started_preturned = abs(steering_angle_deg) > _HUMAN_TURN_ANGLE_DEG
    self._pressed_last = steering_pressed
    if not enabled:
      self.hold_timer_s = 0.0
    elif steering_pressed and abs(steering_angle_deg) > _HUMAN_TURN_ANGLE_DEG:
      self.hold_timer_s += DT_CTRL * CarControllerParams.STEER_STEP
    else:
      self.hold_timer_s = 0.0
    hold_req = _HUMAN_TURN_HOLD_PRETURNED_S if self._press_started_preturned else _HUMAN_TURN_HOLD_S
    self.active = self.hold_timer_s >= hold_req
    return self.active

  @property
  def just_released(self) -> bool:
    return self._active_last and not self.active

  def reset(self) -> None:
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False

def _read_param(name, default, cast=float):
  try:
    with open(os.path.join(_PARAM_DIR, name), "r") as f:
      return cast(f.read().strip())
  except Exception:
    return default

class FordPathAngleGate:

  _PA_SPEEDS = (9., 10., 15., 25.)
  _PA_RATES = (0.055, 0.055, 0.0425, 0.009)
  _PA_MAX = 0.44
  _JERK = 3.6
  _FREQ = 20.0
  _MARGIN = 0.85
  _PA_MARGIN = 0.95

  def __init__(self):
    self.mode = 0
    self.gain = 1.00
    self.enter = 1.20
    self.exit = 0.70
    self.in_curve = False
    self.alpha = 0.0
    self.c1 = 0.0
    self.c2 = 0.0
    self._k_last = 0.0
    self._n = 0
    self._log_n = 0

    self.blend = 0.25
    self.vlt_extra_max = 0.05
    self.lc_factor = 1.00
    self.damp = 1.00
    self.humanturn_enable = True
    self.unwind_rate = 0.40
    self.min_v = 9.0

    self.model = None
    self.lateral_delay = 0.12
    self.lane_change = False
    self.lc_direction = 0

    self._k_plan_last = 0.0
    self.vlt = 0.0
    self.kappa_pred = 0.0
    self.kappa_blend = 0.0
    self.blend_eff = 0.0
    self.dbc_sat = False
    self.dev_limited = False
    self.angle_rate_limited = False
    self.human_turn = HumanTurnDetector()
    self.human_turn_active = False

  def reload(self):
    self.mode = int(_read_param("dp_ford_angle_mode", 0, int))
    self.gain = float(np.clip(_read_param("dp_ford_angle_gain", 1.00, float), 0.5, 2.0))
    self.enter = float(np.clip(_read_param("dp_ford_angle_enter", 1.20, float), 0.30, 3.0))
    lo = float(np.clip(_read_param("dp_ford_angle_exit", 0.70, float), 0.10, 2.90))
    self.exit = min(lo, self.enter - 0.05)
    self.blend = float(np.clip(_read_param("dp_ford_angle_blend", 0.25, float), 0.0, 1.0))
    self.vlt_extra_max = float(np.clip(_read_param("dp_ford_angle_vlt", _VLT_T_EXTRA_MAX, float), 0.0, 0.20))
    self.lc_factor = float(np.clip(_read_param("dp_ford_angle_lc", 1.00, float), 0.85, 1.50))
    self.damp = float(np.clip(_read_param("dp_ford_angle_damp", 1.00, float), 0.50, 1.25))
    self.humanturn_enable = bool(int(_read_param("dp_ford_angle_humanturn", 1, int)))
    self.unwind_rate = float(np.clip(_read_param("dp_ford_angle_unwind", 0.40, float), 0.10, 1.00))
    self.min_v = float(np.clip(_read_param("dp_ford_angle_min_v", 9.0, float), 0.0, 20.0))

  def set_context(self, model, lateral_delay):
    self.model = model
    if lateral_delay is not None:
      try:
        self.lateral_delay = float(lateral_delay)
      except (TypeError, ValueError):
        pass
    if model is not None:
      try:
        lcs = int(model.meta.laneChangeState)
        self.lane_change = lcs in (1, 2, 3)
        self.lc_direction = int(model.meta.laneChangeDirection) if self.lane_change else 0
      except Exception:
        self.lane_change = False
        self.lc_direction = 0
    else:
      self.lane_change = False
      self.lc_direction = 0

  def _pa_step(self, v):
    return float(np.interp(max(v, 1.0), self._PA_SPEEDS, self._PA_RATES)) * self._PA_MARGIN

  def _curv_step(self, v):
    return max(self._JERK / (max(v, 1.0) ** 2) / self._FREQ * self._MARGIN, 1e-6)

  def _step_alpha(self, target, kappa, v):
    pa_full = abs(kappa) * v * self.gain
    cu_full = abs(kappa)
    dk = abs(kappa - self._k_last)

    used_pa = dk * v * self.gain * self.alpha
    used_cu = dk * (1.0 - self.alpha)

    budget_pa = max(self._pa_step(v) - used_pa, 0.0)
    budget_cu = max(self._curv_step(v) - used_cu, 0.0)

    d = 1.0
    if pa_full > 1e-9:
      d = min(d, budget_pa / pa_full)
    if cu_full > 1e-9:
      d = min(d, budget_cu / cu_full)

    self.alpha = float(np.clip(target, self.alpha - d, self.alpha + d))
    return self.alpha

  @staticmethod
  def _move(cur, target, step):
    return float(np.clip(target, cur - step, cur + step))

  def _shape_kappa(self, kappa_plan: float, v: float, current_curvature):
    self.kappa_blend = kappa_plan
    self.blend_eff = 0.0
    self.dev_limited = False
    self.dbc_sat = abs(self.c1) >= self._PA_MAX * 0.90

    if self.mode <= 0:
      self.vlt = 0.0
      self.kappa_pred = 0.0
      self._k_plan_last = kappa_plan
      return kappa_plan

    k = kappa_plan
    b = float(np.clip(self.blend, 0.0, 1.0))

    _blend_ok = _BP_MODEL_AVAILABLE and self.model is not None \
        and len(self.model.orientationRate.z) >= 17 and v > 9

    if _blend_ok:
      curvs = np.array(self.model.orientationRate.z) / max(0.01, v)
      t_base = float(np.clip(self.lateral_delay, 0.1, 0.15)) + _DT_MDL
      k_at_base = abs(float(np.interp(t_base, ModelConstants.T_IDXS, curvs)))
      entering = k_at_base > abs(kappa_plan)
      if entering:
        kfac = 1.0
      else:
        kfac = float(np.interp(abs(kappa_plan), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
      sfac = float(np.interp(v, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
      self.vlt = t_base + self.vlt_extra_max * sfac * kfac
      pred = float(np.interp(self.vlt, ModelConstants.T_IDXS, curvs))
      self.kappa_pred = pred
      falling = abs(kappa_plan) < abs(self._k_plan_last) - _EXIT_FALLING_DK
      if (not entering) and (self.dbc_sat or falling):
        b = b * _EXIT_BLEND_SCALE
      if b > 1e-6:
        k = pred * b + kappa_plan * (1.0 - b)
      self.blend_eff = b
    else:
      self.vlt = 0.0
      self.kappa_pred = 0.0

    if self.lane_change and ((self.lc_direction == 1 and k < 0) or (self.lc_direction == 2 and k > 0)):
      k *= self.lc_factor

    if abs(self.damp - 1.0) > 1e-6:
      k *= float(np.interp(v, [13.5, 26.82], [1.0, self.damp]))

    if current_curvature is not None and v > 9:
      kc = float(np.clip(k, current_curvature - CarControllerParams.CURVATURE_ERROR,
                         current_curvature + CarControllerParams.CURVATURE_ERROR))
      self.dev_limited = bool(abs(kc - k) > 1e-9)
      k = kc

    self._k_plan_last = kappa_plan
    self.kappa_blend = k
    return k

  def update(self, lat_active: bool, kappa: float, v_ego: float,
             steering_pressed: bool = False, steering_angle_deg: float = 0.0,
             current_curvature=None):
    self._n += 1
    if (self._n % 40) == 0:
      self.reload()

    kappa = float(kappa)
    kappa_passthrough = kappa
    v = max(float(v_ego), 1.0)

    if self.humanturn_enable and self.mode >= 2:
      self.human_turn_active = self.human_turn.update(
        bool(lat_active), bool(steering_pressed), float(steering_angle_deg))
    else:
      self.human_turn.reset()
      self.human_turn_active = False

    if not lat_active:
      self.in_curve = False
      self.alpha = 0.0
      self.c1 = 0.0
      self.c2 = kappa
      self._k_last = kappa
      self._k_plan_last = kappa
      self.human_turn.reset()
      self.human_turn_active = False
      self.angle_rate_limited = False
      self.dev_limited = False
      return 0.0, kappa

    if self.human_turn_active:
      self.in_curve = False
      self.alpha = 0.0
      self.c1 = 0.0
      self.c2 = 0.0
      self._k_last = kappa
      self._k_plan_last = kappa
      self.angle_rate_limited = False
      self.dev_limited = False
      return 0.0, 0.0

    kappa = self._shape_kappa(kappa, v, current_curvature)

    lat_accel = abs(kappa) * v * v

    if self.mode <= 0:
      self.in_curve = False
      a_target = 0.0
    else:
      if v < self.min_v:
        self.in_curve = False
      elif self.in_curve:
        if lat_accel < self.exit:
          self.in_curve = False
      else:
        if lat_accel > self.enter:
          self.in_curve = True
      a_target = 1.0 if self.in_curve else 0.0

    if self.mode == 1:
      self._step_alpha(a_target, kappa, v)
      self._maybe_log(v, kappa_passthrough, lat_accel)
      self.c1 = 0.0
      self.c2 = kappa_passthrough
      self._k_last = kappa_passthrough
      return 0.0, kappa_passthrough

    if a_target == 0.0 and self.alpha <= 1e-9:
      self.c1 = 0.0
      if self.mode >= 2:
        self.c2 = self._move(self.c2, kappa_passthrough, self._curv_step(v))
      else:
        self.c2 = kappa
      self._k_last = kappa
      return 0.0, self.c2

    a = self._step_alpha(a_target, kappa, v)

    c1_t = float(np.clip(a * (-kappa * v * self.gain), -self._PA_MAX, self._PA_MAX))
    c2_t = (1.0 - a) * kappa

    if self.dbc_sat and abs(self.c1) > abs(c1_t):
      max_drop = self.unwind_rate / self._FREQ
      if abs(self.c1) - abs(c1_t) > max_drop:
        mag = abs(self.c1) - max_drop
        c1_t = mag if self.c1 >= 0 else -mag

    self.c1 = self._move(self.c1, c1_t, self._pa_step(v))
    self.c2 = self._move(self.c2, c2_t, self._curv_step(v))
    self.angle_rate_limited = bool(abs(self.c1 - c1_t) > 1e-9)
    self._k_last = kappa

    self._maybe_log(v, kappa, lat_accel)
    return self.c1, self.c2

  def _maybe_log(self, v, kappa, lat_accel):
    self._log_n += 1
    if (self._log_n % 5) != 0:
      return
    try:
      try:
        if os.path.getsize(_SHADOW_LOG) > _SHADOW_LOG_MAX_BYTES:
          with open(_SHADOW_LOG, "w"):
            pass
      except OSError:
        pass
      with open(_SHADOW_LOG, "a") as f:
        f.write("%.3f,%.6f,%.4f,%.5f,%.6f,%d,%d,%.6f,%.4f,%.4f,%d,%d,%d\n"
                % (v, kappa, self.alpha, -self.c1, -self.c2,
                   int(self.in_curve), self.mode,
                   self.kappa_blend, self.blend_eff, self.vlt,
                   int(self.dev_limited), int(self.angle_rate_limited),
                   int(self.human_turn_active)))
    except Exception:
      pass

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

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

    self.path_angle_gate = FordPathAngleGate()
    self.path_angle_gate.reload()

    self.sm = None
    if _BP_MODEL_AVAILABLE:
      try:
        self.sm = _cereal_messaging.SubMaster(['modelV2', 'lateralDelay'])
      except Exception:
        self.sm = None

  def _inject_model_context(self):
    if self.sm is None:
      return
    try:
      self.sm.update(0)
      model = self.sm['modelV2'] if self.sm.alive['modelV2'] else None
      delay = None
      if self.sm.alive['lateralDelay']:
        delay = float(self.sm['lateralDelay'].lateralDelay)
      self.path_angle_gate.set_context(model, delay)
    except Exception:
      self.path_angle_gate.set_context(None, None)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

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

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      self._inject_model_context()

      if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
        self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
        apply_curvature = self.anti_overshoot_curvature_last
      else:
        apply_curvature = actuators.curvature

      # apply rate limits, curvature error limit, and clip to signal range
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
      # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
      if CS.out.vEgoRaw > 9:
        apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                        current_curvature + CarControllerParams.CURVATURE_ERROR))
      apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                          0., CC.latActive, CarControllerParams.STEER_STEP)
      self.apply_curvature_last = apply_curvature

      path_angle_cmd, curvature_cmd = self.path_angle_gate.update(
        bool(CC.latActive), self.apply_curvature_last, CS.out.vEgoRaw,
        steering_pressed=bool(CS.out.steeringPressed),
        steering_angle_deg=float(CS.out.steeringAngleDeg),
        current_curvature=float(current_curvature))

      lat_active_cmd = bool(CC.latActive) and not self.path_angle_gate.human_turn_active

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02 m^-1)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if lat_active_cmd else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., -path_angle_cmd, -curvature_cmd, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, lat_active_cmd, 0., -path_angle_cmd, -curvature_cmd, 0.))

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
      if accel_pitch_compensated > 0.3 or not CC.longActive:
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
