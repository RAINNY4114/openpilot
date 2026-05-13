import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from openpilot.common.params import Params

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN FD limits:
# Limit to average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06
MAX_LATERAL_ACCEL = 2.2
HUMAN_TURN_STEERING_ANGLE_DEG = 35.0

def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)
  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))

def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature,
                                 v_ego_raw, steering_angle, lat_active, CP):

  if v_ego_raw > 12:
    apply_curvature = np.clip(
      apply_curvature,
      current_curvature - CarControllerParams.CURVATURE_ERROR,
      current_curvature + CarControllerParams.CURVATURE_ERROR
    )

def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)

  apply_curvature = apply_std_steer_angle_limits(
    apply_curvature,
    apply_curvature_last,
    v_ego_raw,
    steering_angle,
    lat_active,
    CarControllerParams.get_angle_limits(CP)
  )

  if CP.flags & FordFlags.CANFD:
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))

  return apply_curvature


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)

def to_float(x):
  return float(x) if x is not None else None

def safe_get_float(params, key):
  try:
    v = params.get(key)
    return float(v) if v is not None else None
  except Exception:
    return None

class BlindSpotMonitor:
    def __init__(self, radar_interface, blindspot_range: float = 5.0):
        """
        初始化盲点监测系统

        :param radar_interface: 用于获取雷达数据的接口
        :param blindspot_range: 盲点范围的阈值（单位：米）
        """
        self.radar_interface = radar_interface
        self.blindspot_range = blindspot_range
        self.blindspot_warning = False  # 默认没有盲点警告

    def check_blindspot(self):
        """
        检查盲点：获取雷达数据，判断盲点内是否有物体
        """
        radar_data = self.radar_interface.update(can_strings)

        if radar_data is None:
            return False

        for point in radar_data.points:
            # 根据车辆的盲区范围判断目标是否进入盲区
            # 假设我们只关心车辆左右两侧的盲区
            if point.dRel < self.blindspot_range and abs(point.yRel) < 2:  # 设定盲区阈值，例如：距离5米，左右2米
                self.blindspot_warning = True
                return True

        self.blindspot_warning = False
        return False

    def get_blindspot_warning(self):
        """
        获取盲点警告状态

        :return: True 如果盲点内有物体，False 否则
        """
        return self.blindspot_warning

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
    self.brake_request = False

    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    # 初始化 BlindSpotMonitor
    self.blindspot_monitor = BlindSpotMonitor(radar_interface)

    self.enable_human_turn_detection = True
    self.human_turn = False
    self.post_reset_ramp_active = False
    self.reset_steering_last = False

    self._auto_blinker_dir = 0
    self._prev_cc_blinker_dir = 0

    self._lidar_cache = {"t": 0, "left": None, "right": None, "front": None}

  def _compute_dynamic_boundaries(self, v_ego):
    # speed (m/s)
    v = max(v_ego, 0.1)

    # fallback fixed values (your params)
    left = self._lidar_cache["left"]
    right = self._lidar_cache["right"]
    front = self._lidar_cache["front"]

    # 1️⃣ fallback base values
    base_side = 3.0
    base_front = 10.0

    # 2️⃣ dynamic model
    side_dynamic = max(1.5, base_side - 0.04 * v * 3.6)  # convert to km/h scale
    front_dynamic = 5.0 + v * 2.2  # TTC ~2.2s

    # 3️⃣ merge logic
    use_sensor = (
      left is not None and left > 0 and
      right is not None and right > 0 and
      front is not None and front > 0
    )

    if use_sensor:
      return left, right, front

    return side_dynamic, side_dynamic, front_dynamic

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

  def update(self, CC, CS, now_nanos):
    can_sends = []
    self._update_human_turn_detection_enabled()

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # 检查盲点警告
    if self.blindspot_monitor.check_blindspot():
        # 发出盲点警告，例如启用盲点警告灯
        print("Blind Spot Detected! Activating warning light.")
        # 你可以在这里发送相关的警告消息
        can_sends.append(fordcan.create_blindspot_warning_msg(self.packer, self.CAN))

    # ===== 新增：雷达缓存更新（不影响原结构）=====
    now = now_nanos * 1e-9
    if now - self._lidar_cache["t"] > 0.1:
      self._lidar_cache["left"] = to_float(self.params.get("lidar_left_dist"))
      self._lidar_cache["right"] = to_float(self.params.get("lidar_right_dist"))
      self._lidar_cache["front"] = to_float(self.params.get("radar_front_dist"))
      self._lidar_cache["t"] = now

    # Latch whether this lane-change blinker request is "auto"
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
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    if self._auto_blinker_dir != 0 and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_turn_signal_msg(self.packer, self.CAN.main, CS.buttons_stock_values, self._auto_blinker_dir))

    ### lateral control ###
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      self.human_turn = self.enable_human_turn_detection and CS.out.steeringPressed and \
                        abs(CS.out.steeringAngleDeg) > HUMAN_TURN_STEERING_ANGLE_DEG
      reset_steering = self.human_turn or CS.out.vEgoRaw < 0.1

      if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14) and not reset_steering:
        self.anti_overshoot_curvature_last = anti_overshoot(
          actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
        requested_curvature = self.anti_overshoot_curvature_last
      else:
        if reset_steering:
          self.anti_overshoot_curvature_last = 0.0
        requested_curvature = float(actuators.curvature)

    # ===== 安全约束（融合点）=====
    try:
      left = to_float(self._lidar_cache["left"])
      right = to_float(self._lidar_cache["right"])
      front = to_float(self._lidar_cache["front"])

      if left is not None and right is not None:
        if left < 1.5 and right < 1.5:
          requested_curvature = 0.0
        elif left < 1.5:
          requested_curvature = min(requested_curvature, 0.0)
        elif right < 1.5:
          requested_curvature = max(requested_curvature, 0.0)

      if left is not None and left < 2.5:
        requested_curvature += 0.002
      if right is not None and right < 2.5:
        requested_curvature -= 0.002

      if front is not None:
        v = CS.out.vEgo
        ttc = front / max(v, 0.1)

        if front < max(6.0, v * 1.2) or ttc < 2.0:
          actuators.accel = min(actuators.accel, -1.0)

        if front < 4.0 or ttc < 1.2:
          actuators.accel = -2.5
          requested_curvature = 0.0

    except Exception:
      pass

    # ===== 原控制逻辑 =====
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
          0.,
          CC.latActive,
          CarControllerParams.get_angle_limits(self.CP),
        )
      else:
        self.apply_curvature_last = apply_ford_curvature_limits(
          requested_curvature,
          self.apply_curvature_last,
          current_curvature,
          CS.out.vEgoRaw,
          0.,
          CC.latActive,
          self.CP
        )
        smooth = np.interp(CS.out.vEgoRaw, [0, 30], [0.3, 0.15])
        smooth = np.clip(smooth, 0.12, 0.35)
        max_delta = 0.002  # 每帧最大变化
        target = (
           smooth * self.apply_curvature_last +
           (1 - smooth) * requested_curvature
        )
        delta = np.clip(target - self.apply_curvature_last, -max_delta, max_delta)
        self.apply_curvature_last += delta
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
      can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
    else:
      can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = float(actuators.accel)
      gas = accel

      long_emergency = False
      front = to_float(self._lidar_cache["front"])

      if front is not None:
        v = CS.out.vEgo
        ttc = front / max(v, 0.1)

        if front < 4.0 or ttc < 1.2:
          long_emergency = True
        elif front < max(6.0, v * 1.2) or ttc < 2.0:
          accel = min(accel, -1.0)

      if CC.longActive:
        accel = apply_creep_compensation(accel, CS.out.vEgo)
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      if long_emergency:
        accel = -2.5
        gas = 0.0

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      self.accel = accel
      self.gas = gas

    self.frame += 1
    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    return new_actuators, can_sends
