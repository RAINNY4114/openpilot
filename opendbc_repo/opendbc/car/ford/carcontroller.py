import math
import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX

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


# =================================================================================================
# [CHX PATH_ANGLE 20261027] 弯道 angle 模式（Ford c1 通道），α 混合版
#
# 目标：直路保持上游曲率控制（c2）一字不改；弯道渐进切到前向角度（c1）以获得更快的前馈响应。
#
# ---- 符号约定（2026-10-27 用实车数据自证，勿凭直觉改）----
# 语料 route 00000004--f2aac252ac 段 3/4，有效车道线 2096 帧：
#   * 锚点：laneLines[1].y[0] < 0、laneLines[2].y[0] > 0
#     => 模型坐标 y 正方向朝**右**（不是左，这一点极易搞反）
#   * 左弯帧里 carState.yawRate > 0 占 100%（yawRate 正=逆时针=左，物理自洽）
#     同时 carControl.actuators.curvature < 0 占 100%
#     => **openpilot 的 κ 正方向 = 右**
#   * Ford 信号语义（fordcan 注释）：c1 正=右、c2 正=左
#     => 左弯：κ<0，c2 = -κ > 0 ✓ 符合 Ford；c1 必须为负
#     => c1(内部) = -κ·v·gain，发送时再取负 => CAN 上 = +κ·v·gain（左弯为负）✓
#   上游第 104/106 行对 curvature 就是 `-self.apply_curvature_last`，本模块对 c1 同样处理。
#
# ---- 为什么用 α 混合而不是硬切 ----
# panda 的 LateralMotionControl 是"整帧 tx=false"式的否决：任一检查违规，本拍 c1 和 c2 一起丢。
# 实测两条限幅都很紧：
#   * c1：path_angle_cmd_checks，{10,15,25} m/s -> {0.0561,0.04335,0.00918} rad/拍
#   * c2：steer_curvature_cmd_checks 的 ISO 侧向 jerk 上限，
#         (3.6/v²/20) rad/m/拍，15 m/s 时仅 0.0008 rad/m
# 入弯瞬间 |κ| = a_lat/v²，在 a_lat=0.8 / v=15 时是 0.00356 rad/m，
# 是 c2 窗口的 4.4 倍 —— 硬切必然丢帧，而且恰好丢在最需要横向控制的时刻。
# 所以这里改成 α ∈ [0,1] 连续混合，并把 α 的每拍增量压到两条通道都不越窗：
#     c1 = α · (-κ·v·gain)      c2 = (1-α) · κ
# α=0 时输出与上游完全一致（c1=0，c2=κ），α=1 时是纯 angle（c2=0）。
#
# ---- 三档模式（/data/ford_params/dp_ford_angle_mode，热加载）----
#   0 = 关（缺省）  恒 α=0，与打补丁前逐字节一致
#   1 = 影子        α 照常演进并写日志，但 CAN 上 c1 恒 0、c2 恒 κ（不发）
#   2 = 实发        按 α 混合下发
# =================================================================================================

_PARAM_DIR = "/data/ford_params"
_SHADOW_LOG = "/data/media/0/path_angle_shadow.log"
# [LOG_CAP_20261027] 这个日志 4 Hz 追加写、历史上从不截断，而且落在用户可见分区
# 根目录（/data/media/0），长期会无界增长。同族的 aa_brake_apply.py / ao_aa_bridge.py /
# auto_overtake.py / soundd.py 都有上限，只有它漏了。这里补一个 2 MB 上限 + 超限清空，
# 纯文件 I/O，不碰任何控制逻辑（α 混合、c1/c2 计算完全不受影响）。
_SHADOW_LOG_MAX_BYTES = 2_000_000


def _read_param(name, default, cast=float):
  """读 /data/ford_params/<name>。缺失/非法一律返回 default，绝不抛。"""
  try:
    with open(os.path.join(_PARAM_DIR, name), "r") as f:
      return cast(f.read().strip())
  except Exception:
    return default


class FordPathAngleGate:
  """弯道 angle 闸门：α 混合，输出 (c1, c2) 的 openpilot 内部符号值。"""

  # c1 每拍允许增量（rad/tick），与 ford.h FORD_PATH_ANGLE_LIMITS 同源再收紧 2%
  _PA_SPEEDS = (9., 10., 15., 25.)
  _PA_RATES = (0.055, 0.055, 0.0425, 0.009)
  # 固件硬上限 ±0.45 rad，这里留余量
  _PA_MAX = 0.44
  # 固件 ISO 侧向 jerk 上限 ~3.6 m/s^3（ISO_LATERAL_ACCEL + g*road_roll）
  _JERK = 3.6
  _FREQ = 20.0
  # 混合限幅留余量，避免浮点/取整把窗口顶穿
  _MARGIN = 0.85
  _PA_MARGIN = 0.95

  def __init__(self):
    self.mode = 0
    self.gain = 1.20
    self.enter = 0.80
    self.exit = 0.50
    self.in_curve = False
    self.alpha = 0.0
    self.c1 = 0.0
    self.c2 = 0.0
    self._k_last = 0.0
    self._n = 0
    self._log_n = 0

  def reload(self):
    self.mode = int(_read_param("dp_ford_angle_mode", 0, int))
    self.gain = float(np.clip(_read_param("dp_ford_angle_gain", 1.20, float), 0.5, 2.0))
    self.enter = float(np.clip(_read_param("dp_ford_angle_enter", 0.80, float), 0.30, 3.0))
    lo = float(np.clip(_read_param("dp_ford_angle_exit", 0.50, float), 0.10, 2.90))
    self.exit = min(lo, self.enter - 0.05)

  # ---- 限幅窗口 ----
  def _pa_step(self, v):
    """c1 每拍允许的 rad 增量。"""
    return float(np.interp(max(v, 1.0), self._PA_SPEEDS, self._PA_RATES)) * self._PA_MARGIN

  def _curv_step(self, v):
    """c2 每拍允许的 1/m 增量，对齐固件 ISO 侧向 jerk 上限并留余量。"""
    return max(self._JERK / (max(v, 1.0) ** 2) / self._FREQ * self._MARGIN, 1e-6)

  def _step_alpha(self, target, kappa, v):
    """把 α 朝 target 移动，同时保证 Δc1 与 Δc2 都不越固件窗口。

    注意 c1 = α·(-κ·v·g) 的每拍变化有两个来源：
      (a) α 变化       -> Δα·|κ|·v·g
      (b) κ 自身变化   -> α·|Δκ|·v·g        <-- 这一项与 α 无关，必须先从预算里扣掉
    漏掉 (b) 会让"入弯时 α 猛涨 + κ 同时上涨"两项叠加顶穿窗口（实测占 150%）。
    """
    pa_full = abs(kappa) * v * self.gain     # α=1 时的 |c1|
    cu_full = abs(kappa)                     # α=1 时被移除的 |c2|
    dk = abs(kappa - self._k_last)

    # κ 自身变化已经吃掉的部分
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
    """朝 target 移动，单步不超过 step。"""
    return float(np.clip(target, cur - step, cur + step))

  def update(self, lat_active: bool, kappa: float, v_ego: float):
    """返回 (c1, c2)，均为 openpilot 内部符号；调用方对两者取负后下发。"""
    self._n += 1
    if (self._n % 40) == 0:                # 20 Hz 下 2 s 一次热加载（关断要快）
      self.reload()

    kappa = float(kappa)
    v = max(float(v_ego), 1.0)

    # 未激活：固件要求 c1 与 c2 都严格为 0，必须立刻归零，不能按斜坡收。
    if not lat_active:
      self.in_curve = False
      self.alpha = 0.0
      self.c1 = 0.0
      self.c2 = kappa
      self._k_last = kappa
      return 0.0, kappa

    lat_accel = abs(kappa) * v * v

    if self.mode <= 0:
      self.in_curve = False
      a_target = 0.0
    else:
      # ---- 弯道判据：侧向加速度，带滞回 ----
      if self.in_curve:
        if lat_accel < self.exit:
          self.in_curve = False
      else:
        if lat_accel > self.enter:
          self.in_curve = True
      a_target = 1.0 if self.in_curve else 0.0

    # ---- 影子：α 照常演进并写日志，但输出一个字节都不改 ----
    if self.mode == 1:
      self._step_alpha(a_target, kappa, v)
      self._maybe_log(v, kappa, lat_accel)
      self.c1 = 0.0
      self.c2 = kappa
      self._k_last = kappa
      return 0.0, kappa

    # ---- α 已完全收回：原样透传，与上游逐字节一致（直路基线零影响）----
    if a_target == 0.0 and self.alpha <= 1e-9:
      self.c1 = 0.0
      self.c2 = kappa
      self._k_last = kappa
      return 0.0, kappa

    a = self._step_alpha(a_target, kappa, v)

    c1_t = float(np.clip(a * (-kappa * v * self.gain), -self._PA_MAX, self._PA_MAX))
    c2_t = (1.0 - a) * kappa

    # 兜底：无论上面算得如何，逐拍增量绝不越窗（α>0 才会走到这里）
    self.c1 = self._move(self.c1, c1_t, self._pa_step(v))
    self.c2 = self._move(self.c2, c2_t, self._curv_step(v))
    self._k_last = kappa

    self._maybe_log(v, kappa, lat_accel)
    return self.c1, self.c2

  def _maybe_log(self, v, kappa, lat_accel):
    self._log_n += 1
    if (self._log_n % 5) != 0:             # 4 Hz
      return
    try:
      # [LOG_CAP_20261027] 超限先清空，否则 4 Hz 追加会让这个文件无界增长。
      # 单独 try 包住：文件不存在（首次启动）时 getsize 会抛 OSError，
      # 不能因此把下面真正的写日志也跳过。
      try:
        if os.path.getsize(_SHADOW_LOG) > _SHADOW_LOG_MAX_BYTES:
          with open(_SHADOW_LOG, "w"):
            pass
      except OSError:
        pass
      with open(_SHADOW_LOG, "a") as f:
        # v, kappa, alpha, c1_can, c2_can, in_curve, mode
        f.write("%.3f,%.6f,%.4f,%.5f,%.6f,%d,%d\n"
                % (v, kappa, self.alpha, -self.c1, -self.c2,
                   int(self.in_curve), self.mode))
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

    # [CHX PATH_ANGLE]
    self.path_angle_gate = FordPathAngleGate()
    self.path_angle_gate.reload()

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
      # Bronco and some other cars consistently overshoot curv requests
      # Apply some deadzone + smoothing convergence to avoid oscillations
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

      # [CHX PATH_ANGLE] α 混合：mode=0 时返回 (0.0, apply_curvature_last)，与上游逐字节一致。
      path_angle_cmd, curvature_cmd = self.path_angle_gate.update(
        bool(CC.latActive), self.apply_curvature_last, CS.out.vEgoRaw)

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02 m^-1)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., -path_angle_cmd, -curvature_cmd, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., -path_angle_cmd, -curvature_cmd, 0.))

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
