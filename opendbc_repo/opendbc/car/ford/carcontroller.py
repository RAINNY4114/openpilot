import math
import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX

# [BP_ANGLE_UPGRADE 20261102] 模型预测混合所需的可选依赖。
#
# carcontroller 是安全关键路径上的进程，不能因为一个**可选**特性导入失败就起不来，
# 所以这里整体 try 包住：失败时 _BP_MODEL_AVAILABLE=False，_shape_kappa 会自动退化为
# "纯规划曲率"，与升级前逐字节一致。
#
# 注意：BluePilot 用的是 liveDelay 服务，本机 sunnypilot 版本里叫 lateralDelay
# （cereal/services.py 已确认），字段名同为 lateralDelay。
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
#
# =================================================================================================
# [BP_ANGLE_UPGRADE 20261102] 从 BluePilot bp-7.0 移植的增强
# 参考 https://github.com/BluePilotDev/bluepilot/tree/bp-7.0
#      opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py (636 行)
#      opendbc_repo/opendbc/sunnypilot/car/ford/human_turn.py
#
# ---- 为什么不能整份照搬 ----
# BluePilot 的 angle 模式是"纯 angle"策略：c2 恒 0、c1 独立承担全部横向控制，并且配套改了
# panda 的 ford.h（新增 shadow_curvature 偏差检查、FORD_PATH_ANGLE_LIMITS 的 x1.02 放宽）。
# 本机 panda 固件是预编译的，改不了安全层，所以整份移植会直接把车开进 panda 的 veto。
#
# 本机保留 α 混合架构（c1/c2 都在现有 ford.h 窗口内），只移植**不触碰安全边界**的算法层：
#
#   1. 模型预测曲率混合   κ' = κ_pred·b + κ_plan·(1-b)，b 默认 0.50
#                         （BluePilot 在 F-150 车队数据上验证的默认值）
#   2. 可变前瞻时间 VLT    t = t_base + t_extra·speed_factor·kappa_factor
#                         高速收缩额外前瞻；入弯（模型前瞻曲率 > 规划）保持满前瞻以提前预打，
#                         出弯/弯心按曲率幅值收缩，避免把"开始回正"的信号提前导入
#   3. 出弯偏置            接近 DBC 限幅或规划曲率正在下降时，b → 0.25b
#                         解决出弯时模型仍看到弯道导致回正偏慢，以及前瞻跨过弯心造成的突跳
#   4. 偏差限幅后移        混合后再夹到实测曲率 ± CURVATURE_ERROR（原先只在混合前夹一次）
#   5. 变道增益            变道时按方向缩放 κ'（angle 需要放大，curvature 需要收缩）
#   6. 人工转向接管        持续按压 + |转角|>45° 超过 1.5s → 强制 mode 0，PSCM 干净释放；
#                         松开后 c1 从 0 按软 ROC 斜坡回填（无 jump seed）
#                         解决 PSCM 在接管后 2~3 s 的重新接合死区（Mach-E 实测）
#   7. 饱和退绕限速        接近 DBC 限幅时限制 |c1| 的下降速率（默认 0.40 rad/s）
#                         避免 PSCM 被钉住时 desired 掉得比 actual 快，释放瞬间突跳
#
# ---- 明确未移植（需要改 panda / UI，留作后续）----
#   * LaneCenterTrim 车道居中微调（203 行独立模块，需车道线置信度，另需 UI 开关）
#   * 接管后 stall blip 脉冲（依赖偏差限幅死锁检测，300ms 横向空窗，风险偏高）
#   * FordPrefLateralControl 策略切换（需 UI 参数页与 BP 的 CarParamsSP 通道）
#   * shadow_curvature / ford.h 偏差检查（panda 固件不可改）
#
# ---- 新增参数（/data/ford_params/，全部热加载，缺失即用默认值）----
#   dp_ford_angle_blend     0.00~1.00   默认 0.50   模型预测混合比（0=纯规划，1=纯模型）
#   dp_ford_angle_vlt       0.00~0.20   默认 0.10   最大额外前瞻（s）
#   dp_ford_angle_lc        0.85~1.50   默认 1.15   变道增益
#   dp_ford_angle_damp      0.50~1.25   默认 1.00   高速阻尼（1.0=不启用）
#   dp_ford_angle_humanturn 0 / 1       默认 1      人工转向接管强制 mode 0
#   dp_ford_angle_unwind    0.10~1.00   默认 0.40   饱和退绕速率（rad/s）
# =================================================================================================

_PARAM_DIR = "/data/ford_params"
_SHADOW_LOG = "/data/media/0/path_angle_shadow.log"
# [LOG_CAP_20261027] 这个日志 4 Hz 追加写、历史上从不截断，而且落在用户可见分区
# 根目录（/data/media/0），长期会无界增长。同族的 aa_brake_apply.py / ao_aa_bridge.py /
# auto_overtake.py / soundd.py 都有上限，只有它漏了。这里补一个 2 MB 上限 + 超限清空，
# 纯文件 I/O，不碰任何控制逻辑（α 混合、c1/c2 计算完全不受影响）。
_SHADOW_LOG_MAX_BYTES = 2_000_000

# ---- [BP_ANGLE_UPGRADE 20261102] 模型预测混合 / VLT 常量（数值取自 BluePilot bp-7.0）----
_DT_MDL = 0.05                      # 模型循环周期，与 common/realtime.py 一致
_VLT_T_EXTRA_MAX = 0.10             # 额外前瞻上限（s）
_VLT_V_LOW_MS = 25.0 * 0.44704      # 25 mph —— 此速度及以下给满额外前瞻
_VLT_V_HIGH_MS = 55.0 * 0.44704     # 55 mph —— 此速度及以上不再给额外前瞻
_VLT_KAPPA_FULL = 0.005             # 1/m —— 此曲率以下给满额外前瞻（半径 200m+）
_VLT_KAPPA_TAPER = 0.020            # 1/m —— 此曲率以上完全不给（半径 50m）
_EXIT_BLEND_SCALE = 0.25            # 出弯/饱和时 b 的缩放系数（0.50 → 0.125）
_EXIT_FALLING_DK = 0.010            # 判定"规划曲率正在下降"的每拍阈值（1/m）

# ---- 人工转向接管阈值（同 BluePilot human_turn.py）----
_HUMAN_TURN_ANGLE_DEG = 45.0
_HUMAN_TURN_HOLD_S = 1.5
# 首次接触时方向盘已经超过阈值（横向控制把它拧在弯里），说明是"弯中补一手"而不是主动接管，
# 需要更长的持续按压才判定，否则轻触 1.5s 就会把弯中的转向掐掉。
_HUMAN_TURN_HOLD_PRETURNED_S = 3.0


class HumanTurnDetector:
  """持续按压 + 大转角 → 判定驾驶员主动接管。

  判定门槛是"连续 HUMAN_TURN_HOLD_S 同时满足 pressed 与 |angle|>45°"，用于区分主动转向
  与弯中的短暂修正。``just_released`` 在解除的第一拍为 True，供上层重新起坡使用。
  """

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

    # ---- [BP_ANGLE_UPGRADE 20261102] 模型预测混合相关 ----
    self.blend = 0.50            # dp_ford_angle_blend
    self.vlt_extra_max = _VLT_T_EXTRA_MAX   # dp_ford_angle_vlt
    self.lc_factor = 1.15        # dp_ford_angle_lc
    self.damp = 1.00             # dp_ford_angle_damp
    self.humanturn_enable = True  # dp_ford_angle_humanturn
    self.unwind_rate = 0.40      # dp_ford_angle_unwind（rad/s）

    # 由 CarController 每拍注入的上下文（模型缺失时为 None，功能自动退化）
    self.model = None
    self.lateral_delay = 0.12
    self.lane_change = False
    self.lc_direction = 0

    # 运行状态 / 遥测
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
    self.gain = float(np.clip(_read_param("dp_ford_angle_gain", 1.20, float), 0.5, 2.0))
    self.enter = float(np.clip(_read_param("dp_ford_angle_enter", 0.80, float), 0.30, 3.0))
    lo = float(np.clip(_read_param("dp_ford_angle_exit", 0.50, float), 0.10, 2.90))
    self.exit = min(lo, self.enter - 0.05)
    # [BP_ANGLE_UPGRADE] 新增项一律"缺失即保持旧行为"，除 blend 外默认值都不改变现有手感。
    self.blend = float(np.clip(_read_param("dp_ford_angle_blend", 0.50, float), 0.0, 1.0))
    self.vlt_extra_max = float(np.clip(_read_param("dp_ford_angle_vlt", _VLT_T_EXTRA_MAX, float), 0.0, 0.20))
    self.lc_factor = float(np.clip(_read_param("dp_ford_angle_lc", 1.15, float), 0.85, 1.50))
    self.damp = float(np.clip(_read_param("dp_ford_angle_damp", 1.00, float), 0.50, 1.25))
    self.humanturn_enable = bool(int(_read_param("dp_ford_angle_humanturn", 1, int)))
    self.unwind_rate = float(np.clip(_read_param("dp_ford_angle_unwind", 0.40, float), 0.10, 1.00))

  def set_context(self, model, lateral_delay):
    """每拍由 CarController 注入模型与延迟估计。任一缺失都不影响主流程。"""
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

  def _shape_kappa(self, kappa_plan: float, v: float, current_curvature):
    """κ_plan → κ'：模型预测混合 + 出弯偏置 + 变道增益 + 高速阻尼 + 偏差限幅。

    mode<=0 或模型不可用时**原样返回** kappa_plan，保证关断时逐字节一致。
    """
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

    # [BP_ANGLE_UPGRADE 20261102] 模型混合只在 v>9 启用，与下面偏差限幅的门槛一致。
    # 理由：v<=9 时偏差限幅被跳过（低速扭矩未建立、实测曲率不准），混合结果**没有任何兜底**；
    # 而实车回放显示低速掉头/挪车时模型前瞻曲率可达规划的 3~6 倍，直接混进来会让 c1 明显偏离。
    # 低速段 lat_accel=|κ|v² 很小，弯道判据基本不触发 α，所以这里关掉不影响正常弯道能力。
    _blend_ok = _BP_MODEL_AVAILABLE and self.model is not None \
        and len(self.model.orientationRate.z) >= 17 and v > 9

    if _blend_ok:
      curvs = np.array(self.model.orientationRate.z) / max(0.01, v)
      # liveDelay 在本机叫 lateralDelay；BluePilot 把它夹到 [0.1,0.15] 再叠加模型周期，
      # 因为该估计偶尔会标到 400ms+，那会把前瞻推到 5m 之外，模型直接看到弯心。
      t_base = float(np.clip(self.lateral_delay, 0.1, 0.15)) + _DT_MDL
      k_at_base = abs(float(np.interp(t_base, ModelConstants.T_IDXS, curvs)))
      # 入弯：模型在 t_base 处看到的曲率比规划当前的大 → 保持满额外前瞻，提前预打
      entering = k_at_base > abs(kappa_plan)
      if entering:
        kfac = 1.0
      else:
        kfac = float(np.interp(abs(kappa_plan), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
      sfac = float(np.interp(v, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
      self.vlt = t_base + self.vlt_extra_max * sfac * kfac
      pred = float(np.interp(self.vlt, ModelConstants.T_IDXS, curvs))
      self.kappa_pred = pred
      # 出弯偏置：接近 DBC 限幅、或规划曲率正在下降时，把模型权重压到 1/4，
      # 让规划的自然回正主导，避免"模型仍看到弯道"造成的慢回正和前瞻跨弯心的突跳。
      falling = abs(kappa_plan) < abs(self._k_plan_last) - _EXIT_FALLING_DK
      if (not entering) and (self.dbc_sat or falling):
        b = b * _EXIT_BLEND_SCALE
      if b > 1e-6:
        k = pred * b + kappa_plan * (1.0 - b)
      self.blend_eff = b
    else:
      self.vlt = 0.0
      self.kappa_pred = 0.0

    # 变道增益：只在"变道方向与当前曲率方向相反"时缩放（与 BluePilot 同判据）
    if self.lane_change and ((self.lc_direction == 1 and k < 0) or (self.lc_direction == 2 and k > 0)):
      k *= self.lc_factor

    # 高速阻尼（默认 1.0，等于不启用；仅当用户显式设置时才生效）
    if abs(self.damp - 1.0) > 1e-6:
      k *= float(np.interp(v, [13.5, 26.82], [1.0, self.damp]))

    # 偏差限幅后移：混合后的 κ' 可能已经离开实测曲率窗口，这里重新夹一次。
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
    """返回 (c1, c2)，均为 openpilot 内部符号；调用方对两者取负后下发。"""
    self._n += 1
    if (self._n % 40) == 0:                # 20 Hz 下 2 s 一次热加载（关断要快）
      self.reload()

    kappa = float(kappa)
    kappa_passthrough = kappa             # mode<=1 时原样透传用，保证"影子"不改变输出
    v = max(float(v_ego), 1.0)

    # [BP_ANGLE_UPGRADE] 人工转向接管检测：每拍都跑，未激活时计时器清零
    if self.humanturn_enable:
      self.human_turn_active = self.human_turn.update(
        bool(lat_active), bool(steering_pressed), float(steering_angle_deg))
    else:
      self.human_turn.reset()
      self.human_turn_active = False

    # 未激活：固件要求 c1 与 c2 都严格为 0，必须立刻归零，不能按斜坡收。
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

    # [BP_ANGLE_UPGRADE] 接管中：强制全零，并让调用方把 mode 降到 0。
    # 这样 PSCM 收到的是"合法的未激活帧"——ford.h 的每条检查都有 !steer_control_enabled 分支，
    # 所以零帧天然合规，不涉及 reset-bypass 闩锁。松开后 c1 从 0 按软 ROC 回填，无 jump seed。
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

    # [BP_ANGLE_UPGRADE] 模型预测混合 / 出弯偏置 / 变道增益 / 偏差限幅
    kappa = self._shape_kappa(kappa, v, current_curvature)

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
    # [BP_ANGLE_UPGRADE] 注意这里必须用 kappa_passthrough 而不是 kappa：kappa 已被
    # _shape_kappa 混合过，直接下发会破坏"影子模式输出与上游逐字节一致"的契约。
    if self.mode == 1:
      self._step_alpha(a_target, kappa, v)
      self._maybe_log(v, kappa_passthrough, lat_accel)
      self.c1 = 0.0
      self.c2 = kappa_passthrough
      self._k_last = kappa_passthrough
      return 0.0, kappa_passthrough

    # ---- α 已完全收回：原样透传，与上游逐字节一致（直路基线零影响）----
    if a_target == 0.0 and self.alpha <= 1e-9:
      self.c1 = 0.0
      if self.mode >= 2:
        # [BP_ANGLE_UPGRADE 20261102] mode=2 下 κ' 已被模型混合 / 偏差限幅改动过，
        # 单拍可能跳变（改前 κ 来自 CURVATURE_LIMITS.apply_limits，天然已限速）。
        # 这里补上 c2 的逐拍窗口限制，否则会撞 panda 的 steer_curvature_cmd_checks
        # （ISO 侧向 jerk）导致整帧被否决。mode<=1 保持逐字节透传，不受影响。
        self.c2 = self._move(self.c2, kappa, self._curv_step(v))
      else:
        self.c2 = kappa
      self._k_last = kappa
      return 0.0, self.c2

    a = self._step_alpha(a_target, kappa, v)

    c1_t = float(np.clip(a * (-kappa * v * self.gain), -self._PA_MAX, self._PA_MAX))
    c2_t = (1.0 - a) * kappa

    # [BP_ANGLE_UPGRADE] 饱和退绕限速：接近 DBC 限幅时，限制 |c1| 的**下降**速率。
    # 模型和规划在急弯顶点会以 ~0.36 rad/s 掉 path_angle，而此刻 PSCM 被物理钉住，
    # 跟不上这个下降速度，desired 与 actual 之间会拉开最大 47° 的缺口，
    # 一旦 PSCM 释放就会突跳修正。这里把下降速率压到 PSCM 跟得上的水平
    # （默认 0.40 rad/s ÷ 20 Hz = 0.02 rad/拍），代价是饱和期间车被多留在弯里一点。
    if self.dbc_sat and abs(self.c1) > abs(c1_t):
      max_drop = self.unwind_rate / self._FREQ
      if abs(self.c1) - abs(c1_t) > max_drop:
        mag = abs(self.c1) - max_drop
        c1_t = mag if self.c1 >= 0 else -mag

    # 兜底：无论上面算得如何，逐拍增量绝不越窗（α>0 才会走到这里）
    self.c1 = self._move(self.c1, c1_t, self._pa_step(v))
    self.c2 = self._move(self.c2, c2_t, self._curv_step(v))
    self.angle_rate_limited = bool(abs(self.c1 - c1_t) > 1e-9)
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
        # [BP_ANGLE_UPGRADE] 追加 kappa_blend / blend_eff / vlt / 标志位，
        # 便于调 blend 与 VLT 时直接对账（旧字段顺序保持不变，前 7 列含义不变）。
        # v, kappa_plan, alpha, c1_can, c2_can, in_curve, mode,
        # kappa_blend, blend_eff, vlt, dev_limited, angle_rate_limited, human_turn
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

    # [CHX PATH_ANGLE]
    self.path_angle_gate = FordPathAngleGate()
    self.path_angle_gate.reload()

    # [BP_ANGLE_UPGRADE 20261102] 模型预测混合需要 modelV2 与 lateralDelay。
    #
    # 本机没有 liveDelay 服务（那是 BluePilot 新增的），sunnypilot 里对应的是 lateralDelay
    # （cereal/services.py 已确认，字段同为 .lateralDelay）。SubMaster 建失败时置 None，
    # 后续每拍跳过注入，_shape_kappa 自动退化为纯规划曲率。
    self.sm = None
    if _BP_MODEL_AVAILABLE:
      try:
        self.sm = _cereal_messaging.SubMaster(['modelV2', 'lateralDelay'])
      except Exception:
        self.sm = None

  def _inject_model_context(self):
    """把 modelV2 / lateralDelay 喂给 angle 闸门。任何异常都不允许冒泡到控制路径。"""
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
      # [BP_ANGLE_UPGRADE] 注入模型上下文。放在 20 Hz 分支里而不是每拍：
      # 闸门本身就只在 20 Hz 跑，100 Hz 更新 SubMaster 只是白烧 CPU。
      self._inject_model_context()

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
      # [BP_ANGLE_UPGRADE] 额外传入实测曲率（偏差限幅后移用）与驾驶员转向状态（接管检测用）。
      path_angle_cmd, curvature_cmd = self.path_angle_gate.update(
        bool(CC.latActive), self.apply_curvature_last, CS.out.vEgoRaw,
        steering_pressed=bool(CS.out.steeringPressed),
        steering_angle_deg=float(CS.out.steeringAngleDeg),
        current_curvature=float(current_curvature))

      # [BP_ANGLE_UPGRADE] 人工转向接管期间把 mode 降到 0（横向未激活），让 PSCM 干净释放。
      # gate 在同一拍已把 c1/c2 全零，所以这是一帧合法的"未激活"报文。
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
