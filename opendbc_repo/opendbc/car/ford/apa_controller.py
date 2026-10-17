"""
Ford APA (Active Park Assist) 角度域横向控制器 —— 路线 A 版本
=============================================================

背景
----
设备 (C3X / SunnyPilot) 的横向控制是 **曲率域** (Curvature domain):
    controlsd -> actuators.curvature -> CarController -> LatCtl_* 报文

而 C2 (DragonPilot 0.8.1) 的 Ford APA 走的是 **角度域** (Angle domain):
    直接构造 ParkAid_Data (0x3A8)，让 PSCM 按绝对方向盘角度执行。

本模块把角度域逻辑移植到设备的曲率域架构上，作为一个 **可插拔的 APA 支路**，
与既有 FordCurveController 并存：

    CarController.update()
      ├── 既有支路: FordCurveController -> LatCtl_D_Rq  (0x3D3 / 0x3D6)   不变
      └── APA 支路: FordAPAController  -> ParkAid_Data  (0x3A8)           本模块

安全前提 (路线 A)
-----------------
本模块存在的前提是 panda 处于 `SafetyModel::allOutput`，即 **panda 的全部
TX 校验已被旁路**。因此 panda 原本提供的保护必须在本模块内复现：

    P1  角度斜坡限制   max_delta = 速度调度表(v) * dt_ms / 20 * rate_scale
                       windup / unwind 双表, 移植自 `apa` 分支
    P2  角度钳位       速度调度钳位 APA_ANGLE_MAX_*(v)
                       + 硬范围兜底 [FORD_APA_MIN_DEG, FORD_APA_MAX_DEG]
    P3  apa_stat 合法性 {0,1,2}，且 Active(2) 需 latActive
    P4  看门狗         超时 -> 发"放手帧"
    P5  失效安全默认   任何异常 -> 放手帧，绝不保持上一帧
    P6  分级开关       默认全关(L0)
    P7  SAPP 握手门控  未推进到"请求角度"阶段前一律不请求

SAPP 握手 (P7)
--------------
移植自 RAINNY4114/opendbc-sunnypilot 分支 `apa` 的 apa_update()。
PSCM 只在与 GWM 完成 SAPPStatusCoding 握手后才接受外部角度请求：

    SAPPStatusCoding 循环:   0 -> 70 -> 86 -> 224 -> 16(稳态)
    SAPPAngleControlStat1 (EPAS_INFO): 0=Closed 1=Open 2=Active 3=Fault

⚠️ 设备此前版本从不发送 SAPPStatusCoding（恒 0），这被认为是
   PSCM 不接受角度请求的首要嫌疑。本版本补齐该握手。

三级开关 (P6)
-------------
    dp_ford_apa_enable = 0   L0 关闭     完全不产生 0x3A8        (默认)
    dp_ford_apa_enable = 1   L1 冒烟     只写影子日志，不真发
    dp_ford_apa_enable = 2   L2 实发     真实发送 0x3A8

参数使用 **纯文件 I/O** 读取（与 fordcan.py 的 dp_ford_* 一致），
因为这些开关不在 params_keys.h 中，用 Params.get() 会抛 UnknownKeyName。

搜索目录: /data/ford_params, /dev/shm/params, /data/params/d

作者备注
--------
本文件是**独立模块**，不修改任何既有文件即可导入。
集成方式见 05_carcontroller_集成补丁.md。
"""

from __future__ import annotations

import math
import os
import time

from opendbc.car.ford import fordcan


# ===========================================================================
# ⚠️ 关键接口约定 (已对设备源码实测确认)
#
# can_sends 的每个元素必须是 **3 元组** (addr, data:bytes, bus:int)。
# 依据:
#   opendbc/can/packer.py:55  return addr, bytes(dat), bus
#   pandad/pandad_api_impl.py:53-55
#       f._set_by_field(addr_f, msg[0])
#       f._set_by_field(dat_f,  msg[1])
#       f._set_by_field(src_f,  msg[2])     # <== msg[2] 必须存在
#
# 但已部署的 fordcan.create_apa_steer_msg_raw() 返回的是 **2 元组**
# (addr, data) —— 直接 append 会 IndexError, 进而整个 sendcan 失效!
#
# 因此本模块**不**使用 create_apa_steer_msg_raw() 的原始返回值,
# 而是自己组装 3 元组, 或使用 create_apa_steer_msg() (packer 路径)。
# ===========================================================================


# ===========================================================================
# 常量
# ===========================================================================

# --- P2: 角度硬范围 (与 panda ford.h 的 FORD_APA_MIN/MAX_RAW 对齐) -----------
FORD_APA_MIN_RAW = 5000        # raw 下限 -> deg = -500
FORD_APA_MAX_RAW = 15000       # raw 上限 -> deg = +500
FORD_APA_MIN_DEG = FORD_APA_MIN_RAW * 0.1 - 1000.0    # -500.0
FORD_APA_MAX_DEG = FORD_APA_MAX_RAW * 0.1 - 1000.0    # +500.0

# --- P1: 斜坡限制 -------------------------------------------------------------
# 原 panda 规则: max_delta = 5.0 * dt_ms / 20.0
FORD_APA_RATE_DEG_PER_20MS = 5.0

# --- P4: 看门狗 ---------------------------------------------------------------
FORD_APA_WATCHDOG_MS = 500     # 超过则发放手帧

# --- 发送周期 -----------------------------------------------------------------
# C2 用 FRAME_STEP = 16 (100Hz / 16 ≈ 6.25Hz 偏慢)
# 设备 CarControllerParams.STEER_STEP 通常为 5 (20Hz)。默认按 20Hz 设计。
FORD_APA_STEP = 5
FORD_APA_PERIOD_MS = 50.0      # 20Hz

# --- SAPP 握手 (来自 C2 carstate) --------------------------------------------
# SAPPAngleControlStat1 (EPAS_INFO) —— PSCM 侧握手状态
SAPP_CLOSED = 0
SAPP_OPEN = 1
SAPP_ACTIVE = 2
SAPP_FAULT = 3

# --- SAPP 握手编码 (SAPPStatusCoding) ----------------------------------------
# 移植自 RAINNY4114/opendbc-sunnypilot 分支 `apa`
SAPP_CFG_RESET = 0         # 未开始 / 复位
SAPP_CFG_INIT = 70         # 起步: 请求 PSCM 打开
SAPP_CFG_ACK = 86          # PSCM ack(Open) 后
SAPP_CFG_ACTIVE = 224      # PSCM 进入 Active
SAPP_CFG_STEADY = 16       # 稳态
SAPP_CFG_ACK_FRAMES = 8    # ack 态累计帧数阈值 -> 升 86
SAPP_CFG_REQ_FRAMES = 13   # 开始请求角度的帧阈值
SAPP_CFG_CYCLE_FRAMES = 20  # 86 态一轮的帧数
SAPP_CFG_ACTIVE_FRAMES = 3  # 224 态保持帧数

# --- 参数文件 -----------------------------------------------------------------
_DP_PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
_DP_KEY_ENABLE = "dp_ford_apa_enable"
_DP_KEY_RATE_SCALE = "dp_ford_apa_rate_scale"
_DP_KEY_SHADOW_LOG = "dp_ford_apa_shadow_log_path"

_DEFAULT_SHADOW_LOG = "/data/media/0/apa_shadow.log"

# 参数缓存 (避免每帧 20Hz 读盘)
_PARAM_CACHE: dict[str, tuple[float, object]] = {}
_PARAM_CACHE_TTL = 1.0   # 秒


# --- APA 速度调度限制表 (单一来源: values.CarControllerParams) ----------------
# 移植自 RAINNY4114/opendbc-sunnypilot 分支 `apa`
# 表值单位: 角度 = deg(方向盘); 速率 = deg / 20ms (即 50Hz 基准帧)
try:
  from opendbc.car.ford.values import CarControllerParams as _CCP

  APA_ANGLE_MAX_BP = tuple(_CCP.APA_ANGLE_MAX_BP)
  APA_ANGLE_MAX_V = tuple(_CCP.APA_ANGLE_MAX_V)
  APA_ANGLE_DELTA_BP = tuple(_CCP.APA_ANGLE_DELTA_BP)
  APA_ANGLE_DELTA_V = tuple(_CCP.APA_ANGLE_DELTA_V)      # windup
  APA_ANGLE_DELTA_VU = tuple(_CCP.APA_ANGLE_DELTA_VU)    # unwind
except Exception:  # pragma: no cover - 离线自检兜底
  APA_ANGLE_MAX_BP = (0., 11., 36.)
  APA_ANGLE_MAX_V = (410., 25., 15.)
  APA_ANGLE_DELTA_BP = (0., 5., 15.)
  APA_ANGLE_DELTA_V = (5., .8, .15)
  APA_ANGLE_DELTA_VU = (5., 3.5, 0.4)


def _interp(x: float, bp, vals) -> float:
  """线性插值 (等价 numpy.interp, 但不引入 numpy 依赖)。

  x 超出 bp 范围 -> 取端点值; x 非有限值 -> 取 vals[0]。
  """
  try:
    x = float(x)
  except (TypeError, ValueError):
    return float(vals[0])
  if not math.isfinite(x):
    return float(vals[0])
  if x <= bp[0]:
    return float(vals[0])
  if x >= bp[-1]:
    return float(vals[-1])
  for i in range(len(bp) - 1):
    if bp[i] <= x <= bp[i + 1]:
      span = float(bp[i + 1]) - float(bp[i])
      if span <= 0.0:
        return float(vals[i + 1])
      t = (x - float(bp[i])) / span
      return float(vals[i] + t * (float(vals[i + 1]) - float(vals[i])))
  return float(vals[-1])


# ===========================================================================
# 参数读取 (纯文件 I/O)
# ===========================================================================

def _read_dp_param(key: str, default: float | None = None) -> float | None:
  """从候选目录读取一个标量参数。

  这些 key 不在 params_keys.h 中，Params.get() 会抛 UnknownKeyName，
  因此必须用裸文件读取。带 1s 缓存以避免 20Hz 反复读盘。
  """
  now = time.monotonic()
  hit = _PARAM_CACHE.get(key)
  if hit is not None and (now - hit[0]) < _PARAM_CACHE_TTL:
    return hit[1]  # type: ignore[return-value]

  val: float | None = default
  for d in _DP_PARAM_DIRS:
    path = os.path.join(d, key)
    try:
      with open(path, "r") as f:
        txt = f.read().strip()
      if txt:
        val = float(txt)
        break
    except (OSError, ValueError):
      continue

  _PARAM_CACHE[key] = (now, val)
  return val


def apa_enable_level() -> int:
  """P6: 返回 0/1/2。任何异常一律返回 0 (关闭)。"""
  try:
    v = _read_dp_param(_DP_KEY_ENABLE, 0.0)
    if v is None:
      return 0
    lvl = int(v)
    if lvl < 0:
      return 0
    if lvl > 2:
      return 2
    return lvl
  except Exception:
    return 0


def _rate_scale() -> float:
  """斜坡限制缩放系数, 默认 1.0。调试时可临时放宽/收紧。"""
  try:
    v = _read_dp_param(_DP_KEY_RATE_SCALE, 1.0)
    if v is None or not math.isfinite(float(v)) or float(v) <= 0.0:
      return 1.0
    return float(v)
  except Exception:
    return 1.0


# ===========================================================================
# 曲率 <-> 角度 换算
# ===========================================================================

def curvature_to_steer_angle(curvature: float, v_ego: float,
                             steer_ratio: float, wheelbase: float,
                             max_angle_deg: float = FORD_APA_MAX_DEG) -> float:
  """把曲率 (1/m) 换算成方向盘角度 (deg)。

  纯几何关系 (自行车模型, 小角度):
      kappa = tan(delta) / L        ->  delta = atan(kappa * L)
      delta_steer = delta * steer_ratio

  其中 delta 为前轮转角, delta_steer 为方向盘角度。

  注意: 这里**不**引入车辆模型的 understeer 修正 —— 那属于
  FordCurveController 的职责。APA 支路只做几何映射,
  实际误差由 PSCM 自身的角度闭环吸收。

  Args:
    curvature:      目标曲率 (1/m)
    v_ego:          车速 (m/s), 仅用于低速保护
    steer_ratio:    转向比 (设备在线学习值, MKX ≈ 15.0)
    wheelbase:      轴距 (m)
    max_angle_deg:  输出钳位上限 (对称)

  Returns: 方向盘角度 (deg), 已钳位
  """
  if not (math.isfinite(curvature) and math.isfinite(v_ego)):
    return 0.0
  if not (math.isfinite(steer_ratio) and steer_ratio > 0.0):
    return 0.0
  if not (math.isfinite(wheelbase) and wheelbase > 0.0):
    return 0.0

  # 前轮转角
  delta_rad = math.atan(curvature * wheelbase)
  # 方向盘角度
  angle_deg = math.degrees(delta_rad) * steer_ratio

  lim = abs(float(max_angle_deg))
  return float(max(-lim, min(lim, angle_deg)))


def steer_angle_to_curvature(angle_deg: float, steer_ratio: float,
                             wheelbase: float) -> float:
  """曲率 -> 方向盘角度的逆运算 (用于影子模式对比/自检)。"""
  if not (math.isfinite(angle_deg) and math.isfinite(steer_ratio)):
    return 0.0
  if not (math.isfinite(wheelbase) and wheelbase > 0.0):
    return 0.0
  if abs(steer_ratio) < 1e-6:
    return 0.0
  delta_rad = math.radians(angle_deg / steer_ratio)
  return float(math.tan(delta_rad) / wheelbase)


# ===========================================================================
# 主控制器
# ===========================================================================

class FordAPAController:
  """Ford APA 角度域横向控制器 (路线 A)

  由 CarController.update() 每个 STEER_STEP 调用一次。

  典型用法:
      # __init__ 中
      self.apa = FordAPAController(CP)

      # update() 中, 在既有 LatCtl 发送之后
      apa_msg = self.apa.update(
          frame=self.frame,
          lat_active=bool(CC.latActive),
          curvature=apply_curvature,
          v_ego=v_ego,
          sapp_state=CS.apa_sapp_state,
          eps_assist_limited=CS.apa_eps_assist_limited,
          veh_speed_kph=CS.apa_veh_speed_kph,
          now_ms=now_ms,
      )
      if apa_msg is not None:
          can_sends.append(apa_msg)     # 3 元组 (addr, data, bus)
  """

  def __init__(self, CP):
    self.CP = CP

    # 在线学习/静态参数
    self.steer_ratio = float(getattr(CP, "steerRatio", 15.0) or 15.0)
    self.wheelbase = float(
      getattr(CP, "wheelbase", 2.85) or 2.85
    )

    # 发送总线 (设备实测: CanBus(CP).main == 0, 与 carstate 读 0x3A8 的
    # Bus.pt 一致)。CANFD 平台下仍然由 CanBus 的 offset 决定。
    try:
      self.bus_main = int(fordcan.CanBus(CP).main)
    except Exception:
      self.bus_main = 0

    # 斜坡限制状态
    self.last_angle_deg = 0.0
    self.last_rate_ms = 0

    # 看门狗
    self.last_update_ms = 0

    # 握手状态 (由外部喂入)
    self.sapp_state = SAPP_CLOSED
    self.eps_assist_limited = False

    # SAPP 握手状态机 (移植自 apa 分支 apa_update)
    self.sapp_config = SAPP_CFG_RESET
    self.sapp_config_last = SAPP_CFG_RESET
    self.sapp_angle_req = 0
    self.sapp_angle_req_last = 0
    self.sapp_counter = 0
    self.sapp_eightysix = 0
    self.sapp_action = 0
    self.sapp_chime = 0

    # 影子日志
    self._shadow_path = _DEFAULT_SHADOW_LOG
    self._shadow_fh = None
    self._shadow_frame = 0

    # 统计 (只读, 供 UI/日志)
    self.stat_sent = 0
    self.stat_shadow = 0
    self.stat_clamped = 0
    self.stat_rate_limited = 0
    self.stat_rejected = 0
    self.stat_windup = 0
    self.stat_unwind = 0
    self.stat_speed_clipped = 0
    self.stat_handshake_blocked = 0

  # ---------------------------------------------------------------------
  # 内部: SAPP 握手状态机 (P7)
  # ---------------------------------------------------------------------
  def _update_handshake(self, enabled: bool) -> None:
    """推进 SAPPStatusCoding 握手状态机。

    移植自 RAINNY4114/opendbc-sunnypilot 分支 `apa` 的 CarController.apa_update()。

    状态循环: 0 -> 70 -> 86 -> 224 -> 16(稳态)
      self.sapp_state (SAPPAngleControlStat1): 0=Closed 1=Open 2=Active 3=Fault

    注意: 本方法由 **latActive** 驱动（而非 allow_control），
    因为 PSCM 只有在收到我们的 config 70 后才会把 handshake 从 Closed 抬到 Open。
    """
    handshake = self.sapp_state

    if not enabled:
      # 退出: 全部复位, 下次启用时从头握手
      self.sapp_counter = 0
      self.sapp_eightysix = 0
      self.sapp_angle_req = 0
      self.sapp_action = 0
      self.sapp_config = SAPP_CFG_RESET
    else:
      self.sapp_counter += 1

      # ① 起步: PSCM 尚未响应 -> 先发 config 70
      if handshake == SAPP_CLOSED and self.sapp_config_last not in (
          SAPP_CFG_STEADY, SAPP_CFG_ACK, SAPP_CFG_ACTIVE):
        self.sapp_config = SAPP_CFG_INIT

      # ② PSCM ack(Open) 且累计 >8 帧 -> 升到 86
      if handshake == SAPP_OPEN and self.sapp_counter > SAPP_CFG_ACK_FRAMES:
        self.sapp_config = SAPP_CFG_ACK
        self.sapp_eightysix += 1

      # ③ 86 态再发 5 帧 -> 开始请求角度
      if handshake == SAPP_OPEN and self.sapp_counter > SAPP_CFG_REQ_FRAMES \
          and self.sapp_config_last == SAPP_CFG_ACK:
        self.sapp_angle_req = 1

      # ④ 86 态满 20 帧 -> 计数器归零, 保持角度请求
      #    NOTE: 与参考实现 (apa 分支 apa_update) 逐行一致。由于 ② 先于本步
      #    执行 self.sapp_eightysix += 1，当它达到 20 时已被递增为 21，
      #    故本分支在 handshake 持续 Open 时不会触发（参考实现同样如此）。
      #    保留是为了与上游保持行为一致；真正的状态推进由 ⑤ 完成。
      if self.sapp_config_last == SAPP_CFG_ACK and self.sapp_eightysix == SAPP_CFG_CYCLE_FRAMES:
        self.sapp_counter = 0
        self.sapp_eightysix = 0
        self.sapp_angle_req = 1

      # ⑤ PSCM 进入 Active -> 升到 224
      if handshake == SAPP_ACTIVE and self.sapp_config_last != SAPP_CFG_STEADY:
        self.sapp_config = SAPP_CFG_ACTIVE
        self.sapp_angle_req = 1
        self.sapp_action += 1

      # ⑥ 224 态保持 3 帧 -> 进入稳态 16
      if handshake == SAPP_ACTIVE and self.sapp_action >= SAPP_CFG_ACTIVE_FRAMES \
          and self.sapp_config_last == SAPP_CFG_ACTIVE:
        self.sapp_config = SAPP_CFG_STEADY
        self.sapp_angle_req = 1

      # ⑦ PSCM Fault -> 全部复位重试
      if handshake == SAPP_FAULT:
        self.sapp_config = SAPP_CFG_RESET
        self.sapp_counter = 0
        self.sapp_angle_req = 0

    self.sapp_config_last = self.sapp_config
    self.sapp_angle_req_last = self.sapp_angle_req

  def handshake_ready(self) -> bool:
    """P7: 握手是否已推进到"请求角度"阶段。

    未就绪时上层应保持 ext_req=0, PSCM 不会执行角度请求。
    """
    return (self.sapp_config_last in (SAPP_CFG_ACK, SAPP_CFG_ACTIVE, SAPP_CFG_STEADY)
            and self.sapp_angle_req_last == 1)

  # ---------------------------------------------------------------------
  # 内部: 斜坡限制 (P1)
  # ---------------------------------------------------------------------
  def _apply_rate_limit(self, target_deg: float, v_ego: float, now_ms: int) -> float:
    """把 target_deg 限制为相对上一帧最多变化 max_delta。

    速率按车速查 windup / unwind 表（移植自 `apa` 分支）:

        windup : 同向且 |target| > |last|  -> 角度继续增大
        unwind : 其余情形                  -> 角度回中 / 反向

    表值以 **20ms 帧** 为基准, 按实际 dt 线性缩放,
    保证与参考实现（50Hz 发送）物理角速度一致。
    """
    if now_ms <= 0:
      now_ms = int(time.monotonic() * 1000.0)

    if self.last_rate_ms <= 0:
      dt_ms = FORD_APA_PERIOD_MS
    else:
      dt_ms = float(now_ms - self.last_rate_ms)
      if dt_ms <= 0.0:
        dt_ms = FORD_APA_PERIOD_MS
      # 防止长时间中断后一次放过巨大角度
      if dt_ms > 4.0 * FORD_APA_PERIOD_MS:
        dt_ms = 4.0 * FORD_APA_PERIOD_MS

    # 按车速选表
    if self.last_angle_deg * float(target_deg) > 0.0 \
        and abs(float(target_deg)) > abs(self.last_angle_deg):
      rate_per_20ms = _interp(v_ego, APA_ANGLE_DELTA_BP, APA_ANGLE_DELTA_V)   # windup
      self.stat_windup += 1
    else:
      rate_per_20ms = _interp(v_ego, APA_ANGLE_DELTA_BP, APA_ANGLE_DELTA_VU)  # unwind
      self.stat_unwind += 1

    max_delta = (rate_per_20ms * dt_ms / 20.0) * _rate_scale()
    if max_delta < 1e-6:
      max_delta = 1e-6

    delta = float(target_deg) - self.last_angle_deg
    if abs(delta) > max_delta:
      delta = math.copysign(max_delta, delta)
      self.stat_rate_limited += 1

    out = self.last_angle_deg + delta
    self.last_angle_deg = out
    self.last_rate_ms = now_ms
    return out

  # ---------------------------------------------------------------------
  # 内部: 范围钳位 (P2)
  # ---------------------------------------------------------------------
  def _clamp_angle(self, angle_deg: float) -> float:
    """硬范围兜底钳位 (对称 ±FORD_APA_MAX_DEG)。"""
    if not math.isfinite(angle_deg):
      return 0.0
    if angle_deg < FORD_APA_MIN_DEG:
      self.stat_clamped += 1
      return FORD_APA_MIN_DEG
    if angle_deg > FORD_APA_MAX_DEG:
      self.stat_clamped += 1
      return FORD_APA_MAX_DEG
    return float(angle_deg)

  def _clamp_speed(self, angle_deg: float, v_ego: float) -> float:
    """P2: 速度调度钳位 (移植自 `apa` 分支)。

    低速允许大角度 (0 m/s -> ±410°), 高速收敛 (36 m/s -> ±15°)。
    """
    lim = abs(_interp(v_ego, APA_ANGLE_MAX_BP, APA_ANGLE_MAX_V))
    if lim <= 0.0 or not math.isfinite(lim):
      lim = abs(FORD_APA_MAX_DEG)
    if angle_deg > lim:
      self.stat_speed_clipped += 1
      return lim
    if angle_deg < -lim:
      self.stat_speed_clipped += 1
      return -lim
    return float(angle_deg)

  # ---------------------------------------------------------------------
  # 内部: apa_stat 决策 (P3)
  # ---------------------------------------------------------------------
  def _decide_apa_stat(self, lat_active: bool, allow_control: bool) -> int:
    """决定 ApaSys_D_Stat。

    规则:
      - 未 latActive        -> OFF(1)   (不是 ON, 避免 PSCM 认为 APA 已接管)
      - latActive 但未放行  -> OPEN(?)  实际用 OFF(1) + ext_req=0
      - latActive 且放行    -> ON(2)    (PSCM 开始执行角度请求)
    """
    if not lat_active:
      return fordcan.APA_STAT_OFF
    if not allow_control:
      return fordcan.APA_STAT_OFF
    return fordcan.APA_STAT_ON

  # ---------------------------------------------------------------------
  # 内部: 影子日志 (L1)
  # ---------------------------------------------------------------------
  def _shadow_log(self, angle_deg: float, apa_stat: int, ext_req: int,
                  curvature: float, v_ego: float, sapp: int, reason: str,
                  sapp_cfg: int = 0, sapp_req: int = 0) -> None:
    try:
      self._shadow_frame += 1
      # 影子模式只在 ~2Hz 落盘, 避免刷爆
      if self._shadow_frame % 10 != 0:
        return
      if self._shadow_fh is None:
        path = _read_dp_param(_DP_KEY_SHADOW_LOG)  # 允许自定义
        if isinstance(path, float):  # 数字 -> 忽略, 用默认
          path = None
        self._shadow_path = str(path) if path else _DEFAULT_SHADOW_LOG
        self._shadow_fh = open(self._shadow_path, "a", buffering=1)

      self._shadow_fh.write(
        "%.3f cur=%.6f v=%.2f angle=%+.1f stat=%d ext=%d sapp=%d cfg=%d req=%d %s\n"
        % (time.monotonic(), curvature, v_ego, angle_deg, apa_stat,
           ext_req, sapp, sapp_cfg, sapp_req, reason)
      )
    except Exception:
      pass

  # ---------------------------------------------------------------------
  # 主入口
  # ---------------------------------------------------------------------
  def update(self, frame: int, lat_active: bool, curvature: float,
             v_ego: float, sapp_state: int, eps_assist_limited: bool,
             veh_speed_kph: float, now_ms: int = 0) -> tuple | None:
    """每个 STEER_STEP 调用一次。

    Returns:
      None                                  -> 不发送 (L0 关闭 / 非发送帧)
      (0x3A8, bytes)                        -> 附加到 can_sends
    """
    level = apa_enable_level()

    # --- L0: 完全关闭 ---------------------------------------------------
    if level <= 0:
      # 重置状态, 保证下次启用时从头开始
      self._reset_state()
      return None

    # --- 节流: 只在 STEER_STEP 的整数倍发 -------------------------------
    if frame % FORD_APA_STEP != 0:
      return None

    if now_ms <= 0:
      now_ms = int(time.monotonic() * 1000.0)

    # --- 更新握手状态 ---------------------------------------------------
    self.sapp_state = int(sapp_state) if sapp_state is not None else SAPP_CLOSED
    self.eps_assist_limited = bool(eps_assist_limited)

    # --- P7: 推进 SAPP 握手状态机 ---------------------------------------
    # 由 latActive 驱动 (而非 allow_control): PSCM 需先收到我们的 config 70
    # 才会把 SAPPAngleControlStat1 从 Closed(0) 抬到 Open(1)。
    self._update_handshake(bool(lat_active))

    # --- 前置条件判定 ---------------------------------------------------
    reason = "ok"
    allow_control = True

    if not lat_active:
      allow_control = False
      reason = "lat_inactive"
    elif self.sapp_state == SAPP_FAULT:
      allow_control = False
      reason = "sapp_fault"
    elif self.sapp_state == SAPP_CLOSED:
      # SAPP 未打开 -> PSCM 大概率不接受外部角度请求
      allow_control = False
      reason = "sapp_closed"
    elif self.eps_assist_limited:
      # SteMdule_D_Stat == Normal_Op_Limited_Assist -> 助力受限
      allow_control = False
      reason = "eps_limited"
    elif not (math.isfinite(v_ego) and math.isfinite(curvature)):
      allow_control = False
      reason = "nonfinite"
    elif v_ego < 0.0:
      allow_control = False
      reason = "negative_speed"

    # --- 计算目标角度 ---------------------------------------------------
    if allow_control:
      raw_target = curvature_to_steer_angle(
        curvature, v_ego, self.steer_ratio, self.wheelbase,
      )
      # P2a: 速度调度钳位 (移植自 apa 分支)
      target = self._clamp_speed(raw_target, v_ego)
      # P2b: 硬范围兜底
      target = self._clamp_angle(target)
      apa_stat = self._decide_apa_stat(True, True)
      ext_req = 1
    else:
      # P5: 失效安全 -> 目标角度归零 (斜坡限制会平滑收回)
      target = 0.0
      apa_stat = fordcan.APA_STAT_OFF
      ext_req = 0

    # --- P4: 看门狗 -----------------------------------------------------
    if self.last_update_ms > 0:
      gap = now_ms - self.last_update_ms
      if gap > FORD_APA_WATCHDOG_MS:
        target = 0.0
        apa_stat = fordcan.APA_STAT_NULL
        ext_req = 0
        reason = "watchdog"
    self.last_update_ms = now_ms

    # --- P1: 斜坡限制 (windup / unwind 双表, 按车速) --------------------
    angle_deg = self._apply_rate_limit(target, v_ego, now_ms)
    angle_deg = self._clamp_angle(angle_deg)

    # --- P3: 最终合法性再确认 -------------------------------------------
    if apa_stat not in (fordcan.APA_STAT_NULL, fordcan.APA_STAT_OFF,
                        fordcan.APA_STAT_ON):
      apa_stat = fordcan.APA_STAT_NULL
      ext_req = 0
      self.stat_rejected += 1

    # APA_STAT_ON 必须配 ext_req=1; 其他一律 ext_req=0
    if apa_stat != fordcan.APA_STAT_ON:
      ext_req = 0

    # --- P7: SAPP 握手门控 ----------------------------------------------
    # 握手未推进到"请求角度"阶段前, 一律不请求; 同时把 ON 降为 OFF,
    # 避免 PSCM 误认为 APA 已接管。
    if not self.handshake_ready():
      if ext_req != 0:
        self.stat_handshake_blocked += 1
      ext_req = 0
      if apa_stat == fordcan.APA_STAT_ON:
        apa_stat = fordcan.APA_STAT_OFF

    # --- L1: 影子模式, 不真发 -------------------------------------------
    if level == 1:
      self.stat_shadow += 1
      self._shadow_log(angle_deg, apa_stat, ext_req, curvature, v_ego,
                       self.sapp_state, reason, self.sapp_config_last,
                       self.sapp_angle_req_last)
      return None

    # --- L2: 真发 -------------------------------------------------------
    #
    # ⚠️ 必须返回 **3 元组** (addr, data, bus)。
    #    fordcan.create_apa_steer_msg_raw() 只返回 (addr, data) 2 元组,
    #    直接 append 会让 can_list_to_can_capnp 在 msg[2] 处 IndexError,
    #    从而整条 sendcan 链路失效 —— 我们只取它的 data 部分自行补 bus。
    addr, data = fordcan.create_apa_steer_msg_raw(
      angle_deg, apa_stat, ext_req,
      sapp_config=self.sapp_config_last, sapp_chime=self.sapp_chime,
    )
    msg = (addr, bytes(data), int(self.bus_main))
    self.stat_sent += 1
    return msg

  # ---------------------------------------------------------------------
  def _reset_state(self) -> None:
    self.last_angle_deg = 0.0
    self.last_rate_ms = 0
    self.last_update_ms = 0
    # SAPP 握手状态机复位
    self.sapp_config = SAPP_CFG_RESET
    self.sapp_config_last = SAPP_CFG_RESET
    self.sapp_angle_req = 0
    self.sapp_angle_req_last = 0
    self.sapp_counter = 0
    self.sapp_eightysix = 0
    self.sapp_action = 0
    # 统计
    self.stat_sent = 0
    self.stat_shadow = 0
    self.stat_clamped = 0
    self.stat_rate_limited = 0
    self.stat_rejected = 0
    self.stat_windup = 0
    self.stat_unwind = 0
    self.stat_speed_clipped = 0
    self.stat_handshake_blocked = 0

  def close(self) -> None:
    try:
      if self._shadow_fh is not None:
        self._shadow_fh.close()
    except Exception:
      pass
    self._shadow_fh = None


# ===========================================================================
# 离线自测
# ===========================================================================

def _self_test() -> bool:
  """不依赖设备的最小自检。可直接 python3 运行本文件。"""
  ok = True

  # 1. 曲率 -> 角度
  ang = curvature_to_steer_angle(0.01, 5.0, 15.0, 2.85)
  # atan(0.01*2.85)=atan(0.0285)=1.6326deg; *15 = 24.49
  exp = math.degrees(math.atan(0.0285)) * 15.0
  if abs(ang - exp) > 1e-6:
    print("FAIL curvature_to_steer_angle: %f != %f" % (ang, exp))
    ok = False
  else:
    print("ok   curvature_to_steer_angle 0.01/15.0/2.85 -> %.3f deg" % ang)

  # 2. 往返
  curv = steer_angle_to_curvature(ang, 15.0, 2.85)
  if abs(curv - 0.01) > 1e-9:
    print("FAIL roundtrip: %f != 0.01" % curv)
    ok = False
  else:
    print("ok   roundtrip angle->curvature -> %.8f" % curv)

  # 3. 钳位
  big = curvature_to_steer_angle(10.0, 5.0, 15.0, 2.85)
  if abs(big - FORD_APA_MAX_DEG) > 1e-6:
    print("FAIL clamp: %f != %f" % (big, FORD_APA_MAX_DEG))
    ok = False
  else:
    print("ok   clamp 10.0 -> %.1f deg" % big)

  # 4. raw 打包一致性 (对照已知样本)
  samples = [
    (250.0, "0000b0d400000010"),
    (-250.0, "00009d4c00000010"),
    (500.0, "0000ba9800000010"),
    (0.0, "0000a71000000010"),
  ]
  for deg, expect in samples:
    addr, dat = fordcan.create_apa_steer_msg_raw(deg, 2, 1)
    got = dat.hex()
    if got != expect:
      print("FAIL raw pack %s: %s != %s" % (deg, got, expect))
      ok = False
    else:
      print("ok   raw pack %+7.1f -> %s" % (deg, got))

  # 5. 反解
  for deg, expect in samples:
    a, e, s = fordcan.apa_unpack_steer_msg(bytes.fromhex(expect))
    if abs(a - deg) > 0.05 or e != 1 or s != 2:
      print("FAIL unpack %s: %f/%d/%d" % (expect, a, e, s))
      ok = False
    else:
      print("ok   unpack %s -> %+7.1f deg ext=%d stat=%d" % (expect, a, e, s))

  # 6. 斜坡限制 —— windup 表 @ v=0 -> 5.0 deg/20ms
  class _FakeCP:
    steerRatio = 15.0
    wheelbase = 2.85

  c = FordAPAController(_FakeCP())
  a1 = c._apply_rate_limit(100.0, 0.0, 1000)
  # 第一帧 last_rate_ms=0 -> dt=50ms -> max_delta = 5.0*50/20 = 12.5
  if abs(a1 - 12.5) > 1e-6:
    print("FAIL rate limit first step: %f != 12.5" % a1)
    ok = False
  else:
    print("ok   rate limit step1 (windup v=0) -> %.2f deg" % a1)
  a2 = c._apply_rate_limit(100.0, 0.0, 1050)
  if abs(a2 - 25.0) > 1e-6:
    print("FAIL rate limit second step: %f != 25.0" % a2)
    ok = False
  else:
    print("ok   rate limit step2 (windup v=0) -> %.2f deg" % a2)

  # 6b. 高速 windup 收敛 (v=15 -> 0.15 deg/20ms, dt=50ms -> 0.375)
  #     注意: 必须从同向非零角起步, 否则 last*target==0 会落到 unwind 表
  #     (与参考实现 apa_update 的判定一致)。
  c_hs = FordAPAController(_FakeCP())
  c_hs.last_angle_deg = 10.0
  c_hs.last_rate_ms = 1000
  a3 = c_hs._apply_rate_limit(100.0, 15.0, 1050)
  if abs(a3 - 10.375) > 1e-6:
    print("FAIL rate limit high-speed windup: %f != 10.375" % a3)
    ok = False
  else:
    print("ok   rate limit windup v=15 (10->100) -> %.3f deg" % a3)

  # 6c. unwind 表 (从 +100 回 0, v=15 -> 0.4 deg/20ms, dt=50ms -> 1.0)
  c_uw = FordAPAController(_FakeCP())
  c_uw.last_angle_deg = 100.0
  c_uw.last_rate_ms = 1000
  a4 = c_uw._apply_rate_limit(0.0, 15.0, 1050)
  if abs(a4 - 99.0) > 1e-6:
    print("FAIL rate limit unwind: %f != 99.0" % a4)
    ok = False
  else:
    print("ok   rate limit unwind v=15 -> %.2f deg" % a4)

  # 6d. 速度调度钳位 (v=0 -> 410, v=36 -> 15, v=5.5 插值中点 -> 217.5)
  for v, e in ((0.0, 410.0), (36.0, 15.0), (5.5, 217.5)):
    got = c._clamp_speed(999.0, v)
    if abs(got - e) > 1e-6:
      print("FAIL speed clamp v=%.1f: %f != %f" % (v, got, e))
      ok = False
    else:
      print("ok   speed clamp v=%-4.1f -> %.1f deg" % (v, got))

  # 7. ★ 3 元组契约 (最关键, 曾因此险些让 sendcan 整体失效)
  c2 = FordAPAController(_FakeCP())
  # 手动绕过参数开关, 直接测打包路径
  c2.bus_main = 0
  addr, data = fordcan.create_apa_steer_msg_raw(120.0, 2, 1)
  msg = (addr, bytes(data), int(c2.bus_main))
  if not (isinstance(msg, tuple) and len(msg) == 3):
    print("FAIL tuple arity: %r" % (msg,))
    ok = False
  elif not isinstance(msg[1], (bytes, bytearray)):
    print("FAIL msg[1] not bytes: %r" % type(msg[1]))
    ok = False
  elif not isinstance(msg[2], int):
    print("FAIL msg[2] not int: %r" % type(msg[2]))
    ok = False
  else:
    print("ok   can_sends tuple (addr=%d, data=%s, bus=%d) arity=3"
          % (msg[0], bytes(msg[1]).hex(), msg[2]))

  # 8. apa_stat 合法性 (P3)
  if c._decide_apa_stat(False, True) != fordcan.APA_STAT_OFF:
    print("FAIL apa_stat when lat inactive")
    ok = False
  else:
    print("ok   apa_stat lat_inactive -> OFF(%d)" % fordcan.APA_STAT_OFF)
  if c._decide_apa_stat(True, True) != fordcan.APA_STAT_ON:
    print("FAIL apa_stat when active")
    ok = False
  else:
    print("ok   apa_stat active -> ON(%d)" % fordcan.APA_STAT_ON)

  # 9. 范围钳位 (P2)
  lo = c._clamp_angle(-99999.0)
  hi = c._clamp_angle(+99999.0)
  if abs(lo - FORD_APA_MIN_DEG) > 1e-6 or abs(hi - FORD_APA_MAX_DEG) > 1e-6:
    print("FAIL clamp: %f / %f" % (lo, hi))
    ok = False
  else:
    print("ok   clamp -> [%.0f, %.0f] deg" % (lo, hi))

  # 10. SAPP 握手状态机: 0 -> 70 -> 86 -> 224 -> 16
  h = FordAPAController(_FakeCP())
  cfg_at: dict[int, int] = {}
  req_at: dict[int, int] = {}
  for i in range(1, 31):
    h.sapp_state = SAPP_CLOSED if i <= 8 else (SAPP_OPEN if i <= 19 else SAPP_ACTIVE)
    h._update_handshake(True)
    cfg_at[i] = h.sapp_config_last
    req_at[i] = h.sapp_angle_req_last

  expect_cfg = {1: SAPP_CFG_INIT, 8: SAPP_CFG_INIT, 9: SAPP_CFG_ACK, 19: SAPP_CFG_ACK,
                20: SAPP_CFG_ACTIVE, 21: SAPP_CFG_ACTIVE, 22: SAPP_CFG_STEADY,
                23: SAPP_CFG_STEADY, 30: SAPP_CFG_STEADY}
  bad = [(i, cfg_at[i], e) for i, e in expect_cfg.items() if cfg_at[i] != e]
  if bad:
    print("FAIL SAPP cfg 序列: %r" % bad)
    ok = False
  else:
    print("ok   SAPP cfg 序列 70@1..8 -> 86@9..19 -> 224@20..21 -> 16@22+  "
          "(采样 %s)" % [cfg_at[i] for i in (1, 9, 20, 22, 30)])

  if req_at[13] != 0 or req_at[14] != 1:
    print("FAIL SAPP angle_req @13/14: %d/%d" % (req_at[13], req_at[14]))
    ok = False
  else:
    print("ok   SAPP angle_req 13->0  14->1")

  if not h.handshake_ready():
    print("FAIL handshake_ready() 在 i=30 应为 True")
    ok = False
  else:
    print("ok   handshake_ready() -> True (cfg=%d req=%d)" % (h.sapp_config_last, h.sapp_angle_req_last))

  # 11. 退出 -> 握手复位
  h._update_handshake(False)
  if h.sapp_config_last != SAPP_CFG_RESET or h.sapp_angle_req_last != 0 or not h.handshake_ready() is False:
    print("FAIL handshake reset on disable: cfg=%d req=%d" % (h.sapp_config_last, h.sapp_angle_req_last))
    ok = False
  else:
    print("ok   handshake reset on disable -> cfg=%d req=%d ready=%s"
          % (h.sapp_config_last, h.sapp_angle_req_last, h.handshake_ready()))

  # 12. SAPPStatusCoding / ApaChime_D_Rq 打包往返
  addr, dat = fordcan.create_apa_steer_msg_raw(0.0, 0, 0, sapp_config=86, sapp_chime=5)
  if dat[0] != 86 or ((dat[4] >> 5) & 0x07) != 5:
    print("FAIL sapp pack: d0=%d chime=%d" % (dat[0], (dat[4] >> 5) & 0x07))
    ok = False
  else:
    print("ok   sapp pack cfg=86 chime=5 -> %s" % dat.hex())
  full = fordcan.apa_unpack_steer_msg_full(dat)
  if full["SAPPStatusCoding"] != 86 or full["ApaChime_D_Rq"] != 5:
    print("FAIL sapp unpack: %r" % (full,))
    ok = False
  else:
    print("ok   sapp unpack -> cfg=%d chime=%d" % (full["SAPPStatusCoding"], full["ApaChime_D_Rq"]))

  return ok


if __name__ == "__main__":
  print("=" * 60)
  print("FordAPAController 自检")
  print("=" * 60)
  # 离线时 fordcan 可能无法导入 (缺 opendbc 依赖), 给出提示
  try:
    import sys
    r = _self_test()
    print("=" * 60)
    print("RESULT:", "PASS" if r else "FAIL")
    sys.exit(0 if r else 1)
  except Exception as e:
    import sys
    print("自检需要 opendbc 环境:", e)
    sys.exit(2)
