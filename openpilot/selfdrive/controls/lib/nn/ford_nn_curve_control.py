"""
ford_nn_curve_control.py —— NN 弯道控制运行时

这是神经网络真正接入福特/林肯控制链的地方。它只做三件事：

  1. **横向前馈残差**：给 `FordAngleControl` 的 `curvature_factor` 一个乘性
     修正 `(1 + d_gain)`，给 `FordCurveController` 的 VLT `extra_lookahead`
     一个加性修正 `d_vlt`。

     **注入点必须是 `FordAngleControl`，不是 `FordCurveController`。**
     上一轮查明：`carcontroller.py` 只下发 `FordAngleControl.update()` 返回的
     `path_angle`，`FordCurveController` 自己算的 `path_angle` 历史上写死 0.
     从不下发。把 `d_gain` 打到后者身上是无效的（很容易搞错）。
  2. **纵向弯道预算**：给纵向规划器一个横向加速度预算 `a_lat_budget`，
     规划器据此把 `v_target = sqrt(a_lat_budget / |kappa|)` 作为**约束**
     （这正是 ford 分支 DTSC 的思路，上一轮列为"未实施"，这里补上）。
  3. **驾驶风格**：把 `DriverStyle` 的当前风格向量喂进上面两个模型的输入。

四道独立安全边界
---------------------------------------------------------------------------
NN 的输出**永远不是执行器指令**，它只是一组有界残差。四道边界依次收口：

  第 1 道（本文件 `_clamp_residual`）：硬限幅。d_gain ∈ [-0.30, +0.30]，
        d_vlt ∈ [-0.50, +0.50]，a_lat_budget ∈ [1.20, 3.50]。
  第 2 道（本文件 `_rate_limit`）：变化率限幅。防止 NN 逐帧抖动直接变成
        方向盘抖动 —— 这是"NN 引入后舒适性反而变差"的最常见原因。
  第 3 道（本文件 `_TrustMonitor`）：饱和监控。若 NN 长期顶在限幅边界上，
        说明当前工况超出训练分布，自动降级到 shadow 模式。
  第 4 道（panda 固件 `safety/modes/ford.h`，见上一轮补丁）：量程分级、
        ROC 限幅、影子曲率邻近性、ISO 11270 横向加速度上限、下发频率护栏。

**这四道互相独立**：任何一道单独失效都不会让车进入不可接受的状态。
第 4 道在 panda 里、独立于主控进程，是最硬的一道 —— 即使 controlsd 完全失控，
车也不会被推出物理可行域。

降级策略（默认安全）
---------------------------------------------------------------------------
* 权重文件缺失 / 格式不符 / 维度不匹配 → `enabled = False`，输出恒 0，
  行为**逐位**等于打补丁前。
* 输入出现 NaN/Inf → 本帧输出 0，故障计数 +1；连续 `max_consecutive_faults`
  次则整体禁用，直到进程重启（或把 mode 从 0 切回来，见下）。
* 模型推理抛异常 → 同上。
* 配置文件缺失 / 语法错 / 权限不足 → 保留现有配置，静默继续。

配置文件与热加载
---------------------------------------------------------------------------
配置读 `/data/ford_nn_curve.json`（普通 JSON，不用 openpilot 的 Params()，
理由见 DEFAULTS 下方的注释）。每 `CONFIG_REFRESH_SEC`（2 s）检查一次 mtime，
变了就重读 —— 于是**改 mode 不需要重启进程**：

    echo '{"mode": 1}' > /data/ford_nn_curve.json     # 立刻切到影子模式

mode 从 0 切到非 0 时会补做一次权重加载，并复位 trust monitor 的降级状态，
否则一旦被降级就再也起不来了。
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from typing import Any, Mapping, Sequence

import numpy as np

# 双兼容导入：既支持作为包（部署到 openpilot/selfdrive/controls/lib/nn/），
# 也支持平铺在 sys.path 上（离线训练与单元测试）。
try:  # pragma: no cover - 取决于部署形态
  from . import ford_nn_features as F
  from .ford_driver_style import DriverStyle, STYLE_DIM
  from .ford_nn_policy import PolicyBundle
except ImportError:  # pragma: no cover
  import ford_nn_features as F
  from ford_driver_style import DriverStyle, STYLE_DIM
  from ford_nn_policy import PolicyBundle

# ---------------------------------------------------------------------------
# 运行模式（与上一轮 safety 的 L0-L3 保持同一套语义，便于运维理解）
# ---------------------------------------------------------------------------

MODE_OFF = 0      # 完全关闭：输出恒 0
MODE_SHADOW = 1   # 影子：照常计算并写日志，但输出恒 0
MODE_LATERAL = 2  # 横向生效（前馈残差），纵向不生效
MODE_FULL = 3     # 横向 + 纵向都生效

MODE_NAMES = {
  MODE_OFF: "off",
  MODE_SHADOW: "shadow",
  MODE_LATERAL: "lateral",
  MODE_FULL: "full",
}

# ---------------------------------------------------------------------------
# 默认配置
# ---------------------------------------------------------------------------

DEFAULTS: dict[str, Any] = {
  "mode": MODE_OFF,
  "weights": "/data/ford_nn/policy.npz",
  "style_persist": "/data/ford_nn/style.json",

  # 硬限幅（第 1 道边界）
  "max_d_gain": 0.30,
  "max_d_vlt": 0.50,
  "alat_budget_min": 1.20,
  "alat_budget_max": 3.50,
  "alat_iso_limit": 3.60,        # ISO 11270 + 路面横坡，与 panda 侧一致

  # 变化率限幅（第 2 道边界）
  "max_d_gain_rate": 0.60,       # 1/s
  "max_d_vlt_rate": 1.00,        # 1/s
  "max_alat_rate": 1.00,         # (m/s^2)/s

  # 信任监控（第 3 道边界）
  "saturation_ratio": 0.90,      # 占限幅的比例，超过视为饱和
  "saturation_window_s": 60.0,
  "saturation_fraction": 0.80,   # 窗口内饱和帧占比超过此值则降级
  "min_speed_ms": 3.0,           # 低于此车速不启用（泊车/蠕行无意义）
  "max_consecutive_faults": 20,

  # 风格自适应
  "style_tau_s": 1800.0,
  "style_trust_radius": 0.40,
  "style_enabled": True,         # 是否在**本进程**里学习（见下方进程分工说明）
  "style_save_period_s": 0.0,    # >0 时周期性写盘；0 = 从不写盘
  "style_reload_period_s": 0.0,  # >0 时周期性从盘上重读（只读实例用）

  # 诊断
  "log_period_s": 5.0,
}

# 配置文件路径。沿用第一阶段 ford_angle_control / narrow_road_control 的做法：
# 用普通 JSON 而不是 openpilot 的 Params() —— 因为 Params::clearAll() 会删掉
# 所有不在**已编译** params_keys.h 里的 key（本机 libparams_c.so 是预编译的，
# 无法在设备上重建），新增 key 一定会在 manager 启动/点火时被清掉。
CONFIG_PATH = "/data/ford_nn_curve.json"
CONFIG_REFRESH_SEC = 2.0

# ---------------------------------------------------------------------------
# 两个进程的分工（重要，改之前先读完）
# ---------------------------------------------------------------------------
# carcontroller 跑在 **card** 进程，longitudinal_planner 跑在 **controlsd**
# 进程。两者**不能共享 Python 对象**，所以 NN 实例必须各持一份：
#
#   card 侧（横向，权威）
#       FordNNCurveControl(CP)                     # 默认配置
#       -> style_enabled=True, style_save_period_s=300
#          它是风格的**唯一写者**：定期把学到的风格落盘。
#
#   controlsd 侧（纵向，从属）
#       FordNNCurveControl(CP, config={"style_enabled": False,
#                                      "style_reload_period_s": 30.0})
#       -> 不学习、不写盘，只定期把 card 侧写好的风格读回来用。
#          这样避免两个进程同时写同一个 style.json 造成互相覆盖。
#
# 权重文件（policy.npz）是**只读共享**，两边各自加载一份即可。
# ---------------------------------------------------------------------------


class _TrustMonitor:
  """第 3 道边界：饱和监控。

  思路：NN 若长期顶在限幅边界，说明它想输出比允许值更大的修正 ——
  这几乎总是"当前工况超出训练分布"的信号。此时最安全的做法不是让它
  继续顶格输出，而是降级回基线。
  """

  def __init__(self, window_s: float, saturation_ratio: float,
               saturation_fraction: float) -> None:
    self.window_s = float(window_s)
    self.sat_ratio = float(saturation_ratio)
    self.sat_fraction = float(saturation_fraction)
    self._buf: deque[tuple[float, bool]] = deque()
    self.tripped = False

  def reset(self) -> None:
    self._buf.clear()
    self.tripped = False

  def push(self, t: float, residual: np.ndarray, limits: np.ndarray) -> None:
    """记录一帧是否饱和。

    Args:
      t:        时间戳 (s)。
      residual: 本帧残差。
      limits:   对应的限幅半径（与 residual 同形）。
    """
    with np.errstate(divide="ignore", invalid="ignore"):
      frac = np.abs(residual) / np.maximum(limits, 1e-9)
    saturated = bool(np.any(frac >= self.sat_ratio))
    self._buf.append((t, saturated))

    cutoff = t - self.window_s
    while self._buf and self._buf[0][0] < cutoff:
      self._buf.popleft()

    if len(self._buf) >= 50:
      frac_sat = sum(1 for _, s in self._buf if s) / len(self._buf)
      if frac_sat >= self.sat_fraction:
        self.tripped = True

  @property
  def saturation_fraction(self) -> float:
    if not self._buf:
      return 0.0
    return sum(1 for _, s in self._buf if s) / len(self._buf)


class FordNNCurveControl:
  """NN 弯道控制器。

  Usage (in carcontroller.py)::

      self.nn_curve = FordNNCurveControl(CP)
      ...
      out = self.nn_curve.update(
          t=self.frame * 0.05,
          kappa=float(apply_curvature),
          v_ego=v_ego,
          steering_angle_deg=steering_angle_deg,
          path_angle_last=self.apply_path_angle_last,
          yaw_rate=float(CS.out.yawRate),
          lateral_active=bool(CC.latActive),
          a_ego=float(CS.out.aEgo),
      )
      self.angle_controller.set_nn_residual(out["d_gain"])   # 乘性 (1 + d_gain)
      self.curve_controller.set_nn_residual(out["d_vlt"])    # 加性

  Usage (in longitudinal_planner.py, controlsd 进程 —— 独立实例)::

      self._nn_curve = FordNNCurveControl(CP, config={
          "style_enabled": False,        # 不学习，只读 card 侧写好的风格
          "style_reload_period_s": 30.0, # 定期重读
      })
      ...
      kh = F.sample_curvature_horizon(sm['modelV2'].position.x,
                                      sm['modelV2'].orientationRate.z,
                                      sm['modelV2'].velocity.x, v_ego)
      out = self._nn_curve.update(t, kappa=float(kh[0]), v_ego=v_ego,
                                  kappa_horizon=kh, ...)
      v_cruise = min(v_cruise, self._nn_curve.budget_to_speed(
          out["a_lat_budget"], float(np.max(np.abs(kh)))))
  """

  def __init__(self, CP=None, config: Mapping[str, Any] | None = None,
               weights_path: str | None = None,
               style: DriverStyle | None = None) -> None:
    self.CP = CP
    self.cfg: dict[str, Any] = dict(DEFAULTS)
    if config:
      self.cfg.update(dict(config))

    # 车辆几何：`steer_lag` 特征的量纲由它们决定（见
    # `ford_nn_features.FEATURE_VERSION` 的 v3 说明）。取不到就回落常量，
    # **不能**用 1.0 —— 那会让 steer_lag 差 16.8 倍。
    self.steer_ratio = F.STEER_RATIO_DEFAULT
    self.wheelbase = F.WHEELBASE_DEFAULT
    try:
      if CP is not None:
        self.steer_ratio = float(getattr(CP, "steerRatio", 0.0) or 0.0) or F.STEER_RATIO_DEFAULT
        self.wheelbase = float(getattr(CP, "wheelbase", 0.0) or 0.0) or F.WHEELBASE_DEFAULT
    except Exception:  # noqa: BLE001
      self.steer_ratio = F.STEER_RATIO_DEFAULT
      self.wheelbase = F.WHEELBASE_DEFAULT

    self._cfg_mtime: float | None = None
    self._last_cfg_check = 0.0
    self._load_config(force=True)

    self.mode = int(self.cfg["mode"])
    self.weights_path = weights_path or str(self.cfg["weights"])

    # --- 模型 ---
    self.bundle: PolicyBundle | None = None
    self.load_error = ""
    self._load_weights()

    # --- 风格 ---
    self.style = style or DriverStyle(
      tau_s=float(self.cfg["style_tau_s"]),
      trust_radius=float(self.cfg["style_trust_radius"]),
      persist_path=str(self.cfg["style_persist"]),
    )
    # 无论本进程是否学习，都先尝试从盘上恢复 —— 只读实例（controlsd 侧）
    # 也必须拿到 card 侧已经学好的风格。载入失败（无文件/被篡改）时
    # DriverStyle.load() 返回 False 并保持出厂标定值，不需要额外处理。
    try:
      self.style.load()
    except Exception:  # noqa: BLE001 - 风格载入失败绝不允许影响控制
      pass

    self._last_style_save = 0.0
    self._last_style_reload = 0.0

    # --- 状态 ---
    self.kappa_hist = F.KappaHistory(4)
    self._a_lat_filt = 0.0
    self._t_prev: float | None = None
    self._a_lat_prev: float | None = None
    self._last_kappa: float | None = None
    self._kappa_rate = 0.0

    # 上一帧残差（用于变化率限幅）
    self._prev = np.zeros(3, dtype=np.float64)   # [d_gain, d_vlt, d_alat]

    self._monitor = _TrustMonitor(
      float(self.cfg["saturation_window_s"]),
      float(self.cfg["saturation_ratio"]),
      float(self.cfg["saturation_fraction"]),
    )

    self.consecutive_faults = 0
    self.disabled = False
    self.disabled_reason = ""

    self._last_log_t = -1e9
    self.n_frames = 0
    self.n_faults = 0

    # 最近一次输出（供外部诊断读取）
    self.last_output: dict[str, Any] = self._zero_output("init")

  # -------------------------------------------------------------------------
  # 配置热加载
  # -------------------------------------------------------------------------

  def _load_config(self, force: bool = False) -> None:
    """从 CONFIG_PATH 读配置并合并进 self.cfg。

    热生效的意义：不用重启进程就能把 mode 从 0 切到 1（影子）观察日志，
    这对"分阶段上线"是刚需 —— 每次改 mode 都要停车重启进程是不可接受的。
    读取失败（无文件 / JSON 语法错 / 权限）时**静默保留现有配置**，
    绝不因为配置文件问题影响控制环。
    """
    now = time.monotonic()
    if not force and (now - self._last_cfg_check) < CONFIG_REFRESH_SEC:
      return
    self._last_cfg_check = now
    try:
      if not os.path.exists(CONFIG_PATH):
        return
      m = os.path.getmtime(CONFIG_PATH)
      if not force and m == self._cfg_mtime:
        return
      with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        user_cfg = json.load(f)
      if not isinstance(user_cfg, dict):
        return
      # 只接受 DEFAULTS 里已声明的键，防止配置文件写错键名后
      # 在别处 .cfg[...] 取到 KeyError
      for k, v in user_cfg.items():
        if k in DEFAULTS:
          self.cfg[k] = v
      self._cfg_mtime = m
    except Exception:  # noqa: BLE001
      return

  def _maybe_refresh_mode(self) -> None:
    """配置刷新后同步 mode；从 0 切到非 0 时补做一次权重加载。"""
    new_mode = int(self.cfg["mode"])
    if new_mode == self.mode:
      return
    was_off = self.mode == MODE_OFF
    self.mode = new_mode
    if was_off and self.mode != MODE_OFF and self.bundle is None:
      self.load_error = ""
      self._load_weights()
    # mode 变化意味着"人为干预"，把故障禁用状态一并复位，
    # 否则一旦被 trust monitor 降级就再也起不来了。
    self.disabled = False
    self.disabled_reason = ""
    self.consecutive_faults = 0
    self._monitor.reset()
    self._prev[:] = 0.0

  def _maybe_persist_style(self, t: float) -> None:
    """风格的周期性写盘 / 重读。

    两个开关默认都是 0（关闭），因此默认行为与不加这段代码完全一致：

      style_save_period_s > 0    本实例是风格的**写者**（card 侧）
      style_reload_period_s > 0  本实例是风格的**读者**（controlsd 侧）

    为什么不做成"谁都能写"：card 与 controlsd 是两个进程，同时写同一个
    JSON 会互相截断（open() 的 'w' 模式先清空再写），结果是文件里出现
    半截 JSON。用"单写者 + 多读者"从根上避免这个竞态，代价是 controlsd
    侧的风格最多滞后 `style_reload_period_s` 秒 —— 对 τ=1800 s 的慢变量
    完全无所谓。
    """
    save_p = float(self.cfg.get("style_save_period_s", 0.0) or 0.0)
    if save_p > 0.0 and (t - self._last_style_save) >= save_p:
      self._last_style_save = t
      try:
        self.style.save()
      except Exception:  # noqa: BLE001 - 写盘失败不影响控制
        pass

    reload_p = float(self.cfg.get("style_reload_period_s", 0.0) or 0.0)
    if reload_p > 0.0 and (t - self._last_style_reload) >= reload_p:
      self._last_style_reload = t
      try:
        # load() 内部**同样施加信任域**，即使盘上文件被篡改，
        # 读回来的风格也无法越出 s0 ± TRUST_RADIUS。
        self.style.load()
      except Exception:  # noqa: BLE001
        pass

  # -------------------------------------------------------------------------
  # 加载
  # -------------------------------------------------------------------------

  def _load_weights(self) -> None:
    if self.mode == MODE_OFF:
      return
    path = self.weights_path
    if not path or not os.path.exists(path):
      self.load_error = f"weights not found: {path}"
      return
    try:
      b = PolicyBundle.load(path)
      if not b.ready:
        self.load_error = "weights incomplete (need both lateral and longitudinal)"
        return
      if b.lateral_ff.in_dim != F.LATERAL_FF_DIM:
        self.load_error = (f"lateral in_dim mismatch: {b.lateral_ff.in_dim} "
                           f"!= {F.LATERAL_FF_DIM} (feature version drift?)")
        return
      if b.curve_speed.in_dim != F.CURVE_SPEED_DIM:
        self.load_error = (f"curve_speed in_dim mismatch: {b.curve_speed.in_dim} "
                           f"!= {F.CURVE_SPEED_DIM} (feature version drift?)")
        return
      self.bundle = b
    except Exception as e:  # noqa: BLE001 - 任何加载异常都必须降级，不能抛到控制环
      self.load_error = f"{type(e).__name__}: {e}"

  @property
  def enabled(self) -> bool:
    """是否真正生效（模式 > OFF 且模型可用且未因故障禁用）。"""
    return (self.mode != MODE_OFF
            and self.bundle is not None
            and not self.disabled)

  @property
  def lateral_active(self) -> bool:
    return self.enabled and self.mode >= MODE_LATERAL

  @property
  def longitudinal_active(self) -> bool:
    return self.enabled and self.mode >= MODE_FULL

  # -------------------------------------------------------------------------
  # 主入口
  # -------------------------------------------------------------------------

  def update(self, t: float, *,
             kappa: float,
             v_ego: float,
             steering_angle_deg: float = 0.0,
             path_angle_last: float = 0.0,
             yaw_rate: float = 0.0,
             lateral_active: bool = True,
             a_ego: float = 0.0,
             follow_gap_s: float = 0.0,
             gas: float = 0.0,
             speed_limit_ratio: float = 0.0,
             override: bool = False,
             esp_active: bool = False,
             kappa_horizon: Sequence[float] | None = None) -> dict[str, Any]:
    """每个控制周期调用一次。

    Args:
      t:                  单调时间戳 (s)。
      kappa:              当前规划曲率 (1/m)。
      v_ego:              车速 (m/s)。
      steering_angle_deg: 实测方向盘转角 (deg)，**车体约定**（左转为负）。
      path_angle_last:    上一帧下发的 path_angle (rad)，**openpilot 约定**
                          （左转为正）。两者约定不同，`steer_lag` 里会做统一；
                          传反了会让该特征恒等于"两者绝对值相加"。
      yaw_rate:           实测横摆角速度 (rad/s)。
      lateral_active:     openpilot 横向是否在控。
      a_ego:              实测纵向加速度 (m/s^2)。
      follow_gap_s:       跟车时距 (s)。
      gas:                油门开度 [0,1]。
      speed_limit_ratio:  超速比例。
      override:           本帧是否人工接管。
      esp_active:         ESP/ABS 介入。
      kappa_horizon:      前瞻曲率序列（长度 CURVE_HORIZON_N），None 则用当前
                          曲率恒定外推（精度差但不会失败）。

    Returns:
      dict，键：
        d_gain        横向前馈增益修正（乘性，1 + d_gain 作用于 curvature_factor）
        d_vlt         VLT extra_lookahead 修正（加性）
        a_lat_budget  纵向横向加速度预算 (m/s^2)，None 表示纵向未启用
        engaged       横向残差是否真正生效
        mode / reason 诊断
    """
    self.n_frames += 1

    # --- 配置热加载：必须在 mode 判断**之前**，这样把 mode 从 0 改成 1
    #     无需停车重启进程就能生效（分阶段上线的刚需）。---
    self._load_config()
    self._maybe_refresh_mode()

    # --- 模式 OFF：立即返回，且不推进任何控制状态（保证可复现） ---
    if self.mode == MODE_OFF:
      self.last_output = self._zero_output("mode_off")
      return self.last_output

    t = float(t)
    dt = 0.0 if self._t_prev is None else max(t - self._t_prev, 0.0)
    self._t_prev = t

    # --- 风格自适应（即使 NN 故障也继续学习，便于故障恢复后立即生效） ---
    if self.cfg.get("style_enabled", True):
      self.style.update(t, {
        "a_lat": float(kappa) * max(float(v_ego), 0.0) ** 2,
        "a_long": float(a_ego),
        "v_ego": float(v_ego),
        "kappa": float(kappa),
        "follow_gap_s": float(follow_gap_s),
        "gas": float(gas),
        "speed_limit_ratio": float(speed_limit_ratio),
        "override": bool(override),
        "esp_active": bool(esp_active),
        "engaged": bool(lateral_active),
      })
    self._maybe_persist_style(t)

    # --- 硬前置条件：车速太低、模型不可用、已禁用 → 输出 0 ---
    if not self.enabled:
      reason = self.disabled_reason or self.load_error or "not_ready"
      self.last_output = self._zero_output(reason)
      return self.last_output

    if abs(float(v_ego)) < float(self.cfg["min_speed_ms"]):
      self.last_output = self._zero_output("low_speed")
      return self.last_output

    # --- 特征与推理 ---
    try:
      residual = self._infer(t, dt, kappa, v_ego, steering_angle_deg,
                             path_angle_last, yaw_rate, kappa_horizon)
    except Exception as e:  # noqa: BLE001
      self._fault(f"{type(e).__name__}: {e}")
      self.last_output = self._zero_output("inference_fault")
      return self.last_output

    self.consecutive_faults = 0

    # --- 第 1 道：硬限幅 ---
    residual = self._clamp_residual(residual)

    # --- 第 2 道：变化率限幅 ---
    residual = self._rate_limit(residual, dt)

    # --- 第 3 道：饱和监控 ---
    self._monitor.push(t, residual[:2], np.array(
      [self.cfg["max_d_gain"], self.cfg["max_d_vlt"]], dtype=np.float64))
    if self._monitor.tripped:
      self.disabled = True
      self.disabled_reason = "trust_monitor_saturated"
      self.last_output = self._zero_output(self.disabled_reason)
      return self.last_output

    # --- 模式 1（shadow）：计算完就丢弃，只留日志 ---
    engaged = self.mode >= MODE_LATERAL and not self._monitor.tripped
    d_gain = float(residual[0]) if engaged else 0.0
    d_vlt = float(residual[1]) if engaged else 0.0
    a_lat_budget = float(residual[2]) if self.longitudinal_active else None

    out = {
      "d_gain": d_gain,
      "d_vlt": d_vlt,
      "a_lat_budget": a_lat_budget,
      "engaged": bool(engaged),
      "mode": MODE_NAMES.get(self.mode, str(self.mode)),
      "reason": "ok" if engaged else "shadow",
      # 诊断用原始值（未受模式影响）
      "raw_d_gain": float(residual[0]),
      "raw_d_vlt": float(residual[1]),
      "raw_a_lat_budget": float(residual[2]),
      "style": [round(float(x), 4) for x in self.style.s],
      "saturation_fraction": round(self._monitor.saturation_fraction, 3),
      "style_frozen": self.style.frozen,
      "style_drift": round(float(np.max(np.abs(self.style.s - self.style.s0))), 4),
    }
    self.last_output = out
    self._maybe_log(t, out)
    return out

  # -------------------------------------------------------------------------
  # 推理
  # -------------------------------------------------------------------------

  def _infer(self, t: float, dt: float, kappa: float, v_ego: float,
             steering_angle_deg: float, path_angle_last: float,
             yaw_rate: float,
             kappa_horizon: Sequence[float] | None) -> np.ndarray:
    assert self.bundle is not None and self.bundle.lateral_ff is not None
    assert self.bundle.curve_speed is not None

    kappa = float(kappa)
    v_ego = float(v_ego)

    # kappa 变化率
    if self._last_kappa is not None and dt > 1e-3:
      self._kappa_rate = (kappa - self._last_kappa) / dt
    else:
      self._kappa_rate = 0.0
    self._last_kappa = kappa

    # a_lat 低通（tau = 0.3 s）
    a_lat = kappa * v_ego * v_ego
    if dt > 0.0:
      alpha = dt / (dt + 0.3)
      self._a_lat_filt += alpha * (a_lat - self._a_lat_filt)
    else:
      self._a_lat_filt = a_lat

    style = self.style.s

    # --- 横向 ---
    x_lat = F.build_lateral_ff_features(
      kappa=kappa,
      kappa_rate=self._kappa_rate,
      v_ego=v_ego,
      steering_angle_deg=steering_angle_deg,
      path_angle_last=path_angle_last,
      yaw_rate=yaw_rate,
      a_lat_filt=self._a_lat_filt,
      kappa_hist=self.kappa_hist,
      style=style,
      steer_ratio=self.steer_ratio,
      wheelbase=self.wheelbase,
    )
    y_lat = self.bundle.lateral_ff.forward(x_lat)

    # --- 纵向 ---
    if kappa_horizon is None:
      # 用当前曲率恒定外推。精度差，但比"因为没有前瞻就完全不动作"好 ——
      # 且 a_lat_budget 下游还有规划器自己的曲率限速兜底。
      kh = np.full(F.CURVE_HORIZON_N, kappa, dtype=np.float64)
    else:
      kh = np.asarray(kappa_horizon, dtype=np.float64)
      if kh.size != F.CURVE_HORIZON_N:
        kh = np.resize(kh, F.CURVE_HORIZON_N)

    x_long = F.build_curve_speed_features(kh, v_ego, a_lat, style)
    y_long = self.bundle.curve_speed.forward(x_long)

    # 曲率历史最后 push（保证本帧特征用的是"过去"的曲率）
    self.kappa_hist.push(kappa)

    # 输出语义：
    #   y_lat[0]  = d_gain       (已含 r_max=0.30 的有界性)
    #   y_lat[1]  = d_vlt
    #   y_long[0] = d_alat       (相对基准 a_lat 的残差)
    base_alat = 3.0
    return np.array([float(y_lat[0]), float(y_lat[1]), base_alat + float(y_long[0])],
                    dtype=np.float64)

  # -------------------------------------------------------------------------
  # 第 1 / 第 2 道边界
  # -------------------------------------------------------------------------

  def _clamp_residual(self, r: np.ndarray) -> np.ndarray:
    """第 1 道：硬限幅。限幅值同时受配置与 ISO 上限约束。"""
    g = float(np.clip(r[0], -self.cfg["max_d_gain"], self.cfg["max_d_gain"]))
    v = float(np.clip(r[1], -self.cfg["max_d_vlt"], self.cfg["max_d_vlt"]))

    # a_lat_budget 的上限取 min(配置, ISO) —— 配置写错也不能突破 ISO
    hi = min(float(self.cfg["alat_budget_max"]), float(self.cfg["alat_iso_limit"]))
    lo = float(self.cfg["alat_budget_min"])
    a = float(np.clip(r[2], lo, max(lo, hi)))

    if not (math.isfinite(g) and math.isfinite(v) and math.isfinite(a)):
      raise ValueError("non-finite residual")
    return np.array([g, v, a], dtype=np.float64)

  def _rate_limit(self, r: np.ndarray, dt: float) -> np.ndarray:
    """第 2 道：变化率限幅。

    限幅的对象是**残差本身**，不是最终指令。这样做的理由：残差是 NN 的
    全部贡献，限制它的变化率就等于给 NN 的贡献加了一个已知的 jerk 上界；
    而基线的变化由原有控制器负责（它本来就有 ROC 与软限幅）。

    dt 为 0（首帧）或过大（断流）时，直接采用目标值而不是限幅 ——
    否则重启后的第一帧会被错误地锁死在 0。
    """
    if dt <= 1e-3 or dt > 5.0:
      self._prev = r.copy()
      return r

    rates = np.array([self.cfg["max_d_gain_rate"],
                      self.cfg["max_d_vlt_rate"],
                      self.cfg["max_alat_rate"]], dtype=np.float64)
    max_step = rates * dt
    step = np.clip(r - self._prev, -max_step, max_step)
    out = self._prev + step
    self._prev = out.copy()
    return out

  # -------------------------------------------------------------------------
  # 故障与诊断
  # -------------------------------------------------------------------------

  def _fault(self, reason: str) -> None:
    self.n_faults += 1
    self.consecutive_faults += 1
    if self.consecutive_faults >= int(self.cfg["max_consecutive_faults"]):
      self.disabled = True
      self.disabled_reason = f"fault_storm: {reason}"

  def _zero_output(self, reason: str) -> dict[str, Any]:
    return {
      "d_gain": 0.0,
      "d_vlt": 0.0,
      "a_lat_budget": None,
      "engaged": False,
      "mode": MODE_NAMES.get(self.mode, str(self.mode)),
      "reason": reason,
      "raw_d_gain": 0.0,
      "raw_d_vlt": 0.0,
      "raw_a_lat_budget": 0.0,
      "style": [round(float(x), 4) for x in self.style.s],
      "saturation_fraction": 0.0,
      "style_frozen": False,
      "style_drift": 0.0,
    }

  def _maybe_log(self, t: float, out: Mapping[str, Any]) -> None:
    period = float(self.cfg.get("log_period_s", 0.0) or 0.0)
    if period <= 0.0 or (t - self._last_log_t) < period:
      return
    self._last_log_t = t
    try:
      print(f"[ford_nn] mode={out['mode']} reason={out['reason']} "
            f"d_gain={out['d_gain']:+.4f} d_vlt={out['d_vlt']:+.4f} "
            f"alat_budget={out['a_lat_budget']} "
            f"style={out['style']} drift={out['style_drift']} "
            f"frozen={out['style_frozen']} sat={out['saturation_fraction']}")
    except Exception:  # noqa: BLE001 - 日志永远不能影响控制
      pass

  # -------------------------------------------------------------------------
  # 辅助
  # -------------------------------------------------------------------------

  @staticmethod
  def budget_to_speed(a_lat_budget: float, kappa_max: float,
                      v_floor: float = 2.0) -> float:
    """把横向加速度预算换算为过弯目标速度。

        v = sqrt(a_lat / |kappa|)

    Args:
      a_lat_budget: 横向加速度预算 (m/s^2)。
      kappa_max:    前瞻窗内的最大曲率 (1/m)。
      v_floor:      速度下限 (m/s)，避免直线段除零导致无穷大。

    Returns:
      目标速度 (m/s)。
    """
    k = abs(float(kappa_max))
    if k < 1e-5:
      return float("inf")   # 直线，不限制
    return max(math.sqrt(max(float(a_lat_budget), 0.0) / k), float(v_floor))

  def reset(self) -> None:
    """复位内部状态（不重置风格，风格是跨行程累积的）。"""
    self.kappa_hist.reset()
    self._a_lat_filt = 0.0
    self._t_prev = None
    self._last_kappa = None
    self._kappa_rate = 0.0
    self._prev[:] = 0.0
    self._monitor.reset()
    self.consecutive_faults = 0
    self.disabled = False
    self.disabled_reason = ""

  def diagnostics(self) -> dict[str, Any]:
    return {
      "mode": MODE_NAMES.get(self.mode, str(self.mode)),
      "enabled": self.enabled,
      "lateral_active": self.lateral_active,
      "longitudinal_active": self.longitudinal_active,
      "weights_path": self.weights_path,
      "config_path": CONFIG_PATH,
      "style_persist": str(self.cfg.get("style_persist", "")),
      "style_learn": bool(self.cfg.get("style_enabled", True)),
      "load_error": self.load_error,
      "disabled": self.disabled,
      "disabled_reason": self.disabled_reason,
      "n_frames": self.n_frames,
      "n_faults": self.n_faults,
      "saturation_fraction": round(self._monitor.saturation_fraction, 3),
      "style": self.style.as_dict(),
    }
