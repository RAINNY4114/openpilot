"""
ford_driver_style.py —— 驾驶风格向量与"有界在线进化"

这一层回答用户需求里的"逐渐根据驾驶习惯进化提高"。

设计取舍：为什么用 4 个可解释标量，而不是学一个隐变量
---------------------------------------------------------------------------
学术界做个性化常用自编码器学一个 k 维 style latent。但对**安全关键**的
车载系统，隐变量有三个致命问题：

  1. 不可解释 —— 出问题时无法回答"它到底改了什么"。
  2. 无界 —— 隐空间里一次异常的编码就可能落到从未训练过的区域。
  3. 不可审计 —— 无法向用户/监管解释"你的车为什么这样开"。

所以这里用 4 个**物理含义明确、范围固定 [-1, +1]** 的标量：

  aggressiveness    激进程度：平均横向加速度与纵向加速度的偏向
  smoothness        平滑偏好：对 jerk（加加速度）的容忍度，越高越平顺
  curve_commitment  过弯决心：入弯前减速的早晚与幅度，越高越"提前且少减"
  caution           谨慎程度：跟车时距、超速倾向、接管频率的综合

关键性质：**风格向量只能影响"舒适性偏好"维度，绝不能放松安全边界。**
它进的是 NN 的特征（进而影响残差输出），而残差输出本身有 r_max 上限、
上层还有硬限幅、panda 里还有 ISO 11270 与量程约束。四道边界叠加，
风格再怎么漂移也无法把车推出物理可行域。

进化机制：三道闸
---------------------------------------------------------------------------
  1. **慢时间常数**：τ 默认 1800 s（半小时驾驶），即
     `alpha = 1 - exp(-dt / tau)`，每帧只挪一点点。任何单次异常驾驶
     （急刹、避让）都淹没在长时间常数里。
  2. **信任域**：`|s - s0| <= TRUST_RADIUS`（默认 0.40），s0 是出厂标定值。
     个性化可以在标定值附近调整，但不能把车的性格改成另一个人。
  3. **拒绝条件**：当前窗口里出现人工接管 / ESP 介入 / jerk 异常时**冻结**自适应。
     理由很直接 —— 这些时刻的驾驶行为不代表驾驶员的"习惯"，而代表"应急"。
     把应急行为学进习惯里，是最典型也最危险的个性化失效模式。
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from typing import Mapping, Sequence

import numpy as np

STYLE_DIM = 4
STYLE_NAMES: tuple[str, ...] = (
  "aggressiveness",
  "smoothness",
  "curve_commitment",
  "caution",
)

# 风格向量必须落在 [-1, 1]
STYLE_MIN, STYLE_MAX = -1.0, 1.0

# 出厂标定值（中性风格）。个性化围绕它做有界调整。
DEFAULT_STYLE0: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)

# 信任域半径：|s - s0| <= TRUST_RADIUS
TRUST_RADIUS = 0.40

# 自适应时间常数 (s)。默认 1800 s = 半小时驾驶。
DEFAULT_TAU_S = 1800.0

# ---------------------------------------------------------------------------
# 统计量
# ---------------------------------------------------------------------------

STATS_NAMES: tuple[str, ...] = (
  "a_lat_mean",        # 平均横向加速度 (m/s^2)
  "a_lat_p90",         # 90 分位横向加速度
  "jerk_mean",         # 平均横向 jerk (m/s^3)
  "jerk_p90",          # 90 分位横向 jerk
  "brake_decel_mean",  # 减速段的平均减速度幅度
  "brake_event_rate",  # 制动事件频率 (次/min)
  "curve_speed_ratio", # 实际车速 / 几何安全车速
  "follow_gap_mean",   # 平均跟车时距 (s)
  "steer_rate_mean",   # 平均方向盘角速度 (deg/s)
  "override_rate",     # 接管频率 (次/min)
  "speed_over_ratio",  # 超限速比例
  "gas_mean",          # 平均油门开度
)
STATS_DIM = len(STATS_NAMES)

# 统计量归一化除数（让 stats 大致落在 [-1, 1]）
STATS_SCALE = np.array([
  3.0, 3.6, 3.6, 6.0, 2.0, 5.0,
  0.3, 2.0, 200.0, 1.0, 0.2, 1.0,
], dtype=np.float64)

# 需要减去的中心值（例如 curve_speed_ratio 的中心是 1.0）
STATS_CENTER = np.array([
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
], dtype=np.float64)

# ---------------------------------------------------------------------------
# 默认风格映射矩阵 (4 x 12)：stats -> style
#
# 每一行对应一个风格维，元素是"该统计量对它的贡献权重"。
# 这是**手工标定的先验**，不是学出来的 —— 目的是让系统出厂即可用、
# 且行为可解释。若后续有足够个人数据，可用 train_offline.py 训练一个
# 编码器替换它（见 StyleEncoder）。
#
# 符号约定的理由：
#   aggressiveness   : 横向加速度、油门、超速 → 正；跟车时距 → 负
#   smoothness       : jerk → 负；方向盘角速度 → 负
#   curve_commitment : 过弯速度比 → 正；制动频率 → 负（常减速说明决心低）
#   caution          : 跟车时距、接管频率 → 正；超速、激进 → 负
# ---------------------------------------------------------------------------
DEFAULT_STYLE_MATRIX = np.array([
  # a_lat_m  a_lat_p90 jerk_m  jerk_p90 brk_dec brk_rate csr    gap    srate  ovr    over   gas
  [  0.45,    0.35,    -0.10,  -0.10,    0.00,   -0.05,  0.20, -0.25, -0.05, 0.00,  0.30,  0.40],
  [ -0.10,   -0.20,    -0.45,  -0.40,    0.10,   -0.10,  0.05,  0.10, -0.45, 0.00,  0.00, -0.10],
  [  0.15,    0.20,     0.00,   0.00,   -0.20,   -0.35,  0.50, -0.05,  0.00, 0.00,  0.15,  0.20],
  [ -0.20,   -0.25,     0.15,   0.10,    0.25,    0.10, -0.30,  0.50,  0.10, 0.35, -0.35, -0.25],
], dtype=np.float64)


def stats_to_style(stats: Sequence[float],
                   matrix: np.ndarray | None = None,
                   style0: Sequence[float] = DEFAULT_STYLE0) -> np.ndarray:
  """把归一化统计量映射为风格向量。

  Args:
    stats:  归一化后的统计量，长度 STATS_DIM。
    matrix: 4x12 风格映射矩阵，None 用 DEFAULT_STYLE_MATRIX。
    style0: 中性风格基准。

  Returns:
    风格向量，形状 (STYLE_DIM,)，每个元素在 [-1, 1]。
  """
  s = np.asarray(stats, dtype=np.float64).reshape(-1)
  if s.size != STATS_DIM:
    raise ValueError(f"stats must have {STATS_DIM} entries, got {s.size}")
  if not np.all(np.isfinite(s)):
    raise ValueError("non-finite stats")

  M = DEFAULT_STYLE_MATRIX if matrix is None else np.asarray(matrix, dtype=np.float64)
  if M.shape != (STYLE_DIM, STATS_DIM):
    raise ValueError(f"style matrix must be {STYLE_DIM}x{STATS_DIM}, got {M.shape}")

  return np.clip(np.tanh(M @ s) + np.asarray(style0, dtype=np.float64),
                 STYLE_MIN, STYLE_MAX)


# ---------------------------------------------------------------------------
# 滚动统计窗口
# ---------------------------------------------------------------------------


class DrivingStatsWindow:
  """维护最近 `window_s` 秒的驾驶统计量。

  采样降频到 `sample_hz`（默认 4 Hz）后再入窗 —— 20 Hz 原始帧率下
  60 s 窗口会有 1200 个样本，做分位数排序的开销没必要；4 Hz 的 240 个样本
  对均值/分位数的估计已经足够，且抖动更小。
  """

  def __init__(self, window_s: float = 60.0, sample_hz: float = 4.0,
               max_span_s: float = 3600.0) -> None:
    self.window_s = float(window_s)
    self.sample_hz = float(sample_hz)
    self._min_dt = 1.0 / max(self.sample_hz, 0.1)
    self._max_span_s = float(max_span_s)

    self._t: deque[float] = deque()
    self._a_lat: deque[float] = deque()
    self._a_long: deque[float] = deque()
    self._jerk: deque[float] = deque()
    self._v: deque[float] = deque()
    self._kappa: deque[float] = deque()
    self._steer_rate: deque[float] = deque()
    self._gap: deque[float] = deque()
    self._gas: deque[float] = deque()
    self._over: deque[float] = deque()
    self._speed_over: deque[float] = deque()

    self._last_sample_t: float | None = None
    self._last_a_lat: float | None = None

    # 制动事件计数（跨窗口累计，用时间戳做率）
    self._brake_events: deque[float] = deque()
    self._override_events: deque[float] = deque()
    self._brake_active = False
    self._override_active = False

    self._t0: float | None = None
    self._t_last: float | None = None

  # -- 生命周期 ------------------------------------------------------------

  def reset(self) -> None:
    for d in (self._t, self._a_lat, self._a_long, self._jerk, self._v, self._kappa,
              self._steer_rate, self._gap, self._gas, self._over, self._speed_over,
              self._brake_events, self._override_events):
      d.clear()
    self._last_sample_t = None
    self._last_a_lat = None
    self._brake_active = False
    self._override_active = False
    self._t0 = None
    self._t_last = None

  # -- 采样 ----------------------------------------------------------------

  def push(self, t: float, *,
           a_lat: float = 0.0,
           a_long: float = 0.0,
           v_ego: float = 0.0,
           kappa: float = 0.0,
           steer_rate_deg: float = 0.0,
           follow_gap_s: float = 0.0,
           gas: float = 0.0,
           speed_limit_ratio: float = 0.0,
           override: bool = False,
           esp_active: bool = False,
           engaged: bool = True) -> None:
    """推入一帧样本。

    Args:
      t:                单调递增时间戳 (s)。
      a_lat:            横向加速度 (m/s^2)。
      a_long:           纵向加速度 (m/s^2)。
      v_ego:            车速 (m/s)。
      kappa:            当前曲率 (1/m)。
      steer_rate_deg:   方向盘角速度 (deg/s)。
      follow_gap_s:     跟车时距 (s)，无前车时填 0。
      gas:              油门开度 [0,1]。
      speed_limit_ratio: 超速比例 (v / limit - 1)。
      override:         本帧是否人工接管。
      esp_active:       ESP/ABS 是否介入（用于冻结自适应）。
      engaged:          本帧 openpilot 是否在控。仅统计在控时的横纵向行为，
                        否则会把"人开车"的数据当成"车的表现"。
    """
    t = float(t)
    if self._t0 is None:
      self._t0 = t
    self._t_last = t

    # 降频采样
    if self._last_sample_t is not None and (t - self._last_sample_t) < self._min_dt:
      # 但仍要维护事件计数与 jerk 的差分基准
      self._track_events(t, override)
      return
    self._last_sample_t = t

    # jerk = d(a_lat)/dt
    jerk = 0.0
    if self._last_a_lat is not None and self._min_dt > 0:
      jerk = (float(a_lat) - self._last_a_lat) / max(self._min_dt, 1e-3)
    self._last_a_lat = float(a_lat)

    if not engaged:
      # 非在控帧不进行为统计，但事件计数照常
      self._track_events(t, override)
      return

    self._t.append(t)
    self._a_lat.append(float(a_lat))
    self._a_long.append(float(a_long))
    self._jerk.append(jerk)
    self._v.append(float(v_ego))
    self._kappa.append(float(kappa))
    self._steer_rate.append(abs(float(steer_rate_deg)))
    if follow_gap_s > 0.1:
      self._gap.append(float(follow_gap_s))
    self._gas.append(float(gas))
    self._over.append(1.0 if override else 0.0)
    self._speed_over.append(max(0.0, float(speed_limit_ratio)))

    self._track_events(t, override)
    self._evict(t)

  def _track_events(self, t: float, override: bool) -> None:
    if override and not self._override_active:
      self._override_events.append(t)
    self._override_active = bool(override)

    # 制动事件：a_long 低于 -0.3 视为一次制动
    braking = False
    if self._a_long:
      braking = self._a_long[-1] < -0.3
    if braking and not self._brake_active:
      self._brake_events.append(t)
    self._brake_active = braking

  def _evict(self, t: float) -> None:
    cutoff = t - self.window_s
    while self._t and self._t[0] < cutoff:
      for d in (self._t, self._a_lat, self._a_long, self._jerk, self._v,
                self._kappa, self._steer_rate, self._gas, self._over,
                self._speed_over):
        d.popleft()
    # 事件队列保留更长时间（用于"最近 10 分钟接管率"）
    ev_cut = t - min(self.window_s, self._max_span_s)
    while self._brake_events and self._brake_events[0] < ev_cut:
      self._brake_events.popleft()
    while self._override_events and self._override_events[0] < ev_cut:
      self._override_events.popleft()
    # 跟车时距用独立队列（无前车时不入队）
    while self._gap and len(self._gap) > len(self._t):
      self._gap.popleft()

  # -- 统计 ----------------------------------------------------------------

  @property
  def n_samples(self) -> int:
    return len(self._t)

  @property
  def covered_seconds(self) -> float:
    """窗口实际覆盖的时长（s）。用于判断统计量是否可信。"""
    if len(self._t) < 2:
      return 0.0
    return float(self._t[-1] - self._t[0])

  @property
  def total_drive_seconds(self) -> float:
    """本次上电以来的累计驾驶时长（s）。"""
    if self._t0 is None or self._t_last is None:
      return 0.0
    return float(self._t_last - self._t0)

  def stats(self) -> np.ndarray:
    """计算归一化统计量。

    Returns:
      形状 (STATS_DIM,) 的数组。样本不足时返回全 0（即"中性风格"），
      这是刻意的：**宁可退化到出厂标定，也不要用 3 帧数据去个性化。**
    """
    if len(self._t) < 20:
      return np.zeros(STATS_DIM, dtype=np.float64)

    a_lat = np.asarray(self._a_lat, dtype=np.float64)
    jerk = np.asarray(self._jerk, dtype=np.float64)
    a_long = np.asarray(self._a_long, dtype=np.float64)
    v = np.asarray(self._v, dtype=np.float64)
    kappa = np.asarray(self._kappa, dtype=np.float64)
    steer_rate = np.asarray(self._steer_rate, dtype=np.float64)
    gas = np.asarray(self._gas, dtype=np.float64)
    over = np.asarray(self._over, dtype=np.float64)
    speed_over = np.asarray(self._speed_over, dtype=np.float64)

    span_min = max(self.covered_seconds / 60.0, 1e-3)

    # 制动段平均减速度
    decel = -a_long[a_long < -0.3]
    brake_decel = float(decel.mean()) if decel.size else 0.0

    # 过弯速度比：实际速度 / 几何安全速度（a_lat = 3.0 m/s^2 为基准）
    abs_k = np.abs(kappa)
    curved = abs_k > 0.0008
    if np.any(curved):
      v_safe = np.sqrt(3.0 / np.maximum(abs_k[curved], 1e-4))
      csr = float(np.mean(v[curved] / np.maximum(v_safe, 0.1)))
    else:
      csr = 1.0

    gap = np.asarray(self._gap, dtype=np.float64) if self._gap else np.array([0.0])

    raw = np.array([
      float(a_lat.mean()),
      float(np.percentile(np.abs(a_lat), 90)),
      float(np.abs(jerk).mean()),
      float(np.percentile(np.abs(jerk), 90)),
      brake_decel,
      len(self._brake_events) / span_min,
      csr,
      float(gap.mean()) if self._gap else 0.0,
      float(steer_rate.mean()),
      len(self._override_events) / span_min,
      float(speed_over.mean()),
      float(gas.mean()),
    ], dtype=np.float64)

    norm = (raw - STATS_CENTER) / STATS_SCALE
    return np.clip(norm, -3.0, 3.0)


# ---------------------------------------------------------------------------
# 风格状态 + 有界在线进化
# ---------------------------------------------------------------------------


class DriverStyle:
  """驾驶风格向量及其在线进化。

  进化是**慢速、有界、可拒绝**的：

      s_target = stats_to_style(window.stats())
      if not frozen:
          s <- s + alpha * (s_target - s)          # alpha = 1 - exp(-dt/tau)
      s <- clip(s, s0 - TRUST_RADIUS, s0 + TRUST_RADIUS)
      s <- clip(s, STYLE_MIN, STYLE_MAX)

  Attributes:
    s:       当前风格向量。
    s0:      出厂标定值（信任域中心）。
    frozen:  当前是否被拒绝条件冻结。
  """

  def __init__(self,
               style0: Sequence[float] = DEFAULT_STYLE0,
               tau_s: float = DEFAULT_TAU_S,
               trust_radius: float = TRUST_RADIUS,
               window: DrivingStatsWindow | None = None,
               matrix: np.ndarray | None = None,
               persist_path: str | None = None) -> None:
    self.s0 = np.asarray(style0, dtype=np.float64).reshape(-1)
    if self.s0.size != STYLE_DIM:
      raise ValueError(f"style0 must have {STYLE_DIM} entries")
    self.s0 = np.clip(self.s0, STYLE_MIN, STYLE_MAX)

    self.s = self.s0.copy()
    self.tau_s = float(max(tau_s, 1.0))
    self.trust_radius = float(max(trust_radius, 0.0))
    self.matrix = matrix
    self.window = window if window is not None else DrivingStatsWindow()
    self.persist_path = persist_path

    self.frozen = False
    self.freeze_reason = ""
    self.n_updates = 0
    self.total_drive_s = 0.0
    self._t_prev: float | None = None

    # 演化轨迹（用于离线分析"风格是怎么漂过去的"）
    self.trace: deque[tuple[float, tuple[float, ...]]] = deque(maxlen=2000)

    # 拒绝条件的阈值
    self.jerk_freeze_threshold = 8.0   # m/s^3，超过视为应急驾驶
    self.esp_freeze_s = 5.0            # ESP 介入后冻结时长

    self._esp_until_t: float = -1e9

  # -- 生命周期 ------------------------------------------------------------

  def reset(self) -> None:
    self.s = self.s0.copy()
    self.window.reset()
    self.frozen = False
    self.freeze_reason = ""
    self.n_updates = 0
    self.total_drive_s = 0.0
    self._t_prev = None
    self.trace.clear()
    self._esp_until_t = -1e9

  # -- 主循环 --------------------------------------------------------------

  def update(self, t: float, sample: Mapping[str, float | bool]) -> np.ndarray:
    """推入一帧并执行一次有界进化。

    Args:
      t:      单调时间戳 (s)。
      sample: 驾驶样本，键见 DrivingStatsWindow.push。

    Returns:
      当前风格向量（长度为 STYLE_DIM 的只读视图语义）。
    """
    t = float(t)
    self.window.push(t, **sample)  # type: ignore[arg-type]

    if bool(sample.get("esp_active", False)):
      self._esp_until_t = t + self.esp_freeze_s

    self.total_drive_s = self.window.total_drive_seconds

    # --- 拒绝条件 ---
    self.frozen, self.freeze_reason = self._evaluate_freeze(t, sample)

    dt = 0.0 if self._t_prev is None else max(t - self._t_prev, 0.0)
    self._t_prev = t

    if self.frozen or dt <= 0.0 or dt > 5.0:
      # dt > 5 s 说明中间断流（进程重启/长时间非在控），不做大步长更新
      return self.s

    stats = self.window.stats()
    if not np.any(stats):
      # 样本不足，保持出厂标定
      return self.s

    s_target = stats_to_style(stats, self.matrix, self.s0)

    # 慢时间常数
    alpha = 1.0 - math.exp(-dt / self.tau_s)
    s_new = self.s + alpha * (s_target - self.s)

    # 信任域 + 硬范围
    s_new = np.clip(s_new, self.s0 - self.trust_radius, self.s0 + self.trust_radius)
    s_new = np.clip(s_new, STYLE_MIN, STYLE_MAX)

    self.s = s_new
    self.n_updates += 1

    if self.n_updates % 200 == 1:
      self.trace.append((t, tuple(float(x) for x in self.s)))

    return self.s

  def _evaluate_freeze(self, t: float, sample: Mapping[str, float | bool]) -> tuple[bool, str]:
    """判断是否应冻结自适应。

    拒绝条件都指向同一件事：**当前行为不代表"习惯"，而代表"应急"。**
    """
    if t < self._esp_until_t:
      return True, "esp_active"

    if bool(sample.get("override", False)):
      return True, "human_override"

    if not bool(sample.get("engaged", True)):
      return True, "not_engaged"

    # jerk 异常：用窗口内的 p90 而不是瞬时值，避免单帧噪声误触发
    if self.window.n_samples >= 20:
      jerk = np.abs(np.asarray(self.window._jerk, dtype=np.float64))
      if jerk.size and float(np.percentile(jerk, 90)) > self.jerk_freeze_threshold:
        return True, "jerk_high"

    return False, ""

  # -- 诊断 ----------------------------------------------------------------

  def as_dict(self) -> dict:
    return {
      "style": {n: round(float(v), 4) for n, v in zip(STYLE_NAMES, self.s)},
      "style0": {n: round(float(v), 4) for n, v in zip(STYLE_NAMES, self.s0)},
      "drift": round(float(np.max(np.abs(self.s - self.s0))), 4),
      "frozen": self.frozen,
      "freeze_reason": self.freeze_reason,
      "n_updates": self.n_updates,
      "total_drive_s": round(self.total_drive_s, 1),
      "window_samples": self.window.n_samples,
    }

  # -- 持久化 --------------------------------------------------------------

  def save(self, path: str | None = None) -> str:
    """把当前风格与累计里程写盘。返回实际写入路径。"""
    p = path or self.persist_path
    if not p:
      raise ValueError("no persist path given")
    os.makedirs(os.path.dirname(os.path.abspath(p)) or ".", exist_ok=True)
    payload = {
      "version": 1,
      "style": [float(x) for x in self.s],
      "style0": [float(x) for x in self.s0],
      "total_drive_s": float(self.total_drive_s),
      "n_updates": int(self.n_updates),
      "saved_at": time.time(),
    }
    with open(p, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=False, indent=2)
    self.persist_path = p
    return p

  def load(self, path: str | None = None) -> bool:
    """从盘上恢复风格。返回是否成功。

    载入时**同样施加信任域**：即使盘上的文件被篡改（或来自另一辆车、
    另一版标定），风格也无法越出 `s0 ± TRUST_RADIUS`。
    """
    p = path or self.persist_path
    if not p or not os.path.exists(p):
      return False
    try:
      with open(p, encoding="utf-8") as f:
        payload = json.load(f)
      if int(payload.get("version", -1)) != 1:
        return False
      s = np.asarray(payload["style"], dtype=np.float64).reshape(-1)
      if s.size != STYLE_DIM or not np.all(np.isfinite(s)):
        return False
      s = np.clip(s, self.s0 - self.trust_radius, self.s0 + self.trust_radius)
      self.s = np.clip(s, STYLE_MIN, STYLE_MAX)
      self.total_drive_s = float(payload.get("total_drive_s", 0.0))
      self.n_updates = int(payload.get("n_updates", 0))
      return True
    except Exception:
      return False
