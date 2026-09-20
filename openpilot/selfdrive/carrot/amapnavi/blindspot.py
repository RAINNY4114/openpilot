#!/usr/bin/env python3
"""侧向目标跟踪与盲区风险评估。

相比 cpv9-dev 原实现（``RadarSpeedEstimator`` + ``is_side_object_risky``）的改进点：

1. **目标跟踪用一维匀速卡尔曼滤波**替代「加速度限幅微分 + 滑动平均」。
   原实现用 ``(d1-d0)/dt`` 求速度再累加进长度为 5 的滑动平均，缺点：
   * 距离量化误差 / 丢帧造成的抖动会被放大成速度噪声；
   * ``max_acc`` 硬限幅会让真正在急加减速的目标速度被长期拉偏；
   * 滑动平均引入固定 ``smooth_n/2`` 帧延迟，且对丢帧不作区别处理。
   卡尔曼滤波把「距离」和「速度」作为联合状态估计，天然平滑，并且按
   ``dt`` 做预测，丢帧时只用预测值，不会污染估计。

2. **异常值剔除（新息门限 gating）**：单次跳变超过 ``max_speed_step`` 或
   新息超过 ``gate_sigma`` 倍标准差时不参与更新，避免目标 ID 跳变 / 旁边
   车道目标窜入时出现假盲区。这是原实现完全没有的。

3. **风险评估同时考虑 TTC 与横向余量**，并支持迟滞：
   * ``min_clearance``：预测窗口内的最小纵向间距（比原来的单点外推稳健）；
   * ``ttc``：只有真正在接近时才计算，避免远处静止目标误报；
   * 原实现把「未来距离」与「安全距离」两个含义不同的量直接比较，
     且在 ``min_drel_scale`` 为负时切换成绝对距离语义，容易误配置。

4. **两级迟滞的去抖**（``SideObjectMonitor``）：进入用敏感阈值，退出要求
   更大的余量，避免在临界点来回抖动——原实现只做延时消抖。
"""

from dataclasses import dataclass


@dataclass
class TrackerConfig:
  """卡尔曼滤波参数。"""
  process_accel: float = 4.0        # 目标加速度过程噪声 (m/s^2)
  measurement_std_m: float = 0.15   # 距离测量噪声 (m)
  initial_speed_std: float = 10.0   # 初始速度不确定度 (m/s)
  gate_sigma: float = 4.0           # 新息门限（倍标准差）
  max_speed_step: float = 40.0      # 单帧相对速度跳变上限 (m/s)，超出视为异常
  lost_timeout_s: float = 0.5       # 数据丢失多久后彻底重置
  max_predict_s: float = 1.0        # 单次预测最大外推时间
  max_rejects: int = 3              # 连续多少次测量被剔除后视为新目标并重新初始化


class TrackedTarget:
  """一维匀速模型跟踪单个侧向目标的纵向距离 / 相对速度。

  输入距离单位为 **mm**，时间戳单位为 **ms**，与外挂雷达协议保持一致；
  对外输出仍为 ``(mm, m/s)``。

  约定：``dist_mm > 0`` 表示目标在前方，``< 0`` 表示在后方。
  """

  def __init__(self, config: TrackerConfig | None = None):
    self.cfg = config or TrackerConfig()
    self._dist_m = None       # 滤波后的距离 (m)
    self._speed = None        # 相对速度 (m/s)，目标速度 - 本车速度
    self._var_dd = 0.0        # 距离方差
    self._var_vv = 0.0        # 速度方差
    self._var_dv = 0.0        # 协方差
    self._last_t = None
    self._rejects = 0         # 连续被剔除的测量次数

  @property
  def valid(self) -> bool:
    return self._dist_m is not None

  def reset(self) -> None:
    self._dist_m = None
    self._speed = None
    self._last_t = None

  def update(self, dist_mm, t_ms) -> tuple[float, float] | None:
    """喂入一帧距离测量。

    :return: ``(距离 mm, 相对速度 m/s)``，尚未收敛时返回 ``None``。
    """
    cfg = self.cfg

    if dist_mm is None or t_ms is None:
      return self._handle_lost(t_ms)
    if self._dist_m is None or self._last_t is None:
      return self._initialize(dist_mm, t_ms)

    dt = (t_ms - self._last_t) / 1000.0
    if dt <= 0 or dt > cfg.max_predict_s:
      # 时间戳异常（倒退或间隔过大）：只更新时间戳，不用这帧做微分
      if dt > 0:
        self._last_t = t_ms
      return None if self._dist_m is None else (self._dist_m * 1000.0, self._speed)
    self._last_t = t_ms

    self._predict(dt)

    measured = dist_mm / 1000.0
    innovation = measured - self._dist_m
    s = self._var_dd + cfg.measurement_std_m ** 2

    # 异常值剔除：物理不可能的跳变 或 超出新息门限
    implied_speed = innovation / dt
    if (abs(implied_speed) > cfg.max_speed_step or
        innovation ** 2 > (cfg.gate_sigma ** 2) * max(s, 1e-9)):
      # 只用预测值，测量被拒绝。
      # 但连续被拒说明目标已经换了一个（新目标进入 / 距离跳变 / 跟踪换了对象），
      # 此时旧状态没有意义，必须重新初始化——否则距离会永久卡在旧值上，
      # 表现为"旁边明明有车却一直不报盲区"。
      self._rejects += 1
      if self._rejects >= cfg.max_rejects:
        self._initialize(dist_mm, t_ms)
      return (self._dist_m * 1000.0, self._speed)

    self._rejects = 0
    k_d = self._var_dd / s
    k_v = self._var_dv / s
    self._dist_m += k_d * innovation
    self._speed += k_v * innovation

    var_dd, var_dv, var_vv = self._var_dd, self._var_dv, self._var_vv
    self._var_dd = (1.0 - k_d) * var_dd
    self._var_dv = (1.0 - k_d) * var_dv
    self._var_vv = var_vv - k_v * var_dv

    return (self._dist_m * 1000.0, self._speed)

  # ------------------------------------------------------------------ 内部
  def _initialize(self, dist_mm, t_ms):
    cfg = self.cfg
    self._dist_m = dist_mm / 1000.0
    self._speed = 0.0
    self._var_dd = cfg.measurement_std_m ** 2
    self._var_vv = cfg.initial_speed_std ** 2
    self._var_dv = 0.0
    self._last_t = t_ms
    return None

  def _handle_lost(self, t_ms):
    if self._dist_m is None:
      return None
    if t_ms is not None and self._last_t is not None:
      if (t_ms - self._last_t) / 1000.0 > self.cfg.lost_timeout_s:
        self.reset()
        return None
      return (self._dist_m * 1000.0, self._speed)
    return (self._dist_m * 1000.0, self._speed)

  def _predict(self, dt):
    q = (self.cfg.process_accel * dt) ** 2
    self._dist_m += self._speed * dt
    self._var_dd += dt * (2.0 * self._var_dv + dt * self._var_vv) + q * dt * dt / 4.0
    self._var_dv += dt * self._var_vv + q * dt / 2.0
    self._var_vv += q


@dataclass
class RiskResult:
  """一次风险评估的结果。"""
  risky: bool
  clearance_m: float = 0.0     # 预测窗口内的最小纵向间距
  ttc_s: float | None = None   # 碰撞时间，非接近目标为 None
  margin: float = 999.0        # 相对安全余量 (>1 安全，<1 危险)
  danger_dist_m: float = 0.0   # 本次使用的危险距离 (m)
  closing_mps: float = 0.0     # 接近速度 (m/s)
  horizon_s: float = 0.0       # 本次使用的预测窗口 (s)


@dataclass
class RiskConfig:
  """风险评估参数。

  两个阈值的语义（沿用原有 params 的约定，负值表示使用绝对距离）：
  * ``dist_time``  > 0：安全间距 = 本车速度 × 该时距；
  * ``dist_time`` <= 0：安全间距 = |dist_time| 对应的绝对距离。
  """
  horizon_s: float = 3.0        # 预测窗口，覆盖一次完整变道
  ttc_threshold_s: float = 2.5  # TTC 低于该值判定危险
  min_clearance_m: float = 4.0  # 绝对最小间距兜底


def resolve_safe_distance(dist_time, v_ego_mps, min_clearance_m=4.0) -> float:
  """把「时距 / 绝对距离」参数解析成安全距离 (m)。

  * ``dist_time > 0``：安全间距 = 本车速度 × 该时距；
  * ``dist_time <= 0``：安全间距 = |dist_time|（绝对距离，单位 m）。
  两种情况下都不小于 ``min_clearance_m`` 兜底值。
  """
  if dist_time < 0:
    return max(abs(dist_time), min_clearance_m)
  return max(v_ego_mps * dist_time, min_clearance_m)


def assess_side_risk(drel_mm, vrel_mps, v_ego_mps, safe_distance_m,
                     horizon_s=3.0, ttc_threshold_s=2.5) -> RiskResult:
  """评估单个侧向目标的风险。

  :param drel_mm:      纵向相对距离 (mm)，前为正、后为负
  :param vrel_mps:     相对速度 (m/s)，目标速度 - 本车速度
  :param v_ego_mps:    本车速度 (m/s)
  :param safe_distance_m: 允许的最小间距 (m)
  """
  if drel_mm is None or vrel_mps is None or v_ego_mps is None:
    return RiskResult(False)

  d = drel_mm / 1000.0
  d_abs = abs(d)

  # 预测窗口内的最小间距：匀速外推，端点必取
  d_end = d + vrel_mps * horizon_s
  if d * d_end < 0:            # 窗口内发生穿越，间距可以为 0
    clearance = 0.0
  else:
    clearance = min(d_abs, abs(d_end))

  # TTC：只有正在靠近时才计算
  ttc = None
  if vrel_mps < 0 and d > 0:      # 前方目标在被追上
    ttc = d / -vrel_mps
  elif vrel_mps > 0 and d < 0:    # 后方目标在追上来
    ttc = d_abs / vrel_mps

  ttc_risk = ttc is not None and 0.0 <= ttc < ttc_threshold_s
  clearance_risk = clearance < safe_distance_m

  return RiskResult(
    risky=bool(ttc_risk or clearance_risk),
    clearance_m=clearance,
    ttc_s=ttc,
    margin=clearance / safe_distance_m if safe_distance_m > 0 else 999.0,
  )


def side_object_risky(drel_mm, vrel_mps, v_ego_mps,
                      vrel_time_s=4.0, drel_time_s=-5.0,
                      ttc_threshold_s=2.5, min_clearance_m=4.0,
                      latch: bool = False, release_margin: float = 1.4) -> RiskResult:
  """cpv9-dev 口径的侧向车辆风险评估（前方 / 后方通用）。

  与 :func:`assess_side_risk` 的区别（这两个参数原来被读进来但从未使用）：

  * 预测窗口用 **vrel 时距**（``LidarFrontVRelDistTime`` / ``LidarBehindVRelDistTime``），
    也就是 cpv9-dev 的 ``time_horizon``；
  * 危险距离用 **drel 时距**（``LidarFrontVDistTime`` / ``LidarBehindVDistTime``），
    正值 = 基准速度 × 时距，负值 = 绝对距离（m）；对应 cpv9-dev 的 ``min_drel_scale``。
    基准速度按方位区分（与 nav_params 文档一致，B 方案）：

    * 前方 —— **本车速度**（风险来自我追它）；
    * 后方 —— **对方速度** ``v_ego + vrel``（风险来自它追我，后车越快要求越大）；
  * 判据（与 cpv9-dev 一致）::

        接近速度 closing = 前方 max(-vrel, 0) / 后方 max(vrel, 0)
        未来距离 future  = |d| - closing * 窗口
        危险 = future < 危险距离  或  |d| < 危险距离   （另加 TTC 兜底）

  :param latch:            True 表示本角上一帧已判定危险，此时用 ``release_margin``
                           放宽退出条件（迟滞），避免在阈值附近来回抖动。
  :param release_margin:   迟滞退出倍数：已报警时，间距要大于 ``危险距离×该值``
                           才允许清除。
  """
  if drel_mm is None or vrel_mps is None or v_ego_mps is None:
    return RiskResult(False)

  d = float(drel_mm) / 1000.0
  d_abs = abs(d)
  v_ego = max(0.0, float(v_ego_mps))
  v_rel = float(vrel_mps)

  # 危险距离：正=时距，负=绝对距离（cpv9-dev 的 min_drel_scale 语义）
  #   基准速度：前方用本车速度；后方用对方速度(v_ego + vrel)，后车越快要求越大
  t_drel = float(drel_time_s or 0.0)
  if t_drel > 0:
    basis_speed = v_ego if d > 0 else max(0.0, v_ego + v_rel)
    danger_dist = max(basis_speed * t_drel, min_clearance_m)
  else:
    danger_dist = max(abs(t_drel), min_clearance_m)

  # 接近速度：前方目标在被追上，后方目标在追上来
  closing = max(-v_rel, 0.0) if d > 0 else max(v_rel, 0.0)

  horizon = max(0.5, float(vrel_time_s or 0.0))
  future_dist = max(0.0, d_abs - closing * horizon)

  ttc = d_abs / closing if closing > 0.05 else None
  ttc_risk = ttc is not None and ttc < ttc_threshold_s

  # 迟滞：已报警时要求间距退到更远才清除
  limit = danger_dist * release_margin if latch else danger_dist
  risky = bool(future_dist < limit or d_abs < limit or ttc_risk)
  clearance = min(d_abs, future_dist)

  return RiskResult(
    risky=risky,
    clearance_m=clearance,
    ttc_s=ttc,
    margin=(clearance / danger_dist) if danger_dist > 0 else 999.0,
    danger_dist_m=danger_dist,
    closing_mps=closing,
    horizon_s=horizon,
  )


class OccupancyCounter:
  """计数式去抖（兼容原实现的 counter 语义，用于无牙滤波结果前的粗过滤）。"""

  def __init__(self, hold_s=1.0, dt=0.1):
    self.count = 0
    self.detected = False
    self.hold_s = hold_s
    self.dt = dt

  def update(self, active: bool) -> bool:
    if active:
      self.count = 1
    else:
      self.count -= 1
      floor = int(-60.0 / self.dt)
      if self.count < floor:
        self.count = floor
    if self.detected:
      if self.count <= int(-self.hold_s / self.dt):
        self.detected = False
    elif self.count > 0:
      self.detected = True
    return self.detected
