#!/usr/bin/env python3
"""盲区有车时的让行速度规划——按人类驾驶员的习惯来决策。

设计思路（对应真人开车的心理活动）
------------------------------------------------------------------
1. **先看有没有位置**：目标车道的前车 / 后车留出的空档够不够（时距 + 绝对最小间距）。
   够 → **什么都不做**，直接变道，不做任何无谓的加减速。
2. **不够就在两个选择里挑一个**（人只会想这两个）：
   * **插到后车前面**：后车正在逼近 → 微微加速，比它快一点，拉开空档后切入；
   * **落到前车后面**：前车更慢/更近 → 松油门轻踩刹车，让它先走，跟在它后面切。
3. **挑代价小的那个**（速度变化量小的），**打平时优先"让一让"减速**——
   真人默认选择是让行而不是抢行；但**后方跟车太近时禁止减速**（人会本能地
   避免"别人贴着你尾巴你还刹车"），这时只能加速或干脆放弃。
4. **前车/后车正在拉开距离的，不去管它**——空档自己在变大，等着就行。
5. **决策一旦做出就坚持一段时间**（commit），不在两个方案之间反复横跳。
6. **幅度温和**：调整量限制在十几 km/h 内，且按舒适变化率渐进执行，
   到位后保持一会儿再平滑恢复，绝不变道一结束就突然窜回去。
7. **两个方案都太夸张（超出舒适幅度）→ 放弃这次机会**，维持车速等下一次，
   不会为了变道去猛加速或急减速。

相比原来的实现
------------------------------------------------------------------
* 原实现是 3 / 6 / 9 m/s **三档查表**，跨阈值就跳档，速度指令突变；
* 前/后分别算完再用 ``f_drel < b_drel*0.3`` 之类的规则仲裁，容易摇摆；
* 本模块改为**先决策（插前 / 落后 / 不动作 / 放弃）再平滑执行**，
  输出随输入连续，并且带承诺期与死区，天然抗抖。
"""

from dataclasses import dataclass


# 决策模式
MODE_NONE = "none"      # 不用调整
MODE_FRONT = "front"    # 加速插到后车前面
MODE_BEHIND = "behind"  # 减速落到前车后面
MODE_GIVEUP = "giveup"  # 让不了，维持车速等下一次机会


@dataclass
class HumanLikeConfig:
  """人类驾驶风格参数。"""
  time_headway_s: float = 1.2       # 期望的车头时距
  min_gap_m: float = 8.0            # 绝对最小空档
  merge_margin_m: float = 3.0       # 变道本身需要的额外余量
  speed_margin_kph: float = 6.0     # 与目标车拉开的速度差
  opening_vrel_mps: float = 0.3     # 超过该相对速度认为「空档在变大」

  # 让行幅度上限：人减速让行比加速抢行更放得开
  max_speedup_kph: float = 12.0     # 最多加速这么多（超过就放弃）
  max_slowdown_kph: float = 18.0    # 最多减速这么多
  deadband_kph: float = 2.0         # 死区：小于该值不做无意义微调
  rear_danger_gap_m: float = 6.0    # 后车贴这么近时禁止减速

  accel_limit_kphps: float = 1.5    # 加速变化率 (km/h/s)
  decel_limit_kphps: float = 2.5    # 减速变化率 (km/h/s)
  commit_s: float = 2.5             # 决策保持时间
  hold_s: float = 1.5               # 到位后的保持时间
  switch_benefit_kph: float = 3.0   # 换方案至少要省这么多才换
  yield_bias_kph: float = 2.0       # 代价接近时偏向让行(减速)

  min_speed_kph: float = 25.0
  max_speed_kph: float = 140.0


@dataclass
class SideTarget:
  """一个侧向目标。"""
  drel_mm: float     # 纵向相对距离 (mm)，前为正、后为负
  vrel_mps: float    # 相对速度 (m/s)，目标速度 - 本车速度


@dataclass
class Plan:
  """一次规划的结果（便于日志与调试）。"""
  mode: str = MODE_NONE
  delta_kph: float = 0.0            # 期望的速度调整量（正=加速）
  reason: str = ""


def _required_gap(v_ego_mps, cfg: HumanLikeConfig) -> float:
  """变道所需的空档 (m)。"""
  return max(cfg.min_gap_m, v_ego_mps * cfg.time_headway_s) + cfg.merge_margin_m


class SpeedAdvisor:
  """把决策结果变成平滑的速度指令。

  用法（每个控制周期一次）：
      target_kph = advisor.update(targets, v_ego_kph, desired_kph, dt)
  其中 ``targets`` 为该侧的前车 / 后车（最多各一个）。
  """

  def __init__(self, config: HumanLikeConfig | None = None):
    self.cfg = config or HumanLikeConfig()
    self.delta_kph = 0.0        # 当前实际执行的调整量
    self.mode = MODE_NONE
    self.base_kph = None        # 进入让行时记录的车速基准
    self.commit_timer_s = 0.0
    self.hold_timer_s = 0.0
    self.last_plan = Plan()

  def reset(self) -> None:
    self.delta_kph = 0.0
    self.mode = MODE_NONE
    self.base_kph = None
    self.commit_timer_s = 0.0
    self.hold_timer_s = 0.0
    self.last_plan = Plan()

  # ------------------------------------------------------------------ 对外
  def update(self, targets: list[SideTarget], v_ego_kph: float,
             desired_kph: float, dt: float) -> float:
    """:return: 建议下发的目标车速 (km/h)。"""
    cfg = self.cfg

    if not targets:
      return self._release(v_ego_kph, desired_kph, dt)

    # 承诺计时：决策做出后坚持一段时间，避免两个方案之间反复横跳
    self.commit_timer_s = max(0.0, self.commit_timer_s - dt)

    if self.base_kph is None:
      self.base_kph = v_ego_kph

    plan = self._plan(targets, v_ego_kph / 3.6)
    self.last_plan = plan

    # 目标调整量（死区过滤）
    wanted = 0.0 if abs(plan.delta_kph) < cfg.deadband_kph else plan.delta_kph

    # 变化率限制——人不会瞬间把速度掰过去
    limit = (cfg.accel_limit_kphps if wanted > self.delta_kph else cfg.decel_limit_kphps) * dt
    step = wanted - self.delta_kph
    if abs(step) > limit:
      step = limit if step > 0 else -limit
    self.delta_kph += step

    if wanted != 0.0:
      self.hold_timer_s = cfg.hold_s
    elif self.hold_timer_s > 0:
      self.hold_timer_s -= dt

    target = self.base_kph + self.delta_kph
    return max(cfg.min_speed_kph, min(cfg.max_speed_kph, target))

  # ------------------------------------------------------------------ 决策
  def _plan(self, targets: list[SideTarget], v_ego_mps: float) -> Plan:
    cfg = self.cfg
    front = rear = None
    for t in targets:
      if t.drel_mm >= 0 and (front is None or t.drel_mm < front.drel_mm):
        front = t
      elif t.drel_mm < 0 and (rear is None or t.drel_mm > rear.drel_mm):
        rear = t

    needed = _required_gap(v_ego_mps, cfg)
    gap_front = None if front is None else front.drel_mm / 1000.0
    gap_rear = None if rear is None else -rear.drel_mm / 1000.0

    # 「亏空」= 还差多少空档。正在拉开距离的一侧亏空按 0 处理（等着就行）
    deficit_front = 0.0 if gap_front is None else max(0.0, needed - gap_front)
    deficit_rear = 0.0 if gap_rear is None else max(0.0, needed - gap_rear)
    if front is not None and front.vrel_mps > cfg.opening_vrel_mps:
      deficit_front = 0.0
    if rear is not None and rear.vrel_mps < -cfg.opening_vrel_mps:
      deficit_rear = 0.0

    if deficit_front <= 0.0 and deficit_rear <= 0.0:
      self._end_commit()
      return Plan(MODE_NONE, 0.0, "gap ok")

    # ---- 方案 A：加速插到后车前面（只有后方空档不够才需要考虑）----
    accel = None
    if rear is not None and deficit_rear > 0.0 and rear.vrel_mps > 0:
      accel = rear.vrel_mps * 3.6 + cfg.speed_margin_kph

    # ---- 方案 B：减速落到前车后面（只有前方空档不够才需要考虑）----
    decel = None
    if front is not None and deficit_front > 0.0 and front.vrel_mps < cfg.opening_vrel_mps:
      decel = min(front.vrel_mps * 3.6 - cfg.speed_margin_kph, 0.0)

    # 后车贴太近时禁止减速：人会避免「别人贴着你尾巴你还刹车」
    if gap_rear is not None and gap_rear < cfg.rear_danger_gap_m and \
       (rear is None or rear.vrel_mps > 0):
      decel = None

    # 超出舒适幅度就放弃
    if accel is not None and accel > cfg.max_speedup_kph:
      accel = None
    if decel is not None and decel < -cfg.max_slowdown_kph:
      decel = None

    # ---- 先补亏空更大的那一侧（人才这么想：哪边不够就先解决哪边）----
    prefer_behind = deficit_front >= deficit_rear
    ordered = []
    if prefer_behind:
      ordered = [(MODE_BEHIND, decel), (MODE_FRONT, accel)]
    else:
      ordered = [(MODE_FRONT, accel), (MODE_BEHIND, decel)]

    # 承诺期内坚持原方案（除非原方案已不可行）
    if self.commit_timer_s > 0 and self.mode in (MODE_FRONT, MODE_BEHIND):
      for mode, delta in ordered:
        if mode == self.mode and delta is not None:
          return self._make_plan(mode, delta, gap_front, gap_rear, needed, deficit_front, deficit_rear)

    for mode, delta in ordered:
      if delta is not None:
        return self._make_plan(mode, delta, gap_front, gap_rear, needed, deficit_front, deficit_rear)

    self._end_commit()
    return Plan(MODE_GIVEUP, 0.0,
                f"no feasible gap (need +{accel if accel else 0:.1f} / {decel if decel else 0:.1f})")

  def _make_plan(self, mode, delta, gap_front, gap_rear, needed, deficit_front, deficit_rear):
    self._commit(mode)
    return Plan(mode, delta,
                f"gap_f={gap_front} gap_r={gap_rear} need={needed:.1f} "
                f"def_f={deficit_front:.1f} def_r={deficit_rear:.1f}")

  def _commit(self, mode: str) -> None:
    """切换决策时重新开始承诺计时。"""
    if mode != self.mode:
      self.mode = mode
      self.commit_timer_s = self.cfg.commit_s

  def _end_commit(self) -> None:
    self.mode = MODE_NONE
    self.commit_timer_s = 0.0

  # ------------------------------------------------------------------ 恢复
  def _release(self, v_ego_kph: float, desired_kph: float, dt: float) -> float:
    """盲区解除：平滑回到正常车速，不 abrupt。"""
    cfg = self.cfg
    if self.base_kph is None:
      return desired_kph

    if self.hold_timer_s > 0:
      self.hold_timer_s -= dt
      return max(cfg.min_speed_kph, min(cfg.max_speed_kph, self.base_kph + self.delta_kph))

    if self.delta_kph > 0:
      # 之前是加速抢位：慢慢收回
      self.delta_kph = max(0.0, self.delta_kph - cfg.decel_limit_kphps * dt)
    else:
      # 之前是减速让行：慢慢提回去
      self.delta_kph = min(0.0, self.delta_kph + cfg.accel_limit_kphps * dt)

    target = min(self.base_kph, desired_kph) + self.delta_kph
    if abs(self.delta_kph) < 0.2:
      self.reset()
      return desired_kph
    return max(cfg.min_speed_kph, min(cfg.max_speed_kph, target))
