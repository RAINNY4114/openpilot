from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# ============================================================================
# [AUTO_OVERTAKE_WIRING] 自动变道（AO/AA）接入 Desire 状态机
#
# AoAaBridge（selfdrive/controls/lib/ao_aa_bridge.py，由 modeld 调用）把想要
# 变去的方向交给本状态机，语义上等同于驾驶员打转向灯，但有三处刻意差异：
#
#   1. 动作要等 AUTO_LC_CONFIRM_DELAY_SEC 秒才真正开始。这段窗口里转向灯
#      已经亮了（controlsd 从 laneChangeState 推 CC.left/rightBlinker），
#      驾驶员始终有可见、可取消的确认期。
#   2. **我们自己点亮的转向灯不能被当成驾驶员输入**。否则状态机会自激，
#      确认窗口形同虚设。见下方 `driver_blinker`。
#   3. ALC 的轻拨计时器不得提前发起自动变道 —— 只有确认窗口（或驾驶员）能。
#
# ⚠️ 与上游版本的差异：本段逻辑 + 下方 [LEFTBLIND_BITFIX] 的位解码修正。
#    `navi` 参数仍按上游传入，但不再把位打包整数当布尔用。
#
# [UPSTREAM_MERGE_20261028] 已并入上游 sunnypilot v2026.003.000（bb77cbe）的三处语义：
#    (a) 盲区与路沿拆成 blindspot_detected / edge_detected 两个信号；
#    (b) `alc.update_lane_change()` 改为 4 参（新增 edge_detected / torque_applied）；
#    (c) 放行判据改为 `not self.alc.lane_change_blocked`（BSM 恒拦、路沿可被轻拨覆盖）。
#    自动变道路径额外用 `lane_unsafe` 自查 —— 理由见下方调用点注释。
# ============================================================================
AUTO_LC_CONFIRM_DELAY_SEC = 3.0

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    # [AUTO_OVERTAKE_WIRING] 自动变道请求记账
    self.prev_auto_request = False
    self._auto_requested = False
    self._auto_dir = LaneChangeDirection.none
    self._auto_confirm_timer = 0.0
    self._auto_brake_blocked = False
    self._auto_blinker_release = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _clear_auto(self):
    self._auto_requested = False
    self._auto_dir = LaneChangeDirection.none
    self._auto_confirm_timer = 0.0
    self._auto_brake_blocked = False

  def update(self, navi, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             auto_lane_change_direction=None, auto_confirm_delay_sec=None):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    # [LEFTBLIND_BITFIX] `navi.left/rightBlind` 是**位打包整数**，不是布尔：
    #   bit0 = LiDAR 该侧盲区、bit1 = 导航该侧盲区、bit2 = LiDAR 车侧障碍、bit3 = 该侧存在车道
    # 直接 `or navi.leftBlind` 会让 bit3 污染判断 —— 多车道道路上 bit3 几乎恒为 1，
    # 而它恰恰是「能往该侧转弯」的前提。整数非零 ⇒ `not blindspot_left` 恒为假 ⇒
    # 低速打转向灯永远触发不了左/右转意图。这里只保留 bit0/1/2（真正的盲区语义），
    # 剔除 bit3。位定义见 carrot/amap_navi.py:250-253 与 ao_aa_bridge.py:30-40。
    navi_left_bs = bool(int(navi.leftBlind) & 0b0111)
    navi_right_bs = bool(int(navi.rightBlind) & 0b0111)
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot or navi_left_bs,
                                               blindspot_right=carstate.rightBlindspot or navi_right_bs,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    # ------------------------------------------------------------------
    # [AUTO_OVERTAKE_WIRING] 解析自动变道请求
    # ------------------------------------------------------------------
    auto_dir = auto_lane_change_direction
    blinker_dir = LaneChangeDirection.none
    if carstate.leftBlinker:
      blinker_dir = LaneChangeDirection.left
    elif carstate.rightBlinker:
      blinker_dir = LaneChangeDirection.right

    # `was_auto` 是上一帧的决策：它告诉我们此刻看到的转向灯是不是我们自己点的。
    was_auto = self._auto_requested
    auto_owns_blinker = was_auto and blinker_dir == self._auto_dir
    driver_blinker = one_blinker and not auto_owns_blinker

    # 请求要能扛住我们自己点亮的转向灯（blinker_dir == auto_dir），
    # 但驾驶员打了反方向就立刻作废。
    auto_request = (auto_dir in (LaneChangeDirection.left, LaneChangeDirection.right)
                    and blinker_dir in (LaneChangeDirection.none, auto_dir))

    confirm_delay = AUTO_LC_CONFIRM_DELAY_SEC if auto_confirm_delay_sec is None else float(auto_confirm_delay_sec)
    if confirm_delay < 0.0:
      confirm_delay = 0.0

    blinker_rising = driver_blinker and not self.prev_one_blinker
    auto_rising = auto_request and not self.prev_auto_request

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self._clear_auto()
    else:
      if self.lane_change_state == LaneChangeState.off and not self._auto_blinker_release \
         and (blinker_rising or auto_rising) and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # 驾驶员的转向灯永远优先于自动请求
        self._auto_requested = bool(auto_rising and not blinker_rising)
        self._auto_dir = auto_dir if self._auto_requested else LaneChangeDirection.none
        self._auto_confirm_timer = 0.0
        self._auto_brake_blocked = bool(self._auto_requested and carstate.brakePressed)
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self._auto_dir if self._auto_requested else self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if was_auto:
          if auto_request:
            # Update lane change direction
            self.lane_change_direction = self._auto_dir
          elif driver_blinker:
            # 驾驶员接管了这次变道：保留动作，但改由他驱动
            self._clear_auto()
          else:
            # 请求撤回，且没有别的东西在维持这次变道
            self._clear_auto()
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_timer = 0.0

        if self.lane_change_state == LaneChangeState.preLaneChange and not self._auto_requested:
          # Update lane change direction (driver-driven lane change)
          self.lane_change_direction = self.get_lane_change_direction(carstate)

        if self.lane_change_state == LaneChangeState.preLaneChange:
          torque_applied = carstate.steeringPressed and \
                           ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

          # [UPSTREAM_MERGE_20261028] 上游把「盲区」与「路沿」拆成两个信号：
          #   blindspot_detected —— 车端 BSM，任何情况下都拦
          #   edge_detected      —— 路沿/护栏，自动变道要拦，但驾驶员轻拨可以覆盖
          # 判据也由 `not blindspot_detected` 换成 `not self.alc.lane_change_blocked`
          # （合成规则在 auto_lane_change.py：BSM 恒拦，路沿在 torque_applied 时放行）。
          blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                                (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

          edge_detected = ((left_edge_detected and self.lane_change_direction == LaneChangeDirection.left) or
                           (right_edge_detected and self.lane_change_direction == LaneChangeDirection.right))

          self.alc.update_lane_change(blindspot_detected, edge_detected, carstate.brakePressed, torque_applied)

          # [AUTO_OVERTAKE_WIRING] 确认窗口。只有目标车道清晰可用的帧才计入
          # 延时；刹车永久锁死请求，盲区只暂停计时。
          #
          # ⚠️ 这里刻意用 `lane_unsafe`（盲区 **OR** 路沿）而不是只看盲区：
          #    `lane_change_blocked` 在 torque_applied 为真时会放过路沿（为了允许
          #    驾驶员轻拨覆盖），而自动请求恰恰会在确认窗口走满时把 torque_applied
          #    置真 —— 若沿用它，自动变道就会主动压向路沿。自动路径必须自己把关。
          lane_unsafe = blindspot_detected or edge_detected
          if self._auto_requested:
            if carstate.brakePressed:
              self._auto_brake_blocked = True
            if not lane_unsafe:
              self._auto_confirm_timer += DT_MDL
            if not self._auto_brake_blocked and not lane_unsafe and self._auto_confirm_timer >= confirm_delay:
              torque_applied = True

          # 自动请求只由模块本身维持 —— 绝不由我们自己点的转向灯维持
          alive = True if self._auto_requested else (driver_blinker or auto_request)
          # ...ALC 轻拨计时器也不得提前发起自动请求
          alc_allowed = self.alc.auto_lane_change_allowed and not self._auto_requested

          if not alive or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_timer = 0.0
            self._clear_auto()
          elif (torque_applied or alc_allowed) and not self.alc.lane_change_blocked:
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          was_auto_here = self._auto_requested
          self._clear_auto()
          # 自动变道只变一条车道：不要让我们自己点的转向灯继续维持状态机
          if driver_blinker and not was_auto_here:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    # [AUTO_OVERTAKE_WIRING] 自动变道结束后我们点的转向灯还会物理亮一小会儿；
    # 忽略它直到熄灭，否则状态机会立刻开始第二次变道。
    if was_auto and one_blinker:
      self._auto_blinker_release = True
    if not one_blinker:
      self._auto_blinker_release = False

    self.prev_one_blinker = driver_blinker and lateral_active
    self.prev_auto_request = auto_request and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
