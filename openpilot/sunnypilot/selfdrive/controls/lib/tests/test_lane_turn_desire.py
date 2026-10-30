from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.parameterized import parameterized
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase

from openpilot.selfdrive.controls.lib.desire_helper import (DesireHelper, LaneChangeState, LaneChangeDirection,
                                                            AUTO_LC_CONFIRM_DELAY_SEC)
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController, LANE_CHANGE_SPEED_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


TurnDirection = custom.ModelDataV2SP.TurnDirection


class TestLaneTurnDesire(OpenpilotTestCase):
  @parameterized.expand([
    (True, False, 5, False, False, TurnDirection.turnLeft),
    (False, True, 6, False, False, TurnDirection.turnRight),
    (True, False, 9, False, False, TurnDirection.none),
    (True, False, 7, True, False, TurnDirection.none),
    (False, True, 6, False, True, TurnDirection.none),
    (False, False, 5, False, False, TurnDirection.none),
    (True, True, 5, False, False, TurnDirection.none),
  ])
  def test_lane_turn_desire_conditions(self, left_blinker, right_blinker, v_ego, blindspot_left, blindspot_right, expected):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(blindspot_left, blindspot_right, left_blinker, right_blinker, v_ego)
    assert controller.get_turn_direction() == expected

  def test_lane_turn_desire_disabled(self):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = False
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(False, False, True, False, 7)
    assert controller.get_turn_direction() == TurnDirection.none

  def test_lane_turn_overrides_lane_change(self):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    # left turn desire
    controller.update_lane_turn(False, False, True, False, 5)
    assert controller.get_turn_direction() == TurnDirection.turnLeft
    # right turn desire
    controller.update_lane_turn(False, False, False, True, 6)
    assert controller.get_turn_direction() == TurnDirection.turnRight
    # no turn
    controller.update_lane_turn(False, False, False, False, 7)
    assert controller.get_turn_direction() == TurnDirection.none

  @parameterized.expand([
    (8.93, TurnDirection.turnLeft),  # just below threshold
    (8.96, TurnDirection.none),  # above threshold
    (8.95, TurnDirection.none),  # just above threshold
  ])
  def test_lane_turn_desire_speed_boundary(self, v_ego, expected):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(False, True, True, False, v_ego)
    assert controller.get_turn_direction() == expected


class DummyCarState:
  def __init__(self, vEgo=0, leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
               steeringPressed=False, steeringTorque=0, brakePressed=False):
    self.vEgo = vEgo
    self.leftBlinker = leftBlinker
    self.rightBlinker = rightBlinker
    self.leftBlindspot = leftBlindspot
    self.rightBlindspot = rightBlindspot
    self.steeringPressed = steeringPressed
    self.steeringTorque = steeringTorque
    self.brakePressed = brakePressed


class DummyNavi:
  """`amapNavi` 的最小替身。

  ⚠️ `leftBlind`/`rightBlind` 是**位打包整数**，不是布尔（权威定义见
  carrot/amap_navi.py:250-253 与 ao_aa_bridge.py:30-40）：

      bit0 = lidar_*blind        LiDAR 判定该侧有障碍
      bit1 = *_blind             导航判定该侧盲区
      bit2 = lidar_car_*blind    LiDAR 判定车侧有障碍
      bit3 = *_lane > 0          该侧存在车道   ← 不是盲区，不能参与判断
  """
  def __init__(self, leftBlind=0, rightBlind=0):
    self.leftBlind = leftBlind
    self.rightBlind = rightBlind


def set_lane_turn_params():
  params = Params()
  params.put("LaneTurnDesire", True)
  params.put("LaneTurnValue", 20.0)


class TestDesireHelperIntegration(OpenpilotTestCase):
  @parameterized.expand([
    # Lane turn desire overrides lane change desire
    (DummyCarState(vEgo=5, leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=False), True, 1.0,
     log.Desire.turnLeft),
    (DummyCarState(vEgo=7, leftBlinker=False, rightBlinker=True, leftBlindspot=False, rightBlindspot=False), True, 1.0,
     log.Desire.turnRight),
    # Lane change desire only (no turn desires)
    (DummyCarState(vEgo=9, leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
                   steeringPressed=True, steeringTorque=1), True, 1.0, log.Desire.laneChangeLeft),
    (DummyCarState(vEgo=9, leftBlinker=False, rightBlinker=True, leftBlindspot=False, rightBlindspot=False,
                   steeringPressed=True, steeringTorque=-1), True, 1.0, log.Desire.laneChangeRight),
    # No desire (inactive)
    (DummyCarState(vEgo=9, leftBlinker=False, rightBlinker=False), False, 1.0, log.Desire.none),
    (DummyCarState(vEgo=4, leftBlinker=False, rightBlinker=False), True, 1.0, log.Desire.none),  # No blinkers? no desire!
  ], names=["carstate", "lateral_active", "lane_change_prob", "expected_desire"])
  def test_desire_helper_integration(self, carstate, lateral_active, lane_change_prob, expected_desire, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    for _ in range(10):
      dh.update(DummyNavi(), carstate, lateral_active, lane_change_prob,
                left_edge_detected=False, right_edge_detected=False)
    assert dh.desire == expected_desire

  def test_nudge_overrides_edge_detection(self, set_lane_turn_params):
    """Manual steering nudge should override road edge detection and allow lane change."""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=15, leftBlinker=True, steeringPressed=True, steeringTorque=1)
    for _ in range(10):
      dh.update(DummyNavi(), carstate, True, 1.0, left_edge_detected=True, right_edge_detected=False)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.desire == log.Desire.laneChangeLeft

  def test_nudge_does_not_override_bsm(self, set_lane_turn_params):
    """Manual steering nudge must NOT override BSM blindspot detection."""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=15, leftBlinker=True, leftBlindspot=True, steeringPressed=True, steeringTorque=1)
    for _ in range(10):
      dh.update(DummyNavi(), carstate, True, 1.0, left_edge_detected=False, right_edge_detected=False)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.desire == log.Desire.none

  def test_edge_blocks_auto_lane_change(self, set_lane_turn_params):
    """Road edge detection should still block auto (nudgeless) lane changes."""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    dh.alc.param_read_counter = 1
    carstate = DummyCarState(vEgo=15, leftBlinker=True)
    for _ in range(10):
      dh.update(DummyNavi(), carstate, True, 1.0, left_edge_detected=True, right_edge_detected=False)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.desire == log.Desire.none

  def test_auto_lane_change_allowed_without_edge(self, set_lane_turn_params):
    """Auto lane change should proceed when no edge and no BSM detected."""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    dh.alc.param_read_counter = 1
    carstate = DummyCarState(vEgo=15, leftBlinker=True)
    for _ in range(10):
      dh.update(DummyNavi(), carstate, True, 1.0, left_edge_detected=False, right_edge_detected=False)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_right_edge_does_not_block_left_lane_change(self, set_lane_turn_params):
    """Edge on opposite side should not affect lane change direction."""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    dh.alc.param_read_counter = 1
    carstate = DummyCarState(vEgo=15, leftBlinker=True)
    for _ in range(10):
      dh.update(DummyNavi(), carstate, True, 1.0, left_edge_detected=False, right_edge_detected=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_auto_request_blocked_by_edge(self, set_lane_turn_params):
    """[AUTO_OVERTAKE_WIRING] 自动变道请求不得驶向路沿 —— 即使确认窗口走满。

    上游的 `lane_change_blocked` 在 `torque_applied` 为真时会放过路沿（这是为了
    允许驾驶员轻拨覆盖）。而自动请求恰好在确认窗口走满时把 `torque_applied` 置真，
    所以自动路径必须用 `lane_unsafe`（盲区 OR 路沿）单独把关，否则会自动压向路沿。
    """
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    dh.alc.param_read_counter = 1
    carstate = DummyCarState(vEgo=15, leftBlinker=False, rightBlinker=False)
    for _ in range(int(AUTO_LC_CONFIRM_DELAY_SEC / DT_MDL) + 40):
      dh.update(DummyNavi(), carstate, True, 1.0,
                left_edge_detected=True, right_edge_detected=False,
                auto_lane_change_direction=LaneChangeDirection.left)
    assert dh.lane_change_state != LaneChangeState.laneChangeStarting

  def test_auto_request_allowed_without_edge(self, set_lane_turn_params):
    """同上，对照组：没有路沿时自动请求应当能走完确认窗口并开始变道。"""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    dh.alc.param_read_counter = 1
    carstate = DummyCarState(vEgo=15, leftBlinker=False, rightBlinker=False)
    for _ in range(int(AUTO_LC_CONFIRM_DELAY_SEC / DT_MDL) + 40):
      dh.update(DummyNavi(), carstate, True, 1.0,
                left_edge_detected=False, right_edge_detected=False,
                auto_lane_change_direction=LaneChangeDirection.left)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting


class TestNaviBlindBitDecode(OpenpilotTestCase):
  """回归测试：`navi.leftBlind` 必须按位解码，bit3（该侧存在车道）不得参与盲区判断。

  修复前 `carstate.leftBlindspot or navi.leftBlind` 把整数当布尔，
  bit3 让整数恒非零 ⇒ 低速打转向灯永远触发不了转向意图。
  """

  @parameterized.expand([
    (0, log.Desire.turnLeft),    # 无任何信息 -> 允许左转
    (8, log.Desire.turnLeft),    # bit3 左车道存在 -> **必须允许**（本 bug 的核心）
    (9, log.Desire.none),        # bit3 + bit0 LiDAR 盲区 -> 拦
    (10, log.Desire.none),       # bit3 + bit1 导航盲区 -> 拦
    (12, log.Desire.none),       # bit3 + bit2 LiDAR 车侧 -> 拦
    (1, log.Desire.none),        # 仅 bit0 -> 拦
    (2, log.Desire.none),        # 仅 bit1 -> 拦
    (4, log.Desire.none),        # 仅 bit2 -> 拦
  ], names=["navi_left", "expected"], ids=lambda navi_left, expected: f"navi{navi_left}")
  def test_navi_left_blind_bits(self, navi_left, expected, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=5, leftBlinker=True, rightBlinker=False,
                             leftBlindspot=False, rightBlindspot=False)
    for _ in range(10):
      dh.update(DummyNavi(leftBlind=navi_left), carstate, True, 1.0)
    assert dh.desire == expected

  @parameterized.expand([
    (0, log.Desire.turnRight),
    (8, log.Desire.turnRight),   # bit3 右车道存在 -> 必须允许
    (10, log.Desire.none),       # bit3 + bit1 导航盲区 -> 拦
    (2, log.Desire.none),
  ], names=["navi_right", "expected"], ids=lambda navi_right, expected: f"navi{navi_right}")
  def test_navi_right_blind_bits(self, navi_right, expected, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=5, leftBlinker=False, rightBlinker=True,
                             leftBlindspot=False, rightBlindspot=False)
    for _ in range(10):
      dh.update(DummyNavi(rightBlind=navi_right), carstate, True, 1.0)
    assert dh.desire == expected

  def test_navi_bit3_does_not_mask_real_bsm(self, set_lane_turn_params):
    """车自己的 BSM 仍然必须能拦 —— 修复不能顺手把车端盲区一起放过。"""
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=5, leftBlinker=True, leftBlindspot=True)
    for _ in range(10):
      dh.update(DummyNavi(leftBlind=8), carstate, True, 1.0)
    assert dh.desire == log.Desire.none


class TestLaneTurnValueClamp(OpenpilotTestCase):
  """`LaneTurnValue` 的速度阈值必须在 `__init__` 里就截断到 `LANE_CHANGE_SPEED_MIN`。

  历史上 `__init__` 少了这层 min()，只有 `read_params()` 做。
  因为 `DesireHelper.update()` 先调 `update_params()` 且 counter 初值为 0，
  首次调用就会 `read_params()`，所以未截断的初值从未参与判断 —— 但那是调用顺序的
  巧合。这里把它变成不变量，防止将来重排顺序后在任何车速下误判转向意图。
  """

  def test_clamped_at_init_when_over_limit(self):
    p = Params()
    p.put("LaneTurnDesire", True, block=True)
    p.put("LaneTurnValue", 60.0, block=True)   # 远超 UI 上限 20 mph
    dh = DesireHelper()
    assert dh.lane_turn_controller.lane_turn_value == LANE_CHANGE_SPEED_MIN

  def test_clamped_after_read_params_when_over_limit(self):
    p = Params()
    p.put("LaneTurnDesire", True, block=True)
    p.put("LaneTurnValue", 60.0, block=True)
    dh = DesireHelper()
    dh.lane_turn_controller.read_params()
    assert dh.lane_turn_controller.lane_turn_value == LANE_CHANGE_SPEED_MIN

  def test_unchanged_when_below_limit(self):
    p = Params()
    p.put("LaneTurnDesire", True, block=True)
    p.put("LaneTurnValue", 15.0, block=True)
    dh = DesireHelper()
    assert abs(dh.lane_turn_controller.lane_turn_value - 15.0 * CV.MPH_TO_MS) < 1e-9
    dh.lane_turn_controller.read_params()
    assert abs(dh.lane_turn_controller.lane_turn_value - 15.0 * CV.MPH_TO_MS) < 1e-9

  def test_init_and_read_params_agree(self):
    """两条路径必须给出同一个值 —— 这是本修复要锁住的核心不变量。"""
    p = Params()
    p.put("LaneTurnDesire", True, block=True)
    p.put("LaneTurnValue", 60.0, block=True)
    dh = DesireHelper()
    at_init = dh.lane_turn_controller.lane_turn_value
    dh.lane_turn_controller.read_params()
    assert at_init == dh.lane_turn_controller.lane_turn_value
