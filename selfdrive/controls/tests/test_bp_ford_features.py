from types import SimpleNamespace

from cereal import log

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from opendbc.car.ford.carcontroller import CarController


class TestBPFordFeatures:
  def setup_method(self):
    self.params = Params()
    self._keys = [
      "BlinkerPauseLaneChange",
      "BlinkerMinLateralControlSpeed",
      "BlinkerLateralReengageDelay",
      "disable_BP_lat_UI",
      "disable_BP_long_UI",
      "disable_downhill_comp_UI",
      "enable_lane_positioning",
      "custom_path_offset",
      "enable_lane_full_mode",
      "custom_profile",
      "pc_blend_ratio_high_C_UI",
      "pc_blend_ratio_low_C_UI",
      "LC_PID_gain_UI",
      "BPShowConfidenceBall",
      "FordPrefShowRadarLeadOverlay",
      "FordPrefRadarOverlaySize",
    ]
    for key in self._keys:
      self.params.remove(key)

  def teardown_method(self):
    for key in self._keys:
      self.params.remove(key)

  def test_bp_ford_params_defaults(self):
    assert self.params.get("BPShowConfidenceBall", return_default=True) is True
    assert self.params.get("BlinkerPauseLaneChange", return_default=True) is False
    assert self.params.get("BlinkerMinLateralControlSpeed", return_default=True) == 20
    assert self.params.get("BlinkerLateralReengageDelay", return_default=True) == 0
    assert self.params.get("disable_BP_lat_UI", return_default=True) is False
    assert self.params.get("disable_BP_long_UI", return_default=True) is False
    assert self.params.get("disable_downhill_comp_UI", return_default=True) is False
    assert self.params.get("enable_lane_positioning", return_default=True) is False
    assert self.params.get("custom_path_offset", return_default=True) == 0.0
    assert self.params.get("enable_lane_full_mode", return_default=True) is False
    assert self.params.get("custom_profile", return_default=True) == 0
    assert self.params.get("pc_blend_ratio_high_C_UI", return_default=True) == 0.4
    assert self.params.get("pc_blend_ratio_low_C_UI", return_default=True) == 0.4
    assert self.params.get("LC_PID_gain_UI", return_default=True) == 3.0
    assert self.params.get("FordPrefShowRadarLeadOverlay", return_default=True) is False
    assert self.params.get("FordPrefRadarOverlaySize", return_default=True) == 1

  def test_blinker_pause_lateral_below_speed_and_delay(self):
    self.params.put_bool("BlinkerPauseLaneChange", True)
    self.params.put("BlinkerMinLateralControlSpeed", 20)
    self.params.put("BlinkerLateralReengageDelay", 1)
    self.params.put_bool("IsMetric", False)

    helper = BlinkerPauseLateral(en_param="BlinkerPauseLaneChange")
    helper.get_params()

    cs = SimpleNamespace(vEgo=4.0, leftBlinker=True, rightBlinker=False)
    assert helper.update(cs, dt_ctrl=0.1) is True

    cs = SimpleNamespace(vEgo=4.0, leftBlinker=False, rightBlinker=False)
    assert helper.update(cs, dt_ctrl=0.5) is True
    assert helper.update(cs, dt_ctrl=0.5) is False

  def test_desire_helper_respects_blinker_pause_lane_change(self):
    self.params.put_bool("BlinkerPauseLaneChange", True)
    self.params.put("BlinkerMinLateralControlSpeed", 20)
    self.params.put("BlinkerLateralReengageDelay", 0)
    self.params.put_bool("IsMetric", False)

    dh = DesireHelper(dp_lat_lca_speed=0)
    cs = SimpleNamespace(
      vEgo=4.0,
      leftBlinker=True,
      rightBlinker=False,
      leftBlindspot=False,
      rightBlindspot=False,
      steeringPressed=False,
      steeringTorque=0.0,
    )
    dh.update(cs, lateral_active=True, lane_change_prob=1.0, left_edge_detected=False, right_edge_detected=False)
    assert dh.lane_change_state == log.LaneChangeState.off
    assert dh.desire == log.Desire.none

  def test_ford_controller_initializes_with_bp_features(self):
    CP = CarInterface.get_non_essential_params(CAR.FORD_ESCAPE_MK4)
    cc = CarController(DBC[CP.carFingerprint], CP)
    assert cc.disable_BP_long_UI is False
    assert cc.disable_BP_lat_UI is False
