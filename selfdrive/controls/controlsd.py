#!/usr/bin/env python3
import math
import time
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.desire_helper import AUTO_LC_BLINKER_DELAY_SEC
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.selfdrive.livedelay.helpers import get_lat_delay
from openpilot.selfdrive.controls.lib.SceneUnderstanding_v2 import SceneUnderstanding
State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
SIDE_INTRUSION_STALE_TIMEOUT_S = 1.0
SIDE_INTRUSION_LANE_PROB_MIN = 0.65
SIDE_INTRUSION_MIN_DIST_M = 4.0
SIDE_INTRUSION_MAX_DIST_M = 35.0
SIDE_INTRUSION_LINE_MARGIN_M = 0.35
SIDE_INTRUSION_BASE_OFFSET_M = 0.16
SIDE_INTRUSION_MAX_CURVATURE = 0.0010
SIDE_INTRUSION_FILTER_ALPHA = 0.04
SIDE_INTRUSION_PERSON_CLASS = 0
SIDE_INTRUSION_VEHICLE_CLASSES = {1, 2, 3, 5, 7}
SIDE_INTRUSION_PERSON_SCORE_MIN = 0.30
SIDE_INTRUSION_VEHICLE_SCORE_MIN = 0.25
SIDE_INTRUSION_PERSON_HEIGHT_M = 1.7
SIDE_INTRUSION_VEHICLE_HEIGHT_M = 1.5


def _clamp(v: float, lo: float, hi: float) -> float:
  return max(float(lo), min(float(hi), float(v)))


def _interp(x: float, xp: list[float], fp: list[float]) -> float:
  if not xp or not fp or len(xp) != len(fp):
    return 0.0

  x_f = float(x)
  if x_f <= float(xp[0]):
    return float(fp[0])

  for i in range(1, len(xp)):
    x0 = float(xp[i - 1])
    x1 = float(xp[i])
    if x_f <= x1:
      y0 = float(fp[i - 1])
      y1 = float(fp[i])
      if x1 == x0:
        return y1
      return y0 + (y1 - y0) * (x_f - x0) / (x1 - x0)

  return float(fp[-1])


def _road_edge_detected(model_v2: log.ModelDataV2) -> tuple[bool, bool]:
  # Similar to dragonpilot RoadEdgeDetector, but used locally for lateral biasing.
  try:
    stds = list(getattr(model_v2, "roadEdgeStds", []) or [])
    probs = list(getattr(model_v2, "laneLineProbs", []) or [])
    if len(stds) < 2 or len(probs) < 4:
      return False, False

    left_road_edge_prob = _clamp(1.0 - float(stds[0]), 0.0, 1.0)
    right_road_edge_prob = _clamp(1.0 - float(stds[1]), 0.0, 1.0)
    left_lane_nearside_prob = float(probs[0])
    right_lane_nearside_prob = float(probs[3])

    nearside_prob_th = 0.2
    edge_prob_th = 0.35
    left_edge = bool(
      left_road_edge_prob > edge_prob_th and
      left_lane_nearside_prob < nearside_prob_th and
      right_lane_nearside_prob >= left_lane_nearside_prob
    )
    right_edge = bool(
      right_road_edge_prob > edge_prob_th and
      right_lane_nearside_prob < nearside_prob_th and
      left_lane_nearside_prob >= right_lane_nearside_prob
    )
    return left_edge, right_edge
  except Exception:
    return False, False


def _road_edge_lane_offset_curvature(model_v2: log.ModelDataV2, v_ego: float, left_edge: bool, right_edge: bool) -> float:
  if not (left_edge or right_edge):
    return 0.0

  try:
    lane_lines = list(getattr(model_v2, "laneLines", []) or [])
    lane_probs = list(getattr(model_v2, "laneLineProbs", []) or [])
    if len(lane_lines) < 3 or len(lane_probs) < 3:
      return 0.0

    left_prob = float(lane_probs[1])
    right_prob = float(lane_probs[2])
    prob = min(left_prob, right_prob)
    if prob < 0.65:
      return 0.0

    left = lane_lines[1]
    right = lane_lines[2]
    left_x = list(getattr(left, "x", []) or [])
    left_y = list(getattr(left, "y", []) or [])
    right_x = list(getattr(right, "x", []) or [])
    right_y = list(getattr(right, "y", []) or [])
    if len(left_x) < 2 or len(right_x) < 2:
      return 0.0

    lookahead_m = _clamp(float(v_ego) * 0.7 + 10.0, 10.0, 25.0)
    y_left = _interp(lookahead_m, left_x, left_y)
    y_right = _interp(lookahead_m, right_x, right_y)

    lane_width = float(y_right - y_left)
    if not (2.6 < lane_width < 4.6):
      return 0.0

    y_center = 0.5 * (float(y_left) + float(y_right))

    # Conservative edge bias: reduce lateral authority consumption on winding roads.
    base_offset_m = 0.12
    y_target = base_offset_m if left_edge else (-base_offset_m if right_edge else 0.0)
    max_off = max(0.0, 0.5 * lane_width - 0.25)
    y_target = _clamp(float(y_target), -max_off, max_off)

    y_err = float(y_center - y_target)
    correction = -2.0 * y_err / (lookahead_m ** 2)
    scale = _clamp((prob - 0.65) / 0.35, 0.0, 1.0)
    correction *= scale
    speed_scale = _clamp(_interp(float(v_ego), [8.0, 20.0, 35.0], [1.0, 0.75, 0.55]), 0.55, 1.0)
    correction *= speed_scale
    return _clamp(correction, -0.0012, 0.0012)
  except Exception:
    return 0.0


def _side_intrusion_curvature(model_v2: log.ModelDataV2, det_payload: dict | None, v_ego: float) -> float:
  if det_payload is None:
    return 0.0

  try:
    lane_lines = list(getattr(model_v2, "laneLines", []) or [])
    lane_probs = list(getattr(model_v2, "laneLineProbs", []) or [])
    if len(lane_lines) < 3 or len(lane_probs) < 3:
      return 0.0

    left_prob = float(lane_probs[1])
    right_prob = float(lane_probs[2])
    prob = min(left_prob, right_prob)
    if prob < SIDE_INTRUSION_LANE_PROB_MIN:
      return 0.0

    left = lane_lines[1]
    right = lane_lines[2]
    left_x = list(getattr(left, "x", []) or [])
    left_y = list(getattr(left, "y", []) or [])
    right_x = list(getattr(right, "x", []) or [])
    right_y = list(getattr(right, "y", []) or [])
    if len(left_x) < 2 or len(right_x) < 2:
      return 0.0

    img_w = int(det_payload.get("imgW", 0) or 0)
    focal_length_px = float(det_payload.get("focalLengthPx", 0.0) or 0.0)
    objs = det_payload.get("objs", None) or []
    if img_w <= 0 or focal_length_px <= 1.0 or not isinstance(objs, list):
      return 0.0

    lookahead_m = _clamp(float(v_ego) * 0.7 + 10.0, 10.0, 25.0)
    y_left_la = _interp(lookahead_m, left_x, left_y)
    y_right_la = _interp(lookahead_m, right_x, right_y)
    lane_width = float(y_right_la - y_left_la)
    if not (2.6 < lane_width < 4.6):
      return 0.0

    left_intrusion = False
    right_intrusion = False
    cx0 = float(img_w) * 0.5

    for o in objs:
      if not isinstance(o, dict):
        continue
      try:
        cls = int(o.get("c", -1))
        score = float(o.get("s", 0.0))
        x1 = float(o.get("x1", 0.0))
        y1 = float(o.get("y1", 0.0))
        x2 = float(o.get("x2", 0.0))
        y2 = float(o.get("y2", 0.0))
      except Exception:
        continue

      if cls == SIDE_INTRUSION_PERSON_CLASS:
        score_min = SIDE_INTRUSION_PERSON_SCORE_MIN
        obj_height_m = SIDE_INTRUSION_PERSON_HEIGHT_M
      elif cls in SIDE_INTRUSION_VEHICLE_CLASSES:
        score_min = SIDE_INTRUSION_VEHICLE_SCORE_MIN
        obj_height_m = SIDE_INTRUSION_VEHICLE_HEIGHT_M
      else:
        continue
      if score < score_min:
        continue
      if not (math.isfinite(x1) and math.isfinite(y1) and math.isfinite(x2) and math.isfinite(y2)):
        continue
      if x2 <= x1 or y2 <= y1:
        continue

      h_px = max(1.0, y2 - y1)
      dist_m = (float(focal_length_px) * obj_height_m) / h_px
      if not math.isfinite(dist_m) or dist_m < SIDE_INTRUSION_MIN_DIST_M or dist_m > SIDE_INTRUSION_MAX_DIST_M:
        continue

      y_left = _interp(dist_m, left_x, left_y)
      y_right = _interp(dist_m, right_x, right_y)
      left_boundary = min(float(y_left), float(y_right))
      right_boundary = max(float(y_left), float(y_right))
      lane_center = 0.5 * (left_boundary + right_boundary)

      cx = 0.5 * (x1 + x2)
      y_center = (cx - cx0) * dist_m / max(focal_length_px, 1.0)
      y_inner_left_obj = (x2 - cx0) * dist_m / max(focal_length_px, 1.0)
      y_inner_right_obj = (x1 - cx0) * dist_m / max(focal_length_px, 1.0)

      if y_center < lane_center and y_inner_left_obj >= (left_boundary - SIDE_INTRUSION_LINE_MARGIN_M):
        left_intrusion = True
      elif y_center > lane_center and y_inner_right_obj <= (right_boundary + SIDE_INTRUSION_LINE_MARGIN_M):
        right_intrusion = True

    if left_intrusion == right_intrusion:
      return 0.0

    # Reuse the road-edge bias sign convention: left-side risk biases away from the left boundary.
    y_target = SIDE_INTRUSION_BASE_OFFSET_M if left_intrusion else -SIDE_INTRUSION_BASE_OFFSET_M
    max_off = max(0.0, 0.5 * lane_width - 0.35)
    y_target = _clamp(float(y_target), -max_off, max_off)
    y_center_la = 0.5 * (float(y_left_la) + float(y_right_la))
    correction = -2.0 * float(y_center_la - y_target) / (lookahead_m ** 2)
    lane_scale = _clamp((prob - SIDE_INTRUSION_LANE_PROB_MIN) / max(1e-3, 1.0 - SIDE_INTRUSION_LANE_PROB_MIN), 0.0, 1.0)
    speed_scale = _clamp(_interp(float(v_ego), [8.0, 20.0, 35.0], [1.0, 0.75, 0.50]), 0.50, 1.0)
    correction *= lane_scale * speed_scale
    return _clamp(correction, -SIDE_INTRUSION_MAX_CURVATURE, SIDE_INTRUSION_MAX_CURVATURE)
  except Exception:
    return 0.0


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2','radarState',
    'lidarState', 'selfdriveState', 'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                    'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'customReservedRawData0'], poll='carState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState', 'dpControlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    # Sensor Fusion Scene Module
    self.scene_understanding = SceneUnderstanding()
    self.scene_hint = {}
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

    self.alka_enabled = self.params.get_bool("dp_lat_alka")
    self.alka_active = False

    self._road_edge_curv_correction = 0.0
    self._side_intrusion_curv_correction = 0.0
    self._side_intrusion_det_payload: dict | None = None
    self._side_intrusion_last_update_t = 0.0
    self._dp_auto_avoid_enabled = self.params.get_bool("dp_lincoln_auto_avoid")
    self._dp_auto_avoid_last_param_check = 0.0
    self._auto_lc_blinker_delay_until = 0.0
    self._auto_lc_blinker_pending = False
    self._auto_lc_last_state = LaneChangeState.off

  def update(self):
    self.sm.update(15)
    # ==========================================
    # Scene Understanding Update
    # Camera + MR76 Radar + Lidar
    # ==========================================
    try:
      scene_type, objects, road = \
          self.scene_understanding.update(self.sm)
      self.scene_hint = \
          self.scene_understanding.get_decision_hint()
    except Exception as e:
      cloudlog.warning(
          f"SceneUnderstanding error: {e}"
      )
      self.scene_hint = {}
    now_mono = time.monotonic()
    if now_mono - self._dp_auto_avoid_last_param_check >= 1.0:
      self._dp_auto_avoid_last_param_check = now_mono
      try:
        self._dp_auto_avoid_enabled = self.params.get_bool("dp_lincoln_auto_avoid")
      except Exception:
        self._dp_auto_avoid_enabled = False

    if self.sm.updated.get("customReservedRawData0", False):
      try:
        raw = self.sm["customReservedRawData0"]
        payload = decode_cone_detections(raw) if raw else None
        if payload is not None:
          self._side_intrusion_det_payload = payload
          self._side_intrusion_last_update_t = now_mono
      except Exception:
        self._side_intrusion_det_payload = None
        self._side_intrusion_last_update_t = 0.0

    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']
    scene_hint = self.scene_hint

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    scene_speed_limit = \
        scene_hint.get(
            "speed_limit",
            None
        )
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    self.alka_active = self.alka_enabled and CS.cruiseState.available and not standstill and CS.gearShifter != car.CarState.GearShifter.reverse
    lat_active = self.sm['selfdriveState'].active or self.alka_active
    CC.latActive = lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    lc_state = model_v2.meta.laneChangeState
    lc_dir = model_v2.meta.laneChangeDirection
    one_blinker = CS.leftBlinker != CS.rightBlinker
    now_mono = time.monotonic()

    if lc_state == LaneChangeState.off:
      self._auto_lc_blinker_delay_until = 0.0
      self._auto_lc_blinker_pending = False
    elif self._auto_lc_last_state == LaneChangeState.off and lc_state == LaneChangeState.preLaneChange:
      if not one_blinker:
        self._auto_lc_blinker_delay_until = now_mono + AUTO_LC_BLINKER_DELAY_SEC
        self._auto_lc_blinker_pending = True

    # Enable blinkers while lane changing (auto requests can delay briefly so voice leads).
    if lc_state != LaneChangeState.off:
      if lc_state != LaneChangeState.preLaneChange:
        self._auto_lc_blinker_pending = False

      allow_blinker = True
      if self._auto_lc_blinker_pending and now_mono < self._auto_lc_blinker_delay_until:
        allow_blinker = False
      else:
        self._auto_lc_blinker_pending = False

      if allow_blinker:
        CC.leftBlinker = lc_dir == LaneChangeDirection.left
        CC.rightBlinker = lc_dir == LaneChangeDirection.right

    self._auto_lc_last_state = lc_state

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    # =====================================
    # OVA Oncoming Vehicle Avoidance
    # =====================================
    if scene_hint.get("oncoming", False):
        if CS.vEgo > 10:
        # 向右轻微避让
            new_desired_curvature += 0.0005

    # Ford/Lincoln: when driving in the outermost lane (road edge/guardrail), bias away slightly to avoid
    # hugging the edge. The same Auto avoidance switch also enables side-intrusion bias when an adjacent
    # vehicle approaches the current lane boundary; both are lane-within corrections, not lane changes.
    auto_avoid_enabled = bool(
        self._dp_auto_avoid_enabled
        or
        scene_hint.get(
            "avoid",
            False
        )
    )
    lc_state = getattr(model_v2.meta, "laneChangeState", LaneChangeState.off)
    if (not CC.latActive) or getattr(self.CP, "brand", "") != "ford" or CS.leftBlinker or CS.rightBlinker or lc_state != LaneChangeState.off:
      self._road_edge_curv_correction = 0.0
      self._side_intrusion_curv_correction = 0.0
    else:
      raw_corr = 0.0
      raw_side_corr = 0.0
      if CS.vEgo > 8.0:
        left_edge, right_edge = _road_edge_detected(model_v2)
        raw_corr = _road_edge_lane_offset_curvature(model_v2, CS.vEgo, left_edge, right_edge)
        det_fresh = (time.monotonic() - self._side_intrusion_last_update_t) <= SIDE_INTRUSION_STALE_TIMEOUT_S
        if auto_avoid_enabled and det_fresh:
          raw_side_corr = _side_intrusion_curvature(model_v2, self._side_intrusion_det_payload, CS.vEgo)

      alpha = 0.02  # ~0.5s time constant @100Hz
      self._road_edge_curv_correction = (1.0 - alpha) * float(self._road_edge_curv_correction) + alpha * float(raw_corr)
      if auto_avoid_enabled:
        self._side_intrusion_curv_correction = (
          (1.0 - SIDE_INTRUSION_FILTER_ALPHA) * float(self._side_intrusion_curv_correction) +
          SIDE_INTRUSION_FILTER_ALPHA * float(raw_side_corr)
        )
      else:
        self._side_intrusion_curv_correction = 0.0
      new_desired_curvature = float(new_desired_curvature) + float(self._road_edge_curv_correction) + float(self._side_intrusion_curv_correction)

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = get_lat_delay(self.params, self.sm["liveDelay"].lateralDelay, self.CP.steerActuatorDelay) + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay)
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # dpControlsState
    if self.scene_hint:
        cloudlog.info(
            f"""
    Scene:
    {self.scene_hint}
    """
        )
    dat = messaging.new_message('dpControlsState')
    dat.valid = True
    ncs = dat.dpControlsState
    ncs.alkaActive = self.alka_active
    self.pm.send('dpControlsState', dat)

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
