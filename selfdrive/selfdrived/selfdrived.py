#!/usr/bin/env python3
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log
from msgq.visionipc import VisionIpcClient, VisionStreamType


from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.common.gps import get_gps_location_service

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager, set_offroad_alert

from openpilot.system.version import get_build_metadata
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware_diagnostics import append_hardware_diagnostic
from openpilot.system.sensord.imu import probe_lsm6ds3
from opendbc.safety import ALTERNATIVE_EXPERIENCE

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ

LONGITUDINAL_PERSONALITY_MAP = {v: k for k, v in log.LongitudinalPersonality.schema.enumerants.items()}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.SelfdriveState.OpenpilotState
PandaType = log.PandaState.PandaType
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)


class SelfdriveD:
  def __init__(self, CP=None):
    self.params = Params()

    # Ensure the current branch is cached, otherwise the first cycle lags
    build_metadata = get_build_metadata()

    if CP is None:
      cloudlog.info("selfdrived is waiting for CarParams")
      self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
      cloudlog.info("selfdrived got CarParams")
    else:
      self.CP = CP

    self.car_events = CarSpecificEvents(self.CP)

    self.alka = bool(self.CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ALKA)

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    self.excessive_actuation_check = ExcessiveActuationCheck()
    self.excessive_actuation = self.params.get("Offroad_ExcessiveActuation") is not None

    # Setup sockets
    self.pm = messaging.PubMaster(['selfdriveState', 'onroadEvents'])

    self.gps_location_service = get_gps_location_service(self.params)
    self.gps_packets = [self.gps_location_service]
    self.imu_available, self.imu_probe_details = probe_lsm6ds3()
    self.sensor_packets = ["accelerometer", "gyroscope"] if self.imu_available else []
    self.camera_packets = ["roadCameraState", "driverCameraState", "wideRoadCameraState"]

    # TODO: de-couple selfdrived with card/conflate on carState without introducing controls mismatches
    self.car_state_sock = messaging.sub_sock('carState', timeout=20)

    ignore = self.sensor_packets + self.gps_packets + ['alertDebug'] + ['modelExt']
    if SIMULATION:
      ignore += ['driverCameraState', 'managerState']
    if REPLAY:
      # no vipc in replay will make them ignored anyways
      ignore += ['roadCameraState', 'wideRoadCameraState']
    if "DISABLE_DRIVER" in os.environ or "LITE" in os.environ:
      ignore += ['driverCameraState']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                   'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'livePose', 'liveDelay',
                                   'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
                                   'controlsState', 'carControl', 'driverAssistance', 'alertDebug', 'userBookmark', 'audioFeedback', 'modelExt'] + \
                                   self.camera_packets + self.sensor_packets + self.gps_packets,
                                  ignore_alive=ignore, ignore_avg_freq=ignore,
                                  ignore_valid=ignore, frequency=int(1/DT_CTRL))

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")

    car_recognized = self.CP.brand != 'mock'

    # cleanup old params
    if not self.CP.alphaLongitudinalAvailable:
      self.params.remove("AlphaLongitudinalEnabled")
    if not self.CP.openpilotLongitudinalControl:
      self.params.remove("ExperimentalMode")

    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.initialized = False
    self.enabled = False
    self.active = False
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.last_steering_pressed_frame = 0
    self._last_steer_saturated_log_t = 0.0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.logged_comm_issue = None
    self.not_running_prev = None
    self.experimental_mode = False
    self.personality = self.params.get("LongitudinalPersonality", return_default=True)
    self.recalibrating_seen = False
    self.state_machine = StateMachine(self.alka)
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # dp
    self.dp_lat_road_edge_detection_cooldown: float = 0.

    # Determine startup event
    self.startup_event = EventName.startup #if build_metadata.openpilot.comma_remote and build_metadata.tested_channel else EventName.startupMaster
    if HARDWARE.get_device_type() == 'mici':
      self.startup_event = None
    if not car_recognized:
      self.startup_event = EventName.startupNoCar
    elif car_recognized and self.CP.passive:
      self.startup_event = EventName.startupNoControl
    elif self.CP.secOcRequired and not self.CP.secOcKeyAvailable:
      self.startup_event = EventName.startupNoSecOcKey

    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      set_offroad_alert("Offroad_CarUnrecognized", True)
    elif self.CP.passive:
      self.events.add(EventName.dashcamMode, static=True)

  def _service_health(self, service: str) -> dict[str, object]:
    seen = bool(self.sm.seen.get(service, False))
    last_seen_sec = None if not seen else round(max(0., (self.sm.frame - self.sm.recv_frame[service]) * DT_CTRL), 3)
    return {
      "seen": seen,
      "last_seen_sec": last_seen_sec,
      "recv_frame": self.sm.recv_frame.get(service),
      "alive": self.sm.alive.get(service),
      "freq_ok": self.sm.freq_ok.get(service),
      "valid": self.sm.valid.get(service),
    }

  def _log_hardware_issue(self, issue: str, details: dict[str, object] | None = None,
                          dedupe_suffix: str | None = None, min_interval_sec: float = 60.) -> None:
    diagnostic = {
      "device_type": HARDWARE.get_device_type(),
      "frame": self.sm.frame,
      "car_brand": self.CP.brand,
      "car_fingerprint": getattr(self.CP, "carFingerprint", ""),
      "enabled": self.enabled,
      "active": self.active,
    }
    if details:
      diagnostic.update(details)

    suffix = f":{dedupe_suffix}" if dedupe_suffix else ""
    append_hardware_diagnostic(
      "selfdrived",
      issue,
      diagnostic,
      dedupe_key=f"selfdrived:{issue}{suffix}",
      min_interval_sec=min_interval_sec,
    )

  def update_events(self, CS):
    """Compute onroadEvents from carState"""

    self.events.clear()

    if self.sm['controlsState'].lateralControlState.which() == 'debugState':
      self.events.add(EventName.joystickDebug)
      self.startup_event = None

    if self.sm.recv_frame['alertDebug'] > 0:
      self.events.add(EventName.longitudinalManeuver)
      self.startup_event = None

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.selfdriveInitializing)
      return

    # Check for user bookmark press (bookmark button or end of LKAS button feedback)
    if self.sm.updated['userBookmark']:
      self.events.add(EventName.userBookmark)

    if self.sm.updated['audioFeedback']:
      self.events.add(EventName.audioFeedback)

    # Don't add any more events while in dashcam mode
    if self.CP.passive:
      return

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and CS.vCruise > 250 and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    if not self.CP.notCar:
      self.events.add_from_msg(self.sm['driverMonitoringState'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      car_events = self.car_events.update(CS, self.CS_prev, self.sm['carControl']).to_msg()
      self.events.add_from_msg(car_events)
      if CS.vehicleSensorsInvalid:
        self._log_hardware_issue("vehicleSensorsInvalid", {
          "can_valid": bool(CS.canValid),
          "can_timeout": bool(CS.canTimeout),
          "can_error_counter": int(CS.canErrorCounter),
          "steering_angle_deg": float(CS.steeringAngleDeg),
          "steering_rate_deg": float(CS.steeringRateDeg),
          "v_ego": float(CS.vEgo),
          "likely_reason": "Vehicle-reported steering/vehicle sensor data is invalid; check CAN wiring and steering angle related signals",
        }, min_interval_sec=60.)

      if self.CP.notCar:
        # wait for everything to init first
        if self.sm.frame > int(5. / DT_CTRL) and self.initialized:
          # body always wants to enable
          self.events.add(EventName.pcmEnable)

      # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
      if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
        (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
        self.events.add(EventName.pedalPressed)

    # Create events for temperature, disk space, and memory
    device_state = self.sm['deviceState']
    if device_state.thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
      self._log_hardware_issue("overheat", {
        "thermal_status": str(device_state.thermalStatus),
        "max_temp_c": float(device_state.maxTempC),
        "cpu_temp_c": list(device_state.cpuTempC),
        "gpu_temp_c": list(device_state.gpuTempC),
        "memory_temp_c": float(device_state.memoryTempC),
        "pmic_temp_c": list(device_state.pmicTempC),
      }, min_interval_sec=60.)
    if device_state.freeSpacePercent < 7 and not SIMULATION:
      self.events.add(EventName.outOfSpace)
      self._log_hardware_issue("outOfSpace", {
        "free_space_percent": float(device_state.freeSpacePercent),
      }, min_interval_sec=300.)
    if device_state.memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)
      self._log_hardware_issue("lowMemory", {
        "memory_usage_percent": int(device_state.memoryUsagePercent),
      }, min_interval_sec=300.)

    # Alert if fan isn't spinning for 5 seconds
    peripheral_state = self.sm['peripheralState']
    if peripheral_state.pandaType != log.PandaState.PandaType.unknown:
      if peripheral_state.fanSpeedRpm < 500 and device_state.fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
          self._log_hardware_issue("fanMalfunction", {
            "fan_speed_rpm": int(peripheral_state.fanSpeedRpm),
            "fan_speed_percent_desired": int(device_state.fanSpeedPercentDesired),
            "panda_type": str(peripheral_state.pandaType),
          }, min_interval_sec=60.)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        if not self.recalibrating_seen:
          set_offroad_alert("Offroad_Recalibration", True)
        self.recalibrating_seen = True
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Lane departure warning
    if self.is_ldw_enabled and self.sm.valid['driverAssistance']:
      if self.sm['driverAssistance'].leftLaneDeparture or self.sm['driverAssistance'].rightLaneDeparture:
        self.events.add(EventName.ldw)

    # ******************************************************************************************
    #  NOTE: To fork maintainers.
    #  Disabling or nerfing safety features will get you and your users banned from our servers.
    #  We recommend that you do not change these numbers from the defaults.
    if self.sm.updated['liveCalibration']:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated['livePose']:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    if self.calibrated_pose is not None:
      excessive_actuation = self.excessive_actuation_check.update(self.sm, CS, self.calibrated_pose)
      if not self.excessive_actuation and excessive_actuation is not None:
        set_offroad_alert("Offroad_ExcessiveActuation", True, extra_text=str(excessive_actuation))
        self.excessive_actuation = True

    if self.excessive_actuation:
      self.events.add(EventName.excessiveActuation)
    # ******************************************************************************************

    # Handle lane change
    if self.sm['modelV2'].meta.laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['modelV2'].meta.laneChangeDirection
      if ((CS.leftBlindspot or self.sm['modelExt'].leftEdgeDetected) and direction == LaneChangeDirection.left) or \
         ((CS.rightBlindspot or self.sm['modelExt'].rightEdgeDetected) and direction == LaneChangeDirection.right):
        self.dp_lat_road_edge_detection_cooldown = time.monotonic() + 0.5
        if time.monotonic() <= self.dp_lat_road_edge_detection_cooldown:
          self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['modelV2'].meta.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                    LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < len(self.CP.safetyConfigs):
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      # safety mismatch allows some time for pandad to set the safety mode and publish it back from panda
      if (safety_mismatch and self.sm.frame*DT_CTRL > 10.) or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)
        details = {
          "panda_index": i,
          "panda_safety_model": str(pandaState.safetyModel),
          "panda_safety_param": int(pandaState.safetyParam),
          "panda_alternative_experience": int(pandaState.alternativeExperience),
          "panda_safety_rx_checks_invalid": bool(pandaState.safetyRxChecksInvalid),
          "mismatch_counter": int(self.mismatch_counter),
        }
        if i < len(self.CP.safetyConfigs):
          details.update({
            "expected_safety_model": str(self.CP.safetyConfigs[i].safetyModel),
            "expected_safety_param": int(self.CP.safetyConfigs[i].safetyParam),
            "expected_alternative_experience": int(self.CP.alternativeExperience),
          })
        self._log_hardware_issue("controlsMismatch", details, dedupe_suffix=str(i), min_interval_sec=60.)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)
        self._log_hardware_issue("relayMalfunction", {
          "panda_index": i,
          "panda_type": str(pandaState.pandaType),
          "faults": [str(fault) for fault in pandaState.faults],
          "fault_status": str(pandaState.faultStatus),
        }, dedupe_suffix=str(i), min_interval_sec=60.)

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
    if self.sm.recv_frame['managerState'] and len(not_running):
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
        self._log_hardware_issue("processNotRunning", {
          "not_running": sorted(not_running),
        }, dedupe_suffix=",".join(sorted(not_running)), min_interval_sec=10.)
      self.not_running_prev = not_running
    if self.sm.recv_frame['managerState'] and not_running:
      self.events.add(EventName.processNotRunning)
    else:
      if not SIMULATION and not self.rk.lagging:
        camera_not_alive = [s for s in self.camera_packets if s not in self.sm.ignore_alive and not self.sm.alive[s]]
        camera_bad_freq = [s for s in self.camera_packets if s not in self.sm.ignore_average_freq and s not in self.sm.ignore_alive and not self.sm.freq_ok[s]]
        if camera_not_alive:
          pass#self.events.add(EventName.cameraMalfunction)
          self._log_hardware_issue("cameraMalfunction", {
            "camera_not_alive": camera_not_alive,
            "camera_health": {s: self._service_health(s) for s in camera_not_alive},
          }, dedupe_suffix=",".join(camera_not_alive), min_interval_sec=60.)
        elif camera_bad_freq:
          self.events.add(EventName.cameraFrameRate)
          self._log_hardware_issue("cameraFrameRate", {
            "camera_bad_freq": camera_bad_freq,
            "camera_health": {s: self._service_health(s) for s in camera_bad_freq},
          }, dedupe_suffix=",".join(camera_bad_freq), min_interval_sec=60.)
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.selfdrivedLagging)
      self._log_hardware_issue("selfdrivedLagging", {
        "ratekeeper_frame": self.sm.frame,
      }, min_interval_sec=60.)
    radar_errors = self.sm['radarState'].radarErrors.to_dict()
    if self.sm['radarState'].radarErrors.canError:
      self.events.add(EventName.canError)
      self._log_hardware_issue("radarCanError", {
        "radar_errors": radar_errors,
      }, min_interval_sec=60.)
    elif self.sm['radarState'].radarErrors.radarUnavailableTemporary:
      self.events.add(EventName.radarTempUnavailable)
      self._log_hardware_issue("radarTempUnavailable", {
        "radar_errors": radar_errors,
      }, min_interval_sec=60.)
    elif any(radar_errors.values()):
      self.events.add(EventName.radarFault)
      self._log_hardware_issue("radarFault", {
        "radar_errors": radar_errors,
      }, min_interval_sec=60.)
    if not self.sm.valid['pandaStates']:
      self.events.add(EventName.usbError)
      self._log_hardware_issue("usbError", {
        "panda_states_health": self._service_health("pandaStates"),
      }, min_interval_sec=60.)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
      self._log_hardware_issue("canBusMissing", {
        "can_timeout": bool(CS.canTimeout),
        "can_valid": bool(CS.canValid),
        "panda_states_health": self._service_health("pandaStates"),
      }, min_interval_sec=60.)
    elif not CS.canValid:
      self.events.add(EventName.canError)
      self._log_hardware_issue("canError", {
        "can_timeout": bool(CS.canTimeout),
        "can_valid": bool(CS.canValid),
        "panda_states_health": self._service_health("pandaStates"),
      }, min_interval_sec=60.)

    # generic catch-all. ideally, a more specific event should be added above instead
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    if not self.sm.all_checks() and no_system_errors:
      if not self.sm.all_alive():
        pass#self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        pass#self.events.add(EventName.commIssueAvgFreq)
      else:
        pass#self.events.add(EventName.commIssue)

      logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        comm_issue_key = ",".join(logs["invalid"] + logs["not_alive"] + logs["not_freq_ok"])[:120]
        self._log_hardware_issue("commIssue", logs, dedupe_suffix=comm_issue_key, min_interval_sec=30.)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    if not self.CP.notCar:
      if not self.sm['livePose'].posenetOK:
        self.events.add(EventName.posenetInvalid)
        self._log_hardware_issue("posenetInvalid", {
          "live_pose_health": self._service_health("livePose"),
          "posenet_ok": bool(self.sm['livePose'].posenetOK),
          "inputs_ok": bool(self.sm['livePose'].inputsOK),
        }, min_interval_sec=60.)
      if not self.sm['livePose'].inputsOK:
        self.events.add(EventName.locationdTemporaryError)
        self._log_hardware_issue("locationdTemporaryError", {
          "live_pose_health": self._service_health("livePose"),
          "posenet_ok": bool(self.sm['livePose'].posenetOK),
          "inputs_ok": bool(self.sm['livePose'].inputsOK),
          "sensor_health": {s: self._service_health(s) for s in self.sensor_packets},
        }, min_interval_sec=60.)
      if not self.sm['liveParameters'].valid and cal_status == log.LiveCalibrationData.Status.calibrated and not TESTING_CLOSET and (not SIMULATION or REPLAY):
        self.events.add(EventName.paramsdTemporaryError)
        self._log_hardware_issue("paramsdTemporaryError", {
          "live_parameters_health": self._service_health("liveParameters"),
          "calibration_status": str(cal_status),
        }, min_interval_sec=60.)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    missing_sensor_services = [s for s in self.sensor_packets if (self.sm.frame - self.sm.recv_frame[s]) * DT_CTRL > 10.]
    if missing_sensor_services:
      pass#self.events.add(EventName.sensorDataInvalid)
      if "sensord" in not_running:
        likely_reason = "sensord process is not running"
      elif set(missing_sensor_services) == set(self.sensor_packets):
        likely_reason = "IMU is not publishing; check LSM6DS3 I2C bus 1 address 0x6A, GPIO chip 0 line 84 IRQ, power, or sensor variant"
      else:
        likely_reason = "one IMU stream is missing; check shared LSM6DS3 configuration and data-ready status"
      self._log_hardware_issue("sensorDataInvalid", {
        "missing_sensor_services": missing_sensor_services,
        "sensor_health": {s: self._service_health(s) for s in self.sensor_packets},
        "not_running_processes": sorted(not_running),
        "likely_reason": likely_reason,
      }, dedupe_suffix=",".join(missing_sensor_services), min_interval_sec=60.)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Send a "steering required alert" if saturation count has reached the limit
    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0
    controlstate = self.sm['controlsState']
    lac = getattr(controlstate.lateralControlState, controlstate.lateralControlState.which())
    if lac.active and not recent_steer_pressed and not self.CP.notCar:
      clipped_speed = max(CS.vEgo, 0.3)
      actual_lateral_accel = controlstate.curvature * (clipped_speed**2)
      # Use controller-clipped desired curvature to reduce false saturation alerts.
      desired_lateral_accel = controlstate.desiredCurvature * (clipped_speed**2)
      undershoot_ratio = abs(desired_lateral_accel) / abs(1e-3 + actual_lateral_accel)
      undershooting = undershoot_ratio > 1.2
      turning = abs(desired_lateral_accel) > 1.0
      # TODO: lac.saturated includes speed and other checks, should be pulled out
      if undershooting and turning and lac.saturated:
        self.events.add(EventName.steerSaturated)

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    if (planner_fcw or model_fcw) and not self.CP.notCar:
      self.events.add(EventName.fcw)

    # GPS checks
    gps_ok = self.sm.recv_frame[self.gps_location_service] > 0 and (self.sm.frame - self.sm.recv_frame[self.gps_location_service]) * DT_CTRL < 2.0
    if not gps_ok and self.sm['livePose'].inputsOK and (self.distance_traveled > 1500):
      self.events.add(EventName.noGps)
      self._log_hardware_issue("noGps", {
        "gps_service": self.gps_location_service,
        "gps_health": self._service_health(self.gps_location_service),
        "distance_traveled_m": round(self.distance_traveled, 2),
      }, min_interval_sec=60.)
    if gps_ok:
      self.distance_traveled = 0
    self.distance_traveled += abs(CS.vEgo) * DT_CTRL

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      if self.sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)
        self._log_hardware_issue("modeldLagging", {
          "model_frame_drop_percent": float(self.sm['modelV2'].frameDropPerc),
          "model_health": self._service_health("modelV2"),
        }, min_interval_sec=60.)

    # Decrement personality on distance button press
    if self.CP.openpilotLongitudinalControl:
      if any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents):
        self.personality = (self.personality - 1) % 3
        self.params.put_nonblocking('LongitudinalPersonality', self.personality)
        self.events.add(EventName.personalityChanged)

  def data_sample(self):
    _car_state = messaging.recv_one(self.car_state_sock)
    CS = _car_state.carState if _car_state else self.CS_prev

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > 6.
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
          self.sm.ignore_alive.append('roadCameraState')
          self.sm.ignore_valid.append('roadCameraState')
        if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
          self.sm.ignore_alive.append('wideRoadCameraState')
          self.sm.ignore_valid.append('wideRoadCameraState')

        if REPLAY and any(ps.controlsAllowed for ps in self.sm['pandaStates']):
          self.state_machine.state = State.enabled

        self.initialized = True
        cloudlog.event(
          "selfdrived.initialized",
          dt=self.sm.frame*DT_CTRL,
          timeout=timed_out,
          canValid=CS.canValid,
          invalid=[s for s, valid in self.sm.valid.items() if not valid],
          not_alive=[s for s, alive in self.sm.alive.items() if not alive],
          not_freq_ok=[s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
          error=True,
        )

    # When the panda and selfdrived do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
           if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    return CS

  def update_alerts(self, CS):
    clear_event_types = set()
    if ET.WARNING not in self.state_machine.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled:
      clear_event_types.add(ET.NO_ENTRY)

    pers = LONGITUDINAL_PERSONALITY_MAP[self.personality]
    alerts = self.events.create_alerts(self.state_machine.current_alert_types, [self.CP, CS, self.sm, self.is_metric,
                                                                                self.state_machine.soft_disable_timer, pers])
    self.AM.add_many(self.sm.frame, alerts)
    self.AM.process_alerts(self.sm.frame, clear_event_types)

  def publish_selfdriveState(self, CS):
    # selfdriveState
    ss_msg = messaging.new_message('selfdriveState')
    ss_msg.valid = True
    ss = ss_msg.selfdriveState
    ss.enabled = self.enabled
    ss.active = self.active
    ss.state = self.state_machine.state
    ss.engageable = not self.events.contains(ET.NO_ENTRY)
    ss.experimentalMode = self.experimental_mode
    ss.personality = self.personality

    ss.alertText1 = self.AM.current_alert.alert_text_1
    ss.alertText2 = self.AM.current_alert.alert_text_2
    ss.alertSize = self.AM.current_alert.alert_size
    ss.alertStatus = self.AM.current_alert.alert_status
    ss.alertType = self.AM.current_alert.alert_type
    ss.alertSound = self.AM.current_alert.audible_alert
    ss.alertHudVisual = self.AM.current_alert.visual_alert

    self.pm.send('selfdriveState', ss_msg)

    # onroadEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('onroadEvents', len(self.events))
      ce_send.valid = True
      ce_send.onroadEvents = self.events.to_msg()
      self.pm.send('onroadEvents', ce_send)
    self.events_prev = self.events.names.copy()

  def step(self):
    CS = self.data_sample()
    self.update_events(CS)
    if not self.CP.passive and self.initialized:
      self.enabled, self.active = self.state_machine.update(self.events)
    self.update_alerts(CS)

    self.publish_selfdriveState(CS)

    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
      self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      self.personality = self.params.get("LongitudinalPersonality", return_default=True)
      time.sleep(0.1)

  def run(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.keep_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  s = SelfdriveD()
  s.run()

if __name__ == "__main__":
  main()
