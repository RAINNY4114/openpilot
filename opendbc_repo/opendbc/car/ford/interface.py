import numpy as np
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.radar_interface import RadarInterface
from opendbc.car.ford.values import CarControllerParams, DBC, Ecu, FordFlags, RADAR, FordSafetyFlags
from opendbc.car.interfaces import CarInterfaceBase

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # PCM doesn't allow acceleration near cruise_speed,
    # so limit limits of pid to prevent windup
    # Additionally, cap positive accel by speed to reduce kickdown/high-RPM behavior on Ford/Lincoln.
    # This aligns the PID output limits with the planner's Ford cruise accel map.
    accel_max_speed_bp = [0., 10.0, 25., 40.]              # m/s
    # Keep in sync with A_CRUISE_MAX_VALS_FORD in longitudinal_planner.py
    accel_max_speed_vals = [1.0, 0.8, 0.60, 0.50]          # m/s^2
    speed_accel_cap = float(np.interp(current_speed, accel_max_speed_bp, accel_max_speed_vals))

    if cruise_speed < 1e-3:
      return CarControllerParams.ACCEL_MIN, speed_accel_cap

    accel_max_bp = [cruise_speed - 2., cruise_speed - .4]  # m/s
    accel_max_vals = [speed_accel_cap, 0.2]                # m/s^2
    accel_max = float(np.interp(current_speed, accel_max_bp, accel_max_vals))
    return CarControllerParams.ACCEL_MIN, min(speed_accel_cap, accel_max)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, dp_params, docs) -> structs.CarParams:
    ret.brand = "ford"

    ret.radarUnavailable = Bus.radar not in DBC[candidate]
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 5.0
    ret.steerAtStandstill = True

    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.5]

    # Faster stop-and-go starts: use the dedicated starting state to avoid sluggish PID ramp-up at standstill.
    # This is especially noticeable on Ford/Lincoln platforms where the ACC system can be slow to react.
    ret.startingState = True
    # Reduce low-speed surge in stop-and-go; smoother and safer following.
    ret.startAccel = 0.5
    ret.vEgoStarting = 0.25

    if not ret.radarUnavailable and DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR:
      # average of 33.3 Hz radar timestep / 4 scan modes = 60 ms
      # MRR_Header_Timestamps->CAN_DET_TIME_SINCE_MEAS reports 61.3 ms
      ret.radarDelay = 0.06

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(structs.CarParams.SafetyModel.ford)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = cfgs

    ret.alphaLongitudinalAvailable = ret.radarUnavailable
    if alpha_long or not ret.radarUnavailable:
      ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
      ret.openpilotLongitudinalControl = True

    if ret.flags & FordFlags.CANFD:
      ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.CANFD.value

      # TRON (SecOC) platforms are not supported
      # LateralMotionControl2, ACCDATA are 16 bytes on these platforms
      if len(fingerprint[CAN.camera]):
        if fingerprint[CAN.camera].get(0x3d6) != 8 or fingerprint[CAN.camera].get(0x186) != 8:
          carlog.error('dashcamOnly: SecOC is unsupported')
          ret.dashcamOnly = True
    else:
      # Lock out if the car does not have needed lateral and longitudinal control APIs.
      # Note that we also check CAN for adaptive cruise, but no known signal for LCA exists
      pscm_config = next((fw for fw in car_fw if fw.ecu == Ecu.eps and b'\x22\xDE\x01' in fw.request), None)
      if pscm_config:
        if len(pscm_config.fwVersion) != 24:
          carlog.error('dashcamOnly: Invalid EPS FW version')
          ret.dashcamOnly = True
        else:
          config_tja = pscm_config.fwVersion[7]  # Traffic Jam Assist
          config_lca = pscm_config.fwVersion[8]  # Lane Centering Assist
          if config_tja != 0xFF or config_lca != 0xFF:
            carlog.error('dashcamOnly: Car lacks required lateral control APIs')
            ret.dashcamOnly = True

    # Auto Transmission: 0x732 ECU or Gear_Shift_by_Wire_FD1
    found_ecus = [fw.ecu for fw in car_fw]
    if Ecu.shiftByWire in found_ecus or 0x5A in fingerprint[CAN.main] or docs:
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual
      ret.minEnableSpeed = 20.0 * CV.MPH_TO_MS

    # BSM: Side_Detect_L_Stat, Side_Detect_R_Stat
    # On some platforms these messages can be routed on different buses (e.g. CAN FD camera bus),
    # so check both main and camera fingerprints.
    # TODO: detect bsm in car_fw?
    ret.enableBsm = ((0x3A6 in fingerprint[CAN.main] and 0x3A7 in fingerprint[CAN.main]) or
                     (0x3A6 in fingerprint[CAN.camera] and 0x3A7 in fingerprint[CAN.camera]))

    # LCA can steer down to zero
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.centerToFront = ret.wheelbase * 0.44
    return ret
