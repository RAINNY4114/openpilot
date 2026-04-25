import cereal.messaging as messaging
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, CarControllerParams, FordFlags
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.steering_msg = "SteeringPinion_Data_Alt" if CP.flags & FordFlags.ALT_STEER_ANGLE else "SteeringPinion_Data"
    self.use_alt_gear = bool(CP.flags & FordFlags.ALT_GEAR)
    if CP.transmissionType == TransmissionType.automatic:
      if self.use_alt_gear:
        self.shifter_values = can_define.dv["TransGearData"]["GearLvrPos_D_Actl"]
      else:
        self.shifter_values = can_define.dv["PowertrainData_10"]["TrnRng_D_Rq"]

    self.distance_button = 0
    self.lc_button = 0
    self.steering_angle_offset_deg = 0.0
    self.car_state_bp_msg = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    # Occasionally on startup, the ABS module recalibrates the steering pinion offset, so we need to block engagement
    # The vehicle usually recovers out of this state within a minute of normal driving
    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      vehicle_sensors_valid = (
        int((cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"] + 1000) * 10) not in (32766, 32767)
        and cp.vl["ParkAid_Data"]["EPASExtAngleStatReq"] == 0
        and cp.vl["ParkAid_Data"]["ApaSys_D_Stat"] in (0, 1)
      )
    else:
      vehicle_sensors_valid = cp.vl["SteeringPinion_Data"]["StePinCompAnEst_D_Qf"] == 3
    ret.vehicleSensorsInvalid = not vehicle_sensors_valid

    # car speed
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]["Veh_V_ActlBrk"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"]
    ret.standstill = cp.vl["DesiredTorqBrk"]["VehStop_D_Stat"] == 1

    # gas pedal
    ret.gasPressed = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100. > 1e-6

    # brake pedal
    ret.brake = cp.vl["BrakeSnData_4"]["BrkTot_Tq_Actl"] / 32756.  # torque in Nm
    ret.brakePressed = cp.vl["EngBrakeData"]["BpedDrvAppl_D_Actl"] == 2
    ret.parkingBrake = cp.vl["DesiredTorqBrk"]["PrkBrkStatus"] in (1, 2)

    # steering wheel
    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      steering_angle_init = cp.vl[self.steering_msg]["StePinRelInit_An_Sns"]
      if vehicle_sensors_valid:
        steering_angle_est = cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"]
        self.steering_angle_offset_deg = steering_angle_est - steering_angle_init
      ret.steeringAngleDeg = steering_angle_init + self.steering_angle_offset_deg
    else:
      ret.steeringAngleDeg = cp.vl[self.steering_msg]["StePinComp_An_Est"]
    ret.steeringTorque = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steerFaultTemporary = cp.vl["EPAS_INFO"]["EPAS_Failure"] == 1
    ret.steerFaultPermanent = cp.vl["EPAS_INFO"]["EPAS_Failure"] in (2, 3)
    ret.espDisabled = cp.vl["Cluster_Info1_FD1"]["DrvSlipCtlMde_D_Rq"] != 0  # 0 is default mode

    if self.CP.flags & FordFlags.CANFD:
      # this signal is always 0 on non-CAN FD cars
      ret.steerFaultTemporary |= cp.vl["Lane_Assist_Data3_FD1"]["LatCtlSte_D_Stat"] not in (1, 2, 3)

    # cruise state
    is_metric = cp.vl["INSTRUMENT_PANEL"]["METRIC_UNITS"] == 1 if not self.CP.flags & FordFlags.CANFD else False
    ret.cruiseState.speed = cp.vl["EngBrakeData"]["Veh_V_DsplyCcSet"] * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    ret.cruiseState.enabled = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (4, 5)
    ret.cruiseState.available = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (3, 4, 5)
    ret.cruiseState.nonAdaptive = cp.vl["Cluster_Info1_FD1"]["AccEnbl_B_RqDrv"] == 0
    ret.cruiseState.standstill = cp.vl["EngBrakeData"]["AccStopMde_D_Rq"] == 3
    ret.accFaulted = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (1, 2)
    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = ret.accFaulted or cp_cam.vl["ACCDATA"]["CmbbDeny_B_Actl"] == 1

   # Gear
        if self.CP.transmissionType == TransmissionType.automatic:
            if self.use_alt_gear:
                raw_gear = cp.vl["TransGearData"]["GearLvrPos_D_Actl"]
            else:
                raw_gear = cp.vl["PowertrainData_10"]["TrnRng_D_Rq"]
            gear = self.shifter_values.get(raw_gear)
            if raw_gear >= 3:
                ret.gearShifter = GearShifter.drive
            elif gear is not None:
                ret.gearShifter = self.parse_gear_shifter(gear)
            else:
                ret.gearShifter = GearShifter.unknown
        elif self.CP.transmissionType == TransmissionType.manual:
            if bool(cp.vl["BCM_Lamp_Stat_FD1"]["RvrseLghtOn_B_Stat"]):
                ret.gearShifter = GearShifter.reverse
            else:
                ret.gearShifter = GearShifter.drive

    # safety
    ret.stockFcw = bool(cp_cam.vl["ACCDATA_3"]["FcwVisblWarn_B_Rq"])
    ret.stockAeb = bool(cp_cam.vl["ACCDATA_2"]["CmbbBrkDecel_B_Rq"])

    # button presses
    ret.leftBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 1
    ret.rightBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 2
    # TODO: block this going to the camera otherwise it will enable stock TJA
    ret.genericToggle = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])
    prev_distance_button = self.distance_button
    prev_lc_button = self.lc_button
    self.distance_button = cp.vl["Steering_Data_FD1"]["AccButtnGapTogglePress"]
    self.lc_button = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])

    # lock info
    ret.doorOpen = any([cp.vl["BodyInfo_3_FD1"]["DrStatDrv_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatPsngr_B_Actl"],
                        cp.vl["BodyInfo_3_FD1"]["DrStatRl_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatRr_B_Actl"]])
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]["FirstRowBuckleDriver"] == 2

    # blindspot sensors
    if self.CP.enableBsm:
      cp_bsm = cp_cam if self.CP.flags & FordFlags.CANFD else cp
      ret.leftBlindspot = cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
      ret.rightBlindspot = cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0

    # Stock steering buttons so that we can passthru blinkers etc.
    self.buttons_stock_values = cp.vl["Steering_Data_FD1"]
    # Stock values from IPMA so that we can retain some stock functionality
    self.acc_tja_status_stock_values = cp_cam.vl["ACCDATA_3"]
    self.lkas_status_stock_values = cp_cam.vl["IPMA_Data"]

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.lc_button, prev_lc_button, {1: ButtonType.lkas}),
    ]

    self.car_state_bp_msg = self.update_car_state_bp(cp, cp_cam)

    return ret

  def update_car_state_bp(self, cp, cp_cam):
    dat = messaging.new_message("carStateBP")
    dat.valid = True

    hybrid_drive = dat.carStateBP.hybridDrive
    hybrid_battery = dat.carStateBP.hybridBattery
    brake_light_status = dat.carStateBP.brakeLightStatus

    hybrid_drive.dataAvailable = False
    hybrid_drive.throttleDemandPercent = 0.0
    hybrid_drive.throttleThresholdPercent = 0.0
    hybrid_drive.powerFlowMode = ""
    hybrid_drive.engineOnReason = ""
    hybrid_drive.powerFlowModeValue = 0
    hybrid_drive.engineOnReasonValue = 0

    hybrid_battery.dataAvailable = False
    hybrid_battery.voltHighLimit = 0.0
    hybrid_battery.voltLowLimit = 0.0
    hybrid_battery.voltActual = 0.0
    hybrid_battery.ampsActual = 0.0
    hybrid_battery.socMinPerc = 0.0
    hybrid_battery.socMaxPerc = 0.0
    hybrid_battery.socActual = 0.0

    brake_light_status.dataAvailable = False
    brake_light_status.brakeLightsOn = False

    brake_lights_detected = False

    try:
      bcm_data = cp.vl["BCM_Lamp_Stat_FD1"]
      if bcm_data is not None:
        brake_light_status.dataAvailable = True
        if "StopLghtOn_B_Stat" in bcm_data:
          brake_light_status.brakeLightsOn = bool(bcm_data["StopLghtOn_B_Stat"])
          brake_lights_detected = True
        elif "RvrseLghtOn_B_Stat" in bcm_data:
          brake_light_status.brakeLightsOn = bcm_data["RvrseLghtOn_B_Stat"] == 1
          brake_lights_detected = True
        else:
          brake_light_status.dataAvailable = False
    except (KeyError, AttributeError):
      pass

    if not brake_lights_detected:
      try:
        brake_data = cp.vl["BrakeSysFeatures_2"]
        if brake_data is not None:
          brake_light_status.dataAvailable = True
          brake_light_status.brakeLightsOn = brake_data["BrkLamp_B_Rq"] == 1
          brake_lights_detected = True
      except (KeyError, AttributeError):
        pass

    if brake_lights_detected and self.CP.openpilotLongitudinalControl:
      try:
        acc_data = cp_cam.vl["ACCDATA"]
        acc_brake_active = (acc_data["AccBrkPrchg_B_Rq"] == 1 or
                            acc_data["AccBrkDecel_B_Rq"] == 1)
        brake_light_status.brakeLightsOn = (brake_light_status.brakeLightsOn or acc_brake_active)
      except (KeyError, AttributeError):
        pass

    return dat

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).camera),
    }
