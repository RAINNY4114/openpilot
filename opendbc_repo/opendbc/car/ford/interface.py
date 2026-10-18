import os

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

  # Gears in which openpilot may engage. `sport` is Ford's selector position 4
  # ("Sport_DriveSport"). It must be listed here because car_events.py raises
  # EventName.wrongGear for any gear that is neither `drive` nor in this tuple,
  # and wrongGear carries a NO_ENTRY event ("Gear not D") that blocks cruise.
  # Every other brand that can select a sport position declares it too
  # (Hyundai `(sport, manumatic)`, VW, GM, Toyota, Honda).
  DRIVABLE_GEARS = (
    structs.CarState.GearShifter.low,
    structs.CarState.GearShifter.manumatic,
    structs.CarState.GearShifter.sport,
  )

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    # PCM doesn't allow acceleration near cruise_speed,
    # so limit limits of pid to prevent windup
    ACCEL_MAX_VALS = [CarControllerParams.ACCEL_MAX, 0.2]
    ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .4]
    return CarControllerParams.ACCEL_MIN, np.interp(current_speed, ACCEL_MAX_BP, ACCEL_MAX_VALS)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "ford"

    ret.radarUnavailable = Bus.radar not in DBC[candidate]
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0
    ret.steerAtStandstill = True

    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.5]

    if not ret.radarUnavailable and DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR:
      # average of 33.3 Hz radar timestep / 4 scan modes = 60 ms
      # MRR_Header_Timestamps->CAN_DET_TIME_SINCE_MEAS reports 61.3 ms
      ret.radarDelay = 0.06

    CAN = CanBus(fingerprint=fingerprint)

    # ========================================================================
    # [C2_APA][路线A] 安全模式选择
    #
    # 路线 A 需要 panda 处于 allOutput, 才能发 0x3A8 ParkAid_Data。
    # 读 /data/ford_params/dp_ford_apa_panda_alloutput: 1 -> allOutput
    # 默认关闭 (文件不存在 -> 保持 ford), 保证行为与未打补丁时一致。
    # ========================================================================
    _apa_alloutput = False
    try:
      for _d in ("/data/ford_params", "/dev/shm/params", "/data/params/d"):
        try:
          with open(os.path.join(_d, "dp_ford_apa_panda_alloutput"), "r") as _f:
            _apa_alloutput = int(float(_f.read().strip() or "0")) != 0
          break
        except (OSError, ValueError):
          continue
    except Exception:
      _apa_alloutput = False

    if _apa_alloutput:
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]
    else:
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.ford)]
      if CAN.main >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = cfgs

    ret.alphaLongitudinalAvailable = ret.radarUnavailable
    if alpha_long or not ret.radarUnavailable:
      if not _apa_alloutput:
        ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
      ret.openpilotLongitudinalControl = True

    if ret.flags & FordFlags.CANFD:
      if not _apa_alloutput:
        ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.CANFD.value

      # ======================================================================
      # [C2_APA] SecOC dashcam 门控修复
      #   移植来源: RAINNY4114/opendbc-sunnypilot 分支 `apa`
      #             提交 b3a69f25 "apa: gate SecOC dashcam check on CANFD
      #             (fixes Transit MK5 false dashcam)"
      #
      #   上游写法把 SecOC 检查提升为显式 CANFD 门控:
      #       if ret.flags & FordFlags.CANFD and len(fingerprint[CAN.camera]):
      #   原因: 非 CAN FD 车的 camera 总线上没有 0x3d6 (LateralMotionControl2)
      #   与 0x186 (ACCDATA), `.get()` 返回 None, 而 `None != 8` 为真, 会把车
      #   误判成 TRON/SecOC 平台并置 dashcamOnly。
      #
      #   我方此处已经嵌在 `if ret.flags & FordFlags.CANFD:` 之内, 因此内联的
      #   CANFD 项恒为真 —— 保留它只为与上游逐字对齐, 便于后续合并; 语义等价,
      #   对 FORD_EXPLORER_MK6 (非 CAN FD) 行为零变化。
      # ======================================================================
      # TRON (SecOC) platforms are not supported — CAN FD only
      # LateralMotionControl2, ACCDATA are 16 bytes on these platforms
      if ret.flags & FordFlags.CANFD and len(fingerprint[CAN.camera]):
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
    # TODO: detect bsm in car_fw?
    # [ford 分支] 部分平台把 BSM 报文路由在 camera 总线上，两处都查。
    ret.enableBsm = ((0x3A6 in fingerprint[CAN.main] and 0x3A7 in fingerprint[CAN.main]) or
                     (0x3A6 in fingerprint[CAN.camera] and 0x3A7 in fingerprint[CAN.camera]))

    # LCA can steer down to zero
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.centerToFront = ret.wheelbase * 0.44
    return ret
