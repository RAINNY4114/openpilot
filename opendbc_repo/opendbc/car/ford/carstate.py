from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, CarControllerParams, FordFlags
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.ford.mads import MadsCarState


ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["TransGearData"]["GearLvrPos_D_Actl"]

    self.distance_button = 0
    self.lc_button = 0

    # ========================================================================
    # [C2_APA] APA / SAPP 握手探针 (只读)
    # ========================================================================
    self.apa_sapp_state = 0
    self.apa_eps_assist_limited = False
    self.apa_veh_speed_kph = 0.0
    self._apa_probe_frame = 0
    self._apa_probe_prev = None
    self._apa_probe_fh = None

    # ========================================================================
    # [C2_APA][R2] 方向盘角度源
    #   True  : ActiveFrontStrg_Stat_FD1.SteWhlOffst_An_TotActl (真实角度)
    #   False : ParkAid_Data.ExtSteeringAngleReq2 (APA 请求值, 旧行为)
    # ========================================================================
    self.apa_use_true_steer_angle = False   # [R2] 临时关闭: 疑似导致 canError
    self._apa_true_steer_angle_deg = 0.0
    self._apa_true_steer_valid = False

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Occasionally on startup, the ABS module recalibrates the steering pinion offset,
    # so we need to block engagement.
    # The vehicle usually recovers out of this state within a minute of normal driving.
    self.vehicle_sensors_valid = cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"] < 32766
    # [C2_APA][R2] 交叉校验 —— 只在 0x89 已确认存在时才做。
    #
    # ⚠️ 绝不能用 cp.vl[...] 直接探测: 那会**懒加载注册**该报文,
    #    而未指定频率的报文默认按 1Hz 计 (timeout 10s), 一旦 0x89 不在
    #    总线上, MessageState.valid() 恒为 False -> canValid=False ->
    #    触发 "CAN Bus Error"! 必须先查 addresses / ts_nanos 判断存在性。
    _totactl = 0.0
    _totactl_ts = 0
    try:
      if 0x89 in cp.addresses:
        _totactl = float(cp.vl["ActiveFrontStrg_Stat_FD1"]["SteWhlOffst_An_TotActl"])
        _totactl_ts = cp.ts_nanos["ActiveFrontStrg_Stat_FD1"]["SteWhlOffst_An_TotActl"]
    except (KeyError, ValueError, TypeError, AttributeError):
      _totactl_ts = 0
    if _totactl_ts > 0 and abs(_totactl) >= 1600.0:
      self.vehicle_sensors_valid = False

    # ========================================================================
    # [C2_APA] SAPP 握手状态读取 (只读, 不参与任何控制)
    #   来源: C2 carstate.py
    #     self.sappHandshake = cp_cam.vl["EPAS_INFO"]["SAPPAngleControlStat1"]
    #     self.epsAssistLimited = cp_cam.vl["EPAS_INFO"]["SteMdule_D_Stat"] == 1
    # ========================================================================
    # ❗ 只能在主总线 cp 上读 EPAS_INFO。
    #   cp_cam 对应 Bus.cam (相机总线), 本车该总线上没有 0x82
    #   → 一旦在 cam parser 上注册, can_valid 恒为 False
    #   → interfaces.py: ret.canValid = all(pt, cam) = False → canError!
    try:
      self.apa_sapp_state = int(cp.vl["EPAS_INFO"]["SAPPAngleControlStat1"])
    except Exception:
      self.apa_sapp_state = 0

    try:
      self.apa_eps_assist_limited = int(cp.vl["EPAS_INFO"]["SteMdule_D_Stat"]) == 1
    except Exception:
      self.apa_eps_assist_limited = False

    try:
      self.apa_veh_speed_kph = float(cp.vl["EngVehicleSpThrottle2"]["Veh_V_ActlEng"])
    except Exception:
      self.apa_veh_speed_kph = 0.0

    # car speed
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]["Veh_V_ActlBrk"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # ---- [C2_APA] 探针日志: 状态变化时或每 100 帧写一次 ----
    #   修正: 必须在 vEgo 计算**之后**写日志, 否则 ret.vEgo 还是 structs 默认值 0.0
    self._apa_probe_frame += 1
    _apa_now = (self.apa_sapp_state, self.apa_eps_assist_limited)
    if (_apa_now != self._apa_probe_prev) or ((self._apa_probe_frame % 100) == 0):
      self._apa_probe_prev = _apa_now
      try:
        if self._apa_probe_fh is None:
          self._apa_probe_fh = open("/data/media/0/apa_probe.log", "a", buffering=1)
        self._apa_probe_fh.write(
          "sapp=%d limited=%d veh_kph=%.1f vego=%.2f vegoRaw=%.2f steer=%.1f\n"
          % (self.apa_sapp_state, 1 if self.apa_eps_assist_limited else 0,
             self.apa_veh_speed_kph, float(ret.vEgo), float(ret.vEgoRaw),
             float(ret.steeringAngleDeg))
        )
      except Exception:
        pass
    ret.yawRate = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"]
    ret.standstill = cp.vl["DesiredTorqBrk"]["VehStop_D_Stat"] == 1

    # gas pedal
    ret.gasPressed = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100. > 1e-6

    # brake pedal
    ret.brakePressed = cp.vl["EngBrakeData"]["BpedDrvAppl_D_Actl"] == 2
    ret.parkingBrake = cp.vl["DesiredTorqBrk"]["PrkBrkStatus"] in (1, 2)

    # steering wheel
    # [C2_APA][R2] 角度源切换 (APA 接管 0x3A8 后旧源会被自己污染)
    #
    # ⚠️ 盲点防护: CANParser 的 VLDict 是**懒加载**的, 首次访问会自动注册
    #    报文, 但此时信号值初值为 0.0 —— 且 0.0 落在合法范围内!
    #    若 0x89 根本不在总线上, 天真实现会把 "恒为 0" 误判为有效角度,
    #    结果比旧源更糟。因此额外用 ts_nanos 判定"真的收到过报文"。
    if self.apa_use_true_steer_angle:
      _r2_ok = False
      try:
        # ⚠️ 必须先确认 0x89 已注册且真的收到过, 否则**绝不能**访问
        #    cp.vl[...] —— 那会懒加载注册一条不存在的报文, 使其在
        #    can_valid 检查中永远超时, 直接触发 "CAN Bus Error"。
        if 0x89 in cp.addresses:
          _ts = cp.ts_nanos["ActiveFrontStrg_Stat_FD1"]["SteWhlOffst_An_TotActl"]
          if _ts > 0:
            _true_angle = float(
              cp.vl["ActiveFrontStrg_Stat_FD1"]["SteWhlOffst_An_TotActl"]
            )
            if -1601.0 <= _true_angle <= 1677.0:
              self._apa_true_steer_angle_deg = _true_angle
              self._apa_true_steer_valid = True
              ret.steeringAngleDeg = _true_angle
              _r2_ok = True
      except (KeyError, ValueError, TypeError, AttributeError):
        pass

      if not _r2_ok:
        if self._apa_true_steer_valid:
          # 降级 2: 用缓存, 不跳变
          ret.steeringAngleDeg = self._apa_true_steer_angle_deg
        else:
          # 降级 3: 冷启动回落旧源
          ret.steeringAngleDeg = cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"]
    else:
      ret.steeringAngleDeg = cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"]
    ret.steeringTorque = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
    ret.steeringPressed = self.update_steering_pressed(
      abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE,
      5,
    )
    ret.steerFaultTemporary = cp.vl["EPAS_INFO"]["EPAS_Failure"] == 1
    ret.steerFaultPermanent = cp.vl["EPAS_INFO"]["EPAS_Failure"] in (2, 3)
    ret.espDisabled = cp.vl["Cluster_Info1_FD1"]["DrvSlipCtlMde_D_Rq"] != 0

    if self.CP.flags & FordFlags.CANFD:
      ret.steerFaultTemporary |= (
        cp.vl["Lane_Assist_Data3_FD1"]["LatCtlSte_D_Stat"] not in (1, 2, 3)
      )

    # cruise state
    is_metric = (
      cp.vl["INSTRUMENT_PANEL"]["METRIC_UNITS"] == 1
      if not self.CP.flags & FordFlags.CANFD
      else False
    )

    ret.cruiseState.speed = cp.vl["EngBrakeData"]["Veh_V_DsplyCcSet"] * (
      CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
    )
    ret.cruiseState.enabled = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (4, 5)
    ret.cruiseState.available = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (3, 4, 5)
    ret.cruiseState.nonAdaptive = cp.vl["Cluster_Info1_FD1"]["AccEnbl_B_RqDrv"] == 0
    ret.cruiseState.standstill = cp.vl["EngBrakeData"]["AccStopMde_D_Rq"] == 3
    ret.accFaulted = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (1, 2)

    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = (
        ret.accFaulted
        or cp_cam.vl["ACCDATA"]["CmbbDeny_B_Actl"] == 1
      )

    # gear
    if self.CP.transmissionType == TransmissionType.automatic:
      if (cp.vl["TransGearData"]["GearLvrPos_D_Actl"] in (3, 4, 5)):
        ret.gearShifter = GearShifter.drive
      elif (cp.vl["TransGearData"]["GearLvrPos_D_Actl"] == 1):
        ret.gearShifter = GearShifter.reverse      

    elif self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = cp.vl["Engine_Clutch_Data"]["CluPdlPos_Pc_Meas"] > 0
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
    ret.genericToggle = bool(
      cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"]
    )

    prev_distance_button = self.distance_button
    prev_lc_button = self.lc_button

    self.distance_button = cp.vl["Steering_Data_FD1"]["AccButtnGapTogglePress"]
    self.lc_button = bool(
      cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"]
    )

    # lock info
    ret.doorOpen = any([
      cp.vl["BodyInfo_3_FD1"]["DrStatDrv_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatPsngr_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatRl_B_Actl"],
      cp.vl["BodyInfo_3_FD1"]["DrStatRr_B_Actl"],
    ])

    ret.seatbeltUnlatched = (
      cp.vl["RCMStatusMessage2_FD1"]["FirstRowBuckleDriver"] == 2
    )

    # blindspot sensors
    if self.CP.enableBsm:
      cp_bsm = cp_cam if self.CP.flags & FordFlags.CANFD else cp

      ret.leftBlindspot = (
        cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
      )
      ret.rightBlindspot = (
        cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0
      )

      # ------------------------------------------------------------------
      # [AO_SAFETY_TIGHTEN]
      #
      # Publish the full Ford cross-traffic / blind-spot payload so the
      # automatic-overtake helper can require a real rear gap instead of a
      # single boolean.
      #
      #   CtaAlrtLeft2_D_Stat : 0 Off, 1 Zone1(near) .. 4 Zone4(far)
      #   CtaSnsLeft_D_Stat   : 0 Clear, 1 Blocked, 2 Failure, 3 Invalid
      #   SodDetctLeft_D_Stat : 0 Clear, 1 Alert, 2 Flash, 3 Fault, 4 Blocked
      #
      # Packed as  left*100 + right   (0..499) into a single integer Param.
      # ------------------------------------------------------------------
      try:
        import os as _ao_os

        _l_zone = int(cp_bsm.vl["Side_Detect_L_Stat"]["CtaAlrtLeft2_D_Stat"])
        _r_zone = int(cp_bsm.vl["Side_Detect_R_Stat"]["CtaAlrtRight2_D_Stat"])

        _l_sns = int(cp_bsm.vl["Side_Detect_L_Stat"]["CtaSnsLeft_D_Stat"])
        _r_sns = int(cp_bsm.vl["Side_Detect_R_Stat"]["CtaSnsRight_D_Stat"])

        _l_fault = 1 if (_l_sns != 0 or _l_zone > 4) else 0
        _r_fault = 1 if (_r_sns != 0 or _r_zone > 4) else 0

        if _l_zone > 4:
          _l_zone = 4
        if _r_zone > 4:
          _r_zone = 4

        _zone_val = _l_zone * 100 + _r_zone
        _fault_val = _l_fault * 10 + _r_fault

        # Write only on change: this runs at CAN rate (100 Hz), so an
        # unconditional write would hammer the param files.
        _cache = getattr(self, "_ao_bsm_cache", None)
        if _cache != (_zone_val, _fault_val):
          self._ao_bsm_cache = (_zone_val, _fault_val)

          # This fork's libparams_c.so does not know custom keys, so publish
          # through the same file channel modeld already reads.
          for _k, _v in (
            ("AOBsmZone", "%d" % _zone_val),
            ("AOBsmFault", "%d" % _fault_val),
          ):
            for _d in ("/dev/shm/params", "/data/params/d"):
              try:
                with open(_ao_os.path.join(_d, _k), "w") as _f:
                  _f.write(_v)
                break
              except Exception:
                continue
      except Exception:
        pass

    self.buttons_stock_values = cp.vl["Steering_Data_FD1"]
    self.acc_tja_status_stock_values = cp_cam.vl["ACCDATA_3"]
    self.lkas_status_stock_values = cp_cam.vl["IPMA_Data"]

    MadsCarState.update_mads(self, ret, can_parsers)

    ret.buttonEvents = [
      *create_button_events(
        self.distance_button,
        prev_distance_button,
        {1: ButtonType.gapAdjustCruise},
      ),
      *create_button_events(
        self.lc_button,
        prev_lc_button,
        {1: ButtonType.lkas},
      ),
    ]

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.pt: CANParser(
        DBC[CP.carFingerprint][Bus.pt],
        [],
        CanBus(CP).main,
      ),
      Bus.cam: CANParser(
        DBC[CP.carFingerprint][Bus.pt],
        [],
        CanBus(CP).camera,
      ),
    }
