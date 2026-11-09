#!/usr/bin/env python3
import os
import time
import threading

import openpilot.cereal.messaging as messaging

from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient


from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.common.gps import get_gps_location_service

from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager, set_offroad_alert

from openpilot.common.version import get_build_metadata
from openpilot.common.hardware import HARDWARE

from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot import get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.selfdrived.button_state_tracker import ButtonStateTracker
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

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
AlertLevel = log.DriverMonitoringState.AlertLevel
MonitoringPolicy = log.DriverMonitoringState.MonitoringPolicy
TurnDirection = custom.ModelDataV2SP.TurnDirection

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

# =================================================================================================
# [LOC_TEMP_FIX_20261109] 检查并修正此前处理 "locationd Temporary Error" 的方法
#
# 此前的方法（chestnut 大模型适配时引入）：用 big_model_settling 在加载窗口内一次性抑制
# commIssue / 定位类 / 硬件类共 8 类告警。实测有三个缺陷：
#   1) 完全依赖 ChestnutLoading 参数。参数未写（offroad、未插 eGPU、modeld 未起）时
#      big_model_settling 恒 False，抑制整体失效。11-02 已发现，最新 rlog 仍复现。
#   2) ChestnutLoading 若卡在 True（加载失败/超时未清），big_model_settling 会永久为 True，
#      8 类告警被无限期屏蔽，真实硬件/定位故障无声。
#   3) 加载窗口外的 inputsOK 单帧毛刺完全没处理。locationd 里
#      inputs_valid = sm.all_valid() and critical_service_inputs_valid，任一服务单帧 invalid
#      即 inputsOK=False；selfdrived 立刻 add locationdTemporaryError(NO_ENTRY + SOFT_DISABLE)。
#      最新 rlog：一段 1200 帧里仅 1~2 帧 False，就产生 1~3 次告警并可导致瞬时软退出。
#
# 修正（保留原方法"加载期不误报"的效果，补齐上述缺陷）：
#   * big_model_settling 加硬上限（默认 120s，与 BIG_MODEL_TIMEOUT 对齐），warmup 分支要求
#     big_model_ready_t 确实被写过，杜绝永久屏蔽。
#   * locationdTemporaryError 改为 inputsOK 需连续无效 >= LocTempDebounceSec(默认 0.5s) 才上报，
#     单帧毛刺不再阻止 engage / 触发软退出。该逻辑不依赖任何参数，所有场景都生效。
#   * 安全兜底：连续无效 >= LocTempHardSec(默认 10s) 时，即使仍在加载窗口也照常上报，
#     避免真定位故障被窗口吞掉。
# 热配参数（/data/ford_params/ 纯文本，缺失即用默认值，1Hz 节流读取）：
#   LocTempDebounceSec       0.0~5.0    默认 0.5
#   LocTempHardSec           3.0~60.0   默认 10.0
#   BigModelSettlingMaxSec   30~300     默认 120.0
# =================================================================================================
_PARAM_DIR = "/data/ford_params"
LOC_TEMP_DEBOUNCE_SEC = 0.5
LOC_TEMP_HARD_SEC = 10.0
BIG_MODEL_SETTLING_MAX_SEC = 120.0

# =================================================================================================
# [MODEL_LAG_FIX_20261109] 检查并修正此前处理 "Driving Model Lagging / N% frames dropped" 的方法
#
# 历史方法（git 历史 368ce473a6，标记 [CHX_LAG_TOLERANCE 20261017]）：
#   chestnut 大模型走 eGPU(USB3) 时推理 71.5ms/帧（upload 40.4ms 是隧道瓶颈），
#   超 20Hz 的 50ms 预算，稳定掉帧约 34%；上游 1% 阈值会永久触发 modeldLagging
#   （NO_ENTRY + SOFT_DISABLE），ACC 无法接合。当时把 `> 1` 放宽到 `> 50`。
#   该方法在后续 staging 更新中被冲掉，现所有分支又变回 `> 1 and not big_model_settling`。
#
# 实测（2026-11-09 今早 route 0000009b，分辨率已降到 1344x760 后）：
#   modelV2.big 全程 True，modelExecutionTime 约 36.5ms（在 50ms 预算内），
#   大部分段 frameDropPerc=0；仅段 1 有 2.5% 峰值 / 均值 0.5%，
#   但 18.8% 的帧 >1%，硬触发了 31 次 modeldLagging。用户反馈提示约 12%。
#   frameDropPerc 是 10s 一阶滤波的 frames_dropped/(1+frames_dropped)：
#   12% 约等于 17.6Hz 的降级推理，每帧仍是完整推理，不是模型失效。
#
# 原方法 / 现状的问题：
#   1) 单一 1% 阈值对大模型过严，降级运行也被判为故障并挡住 engage。
#   2) 历史 50% 硬编码又过松：模型掉一半帧还继续开，且对小模型同样放宽，安全风险。
#   3) big_model_settling 只覆盖加载窗口；行驶中的偶发掉帧完全不设防（参数未写时恒 False）。
#   4) 单帧尖峰即报，没有持续时间判断。
#
# 修正：区分大小模型 + 去抖 + 硬兜底，全部阈值热配：
#   * 大模型(big=True) 用 ModelLagBigPerc(默认 25%，覆盖实测 12% 并留 2x 余量)；
#     小模型保持上游严格阈值 ModelLagSmallPerc(默认 1%)。
#   * 超阈值需持续 ModelLagDebounceSec(默认 2s) 才上报，滤掉单帧尖峰。
#   * 掉帧 >= ModelLagHardPerc(默认 60%) 视为模型近乎不工作，立即上报且不受加载窗口抑制。
# 热配（/data/ford_params/，1Hz 节流，缺失即用默认值）：
#   ModelLagBigPerc      1~80    默认 25.0
#   ModelLagSmallPerc    0~20    默认 1.0
#   ModelLagHardPerc     20~100  默认 60.0
#   ModelLagDebounceSec  0~10    默认 2.0
# =================================================================================================
MODEL_LAG_BIG_PERC = 25.0
MODEL_LAG_SMALL_PERC = 1.0
MODEL_LAG_HARD_PERC = 60.0
MODEL_LAG_DEBOUNCE_SEC = 2.0

# =================================================================================================
# [COMM_ISSUE_FIX_20261109] "Communication Issue Between Processes" 误报修复
#
# 实测（2026-11-09 route 0000009c / 0000009d，rlog 逐帧对齐）：
#   commIssue 出现时 managerState.processes 全部 shouldBeRunning=running，
#   memoryUsagePercent 60%、frameDropPerc < 5.2%，没有任何进程崩溃或资源异常。
#   cloudlog 明细里 not_alive/not_freq_ok 恒为 ['alertDebug','lateralManeuverPlan']
#   （这两个已在 ignore 列表，只是明细遍历原始 dict 时的噪音，并非触发者）；
#   真正的触发者是 invalid 列表中的 5 个学习参数服务：
#     extrinsicsCalibration / driverMonitoringState / vehicleParameters /
#     lateralDelay / lateralTorqueParameters
#
# 根因：calibrationd -> paramsd/torqued/lagd -> dmonitoringd 这条学习链路，
#   每个进程都用 `msg.valid = sm.all_checks()` 作为自己输出消息的 Event.valid。
#   任一上游输入单帧抖动，整条链会在同一时刻连锁发出 Event.valid=False。
#   SubMaster.update_msgs() 里 `self.valid[s] = msg.valid`，于是 selfdrived 的
#   all_checks() 把这些"内容尚未收敛"误判为"进程间通信中断"，
#   触发 commIssue（NO_ENTRY + SOFT_DISABLE），行车中直接打断 ACC。
#   逐帧证据：这 5 个服务 Event.valid=False 的帧仅占 0.8%~3.4%，
#   其时间戳与 commIssue 时间窗完全重合；无告警的段则 100% 为 True。
#   注意 lateralTorqueParameters 的**结构体内部** valid 全程 False，
#   那是 torqued 学习器自身状态，与 SubMaster 使用的 Event.valid 无关。
#
# 修正（只放宽"内容 valid"，绝不放宽通信活性）：
#   * 仅把这 5 个服务加入 ignore_valid；ignore_alive 与 ignore_avg_freq 保持原样。
#     进程崩溃 / 停止发布 / 频率异常仍会照常触发 commIssue。
#   * 启动期(从未收到过第一帧)同样豁免，避免点火后必然误报。
#   * 硬兜底：已 seen 的服务若连续 invalid >= CommIssueLearnerHardSec(默认 5s)，
#     临时撤掉豁免，真实的学习链路故障仍会报警，不会永久屏蔽。
#   * cloudlog 明细改为只列真正参与 all_checks() 判定的服务，避免误导排查。
# 热配（/data/ford_params/，1Hz 节流，缺失即用默认值）：
#   CommIssueIgnoreLearnerValid  0/1    默认 1（写 0 可一键回退到上游行为）
#   CommIssueLearnerHardSec     0~30   默认 5.0
# =================================================================================================
COMM_ISSUE_LEARNER_SERVICES = (
  'extrinsicsCalibration', 'driverMonitoringState', 'vehicleParameters',
  'lateralDelay', 'lateralTorqueParameters',
)
COMM_ISSUE_LEARNER_HARD_SEC = 5.0


def _ford_num(key: str, default: float, lo: float, hi: float) -> float:
  """Read a hot-reloadable numeric param from /data/ford_params, fall back to default."""
  try:
    with open(f"{_PARAM_DIR}/{key}") as f:
      v = float(f.read().strip())
    return min(hi, max(lo, v))
  except Exception:
    return default


class SelfdriveD:
  def __init__(self, CP=None, CP_SP=None):
    self.params = Params()

    # Ensure the current branch is cached, otherwise the first cycle lags
    build_metadata = get_build_metadata()

    if CP is None:
      cloudlog.info("selfdrived is waiting for CarParams")
      self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
      cloudlog.info("selfdrived got CarParams")
    else:
      self.CP = CP

    if CP_SP is None:
      cloudlog.info("selfdrived is waiting for CarParamsSP")
      self.CP_SP = messaging.log_from_bytes(self.params.get("CarParamsSP", block=True), custom.CarParamsSP)
      cloudlog.info("selfdrived got CarParamsSP")
    else:
      self.CP_SP = CP_SP

    self.car_events = CarEvents(self.CP)

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    self.excessive_actuation_check = ExcessiveActuationCheck()
    self.excessive_actuation = self.params.get("Offroad_ExcessiveActuation") is not None
    self.big_model_loading = False
    self.big_model_active = False
    self.big_model_failed = False
    self.big_model_ready_t = 0.
    # [LOC_TEMP_FIX_20261109] 加载窗口起点 + inputsOK 去抖状态
    self.big_model_loading_t = 0.
    self.loc_inputs_bad_t = 0.
    self.loc_reported = False
    self.loc_cfg_t = 0.
    self.loc_debounce_sec = LOC_TEMP_DEBOUNCE_SEC
    self.loc_hard_sec = LOC_TEMP_HARD_SEC
    self.settling_max_sec = BIG_MODEL_SETTLING_MAX_SEC
    # [MODEL_LAG_FIX_20261109] 掉帧率超阈值计时与热配值
    self.model_lag_over_t = 0.
    self.model_lag_reported = False
    self.model_lag_big_perc = MODEL_LAG_BIG_PERC
    self.model_lag_small_perc = MODEL_LAG_SMALL_PERC
    self.model_lag_hard_perc = MODEL_LAG_HARD_PERC
    self.model_lag_debounce_sec = MODEL_LAG_DEBOUNCE_SEC
    # [COMM_ISSUE_FIX_20261109] 学习参数 valid 豁免状态与热配值
    self.comm_learner_ignore = True
    self.comm_learner_hard_sec = COMM_ISSUE_LEARNER_HARD_SEC
    self.comm_learner_bad_t = 0.

    # Setup sockets
    self.pm = messaging.PubMaster(['selfdriveState', 'onroadEvents'] + ['selfdriveStateSP', 'onroadEventsSP'])

    self.gps_location_service = get_gps_location_service(self.params)
    self.gps_packets = [self.gps_location_service]
    self.sensor_packets = ["accelerometer", "gyroscope"]
    self.camera_packets = ["narrowRoadCameraState", "cabinCameraState", "wideRoadCameraState"]

    # TODO: de-couple selfdrived with card/conflate on carState without introducing controls mismatches
    self.car_state_sock = messaging.sub_sock('carState', timeout=20)

    ignore = self.sensor_packets + self.gps_packets + ['alertDebug', 'lateralManeuverPlan'] + ['modelDataV2SP', 'longitudinalPlanSP']
    if SIMULATION:
      ignore += ['cabinCameraState', 'managerState']
    if REPLAY:
      # no vipc in replay will make them ignored anyways
      ignore += ['narrowRoadCameraState', 'wideRoadCameraState']
    # [COMM_ISSUE_FIX_20261109] 学习参数服务只忽略"内容 valid"，alive / avg_freq 仍严格检查
    ignore_valid = ignore + list(COMM_ISSUE_LEARNER_SERVICES)
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'extrinsicsCalibration',
                                   'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'deviceMotion', 'lateralDelay',
                                   'managerState', 'vehicleParameters', 'radarState', 'lateralTorqueParameters',
                                   'controlsState', 'carControl', 'driverAssistance', 'alertDebug', 'userBookmark',
                                   'lateralManeuverPlan', 'modelDataV2SP', 'longitudinalPlanSP'] + \
                                   self.camera_packets + self.sensor_packets + self.gps_packets,
                                  ignore_alive=ignore, ignore_avg_freq=ignore,
                                  ignore_valid=ignore_valid, frequency=int(1/DT_CTRL))

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")

    car_recognized = self.CP.brand != 'mock'

    # cleanup old params
    if not self.CP.alphaLongitudinalAvailable:
      self.params.remove("AlphaLongitudinalEnabled")

    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.initialized = False
    self.enabled = False
    self.active = False
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.last_steering_pressed_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.logged_comm_issue = None
    self.not_running_prev = None
    self.personality = get_sanitize_int_param(
      "LongitudinalPersonality",
      min(log.LongitudinalPersonality.schema.enumerants.values()),
      max(log.LongitudinalPersonality.schema.enumerants.values()),
      self.params
    )
    self.recalibrating_seen = False
    self.dm_lockout_set = False
    self.dm_uncertain_alerted = False
    self.state_machine = StateMachine()
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    self.ignored_processes = {'mapd', }

    # Determine startup event
    is_remote = build_metadata.openpilot.comma_remote or build_metadata.openpilot.sunnypilot_remote
    self.startup_event = EventName.startup if is_remote and build_metadata.tested_channel else EventName.startupMaster
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

    self.events_sp = EventsSP()
    self.events_sp_prev = []

    self.mads = ModularAssistiveDrivingSystem(self)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP)

    self.car_events_sp = CarSpecificEventsSP(self.CP, self.CP_SP)

    self.button_state_tracker = ButtonStateTracker()

  def update_events(self, CS):
    """Compute onroadEvents from carState"""

    self.events.clear()
    self.events_sp.clear()

    if self.sm['controlsState'].lateralControlState.which() == 'debugState':
      self.events.add(EventName.joystickDebug)
      self.startup_event = None

    loading = self.params.get_bool("ChestnutLoading")
    if loading and not self.big_model_loading:
      self.big_model_loading_t = time.monotonic()  # [LOC_TEMP_FIX_20261109] 窗口起点, 用于硬上限
    if self.big_model_loading and not loading:
      self.big_model_ready_t = time.monotonic()
      self.events_sp.add(custom.OnroadEventSP.EventName.bigModelReady)
    self.big_model_loading = loading
    if self.big_model_loading:
      self.events.add(EventName.bigModelLoading)

    big_active = self.params.get("ChestnutActive")
    chestnut_present = self.sm['deviceState'].chestnutPresent
    model_unavailable = big_active is True and self.sm.seen['modelV2'] and not self.sm.alive['modelV2']
    big_failed = big_active is False or model_unavailable or (self.big_model_active and not chestnut_present)
    if big_failed and not self.big_model_failed:
      self.events.add(EventName.bigModelFailed)
      self.big_model_ready_t = time.monotonic()
    self.big_model_failed = big_failed

    # soft disable if the big model fails
    if big_active:
      self.big_model_active = True
    if not self.enabled and not model_unavailable:
      self.big_model_active = False

    if self.sm.recv_frame['lateralManeuverPlan'] > 0:
      self.events.add(EventName.lateralManeuver)
      self.startup_event = None
    elif self.sm.recv_frame['alertDebug'] > 0:
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

    # Check for user bookmark press
    if self.sm.updated['userBookmark']:
      prime_type = self.params.get("PrimeType")
      paired = prime_type is not None and int(prime_type) >= 0
      self.events.add(EventName.userBookmark if paired else EventName.userBookmarkNotPaired)

    # Don't add any more events while in dashcam mode
    if self.CP.passive:
      return

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and CS.vCruise > 250 and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    # Handle DM
    if not self.CP.notCar:
      # Block engaging until lockout times out or ignition reset
      if self.sm['driverMonitoringState'].lockout and not self.dm_lockout_set:
        self.params.put_bool("DriverTooDistracted", True)
        self.dm_lockout_set = True
      elif not self.sm['driverMonitoringState'].lockout and self.dm_lockout_set:
        self.params.remove("DriverTooDistracted")
        self.dm_lockout_set = False
      # No entry conditions
      if self.sm['driverMonitoringState'].lockout or self.sm['driverMonitoringState'].alwaysOnLockout:
        self.events.add(EventName.tooDistracted)
      # Alerts
      vision_dm = self.sm['driverMonitoringState'].activePolicy == MonitoringPolicy.vision
      if self.sm['driverMonitoringState'].alertLevel == AlertLevel.one:
        self.events.add(EventName.driverDistracted1 if vision_dm else EventName.driverUnresponsive1)
      elif self.sm['driverMonitoringState'].alertLevel == AlertLevel.two:
        self.events.add(EventName.driverDistracted2 if vision_dm else EventName.driverUnresponsive2)
      elif self.sm['driverMonitoringState'].alertLevel == AlertLevel.three:
        self.events.add(EventName.driverDistracted3 if vision_dm else EventName.driverUnresponsive3)
      # Warn consistent DM uncertainty
      if self.sm['driverMonitoringState'].visionPolicyState.uncertainOffroadAlertPercent >= 100 and not self.dm_uncertain_alerted:
        set_offroad_alert("Offroad_DriverMonitoringUncertain", True)
        self.dm_uncertain_alerted = True
      self.events_sp.add_from_msg(self.sm['longitudinalPlanSP'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      car_events = self.car_events.update(CS, self.CS_prev, self.sm['carControl']).to_msg()
      self.events.add_from_msg(car_events)

      car_events_sp = self.car_events_sp.update(CS, self.events).to_msg()
      self.events_sp.add_from_msg(car_events_sp)

      if self.CP.notCar:
        # wait for everything to init first
        if self.sm.frame > int(2. / DT_CTRL) and self.initialized:
          # body always wants to enable
          self.events.add(EventName.pcmEnable)

      # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
      if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
        (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
        self.events.add(EventName.pedalPressed)

    # Create events for temperature, disk space, and memory
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.overheated:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['extrinsicsCalibration'].calStatus
    if cal_status != log.ExtrinsicsCalibration.Status.calibrated:
      if cal_status == log.ExtrinsicsCalibration.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.ExtrinsicsCalibration.Status.recalibrating:
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
    if self.sm.updated['extrinsicsCalibration']:
      self.pose_calibrator.feed_extrinsics_calibration(self.sm['extrinsicsCalibration'])
    if self.sm.updated['deviceMotion']:
      device_motion = Pose.from_device_motion(self.sm['deviceMotion'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_motion)

    if self.calibrated_pose is not None and not self.CP.notCar:
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
      mdv2sp = self.sm['modelDataV2SP']

      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)

      elif (mdv2sp.leftLaneChangeEdgeBlock and direction == LaneChangeDirection.left) or \
           (mdv2sp.rightLaneChangeEdgeBlock and direction == LaneChangeDirection.right):
        self.events_sp.add(custom.OnroadEventSP.EventName.laneChangeRoadEdge)

      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['modelV2'].meta.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                    LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    # Handle lane turn
    lane_turn_direction = self.sm['modelDataV2SP'].laneTurnDirection
    if lane_turn_direction == TurnDirection.turnLeft:
      self.events_sp.add(custom.OnroadEventSP.EventName.laneTurnLeft)
    elif lane_turn_direction == TurnDirection.turnRight:
      self.events_sp.add(custom.OnroadEventSP.EventName.laneTurnRight)

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

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    if self.big_model_active and big_failed:
      self.events.add(EventName.bigModelFailed)

    not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
    if self.sm.recv_frame['managerState'] and len(not_running):
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
      self.not_running_prev = not_running
    if self.sm.recv_frame['managerState'] and (not_running - self.ignored_processes):
      self.events.add(EventName.processNotRunning)
    else:
      if not SIMULATION and not self.rk.lagging:
        if not self.sm.all_alive(self.camera_packets):
          self.events.add(EventName.cameraMalfunction)
        elif not self.sm.all_freq_ok(self.camera_packets):
          self.events.add(EventName.cameraFrameRate)
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.selfdrivedLagging)
    if self.CP.openpilotLongitudinalControl:
      if self.sm['radarState'].radarErrors.canError:
        self.events.add(EventName.canError)
      elif self.sm['radarState'].radarErrors.radarUnavailableTemporary:
        self.events.add(EventName.radarTempUnavailable)
      elif any(self.sm['radarState'].radarErrors.to_dict().values()):
        self.events.add(EventName.radarFault)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid:
      self.events.add(EventName.canError)

    # generic catch-all. ideally, a more specific event should be added above instead
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    warmup_sec = 5.
    now = time.monotonic()

    # [LOC_TEMP_FIX_20261109] 热配参数 1Hz 节流读取, 不增加 100Hz 主循环的文件 IO
    if now - self.loc_cfg_t >= 1.0:
      self.loc_cfg_t = now
      self.loc_debounce_sec = _ford_num("LocTempDebounceSec", LOC_TEMP_DEBOUNCE_SEC, 0.0, 5.0)
      self.loc_hard_sec = _ford_num("LocTempHardSec", LOC_TEMP_HARD_SEC, 3.0, 60.0)
      self.settling_max_sec = _ford_num("BigModelSettlingMaxSec", BIG_MODEL_SETTLING_MAX_SEC, 30.0, 300.0)
      self.model_lag_big_perc = _ford_num("ModelLagBigPerc", MODEL_LAG_BIG_PERC, 1.0, 80.0)
      self.model_lag_small_perc = _ford_num("ModelLagSmallPerc", MODEL_LAG_SMALL_PERC, 0.0, 20.0)
      self.model_lag_hard_perc = _ford_num("ModelLagHardPerc", MODEL_LAG_HARD_PERC, 20.0, 100.0)
      self.model_lag_debounce_sec = _ford_num("ModelLagDebounceSec", MODEL_LAG_DEBOUNCE_SEC, 0.0, 10.0)
      # [COMM_ISSUE_FIX_20261109]
      self.comm_learner_ignore = _ford_num("CommIssueIgnoreLearnerValid", 1.0, 0.0, 1.0) >= 0.5
      self.comm_learner_hard_sec = _ford_num("CommIssueLearnerHardSec", COMM_ISSUE_LEARNER_HARD_SEC, 0.0, 30.0)

    # 原方法在这里是 "self.big_model_loading or now < ready_t + 5s": 无上限, 参数卡 True 会永久屏蔽。
    # 现在两段都要求时间戳确实被写过, 且 loading 段有硬上限。
    loading_settling = (self.big_model_loading and self.big_model_loading_t > 0. and
                        (now - self.big_model_loading_t) < self.settling_max_sec)
    warmup_settling = self.big_model_ready_t > 0. and now < self.big_model_ready_t + warmup_sec
    big_model_settling = loading_settling or warmup_settling

    # inputsOK 连续无效计时。特意放在加载窗口之外累积: 窗口一结束就能立即反映真实状态,
    # 不会被 5s warmup 再拖一轮。seen=False 视为未收到消息(沿用上游语义, 不算定位故障)。
    dm_seen = self.sm.seen['deviceMotion']
    inputs_ok = (not dm_seen) or bool(self.sm['deviceMotion'].inputsOK)
    if not inputs_ok:
      if self.loc_inputs_bad_t == 0.:
        self.loc_inputs_bad_t = now
    else:
      self.loc_inputs_bad_t = 0.
      self.loc_reported = False
    loc_bad_dur = (now - self.loc_inputs_bad_t) if self.loc_inputs_bad_t > 0. else 0.
    loc_inputs_bad = loc_bad_dur >= self.loc_debounce_sec
    loc_inputs_hard = loc_bad_dur >= max(self.loc_hard_sec, self.loc_debounce_sec)

    # [COMM_ISSUE_FIX_20261109] 学习链路的 Event.valid 瞬时抖动不是通信故障。
    # 只豁免"已收到过消息但内容未收敛"的短暂窗口；持续 invalid 超硬阈值则撤掉豁免，
    # 真实的学习链路故障仍能触发 commIssue。alive / freq_ok 检查始终不豁免。
    learner_bad = any(self.sm.seen.get(s, False) and not self.sm.valid.get(s, True)
                      for s in COMM_ISSUE_LEARNER_SERVICES)
    if learner_bad:
      if self.comm_learner_bad_t == 0.:
        self.comm_learner_bad_t = now
    else:
      self.comm_learner_bad_t = 0.
    learner_bad_dur = (now - self.comm_learner_bad_t) if self.comm_learner_bad_t > 0. else 0.
    learner_hard = learner_bad_dur >= max(self.comm_learner_hard_sec, 0.0)
    learner_ignore = self.comm_learner_ignore and not learner_hard
    for _svc in COMM_ISSUE_LEARNER_SERVICES:
      if learner_ignore:
        if _svc not in self.sm.ignore_valid:
          self.sm.ignore_valid.append(_svc)
      elif _svc in self.sm.ignore_valid:
        self.sm.ignore_valid.remove(_svc)

    if not self.sm.all_checks() and no_system_errors and not big_model_settling:  # the load holds modelV2 and friends back on purpose
      if not self.sm.all_alive():
        self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else:
        self.events.add(EventName.commIssue)

      # 只列真正参与 all_checks() 判定的服务，避免把已在 ignore 中的项当故障记录
      logs = {
        'invalid': [s for s, valid in self.sm.valid.items()
                    if not valid and s not in self.sm.ignore_valid],
        'not_alive': [s for s, alive in self.sm.alive.items()
                      if not alive and s not in self.sm.ignore_alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items()
                        if not freq_ok and s not in self.sm.ignore_alive
                        and s not in self.sm.ignore_average_freq],
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    # [LOC_TEMP_FIX_20261109] 加载窗口仍抑制预期内的定位抖动, 但连续无效超过硬阈值时
    # 必须放行 —— 真定位故障不能被大模型加载窗口吞掉。
    if not self.CP.notCar and (not big_model_settling or loc_inputs_hard):
      # the defaults of a message that was never received are not a localizer failure
      if dm_seen and not self.sm['deviceMotion'].posenetOK:
        self.events.add(EventName.posenetInvalid)
      if dm_seen and loc_inputs_bad:
        self.events.add(EventName.locationdTemporaryError)
        if not self.loc_reported:
          cloudlog.warning(f"locationdTemporaryError: inputsOK invalid for {loc_bad_dur:.2f}s "
                           f"(debounce={self.loc_debounce_sec:.2f}s, settling={big_model_settling})")
          self.loc_reported = True
      if (self.sm.seen['vehicleParameters'] and not self.sm['vehicleParameters'].valid and cal_status == log.ExtrinsicsCalibration.Status.calibrated and
          not TESTING_CLOSET and (not SIMULATION or REPLAY)):
        self.events.add(EventName.paramsdTemporaryError)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    if any((self.sm.frame - self.sm.recv_frame[s])*DT_CTRL > 10. for s in self.sensor_packets):
      self.events.add(EventName.sensorDataInvalid)

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
      desired_lateral_accel = self.sm['modelV2'].action.desiredCurvature * (clipped_speed**2)
      undershooting = abs(desired_lateral_accel) / abs(1e-3 + actual_lateral_accel) > 1.2
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
    if not gps_ok and self.sm['deviceMotion'].inputsOK and (self.distance_traveled > 1500):
      self.events.add(EventName.noGps)
    if gps_ok:
      self.distance_traveled = 0
    self.distance_traveled += abs(CS.vEgo) * DT_CTRL

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      # [MODEL_LAG_FIX_20261109] 大模型降级推理不等于模型失效, 详见文件头注释
      mv = self.sm['modelV2']
      drop = float(mv.frameDropPerc)
      is_big = bool(mv.big)
      limit = self.model_lag_big_perc if is_big else self.model_lag_small_perc
      over = drop > limit
      hard = drop >= max(self.model_lag_hard_perc, limit)
      if over:
        if self.model_lag_over_t == 0.:
          self.model_lag_over_t = now
      else:
        self.model_lag_over_t = 0.
        self.model_lag_reported = False
      lag_dur = (now - self.model_lag_over_t) if self.model_lag_over_t > 0. else 0.
      sustained = lag_dur >= self.model_lag_debounce_sec

      # 硬超限立即上报且不受加载窗口抑制; 其余需持续超阈值, 滤掉单帧尖峰
      if hard or (over and sustained and not big_model_settling):
        self.events.add(EventName.modeldLagging)
        if not self.model_lag_reported:
          cloudlog.warning(f"modeldLagging: frameDropPerc={drop:.1f}% big={is_big} limit={limit:.1f}% "
                           f"sustained={lag_dur:.1f}s hard={hard} settling={big_model_settling}")
          self.model_lag_reported = True

    # mute canBusMissing event if in Park, as it sometimes may trigger a false alarm with MADS in Paused state
    if CS.gearShifter == car.CarState.GearShifter.park and self.mads.enabled:
      self.events.remove(EventName.canBusMissing)

    # Decrement personality on distance button press
    if self.CP.openpilotLongitudinalControl:
      if any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents):
        self.personality = (self.personality - 1) % 3
        self.params.put('LongitudinalPersonality', self.personality)
        self.events.add(EventName.personalityChanged)

    self.icbm.run(CS, self.sm['carControl'], self.sm['longitudinalPlanSP'], self.is_metric)

  def data_sample(self):
    _car_state = messaging.recv_one(self.car_state_sock)
    CS = _car_state.carState if _car_state else self.CS_prev

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > 6.
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_NARROW_ROAD not in available_streams:
          self.sm.ignore_alive.append('narrowRoadCameraState')
          self.sm.ignore_valid.append('narrowRoadCameraState')
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
    if self.enabled or (ET.NO_ENTRY not in (c := self.state_machine.current_alert_types) and (ET.ENABLE in c or ET.USER_DISABLE in c)):
      clear_event_types.add(ET.NO_ENTRY)

    pers = LONGITUDINAL_PERSONALITY_MAP[self.personality]
    callback_args = [self.CP, CS, self.sm, self.is_metric,
                     self.state_machine.soft_disable_timer, pers]

    alerts = self.events.create_alerts(self.state_machine.current_alert_types, callback_args)
    alerts_sp = self.events_sp.create_alerts(self.state_machine.current_alert_types, callback_args)

    self.AM.add_many(self.sm.frame, alerts + alerts_sp)
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

    # selfdriveStateSP
    ss_sp_msg = messaging.new_message('selfdriveStateSP')
    ss_sp_msg.valid = True
    ss_sp = ss_sp_msg.selfdriveStateSP
    mads = ss_sp.mads
    mads.state = self.mads.state_machine.state
    mads.enabled = self.mads.enabled
    mads.active = self.mads.active
    mads.available = self.mads.enabled_toggle

    icbm = ss_sp.intelligentCruiseButtonManagement
    icbm.state = self.icbm.state
    icbm.sendButton = self.icbm.cruise_button
    icbm.vTarget = self.icbm.v_target

    self.button_state_tracker.publish(ss_sp)

    self.pm.send('selfdriveStateSP', ss_sp_msg)

    # onroadEventsSP - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events_sp.names != self.events_sp_prev):
      ce_send_sp = messaging.new_message('onroadEventsSP')
      ce_send_sp.valid = True
      ce_send_sp.onroadEventsSP.events = self.events_sp.to_msg()
      self.pm.send('onroadEventsSP', ce_send_sp)
    self.events_sp_prev = self.events_sp.names.copy()

  def step(self):
    CS = self.data_sample()
    self.update_events(CS)
    if not self.CP.passive and self.initialized:
      self.enabled, self.active = self.state_machine.update(self.events)
    if not self.CP.notCar:
      self.mads.update(CS)
    self.update_alerts(CS)

    self.button_state_tracker.update(CS)
    self.publish_selfdriveState(CS)

    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
      self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
      self.personality = self.params.get("LongitudinalPersonality", return_default=True)

      self.mads.read_params()
      time.sleep(0.1)

  def run(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  s = SelfdriveD()
  s.run()

if __name__ == "__main__":
  main()
