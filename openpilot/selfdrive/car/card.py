#!/usr/bin/env python3
import os
import time
import threading

import openpilot.cereal.messaging as messaging

from openpilot.cereal import log, custom
from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.helpers import convert_carControlSP, convert_to_capnp

from openpilot.sunnypilot.mads.helpers import set_alternative_experience, set_car_specific_params
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# ============================================================================
# MR76 front radar -- panda CAN bus, and the on-device stability log
#
# Ported from carrotpilot's card.py.  MR76_BUS is defined HERE and card.py owns
# its own reader; nothing in the radar module is touched.
#
#   bus 0 = powertrain / camera
#   bus 1 = OEM front radar bus   <-- the MR76 is tapped here
#   bus 2 = ADAS / steering
#
# Verified 2026-09-29 against all 11 recorded segments
# (/data/media/0/realdata/0000001a--4c1c31bdde--0..10): 0x60A / 0x60B / 0x201
# appear on bus 1 only, never on bus 2.  git HEAD also carried MR76_BUS = 1,
# and the MR76 block's docstring says "physical connection: C3X CAN1".
# Re-verify any time with /data/mr76/mr76_bus.py (needs the car awake).
#
# MR76Analyzer.feed() filters the raw CAN stream with
#     if src != MR76_BUS: continue
# which is an independent check that frames really do arrive on bus 1.
# ============================================================================
MR76_BUS = 1                # MR76在panda bus1 (C3X CAN1)

# 日志开关: 1=开启日志输出  0=关闭(零开销)。
# 运行时热切换: 参数 dp_mr76_log_enable (无需重启, 1Hz 刷新)。
# 日志: 目标明细(50Hz全量) + 每秒统计行(#STAT) + 丢报警告行(#WARN)
#       写入 /data/media/0/mr76_analysis.log
MR76_LOG_ENABLE = 0
MR76_LOG_ENABLE_PARAM = "dp_mr76_log_enable"
MR76_LOG_PATH = "/data/media/0/mr76_analysis.log"
MR76_MSG_STATUS = 0x60A     # 目标状态报文(NoOfObjects/MeasCount)
MR76_MSG_OBJECT = 0x60B     # 单目标报文(50Hz轮询全部track)
MR76_REPORT_INTERVAL = 1.0  # 统计行输出间隔(秒)
MR76_TRACK_TIMEOUT = 0.5    # track失联判定时长(秒)
MR76_LOG_MIN_VEGO = 0.5     # 行驶判定阈值(m/s): 低于此值不记录(静止不收集)
MR76_WARN_RATE_MIN = 25.0   # 帧率下限(正常50Hz, 低于50%判定丢报)
MR76_WARN_GAP_MAX = 0.5     # 帧间最大间隔(秒, 超过=断流)
MR76_WARN_COOLDOWN = 5.0    # 警告冷却(秒, 防刷屏)


def _moto_bits(start: int, length: int):
  """DBC Motorola锯齿位序 -> [(byte, bit_lsb_pos), ...]"""
  bits, b, p = [], start // 8, start % 8
  while len(bits) < length:
    bits.append((b, p))
    if p == 0:
      b, p = b + 1, 7
    else:
      p -= 1
  return bits


def _sig(dat: bytes, start: int, length: int) -> int:
  """按Motorola位序从报文提取原始信号值"""
  v = 0
  for (b, p) in _moto_bits(start, length):
    v = (v << 1) | ((dat[b] >> p) & 1)
  return v


class MR76Analyzer:
  """解析bus1 MR76目标报文(0x60A/0x60B), 输出目标明细+稳定性统计。

  位序/信号定义经carrotpilot 2026-08-16实测日志逆向验证:
    ID:7|8  DistLong:15|13(0.2m,-500)  DistLat:18|11(0.2m,-204.6,右正)
    VRelLong:39|10(0.25,-128)  VRelLat:45|9(0.25,-64)  DynProp:50|3
    Class:52|2  RCS:63|8(0.5dB,-64)

  目标明细列: t,vEgo,tid,age,dRel,yRel(左正),vRel,yvRel,dynProp,cls,rcs
    age=该ID连续存活秒数(闪烁track的age恒小, 稳定track持续增长→直接统计闪烁频率)
  统计行指标: noObj(雷达自报目标数)/alive(存活track)/rate(0x60B帧率Hz)/
              jump(单帧跳变虚警)/new+lost(ID增减=跳变)/gapMax(最大帧间隔s)/meas(测量计数)
  """

  def __init__(self):
    self.enabled = bool(MR76_LOG_ENABLE)
    self.f = None
    self.tracks = {}          # tid -> {"first_t","last_t","dRel","yRel","vRel","rcs"}
    self.no_obj = -1
    self.meas_count = -1
    self.n_obj_msg = 0        # 本统计周期0x60B帧数
    self.n_jump = 0           # 单帧跳变(虚警)计数
    self.win_new = self.win_lost = 0
    self.last_report = time.time()
    self.last_obj_msg_t = time.time()   # 最近一条0x60B时间(断流检测)
    self.gap_max = 0.0        # 本统计周期最大帧间隔
    self.warn_cooldown_t = 0.0
    self.prev_t = None        # 上一帧时间(设备时钟跳变检测)
    self.warn_suppress_until = 0.0  # 时钟跳变后的WARN抑制截止时刻
    if self.enabled:
      self._open_log()

  def _open_log(self) -> None:
    try:
      # [FIX-IO] 64KB块缓冲(非行缓冲): 行缓冲时每行一次write+flush →
      # 行驶中约490次/s 的写盘系统调用打在CAN关键线程上, 实测card主循环被
      # 周期性阻塞到40~60ms(等效仅36.6Hz)。块缓冲配合1Hz显式flush,
      # 写盘次数降到约1次/秒。
      self.f = open(MR76_LOG_PATH, "a", buffering=1 << 16)
      self.f.write(f"# MR76Analyzer start {time.strftime('%F %T')} "
                   f"cols: t,vEgo,tid,age,dRel,yRel,vRel,yvRel,dynProp,cls,rcs\n")
    except OSError as e:
      print(f"MR76Analyzer: 日志不可用, 已停用 ({e})")
      self.f = None

  def set_enabled(self, on: bool) -> None:
    """运行时热切换(dp_mr76_log_enable)。重新对齐时间基准, 防启用瞬间误报。"""
    on = bool(on)
    if on == self.enabled:
      return
    self.enabled = on
    if on:
      if self.f is None:
        self._open_log()
      t = time.time()
      for tr in self.tracks.values():
        tr["first_t"], tr["last_t"] = t, t
      self.last_obj_msg_t, self.last_report = t, t
      self.n_obj_msg, self.gap_max = 0, 0.0
      self.warn_suppress_until = t + 15.0
    elif self.f is not None:
      try:
        self.f.flush()
      except Exception:
        pass

  def feed(self, can_list, v_ego: float) -> None:
    """card主循环调用: 按总线/地址过滤解析MR76。关闭开关时零开销。"""
    if not self.enabled or self.f is None:
      return
    try:
      t = time.time()
      # 设备时钟跳变防护(NTP step校准: 时间回拨或大前跳): 重置全部时间基准,
      # 否则age/gap/rate全部污染
      if self.prev_t is not None:
        dt = t - self.prev_t
        if dt < -0.001 or dt > 60.0:
          for tr in self.tracks.values():
            tr["first_t"], tr["last_t"] = t, t
          self.last_obj_msg_t, self.last_report = t, t
          self.n_obj_msg, self.gap_max = 0, 0.0
          self.warn_suppress_until = t + 15.0
          self.f.write(f"#CLOCKJUMP {time.strftime('%F %T')},{t:.2f},dt={dt:+.1f}s,vEgo={v_ego:.2f}\n")
      self.prev_t = t
      # can_capnp_to_list返回 [(nanos, [(address, dat, src), ...]), ...]
      for _nanos, frames in can_list:
        for address, dat, src in frames:
          if src != MR76_BUS:
            continue
          if address == MR76_MSG_OBJECT:
            if len(dat) < 8:
              continue
            self.n_obj_msg += 1
            # 帧间隔跟踪(断流检测): 本帧与上一条0x60B的间隔
            gap = t - self.last_obj_msg_t
            if gap > self.gap_max:
              self.gap_max = gap
            self.last_obj_msg_t = t
            tid = _sig(dat, 7, 8)
            d_rel = _sig(dat, 15, 13) * 0.2 - 500.0
            y_rel = -(_sig(dat, 18, 11) * 0.2 - 204.6)   # DistLat右正 → OP左正
            v_rel = _sig(dat, 39, 10) * 0.25 - 128.0
            yv_rel = _sig(dat, 45, 9) * 0.25 - 64.0
            dyn = _sig(dat, 50, 3)
            cls = _sig(dat, 52, 2)
            rcs = _sig(dat, 63, 8) * 0.5 - 64.0
            tr = self.tracks.get(tid)
            if tr is not None:
              # 单帧跳变检测(虚警特征): 纵向>6m或横向>3m
              if abs(d_rel - tr["dRel"]) > 6.0 or abs(y_rel - tr["yRel"]) > 3.0:
                self.n_jump += 1
              tr["last_t"], tr["dRel"], tr["yRel"] = t, d_rel, y_rel
              tr["vRel"], tr["rcs"] = v_rel, rcs
            else:
              self.tracks[tid] = {"first_t": t, "last_t": t, "dRel": d_rel,
                                  "yRel": y_rel, "vRel": v_rel, "rcs": rcs}
              tr = self.tracks[tid]
              self.win_new += 1
            # age=该ID连续存活时长(闪烁track的age会很小, 稳定track持续增长)
            if v_ego >= MR76_LOG_MIN_VEGO:   # 仅行驶时记录明细, 静止不收集
              self.f.write(f"{t:.2f},{v_ego:.2f},{tid},{t - tr['first_t']:.2f},{d_rel:.2f},"
                           f"{y_rel:.2f},{v_rel:.2f},{yv_rel:.2f},{dyn},{cls},{rcs:.1f}\n")
          elif address == MR76_MSG_STATUS and len(dat) >= 3:
            self.no_obj = dat[0]
            self.meas_count = (dat[1] << 8) | dat[2]
      # 1Hz统计行 + track超时清理 + 丢报自动警告(超时清理始终执行, 输出仅行驶时)
      if t - self.last_report >= MR76_REPORT_INTERVAL:
        alive = 0
        for tid in list(self.tracks.keys()):
          if t - self.tracks[tid]["last_t"] > MR76_TRACK_TIMEOUT:
            del self.tracks[tid]
            self.win_lost += 1
          else:
            alive += 1
        period = t - self.last_report
        # 周期时长异常(时钟跳变残留)时rate无效(-1), 不参与WARN判定
        rate = self.n_obj_msg / period if 0.001 < period < 10.0 else -1.0
        cur_gap = t - self.last_obj_msg_t if rate >= 0 else 0.0   # 当前已持续的无帧时长
        gap_worst = max(self.gap_max, cur_gap) if rate >= 0 else 0.0
        if v_ego >= MR76_LOG_MIN_VEGO:   # 仅行驶时输出统计/警告, 静止不收集
          self.f.write(f"#STAT {t:.2f},{v_ego:.2f},noObj={self.no_obj},alive={alive},"
                       f"rate={rate:.1f}Hz,jump={self.n_jump},new={self.win_new},"
                       f"lost={self.win_lost},gapMax={gap_worst:.2f}s,meas={self.meas_count}\n")
          # 丢报警告: 雷达有目标(noObj>0)但帧率骤降或帧间隔拉长 → #WARN标记时间点+车速
          # (时钟跳变后15s内抑制, 防NTP校准的回拨/前跳误报)
          if (self.no_obj > 0 and rate >= 0 and t >= self.warn_suppress_until
                  and t - self.warn_cooldown_t > MR76_WARN_COOLDOWN):
            reasons = []
            if rate < MR76_WARN_RATE_MIN:
              reasons.append(f"RATE_DROP {rate:.1f}Hz<{MR76_WARN_RATE_MIN:.0f}Hz")
            if gap_worst > MR76_WARN_GAP_MAX:
              reasons.append(f"GAP {gap_worst:.2f}s>{MR76_WARN_GAP_MAX}s")
            if reasons:
              self.f.write(f"#WARN {time.strftime('%F %T')},{t:.2f},vEgo={v_ego:.2f},"
                           f"{' & '.join(reasons)},noObj={self.no_obj},alive={alive},"
                           f"msgCnt={self.n_obj_msg},meas={self.meas_count}\n")
              self.warn_cooldown_t = t
        self.last_report, self.n_obj_msg, self.n_jump = t, 0, 0
        self.win_new = self.win_lost = 0
        self.gap_max = 0.0
        # [FIX-IO] 1Hz显式落盘: 配合64KB块缓冲, 保证日志仍可实时查看,
        # 同时把写盘次数从"每行一次"降到约1次/秒。断电最多丢1秒明细。
        self.f.flush()
    except Exception as e:   # 分析日志绝不影响主循环
      print(f"MR76Analyzer feed异常: {e}")


# forward
carlog.addHandler(ForwardingHandler(cloudlog))


# ============================================================================
# Auxiliary obstacle sensors -> carState blind-spot veto
#
# >>> CURRENTLY DISABLED (both sources default OFF).  See __init__. <<<
#
# Road test 2026-09-28: OR-ing these sensors into the blind-spot check made the
# check assert almost continuously.  That is NOT acceptable -- the blind-spot
# check is a safety-relevant signal, and a permanently-asserted blind spot both
# blocks legitimate lane changes and makes the voice prompt announce "car there"
# nonstop.  The factory BSM alone is the source of truth; the auxiliary sensors
# are deliberately NOT used to "enhance" it.
#
# The wiring below is kept intact and can be re-enabled at runtime by setting
#   dp_amap_lidar_enable=1 / dp_mr76_lane_enable=1
# but both default to OFF, so a missing/cleared param means "do not block".
#
# What the veto does when enabled:
# The side WiFi LiDAR (amapNavi) and the front MR76 radar are AUXILIARY ONLY.
# They never create RadarData / radarTracks / leads, and they can never
# command brake, throttle or steering.  The single thing they may do is
# *block* a lane change, by OR-ing into carState.leftBlindspot /
# rightBlindspot -- the one choke point every downstream consumer honours:
#
#   desire_helper.py:67        blocks the automatic lane change
#   selfdrived.py:356          raises the laneChangeBlocked alert
#   ui blind_spot_indicators / turn_signal
#
# The OEM BSM bits already written by the car port (ford/carstate.py:159) are
# OR-ed, never overwritten, so the factory blind-spot detection keeps working.
# ============================================================================

# amapNavi.leftBlind / rightBlind bit layout (see carrot/amap_navi.py:250):
#   0x8 = side lane line present  (NOT an obstacle -> must never be used, or
#                                  the blind spot latches on and lane changes
#                                  are blocked permanently)
#   0x4 = LiDAR detected a vehicle
#   0x2 = camera blind spot (reported by the external app)
#   0x1 = LiDAR blind spot
AMAP_LIDAR_MASK_DEFAULT = 0x5  # LiDAR only: 0x1 | 0x4

AMAP_LIDAR_ENABLE_PARAM = "dp_amap_lidar_enable"
AMAP_LIDAR_MASK_PARAM = "dp_amap_lidar_mask"
MR76_LANE_ENABLE_PARAM = "dp_mr76_lane_enable"

# Param refresh period, in card cycles.  card runs at 100 Hz.
AUX_VETO_PARAM_PERIOD = 100


def read_param_direct(key: str) -> str | None:
  """Read a param straight from disk.

  The prebuilt libparams_c.so in this fork does not know the custom dp_* keys,
  so Params() cannot be used for them.  The control code in this fork reads the
  param files directly; follow the same convention here.
  """
  for path in (f"/dev/shm/params/{key}", f"/data/params/d/{key}"):
    try:
      with open(path) as f:
        return f.read().strip()
    except Exception:
      continue
  return None


def read_param_int(key: str, default: int) -> int:
  raw = read_param_direct(key)
  if not raw:
    return default
  try:
    return int(float(raw))
  except (TypeError, ValueError):
    return default


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing, block=True)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams
  CP_SP: structs.CarParamsSP
  CP_SP_capnp: custom.CarParamsSP

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'amapNavi'] + ['carControlSP', 'longitudinalPlanSP'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'radarTracks'] + ['carParamsSP', 'carStateSP'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.CS_SP_prev = custom.CarStateSP.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = False  # self.params.get_bool("IsReleaseBranch")
    is_release_sp = self.params.get_bool("IsReleaseSpBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      fixed_fingerprint = (self.params.get("CarPlatformBundle") or {}).get("platform", None)
      init_params_list_sp = sunnypilot_interfaces.initialize_params(self.params)

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params,
                        fixed_fingerprint, init_params_list_sp, is_release_sp)
      sunnypilot_interfaces.setup_interfaces(self.CI, self.params)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP, self.CI.CP_SP)
      self.CP = self.CI.CP
      self.CP_SP = self.CI.CP_SP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True, block=True)
    else:
      self.CI, self.CP, self.CP_SP = CI, CI.CP, CI.CP_SP
      self.RI = RI

    self.CP.alternativeExperience = 0
    # mads
    set_alternative_experience(self.CP, self.CP_SP, self.params)
    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Dynamic Experimental Control
    self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")

    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key, block=True)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp, block=True)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes, block=True)
    self.params.put("CarParamsCache", cp_bytes)
    self.params.put("CarParamsPersistent", cp_bytes)

    # Write CarParamsSP for controls
    # convert to pycapnp representation for caching and logging
    self.CP_SP_capnp = convert_to_capnp(self.CP_SP)
    cp_sp_bytes = self.CP_SP_capnp.to_bytes()
    self.params.put("CarParamsSP", cp_sp_bytes, block=True)
    self.params.put("CarParamsSPCache", cp_sp_bytes)
    self.params.put("CarParamsSPPersistent", cp_sp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # auxiliary obstacle-veto state (side WiFi LiDAR + front MR76 radar)
    #
    # OFF BY DEFAULT -- road test 2026-09-28.
    # Feeding the LiDAR / MR76 into the blind-spot check made it assert almost
    # continuously, which simultaneously (a) made the voice prompt announce
    # "car there" nonstop and (b) blocked lane changes that were actually clear.
    # The blind-spot check is therefore driven by the FACTORY BSM ALONE.
    # The auxiliary sensors are deliberately NOT used to "enhance" it.
    # Both sources remain available behind dp_* params, but are off unless the
    # user explicitly turns them on.
    self.aux_veto_param_counter = 0
    self.aux_lidar_enable = False
    self.aux_lidar_mask = AMAP_LIDAR_MASK_DEFAULT
    self.aux_mr76_lane_enable = False
    self.aux_lidar_bits = (0, 0)
    self.aux_mr76_lane = {"left": False, "right": False, "fresh": False, "count": 0}
    # [AO_MR76_PUBLISH] params 落盘去重缓存
    self._ao_mr76_pub_cache = None

    # MR76 bus1 稳定性日志(MR76_LOG_ENABLE=0 时零开销; 运行时可热切换)
    self.mr76_log = MR76Analyzer()

    # log fingerprint in sentry
    sunnypilot_interfaces.log_fingerprint(self.CP)

  def _refresh_aux_veto_params(self) -> None:
    # defaults are 0 == OFF; see the note in __init__ for why
    self.aux_lidar_enable = read_param_int(AMAP_LIDAR_ENABLE_PARAM, 0) != 0
    self.aux_lidar_mask = read_param_int(AMAP_LIDAR_MASK_PARAM, AMAP_LIDAR_MASK_DEFAULT)
    self.aux_mr76_lane_enable = read_param_int(MR76_LANE_ENABLE_PARAM, 0) != 0
    # MR76 bus1 日志热开关: 置 1 立即开始记录, 置 0 停止(flush后零开销)
    try:
      self.mr76_log.set_enabled(read_param_int(MR76_LOG_ENABLE_PARAM, MR76_LOG_ENABLE) != 0)
    except Exception:
      pass

  def _ao_publish_mr76_lane(self) -> None:
    """[AO_MR76_PUBLISH] 把 MR76 相邻车道占用写入 params 文件。

    card.py 与 modeld.py 是两个进程；fork 的 libparams_c.so 不认自定义键，
    因此沿用 AOBsmZone 的 "直接写参数文件" 约定。

    只在值变化时落盘。异常一律吞掉：本函数绝不能影响 card 主循环。
    """
    try:
      lane = self.aux_mr76_lane or {}
      enabled = bool(self.aux_mr76_lane_enable)
      fresh = bool(enabled and lane.get("fresh"))

      if not fresh:
        fresh_v = 0
        lane_v = 0
      else:
        fresh_v = 1
        lane_v = (10 if lane.get("left") else 0) + (1 if lane.get("right") else 0)

      if getattr(self, "_ao_mr76_pub_cache", None) == (fresh_v, lane_v):
        return
      self._ao_mr76_pub_cache = (fresh_v, lane_v)

      for _k, _v in (("AOMr76Fresh", str(fresh_v)), ("AOMr76Lane", str(lane_v))):
        for _d in ("/dev/shm/params", "/data/params/d"):
          try:
            with open(os.path.join(_d, _k), "w") as _f:
              _f.write(_v)
            break
          except Exception:
            continue
    except Exception:
      pass

  def apply_aux_blindspot_veto(self, CS: car.CarState) -> None:
    """OR the auxiliary obstacle sensors into the carState blind spots.

    Strictly additive: the OEM BSM bits set by the car port are preserved.
    Blocking-only: it can prevent a lane change, never cause one.
    """
    if self.aux_veto_param_counter % AUX_VETO_PARAM_PERIOD == 0:
      self._refresh_aux_veto_params()
    self.aux_veto_param_counter += 1

    block_left = False
    block_right = False

    # ---- side WiFi LiDAR (amapNavi) --------------------------------------
    if self.aux_lidar_enable and self.sm.seen['amapNavi'] and self.sm.valid['amapNavi']:
      try:
        left_bits = int(self.sm['amapNavi'].leftBlind)
        right_bits = int(self.sm['amapNavi'].rightBlind)
        self.aux_lidar_bits = (left_bits, right_bits)
        if left_bits & self.aux_lidar_mask:
          block_left = True
        if right_bits & self.aux_lidar_mask:
          block_right = True
      except Exception:
        self.aux_lidar_bits = (0, 0)

    # ---- front MR76 radar, adjacent-lane occupancy ------------------------
    #
    # [AO_MR76_PUBLISH] 把占用结果发布到 params 文件，供独立的 modeld 进程
    # 读取并交给 auto_overtake 的 _mr76_lane_safe() 硬门使用。
    #
    # 背景：modeld 通过 SubMaster 收 capnp 消息，拿不到本进程的 self.RI；
    # 而 auto_overtake 需要 mr76_left_obstacle / mr76_right_obstacle 等
    # 独立参数（不只是 OR 进 BSM 的布尔量）。此处复用 AOBsmZone 那套
    # "写 params 文件" 的已验证通道。
    #
    # 语义：
    #   AOMr76Fresh = 0  未接入 / stale / 未使能 -> 消费者必须"不阻断"
    #   AOMr76Fresh = 1  数据可用 -> AOMr76Lane 的左右位才有效
    #   AOMr76Lane  = left*10 + right   （1 = 该侧相邻车道有目标）
    #
    # 注意：仅在数值变化时写文件（本函数跑在 CAN 100Hz 循环里）。
    if self.aux_mr76_lane_enable:
      try:
        get_lane_occupancy = getattr(self.RI, 'get_mr76_lane_occupancy', None)
        if get_lane_occupancy is not None:
          lane = get_lane_occupancy()
          self.aux_mr76_lane = lane
          if lane.get('fresh'):
            block_left = block_left or bool(lane.get('left'))
            block_right = block_right or bool(lane.get('right'))
      except Exception:
        self.aux_mr76_lane = {"left": False, "right": False, "fresh": False, "count": 0}

    self._ao_publish_mr76_lane()

    # ---- OR into carState (OEM BSM already set by the car port) -----------
    if block_left:
      CS.leftBlindspot = True
    if block_right:
      CS.rightBlindspot = True

  def state_update(self) -> tuple[car.CarState, custom.CarStateSP, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)

    # Update carState from CAN
    CS, CS_SP = self.CI.update(can_list)
    CS_SP = convert_to_capnp(CS_SP)

    # Update radar tracks from CAN
    # [FIX] vEgo feeds the standstill exemption in the Ford radar interface.
    RD: structs.RadarDataT | None = self.RI.update(can_list, CS.vEgo)

    # MR76 bus1 稳定性日志(bus1 0x60A/0x60B, 关闭时零开销)
    self.mr76_log.feed(can_list, CS.vEgo)

    self.sm.update(0)

    # auxiliary obstacle sensors -> blind-spot veto (additive, fail-safe)
    try:
      self.apply_aux_blindspot_veto(CS)
    except Exception:
      pass

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_speed_limit_assist(self.is_metric, self.sm['longitudinalPlanSP'])
    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode, self.dynamic_experimental_control)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, CS_SP, RD

  def state_publish(self, CS: car.CarState, CS_SP: custom.CarStateSP, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('radarTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.radarTracks = RD
      self.pm.send('radarTracks', tracks_msg)

    # carParamsSP - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_sp_send = messaging.new_message('carParamsSP')
      cp_sp_send.valid = True
      cp_sp_send.carParamsSP = self.CP_SP_capnp
      self.pm.send('carParamsSP', cp_sp_send)

    cs_sp_send = messaging.new_message('carStateSP')
    cs_sp_send.valid = CS.canValid
    cs_sp_send.carStateSP = CS_SP
    self.pm.send('carStateSP', cs_sp_send)

  def controls_update(self, CS: car.CarState, CC: car.CarControl, CC_SP: custom.CarControlSP):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, self.CP_SP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, convert_carControlSP(CC_SP), now_nanos)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def step(self):
    CS, CS_SP, RD = self.state_update()

    self.state_publish(CS, CS_SP, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'], self.sm['carControlSP'])

    self.initialized_prev = initialized
    self.CS_prev = CS
    self.CS_SP_prev = CS_SP

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl

      # sunnypilot
      self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")
      self.v_cruise_helper.read_custom_set_speed_params()

      time.sleep(0.1)

  def card_thread(self):
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
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
