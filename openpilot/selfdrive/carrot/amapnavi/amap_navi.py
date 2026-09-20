#!/usr/bin/env python3
"""amapnavi 主服务：外挂雷达 / 摄像头 / 转向灯模块的中枢。

由 cpv9-dev 的 ``amap_navi.py`` 移植并模块化，原本近 2000 行的单体文件拆成：

============================  ==========================================
模块                          职责
============================  ==========================================
``shared_state``              共享状态容器与常量
``blindspot``                 侧向目标跟踪（卡尔曼）与风险评估
``decel_advisor``             盲区受阻时的车速规划
``vehicle_state``             车辆状态获取（自带 carState 订阅）
``protocol``                  UDP 报文解析（协议语义）
``transport``                 UDP 收发 / 客户端管理 / 广播信标
``messages``                  下发消息构造
``config`` / ``config_web``   参数管理 / 参数配置 Web 服务
``lane``                      车道线视频流服务（独立进程）
============================  ==========================================

本文件只做编排：拉起各子系统、按 20Hz 汇总盲区状态、发布 ``amapNavi``。

**解耦说明**：车辆状态由本模块自己订阅 ``carState`` 获得（见
``vehicle_state``），不再依赖项目里既有的 ``CarrotMan`` 等代码。
"""

import threading
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.carrot.amapnavi.blindspot import OccupancyCounter
from openpilot.selfdrive.carrot.amapnavi.config import UnifiedParams
from openpilot.selfdrive.carrot.amapnavi.decel_advisor import HumanLikeConfig, SideTarget, SpeedAdvisor
from openpilot.selfdrive.carrot.amapnavi.messages import NaviMessageBuilder
from openpilot.selfdrive.carrot.amapnavi.protocol import PacketHandler, body_blind

# ------------------------------------------------------------------ 动态盲区判定区域
# 判定区域 = 该侧「目标车道」的横向范围，用检测到的侧面车道宽度换算。
#   EGO 基本居中，所以邻道的横向范围约为 [半个本车道, 1.5 个本车道]，
#   取 1.5 倍车道宽作上限：邻道车（横向约 1 个车道宽）保留，
#   只有更外侧车道/路边（超宽）的目标才屏蔽。
#   旧实现直接拿 1 倍车道宽当上限、还把范围压到 1.2~3.5m，
#   结果邻道车（横向 3.0~3.5m）几乎必然被判为"超宽"而全部屏蔽，
#   动态模式下因此"基本不报盲区"。
SIDE_REGION_MIN_LANE_M = 2.5       # 车道宽读数下限，低于此值认为不可信
SIDE_REGION_MAX_LANE_M = 4.0       # 车道宽读数上限，高于此值认为是异常值
SIDE_REGION_FALLBACK_LANE_M = 3.2  # 读数不可信时的兜底车道宽
SIDE_REGION_FACTOR = 1.5           # 判定区域 = 车道宽 × 该倍数
SIDE_REGION_HARD_MIN_M = 3.5       # 判定区域硬下限（保证邻道车不被误屏蔽）
SIDE_REGION_HARD_MAX_M = 5.5       # 判定区域硬上限（再远就不是邻道了）


def side_region_limit_mm(lane_width_m) -> float:
  """把某侧车道宽度换算成动态盲区判定区域的横向上限 (mm)。"""
  w = float(lane_width_m or 0.0)
  if not (SIDE_REGION_MIN_LANE_M <= w <= SIDE_REGION_MAX_LANE_M):
    w = SIDE_REGION_FALLBACK_LANE_M
  limit = w * SIDE_REGION_FACTOR
  limit = max(SIDE_REGION_HARD_MIN_M, min(SIDE_REGION_HARD_MAX_M, limit))
  return limit * 1000.0
from openpilot.selfdrive.carrot.amapnavi.shared_state import (
  BLINKER_LEFT,
  BLINKER_RIGHT,
  DT_BROADCAST,
  SharedData,
)
from openpilot.selfdrive.carrot.amapnavi.transport import (
  LISTEN_PORT,
  LANE_PORT,
  NAVI_PORT,
  UdpTransport,
  refresh_timeouts,
)
from openpilot.selfdrive.carrot.amapnavi.config import unified_params
from openpilot.selfdrive.carrot.amapnavi.stock_front_blind import (
  StockFrontBlindMonitor,
  apply_stock_front_blind,
)
from openpilot.selfdrive.carrot.amapnavi.vehicle_state import update_from_submaster

CORNERS = ("lf", "lb", "rf", "rb")
DATA_HZ = 20
CLIENT_TIMEOUT_FRAMES = 60   # 计数器下限（秒 / DT）

ATC_TURN = ("turn left", "turn right")
ATC_FORK = ("fork left", "fork right")
ATC_FORK_NOW = ("fork left now", "fork right now")
ATC_EARLY = ("atc left", "atc right")


class AmapNaviServ:
  """外挂 BSD 模块的主服务。"""

  def __init__(self):
    self.shared_data = SharedData()
    self.params = UnifiedParams()

    # 自有订阅：只依赖 cereal，不依赖项目其它代码
    self.sm = messaging.SubMaster(['carState', 'modelV2', 'selfdriveState', 'radarState', 'carrotMan'])
    try:
      self.pm = messaging.PubMaster(['amapNavi'])
    except Exception as e:
      self.pm = None
      print(f"[amapnavi] 'amapNavi' service unavailable (cereal rebuilt?): {e}")

    # 协议层 + 消息层 + 传输层
    self.packet_handler = PacketHandler(self.shared_data, self.params)
    self.builder = NaviMessageBuilder(self.shared_data, self.params, self.sm,
                                      listen_port=LISTEN_PORT, lane_port=LANE_PORT, navi_port=NAVI_PORT)
    self.transport = UdpTransport(self.shared_data, self.params,
                                  packet_handler=self.packet_handler,
                                  message_provider=self.builder.build,
                                  message_builder=self.builder)

    # 去抖（延时释放）
    self.corner_counters = {c: OccupancyCounter(hold_s=1.0, dt=DT_BROADCAST) for c in CORNERS}
    self.solid_counters = {
      'left': OccupancyCounter(hold_s=1.0, dt=DT_BROADCAST),
      'right': OccupancyCounter(hold_s=1.0, dt=DT_BROADCAST),
    }
    self.blind_mask = {c: False for c in CORNERS}

    self.atc_flag = False
    self.dynamicBlindRange = 0
    self.dynamicBlindDistance = 0
    self.disableBlindSpot = False
    self.frame = 0

    # 左右两侧的让行速度规划（人类驾驶风格）
    self.advisors = {'left': SpeedAdvisor(), 'right': SpeedAdvisor()}
    self.last_speed_plan = {'left': None, 'right': None}
    self._update_advisor_config()

    # 原车前雷达的「前侧盲区」（左右相邻车道是否有妨碍变道的目标）
    # 注意：StockFrontBlindMonitor 的第一个位置参数是 cfg，参数对象必须用关键字传，
    # 否则会把 cfg 当成 UnifiedParams，每帧抛 AttributeError（异常被
    # _data_deal_thread 吞掉后，这一帧后面的实线判定/距离汇总等都会被跳过）。
    self.stock_front_blind = StockFrontBlindMonitor(params=self.params)

    self.transport.start()
    threading.Thread(target=self._data_deal_thread, daemon=True).start()

  # ------------------------------------------------------------------ 兼容接口
  def update_navi_carstate(self, sm=None):
    """兼容旧接口：外部有现成 SubMaster 时可直接注入 carState。"""
    from openpilot.selfdrive.carrot.amapnavi.vehicle_state import apply_carstate

    sm = sm if sm is not None else self.sm
    if sm is not None and sm.alive['carState']:
      apply_carstate(self.shared_data, sm['carState'])

  # ------------------------------------------------------------------ 参数
  def update_param(self):
    if self.frame % 100 == 0:
      # 参数统一走 amapnavi 自己的参数模块（config.py 的 UnifiedParams）：
      # 这些 key 没有注册进 openpilot 的 params_keys.h，直接 Params().get_int
      # 会抛 UnknownKeyName，导致整块参数更新失败（且异常被静默吞掉）。
      hold = unified_params.get_int("LidarBsdDelayTime") * 0.1
      for counter in self.corner_counters.values():
        counter.hold_s = hold
      self.solid_counters['left'].hold_s = unified_params.get_int("LaneLineDelayTime") * 0.1
      self.solid_counters['right'].hold_s = unified_params.get_int("LaneLineDelayTime") * 0.1

      handler = self.packet_handler
      handler.min_front_drel_vego_time = unified_params.get_int("LidarFrontVDistTime") * 0.1
      handler.min_front_vrel_vego_time = unified_params.get_int("LidarFrontVRelDistTime") * 0.1
      handler.min_behind_drel_vego_time = unified_params.get_int("LidarBehindVDistTime") * 0.1
      handler.min_behind_vrel_vego_time = unified_params.get_int("LidarBehindVRelDistTime") * 0.1
      handler.min_clearance_m = max(1.0, unified_params.get_int("LidarMinClearance") * 0.1)
      handler.ttc_threshold_s = max(0.5, unified_params.get_int("LidarTtcThreshold") * 0.1)
      handler.risk_horizon_s = max(1.0, unified_params.get_int("LidarRiskHorizon") * 0.1)

      self.dynamicBlindRange = unified_params.get_int("DynamicBlindRange")
      self.dynamicBlindDistance = unified_params.get_int("DynamicBlindDistance")
      self.disableBlindSpot = unified_params.get_bool("DisableBlindSpot")
      self._update_advisor_config()

      # 外部(carrotMan)没有设置调试标志时，使用Web页面设置的值
      if self.shared_data.showDebugLog == 0:
        self.shared_data.showDebugLog = unified_params.get_int("ShowDebugLog")
    self.frame += 1

  def _update_advisor_config(self):
    """把网页上可调的让行参数同步到左右两侧的 SpeedAdvisor。"""
    cfg = HumanLikeConfig(
      time_headway_s=max(0.5, unified_params.get_int("BsdTimeHeadway") * 0.1),
      min_gap_m=max(2.0, unified_params.get_int("BsdMinGap") * 0.1),
      merge_margin_m=max(0.0, unified_params.get_int("BsdMergeMargin") * 0.1),
      rear_danger_gap_m=max(1.0, unified_params.get_int("BsdRearDangerGap") * 0.1),
      speed_margin_kph=max(0.0, float(unified_params.get_int("BsdSpeedMargin"))),
      max_speedup_kph=max(0.0, float(unified_params.get_int("BsdMaxSpeedup"))),
      max_slowdown_kph=max(0.0, float(unified_params.get_int("BsdMaxSlowdown"))),
      accel_limit_kphps=max(0.2, unified_params.get_int("BsdAccelLimit") * 0.1),
      decel_limit_kphps=max(0.2, unified_params.get_int("BsdDecelLimit") * 0.1),
      commit_s=max(0.0, unified_params.get_int("BsdCommitTime") * 0.1),
    )
    for advisor in self.advisors.values():
      advisor.cfg = cfg

  # ------------------------------------------------------------------ 主循环
  def _data_deal_thread(self):
    rk = Ratekeeper(DATA_HZ, print_delay_threshold=0.02)

    while True:
      try:
        self.sm.update(0)
        self.update_param()
        update_from_submaster(self.shared_data, self.sm)
        apply_stock_front_blind(self.shared_data, self.sm, 1.0 / DATA_HZ, self.stock_front_blind)

        self.solid_line_blind()
        self.lidar_object_blind()

        clients = self.transport.snapshot_clients()
        if clients:
          self._aggregate_clients(clients)
        else:
          # 所有外挂设备都掉线了：必须清空派生状态，
          # 否则雷达/摄像头在线标志与盲区标志会一直保持掉线前的值，
          # 造成设备断电后仍然持续误报盲区。
          self._clear_client_state()

        self.public_amap_navi()
        rk.keep_time()
      except Exception as e:
        print(f"_data_deal_thread error: {e}")
        time.sleep(1)

  def _clear_client_state(self):
    """清空所有由外挂设备上报派生出来的状态（设备全部掉线时调用）。"""
    shared = self.shared_data
    shared.clear_lidar_distances()

    shared.lidar_l = shared.lidar_r = False
    shared.camera_l = shared.camera_r = False
    shared.left_blind = shared.right_blind = False
    shared.lidar_lblind = shared.lidar_rblind = False
    shared.lidar_lfblind = shared.lidar_lbblind = False
    shared.lidar_rfblind = shared.lidar_rbblind = False
    shared.lidar_car_lblind = shared.lidar_car_rblind = False

    for corner in CORNERS:
      setattr(shared, f"main_{corner}_drel", None)
      setattr(shared, f"main_{corner}_xrel", None)
      setattr(shared, f"{corner}_vrel", None)
      # 关掉危险标志并复位跟踪器，避免掉线前的残值继续参与风险评估
      self.packet_handler.object_detected[corner] = False
      self.packet_handler.trackers[corner].reset()

  # ------------------------------------------------------------------ 汇总
  def _aggregate_clients(self, clients):
    """遍历所有客户端，汇总设备在线标志与盲区状态。"""
    shared = self.shared_data
    shared.clear_lidar_distances()

    lidar_l = lidar_r = camera_l = camera_r = False
    lidar_lblind = lidar_rblind = left_blind = right_blind = False
    lidar_lfblind = lidar_rfblind = lidar_lbblind = lidar_rbblind = False
    lidar_car_lblind = lidar_car_rblind = False
    left_lidar_id = right_lidar_id = 0

    now = time.time()
    for ip, info in clients.items():
      try:
        device_type = info.get("device", None)
        detect_side = info.get("detect_side", 0)

        if device_type == "lidar":
          if (detect_side & 1) > 0: lidar_l = True
          if (detect_side & 2) > 0: lidar_r = True
          refresh_timeouts(info, now)
        elif device_type == "camera":
          if (detect_side & 1) > 0: camera_l = True
          if (detect_side & 2) > 0: camera_r = True
          refresh_timeouts(info, now)
        else:
          continue

        if info.get("lidar_lfblind", False): lidar_lfblind = True
        if info.get("lidar_lbblind", False): lidar_lbblind = True
        if info.get("lidar_lblind", False):
          lidar_lblind = True
          if body_blind(info.get("lf_drel"), info.get("lf_xrel"),
                        info.get("lb_drel"), info.get("lb_xrel")):
            lidar_car_lblind = True

        if info.get("lidar_rfblind", False): lidar_rfblind = True
        if info.get("lidar_rbblind", False): lidar_rbblind = True
        if info.get("lidar_rblind", False):
          lidar_rblind = True
          if body_blind(info.get("rf_drel"), info.get("rf_xrel"),
                        info.get("rb_drel"), info.get("rb_xrel")):
            lidar_car_rblind = True

        if info.get("left_blind", False): left_blind = True
        if info.get("right_blind", False): right_blind = True

        if (detect_side & 0x01) > 0:
          shared.lf_drel[left_lidar_id] = info.get("lf_drel", None)
          shared.lb_drel[left_lidar_id] = info.get("lb_drel", None)
          shared.lf_xrel[left_lidar_id] = info.get("lf_xrel", None)
          shared.lb_xrel[left_lidar_id] = info.get("lb_xrel", None)
          left_lidar_id += 1
        if (detect_side & 0x02) > 0:
          shared.rf_drel[right_lidar_id] = info.get("rf_drel", None)
          shared.rb_drel[right_lidar_id] = info.get("rb_drel", None)
          shared.rf_xrel[right_lidar_id] = info.get("rf_xrel", None)
          shared.rb_xrel[right_lidar_id] = info.get("rb_xrel", None)
          right_lidar_id += 1

        self.transport.replace_client(ip, info)
      except Exception as e:
        print(f"deal client {ip} failed: {e}")

    # 某个角已经没有任何客户端上报时，main_* 的上一帧残值必须清掉。
    # main_* 由 protocol 在收包时写入，只在"所有客户端掉光"时才会被
    # _clear_client_state() 复位；若某侧雷达掉线而其它客户端仍在，
    # 掉线前的距离/横向/相对速度会一直挂着（??DrelValid 恒为 1）。
    for corner in CORNERS:
      if not any(v is not None for v in getattr(shared, f"{corner}_drel").values()):
        setattr(shared, f"main_{corner}_drel", None)
        setattr(shared, f"main_{corner}_xrel", None)
        setattr(shared, f"{corner}_vrel", None)

    # 动态盲区时用去抖后的四角结果，否则直接用模块上报值
    if (self.dynamicBlindRange == 0 and self.dynamicBlindDistance == 0) or \
       (self.dynamicBlindRange == 1 and not self.atc_flag):
      shared.lidar_lblind = lidar_lblind
      shared.lidar_rblind = lidar_rblind
      shared.lidar_lfblind = lidar_lfblind
      shared.lidar_lbblind = lidar_lbblind
      shared.lidar_rfblind = lidar_rfblind
      shared.lidar_rbblind = lidar_rbblind
    else:
      shared.lidar_lblind = self.left_side_detected()
      shared.lidar_rblind = self.right_side_detected()
      shared.lidar_lfblind = self.corner_active('lf')
      shared.lidar_lbblind = self.corner_active('lb')
      shared.lidar_rfblind = self.corner_active('rf')
      shared.lidar_rbblind = self.corner_active('rb')

    shared.lidar_car_lblind = lidar_car_lblind
    shared.lidar_car_rblind = lidar_car_rblind
    shared.left_blind = left_blind
    shared.right_blind = right_blind
    shared.lidar_l = lidar_l
    shared.lidar_r = lidar_r
    shared.camera_l = camera_l
    shared.camera_r = camera_r

  # ------------------------------------------------------------------ 评估
  def solid_line_blind(self):
    """实线阻止变道（去抖后写入共享状态）。"""
    shared = self.shared_data
    left = self.solid_counters['left'].update(shared.left_lane >= 1)
    right = self.solid_counters['right'].update(shared.right_lane >= 1)
    shared.left_lane_blind = left
    shared.right_lane_blind = right

  def lidar_object_blind(self):
    """计算动态盲区屏蔽，并对四角危险标志做去抖。"""
    self._update_atc_flag()
    # 数据超时复位：设备静默时一个包都不会来，只能在这里按时间清标志，
    # 否则目标消失后危险标志会一直亮着（UI 上表现为"距离没了图标还在"）。
    self.packet_handler.expire_corners()
    self.blind_mask = self._dynamic_blind_mask()

    for corner in CORNERS:
      active = self.packet_handler.object_detected[corner]
      if self.blind_mask[corner] or self.shared_data.standstill:
        active = False
      self.corner_counters[corner].update(active)

  def _update_atc_flag(self):
    atc_type = self.sm['carrotMan'].atcType if self.sm.alive['carrotMan'] else "none"
    self.atc_flag = atc_type in (ATC_TURN + ATC_FORK + ATC_FORK_NOW + ATC_EARLY)
    self.packet_handler.atc_flag = self.atc_flag

  # 动态盲区的判定区域 = 该侧「目标车道」的横向范围（用检测到的侧面车道宽度算）。
  #   EGO 基本居中，所以邻道的横向范围约为 [半个本车道, 1.5 个本车道]，
  #   取 1.5 倍车道宽作上限：邻道车（横向约 1 个车道宽）保留，
  #   只有更外侧车道/路边（超宽）的目标才屏蔽。
  #   旧实现直接拿 1 倍车道宽当上限并把范围压到 1.2~3.5m，
  #   结果邻道车（3.0~3.5m）几乎必然被判为"超宽"而全部屏蔽 → 动态模式基本不报盲区。
  SIDE_REGION_MIN_LANE_M = 2.5     # 车道宽读数下限，低于此值认为不可信
  SIDE_REGION_MAX_LANE_M = 4.0     # 车道宽读数上限，高于此值认为是异常值
  SIDE_REGION_FALLBACK_LANE_M = 3.2  # 读数不可信时的兜底车道宽
  SIDE_REGION_FACTOR = 1.5         # 判定区域 = 车道宽 × 该倍数
  SIDE_REGION_HARD_MIN_M = 3.5     # 判定区域硬下限（保证邻道车不被误屏蔽）
  SIDE_REGION_HARD_MAX_M = 5.5     # 判定区域硬上限（再远就不是邻道了）

  def _side_region_limit_mm(self, lane_width_m) -> float:
    """把该侧车道宽度换算成动态盲区判定区域的横向上限 (mm)。"""
    w = float(lane_width_m or 0.0)
    if not (self.SIDE_REGION_MIN_LANE_M <= w <= self.SIDE_REGION_MAX_LANE_M):
      w = self.SIDE_REGION_FALLBACK_LANE_M
    limit = w * self.SIDE_REGION_FACTOR
    limit = max(self.SIDE_REGION_HARD_MIN_M, min(self.SIDE_REGION_HARD_MAX_M, limit))
    return limit * 1000.0

  def _dynamic_blind_mask(self):
    """动态盲区屏蔽：目标横向超出该侧目标车道范围时不计入盲区。

    ``DynamicBlindRange`` 的两种语义（与 cpv9-dev 一致）：
      1 = 只在导航变道（ATC/转向）期间使用侧面车道宽度判定；
      2 = 一直使用侧面车道宽度判定。

    判定区域由 :func:`side_region_limit_mm` 按侧面车道宽度换算。
    """
    mask = {c: False for c in CORNERS}
    if self.dynamicBlindRange < 1:
      return mask
    if not (self.atc_flag or self.dynamicBlindRange >= 2):
      return mask

    meta = self.sm['modelV2'].meta if self.sm.alive['modelV2'] else None
    if meta is None:
      return mask

    shared = self.shared_data
    limit_left = side_region_limit_mm(getattr(meta, 'laneWidthLeft', 0.0))
    limit_right = side_region_limit_mm(getattr(meta, 'laneWidthRight', 0.0))
    for corner, limit in (("lf", limit_left), ("lb", limit_left),
                          ("rf", limit_right), ("rb", limit_right)):
      xrel = getattr(shared, f"main_{corner}_xrel")
      if xrel is not None and xrel > limit:
        mask[corner] = True
    return mask

  def corner_active(self, corner):
    return self.corner_counters[corner].detected

  def left_side_detected(self):
    return self.corner_active('lf') or self.corner_active('lb')

  def right_side_detected(self):
    return self.corner_active('rf') or self.corner_active('rb')

  # ------------------------------------------------------------------ 发布
  def public_amap_navi(self):
    if self.pm is None:
      return
    shared = self.shared_data
    msg = messaging.new_message('amapNavi')
    msg.valid = True
    # 外挂转向灯板回传的转向灯状态：单独一个字段(extBlinker)。数据源只有
    # shared.ext_blinker（板子真实回传，protocol.update_blinker），不用 CP 下发的
    # 命令兜底——UI 要反映的是"控制板到底有没有在打灯"。
    msg.amapNavi.extBlinker = int(shared.ext_blinker)

    # 侧向标志：拆成独立字段，不再按位或。
    #   左侧 6 类来源：激光侧方 / 激光前角 / 激光后角 / 综合盲区 / 车身盲区 / 实线
    msg.amapNavi.blindLidarL = bool(shared.lidar_lblind)
    msg.amapNavi.blindLidarLf = bool(shared.lidar_lfblind)
    msg.amapNavi.blindLidarLb = bool(shared.lidar_lbblind)
    msg.amapNavi.blindCombinedL = bool(shared.left_blind)
    msg.amapNavi.blindCarL = bool(shared.lidar_car_lblind)
    msg.amapNavi.laneBlindL = bool(shared.left_lane_blind)
    msg.amapNavi.blindLidarR = bool(shared.lidar_rblind)
    msg.amapNavi.blindLidarRf = bool(shared.lidar_rfblind)
    msg.amapNavi.blindLidarRb = bool(shared.lidar_rbblind)
    msg.amapNavi.blindCombinedR = bool(shared.right_blind)
    msg.amapNavi.blindCarR = bool(shared.lidar_car_rblind)
    msg.amapNavi.laneBlindR = bool(shared.right_lane_blind)

    # 位图字段仅作兼容保留（旧代码/诊断用），UI 已改用上面的独立字段
    msg.amapNavi.leftBlind = ((8 if shared.left_lane_blind else 0) + (4 if shared.lidar_car_lblind else 0) +
                              (2 if shared.left_blind else 0) + (1 if shared.lidar_lblind else 0) +
                              (16 if shared.lidar_lfblind else 0) + (32 if shared.lidar_lbblind else 0))
    msg.amapNavi.rightBlind = ((8 if shared.right_lane_blind else 0) + (4 if shared.lidar_car_rblind else 0) +
                               (2 if shared.right_blind else 0) + (1 if shared.lidar_rblind else 0) +
                               (16 if shared.lidar_rfblind else 0) + (32 if shared.lidar_rbblind else 0))
    msg.amapNavi.leftLine = shared.left_lane
    msg.amapNavi.rightLine = shared.right_lane
    msg.amapNavi.lineValid = self.transport.lane_online
    msg.amapNavi.leftDevice = ((2 if shared.camera_l else 0) + (1 if shared.lidar_l else 0))
    msg.amapNavi.rightDevice = ((2 if shared.camera_r else 0) + (1 if shared.lidar_r else 0))

    for corner in CORNERS:
      valid = getattr(shared, f"main_{corner}_drel")
      setattr(msg.amapNavi, f"{corner}DrelValid", 1 if valid is not None else 0)
      # 原始单位为 mm，/100 → dm（与源端一致，UI 再 /10 得到米）
      setattr(msg.amapNavi, f"{corner}Drel", 0 if valid is None else int(round(valid / 100)))

    # 外挂客户端(控制器)数量：雷达 / 摄像头 / 转向灯板 + lane 服务
    msg.amapNavi.extState = int(shared.ext_state)

    # 原车前雷达的「前侧盲区」：UI 第一行黄圆图标 + 变道护栏配色（源端取自
    # modelV2.meta.leftFrontBlind，本 fork 由 stock_front_blind 算好后从这里下发）
    msg.amapNavi.lFrontBlind = bool(shared.leftFrontBlind)
    msg.amapNavi.rFrontBlind = bool(shared.rightFrontBlind)

    self.pm.send('amapNavi', msg)

  # ------------------------------------------------------------------ 车速建议
  def suggest_speed_kph(self, side, v_ego_kph, desired_kph, dt=1.0 / DATA_HZ):
    """盲区受阻时的目标车速建议（人类驾驶风格：先决策插前/落后，再平稳执行）。

    :param side: ``'left'`` / ``'right'``
    :return: 建议下发的目标车速 (km/h)
    """
    advisor = self.advisors[side]

    # 静止或极低速时不做让行（人也不会）
    if self.shared_data.standstill or v_ego_kph < 5.0:
      advisor.reset()
      self.last_speed_plan[side] = advisor.last_plan
      return desired_kph

    corners = ("lf", "lb") if side == "left" else ("rf", "rb")
    targets = []
    for corner in corners:
      drel = getattr(self.shared_data, f"main_{corner}_drel")
      vrel = getattr(self.shared_data, f"{corner}_vrel")
      if drel is not None and vrel is not None:
        targets.append(SideTarget(drel_mm=drel, vrel_mps=vrel))

    target = advisor.update(targets, v_ego_kph, desired_kph, dt)
    self.last_speed_plan[side] = advisor.last_plan
    return target


def main():
  """进程入口（被 manager 作为 PythonProcess 拉起，需要一直阻塞）。"""
  serv = AmapNaviServ()

  # 参数配置页面（http://<设备IP>:8088/nav_params），与 BSD 服务同进程
  try:
    from openpilot.selfdrive.carrot.amapnavi.config_web import ConfigWeb

    ConfigWeb(controller=serv, port=8088).start()
  except Exception as e:
    print(f"[amapnavi] 参数配置 Web 服务启动失败: {e}")

  # 工作线程都是 daemon，这里必须阻塞，否则进程会立刻退出
  while True:
    time.sleep(1)


if __name__ == "__main__":
  main()
