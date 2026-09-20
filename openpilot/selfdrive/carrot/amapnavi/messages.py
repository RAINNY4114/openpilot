#!/usr/bin/env python3
"""发往外挂模块（导航 / 雷达 / 转向灯）的 JSON 消息构造。

从 amap_navi.py 中拆出，只依赖 cereal 与共享状态。
"""

import json

from openpilot.selfdrive.carrot.amapnavi.config import unified_params
from openpilot.selfdrive.carrot.amapnavi.shared_state import BLINKER_NONE, f1, f2

DISTANCE_FIELDS = ("lf_drel", "lb_drel", "rf_drel", "rb_drel",
                   "lf_xrel", "lb_xrel", "rf_xrel", "rb_xrel")


class NaviMessageBuilder:
  """按设备类型生成要下发的一帧消息。"""

  def __init__(self, shared_data, params, sm, listen_port=4211, lane_port=4213, navi_port=7706):
    self.shared_data = shared_data
    self.params = params
    self.sm = sm
    self.listen_port = listen_port
    self.lane_port = lane_port
    self.navi_port = navi_port
    self.local_ip_address = "0.0.0.0"

    # 模型事件（提示音 / 倒计时）边沿检测
    self.model_event_type = 0
    self.sec_count_down = 0

  # ------------------------------------------------------------------ 分发
  def build(self, kind):
    if kind == "navi":
      return self.build_navi()
    if kind == "lidar":
      return self.build_lidar()
    if kind == "blinker":
      return self.build_blinker()
    if kind == "broadcast_op":
      return self.build_broadcast(self.listen_port, "op")
    if kind == "broadcast_lane":
      return self.build_broadcast(self.lane_port, "lane")
    if kind == "broadcast_navi":
      return self.build_broadcast(self.navi_port, "navi")
    raise ValueError(f"unknown message kind: {kind}")

  # ------------------------------------------------------------------ 导航
  def build_navi(self):
    shared = self.shared_data
    sm = self.sm
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port
    msg['device'] = "op"
    isOnroad = unified_params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad

    if isOnroad:
      if shared.carState:
        if shared.v_cruise_kph is not None:
          msg['v_cruise_kph'] = shared.v_cruise_kph
        if shared.v_ego_kph is not None:
          msg['v_ego_kph'] = shared.v_ego_kph
        if shared.vEgo is not None:
          msg["vego"] = shared.vEgo
        if shared.aEgo is not None:
          msg["aego"] = shared.aEgo
        if shared.steer_angle is not None:
          msg["steer_angle"] = shared.steer_angle
        if shared.gas_press is not None:
          msg["gas_press"] = shared.gas_press
        if shared.break_press is not None:
          msg["break_press"] = shared.break_press
        if unified_params.get_bool("EnableCruiseStateShow"):
          if shared.engaged is not None:
            msg["engaged"] = shared.engaged
        else:
          msg["engaged"] = False
        if not unified_params.get_bool("DisableBlindSpot"):
          if shared.left_blindspot is not None:
            msg["left_blindspot"] = shared.left_blindspot
          if shared.right_blindspot is not None:
            msg["right_blindspot"] = shared.right_blindspot
        else:
          msg["left_blindspot"] = False
          msg["right_blindspot"] = False

      if sm.alive['radarState']:
        self._add_radar_state(msg, disable_blindspot=unified_params.get_bool("DisableBlindSpot"), full=True)

      # 侧向目标的相对速度
      for corner in ("lf", "lb", "rf", "rb"):
        value = getattr(shared, f"{corner}_vrel")
        if value is not None:
          msg[f"{corner}_vrel"] = int(value * 3.6)

      # 前雷达盲区信号
      if shared.leftFrontBlind is not None:
        msg['l_front_blind'] = shared.leftFrontBlind
      if shared.rightFrontBlind is not None:
        msg['r_front_blind'] = shared.rightFrontBlind

      # 激光雷达自身的盲区（纯激光），供 App 判断"激光雷达有没有目标"；
      # 四角分开下发，App 才能在左后/右后/左前/右前卡片上写清是哪一侧的激光盲区。
      msg['lidar_lblind'] = shared.lidar_lblind
      msg['lidar_rblind'] = shared.lidar_rblind
      msg['lidar_lfblind'] = shared.lidar_lfblind
      msg['lidar_lbblind'] = shared.lidar_lbblind
      msg['lidar_rfblind'] = shared.lidar_rfblind
      msg['lidar_rbblind'] = shared.lidar_rbblind
      # 侧向判定阈值：下发给 App，让 App 的自算判据与 CP 同源（避免两套参数各调各的）。
      # 单位与网页参数一致（x0.1）：正=时距(s)，负=绝对距离(m)。
      msg['lidar_front_vdist_time'] = unified_params.get_int("LidarFrontVDistTime")
      msg['lidar_front_vrel_time'] = unified_params.get_int("LidarFrontVRelDistTime")
      msg['lidar_behind_vdist_time'] = unified_params.get_int("LidarBehindVDistTime")
      msg['lidar_behind_vrel_time'] = unified_params.get_int("LidarBehindVRelDistTime")
      # 综合盲区（车道线 / 车身 / 激光 / 摄像头）：App 用它判定"禁止变道"，
      # 与上面的纯激光标志分开，避免把"右侧实线/摄像头"显示成"右后激光有车"。
      msg['blind_l'] = shared.left_blindspot_combined()
      msg['blind_r'] = shared.right_blindspot_combined()
      # 综合盲区的分解标志：摄像头 / 车身 / 车道实线。
      # 没有这些字段时 App 只能把"综合盲区"笼统地标成"摄像头盲区"，
      # 于是"CP 判的实线""车身盲区"都会被误显示成摄像头（实车反馈的问题）。
      msg['blind_camera_l'] = bool(shared.left_blind)
      msg['blind_camera_r'] = bool(shared.right_blind)
      msg['blind_car_l'] = bool(shared.lidar_car_lblind)
      msg['blind_car_r'] = bool(shared.lidar_car_rblind)
      msg['blind_lane_l'] = bool(shared.left_lane_blind)
      msg['blind_lane_r'] = bool(shared.right_lane_blind)

      for field in DISTANCE_FIELDS:
        table = getattr(shared, field)
        for idx in sorted(table.keys()):
          if table[idx] is not None:
            key = field if idx == 0 else f"{field}{idx}"
            msg[key] = table[idx]

      msg['lidar_l'] = shared.lidar_l
      msg['lidar_r'] = shared.lidar_r
      msg['camera_l'] = shared.camera_l
      msg['camera_r'] = shared.camera_r

      if shared.roadcate is not None:
        msg['roadcate'] = shared.roadcate
      if shared.lat_a is not None:
        msg['lat_a'] = f2(shared.lat_a)
      if shared.max_curve is not None:
        msg['max_curve'] = f2(shared.max_curve)

      if sm.alive['carrotMan']:
        carrotMan = sm['carrotMan']
        msg["desire_speed"] = int(carrotMan.desiredSpeed)
        atc_type = carrotMan.atcType
        road_name = carrotMan.szPosRoadName
        x_spd_type = carrotMan.xSpdType
        x_spd_dist = carrotMan.xSpdDist
        shared.op_blocked = ("none" not in atc_type and "prepare" not in atc_type)
        shared.road_blocked = ("隧道" in road_name) or (x_spd_type >= 0 and 0 < x_spd_dist < 500)

      msg['blind_enable'] = (shared.lidar_l or shared.camera_l) and (shared.lidar_r or shared.camera_r)
      msg['op_blocked'] = shared.op_blocked
      msg['road_blocked'] = shared.road_blocked

    if sm.alive['modelV2']:
      self._add_model_state(msg, isOnroad)

    if sm.alive['selfdriveState']:
      shared.selfdrive_active = bool(sm['selfdriveState'].active)
    if shared.cruise_valid is not None:
      msg['active'] = shared.cruise_valid
    cruise_enable = bool(shared.selfdrive_active)
    if cruise_enable:
      msg['active'] = True
    if not msg.get('engaged', False):
      msg['engaged'] = cruise_enable

    return json.dumps(msg)

  # ------------------------------------------------------------------ 雷达模块
  def build_lidar(self):
    shared = self.shared_data
    sm = self.sm
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port
    msg['device'] = "op"
    isOnroad = unified_params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad

    if isOnroad:
      if shared.carState:
        if shared.v_cruise_kph is not None:
          msg['v_cruise_kph'] = shared.v_cruise_kph
        if shared.v_ego_kph is not None:
          msg['v_ego_kph'] = shared.v_ego_kph

      if sm.alive['selfdriveState']:
        shared.selfdrive_active = bool(sm['selfdriveState'].active)
      if shared.cruise_valid is not None:
        msg['active'] = shared.cruise_valid
      cruise_enable = bool(shared.selfdrive_active)
      if cruise_enable:
        msg['active'] = True
      if not msg.get('engaged', False):
        msg['engaged'] = cruise_enable

      if sm.alive['carrotMan']:
        carrotMan = sm['carrotMan']
        msg['atc_type'] = carrotMan.atcType
        msg['road_name'] = carrotMan.szPosRoadName

    return json.dumps(msg)

  # ------------------------------------------------------------------ 转向灯
  def build_blinker(self):
    shared = self.shared_data
    sm = self.sm
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port
    msg['device'] = "op"
    isOnroad = unified_params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad

    if isOnroad:
      if shared.carState:
        if shared.v_cruise_kph is not None:
          msg['v_cruise_kph'] = shared.v_cruise_kph
        if shared.v_ego_kph is not None:
          msg['v_ego_kph'] = shared.v_ego_kph
        if shared.vEgo is not None:
          msg["vego"] = shared.vEgo
        if shared.gas_press is not None:
          msg["gas_press"] = shared.gas_press
        if shared.break_press is not None:
          msg["break_press"] = shared.break_press
      if sm.alive['radarState']:
        self._add_radar_state(msg, disable_blindspot=False, full=False)

    if sm.alive['modelV2']:
      meta = sm['modelV2'].meta
      if hasattr(meta, 'blinker'):
        if shared.blinker_ctrl == BLINKER_NONE or meta.blinker != "none":
          msg['blinker'] = meta.blinker
        else:
          msg['blinker'] = self._blinker_ctrl_value()
      else:
        # 没有 modelV2.meta.blinker 的分支：只下发遥控转向灯控制值
        msg['blinker'] = self._blinker_ctrl_value()

    if sm.alive['selfdriveState']:
      shared.selfdrive_active = bool(sm['selfdriveState'].active)
    if shared.cruise_valid is not None:
      msg['active'] = shared.cruise_valid
    cruise_enable = bool(shared.selfdrive_active)
    if cruise_enable:
      msg['active'] = True
    if not msg.get('engaged', False):
      msg['engaged'] = cruise_enable

    return json.dumps(msg)

  # ------------------------------------------------------------------ 广播
  def build_broadcast(self, port, device):
    return json.dumps({
      'ip': self.local_ip_address,
      'port': port,
      'device': device,
    })

  # ------------------------------------------------------------------ 内部
  def _blinker_ctrl_value(self, default="none"):
    stock = int(unified_params.get_int("StockBlinkerCtrl"))
    ctrl = self.shared_data.blinker_ctrl
    if ctrl == 1:
      return "left" if stock == 0 else "stockleft"
    if ctrl == 2:
      return "right" if stock == 0 else "stockright"
    return default

  def _add_radar_state(self, msg, disable_blindspot, full):
    radar_state = self.sm['radarState']
    lead_one = getattr(radar_state, 'leadOne', None)
    if lead_one is not None and getattr(lead_one, 'status', False):
      msg["lead1"] = True
      if hasattr(lead_one, 'dRel'):
        msg["drel"] = int(lead_one.dRel) if full else f1(lead_one.dRel)
      if hasattr(lead_one, 'vLead'):
        msg["vlead"] = int(lead_one.vLead * 3.6) if full else f1(lead_one.vLead * 3.6)
      if hasattr(lead_one, 'vRel'):
        msg["vrel"] = int(lead_one.vRel * 3.6) if full else f1(lead_one.vRel * 3.6)
      if hasattr(lead_one, 'aRel'):
        msg["lead_accel"] = lead_one.aRel
    else:
      msg["lead1"] = False

    if not full:
      return

    if disable_blindspot:
      msg["l_lead"] = False
      msg["r_lead"] = False
      return

    for side, key in (("leadLeft", "l"), ("leadRight", "r")):
      lead = getattr(radar_state, side, None)
      if lead is not None and getattr(lead, 'status', False):
        msg[f"{key}_lead"] = True
        if hasattr(lead, 'dRel'):
          msg[f"{key}_drel"] = int(lead.dRel)
        if hasattr(lead, 'vLead'):
          msg[f"{key}_vlead"] = int(lead.vLead * 3.6)
        if hasattr(lead, 'vRel'):
          msg[f"{key}_vrel"] = int(lead.vRel * 3.6)
      else:
        msg[f"{key}_lead"] = False

  def _add_model_state(self, msg, isOnroad):
    shared = self.shared_data
    modelV2 = self.sm['modelV2']
    meta = modelV2.meta

    if hasattr(meta, 'eventType'):
      event_type = meta.eventType
      if event_type > 0 and event_type != self.model_event_type:
        self.model_event_type = event_type
        value = event_type & 255
        msg['sound'] = value
        print(f"------sound index {value}")

    if hasattr(meta, 'leftSec'):
      sec_count_down = meta.leftSec
      if self.sec_count_down != sec_count_down:
        self.sec_count_down = sec_count_down
        if sec_count_down == 0:
          msg['sound'] = 26
        elif 0 < sec_count_down <= 10:
          msg['sound'] = 30 + sec_count_down
        elif sec_count_down == 11:
          msg['sound'] = 23
        if 'sound' in msg:
          print(f"------sound index {msg['sound']}")

    if hasattr(meta, 'blinker'):
      msg['blinker'] = meta.blinker

    if not isOnroad:
      return

    left_edge_prob = max(1 - modelV2.roadEdgeStds[0], 0)
    right_edge_prob = max(1 - modelV2.roadEdgeStds[1], 0)
    msg['prob'] = True
    msg['l_lane_prob'] = round(modelV2.laneLineProbs[0], 1)
    msg['l_line_prob'] = round(modelV2.laneLineProbs[1], 1)
    msg['r_line_prob'] = round(modelV2.laneLineProbs[2], 1)
    msg['r_lane_prob'] = round(modelV2.laneLineProbs[3], 1)
    msg['l_edge_prob'] = round(left_edge_prob, 1)
    msg['r_edge_prob'] = round(right_edge_prob, 1)
    msg['l_lane_width'] = round(meta.laneWidthLeft, 1)
    msg['r_lane_width'] = round(meta.laneWidthRight, 1)
    msg['l_edge_dist'] = round(meta.distanceToRoadEdgeLeft, 1)
    msg['r_edge_dist'] = round(meta.distanceToRoadEdgeRight, 1)
    msg['atc_state'] = meta.laneChangeState.raw
    if hasattr(meta, 'laneWidth'):
      msg['lane_width'] = round(meta.laneWidth, 1)

    if shared.max_curve is None and hasattr(modelV2, 'orientationRate') and len(modelV2.orientationRate.z) > 0:
      rates = [float(x) for x in modelV2.orientationRate.z]
      if rates:
        msg['max_curve'] = f1(max(rates, key=abs))

    if shared.leftFrontBlind is None and hasattr(meta, 'leftFrontBlind'):
      msg['l_front_blind'] = meta.leftFrontBlind
    if shared.rightFrontBlind is None and hasattr(meta, 'rightFrontBlind'):
      msg['r_front_blind'] = meta.rightFrontBlind
