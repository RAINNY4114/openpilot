#!/usr/bin/env python3
"""amapnavi 的共享状态容器与公共常量。

从 amap_navi.py 中拆出来，供传输层 / 评估层 / 消息层共同引用，
避免各模块之间直接互相依赖。
"""

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

DT_BROADCAST = 0.1


def f1(x):
  return round(float(x), 1)


def f2(x):
  return round(float(x), 2)


class SharedData:
  """采集线程与消费线程之间交换的全部状态。"""

  def __init__(self):
    #=============共享数据（来自amap_navi）=============
    #盲区信号
    self.left_lane = 0 #车道线类型
    self.right_lane = 0
    self.left_lane_blind = 0 #车道线阻止变道
    self.right_lane_blind = 0
    self.left_blind = False #摄像头盲区信号
    self.right_blind = False
    self.lidar_lblind = False #雷达盲区信号
    self.lidar_rblind = False
    self.lidar_lfblind = False #雷达盲区信号
    self.lidar_lbblind = False
    self.lidar_rfblind = False
    self.lidar_rbblind = False
    self.lidar_car_lblind = False #车身雷达盲区
    self.lidar_car_rblind = False #车身雷达盲区
    self.lf_drel = {} #雷达左前车距离
    self.lb_drel = {} #雷达左后车距离
    self.rf_drel = {} #雷达右前车距离
    self.rb_drel = {} #雷达右后车距离
    self.lf_xrel = {} #雷达左前车距离
    self.lb_xrel = {} #雷达左后车距离
    self.rf_xrel = {} #雷达右前车距离
    self.rb_xrel = {} #雷达右后车距离
    self.lidar_l = False
    self.lidar_r = False
    self.camera_l = False
    self.camera_r = False
    self.lf_vrel = None #雷达左前车相对速度
    self.lb_vrel = None #雷达左后车相对速度
    self.rf_vrel = None #雷达右前车相对速度
    self.rb_vrel = None #雷达右后车相对速度
    self.op_blocked = False
    self.road_blocked = False

    self.main_lf_xrel = None
    self.main_lb_xrel = None
    self.main_rf_xrel = None
    self.main_rb_xrel = None

    self.main_lf_drel = None
    self.main_lb_drel = None
    self.main_rf_drel = None
    self.main_rb_drel = None

    #客户端控制命令
    self.cmd_index = -1
    self.remote_cmd = ""
    self.remote_arg = ""

    self.ext_blinker = BLINKER_NONE # 外挂控制器转向灯状态
    self.ext_state = 0  # 外挂控制器的数量
    self.blinker_ctrl = BLINKER_NONE

    #=============共享数据（desire_helper）=============
    self.leftFrontBlind = None
    self.rightFrontBlind = None

    # =============共享数据（carrotMan）=============
    self.roadcate = None
    self.lat_a = None
    self.max_curve = None

    # =============共享数据（carState）=============
    self.carState = False
    self.standstill = False
    self.v_ego_kph = None
    self.v_cruise_kph = None
    self.v_ego_m = None
    self.vEgo = None
    self.aEgo = None
    self.steer_angle = None
    self.gas_press = None
    self.break_press = None
    self.engaged = None
    self.cruise_valid = None
    self.cruise_enable = None
    self.selfdrive_active = None
    self.left_blindspot = None
    self.right_blindspot = None

    self.showDebugLog = 0

  # 注意：left_blindspot / right_blindspot 是「原车盲区」字段（见 __init__ 与
  # vehicle_state），综合盲区必须用下面这两个 *_combined 方法——两者同名会让
  # 实例字段把方法遮蔽掉，调用时变成 "NoneType is not callable"。
  def left_blindspot_combined(self):
    """左侧综合盲区（车道线 / 车身 / 雷达 / 摄像头）。"""
    return bool(self.left_blind or self.lidar_lblind or self.left_lane_blind
                or self.lidar_lfblind or self.lidar_lbblind)

  def right_blindspot_combined(self):
    """右侧综合盲区（车道线 / 车身 / 雷达 / 摄像头）。"""
    return bool(self.right_blind or self.lidar_rblind or self.right_lane_blind
                or self.lidar_rfblind or self.lidar_rbblind)

  def clear_lidar_distances(self):
    """清空每帧重填的雷达距离表。"""
    for field in ("lf_drel", "lb_drel", "rf_drel", "rb_drel",
                  "lf_xrel", "lb_xrel", "rf_xrel", "rb_xrel"):
      getattr(self, field).clear()
