#!/usr/bin/env python3
"""
openpilot Long/Lat 实时数据监控页
==================================

通过 openpilot 原接口（cereal messaging / zmq）订阅 long（纵向）与 lat（横向）
相关消息，启动一个本地 HTTP 服务，用 HTML 页面 + SSE 实时展示数据。

数据来源（原接口）：
  - carState            车速 / 加速度 / 转向角等车辆原始状态
  - controlsState       纵向控制器（PID accel 命令）与横向控制器状态
  - selfdriveState      系统运行状态（enabled / active / 告警）
  - longitudinalPlan    纵向规划（目标速度 / 目标加速度 / 前车距离）
  - lateralPlan         横向规划（曲率 / 换道状态）
  - carControl          最终控制指令（latActive / longActive）
  - gpsLocationExternal GPS 经纬度（可选服务，收到后显示）
  - deviceState         设备状态（CPU/GPU/内存/温度/网络/存储等）

运行前提：
  1) 已编译 cereal（在 openpilot 目录执行 scons）
  2) 使用 openpilot 的 python 环境
  3) 数据源运行中：
       - 设备上运行 openpilot（默认本机 127.0.0.1），或
       - 通过 --addr 连接其他主机的消息端（如 comma 设备 IP）

用法示例：
  python tools/long_lat_dashboard/long_lat_dashboard.py
  python tools/long_lat_dashboard/long_lat_dashboard.py --addr 192.168.1.10 --port 8894
  python tools/long_lat_dashboard/long_lat_dashboard.py --mock        # 无数据时用模拟数据演示

打开浏览器访问：http://127.0.0.1:8894
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

# cereal messaging 仅在真实数据模式（默认）下需要；--mock 模拟模式不依赖它。
#
# 不同 openpilot checkout 的目录结构 / PYTHONPATH 不一致（新版把源码放在 <repo>/openpilot 下，
# cereal 以 openpilot.cereal 命名空间发布，且编译产物 capnp 在 venv 里）。这里把常见路径
# 自动加入 sys.path，让脚本“开箱即用”，无需手动 export PYTHONPATH。
for _p in [
  os.path.dirname(os.path.abspath(__file__)),
  os.path.join(os.path.dirname(os.path.abspath(__file__)), 'openpilot'),
  '/data/openpilot',
  '/data/openpilot/openpilot',
]:
  if os.path.isdir(_p) and _p not in sys.path:
    sys.path.insert(0, _p)

try:
  from cereal import messaging
except ImportError:
  messaging = None  # type: ignore[assignment]

SERVICES = ['carState', 'controlsState', 'selfdriveState', 'longitudinalPlan',
            'lateralPlan', 'carControl', 'gpsLocationExternal', 'radarState', 'liveTracks',
            'deviceState']

# 雷达目标类型描述（参考 selfdrive/controls/lib/radar_speed_lib.py 的 target_type_config）
TARGET_TYPE_NAMES = {0: '点目标', 1: '小车', 2: '卡车', 3: '行人',
                     4: '摩托车', 5: '自行车', 6: '宽目标', 7: '未知'}

# 枚举名映射（cereal 中 enum 读取返回成员名字符串）
OPENPILOT_STATE_NAMES = ['disabled', 'preEnabled', 'enabled', 'softDisabling', 'overriding']

# 横向控制器 union 各分支需要提取的字段
LAT_STATE_FIELDS = {
  'pidState': ['active', 'steeringAngleDeg', 'steeringRateDeg', 'angleError',
               'p', 'i', 'f', 'output', 'saturated', 'steeringAngleDesiredDeg'],
  'angleState': ['active', 'steeringAngleDeg', 'output', 'saturated', 'steeringAngleDesiredDeg'],
  'torqueState': ['active', 'error', 'errorRate', 'p', 'i', 'd', 'f', 'output',
                  'saturated', 'actualLateralAccel', 'desiredLateralAccel'],
  'debugState': ['active', 'steeringAngleDeg', 'output', 'saturated'],
  'lqrStateDEPRECATED': ['active', 'steeringAngleDeg', 'i', 'output', 'lqrOutput',
                         'saturated', 'steeringAngleDesiredDeg'],
  'indiStateDEPRECATED': ['active', 'steeringAngleDeg', 'steeringRateDeg',
                          'steeringAccelDeg', 'rateSetPoint', 'accelSetPoint',
                          'accelError', 'delayedOutput', 'delta', 'output',
                          'saturated', 'steeringAngleDesiredDeg',
                          'steeringRateDesiredDeg'],
  'curvatureStateDEPRECATED': ['active', 'actualCurvature', 'desiredCurvature',
                               'error', 'p', 'i', 'f', 'output', 'saturated'],
}

PUSH_INTERVAL = 0.05  # SSE 推送间隔（秒）

# processingDelay 合理上限（秒）：上游定义为 plan 发送时刻 - modelV2 时间戳，
# 在时钟域不一致 / logMonoTime 未填充（为 0）时会得到负值，且随设备运行时间负向无限增大。
# 只有处于 [0, PROCESSING_DELAY_MAX) 的才视为有效。
PROCESSING_DELAY_MAX = 5.0


def _num(obj, attr, default=0.0) -> float:
  """按字段名安全读取数值字段（字段不存在返回 default，不抛异常）
  obj:    capnp 消息对象
  attr:   字段名
  default: 读取失败或字段不存在时的缺省值"""
  try:
    return float(getattr(obj, attr, default))
  except Exception:
    return default


def _boolean(obj, attr, default=False) -> bool:
  """按字段名安全读取布尔字段（字段不存在返回 default，不抛异常）"""
  try:
    return bool(getattr(obj, attr, default))
  except Exception:
    return default


def _enum_name(obj, attr, default='unknown') -> str:
  """按字段名安全读取 enum 名（字段不存在返回 default，不抛异常）"""
  try:
    return str(getattr(obj, attr, default))
  except Exception:
    return 'unknown'


def _valid_delay(obj, attr) -> Optional[float]:
  """按字段名安全读取 processingDelay 并校验（字段缺失或越界返回 None）"""
  try:
    val = float(getattr(obj, attr))
  except Exception:
    return None
  if val < 0 or val > PROCESSING_DELAY_MAX:
    return None
  return val


def extract_state(sm: 'messaging.SubMaster') -> dict:
  """从 SubMaster 中提取 long/lat 相关数据，返回 JSON 可序列化 dict
  sm:    cereal SubMaster（已 update 过一轮的订阅句柄）
  state: 汇总所有服务数据的顶层 dict，供 SSE 推送给前端"""
  state = {
    'frame': sm.frame,                 # 当前已处理的消息总帧号
    'monotime': time.monotonic(),      # 本机单调时钟（秒），前端据此判断数据新鲜度
    'services': {s: {'alive': sm.alive[s], 'valid': sm.valid[s],   # 各订阅服务的
                     'freqOk': sm.freq_ok[s], 'recvFrame': sm.recv_frame[s]}  # 健康状态
                 for s in sm.services},
  }

  # ---------- carState ----------
  cs = sm['carState']                    # carState 消息对象（车辆传感器原始状态）
  cruise = getattr(cs, 'cruiseState', None)  # 巡航子状态（可能为空）
  state['carState'] = {          # 车辆原始状态（车速/踏板/转向/TPMS 等）
    'vEgo': _num(cs, 'vEgo'),                          # 自车速度（m/s，经过滤波）
    'vEgoRaw': _num(cs, 'vEgoRaw'),                    # 自车速度（m/s，未滤波原始值）
    'vEgoCluster': _num(cs, 'vEgoCluster'),            # 仪表盘显示的整车速度（m/s）
    'aEgo': _num(cs, 'aEgo'),                          # 自车纵向加速度（m/s²）
    'yawRate': _num(cs, 'yawRate'),                    # 自车横摆角速度（rad/s）
    'standstill': _boolean(cs, 'standstill'),          # 是否处于静止状态
    'gas': _num(cs, 'gas'),                            # 油门踏板位置（0~1）
    'gasPressed': _boolean(cs, 'gasPressed'),          # 油门踏板是否被踩下
    'brake': _num(cs, 'brake'),                        # 刹车踏板位置（0~1）
    'brakePressed': _boolean(cs, 'brakePressed'),      # 刹车踏板是否被踩下
    'vCruise': _num(cs, 'vCruise'),                    # 设定的巡航车速（m/s）
    'vCruiseCluster': _num(cs, 'vCruiseCluster'),      # 仪表盘显示的巡航车速（m/s）
    'steeringAngleDeg': _num(cs, 'steeringAngleDeg'),  # 方向盘转角（度）
    'steeringAngleOffsetDeg': _num(cs, 'steeringAngleOffsetDeg'),  # 方向盘转角零点偏移（度）
    'steeringRateDeg': _num(cs, 'steeringRateDeg'),    # 方向盘转角速率（度/s）
    'steeringTorque': _num(cs, 'steeringTorque'),      # 驾驶员施加的方向盘扭矩（Nm）
    'steeringTorqueEps': _num(cs, 'steeringTorqueEps'),  # EPS 助力扭矩（Nm）
    'steeringPressed': _boolean(cs, 'steeringPressed'),  # 驾驶员是否握持方向盘
    'gearShifter': _enum_name(cs, 'gearShifter'),      # 当前挡位（P/R/N/D 等枚举名）
    'cruiseEnabled': _boolean(cruise, 'enabled') if cruise else False,  # 巡航是否已激活
    'cruiseSpeed': _num(cruise, 'speed') if cruise else 0.0,            # 巡航设定速度（m/s）
    'cruiseAvailable': _boolean(cruise, 'available') if cruise else False,  # 巡航是否可用
    'latEnabled': _boolean(cs, 'latEnabled'),          # 横向控制（车道保持）是否启用
    'leftBlinker': _boolean(cs, 'leftBlinker'),        # 左转向灯是否点亮
    'rightBlinker': _boolean(cs, 'rightBlinker'),      # 右转向灯是否点亮
    'doorOpen': _boolean(cs, 'doorOpen'),              # 是否有车门未关闭
    'seatbeltUnlatched': _boolean(cs, 'seatbeltUnlatched'),  # 安全带是否未系
    'stockAeb': _boolean(cs, 'stockAeb'),              # 原厂 AEB（自动紧急制动）是否介入
    'stockFcw': _boolean(cs, 'stockFcw'),              # 原厂 FCW（前向碰撞预警）是否触发
    'espActive': _boolean(cs, 'espActive'),            # ESP 车身稳定系统是否激活
    'speedLimit': _num(cs, 'speedLimit'),              # 道路限速（m/s）
    'vCluRatio': _num(cs, 'vCluRatio'),                # 车速与仪表显示的比例系数
    'leftLaneLine': _num(cs, 'leftLaneLine'),          # 左侧车道线检测概率（0~1）
    'rightLaneLine': _num(cs, 'rightLaneLine'),        # 右侧车道线检测概率（0~1）
    'leftLatDist': _num(cs, 'leftLatDist'),            # 到左侧车道线距离（m）
    'rightLatDist': _num(cs, 'rightLatDist'),          # 到右侧车道线距离（m）
    'tpms': {'fl': _num(cs.tpms, 'fl'), 'fr': _num(cs.tpms, 'fr'),   # 四轮胎压（左前/右前）
             'rl': _num(cs.tpms, 'rl'), 'rr': _num(cs.tpms, 'rr')} if getattr(cs, 'tpms', None) is not None else None,  # 四轮胎压（左后/右后），无数据时为 None
  }

  # ---------- controlsState（long/lat 控制器核心数据）----------
  try:
    ctl = sm['controlsState']    # controlsState 消息对象（long/lat 控制器核心数据）
    state['controlsState'] = {
      'longControlState': _enum_name(ctl, 'longControlState'),  # 纵向控制状态机状态（off/pid/longitudinal/pivot 等）
      'upAccelCmd': _num(ctl, 'upAccelCmd'),                    # 纵向 PID 位置项加速度指令（m/s²）
      'uiAccelCmd': _num(ctl, 'uiAccelCmd'),                    # 纵向 PID 积分项加速度指令（m/s²）
      'ufAccelCmd': _num(ctl, 'ufAccelCmd'),                    # 纵向 PID 前馈项加速度指令（m/s²）
      'curvature': _num(ctl, 'curvature'),                      # 当前实际路径曲率（1/m）
      'desiredCurvature': _num(ctl, 'desiredCurvature'),        # 期望路径曲率（1/m）
      'forceDecel': _boolean(ctl, 'forceDecel'),                # 是否强制减速（如碰撞风险）
      'activeLaneLine': _boolean(ctl, 'activeLaneLine'),        # 是否有有效车道线可供横向控制
      'latState': extract_lat_state(ctl),                    # 横向控制状态（详见 extract_lat_state）
    }
  except Exception:
    state['controlsState'] = {}

  # ---------- selfdriveState ----------
  try:
    ss = sm['selfdriveState']    # selfdriveState 消息对象（驾驶状态/告警/行程信息）
    state['selfdriveState'] = {
      'state': _enum_name(ss, 'state'),                    # openpilot 整体状态（disabled/enabled/overriding 等）
      'enabled': _boolean(ss, 'enabled'),                  # 驾驶辅助是否已启用
      'active': _boolean(ss, 'active'),                    # 是否正在主动控制车辆
      'engageable': _boolean(ss, 'engageable'),            # 是否满足可启用条件
      'experimentalMode': _boolean(ss, 'experimentalMode'),  # 是否处于实验模式
      'personality': _enum_name(ss, 'personality'),        # 驾驶风格（激进/标准/保守）
      'alertText1': str(ss.alertText1 or ''),           # 告警主文本
      'alertText2': str(ss.alertText2 or ''),           # 告警副文本
      'alertStatus': _enum_name(ss, 'alertStatus'),        # 告警等级（normal/userPrompt/critical 等）
      'alertSize': _enum_name(ss, 'alertSize'),            # 告警显示尺寸
      'alertType': str(ss.alertType or ''),             # 告警类型描述串
      'distanceTraveled': _num(ss, 'distanceTraveled'),    # 本次行程累计行驶距离（m）
    }
  except Exception:
    state['selfdriveState'] = {}

  # ---------- longitudinalPlan ----------
  try:
    lp = sm['longitudinalPlan']  # longitudinalPlan 消息对象（纵向 MPC 规划结果）
    state['longitudinalPlan'] = {
      'hasLead': _boolean(lp, 'hasLead'),                  # 是否存在有效前车
      'fcw': _boolean(lp, 'fcw'),                          # 前向碰撞预警是否触发
      'aTarget': _num(lp, 'aTarget'),                      # 目标纵向加速度（m/s²）
      'vTargetNow': _num(lp, 'vTargetNow'),                # 当前目标车速（m/s）
      'jTargetNow': _num(lp, 'jTargetNow'),                # 当前目标加加速度（m/s³）
      'cruiseTarget': _num(lp, 'cruiseTarget'),            # 巡航目标车速（m/s，不跟随前车时）
      'desiredDistance': _num(lp, 'desiredDistance'),      # 期望跟车距离（m）
      'tFollow': _num(lp, 'tFollow'),                      # 跟车时距（s）
      'shouldStop': _boolean(lp, 'shouldStop'),            # 是否应停车
      'allowThrottle': _boolean(lp, 'allowThrottle'),      # 是否允许加速
      'allowBrake': _boolean(lp, 'allowBrake'),            # 是否允许制动
      'source': _enum_name(lp, 'longitudinalPlanSource'),  # 纵向规划来源（cruise/lead/radar 等）
      'speeds': list(lp.speeds),                        # MPC 规划的速度曲线（m/s）
      'accels': list(lp.accels),                        # MPC 规划的加速度曲线（m/s²）
      'jerks': list(lp.jerks),                          # MPC 规划的加加速度曲线（m/s³）
      'solverExecutionTime': _num(lp, 'solverExecutionTime'),  # MPC 求解耗时（s）
      'processingDelay': _valid_delay(lp, 'processingDelay'),  # 规划处理延迟（s，异常值过滤为 None）
    }
  except Exception:
    state['longitudinalPlan'] = {}

  # ---------- lateralPlan ----------
  try:
    latp = sm['lateralPlan']     # lateralPlan 消息对象（横向 MPC 规划结果）
    state['lateralPlan'] = {
      'mpcSolutionValid': _boolean(latp, 'mpcSolutionValid'),  # MPC 解是否有效
      'laneWidth': _num(latp, 'laneWidth'),                    # 车道宽度（m）
      'laneChangeState': _enum_name(latp, 'laneChangeState'),  # 变道状态机状态
      'laneChangeDirection': _enum_name(latp, 'laneChangeDirection'),  # 变道方向（left/right/none）
      'desire': _enum_name(latp, 'desire'),                    # 驾驶意图（keepLane/laneChangeLeft 等）
      'useLaneLines': _boolean(latp, 'useLaneLines'),          # 是否使用车道线规划
      'solverExecutionTime': _num(latp, 'solverExecutionTime'),  # MPC 求解耗时（s）
      'solverCost': _num(latp, 'solverCost'),                  # MPC 求解代价
      'latDebugText': str(latp.latDebugText or ''),         # 横向控制调试文本
      'curvatures': list(latp.curvatures),                  # MPC 规划曲率序列（1/m）
      'curvatureRates': list(latp.curvatureRates),          # MPC 规划曲率变化率序列（1/m²）
      'psis': list(latp.psis),                              # MPC 规划横摆角序列（rad）
    }
  except Exception:
    state['lateralPlan'] = {}

  # ---------- carControl ----------
  try:
    cc = sm['carControl']                          # carControl 消息对象（最终控制指令）
    act = cc.actuators if cc.actuators is not None else None                  # 执行器输出子对象
    cc_ctrl = cc.cruiseControl if cc.cruiseControl is not None else None      # 巡航控制子对象
    hud = cc.hudControl if cc.hudControl is not None else None                # HUD 显示子对象
    state['carControl'] = {
      'enabled': _boolean(cc, 'enabled'),                  # 控制是否启用
      'latActive': _boolean(cc, 'latActive'),              # 横向控制是否激活
      'longActive': _boolean(cc, 'longActive'),            # 纵向控制是否激活
      'currentCurvature': _num(cc, 'currentCurvature'),    # 当前实际曲率（1/m）
      'accel': _num(act, 'accel') if act else 0.0,         # 最终纵向加速度指令（m/s²）
      'longControlState': _enum_name(act, 'longControlState') if act else '',  # 纵向控制状态机状态
      'gas': _num(act, 'gas') if act else 0.0,             # 油门指令（0~1）
      'brake': _num(act, 'brake') if act else 0.0,         # 刹车指令（0~1）
      'jerk': _num(act, 'jerk') if act else 0.0,           # 加加速度指令（m/s³）
      'aTarget': _num(act, 'aTarget') if act else 0.0,     # 目标加速度（m/s²）
      'speed': _num(act, 'speed') if act else 0.0,         # 目标速度（m/s）
      'torque': _num(act, 'torque') if act else 0.0,       # 转向扭矩指令（Nm）
      'steeringAngleDeg': _num(act, 'steeringAngleDeg') if act else 0.0,  # 方向盘转角指令（度）
      'curvature': _num(act, 'curvature') if act else 0.0, # 期望曲率（1/m）
      'torqueOutputCan': _num(act, 'torqueOutputCan') if act else 0.0,  # 实际下发的 CAN 转向扭矩
      'ccCancel': _boolean(cc_ctrl, 'cancel') if cc_ctrl else False,    # 巡航取消请求
      'ccResume': _boolean(cc_ctrl, 'resume') if cc_ctrl else False,    # 巡航恢复请求
      'ccOverride': _boolean(cc_ctrl, 'override') if cc_ctrl else False,  # 驾驶员是否踩下踏板接管
      'hudLeadVisible': _boolean(hud, 'leadVisible') if hud else False,    # HUD 是否显示前车
      'hudLeadDistance': _num(hud, 'leadDistance') if hud else 0.0,        # HUD 前车距离（m）
      'hudLeadRelSpeed': _num(hud, 'leadRelSpeed') if hud else 0.0,        # HUD 前车相对速度（m/s）
      'hudSetSpeed': _num(hud, 'setSpeed') if hud else 0.0,                # HUD 设定速度（m/s）
      'hudActiveCarrot': _num(hud, 'activeCarrot') if hud else 0,          # HUD 提示胡萝卜等级
    }
  except Exception:
    state['carControl'] = {}

  # ---------- gpsLocationExternal（经纬度）----------
  try:
    gps = sm['gpsLocationExternal']  # gpsLocationExternal 消息对象（定位数据）
    state['gps'] = {
      'latitude': _num(gps, 'latitude', default=float('nan')),   # 纬度（度）
      'longitude': _num(gps, 'longitude', default=float('nan')),  # 经度（度）
      'altitude': _num(gps, 'altitude'),                          # 海拔（m）
      'speed': _num(gps, 'speed'),                                # 地面速度（m/s）
      'bearing': _num(gps, 'bearing'),                            # 航向角（度，相对北）
      'accuracy': _num(gps, 'accuracy'),                          # 水平定位精度（m）
      'verticalAccuracy': _num(gps, 'verticalAccuracy'),          # 垂直定位精度（m）
      'bearingAccuracy': _num(gps, 'bearingAccuracyDeg'),         # 航向精度（度）
      'hasFix': _boolean(gps, 'hasFix'),                          # 是否已定位
      'satelliteCount': _num(gps, 'satelliteCount'),              # 可见卫星数
      'source': _enum_name(gps, 'source'),                        # 定位源（u-blox/gps 等）
      'timestampMillis': _num(gps, 'unixTimestampMillis'),        # 定位时间戳（毫秒）
    }
  except Exception:
    state['gps'] = None

  # ---------- radarState（radard 输出的聚合前车数据）----------
  try:
    rs = sm['radarState']  # radarState 消息对象（radard 输出的聚合前车数据）

    def lead_to_dict(lead):
      """把单条 LeadData 转成可序列化 dict
      lead: radarState 的 leadOne/Two/Left/Right 子对象（可能为 None）"""
      if lead is None:
        return None
      return {
        'dRel': _num(lead, 'dRel'), 'yRel': _num(lead, 'yRel'), 'vRel': _num(lead, 'vRel'),   # 纵向/横向相对距离（m）、相对速度（m/s）
        'aRel': _num(lead, 'aRel'), 'vLead': _num(lead, 'vLead'), 'dPath': _num(lead, 'dPath'),  # 相对加速度、前车绝对速度（m/s）、到规划路径的距离（m）
        'vLat': _num(lead, 'vLat'), 'vLeadK': _num(lead, 'vLeadK'), 'aLeadK': _num(lead, 'aLeadK'),  # 横向相对速度、卡尔曼滤波的前车速度/加速度
        'fcw': _boolean(lead, 'fcw'), 'status': _boolean(lead, 'status'),  # 前向碰撞预警、前车状态是否有效
        'aLeadTau': _num(lead, 'aLeadTau'), 'modelProb': _num(lead, 'modelProb'),  # 前车加速度估计时间常数、模型置信度
        'radar': _boolean(lead, 'radar'), 'radarTrackId': int(_num(lead, 'radarTrackId')),  # 是否来自雷达、雷达跟踪 ID
        'aLead': _num(lead, 'aLead'), 'jLead': _num(lead, 'jLead'), 'score': _num(lead, 'score'),  # 前车加速度/加加速度、跟踪得分
      }

    state['radarState'] = {
      'leadOne': lead_to_dict(rs.leadOne),
      'leadTwo': lead_to_dict(rs.leadTwo),
      'leadLeft': lead_to_dict(rs.leadLeft),
      'leadRight': lead_to_dict(rs.leadRight),
    }
  except Exception:
    state['radarState'] = {}

  # ---------- liveTracks（Car.RadarData 原始雷达点云，radard 输入）----------
  try:
    rt = sm['liveTracks']       # liveTracks 消息对象（Car.RadarData 原始点云）
    err = rt.errors if rt.errors is not None else None  # 雷达错误标志子对象
    points = []                 # 转换后的点云列表
    for pt in rt.points:        # 遍历每个雷达目标点
      tc = int(_num(pt, 'typeclass'))     # 目标类型码（0~7，对应 TARGET_TYPE_NAMES）
      points.append({
        'trackId': int(_num(pt, 'trackId')),      # 雷达跟踪 ID
        'dRel': _num(pt, 'dRel'),                  # 纵向相对距离（m）
        'yRel': _num(pt, 'yRel'),                  # 横向相对距离（m，左正右负）
        'vRel': _num(pt, 'vRel'),                  # 纵向相对速度（m/s）
        'aRel': _num(pt, 'aRel'),                  # 纵向相对加速度（m/s²）
        'yvRel': _num(pt, 'yvRel'),                # 横向相对速度（m/s）
        'measured': _boolean(pt, 'measured'),      # 是否为实测点（False 为预测点）
        'vLead': _num(pt, 'vLead'),                # 前车绝对速度估计（m/s）
        'aLead': _num(pt, 'aLead'),                # 前车加速度估计（m/s²）
        'jLead': _num(pt, 'jLead'),                # 前车加加速度估计（m/s³）
        'typeclass': tc,                        # 目标类型码（0~7）
        'typeName': TARGET_TYPE_NAMES.get(tc, '未知'),  # 目标类型名称（小车/卡车等）
      })
    state['liveTracks'] = {
      'errors': {
        'canError': _boolean(err, 'canError') if err else False,              # CAN 通讯错误
        'radarFault': _boolean(err, 'radarFault') if err else False,          # 雷达硬件故障
        'wrongConfig': _boolean(err, 'wrongConfig') if err else False,        # 雷达配置错误
        'radarUnavailableTemporary': _boolean(err, 'radarUnavailableTemporary') if err else False,  # 雷达临时不可用
      },
      'pointsCount': len(points),   # 雷达目标点数量
      'points': points,             # 雷达目标点列表
    }
  except Exception:
    state['liveTracks'] = {}

  # ---------- deviceState（设备健康状态：CPU/GPU/内存/温度/网络等）----------
  try:
    ds = sm['deviceState']  # deviceState 消息对象（devicehealthd 发布）
    cpu_t = [_num(x) for x in ds.cpuTempC]              # 各 CPU 核心温度（°C）
    gpu_t = [_num(x) for x in ds.gpuTempC]              # 各 GPU 温度（°C）
    cpu_u = [_num(x) for x in ds.cpuUsagePercent]       # 各 CPU 核心使用率（%）
    zones = [{'name': str(z.name), 'temp': _num(z, 'temp')} for z in ds.thermalZones]  # 热区明细（name/温度°C）
    state['deviceState'] = {
      'deviceType': _enum_name(ds, 'deviceType'),            # 设备类型（tici/pc/mici 等）
      'started': _boolean(ds, 'started'),                    # openpilot 进程是否运行
      'thermalStatus': _enum_name(ds, 'thermalStatus'),      # 热状态（green/yellow/red/danger）
      'cpuUsage': cpu_u,                                  # 各核 CPU 使用率列表（%）
      'gpuUsage': _num(ds, 'gpuUsagePercent'),               # GPU 使用率（%）
      'memoryUsage': _num(ds, 'memoryUsagePercent'),         # 内存使用率（%）
      'freeSpacePercent': _num(ds, 'freeSpacePercent'),      # 存储剩余空间（%）
      'maxTempC': _num(ds, 'maxTempC'),                      # 全机最高温度（°C）
      'cpuTemps': cpu_t,                                  # 各核 CPU 温度列表（°C）
      'gpuTemps': gpu_t,                                  # 各 GPU 温度列表（°C）
      'memoryTempC': _num(ds, 'memoryTempC'),                # 内存温度（°C）
      'fanSpeed': _num(ds, 'fanSpeedPercentDesired'),        # 风扇目标转速（%）
      'screenBrightness': _num(ds, 'screenBrightnessPercent'),  # 屏幕亮度（%）
      'networkType': _enum_name(ds, 'networkType'),          # 网络类型（none/wifi/cell4G 等）
      'networkStrength': _enum_name(ds, 'networkStrength'),  # 网络强度（poor/moderate/good/great）
      'networkMetered': _boolean(ds, 'networkMetered'),      # 是否按流量计费网络
      'powerDrawW': _num(ds, 'powerDrawW'),                  # 整机功耗（W）
      'somPowerDrawW': _num(ds, 'somPowerDrawW'),            # 核心板功耗（W）
      'thermalZones': zones,                              # 热区明细列表
    }
  except Exception:
    state['deviceState'] = {}

  return state


def extract_lat_state(ctl) -> Optional[dict]:
  """提取 controlsState.lateralControlState union 的当前分支数据
  ctl:   controlsState 消息对象（从中读取 lateralControlState）
  lat:   lateralControlState union 对象
  which: 当前激活的分支名（如 torqueState / debugState）
  src:   当前分支的实体对象
  out:   输出的 dict（含 type 分支名与各字段值）
  k:     字段名
  v:     字段原始值"""
  try:
    lat = ctl.lateralControlState
    which = lat.which()
  except Exception:
    return None
  if which is None or which not in LAT_STATE_FIELDS:
    return None
  src = getattr(lat, which)
  out = {'type': which}
  for k in LAT_STATE_FIELDS[which]:
    try:
      v = getattr(src, k)
      out[k] = _num(v, default=0.0) if not isinstance(v, bool) else _boolean(v)
    except Exception:
      out[k] = 0.0
  return out


class Dashboard:
  """持有最新数据，供 SSE 推送"""

  def __init__(self, addr: str, mock: bool):
    self.addr = addr        # 消息总线地址（''=本机，或远程 IP:PORT）
    self.mock = mock        # True=无数据源时用模拟数据演示
    self.lock = threading.Lock()  # 保护 state 的线程锁
    self.state = {}         # 最新提取的数据（后台线程写入，SSE 线程读取）
    self._stop = threading.Event()            # 停止信号（stop() 置位后各循环退出）
    self._start_time = time.time()            # 启动时刻（用于计算运行时长）
    self._thread: Optional[threading.Thread] = None   # 后台数据采集线程
    self._sm: 'Optional[messaging.SubMaster]' = None  # 供调试/外部查询的 SubMaster 引用
    if mock:
      self._mock_thread: Optional[threading.Thread] = None  # （预留）模拟线程引用
    else:
      self._mock_thread = None

  def start(self) -> None:
    # 根据模式选择真实订阅循环或模拟数据循环，并启动后台线程
    if self.mock:
      self._thread = threading.Thread(target=self._mock_loop, daemon=True)
    else:
      self._thread = threading.Thread(target=self._sub_loop, daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()

  # ---- 真实数据：cereal messaging 原接口 ----
  def _sub_loop(self) -> None:
    # 仅订阅本机 cereal 实际存在的服务：不同 openpilot 版本服务名会增删
    # （例如新版已无 lateralPlan，改为 lateralManeuverPlan 等），写死会导致 KeyError 崩溃。
    valid = [svc for svc in SERVICES if svc in messaging.SERVICE_LIST]
    sm = messaging.SubMaster(valid, addr=self.addr)  # 订阅 long/lat 相关服务
    self._sm = sm
    while not self._stop.is_set():
      sm.update(100)                    # 阻塞接收最新一轮消息
      new_state = extract_state(sm)     # 提取成 JSON 可序列化 dict
      with self.lock:
        self.state = new_state          # 加锁写共享状态
      time.sleep(PUSH_INTERVAL)

  # ---- 模拟数据（--mock，便于无数据源时预览页面）----
  def _mock_loop(self) -> None:
    t0 = time.time()       # 模拟起点（用于生成随时间变化的数据）
    services = {s: {'alive': True, 'valid': True, 'freqOk': True, 'recvFrame': 0} for s in SERVICES}
    while not self._stop.is_set():
      t = time.time() - t0   # 距模拟开始的秒数
      v_ego = 15.0 + 3.0 * math.sin(t * 0.05)      # 模拟本车车速 m/s
      a_ego = 0.4 * math.cos(t * 0.05)             # 模拟本车加速度 m/s²
      target_a = 0.5 * math.sin(t * 0.03)          # 模拟目标加速度 m/s²
      steer = 3.0 * math.sin(t * 0.1) + 1.0 * math.sin(t * 0.37)          # 实际转向角 deg
      steer_des = 3.1 * math.sin(t * 0.1 + 0.15) + 1.0 * math.sin(t * 0.37)  # 期望转向角 deg
      curv = 0.0008 * math.sin(t * 0.1)            # 实际曲率 1/m
      d_curv = 0.00085 * math.sin(t * 0.1 + 0.1)   # 期望曲率 1/m

      state = {    # 组装一份完整的模拟数据帧（字段结构与真实提取保持一致）
        'frame': int(t / PUSH_INTERVAL),
        'monotime': time.time(),
        'services': services,
        'carState': {
          'vEgo': v_ego, 'vEgoRaw': v_ego, 'vEgoCluster': v_ego, 'aEgo': a_ego,
          'yawRate': 0.05 * math.sin(t * 0.1),
          'standstill': v_ego < 0.2,
          'gas': 0.1, 'gasPressed': False,
          'brake': 0.0, 'brakePressed': False,
          'vCruise': 20.0, 'vCruiseCluster': 20.0,
          'steeringAngleDeg': steer, 'steeringAngleOffsetDeg': 0.1,
          'steeringRateDeg': 2.0 * math.cos(t * 0.1),
          'steeringTorque': 1.5 * math.sin(t * 0.1),
          'steeringTorqueEps': 1.5 * math.sin(t * 0.1),
          'steeringPressed': False,
          'gearShifter': 'drive',
          'cruiseEnabled': True, 'cruiseSpeed': 20.0, 'cruiseAvailable': True,
          'latEnabled': True,
          'leftBlinker': False, 'rightBlinker': False,
          'doorOpen': False, 'seatbeltUnlatched': False,
          'stockAeb': False, 'stockFcw': False, 'espActive': False,
          'speedLimit': 0.0, 'vCluRatio': 1.0,
          'leftLaneLine': 1, 'rightLaneLine': 1,
          'leftLatDist': 1.6, 'rightLatDist': 1.7,
          'tpms': {'fl': 2.4, 'fr': 2.4, 'rl': 2.3, 'rr': 2.3},
        },
        'controlsState': {
          'longControlState': 'pid' if t % 20 > 3 else 'stopping',
          'upAccelCmd': target_a * 1.2,
          'uiAccelCmd': target_a,
          'ufAccelCmd': target_a * 0.9,
          'curvature': curv,
          'desiredCurvature': d_curv,
          'forceDecel': False,
          'activeLaneLine': True,
          'latState': {
            'type': 'pidState',
            'active': True,
            'steeringAngleDeg': steer,
            'steeringAngleDesiredDeg': steer_des,
            'steeringRateDeg': 2.0 * math.cos(t * 0.1),
            'angleError': steer_des - steer,
            'p': 0.8, 'i': 0.05, 'f': 0.3,
            'output': 0.4 * math.sin(t * 0.1),
            'saturated': False,
          },
        },
        'selfdriveState': {
          'state': 'enabled',
          'enabled': True,
          'active': True,
          'engageable': True,
          'experimentalMode': False,
          'personality': 'sport',
          'alertText1': '模拟数据演示',
          'alertText2': '',
          'alertStatus': 'normal',
          'alertSize': 'none',
          'alertType': '',
          'distanceTraveled': t * 15.0,
        },
        'longitudinalPlan': {
          'hasLead': bool(int(t / 10) % 2),
          'fcw': False,
          'aTarget': target_a,
          'vTargetNow': 19.5 + 1.0 * math.sin(t * 0.05),
          'jTargetNow': 0.1,
          'cruiseTarget': 20.0,
          'desiredDistance': 30.0 + 5.0 * math.sin(t * 0.02),
          'tFollow': 1.8,
          'shouldStop': False,
          'allowThrottle': True,
          'allowBrake': True,
          'source': 'e2e',
          'speeds': [v_ego + 0.1 * i for i in range(50)],
          'accels': [target_a + 0.01 * i for i in range(50)],
          'jerks': [0.05 + 0.001 * i for i in range(50)],
          'solverExecutionTime': 0.006,
          'processingDelay': 0.002,
        },
        'lateralPlan': {
          'mpcSolutionValid': True,
          'laneWidth': 3.6,
          'laneChangeState': 'off',
          'laneChangeDirection': 'none',
          'desire': 'none',
          'useLaneLines': True,
          'solverExecutionTime': 0.008,
          'solverCost': 1.2,
          'latDebugText': '',
          'curvatures': [d_curv + 0.00001 * i for i in range(50)],
          'curvatureRates': [0.00001 + 0.000001 * i for i in range(50)],
          'psis': [0.1 * i for i in range(50)],
        },
        'carControl': {
          'enabled': True,
          'latActive': True,
          'longActive': True,
          'currentCurvature': curv,
          'accel': target_a,
          'longControlState': 'pid',
          'gas': 0.15, 'brake': 0.0,
          'jerk': 0.1, 'aTarget': target_a, 'speed': v_ego,
          'torque': 0.4 * math.sin(t * 0.1),
          'steeringAngleDeg': steer_des,
          'curvature': d_curv,
          'torqueOutputCan': 0.4 * math.sin(t * 0.1),
          'ccCancel': False, 'ccResume': False, 'ccOverride': False,
          'hudLeadVisible': bool(int(t / 10) % 2),
          'hudLeadDistance': 30.0 + 5.0 * math.sin(t * 0.02),
          'hudLeadRelSpeed': 0.0,
          'hudSetSpeed': 20.0,
          'hudActiveCarrot': 0,
        },
        'gps': {
          'latitude': 39.9088 + 0.0001 * math.sin(t * 0.01),
          'longitude': 116.3974 + 0.0001 * math.cos(t * 0.01),
          'altitude': 43.0,
          'speed': v_ego,
          'bearing': 45.0 + 10.0 * math.sin(t * 0.05),
          'accuracy': 3.5,
          'verticalAccuracy': 4.2,
          'bearingAccuracy': 5.0,
          'hasFix': True,
          'satelliteCount': 14,
          'source': 'fusion',
          'timestampMillis': int(time.time() * 1000),
        },
        'radarState': {
          'leadOne': {'dRel': 28.0 + 2.0 * math.sin(t * 0.05), 'yRel': 0.1, 'vRel': -2.0,
                      'aRel': 0.0, 'vLead': 13.0, 'dPath': 28.0, 'vLat': 0.0,
                      'vLeadK': 13.0, 'aLeadK': 0.0, 'fcw': False, 'status': True,
                      'aLeadTau': 1.5, 'modelProb': 0.95, 'radar': True,
                      'radarTrackId': 1001, 'aLead': 0.0, 'jLead': 0.0, 'score': 1.0},
          'leadTwo': {'dRel': 45.0 + 3.0 * math.sin(t * 0.03), 'yRel': -0.2, 'vRel': 1.0,
                      'aRel': 0.1, 'vLead': 16.0, 'dPath': 45.0, 'vLat': 0.0,
                      'vLeadK': 16.0, 'aLeadK': 0.0, 'fcw': False, 'status': True,
                      'aLeadTau': 1.6, 'modelProb': 0.8, 'radar': True,
                      'radarTrackId': 1002, 'aLead': 0.1, 'jLead': 0.0, 'score': 0.8},
          'leadLeft': {'dRel': 38.0, 'yRel': 3.4, 'vRel': -4.0, 'aRel': 0.0,
                       'vLead': 17.0, 'dPath': 38.0, 'vLat': 0.5, 'vLeadK': 17.0,
                       'aLeadK': 0.0, 'fcw': False, 'status': True, 'aLeadTau': 1.4,
                       'modelProb': 0.6, 'radar': True, 'radarTrackId': 1003,
                       'aLead': 0.0, 'jLead': 0.0, 'score': 0.6},
          'leadRight': None,
        },
        'liveTracks': {
          'errors': {'canError': False, 'radarFault': False, 'wrongConfig': False,
                     'radarUnavailableTemporary': False},
          'pointsCount': 6,
          'points': [
            {'trackId': 1001, 'dRel': 28.0 + 2.0 * math.sin(t * 0.05), 'yRel': 0.1,
             'vRel': -2.0, 'aRel': 0.0, 'yvRel': 0.0, 'measured': True,
             'vLead': 13.0, 'aLead': 0.0, 'jLead': 0.0, 'typeclass': 1, 'typeName': '小车'},
            {'trackId': 1002, 'dRel': 45.0 + 3.0 * math.sin(t * 0.03), 'yRel': -0.2,
             'vRel': 1.0, 'aRel': 0.1, 'yvRel': 0.0, 'measured': True,
             'vLead': 16.0, 'aLead': 0.1, 'jLead': 0.0, 'typeclass': 1, 'typeName': '小车'},
            {'trackId': 1003, 'dRel': 38.0, 'yRel': 3.4, 'vRel': -4.0, 'aRel': 0.0,
             'yvRel': 0.5, 'measured': False, 'vLead': 17.0, 'aLead': 0.0,
             'jLead': 0.0, 'typeclass': 2, 'typeName': '卡车'},
            {'trackId': 1004, 'dRel': 52.0, 'yRel': -3.6, 'vRel': 2.5, 'aRel': -0.2,
             'yvRel': 0.0, 'measured': True, 'vLead': 14.0, 'aLead': -0.2,
             'jLead': 0.0, 'typeclass': 6, 'typeName': '宽目标'},
            {'trackId': 1005, 'dRel': 18.5, 'yRel': 0.0, 'vRel': 3.0, 'aRel': 0.0,
             'yvRel': 0.0, 'measured': False, 'vLead': 0.0, 'aLead': 0.0,
             'jLead': 0.0, 'typeclass': 0, 'typeName': '点目标'},
            {'trackId': 1006, 'dRel': 33.0, 'yRel': -1.9, 'vRel': 1.5, 'aRel': 0.0,
             'yvRel': 0.0, 'measured': True, 'vLead': 12.0, 'aLead': 0.0,
             'jLead': 0.0, 'typeclass': 3, 'typeName': '行人'},
          ],
        },
        'deviceState': {
          'deviceType': 'tici',
          'started': True,
          'thermalStatus': 'green',
          'cpuUsage': [38 + 5 * math.sin(t * 0.1 + i) for i in range(8)],
          'gpuUsage': 35 + 5 * math.sin(t * 0.15),
          'memoryUsage': 55 + 3 * math.sin(t * 0.02),
          'freeSpacePercent': 68.0,
          'maxTempC': 62.0 + 2.0 * math.sin(t * 0.03),
          'cpuTemps': [58 + 4 * math.sin(t * 0.03 + i * 0.5) for i in range(8)],
          'gpuTemps': [55 + 3 * math.sin(t * 0.05), 57 + 3 * math.sin(t * 0.05)],
          'memoryTempC': 45.0,
          'fanSpeed': 32,
          'screenBrightness': 70,
          'networkType': 'wifi',
          'networkStrength': 'good',
          'networkMetered': False,
          'powerDrawW': 7.5 + 0.5 * math.sin(t * 0.1),
          'somPowerDrawW': 4.2,
          'thermalZones': [
            {'name': 'msm_therm', 'temp': 62.0 + 2.0 * math.sin(t * 0.03)},
            {'name': 'pm8998_tz', 'temp': 48.0},
            {'name': 'xo_therm', 'temp': 41.0},
          ],
        },
      }
      with self.lock:
        self.state = state     # 加锁写回共享状态
      time.sleep(PUSH_INTERVAL)

  def get_state(self) -> dict:
    """线程安全地读取最新状态"""
    with self.lock:
      return self.state

  def latest_frame(self) -> int:
    """读取最近一次数据帧号（无数据时为 -1）"""
    with self.lock:
      return self.state.get('frame', -1)


class DashboardHandler(BaseHTTPRequestHandler):
  dashboard: Dashboard   # 类级引用，指向共享的 Dashboard 实例（由 serve 时注入）

  def log_message(self, fmt, *args):
    # fmt: 标准访问日志格式串；args: 插值参数
    print(f"[HTTP] {self.client_address[0]} - {fmt % args}")

  def _send_json(self, obj, code=200):
    # obj: 要发送的 Python 对象；code: HTTP 状态码；data: 序列化后的字节
    data = json.dumps(obj).encode()
    self.send_response(code)
    self.send_header('Content-Type', 'application/json')
    self.send_header('Content-Length', str(len(data)))
    self.end_headers()
    self.wfile.write(data)

  def do_GET(self):
    path = self.path.split('?')[0]   # 去掉查询串后的请求路径
    if path == '/' or path == '/index.html':
      self._serve_html()
    elif path == '/health':
      self._send_json({'status': 'ok', 'uptime': time.time() - self.server.start_time,
                       'lastFrame': self.server.dashboard.latest_frame(),
                       'mock': self.server.dashboard.mock,
                       'addr': self.server.dashboard.addr})
    elif path == '/stream':
      self._serve_sse()
    else:
      self._send_json({'error': 'not found'}, 404)

  def _serve_html(self):
    data = HTML_PAGE.encode('utf-8')   # 内嵌页面源码转成 UTF-8 字节
    self.send_response(200)
    self.send_header('Content-Type', 'text/html; charset=utf-8')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('Cache-Control', 'no-cache')
    self.end_headers()
    self.wfile.write(data)

  def _serve_sse(self):
    self.send_response(200)
    self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
    self.send_header('Cache-Control', 'no-cache')
    self.send_header('Connection', 'keep-alive')
    self.send_header('X-Accel-Buffering', 'no')
    self.end_headers()

    last_frame = -1   # 上次推送过的帧号（用于去重）
    heartbeat = 0.0   # 上次发送心跳的时刻
    try:
      while True:
        state = self.server.dashboard.get_state()   # 当前最新状态
        frame = state.get('frame', -1)              # 当前帧号
        if frame != last_frame:                     # 有新的数据帧才推送
          last_frame = frame
          payload = json.dumps(state)               # 序列化为 JSON 字符串
          self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
          self.wfile.flush()
        else:
          now = time.time()                         # 当前时刻
          if now - heartbeat > 5.0:                 # 超过 5s 无数据则发心跳保活
            self.wfile.write(b": keepalive\n\n")
            self.wfile.flush()
            heartbeat = now
          time.sleep(0.01)
    except (BrokenPipeError, ConnectionResetError, OSError):
      pass  # 客户端断开


class DashboardServer(ThreadingHTTPServer):
  daemon_threads = True

  def __init__(self, addr, handler, dashboard):
    super().__init__(addr, handler)
    self.dashboard = dashboard    # 供 Handler 访问的共享数据
    self.start_time = time.time() # 服务器启动时刻


def main():
  parser = argparse.ArgumentParser(description='openpilot Long/Lat 实时数据监控页')  # 命令行参数解析器
  parser.add_argument('--addr', default='127.0.0.1',
                      help='cereal 消息地址（默认 127.0.0.1，可填 comma 设备 IP 远程连接）')
  parser.add_argument('--host', default='0.0.0.0', help='HTTP 绑定地址（默认 0.0.0.0）')
  parser.add_argument('--port', type=int, default=8894, help='HTTP 端口（默认 8894）')
  parser.add_argument('--mock', action='store_true', help='无数据源时用模拟数据演示页面')
  args = parser.parse_args()   # 解析命令行参数

  print("=" * 60)
  print(" openpilot Long/Lat 实时数据监控")
  print(f" 消息源地址 : {args.addr}  {'(模拟数据)' if args.mock else ''}")
  print(f" 页面地址   : http://127.0.0.1:{args.port}")
  print("=" * 60)

  # 缺少 cereal/messaging（未编译 msgq 或未安装 pycapnp）时无法连真实总线，
  # 自动降级为 --mock 模拟模式，让页面照样能预览，而不是一上来就崩溃。
  if messaging is None and not args.mock:
    print('警告：未找到 cereal/messaging（未编译 msgq 或未安装 pycapnp），自动切换到 --mock 模拟模式。')
    args.mock = True

  dashboard = Dashboard(args.addr, args.mock)  # 创建数据源（真实或模拟）
  dashboard.start()                            # 启动后台采集线程

  server = DashboardServer((args.host, args.port), DashboardHandler, dashboard)  # 创建 HTTP 服务
  try:
    server.serve_forever()                     # 阻塞式对外提供服务
  except KeyboardInterrupt:
    print("\n正在退出...")
  finally:
    dashboard.stop()                           # 通知后台线程退出
    server.server_close()                      # 关闭 HTTP 服务


HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>openpilot Long/Lat 实时监控</title>
<style>
  :root {
    --bg: #0d1117;          /* 页面背景色 */
    --panel: #161b22;       /* 卡片面板背景色 */
    --border: #30363d;      /* 边框/分隔线颜色 */
    --text: #e6edf3;        /* 正文文字颜色 */
    --muted: #8b949e;       /* 次要/置灰文字颜色 */
    --green: #3fb950;       /* 正常/OK 状态颜色 */
    --orange: #d29922;      /* 警告/警告状态颜色 */
    --blue: #58a6ff;        /* 强调/链接/自车颜色 */
    --red: #f85149;         /* 错误/FCW 颜色 */
    --purple: #bc8cff;      /* 特殊类型（摩托车/自行车）颜色 */
  }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: "SF Mono", Consolas, "Courier New", monospace;
    padding: 16px;
    min-height: 100vh;
  }
  header {
    display: flex; align-items: center; justify-content: space-between;
    flex-wrap: wrap; gap: 10px;
    padding-bottom: 14px; margin-bottom: 14px;
    border-bottom: 1px solid var(--border);
  }
  .title { font-size: 20px; font-weight: 700; letter-spacing: 1px; }
  .title small { color: var(--muted); font-weight: 400; font-size: 13px; margin-left: 8px; }
  .badges { display: flex; gap: 8px; flex-wrap: wrap; }
  .badge {
    padding: 4px 12px; border-radius: 20px; font-size: 13px;
    border: 1px solid var(--border); background: var(--panel);
  }
  .badge.on  { color: var(--green);  border-color: var(--green); }
  .badge.off { color: var(--muted); }
  .badge.warn{ color: var(--orange); border-color: var(--orange); }
  .badge.err { color: var(--red);    border-color: var(--red); }
  main { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 14px;
  }
  .panel h2 {
    font-size: 14px; color: var(--blue); margin-bottom: 10px;
    display: flex; align-items: center; gap: 8px;
  }
  .panel h2 .tag {
    font-size: 11px; color: var(--muted);
    border: 1px solid var(--border); border-radius: 4px; padding: 1px 6px;
  }
  .grid-cards {
    display: grid; grid-template-columns: repeat(auto-fill, minmax(120px, 1fr));
    gap: 8px; margin-bottom: 10px;
  }
  .card {
    background: var(--bg); border: 1px solid var(--border);
    border-radius: 8px; padding: 8px 10px;
  }
  .card .label { font-size: 11px; color: var(--muted); margin-bottom: 4px; }
  .card .value { font-size: 16px; font-weight: 600; }
  .card .value.small { font-size: 13px; }
  .card .unit { font-size: 11px; color: var(--muted); margin-left: 3px; font-weight: 400; }
  .big-row { display: flex; gap: 10px; margin-bottom: 10px; }
  .big-value {
    flex: 1; background: var(--bg); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px 14px;
  }
  .big-value .num { font-size: 34px; font-weight: 700; color: var(--green); line-height: 1; }
  .big-value .lbl { font-size: 12px; color: var(--muted); margin-top: 4px; }
  .alert-line {
    background: var(--bg); border: 1px dashed var(--border);
    border-radius: 8px; padding: 6px 10px; font-size: 12px; margin-bottom: 10px;
    color: var(--orange); min-height: 20px;
  }
  .chart-box { margin-bottom: 10px; }
  .chart-box:last-child { margin-bottom: 0; }
  .legend { display: flex; gap: 14px; font-size: 11px; color: var(--muted); margin-bottom: 4px; flex-wrap: wrap; }
  .legend span::before {
    content: ""; display: inline-block; width: 10px; height: 3px;
    margin-right: 5px; vertical-align: middle;
  }
  canvas { width: 100%; height: 130px; display: block; background: var(--bg); border-radius: 6px; }
  table { width: 100%; border-collapse: collapse; font-size: 12px; }
  th, td { text-align: left; padding: 4px 8px; border-bottom: 1px solid var(--border); }
  th { color: var(--muted); font-weight: 400; }
  .ok { color: var(--green); }
  .bad { color: var(--red); }
  .panel-full { grid-column: 1 / -1; }
  .lat-params { font-size: 12px; color: var(--muted); background: var(--bg); border-radius: 6px; padding: 8px 10px; margin-top: 10px; }
  .lat-params b { color: var(--text); }
  .conn-dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: var(--red); margin-right: 6px; }
  .conn-dot.live { background: var(--green); }
  .dev-tables { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  @media (max-width: 900px) {
    main { grid-template-columns: 1fr; }
    .dev-tables { grid-template-columns: 1fr; }
  }
</style>
</head>
<body>
<header>
  <div class="title">openpilot <span style="color:var(--green)">LONG</span> / <span style="color:var(--orange)">LAT</span> 实时监控 <small>cereal messaging</small></div>
  <div class="badges">
    <span id="badge-conn" class="badge off"><span id="conn-dot" class="conn-dot"></span>连接中...</span>
    <span id="badge-state" class="badge">--</span>
    <span id="badge-enabled" class="badge off">ENABLED</span>
    <span id="badge-active" class="badge off">ACTIVE</span>
    <span id="badge-long" class="badge off">LONG</span>
    <span id="badge-lat" class="badge off">LAT</span>
  </div>
</header>

<main>
  <!-- ==================== 纵向 LONG ==================== -->
  <section class="panel">
    <h2>纵向 Longitudinal <span class="tag">LONG</span></h2>
    <div class="big-row">
      <div class="big-value">
        <div class="num" id="vEgo-kmh">--</div>
        <div class="lbl">vEgo (km/h)</div>
      </div>
      <div class="big-value">
        <div class="num" id="aEgo-display" style="color:var(--orange)">--</div>
        <div class="lbl">aEgo (m/s²)</div>
      </div>
    </div>
    <div class="grid-cards">
      <div class="card"><div class="label">vEgo (m/s)</div><div class="value" id="vEgo">--</div></div>
      <div class="card"><div class="label">vTargetNow</div><div class="value" id="vTargetNow">--</div></div>
      <div class="card"><div class="label">aTarget</div><div class="value" id="aTarget">--</div></div>
      <div class="card"><div class="label">jTargetNow</div><div class="value" id="jTargetNow">--</div></div>
      <div class="card"><div class="label">uiAccelCmd</div><div class="value" id="uiAccelCmd">--</div></div>
      <div class="card"><div class="label">upAccelCmd</div><div class="value" id="upAccelCmd">--</div></div>
      <div class="card"><div class="label">ufAccelCmd</div><div class="value" id="ufAccelCmd">--</div></div>
      <div class="card"><div class="label">vCruise</div><div class="value" id="vCruise">--</div></div>
      <div class="card"><div class="label">cruiseTarget</div><div class="value" id="cruiseTarget">--</div></div>
      <div class="card"><div class="label">hasLead / fcw</div><div class="value small" id="hasLead">--</div></div>
      <div class="card"><div class="label">desiredDistance (m)</div><div class="value" id="desiredDistance">--</div></div>
      <div class="card"><div class="label">tFollow (s)</div><div class="value" id="tFollow">--</div></div>
      <div class="card"><div class="label">longControlState</div><div class="value small" id="longControlState">--</div></div>
      <div class="card"><div class="label">forceDecel / 油门 / 刹车</div><div class="value small" id="lon-allow">--</div></div>
      <div class="card"><div class="label">LON 求解时间 (ms)</div><div class="value" id="lon-mpc-time">--</div></div>
      <div class="card"><div class="label">处理延迟 (s)</div><div class="value" id="lon-delay">--</div></div>
    </div>
    <div class="alert-line" id="alert-line">--</div>
    <div class="chart-box">
      <div class="legend">
        <span style="color:var(--green)">aEgo</span>
        <span style="color:var(--orange)">aTarget</span>
        <span style="color:var(--blue)">uiAccelCmd</span>
        <span style="color:var(--purple)">upAccelCmd</span>
      </div>
      <canvas id="chart-accel"></canvas>
    </div>
    <div class="chart-box">
      <div class="legend">
        <span style="color:var(--green)">vEgo</span>
        <span style="color:var(--orange)">vTargetNow</span>
        <span style="color:var(--blue)">vCruise</span>
      </div>
      <canvas id="chart-speed"></canvas>
    </div>
  </section>

  <!-- ==================== 横向 LAT ==================== -->
  <section class="panel">
    <h2>横向 Lateral <span class="tag">LAT</span></h2>
    <div class="grid-cards">
      <div class="card"><div class="label">steeringAngleDeg (实际)</div><div class="value" id="steeringAngleDeg">--</div></div>
      <div class="card"><div class="label">期望转向角</div><div class="value" id="steeringAngleDesiredDeg">--</div></div>
      <div class="card"><div class="label">angleError (°)</div><div class="value" id="angleError">--</div></div>
      <div class="card"><div class="label">steeringRateDeg</div><div class="value" id="steeringRateDeg">--</div></div>
      <div class="card"><div class="label">curvature (实际)</div><div class="value" id="curvature">--</div></div>
      <div class="card"><div class="label">desiredCurvature</div><div class="value" id="desiredCurvature">--</div></div>
      <div class="card"><div class="label">steeringTorque</div><div class="value" id="steeringTorque">--</div></div>
      <div class="card"><div class="label">控制器输出</div><div class="value" id="lat-output">--</div></div>
      <div class="card"><div class="label">saturated</div><div class="value" id="lat-saturated">--</div></div>
      <div class="card"><div class="label">MPC valid</div><div class="value" id="mpc-valid">--</div></div>
      <div class="card"><div class="label">换道状态</div><div class="value" id="lane-change">--</div></div>
      <div class="card"><div class="label">MPC 求解时间</div><div class="value small" id="mpc-time">--</div></div>
      <div class="card"><div class="label">laneWidth (m)</div><div class="value" id="laneWidth">--</div></div>
      <div class="card"><div class="label">换道方向</div><div class="value" id="lane-change-dir">--</div></div>
      <div class="card"><div class="label">desire</div><div class="value" id="lat-desire">--</div></div>
      <div class="card"><div class="label">useLaneLines</div><div class="value" id="use-lane-lines">--</div></div>
      <div class="card"><div class="label">solverCost</div><div class="value" id="solver-cost">--</div></div>
    </div>
    <div class="chart-box">
      <div class="legend">
        <span style="color:var(--green)">steeringAngleDeg</span>
        <span style="color:var(--orange)">steeringAngleDesiredDeg</span>
      </div>
      <canvas id="chart-steer"></canvas>
    </div>
    <div class="chart-box">
      <div class="legend">
        <span style="color:var(--green)">curvature</span>
        <span style="color:var(--orange)">desiredCurvature</span>
      </div>
      <canvas id="chart-curv"></canvas>
    </div>
    <div class="lat-params" id="lat-params">横向控制器参数：--</div>
  </section>

  <!-- ==================== GPS 经纬度 ==================== -->
  <section class="panel">
    <h2>GPS 经纬度 <span class="tag">LONG/LAT</span></h2>
    <div class="grid-cards">
      <div class="card"><div class="label">纬度 Latitude</div><div class="value" id="gps-lat">--</div></div>
      <div class="card"><div class="label">经度 Longitude</div><div class="value" id="gps-lon">--</div></div>
      <div class="card"><div class="label">高度 (m)</div><div class="value" id="gps-alt">--</div></div>
      <div class="card"><div class="label">速度 (m/s)</div><div class="value" id="gps-speed">--</div></div>
      <div class="card"><div class="label">航向 (°)</div><div class="value" id="gps-bearing">--</div></div>
      <div class="card"><div class="label">水平精度 (m)</div><div class="value" id="gps-acc">--</div></div>
      <div class="card"><div class="label">垂直精度 (m)</div><div class="value" id="gps-vacc">--</div></div>
      <div class="card"><div class="label">定位精度 (°)</div><div class="value" id="gps-bacc">--</div></div>
      <div class="card"><div class="label">fix / 卫星数</div><div class="value small" id="gps-fix">--</div></div>
      <div class="card"><div class="label">数据源</div><div class="value" id="gps-source">--</div></div>
    </div>
  </section>

  <!-- ==================== 雷达 RADAR ==================== -->
  <section class="panel panel-full">
    <h2>雷达 Radar <span class="tag">radarState / liveTracks</span></h2>
    <div class="grid-cards" style="grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));">
      <div class="card">
        <div class="label">leadOne 主目标</div>
        <div class="value" id="leadOne-d">--</div>
        <div class="label" id="leadOne-info" style="margin-top:4px">--</div>
      </div>
      <div class="card">
        <div class="label">leadTwo 第二目标</div>
        <div class="value" id="leadTwo-d">--</div>
        <div class="label" id="leadTwo-info" style="margin-top:4px">--</div>
      </div>
      <div class="card">
        <div class="label">leadLeft 左侧</div>
        <div class="value" id="leadLeft-d">--</div>
        <div class="label" id="leadLeft-info" style="margin-top:4px">--</div>
      </div>
      <div class="card">
        <div class="label">leadRight 右侧</div>
        <div class="value" id="leadRight-d">--</div>
        <div class="label" id="leadRight-info" style="margin-top:4px">--</div>
      </div>
      <div class="card">
        <div class="label">点云数量</div>
        <div class="value" id="radar-count">--</div>
        <div class="label" id="radar-errors" style="margin-top:4px; color:var(--orange)">--</div>
      </div>
    </div>
    <div class="chart-box">
      <div class="legend">
        <span style="color:var(--green)">● 小车</span>
        <span style="color:var(--orange)">● 卡车</span>
        <span style="color:var(--blue)">● 行人</span>
        <span style="color:var(--purple)">● 摩托车/自行车</span>
        <span style="color:var(--muted)">● 其他</span>
        <span>横向 y(m) × 纵向 d(m)</span>
      </div>
      <canvas id="radar-scatter" style="height:220px;"></canvas>
    </div>
    <div class="chart-box">
      <table>
        <thead><tr><th>trackId</th><th>类型</th><th>dRel(m)</th><th>yRel(m)</th><th>vRel(m/s)</th><th>vLead(m/s)</th><th>aLead(m/s²)</th><th>measured</th></tr></thead>
        <tbody id="radar-points-body"></tbody>
      </table>
    </div>
  </section>

  <!-- ==================== 更多状态 ==================== -->
  <section class="panel panel-full">
    <h2>更多状态 <span class="tag">carState / carControl / cruise / hud</span></h2>
    <div class="grid-cards">
      <div class="card"><div class="label">yawRate (°/s)</div><div class="value" id="yawRate">--</div></div>
      <div class="card"><div class="label">standstill</div><div class="value" id="standstill">--</div></div>
      <div class="card"><div class="label">gas / brake 开度</div><div class="value small" id="pedal">--</div></div>
      <div class="card"><div class="label">steeringTorqueEps</div><div class="value" id="steeringTorqueEps">--</div></div>
      <div class="card"><div class="label">steeringPressed</div><div class="value" id="steeringPressed">--</div></div>
      <div class="card"><div class="label">gearShifter</div><div class="value" id="gearShifter">--</div></div>
      <div class="card"><div class="label">巡航 enabled / 车速</div><div class="value small" id="cruise-state">--</div></div>
      <div class="card"><div class="label">巡航 available</div><div class="value" id="cruise-available">--</div></div>
      <div class="card"><div class="label">latEnabled</div><div class="value" id="latEnabled">--</div></div>
      <div class="card"><div class="label">左/右转向灯</div><div class="value small" id="blinker">--</div></div>
      <div class="card"><div class="label">门 / 安全带</div><div class="value small" id="door-belt">--</div></div>
      <div class="card"><div class="label">stockAeb / Fcw</div><div class="value small" id="stock-safety">--</div></div>
      <div class="card"><div class="label">espActive</div><div class="value" id="espActive">--</div></div>
      <div class="card"><div class="label">speedLimit (km/h)</div><div class="value" id="speedLimit">--</div></div>
      <div class="card"><div class="label">vCluRatio</div><div class="value" id="vCluRatio">--</div></div>
      <div class="card"><div class="label">左/右车道线类型</div><div class="value small" id="lane-line">--</div></div>
      <div class="card"><div class="label">距左/右车道线 (m)</div><div class="value small" id="lane-dist">--</div></div>
      <div class="card"><div class="label">TPMS fl/fr (bar)</div><div class="value small" id="tpms-f">--</div></div>
      <div class="card"><div class="label">TPMS rl/rr (bar)</div><div class="value small" id="tpms-r">--</div></div>
      <div class="card"><div class="label">carControl 执行器</div><div class="value small" id="actuators">--</div></div>
      <div class="card"><div class="label">currentCurvature</div><div class="value" id="currentCurvature">--</div></div>
      <div class="card"><div class="label">巡航 cancel/resume/override</div><div class="value small" id="cc-cmd">--</div></div>
      <div class="card"><div class="label">HUD lead 可见 / 距离</div><div class="value small" id="hud-lead">--</div></div>
      <div class="card"><div class="label">HUD 前车相对速度</div><div class="value" id="hud-lead-rel">--</div></div>
      <div class="card"><div class="label">HUD setSpeed / carrot</div><div class="value small" id="hud-speed">--</div></div>
    </div>
  </section>

  <!-- ==================== 设备状态 DEVICE ==================== -->
  <section class="panel panel-full">
    <h2>设备状态 <span class="tag">deviceState</span></h2>
    <div class="grid-cards">
      <div class="card"><div class="label">设备类型</div><div class="value" id="dev-type">--</div></div>
      <div class="card"><div class="label">openpilot 进程</div><div class="value" id="dev-started">--</div></div>
      <div class="card"><div class="label">热状态 Thermal</div><div class="value" id="dev-thermal">--</div></div>
      <div class="card"><div class="label">网络 类型/强度</div><div class="value small" id="dev-net">--</div></div>
      <div class="card"><div class="label">CPU 平均使用率 (%)</div><div class="value" id="dev-cpu">--</div></div>
      <div class="card"><div class="label">GPU 使用率 (%)</div><div class="value" id="dev-gpu">--</div></div>
      <div class="card"><div class="label">内存使用率 (%)</div><div class="value" id="dev-mem">--</div></div>
      <div class="card"><div class="label">存储剩余 (%)</div><div class="value" id="dev-free">--</div></div>
      <div class="card"><div class="label">最高温度 (°C)</div><div class="value" id="dev-temp">--</div></div>
      <div class="card"><div class="label">风扇 (%)</div><div class="value" id="dev-fan">--</div></div>
      <div class="card"><div class="label">屏幕亮度 (%)</div><div class="value" id="dev-bright">--</div></div>
      <div class="card"><div class="label">整机功耗 (W)</div><div class="value" id="dev-power">--</div></div>
    </div>
    <div class="dev-tables">
      <div class="chart-box">
        <table>
          <thead><tr><th>核心</th><th>CPU 使用率</th><th>CPU 温度</th></tr></thead>
          <tbody id="dev-cores-body"></tbody>
        </table>
      </div>
      <div class="chart-box">
        <table>
          <thead><tr><th>热区 Zone</th><th>温度</th></tr></thead>
          <tbody id="dev-zones-body"></tbody>
        </table>
      </div>
    </div>
  </section>

  <!-- ==================== 消息健康状态 ==================== -->
  <section class="panel panel-full">
    <h2>消息健康状态 <span class="tag">cereal services</span></h2>
    <table>
      <thead><tr><th>服务</th><th>alive</th><th>valid</th><th>freqOK</th><th>接收帧</th></tr></thead>
      <tbody id="health-body"></tbody>
    </table>
  </section>
</main>

<script>
"use strict";
const HISTORY_MS = 30000;  // 曲线历史窗口长度（毫秒）
const hist = {};           // 曲线历史数据缓存: { 数据键: [{t, v}, ...] }

// 各图表的序列配置: [canvasId, [[数据键, 颜色], ...]]
const CHARTS = [
  { canvas: 'chart-accel', series: [['aEgo', '#3fb950'], ['aTarget', '#d29922'], ['uiAccelCmd', '#58a6ff'], ['upAccelCmd', '#bc8cff']] },  // 加速度曲线
  { canvas: 'chart-speed', series: [['vEgo', '#3fb950'], ['vTargetNow', '#d29922'], ['vCruise', '#58a6ff']] },  // 速度曲线
  { canvas: 'chart-steer', series: [['steeringAngleDeg', '#3fb950'], ['steeringAngleDesiredDeg', '#d29922']] },  // 转向角曲线
  { canvas: 'chart-curv', series: [['curvature', '#3fb950'], ['desiredCurvature', '#d29922']] },  // 曲率曲线
];

function pushHist(key, t, v) {
  // key: 数据键名；t: 时间戳；v: 数值；arr: 该键的历史数组
  if (!hist[key]) hist[key] = [];
  const arr = hist[key];
  arr.push({ t, v });
  while (arr.length && t - arr[0].t > HISTORY_MS) arr.shift();  // 裁剪超出窗口的旧点
}

function setText(id, val, digits) {
  // id: 元素 ID；val: 显示值（数字/字符串/无效值）；digits: 保留小数位
  const el = document.getElementById(id);
  if (!el) return;
  if (typeof val === 'string') { el.textContent = val; return; }
  if (typeof val !== 'number' || !isFinite(val)) { el.textContent = '--'; return; }
  el.textContent = digits === undefined ? val : val.toFixed(digits);
}

function setBadge(id, on, text) {
  // id: 徽章元素 ID；on: 是否点亮；text: 显示的文案
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = 'badge ' + (on ? 'on' : 'off');
}

// ---------- 数据提取与前端渲染 ----------
let last = null;   // 最近一次收到的完整数据（供调试）
function onData(d) {
  // d: SSE 推送的完整数据对象
  last = d;
  const t = performance.now();      // 当前页面时间戳（毫秒），用作曲线横轴
  const cs = d.carState || {};      // 车辆原始状态
  const ctl = d.controlsState || {};// 控制器核心数据
  const ss = d.selfdriveState || {};// 驾驶状态/告警
  const lp = d.longitudinalPlan || {}; // 纵向规划
  const latp = d.lateralPlan || {};    // 横向规划
  const cc = d.carControl || {};    // 最终控制指令
  const gps = d.gps;                // GPS 定位（可能为 null）

  // 状态徽章
  const st = ss.state || '--';      // 主状态枚举名（enabled/disabled/...）
  const badgeState = document.getElementById('badge-state');  // 主状态徽章元素
  badgeState.textContent = 'STATE: ' + st.toUpperCase();
  badgeState.className = 'badge ' + (st === 'enabled' ? 'on' : (st === 'disabled' ? 'off' : 'warn'));
  setBadge('badge-enabled', !!ss.enabled, 'ENABLED');   // 系统启用徽章
  setBadge('badge-active', !!ss.active, 'ACTIVE');      // 系统激活徽章
  setBadge('badge-long', !!cc.longActive, 'LONG');      // 纵向控制徽章
  setBadge('badge-lat', !!cc.latActive, 'LAT');         // 横向控制徽章

  // 告警
  const alertEl = document.getElementById('alert-line');  // 告警栏元素
  const alertText = [ss.alertText1, ss.alertText2].filter(Boolean).join(' | ');  // 拼接两条告警文本
  if (alertEl) alertEl.textContent = alertText || '无告警';
  if (alertEl) alertEl.style.color = alertText ? 'var(--orange)' : 'var(--muted)';

  // 纵向数字
  setText('vEgo-kmh', cs.vEgo * 3.6, 1);
  setText('aEgo-display', cs.aEgo, 2);
  setText('vEgo', cs.vEgo, 2);
  setText('vTargetNow', lp.vTargetNow, 2);
  setText('aTarget', lp.aTarget, 3);
  setText('jTargetNow', lp.jTargetNow, 3);
  setText('uiAccelCmd', ctl.uiAccelCmd, 3);
  setText('upAccelCmd', ctl.upAccelCmd, 3);
  setText('ufAccelCmd', ctl.ufAccelCmd, 3);
  setText('vCruise', cs.vCruise, 2);
  setText('cruiseTarget', lp.cruiseTarget, 2);
  setText('desiredDistance', lp.desiredDistance, 1);
  setText('tFollow', lp.tFollow, 2);
  const leadEl = document.getElementById('hasLead');
  if (leadEl) leadEl.textContent = (lp.hasLead ? 'LEAD ' : '') + (lp.fcw ? 'FCW!' : '');
  setText('longControlState', ctl.longControlState || '--');
  const lonAllowEl = document.getElementById('lon-allow');  // 纵向允许状态元素
  if (lonAllowEl) lonAllowEl.textContent =
    (ctl.forceDecel ? 'forceDecel ' : '') + (lp.allowThrottle ? 'THR ' : '') + (lp.allowBrake ? 'BRK' : '');
  setText('lon-mpc-time', lp.solverExecutionTime * 1000, 2);   // MPC 求解耗时（毫秒）
  setText('lon-delay', lp.processingDelay, 4);                 // 处理延迟（秒，异常时显示 --）

  // 横向数字
  const latSt = ctl.latState || {};   // 当前横向控制器分支数据
  setText('steeringAngleDeg', cs.steeringAngleDeg, 2);
  setText('steeringAngleDesiredDeg', latSt.steeringAngleDesiredDeg, 2);
  setText('angleError', latSt.angleError, 3);
  setText('steeringRateDeg', cs.steeringRateDeg, 2);
  setText('curvature', ctl.curvature, 6);
  setText('desiredCurvature', ctl.desiredCurvature, 6);
  setText('steeringTorque', cs.steeringTorque, 3);
  setText('lat-output', latSt.output, 3);
  setText('lat-saturated', latSt.saturated ? 'SAT' : 'OK');
  setText('mpc-valid', latp.mpcSolutionValid ? 'YES' : 'NO');
  setText('lane-change', (latp.laneChangeState || 'off').toUpperCase());
  setText('mpc-time', latp.solverExecutionTime, 3);
  setText('laneWidth', latp.laneWidth, 2);
  setText('lane-change-dir', (latp.laneChangeDirection || 'none').toUpperCase());
  setText('lat-desire', (latp.desire || 'none').toUpperCase());
  setText('use-lane-lines', latp.useLaneLines ? 'YES' : 'NO');
  setText('solver-cost', latp.solverCost, 4);

  // 横向控制器参数明细
  const paramsEl = document.getElementById('lat-params');   // 参数明细元素
  if (paramsEl && latSt && latSt.type) {
    // keys: 允许展示的参数字段名列表；parts: 存在的字段渲染成的 HTML 片段
    const keys = ['p', 'i', 'd', 'f', 'error', 'errorRate', 'actualLateralAccel', 'desiredLateralAccel',
                  'lqrOutput', 'actualCurvature', 'desiredCurvature', 'steeringRateDeg',
                  'rateSetPoint', 'accelSetPoint', 'accelError', 'delta'];
    const parts = keys.filter(k => latSt[k] !== undefined).map(k => `<b>${k}</b>: ${latSt[k]}`);
    paramsEl.innerHTML = `横向控制器 [${latSt.type}]：` + (parts.join(' | ') || '--');
  }

  // GPS
  if (gps) {
    setText('gps-lat', gps.latitude, 6);
    setText('gps-lon', gps.longitude, 6);
    setText('gps-alt', gps.altitude, 1);
    setText('gps-speed', gps.speed, 1);
    setText('gps-bearing', gps.bearing, 1);
    setText('gps-acc', gps.accuracy, 1);
    setText('gps-vacc', gps.verticalAccuracy, 1);
    setText('gps-bacc', gps.bearingAccuracy, 1);
    setText('gps-fix', (gps.hasFix ? 'FIX' : 'NO-FIX') + ' / ' + Math.round(gps.satelliteCount));
    setText('gps-source', gps.source || '--');
  } else {
    ['gps-lat', 'gps-lon', 'gps-alt', 'gps-speed', 'gps-bearing', 'gps-acc',
     'gps-vacc', 'gps-bacc', 'gps-fix', 'gps-source'].forEach(id => setText(id, NaN));
  }

  // 更多状态
  setText('yawRate', cs.yawRate, 3);
  setText('standstill', cs.standstill ? 'YES' : 'NO');
  setText('pedal', (cs.gas * 100).toFixed(0) + ' / ' + (cs.brake * 100).toFixed(0));
  setText('steeringTorqueEps', cs.steeringTorqueEps, 3);
  setText('steeringPressed', cs.steeringPressed ? 'YES' : 'NO');
  setText('gearShifter', cs.gearShifter || '--');
  setText('cruise-state', (cs.cruiseEnabled ? 'ON' : 'OFF') + ' / ' + (cs.cruiseSpeed || 0).toFixed(1));
  setText('cruise-available', cs.cruiseAvailable ? 'YES' : 'NO');
  setText('latEnabled', cs.latEnabled ? 'YES' : 'NO');
  setText('blinker', (cs.leftBlinker ? 'L' : '') + (cs.rightBlinker ? 'R' : '') || '--');
  setText('door-belt', (cs.doorOpen ? 'DOOR!' : '') + (cs.seatbeltUnlatched ? ' BELT!' : '') || 'OK');
  setText('stock-safety', (cs.stockAeb ? 'AEB!' : '') + (cs.stockFcw ? ' FCW!' : '') || '--');
  setText('espActive', cs.espActive ? 'YES' : 'NO');
  setText('speedLimit', cs.speedLimit * 3.6, 1);
  setText('vCluRatio', cs.vCluRatio, 4);
  setText('lane-line', (cs.leftLaneLine || 0) + ' / ' + (cs.rightLaneLine || 0));
  setText('lane-dist', (cs.leftLatDist != null ? cs.leftLatDist.toFixed(2) : '--') + ' / ' +
                      (cs.rightLatDist != null ? cs.rightLatDist.toFixed(2) : '--'));
  if (cs.tpms) {
    setText('tpms-f', cs.tpms.fl.toFixed(2) + ' / ' + cs.tpms.fr.toFixed(2));
    setText('tpms-r', cs.tpms.rl.toFixed(2) + ' / ' + cs.tpms.rr.toFixed(2));
  }
  setText('actuators', 'torque ' + (cc.torque != null ? cc.torque.toFixed(2) : '--') +
                      ' accel ' + (cc.accel != null ? cc.accel.toFixed(2) : '--') +
                      ' gas ' + (cc.gas != null ? cc.gas.toFixed(2) : '--') +
                      ' brk ' + (cc.brake != null ? cc.brake.toFixed(2) : '--') +
                      (cc.longControlState ? ' [' + cc.longControlState + ']' : ''));
  setText('currentCurvature', cc.currentCurvature, 6);
  setText('cc-cmd', (cc.ccCancel ? 'CAN ' : '') + (cc.ccResume ? 'RES ' : '') + (cc.ccOverride ? 'OVR' : '') || '--');
  setText('hud-lead', (cc.hudLeadVisible ? 'VIS ' : '') + (cc.hudLeadDistance ? cc.hudLeadDistance.toFixed(1) + 'm' : '--'));
  setText('hud-lead-rel', cc.hudLeadRelSpeed, 2);
  setText('hud-speed', (cc.hudSetSpeed ? cc.hudSetSpeed.toFixed(1) : '--') + ' / ' + (cc.hudActiveCarrot || 0));

  // 设备状态（deviceState）
  const dev = d.deviceState || {};   // deviceState 数据（设备健康状态）
  setText('dev-type', dev.deviceType || '--');
  const devStartedEl = document.getElementById('dev-started');  // openpilot 进程状态
  if (devStartedEl) {
    devStartedEl.textContent = dev.started ? 'RUNNING' : 'STOPPED';
    devStartedEl.style.color = dev.started ? 'var(--green)' : 'var(--red)';
  }
  const devThermEl = document.getElementById('dev-thermal');    // 热状态
  if (devThermEl) {
    const ts = dev.thermalStatus || 'unknown';
    devThermEl.textContent = ts.toUpperCase();
    devThermEl.style.color = ts === 'green' ? 'var(--green)' :
                             ts === 'yellow' ? 'var(--orange)' :
                             ts === 'red' ? 'var(--red)' : 'var(--muted)';
  }
  setText('dev-net', (dev.networkType || '--') + ' / ' + (dev.networkStrength || '--'));
  const cpuArr = dev.cpuUsage || [];   // 各核 CPU 使用率（%）
  const cpuAvg = cpuArr.length ? cpuArr.reduce((a, b) => a + b, 0) / cpuArr.length : NaN;
  setText('dev-cpu', cpuAvg, 0);
  setText('dev-gpu', dev.gpuUsage, 0);
  setText('dev-mem', dev.memoryUsage, 0);
  setText('dev-free', dev.freeSpacePercent, 0);
  setText('dev-temp', dev.maxTempC, 1);
  setText('dev-fan', dev.fanSpeed, 0);
  setText('dev-bright', dev.screenBrightness, 0);
  setText('dev-power', dev.powerDrawW, 1);
  const coreBody = document.getElementById('dev-cores-body');   // 每核 CPU 表格
  if (coreBody && cpuArr.length) {
    const cpuT = dev.cpuTemps || [];   // 各核 CPU 温度（°C）
    coreBody.innerHTML = cpuArr.map((c, i) =>
      `<tr><td>CPU${i}</td><td>${c.toFixed(0)}%</td><td>${cpuT[i] != null ? cpuT[i].toFixed(1) + '°C' : '--'}</td></tr>`).join('');
  }
  const zoneBody = document.getElementById('dev-zones-body');   // 热区表格
  if (zoneBody && Array.isArray(dev.thermalZones) && dev.thermalZones.length) {
    zoneBody.innerHTML = dev.thermalZones.map(z =>
      `<tr><td>${z.name}</td><td>${z.temp.toFixed(1)}°C</td></tr>`).join('');
  }

  // 雷达（radarState / liveTracks）
  const rs = d.radarState || {};   // radarState 数据（前车聚合结果）
  const rt = d.liveTracks || {};   // liveTracks 数据（原始点云）
  window.__radarPoints = rt.points || [];  // 挂到 window 供 drawRadar 读取
  window.__radarErrors = rt.errors || {};  // 雷达错误标志（预留）

  function renderLead(prefix, lead) {
    // prefix: 目标编号前缀（leadOne/Two/Left/Right）；lead: 单个前车数据
    const dEl = document.getElementById(prefix + '-d');      // 距离显示元素
    const infoEl = document.getElementById(prefix + '-info');// 详细信息元素
    if (!dEl || !infoEl) return;
    if (!lead || !lead.status) {
      dEl.textContent = '--';
      infoEl.textContent = '无目标';
      return;
    }
    dEl.textContent = lead.dRel.toFixed(1) + ' m';
    dEl.style.color = lead.fcw ? 'var(--red)' : 'var(--green)';  // FCW 时标红
    infoEl.textContent =
      'vRel ' + lead.vRel.toFixed(1) + ' | vLead ' + lead.vLead.toFixed(1) +
      ' | yRel ' + lead.yRel.toFixed(1) +
      (lead.radar ? ' | 雷达' : ' | 模型') +
      (lead.fcw ? ' | FCW!' : '');
  }
  renderLead('leadOne', rs.leadOne);
  renderLead('leadTwo', rs.leadTwo);
  renderLead('leadLeft', rs.leadLeft);
  renderLead('leadRight', rs.leadRight);
  setText('radar-count', rt.pointsCount);
  const errEl = document.getElementById('radar-errors');   // 雷达错误信息元素
  if (errEl && rt.errors) {
    // errs: 当前为 true 的错误项名称列表
    const errs = Object.entries(rt.errors).filter(([, v]) => v).map(([k]) => k);
    errEl.textContent = errs.length ? '错误: ' + errs.join(', ') : '无雷达错误';
    errEl.style.color = errs.length ? 'var(--red)' : 'var(--green)';
  }
  const rpBody = document.getElementById('radar-points-body');  // 点云表格 tbody
  if (rpBody && rt.points) {
    // p: 单个点目标；逐行渲染 trackId/类型/距离/速度等
    rpBody.innerHTML = rt.points.map(p =>
      `<tr>
        <td>${p.trackId}</td>
        <td>${p.typeName}</td>
        <td>${p.dRel.toFixed(1)}</td>
        <td>${p.yRel.toFixed(1)}</td>
        <td>${p.vRel.toFixed(1)}</td>
        <td>${p.vLead.toFixed(1)}</td>
        <td>${p.aLead.toFixed(2)}</td>
        <td class="${p.measured ? 'ok' : 'bad'}">${p.measured ? '✓' : '✗'}</td>
      </tr>`).join('');
  }

  // 曲线历史
  pushHist('aEgo', t, cs.aEgo);
  pushHist('aTarget', t, lp.aTarget);
  pushHist('uiAccelCmd', t, ctl.uiAccelCmd);
  pushHist('upAccelCmd', t, ctl.upAccelCmd);
  pushHist('vEgo', t, cs.vEgo);
  pushHist('vTargetNow', t, lp.vTargetNow);
  pushHist('vCruise', t, cs.vCruise);
  pushHist('steeringAngleDeg', t, cs.steeringAngleDeg);
  pushHist('steeringAngleDesiredDeg', t, latSt.steeringAngleDesiredDeg);
  pushHist('curvature', t, ctl.curvature);
  pushHist('desiredCurvature', t, ctl.desiredCurvature);

  // 健康状态表
  const body = document.getElementById('health-body');
  if (body && d.services) {
    body.innerHTML = Object.entries(d.services).map(([name, s]) =>
      `<tr>
        <td>${name}</td>
        <td class="${s.alive ? 'ok' : 'bad'}">${s.alive ? '✓' : '✗'}</td>
        <td class="${s.valid ? 'ok' : 'bad'}">${s.valid ? '✓' : '✗'}</td>
        <td class="${s.freqOk ? 'ok' : 'bad'}">${s.freqOk ? '✓' : '✗'}</td>
        <td>${s.recvFrame}</td>
      </tr>`).join('');
  }
}

// ---------- Canvas 折线图 ----------
function drawChart(cfg) {
  // cfg: 图表配置 { canvas, series }
  const canvas = document.getElementById(cfg.canvas);   // 画布元素
  if (!canvas) return;
  const ctx = canvas.getContext('2d');   // 2D 绘图上下文
  const dpr = window.devicePixelRatio || 1;   // 设备像素比（高分屏适配）
  const w = canvas.clientWidth, h = canvas.clientHeight;   // 画布 CSS 尺寸
  if (!w || !h) return;
  if (canvas.width !== w * dpr) { canvas.width = w * dpr; canvas.height = h * dpr; }  // 按 dpr 设置物理像素
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const now = performance.now();   // 当前时刻（毫秒）
  const t0 = now - HISTORY_MS;     // 时间窗口起点

  // 收集数据与范围
  let minV = Infinity, maxV = -Infinity;   // 所有序列的数值范围
  const all = [];                          // [{key, arr}] 待绘制序列列表
  for (const [key] of cfg.series) {
    const arr = (hist[key] || []).filter(p => p.t >= t0);   // 窗口内的历史点
    all.push({ key, arr });
    for (const p of arr) {   // p: 单个历史点 {t, v}
      if (isFinite(p.v)) { if (p.v < minV) minV = p.v; if (p.v > maxV) maxV = p.v; }
    }
  }
  if (maxV === minV) { maxV += 1; minV -= 1; }   // 避免零范围导致除零
  const pad = (maxV - minV) * 0.1 || 0.1;        // 上下留白
  minV -= pad; maxV += pad;

  // 网格（水平刻度线）
  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1;
  const rows = 4;
  for (let i = 0; i <= rows; i++) {
    const y = (h / rows) * i;   // 当前刻度线 y 坐标
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    const val = maxV - ((maxV - minV) / rows) * i;   // 当前刻度线的数值
    ctx.fillStyle = '#6e7681';
    ctx.font = '10px monospace';
    ctx.fillText(val.toFixed(val < 10 ? 2 : 1), 4, y - 3);
  }

  // 时间网格（竖直刻度线）
  ctx.strokeStyle = '#1f242c';
  for (let i = 1; i < 6; i++) {
    const x = (w / 6) * i;   // 当前竖直刻度线 x 坐标
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }

  // 折线
  for (const { key, arr, color } of all.map((o, i) => ({ ...o, color: cfg.series[i][1] }))) {
    // key: 数据键；arr: 历史点；color: 序列颜色
    if (arr.length < 2) continue;
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    for (let i = 0; i < arr.length; i++) {
      const x = ((arr[i].t - t0) / HISTORY_MS) * w;   // 横坐标（时间映射）
      const y = h - ((arr[i].v - minV) / (maxV - minV)) * h;   // 纵坐标（数值映射）
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

// ---------- SSE 连接 ----------
function connect() {
  // es: SSE 事件源，建立 /stream 长连接接收数据推送
  const es = new EventSource('/stream');
  es.onopen = () => {
    // dot: 连接指示灯元素；badge: 连接状态徽章元素
    const dot = document.getElementById('conn-dot');
    const badge = document.getElementById('badge-conn');
    if (dot) dot.className = 'conn-dot live';
    if (badge) { badge.textContent = '已连接'; badge.className = 'badge on'; }
  };
  es.onmessage = (e) => {
    // e.data: 服务器推送的 JSON 字符串
    try { onData(JSON.parse(e.data)); } catch (_) {}
  };
  es.onerror = () => {
    const dot = document.getElementById('conn-dot');
    const badge = document.getElementById('badge-conn');
    if (dot) dot.className = 'conn-dot';
    if (badge) { badge.textContent = '重连中...'; badge.className = 'badge warn'; }
    es.close();
    setTimeout(connect, 2000);   // 2 秒后自动重连
  };
}

// ---------- 雷达散点图（俯视图） ----------
function drawRadar() {
  const canvas = document.getElementById('radar-scatter');  // 雷达散点图画布
  if (!canvas) return;
  const ctx = canvas.getContext('2d');   // 2D 绘图上下文
  const dpr = window.devicePixelRatio || 1;   // 设备像素比（高分屏适配）
  const w = canvas.clientWidth, h = canvas.clientHeight;   // 画布 CSS 尺寸
  if (!w || !h) return;
  if (canvas.width !== w * dpr) { canvas.width = w * dpr; canvas.height = h * dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const pts = window.__radarPoints || [];   // 雷达目标点数组
  const MAX_D = 100, MAX_Y = 12;            // 纵向/横向显示范围（米）
  const px = w / (MAX_Y * 2), py = h / MAX_D;   // 每米对应的像素数
  const cx = w / 2;                         // 本车横向中心（画布中点）

  // 车道参考线
  ctx.strokeStyle = '#1f242c';
  ctx.setLineDash([4, 4]);
  for (const yOff of [-1.8, 1.8]) {   // yOff: 车道边线的横向偏移（米）
    ctx.beginPath();
    ctx.moveTo(cx + yOff * px, 0); ctx.lineTo(cx + yOff * px, h);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  // 距离刻度
  ctx.fillStyle = '#6e7681';
  ctx.font = '10px monospace';
  for (let d = 20; d <= MAX_D; d += 20) {   // d: 刻度距离（米）
    const y = h - d * py;   // 该距离刻度对应的 y 坐标
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y);
    ctx.strokeStyle = '#21262d'; ctx.stroke();
    ctx.fillText(d + 'm', 4, y - 3);
  }
  // 本车位置（画布底部三角箭头）
  ctx.fillStyle = '#58a6ff';
  ctx.beginPath();
  ctx.moveTo(cx, h - 8); ctx.lineTo(cx - 8, h); ctx.lineTo(cx + 8, h);
  ctx.fill();

  // 目标点（颜色按类型）
  // typeColor: 目标类型名 → 颜色映射
  const typeColor = { '小车': '#3fb950', '卡车': '#d29922', '行人': '#58a6ff',
                      '摩托车': '#bc8cff', '自行车': '#bc8cff', '宽目标': '#f0883e' };
  for (const p of pts) {   // p: 单个雷达目标点
    if (!isFinite(p.dRel) || !isFinite(p.yRel)) continue;
    // 雷达坐标系 y 轴向左为正（左侧 yRel>0），而画布 x 向右为正，故取负以镜像左右
    const x = cx - Math.max(-MAX_Y, Math.min(MAX_Y, p.yRel)) * px;   // 横向像素坐标
    const y = h - Math.max(0, Math.min(MAX_D, p.dRel)) * py;         // 纵向像素坐标
    const c = typeColor[p.typeName] || '#8b949e';   // 该目标点的颜色
    ctx.beginPath();
    ctx.arc(x, y, p.measured ? 5 : 3.5, 0, Math.PI * 2);   // 实测点大、预测点小
    ctx.fillStyle = c;
    ctx.globalAlpha = 0.85;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = '#0d1117';
    ctx.lineWidth = 1.2;
    ctx.stroke();
  }
}

// ---------- 渲染循环 ----------
function renderLoop() {
  for (const cfg of CHARTS) drawChart(cfg);   // cfg: 单个图表配置
  drawRadar();
  requestAnimationFrame(renderLoop);   // 每帧重绘（约 60fps）
}

document.addEventListener('DOMContentLoaded', () => {
  connect();
  renderLoop();
});
</script>
</body>
</html>
"""


if __name__ == "__main__":
  main()
