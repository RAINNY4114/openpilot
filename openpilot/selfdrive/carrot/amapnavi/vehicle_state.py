#!/usr/bin/env python3
"""车辆状态获取。

**解耦说明**：原实现由外部（cpv9-dev 的 ``CarrotMan``）每帧调用
``AmapNaviServ.update_navi_carstate(sm)`` 把 ``carState`` 喂进来，
这样 amapnavi 就强依赖了项目里既有的 ``CarrotMan`` 代码。

这里改成：amapnavi 自己订阅 ``carState``（``AmapNaviServ`` 内部的
``SubMaster`` 已包含该服务），只依赖 cereal 消息定义，不依赖任何
cp / carrot 的既有业务代码。外界也可以通过 ``apply_carstate()``
手动注入（便于单元测试或复用已有的 SubMaster）。
"""

from openpilot.selfdrive.carrot.amapnavi.shared_state import f1

# 订阅车辆状态所需的服务（只依赖 cereal，不依赖项目其它模块）
REQUIRED_SERVICES = ("carState",)


def apply_carstate(shared_data, carState) -> None:
  """把一帧 carState 展开到共享数据中（字段缺失时跳过）。"""
  shared_data.carState = True

  if hasattr(carState, 'standstill'):
    shared_data.standstill = carState.standstill
  else:
    shared_data.standstill = (shared_data.v_ego_m or 0.0) < 0.1

  if hasattr(carState, 'vEgoCluster'):
    shared_data.v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
  if hasattr(carState, 'vCruise'):
    shared_data.v_cruise_kph = carState.vCruise
  if hasattr(carState, 'vEgo'):
    shared_data.v_ego_m = carState.vEgo
    shared_data.vEgo = f1(carState.vEgo * 3.6)
  if hasattr(carState, 'aEgo'):
    shared_data.aEgo = round(carState.aEgo, 1)
  if hasattr(carState, 'steeringAngleDeg'):
    shared_data.steer_angle = round(carState.steeringAngleDeg, 1)
  if hasattr(carState, 'gasPressed'):
    shared_data.gas_press = carState.gasPressed
  if hasattr(carState, 'brakePressed'):
    shared_data.break_press = carState.brakePressed
  if hasattr(carState, 'cruiseState'):
    shared_data.engaged = carState.cruiseState.enabled
    shared_data.cruise_valid = carState.cruiseState.available
    shared_data.cruise_enable = carState.cruiseState.enabled
  # 原车盲区检测
  if hasattr(carState, 'leftBlindspot'):
    shared_data.left_blindspot = int(carState.leftBlindspot)
  if hasattr(carState, 'rightBlindspot'):
    shared_data.right_blindspot = int(carState.rightBlindspot)


def update_from_submaster(shared_data, sm) -> bool:
  """从自有 SubMaster 更新车辆状态。

  :return: 本次是否取到了有效帧。
  """
  if not sm.alive['carState']:
    shared_data.carState = False
    return False
  apply_carstate(shared_data, sm['carState'])
  return True
