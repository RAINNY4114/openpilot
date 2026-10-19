#!/usr/bin/env python3
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.stopping_lead_filter import StoppingLeadFilter  # [CHX_AVOID 20261019]
from openpilot.selfdrive.controls.lib.ford_curve_speed import _read_param_bool_direct
import openpilot.cereal.messaging as messaging


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  cloudlog.info("plannerd is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("plannerd got CarParamsSP")

  gps_location_service = get_gps_location_service(params)
  ignore_services = ["liveMapDataSP", "carStateSP", "selfdriveStateSP", gps_location_service]

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP, CP_SP)

  # [CHX_AVOID 20261019] 停车跟停时前车「起步」需要距离证据（carrot-wip 移植）。
  # 停车瞬间雷达测距 5cm 台阶跳变会让 MPC 误判前车起步 -> 自车跟着往前窜。
  stopping_lead_filter = StoppingLeadFilter()
  stopping_lead_tick = 0
  stopping_lead_enable = True
  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'longitudinalPlanSP'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState', 'modelV2', 'selfdriveState',
                            'liveMapDataSP', 'carStateSP', 'selfdriveStateSP', gps_location_service],
                           poll='modelV2', ignore_alive=ignore_services, ignore_avg_freq=ignore_services, ignore_valid=ignore_services)

  while True:
    sm.update()
    longitudinal_planner.sla.update_buttons(sm['selfdriveStateSP'].buttonsReleaseToggle)
    if sm.updated['modelV2']:
      # [CHX_AVOID 20261019] 停车跟停的前车起步证据过滤；开关每 5s 复读一次。
      stopping_lead_tick += 1
      if stopping_lead_tick % 100 == 1:
        stopping_lead_enable = _read_param_bool_direct("dp_stopping_lead_filter", default=True)

      radar_state = sm['radarState']
      if stopping_lead_enable:
        try:
          radar_state = stopping_lead_filter.update(
            radar_state,
            stopping=(CP.openpilotLongitudinalControl
                      and sm['controlsState'].longControlState == car.CarControl.Actuators.LongControlState.stopping
                      and not sm['selfdriveState'].experimentalMode
                      and not sm['carState'].gasPressed),
            v_ego=float(sm['carState'].vEgo),
            mono_time_ns=int(sm.logMonoTime.get('radarState', 0)),
            valid=bool(sm.valid.get('radarState', True) and sm.alive.get('radarState', True)),
          )
        except Exception:
          radar_state = sm['radarState']

      longitudinal_planner.update(sm, radar_state=radar_state)
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks()
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
