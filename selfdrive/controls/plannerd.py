#!/usr/bin/env python3
import os

from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, DPFlags
import cereal.messaging as messaging


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'liveParameters', 'radarState',
                           'gpsLocationExternal', 'gpsLocation',
                           'customReservedRawData0',
                           'modelV2', 'selfdriveState'],
                           poll='modelV2')

  dp_flags = 0
  def refresh_dp_flags() -> int:
    flags = 0
    if params.get_bool("dp_lon_acm"):
      flags |= DPFlags.ACM
    if params.get_bool("dp_lon_aem"):
      flags |= DPFlags.AEM
    return flags

  dp_flags = refresh_dp_flags()
  last_flag_refresh_frame = 0

  def curve_speed_control_enabled() -> bool:
    return params.get_bool("CurveSpeedControl")

  while True:
    sm.update()
    if sm.updated['modelV2']:
      # Refresh DP flags periodically so UI toggles take effect without restart
      if sm.frame - last_flag_refresh_frame > 100:  # ~0.5s @ 20 Hz model
        dp_flags = refresh_dp_flags()
        last_flag_refresh_frame = sm.frame
      longitudinal_planner.update(sm, dp_flags, curve_speed_control=curve_speed_control_enabled())
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks(['carState', 'carControl', 'modelV2', 'liveParameters'])
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
