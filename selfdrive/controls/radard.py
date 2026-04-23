#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

# ✅ 引入场景模块
#from openpilot.selfdrive.controls.lib.scene_understanding import SceneUnderstanding

_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1
V_EGO_STATIONARY = 4.

RADAR_TO_CAMERA = 1.52
LEAD_HOLD_TIME_S = 0.4
LEAD_HOLD_MAX_SPEED = 12.0


# ================= Kalman =================
class KalmanParams:
  def __init__(self, dt: float):
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673,0.14556536,0.16522756,0.18281627,0.1988689,0.21372394,0.22761098,0.24069424,0.253096,0.26491023,
          0.27621103,0.28705801,0.29750003,0.30757767,0.31732515,0.32677158,0.33594201,0.34485814,0.35353899,0.36200124]
    K1 = [0.29666309,0.29330885,0.29042818,0.28787125,0.28555364,0.28342219,0.28144091,0.27958406,0.27783249,0.27617149,
          0.27458948,0.27307714,0.27162685,0.27023228,0.26888809,0.26758976,0.26633338,0.26511557,0.26393339,0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


# ================= Track =================
class Track:
  def __init__(self, identifier, v_lead, kalman_params):
    self.identifier = identifier
    self.cnt = 0
    self.kf = KF1D([[v_lead],[0.0]], kalman_params.A, kalman_params.C, kalman_params.K)

    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    # ===== 新增 =====
    self.in_lane_prob = 0.0
    self.stationary_prob = 0.0
    self.is_stationary = False
    self.priority = 0.0
    self.track_stability = 0.0
    self.scene_type = "normal"
    self.scene_info = {}

  def update(self, d_rel, y_rel, v_rel, v_lead, measured,
             scene_type="normal", scene_info=None):

    self.dRel = d_rel
    self.yRel = y_rel
    self.vRel = v_rel
    self.vLead = v_lead
    self.measured = measured

    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # ===== 基础特征 =====
    self.in_lane_prob = max(0.0, 1.0 - abs(self.yRel)/2.0)

    if abs(self.vLead) < 0.5 and self.dRel < 80:
      self.stationary_prob = min(1.0, self.stationary_prob + 0.1)
    else:
      self.stationary_prob = max(0.0, self.stationary_prob - 0.1)

    self.is_stationary = self.stationary_prob > 0.7
    self.track_stability = min(1.0, self.cnt / 5.0)

    self.priority = (
      (1.0 / max(self.dRel, 1.0)) * 0.5 +
      self.in_lane_prob * 0.3 +
      self.track_stability * 0.2
    )

    # ===== 场景影响 =====
    self.scene_type = scene_type
    self.scene_info = scene_info or {}

    if scene_type == "highway" and self.is_stationary:
      self.priority *= 0.3
    elif scene_type == "curve":
      self.priority *= (1.0 + self.in_lane_prob)
    elif scene_type == "pedestrian_area":
      self.priority *= 1.3

    self.cnt += 1

  def get_RadarState(self, model_prob=0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "status": True,
      "fcw": model_prob > 0.9,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }


# ================= RadarD =================
class RadarD:
  def __init__(self, delay=0.0, CP=None):
    self.CP = CP
    self.tracks = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=10)

    self.scene = SceneUnderstanding()
    self.scene_type = "normal"
    self.scene_info = {}

  def update(self, sm, rr):
    self.scene_type, _, _ = self.scene.update(sm)
    self.scene_info = self.scene.get_scene_info()

    self.v_ego = sm['carState'].vEgo

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids)

    for ids, rpt in ar_pts.items():
      v_lead = rpt[2] + self.v_ego

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)

      self.tracks[ids].update(
        rpt[0], rpt[1], rpt[2], v_lead, rpt[3],
        self.scene_type, self.scene_info
      )

    # ===== 选lead =====
    lead = None
    if self.tracks:
      candidates = list(self.tracks.values())

      # 场景过滤
      if self.scene_type == "highway":
        candidates = [t for t in candidates if not t.is_stationary]
      elif self.scene_type == "curve":
        candidates = [t for t in candidates if t.in_lane_prob > 0.5]
      elif self.scene_type == "pedestrian_area":
        candidates = [t for t in candidates if t.dRel < 40]

      if candidates:
        lead = max(candidates, key=lambda t: t.priority)

    self.radar_state = log.RadarState.new_message()

    if lead:
      self.radar_state.leadOne = lead.get_RadarState(0.5)
    else:
      self.radar_state.leadOne = {"status": False}

  def publish(self, pm):
    msg = messaging.new_message("radarState")
    msg.radarState = self.radar_state
    msg.valid = True
    pm.send("radarState", msg)


# ================= main =================
def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)

  sm = messaging.SubMaster(['modelV2','carState','liveTracks'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay, CP)

  while True:
    sm.update()
    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
