#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

from opendbc.can.dbc import DBC
from opendbc import DBC_PATH

ADDR_OBJECT = 0x60B
ADDR_STATUS = 0x60A

_dbc = DBC(os.path.join(DBC_PATH, "u_radar.dbc"))
_status_msg = _dbc.msgs[ADDR_STATUS]
_object_msg = _dbc.msgs[ADDR_OBJECT]

_sig_n_objects = _status_msg.sigs["NoOfObjects"]
_sig_meas_count = _status_msg.sigs["MeasCount"]
_sig_iface_ver = _status_msg.sigs["InterfaceVersion"]

_sig_id = _object_msg.sigs["ID"]
_sig_dist_long = _object_msg.sigs["DistLong"]
_sig_dist_lat = _object_msg.sigs["DistLat"]
_sig_vrel_long = _object_msg.sigs["VRelLong"]
_sig_vrel_lat = _object_msg.sigs["VRelLat"]
_sig_dyn = _object_msg.sigs["DynProp"]
_sig_cls = _object_msg.sigs["Class"]
_sig_rcs = _object_msg.sigs["RCS"]

STATUS_GATE_SEC = 5.0
MAX_OBJECTS_HDR = 50

D_LONG_RANGE = (-0.5, 150.0)
D_LAT_RANGE = (-25.0, 25.0)
VR_RANGE = (-80.0, 80.0)
VLAT_RANGE = (-50.0, 50.0)
VALID_CLASSES = frozenset({0, 1})

OBJECT_TIMEOUT_SEC = 0.50
STATUS_TIMEOUT_SEC = 1.00
MAX_STALE_UPDATES = 10

POINT_CLASS = 0
VEHICLE_CLASS = 1
POINT_HALF_WIDTH = 0.60
HALF_CAR_WIDTH = 0.90
MIN_SAFETY_MARGIN = 0.25
POINT_RELEVANT_Y = HALF_CAR_WIDTH + POINT_HALF_WIDTH + MIN_SAFETY_MARGIN

VLAT_STABLE = 1.0

RCS_FLOOR = -20.0

MIN_HITS = 3

DYN_MOVING = 0
DYN_STATIONARY = 1
DYN_ONCOMING = 2
DYN_CROSSING = (3, 4)
DYN_STOPPED = 6

_DYN_MAP = {
  DYN_MOVING: "unknown",
  DYN_STATIONARY: "static",
  DYN_ONCOMING: "oncoming",
  3: "pedestrian",
  4: "pedestrian",
  5: "unknown",
  DYN_STOPPED: "static",
}

def decode_object(data):
  if data is None or len(data) < 8:
    return None
  d = bytes(data) if not isinstance(data, bytes) else data
  return {
    "id": _sig_id.get_raw_value(d),
    "d": _sig_dist_long.get_raw_value(d) * 0.2 - 500.0,
    "lat": _sig_dist_lat.get_raw_value(d) * 0.2 - 204.6,
    "vr": _sig_vrel_long.get_raw_value(d) * 0.25 - 128.0,
    "vr_lat": _sig_vrel_lat.get_raw_value(d) * 0.25 - 64.0,
    "dyn": _sig_dyn.get_raw_value(d),
    "cls": _sig_cls.get_raw_value(d),
    "rcs": _sig_rcs.get_raw_value(d) * 0.5 - 64.0,
  }

def classify(obj):
  if obj["cls"] == VEHICLE_CLASS:
    return _DYN_MAP.get(obj["dyn"], "unknown")
  dyn = obj["dyn"]
  if dyn in DYN_CROSSING:
    return "pedestrian"
  if dyn == DYN_MOVING:
    return "motorcycle"
  if dyn in (DYN_STATIONARY, DYN_STOPPED):
    return "static"
  return "unknown"

def decode_status(data):
  if data is None or len(data) < 8:
    return None
  d = bytes(data) if not isinstance(data, bytes) else data
  return {
    "n_objects": _sig_n_objects.get_raw_value(d),
    "meas_count": _sig_meas_count.get_raw_value(d),
    "iface_ver": _sig_iface_ver.get_raw_value(d),
  }

def _plausible(obj):
  return (D_LONG_RANGE[0] < obj["d"] < D_LONG_RANGE[1]
          and D_LAT_RANGE[0] < obj["lat"] < D_LAT_RANGE[1]
          and VR_RANGE[0] < obj["vr"] < VR_RANGE[1]
          and VLAT_RANGE[0] < obj["vr_lat"] < VLAT_RANGE[1]
          and obj["cls"] in VALID_CLASSES)

class Mr76Tracker:

  def __init__(self, timeout=0.6, lat_sign=-1.0, max_targets=24,
               require_status=True, trust_class=True, min_hits=MIN_HITS):
    self.timeout = timeout
    self.lat_sign = lat_sign
    self.max_targets = max_targets
    self.require_status = require_status
    self.trust_class = trust_class
    self.min_hits = min_hits
    self._objs = {}
    self._hits = {}
    self.n_objects = 0
    self.frames = 0
    self.object_frames = 0
    self.status_frames = 0
    self.rejected = 0
    self.last_rx = 0.0
    self.last_object_rx = 0.0
    self.last_status_rx = 0.0
    self._last_status = 0.0
    self._stale_updates = 0
    self._saw_batch = False

  def feed(self, src, address, data, t=None):
    t = time.monotonic() if t is None else t

    if address == ADDR_STATUS:
      self._saw_batch = True
      st = decode_status(data)
      if st is None:
        return True
      if 0 <= st["n_objects"] <= MAX_OBJECTS_HDR:
        self.n_objects = st["n_objects"]
        self._last_status = t
        self.last_status_rx = t
        self.last_rx = t
        self.status_frames += 1
      else:
        self.rejected += 1
      return True

    if address != ADDR_OBJECT:
      return False

    self._saw_batch = True
    self.object_frames += 1

    if self.require_status and (t - self._last_status) > STATUS_GATE_SEC:
      self.rejected += 1
      return True

    self.last_rx = t
    self.last_object_rx = t

    obj = decode_object(data)
    if obj is None or not _plausible(obj):
      self.rejected += 1
      return True

    oid = obj["id"]
    self._objs[oid] = (t, obj)
    self._hits[oid] = self._hits.get(oid, 0) + 1
    self.frames += 1
    return True

  def tick(self):
    if self._saw_batch:
      self._saw_batch = False
      self._stale_updates = 0
      return
    self._stale_updates += 1
    if self._stale_updates > MAX_STALE_UPDATES:
      self._objs.clear()
      self._hits.clear()

  def prune(self, t=None):
    t = time.monotonic() if t is None else t
    dead = [k for k, (ts, _) in self._objs.items() if (t - ts) > self.timeout]
    for k in dead:
      self._objs.pop(k, None)
      self._hits.pop(k, None)
    if len(self._objs) > self.max_targets:
      keep = sorted(self._objs.items(), key=lambda kv: kv[1][1]["d"])[:self.max_targets]
      self._objs = dict(keep)
      self._hits = {k: v for k, v in self._hits.items() if k in self._objs}

  def _trustworthy(self, obj, y):
    if obj["rcs"] < RCS_FLOOR:
      return False
    if obj["dyn"] in DYN_CROSSING:
      return True
    if abs(obj["vr_lat"]) > VLAT_STABLE:
      return False
    if not self.trust_class or obj["cls"] == VEHICLE_CLASS:
      return True
    return abs(y) < POINT_RELEVANT_Y

  def targets(self, max_long=40.0):
    self.prune()
    out = []
    for oid, (_, obj) in self._objs.items():
      if self._hits.get(oid, 0) < self.min_hits:
        continue
      d = obj["d"]
      if not (0.5 < d < max_long):
        continue
      y = obj["lat"] * self.lat_sign
      if abs(y) > 3.2:
        continue
      if not self._trustworthy(obj, y):
        continue
      out.append((d, y, obj["vr"], classify(obj)))
    return out

  def fresh(self, now=None):
    now = time.monotonic() if now is None else now
    self.prune(now)
    if self._objs:
      newest = max(ts for ts, _ in self._objs.values())
      return (now - newest) <= OBJECT_TIMEOUT_SEC
    return (self.last_object_rx > 0.0
            and (now - self.last_object_rx) <= OBJECT_TIMEOUT_SEC
            and self.last_status_rx > 0.0
            and (now - self.last_status_rx) <= STATUS_TIMEOUT_SEC)

  def is_stale(self):
    return not self.fresh()

  def class_hist(self):
    ch, dh = {}, {}
    for _, obj in self._objs.values():
      ch[obj["cls"]] = ch.get(obj["cls"], 0) + 1
      dh[obj["dyn"]] = dh.get(obj["dyn"], 0) + 1
    return ch, dh

  def status_dict(self):
    ch, dh = self.class_hist()
    return {
      "fresh": self.fresh(),
      "objects": len(self._objs),
      "frames": self.frames,
      "object_frames": self.object_frames,
      "status_frames": self.status_frames,
      "rejected": self.rejected,
      "n_obj_hdr": self.n_objects,
      "cls": ch,
      "dyn": dh,
    }
