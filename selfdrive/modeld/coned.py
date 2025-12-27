#!/usr/bin/env python3
import os
import pickle
import time
from pathlib import Path

from openpilot.system.hardware import TICI
os.environ["DEV"] = "QCOM" if TICI else os.getenv("DEV", "CL")

import cv2
import numpy as np
import cereal.messaging as messaging
from cereal.messaging import PubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from tinygrad.tensor import Tensor

from openpilot.selfdrive.modeld.cone_detections import ConeDet, encode_cone_detections
from openpilot.selfdrive.modeld.cone_detections import ObjDet


PROCESS_NAME = "selfdrive.modeld.coned"

MODEL_PKL_PATH = Path(__file__).parent / "models/Cone_YOLO11n_tinygrad.pkl"
MODEL_INPUT_SIZE = 320

TRAFFIC_CONE_CLASS_IDX = 80  # matches `Cone_YOLO11n.MODEL.md`
PERSON_CLASS_IDX = 0
BICYCLE_CLASS_IDX = 1
CAR_CLASS_IDX = 2
MOTORCYCLE_CLASS_IDX = 3
BUS_CLASS_IDX = 5
TRUCK_CLASS_IDX = 7

# Runtime tuning via env (keeps defaults safe-ish, but still experimental)
CONE_SCORE_MIN = float(os.getenv("CONE_SCORE_MIN", "0.10"))
PERSON_SCORE_MIN = float(os.getenv("PERSON_SCORE_MIN", "0.25"))
VEHICLE_SCORE_MIN = float(os.getenv("VEHICLE_SCORE_MIN", "0.30"))
NMS_IOU_THRES = float(os.getenv("CONE_NMS_IOU", "0.45"))
MAX_DET = int(os.getenv("CONE_MAX_DET", "8"))
PUB_HZ = float(os.getenv("CONE_PUB_HZ", "5"))

ENABLE_PARAMS = ("dp_lat_cone_detection", "dp_lincoln_auto_avoid", "dp_lincoln_hazard_alert")


def _letterbox_rgb(img_rgb: np.ndarray, out_size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
  h, w = img_rgb.shape[:2]
  scale = min(out_size / h, out_size / w)
  new_w = int(round(w * scale))
  new_h = int(round(h * scale))
  resized = cv2.resize(img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

  pad_w = out_size - new_w
  pad_h = out_size - new_h
  pad_left = pad_w // 2
  pad_top = pad_h // 2

  padded = cv2.copyMakeBorder(
    resized,
    pad_top,
    pad_h - pad_top,
    pad_left,
    pad_w - pad_left,
    cv2.BORDER_CONSTANT,
    value=(114, 114, 114),
  )
  return padded, scale, (pad_left, pad_top)


def _nms(boxes_xyxy: np.ndarray, scores: np.ndarray, iou_thres: float, max_det: int) -> list[int]:
  if boxes_xyxy.size == 0:
    return []

  x1 = boxes_xyxy[:, 0]
  y1 = boxes_xyxy[:, 1]
  x2 = boxes_xyxy[:, 2]
  y2 = boxes_xyxy[:, 3]
  areas = (np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)).astype(np.float32)

  order = scores.argsort()[::-1]
  keep: list[int] = []
  while order.size > 0 and len(keep) < max_det:
    i = int(order[0])
    keep.append(i)
    if order.size == 1:
      break

    rest = order[1:]
    xx1 = np.maximum(x1[i], x1[rest])
    yy1 = np.maximum(y1[i], y1[rest])
    xx2 = np.minimum(x2[i], x2[rest])
    yy2 = np.minimum(y2[i], y2[rest])

    w = np.maximum(0.0, xx2 - xx1)
    h = np.maximum(0.0, yy2 - yy1)
    inter = w * h
    iou = inter / (areas[i] + areas[rest] - inter + 1e-6)

    order = rest[iou < iou_thres]

  return keep


def _nv12_to_rgb(buf: VisionBuf) -> np.ndarray:
  h = int(buf.height)
  w = int(buf.width)
  stride = int(buf.stride)
  uv_offset = int(buf.uv_offset)
  data = buf.data  # uint8 view of the full buffer

  if data.size < uv_offset + (h // 2) * stride:
    raise ValueError(f"VisionBuf too small for NV12: len={data.size} uv_offset={uv_offset} stride={stride} h={h}")

  y = data[:uv_offset].reshape((h, stride))[:, :w]
  uv = data[uv_offset:uv_offset + (h // 2) * stride].reshape((h // 2, stride))[:, :w]
  yuv = np.vstack((y, uv))
  return cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_NV12)


def _cones_in_path(cones: list[ConeDet], img_w: int, img_h: int) -> bool:
  if not cones:
    return False
  x_min = img_w * 0.35
  x_max = img_w * 0.65
  y_min = img_h * 0.55
  for c in cones:
    cx = (c.x1 + c.x2) * 0.5
    if x_min <= cx <= x_max and c.y2 >= y_min:
      return True
  return False


def _dets_in_path(dets: list[ConeDet], img_w: int, img_h: int, *, area_min_frac: float) -> bool:
  if not dets:
    return False
  x_min = img_w * 0.35
  x_max = img_w * 0.65
  y_min = img_h * 0.55
  area_min = float(img_w * img_h) * float(area_min_frac)
  for d in dets:
    cx = (d.x1 + d.x2) * 0.5
    area = max(0.0, (d.x2 - d.x1)) * max(0.0, (d.y2 - d.y1))
    if x_min <= cx <= x_max and d.y2 >= y_min and area >= area_min:
      return True
  return False


def _dets_metric(dets: list[ConeDet], img_w: int, img_h: int, *, area_min_frac: float) -> float:
  if not dets:
    return 0.0
  x_min = img_w * 0.35
  x_max = img_w * 0.65
  y_min = img_h * 0.55

  img_area = float(img_w * img_h)
  area_min = img_area * float(area_min_frac)
  # Treat 4x the min-area threshold as "very close" for normalization.
  area_scale = img_area * float(max(area_min_frac, 1e-6)) * 4.0

  metric = 0.0
  for d in dets:
    cx = (d.x1 + d.x2) * 0.5
    if not (x_min <= cx <= x_max and d.y2 >= y_min):
      continue

    area = max(0.0, (d.x2 - d.x1)) * max(0.0, (d.y2 - d.y1))
    if area_min_frac > 0.0 and area < area_min:
      continue

    y_score = float((d.y2 - y_min) / max(1.0, (img_h - y_min)))
    y_score = max(0.0, min(1.0, y_score))

    if area_min_frac <= 0.0:
      metric = max(metric, y_score)
    else:
      area_score = float(area / max(1.0, area_scale))
      area_score = max(0.0, min(1.0, area_score))
      metric = max(metric, 0.6 * y_score + 0.4 * area_score)

  return float(metric)


class _BoolDebouncer:
  def __init__(self, on_cnt: int, off_cnt: int):
    self._on_cnt = int(on_cnt)
    self._off_cnt = int(off_cnt)
    self._on = 0
    self._off = 0
    self.state = False

  def reset(self) -> None:
    self._on = 0
    self._off = 0
    self.state = False

  def update(self, val: bool) -> bool:
    if val:
      self._on += 1
      self._off = 0
    else:
      self._off += 1
      self._on = 0
    if self._on >= self._on_cnt:
      self.state = True
    if self._off >= self._off_cnt:
      self.state = False
    return self.state


def _update_metric_hold(prev: float, new: float, *, decay: float) -> float:
  if new > prev:
    return float(new)
  return float(max(0.0, prev - float(decay)))


class ConeDetector:
  def __init__(self):
    if not MODEL_PKL_PATH.exists():
      raise FileNotFoundError(f"missing model: {MODEL_PKL_PATH}")

    with open(MODEL_PKL_PATH, "rb") as f:
      self._model = pickle.load(f)

  def _detect_class(self, *, boxes_xywh: np.ndarray, class_scores: np.ndarray, score_min: float,
                    scale: float, pad_x: int, pad_y: int, img_w: int, img_h: int) -> list[ConeDet]:
    idxs = np.flatnonzero(class_scores >= score_min)
    if idxs.size == 0:
      return []

    boxes = boxes_xywh[idxs].astype(np.float32)
    scores_sel = class_scores[idxs].astype(np.float32)

    # xywh (center) -> xyxy
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2]
    h = boxes[:, 3]
    boxes_xyxy = np.stack((x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0), axis=1)
    boxes_xyxy = np.clip(boxes_xyxy, 0.0, float(MODEL_INPUT_SIZE))

    keep = _nms(boxes_xyxy, scores_sel, NMS_IOU_THRES, MAX_DET)
    if not keep:
      return []

    boxes_xyxy = boxes_xyxy[keep]
    scores_sel = scores_sel[keep]

    # Undo letterbox
    boxes_xyxy[:, [0, 2]] -= float(pad_x)
    boxes_xyxy[:, [1, 3]] -= float(pad_y)
    boxes_xyxy /= float(scale)

    boxes_xyxy[:, [0, 2]] = np.clip(boxes_xyxy[:, [0, 2]], 0.0, float(img_w))
    boxes_xyxy[:, [1, 3]] = np.clip(boxes_xyxy[:, [1, 3]], 0.0, float(img_h))

    return [
      ConeDet(float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s))
      for b, s in zip(boxes_xyxy, scores_sel)
    ]

  def detect(self, img_rgb: np.ndarray) -> tuple[list[ConeDet], list[ObjDet]]:
    letterboxed, scale, (pad_x, pad_y) = _letterbox_rgb(img_rgb, MODEL_INPUT_SIZE)
    inp = letterboxed.astype(np.float32) / 255.0
    inp = np.transpose(inp, (2, 0, 1))[None, :, :, :]  # BCHW

    out = self._model(images=Tensor(inp, device="NPY")).numpy()[0]  # (85, 2100)
    boxes_xywh = out[:4].T
    h0, w0 = img_rgb.shape[:2]

    cones = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + TRAFFIC_CONE_CLASS_IDX],
      score_min=CONE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    persons = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + PERSON_CLASS_IDX],
      score_min=PERSON_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    bicycles = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + BICYCLE_CLASS_IDX],
      score_min=VEHICLE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    cars = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + CAR_CLASS_IDX],
      score_min=VEHICLE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    motorcycles = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + MOTORCYCLE_CLASS_IDX],
      score_min=VEHICLE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    buses = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + BUS_CLASS_IDX],
      score_min=VEHICLE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    trucks = self._detect_class(
      boxes_xywh=boxes_xywh,
      class_scores=out[4 + TRUCK_CLASS_IDX],
      score_min=VEHICLE_SCORE_MIN,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    objs = [
      *[ObjDet(TRAFFIC_CONE_CLASS_IDX, c.x1, c.y1, c.x2, c.y2, c.score) for c in cones],
      *[ObjDet(PERSON_CLASS_IDX, p.x1, p.y1, p.x2, p.y2, p.score) for p in persons],
      *[ObjDet(BICYCLE_CLASS_IDX, b.x1, b.y1, b.x2, b.y2, b.score) for b in bicycles],
      *[ObjDet(CAR_CLASS_IDX, c.x1, c.y1, c.x2, c.y2, c.score) for c in cars],
      *[ObjDet(MOTORCYCLE_CLASS_IDX, m.x1, m.y1, m.x2, m.y2, m.score) for m in motorcycles],
      *[ObjDet(BUS_CLASS_IDX, b.x1, b.y1, b.x2, b.y2, b.score) for b in buses],
      *[ObjDet(TRUCK_CLASS_IDX, t.x1, t.y1, t.x2, t.y2, t.score) for t in trucks],
    ]
    return cones, objs


def main() -> None:
  config_realtime_process(7, 5)

  params = Params()
  pm = PubMaster(["customReservedRawData0"])

  # camera stream
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      stream = VisionStreamType.VISION_STREAM_ROAD if VisionStreamType.VISION_STREAM_ROAD in available_streams else VisionStreamType.VISION_STREAM_WIDE_ROAD
      break
    time.sleep(0.1)

  vipc = VisionIpcClient("camerad", stream, True)
  while not vipc.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"{PROCESS_NAME} connected to stream={stream} ({vipc.width}x{vipc.height}, stride={vipc.stride}, uv_offset={vipc.uv_offset})")

  detector = ConeDetector()

  # Debounce to reduce flicker (runs at PUB_HZ).
  cone_deb = _BoolDebouncer(on_cnt=2, off_cnt=3)
  person_deb = _BoolDebouncer(on_cnt=2, off_cnt=3)
  vehicle_deb = _BoolDebouncer(on_cnt=2, off_cnt=3)
  cone_metric_hold = 0.0
  person_metric_hold = 0.0
  vehicle_metric_hold = 0.0
  metric_decay_per_pub = float(os.getenv("CONE_METRIC_DECAY", "0.12"))

  next_pub_t = 0.0
  pub_dt = 1.0 / max(PUB_HZ, 0.1)

  while True:
    buf = vipc.recv(timeout_ms=1000)
    if buf is None:
      continue

    now = time.monotonic()
    if now < next_pub_t:
      continue
    next_pub_t = now + pub_dt

    enabled = any(params.get_bool(p) for p in ENABLE_PARAMS)
    cones: list[ConeDet] = []
    objs: list[ObjDet] = []
    cone_in_path_raw = False
    person_in_path_raw = False
    vehicle_in_path_raw = False
    hazard_in_path_raw = False
    cone_in_path = False
    person_in_path = False
    vehicle_in_path = False
    hazard_in_path = False
    cone_metric = 0.0
    person_metric = 0.0
    vehicle_metric = 0.0
    obstacle_metric = 0.0
    hazard_metric = 0.0
    if enabled:
      try:
        img_rgb = _nv12_to_rgb(buf)
        cones, objs = detector.detect(img_rgb)
        cone_in_path_raw = _cones_in_path(cones, img_rgb.shape[1], img_rgb.shape[0])
        # "Hazard" is a conservative heuristic for driver alerting (not used for auto lane changes).
        persons = [ConeDet(o.x1, o.y1, o.x2, o.y2, o.score) for o in objs if o.cls == PERSON_CLASS_IDX]
        vehicles = [ConeDet(o.x1, o.y1, o.x2, o.y2, o.score) for o in objs if o.cls in (BICYCLE_CLASS_IDX, CAR_CLASS_IDX, MOTORCYCLE_CLASS_IDX, BUS_CLASS_IDX, TRUCK_CLASS_IDX)]
        person_in_path_raw = _dets_in_path(persons, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.004)
        vehicle_in_path_raw = _dets_in_path(vehicles, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.010)
        hazard_in_path_raw = person_in_path_raw or vehicle_in_path_raw

        cone_metric = _dets_metric(cones, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.0)
        person_metric = _dets_metric(persons, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.004)
        vehicle_metric = _dets_metric(vehicles, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.010)
      except Exception:
        cloudlog.exception("cone detection failed")
        cones = []
        objs = []
        cone_in_path_raw = False
        person_in_path_raw = False
        vehicle_in_path_raw = False
        hazard_in_path_raw = False
        cone_metric = 0.0
        person_metric = 0.0
        vehicle_metric = 0.0
    else:
      cone_deb.reset()
      person_deb.reset()
      vehicle_deb.reset()
      cone_metric_hold = 0.0
      person_metric_hold = 0.0
      vehicle_metric_hold = 0.0

    # Debounce (2 frames to turn on, 3 frames to turn off)
    cone_in_path = cone_deb.update(cone_in_path_raw)
    person_in_path = person_deb.update(person_in_path_raw)
    vehicle_in_path = vehicle_deb.update(vehicle_in_path_raw)
    hazard_in_path = person_in_path or vehicle_in_path

    cone_metric_hold = _update_metric_hold(cone_metric_hold, cone_metric, decay=metric_decay_per_pub)
    person_metric_hold = _update_metric_hold(person_metric_hold, person_metric, decay=metric_decay_per_pub)
    vehicle_metric_hold = _update_metric_hold(vehicle_metric_hold, vehicle_metric, decay=metric_decay_per_pub)
    if not cone_in_path:
      cone_metric_hold = 0.0
    if not person_in_path:
      person_metric_hold = 0.0
    if not vehicle_in_path:
      vehicle_metric_hold = 0.0
    obstacle_metric = max(cone_metric_hold, vehicle_metric_hold)
    hazard_metric = max(person_metric_hold, vehicle_metric_hold)

    payload = encode_cone_detections(
      frame_id=int(vipc.frame_id),
      timestamp_sof=int(vipc.timestamp_sof),
      img_width=int(buf.width),
      img_height=int(buf.height),
      cones=cones,
      in_path=cone_in_path,
      objects=objs,
      hazard_in_path=hazard_in_path,
      person_in_path=person_in_path,
      vehicle_in_path=vehicle_in_path,
      cone_metric=cone_metric_hold,
      person_metric=person_metric_hold,
      vehicle_metric=vehicle_metric_hold,
      obstacle_metric=obstacle_metric,
      hazard_metric=hazard_metric,
      in_path_raw=cone_in_path_raw,
      person_in_path_raw=person_in_path_raw,
      vehicle_in_path_raw=vehicle_in_path_raw,
      hazard_in_path_raw=hazard_in_path_raw,
    )
    msg = messaging.new_message("customReservedRawData0", valid=True)
    msg.customReservedRawData0 = payload
    pm.send("customReservedRawData0", msg)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
