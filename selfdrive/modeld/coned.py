#!/usr/bin/env python3
import os
import math
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
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
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

# Lane-change safety: treat these classes as "vehicles" for adjacent-lane occupancy.
LC_VEHICLE_CLASS_IDS = {BICYCLE_CLASS_IDX, MOTORCYCLE_CLASS_IDX, CAR_CLASS_IDX, BUS_CLASS_IDX, TRUCK_CLASS_IDX}

# Runtime tuning via env (keeps defaults safe-ish, but still experimental)
CONE_SCORE_MIN = float(os.getenv("CONE_SCORE_MIN", "0.10"))
PERSON_SCORE_MIN = float(os.getenv("PERSON_SCORE_MIN", "0.25"))
# Slightly lower default to improve recall (safer for auto lane-change gating; may increase false positives).
VEHICLE_SCORE_MIN = float(os.getenv("VEHICLE_SCORE_MIN", "0.25"))
OTHER_SCORE_MIN = float(os.getenv("CONE_OTHER_SCORE_MIN", "0.25"))
PERSON_AREA_MIN_FRAC = float(os.getenv("CONE_PERSON_AREA_MIN_FRAC", "0.004"))
VEHICLE_AREA_MIN_FRAC = float(os.getenv("CONE_VEHICLE_AREA_MIN_FRAC", "0.010"))
TWOWHEEL_AREA_MIN_FRAC = float(os.getenv("CONE_TWOWHEEL_AREA_MIN_FRAC", "0.006"))
NMS_IOU_THRES = float(os.getenv("CONE_NMS_IOU", "0.45"))
MAX_DET = int(os.getenv("CONE_MAX_DET", "64"))
PUB_HZ = float(os.getenv("CONE_PUB_HZ", "10"))

# Optional edge-based bbox refinement (for visualization).
# This refines YOLO boxes by running a cheap edge pass inside the ROI and taking the
# best contour's bounding rect. It's not true instance segmentation; use as a best-effort.
REFINE_EDGES = bool(int(os.getenv("CONE_REFINE_EDGES", "1")))
REFINE_MAX_SIDE = int(os.getenv("CONE_REFINE_MAX_SIDE", "320"))
REFINE_PAD_FRAC = float(os.getenv("CONE_REFINE_PAD_FRAC", "0.08"))
REFINE_MIN_AREA_FRAC = float(os.getenv("CONE_REFINE_MIN_AREA_FRAC", "0.02"))
REFINE_IOU_MIN = float(os.getenv("CONE_REFINE_IOU_MIN", "0.25"))
REFINE_MAX_OBJS = int(os.getenv("CONE_REFINE_MAX_OBJS", "32"))

# Target-lane forward occupancy (for auto lane-change safety).
# Distance is estimated from bbox height using a simple pinhole approximation.
LC_ASSUMED_VEHICLE_HEIGHT_M = float(os.getenv("CONE_LC_ASSUMED_VEHICLE_HEIGHT_M", "1.5"))
LC_ASSUMED_PERSON_HEIGHT_M = float(os.getenv("CONE_LC_ASSUMED_PERSON_HEIGHT_M", "1.7"))
LC_ASSUMED_CONE_HEIGHT_M = float(os.getenv("CONE_LC_ASSUMED_CONE_HEIGHT_M", "0.7"))
LC_CONE_SCORE_MIN = float(os.getenv("CONE_LC_CONE_SCORE_MIN", "0.25"))
LC_LANE_Y_MIN_FRAC = float(os.getenv("CONE_LC_LANE_Y_MIN_FRAC", "0.20"))
LC_LEFT_LANE_X_MIN_FRAC = float(os.getenv("CONE_LC_LEFT_LANE_X_MIN_FRAC", "0.05"))
LC_LEFT_LANE_X_MAX_FRAC = float(os.getenv("CONE_LC_LEFT_LANE_X_MAX_FRAC", "0.40"))
LC_RIGHT_LANE_X_MIN_FRAC = float(os.getenv("CONE_LC_RIGHT_LANE_X_MIN_FRAC", "0.60"))
LC_RIGHT_LANE_X_MAX_FRAC = float(os.getenv("CONE_LC_RIGHT_LANE_X_MAX_FRAC", "0.95"))


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


def _focal_length_candidates(img_w: int, img_h: int) -> list[float]:
  focals: set[float] = set()
  for cfg in DEVICE_CAMERAS.values():
    for _, cam in cfg.all_cams():
      try:
        if int(cam.width) == int(img_w) and int(cam.height) == int(img_h) and float(cam.focal_length) > 1.0:
          focals.add(float(cam.focal_length))
      except Exception:
        continue
  return sorted(focals)


def _focal_length_for_stream(img_w: int, img_h: int, stream: VisionStreamType) -> float:
  focals = _focal_length_candidates(img_w, img_h)
  if not focals:
    # Last-resort fallback; only used for very rough distance estimates.
    return float(max(img_w, img_h))
  if stream == VisionStreamType.VISION_STREAM_WIDE_ROAD:
    return float(min(focals))
  return float(max(focals))


def _lane_min_distance_m(objs: list[ObjDet], img_w: int, img_h: int, *, x_min_frac: float, x_max_frac: float, y_min_frac: float,
                         class_ids: set[int], score_min: float, focal_length_px: float, obj_height_m: float) -> float:
  if not objs or focal_length_px <= 1.0 or obj_height_m <= 0.1:
    return 0.0

  x_min = float(img_w) * float(x_min_frac)
  x_max = float(img_w) * float(x_max_frac)
  y_min = float(img_h) * float(y_min_frac)

  min_dist = float("inf")
  for o in objs:
    if int(o.cls) not in class_ids:
      continue
    if float(o.score) < float(score_min):
      continue

    cx = (float(o.x1) + float(o.x2)) * 0.5
    if not (x_min <= cx <= x_max):
      continue
    if float(o.y2) < y_min:
      continue

    h_px = max(1.0, float(o.y2) - float(o.y1))
    dist_m = (float(focal_length_px) * float(obj_height_m)) / h_px
    if dist_m < min_dist:
      min_dist = dist_m

  if not math.isfinite(min_dist) or min_dist <= 0.0:
    return 0.0
  # Clamp to a reasonable range; consumers treat 0 as "no data / no object".
  return float(min(300.0, max(0.0, min_dist)))


def _bbox_iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
  ax1, ay1, ax2, ay2 = a
  bx1, by1, bx2, by2 = b
  ix1 = max(ax1, bx1)
  iy1 = max(ay1, by1)
  ix2 = min(ax2, bx2)
  iy2 = min(ay2, by2)
  iw = max(0.0, ix2 - ix1)
  ih = max(0.0, iy2 - iy1)
  inter = iw * ih
  if inter <= 0.0:
    return 0.0
  area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
  area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
  union = area_a + area_b - inter
  if union <= 0.0:
    return 0.0
  return float(inter / union)


def _refine_bbox_edges(img_rgb: np.ndarray, x1: float, y1: float, x2: float, y2: float) -> tuple[float, float, float, float] | None:
  h, w = img_rgb.shape[:2]
  if w <= 0 or h <= 0:
    return None

  x1 = float(max(0.0, min(float(w), x1)))
  y1 = float(max(0.0, min(float(h), y1)))
  x2 = float(max(0.0, min(float(w), x2)))
  y2 = float(max(0.0, min(float(h), y2)))
  if x2 <= x1 or y2 <= y1:
    return None

  bw = x2 - x1
  bh = y2 - y1
  pad = int(max(4.0, REFINE_PAD_FRAC * float(max(bw, bh))))
  rx1 = max(0, int(math.floor(x1)) - pad)
  ry1 = max(0, int(math.floor(y1)) - pad)
  rx2 = min(w, int(math.ceil(x2)) + pad)
  ry2 = min(h, int(math.ceil(y2)) + pad)
  if rx2 - rx1 < 8 or ry2 - ry1 < 8:
    return None

  roi = img_rgb[ry1:ry2, rx1:rx2]
  roi_h, roi_w = roi.shape[:2]

  # Downsample large ROIs for speed, keep aspect ratio.
  scale = 1.0
  max_side = float(max(roi_w, roi_h))
  if REFINE_MAX_SIDE > 0 and max_side > float(REFINE_MAX_SIDE):
    scale = float(REFINE_MAX_SIDE) / max_side
    new_w = max(8, int(round(roi_w * scale)))
    new_h = max(8, int(round(roi_h * scale)))
    roi = cv2.resize(roi, (new_w, new_h), interpolation=cv2.INTER_AREA)
    roi_h, roi_w = roi.shape[:2]

  gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
  gray = cv2.GaussianBlur(gray, (5, 5), 0)

  v = float(np.median(gray))
  sigma = 0.33
  lower = int(max(0.0, (1.0 - sigma) * v))
  upper = int(min(255.0, (1.0 + sigma) * v))
  edges = cv2.Canny(gray, lower, upper)

  # Close gaps a bit.
  k = np.ones((3, 3), np.uint8)
  edges = cv2.dilate(edges, k, iterations=1)
  edges = cv2.erode(edges, k, iterations=1)

  cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  if not cnts:
    return None

  roi_area = float(roi_w * roi_h)
  min_area = roi_area * float(max(0.0, REFINE_MIN_AREA_FRAC))
  cx0 = roi_w * 0.5
  cy0 = roi_h * 0.5
  best_score = -1.0
  best_rect: tuple[int, int, int, int] | None = None
  for c in cnts:
    x, y, cw, ch = cv2.boundingRect(c)
    if cw <= 1 or ch <= 1:
      continue
    area = float(cw * ch)
    if area < min_area:
      continue

    cx = x + cw * 0.5
    cy = y + ch * 0.5
    dist = math.hypot(cx - cx0, cy - cy0) / max(1.0, float(max(roi_w, roi_h)))
    border = (x <= 0) or (y <= 0) or (x + cw >= roi_w - 1) or (y + ch >= roi_h - 1)
    border_pen = 0.75 if border else 1.0
    score = area * (1.0 - min(0.9, dist)) * border_pen
    if score > best_score:
      best_score = score
      best_rect = (x, y, cw, ch)

  if best_rect is None:
    return None

  x, y, cw, ch = best_rect
  # Scale back to original ROI coordinates.
  if scale != 1.0:
    inv = 1.0 / scale
    x = int(round(x * inv))
    y = int(round(y * inv))
    cw = int(round(cw * inv))
    ch = int(round(ch * inv))

  gx1 = float(rx1 + x)
  gy1 = float(ry1 + y)
  gx2 = float(rx1 + x + cw)
  gy2 = float(ry1 + y + ch)

  gx1 = float(max(0.0, min(float(w), gx1)))
  gy1 = float(max(0.0, min(float(h), gy1)))
  gx2 = float(max(0.0, min(float(w), gx2)))
  gy2 = float(max(0.0, min(float(h), gy2)))
  if gx2 <= gx1 or gy2 <= gy1:
    return None

  iou = _bbox_iou((x1, y1, x2, y2), (gx1, gy1, gx2, gy2))
  if iou < REFINE_IOU_MIN:
    return None

  # Small padding to avoid cutting off the object.
  pad2 = int(max(2.0, 0.02 * float(max(bw, bh))))
  gx1 = float(max(0.0, gx1 - pad2))
  gy1 = float(max(0.0, gy1 - pad2))
  gx2 = float(min(float(w), gx2 + pad2))
  gy2 = float(min(float(h), gy2 + pad2))
  return gx1, gy1, gx2, gy2


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
     for b, s in zip(boxes_xyxy, scores_sel, strict=True)
    ]

  def _detect_all_objects(self, *, boxes_xywh: np.ndarray, class_scores: np.ndarray,
                          scale: float, pad_x: int, pad_y: int, img_w: int, img_h: int) -> list[ObjDet]:
    # `class_scores` expected shape: (N, C), with C including `traffic cone (80)`.
    if class_scores.ndim != 2 or class_scores.shape[0] != boxes_xywh.shape[0]:
      return []

    cls = np.argmax(class_scores, axis=1).astype(np.int32)
    score = class_scores[np.arange(class_scores.shape[0]), cls].astype(np.float32)

    # Per-class minimum score thresholds (vectorized for common driving classes).
    score_min = np.full_like(score, float(OTHER_SCORE_MIN), dtype=np.float32)
    score_min[cls == int(TRAFFIC_CONE_CLASS_IDX)] = float(CONE_SCORE_MIN)
    score_min[cls == int(PERSON_CLASS_IDX)] = float(PERSON_SCORE_MIN)
    if LC_VEHICLE_CLASS_IDS:
      score_min[np.isin(cls, list(LC_VEHICLE_CLASS_IDS))] = float(VEHICLE_SCORE_MIN)

    idxs = np.flatnonzero(score >= score_min)
    if idxs.size == 0:
      return []

    boxes = boxes_xywh[idxs].astype(np.float32)
    scores_sel = score[idxs].astype(np.float32)
    cls_sel = cls[idxs].astype(np.int32)

    # xywh (center) -> xyxy in MODEL_INPUT_SIZE coordinates
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2]
    h = boxes[:, 3]
    boxes_xyxy = np.stack((x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0), axis=1)
    boxes_xyxy = np.clip(boxes_xyxy, 0.0, float(MODEL_INPUT_SIZE))

    # Per-class NMS, then keep top-N across all classes.
    keep_all: list[int] = []
    for c in np.unique(cls_sel):
      cls_mask = np.flatnonzero(cls_sel == c)
      if cls_mask.size == 0:
        continue
      keep_local = _nms(boxes_xyxy[cls_mask], scores_sel[cls_mask], NMS_IOU_THRES, MAX_DET)
      keep_all.extend(int(cls_mask[k]) for k in keep_local)

    if not keep_all:
      return []

    keep_all.sort(key=lambda i: float(scores_sel[i]), reverse=True)
    keep_all = keep_all[:MAX_DET]

    boxes_xyxy = boxes_xyxy[keep_all]
    scores_sel = scores_sel[keep_all]
    cls_sel = cls_sel[keep_all]

    # Undo letterbox
    boxes_xyxy[:, [0, 2]] -= float(pad_x)
    boxes_xyxy[:, [1, 3]] -= float(pad_y)
    boxes_xyxy /= float(scale)

    boxes_xyxy[:, [0, 2]] = np.clip(boxes_xyxy[:, [0, 2]], 0.0, float(img_w))
    boxes_xyxy[:, [1, 3]] = np.clip(boxes_xyxy[:, [1, 3]], 0.0, float(img_h))

    out: list[ObjDet] = []
    for b, s, c in zip(boxes_xyxy, scores_sel, cls_sel, strict=True):
      out.append(ObjDet(int(c), float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s)))
    return out

  def detect(self, img_rgb: np.ndarray) -> tuple[list[ConeDet], list[ObjDet]]:
    letterboxed, scale, (pad_x, pad_y) = _letterbox_rgb(img_rgb, MODEL_INPUT_SIZE)
    inp = letterboxed.astype(np.float32) / 255.0
    inp = np.transpose(inp, (2, 0, 1))[None, :, :, :]  # BCHW

    out = self._model(images=Tensor(inp, device="NPY")).numpy()[0]  # (85, 2100)
    boxes_xywh = out[:4].T
    all_scores = out[4:].T
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

    objs = self._detect_all_objects(
      boxes_xywh=boxes_xywh,
      class_scores=all_scores,
      scale=scale,
      pad_x=pad_x,
      pad_y=pad_y,
      img_w=w0,
      img_h=h0,
    )

    # Ensure cone detections (low threshold) are visible in the HUD box stream too.
    if cones:
      existing_cones = [o for o in objs if int(o.cls) == TRAFFIC_CONE_CLASS_IDX]
      for c in cones:
        if any(_bbox_iou((o.x1, o.y1, o.x2, o.y2), (c.x1, c.y1, c.x2, c.y2)) >= 0.60 for o in existing_cones):
          continue
        objs.append(ObjDet(TRAFFIC_CONE_CLASS_IDX, c.x1, c.y1, c.x2, c.y2, c.score))
    return cones, objs


def main() -> None:
  config_realtime_process(7, 5)

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
  focal_length_px = _focal_length_for_stream(int(vipc.width), int(vipc.height), stream)

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

    img_rgb: np.ndarray | None = None
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
    try:
      img_rgb = _nv12_to_rgb(buf)
      cones, objs = detector.detect(img_rgb)
      cone_in_path_raw = _cones_in_path(cones, img_rgb.shape[1], img_rgb.shape[0])
      # "Hazard" is a conservative heuristic for driver alerting (not used for auto lane changes).
      persons = [ConeDet(o.x1, o.y1, o.x2, o.y2, o.score) for o in objs if o.cls == PERSON_CLASS_IDX]
      two_wheel = [ConeDet(o.x1, o.y1, o.x2, o.y2, o.score) for o in objs if o.cls in (BICYCLE_CLASS_IDX, MOTORCYCLE_CLASS_IDX)]
      four_wheel = [ConeDet(o.x1, o.y1, o.x2, o.y2, o.score) for o in objs if o.cls in (CAR_CLASS_IDX, BUS_CLASS_IDX, TRUCK_CLASS_IDX)]

      person_in_path_raw = _dets_in_path(persons, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=PERSON_AREA_MIN_FRAC)
      two_wheel_in_path_raw = _dets_in_path(two_wheel, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=TWOWHEEL_AREA_MIN_FRAC)
      four_wheel_in_path_raw = _dets_in_path(four_wheel, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=VEHICLE_AREA_MIN_FRAC)
      vehicle_in_path_raw = two_wheel_in_path_raw or four_wheel_in_path_raw
      hazard_in_path_raw = person_in_path_raw or vehicle_in_path_raw

      cone_metric = _dets_metric(cones, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=0.0)
      person_metric = _dets_metric(persons, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=PERSON_AREA_MIN_FRAC)
      two_wheel_metric = _dets_metric(two_wheel, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=TWOWHEEL_AREA_MIN_FRAC)
      four_wheel_metric = _dets_metric(four_wheel, img_rgb.shape[1], img_rgb.shape[0], area_min_frac=VEHICLE_AREA_MIN_FRAC)
      vehicle_metric = max(two_wheel_metric, four_wheel_metric)
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

    objs_refined: list[ObjDet] = list(objs)
    if REFINE_EDGES and img_rgb is not None and objs:
      try:
        objs_refined = list(objs)
        refine_idxs = list(range(len(objs)))
        if REFINE_MAX_OBJS > 0 and len(refine_idxs) > REFINE_MAX_OBJS:
          refine_idxs.sort(key=lambda i: float(objs[i].score), reverse=True)
          refine_idxs = refine_idxs[:REFINE_MAX_OBJS]

        for i in refine_idxs:
          o = objs[i]
          refined = _refine_bbox_edges(img_rgb, o.x1, o.y1, o.x2, o.y2)
          if refined is None:
            continue
          rx1, ry1, rx2, ry2 = refined
          objs_refined[i] = ObjDet(int(o.cls), float(rx1), float(ry1), float(rx2), float(ry2), float(o.score))
      except Exception:
        # Best-effort only; fall back to raw boxes on any failure.
        objs_refined = list(objs)

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

    left_lane_haz_dist_m = 0.0
    right_lane_haz_dist_m = 0.0
    # NOTE: Use the *raw* YOLO boxes for distance estimation. The optional edge-refined boxes are
    # optimized for visualization and can shrink the bbox height, which would bias the pinhole
    # distance estimate to be farther (less conservative) for lane-change safety.
    if img_rgb is not None and objs:
      # Compute min estimated distance for any hazard in the adjacent lanes.
      left_vehicle_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_LEFT_LANE_X_MIN_FRAC, x_max_frac=LC_LEFT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids=LC_VEHICLE_CLASS_IDS, score_min=VEHICLE_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_VEHICLE_HEIGHT_M,
      )
      right_vehicle_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_RIGHT_LANE_X_MIN_FRAC, x_max_frac=LC_RIGHT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids=LC_VEHICLE_CLASS_IDS, score_min=VEHICLE_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_VEHICLE_HEIGHT_M,
      )
      left_person_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_LEFT_LANE_X_MIN_FRAC, x_max_frac=LC_LEFT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids={PERSON_CLASS_IDX}, score_min=PERSON_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_PERSON_HEIGHT_M,
      )
      right_person_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_RIGHT_LANE_X_MIN_FRAC, x_max_frac=LC_RIGHT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids={PERSON_CLASS_IDX}, score_min=PERSON_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_PERSON_HEIGHT_M,
      )
      left_cone_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_LEFT_LANE_X_MIN_FRAC, x_max_frac=LC_LEFT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids={TRAFFIC_CONE_CLASS_IDX}, score_min=LC_CONE_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_CONE_HEIGHT_M,
      )
      right_cone_dist_m = _lane_min_distance_m(
        objs, img_rgb.shape[1], img_rgb.shape[0],
        x_min_frac=LC_RIGHT_LANE_X_MIN_FRAC, x_max_frac=LC_RIGHT_LANE_X_MAX_FRAC, y_min_frac=LC_LANE_Y_MIN_FRAC,
        class_ids={TRAFFIC_CONE_CLASS_IDX}, score_min=LC_CONE_SCORE_MIN,
        focal_length_px=focal_length_px, obj_height_m=LC_ASSUMED_CONE_HEIGHT_M,
      )

      # Merge (min non-zero).
      left_candidates = [d for d in (left_vehicle_dist_m, left_person_dist_m, left_cone_dist_m) if d > 0.0]
      right_candidates = [d for d in (right_vehicle_dist_m, right_person_dist_m, right_cone_dist_m) if d > 0.0]
      left_lane_haz_dist_m = float(min(left_candidates)) if left_candidates else 0.0
      right_lane_haz_dist_m = float(min(right_candidates)) if right_candidates else 0.0

    payload = encode_cone_detections(
      frame_id=int(vipc.frame_id),
      timestamp_sof=int(vipc.timestamp_sof),
      img_width=int(buf.width),
      img_height=int(buf.height),
      focal_length_px=float(focal_length_px),
      cones=cones,
      in_path=cone_in_path,
      objects=objs,
      objects_refined=objs_refined,
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
      left_lane_haz_dist_m=left_lane_haz_dist_m,
      right_lane_haz_dist_m=right_lane_haz_dist_m,
    )
    # `customReservedRawData0` is `Data` in `cereal/log.capnp`, so we must not `init()` it without a size.
    # Assigning bytes sets the union + size correctly.
    msg = messaging.new_message(None, valid=True)
    msg.customReservedRawData0 = payload
    pm.send("customReservedRawData0", msg)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
