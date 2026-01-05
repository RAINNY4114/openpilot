from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


# COCO classes used by `Cone_YOLO11n` (plus custom `traffic cone = 80`).
PERSON_CLASS_ID = 0
CONE_CLASS_ID = 80
VEHICLE_CLASS_IDS = {1, 2, 3, 4, 5, 6, 7, 8}  # bicycle..truck


@dataclass(frozen=True)
class LaneOccupancy:
  left_min_dist_m: float
  right_min_dist_m: float
  left_side_close: bool
  right_side_close: bool


def _min_nonzero(a: float, b: float) -> float:
  a = float(a)
  b = float(b)
  if a <= 0.0:
    return b
  if b <= 0.0:
    return a
  return min(a, b)


def _obj_height_m(cls: int) -> float:
  cls = int(cls)
  if cls == PERSON_CLASS_ID:
    return 1.7
  if cls == CONE_CLASS_ID:
    return 0.7
  if cls in VEHICLE_CLASS_IDS:
    return 1.5
  return 0.0


def _lane_y_at_x(lane_x: np.ndarray, lane_y: np.ndarray, x_m: float) -> float | None:
  if lane_x.size < 2 or lane_y.size < 2 or lane_x.size != lane_y.size:
    return None
  x0 = float(lane_x[0])
  x1 = float(lane_x[-1])
  if not (x0 <= x_m <= x1):
    return None
  return float(np.interp(float(x_m), lane_x, lane_y))


def compute_lane_occupancy(*,
                           objs: Sequence[dict],
                           img_w: int,
                           img_h: int,
                           focal_length_px: float,
                           lane_left_x: Sequence[float],
                           lane_left_y: Sequence[float],
                           lane_right_x: Sequence[float],
                           lane_right_y: Sequence[float],
                           lane_margin_m: float = 0.0,
                           score_min_person: float = 0.25,
                           score_min_vehicle: float = 0.30,
                           score_min_cone: float = 0.25,
                           side_close_dist_m: float = 15.0) -> LaneOccupancy:
  """
  Estimate adjacent-lane occupancy using:
    - YOLO bboxes in image space
    - a pinhole approximation (distance from bbox height)
    - lane boundaries from model lane lines (in car-space meters)

  Returns:
    - left/right minimum forward distance (m) to a hazard object in that lane (0 = none/unknown)
    - left/right "side close" flags (parallel/nearby vehicle guard)
  """
  img_w = int(img_w)
  img_h = int(img_h)
  f_px = float(focal_length_px)
  if img_w <= 0 or img_h <= 0 or f_px <= 1.0:
    return LaneOccupancy(0.0, 0.0, False, False)

  lane_left_x = np.asarray(lane_left_x, dtype=np.float32)
  lane_left_y = np.asarray(lane_left_y, dtype=np.float32)
  lane_right_x = np.asarray(lane_right_x, dtype=np.float32)
  lane_right_y = np.asarray(lane_right_y, dtype=np.float32)
  if lane_left_x.size < 2 or lane_right_x.size < 2:
    return LaneOccupancy(0.0, 0.0, False, False)

  # Require sane ordering for interpolation.
  if float(lane_left_x[0]) > float(lane_left_x[-1]):
    lane_left_x = lane_left_x[::-1]
    lane_left_y = lane_left_y[::-1]
  if float(lane_right_x[0]) > float(lane_right_x[-1]):
    lane_right_x = lane_right_x[::-1]
    lane_right_y = lane_right_y[::-1]

  left_min = 0.0
  right_min = 0.0
  left_side_close = False
  right_side_close = False

  for o in objs:
    if not isinstance(o, dict):
      continue

    try:
      cls = int(o.get("c", -1))
      score = float(o.get("s", 0.0))
      x1 = float(o.get("x1", 0.0))
      y1 = float(o.get("y1", 0.0))
      x2 = float(o.get("x2", 0.0))
      y2 = float(o.get("y2", 0.0))
    except Exception:
      continue

    if not (math.isfinite(score) and math.isfinite(x1) and math.isfinite(y1) and math.isfinite(x2) and math.isfinite(y2)):
      continue
    if x2 <= x1 or y2 <= y1:
      continue

    # Only classify hazards we can assign a coarse physical size to.
    h_m = _obj_height_m(cls)
    if h_m <= 0.1:
      continue

    if cls == PERSON_CLASS_ID and score < float(score_min_person):
      continue
    if cls == CONE_CLASS_ID and score < float(score_min_cone):
      continue
    if cls in VEHICLE_CLASS_IDS and score < float(score_min_vehicle):
      continue

    h_px = float(y2 - y1)
    if h_px < 1.0:
      continue

    # Distance along the optical axis (pinhole approximation).
    dist_m = (f_px * float(h_m)) / h_px
    if not math.isfinite(dist_m):
      continue
    dist_m = float(max(0.0, min(250.0, dist_m)))
    if dist_m <= 0.0:
      continue

    # Lateral position in car space (y left +): y = -(cx - cx0) * x / f
    cx = 0.5 * (x1 + x2)
    y_m = -((float(cx) - float(img_w) * 0.5) * dist_m / max(f_px, 1.0))
    if not math.isfinite(y_m):
      continue

    y_left = _lane_y_at_x(lane_left_x, lane_left_y, dist_m)
    y_right = _lane_y_at_x(lane_right_x, lane_right_y, dist_m)
    if y_left is None or y_right is None:
      continue

    left_boundary = max(float(y_left), float(y_right))
    right_boundary = min(float(y_left), float(y_right))
    # Positive margin is stricter (fewer objects considered "in adjacent lanes").
    # Negative margin is more conservative (more objects block adjacent-lane changes).
    margin = float(lane_margin_m)

    in_left_lane = y_m > (left_boundary + margin)
    in_right_lane = y_m < (right_boundary - margin)

    if in_left_lane:
      left_min = _min_nonzero(left_min, dist_m)
      if (cls in VEHICLE_CLASS_IDS) and dist_m <= float(side_close_dist_m):
        left_side_close = True
    elif in_right_lane:
      right_min = _min_nonzero(right_min, dist_m)
      if (cls in VEHICLE_CLASS_IDS) and dist_m <= float(side_close_dist_m):
        right_side_close = True

  return LaneOccupancy(float(left_min), float(right_min), bool(left_side_close), bool(right_side_close))
