import json
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ConeDet:
  x1: float
  y1: float
  x2: float
  y2: float
  score: float


@dataclass(frozen=True)
class ObjDet:
  cls: int
  x1: float
  y1: float
  x2: float
  y2: float
  score: float


def encode_cone_detections(*, frame_id: int, timestamp_sof: int, img_width: int, img_height: int,
                           cones: list[ConeDet], in_path: bool,
                            focal_length_px: float = 0.0,
                             objects: list[ObjDet] | None = None,
                             objects_refined: list[ObjDet] | None = None,
                             hazard_in_path: bool = False, person_in_path: bool = False, vehicle_in_path: bool = False,
                             cone_metric: float = 0.0, person_metric: float = 0.0, vehicle_metric: float = 0.0,
                             obstacle_metric: float = 0.0, hazard_metric: float = 0.0,
                            in_path_raw: bool = False, person_in_path_raw: bool = False, vehicle_in_path_raw: bool = False,
                            hazard_in_path_raw: bool = False,
                            left_lane_haz_dist_m: float = 0.0, right_lane_haz_dist_m: float = 0.0,
                            version: int = 1) -> bytes:
  payload = {
    "v": int(version),
    "frameId": int(frame_id),
    "timestampSof": int(timestamp_sof),
    "imgW": int(img_width),
    "imgH": int(img_height),
    "focalLengthPx": float(focal_length_px),
    "inPath": bool(in_path),
    "inPathRaw": bool(in_path_raw),
    "hazInPath": bool(hazard_in_path),
    "hazInPathRaw": bool(hazard_in_path_raw),
    "personInPath": bool(person_in_path),
    "personInPathRaw": bool(person_in_path_raw),
    "vehicleInPath": bool(vehicle_in_path),
    "vehicleInPathRaw": bool(vehicle_in_path_raw),
    "coneMetric": float(cone_metric),
    "personMetric": float(person_metric),
    "vehicleMetric": float(vehicle_metric),
    "obstacleMetric": float(obstacle_metric),
    "hazMetric": float(hazard_metric),
    "leftLaneHazDistM": float(left_lane_haz_dist_m),
    "rightLaneHazDistM": float(right_lane_haz_dist_m),
    "cones": [
      {"x1": float(c.x1), "y1": float(c.y1), "x2": float(c.x2), "y2": float(c.y2), "s": float(c.score)}
      for c in cones
    ],
    "objs": [
      {"c": int(o.cls), "x1": float(o.x1), "y1": float(o.y1), "x2": float(o.x2), "y2": float(o.y2), "s": float(o.score)}
      for o in (objects or [])
    ],
    "objsR": [
      {"c": int(o.cls), "x1": float(o.x1), "y1": float(o.y1), "x2": float(o.x2), "y2": float(o.y2), "s": float(o.score)}
      for o in (objects_refined or [])
    ],
  }
  return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_cone_detections(raw: bytes) -> dict[str, Any] | None:
  try:
    obj = json.loads(raw.decode("utf-8", errors="strict"))
  except Exception:
    return None

  if not isinstance(obj, dict) or obj.get("v") != 1:
    return None
  return obj
