from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np


@dataclass(frozen=True)
class TrackedObject:
  track_id: int
  cls: int
  score: float
  x1: float
  y1: float
  x2: float
  y2: float
  missed: int


@dataclass
class _Detection:
  group: int
  cls: int
  score: float
  x1: float
  y1: float
  x2: float
  y2: float


@dataclass
class _Track:
  track_id: int
  group: int
  cls: int
  score: float
  x1: float
  y1: float
  x2: float
  y2: float
  vx1: float = 0.0
  vy1: float = 0.0
  vx2: float = 0.0
  vy2: float = 0.0
  last_update_t: float = 0.0
  missed: int = 0
  P: np.ndarray = field(default_factory=lambda: np.diag([100.0, 100.0, 100.0, 100.0, 10000.0, 10000.0, 10000.0, 10000.0]).astype(np.float32))

  def bbox_at(self, now: float) -> tuple[float, float, float, float]:
    dt = max(0.0, float(now - self.last_update_t))
    x1 = self.x1 + self.vx1 * dt
    y1 = self.y1 + self.vy1 * dt
    x2 = self.x2 + self.vx2 * dt
    y2 = self.y2 + self.vy2 * dt
    return x1, y1, x2, y2


def _group_for_cls(cls: int) -> int:
  if cls == 0:  # person
    return 0
  if cls == 80:  # traffic cone
    return 1
  if cls in (1, 2, 3, 5, 7):  # bicycle + vehicles
    return 2
  return -1


def _iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
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


class ObjectTracker:
  """
  Light-weight tracker (SORT-style track-by-detection) with:
  - IoU matching
  - Kalman filter for bbox smoothing/prediction (constant velocity)
  - EMA smoothing for score

  Designed for small `MAX_DET` inputs from `coned` (<= ~8 objects).
  """

  def __init__(self, *, iou_thres: float = 0.30, max_missed: int = 5,
               bbox_alpha: float = 0.65, vel_alpha: float = 0.60, score_alpha: float = 0.60) -> None:
    self._next_id = 1
    self._tracks: list[_Track] = []
    self._iou_thres = float(iou_thres)
    self._max_missed = int(max_missed)
    self._bbox_alpha = float(bbox_alpha)
    self._vel_alpha = float(vel_alpha)
    self._score_alpha = float(score_alpha)

  def reset(self) -> None:
    self._next_id = 1
    self._tracks.clear()

  @staticmethod
  def _parse_detections(objs: list[dict]) -> list[_Detection]:
    dets: list[_Detection] = []
    if not isinstance(objs, list):
      return dets

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

      if not (math.isfinite(x1) and math.isfinite(y1) and math.isfinite(x2) and math.isfinite(y2) and math.isfinite(score)):
        continue
      if x2 <= x1 or y2 <= y1:
        continue

      group = _group_for_cls(cls)
      if group < 0:
        continue

      dets.append(_Detection(group=group, cls=cls, score=score, x1=x1, y1=y1, x2=x2, y2=y2))

    return dets

  def update(self, *, objs: list[dict], now: float) -> None:
    dets = self._parse_detections(objs)
    if not dets:
      for t in self._tracks:
        t.missed += 1
      self._tracks = [t for t in self._tracks if t.missed <= self._max_missed]
      return

    track_updated: set[int] = set()
    det_used: set[int] = set()

    groups = sorted({t.group for t in self._tracks} | {d.group for d in dets})
    for g in groups:
      track_idxs = [i for i, t in enumerate(self._tracks) if t.group == g]
      det_idxs = [i for i, d in enumerate(dets) if d.group == g]
      if not track_idxs or not det_idxs:
        continue

      # Greedy IoU matching (small N, stable enough for HUD visualization)
      while True:
        best = (self._iou_thres, None, None)  # (iou, t_idx, d_idx)
        for ti in track_idxs:
          if ti in track_updated:
            continue
          t = self._tracks[ti]
          tb = t.bbox_at(now)
          for di in det_idxs:
            if di in det_used:
              continue
            d = dets[di]
            iou = _iou(tb, (d.x1, d.y1, d.x2, d.y2))
            if iou > best[0]:
              best = (iou, ti, di)

        _, ti, di = best
        if ti is None or di is None:
          break

        track_updated.add(ti)
        det_used.add(di)

        t = self._tracks[ti]
        d = dets[di]

        dt = max(1e-3, float(now - t.last_update_t))

        # 8D Kalman state: [x1, y1, x2, y2, vx1, vy1, vx2, vy2]
        x = np.array([t.x1, t.y1, t.x2, t.y2, t.vx1, t.vy1, t.vx2, t.vy2], dtype=np.float32)

        F = np.eye(8, dtype=np.float32)
        F[0, 4] = dt
        F[1, 5] = dt
        F[2, 6] = dt
        F[3, 7] = dt

        # Predict
        x = F @ x
        P = F @ t.P @ F.T

        bw = max(1.0, float(d.x2 - d.x1))
        bh = max(1.0, float(d.y2 - d.y1))
        size = max(bw, bh)

        # Process noise (scaled with object size and dt)
        q_pos = float(max(4.0, 0.02 * size)) ** 2
        q_vel = float(max(20.0, 0.20 * size)) ** 2
        Q = np.diag([q_pos, q_pos, q_pos, q_pos, q_vel, q_vel, q_vel, q_vel]).astype(np.float32) * dt
        P = P + Q

        # Measurement update
        z = np.array([d.x1, d.y1, d.x2, d.y2], dtype=np.float32)
        H = np.zeros((4, 8), dtype=np.float32)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        H[3, 3] = 1.0

        r_x = float(max(3.0, 0.03 * bw))
        r_y = float(max(3.0, 0.03 * bh))
        R = np.diag([r_x * r_x, r_y * r_y, r_x * r_x, r_y * r_y]).astype(np.float32)

        y = z - (H @ x)
        S = H @ P @ H.T + R
        PHT = P @ H.T
        # K = PHT @ inv(S)  (use solve for stability)
        K = np.linalg.solve(S, PHT.T).T
        x = x + (K @ y)
        P = (np.eye(8, dtype=np.float32) - (K @ H)) @ P

        # Write back
        t.x1, t.y1, t.x2, t.y2, t.vx1, t.vy1, t.vx2, t.vy2 = (float(v) for v in x.tolist())
        t.P = P

        # Sanity: keep bbox valid
        if t.x2 <= t.x1:
          mid = 0.5 * (t.x1 + t.x2)
          t.x1 = mid - 1.0
          t.x2 = mid + 1.0
        if t.y2 <= t.y1:
          mid = 0.5 * (t.y1 + t.y2)
          t.y1 = mid - 1.0
          t.y2 = mid + 1.0

        t.score = self._score_alpha * float(d.score) + (1.0 - self._score_alpha) * float(t.score)
        t.cls = int(d.cls)
        t.last_update_t = float(now)
        t.missed = 0

    # Mark unmatched tracks as missed
    for i, t in enumerate(self._tracks):
      if i not in track_updated:
        t.missed += 1

    # Drop stale tracks
    self._tracks = [t for t in self._tracks if t.missed <= self._max_missed]

    # Create new tracks for unmatched detections
    for i, d in enumerate(dets):
      if i in det_used:
        continue
      self._tracks.append(_Track(
        track_id=self._next_id,
        group=d.group,
        cls=d.cls,
        score=d.score,
        x1=d.x1,
        y1=d.y1,
        x2=d.x2,
        y2=d.y2,
        last_update_t=float(now),
        missed=0,
      ))
      self._next_id += 1

  def get_tracked(self, *, now: float) -> list[TrackedObject]:
    out: list[TrackedObject] = []
    for t in self._tracks:
      x1, y1, x2, y2 = t.bbox_at(now)
      if x2 <= x1 or y2 <= y1:
        continue
      out.append(TrackedObject(
        track_id=int(t.track_id),
        cls=int(t.cls),
        score=float(t.score),
        x1=float(x1),
        y1=float(y1),
        x2=float(x2),
        y2=float(y2),
        missed=int(t.missed),
      ))

    # Stable draw order: group -> id
    out.sort(key=lambda o: (_group_for_cls(o.cls), o.track_id))
    return out
