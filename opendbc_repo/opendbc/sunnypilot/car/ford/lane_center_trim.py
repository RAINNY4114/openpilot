#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaneCenterTrim - angle-mode lane centering trim (BluePilot bp-7.0, verbatim algorithm).

A small curvature-domain correction toward the true lane-line center (plus an
optional user left/right bias), for path_angle-primary control.

The correction is added to kappa_cmd BEFORE the deviation clip / PSCM clamp /
DBC clip / soft ROC, so it inherits every one of those limiters automatically
instead of bypassing them.  Geometry: raw = 2*error / lookahead**2 (from
y ~= 1/2 * kappa * x**2), solved for the curvature that removes a lateral
position error at the lookahead distance.

Target = confidence blend of (model's own predicted path) and (geometric lane
center).  Good lane lines -> full centering; missing/poor lines -> falls back
to the model's own path (NOT zero), so the user offset bias still applies.

Confidence formula is ported verbatim from lateral_curv_ext.py's path_offset
blend (width std tolerance + laneline probs, smooth not a hard cutoff).

Safety: default disabled via enable flag; any invalid input / inactive / low
speed / lane change -> reset and return kappa_cmd unchanged.  The correction is
hard-clipped (0.004 1/m), exponentially smoothed (tau 0.4s) and rate-limited
(0.00015 per 20Hz tick).
"""
import numpy as np
from numpy import interp

# Laneline confidence blend (verbatim from lateral_curv_ext.py path_offset blend)
_WIDTH_TOLERANCE_BP = (2.4, 2.8, 3.75, 4.25)
_WIDTH_TOLERANCE_V = (0.0, 0.81, 0.81, 0.59)
_STD_TOLERANCE_BP = (0.3, 0.5)
_STD_TOLERANCE_V = (0.81, 0.0)
_CONFIDENCE_BP = (0.6, 0.8)
_CONFIDENCE_V = (0.0, 1.0)

# Speed ramp (reuses curvature-mode centering authority envelope)
_SPEED_RAMP_BP = (0.0, 9.0, 15.0)  # m/s
_SPEED_RAMP_V = (0.0, 0.0, 1.0)

_LOOKAHEAD_MIN_M = 8.0
_LOOKAHEAD_MAX_M = 35.0
_MAX_RAW_CORRECTION = 0.004
_SMOOTH_TAU_S = 0.4
_CORRECTION_ROC_PER_TICK = 0.00015


class LaneCenterTrim:
  def __init__(self):
    self._correction = 0.0

  def reset(self) -> None:
    self._correction = 0.0

  def update(self, kappa_cmd: float, model, v_ego: float, enabled: bool, offset: float,
             gain: float, lat_active: bool, lane_change: bool) -> float:
    """Return kappa_cmd nudged toward (lane-blend target + offset) when active.

    offset (m): positive shifts target right, negative left.
    gain (0.0-1.0): authority; 0 disables effect without disabling detection.
    """
    if not enabled or not lat_active or lane_change or model is None:
      self.reset()
      return kappa_cmd

    speed_factor = float(interp(v_ego, _SPEED_RAMP_BP, _SPEED_RAMP_V))
    if speed_factor <= 0.0:
      self.reset()
      return kappa_cmd
    if not (np.isfinite(offset) and np.isfinite(gain)):
      self.reset()
      return kappa_cmd

    valid, raw = self._raw_correction(model, v_ego, offset)
    if not valid:
      self.reset()
      return kappa_cmd

    target = float(np.clip(raw, -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION)) * float(np.clip(gain, 0.0, 1.0)) * speed_factor
    alpha = 1.0 - np.exp(-0.05 / _SMOOTH_TAU_S)  # 20 Hz tick
    filtered = float(alpha * target + (1.0 - alpha) * self._correction)
    self._correction = float(np.clip(filtered, self._correction - _CORRECTION_ROC_PER_TICK,
                                      self._correction + _CORRECTION_ROC_PER_TICK))
    return kappa_cmd + self._correction

  @property
  def correction(self) -> float:
    return self._correction

  def _raw_correction(self, model, v_ego: float, offset: float) -> tuple[bool, float]:
    try:
      pos_x = np.asarray(model.position.x, dtype=float)
      pos_y = np.asarray(model.position.y, dtype=float)
      if pos_x.size < 2 or pos_x.size != pos_y.size:
        return False, 0.0
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all() and np.all(np.diff(pos_x) > 0)):
        return False, 0.0
      lookahead = float(np.clip(v_ego, _LOOKAHEAD_MIN_M, _LOOKAHEAD_MAX_M))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0

    scale, laneline_center_y = self._laneline_blend(model, lookahead)
    target_y = model_y * (1.0 - scale) + laneline_center_y * scale
    error = (target_y + offset) - model_y
    raw = 2.0 * error / (lookahead ** 2)
    return True, float(raw)

  def _laneline_blend(self, model, lookahead: float) -> tuple[float, float]:
    try:
      lane_lines = model.laneLines
      probs = model.laneLineProbs
      stds = model.laneLineStds
      if len(lane_lines) < 3 or len(probs) < 3 or len(stds) < 3:
        return 0.0, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      if (left_x.size < 2 or left_x.size != left_y.size or
          right_x.size < 2 or right_x.size != right_y.size):
        return 0.0, 0.0
      if not (np.isfinite(left_x).all() and np.isfinite(left_y).all() and
              np.isfinite(right_x).all() and np.isfinite(right_y).all()):
        return 0.0, 0.0
      if not (np.all(np.diff(left_x) > 0) and np.all(np.diff(right_x) > 0)):
        return 0.0, 0.0
      if not all(np.isfinite(v) for v in (probs[1], probs[2], stds[1], stds[2])):
        return 0.0, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left

      width_tolerance = float(np.interp(width, _WIDTH_TOLERANCE_BP, _WIDTH_TOLERANCE_V))
      std_tolerance = float(np.interp(max(float(stds[1]), float(stds[2])), _STD_TOLERANCE_BP, _STD_TOLERANCE_V))
      confidence = min(float(probs[1]), float(probs[2]), width_tolerance, std_tolerance)
      scale = float(np.clip(np.interp(confidence, _CONFIDENCE_BP, _CONFIDENCE_V), 0.0, 1.0))
      center_y = 0.5 * (left + right)
      return scale, center_y
    except (AttributeError, IndexError, TypeError, ValueError):
      return 0.0, 0.0
