#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ford / Lincoln Curve Speed Fusion Controller
=============================================

Fuses vision model predictions and OSM map data for predictive curve
speed management, building on top of sunnypilot's Smart Cruise Control
(SCC) infrastructure.

Architecture
------------
The C3X sunnypilot system already has two curve speed controllers:
  - SmartCruiseControlVision: uses model orientationRate.z * velocity.x
    to predict lateral acceleration and slow for curves (~10s lookahead)
  - SmartCruiseControlMap: uses GPS + OSM MapTargetVelocities from mapd
    to detect curves far ahead (100s of meters)

This module is a Ford/Lincoln-specific fusion layer that:
  1. Reads both SCC vision and SCC map outputs
  2. Also directly reads model predictions for Ford-specific physics
  3. Fuses the two sources intelligently:
     - Map = long-range early warning (start gentle decel early)
     - Vision = short-range precise geometry (precise decel curve)
     - Both agree = higher confidence, stronger decel
  4. Applies Ford/Lincoln comfort tuning:
     - Configurable comfort lateral G (CurveSensitivity)
     - Ford-specific safety factor
     - Configurable max deceleration

Core physics: v_max = sqrt(lateral_acceleration / curvature) * safety_factor

Configuration (via FordLincolnConfig):
  - CurveSpeedControl: master enable/disable
  - CurveSensitivity: 50-150 (maps to aggressiveness 0.5-1.5)
  - MapTurnControl: use map data (long-range)
  - VisionTurnControl: use vision model (short-range)
"""

import math
import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.ford_lincoln_config import get_ford_config


# ============================================================================
# Physics constants
# ============================================================================

COMFORT_LAT_G = 0.2       # g units - universal human comfort threshold
BASE_LAT_ACC = COMFORT_LAT_G * 9.81  # ~2.0 m/s^2
SAFETY_FACTOR = 0.9       # 10% safety margin on calculated speeds
MIN_CURVE_DISTANCE = 5.0  # meters - minimum distance to react to curves
MAX_DECEL = -2.0          # m/s^2 - maximum comfortable deceleration

# Minimum speed for curve speed control to be active
DTSC_MIN_SPEED_MS = 5.0   # ~18 km/h

# Fusion confidence boost: when both vision and map agree, apply extra decel
FUSION_CONFIDENCE_BOOST = 1.15  # 15% stronger decel when both sources agree

# Long-range map deceleration: gentler than vision-based, starts earlier
MAP_MAX_DECEL = -1.2       # m/s^2 - gentler for long-range prediction
MAP_DEFAULT_DISTANCE = 150.0  # meters - assumed detection distance for map


class FordCurveSpeedFusion:
    """
    Ford / Lincoln curve speed fusion controller.

    Fuses vision model predictions and OSM map data for predictive curve
    speed management. Builds on sunnypilot's SCC infrastructure rather than
    duplicating it.

    Usage:
        fusion = FordCurveSpeedFusion()
        # In longitudinal planner update():
        a_max_constraint = fusion.update(
            v_ego, model_msg,
            scc_vision=planner.scc.vision,
            scc_map=planner.scc.map,
        )
        if a_max_constraint is not None:
            a_desired_trajectory = np.minimum(a_desired_trajectory, a_max_constraint)
    """

    def __init__(self, aggressiveness=1.0):
        self.aggressiveness = float(np.clip(aggressiveness, 0.5, 1.5))
        self.active = False
        self.debug_msg = ""
        self.target_speed_ms = 0.0
        self.curve_distance_m = 0.0
        self.required_decel = 0.0

        # Source states for fusion
        self.vision_active = False
        self.map_active = False
        self.fusion_mode = "none"  # "vision", "map", "fused"

        # Config
        self._config = get_ford_config()

        cloudlog.info("FordCurveSpeedFusion: Initialized with aggressiveness %.2f" % self.aggressiveness)

    def _refresh_config(self):
        """Refresh configuration from FordLincolnConfig."""
        self._config.update()

        sensitivity = self._config.get_int("CurveSensitivity")
        if sensitivity > 0:
            self.aggressiveness = float(np.clip(sensitivity / 100.0, 0.5, 1.5))

    def is_enabled(self):
        """Check if curve speed control is enabled via configuration."""
        self._refresh_config()
        return self._config.get_bool("CurveSpeedControl")

    def update(self, v_ego, model_msg=None, scc_vision=None, scc_map=None):
        """
        Fuse vision and map data to produce curve speed deceleration constraint.

        Args:
            v_ego: Current vehicle speed (m/s)
            model_msg: ModelDataV2 for direct curvature prediction (optional)
            scc_vision: SmartCruiseControlVision instance (optional)
            scc_map: SmartCruiseControlMap instance (optional)

        Returns:
            a_max array (np.ndarray) for MPC constraint, or None if inactive.
            Length matches T_IDXS_MPC.
        """
        self._refresh_config()

        # Reset state
        self.vision_active = False
        self.map_active = False
        self.fusion_mode = "none"

        # Check master enable
        if not self._config.get_bool("CurveSpeedControl"):
            self._deactivate()
            return None

        # Below minimum speed, don't intervene
        if v_ego < DTSC_MIN_SPEED_MS:
            self._deactivate()
            return None

        use_vision = self._config.get_bool("VisionTurnControl")
        use_map = self._config.get_bool("MapTurnControl")

        # Initialize a_max with no constraint
        a_max = np.ones(len(T_IDXS_MPC)) * 1e3  # effectively no limit

        # --- Vision-based curve speed (short-range, precise) ---
        vision_decel = None
        if use_vision:
            vision_decel = self._vision_fusion(v_ego, model_msg, scc_vision)

        # --- Map-based curve speed (long-range, early warning) ---
        map_decel = None
        if use_map:
            map_decel = self._map_fusion(v_ego, scc_map)

        # --- Fusion logic ---
        if vision_decel is not None and map_decel is not None:
            # Both sources active: fusion mode
            # Use the more conservative (lower) deceleration, with confidence boost
            fused_decel = min(vision_decel, map_decel) * FUSION_CONFIDENCE_BOOST
            fused_decel = max(fused_decel, MAX_DECEL)

            self._apply_decel_to_a_max(a_max, fused_decel, v_ego, self.curve_distance_m)
            self.fusion_mode = "fused"
            self.required_decel = fused_decel
            self.active = True

        elif vision_decel is not None:
            # Only vision: short-range precise control
            self._apply_decel_to_a_max(a_max, vision_decel, v_ego, self.curve_distance_m)
            self.fusion_mode = "vision"
            self.required_decel = vision_decel
            self.active = True

        elif map_decel is not None:
            # Only map: long-range early deceleration
            self._apply_decel_to_a_max(a_max, map_decel, v_ego, self.curve_distance_m)
            self.fusion_mode = "map"
            self.required_decel = map_decel
            self.active = True

        else:
            self._deactivate()
            return None

        # Update debug
        source_desc = self.fusion_mode
        if self.target_speed_ms > 0:
            self.debug_msg = "[%s] Curve -> %.0f km/h (decel %.1f)" % (
                source_desc, self.target_speed_ms * 3.6, self.required_decel
            )
        else:
            self.debug_msg = "[%s] Decel %.1f m/s^2" % (source_desc, self.required_decel)

        cloudlog.debug("FordCurveSpeedFusion: %s" % self.debug_msg)

        return a_max

    def _vision_fusion(self, v_ego, model_msg, scc_vision):
        """
        Vision-based curve speed using SCC vision data + direct model prediction.

        Combines two data sources:
        1. SCC vision controller state (max_pred_lat_acc, v_target, a_target)
        2. Direct model prediction (orientationRate.z, velocity.x, position.x)

        The SCC vision controller provides a state machine (entering/turning/leaving)
        with v_target and a_target. We use this as the primary signal, and supplement
        with direct physics calculation for Ford-specific tuning.
        """
        # --- Source 1: SCC Vision controller output ---
        scc_v_target = 0.0
        scc_a_target = 0.0
        scc_active = False
        scc_max_pred_lat_acc = 0.0

        if scc_vision is not None:
            try:
                scc_v_target = float(scc_vision.output_v_target)
                scc_a_target = float(scc_vision.output_a_target)
                scc_active = bool(scc_vision.is_active)
                scc_max_pred_lat_acc = float(scc_vision.max_pred_lat_acc)
            except Exception:
                pass

        # --- Source 2: Direct model prediction (Ford physics) ---
        model_decel = None
        if model_msg is not None and self._is_model_data_valid(model_msg):
            model_decel = self._vision_direct_physics(model_msg, v_ego)

        # --- Fuse SCC vision + direct model prediction ---
        if scc_active and model_decel is not None:
            # Both available: use more conservative
            scc_decel = max(scc_a_target, MAX_DECEL)
            return min(scc_decel, model_decel)
        elif model_decel is not None:
            return model_decel
        elif scc_active and scc_a_target < 0.0:
            # Use SCC vision's a_target as deceleration
            self.vision_active = True
            self.target_speed_ms = scc_v_target if scc_v_target > 0 else 0.0
            self.curve_distance_m = self._estimate_curve_distance(v_ego, scc_max_pred_lat_acc)
            return max(scc_a_target, MAX_DECEL)

        return None

    def _vision_direct_physics(self, model_msg, v_ego):
        """
        Direct physics-based curve speed from model predictions.

        Core formula: v_max = sqrt(a_lat / kappa) * safety

        Scans the model's predicted path for curvature and calculates
        safe speeds based on Ford/Lincoln lateral acceleration limits.
        """
        try:
            v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
            turn_rates = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)
            positions = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
        except Exception:
            return None

        # Calculate curvature (turn_rate / velocity)
        curvatures = np.abs(turn_rates / np.clip(v_pred, 1.0, 100.0))

        # Calculate safe speeds: v_max = sqrt(a_lat / kappa) * safety
        lat_acc_limit = BASE_LAT_ACC * self.aggressiveness
        safe_speeds = np.sqrt(lat_acc_limit / (curvatures + 1e-6)) * SAFETY_FACTOR

        # Find speed violations
        speed_excess = v_pred - safe_speeds
        if np.all(speed_excess <= 0):
            return None

        # Find critical point (maximum speed excess)
        critical_idx = int(np.argmax(speed_excess))
        critical_distance = float(positions[critical_idx])
        critical_safe_speed = float(safe_speeds[critical_idx])

        if critical_distance <= MIN_CURVE_DISTANCE:
            return None

        # Calculate required deceleration: a = (v_f^2 - v_i^2) / (2*d)
        required_decel = (critical_safe_speed**2 - v_ego**2) / (2.0 * critical_distance)
        required_decel = max(required_decel, MAX_DECEL)

        self.vision_active = True
        self.target_speed_ms = critical_safe_speed
        self.curve_distance_m = critical_distance

        return required_decel

    def _map_fusion(self, v_ego, scc_map):
        """
        Map-based curve speed using SCC map data.

        The SCC map controller reads OSM MapTargetVelocities (from mapd) and
        calculates target velocities for upcoming curves based on GPS position.

        We use the SCC map's v_target and a_target, supplemented with
        Ford-specific long-range deceleration planning.
        """
        scc_v_target = 0.0
        scc_a_target = 0.0
        scc_active = False

        if scc_map is not None:
            try:
                scc_v_target = float(scc_map.output_v_target)
                scc_a_target = float(scc_map.output_a_target)
                scc_active = bool(scc_map.is_active)
            except Exception:
                pass

        if scc_active and scc_v_target > 0 and scc_v_target < v_ego:
            # SCC map detected a curve ahead with lower target speed
            # Use gentler deceleration for long-range prediction
            required_decel = (scc_v_target**2 - v_ego**2) / (2.0 * MAP_DEFAULT_DISTANCE)
            required_decel = max(required_decel, MAP_MAX_DECEL)

            self.map_active = True
            # Don't overwrite target_speed if vision already set it
            if self.target_speed_ms == 0.0 or self.target_speed_ms > scc_v_target:
                self.target_speed_ms = scc_v_target
            # Use map distance for long-range planning
            if self.curve_distance_m == 0.0 or self.curve_distance_m < MAP_DEFAULT_DISTANCE:
                self.curve_distance_m = MAP_DEFAULT_DISTANCE

            return required_decel

        return None

    def _apply_decel_to_a_max(self, a_max, decel, v_ego, curve_distance):
        """
        Apply deceleration constraint to a_max array.

        Progressively limits acceleration until the critical curve point.
        """
        for i in range(len(T_IDXS_MPC)):
            t = T_IDXS_MPC[i]
            distance_at_t = v_ego * t + 0.5 * decel * t**2

            if distance_at_t < curve_distance:
                a_max[i] = min(a_max[i], decel)

    def _estimate_curve_distance(self, v_ego, max_pred_lat_acc):
        """Estimate distance to curve based on predicted lateral acceleration."""
        if max_pred_lat_acc <= 0:
            return MIN_CURVE_DISTANCE
        # Higher predicted lat acc = closer curve
        # Rough estimate: scale distance inversely with lat acc
        return max(MIN_CURVE_DISTANCE, min(200.0, v_ego * 5.0 / max(1.0, max_pred_lat_acc)))

    def _is_model_data_valid(self, model_msg):
        """Check if model message contains valid prediction data."""
        try:
            return (len(model_msg.position.x) == ModelConstants.IDX_N and
                    len(model_msg.velocity.x) == ModelConstants.IDX_N and
                    len(model_msg.orientationRate.z) == ModelConstants.IDX_N)
        except Exception:
            return False

    def _deactivate(self):
        """Clear active state and debug message."""
        self.active = False
        self.debug_msg = ""
        self.target_speed_ms = 0.0
        self.curve_distance_m = 0.0
        self.required_decel = 0.0
        self.vision_active = False
        self.map_active = False
        self.fusion_mode = "none"
