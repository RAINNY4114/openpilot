#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ford / Lincoln Configuration Module
====================================

Provides configurable options for Ford/Lincoln vehicles, adapted from
RAINNY4114/openpilot (dragonpilot) ford branch.

Architecture
------------
Uses a JSON config file at /data/ford_lincoln_config.json to store all
Ford/Lincoln-specific options. This avoids modifying the compiled params
system (libparams_c.so) while still providing persistent, user-tunable
configuration.

The module also attempts to read from the standard Params() system for
keys that are already registered (e.g. LaneTurnDesire, MapSpeedLimit),
falling back to the JSON config file for unregistered keys.

Options provided:
  - Curve Speed Control (CSC): enable/disable predictive curve speed management
  - Curve Sensitivity: aggressiveness factor for curve speed (0.5-1.5)
  - Turn Aggressiveness: lateral curve gain multiplier (0.5-2.0)
  - Human Turn Detection: enable/disable driver turn override
  - HTD Angle Threshold: steering angle threshold for human turn detection
  - Custom Path Offset: user-adjustable lateral path offset (meters)
  - Path Curvature Blend Ratios: low/high curvature blending factors
  - Follow Coast: suppress gentle braking in traffic
  - Stop Distance: desired stopping distance (meters)
  - BSM Voice Alerts: blind spot monitoring voice alerts
  - Auto Avoidance: emergency lateral avoidance
  - Auto Overtake: automatic overtaking
  - Lane Change Assist: speed threshold and auto delay
  - Road Edge Detection: block lane change at road edges
  - AEM: Adaptive Experimental Mode
  - Startup Logo: Ford/Lincoln startup logo selection
"""

import json
import os
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog


# ============================================================================
# Configuration file path
# ============================================================================

CONFIG_FILE_PATH = "/data/ford_lincoln_config.json"
CONFIG_REFRESH_SEC = 2.0


# ============================================================================
# Default configuration
# ============================================================================

DEFAULT_CONFIG = {
    # --- Curve Speed Control (DTSC) ---
    "CurveSpeedControl": True,          # Enable predictive curve speed management
    "CurveSensitivity": 100,            # 50-150 (100 = default, lower = slower in curves)
    "TurnAggressiveness": 100,          # 50-200 (100 = default, controls lateral curve gain)
    "MapTurnControl": True,             # Use map data for turn speed prediction
    "VisionTurnControl": True,          # Use vision model for turn speed prediction
    "ShowCSCStatus": True,              # Show curve speed control status on UI

    # --- Human Turn Detection ---
    "enable_human_turn_detection": True, # Enable driver turn override detection
    "htd_turn_angle_threshold": 60.0,   # Degrees (20-120)

    # --- Lateral / Path Control ---
    "custom_path_offset": 0.0,          # meters, user-adjustable lateral offset
    "pc_blend_ratio_low_C": 0.4,        # Low-curvature path blending ratio
    "pc_blend_ratio_high_C": 0.4,       # High-curvature path blending ratio
    "lane_change_factor_high": 0.85,    # Lane change aggressiveness factor

    # --- Longitudinal Comfort ---
    "dp_lincoln_follow_coast": True,    # Suppress gentle braking in traffic
    "dp_lincoln_stop_distance_m": 4.0,  # Desired stopping distance (meters)
    "dp_lincoln_hazard_alert": False,   # Hazard alert system

    # --- BSM Voice Alerts ---
    "dp_lincoln_bsm_voice_enabled": True,
    "dp_lincoln_bsm_voice_interval_sec": 3,
    "dp_lincoln_bsm_voice_volume_pct": 100,

    # --- Auto Avoidance / Overtake ---
    "dp_lincoln_auto_avoid": False,     # Emergency lateral avoidance
    "dp_lincoln_auto_overtake": False,  # Automatic overtaking
    "dp_lincoln_auto_overtake_min_cruise_kph": 90,
    "dp_lincoln_auto_lc_confirm_delay_sec": 3,
    "dp_lincoln_auto_lc_edge_clearance_m": 0.6,
    "dp_lincoln_lane_preference": 0,    # 0=center, 1=left, 2=right

    # --- Lane Change Assist ---
    "dp_lat_alka": False,               # Always-on Lane Keeping Assist
    "dp_lat_lca_speed": 20,            # mph, lane change assist speed threshold
    "dp_lat_lca_auto_sec": 0.0,         # Auto lane change delay (0 = off)
    "dp_lat_cone_detection": True,      # Cone detection for lane change blocking
    "dp_lat_road_edge_detection": False, # Road edge detection for lane change blocking

    # --- Adaptive Experimental Mode ---
    "dp_lon_aem": False,                # Adaptive Experimental Mode
    "dp_lon_ext_radar": False,          # External radar addon

    # --- Startup Logo ---
    "dp_startup_logo_ford_index": 0,
    "dp_startup_logo_lincoln_index": 0,
    "dp_startup_logo_active_brand": 0,  # 0=default, 1=ford, 2=lincoln

    # --- Path Angle (c1) Fine-Tuning ---
    "path_angle_gain_low_curv_high_speed": 1.15,
    "path_angle_gain_high_curv_low_speed": 1.30,
    "path_angle_gain_high_curv_high_speed": 1.05,
}


class FordLincolnConfig:
    """
    Ford/Lincoln configuration manager.

    Reads configuration from a JSON file and provides typed access to all
    Ford/Lincoln-specific options. Caches values and refreshes periodically.

    Usage:
        config = FordLincolnConfig()
        config.update()  # call periodically to refresh

        if config.get_bool("CurveSpeedControl"):
            ...
        aggressiveness = config.get_float("TurnAggressiveness") / 100.0
    """

    def __init__(self):
        self._config = dict(DEFAULT_CONFIG)
        self._last_refresh = 0.0
        self._loaded = False
        self._load_config()

    def _load_config(self):
        """Load configuration from JSON file."""
        try:
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r") as f:
                    user_config = json.load(f)
                # Merge user config over defaults
                self._config = dict(DEFAULT_CONFIG)
                self._config.update(user_config)
                self._loaded = True
                cloudlog.info("FordLincolnConfig: loaded from %s" % CONFIG_FILE_PATH)
            else:
                self._config = dict(DEFAULT_CONFIG)
                self._loaded = False
                cloudlog.info("FordLincolnConfig: using defaults (no config file)")
        except Exception as e:
            self._config = dict(DEFAULT_CONFIG)
            self._loaded = False
            cloudlog.warning("FordLincolnConfig: failed to load config: %s" % str(e))

    def update(self):
        """Refresh configuration from file (called periodically)."""
        now = time.monotonic()
        if now - self._last_refresh < CONFIG_REFRESH_SEC:
            return
        self._last_refresh = now
        self._load_config()

    def get(self, key, default=None):
        """Get a configuration value as string."""
        return self._config.get(key, default)

    def get_bool(self, key):
        """Get a boolean configuration value."""
        val = self._config.get(key)
        if val is None:
            return False
        if isinstance(val, bool):
            return val
        if isinstance(val, (int, float)):
            return bool(val)
        if isinstance(val, str):
            return val.lower() in ("1", "true", "yes", "on")
        return bool(val)

    def get_int(self, key):
        """Get an integer configuration value."""
        val = self._config.get(key)
        if val is None:
            return 0
        try:
            return int(val)
        except (ValueError, TypeError):
            return 0

    def get_float(self, key):
        """Get a float configuration value."""
        val = self._config.get(key)
        if val is None:
            return 0.0
        try:
            return float(val)
        except (ValueError, TypeError):
            return 0.0

    def set(self, key, value):
        """Set a configuration value and save to file."""
        self._config[key] = value
        self._save_config()

    def _save_config(self):
        """Save configuration to JSON file."""
        try:
            os.makedirs(os.path.dirname(CONFIG_FILE_PATH), exist_ok=True)
            with open(CONFIG_FILE_PATH, "w") as f:
                json.dump(self._config, f, indent=2)
            cloudlog.info("FordLincolnConfig: saved to %s" % CONFIG_FILE_PATH)
        except Exception as e:
            cloudlog.warning("FordLincolnConfig: failed to save config: %s" % str(e))

    def create_default_config(self):
        """Create the default config file if it doesn't exist."""
        if not os.path.exists(CONFIG_FILE_PATH):
            self._save_config()
            cloudlog.info("FordLincolnConfig: created default config at %s" % CONFIG_FILE_PATH)

    @property
    def all_keys(self):
        """Return all configuration keys."""
        return list(self._config.keys())


# ============================================================================
# Singleton accessor
# ============================================================================

_ford_config_instance = None


def get_ford_config():
    """Get the singleton FordLincolnConfig instance."""
    global _ford_config_instance
    if _ford_config_instance is None:
        _ford_config_instance = FordLincolnConfig()
    return _ford_config_instance
