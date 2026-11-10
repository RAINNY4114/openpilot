#!/usr/bin/env python3

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/nav_lat_blend.json"
LOG_PATH = "/data/media/0/nav_lat_blend.log"

R_MIN_ANY = 150.0

DEFAULTS = {
    "enabled": False,
    "observe_only": True,

    "log": True,
    "log_period_s": 1.0,

    "weight": 0.25,
    "weight_max": 0.25,
    "max_delta_curvature": 0.0015,
    "max_delta_rate_per_s": 0.002,

    "max_nav_curvature": 1.0 / R_MIN_ANY,
    "max_diff": 0.008,
    "preview_m": 15.0,
    "v_min_kmh": 20.0,
    "profile_stale_sec": 2.5,
}

def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def _f(x, default=0.0):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default

def nav_curvature_at(profile_msg, distance_ahead, v_ego=0.0, now=None, stale_s=2.5):
    if profile_msg is None or now is None:
        return None
    try:
        if not bool(getattr(profile_msg, "valid", False)):
            return None
        pts = profile_msg.points
        n = len(pts)
        if n < 2:
            return None

        stamp = _f(getattr(profile_msg, "sourceMonoTime", 0)) / 1e9
        if stamp <= 0.0 or (now - stamp) > stale_s:
            return None

        lag = _clip(now - stamp, 0.0, stale_s)
        ego_progress = _f(profile_msg.progress) + max(v_ego, 0.0) * lag

        p_prog = np.empty(n)
        p_curv = np.empty(n)
        for i in range(n):
            p_prog[i] = _f(pts[i].progress, float("nan"))
            p_curv[i] = _f(pts[i].curvature)
        if not np.all(np.isfinite(p_prog)):
            return None
        if np.any(np.diff(p_prog) < -1e-3):
            return None

        target = ego_progress + max(distance_ahead, 0.0)
        return float(np.interp(target, p_prog, p_curv))
    except Exception:
        return None

class NavCurvatureBlend:

    def __init__(self, dt=0.01):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self.dt = max(float(dt), 1e-3)

        self._delta = 0.0
        self._last_log_t = 0.0
        self._last_reason = "init"
        self.last_delta = 0.0
        self.last_nav_curvature = 0.0

        self._load_config(force=True)

    def _load_config(self, force=False):
        try:
            st = os.stat(CONFIG_PATH)
            if (not force) and self._cfg_mtime == st.st_mtime:
                return
            with open(CONFIG_PATH, "r") as f:
                user = json.load(f)
            cfg = dict(DEFAULTS)
            if isinstance(user, dict):
                for k, v in user.items():
                    if k in cfg:
                        cfg[k] = v
            self.cfg = cfg
            self._cfg_mtime = st.st_mtime
        except Exception:
            if force:
                self.cfg = dict(DEFAULTS)
                self._cfg_mtime = None

    def blend(self, v_ego, lat_active, model_curvature, profile_msg, now=None):
        model_curvature = _f(model_curvature)
        try:
            return self._blend(v_ego, lat_active, model_curvature, profile_msg, now)
        except Exception:
            self._delta = 0.0
            self.last_delta = 0.0
            self._last_reason = "exception"
            return model_curvature

    def _blend(self, v_ego, lat_active, model_curvature, profile_msg, now):
        if now is None:
            now = time.monotonic()
        v_ego = _f(v_ego)

        self._load_config()
        cfg = self.cfg

        enabled = bool(cfg.get("enabled", False))
        observe_only = bool(cfg.get("observe_only", True))

        k_nav = nav_curvature_at(profile_msg, float(cfg.get("preview_m", 15.0)), v_ego, now,
                                 float(cfg.get("profile_stale_sec", 2.5)))
        self.last_nav_curvature = 0.0 if k_nav is None else k_nav

        reason = None
        if not enabled or observe_only:
            reason = "observe_only" if enabled else "disabled"
        elif not bool(lat_active):
            reason = "lat_inactive"
        elif v_ego < float(cfg.get("v_min_kmh", 20.0)) / 3.6:
            reason = "v_low"
        elif k_nav is None:
            reason = "no_profile"
        elif abs(k_nav) > float(cfg.get("max_nav_curvature", 1.0 / R_MIN_ANY)):
            reason = "curve_too_tight"
        elif abs(model_curvature - k_nav) > float(cfg.get("max_diff", 0.008)):
            reason = "disagree"

        if reason is not None:
            rel = abs(float(cfg.get("max_delta_rate_per_s", 0.002))) * self.dt
            self._delta -= math.copysign(min(rel, abs(self._delta)), self._delta) if self._delta != 0.0 else 0.0
            self.last_delta = self._delta
            self._last_reason = reason
            self._log(now, v_ego, model_curvature, k_nav, 0.0, self._delta, reason)
            return model_curvature + self._delta

        w = _clip(float(cfg.get("weight", 0.25)), 0.0, float(cfg.get("weight_max", 0.25)))
        target = _clip(w * (k_nav - model_curvature),
                       -abs(float(cfg.get("max_delta_curvature", 0.0015))),
                       abs(float(cfg.get("max_delta_curvature", 0.0015))))

        rate = abs(float(cfg.get("max_delta_rate_per_s", 0.002))) * self.dt
        self._delta += _clip(target - self._delta, -rate, rate)

        self.last_delta = self._delta
        self._last_reason = "applied" if abs(self._delta) > 1e-6 else "idle"
        self._log(now, v_ego, model_curvature, k_nav, target, self._delta, self._last_reason)
        return model_curvature + self._delta

    def reset(self):
        self._delta = 0.0
        self.last_delta = 0.0

    def _log(self, now, v_ego, k_model, k_nav, target, delta, reason):
        if not bool(self.cfg.get("log", True)):
            return
        if now - self._last_log_t < float(self.cfg.get("log_period_s", 1.0)):
            return
        self._last_log_t = now
        try:
            k_nav_s = "   --" if k_nav is None else f"{k_nav * 1e3:6.2f}"
            line = (
                f"{time.strftime('%H:%M:%S')} "
                f"v={v_ego * 3.6:6.1f}kph "
                f"k_model={k_model * 1e3:7.2f} k_nav={k_nav_s} "
                f"tgt={target * 1e3:6.2f} out={delta * 1e3:6.2f} "
                f"(1e-3 1/m) why={reason}\n"
            )
            try:
                if os.path.getsize(LOG_PATH) > 2_000_000:
                    with open(LOG_PATH, "w") as f:
                        f.write("")
            except Exception:
                pass
            with open(LOG_PATH, "a") as f:
                f.write(line)
        except Exception:
            pass

    def state(self):
        return {
            "enabled": bool(self.cfg.get("enabled", False)),
            "observe_only": bool(self.cfg.get("observe_only", True)),
            "delta": float(self._delta),
            "k_nav": float(self.last_nav_curvature),
            "why": self._last_reason,
        }
