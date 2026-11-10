#!/usr/bin/env python3

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/nav_speed.json"
LOG_PATH = "/data/media/0/nav_speed.log"

D_BACK_MARGIN = 5.0
D_MIN = 1.0
STOP_V = 0.3
STOP_D = 10.0

DEFAULTS = {
    "enabled": False,
    "observe_only": True,

    "log": True,
    "log_period_s": 1.0,

    "use_speed_limit": True,
    "use_profile": True,

    "speed_limit_offset_kmh": 0.0,
    "amap_priority": True,
    "amap_stale_sec": 3.0,
    "amap_limit_unit": "auto",
    "amap_kph_threshold": 45.0,
    "limit_closed_loop_gain": 0.5,
    "limit_deadband_ms": 0.3,

    "a_dec_ms2": 0.8,
    "lookahead_m": 300.0,

    "max_decel_ms2": 2.0,
    "max_rise_ms2_per_s": 1.5,
    "release_ms2_per_s": 3.0,

    "v_min_kmh": 10.0,
    "profile_stale_sec": 2.5,
    "stale_hold_sec": 1.0,
}

def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def _f(x, default=0.0):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default

class NavProfileView:

    __slots__ = ("progress", "speed_limit", "p_progress", "p_curv", "p_cap", "stamp")

    def __init__(self, progress, speed_limit, p_progress, p_curv, p_cap, stamp):
        self.progress = progress
        self.speed_limit = speed_limit
        self.p_progress = p_progress
        self.p_curv = p_curv
        self.p_cap = p_cap
        self.stamp = stamp

    @classmethod
    def from_msg(cls, msg):
        if msg is None:
            return None
        if not bool(getattr(msg, "valid", False)):
            return None
        pts = getattr(msg, "points", None)
        if pts is None:
            return None
        n = len(pts)
        if n < 2:
            return None

        p_progress = np.empty(n, dtype=np.float64)
        p_curv = np.empty(n, dtype=np.float64)
        p_cap = np.empty(n, dtype=np.float64)
        for i in range(n):
            p = pts[i]
            p_progress[i] = _f(p.progress, float("nan"))
            p_curv[i] = _f(p.curvature)
            p_cap[i] = _f(p.speedCap, -1.0)

        if not np.all(np.isfinite(p_progress)):
            return None
        if np.any(np.diff(p_progress) < -1e-3):
            return None

        stamp = _f(getattr(msg, "sourceMonoTime", 0)) / 1e9
        if stamp <= 0.0:
            return None

        return cls(_f(msg.progress), _f(msg.speedLimit, -1.0), p_progress, p_curv, p_cap, stamp)

class NavSpeedPlanner:

    def __init__(self, dt=0.05):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self.dt = max(float(dt), 1e-3)

        self._shaped = 0.0
        self._last_log_t = 0.0
        self._last_reason = "init"

        self.a_target = 0.0
        self.should_stop = False
        self.v_cruise_cap = None
        self.v_cap = float("inf")
        self.d_cap = float("inf")
        self.limit_source = "none"
        self.amap_limit_raw = -1.0

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

    def update(self, v_ego, profile_msg=None, amap_msg=None, long_active=True, now=None,
               amap_recv_time=None):
        try:
            return self._update(v_ego, profile_msg, amap_msg, long_active, now, amap_recv_time)
        except Exception:
            self._shaped = 0.0
            self.a_target = 0.0
            self.should_stop = False
            self.v_cruise_cap = None
            self._last_reason = "exception"
            return 0.0, None

    def _update(self, v_ego, profile_msg, amap_msg, long_active, now, amap_recv_time=None):
        if now is None:
            now = time.monotonic()
        v_ego = _f(v_ego)

        self._load_config()
        cfg = self.cfg

        enabled = bool(cfg.get("enabled", False))
        observe_only = bool(cfg.get("observe_only", True))

        if isinstance(profile_msg, NavProfileView):
            view = profile_msg
        else:
            view = NavProfileView.from_msg(profile_msg)
        amap_limit = self._amap_speed_limit(amap_msg, now, amap_recv_time)

        out_a, out_cap, reason, v_cap, d_cap, lim_src, out_stop = self._compute(
            v_ego, view, amap_limit, now, cfg)

        self.v_cap = v_cap
        self.d_cap = d_cap
        self.limit_source = lim_src

        if not enabled or observe_only:
            self._shaped = 0.0
            self.a_target = 0.0
            self.should_stop = False
            self.v_cruise_cap = None
            self._last_reason = "observe_only" if enabled else "disabled"
            self._log(now, v_ego, out_a, 0.0, out_cap, v_cap, d_cap, self._last_reason)
            self.should_stop = bool(out_stop)
            return 0.0, None

        if not bool(long_active):
            self._shaped = 0.0
            self.a_target = 0.0
            self.should_stop = False
            self.v_cruise_cap = None
            self._last_reason = "long_inactive"
            self._log(now, v_ego, out_a, 0.0, out_cap, v_cap, d_cap, self._last_reason)
            return 0.0, None

        if v_ego < float(cfg.get("v_min_kmh", 10.0)) / 3.6:
            self._shaped = 0.0
            self.a_target = 0.0
            self.should_stop = bool(out_stop)
            self.v_cruise_cap = None
            self._last_reason = "v_low"
            self._log(now, v_ego, out_a, 0.0, out_cap, v_cap, d_cap, self._last_reason)
            return 0.0, None

        if reason in ("no_profile", "profile_stale", "profile_off", "profile_empty"):
            a_lim = 0.0
            if out_cap is not None and bool(cfg.get("use_speed_limit", True)):
                over = v_ego - float(out_cap)
                gain = float(cfg.get("limit_closed_loop_gain", 0.5))
                if over > float(cfg.get("limit_deadband_ms", 0.3)) and gain > 0.0:
                    a_lim = -over * gain

            prev_shaped = self._shaped
            prev_stop = self.should_stop
            hold = 0.0
            if reason == "profile_stale" and view is not None:
                stale_s = float(cfg.get("profile_stale_sec", 2.5))
                hold_s = max(float(cfg.get("stale_hold_sec", 1.0)), 0.0)
                if (now - view.stamp) <= stale_s + hold_s:
                    hold = prev_shaped

            target = min(a_lim, hold)
            self._shaped = self._shape(target, cfg)
            self.a_target = float(self._shaped)
            self.should_stop = bool(hold < -0.01 and prev_stop)
            self.v_cruise_cap = out_cap
            if hold < -0.01:
                self._last_reason = "profile_hold"
            else:
                self._last_reason = "limit_only" if self.a_target < -0.01 else reason
            self._log(now, v_ego, target, self.a_target, out_cap, v_cap, d_cap, self._last_reason)
            return self.a_target, out_cap

        self._shaped = self._shape(out_a, cfg)
        self.a_target = float(self._shaped)
        self.v_cruise_cap = out_cap
        self.should_stop = bool(out_stop)
        self._last_reason = "applied" if self.a_target < -0.01 else "idle"
        self._log(now, v_ego, out_a, self.a_target, out_cap, v_cap, d_cap, self._last_reason)
        return self.a_target, self.v_cruise_cap

    def _shape(self, target, cfg):
        max_d = abs(float(cfg.get("max_decel_ms2", 2.0)))
        t = _clip(_f(target), -max_d, 0.0)

        rise = abs(float(cfg.get("max_rise_ms2_per_s", 1.5))) * self.dt
        rel = abs(float(cfg.get("release_ms2_per_s", 3.0))) * self.dt

        if t <= self._shaped:
            self._shaped = max(t, self._shaped - rise)
        else:
            self._shaped = min(t, self._shaped + rel)
        return _clip(self._shaped, -max_d, 0.0)

    def _compute(self, v_ego, view, amap_limit, now, cfg):
        v_cruise_cap = None
        limit_source = "none"
        if bool(cfg.get("use_speed_limit", True)):
            offset = float(cfg.get("speed_limit_offset_kmh", 0.0)) / 3.6
            limit, limit_source = self._pick_speed_limit(view, amap_limit, cfg)
            if limit is not None and limit > 0.0:
                v_cruise_cap = max(limit + offset, 0.0)

        inf = float("inf")

        if not bool(cfg.get("use_profile", True)):
            return 0.0, v_cruise_cap, "profile_off", inf, inf, limit_source, False

        if view is None:
            return 0.0, v_cruise_cap, "no_profile", inf, inf, limit_source, False

        stale_s = float(cfg.get("profile_stale_sec", 2.5))
        if (now - view.stamp) > stale_s:
            return 0.0, v_cruise_cap, "profile_stale", inf, inf, limit_source, False

        lag = _clip(now - view.stamp, 0.0, stale_s)
        ego_progress = view.progress + max(v_ego, 0.0) * lag

        d = view.p_progress - ego_progress
        lookahead = float(cfg.get("lookahead_m", 300.0))
        mask = (d > -D_BACK_MARGIN) & (d <= lookahead) & (view.p_cap >= 0.0)
        if not np.any(mask):
            return 0.0, v_cruise_cap, "profile_empty", inf, inf, limit_source, False

        d_sel = d[mask]
        cap_sel = view.p_cap[mask]

        a_dec = max(float(cfg.get("a_dec_ms2", 0.8)), 0.1)
        v_allow = np.sqrt(np.maximum(cap_sel * cap_sel + 2.0 * a_dec * np.maximum(d_sel, 0.0), 0.0))

        i = int(np.argmin(v_allow))
        v_cap = float(v_allow[i])
        cap_i = float(cap_sel[i])
        d_cap = float(max(d_sel[i], D_MIN))

        a_needed = (cap_i * cap_i - v_ego * v_ego) / (2.0 * d_cap)
        a_target = min(a_needed, 0.0)

        should_stop = bool(cap_i < 0.5 and d_cap < STOP_D)

        return float(a_target), v_cruise_cap, "ok", v_cap, d_cap, limit_source, should_stop

    def _pick_speed_limit(self, view, amap_limit, cfg):
        has_amap = amap_limit is not None and amap_limit > 0.0
        has_mapbox = view is not None and view.speed_limit > 0.0

        if bool(cfg.get("amap_priority", True)):
            if has_amap:
                return amap_limit, "amap"
            if has_mapbox:
                return view.speed_limit, "mapbox"
        else:
            if has_mapbox:
                return view.speed_limit, "mapbox"
            if has_amap:
                return amap_limit, "amap"
        return None, "none"

    def _amap_speed_limit(self, amap_msg, now, amap_recv_time=None):
        if amap_msg is None:
            return None
        try:
            if not bool(getattr(amap_msg, "navValid", False)):
                return None
            if amap_recv_time is not None and amap_recv_time > 0.0:
                if (now - amap_recv_time) > float(self.cfg.get("amap_stale_sec", 3.0)):
                    return None
            raw = _f(getattr(amap_msg, "navSpeedLimit", -1.0), -1.0)
            if raw <= 0.0:
                return None

            self.amap_limit_raw = raw
            unit = str(self.cfg.get("amap_limit_unit", "auto")).lower()
            if unit == "ms":
                limit = raw
            elif unit == "kmh":
                limit = raw / 3.6
            else:
                thr = float(self.cfg.get("amap_kph_threshold", 45.0))
                limit = raw / 3.6 if raw > thr else raw
            return limit if limit > 0.0 else None
        except Exception:
            return None

    def _log(self, now, v_ego, a_raw, a_out, v_cap_cruise, v_cap, d_cap, reason):
        if not bool(self.cfg.get("log", True)):
            return
        if now - self._last_log_t < float(self.cfg.get("log_period_s", 1.0)):
            return
        self._last_log_t = now
        try:
            v_cap_s = "  inf" if not math.isfinite(v_cap) else f"{v_cap:5.1f}"
            d_cap_s = "  inf" if not math.isfinite(d_cap) else f"{d_cap:5.1f}"
            cap_s = "  --" if v_cap_cruise is None else f"{v_cap_cruise * 3.6:5.1f}"
            src_s = self.limit_source
            if src_s == "amap" and self.amap_limit_raw > 0.0:
                src_s = f"amap raw={self.amap_limit_raw:.1f}"
            line = (
                f"{time.strftime('%H:%M:%S')} "
                f"v={v_ego * 3.6:6.1f}kph "
                f"vcap={v_cap_s} dcap={d_cap_s} "
                f"lim={cap_s}kph({src_s}) "
                f"a_raw={a_raw:6.2f} a_out={a_out:6.2f} why={reason}\n"
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
            "a_target": float(self.a_target),
            "should_stop": bool(self.should_stop),
            "v_cruise_cap": self.v_cruise_cap,
            "v_cap": float(self.v_cap),
            "d_cap": float(self.d_cap),
            "limit_source": self.limit_source,
            "why": self._last_reason,
        }
