#!/usr/bin/env python3

import json
import os
import time

CONFIG_PATH = "/data/aa_brake_apply.json"
REQ_PARAM_PATHS = ("/data/ford_params/AvoidBrakeReq", "/dev/shm/params/AvoidBrakeReq", "/data/params/d/AvoidBrakeReq")
REQ_PARAM_PATH = REQ_PARAM_PATHS[-1]
LOG_PATH = "/data/media/0/aa_brake_apply.log"

DEFAULTS = {
    "enabled": False,
    "observe_only": True,

    "log": True,
    "log_period_s": 1.0,

    "max_decel_ms2": 3.0,
    "max_rise_ms2_per_s": 2.0,
    "gain": 1.0,

    "v_min_kmh": 25.0,
    "req_stale_sec": 0.5,
}

def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

class AABrakeApply:

    def __init__(self):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self._shaped = 0.0
        self._req = 0.0
        self._req_t = 0.0
        self._req_stamp = None
        self._req_content_prev = None
        self._last_req_raw = None
        self._last_log_t = 0.0
        self._last_reason = "init"
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
                    if k in cfg and not isinstance(cfg[k], dict):
                        cfg[k] = v
                    elif isinstance(v, dict) and isinstance(cfg.get(k), dict):
                        cfg[k].update(v)
            self.cfg = cfg
            self._cfg_mtime = st.st_mtime
        except Exception:
            if force:
                self.cfg = dict(DEFAULTS)
                self._cfg_mtime = None

    def _read_req(self, now):
        raw = None
        for _p in REQ_PARAM_PATHS:
            try:
                with open(_p, "r") as f:
                    raw = f.read().strip()
                if raw:
                    break
            except Exception:
                raw = None

        if not raw:
            self._last_req_raw = None
            self._req_stamp = None
            return 0.0, now

        self._last_req_raw = raw

        val_s, stamp_s = raw, None
        if ":" in raw:
            parts = raw.split(":")
            if len(parts) >= 2:
                val_s = parts[0]
                stamp_s = parts[1]

        if stamp_s is not None:
            if stamp_s != self._req_stamp:
                self._req_stamp = stamp_s
                self._req_t = now
        else:
            if raw != self._req_content_prev:
                self._req_content_prev = raw
                self._req_t = now

        try:
            v = float(val_s)
        except (TypeError, ValueError):
            return 0.0, now

        if v != v or v < 0.0:
            return 0.0, now
        return v, self._req_t

    def update(self, *, v_ego, long_active, now=None):
        try:
            return self._update(v_ego, long_active, now)
        except Exception:
            self._shaped = 0.0
            self._last_reason = "exception"
            return 0.0

    def _update(self, v_ego, long_active, now=None):
        if now is None:
            now = time.monotonic()

        self._load_config()

        cfg = self.cfg
        enabled = bool(cfg.get("enabled", False))
        observe_only = bool(cfg.get("observe_only", True))

        if not enabled or observe_only:
            self._shaped = 0.0
            self._last_reason = "observe_only" if enabled else "disabled"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        if not bool(long_active):
            self._shaped = 0.0
            self._last_reason = "long_inactive"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        v_min = float(cfg.get("v_min_kmh", 25.0)) / 3.6
        if v_ego < v_min:
            self._shaped = 0.0
            self._last_reason = "v_low"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        req, req_t = self._read_req(now)

        stale_s = float(cfg.get("req_stale_sec", 0.5))
        if req > 0.0 and (now - req_t) > stale_s:
            self._shaped = 0.0
            self._last_reason = "req_stale"
            self._log(now, v_ego, req, 0.0)
            return 0.0

        gain = float(cfg.get("gain", 1.0))
        max_d = abs(float(cfg.get("max_decel_ms2", 3.0)))
        target = -_clip(gain * req, 0.0, max_d)

        dt = 1.0 / 20.0
        try:
            from openpilot.common.realtime import DT_CTRL as _DT
            dt = float(_DT)
        except Exception:
            pass
        rise = abs(float(cfg.get("max_rise_ms2_per_s", 2.0))) * max(dt, 1e-3)

        if target <= self._shaped:
            self._shaped = max(target, self._shaped - rise)
        else:
            self._shaped = target

        out = _clip(self._shaped, -max_d, 0.0)
        self._last_reason = "applied" if out < 0.0 else "idle"
        self._log(now, v_ego, req, out)
        return float(out)

    def _log(self, now, v_ego, req, out):
        if not bool(self.cfg.get("log", True)):
            return
        period = float(self.cfg.get("log_period_s", 1.0))
        if now - self._last_log_t < period:
            return
        self._last_log_t = now
        try:
            line = (
                f"{time.strftime('%H:%M:%S')} "
                f"v={v_ego * 3.6:6.1f}kph req={req:5.2f} out={out:6.2f} "
                f"why={self._last_reason}\n"
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
            "out": float(self._shaped),
            "req": float(self._last_req_raw or 0.0) if self._last_req_raw else 0.0,
            "why": self._last_reason,
        }
