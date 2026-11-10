#!/usr/bin/env python3

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/ford_curve.json"

DEFAULTS = {
    "bend_deg_min": 45.0,
    "bend_deg_min_display": 12.0,
    "bend_deg_min_lat": 10.0,
    "bend_extend_m": 400.0,
    "use_map_bend": True,
    "map_bend_horizon_m": 400.0,
    "map_max_age_s": 5.0,
}

_CACHE = {"cfg": None, "mtime": None, "last_check": 0.0, "path": None}

def load_config(force: bool = False) -> dict:
    now = time.monotonic()
    if (not force) and _CACHE["cfg"] is not None and (now - _CACHE["last_check"]) < 1.0:
        return _CACHE["cfg"]
    _CACHE["last_check"] = now

    cfg = dict(_CACHE["cfg"]) if _CACHE["cfg"] is not None else dict(DEFAULTS)
    try:
        st = os.stat(CONFIG_PATH)
        if force or _CACHE["mtime"] != st.st_mtime:
            with open(CONFIG_PATH, "r") as f:
                user = json.load(f)
            if isinstance(user, dict):
                for k, v in user.items():
                    if k in cfg:
                        cfg[k] = v
            _CACHE["mtime"] = st.st_mtime
    except FileNotFoundError:
        _CACHE["mtime"] = None
    except Exception:
        pass

    _CACHE["cfg"] = cfg
    return cfg

def bend_deg_min() -> float:
    try:
        return float(load_config().get("bend_deg_min", DEFAULTS["bend_deg_min"]))
    except Exception:
        return float(DEFAULTS["bend_deg_min"])

def bend_extend_m() -> float:
    try:
        return float(load_config().get("bend_extend_m", DEFAULTS["bend_extend_m"]))
    except Exception:
        return float(DEFAULTS["bend_extend_m"])

def bend_deg_min_display() -> float:
    try:
        return float(load_config().get("bend_deg_min_display",
                                       DEFAULTS["bend_deg_min_display"]))
    except Exception:
        return float(DEFAULTS["bend_deg_min_display"])

def bend_deg_min_lat() -> float:
    try:
        return float(load_config().get("bend_deg_min_lat",
                                       DEFAULTS["bend_deg_min_lat"]))
    except Exception:
        return float(DEFAULTS["bend_deg_min_lat"])

def bend_weight(bend_deg: float, gate: float = None, ramp: float = 8.0) -> float:
    try:
        g = float(bend_deg_min_lat() if gate is None else gate)
        x = (float(bend_deg) - g) / max(float(ramp), 1e-3)
        x = 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
        return float(x * x * (3.0 - 2.0 * x))
    except Exception:
        return 0.0

def path_curvature(positions, turn_rates, v_preds, v_ego):
    pos = np.asarray(positions, dtype=float)
    tr = np.asarray(turn_rates, dtype=float)
    vp = np.asarray(v_preds, dtype=float)
    n = min(pos.size, tr.size, vp.size)
    if n < 2:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    pos, tr, vp = pos[:n], tr[:n], vp[:n]
    v_denom = np.clip(vp, max(1.0, float(v_ego) * 0.7), 100.0)
    signed = np.clip(tr / v_denom, -0.02, 0.02)
    return pos, np.abs(signed), signed

def cumulative_bend_deg(positions, abs_curvature, extend_m):
    try:
        s = np.asarray(positions, dtype=float)
        k = np.asarray(abs_curvature, dtype=float)
        n = min(s.size, k.size)
        s, k = s[:n], k[:n]
        if n < 2:
            return 0.0

        order = np.argsort(s, kind="stable")
        s, k = s[order], k[order]

        s_max = float(s[-1])
        ext = float(extend_m)
        if ext > s_max + 1e-6 and s_max > 1.0:
            step = 5.0
            extra = np.arange(s_max + step, ext + 1e-6, step)
            if extra.size:
                s = np.concatenate([s, extra])
                k = np.concatenate([k, np.full(extra.size, float(k[-1]))])

        ds = np.diff(s)
        bend_rad = float(np.sum(0.5 * (k[1:] + k[:-1]) * ds))
        if not math.isfinite(bend_rad):
            return 0.0
        return math.degrees(max(0.0, bend_rad))
    except Exception:
        return 0.0

def bend_from_model(model_v2, v_ego):
    try:
        pos = list(model_v2.position.x)
        tr = list(model_v2.orientationRate.z)
        vp = list(model_v2.velocity.x)
        if len(pos) < 2 or len(pos) != len(tr) or len(tr) != len(vp):
            return 0.0
        positions, abs_k, _ = path_curvature(pos, tr, vp, v_ego)
        return cumulative_bend_deg(positions, abs_k, bend_extend_m())
    except Exception:
        return 0.0

def curve_summary_from_model(model_v2, v_ego, k_enter: float, window_m: float):
    out = {"bend_deg": 0.0, "k_max": 0.0, "k_signed": 0.0, "dist_m": 0.0, "has_curve": False}
    try:
        pos = list(model_v2.position.x)
        tr = list(model_v2.orientationRate.z)
        vp = list(model_v2.velocity.x)
        if len(pos) < 2 or len(pos) != len(tr) or len(tr) != len(vp):
            return out
        positions, abs_k, signed_k = path_curvature(pos, tr, vp, v_ego)
        if positions.size < 2:
            return out

        out["bend_deg"] = cumulative_bend_deg(positions, abs_k, bend_extend_m())

        mask = positions <= float(window_m)
        if not np.any(mask):
            mask = np.ones_like(positions, dtype=bool)
        k_w = abs_k[mask]
        s_w = positions[mask]
        if k_w.size:
            idx = int(np.argmax(k_w))
            out["k_max"] = float(k_w[idx])
            out["k_signed"] = float(signed_k[mask][idx])
            out["dist_m"] = float(s_w[idx])

        trig = k_w >= float(k_enter)
        if np.any(trig):
            first = int(np.argmax(trig))
            out["dist_m"] = float(s_w[first])
            out["has_curve"] = True
        else:
            out["has_curve"] = False
    except Exception:
        pass
    return out

MEM_PARAMS_DIRS = ("/dev/shm/params/d", "/data/params/d")
MAP_REFRESH_S = 0.2
_MAP_CACHE = {"t": 0.0, "ready": False, "curv": [], "curv_age": None,
              "vel": [], "vel_age": None, "gps": None, "gps_age": None}

def use_map_bend() -> bool:
    try:
        return bool(load_config().get("use_map_bend", DEFAULTS["use_map_bend"]))
    except Exception:
        return bool(DEFAULTS["use_map_bend"])

def map_bend_horizon_m() -> float:
    try:
        return float(load_config().get("map_bend_horizon_m", DEFAULTS["map_bend_horizon_m"]))
    except Exception:
        return float(DEFAULTS["map_bend_horizon_m"])

def _map_max_age_s() -> float:
    try:
        return float(load_config().get("map_max_age_s", DEFAULTS["map_max_age_s"]))
    except Exception:
        return float(DEFAULTS["map_max_age_s"])

def _read_mem_param(key: str):
    now = time.time()
    for d in MEM_PARAMS_DIRS:
        p = os.path.join(d, key)
        try:
            with open(p, "r") as f:
                txt = f.read()
        except FileNotFoundError:
            continue
        except Exception:
            continue
        try:
            age = max(0.0, now - os.stat(p).st_mtime)
        except OSError:
            age = None
        return txt, age
    return None, None

def _parse_map_curvatures(txt):
    out = []
    if not txt:
        return out
    try:
        raw = json.loads(txt)
    except Exception:
        return out
    if not isinstance(raw, list):
        return out
    for e in raw:
        try:
            if isinstance(e, dict):
                k = float(e.get("curvature", 0.0))
                s = float(e.get("distance", 0.0))
            elif isinstance(e, (list, tuple)) and len(e) >= 2:
                k = float(e[0])
                s = float(e[1])
            else:
                continue
        except Exception:
            continue
        if math.isfinite(s) and math.isfinite(k):
            out.append((max(0.0, s), k))
    if out and max(abs(s) for s, _ in out) < 1.0 and max(abs(k) for _, k in out) > 1.0:
        out = [(k, s) for s, k in out]
    out.sort(key=lambda x: x[0])
    return out

def _parse_map_velocities(txt):
    out = []
    if not txt:
        return out
    try:
        raw = json.loads(txt)
    except Exception:
        return out
    if not isinstance(raw, list):
        return out
    for e in raw:
        if not isinstance(e, dict):
            continue
        try:
            lat = float(e.get("latitude", e.get("lat", float("nan"))))
            lon = float(e.get("longitude", e.get("lon", e.get("lng", float("nan")))))
        except Exception:
            continue
        if math.isfinite(lat) and math.isfinite(lon) and (abs(lat) > 1e-6 or abs(lon) > 1e-6):
            out.append((lat, lon))
    return out

def _parse_gps(txt):
    if not txt:
        return None
    try:
        raw = json.loads(txt)
        lat = float(raw.get("latitude", float("nan")))
        lon = float(raw.get("longitude", float("nan")))
    except Exception:
        return None
    if math.isfinite(lat) and math.isfinite(lon):
        return (lat, lon)
    return None

def _map_snapshot():
    now = time.monotonic()
    if _MAP_CACHE["ready"] and (now - _MAP_CACHE["t"]) < MAP_REFRESH_S:
        return _MAP_CACHE
    _MAP_CACHE["t"] = now
    _MAP_CACHE["ready"] = True

    txt, age = _read_mem_param("MapCurvatures")
    _MAP_CACHE["curv"], _MAP_CACHE["curv_age"] = _parse_map_curvatures(txt), age

    txt, age = _read_mem_param("MapTargetVelocities")
    _MAP_CACHE["vel"], _MAP_CACHE["vel_age"] = _parse_map_velocities(txt), age

    txt, age = _read_mem_param("LastGPSPosition")
    _MAP_CACHE["gps"], _MAP_CACHE["gps_age"] = _parse_gps(txt), age
    return _MAP_CACHE

_EARTH_R_M = 6373000.0
_DEG2RAD = math.pi / 180.0

def _haversine_m(lat_a, lon_a, lat_b, lon_b) -> float:
    ax, ay = lat_a * _DEG2RAD, lon_a * _DEG2RAD
    bx, by = lat_b * _DEG2RAD, lon_b * _DEG2RAD
    h = math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2
    return 2.0 * _EARTH_R_M * math.atan2(math.sqrt(h), math.sqrt(max(1e-12, 1.0 - h)))

def _circumcircle_kappa(p1, p2, p3):
    a = _haversine_m(p2[0], p2[1], p3[0], p3[1])
    b = _haversine_m(p1[0], p1[1], p3[0], p3[1])
    c = _haversine_m(p1[0], p1[1], p2[0], p2[1])
    s = (a + b + c) / 2.0
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0.0:
        return 0.0
    area = math.sqrt(area_sq)
    if area <= 1e-9 or a <= 1e-6 or b <= 1e-6 or c <= 1e-6:
        return 0.0
    radius = (a * b * c) / (4.0 * area)
    if radius <= 0.0:
        return 0.0
    k = 1.0 / radius

    lat0 = p2[0] * _DEG2RAD
    m_per_deg_lat = _EARTH_R_M * _DEG2RAD
    m_per_deg_lon = _EARTH_R_M * _DEG2RAD * math.cos(lat0)
    d1x = (p2[1] - p1[1]) * m_per_deg_lon
    d1y = (p2[0] - p1[0]) * m_per_deg_lat
    d2x = (p3[1] - p2[1]) * m_per_deg_lon
    d2y = (p3[0] - p2[0]) * m_per_deg_lat
    cross = d1x * d2y - d1y * d2x
    if cross > 0.0:
        k = -k
    return k

def _kappa_from_polyline(points, ego):
    if len(points) < 3:
        return []
    if ego is not None:
        dists = [_haversine_m(ego[0], ego[1], p[0], p[1]) for p in points]
        i0 = int(min(range(len(dists)), key=lambda i: dists[i]))
        fwd = points[i0:]
        if i0 > 0:
            fwd = [points[i0 - 1]] + fwd
    else:
        fwd = list(points)
    if len(fwd) < 3:
        return []

    s = [0.0]
    for i in range(1, len(fwd)):
        s.append(s[-1] + _haversine_m(fwd[i - 1][0], fwd[i - 1][1], fwd[i][0], fwd[i][1]))

    out = []
    for i in range(1, len(fwd) - 1):
        k = _circumcircle_kappa(fwd[i - 1], fwd[i], fwd[i + 1])
        if math.isfinite(k):
            out.append((s[i], k))
    return out

def _summarize_kappa(pts, window_m: float, k_enter: float):
    out = {"bend_deg": 0.0, "k_max": 0.0, "k_signed": 0.0, "dist_m": 0.0, "has_curve": False}
    if not pts:
        return out
    try:
        s = np.array([p[0] for p in pts], dtype=float)
        k = np.array([p[1] for p in pts], dtype=float)
        if s.size < 2:
            return out
        order = np.argsort(s, kind="stable")
        s, k = s[order], k[order]

        s_cap = float(min(max(30.0, window_m), float(s[-1])))
        if s_cap <= 1.0:
            return out

        grid = np.arange(0.0, s_cap, 2.0)
        if grid.size < 2 or grid[-1] < s_cap - 1e-9:
            grid = np.append(grid, s_cap)
        kk = np.interp(grid, s, k)

        out["bend_deg"] = cumulative_bend_deg(grid, np.abs(kk), 0.0)

        idx = int(np.argmax(np.abs(kk)))
        out["k_max"] = float(abs(kk[idx]))
        out["k_signed"] = float(kk[idx])
        out["dist_m"] = float(grid[idx])

        trig = np.abs(kk) >= float(k_enter)
        if np.any(trig):
            first = int(np.argmax(trig))
            out["dist_m"] = float(grid[first])
            out["has_curve"] = True
    except Exception:
        pass
    return out

def map_curve_summary(v_ego: float = 0.0, window_m: float = None, k_enter: float = 4e-3) -> dict:
    out = {"bend_deg": 0.0, "k_max": 0.0, "k_signed": 0.0, "dist_m": 0.0,
           "has_curve": False, "src": None, "age_s": None, "n": 0}
    if not use_map_bend():
        return out
    w = float(map_bend_horizon_m() if window_m is None else window_m)
    max_age = _map_max_age_s()

    try:
        snap = _map_snapshot()
    except Exception:
        return out

    curv, c_age = snap.get("curv") or [], snap.get("curv_age")
    if curv and (c_age is None or c_age <= max_age):
        s = _summarize_kappa(curv, w, k_enter)
        s.update(src="curvatures", age_s=c_age, n=len(curv))
        return s

    vel, v_age = snap.get("vel") or [], snap.get("vel_age")
    gps, g_age = snap.get("gps"), snap.get("gps_age")
    if vel and (v_age is None or v_age <= max_age):
        if gps is not None and (g_age is None or g_age <= max_age):
            pts = _kappa_from_polyline(vel, gps)
        else:
            pts = _kappa_from_polyline(vel, None)
        if pts:
            s = _summarize_kappa(pts, w, k_enter)
            s.update(src="polyline", age_s=v_age, n=len(pts))
            return s

    return out

