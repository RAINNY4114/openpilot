#!/usr/bin/env python3
"""
curve_bend.py — 弯道"累计转角"计算 + 配置（planner 与 UI 共用）

为什么单独一个模块
------------------
纵向弯道减速（selfdrive/controls/lib/ford_curve_speed.py）和 HUD 弯道标志
（selfdrive/ui/onroad/hud_renderer.py）必须用**完全相同**的弯道判定，否则会出现
"标志亮了但没减速" / "减速了但没标志" 的不一致。这里放唯一实现，两边 import。

配置为什么不用 param
--------------------
openpilot 的 Params::clearAll() 会删除**所有不在已编译 params_keys.h 里的 key**
（openpilot/common/params.cc:208-227：

    auto it = keys.find(de->d_name);
    if (it == keys.end() || (it->second.flags & key_flag)) unlink(...);

）。本机 libparams_c.so 是预编译的、无法在设备上重建，所以**新增的 param key 一定会
在 manager 启动 / onroad / offroad / 点火时被清掉**（这正是 2026-09-28 盲区语音那次
路试问题的根因）。因此这里用**普通 JSON 文件**保存配置，manager 不会碰它。

配置文件：/data/ford_curve.json
  {
    "bend_deg_min": 45.0,    # 累计转角达到多少度才算"弯道"（触发标志 + 弯道减速检查）
    "bend_extend_m": 400.0   # 向前累计到多远；超出模型预测终点后按末点曲率恒定外推
  }

数值参考（沿预测路径的累计航向变化 ∫|κ|ds；恒定半径下 ≈ 弧长/半径）
  半径 1000 m 走 250 m -> 14°        半径 400 m 走 250 m -> 36°
  半径  500 m 走 393 m -> 45°        半径 250 m 走 196 m -> 45°
  => bend_deg_min = 45° 在中速弯道（大致 R ≤ 510 m）就会触发，
     比 100°（R ≤ 229 m）/ 95°（R ≤ 241 m）灵敏得多：高速上的缓弯也会亮标志、
     并进入弯道减速检查。若发现高速上频繁误触发，把 bend_deg_min 调大即可（热生效）。
"""

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/ford_curve.json"

# Two separate gates -- do NOT collapse them back into one number.
#
#   bend_deg_min          control gate: whether to do curve deceleration
#                         (up to -3.2 m/s^2). Changing it changes braking.
#   bend_deg_min_display  display gate: whether the HUD curve sign appears.
#                         Independent and lower, so gentle curves get a hint
#                         WITHOUT making the car brake more.
#
# Measured over 116 routes (peak cumulative turn angle per route):
#   >= 10 deg : 76.7%    >= 20 deg : 50.0%    >= 30 deg : 25.9%    >= 45 deg : 15.5%
# and of the 102 highway routes (90-105 km/h) only 4 ever reached 45 deg.
# Sharing one 45 deg gate is why the sign "never triggered" on the highway.
DEFAULTS = {
    "bend_deg_min": 45.0,
    # 2026-10-18: 20 -> 12 度。实测高速路段的弯角很少到 20 度（190 m 窗、
    # k_enter 门限下），弯道标志因此"从来不亮"。12 度只影响 HUD 图标是否显示，
    # 不影响 dp_lincoln_curve_speed / TSC 的控制闸门（仍用 bend_deg_min）。
    "bend_deg_min_display": 12.0,
    # 横向前瞻混合（FordPlanLead）单独一个闸门，默认 10 度。
    # 纵向减速仍用 bend_deg_min（默认 45 度），两者互不影响：
    # 缓弯里不需要减速，但横向确实需要提前打方向补延迟。
    "bend_deg_min_lat": 10.0,
    "bend_extend_m": 400.0,
    # ---- 地图弯道（OSM / mapd）[CURVE_MAP_20261027] ----
    # 地图源来自 mapd 每秒写入 /dev/shm/params 的 MapCurvatures /
    # MapTargetVelocities。只影响 HUD 是否显示"地图弯角"，不参与任何控制。
    # 键名沿用 /data/ford_curve.json 里已有的 use_map_bend / map_bend_horizon_m，
    # 不要改名 —— 那份配置是历史遗留、已经写在车上了。
    "use_map_bend": True,
    "map_bend_horizon_m": 400.0,
    "map_max_age_s": 5.0,
}

# mtime 缓存，避免每帧 stat 两次
_CACHE = {"cfg": None, "mtime": None, "last_check": 0.0, "path": None}


def load_config(force: bool = False) -> dict:
    """读取 /data/ford_curve.json（热加载，按 mtime）。任何异常都退回默认值。"""
    now = time.monotonic()
    if (not force) and _CACHE["cfg"] is not None and (now - _CACHE["last_check"]) < 1.0:
        return _CACHE["cfg"]
    _CACHE["last_check"] = now

    # CHX_CURVE_BEND_CACHE: 必须沿用上次读到的配置。本函数每 1 秒刷新一次，
    # 若 mtime 未变则跳过读文件；此时若从 DEFAULTS 重建，
    # 用户配置（如 bend_deg_min=22）会在 1 秒后被打回 45。
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
        # 配置损坏 -> 用默认值，绝不把异常抛出去
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
    """HUD 弯道标志的显示闸门 —— 与控制闸门 `bend_deg_min` 分离。

    控制侧用 `bend_deg_min`（默认 45 度）决定是否做弯道减速，改它会改变刹车；
    显示侧用这个更低的值（默认 20 度），让缓弯也能给出提示而**不加剧刹车**。

    参照 RAINNY4114/openpilot @ ford：标志应该提示"前方有值得注意的弯"，
    而不是等到弯道减速控制器已经被触发才亮。
    """
    try:
        return float(load_config().get("bend_deg_min_display",
                                       DEFAULTS["bend_deg_min_display"]))
    except Exception:
        return float(DEFAULTS["bend_deg_min_display"])


def bend_deg_min_lat() -> float:
    """横向前瞻混合（FordPlanLead）的弯道闸门 —— 与纵向减速闸门分离。

    纵向用 bend_deg_min（默认 45 度）决定要不要减速；横向只需要"前方确实
    在转弯"就该提前打方向。实测高速缓弯（8~20 度累计转角）占比很高，用
    45 度闸门时横向前瞻只有约 8% 的时间能介入，等于没开。

    配置写在 /data/ford_curve.json 的 "bend_deg_min_lat"，热生效。
    """
    try:
        return float(load_config().get("bend_deg_min_lat",
                                       DEFAULTS["bend_deg_min_lat"]))
    except Exception:
        return float(DEFAULTS["bend_deg_min_lat"])


def bend_weight(bend_deg: float, gate: float = None, ramp: float = 8.0) -> float:
    """累计转角 -> 0..1 软权重（smoothstep）。

    硬闸门在阈值处会突变：横向前瞻一开，混合项立刻从 0 跳到 b*(model-base)，
    等效于给转向一个阶跃。这里让权重在 [gate, gate+ramp] 之间平滑爬升，
    闸门附近不再有台阶。
    """
    try:
        g = float(bend_deg_min_lat() if gate is None else gate)
        x = (float(bend_deg) - g) / max(float(ramp), 1e-3)
        x = 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
        return float(x * x * (3.0 - 2.0 * x))
    except Exception:
        return 0.0


def path_curvature(positions, turn_rates, v_preds, v_ego):
    """把模型输出换成 (s, |κ|, 带符号 κ)。

    与 ford_curve_speed.py 原来的算法完全一致：
        v_denom = clip(v_pred, max(1, 0.7*v_ego), 100)
        κ = orientationRate.z / v_denom
        clip 到 ±0.02 (1/m)
    """
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
    """∫|κ| ds，单位：度 —— 沿预测路径的累计航向变化（= 弯道"总转角"）。

    超出模型预测终点后按**末点曲率恒定**外推，最多到 extend_m 米。
    这样长弯（曲率一直存在、超出模型 10 s 视野）也能把转角累计完整。
    """
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
        # 手工梯形积分：兼容所有 numpy 版本（np.trapz 在 numpy>=2 已移除）
        bend_rad = float(np.sum(0.5 * (k[1:] + k[:-1]) * ds))
        if not math.isfinite(bend_rad):
            return 0.0
        return math.degrees(max(0.0, bend_rad))
    except Exception:
        return 0.0


def bend_from_model(model_v2, v_ego):
    """直接从 modelV2 算累计转角（度）。任何异常返回 0.0。"""
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
    """给 HUD 用的一次性汇总。

    返回 dict:
      bend_deg   累计转角（度）—— 与 planner 的闸门用的是同一个数
      k_max      窗口内最大 |κ| (1/m)
      k_signed   最大 |κ| 处的带符号 κ（>0 = 右转）
      dist_m     第一次 |κ| >= k_enter 的距离 (m)；没找到 -> 窗口末端
      has_curve  窗口内是否存在 |κ| >= k_enter
    """
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
            # 没到 k_enter，把"最大曲率所在距离"当作最近的可疑点
            out["has_curve"] = False
    except Exception:
        pass
    return out


# ============================================================================
# 地图弯道  [CURVE_MAP_20261027]
# ----------------------------------------------------------------------------
# 数据从哪来
# ----------
# 本机 sunnypilot 的 mapd（third_party/mapd_pfeiferj/mapd，进程 mapd +
# mapd_manager）已经把中国 OSM 离线库（/data/media/0/osm/offline，5.8 GB，
# OsmLocal=True / OsmLocationName=CN）解析好，并且**每秒**把结果写进
# /dev/shm/params/d/ 下的内存参数：
#
#   MapCurvatures          [{"curvature": κ(1/m), "distance": s(m)}, ...]
#   MapTargetVelocities    [{"latitude","longitude","velocity","distance"}, ...]
#   LastGPSPosition        {"latitude","longitude"}
#   MapSpeedLimit / NextMapSpeedLimit / MapAdvisoryLimit / RoadName ...
#
# 为什么直接读文件而不是 Params
# ----------------------------
# Params 的内存目录要走 libparams_c.so 的 key 白名单；而且 clearAll()
# （params.cc:208-227）会删掉未编译进 params_keys.h 的 key。直接按文件读，
# 既不受白名单影响，也不会被 clearAll 波及（与 /data/ford_curve.json 同思路）。
#
# 怎么算"弯道角度"
# ----------------
# 和视觉侧完全同一个物理量：沿路的累计航向变化 ∫|κ| ds（弧度 -> 度）。
# 所以两边可以直接对比：半径 300 m 的弯走 250 m，两边都该读出 ≈47.7°。
#
# 两路来源，谁有用谁
# ------------------
#   1. MapCurvatures        直接给 κ(s)，最干净（首选）
#   2. MapTargetVelocities  只给经纬度折线，用三点外接圆求 κ（ford 分支
#                           _fp_map_curvature 的同一套几何），再用
#                           LastGPSPosition 定"我当前在哪一段"
#
# 只在 HUD 上显示，绝不参与控制：bend_deg_min（45° 控制闸门）不受影响。
# ============================================================================

MEM_PARAMS_DIRS = ("/dev/shm/params/d", "/data/params/d")
MAP_REFRESH_S = 0.2          # 5 Hz 读盘上限，UI 20 fps 也不会打爆文件系统
_MAP_CACHE = {"t": 0.0, "ready": False, "curv": [], "curv_age": None,
              "vel": [], "vel_age": None, "gps": None, "gps_age": None}


def use_map_bend() -> bool:
    try:
        return bool(load_config().get("use_map_bend", DEFAULTS["use_map_bend"]))
    except Exception:
        return bool(DEFAULTS["use_map_bend"])


def map_bend_horizon_m() -> float:
    """地图弯道的累计视野（米）。默认 400 = 视觉 bend_extend_m，保证同口径。"""
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
    """读一个内存参数文件。返回 (text, age_s)；读不到返回 (None, None)。

    注意：age 必须用 time.time() 算（st_mtime 是墙上时钟），
    不能用 time.monotonic()，否则两个时基相减得到天文数字。
    """
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
    """MapCurvatures -> [(s, κ)]，按 s 升序。容忍 dict / 二元列表两种写法。"""
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
        # 列序被写反了：distance 全是米级、curvature 全是 1e-3 级。
        # 反过来之后"distance 列" < 1 m、"curvature 列" > 1 1/m（半径 < 1 m，
        # 物理上不存在），据此识别并换回来。
        out = [(k, s) for s, k in out]
    out.sort(key=lambda x: x[0])
    return out


def _parse_map_velocities(txt):
    """MapTargetVelocities -> [(lat, lon)]（只要几何，速度值这里用不到）。"""
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
    """5 Hz 缓存的一次性快照（UI 20 fps 调用也只读 5 次/秒）。"""
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
    """三点外接圆曲率 1/R，带符号（>0 = 右转，与视觉侧同约定）。

    与 ford 分支 longitudinal_planner._fp_calc_curvature 同一套几何（海伦公式
    -> 面积 -> R = abc/4A），只是这里多带一个叉积定方向。
    """
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

    # 方向：局部东-北平面里的叉积。东为 +x、北为 +y 时，顺时针（右转）
    # 的叉积 < 0，所以取负号让"右转 = 正"，与视觉侧一致。
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
    """经纬度折线 + 自车位置 -> [(s, κ)]，s 是从折线起点累计的米数。"""
    if len(points) < 3:
        return []
    if ego is not None:
        dists = [_haversine_m(ego[0], ego[1], p[0], p[1]) for p in points]
        i0 = int(min(range(len(dists)), key=lambda i: dists[i]))
        fwd = points[i0:]
        # 距自车最近点之前的那一点也保留，避免刚好卡在段边界时只剩 2 点
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
    """[(s, κ)] -> (bend_deg, k_max, k_signed, dist_m, has_curve)。"""
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

        # 视野上限：不超过地图数据实际覆盖的范围（绝不凭空外推）
        s_cap = float(min(max(30.0, window_m), float(s[-1])))
        if s_cap <= 1.0:
            return out

        # 重采样到 2 m 网格再积分。两个原因：
        #   1. 地图点间距不固定（MapCurvatures 约 5 m、折线 10 m 起）；
        #   2. np.interp 在 [0, s[0]] 区间会用 k[0] 填满，正好补上"第一个采样
        #      点之前那 0~10 m"，否则窗口会漏掉首尾各一个间隔（实测差 4 度）。
        grid = np.arange(0.0, s_cap, 2.0)
        if grid.size < 2 or grid[-1] < s_cap - 1e-9:
            grid = np.append(grid, s_cap)
        kk = np.interp(grid, s, k)

        # 与视觉侧同一个积分器：∫|κ|ds（不外推）
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
    """地图弯道汇总（只给 HUD 用，不参与控制）。

    返回 dict:
      bend_deg   地图累计转角（度），与视觉侧 bend_deg 同一定义
      k_max      窗口内最大 |κ| (1/m)
      k_signed   最大 |κ| 处的带符号 κ（>0 = 右转）
      dist_m     第一个 |κ| >= k_enter 的距离 (m)
      has_curve  窗口内是否存在 |κ| >= k_enter
      src        数据来源 'curvatures' / 'polyline' / None
      age_s      该数据源的写入延迟（秒）
      n          参与计算的点数
    """
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

    # ---- 首选：MapCurvatures 直接给 κ(s) ----
    curv, c_age = snap.get("curv") or [], snap.get("curv_age")
    if curv and (c_age is None or c_age <= max_age):
        s = _summarize_kappa(curv, w, k_enter)
        s.update(src="curvatures", age_s=c_age, n=len(curv))
        return s

    # ---- 兜底：MapTargetVelocities 折线 + LastGPSPosition ----
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

