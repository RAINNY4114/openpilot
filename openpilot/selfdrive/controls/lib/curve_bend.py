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
    "bend_deg_min_display": 20.0,
    "bend_extend_m": 400.0,
}

# mtime 缓存，避免每帧 stat 两次
_CACHE = {"cfg": None, "mtime": None, "last_check": 0.0, "path": None}


def load_config(force: bool = False) -> dict:
    """读取 /data/ford_curve.json（热加载，按 mtime）。任何异常都退回默认值。"""
    now = time.monotonic()
    if (not force) and _CACHE["cfg"] is not None and (now - _CACHE["last_check"]) < 1.0:
        return _CACHE["cfg"]
    _CACHE["last_check"] = now

    cfg = dict(DEFAULTS)
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
