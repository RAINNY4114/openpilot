#!/usr/bin/env python3
"""
nav_curvature_blend.py —— 导航曲率的「横向弱融合」（[NAV_LAT_FALLBACK]）

定位
----
**不是**用导航替代视觉模型，而是当视觉模型和导航在地图上的走向**一致**
且弯道**很缓**时，把期望曲率往导航方向轻轻拉一点点，抑制长直路/大半径
弯道上的缓慢漂移。

一旦两者分歧变大（模型认为要转、地图认为是直路，或反之），说明其中一方
不可信 —— 此时**什么都不做**，原样返回模型的曲率。这是本模块唯一的安全
假设：只在"两个独立来源互相印证"时才介入。

与本机既有模块的关系
--------------------
对齐 `aa_brake_apply.py` / `nav_speed_planner.py` 的范式：
  - 输出"修正量"，由 controlsd 叠加到主链路，**不替代**主链路
  - 默认**关闭**；`observe_only=true` 时恒返原值，只写日志
  - 配置热加载（按 mtime）；异常一律降级为"原样返回"
  - 权重限幅 + 修正量限幅 + 速率整形

安全设计（横向是安全关键通道，约束比纵向更严）
----------------------------------------------
1. **默认关闭**：配置缺失 / enabled!=true -> 直接返回模型曲率。
2. **权重限幅**：`weight <= weight_max`（默认 0.25），且最终修正量
   `|delta| <= max_delta_curvature`（默认 0.0015 1/m ≈ 半径 667 m 的曲率）。
3. **只在大半径弯道介入**：`|k_nav| <= max_nav_curvature`（默认 1/150 1/m）。
   急弯一律不碰 —— 那是模型和驾驶员监控的领域。
4. **一致性闸门**：`|k_model - k_nav| <= max_diff` 才介入。分歧即退出。
5. **速率整形**：修正量按 `max_delta_rate_per_s` 爬升/释放，不出现阶跃。
6. **闸门**：横向未激活 / 车速过低 / 剖面过期 / 剖面无效 -> 原样返回。
7. **异常安全**：任何异常 -> 原样返回。

输出契约
--------
    blend(...) -> curvature   # 与入参 model_curvature 同量纲、同符号约定
                              # 正 = 左转（openpilot 约定）

配置文件：/data/nav_lat_blend.json
日志：    /data/media/0/nav_lat_blend.log
"""

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/nav_lat_blend.json"
LOG_PATH = "/data/media/0/nav_lat_blend.log"

# 曲率半径小于该值视为"急弯"，一律不介入（m）
R_MIN_ANY = 150.0

DEFAULTS = {
    # ---- 总开关 ----
    "enabled": False,
    # ---- 观测模式：true 时恒返原值，只写日志 ----
    "observe_only": True,

    # ---- 日志 ----
    "log": True,
    "log_period_s": 1.0,

    # ---- 权重 / 限幅 ----
    # 融合权重（0 = 完全用模型，1 = 完全用导航）
    "weight": 0.25,
    "weight_max": 0.25,
    # 单帧修正量上限（1/m）
    "max_delta_curvature": 0.0015,
    # 修正量变化速率上限（1/m per s）
    "max_delta_rate_per_s": 0.002,

    # ---- 介入条件 ----
    # 只在大半径弯道介入：|k_nav| 上限（1/m）
    "max_nav_curvature": 1.0 / R_MIN_ANY,
    # 模型与导航曲率允许的最大分歧（1/m），超过则不介入
    "max_diff": 0.008,
    # 取导航剖面中「本车前方多远」的曲率做比较（m）
    "preview_m": 15.0,
    # 车速下限（km/h）
    "v_min_kmh": 20.0,
    # 剖面过期阈值（s）
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
    """从 navProfile 取「本车前方 distance_ahead 米」处的曲率。

    与 `nav_speed_planner` 一致，用 v_ego 外推补掉 1 Hz 发布的滞后。
    不可用返回 None。
    """
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
    """导航曲率的弱融合器。输出 = 模型曲率 + 受限修正量。"""

    def __init__(self, dt=0.01):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self.dt = max(float(dt), 1e-3)

        self._delta = 0.0           # 当前整形后的修正量（1/m）
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

    # ------------------------------------------------------------------
    def blend(self, v_ego, lat_active, model_curvature, profile_msg, now=None):
        """返回融合后的曲率。任何异常 -> model_curvature。"""
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

        # ---- 闸门 ----
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
            # 释放：按速率把修正量收回 0（安全方向 = 回到纯模型曲率）
            rel = abs(float(cfg.get("max_delta_rate_per_s", 0.002))) * self.dt
            self._delta -= math.copysign(min(rel, abs(self._delta)), self._delta) if self._delta != 0.0 else 0.0
            self.last_delta = self._delta
            self._last_reason = reason
            self._log(now, v_ego, model_curvature, k_nav, 0.0, self._delta, reason)
            return model_curvature + self._delta

        # ---- 融合 ----
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

    # ------------------------------------------------------------------
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
