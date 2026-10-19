#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_trim_governor.py — 横向叠加修正的统一整形器

为什么需要它
------------
本机横向链路是「端到端模型直出曲率」：

    modelV2.action.desiredCurvature
      + FordPlanLead（前瞻混合）
      + lateral_clearance（居中微调 / 间距避让）
      + curve_lane_bias（弯道内圈偏移）
      + lateral_planner_avoid（静止障碍避让）
      -> clip_curvature -> FordCurveController -> CAN

这四路是各自独立的「位置环」，各有自己的积分器和饱和限幅，但作用于
**同一个** 车道中心误差。实车表现：居中变好了，但车道内左右摇晃，越快越厉害。

三个机理：
  1) 多路积分器并联：总积分增益翻倍，且各自饱和点不同 -> 低频极限环；
  2) 偏置目标存在阶跃：护栏/邻车/内圈偏置都会在 0.1~0.3 m 上跳变，
     低通系数 0.10~0.15 @100Hz 意味着 70~100 ms 就完成跳变 -> 直接给转向
     一个阶跃扰动；
  3) 位置环增益固定：带宽 ∝ 车速，速度越高相位裕度越低 -> 越快越厉害。

本模块只做三件「整形」的事，不改变任何模块的语义：
    低通 (lp_hz)  ->  速率限制 (rate_max)  ->  幅值限制 (max_trim)

安全设计
--------
- 默认 enabled=true，但配置缺失时用保守默认值；任何异常都返回 0
- 未激活 / 低速 / 非有限值 -> 清零并复位
- 配置热加载（/data/lane_trim.json，按 mtime）
"""

import json
import math
import os
import time

CONFIG_PATH = "/data/lane_trim.json"

DEFAULTS = {
    "enabled": True,
    "lp_hz": 0.60,          # 一阶低通截止频率（Hz）：去掉 1~3 Hz 车道线噪声
    "rate_max": 2.5e-4,     # 输出最大变化率 (1/m)/s：任何阶跃都过不去
    "max_trim": 5.0e-4,     # 总修正幅值上限 (1/m)
    "min_speed_ms": 3.0,    # 低于此速度不介入
    # 速度自适应收缩（2026-10-19）：rate_max/max_trim 按 (v_ref/v)^2 收缩。
    # 修正量是曲率量纲，高速时等效 a_lat 按 v^2 放大。0 = 关闭。
    "speed_scale_v_ref": 0.0,
    "speed_scale_floor": 0.30,
    "log": False,
}


class LaneTrimGovernor:
    """把多路横向修正合并成一路并整形。update() 永远返回有限 float。"""

    def __init__(self, config_path: str = CONFIG_PATH):
        self.config_path = config_path
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self._last_cfg_check = 0.0
        self.out = 0.0
        self.raw = 0.0
        self._last_log_t = 0.0
        self._load_config(force=True)

    # ------------------------------------------------------------------ config
    def _load_config(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_cfg_check) < 1.0:
            return
        self._last_cfg_check = now
        try:
            st = os.stat(self.config_path)
            if not force and self._cfg_mtime is not None and st.st_mtime == self._cfg_mtime:
                return
            with open(self.config_path, "r") as f:
                user = json.load(f)
            cfg = dict(DEFAULTS)
            if isinstance(user, dict):
                for k, v in user.items():
                    if k in cfg:
                        cfg[k] = v
            self.cfg = cfg
            self._cfg_mtime = st.st_mtime
        except FileNotFoundError:
            self.cfg = dict(DEFAULTS)
            self._cfg_mtime = None
        except Exception:
            pass

    def _f(self, key: str, default: float) -> float:
        try:
            return float(self.cfg[key])
        except Exception:
            return float(default)

    def _b(self, key: str, default: bool) -> bool:
        try:
            return bool(self.cfg[key])
        except Exception:
            return bool(default)

    def reset(self) -> None:
        self.out = 0.0
        self.raw = 0.0

    # ------------------------------------------------------------------ update
    def update(self, trim_raw: float, v_ego: float, active: bool, dt: float) -> float:
        try:
            self._load_config()
            if (not self._b("enabled", True)) or (not active) or \
               (float(v_ego) < self._f("min_speed_ms", 3.0)):
                self.reset()
                return 0.0

            x = float(trim_raw)
            if not math.isfinite(x):
                self.reset()
                return 0.0
            self.raw = x

            dt = max(1e-3, min(0.1, float(dt)))

            # ---- 速度自适应收缩（2026-10-19）----
            # 修正量是曲率量纲：同样幅值的修正，高速时等效横向加速度按
            # v^2 放大。速率与幅值上限一并按 (v_ref/v)^2 收缩，避免高速时
            # 修正项把车推来推去。v_ref<=0 时 _sc=1.0，与原行为逐位一致。
            _sc = 1.0
            _vref = max(0.0, self._f("speed_scale_v_ref", 0.0))
            _v = float(v_ego)
            if _vref > 0.0 and _v > 0.5:
                _sc = max(self._f("speed_scale_floor", 0.30), min(1.0, (_vref / _v) ** 2))

            # ---- 1) 低通 -----------------------------------------------------
            fc = max(0.02, min(10.0, self._f("lp_hz", 0.60)))
            a = 1.0 - math.exp(-2.0 * math.pi * fc * dt)
            y = self.out + a * (x - self.out)

            # ---- 2) 速率限制 -------------------------------------------------
            rate = max(0.0, self._f("rate_max", 2.5e-4)) * _sc
            if rate > 0.0:
                step = rate * dt
                y = max(self.out - step, min(self.out + step, y))

            # ---- 3) 幅值限制 -------------------------------------------------
            lim = max(0.0, self._f("max_trim", 5.0e-4)) * _sc
            if lim > 0.0:
                y = max(-lim, min(lim, y))

            if not math.isfinite(y):
                self.reset()
                return 0.0
            self.out = y

            if self._b("log", False):
                now = time.monotonic()
                if now - self._last_log_t > 2.0:
                    self._last_log_t = now
                    print(f"[lane_trim] raw={x:+.6f} out={y:+.6f} v={float(v_ego):.1f}")
            return float(y)
        except Exception:
            self.reset()
            return 0.0

    # ------------------------------------------------------------------ diag
    def status(self) -> dict:
        return {
            "enabled": self._b("enabled", True),
            "raw": round(self.raw, 6),
            "out": round(self.out, 6),
        }
