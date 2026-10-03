#!/usr/bin/env python3
"""
lateral_clearance.py — 车道居中微调 + 横向间距避让（护栏 / 大货车）

设计目标
--------
1) 居中微调：消除"居中后偏左/偏右"的稳态偏差（相机标定 / 模型固有偏差）。
2) 横向间距避让：遇到**中央护栏/栅栏**或**相邻车道的大车**时，主动把车往
   反方向挪一点，拉开横向间距。

为什么这样做
------------
本机横向链路是「端到端模型直出曲率」：

    modelV2.action.desiredCurvature -> clip_curvature -> actuators.curvature
      -> Ford carcontroller -> FordCurveController -> CAN

`LatControlAngle` / `angleOffsetDeg` / 转向比在福特上都被 carcontroller 丢弃，
**不影响横向**（详见 c3x_analysis/福特横向控制诊断与优化方案.md）。

模型直出曲率的链路没有"横向位置积分器"，所以单纯加一个恒定曲率偏置会被
模型闭环抵消。正确做法是加一个**位置闭环**：

    lane_center_y = (laneLines[1].y[0] + laneLines[2].y[0]) / 2
    error = lane_center_y + bias
    trim  = clamp(kp*error + ki*∫error, -max_curv, +max_curv)

符号约定（来自本仓库 ldw.py：左车道线 y 为负、右为正）
    lane_center_y > 0  <=>  车道中心在车右侧  <=>  车偏左
    desiredCurvature > 0  <=>  右转（LatControlAngle 里取负号）
    bias > 0  <=>  希望车向右挪

推导：设 s 为车相对车道中心的真实横向位置（右正），则 lane_center_y = -s_meas，
s_meas = s_true + beta（beta 为感知偏差）。闭环把 error 打到 0，即
s_true = bias - beta。所以把 bias 调到 beta 即可真正居中；之后再叠加避让量
delta，最终 s_true = delta。 => 静态微调与动态避让可以简单相加。

安全设计
--------
- 默认 **关闭**（配置文件不存在 / enabled=false 时输出恒为 0，行为与原生完全一致）
- 输出硬限幅 max_curv
- 积分抗饱和（clamp 到 ±max_curv/ki 附近），非激活/车道线失效时清零
- 整个 update 包在 try/except 里：**任何异常都返回 0，绝不把异常抛进 controlsd**
- 配置热加载（按 mtime），可边开边调

配置文件：/data/lateral_clearance.json
"""

import json
import math
import os
import time

CONFIG_PATH = "/data/lateral_clearance.json"

# ---- 默认值（配置文件缺字段时使用）----
DEFAULTS = {
    "enabled": False,
    "trim_m": 0.0,          # 静态居中微调，>0 = 向右挪
    "kp": 0.0010,           # 曲率 / 米
    "ki": 0.0006,           # 积分增益
    "max_curv": 0.0004,     # 输出硬限幅 (1/m)
    "min_speed_ms": 4.0,    # 低于此速度不介入
    "smooth_alpha": 0.10,   # 避让量低通系数
    "bias_max_m": 1.0,      # 总横向偏置限幅 (m)
    "lane_prob_thr": 0.50,  # 车道线可信度阈值
    "log": False,
    "barrier": {
        "enable": True,
        "bias_m": 0.30,         # 中央护栏 -> 向右让
        "edge_std_thr": 0.40,   # roadEdgeStds[0] 置信度阈值 (m, 越小越可信)
        "max_edge_dist_m": 4.0  # 左路沿进入此距离才认为"贴着护栏"
    },
    "vehicle": {
        "enable": True,
        "bias_m": 0.25,        # 相邻车道有车 -> 让开
        "large_bias_m": 0.45,  # 判定为"大车"时让更多
        "large_dwell_s": 2.5,  # 相邻车道持续存在超过该时长 -> 视为大车
        "prob_thr": 0.45,
        "x_min_m": 1.5,
        "x_max_m": 45.0,
        "y_min_m": 1.5,
        "y_max_m": 5.0
    }
}


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class LateralClearance:
    """横向间距避让 + 车道居中微调。update() 永远返回一个有限 float。"""

    def __init__(self, config_path: str = CONFIG_PATH):
        self.config_path = config_path
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self._last_cfg_check = 0.0

        self.integral = 0.0
        self.bias_filtered = 0.0
        self.lead_dwell = 0.0
        self.last_error = 0.0
        self.last_trim = 0.0
        self.last_bias = 0.0
        self.last_reason = "off"
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
            self.cfg = _deep_merge(DEFAULTS, user)
            self._cfg_mtime = st.st_mtime
        except FileNotFoundError:
            self.cfg = dict(DEFAULTS)
            self._cfg_mtime = None
        except Exception:
            # 配置损坏 -> 保持上一次的有效配置，绝不让异常外泄
            pass

    def _f(self, section: str, key: str, default: float) -> float:
        try:
            v = self.cfg[section][key] if section else self.cfg[key]
            return float(v)
        except Exception:
            return float(default)

    def _b(self, section: str, key: str, default: bool) -> bool:
        try:
            v = self.cfg[section][key] if section else self.cfg[key]
            return bool(v)
        except Exception:
            return bool(default)

    # ------------------------------------------------------------------ signals
    @staticmethod
    def _lane_center_y(model_v2, prob_thr: float):
        """返回 (lane_center_y, ok)。lane_center_y > 0 表示车偏左。"""
        try:
            lines = model_v2.laneLines
            probs = model_v2.laneLineProbs
            if len(lines) < 4 or len(probs) < 4:
                return 0.0, False
            if probs[1] < prob_thr or probs[2] < prob_thr:
                return 0.0, False
            y_left = float(lines[1].y[0])    # 左线为负
            y_right = float(lines[2].y[0])   # 右线为正
            if not (math.isfinite(y_left) and math.isfinite(y_right)):
                return 0.0, False
            # 车道宽度合理性检查：太窄/太宽都不可信
            width = y_right - y_left
            if not (2.0 <= width <= 6.0):
                return 0.0, False
            return (y_left + y_right) / 2.0, True
        except Exception:
            return 0.0, False

    def _barrier_bias(self, model_v2) -> tuple[float, str]:
        """中央护栏/栅栏在左侧 -> 向右让。返回 (bias, reason)。

        roadEdges[0] = 左路沿（已由 relc.py 确认），y 为负值；
        可信度用 roadEdgeStds[0]（单位米，越小越可信）——
        base modelV2 没有 roadEdgeProbs，所以用 std 而不是 prob。
        """
        if not self._b("barrier", "enable", True):
            return 0.0, ""
        std_thr = self._f("barrier", "edge_std_thr", 0.40)
        max_d = self._f("barrier", "max_edge_dist_m", 4.0)
        bias = self._f("barrier", "bias_m", 0.30)
        try:
            edges = model_v2.roadEdges
            if len(edges) < 2:
                return 0.0, ""
            # 左路沿越接近 0 说明越贴着护栏
            y_left_edge = float(edges[0].y[0])
            if not math.isfinite(y_left_edge):
                return 0.0, ""
            try:
                stds = model_v2.roadEdgeStds
                if len(stds) >= 1:
                    s0 = float(stds[0])
                    if math.isfinite(s0) and s0 > std_thr:
                        return 0.0, ""   # 路沿估计不可信
            except Exception:
                pass
            if abs(y_left_edge) <= max_d:
                return bias, "barrier_L"
        except Exception:
            pass
        return 0.0, ""

    def _vehicle_bias(self, model_v2, dt: float, v_ego: float) -> tuple[float, str]:
        """相邻车道有车 -> 让开；持续存在判定为大车。返回 (bias, reason)。"""
        if not self._b("vehicle", "enable", True):
            return 0.0, ""
        prob_thr = self._f("vehicle", "prob_thr", 0.45)
        x_min = self._f("vehicle", "x_min_m", 1.5)
        x_max = self._f("vehicle", "x_max_m", 45.0)
        y_min = self._f("vehicle", "y_min_m", 1.5)
        y_max = self._f("vehicle", "y_max_m", 5.0)

        right_bias = 0.0   # 左侧有车 -> 向右让
        left_bias = 0.0    # 右侧有车 -> 向左让
        found = False
        try:
            for lead in model_v2.leadsV3:
                if lead.prob < prob_thr:
                    continue
                x = float(lead.x[0])
                y = float(lead.y[0])
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                if not (x_min <= x <= x_max):
                    continue
                ay = abs(y)
                if not (y_min <= ay <= y_max):
                    continue
                found = True
                if y < 0.0:      # 左侧相邻车道
                    right_bias = max(right_bias, 1.0)
                else:            # 右侧相邻车道
                    left_bias = max(left_bias, 1.0)
        except Exception:
            found = False

        # 持续存在 -> 判定为"大车"（大货车通过时间长）
        if found:
            self.lead_dwell += dt
        else:
            self.lead_dwell = max(0.0, self.lead_dwell - dt * 2.0)

        large = self.lead_dwell >= self._f("vehicle", "large_dwell_s", 2.5)
        b = self._f("vehicle", "large_bias_m", 0.45) if large else self._f("vehicle", "bias_m", 0.25)

        if right_bias <= 0.0 and left_bias <= 0.0:
            return 0.0, ""
        net = (right_bias - left_bias) * b
        if net == 0.0:
            return 0.0, "veh_both"
        return net, ("veh_L" + ("_large" if large else "")) if net > 0 else ("veh_R" + ("_large" if large else ""))

    # ------------------------------------------------------------------ update
    def update(self, active: bool, model_v2, v_ego: float, dt: float) -> float:
        """返回要加到 desired_curvature 上的曲率修正量 (1/m)。"""
        try:
            self._load_config()
            cfg_enabled = self._b(None, "enabled", False)

            if (not cfg_enabled) or (not active) or v_ego < self._f(None, "min_speed_ms", 4.0):
                self.integral = 0.0
                self.bias_filtered = 0.0
                self.lead_dwell = 0.0
                self.last_trim = 0.0
                self.last_bias = 0.0
                self.last_reason = "off"
                return 0.0

            lane_center_y, ok = self._lane_center_y(model_v2, self._f(None, "lane_prob_thr", 0.50))
            if not ok:
                # 车道线不可信：保持积分不动但输出 0，避免乱打方向
                self.last_trim = 0.0
                self.last_reason = "no_lane"
                return 0.0

            # ---- 动态避让量 ----
            b_bar, r_bar = self._barrier_bias(model_v2)
            b_veh, r_veh = self._vehicle_bias(model_v2, dt, v_ego)
            bias_raw = self._f(None, "trim_m", 0.0) + b_bar + b_veh
            bmax = self._f(None, "bias_max_m", 1.0)
            bias_raw = max(-bmax, min(bmax, bias_raw))

            a = self._f(None, "smooth_alpha", 0.10)
            a = max(0.01, min(1.0, a))
            self.bias_filtered += a * (bias_raw - self.bias_filtered)

            # ---- PI 位置闭环 ----
            error = lane_center_y + self.bias_filtered
            kp = self._f(None, "kp", 0.0010)
            ki = self._f(None, "ki", 0.0006)
            max_curv = self._f(None, "max_curv", 0.0004)

            p_term = kp * error
            self.integral += ki * error * dt
            # 抗饱和：积分项单独也受限
            if ki > 1e-9:
                lim = max_curv / ki
                self.integral = max(-lim, min(lim, self.integral))

            trim = p_term + self.integral
            trim = max(-max_curv, min(max_curv, trim))
            # 若被限幅，把积分拉回到可用范围（防 windup）
            if ki > 1e-9 and abs(p_term + self.integral) > max_curv:
                self.integral = max(-max_curv, min(max_curv, trim - p_term))

            self.last_error = error
            self.last_trim = trim
            self.last_bias = self.bias_filtered
            self.last_reason = (r_bar or "") + ("+" if (r_bar and r_veh) else "") + (r_veh or "") or "trim"

            if self._b(None, "log", False):
                now = time.monotonic()
                if now - self._last_log_t > 2.0:
                    self._last_log_t = now
                    print(f"[lateral_clearance] e={error:+.3f} bias={self.bias_filtered:+.3f} "
                          f"trim={trim:+.5f} why={self.last_reason} v={v_ego:.1f}")

            if not math.isfinite(trim):
                self.integral = 0.0
                return 0.0
            return float(trim)
        except Exception:
            # 绝不把异常抛进 controlsd
            self.integral = 0.0
            self.bias_filtered = 0.0
            return 0.0

    # ------------------------------------------------------------------ diag
    def status(self) -> dict:
        return {
            "enabled": self._b(None, "enabled", False),
            "bias_m": round(self.bias_filtered, 3),
            "error_m": round(self.last_error, 3),
            "trim_curv": round(self.last_trim, 6),
            "reason": self.last_reason,
            "lead_dwell_s": round(self.lead_dwell, 2),
        }
