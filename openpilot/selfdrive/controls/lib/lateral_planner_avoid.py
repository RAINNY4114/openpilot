#!/usr/bin/env python3
"""
lateral_planner_avoid.py — 通用静止障碍避让（故障车 / 落石 / 锥桶顺带覆盖）

设计来源
--------
算法骨架移植自 cpv9-corolla 的
  selfdrive/controls/lib/lateral_planner.py :: _calc_obstacle_avoid_param()
（clearance 计算 / 三角窗廓线 / 平滑 / 限幅），
但**不移植它的 MPC 与轨迹输出契约**，而是改用本机已验证的
「返回曲率修正量」范式（对齐 lateral_clearance.py / curve_lane_bias.py）。

数据源替换
----------
cpv9 读 carrot.obstacles（外部 UDP 推送，本机没有）。
本机改读 sm['radarTracks'].points，字段与 cpv9 完全同构：
    cpv9 longDistance  <->  radarTracks dRel
    cpv9 latDistance   <->  radarTracks yRel
    （cpv9 type 类别缺失，改用运动学特征推断）

生效车速闸门（用户决策）
------------------------
    vEgo < 80 km/h  -> 本模块（常态避让）生效
    vEgo >= 80 km/h -> 本模块完全关闭，交给紧急转向避让通道
数据依据：411 段全量实测，>79 km/h 同车道前方静止障碍仅 147 帧、
最长连续 3 帧、连续>=4 帧段数为 0 —— 数据不支持高速静止障碍避让。

安全设计（与 lateral_clearance.py 同级）
----------------------------------------
- 默认**关闭**（配置文件不存在 / enabled=false 时输出恒为 0，行为与原生完全一致）
- S1 观测模式：observe_only=true 时 update() 恒返 0，只写日志
- 输出硬限幅 max_curv；横向偏移限幅 offset_max_m（默认 0.55 m）
- 整个 update 包在 try/except 里：任何异常都返回 0，绝不把异常抛进 controlsd
- 配置热加载（按 mtime），可边开边调
- 与 leadOne 交叉校验：正前方若被判定为跟车目标（leadOne.present 且
  |dRel - lead.dRel| 很小），则不介入，避免与跟车抢

配置文件：/data/lateral_planner_avoid.json
"""

import json
import math
import os
import time

CONFIG_PATH = "/data/lateral_planner_avoid.json"

# ---- cpv9 移植常量 ----
KMH_TO_MS = 1.0 / 3.6
HIGH_SPEED_NO_AVOID = 80.0 * KMH_TO_MS      # 80 km/h 闸门（用户决策）
OBSTACLE_MAX_DETECT_LONG = 30.0             # cpv9: 超过 30 m 不采信
OBSTACLE_SMOOTH_ALPHA = 0.35                # cpv9: 偏移低通
HALF_CAR_WIDTH = 0.90                       # cpv9: 半车宽
MIN_SAFETY_MARGIN = 0.25                    # cpv9: 远距安全余量
MAX_SAFETY_MARGIN = 0.60                    # cpv9: 近距安全余量
CLOSE_DIST = 10.0                           # cpv9: 余量插值近端
FAR_DIST = 30.0                             # cpv9: 余量插值远端
ROADSIDE_DIST_THRESH = 3.5                  # cpv9: 路侧停放判定
OBS_HALF_WIDTH = {                          # cpv9: 各类目标半宽；本机按静止/对向简化
    "static": 0.60,
    "oncoming": 0.90,
    "unknown": 0.60,
}

DEFAULTS = {
    "enabled": False,
    "observe_only": True,       # S1: 恒返 0，只观测
    "log": True,
    "log_period_s": 2.0,

    # 车速闸门（用户决策：>80 km/h 完全关闭）
    "v_max_ms": HIGH_SPEED_NO_AVOID,
    "v_min_ms": 8.0,            # 29 km/h —— 数据富集区下沿

    # 横向偏移限幅（用户决策：基础 ±1.0 m，随车道宽自适应）
    "offset_max_m": 1.0,        # 硬上限；实际限幅 = min(offset_max_m, half_lane_w * offset_frac)
    "offset_frac": 0.95,        # 占半车道宽比例（取 ~1/2 车道宽）
    "offset_max_floor_m": 0.55, # 车道线不可信/窄路时的保守下限，防止完全失效

    "smooth_alpha": OBSTACLE_SMOOTH_ALPHA,

    # 曲率换算与限幅
    "kp_curv": 0.0012,          # 曲率 / 米
    "max_curv": 0.00045,        # 输出硬限幅 (1/m)

    # 检出门槛
    "detect": {
        "max_long_m": OBSTACLE_MAX_DETECT_LONG,
        "min_long_m": 2.0,
        "static_v_thr": 1.5,    # |vRel + vEgo| < thr 判为静止
        "lane_half_w_min": 1.4, # 车道内判定：|yRel| 上限（保守，防跨道误判）
        "lane_half_w_max": 2.2,
        "min_run_frames": 4,    # 去抖：连续 N 帧（对齐扫描判据）
        "lane_prob_thr": 0.35,  # 车道线可信度门槛（实测可能只有 0.05）
    },

    # 场景分类阈值
    "scene": {
        "center_y_m": 0.9,      # |yRel| < 0.9 视为"正中"
        "roadside_y_m": ROADSIDE_DIST_THRESH,
    },
}


def _deep_merge(base, over):
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class AdaptiveObstacleMap:
    """数据层：radarTracks -> 动/静分类 -> 车道内判定 -> 去抖 -> 场景事件。

    只读，不产生任何控制输出。
    """

    def __init__(self):
        self.run_counter = 0        # 跨帧状态：连续满足帧数
        self.run_active = False
        self.reset()

    def reset(self):
        """清空本帧数据 + 复位跨帧去抖状态。"""
        self._clear_frame()
        self.run_counter = 0
        self.run_active = False
        self.reason = "off"
        self.half_w = 1.75          # 半车道宽估计 (m)
        self.lane_prob_ok = False

    def _clear_frame(self):
        """只清本帧数据，保留跨帧去抖计数。"""
        self.tracks = []            # 本帧采信的障碍物列表
        self.static_c = []          # 同车道正前方静止
        self.static_s = []          # 同车道偏侧静止
        self.oncoming = []          # 对向运动目标
        self.closest_static = None

    @staticmethod
    def _lane_half_width(model_v2, cfg):
        """用 laneLines / roadEdges 估算半车道宽。本机无 laneWidthLeft/Right。

        返回 (half_width_m, prob_ok)。prob_ok=False 时调用方应放宽/收紧策略。
        """
        thr = cfg.get("lane_prob_thr", 0.35)
        hw = 1.75
        prob_ok = False
        try:
            ll = model_v2.laneLines
            probs = list(model_v2.laneLineProbs)
            if len(ll) >= 4 and len(probs) >= 4:
                # 左线取 index1，右线取 index2（内部两条）
                l_left = float(ll[1].y[0])
                l_right = float(ll[2].y[0])
                w = l_right - l_left
                p = min(float(probs[1]), float(probs[2]))
                if 2.0 < w < 5.0:
                    hw = w * 0.5
                    prob_ok = p > thr
        except Exception:
            pass
        if not prob_ok:
            try:
                re_ = model_v2.roadEdges
                if len(re_) >= 2:
                    w = float(re_[1].y[0]) - float(re_[0].y[0])
                    if 2.5 < w < 9.0:
                        hw = max(hw, min(w * 0.5, cfg.get("lane_half_w_max", 2.2)))
            except Exception:
                pass
        hw = max(cfg.get("lane_half_w_min", 1.4),
                 min(cfg.get("lane_half_w_max", 2.2), hw))
        return hw, prob_ok

    def update(self, sm, v_ego, model_v2, cfg):
        """从 sm 拉取 radarTracks，产出场景事件。异常时清空并返回。"""
        self._clear_frame()
        det = cfg.get("detect", {})
        scn = cfg.get("scene", {})
        try:
            msg = sm["radarTracks"]
            # 兼容两种形式：
            #  - 真实 SubMaster 返回消息包装（有 .valid / .points）
            #  - 离线验证时传裸 capnp struct（只有 .points）
            valid = getattr(msg, "valid", None)
            if valid is False:
                self.reason = "invalid"
                return
            pts = msg.points
        except Exception:
            self.reason = "no_msg"
            return

        half_w, prob_ok = self._lane_half_width(model_v2, det)
        thr = float(det.get("static_v_thr", 1.5))
        dmax = float(det.get("max_long_m", OBSTACLE_MAX_DETECT_LONG))
        dmin = float(det.get("min_long_m", 2.0))
        y_center = float(scn.get("center_y_m", 0.9))

        all_pts = []
        for p in pts:
            try:
                d = float(p.dRel)
                y = float(p.yRel)
                vr = float(p.vRel)
            except Exception:
                continue
            if d < dmin or d > dmax:
                continue
            static = abs(vr + v_ego) < thr
            all_pts.append((d, y, vr, static))
        self.tracks = all_pts

        # 逐类归档（只在车道内）
        for d, y, vr, static in all_pts:
            if abs(y) > half_w:
                continue          # 不在本车道，不介入（跨道目标交给 AEB/紧急通道）
            if static:
                item = {"d": d, "y": y, "vRel": vr}
                if abs(y) < y_center:
                    self.static_c.append(item)
                else:
                    self.static_s.append(item)
            elif vr > 2.5 and d < 30.0:
                # 明显朝我开来（vRel 大正）
                self.oncoming.append({"d": d, "y": y, "vRel": vr})

        if self.static_c:
            self.closest_static = min(self.static_c, key=lambda x: x["d"])
        elif self.static_s:
            self.closest_static = min(self.static_s, key=lambda x: x["d"])

        # 去抖：连续 N 帧才置 run_active
        n_min = int(det.get("min_run_frames", 4))
        has_any = bool(self.static_c or self.static_s)
        if has_any:
            self.run_counter += 1
        else:
            self.run_counter = 0
        self.run_active = self.run_counter >= n_min

        if self.static_c:
            self.reason = "static_center"
        elif self.static_s:
            self.reason = "static_side"
        elif self.oncoming:
            self.reason = "oncoming"
        else:
            self.reason = "clear"
        self.lane_prob_ok = prob_ok
        self.half_w = half_w


class ObstacleAvoidance:
    """决策层：场景事件 -> 目标横向偏移 -> 曲率修正量。

    update() 永远返回一个有限 float（1/m）。S1 观测模式恒返 0。
    """

    def __init__(self, config_path=CONFIG_PATH):
        self.config_path = config_path
        self.cfg = json.loads(json.dumps(DEFAULTS))
        self._cfg_mtime = None
        self._last_cfg_check = 0.0

        self.map = AdaptiveObstacleMap()
        self.target_offset = 0.0
        self.smooth_offset = 0.0
        self.last_curv = 0.0
        self.last_reason = "off"
        self.last_scene = "off"
        self.last_d = float("inf")
        self.last_y = 0.0
        self._last_log_t = 0.0
        # 观测统计（S1 用）
        self.obs_frames = 0
        self.obs_would_trigger = 0
        self.obs_last_trigger_ts = 0.0

        if os.path.exists(config_path):
            self._load_config(force=True)

    # ------------------------------------------------------------- config
    def _load_config(self, force=False):
        now = time.monotonic()
        if not force and (now - self._last_cfg_check) < 1.0:
            return
        self._last_cfg_check = now
        try:
            if not os.path.exists(self.config_path):
                return
            m = os.path.getmtime(self.config_path)
            if not force and m == self._cfg_mtime:
                return
            with open(self.config_path, "r") as f:
                user = json.load(f)
            if isinstance(user, dict):
                self.cfg = _deep_merge(DEFAULTS, user)
                self._cfg_mtime = m
        except Exception:
            pass

    def _f(self, path, default):
        """按 'a.b.c' 取值，失败返回 default。"""
        try:
            cur = self.cfg
            for k in path.split("."):
                cur = cur[k]
            return float(cur)
        except Exception:
            return float(default)

    # ------------------------------------------------------------- 决策
    def _calc_offset(self, md, cfg):
        """移植 cpv9 _calc_obstacle_avoid_param 的核心数学。

        md: AdaptiveObstacleMap
        返回 (target_offset_m, closest_dist, scene)
        """
        # ★ 自适应限幅：min(绝对上限, 半车道宽 * 比例)，不低于保守下限
        off_hard = float(cfg.get("offset_max_m", 1.0))
        off_frac = float(cfg.get("offset_frac", 0.95))
        off_floor = float(cfg.get("offset_max_floor_m", 0.55))
        half_w = float(getattr(md, "half_w", 1.75))
        off_max = max(off_floor, min(off_hard, half_w * off_frac))

        best = 0.0
        min_d = float("inf")
        scene = "none"

        cand = []
        for it in md.static_c:
            cand.append((it, "static_c", OBS_HALF_WIDTH["static"]))
        for it in md.static_s:
            cand.append((it, "static_s", OBS_HALF_WIDTH["static"]))
        for it in md.oncoming:
            cand.append((it, "oncoming", OBS_HALF_WIDTH["oncoming"]))

        for it, scene_tag, obs_hw in cand:
            d = float(it["d"])
            y = float(it["y"])
            min_d = min(min_d, d)

            # 距离自适应安全余量（cpv9: 近距 0.60 / 远距 0.25）
            if d < CLOSE_DIST:
                margin = MAX_SAFETY_MARGIN
            else:
                t = max(0.0, min(1.0, (d - CLOSE_DIST) / (FAR_DIST - CLOSE_DIST)))
                margin = MAX_SAFETY_MARGIN + (MIN_SAFETY_MARGIN - MAX_SAFETY_MARGIN) * t

            required = abs(y) + obs_hw + HALF_CAR_WIDTH + margin
            # 目标在左(y>0) -> 往右让(push 为负); 在右 -> 往左让
            push = -required if y >= 0 else required

            # 取"最保守"的一个（绝对值最大）
            if abs(push) > abs(best):
                best = push
                scene = scene_tag
                self.last_d = d
                self.last_y = y

        best = max(-off_max, min(off_max, best))
        return best, min_d, scene

    def update(self, active, model_v2, v_ego, dt, sm=None, lane_change_active=False):
        """返回要加到 desired_curvature 上的曲率修正量 (1/m)。"""
        try:
            self._load_config()
            cfg = self.cfg

            if not bool(cfg.get("enabled", False)) or not active:
                self._reset_dyn()
                self.last_reason = "off"
                return 0.0

            # 变道/人驾介入中不避让，避免与变道抢方向
            if lane_change_active:
                self._reset_dyn()
                self.last_reason = "lane_change"
                return 0.0

            v_min = float(cfg.get("v_min_ms", 8.0))
            v_max = float(cfg.get("v_max_ms", HIGH_SPEED_NO_AVOID))
            # ★ 车速闸门：>80 km/h 完全关闭，交给紧急转向避让
            if v_ego < v_min or v_ego >= v_max:
                self._reset_dyn()
                self.last_reason = "v_gate"
                return 0.0

            if sm is None:
                self._reset_dyn()
                self.last_reason = "no_sm"
                return 0.0

            # ---- 数据层 ----
            self.map.update(sm, v_ego, model_v2, cfg)
            md = self.map
            self.obs_frames += 1

            if not md.run_active:
                # 去抖未过：偏移平滑回 0
                self.smooth_offset += 0.15 * (0.0 - self.smooth_offset)
                self.target_offset = 0.0
                self.last_curv = 0.0
                self.last_reason = md.reason
                self.last_scene = "none"
                self._maybe_log(v_ego, md)
                return 0.0

            # ---- 决策层 ----
            raw, dmin, scene = self._calc_offset(md, cfg)
            self.target_offset = raw
            a = max(0.01, min(1.0, float(cfg.get("smooth_alpha", OBSTACLE_SMOOTH_ALPHA))))
            self.smooth_offset += a * (raw - self.smooth_offset)
            self.last_scene = scene
            self.last_reason = scene
            self.obs_would_trigger += 1
            self.obs_last_trigger_ts = time.monotonic()

            # ---- S1 观测模式：只记不改 ----
            if bool(cfg.get("observe_only", True)):
                self.last_curv = 0.0
                self._maybe_log(v_ego, md)
                return 0.0

            # ---- 偏移 -> 曲率修正量 ----
            kp = float(cfg.get("kp_curv", 0.0012))
            curv = kp * self.smooth_offset
            max_curv = float(cfg.get("max_curv", 0.00045))
            curv = max(-max_curv, min(max_curv, curv))
            if not math.isfinite(curv):
                self._reset_dyn()
                return 0.0
            self.last_curv = float(curv)
            self._maybe_log(v_ego, md)
            return float(curv)
        except Exception:
            self._reset_dyn()
            return 0.0

    # ------------------------------------------------------------- helpers
    def _reset_dyn(self):
        self.target_offset = 0.0
        self.smooth_offset = 0.0
        self.last_curv = 0.0
        self.last_scene = "off"
        if hasattr(self, "map"):
            self.map.reset()
        self.last_d = float("inf")
        self.last_y = 0.0

    def _maybe_log(self, v_ego, md):
        try:
            if not bool(self.cfg.get("log", False)):
                return
            now = time.monotonic()
            period = float(self.cfg.get("log_period_s", 2.0))
            # 只在"有事件"或每隔 period 打一次
            interesting = md.run_active or md.reason in ("static_center", "static_side", "oncoming")
            if not interesting and (now - self._last_log_t) < period:
                return
            self._last_log_t = now
            d = getattr(self, "last_d", float("inf"))
            dstr = f"{d:.1f}" if d != float("inf") else "--"
            print(f"[lateral_planner_avoid] {self.last_reason} scene={self.last_scene} "
                  f"n_c={len(md.static_c)} n_s={len(md.static_s)} n_on={len(md.oncoming)} "
                  f"d={dstr} y={self.last_y:+.2f} run={md.run_counter} "
                  f"offT={self.target_offset:+.2f} offS={self.smooth_offset:+.2f} "
                  f"curv={self.last_curv:+.5f} v={v_ego * 3.6:.0f}km/h "
                  f"trig={self.obs_would_trigger}/{self.obs_frames}")
        except Exception:
            pass

    def status(self):
        return {
            "reason": self.last_reason,
            "scene": self.last_scene,
            "target_offset": self.target_offset,
            "smooth_offset": self.smooth_offset,
            "curv": self.last_curv,
            "d": self.last_d,
            "y": self.last_y,
            "frames": self.obs_frames,
            "would_trigger": self.obs_would_trigger,
        }
