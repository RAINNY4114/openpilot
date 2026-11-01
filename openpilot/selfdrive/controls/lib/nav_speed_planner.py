#!/usr/bin/env python3
"""
nav_speed_planner.py —— 导航纵向速度规划器（[NAV_LONG]）

目标
----
把 `navd` 发布的「导航速度剖面」（navProfile）翻译成纵向控制可直接消费的
两个量：

    1. `v_cruise_cap`  —— 法定限速上限（m/s），由调用方压 `v_cruise`
    2. `a_target`      —— 额外减速度候选（m/s^2，恒 <= 0），进入
                          `LongitudinalPlanner` 的 candidates 取 min

覆盖三个需求：
    - 导航限速：路线所在路段的法定限速（Mapbox maxspeed 注解，或高德
      AmapNavi.navSpeedLimit）作为巡航速度上限
    - 转弯前减速：剖面上每个采样点带「该点允许的最高速度」，本模块用
      `v_allow = sqrt(v_cap^2 + 2*a_dec*d)` 把它折算成「此刻允许的速度」，
      取最小 -> 自然得到"提前 N 米开始收油/刹车"的效果
    - 到达停车：剖面终点 speedCap = 0，折算后 v_allow 随距离收敛到 0，
      并在足够近时给出 should_stop

与本机既有模块的关系
--------------------
完全对齐 `aa_brake_apply.py` 的范式：
  - 输出"修正量/候选值"，**不替代**主链路（lead / e2e / MPC 谁更慢谁赢）
  - 默认**关闭**（配置缺失 -> enabled=false -> 输出恒 0）
  - S1 观测模式 `observe_only=true` 时恒返 0，只写日志（先看数不生效）
  - 配置热加载（按 mtime）
  - 整个 update 包 try/except，异常一律降级为 0，绝不把异常抛进 plannerd
  - 硬限幅 + 速率整形

安全设计
--------
1. **只会更慢**：`a_target <= 0` 恒成立；正的加速度需求一律输出 0，
   加速永远由原链路的 cruise / e2e 决定。
2. **默认关闭**：配置不存在或 enabled!=true -> 恒 0，纵向行为与原生一致。
3. **二次限幅**：即使剖面算出 -8 m/s^2，也被 `max_decel_ms2` 硬夹。
4. **速率整形**：减速按 `max_rise_ms2_per_s` 爬升；释放按
   `release_ms2_per_s` 收回，避免一帧跳变。
5. **超时闸门**：剖面超过 `profile_stale_sec` 未刷新 -> 视为失效。失效后
   **先保持** `stale_hold_sec`（默认 1 s）当前减速度，再按 `release_ms2_per_s`
   释放 —— 防止 GPS 短暂丢星时"正在执行的刹车被瞬间松开"，同时也不会
   无限期刹着（真正的失效最终一定会释放）。
6. **车速闸门**：低于 `v_min_kmh` 不介入（低速交给常规制动/停车逻辑）。
7. **异常安全**：任何异常 -> 0。

输出契约
--------
    update(...) -> (a_target, v_cruise_cap)
    a_target      : float <= 0.0
    v_cruise_cap  : float > 0 或 None（None = 本次不限速）

配置文件：/data/nav_speed.json
日志：    /data/media/0/nav_speed.log
"""

import json
import math
import os
import time

import numpy as np

CONFIG_PATH = "/data/nav_speed.json"
LOG_PATH = "/data/media/0/nav_speed.log"

# 剖面点相对本车的有效区间：略往前留一点，避免投影抖动导致边界点被丢弃
D_BACK_MARGIN = 5.0
# 计算 a_target 时分母的最小距离，避免除零放大
D_MIN = 1.0
# 判定"该停车了"的速度 / 距离阈值
STOP_V = 0.3
STOP_D = 10.0

DEFAULTS = {
    # ---- 总开关：false 时输出恒为 0，与原生完全一致 ----
    "enabled": False,
    # ---- S1 观测模式：true 时恒返 0，只写日志（先看数不生效）----
    "observe_only": True,

    # ---- 日志 ----
    "log": True,
    "log_period_s": 1.0,

    # ---- 功能开关 ----
    # 法定限速 -> 压 v_cruise
    "use_speed_limit": True,
    # 弯道/终点剖面 -> a_target
    "use_profile": True,

    # ---- 限速 ----
    # 限速偏移（km/h）：最终上限 = 限速 + offset
    "speed_limit_offset_kmh": 0.0,
    # 高德 navSpeedLimit 优先级高于 Mapbox maxspeed（国内 Mapbox 覆盖差）
    "amap_priority": True,
    # 高德数据超过该秒数未刷新则不用
    "amap_stale_sec": 3.0,
    # ★ AmapNavi.navSpeedLimit 的单位。
    # 这个字段是 carrot/amap_navi.py:801-803 从手机 JSON 里**原样透传**的，没有任何
    # 换算；而高德 SDK 的 LIMITED_SPEED 原始字段是 km/h，转发 App 也可能自己转成 m/s。
    # 两种都见过，所以必须显式声明：
    #   "auto" -> > amap_kph_threshold 视为 km/h，否则 m/s（默认）
    #   "kmh"  -> 一律除以 3.6
    #   "ms"   -> 原样当 m/s
    # auto 的失效方向始终是「把限速看大 -> 不减速」，不会造成过度减速（安全优先）。
    "amap_limit_unit": "auto",
    # auto 模式的判别阈值（m/s）。45 m/s = 162 km/h，作为限速当 m/s 解释不合理。
    "amap_kph_threshold": 45.0,
    # 无几何剖面时（只有限速、没有弯道/终点信息）的「限速闭环」：
    # 减速度 = -(v_ego - 限速) * gain，夹到 max_decel_ms2。
    # 不做这个闭环的话，减速只剩 cruise 通道（A_CRUISE_MIN=-0.5），
    # 30 m/s -> 11 m/s 要 ~780 m，实际等于限速没生效。
    # 调小 -> 减速更温和、更晚；调大 -> 更快压到限速。0 = 关闭闭环（只压 v_cruise）。
    "limit_closed_loop_gain": 0.5,
    # 限速闭环死区（m/s）：超出量小于该值不减速，避免在限速附近来回抖动
    "limit_deadband_ms": 0.3,

    # ---- 剖面 -> 速度 ----
    # 折算速度上限时使用的舒适减速度（m/s^2），越小 => 越早开始减速
    "a_dec_ms2": 0.8,
    # 只处理本车前方该距离内的剖面点（与 navd 的 PROFILE_LOOKAHEAD 对齐）
    "lookahead_m": 300.0,

    # ---- 限幅 / 整形 ----
    # a_target 最负值（m/s^2）。硬夹。
    "max_decel_ms2": 2.0,
    # 减速度爬升速率上限（m/s^2 per s）
    "max_rise_ms2_per_s": 1.5,
    # 减速度释放速率上限（m/s^2 per s），偏快一点，避免剖面失效后刹车残留
    "release_ms2_per_s": 3.0,

    # ---- 闸门 ----
    "v_min_kmh": 10.0,
    # 剖面超过该秒数未刷新 -> 视为失效
    "profile_stale_sec": 2.5,
    # ★ 失效后**先保持**当前减速度这么久，再开始释放。
    # 没有这一段的话，release_ms2_per_s=3.0 会让正在执行的 -2.0 m/s^2 在
    # 0.67 s 内完全松开 —— GPS 进隧道丢星 2.5 s 时，本来在弯道前刹车，
    # 刹车会突然消失然后加速冲进弯道。保持一段时间更符合「只会更慢」的
    # 安全原则：宁可多刹一会儿（最多慢一点），不要松开正在起作用的刹车。
    # 0 = 关闭保持（失效即释放，旧行为）。
    "stale_hold_sec": 1.0,
}


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _f(x, default=0.0):
    """把 capnp / json 里可能出现的 None / NaN 统一成有限 float。"""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class NavProfileView:
    """把 navProfile 消息拍平成 numpy，并做一次结构校验。

    之所以不直接吃 capnp reader：本模块要保持「纯 numpy」可单测，
    测试里可以直接构造 View 而不用起 messaging。
    """

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
        """msg: cereal 的 navProfile reader（或 None）。无效返回 None。"""
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

        # 进度必须单调递增，否则整条剖面不可信
        if not np.all(np.isfinite(p_progress)):
            return None
        if np.any(np.diff(p_progress) < -1e-3):
            return None

        stamp = _f(getattr(msg, "sourceMonoTime", 0)) / 1e9
        if stamp <= 0.0:
            return None

        return cls(_f(msg.progress), _f(msg.speedLimit, -1.0), p_progress, p_curv, p_cap, stamp)


class NavSpeedPlanner:
    """导航速度剖面 -> (a_target <= 0, v_cruise_cap)。"""

    def __init__(self, dt=0.05):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self.dt = max(float(dt), 1e-3)

        self._shaped = 0.0          # 当前整形后的 a_target（<= 0）
        self._last_log_t = 0.0
        self._last_reason = "init"

        # 最近一次的计算结果（给日志 / 外部观察用）
        self.a_target = 0.0
        self.should_stop = False
        self.v_cruise_cap = None
        self.v_cap = float("inf")
        self.d_cap = float("inf")
        self.limit_source = "none"
        self.amap_limit_raw = -1.0   # 高德原始值（未换算），用于核对单位

        self._load_config(force=True)

    # ------------------------------------------------------------------
    # 配置热加载
    # ------------------------------------------------------------------
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
    # 主入口
    # ------------------------------------------------------------------
    def update(self, v_ego, profile_msg=None, amap_msg=None, long_active=True, now=None,
               amap_recv_time=None):
        """返回 (a_target, v_cruise_cap)。任何异常 -> (0.0, None)。

        amap_recv_time: SubMaster 记录的 amapNavi 最后接收时刻（time.monotonic 域）。
        AmapNavi 消息本身没有时间戳字段，只能靠这个判新鲜度。
        """
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

        # 既接受 cereal 的 navProfile 消息，也接受已经拍平好的 View（便于单测）
        if isinstance(profile_msg, NavProfileView):
            view = profile_msg
        else:
            view = NavProfileView.from_msg(profile_msg)
        amap_limit = self._amap_speed_limit(amap_msg, now, amap_recv_time)

        # 观测模式下也把"本应输出什么"算出来，只是不外发（日志可见）
        out_a, out_cap, reason, v_cap, d_cap, lim_src, out_stop = self._compute(
            v_ego, view, amap_limit, now, cfg)

        self.v_cap = v_cap
        self.d_cap = d_cap
        self.limit_source = lim_src

        # ---- 闸门 ----
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
            # 低速不再叠加减速度，但「终点停车」要保持住，否则会蠕行
            self.should_stop = bool(out_stop)
            self.v_cruise_cap = None
            self._last_reason = "v_low"
            self._log(now, v_ego, out_a, 0.0, out_cap, v_cap, d_cap, self._last_reason)
            return 0.0, None

        # 这些 reason 都表示「没有可用的几何约束」，退化为纯限速闭环。
        # profile_off / profile_empty 也要走这里：关掉剖面或剖面上没有约束点时，
        # 若不闭环就只剩 cruise 通道（-0.5 m/s^2），限速基本等于没生效。
        if reason in ("no_profile", "profile_stale", "profile_off", "profile_empty"):
            # 剖面失效时仍可保留限速上限（限速不依赖几何，来自 Amap / 静态注解）。
            # 但只压 v_cruise 不够：cruise 通道的减速度被 A_CRUISE_MIN(-0.5) 限死，
            # 高速超速要几百米才降得下来。这里补一个「限速闭环」——只按超出量减速，
            # 不需要距离信息，因此没有剖面也能及时压到限速。
            #
            # ★ 这是 navd 不可用（Mapbox 代理 404）时唯一有效的减速来源，
            #   也是「手机高德投屏」场景下真正起作用的那条路径。
            a_lim = 0.0
            if out_cap is not None and bool(cfg.get("use_speed_limit", True)):
                over = v_ego - float(out_cap)
                gain = float(cfg.get("limit_closed_loop_gain", 0.5))
                if over > float(cfg.get("limit_deadband_ms", 0.3)) and gain > 0.0:
                    a_lim = -over * gain

            # ★ 陈旧保护：剖面刚失效的一小段时间内保持上一帧的减速度，
            #   不要立刻松开正在起作用的刹车（见 DEFAULTS.stale_hold_sec）。
            #   只对 profile_stale 生效 —— 其余三种 reason 说明"本来就没有
            #   剖面"，此时 _shaped 自然已经是 0，没有可保持的东西。
            prev_shaped = self._shaped
            prev_stop = self.should_stop
            hold = 0.0
            if reason == "profile_stale" and view is not None:
                stale_s = float(cfg.get("profile_stale_sec", 2.5))
                hold_s = max(float(cfg.get("stale_hold_sec", 1.0)), 0.0)
                if (now - view.stamp) <= stale_s + hold_s:
                    hold = prev_shaped

            # a_lim 与 hold 都 <= 0，取更负的那个 = 更保守
            target = min(a_lim, hold)
            self._shaped = self._shape(target, cfg)
            self.a_target = float(self._shaped)
            # 保持期间保留"该停车"的意图，否则临近终点的停车约束会随丢星一起消失
            self.should_stop = bool(hold < -0.01 and prev_stop)
            self.v_cruise_cap = out_cap
            if hold < -0.01:
                self._last_reason = "profile_hold"
            else:
                self._last_reason = "limit_only" if self.a_target < -0.01 else reason
            self._log(now, v_ego, target, self.a_target, out_cap, v_cap, d_cap, self._last_reason)
            return self.a_target, out_cap

        # ---- 速率整形 ----
        self._shaped = self._shape(out_a, cfg)
        self.a_target = float(self._shaped)
        self.v_cruise_cap = out_cap
        # 停车保持不受 v_min 闸门影响：到了终点就该踩住，否则会蠕行
        self.should_stop = bool(out_stop)
        self._last_reason = "applied" if self.a_target < -0.01 else "idle"
        self._log(now, v_ego, out_a, self.a_target, out_cap, v_cap, d_cap, self._last_reason)
        return self.a_target, self.v_cruise_cap

    def _shape(self, target, cfg):
        """减速度速率整形。

        target 恒 <= 0（正的加速度需求一律按 0 处理）。爬升受 max_rise_ms2_per_s
        限制，释放（往 0 回）受 release_ms2_per_s 限制，硬夹在 max_decel_ms2 内。
        结果写回 self._shaped 并返回。
        """
        max_d = abs(float(cfg.get("max_decel_ms2", 2.0)))
        t = _clip(_f(target), -max_d, 0.0)

        rise = abs(float(cfg.get("max_rise_ms2_per_s", 1.5))) * self.dt
        rel = abs(float(cfg.get("release_ms2_per_s", 3.0))) * self.dt

        if t <= self._shaped:
            self._shaped = max(t, self._shaped - rise)
        else:
            self._shaped = min(t, self._shaped + rel)
        return _clip(self._shaped, -max_d, 0.0)

    # ------------------------------------------------------------------
    # 纯计算（无副作用，可单测）
    # ------------------------------------------------------------------
    def _compute(self, v_ego, view, amap_limit, now, cfg):
        """返回 (a_target, v_cruise_cap, reason, v_cap, d_cap, limit_source, should_stop)。

        a_target 恒 <= 0；v_cruise_cap 为 None 表示不限速。
        """
        # ---------- 1) 法定限速 -> v_cruise 上限 ----------
        v_cruise_cap = None
        limit_source = "none"
        if bool(cfg.get("use_speed_limit", True)):
            offset = float(cfg.get("speed_limit_offset_kmh", 0.0)) / 3.6
            limit, limit_source = self._pick_speed_limit(view, amap_limit, cfg)
            if limit is not None and limit > 0.0:
                v_cruise_cap = max(limit + offset, 0.0)

        inf = float("inf")

        # ---------- 2) 速度剖面 -> a_target ----------
        if not bool(cfg.get("use_profile", True)):
            return 0.0, v_cruise_cap, "profile_off", inf, inf, limit_source, False

        if view is None:
            return 0.0, v_cruise_cap, "no_profile", inf, inf, limit_source, False

        stale_s = float(cfg.get("profile_stale_sec", 2.5))
        if (now - view.stamp) > stale_s:
            return 0.0, v_cruise_cap, "profile_stale", inf, inf, limit_source, False

        # 剖面按「沿路线绝对进度」发布，1 Hz。用 v_ego 外推补掉发布延迟，
        # 否则高速下会有一个 ~30 m 的系统性滞后（表现为减速开始得太晚）。
        lag = _clip(now - view.stamp, 0.0, stale_s)
        ego_progress = view.progress + max(v_ego, 0.0) * lag

        d = view.p_progress - ego_progress
        lookahead = float(cfg.get("lookahead_m", 300.0))
        # 用 <= 而不是 <：剖面最后一个点常常正好落在 lookahead 处（终点或铺满位置），
        # 用 < 会把它丢掉，导致「到达停车」缺约束。
        mask = (d > -D_BACK_MARGIN) & (d <= lookahead) & (view.p_cap >= 0.0)
        if not np.any(mask):
            return 0.0, v_cruise_cap, "profile_empty", inf, inf, limit_source, False

        d_sel = d[mask]
        cap_sel = view.p_cap[mask]

        a_dec = max(float(cfg.get("a_dec_ms2", 0.8)), 0.1)
        # 「此刻允许的速度」：v^2 = v_cap^2 + 2*a*d。取最小者即为最紧的约束点。
        v_allow = np.sqrt(np.maximum(cap_sel * cap_sel + 2.0 * a_dec * np.maximum(d_sel, 0.0), 0.0))

        i = int(np.argmin(v_allow))
        v_cap = float(v_allow[i])          # 此刻允许的最高速度（日志 / 停车判定用）
        cap_i = float(cap_sel[i])          # 约束点自身的上限
        d_cap = float(max(d_sel[i], D_MIN))

        # ★ 关键：所需减速度必须用「约束点自身的 cap」，不能用外推后的 v_allow。
        #   例：终点 cap=0、距离 300 m、v=33 m/s -> a=(0-1089)/600=-1.81。
        #   若误用 v_allow=21.9 -> a=-1.0，实际会冲过终点。
        a_needed = (cap_i * cap_i - v_ego * v_ego) / (2.0 * d_cap)
        a_target = min(a_needed, 0.0)

        # 终点停车：约束点自身已接近 0 速，且已经很近
        should_stop = bool(cap_i < 0.5 and d_cap < STOP_D)

        return float(a_target), v_cruise_cap, "ok", v_cap, d_cap, limit_source, should_stop

    def _pick_speed_limit(self, view, amap_limit, cfg):
        """限速来源优先级：Amap（可选优先） > Mapbox 剖面。返回 (m/s 或 None, 来源名)。"""
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
        """从 AmapNavi 自定义消息取限速（m/s）。不可用返回 None。

        ★ 单位换算：navSpeedLimit 是 amap_navi.py 从手机 JSON 原样透传的，没有换算。
        高德 SDK 的 LIMITED_SPEED 是 km/h，但转发 App 也可能转成 m/s，所以按
        amap_limit_unit 解释（默认 auto，见 DEFAULTS 里的说明）。
        """
        if amap_msg is None:
            return None
        try:
            if not bool(getattr(amap_msg, "navValid", False)):
                return None
            # AmapNavi 没有自带时间戳字段，新鲜度只能看 SubMaster 的接收时刻。
            # 拿不到接收时刻时不判过期（由写入侧的 navValid 负责）。
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
            else:  # auto
                thr = float(self.cfg.get("amap_kph_threshold", 45.0))
                limit = raw / 3.6 if raw > thr else raw
            return limit if limit > 0.0 else None
        except Exception:
            return None

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
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
            # 高德来源时额外打印原始值，方便核对单位（auto 模式下最关键的一列）
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

    # ------------------------------------------------------------------
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
