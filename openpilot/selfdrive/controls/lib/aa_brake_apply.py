#!/usr/bin/env python3
"""
aa_brake_apply.py —— AA 紧急刹车请求的「纵向落地」通道（Step-2）

背景
----
`auto_avoidance.py` 的 `_calc_emergency_brake()` 现在已经能算出
「无安全逃逸车道 + 风险已达 emergency_active」时需要多少减速度
（`brake_request`，单位 m/s^2，恒为 >= 0 的正数表示"要减多少速"），
但该值此前只写日志 / 镜像到 `/data/params/d/AvoidBrakeReq`，**未落地**。

本模块负责：把那个请求值，转成对 `controlsd` 纵向通道可用的
**a_target 修正量**。

与本机既有模块的关系
--------------------
完全对齐 `lateral_planner_avoid.py` / `lateral_clearance.py` 的范式：
  - 输出一个"修正量"，由 controlsd 叠加到主链路，**不替代**主链路
  - 默认**关闭**（配置缺失 / enabled=false 时输出恒为 0，行为与原生一致）
  - S1 观测模式 `observe_only=true` 时恒返 0，只写日志
  - 配置热加载（按 mtime）
  - 整个 update 包 try/except，任何异常都返回 0，绝不把异常抛进 controlsd
  - 硬限幅：输出的减速量绝不超过 `max_decel_ms2`

安全设计（本模块是唯一会主动减速的通道，约束最严）
--------------------------------------------------
1. **默认关闭**：`/data/aa_brake_apply.json` 不存在或 enabled!=true 恒返 0。
2. **二次限幅**：即使输入请求值异常大，也被 `max_decel_ms2` 硬夹住。
3. **速率限制**：减速度请求按 `max_rise_ms2_per_s` 爬升，不允许一帧
   从 0 跳到 -3.0（避免乘员前冲 / PID 抖动）。释放则立即（安全方向）。
4. **需 CC.longActive**：未使能纵向时输出 0（由调用方保证，本模块也自查）。
5. **超时闸门**：请求值超过 `req_stale_sec` 未刷新（例如 AA 崩溃/禁用，
   `AvoidBrakeReq` 停在旧值），视为无效 → 输出 0。防止"卡住的刹车"。
6. **车速闸门**：低于 `v_min_ms` 不再介入（已足够慢，交给常规制动）。
7. **异常安全**：任何异常 → 输出 0。

输出契约
--------
    delta_a(m/s^2, <= 0.0)

调用方应做：
    a_target_final = min(a_target_plan, a_target_plan + delta_a)
    等价于 a_target_final = a_target_plan + delta_a   (因为 delta_a <= 0)
即：**只在需要更负（更减速）时介入，永远不会让车加速。**

配置文件：/data/aa_brake_apply.json
请求来源：/data/params/d/AvoidBrakeReq  （由 modeld.py 镜像写入）
"""

import json
import os
import time

CONFIG_PATH = "/data/aa_brake_apply.json"
REQ_PARAM_PATH = "/data/params/d/AvoidBrakeReq"
LOG_PATH = "/data/media/0/aa_brake_apply.log"

DEFAULTS = {
    # ---- 总开关：false 时输出恒为 0，与原生完全一致 ----
    "enabled": False,
    # ---- S1 观测模式：true 时恒返 0，只写日志（先看数不生效）----
    "observe_only": True,

    # ---- 日志 ----
    "log": True,
    "log_period_s": 1.0,

    # ---- 限幅 / 形状 ----
    # 最终叠加到 a_target 的最负修正量上限（m/s^2）。硬夹。
    "max_decel_ms2": 3.0,
    # 修正量爬升速率上限（m/s^2 per s），防止一帧突变。
    "max_rise_ms2_per_s": 2.0,
    # 请求值缩放系数（1.0 = 原样采信 AA 的请求）
    "gain": 1.0,

    # ---- 闸门 ----
    # 低于此车速不介入（km/h）
    "v_min_kmh": 25.0,
    # 请求值超过该秒数未刷新则视为失效 -> 输出 0
    "req_stale_sec": 0.5,
}


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class AABrakeApply:
    """把 AA 的 brake_request 转成 a_target 修正量。"""

    def __init__(self):
        self.cfg = dict(DEFAULTS)
        self._cfg_mtime = None
        self._shaped = 0.0          # 当前已"爬升整形"过的修正量（<= 0）
        self._req = 0.0             # 最近一次读到的请求值（> 0 表示要减速）
        self._req_t = 0.0           # 最近一次"心跳"时间
        self._req_stamp = None      # 内容内嵌时间戳（首选心跳源）
        self._req_content_prev = None  # 无时间戳时的退化心跳源
        self._last_req_raw = None
        self._last_log_t = 0.0
        self._last_reason = "init"
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
                    if k in cfg and not isinstance(cfg[k], dict):
                        cfg[k] = v
                    elif isinstance(v, dict) and isinstance(cfg.get(k), dict):
                        cfg[k].update(v)
            self.cfg = cfg
            self._cfg_mtime = st.st_mtime
        except Exception:
            # 配置缺失/损坏 -> 保持默认（默认 enabled=False，安全）
            if force:
                self.cfg = dict(DEFAULTS)
                self._cfg_mtime = None

    # ------------------------------------------------------------------
    # 读请求值
    # ------------------------------------------------------------------
    def _read_req(self, now):
        """读 /data/params/d/AvoidBrakeReq。返回值 (req, req_t)。

        文件格式（由 modeld 写入）：
            "<减速度请求>:<monotonic秒>"
        例：  "2.880:12345.678"
        兼容旧格式：纯数字（无冒号）仍可解析，此时退化为"内容变化即刷新"。

        ★ 为什么用内容内嵌时间戳，而不是文件 mtime：
          1. mtime 与模块内部用于 staleness 的 `now` 不是同一时钟域，
             两者做差值在负载抖动时不可靠；
          2. 实测 tmpfs/relatime 场景下，短时间内重复写相同内容可能
             得到相同 mtime，导致"写入侧活着"被误判为"已死"，进而
             误杀正在生效的刹车 —— 这是不可接受的安全缺陷。
          内嵌时间戳由写入侧用 time.monotonic() 生成，与消费侧同域，
          只要写入侧还在跑，该值就必然推进。
        """
        raw = None
        try:
            with open(REQ_PARAM_PATH, "r") as f:
                raw = f.read().strip()
        except Exception:
            raw = None

        if not raw:
            # 文件不存在 == AA 未在运行 / 未产生请求 -> 视为 0
            self._last_req_raw = None
            self._req_stamp = None
            return 0.0, now

        self._last_req_raw = raw

        # ---- 解析 "<val>:<stamp>" ----
        val_s, stamp_s = raw, None
        if ":" in raw:
            parts = raw.split(":")
            if len(parts) >= 2:
                val_s = parts[0]
                stamp_s = parts[1]

        # ---- 心跳判定 ----
        if stamp_s is not None:
            # 首选：内容内嵌时间戳（同域单调）
            if stamp_s != self._req_stamp:
                self._req_stamp = stamp_s
                self._req_t = now
        else:
            # 退化：无时间戳 -> 用内容变化当心跳
            if raw != self._req_content_prev:
                self._req_content_prev = raw
                self._req_t = now

        try:
            v = float(val_s)
        except (TypeError, ValueError):
            return 0.0, now

        if v != v or v < 0.0:      # NaN / 负数
            return 0.0, now
        return v, self._req_t

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def update(self, *, v_ego, long_active, now=None):
        """返回 delta_a（<= 0.0）。任何异常 -> 0.0。"""
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

        # ---- 闸门 1: 未使能 / 观测模式 -> 恒 0 ----
        if not enabled or observe_only:
            self._shaped = 0.0
            self._last_reason = "observe_only" if enabled else "disabled"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        # ---- 闸门 2: 纵向未激活 -> 0 ----
        if not bool(long_active):
            self._shaped = 0.0
            self._last_reason = "long_inactive"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        # ---- 闸门 3: 车速太低 -> 0 ----
        v_min = float(cfg.get("v_min_kmh", 25.0)) / 3.6
        if v_ego < v_min:
            self._shaped = 0.0
            self._last_reason = "v_low"
            self._log(now, v_ego, 0.0, 0.0)
            return 0.0

        # ---- 读请求 ----
        req, req_t = self._read_req(now)

        # ---- 闸门 4: 请求过期 -> 0 ----
        stale_s = float(cfg.get("req_stale_sec", 0.5))
        if req > 0.0 and (now - req_t) > stale_s:
            self._shaped = 0.0
            self._last_reason = "req_stale"
            self._log(now, v_ego, req, 0.0)
            return 0.0

        # ---- 目标修正量（负数）----
        gain = float(cfg.get("gain", 1.0))
        max_d = abs(float(cfg.get("max_decel_ms2", 3.0)))
        target = -_clip(gain * req, 0.0, max_d)

        # ---- 整形：减速方向可快速爬升但受速率限制；释放立即 ----
        dt = 1.0 / 20.0     # controlsd DT_CTRL == 0.01; 用保守 20Hz 上界做步长
        try:
            from openpilot.common.realtime import DT_CTRL as _DT
            dt = float(_DT)
        except Exception:
            pass
        rise = abs(float(cfg.get("max_rise_ms2_per_s", 2.0))) * max(dt, 1e-3)

        if target <= self._shaped:
            # 需要更负 -> 按 rise 逐步逼近
            self._shaped = max(target, self._shaped - rise)
        else:
            # 需要放松 -> 立即（安全方向）
            self._shaped = target

        out = _clip(self._shaped, -max_d, 0.0)
        self._last_reason = "applied" if out < 0.0 else "idle"
        self._log(now, v_ego, req, out)
        return float(out)

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
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
            # 简单大小保护：超过 2MB 直接截断重来
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
            "out": float(self._shaped),
            "req": float(self._last_req_raw or 0.0) if self._last_req_raw else 0.0,
            "why": self._last_reason,
        }
