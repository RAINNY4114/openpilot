#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto Avoidance Helper
=====================

EMERGENCY LATERAL AVOIDANCE ONLY.

This module exists ONLY for genuine imminent collision avoidance.

It does NOT perform:

    - normal overtaking
    - normal lane change
    - cruise control
    - longitudinal planning
    - braking control
    - throttle control
    - radar lead control
    - direct steering actuator control
    - CAN transmission

MR76 is AUXILIARY ONLY.

MR76 MUST NEVER:

    - become leadOne
    - become leadTwo
    - replace OEM radar
    - modify OEM lead distance
    - modify OEM lead relative speed
    - modify longitudinalPlan
    - modify aTarget
    - modify shouldStop
    - directly command CAN
    - publish radarState

NOTE: brake_request is a *request* only.  It never becomes a CAN command
here, and it is 0.0 unless no safe escape lane exists (see below).


Emergency architecture
----------------------

OEM radar / filtered obstacle
              |
              v
      Emergency risk
              |
              v
    AutoAvoidanceHelper
              |
              +---- LaneChangeDirection
              |
              +---- hazard
              |
                +---- lane_offset diagnostic only
                |
                +---- brake_request (0.0 by default)
                |
                v
         DesireHelper

[AA_BRAKE_FALLBACK] brake_request 的两层决策
--------------------------------------------

默认 brake_request == 0.0（纵向不介入，与旧版一致）。

仅当以下条件【同时】满足时，才输出非零减速请求：

    1. 风险已达 emergency_active（URGENT/CRITICAL 级）
    2. 没有任何安全逃逸车道可用（_pick_out_direction 返回 none）

即「安全前提下优先转向避让；转向不可行时优先刹车」。

该请求：
    - 受 avoid_brake_max 硬限幅（默认 0.0 == 完全关闭）
    - 随 TTC 线性收敛，TTC >= avoid_brake_ttc_ref 时不请求
    - 任何异常/缺数据都回落到 0.0
    - 由接线侧决定如何转成执行器命令（本模块不直接命令 CAN）


MR76 + side LiDAR
-----------------

MR76 and side WiFi LiDAR are NOT longitudinal sensors here.

They are used to determine whether an escape lane is safe.

For emergency automatic avoidance:

    missing
    invalid
    stale

auxiliary sensor data is NOT treated as "clear".

The destination lane must have sufficient independent sensor
confirmation before an automatic escape request is generated.


Direction lock
--------------

Once an actual emergency lane change has started:

    - direction is locked
    - temporary sensor loss does not cause an opposite-direction command
    - the module never oscillates left/right

This module is a decision layer, not an actuator.
"""

import math
import time

from openpilot.cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Parameters
# ============================================================================

AUTO_AVOID_MIN_SPEED = (
    25.0 * CV.KPH_TO_MS
)


# --------------------------------------------------------------------------
# Emergency risk
# --------------------------------------------------------------------------

TTC_WARNING = 5.0
TTC_URGENT = 3.5
TTC_CRITICAL = 2.0

DIST_WARNING = 30.0
DIST_URGENT = 20.0
DIST_CRITICAL = 12.0

CLOSING_WARNING = 2.0
CLOSING_URGENT = 4.0
CLOSING_CRITICAL = 7.0

# --------------------------------------------------------------------------
# [AA_TTC_DIST_GATE] TTC 分支的"绝对距离护栏"。
#
# 语义与 DIST_* 完全不同：
#   DIST_*        = 风险分级的**判定**距离门（近距离才升级）
#   本组护栏      = 拦截**物理上不可能紧急**的远距离目标
#
# 因此护栏必须显著宽松于 DIST_CRITICAL / DIST_URGENT。若直接复用
# DIST_CRITICAL=12m 去卡 TTC_CRITICAL 分支，会把「13m、rel=-25m/s、
# TTC=0.52s」这种极端紧急场景错误降级为 URGENT —— 那是安全退化。
# （该退化已在纯函数穷举中实测到，故改用独立护栏。）
#
# 取值依据：
#   - 30m 内若 TTC<=2.0s，接近速度必然 >= 15m/s 的相对运动，真紧急。
#     30m 外 TTC<=2s 意味着 rel<=-15m/s，该相对速度在城市/高速场景
#     几乎不会出现于远距离静止目标（多为对向车/弯道内侧车误测）。
#   - 40m 外的目标不应仅凭 TTC 升 URGENT，必须退回 distance+closing
#     联合判定（那里有 DIST_URGENT=20m 的真实距离门）。
# --------------------------------------------------------------------------
TTC_CRITICAL_DIST_GUARD = 30.0
TTC_URGENT_DIST_GUARD = 40.0


# --------------------------------------------------------------------------
# Confirmation
# --------------------------------------------------------------------------

CRITICAL_CONFIRM_FRAMES = 3

CLEAR_LANE_STABLE_SEC = 0.30

MIN_ACTIVE_SEC = 0.40

AVOID_COOLDOWN_SEC = 1.5

# --------------------------------------------------------------------------
# [AA_ESC_ABORT] 逃逸请求的"风险消失"脱困窗口。
#
# 缺陷背景（route 43 实测）：
#   avoiding 状态在 _observed_lane_change == False（变道尚未物理启动）
#   时，**没有任何退出路径**。只要方向持续可用（direction_safe 恒真），
#   就会一直 lane_request = self._out_dir，永久停留在 avoiding。
#   实测一次误判 URGENT 后，AA 持续输出变道方向 825 帧（约 41 秒），
#   期间 risk 早已回落（401 帧 risk=0、333 帧 risk=1），why 被逐帧
#   改写成 "normal"/"warning_only"，完全掩盖了真实触发原因 —— 这正是
#   用户感受到的"自动变道逻辑混乱、频繁提示自动变道"。
#
# 修复：若变道尚未启动，且风险已回落到非紧急状态并持续本窗口，
#       则主动放弃逃逸请求，转入 cooldown 后再回 idle。
#
# 安全性说明：
#   - 仅在 _observed_lane_change == False（车还没真的开始变道）时生效，
#     绝不会在变道进行中打断 manoeuvre（避免侧向失稳）。
#   - 需要"持续非紧急"满 AVOID_ESC_ABORT_SEC 才放弃，瞬时的风险回落
#     不会误杀真实逃逸（真实紧急时 emergency_active 会保持为真）。
#   - 放弃后进入 AVOID_COOLDOWN_SEC 冷却，与正常退出路径一致。
# --------------------------------------------------------------------------
AVOID_ESC_ABORT_SEC = 2.0

# --------------------------------------------------------------------------
# Rear / side traffic
# --------------------------------------------------------------------------

REAR_DIST_TH = 8.0

REAR_CLOSING_SPEED_TH = 15.0

REAR_SECONDARY_DIST_TH = 15.0

REAR_SECONDARY_CLOSING_SPEED_TH = 10.0


# --------------------------------------------------------------------------
# Sensor freshness
# --------------------------------------------------------------------------

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25

# ---------------------------------------------------------------------------
# Auxiliary-sensor gating
#
# MR76 is physically present and healthy on this car; the side WiFi LiDAR is
# installed but NOT wired into openpilot, so it never publishes and must not be
# allowed to veto by its own absence.
#
#   require_aux_sensors (the historical master flag) is kept, but it now defers
#   to the two per-sensor flags below.  A caller that leaves everything at its
#   default gets: MR76 required, LiDAR optional.
#
# "LiDAR optional" means: if LiDAR data IS present it can still block an escape
# lane (fail-safe), it just cannot block by being absent.
# ---------------------------------------------------------------------------
AVOID_REQUIRE_MR76_DEFAULT = True
AVOID_REQUIRE_LIDAR_DEFAULT = False

# ---------------------------------------------------------------------------
# Escape-lane stability policy
#
# "strict"  : lane must be OK continuously for `stable_sec` (historical).
# "ratio"   : over a sliding window of `window_sec`, the fraction of OK frames
#             must be >= `ratio`.  Tolerates the BSM flicker seen in traffic.
#
# Default "strict" == unchanged behaviour.
# ---------------------------------------------------------------------------
AVOID_STABLE_MODE_DEFAULT = "strict"
AVOID_STABLE_RATIO_DEFAULT = 0.60
AVOID_STABLE_WINDOW_SEC_DEFAULT = 0.60

# Number of consecutive frames a sensor must report "blocked" before the
# blocker is honoured.  0 == off (historical behaviour).
AVOID_BSM_DEBOUNCE_DEFAULT = 0

# ---------------------------------------------------------------------------
# [AA_BRAKE_FALLBACK] 无安全逃逸车道时的纵向兜底参数。
#
# brake_gain    : 0..1，在 TTC 极小时占 brake_max 的比例
# brake_max     : 硬上限 (m/s^2)，防止请求过猛
# brake_ttc_ref : TTC 参考值 (s)，达到此值即不再请求减速
#
# brake_max = 0.0 时该通道完全关闭，行为与旧版一致。
# ---------------------------------------------------------------------------
AVOID_BRAKE_GAIN_DEFAULT = 1.0
AVOID_BRAKE_MAX_DEFAULT = 0.0
AVOID_BRAKE_TTC_REF_DEFAULT = 3.0

# Whether the caller has any rear-side sensing at all.  When 0, the
# rear-distance gate cannot block (there is simply nothing to report), and BSM
# remains the side/rear veto.
AVOID_REAR_PRESENT_DEFAULT = 1

# ---------------------------------------------------------------------------
# Lateral-intrusion risk channel
#
# Independent of the forward/lead channel.  Fires when a vehicle occupies a
# side blind spot at high speed for a sustained period -- the signature of the
# 106 km/h tunnel intrusion (route 37 seg 11) which the lead channel cannot see
# because the intruder is BESIDE the car, not ahead of it.
# Default 0 == disabled (historical behaviour preserved).
AVOID_LATERAL_ENABLE_DEFAULT = 0
AVOID_LATERAL_MIN_SPEED_DEFAULT = 16.67   # m/s == 60 km/h
AVOID_LATERAL_CONFIRM_DEFAULT = 5         # consecutive frames (~0.25 s @20Hz BSM)
AVOID_LATERAL_CLOSING_MIN_DEFAULT = 0.0   # m/s; 0 == do not require closing rate

# ---------------------------------------------------------------------------
# Risk-input hardening
#
# A single-frame leadOne spike must not be able to start an emergency lane
# change.  These two knobs are OFF by default; turning them on only ever
# REDUCES the number of triggers.
#
#   avoid_lead_lpf_alpha : first-order IIR coefficient for (dRel, vRel).
#                          0.0 == disabled (raw values, historical).
#                          0.35 == ~3-frame time constant at 100 Hz.
#   avoid_urgent_confirm : consecutive URGENT frames required before the state
#                          machine may act.  0 == historical (immediate).
# ---------------------------------------------------------------------------
AVOID_LEAD_LPF_ALPHA_DEFAULT = 0.0
AVOID_URGENT_CONFIRM_DEFAULT = 0


# --------------------------------------------------------------------------
# Diagnostic lateral offset
# --------------------------------------------------------------------------

EMERGENCY_OFFSET = 0.8


# --------------------------------------------------------------------------
# Risk levels
# --------------------------------------------------------------------------

RISK_NORMAL = 0
RISK_WARNING = 1
RISK_URGENT = 2
RISK_CRITICAL = 3


# ============================================================================
# Utility
# ============================================================================

def _safe_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if value != value:
        return None

    return value


def _sensor_fresh(
    available,
    valid,
    age,
    max_age,
):
    """
    Explicit sensor health check.

    False / None / stale is NOT safe.
    """

    if not bool(available):
        return False

    if valid is not None and not bool(valid):
        return False

    if age is None:
        return True

    age = _safe_float(age)

    if age is None:
        return False

    if age < 0.0:
        return False

    return age <= max_age


def compute_ttc(
    distance,
    relative_speed,
):
    """
    relative_speed convention:

        positive:
            target moving away / less closing

        negative:
            target approaching ego

    Returns:
        TTC seconds, or None if not closing.
    """

    distance = _safe_float(distance)
    relative_speed = _safe_float(relative_speed)

    if distance is None or relative_speed is None:
        return None

    if distance <= 0.0:
        return 0.0

    closing_speed = -relative_speed

    if closing_speed <= 0.01:
        return None

    return distance / closing_speed


def generate_smooth_evasive_path(
    current_path_len,
    target_offset,
):
    """
    Diagnostic/path utility only.

    It does NOT directly command steering.
    """

    try:
        current_path_len = int(
            current_path_len
        )
    except (TypeError, ValueError):
        return []

    if current_path_len <= 0:
        return []

    try:
        target_offset = float(
            target_offset
        )
    except (TypeError, ValueError):
        target_offset = 0.0

    if current_path_len == 1:
        return [target_offset]

    path = []

    for i in range(current_path_len):
        t = i / float(
            current_path_len - 1
        )

        y = target_offset * (
            3.0 * t * t
            - 2.0 * t * t * t
        )

        path.append(y)

    return path


# ============================================================================
# Helper
# ============================================================================

class AutoAvoidanceHelper:

    def __init__(self):

        self._mode = "idle"

        self._out_dir = (
            LaneChangeDirection.none
        )

        self._cooldown_until = 0.0

        self._clear_since = None

        self._active_since = None

        self._emergency_since = None

        self._last_lc_state = (
            LaneChangeState.off
        )

        self._observed_lane_change = False

        self._left_ok_since = None
        self._right_ok_since = None

        # Lateral-intrusion channel state.
        self._lateral_left_frames = 0
        self._lateral_right_frames = 0
        self._lateral_active = False
        self._lateral_dir = 0

        self._critical_frames = 0

        # Risk-input hardening state (see AVOID_LEAD_LPF_ALPHA_DEFAULT).
        self._lead_lpf_d = None
        self._lead_lpf_v = None
        self._urgent_frames = 0

        self._risk_level = RISK_NORMAL

        self._last_reason = "idle"

        # [AA_ESC_ABORT] 风险消失脱困计时器（见 AVOID_ESC_ABORT_SEC）。
        # 独立于 _clear_since，避免与"变道完成后清空"的语义冲突。
        self._esc_abort_since = None

    # ==================================================================
    # Reset
    # ==================================================================

    def reset(self):
        self.__init__()

    def _reset_stability_histories(self):
        self._left_ok_hist = []
        self._right_ok_hist = []
        self._bsm_l_run = 0
        self._bsm_r_run = 0

    def _reset_risk_filters(self):
        self._lead_lpf_d = None
        self._lead_lpf_v = None
        self._urgent_frames = 0
        self._lateral_left_frames = 0
        self._lateral_right_frames = 0
        self._lateral_active = False
        self._lateral_dir = 0
        # [AA_ESC_ABORT] 风险输入被复位时，脱困计时器一并清零。
        self._esc_abort_since = None

    # ==================================================================
    # Stable safety
    # ==================================================================

    @staticmethod
    def _stable_ok(
        ok,
        ok_since,
        now,
        stable_sec,
        history=None,
        mode=None,
        ratio=None,
        window_sec=None,
    ):
        """Escape-lane stabilisation.

        mode "strict" (default, historical): require `ok` to hold continuously
        for `stable_sec`.  Any False frame resets the timer.

        mode "ratio": keep a short history of (timestamp, ok) and require that
        at least `ratio` of the samples inside the last `window_sec` seconds are
        OK.  This tolerates sensor flicker while still rejecting a lane that is
        genuinely occupied.  `ok_since` is still maintained for reporting.
        """

        mode = AVOID_STABLE_MODE_DEFAULT if mode is None else str(mode)

        # ---------------- historical path ----------------

        if mode != "ratio" or history is None:
            if not ok:
                return None, False

            if ok_since is None:
                ok_since = now

            return (
                ok_since,
                (now - ok_since) >= stable_sec,
            )

        # ---------------- sliding-window ratio path ----------------

        if ratio is None:
            ratio = AVOID_STABLE_RATIO_DEFAULT

        if window_sec is None:
            window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        history.append((now, bool(ok)))

        # drop old samples
        cutoff = now - window_sec
        while history and history[0][0] < cutoff:
            history.pop(0)

        n = len(history)
        n_ok = sum(1 for _t, v in history if v)

        # do not judge until we have seen a meaningful slice of the window
        covered = (now - history[0][0]) if history else 0.0
        if n < 2 or covered < window_sec * 0.5:
            if not ok:
                return None, False
            if ok_since is None:
                ok_since = now
            return ok_since, False

        frac = n_ok / float(n)

        if not ok:
            ok_since = None
        elif ok_since is None:
            ok_since = now

        return (
            ok_since,
            frac >= float(ratio),
        )

    # ==================================================================
    # Opposite
    # ==================================================================

    @staticmethod
    def _opposite(direction):

        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right

        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left

        return LaneChangeDirection.none

    # ==================================================================
    # Rear traffic
    # ==================================================================

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
        rear_sensing_present=True,
    ):
        """
        Conservative emergency escape-lane veto.

        `rear_sensing_present=False` means the vehicle has no rear-side sensor
        at all, so the absence of a reading carries no information.  In that
        case this gate abstains (BSM still vetoes from the side/rear).
        """

        distance = _safe_float(distance)

        if distance is None:
            # No reading.  Block only if the caller claims to HAVE rear sensing
            # (then "no data" really is suspicious); otherwise abstain.
            return bool(rear_sensing_present)

        if distance <= 0.0:
            return True

        if distance < REAR_DIST_TH:
            return True

        closing_speed = _safe_float(
            closing_speed
        )

        if closing_speed is None:
            return True

        if (
            distance < REAR_SECONDARY_DIST_TH
            and
            closing_speed
            > REAR_SECONDARY_CLOSING_SPEED_TH
        ):
            return True

        if (
            distance < REAR_DIST_TH * 2.0
            and
            closing_speed
            > REAR_CLOSING_SPEED_TH
        ):
            return True

        return False

    # ==================================================================
    # Escape lane safety
    # ==================================================================

    def _escape_lane_safe(
        self,
        *,
        base_lane_ok,
        require_aux_sensors_mr76=None,
        require_aux_sensors_lidar=None,
        rear_sensing_present=None,

        rear_dist,
        rear_speed,

        mr76_available,
        mr76_valid,
        mr76_age,

        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,

        bsm_blocked,
        bsm_available,

        require_aux_sensors,
    ):
        """
        Emergency escape lane must pass all required gates.

        Base lane
            AND
        MR76
            AND
        LiDAR
            AND
        BSM
        """

        if not bool(base_lane_ok):
            return False

        # --------------------------------------------------------------
        # BSM
        # --------------------------------------------------------------

        if (
            bool(bsm_available)
            and
            bool(bsm_blocked)
        ):
            return False

        # --------------------------------------------------------------
        # MR76
        # --------------------------------------------------------------

        mr76_ok = _sensor_fresh(
            mr76_available,
            mr76_valid,
            mr76_age,
            MR76_MAX_AGE_SEC,
        )

        if require_aux_sensors_mr76 is None:
            require_aux_sensors_mr76 = (
                AVOID_REQUIRE_MR76_DEFAULT
                if require_aux_sensors is None
                else bool(require_aux_sensors)
            )

        # The rear-distance veto below is meaningless without rear sensing.
        # Callers that know their vehicle has none pass rear_sensing_present=0
        # (or leave dp_avoid_rear_present at 0) and the gate simply abstains.
        if not bool(rear_sensing_present):
            mr76_ok_for_rear = False
        else:
            mr76_ok_for_rear = mr76_ok

        if require_aux_sensors_mr76 and not mr76_ok:
            return False

        if mr76_ok_for_rear:
            if self._rear_lane_blocked(
                rear_dist,
                rear_speed,
                rear_sensing_present=rear_sensing_present,
            ):
                return False

        # --------------------------------------------------------------
        # LiDAR
        # --------------------------------------------------------------

        lidar_ok = _sensor_fresh(
            lidar_available,
            lidar_valid,
            lidar_age,
            LIDAR_MAX_AGE_SEC,
        )

        if require_aux_sensors_lidar is None:
            require_aux_sensors_lidar = (
                AVOID_REQUIRE_LIDAR_DEFAULT
                if require_aux_sensors is None
                else bool(require_aux_sensors)
            )

        # LiDAR is optional on this car: when there is no LiDAR data at all we
        # must NOT treat that as "lane blocked".  When data IS present, the
        # `lidar_free is not True` check below still applies as a fail-safe.
        if require_aux_sensors_lidar and not lidar_ok:
            return False

        if lidar_ok:
            if lidar_free is not True:
                return False

        return True

    # ==================================================================
    # Direction selection
    # ==================================================================

    @staticmethod
    def _pick_out_direction(
        left_ok,
        right_ok,
        is_rhd,
        prefer_dir=LaneChangeDirection.none,
    ):
        """
        Select ONLY from already-confirmed safe lanes.

        No direction is selected solely because of RHD/LHD.
        """

        if (
            prefer_dir == LaneChangeDirection.left
            and left_ok
        ):
            return LaneChangeDirection.left

        if (
            prefer_dir == LaneChangeDirection.right
            and right_ok
        ):
            return LaneChangeDirection.right

        if left_ok and right_ok:

            # Existing convention is only a tie breaker.
            if is_rhd:
                return LaneChangeDirection.right

            return LaneChangeDirection.left

        if left_ok:
            return LaneChangeDirection.left

        if right_ok:
            return LaneChangeDirection.right

        return LaneChangeDirection.none

    # ==================================================================
    # ==================================================================
    # [AA_BRAKE_FALLBACK] 紧急纵向兜底
    # ==================================================================

    @staticmethod
    def _calc_emergency_brake(
        *,
        obstacle_dist,
        obstacle_rel_speed,
        lead_dist,
        lead_rel_speed,
        v_ego,
        brake_gain=None,
        brake_max=None,
        brake_ttc_ref=None,
    ):
        """
        计算紧急减速请求（返回 m/s^2 的正数，表示"要减多少速"）。

        仅用于「无安全逃逸车道」的兜底路径。判据与 _classify_risk 保持一致：
          - 优先用障碍物通道（本机 = radarTracks）
          - 障碍物缺失时回落到 OEM lead
          - 两者都没有 -> 返回 0.0（不介入）

        幅度策略：
          TTC 越小，请求越大；TTC >= brake_ttc_ref 时不请求。
          公式：req = brake_gain * (1 - ttc / brake_ttc_ref)，再乘 v_ego 归一。
          最终受 brake_max 硬限幅。
        """

        if brake_gain is None:
            brake_gain = AVOID_BRAKE_GAIN_DEFAULT

        if brake_max is None:
            brake_max = AVOID_BRAKE_MAX_DEFAULT

        if brake_ttc_ref is None:
            brake_ttc_ref = AVOID_BRAKE_TTC_REF_DEFAULT

        brake_max = max(0.0, float(brake_max))
        if brake_max <= 0.0:
            return 0.0

        v = _safe_float(v_ego)
        if v is None or v <= 0.0:
            return 0.0

        # ---- 选择有效的前向通道 ----
        distance = _safe_float(obstacle_dist)
        rel = _safe_float(obstacle_rel_speed)

        if distance is None or rel is None:
            distance = _safe_float(lead_dist)
            rel = _safe_float(lead_rel_speed)

        if distance is None or rel is None:
            return 0.0

        if distance <= 0.0:
            # 已经贴上了：给最大值
            return float(brake_max)

        ttc = compute_ttc(distance, rel)
        if ttc is None:
            # 相对速度非接近（rel >= 0）：不需要减速
            return 0.0

        ref = max(0.1, float(brake_ttc_ref))
        if ttc >= ref:
            return 0.0

        # 线性收敛：ttc=0 时满额，ttc=ref 时 0
        urgency = (ref - ttc) / ref
        urgency = max(0.0, min(1.0, urgency))

        req = float(brake_gain) * urgency * float(brake_max)

        if not math.isfinite(req):
            return 0.0

        return float(max(0.0, min(brake_max, req)))

    # ==================================================================
    # Risk classification
    # ==================================================================

    @staticmethod
    def _classify_risk(
        *,
        obstacle_in_path,
        obstacle_dist,
        obstacle_rel_speed,

        lead_dist,
        lead_rel_speed,

        is_pedestrian,
        is_cone,

        imminent_collision,
    ):
        """
        Risk classification ONLY.

        This function:

            - does not brake
            - does not modify OEM lead
            - does not create radar targets
        """

        if bool(imminent_collision):
            return RISK_CRITICAL

        # --------------------------------------------------------------
        # Filtered obstacle
        # --------------------------------------------------------------

        if bool(obstacle_in_path):

            distance = _safe_float(
                obstacle_dist
            )

            rel = _safe_float(
                obstacle_rel_speed
            )

            if (
                distance is not None
                and
                rel is not None
            ):

                ttc = compute_ttc(
                    distance,
                    rel,
                )

                # ------------------------------------------------------
                # [AA_TTC_DIST_GATE] TTC 分支的距离护栏。
                #
                # 原实现只用 TTC 判定，完全没有距离约束，导致远距离目标
                # （例如 58m 外、相对速度较大但实际无害的车）仅凭 TTC
                # 就被判为 URGENT/CRITICAL，进而触发自动变道语音与逃逸
                # 请求 —— 这是"经常提示自动左/右变道"的根因。
                #
                # 实测误判（route 43）：
                #   d=58.4 rel=-18.6 -> URGENT    (58m 外！)
                #   d=50.7 rel=-24.1 -> URGENT
                #   d=39.7 rel=-24.0 -> CRITICAL  (39m 外判 CRITICAL！)
                #
                # ★ 关键设计：护栏距离**必须显著大于** DIST_CRITICAL /
                #   DIST_URGENT，绝不能直接复用它们。原因：TTC 本身已经
                #   编码了时间信息，一个 d=13m、rel=-25m/s（TTC=0.52s）的
                #   目标是极度紧急的，若用 DIST_CRITICAL=12m 硬卡，会把
                #   13m 这种真紧急场景错误降级 —— 那是安全退化！
                #
                #   护栏只负责拦截"远到物理上不可能紧急"的目标：
                #     - CRITICAL 护栏 30m：30m 内 TTC<=2s 必为真紧急
                #     - URGENT   护栏 40m：40m 外的目标不该仅凭 TTC 升级，
                #       应退回下面的 distance + closing 联合判定
                # ------------------------------------------------------

                if (
                    ttc is not None
                    and
                    ttc <= TTC_CRITICAL
                    and
                    distance <= TTC_CRITICAL_DIST_GUARD
                ):
                    return RISK_CRITICAL

                if (
                    ttc is not None
                    and
                    ttc <= TTC_URGENT
                    and
                    distance <= TTC_URGENT_DIST_GUARD
                ):
                    return RISK_URGENT

                closing = max(
                    0.0,
                    -rel,
                )

                if (
                    distance <= DIST_CRITICAL
                    and
                    closing >= CLOSING_CRITICAL
                ):
                    return RISK_CRITICAL

                if (
                    distance <= DIST_URGENT
                    and
                    closing >= CLOSING_URGENT
                ):
                    return RISK_URGENT

                if (
                    distance <= DIST_WARNING
                    and
                    closing >= CLOSING_WARNING
                ):
                    return RISK_WARNING

            # ----------------------------------------------------------
            # Pedestrian / cone
            #
            # Boolean classification alone is not enough to create
            # emergency steering.
            # ----------------------------------------------------------

            if (
                bool(is_pedestrian)
                or
                bool(is_cone)
            ):

                if (
                    distance is not None
                    and
                    distance <= DIST_URGENT
                ):
                    return RISK_URGENT

                return RISK_WARNING

            return RISK_WARNING

        # --------------------------------------------------------------
        # OEM lead emergency condition
        #
        # OEM lead is READ ONLY here.
        # --------------------------------------------------------------

        distance = _safe_float(
            lead_dist
        )

        rel = _safe_float(
            lead_rel_speed
        )

        if (
            distance is not None
            and
            rel is not None
        ):

            closing = max(
                0.0,
                -rel,
            )

            ttc = compute_ttc(
                distance,
                rel,
            )

            if (
                ttc is not None
                and
                ttc <= TTC_CRITICAL
                and
                closing >= CLOSING_CRITICAL
            ):
                return RISK_CRITICAL

            if (
                distance <= DIST_CRITICAL
                and
                closing >= CLOSING_CRITICAL
            ):
                return RISK_CRITICAL

            if (
                ttc is not None
                and
                ttc <= TTC_URGENT
                and
                closing >= CLOSING_URGENT
            ):
                return RISK_URGENT

            if (
                distance <= DIST_URGENT
                and
                closing >= CLOSING_URGENT
            ):
                return RISK_URGENT

            if (
                ttc is not None
                and
                ttc <= TTC_WARNING
                and
                closing >= CLOSING_WARNING
            ):
                return RISK_WARNING

        return RISK_NORMAL

    # ==================================================================
    # Main
    # ==================================================================

    def update(
        self,
        *,
        enabled,

        obstacle_in_path=False,

        lc_state=LaneChangeState.off,

        v_ego=0.0,

        left_ok=False,
        right_ok=False,

        is_rhd=False,

        manual_blinker=False,

        bsm_available=True,

        obstacle_dist=None,
        obstacle_rel_speed=None,

        lead_dist=None,
        lead_rel_speed=None,

        is_pedestrian=False,
        is_cone=False,

        imminent_collision=False,

        # --------------------------------------------------------------
        # Legacy rear interface
        # --------------------------------------------------------------

        rear_left_dist=None,
        rear_left_speed=None,
        rear_left_available=False,

        rear_right_dist=None,
        rear_right_speed=None,
        rear_right_available=False,

        # --------------------------------------------------------------
        # Legacy LiDAR interface
        # --------------------------------------------------------------

        left_lidar_free=None,
        right_lidar_free=None,

        left_bsm_blocked=False,
        right_bsm_blocked=False,

        prefer_dir=LaneChangeDirection.none,

        # --------------------------------------------------------------
        # Explicit MR76 interface
        # --------------------------------------------------------------

        mr76_left_available=None,
        mr76_right_available=None,

        mr76_left_valid=None,
        mr76_right_valid=None,

        mr76_left_age=None,
        mr76_right_age=None,

        # --------------------------------------------------------------
        # Explicit LiDAR interface
        # --------------------------------------------------------------

        left_lidar_available=None,
        right_lidar_available=None,

        left_lidar_valid=None,
        right_lidar_valid=None,

        left_lidar_age=None,
        right_lidar_age=None,

        # --------------------------------------------------------------
        # Integration policy
        # --------------------------------------------------------------

        require_aux_sensors=True,

        # Per-sensor overrides.  None == "follow require_aux_sensors".
        # Defaults chosen for this car: MR76 required, LiDAR optional.
        require_aux_sensors_mr76=None,
        require_aux_sensors_lidar=None,

        # Escape-lane stability policy.  None == module default (strict).
        avoid_stable_mode=None,
        avoid_stable_ratio=None,
        avoid_stable_window_sec=None,

        # [AA_BRAKE_FALLBACK] 无安全逃逸车道时的纵向兜底。
        avoid_brake_gain=None,
        avoid_brake_max=None,
        avoid_brake_ttc_ref=None,

        # BSM blocker debounce (frames).  None/0 == off.
        bsm_debounce_frames=None,

        # Rear-side sensing presence.  None == module default (present).
        rear_sensing_present=None,

        # Risk-input hardening.  None == module default (off).
        avoid_lead_lpf_alpha=None,
        avoid_urgent_confirm=None,

        # Lateral-intrusion channel.  None == module default (disabled).
        lateral_closing=None,
        avoid_lateral_enable=None,
        avoid_lateral_min_speed=None,
        avoid_lateral_confirm=None,
        avoid_lateral_closing_min=None,
    ):

        now = time.monotonic()

        lane_request = (
            LaneChangeDirection.none
        )

        hazard = False

        lane_offset = 0.0

        # [AA_BRAKE_FALLBACK] 默认不介入纵向；只有唯一的 fallback 路径会写入。
        # 必须在函数开头初始化，否则早期 return 路径会 UnboundLocalError。
        brake_request = 0.0

        # --------------------------------------------------------------
        # Validate ego speed
        # --------------------------------------------------------------

        v_ego = _safe_float(v_ego)

        if v_ego is None:
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Enable / speed gate
        # --------------------------------------------------------------

        if (
            not bool(enabled)
            or
            v_ego < AUTO_AVOID_MIN_SPEED
        ):
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Manual driver input
        #
        # Do not start an automatic emergency maneuver from idle while
        # the driver is manually commanding a lane change.
        # --------------------------------------------------------------

        if (
            bool(manual_blinker)
            and
            self._mode == "idle"
        ):
            self.reset()

            return (
                lane_request,
                0.0,
                False,
                0.0,
            )

        # --------------------------------------------------------------
        # Compatibility mapping
        # --------------------------------------------------------------

        if mr76_left_available is None:
            mr76_left_available = bool(
                rear_left_available
            )

        if mr76_right_available is None:
            mr76_right_available = bool(
                rear_right_available
            )

        if left_lidar_available is None:
            left_lidar_available = (
                left_lidar_free is not None
            )

        if right_lidar_available is None:
            right_lidar_available = (
                right_lidar_free is not None
            )

        # --------------------------------------------------------------
        # Escape-lane policy defaults
        # --------------------------------------------------------------

        if avoid_stable_mode is None:
            avoid_stable_mode = AVOID_STABLE_MODE_DEFAULT

        if avoid_stable_ratio is None:
            avoid_stable_ratio = AVOID_STABLE_RATIO_DEFAULT

        if avoid_stable_window_sec is None:
            avoid_stable_window_sec = AVOID_STABLE_WINDOW_SEC_DEFAULT

        # [AA_BRAKE_FALLBACK]
        if avoid_brake_gain is None:
            avoid_brake_gain = AVOID_BRAKE_GAIN_DEFAULT

        if avoid_brake_max is None:
            avoid_brake_max = AVOID_BRAKE_MAX_DEFAULT

        if avoid_brake_ttc_ref is None:
            avoid_brake_ttc_ref = AVOID_BRAKE_TTC_REF_DEFAULT

        if bsm_debounce_frames is None:
            bsm_debounce_frames = AVOID_BSM_DEBOUNCE_DEFAULT

        if rear_sensing_present is None:
            rear_sensing_present = bool(AVOID_REAR_PRESENT_DEFAULT)

        if not hasattr(self, "_left_ok_hist") or self._left_ok_hist is None:
            self._left_ok_hist = []

        if not hasattr(self, "_right_ok_hist") or self._right_ok_hist is None:
            self._right_ok_hist = []

        # --------------------------------------------------------------
        # BSM blocker debounce
        #
        # A single-frame BSM blip must not reset the escape-lane timer.
        # 0 frames == honour the raw flag (historical behaviour).
        # --------------------------------------------------------------

        self._bsm_l_run = getattr(self, "_bsm_l_run", 0)
        self._bsm_r_run = getattr(self, "_bsm_r_run", 0)

        if bool(left_bsm_blocked):
            self._bsm_l_run += 1
        else:
            self._bsm_l_run = 0

        if bool(right_bsm_blocked):
            self._bsm_r_run += 1
        else:
            self._bsm_r_run = 0

        if int(bsm_debounce_frames) > 0:
            left_bsm_blocked = (
                self._bsm_l_run >= int(bsm_debounce_frames)
            )
            right_bsm_blocked = (
                self._bsm_r_run >= int(bsm_debounce_frames)
            )

        # --------------------------------------------------------------
        # Base lane safety
        # --------------------------------------------------------------

        left_base = bool(left_ok)
        right_base = bool(right_ok)

        # --------------------------------------------------------------
        # Escape lane safety
        # --------------------------------------------------------------

        left_safe = self._escape_lane_safe(
            base_lane_ok=left_base,

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            mr76_available=mr76_left_available,
            mr76_valid=mr76_left_valid,
            mr76_age=mr76_left_age,

            lidar_free=left_lidar_free,
            lidar_available=left_lidar_available,
            lidar_valid=left_lidar_valid,
            lidar_age=left_lidar_age,

            bsm_blocked=left_bsm_blocked,
            bsm_available=bsm_available,

            require_aux_sensors=require_aux_sensors,
            require_aux_sensors_mr76=require_aux_sensors_mr76,
            require_aux_sensors_lidar=require_aux_sensors_lidar,
            rear_sensing_present=rear_sensing_present,
        )

        right_safe = self._escape_lane_safe(
            base_lane_ok=right_base,

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            mr76_available=mr76_right_available,
            mr76_valid=mr76_right_valid,
            mr76_age=mr76_right_age,

            lidar_free=right_lidar_free,
            lidar_available=right_lidar_available,
            lidar_valid=right_lidar_valid,
            lidar_age=right_lidar_age,

            bsm_blocked=right_bsm_blocked,
            bsm_available=bsm_available,

            require_aux_sensors=require_aux_sensors,
            require_aux_sensors_mr76=require_aux_sensors_mr76,
            require_aux_sensors_lidar=require_aux_sensors_lidar,
            rear_sensing_present=rear_sensing_present,
        )

        # --------------------------------------------------------------
        # Stable safety
        # --------------------------------------------------------------

        (
            self._left_ok_since,
            left_stable,
        ) = self._stable_ok(
            left_safe,
            self._left_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
            history=self._left_ok_hist,
            mode=avoid_stable_mode,
            ratio=avoid_stable_ratio,
            window_sec=avoid_stable_window_sec,
        )

        (
            self._right_ok_since,
            right_stable,
        ) = self._stable_ok(
            right_safe,
            self._right_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
            history=self._right_ok_hist,
            mode=avoid_stable_mode,
            ratio=avoid_stable_ratio,
            window_sec=avoid_stable_window_sec,
        )

        # --------------------------------------------------------------
        # Risk-input hardening
        #
        # (a) low-pass the lead measurements so a one-frame radar spike cannot
        #     be classified as urgent;
        # (b) optionally require the URGENT level to persist.
        # --------------------------------------------------------------

        if avoid_lead_lpf_alpha is None:
            avoid_lead_lpf_alpha = AVOID_LEAD_LPF_ALPHA_DEFAULT

        if avoid_urgent_confirm is None:
            avoid_urgent_confirm = AVOID_URGENT_CONFIRM_DEFAULT

        alpha = _safe_float(avoid_lead_lpf_alpha)
        if alpha is None or alpha <= 0.0 or alpha > 1.0:
            # disabled -- keep raw values, but still track nothing
            pass
        else:
            d_raw = _safe_float(lead_dist)
            v_raw = _safe_float(lead_rel_speed)

            if d_raw is None or v_raw is None:
                self._lead_lpf_d = None
                self._lead_lpf_v = None
            else:
                if self._lead_lpf_d is None:
                    self._lead_lpf_d = d_raw
                    self._lead_lpf_v = v_raw
                else:
                    self._lead_lpf_d = (
                        (1.0 - alpha) * self._lead_lpf_d + alpha * d_raw
                    )
                    self._lead_lpf_v = (
                        (1.0 - alpha) * self._lead_lpf_v + alpha * v_raw
                    )

                lead_dist = self._lead_lpf_d
                lead_rel_speed = self._lead_lpf_v

        # --------------------------------------------------------------
        # Lateral intrusion (independent channel)
        #
        # A vehicle beside the car is invisible to leadOne, so it needs its own
        # path.  We require SUSTAINED blind-spot occupancy (not a flicker) at a
        # minimum speed; optionally also a lateral closing rate.
        # --------------------------------------------------------------

        if avoid_lateral_enable is None:
            avoid_lateral_enable = AVOID_LATERAL_ENABLE_DEFAULT

        if avoid_lateral_min_speed is None:
            avoid_lateral_min_speed = AVOID_LATERAL_MIN_SPEED_DEFAULT

        if avoid_lateral_confirm is None:
            avoid_lateral_confirm = AVOID_LATERAL_CONFIRM_DEFAULT

        if avoid_lateral_closing_min is None:
            avoid_lateral_closing_min = AVOID_LATERAL_CLOSING_MIN_DEFAULT

        lateral_active_now = False
        lateral_dir_now = 0

        if bool(avoid_lateral_enable):

            v_ok = (
                _safe_float(v_ego) is not None
                and _safe_float(v_ego) >= _safe_float(avoid_lateral_min_speed)
            )

            lateral_closing_v = _safe_float(lateral_closing)

            closing_ok = (
                _safe_float(avoid_lateral_closing_min) <= 0.0
                or (
                    lateral_closing_v is not None
                    and lateral_closing_v >= _safe_float(avoid_lateral_closing_min)
                )
            )

            bl = bool(left_bsm_blocked)
            br = bool(right_bsm_blocked)

            if bl:
                self._lateral_left_frames += 1
            else:
                self._lateral_left_frames = 0

            if br:
                self._lateral_right_frames += 1
            else:
                self._lateral_right_frames = 0

            need = max(1, int(avoid_lateral_confirm))

            if v_ok and closing_ok:
                if self._lateral_left_frames >= need:
                    lateral_active_now = True
                    lateral_dir_now = 1     # +1 == LEFT
                elif self._lateral_right_frames >= need:
                    lateral_active_now = True
                    lateral_dir_now = -1    # -1 == RIGHT
        else:
            self._lateral_left_frames = 0
            self._lateral_right_frames = 0

        self._lateral_active = lateral_active_now
        self._lateral_dir = lateral_dir_now

        # --------------------------------------------------------------
        # Risk
        # --------------------------------------------------------------

        risk = self._classify_risk(
            obstacle_in_path=bool(
                obstacle_in_path
            ),

            obstacle_dist=obstacle_dist,

            obstacle_rel_speed=obstacle_rel_speed,

            lead_dist=lead_dist,

            lead_rel_speed=lead_rel_speed,

            is_pedestrian=bool(
                is_pedestrian
            ),

            is_cone=bool(
                is_cone
            ),

            imminent_collision=bool(
                imminent_collision
            ),
        )

        self._risk_level = risk

        # --------------------------------------------------------------
        # Critical confirmation
        # --------------------------------------------------------------

        if risk == RISK_CRITICAL:
            self._critical_frames += 1
        else:
            self._critical_frames = 0

        critical_confirmed = (
            self._critical_frames
            >= CRITICAL_CONFIRM_FRAMES
        )

        # --------------------------------------------------------------
        # Urgent confirmation
        #
        # emergency_active needs CRITICAL_CONFIRM_FRAMES to confirm a
        # CRITICAL level, but URGENT used to fire with zero confirmation.
        # Make the URGENT requirement explicit and tunable (default 0 ==
        # historical immediate behaviour).
        # --------------------------------------------------------------

        if risk == RISK_URGENT:
            self._urgent_frames += 1
        else:
            self._urgent_frames = 0

        urgent_confirmed = (
            int(avoid_urgent_confirm) <= 0
            or
            self._urgent_frames >= int(avoid_urgent_confirm)
        )

        emergency_active = (
            (
                risk == RISK_URGENT
                and
                urgent_confirmed
            )
            or
            (
                risk == RISK_CRITICAL
                and
                critical_confirmed
            )
            or
            lateral_active_now
        )

        # --------------------------------------------------------------
        # Debug reason
        # --------------------------------------------------------------

        if risk == RISK_NORMAL:
            self._last_reason = "normal"

        elif risk == RISK_WARNING:
            self._last_reason = "warning_only"

        elif risk == RISK_URGENT:
            self._last_reason = "urgent"

        else:
            self._last_reason = (
                "critical_confirmed"
                if critical_confirmed
                else "critical_confirming"
            )

        # ==============================================================
        # STATE MACHINE
        # ==============================================================

        # --------------------------------------------------------------
        # IDLE
        # --------------------------------------------------------------

        if self._mode == "idle":

            if (
                emergency_active
                and
                now >= self._cooldown_until
            ):

                direction = (
                    self._pick_out_direction(
                        left_stable,
                        right_stable,
                        is_rhd,
                        prefer_dir,
                    )
                )

                self._out_dir = direction

                if (
                    direction
                    != LaneChangeDirection.none
                ):

                    self._mode = "avoiding"

                    self._active_since = now

                    self._emergency_since = now

                    self._observed_lane_change = False

                    lane_request = direction

                    hazard = True

                    self._last_reason = (
                        "emergency_escape_request"
                    )

                else:

                    # --------------------------------------------------
                    # [AA_BRAKE_FALLBACK] 第二层：优先刹车
                    #
                    # 没有任何逃逸车道可用（两侧都不安全），但风险已达
                    # 到 emergency_active（URGENT/CRITICAL）。此时只做
                    # 横向避让已经不可能，必须走纵向兜底：
                    #
                    #   brake_request = 与风险等级/TTC 相关的减速度需求
                    #
                    # 设计约束（安全边界）：
                    #   - 只在 emergency_active 为真（即确有紧急风险）时才非零
                    #   - 幅度受 brake_max_ms2 硬限幅，且随 TTC 收敛
                    #   - 任何异常/缺数据都回落到 0.0（与旧行为一致）
                    #   - 该值由接线侧决定如何转成执行器命令
                    # --------------------------------------------------
                    if bool(emergency_active):
                        brake_request = self._calc_emergency_brake(
                            obstacle_dist=obstacle_dist,
                            obstacle_rel_speed=obstacle_rel_speed,
                            lead_dist=lead_dist,
                            lead_rel_speed=lead_rel_speed,
                            v_ego=v_ego,
                            brake_gain=avoid_brake_gain,
                            brake_max=avoid_brake_max,
                            brake_ttc_ref=avoid_brake_ttc_ref,
                        )
                        hazard = True

                    self._last_reason = (
                        "emergency_no_safe_lane"
                    )

        # --------------------------------------------------------------
        # AVOIDING
        # --------------------------------------------------------------

        elif self._mode == "avoiding":

            hazard = True

            # ----------------------------------------------------------
            # BEFORE actual LC start:
            #
            # The locked direction may be cancelled if the destination
            # lane is no longer safe.
            #
            # We DO NOT automatically reverse direction after starting.
            # ----------------------------------------------------------

            if not self._observed_lane_change:

                if (
                    lc_state
                    == LaneChangeState.laneChangeStarting
                ):
                    self._observed_lane_change = True

                else:

                    direction_safe = (
                        (
                            self._out_dir
                            == LaneChangeDirection.left
                            and
                            left_stable
                        )
                        or
                        (
                            self._out_dir
                            == LaneChangeDirection.right
                            and
                            right_stable
                        )
                    )

                    if direction_safe:
                        lane_request = self._out_dir

                    else:
                        # Do not blindly switch sides.
                        #
                        # The original destination is unsafe.
                        # Re-evaluate both sides only before the actual
                        # maneuver has started.
                        opposite = self._opposite(
                            self._out_dir
                        )

                        opposite_safe = (
                            (
                                opposite
                                == LaneChangeDirection.left
                                and
                                left_stable
                            )
                            or
                            (
                                opposite
                                == LaneChangeDirection.right
                                and
                                right_stable
                            )
                        )

                        if opposite_safe:
                            self._out_dir = opposite

                            lane_request = opposite

                        else:
                            lane_request = (
                                LaneChangeDirection.none
                            )

                            self._last_reason = (
                                "escape_lane_lost"
                            )

                            # --------------------------------------
                            # [AA_BRAKE_FALLBACK] 第二层：优先刹车
                            #
                            # 本路径 = 变道尚未物理启动（_observed_lc=False）
                            #         且当前逃逸车道已失效、对向车道也不安全
                            #         → 既不能转向、也无逃逸车道
                            # 属于 emerg_active 下的"安全无解"状态，按指令
                            # 走纵向兜底。
                            #
                            # 注意：本路径天然只在 _observed_lane_change==False
                            #       时到达（见外层 if），故不会在"变道已启动"
                            #       后急刹，避免侧向失稳。
                            # --------------------------------------
                            if bool(emergency_active):
                                brake_request = self._calc_emergency_brake(
                                    obstacle_dist=obstacle_dist,
                                    obstacle_rel_speed=obstacle_rel_speed,
                                    lead_dist=lead_dist,
                                    lead_rel_speed=lead_rel_speed,
                                    v_ego=v_ego,
                                    brake_gain=avoid_brake_gain,
                                    brake_max=avoid_brake_max,
                                    brake_ttc_ref=avoid_brake_ttc_ref,
                                )
                                hazard = True

                # ------------------------------------------------------
                # [AA_ESC_ABORT] 风险消失脱困（治"卡死在 avoiding"）。
                #
                # 缺陷：本分支（_observed_lane_change == False，即变道尚未
                #       物理启动）原本没有任何退出路径。一次误判的 URGENT
                #       就能把状态机推进 avoiding 并永久驻留 —— 只要方向
                #       持续可用就一直发 lane_request，实测达 825 帧。
                #
                # 修复：变道尚未启动 + 风险已回落为非紧急 + 持续满
                #       AVOID_ESC_ABORT_SEC -> 主动放弃逃逸，转 cooldown。
                #
                # 安全：只在"还没真的开始变道"时生效，不会打断进行中的
                #       变道；且要求"持续"非紧急（真实紧急时 emergency_active
                #       会持续为真，计时器会被复位），不会误杀真实逃逸。
                # ------------------------------------------------------
                if not emergency_active:

                    if self._esc_abort_since is None:
                        self._esc_abort_since = now

                    elif (
                        (now - self._esc_abort_since)
                        >= AVOID_ESC_ABORT_SEC
                    ):
                        self._mode = "cooldown"

                        self._cooldown_until = (
                            now + AVOID_COOLDOWN_SEC
                        )

                        self._esc_abort_since = None

                        self._out_dir = (
                            LaneChangeDirection.none
                        )

                        lane_request = (
                            LaneChangeDirection.none
                        )

                        self._last_reason = (
                            "escape_aborted_no_risk"
                        )

                else:
                    # 风险重新升级 -> 复位计时器，保持逃逸请求。
                    self._esc_abort_since = None

            # ----------------------------------------------------------
            # AFTER actual LC start:
            #
            # Direction is LOCKED.
            # ----------------------------------------------------------

            else:

                lane_request = self._out_dir

            # ----------------------------------------------------------
            # Detect actual completion.
            # ----------------------------------------------------------

            if (
                self._observed_lane_change
                and
                lc_state == LaneChangeState.off
            ):

                self._observed_lane_change = False

                self._mode = "waiting_clear"

                self._clear_since = now

                # [AA_ESC_ABORT] 离开 avoiding，脱困计时器清零。
                self._esc_abort_since = None

                self._last_reason = (
                    "emergency_lc_completed"
                )

        # --------------------------------------------------------------
        # WAITING CLEAR
        # --------------------------------------------------------------

        elif self._mode == "waiting_clear":

            hazard = False

            # ----------------------------------------------------------
            # Emergency returned immediately.
            # ----------------------------------------------------------

            if emergency_active:

                self._mode = "avoiding"

                self._clear_since = None

                hazard = True

                # IMPORTANT:
                #
                # Reuse locked direction only if it is still safe.
                #
                # Otherwise select a new safe escape direction.
                #

                direction_safe = (
                    (
                        self._out_dir
                        == LaneChangeDirection.left
                        and
                        left_stable
                    )
                    or
                    (
                        self._out_dir
                        == LaneChangeDirection.right
                        and
                        right_stable
                    )
                )

                if not direction_safe:

                    self._out_dir = (
                        self._pick_out_direction(
                            left_stable,
                            right_stable,
                            is_rhd,
                            prefer_dir,
                        )
                    )

                if (
                    self._out_dir
                    != LaneChangeDirection.none
                ):
                    lane_request = self._out_dir

                self._active_since = now

            else:

                if self._clear_since is None:
                    self._clear_since = now

                clear_time = (
                    now - self._clear_since
                )

                active_time_ok = (
                    self._active_since is None
                    or
                    (
                        now - self._active_since
                    ) >= MIN_ACTIVE_SEC
                )

                if (
                    clear_time >= 1.0
                    and
                    active_time_ok
                ):

                    self._mode = "cooldown"

                    self._cooldown_until = (
                        now + AVOID_COOLDOWN_SEC
                    )

                    self._clear_since = None

                    # [AA_ESC_ABORT] 离开 avoiding，脱困计时器清零。
                    self._esc_abort_since = None

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._last_reason = (
                        "emergency_cleared"
                    )

        # --------------------------------------------------------------
        # COOLDOWN
        # --------------------------------------------------------------

        elif self._mode == "cooldown":

            if (
                emergency_active
                and
                now >= self._cooldown_until
            ):

                self._mode = "avoiding"

                self._active_since = now

                self._emergency_since = now

                self._out_dir = (
                    self._pick_out_direction(
                        left_stable,
                        right_stable,
                        is_rhd,
                        prefer_dir,
                    )
                )

                if (
                    self._out_dir
                    != LaneChangeDirection.none
                ):

                    lane_request = self._out_dir

                    hazard = True

            elif (
                now >= self._cooldown_until
            ):

                self._mode = "idle"

                # [AA_ESC_ABORT] 回到 idle，脱困计时器清零。
                self._esc_abort_since = None

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._last_reason = (
                    "idle_after_cooldown"
                )

        else:

            self.reset()

        # --------------------------------------------------------------
        # Diagnostic lane offset
        #
        # IMPORTANT:
        #
        # This is NOT a steering command.
        # Caller must NOT directly convert this value into actuator CAN.
        # --------------------------------------------------------------

        if (
            emergency_active
            and
            self._mode == "avoiding"
        ):

            if (
                self._out_dir
                == LaneChangeDirection.left
            ):
                lane_offset = (
                    -EMERGENCY_OFFSET
                )

            elif (
                self._out_dir
                == LaneChangeDirection.right
            ):
                lane_offset = (
                    EMERGENCY_OFFSET
                )

        # --------------------------------------------------------------
        # --------------------------------------------------------------
        # ABSOLUTE LONGITUDINAL SAFETY BOUNDARY
        #
        # [AA_BRAKE_FALLBACK] 旧版此处无条件 brake_request = 0.0（纵向完全
        # 不介入）。现改为：默认仍为 0.0，只有在「无安全逃逸车道 + 风险
        # 已达 emergency_active」这一唯一路径下，才由 _calc_emergency_brake
        # 写入有限幅度值（受 avoid_brake_max 硬限幅，默认 0.0 == 关闭）。
        # 其余所有路径的 brake_request 保持 0.0，行为与旧版一致。
        # --------------------------------------------------------------

        if not math.isfinite(float(brake_request)):
            brake_request = 0.0

        brake_request = float(max(0.0, min(float(avoid_brake_max), float(brake_request))))

        self._last_lc_state = lc_state

        return (
            lane_request,
            brake_request,
            hazard,
            lane_offset,
        )

    # ==================================================================
    # Debug
    # ==================================================================

    def get_state(self):
        return {
            "mode": self._mode,
            "out_dir": self._out_dir,
            "cooldown_until": self._cooldown_until,
            "active_since": self._active_since,
            "emergency_since": self._emergency_since,
            "risk_level": self._risk_level,
            "critical_frames": self._critical_frames,
            "left_ok_since": self._left_ok_since,
            "right_ok_since": self._right_ok_since,
            "observed_lane_change": self._observed_lane_change,
            "last_reason": self._last_reason,
            "lateral_active": self._lateral_active,
            "lateral_dir": self._lateral_dir,
        }