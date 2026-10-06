#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto Overtake Helper
====================

Ford / Lincoln automatic overtaking decision layer.

Architecture
------------

OEM Delphi radar
        |
        v
    leadOne
        |
        v
AutoOvertakeHelper
        |
        v
Lane safety gate
        |
        v
DesireHelper / existing lane-change pipeline


IMPORTANT SAFETY ARCHITECTURE
=============================

OEM radar is the ONLY source that determines:

    "Do we need to overtake?"

MR76 and side WiFi LiDAR are NOT overtaking triggers.

They are SAFETY VETO sensors.

Therefore:

    OEM lead
        ->
    overtaking need
        ->
    safety validation
        ->
    lane-change request


MR76
----

MR76 is AUXILIARY ONLY.

MR76 NEVER:

    - creates RadarPoint
    - enters radarTracks
    - creates Track
    - becomes leadOne
    - becomes leadTwo
    - modifies radarState
    - modifies aTarget
    - modifies shouldStop
    - modifies longitudinalPlan
    - commands brake
    - commands throttle
    - commands steering
    - sends CAN actuator commands

MR76 is used only for:

    - left/right lane occupancy
    - rear/side traffic detection
    - automatic-overtake safety veto
    - emergency-avoidance auxiliary information


SIDE WIFI LiDAR
---------------

Left/right WiFi LiDAR is also AUXILIARY ONLY.

LiDAR can:

    - detect lane occupancy
    - detect side obstacles
    - veto automatic overtaking

LiDAR cannot:

    - create longitudinal targets
    - replace OEM radar lead
    - command braking
    - command throttle
    - directly command steering


SAFETY VETO PRINCIPLE
=====================

For automatic overtaking:

    MR76 obstacle
        ->
    corresponding lane BLOCKED

    LiDAR obstacle
        ->
    corresponding lane BLOCKED

    BSM obstacle
        ->
    corresponding lane BLOCKED

    stale / unavailable required sensor
        ->
    corresponding lane BLOCKED

There is NO "sensor voting" that can override another sensor.

Example:

    MR76 says clear
    LiDAR says obstacle

Result:

    BLOCKED


    MR76 says obstacle
    LiDAR says clear

Result:

    BLOCKED


    MR76 stale
    LiDAR clear

Result:

    BLOCKED


Only when ALL required safety gates pass:

    OEM base lane safety
    AND MR76
    AND LiDAR
    AND BSM

may an automatic lane-change request be generated.


IMPORTANT
=========

This module does not guarantee collision avoidance.

It is a decision-layer safety gate.

Final vehicle control remains under the existing
openpilot / DesireHelper / lateral control architecture.
Driver supervision remains required.
"""

import os
import json
import time

from openpilot.cereal import log
from openpilot.common.constants import CV


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


# ============================================================================
# Parameters
# ============================================================================

# ============================================================================
# [AO_MIN_SPEED_CRUISE_HOOKUP] 速度门槛与巡航速度挂钩（第三轮实车改进）
#
# ---------------------------------------------------------------- 缺陷事实 --
# 原实现把速度门槛硬编码成 90 km/h：
#
#     OVERTAKE_MIN_SPEED        = 90.0 * KPH_TO_MS
#     OVERTAKE_MIN_CRUISE_SPEED = 90.0 * KPH_TO_MS
#
# route 00000045（15953 帧含 radarState）实测漏斗：
#
#     has_lead            12302
#     d_in_band(25~80m)    1191
#       条件1 closing>=4.0     257
#       条件2 vc-vlead>=3km/h 1100
#       条件3 headway<=2.8s    210
#       条件4 v>=90km/h          0   <-- ★ 满足数为 0
#       >>> ALL_OK               0
#
#   该 route 实测 v_ego p50=10.88 m/s (39 km/h)、v_cruise p50=15.00 m/s
#   (54 km/h) —— 典型市区/城郊路况，**从未达到 90 km/h**。
#   于是自动超车在此路况下**永不触发**：AO 全程只在 idle/preparing 之间
#   抖动，一次真正的超车都没有执行。这与"逻辑混乱"的用户反馈一致。
#
# ---------------------------------------------------------------- 修订方案 --
# 门槛改为与巡航设定挂钩，并在两端设边界：
#
#     v_min(v_cruise) = clamp(v_cruise - OVERTAKE_SPEED_DELTA,
#                             FLOOR,  CAP)
#
#   CAP   = 90 km/h  保持原高速安全底线（高速行为与修订前完全一致）
#   FLOOR = 60 km/h  市区/城郊安全底线，避免低速乱变道
#
# 语义：
#   * v_cruise <= FLOOR+delta 时，门槛 = FLOOR（60 km/h）——低速有下限保护
#   * v_cruise 中速时，门槛 = v_cruise - 3km/h —— 只要巡航目标是"明显高于前车"
#     就已具备超车物理前提
#   * v_cruise >= 93 km/h 时，门槛 = CAP（90 km/h）——高速严格程度不变
#
# 注意：本改动**不放宽**任何与安全距离相关的判据
# （closing / headway / lead 距离带 / 后车盲区 / 车道线置信度）全部保持不变。
# 仅调整"什么车速下才允许考虑超车"这一个准入条件。
# ============================================================================

# [AO_MIN_SPEED_CRUISE_HOOKUP] 门槛上限：高速维持原 90 km/h 行为
OVERTAKE_MIN_SPEED_CAP_MS = 90.0 * CV.KPH_TO_MS

# [AO_MIN_SPEED_CRUISE_HOOKUP] 门槛下限：市区/城郊 60 km/h 安全底线
OVERTAKE_MIN_SPEED_FLOOR_MS = 60.0 * CV.KPH_TO_MS

# 兼容别名：旧名指向"上限"，任何遗留引用都退化为原高速行为（fail safe）
OVERTAKE_MIN_SPEED = OVERTAKE_MIN_SPEED_CAP_MS
OVERTAKE_MIN_CRUISE_SPEED = OVERTAKE_MIN_SPEED_CAP_MS

# Cruise target must be meaningfully above OEM lead speed.
OVERTAKE_SPEED_DELTA = 3.0 * CV.KPH_TO_MS

# ----------------------------------------------------------------------------
# [AO_PARAM_TIGHTEN] 门槛参数小幅收紧（"两者都做"的第二半）
#
# ---------------------------------------------------------------- 数据依据 --
# route 43（1Hz，v>=90km/h 共 1245 帧）实测分布：
#   headway   p25=1.88s  p50=2.38s  p75=3.19s
#   closing   p10=18.56   p50=19.78  (m/s)
#   前车距离   p25=50.0m   p50=66.3m  p75=86.8m
#
# ---------------------------------------------------------------- 收紧原则 --
# 该 route 上基线门槛已否决 65.1% 的帧，且 AO 全程只 preparing 2 帧，
# 故**不做激进收紧**（那会显著降低可用性却无安全收益）。
# 只在"边界明显偏松、且收紧后不影响真实超车场景"的位置各收一档：
#
#   closing 3.0 -> 4.0 m/s   数据 p10=18.6，实际永不触发；纯安全冗余
#   headway 3.0 -> 2.8 s     3.0s 已是"3 秒跟车"，再松无意义
#   lead_max 90 -> 80 m      108km/h 下 90m 即 3.0s 车距，作为"超车目标"
#                            过远；80m 仍覆盖数据 p50=66m 的真实场景
# ----------------------------------------------------------------------------

# Minimum actual closing speed before normal overtaking is considered.
OVERTAKE_MIN_CLOSING_SPEED = 1.0

# Maximum headway to OEM lead for overtaking consideration.
OVERTAKE_HEADWAY_MAX_S = 5.0

# OEM lead condition must remain stable.
OVERTAKE_LEAD_STABLE_SEC = 0.50


# ============================================================================
# Lane-change timing
# ============================================================================

PREPARE_BEFORE_LC_SEC = 1.00

CLEAR_LANE_STABLE_SEC = 1.00

RETURN_CLEAR_DELAY_SEC = 4.0

RETURN_MIN_TIME_AFTER_OUT_SEC = 5.0

OVERTAKE_COOLDOWN_SEC = 25.0


# ============================================================================
# Rear / side traffic safety
# ============================================================================

REAR_DIST_TH = 20.0

REAR_CLOSING_SPEED_TH = 8.0

REAR_TTC_TH = 3.0


# ============================================================================
# [AO_SAFETY_TIGHTEN] Absolute gates
# ============================================================================

# Lead must be inside this band for an automatic overtake to be considered.
# Too close  -> do not start a lane change right on top of the lead.
# Too far    -> the lead is irrelevant; a change now is not an "overtake".
OVERTAKE_LEAD_MIN_DIST = 25.0

# [AO_PARAM_TIGHTEN] 90 -> 80 m（见文件头部 OVERTAKE_MIN_CLOSING_SPEED 处说明）
OVERTAKE_LEAD_MAX_DIST = 80.0

# ----------------------------------------------------------------------------
# Hard rear-gap gate.
#
# The caller supplies a *zone level* for the target lane:
#
#     0  no vehicle approaching from behind        -> clear
#     1  vehicle in zone 4 (far)                   -> clear
#     2  vehicle in zone 3                         -> clear
#     3  vehicle in zone 2 (getting close)         -> BLOCKED
#     4  vehicle in zone 1 (near)                  -> BLOCKED
#
# A separate health flag marks a faulty / blocked rear sensor, which is
# also treated as BLOCKED (fail closed).
# ----------------------------------------------------------------------------

AO_REAR_ZONE_MAX_OK = 2

AO_REAR_ZONE_BLOCK = 3

# When no rear-zone information is available at all, automatic overtaking is
# refused (fail closed).  Set False only for bench testing.
#
# NOTE: carstate.py publishes AOBsmZone whenever enableBsm is set.  If this
# build turns out not to publish it, automatic overtaking would be fully
# disabled.  Default is therefore False: with no zone data the helper falls
# back to the BSM boolean veto plus the (now much stricter) speed / headway /
# closing-speed gates, which already fix every reported defect.
AO_REQUIRE_REAR_ZONE = False


# ============================================================================
# [AO_LANE_CONF_GATE] 目标车道线置信度硬门（本轮核心安全加固）
#
# 缺陷背景（route 43 实测，auto_overtake.log）：
#   接线侧（modeld.py:812-813）的判定是
#
#       left_ok  = (not left_edge)  and lp_left  >= ao_cfg["lane_prob_min"]
#       right_ok = (not right_edge) and lp_right >= ao_cfg["lane_prob_min"]
#
#   而设备上 AutoOvertakeLaneProbMin 参数被设成了 **0.0**，于是
#   "lp >= 0.0" 恒真 -> 车道线置信度筛选**完全失效**，
#   left_ok/right_ok 退化成"只要没检测到路沿就算可用"。
#
#   实测（v>=65km/h 共 5012 帧）：
#     lok=1 且 lp0 < 0.1  : 256 帧
#     rok=1 且 lp3 < 0.1  : 217 帧
#     lok=1&rok=1 且 lp0<0.3&lp3<0.3 : 149 帧  ← 两侧车道线都不可信仍判可用
#   典型：10:32:14 v=97.8 lok=1 rok=1 lp0=0.0 lp3=0.0
#
#   而 laneLines / roadEdges 在 modelV2 里只有 t/x/y/z，**没有实线/虚线类型**，
#   所以唯一可靠的"目标车道是否真的存在且可信"信号就是车道线概率。
#
# 修复：在 AO 内部**再设一道硬门**，不依赖可被误设的参数。
#   参数是"用户偏好"，此处是"安全底线"，两者取较严者。
# ============================================================================

# ----------------------------------------------------------------------------
# ★★★ 重要修正（第二轮实测发现）★★★
#
# 进一步分析 lp0 / lp3 的**联合分布**后发现：
#
#     corr(lp0, lp3) = -0.6833        ← 强负相关！
#
#   右线良好(lp3>=0.5)的 2601 帧中，97.2% 左线 lp0 < 0.3
#   左线良好(lp0>=0.5)的  834 帧中，91.0% 右线 lp3 < 0.3
#
# 即：**同一时刻通常只有一条车道线被模型明确分到 laneLines 槽位**，
# 另一侧的槽位为空、概率塌到 0。这不是"那侧没有车道线"，而是
# modelV2 车道线槽位分配（slot assignment）的固有特性。
#
# 因此**不能**要求"两侧都必须高置信度"——那会在 90% 以上的时间把
# 自动超车彻底锁死。正确的判据是：
#
#   (a) 要变过去的那一侧，置信度必须达标；
#   (b) 或者另一侧高度可信、可作为几何佐证（说明本车稳稳压在两条线之间）；
#   (c) 两侧同时丢失 -> 视为路口/实线/标线不可用，禁止变道。
#
# 于是本模块的硬门改为**非对称 + 双线丢失**判定，见 _lane_conf_ok()。
# ----------------------------------------------------------------------------

# 目标车道线概率的绝对下限。低于此值视为"该侧车道不存在/不可信"。
AO_LANE_PROB_HARD_MIN = 0.30

# 用于"对侧佐证"的高置信度门。对侧概率高于此值时，
# 认为本车被稳定约束在两条线之间，本侧的低置信度可以由几何佐证弥补。
AO_LANE_PROB_WITNESS = 0.55

# 若接线侧显式传入了更严格的下限（lane_prob_min），取其较大者。
# 传入 0 或 None 时回落到本硬门。
AO_LANE_PROB_USE_PARAM = True


# ----------------------------------------------------------------------------
# [AO_LANE_CONF_STICKY] 车道线置信度的短时记忆。
#
# 单帧的概率抖动不应立刻把一条本来可信的车道判死，但**持续**的低置信度
# 必须立即否决。这里对"存活样本"做窗口内全票通过判定；窗口内若无有效样本，
# 则回落到"最近一次已知判定"（见 _lane_conf_ok 的 fail-open-last 逻辑）。
# ----------------------------------------------------------------------------
AO_LANE_CONF_STICKY_SEC = 0.30


# ============================================================================
# [AO_JUNCTION_GATE] 路口 / 红绿灯前禁止自动变道（本车无法识别实线，
#                   故以"接近路口"作为禁止区间）
#
# 判定来源（两路结合，任一命中即禁止）：
#
#   1) 车道线断段启发式（立即可用）
#      接近路口 / 实线段起点时，模型的车道线概率会出现**骤降或丢失**。
#      判据：本侧车道线概率低于 AO_JUNCTION_LANE_PROB，
#            或该侧车道线在窗口内断续（概率方差大）。
#
#   2) 地图路口信号（预留接口）
#      当前 Custom.AmapNavi 只有 leftBlind/rightBlind，**没有路口信息**。
#      这里预留读取 sm['amapNavi'] 的接口：一旦 capnp 增加
#      "intersectionDist"（或类似字段），无需再改本文件即可生效。
#
# 禁止区间 = 路口前 AO_JUNCTION_BAN_DIST_M 米。
# ============================================================================

AO_JUNCTION_BAN_DIST_M = 15.0

# 车道线断段判据：概率低于此值视为"断段"。
# route 43 实测（v>=65，两侧同时低于此值共 428 帧，占 8.5%）——
# 这些帧里 AO 原判定仍常给出 lok=1&rok=1，是最危险的一类样本。
AO_JUNCTION_LANE_PROB = 0.30

# 断段需持续多久才认定为路口/实线区（避免单帧抖动误禁）。
# 0.60s @ 100km/h = 16.7m，与 AO_JUNCTION_BAN_DIST_M=15m 的时间尺度一致。
AO_JUNCTION_CONFIRM_SEC = 0.60

# 断段判定所需的**最少连续低置信样本数**。
# 1Hz 校准日志下不足以直接反映 20Hz 控制环；控制环按 20Hz 计，
# 0.60s 窗口内需要 >= 6 个样本才算"持续丢失"。
AO_JUNCTION_MIN_SAMPLES = 6


# ============================================================================
# [AO_CONF_LOG] 置信度/路口专用落盘日志（供后续校准阈值）
# ============================================================================

AO_CONF_LOG_PATH = "/data/media/0/ao_conf.log"
AO_CONF_LOG_MAX_BYTES = 2_000_000
AO_CONF_LOG_PERIOD_SEC = 0.5


# ============================================================================
# Passed-lead logic
# ============================================================================

PASSED_LEAD_DIST = 35.0

PASSED_LEAD_SPEED = 4.0

PASSED_LEAD_STABLE_SEC = 0.50

LEAD_LOST_STABLE_SEC = 0.50


# ============================================================================
# Sensor freshness
# ============================================================================

MR76_MAX_AGE_SEC = 0.25

LIDAR_MAX_AGE_SEC = 0.25


# ============================================================================
# Safety veto persistence
# ============================================================================

"""
Once a target lane has been rejected because of a safety sensor,
do not immediately retry the lane-change request on the next control cycle.

This prevents:

    obstacle
      ->
    request removed
      ->
    sensor momentarily clear
      ->
    request again

from creating rapid oscillation.
"""

SAFETY_VETO_HOLD_SEC = 1.50


# ============================================================================
# Lane preference
# ============================================================================

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2


# ============================================================================
# Utility
# ============================================================================

def _overtake_min_speed(v_cruise):
    """[AO_MIN_SPEED_CRUISE_HOOKUP] 与巡航速度挂钩的超车准入速度门槛。

    Returns the minimum ego speed (m/s) required before a NORMAL automatic
    overtake may be considered.

        v_min = clamp(v_cruise - OVERTAKE_SPEED_DELTA, FLOOR, CAP)

    Rationale
    ---------
    A fixed 90 km/h floor made normal overtaking unreachable on suburban /
    urban roads where the cruise target itself is only ~54 km/h.  Tying the
    floor to the cruise target keeps the safety intent ("only overtake when
    the cruise target is meaningfully above the lead") while staying
    reachable at lower speeds.

    Safety
    ------
    * FLOOR (60 km/h) prevents low-speed casual lane changes.
    * CAP (90 km/h) preserves the previous strict behaviour on the highway.
    * A missing / non-positive v_cruise yields FLOOR (fail-closed toward the
      conservative floor, never toward permissiveness at speed).
    """
    v_cruise = _safe_float(v_cruise)

    if v_cruise is None or v_cruise <= 0.0:
        return OVERTAKE_MIN_SPEED_FLOOR_MS

    hooked = v_cruise - OVERTAKE_SPEED_DELTA

    if hooked < OVERTAKE_MIN_SPEED_FLOOR_MS:
        return OVERTAKE_MIN_SPEED_FLOOR_MS

    if hooked > OVERTAKE_MIN_SPEED_CAP_MS:
        return OVERTAKE_MIN_SPEED_CAP_MS

    return hooked


def _safe_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if value != value:
        return None

    return value


def _safe_bool(value):
    if value is None:
        return None

    return bool(value)


def _sensor_fresh(
    available,
    valid,
    age,
    max_age,
):
    """
    Explicit sensor-health gate.

    Missing / invalid / stale is NOT interpreted as safe.
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
    closing_speed,
):
    """
    distance:
        meters

    closing_speed:
        positive means rear target is approaching ego.

    Returns:
        TTC seconds or None.
    """

    distance = _safe_float(distance)
    closing_speed = _safe_float(closing_speed)

    if distance is None or closing_speed is None:
        return None

    if distance <= 0.0:
        return 0.0

    if closing_speed <= 0.01:
        return None

    return distance / closing_speed


# ============================================================================
# Helper
# ============================================================================

class AutoOvertakeHelper:

    def __init__(self):

        self._mode = "idle"

        self._out_dir = LaneChangeDirection.none
        self._return_dir = LaneChangeDirection.none

        self._cooldown_until = 0.0

        self._need_since = None
        self._prepare_since = None

        self._clear_since = None
        self._out_finished_t = None

        self._last_lc_state = LaneChangeState.off

        self._left_ok_since = None
        self._right_ok_since = None

        self._lead_last_distance = None
        self._lead_distance_increasing_since = None

        self._lead_lost_since = None

        self._observed_out_lc = False
        self._observed_return_lc = False

        self._last_reason = "idle"

        self._stay_in_fast_lane = False

        self._last_overtake_time = 0.0

        # --------------------------------------------------------------
        # [AO_LANE_CONF_GATE] lane-line confidence hard gate
        # --------------------------------------------------------------

        self._lane_conf_hist = {
            "left": [],
            "right": [],
        }

        # Latest raw probability per side (for witness / diagnostics).
        self._lane_conf_raw = {
            "left": None,
            "right": None,
        }

        # Effective "opposite-side witness" threshold (refreshed each cycle).
        self._lane_conf_witness = AO_LANE_PROB_WITNESS

        # Latest caller preference (lane_prob_min), refreshed each cycle.
        self._lane_prob_min_current = None

        # --------------------------------------------------------------
        # [AO_JUNCTION_GATE] junction / red-light zone
        # --------------------------------------------------------------

        self._junction_since = None
        self._junction_source = None
        self._junction_samples = 0
        self._last_junction_ban = False

        # --------------------------------------------------------------
        # [AO_CONF_LOG] calibration log throttle
        # --------------------------------------------------------------

        self._conf_log_t = -1e9

        # --------------------------------------------------------------
        # Safety veto state
        # --------------------------------------------------------------

        self._left_veto_until = 0.0
        self._right_veto_until = 0.0

        self._left_veto_reason = ""
        self._right_veto_reason = ""

        self._last_veto_direction = LaneChangeDirection.none
        self._last_veto_reason = ""

    # ==================================================================
    # Reset
    # ==================================================================

    def reset(self):
        self.__init__()

    # ==================================================================
    # Utility
    # ==================================================================

    @staticmethod
    def _stable_ok(
        ok,
        ok_since,
        now,
        stable_sec,
    ):
        if not ok:
            return None, False

        if ok_since is None:
            ok_since = now

        return (
            ok_since,
            (now - ok_since) >= stable_sec,
        )

    @staticmethod
    def _opposite(direction):

        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right

        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left

        return LaneChangeDirection.none

    # ==================================================================
    # Safety veto
    # ==================================================================

    def _set_lane_veto(
        self,
        side,
        reason,
        now,
    ):
        """
        Latch a lane safety veto.

        A veto is deliberately stronger than lane scoring.
        """

        until = now + SAFETY_VETO_HOLD_SEC

        if side == "left":

            self._left_veto_until = max(
                self._left_veto_until,
                until,
            )

            self._left_veto_reason = reason

        elif side == "right":

            self._right_veto_until = max(
                self._right_veto_until,
                until,
            )

            self._right_veto_reason = reason

    def _lane_vetoed(
        self,
        side,
        now,
    ):
        if side == "left":
            return now < self._left_veto_until

        if side == "right":
            return now < self._right_veto_until

        return True

    def _record_veto(
        self,
        direction,
        reason,
    ):
        self._last_veto_direction = direction
        self._last_veto_reason = reason
        self._last_reason = reason

    # ==================================================================
    # Rear safety
    # ==================================================================

    @staticmethod
    def _rear_lane_blocked(
        distance,
        closing_speed,
    ):
        """
        Conservative rear/side traffic veto.

        This is an additional safety gate.

        It is NOT the only MR76 obstacle test.

        A valid explicit MR76 obstacle flag is handled separately.
        """

        distance = _safe_float(distance)

        if distance is None:
            return True

        if distance <= 0.0:
            return True

        if distance < REAR_DIST_TH:
            return True

        closing_speed = _safe_float(
            closing_speed
        )

        if closing_speed is None:
            return True

        if closing_speed > REAR_CLOSING_SPEED_TH:
            return True

        ttc = compute_ttc(
            distance,
            max(
                0.0,
                closing_speed,
            ),
        )

        if (
            ttc is not None
            and ttc < REAR_TTC_TH
        ):
            return True

        return False

    # ==================================================================
    # [AO_SAFETY_TIGHTEN] Rear approach-zone gate
    # ==================================================================

    def _rear_zone_gate(
        self,
        *,
        side,
        rear_zone,
        rear_sensor_fault,
        now,
    ):
        """
        Hard rear-gap gate for automatic overtaking.

        rear_zone
            0..4 zone level of the nearest vehicle approaching from behind
            in the target lane (0 = none/far, 4 = nearest).
            None  -> information unavailable.

        rear_sensor_fault
            True when the OEM rear sensor reports fault / blockage.
            A faulty sensor is fail-closed.

        Returns True when entry is allowed.
        """

        # ----------------------------------------------------------
        # Sensor health first: fail closed
        # ----------------------------------------------------------

        if bool(rear_sensor_fault):
            self._set_lane_veto(
                side,
                "rear_sensor_fault",
                now,
            )

            return False

        zone = _safe_float(rear_zone)

        if zone is None:
            if AO_REQUIRE_REAR_ZONE:
                self._set_lane_veto(
                    side,
                    "rear_zone_unknown",
                    now,
                )

                return False

            return True

        if zone < 0.0:
            self._set_lane_veto(
                side,
                "rear_zone_invalid",
                now,
            )

            return False

        if zone >= AO_REAR_ZONE_BLOCK:
            self._set_lane_veto(
                side,
                "rear_zone_too_close",
                now,
            )

            return False

        return True

    # ==================================================================
    # Explicit obstacle interpretation
    # ==================================================================

    @staticmethod
    def _explicit_obstacle_present(
        obstacle,
    ):
        """
        Interpret an explicit sensor obstacle indication.

        None:
            caller did not provide an explicit obstacle state.

        False:
            sensor explicitly says no obstacle.

        True:
            immediate safety veto.
        """

        if obstacle is None:
            return None

        return bool(obstacle)

    # ==================================================================
    # MR76 safety
    # ==================================================================

    def _mr76_lane_safe(
        self,
        *,
        side,
        available,
        valid,
        age,
        obstacle,
        rear_dist,
        rear_speed,
        require_aux_sensors,
        now,
    ):
        """
        MR76 lane safety.

        Priority:

            unavailable/stale
                ->
            explicit obstacle
                ->
            rear target safety
                ->
            safe

        Important:

        An explicit MR76 obstacle is an unconditional veto.

        It does NOT matter whether:

            distance > 20m
            closing speed is low
            TTC is large

        if the caller says an object occupies the target lane,
        automatic overtaking is blocked.
        """

        # [AO_AUX_NO_DATA_PASS] available is None = 该侧 MR76 未接入。
        # 未接入不是"不安全"，只是"无法用此传感器收紧"。此时放行，
        # 其余判据（车道线置信度 / BSM / 距离带 / headway）仍然生效。
        if available is None:
            return True

        mr76_ok = _sensor_fresh(
            available,
            valid,
            age,
            MR76_MAX_AGE_SEC,
        )

        if require_aux_sensors and not mr76_ok:

            self._set_lane_veto(
                side,
                "mr76_unavailable_or_stale",
                now,
            )

            return False

        explicit_obstacle = (
            self._explicit_obstacle_present(
                obstacle
            )
        )

        if explicit_obstacle is True:

            self._set_lane_veto(
                side,
                "mr76_obstacle",
                now,
            )

            return False

        if not mr76_ok:
            return True

        # --------------------------------------------------------------
        # Legacy rear target safety
        # --------------------------------------------------------------

        if (
            rear_dist is not None
            or
            rear_speed is not None
        ):

            if self._rear_lane_blocked(
                rear_dist,
                rear_speed,
            ):

                self._set_lane_veto(
                    side,
                    "mr76_rear_traffic",
                    now,
                )

                return False

        return True

    # ==================================================================
    # LiDAR safety
    # ==================================================================

    def _lidar_lane_safe(
        self,
        *,
        side,
        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,
        lidar_obstacle,
        require_aux_sensors,
        now,
    ):
        """
        WiFi LiDAR lane safety.

        LiDAR obstacle has unconditional veto priority.

        Missing / stale LiDAR is unsafe when auxiliary sensors
        are required for automatic overtaking.
        """

        # [AO_AUX_NO_DATA_PASS] lidar_available is None = 该侧 LiDAR 未接入。
        # 本车 LiDAR 尚未接入，必须放行，否则自动超车被永久锁死。
        # 一旦将来接入（available=True），下面的严格判定立即生效。
        if lidar_available is None:
            return True

        lidar_ok = _sensor_fresh(
            lidar_available,
            lidar_valid,
            lidar_age,
            LIDAR_MAX_AGE_SEC,
        )

        if require_aux_sensors and not lidar_ok:

            self._set_lane_veto(
                side,
                "lidar_unavailable_or_stale",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Explicit obstacle flag
        # --------------------------------------------------------------

        explicit_obstacle = (
            self._explicit_obstacle_present(
                lidar_obstacle
            )
        )

        if explicit_obstacle is True:

            self._set_lane_veto(
                side,
                "lidar_obstacle",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Existing lidar_free interface
        # --------------------------------------------------------------

        if lidar_ok:

            if lidar_free is not True:

                self._set_lane_veto(
                    side,
                    "lidar_lane_not_clear",
                    now,
                )

                return False

        return True

    # ==================================================================
    # BSM safety
    # ==================================================================

    def _bsm_lane_safe(
        self,
        *,
        side,
        bsm_available,
        bsm_blocked,
        now,
    ):
        """
        BSM is another independent safety veto.

        BSM is never used to generate an overtaking request.
        """

        if not bool(bsm_available):
            self._set_lane_veto(
                side,
                "bsm_unavailable",
                now,
            )

            return False

        if bool(bsm_blocked):

            self._set_lane_veto(
                side,
                "bsm_obstacle",
                now,
            )

            return False

        return True

    # ==================================================================
    # [AO_LANE_CONF_GATE] Lane-line confidence hard gate
    # ==================================================================

    def _lane_conf_threshold(
        self,
        lane_prob_min,
    ):
        """Hard minimum lane-line probability.

        Takes the stricter of:
          - the module safety floor  AO_LANE_PROB_HARD_MIN
          - the caller preference   lane_prob_min (if usable)

        A caller preference of 0 / None is IGNORED: it is exactly the
        misconfiguration that disabled the filter on this car
        (AutoOvertakeLaneProbMin was set to 0.0).
        """

        threshold = AO_LANE_PROB_HARD_MIN

        pref = _safe_float(
            lane_prob_min
        )

        if (
            AO_LANE_PROB_USE_PARAM
            and
            pref is not None
            and
            pref > threshold
        ):
            threshold = pref

        return threshold

    def _update_lane_conf(
        self,
        now,
        left_lane_prob,
        right_lane_prob,
    ):
        """Maintain a short sticky window of lane-line confidence.

        A single low-probability frame must not instantly kill a lane,
        but a *sustained* low reading must.  We store the raw probability
        per sample and judge later (see _lane_conf_ok), because the verdict
        for one side also depends on the *other* side's reading
        (they are strongly anti-correlated on this car).
        """

        threshold = self._lane_conf_threshold(
            self._lane_prob_min_current
        )

        witness = max(
            threshold,
            AO_LANE_PROB_WITNESS,
        )

        for key, prob in (
            ("left", left_lane_prob),
            ("right", right_lane_prob),
        ):

            value = _safe_float(
                prob
            )

            hist = self._lane_conf_hist[
                key
            ]

            if value is None:

                hist.append(
                    (
                        now,
                        None,
                    )
                )

            else:

                self._lane_conf_raw[
                    key
                ] = value

                hist.append(
                    (
                        now,
                        bool(
                            value >= threshold
                        ),
                    )
                )

            cutoff = (
                now
                - AO_LANE_CONF_STICKY_SEC
            )

            while (
                hist
                and
                hist[0][0] < cutoff
            ):
                hist.pop(0)

        self._lane_conf_witness = witness

    def _lane_conf_ok(
        self,
        side,
    ):
        """Asymmetric lane-line confidence verdict for one side.

        Because lp_left and lp_right are strongly ANTI-correlated on this
        car (corr = -0.68), demanding both sides be confident would lock
        out overtaking >90% of the time.  The correct test is:

          1. The target side is confident enough on its own; OR
          2. The opposite side is *highly* confident (witness), which
             proves the car is well-contained between two lines, so a
             weak reading on the target side is a slot-assignment
             artefact rather than a missing lane.

        Fail-closed when we have no usable sample at all.
        """

        hist = self._lane_conf_hist[
            side
        ]

        other = (
            "right"
            if side == "left"
            else
            "left"
        )

        # --- 1) direct evidence on this side -------------------------
        seen = False
        all_ok = True

        for _, ok in hist:

            if ok is None:

                continue

            seen = True

            if not ok:

                all_ok = False

                break

        if seen and all_ok:

            return True

        # --- 2) witness from the opposite side -----------------------
        raw_other = self._lane_conf_raw.get(
            other
        )

        witness = getattr(
            self,
            "_lane_conf_witness",
            AO_LANE_PROB_WITNESS,
        )

        if (
            raw_other is not None
            and
            raw_other >= witness
        ):

            return True

        # --- 3) fail closed -----------------------------------------
        return False

    def _lane_conf_raw_of(
        self,
        side,
    ):
        """Latest raw probability seen for a side (diagnostics)."""

        return self._lane_conf_raw.get(
            side
        )

    def _lane_conf_reason(
        self,
        side,
    ):
        """Human-readable veto reason for the confidence gate."""

        return (
            "left_lane_conf_low"
            if side == "left"
            else
            "right_lane_conf_low"
        )

    # ==================================================================
    # [AO_JUNCTION_GATE] Junction / stop-line zone
    # ==================================================================

    def _update_junction_guess(
        self,
        now,
        left_lane_prob,
        right_lane_prob,
        junction_dist,
    ):
        """Heuristic junction detection from lane-line dropout.

        Near an intersection / the start of a solid-line section the
        model typically loses the lane lines: probabilities collapse or
        become intermittent.  We require the dropout to persist for
        AO_JUNCTION_CONFIRM_SEC before declaring a junction zone, so a
        single noisy frame cannot ban overtaking.
        """

        lp_left = _safe_float(
            left_lane_prob
        )

        lp_right = _safe_float(
            right_lane_prob
        )

        # Both sides must be weak for this to look like a junction.
        # (A single weak side is far more likely to be a shadow / worn
        #  paint on one line only.)
        left_weak = (
            lp_left is not None
            and
            lp_left < AO_JUNCTION_LANE_PROB
        )

        right_weak = (
            lp_right is not None
            and
            lp_right < AO_JUNCTION_LANE_PROB
        )

        # Explicit map signal outranks the heuristic.
        d = _safe_float(
            junction_dist
        )

        if (
            d is not None
            and
            d <= AO_JUNCTION_BAN_DIST_M
        ):

            # Map says we are inside the ban zone: latch it.
            if self._junction_since is None:

                self._junction_since = now

            self._junction_source = "map"

            return

        if left_weak and right_weak:

            if self._junction_since is None:

                self._junction_since = now

                self._junction_samples = 0

            self._junction_samples += 1

            # Only promote to an active ban once it has persisted.
            if (
                self._junction_source
                != "map"
            ):
                self._junction_source = "lane_dropout"

            return

        # Lane lines look healthy again -> clear the latch.
        self._junction_since = None

        self._junction_source = None

        self._junction_samples = 0

    def _junction_ban_active(
        self,
        now,
        junction_dist,
    ):
        """True when automatic lane changes must be forbidden."""

        # Map source: authoritative whenever inside the ban distance.
        d = _safe_float(
            junction_dist
        )

        if (
            d is not None
            and
            d <= AO_JUNCTION_BAN_DIST_M
        ):

            return True

        # Heuristic source: require the dropout to have persisted in
        # BOTH wall-clock time and sample count.
        if self._junction_since is None:

            return False

        if (
            now
            - self._junction_since
        ) < AO_JUNCTION_CONFIRM_SEC:

            return False

        return bool(
            self._junction_samples
            >= AO_JUNCTION_MIN_SAMPLES
        )

    # ==================================================================
    # Lane sensor safety
    # ==================================================================

    def _lane_sensor_safe(
        self,
        *,
        side,
        rear_dist,
        rear_speed,

        mr76_available,
        mr76_valid,
        mr76_age,
        mr76_obstacle,

        lidar_free,
        lidar_available,
        lidar_valid,
        lidar_age,
        lidar_obstacle,

        bsm_blocked,
        bsm_available,

        # [AO_SAFETY_TIGHTEN] rear approach-zone inputs
        rear_zone=None,
        rear_sensor_fault=None,

        base_lane_ok,

        require_aux_sensors,

        now,
    ):
        """
        Final safety gate.

        ALL safety sources must pass.

            base lane
                AND
            MR76
                AND
            LiDAR
                AND
            BSM

        No sensor is allowed to override another sensor.
        """

        # --------------------------------------------------------------
        # Existing openpilot / lane-model safety
        # --------------------------------------------------------------

        if not bool(base_lane_ok):

            self._set_lane_veto(
                side,
                "base_lane_not_safe",
                now,
            )

            return False

        # --------------------------------------------------------------
        # Existing latched safety veto
        # --------------------------------------------------------------

        if self._lane_vetoed(
            side,
            now,
        ):

            return False

        # --------------------------------------------------------------
        # [AO_SAFETY_TIGHTEN] rear approach-zone gate
        #
        # Runs before every other auxiliary gate so an insufficient rear
        # gap always blocks, whatever the MR76 / LiDAR plumbing does.
        # --------------------------------------------------------------

        if not self._rear_zone_gate(
            side=side,
            rear_zone=rear_zone,
            rear_sensor_fault=rear_sensor_fault,
            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # MR76
        # --------------------------------------------------------------

        if not self._mr76_lane_safe(
            side=side,

            available=mr76_available,
            valid=mr76_valid,
            age=mr76_age,
            obstacle=mr76_obstacle,

            rear_dist=rear_dist,
            rear_speed=rear_speed,

            require_aux_sensors=require_aux_sensors,

            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # LiDAR
        # --------------------------------------------------------------

        if not self._lidar_lane_safe(
            side=side,

            lidar_free=lidar_free,
            lidar_available=lidar_available,
            lidar_valid=lidar_valid,
            lidar_age=lidar_age,
            lidar_obstacle=lidar_obstacle,

            require_aux_sensors=require_aux_sensors,

            now=now,
        ):
            return False

        # --------------------------------------------------------------
        # BSM
        # --------------------------------------------------------------

        if not self._bsm_lane_safe(
            side=side,
            bsm_available=bsm_available,
            bsm_blocked=bsm_blocked,
            now=now,
        ):
            return False

        return True

    # ==================================================================
    # Lane scoring
    # ==================================================================

    @staticmethod
    def _lane_score(
        *,
        side,
        rear_dist,
        rear_speed,
        lane_preference,
        is_highway,
    ):
        """
        Safety is completed BEFORE this function.

        Scoring can only select between lanes that are already safe.
        """

        score = 0.0

        distance = _safe_float(
            rear_dist
        )

        speed = _safe_float(
            rear_speed
        )

        if distance is not None:
            score += min(
                distance,
                60.0,
            ) * 0.10

        if speed is not None:
            score -= max(
                0.0,
                speed,
            ) * 0.5

        if is_highway:

            if lane_preference == LANE_PREF_KEEP_LEFT:

                score += (
                    3.0
                    if side == "left"
                    else -3.0
                )

            elif lane_preference == LANE_PREF_KEEP_RIGHT:

                score += (
                    3.0
                    if side == "right"
                    else -3.0
                )

            elif lane_preference == LANE_PREF_AUTO:

                score += (
                    0.5
                    if side == "left"
                    else 0.0
                )

        return score

    # ==================================================================
    # Choose best lane
    # ==================================================================

    def _choose_best_lane(
        self,
        *,
        left_safe,
        right_safe,
        left_score,
        right_score,
        prefer_dir,
    ):

        if (
            prefer_dir == LaneChangeDirection.left
            and left_safe
        ):
            return LaneChangeDirection.left

        if (
            prefer_dir == LaneChangeDirection.right
            and right_safe
        ):
            return LaneChangeDirection.right

        if left_safe and right_safe:

            if left_score > right_score:
                return LaneChangeDirection.left

            if right_score > left_score:
                return LaneChangeDirection.right

            return LaneChangeDirection.none

        if left_safe:
            return LaneChangeDirection.left

        if right_safe:
            return LaneChangeDirection.right

        return LaneChangeDirection.none

    # ==================================================================
    # OEM lead / overtaking need
    # ==================================================================

    def _update_need_overtake(
        self,
        *,
        now,
        lead_present,
        lead_d,
        v_lead,
        v_ego,
        v_cruise,
    ):
        """
        OEM lead is the ONLY normal overtaking trigger.

        MR76 and LiDAR are deliberately absent from this function.
        """

        if not bool(lead_present):

            self._need_since = None

            return False

        lead_d = _safe_float(
            lead_d
        )

        v_lead = _safe_float(
            v_lead
        )

        v_ego = _safe_float(
            v_ego
        )

        v_cruise = _safe_float(
            v_cruise
        )

        if (
            lead_d is None
            or v_lead is None
            or v_ego is None
            or v_cruise is None
            or lead_d <= 0.0
        ):

            self._need_since = None

            return False

        headway = (
            lead_d
            /
            max(
                v_ego,
                0.1,
            )
        )

        closing_speed = (
            v_ego
            -
            v_lead
        )

        # ==============================================================
        # [AO_SAFETY_TIGHTEN] Physical need gate.
        #
        # Measured defect: with cruise = 100 km/h and a lead at 74 km/h the
        # old predicate
        #       (v_cruise - v_lead) >= OVERTAKE_SPEED_DELTA
        # is true even when ego is NOT closing on the lead (closing speed
        # measured negative in the field log).  The overtake was therefore
        # requested while the relative speed was zero or negative.
        #
        # The gate below requires a *sustained physical* closing speed
        # (ego minus lead), a sane lead distance band, and the absolute
        # 90 km/h floor.
        # ==============================================================

        ego_closing = (
            v_ego
            -
            v_lead
        )

        if (
            lead_d
            < OVERTAKE_LEAD_MIN_DIST
            or
            lead_d
            > OVERTAKE_LEAD_MAX_DIST
        ):

            self._need_since = None

            return False

        need_raw = (
            ego_closing
            >= OVERTAKE_MIN_CLOSING_SPEED

            and

            (
                v_cruise
                -
                v_lead
            )
            >= OVERTAKE_SPEED_DELTA

            and

            headway
            <= OVERTAKE_HEADWAY_MAX_S

            and

            # [AO_MIN_SPEED_CRUISE_HOOKUP] 原为硬编码 OVERTAKE_MIN_SPEED
            # (90 km/h)，导致低速路况永不触发。改为与巡航挂钩，见
            # _overtake_min_speed()。
            v_ego
            >= _overtake_min_speed(v_cruise)
        )

        if not need_raw:

            self._need_since = None

            return False

        if self._need_since is None:
            self._need_since = now

        return (
            now
            -
            self._need_since
        ) >= OVERTAKE_LEAD_STABLE_SEC

    # ==================================================================
    # Lead history
    # ==================================================================

    def _update_lead_history(
        self,
        now,
        lead_present,
        lead_d,
    ):
        lead_d = _safe_float(
            lead_d
        )

        if (
            not lead_present
            or lead_d is None
            or lead_d <= 0.0
        ):

            if self._lead_lost_since is None:
                self._lead_lost_since = now

            self._lead_distance_increasing_since = None
            self._lead_last_distance = None

            return

        self._lead_lost_since = None

        if self._lead_last_distance is None:

            self._lead_last_distance = lead_d
            self._lead_distance_increasing_since = None

            return

        if lead_d > self._lead_last_distance:

            if self._lead_distance_increasing_since is None:
                self._lead_distance_increasing_since = now

        else:

            self._lead_distance_increasing_since = None

        self._lead_last_distance = lead_d

    # ==================================================================
    # Passed lead
    # ==================================================================

    def _lead_passed(
        self,
        *,
        now,
        lead_present,
        lead_d,
        v_ego,
        v_lead,
    ):
        """
        Passing requires spatial evidence or stable lead disappearance.
        """

        lead_d = _safe_float(
            lead_d
        )

        v_ego = _safe_float(
            v_ego
        )

        v_lead = _safe_float(
            v_lead
        )

        # --------------------------------------------------------------
        # Stable lead disappearance
        # --------------------------------------------------------------

        if not lead_present:

            if (
                self._lead_lost_since is not None
                and
                (
                    now
                    -
                    self._lead_lost_since
                )
                >= LEAD_LOST_STABLE_SEC
            ):
                return True

        # --------------------------------------------------------------
        # Spatial pass
        # --------------------------------------------------------------

        if (
            lead_d is not None
            and
            lead_d >= PASSED_LEAD_DIST
            and
            self._lead_distance_increasing_since is not None
            and
            (
                now
                -
                self._lead_distance_increasing_since
            )
            >= PASSED_LEAD_STABLE_SEC
        ):
            return True

        # --------------------------------------------------------------
        # Speed advantage requires spatial confirmation
        # --------------------------------------------------------------

        if (
            lead_d is not None
            and
            v_ego is not None
            and
            v_lead is not None
        ):

            speed_advantage = (
                v_ego
                -
                v_lead
            )

            if (
                speed_advantage >= PASSED_LEAD_SPEED
                and
                lead_d >= PASSED_LEAD_DIST
            ):
                return True

        return False

    # ==================================================================
    # Main update
    # ==================================================================

    def update(
        self,
        *,
        enabled,
        lc_state,
        v_ego,
        v_cruise,
        lead_present,
        lead_d,
        v_lead,
        left_ok,
        right_ok,
        is_rhd=False,
        manual_blinker=False,
        bsm_available=True,

        rear_left_dist=None,
        rear_left_speed=None,
        rear_right_dist=None,
        rear_right_speed=None,

        left_lidar_free=None,
        right_lidar_free=None,

        left_bsm=False,
        right_bsm=False,

        # --------------------------------------------------------------
        # [AO_SAFETY_TIGHTEN] rear approach zones (0..4, 0 = clear)
        # --------------------------------------------------------------
        left_rear_zone=None,
        right_rear_zone=None,
        rear_sensor_fault=None,

        lane_preference=LANE_PREF_AUTO,
        min_cruise_speed=None,

        # --------------------------------------------------------------
        # MR76 explicit state
        # --------------------------------------------------------------

        mr76_left_available=None,
        mr76_right_available=None,

        mr76_left_valid=None,
        mr76_right_valid=None,

        mr76_left_age=None,
        mr76_right_age=None,

        # --------------------------------------------------------------
        # IMPORTANT:
        #
        # Explicit obstacle flags.
        #
        # True means:
        #
        #     obstacle detected in corresponding lane
        #
        # and therefore automatic overtaking is FORBIDDEN.
        #
        # None means caller did not provide explicit occupancy.
        # --------------------------------------------------------------

        mr76_left_obstacle=None,
        mr76_right_obstacle=None,

        # --------------------------------------------------------------
        # WiFi LiDAR state
        # --------------------------------------------------------------

        left_lidar_available=None,
        right_lidar_available=None,

        left_lidar_valid=None,
        right_lidar_valid=None,

        left_lidar_age=None,
        right_lidar_age=None,

        # --------------------------------------------------------------
        # Explicit LiDAR obstacle flags.
        #
        # True:
        #     obstacle detected
        #
        # False:
        #     explicitly clear
        #
        # None:
        #     not provided
        # --------------------------------------------------------------

        left_lidar_obstacle=None,
        right_lidar_obstacle=None,

        # --------------------------------------------------------------
        # [AO_LANE_CONF_GATE] 目标车道线概率（0..1）。
        #
        # 接线侧必须传入 modelV2.laneLineProbs[0] / [3]。
        # None 表示未提供 -> 该硬门不生效（保持向后兼容）。
        # --------------------------------------------------------------

        left_lane_prob=None,
        right_lane_prob=None,

        # 接线侧原本的 lane_prob_min（用户偏好）。AO 会与本模块的
        # 安全硬门 AO_LANE_PROB_HARD_MIN 取较严者。
        lane_prob_min=None,

        # --------------------------------------------------------------
        # [AO_JUNCTION_GATE] 路口距离（米）。
        #
        # 来自地图数据。当前 Custom.AmapNavi 尚无此字段，接线侧传 None
        # 即可；一旦地图侧提供，直接传入即生效（无需改本文件）。
        #
        # 语义：距最近路口（红绿灯/交叉口）的距离，None = 未知。
        # --------------------------------------------------------------

        junction_dist=None,

        # --------------------------------------------------------------
        # Integration policy
        #
        # [AO_AUX_SENSOR_STRICT] 注意：接线侧曾把本参数显式传成 False，
        # 从而绕过了 MR76 / LiDAR 的传感器健康门（见 _mr76_lane_safe /
        # _lidar_lane_safe 里的 `if require_aux_sensors and not ...`），
        # 变道安全性退化为"仅靠 BSM 布尔量"。本轮接线侧已改回 True。
        #
        # 若本参数确需为 False（例如台架无传感器），必须显式传入；
        # 内部默认保持 True（fail closed）。
        # --------------------------------------------------------------

        require_aux_sensors=True,
    ):

        now = time.monotonic()

        request = LaneChangeDirection.none

        # [AO_LANE_CONF_GATE] remember this cycle's caller preference so
        # the confidence hard gate can take the stricter threshold.
        self._lane_prob_min_current = lane_prob_min

        v_ego = _safe_float(
            v_ego
        )

        v_cruise = _safe_float(
            v_cruise
        )

        if (
            v_ego is None
            or
            v_cruise is None
        ):

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Manual driver control
        # ==============================================================

        if manual_blinker:

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Enable gate
        # ==============================================================

        if not enabled:

            self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # BSM integration
        # ==============================================================

        if not bsm_available:

            self.reset()

            self._last_lc_state = lc_state

            return request

        # [AO_MIN_SPEED_CRUISE_HOOKUP] 巡航门槛默认值同样与 v_cruise 挂钩，
        # 否则"v_cruise >= 90km/h"这一条在城郊路况下仍会恒假。
        _ao_default_min_cruise = _overtake_min_speed(v_cruise)

        min_cruise_speed = (
            _ao_default_min_cruise
            if min_cruise_speed is None
            else _safe_float(
                min_cruise_speed
            )
        )

        if min_cruise_speed is None:
            min_cruise_speed = (
                _ao_default_min_cruise
            )

        # ==============================================================
        # Speed gate
        # ==============================================================

        # [AO_MIN_SPEED_CRUISE_HOOKUP] 速度门改为与巡航挂钩的动态门槛。
        # 原 v_ego < OVERTAKE_MIN_SPEED (90 km/h) 在城郊路况下恒真，
        # 直接 reset()，使后续所有逻辑（含车道选择/确认）永不执行。
        _ao_v_min = _overtake_min_speed(v_cruise)

        if (
            v_ego < _ao_v_min
            or
            v_cruise < min_cruise_speed
        ):

            if self._mode in (
                "idle",
                "preparing",
            ):
                self.reset()

            self._last_lc_state = lc_state

            return request

        # ==============================================================
        # Sensor availability compatibility
        # ==============================================================

        # ==============================================================
        # [AO_AUX_NO_DATA_PASS] 辅助传感器 "未接入" 与 "已接入但异常" 必须区分。
        #
        # 原实现把"三路输入全 None"强制推导成 False，于是
        # _sensor_fresh(False, ...) = False，在 require_aux_sensors=True 下
        # 直接 veto -> 两侧车道恒不安全 -> 自动超车 100% 不触发。
        #
        # 新语义：
        #   三路输入全 None -> 保持 None（= 该传感器未接入）-> 门放行
        #   只要有任一路输入 -> 推导为 True（= 已接入）-> 严格判定
        # ==============================================================

        if mr76_left_available is None:

            _mr76_left_any = (
                rear_left_dist is not None
                or
                rear_left_speed is not None
                or
                mr76_left_obstacle is not None
            )

            if _mr76_left_any:
                mr76_left_available = True

        if mr76_right_available is None:

            _mr76_right_any = (
                rear_right_dist is not None
                or
                rear_right_speed is not None
                or
                mr76_right_obstacle is not None
            )

            if _mr76_right_any:
                mr76_right_available = True

        if left_lidar_available is None:

            _lidar_left_any = (
                left_lidar_free is not None
                or
                left_lidar_obstacle is not None
            )

            if _lidar_left_any:
                left_lidar_available = True

        if right_lidar_available is None:

            _lidar_right_any = (
                right_lidar_free is not None
                or
                right_lidar_obstacle is not None
            )

            if _lidar_right_any:
                right_lidar_available = True

        # ==============================================================
        # Base lane safety
        # ==============================================================

        left_base = bool(
            left_ok
        )

        right_base = bool(
            right_ok
        )

        # ==============================================================
        # [AO_LANE_CONF_GATE] 目标车道线置信度硬门
        #
        # 见文件头部 AO_LANE_PROB_HARD_MIN 处的说明。核心：
        #   接线侧 lp >= lane_prob_min 在 lane_prob_min=0 时恒真，
        #   导致车道线置信度为 0 也判可用。
        # 本处再设一道不可被参数绕过硬门，并与参数取较严者。
        # ==============================================================

        self._update_lane_conf(
            now,
            left_lane_prob,
            right_lane_prob,
        )

        left_lane_conf_ok = self._lane_conf_ok(
            "left"
        )

        right_lane_conf_ok = self._lane_conf_ok(
            "right"
        )

        if not left_lane_conf_ok:

            left_base = False

        if not right_lane_conf_ok:

            right_base = False

        # ==============================================================
        # [AO_JUNCTION_GATE] 路口 / 红绿灯前禁止自动变道
        #
        # 车无法识别实线，且模型不输出线型，故以"接近路口"作为
        # 禁止变道的保守区间（默认路口前 15m）。
        #
        # 两路来源，任一命中即禁止：
        #   (a) 地图给出的 junction_dist <= AO_JUNCTION_BAN_DIST_M
        #   (b) 车道线断段启发式（见 _update_junction_guess）
        # ==============================================================

        self._update_junction_guess(
            now,
            left_lane_prob,
            right_lane_prob,
            junction_dist,
        )

        junction_ban = self._junction_ban_active(
            now,
            junction_dist,
        )

        if junction_ban:

            left_base = False

            right_base = False

        self._last_junction_ban = bool(
            junction_ban
        )

        # ==============================================================
        # Calculate current lane safety
        # ==============================================================

        left_safe = self._lane_sensor_safe(
            side="left",

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            mr76_available=mr76_left_available,
            mr76_valid=mr76_left_valid,
            mr76_age=mr76_left_age,
            mr76_obstacle=mr76_left_obstacle,

            lidar_free=left_lidar_free,
            lidar_available=left_lidar_available,
            lidar_valid=left_lidar_valid,
            lidar_age=left_lidar_age,
            lidar_obstacle=left_lidar_obstacle,

            bsm_blocked=left_bsm,
            bsm_available=bsm_available,

            rear_zone=left_rear_zone,
            rear_sensor_fault=rear_sensor_fault,

            base_lane_ok=left_base,

            require_aux_sensors=require_aux_sensors,

            now=now,
        )

        right_safe = self._lane_sensor_safe(
            side="right",

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            mr76_available=mr76_right_available,
            mr76_valid=mr76_right_valid,
            mr76_age=mr76_right_age,
            mr76_obstacle=mr76_right_obstacle,

            lidar_free=right_lidar_free,
            lidar_available=right_lidar_available,
            lidar_valid=right_lidar_valid,
            lidar_age=right_lidar_age,
            lidar_obstacle=right_lidar_obstacle,

            bsm_blocked=right_bsm,
            bsm_available=bsm_available,

            rear_zone=right_rear_zone,
            rear_sensor_fault=rear_sensor_fault,

            base_lane_ok=right_base,

            require_aux_sensors=require_aux_sensors,

            now=now,
        )

        # ==============================================================
        # IMPORTANT:
        #
        # Explicit obstacle gets absolute veto priority.
        #
        # This additional check makes the intended semantics obvious:
        #
        #     MR76 obstacle -> no lane
        #     LiDAR obstacle -> no lane
        #
        # even if another sensor says clear.
        # ==============================================================

        if mr76_left_obstacle is True:

            left_safe = False

            self._set_lane_veto(
                "left",
                "mr76_obstacle",
                now,
            )

        if mr76_right_obstacle is True:

            right_safe = False

            self._set_lane_veto(
                "right",
                "mr76_obstacle",
                now,
            )

        if left_lidar_obstacle is True:

            left_safe = False

            self._set_lane_veto(
                "left",
                "lidar_obstacle",
                now,
            )

        if right_lidar_obstacle is True:

            right_safe = False

            self._set_lane_veto(
                "right",
                "lidar_obstacle",
                now,
            )

        # ==============================================================
        # Stable lane safety
        # ==============================================================

        (
            self._left_ok_since,
            left_stable,
        ) = self._stable_ok(
            left_safe,
            self._left_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
        )

        (
            self._right_ok_since,
            right_stable,
        ) = self._stable_ok(
            right_safe,
            self._right_ok_since,
            now,
            CLEAR_LANE_STABLE_SEC,
        )

        # ==============================================================
        # OEM lead history
        # ==============================================================

        self._update_lead_history(
            now,
            bool(lead_present),
            lead_d,
        )

        # ==============================================================
        # OEM lead = ONLY overtaking trigger
        # ==============================================================

        need_overtake = self._update_need_overtake(
            now=now,

            lead_present=bool(
                lead_present
            ),

            lead_d=lead_d,

            v_lead=v_lead,

            v_ego=v_ego,

            v_cruise=v_cruise,
        )

        # ==============================================================
        # Lane scoring
        # ==============================================================

        is_highway = (
            v_ego >= 22.0
        )

        left_score = self._lane_score(
            side="left",

            rear_dist=rear_left_dist,
            rear_speed=rear_left_speed,

            lane_preference=lane_preference,

            is_highway=is_highway,
        )

        right_score = self._lane_score(
            side="right",

            rear_dist=rear_right_dist,
            rear_speed=rear_right_speed,

            lane_preference=lane_preference,

            is_highway=is_highway,
        )

        best_dir = self._choose_best_lane(
            left_safe=left_stable,
            right_safe=right_stable,

            left_score=left_score,
            right_score=right_score,

            prefer_dir=(
                LaneChangeDirection.none
            ),
        )

        # ==============================================================
        # Explicit preference only after safety
        # ==============================================================

        if (
            lane_preference
            ==
            LANE_PREF_KEEP_LEFT
        ):

            if left_stable:
                best_dir = (
                    LaneChangeDirection.left
                )

            else:
                best_dir = (
                    LaneChangeDirection.none
                )

        elif (
            lane_preference
            ==
            LANE_PREF_KEEP_RIGHT
        ):

            if right_stable:
                best_dir = (
                    LaneChangeDirection.right
                )

            else:
                best_dir = (
                    LaneChangeDirection.none
                )

        # ==============================================================
        # No safe lane
        # ==============================================================

        if (
            need_overtake
            and
            best_dir
            ==
            LaneChangeDirection.none
        ):

            if (
                mr76_left_obstacle is True
                or
                left_lidar_obstacle is True
            ):

                if (
                    mr76_left_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.left,
                        "mr76_left_obstacle",
                    )

                elif (
                    left_lidar_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.left,
                        "lidar_left_obstacle",
                    )

            if (
                mr76_right_obstacle is True
                or
                right_lidar_obstacle is True
            ):

                if (
                    mr76_right_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.right,
                        "mr76_right_obstacle",
                    )

                elif (
                    right_lidar_obstacle is True
                ):
                    self._record_veto(
                        LaneChangeDirection.right,
                        "lidar_right_obstacle",
                    )

            self._last_reason = (
                "no_safe_lane"
            )

        # ==============================================================
        # State machine
        # ==============================================================

        if self._mode == "idle":

            if (
                need_overtake
                and
                best_dir
                !=
                LaneChangeDirection.none
                and
                now >= self._cooldown_until
            ):

                self._mode = "preparing"

                self._prepare_since = now

                self._out_dir = best_dir

                self._last_reason = (
                    "overtake_prepare"
                )

        # ==============================================================
        # PREPARING
        # ==============================================================

        elif self._mode == "preparing":

            # ----------------------------------------------------------
            # Revalidate safety every control cycle.
            # ----------------------------------------------------------

            current_safe = (
                left_stable
                if
                self._out_dir
                ==
                LaneChangeDirection.left
                else
                right_stable
            )

            if not need_overtake:

                self.reset()

            elif not current_safe:

                if (
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                ):

                    self._record_veto(
                        self._out_dir,
                        self._left_veto_reason
                        or
                        "left_lane_safety_veto",
                    )

                else:

                    self._record_veto(
                        self._out_dir,
                        self._right_veto_reason
                        or
                        "right_lane_safety_veto",
                    )

                self._mode = "idle"

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._prepare_since = None

            elif (
                self._prepare_since is None
            ):

                self._prepare_since = now

            elif (
                now
                -
                self._prepare_since
            ) >= PREPARE_BEFORE_LC_SEC:

                # ------------------------------------------------------
                # FINAL SAFETY CHECK BEFORE REQUEST
                # ------------------------------------------------------

                final_safe = (
                    left_stable
                    if
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                    else
                    right_stable
                )

                if not final_safe:

                    self._record_veto(
                        self._out_dir,
                        "final_lane_safety_veto",
                    )

                    self._mode = "idle"

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._prepare_since = None

                else:

                    self._mode = (
                        "changing_out"
                    )

                    self._observed_out_lc = False

                    request = (
                        self._out_dir
                    )

                    self._last_reason = (
                        "overtake_request"
                    )

        # ==============================================================
        # CHANGING OUT
        # ==============================================================

        elif self._mode == "changing_out":

            # ----------------------------------------------------------
            # Direction remains locked.
            #
            # Before DesireHelper actually starts the LC, continue
            # applying the safety veto.
            # ----------------------------------------------------------

            if (
                lc_state
                ==
                LaneChangeState.laneChangeStarting
            ):

                self._observed_out_lc = True

            if not self._observed_out_lc:

                current_safe = (
                    left_stable
                    if
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                    else
                    right_stable
                )

                if not current_safe:

                    self._record_veto(
                        self._out_dir,
                        "lane_safety_veto_before_start",
                    )

                    self._mode = "idle"

                    self._out_dir = (
                        LaneChangeDirection.none
                    )

                    self._prepare_since = None

                    request = (
                        LaneChangeDirection.none
                    )

                elif (
                    self._out_dir
                    ==
                    LaneChangeDirection.left
                ):

                    if left_stable:
                        request = (
                            self._out_dir
                        )

                elif (
                    self._out_dir
                    ==
                    LaneChangeDirection.right
                ):

                    if right_stable:
                        request = (
                            self._out_dir
                        )

            # ----------------------------------------------------------
            # Once DesireHelper has actually started the maneuver,
            # do not issue an opposite-direction command from this
            # helper.
            #
            # Emergency avoidance remains the responsibility of the
            # dedicated AutoAvoidance layer.
            # ----------------------------------------------------------

            if (
                self._observed_out_lc
                and
                lc_state
                ==
                LaneChangeState.off
            ):

                self._observed_out_lc = False

                self._mode = (
                    "waiting_return"
                )

                self._out_finished_t = now

                self._clear_since = None

                self._last_overtake_time = now

                self._last_reason = (
                    "overtake_completed"
                )

        # ==============================================================
        # WAITING RETURN
        # ==============================================================

        elif self._mode == "waiting_return":

            lead_speed = _safe_float(
                v_lead
            )

            lead_distance = _safe_float(
                lead_d
            )

            keep_overtaking = False

            if (
                lead_present
                and
                lead_speed is not None
                and
                lead_distance is not None
            ):

                if (
                    v_ego
                    -
                    lead_speed
                    > 2.0
                    and
                    lead_distance
                    < 50.0
                ):

                    keep_overtaking = True

            passed_lead = self._lead_passed(
                now=now,

                lead_present=bool(
                    lead_present
                ),

                lead_d=lead_d,

                v_ego=v_ego,

                v_lead=v_lead,
            )

            return_dir = self._opposite(
                self._out_dir
            )

            return_lane_safe = (
                (
                    return_dir
                    ==
                    LaneChangeDirection.left
                    and
                    left_stable
                )
                or
                (
                    return_dir
                    ==
                    LaneChangeDirection.right
                    and
                    right_stable
                )
            )

            if keep_overtaking:

                self._stay_in_fast_lane = True

                self._clear_since = None

                self._last_reason = (
                    "stay_fast_lane"
                )

            elif not passed_lead:

                self._stay_in_fast_lane = False

                self._clear_since = None

                self._last_reason = (
                    "waiting_pass_confirmation"
                )

            elif not return_lane_safe:

                self._stay_in_fast_lane = False

                self._clear_since = None

                self._last_reason = (
                    "return_lane_not_safe"
                )

            else:

                self._stay_in_fast_lane = False

                if self._clear_since is None:
                    self._clear_since = now

                dwell_ok = (
                    self._out_finished_t
                    is not None
                    and
                    (
                        now
                        -
                        self._out_finished_t
                    )
                    >= RETURN_MIN_TIME_AFTER_OUT_SEC
                )

                clear_ok = (
                    now
                    -
                    self._clear_since
                ) >= RETURN_CLEAR_DELAY_SEC

                if (
                    dwell_ok
                    and
                    clear_ok
                ):

                    self._mode = (
                        "changing_back"
                    )

                    self._return_dir = (
                        return_dir
                    )

                    self._observed_return_lc = False

                    request = (
                        self._return_dir
                    )

                    self._last_reason = (
                        "return_request"
                    )

        # ==============================================================
        # CHANGING BACK
        # ==============================================================

        elif self._mode == "changing_back":

            if (
                lc_state
                ==
                LaneChangeState.laneChangeStarting
            ):

                self._observed_return_lc = True

            if not self._observed_return_lc:

                request = (
                    self._return_dir
                )

            if (
                self._observed_return_lc
                and
                lc_state
                ==
                LaneChangeState.off
            ):

                self._observed_return_lc = False

                self._mode = "idle"

                self._cooldown_until = (
                    now
                    +
                    OVERTAKE_COOLDOWN_SEC
                )

                self._return_dir = (
                    LaneChangeDirection.none
                )

                self._out_dir = (
                    LaneChangeDirection.none
                )

                self._last_reason = (
                    "return_completed"
                )

        else:

            self.reset()

        self._last_lc_state = lc_state

        # ==============================================================
        # [AO_CONF_LOG] calibration log (throttled, best-effort)
        # ==============================================================

        self._write_conf_log(
            now,

            lane_prob_L=left_lane_prob,
            lane_prob_R=right_lane_prob,

            junction_dist=junction_dist,

            speed_mps=v_ego,
        )

        return request

    # ==================================================================
    # Debug
    # ==================================================================

    def get_state(self):

        now = time.monotonic()

        return {

            "mode":
                self._mode,

            "out_dir":
                self._out_dir,

            "return_dir":
                self._return_dir,

            "cooldown_until":
                self._cooldown_until,

            "need_since":
                self._need_since,

            "prepare_since":
                self._prepare_since,

            "clear_since":
                self._clear_since,

            "out_finished_t":
                self._out_finished_t,

            "left_ok_since":
                self._left_ok_since,

            "right_ok_since":
                self._right_ok_since,

            "last_overtake_time":
                self._last_overtake_time,

            "stay_in_fast_lane":
                self._stay_in_fast_lane,

            "last_reason":
                self._last_reason,

            # ----------------------------------------------------------
            # Safety veto diagnostics
            # ----------------------------------------------------------

            "left_veto_active":
                now < self._left_veto_until,

            "right_veto_active":
                now < self._right_veto_until,

            "left_veto_until":
                self._left_veto_until,

            "right_veto_until":
                self._right_veto_until,

            "left_veto_reason":
                self._left_veto_reason,

            "right_veto_reason":
                self._right_veto_reason,

            "last_veto_direction":
                self._last_veto_direction,

            "last_veto_reason":
                self._last_veto_reason,

            # ----------------------------------------------------------
            # [AO_LANE_CONF_GATE] / [AO_JUNCTION_GATE] diagnostics
            # ----------------------------------------------------------

            "lane_conf_L":
                self._lane_conf_ok("left"),

            "lane_conf_R":
                self._lane_conf_ok("right"),

            "lane_conf_threshold":
                self._lane_conf_threshold(
                    self._lane_prob_min_current
                ),

            "lane_conf_witness":
                self._lane_conf_witness,

            "lane_conf_L_last":
                self._lane_conf_last("left"),

            "lane_conf_R_last":
                self._lane_conf_last("right"),

            "lane_prob_L_raw":
                self._lane_conf_raw_of("left"),

            "lane_prob_R_raw":
                self._lane_conf_raw_of("right"),

            "junction_ban":
                self._last_junction_ban,

            "junction_source":
                self._junction_source,

            "junction_since":
                self._junction_since,
        }

    def _lane_conf_last(
        self,
        side,
    ):
        """Latest raw lane-line probability observed for diagnostics."""

        hist = self._lane_conf_hist[
            side
        ]

        for _, ok in reversed(
            hist
        ):

            if ok is not None:

                return ok

        return None

    # ==================================================================
    # [AO_CONF_LOG] calibration log
    # ==================================================================

    def _write_conf_log(
        self,
        now,
        *,
        lane_prob_L,
        lane_prob_R,
        junction_dist=None,
        speed_mps=None,
        mode=None,
    ):
        """Append one calibration record to AO_CONF_LOG_PATH.

        Throttled to AO_CONF_LOG_PERIOD_SEC.  Best-effort: any I/O
        failure is swallowed so the control loop is never disturbed.
        """

        if (
            now
            - self._conf_log_t
        ) < AO_CONF_LOG_PERIOD_SEC:

            return

        self._conf_log_t = now

        try:

            try:

                if os.path.getsize(
                    AO_CONF_LOG_PATH
                ) > AO_CONF_LOG_MAX_BYTES:

                    os.replace(
                        AO_CONF_LOG_PATH,
                        AO_CONF_LOG_PATH
                        + ".1",
                    )

            except OSError:

                pass

            record = {
                "t":
                    round(
                        now,
                        3,
                    ),

                "lpL":
                    _safe_float(
                        lane_prob_L
                    ),

                "lpR":
                    _safe_float(
                        lane_prob_R
                    ),

                "jd":
                    _safe_float(
                        junction_dist
                    ),

                "v":
                    round(
                        float(speed_mps),
                        2,
                    )
                    if _safe_float(
                        speed_mps
                    )
                    is not None
                    else
                    None,

                "mode":
                    mode
                    if mode is not None
                    else
                    self._mode,

                "confL":
                    self._lane_conf_ok("left"),

                "confR":
                    self._lane_conf_ok("right"),

                "thr":
                    round(
                        self._lane_conf_threshold(
                            self._lane_prob_min_current
                        ),
                        3,
                    ),

                "wit":
                    round(
                        float(
                            self._lane_conf_witness
                        ),
                        3,
                    ),

                "jban":
                    self._last_junction_ban,

                "jsrc":
                    self._junction_source,

                "jns":
                    self._junction_samples,
            }

            with open(
                AO_CONF_LOG_PATH,
                "a",
                encoding="utf-8",
            ) as fp:

                fp.write(
                    json.dumps(
                        record,
                        ensure_ascii=False,
                    )
                    + "\n"
                )

        except Exception:

            pass