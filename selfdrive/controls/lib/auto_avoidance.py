#!/usr/bin/env python3
import time
from cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# ================= 参数 =================
AUTO_AVOID_MIN_SPEED = 30 * CV.KPH_TO_MS
SLOWDOWN_BEFORE_LC_SEC = 1.0
OBSTACLE_CLEAR_DELAY_SEC = 2.0
RETURN_MIN_TIME_AFTER_OUT_SEC = 3.0
AVOID_COOLDOWN_SEC = 8.0
CLEAR_LANE_STABLE_SEC = 1.5

REAR_SPEED_TH = 5.0
REAR_DIST_TH = 30.0

# ================= 刹车模型 =================
def compute_adaptive_brake(v_ego, lead_dist=None, lead_rel_speed=None, max_brake=1.0):
    if lead_dist is None or lead_rel_speed is None:
        return 0.0
    d_safe = max(5.0, v_ego * 1.5)
    if lead_dist > d_safe:
        return 0.0
    distance_factor = max(0.0, 1 - lead_dist / d_safe)
    speed_factor = max(0.0, (-lead_rel_speed) / max(v_ego, 1e-3))
    return min(max_brake, max_brake * (0.6 * distance_factor + 0.4 * speed_factor))

# ================= 平滑轨迹 =================
def generate_smooth_evasive_path(n, offset):
    return [offset * (3*(i/(n-1))**2 - 2*(i/(n-1))**3) for i in range(n)]

# ================= 主类 =================
class AutoAvoidanceHelper:
    def __init__(self):
        self.reset()

    def reset(self):
        self._mode = "idle"
        self._out_dir = LaneChangeDirection.none
        self._return_dir = LaneChangeDirection.none
        self._cooldown_until = 0.0
        self._clear_since = None
        self._slow_since = None
        self._out_finished_t = None
        self._last_lc_state = LaneChangeState.off
        self._left_ok_since = None
        self._right_ok_since = None

    def _stable(self, ok, t, now):
        if ok:
            return t or now, (now - (t or now)) > CLEAR_LANE_STABLE_SEC
        return None, False

    def update(self, *,
               enabled,
               obstacle_in_path,
               lc_state,
               v_ego,
               left_ok=True,
               right_ok=True,
               is_rhd=False,
               manual_blinker=False,
               bsm_available=True,
               lead_dist=None,
               lead_rel_speed=None,
               is_pedestrian=False,
               is_cone=False,
               rear_left_dist=None,
               rear_left_speed=None,
               rear_right_dist=None,
               rear_right_speed=None,
               left_lidar_free=None,
               right_lidar_free=None,
               prefer_dir=LaneChangeDirection.none):

        now = time.monotonic()
        lane_req = LaneChangeDirection.none
        brake = 0.0
        hazard = False
        offset = 0.0

        # ===== 安全入口 =====
        if not enabled or v_ego < AUTO_AVOID_MIN_SPEED:
            self.reset()
            return lane_req, 0.0, False, 0.0

        # ===== 雷达优先 / BSM fallback =====
        if left_lidar_free is not None:
            left_ok = left_ok and left_lidar_free
        if right_lidar_free is not None:
            right_ok = right_ok and right_lidar_free

        # ===== 后车判断 =====
        if rear_left_dist and rear_left_speed:
            if rear_left_dist < REAR_DIST_TH and rear_left_speed > REAR_SPEED_TH:
                left_ok = False
        if rear_right_dist and rear_right_speed:
            if rear_right_dist < REAR_DIST_TH and rear_right_speed > REAR_SPEED_TH:
                right_ok = False

        # ===== 稳定检测 =====
        self._left_ok_since, left_stable = self._stable(left_ok, self._left_ok_since, now)
        self._right_ok_since, right_stable = self._stable(right_ok, self._right_ok_since, now)

        # ===== 紧急判断 =====
        emergency = obstacle_in_path or is_pedestrian or is_cone

        brake = compute_adaptive_brake(v_ego, lead_dist, lead_rel_speed)
        if emergency:
            brake = max(brake, 0.8)

        # ===== 状态机 =====
        if self._mode == "idle":
            if emergency:
                self._mode = "prepare"
                self._slow_since = now

        elif self._mode == "prepare":
            if now - self._slow_since > SLOWDOWN_BEFORE_LC_SEC:
                if left_stable:
                    self._out_dir = LaneChangeDirection.left
                elif right_stable:
                    self._out_dir = LaneChangeDirection.right

                if self._out_dir != LaneChangeDirection.none:
                    self._mode = "changing"
                    lane_req = self._out_dir

        elif self._mode == "changing":
            lane_req = self._out_dir
            if lc_state == LaneChangeState.off:
                self._mode = "done"

        elif self._mode == "done":
            if not emergency:
                self.reset()

        # ===== 平滑偏移 =====
        if emergency:
            offset_target = -3.0 if self._out_dir == LaneChangeDirection.left else 3.0
            path = generate_smooth_evasive_path(20, offset_target)
            offset = path[min(3, len(path)-1)]

        hazard = emergency
        self._last_lc_state = lc_state

        return lane_req, brake, hazard, offset