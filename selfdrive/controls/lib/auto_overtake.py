import time
from cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# ---------------- 参数 ----------------
OVERTAKE_MIN_SPEED = 80 * CV.KPH_TO_MS
OVERTAKE_MIN_CRUISE_SPEED = 90 * CV.KPH_TO_MS
OVERTAKE_SPEED_DELTA = 10 * CV.KPH_TO_MS
OVERTAKE_HEADWAY_MAX_S = 3.5
OVERTAKE_LEAD_STABLE_SEC = 1.0

PREPARE_BEFORE_LC_SEC = 0.6
RETURN_CLEAR_DELAY_SEC = 5.0
RETURN_MIN_TIME_AFTER_OUT_SEC = 4.0
OVERTAKE_COOLDOWN_SEC = 20.0
CLEAR_LANE_STABLE_SEC = 0.6

LANE_PREF_AUTO = 0
LANE_PREF_KEEP_LEFT = 1
LANE_PREF_KEEP_RIGHT = 2

REAR_DIST_TH = 30.0
REAR_SPEED_TH = 5.0


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

    def reset(self):
        self.__init__()

    @staticmethod
    def _stable_ok(ok, ok_since, now, stable_sec):
        if ok:
            if ok_since is None:
                ok_since = now
            return ok_since, (now - ok_since) >= stable_sec
        return None, False

    @staticmethod
    def _opposite(direction):
        if direction == LaneChangeDirection.left:
            return LaneChangeDirection.right
        if direction == LaneChangeDirection.right:
            return LaneChangeDirection.left
        return LaneChangeDirection.none

    # ---------------- 评分 ----------------
    def _lane_score(self, *, side, v_ego, v_lead, lead_d,
                    rear_dist, rear_speed, lane_free, bsm, is_highway):

        if not lane_free or bsm:
            return -100.0

        score = 0.0

        if v_lead is not None:
            dv = v_ego - v_lead
            if dv > 0:
                score += min(dv * 2.0, 20.0)

        if rear_dist is not None and rear_speed is not None:
            if rear_dist < 20:
                score -= 30
            if rear_speed > 8:
                score -= 20

        if is_highway:
            score += 5 if side == "left" else -2
        else:
            if side == "right":
                score += 3

        return score

    def _choose_best_lane(self, left_data, right_data):
        left_score = self._lane_score(**left_data)
        right_score = self._lane_score(**right_data)

        if abs(left_score - right_score) < 3:
            return LaneChangeDirection.none

        return LaneChangeDirection.left if left_score > right_score else LaneChangeDirection.right

    # ---------------- 超车需求 ----------------
    def _update_need_overtake(self, now, lead_present, lead_d, v_lead, v_ego, v_cruise):
        if not (lead_present and lead_d > 0):
            self._need_since = None
            return False

        headway = lead_d / max(v_ego, 0.1)
        need_raw = (v_cruise - v_lead) >= OVERTAKE_SPEED_DELTA and headway <= OVERTAKE_HEADWAY_MAX_S

        if need_raw:
            if self._need_since is None:
                self._need_since = now
            return (now - self._need_since) >= OVERTAKE_LEAD_STABLE_SEC

        self._need_since = None
        return False

    # ================= 主逻辑 =================
    def update(self, *, enabled, lc_state, v_ego, v_cruise,
               lead_present, lead_d, v_lead,
               left_ok, right_ok, is_rhd, manual_blinker, bsm_available,
               rear_left_dist=None, rear_left_speed=None,
               rear_right_dist=None, rear_right_speed=None,
               left_lidar_free=None, right_lidar_free=None,
               left_bsm=False, right_bsm=False,
               lane_preference=LANE_PREF_AUTO,
               min_cruise_speed=None):

        now = time.monotonic()
        request = LaneChangeDirection.none

        # 手动优先
        if manual_blinker:
            self.reset()
            return request

        if not enabled or not bsm_available:
            self.reset()
            self._last_lc_state = lc_state
            return request

        min_cruise_speed = OVERTAKE_MIN_CRUISE_SPEED if min_cruise_speed is None else min_cruise_speed

        if v_ego < OVERTAKE_MIN_SPEED or v_cruise < min_cruise_speed:
            self.reset()
            self._last_lc_state = lc_state
            return request

        # 后车阻断
        if rear_left_dist and rear_left_speed:
            if rear_left_dist < REAR_DIST_TH and rear_left_speed > REAR_SPEED_TH:
                left_ok = False

        if rear_right_dist and rear_right_speed:
            if rear_right_dist < REAR_DIST_TH and rear_right_speed > REAR_SPEED_TH:
                right_ok = False

        # 雷达优先
        if left_lidar_free is not None:
            left_ok = left_lidar_free
        if right_lidar_free is not None:
            right_ok = right_lidar_free

        # BSM
        if left_bsm:
            left_ok = False
        if right_bsm:
            right_ok = False

        # 稳定性
        self._left_ok_since, left_stable = self._stable_ok(left_ok, self._left_ok_since, now, CLEAR_LANE_STABLE_SEC)
        self._right_ok_since, right_stable = self._stable_ok(right_ok, self._right_ok_since, now, CLEAR_LANE_STABLE_SEC)

        is_highway = v_ego > 22.0

        best_dir = self._choose_best_lane(
            left_data=dict(side="left", v_ego=v_ego, v_lead=v_lead, lead_d=lead_d,
                           rear_dist=rear_left_dist, rear_speed=rear_left_speed,
                           lane_free=left_ok, bsm=left_bsm, is_highway=is_highway),
            right_data=dict(side="right", v_ego=v_ego, v_lead=v_lead, lead_d=lead_d,
                            rear_dist=rear_right_dist, rear_speed=rear_right_speed,
                            lane_free=right_ok, bsm=right_bsm, is_highway=is_highway)
        )

        pass_ok = (
            (best_dir == LaneChangeDirection.left and left_stable) or
            (best_dir == LaneChangeDirection.right and right_stable)
        )

        need_overtake = self._update_need_overtake(now, lead_present, lead_d, v_lead, v_ego, v_cruise)

        lc_finished = (self._last_lc_state == LaneChangeState.laneChangeFinishing and lc_state == LaneChangeState.off)

        # ================= 状态机 =================
        if self._mode == "idle":
            if need_overtake and pass_ok and now >= self._cooldown_until:
                self._mode = "preparing"
                self._prepare_since = now
                self._out_dir = best_dir  # 锁方向

        elif self._mode == "preparing":
            if not need_overtake:
                self.reset()
            elif now - self._prepare_since >= PREPARE_BEFORE_LC_SEC:
                if pass_ok:
                    self._mode = "changing_out"
                    request = self._out_dir

        elif self._mode == "changing_out":
            if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange) and pass_ok:
                request = self._out_dir
            if lc_finished:
                self._mode = "waiting_return"
                self._out_finished_t = now
                self._clear_since = None

        elif self._mode == "waiting_return":
            # ✅ 是否已完成超车
            passed_lead = (lead_d is None) or (lead_d > 40) or (v_ego > (v_lead or 0) + 5)

            return_dir = self._opposite(self._out_dir)

            return_lane_safe = (
                (return_dir == LaneChangeDirection.left and left_stable) or
                (return_dir == LaneChangeDirection.right and right_stable)
            )

            if not (passed_lead and return_lane_safe):
                self._clear_since = None
            else:
                if self._clear_since is None:
                    self._clear_since = now

                if self._out_finished_t and now - self._out_finished_t < RETURN_MIN_TIME_AFTER_OUT_SEC:
                    pass
                elif (now - self._clear_since) >= RETURN_CLEAR_DELAY_SEC:
                    self._mode = "changing_back"
                    self._return_dir = return_dir
                    request = self._return_dir

        elif self._mode == "changing_back":
            if lc_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
                request = self._return_dir
            if lc_finished:
                self._mode = "idle"
                self._cooldown_until = now + OVERTAKE_COOLDOWN_SEC

        else:
            self.reset()

        self._last_lc_state = lc_state
        return request