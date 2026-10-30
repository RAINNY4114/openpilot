#!/usr/bin/env python3
"""
AO/AA 接线层（bridge）— ford-chestnut / staging-chestnut 基线。

把 `auto_overtake.AutoOvertakeHelper` 与 `auto_avoidance.AutoAvoidanceHelper`
接入 sunnypilot 的 `modeld_v2/modeld.py`，而不把 ~900 行接线散进 modeld。

设计原则
--------
1. **modeld 只加 3 行**：构造 / update / 取 direction。其余全在本文件。
2. **不碰横向控制链**：本模块的唯一输出是 `Desire` 的方向与 `AvoidBrakeReq`
   的标量请求。居中逻辑（LatControlAngle + trim）完全不受影响。
3. **全部 fail-safe**：任何异常 -> direction=none、brake=0，并 `reset()` 内部
   状态机。绝不因为 AO/AA 崩掉 modeld（modeld 挂了整条横向链一起没）。
4. **配置走文件不走 Params()**：本 fork 的 `libparams_c.so` 是预编译产物，
   不认识自定义键，`Params().get("AutoOvertake*")` 会抛 `UnknownKeyName`。
   因此与 fork 既有惯例一致，直接读 `/data/ford_params/<key>` 文本文件。

数据源说明（本基线实测）
------------------------
- `radarState.leadOne`  : 前车（Delphi MRR/ESR 经 radard 融合）  -> AO 唯一触发源
- `radarTracks.points`  : 前向雷达点云（card.py 发布）          -> AA 障碍物源
- `carState.left/rightBlindspot` : Ford BSM 0x3A6/0x3A7（HAS_BSM）-> AO/AA 安全门
- `amapNavi.left/rightBlind`     : **位打包**，见下方 `_decode_navi`
- `MR76`                : **本基线未实现**（radar_interface.py 无 MR76 分支）
                          => `require_lane_gain` 必须为 0，否则 AO 永不触发

amapNavi 位定义（carrot/amap_navi.py:250-253，20 Hz 发布）
----------------------------------------------------------
    leftBlind  = 8*(left_lane > 0) + 4*lidar_car_lblind + 2*left_blind + 1*lidar_lblind
    rightBlind = 8*(right_lane > 0) + 4*lidar_car_rblind + 2*right_blind + 1*lidar_rblind

    bit0 = lidar_*blind        LiDAR 判定该侧有障碍
    bit1 = *_blind             导航判定该侧盲区
    bit2 = lidar_car_*blind    LiDAR 判定车侧有障碍
    bit3 = *_lane > 0          该侧存在车道

⚠️ 不能把 `navi.leftBlind` 直接当布尔用：只要**任一位**非零它就 truthy，
   而 bit3 在正常多车道道路上几乎恒为 1 —— 直接把变道锁死。
   旧 `desire_helper` 的 `carstate.leftBlindspot or navi.leftBlind` 就是这个
   行为（属上游 carrot 语义，**本模块不动它**，只在 AO/AA 内按位解码）。
"""

import os
import time
from typing import NamedTuple

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.controls.lib.auto_overtake import AutoOvertakeHelper, LANE_PREF_AUTO
from openpilot.selfdrive.controls.lib.auto_avoidance import AutoAvoidanceHelper

LaneChangeDirection = log.LaneChangeDirection

# ============================================================================
# 配置
# ============================================================================

# [FORK_PARAM_DIR] /data/ford_params 不在 params 目录下，
# Params::clearAll()（params.cc:208-224）永远不会删它。
PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")

AO_LOG_PATH = "/data/media/0/auto_overtake.log"
AA_LOG_PATH = "/data/media/0/auto_avoidance.log"
LOG_MAX_BYTES = 5_000_000

# 自动变道的确认窗口（秒）。这期间转向灯已经亮起（controlsd 从
# laneChangeState 推 CC.left/rightBlinker），驾驶员随时可取消。
CONFIRM_DELAY_SEC_DEFAULT = 3.0

AO_CFG_DEFAULTS = {
  # ---- 总开关。默认关闭，需显式写 /data/ford_params/AutoOvertakeEnabled=1 ----
  "enabled": 0,
  "lane_preference": LANE_PREF_AUTO,
  "confirm_delay_sec": CONFIRM_DELAY_SEC_DEFAULT,
  # 与 AO 内部 OVERTAKE_MIN_SPEED_FLOOR_MS 对齐
  "min_cruise_kph": 60.0,
  # ⚠️ 必须 > 0：设为 0.0 会让 `lp >= lane_prob_min` 恒真，车道线筛选完全失效。
  "lane_prob_min": 0.30,
  # ⚠️⚠️ 本车**没有 MR76**（radar_interface.py 里 grep MR76 零结果，
  #      只有 values.py 的常量）。收益门在 target_present=None 时是
  #      fail-closed（_lane_gain_ok -> "gain_unavailable" -> False），
  #      所以 require_lane_gain=1 会让 AO **永久无法触发**。
  #      默认 0 = 关掉收益门；等 MR76 真正实现后再改 1。
  "require_lane_gain": 0,
}

AA_CFG_DEFAULTS = {
  # ---- 总开关。默认关闭 ----
  "enabled": 0,
  # 逃逸车道稳定性判定：strict / ratio。
  # strict 在车道线抖动时计时器被反复打断 -> left_stable 恒 False -> 永不触发。
  "stable_mode": "ratio",
  "stable_ratio": 0.60,
  "stable_window_sec": 0.60,
  # 障碍物输入短时保持（秒）。radarTracks 只有 ~8 Hz，AA 跑 20 Hz，
  # 不保持的话 _critical_frames 攒不满 CRITICAL_CONFIRM_FRAMES。
  "obs_hold_sec": 0.45,
  # 纵向兜底。brake_max=0 == 完全关闭（与历史行为一致）。
  "brake_gain": 1.0,
  "brake_max": 0.0,
  "brake_ttc_ref": 3.0,
}


def _read_raw(key):
  for d in PARAM_DIRS:
    try:
      with open(os.path.join(d, key)) as f:
        return f.read().strip()
    except Exception:
      continue
  return None


def _read_num(key, default, cast=float):
  raw = _read_raw(key)
  if not raw:
    return default
  try:
    return cast(float(raw))
  except (TypeError, ValueError):
    return default


def write_param(key, value):
  """写 fork 自有参数目录。Params().put 会抛 UnknownKeyName，所以直接写文件。"""
  try:
    os.makedirs("/data/ford_params", exist_ok=True)
  except Exception:
    pass
  for d in ("/data/ford_params", "/dev/shm/params", "/data/params/d"):
    try:
      if not os.path.isdir(d):
        continue
      with open(os.path.join(d, str(key)), "w") as f:
        f.write(str(value))
      return True
    except Exception:
      continue
  return False


def read_cfg():
  ao = dict(AO_CFG_DEFAULTS)
  ao["enabled"] = _read_num("AutoOvertakeEnabled", ao["enabled"], int) != 0
  ao["lane_preference"] = _read_num("AutoOvertakeLanePref", ao["lane_preference"], int)
  ao["confirm_delay_sec"] = _read_num("AutoOvertakeConfirmSec", ao["confirm_delay_sec"])
  ao["min_cruise_kph"] = _read_num("AutoOvertakeMinCruiseKph", ao["min_cruise_kph"])
  ao["lane_prob_min"] = _read_num("AutoOvertakeLaneProbMin", ao["lane_prob_min"])
  ao["require_lane_gain"] = _read_num("AutoOvertakeRequireLaneGain", ao["require_lane_gain"], int) != 0

  aa = dict(AA_CFG_DEFAULTS)
  aa["enabled"] = _read_num("AutoAvoidanceEnabled", aa["enabled"], int) != 0

  try:
    raw = _read_raw("AvoidStableMode")
    if raw:
      m = str(raw).strip().lower()
      if m in ("strict", "ratio"):
        aa["stable_mode"] = m
  except Exception:
    pass

  for key, cfg_key, allow_zero in (("AvoidObsHold", "obs_hold_sec", True),
                                   ("AvoidBrakeMax", "brake_max", True),
                                   ("AvoidBrakeGain", "brake_gain", True),
                                   ("AvoidBrakeTtcRef", "brake_ttc_ref", False)):
    try:
      raw = _read_raw(key)
      if raw:
        v = float(raw)
        if v > 0.0 or (allow_zero and v >= 0.0):
          aa[cfg_key] = v
    except Exception:
      pass

  return ao, aa


# ============================================================================
# 日志
# ============================================================================

class _ThrottledLog:
  def __init__(self, path, period=1.0):
    self.path = path
    self.period = period
    self._t = -1e9

  def write(self, text):
    now = time.monotonic()
    if now - self._t < self.period:
      return
    self._t = now
    try:
      if os.path.exists(self.path) and os.path.getsize(self.path) > LOG_MAX_BYTES:
        os.replace(self.path, self.path + ".1")
      with open(self.path, "a") as f:
        f.write(text + "\n")
    except Exception:
      pass


def _fmt(fields):
  return time.strftime("%H:%M:%S") + " " + " ".join(f"{k}={v}" for k, v in fields.items())


# ============================================================================
# amapNavi 位解码
# ============================================================================

class NaviBits(NamedTuple):
  left_lidar_obstacle: bool
  right_lidar_obstacle: bool
  left_nav_blind: bool
  right_nav_blind: bool
  left_lane: bool
  right_lane: bool
  seen: bool


def decode_navi(sm):
  """按位解码 amapNavi。未收到消息 -> 全部 False 且 seen=False。"""
  try:
    if not (sm.seen['amapNavi'] and sm.valid['amapNavi']):
      return NaviBits(False, False, False, False, False, False, False)
    n = sm['amapNavi']
    lb = int(n.leftBlind)
    rb = int(n.rightBlind)
    return NaviBits(
      left_lidar_obstacle=bool(lb & 0b0101),
      right_lidar_obstacle=bool(rb & 0b0101),
      left_nav_blind=bool(lb & 0b0010),
      right_nav_blind=bool(rb & 0b0010),
      left_lane=bool(lb & 0b1000),
      right_lane=bool(rb & 0b1000),
      seen=True,
    )
  except Exception:
    return NaviBits(False, False, False, False, False, False, False)


class LidarLatch:
  """LiDAR "是否接入" 的会话级锁存。

  amapNavi 只有两位描述 LiDAR（bit0 / bit2），**无法区分**
  "未安装" 与 "已安装且干净"。因此用锁存：
    - 只要本会话内任一侧的 LiDAR 位曾经置位 -> 判定为已接入
    - 从未置位 -> 判定为未接入，AO/AA 的 LiDAR 门传 None（弃权放行）

  这与历史现场行为一致（2026-10-19 实测 llid=rlid=0，两侧均未接入），
  同时保证将来真装了 LiDAR 时无需改代码即可自动启用严格判定。
  """

  def __init__(self):
    self.left = False
    self.right = False

  def update(self, bits):
    if bits.left_lidar_obstacle:
      self.left = True
    if bits.right_lidar_obstacle:
      self.right = True


# ============================================================================
# 输出
# ============================================================================

class BridgeOut(NamedTuple):
  direction: int          # log.LaneChangeDirection
  brake_req: float        # >= 0，AA 请求的减速度大小（m/s^2）
  confirm_delay_sec: float
  ao_state: dict
  aa_state: dict


_EMPTY_AO = {"mode": "-", "last_reason": "-"}
_EMPTY_AA = {"mode": "-", "last_reason": "-", "risk_level": 0}


# ============================================================================
# 主体
# ============================================================================

class AoAaBridge:
  def __init__(self):
    self.ao = AutoOvertakeHelper()
    self.aa = AutoAvoidanceHelper()
    self.lidar = LidarLatch()
    self.ao_log = _ThrottledLog(AO_LOG_PATH)
    self.aa_log = _ThrottledLog(AA_LOG_PATH)
    self._ao_cfg, self._aa_cfg = read_cfg()
    self._cfg_n = 0
    self._brake_pub = -1.0
    self._obs_hold = {"d": None, "y": None, "vr": None, "t": 0.0}
    self._n = 0

  # ---------------------------------------------------------------- 配置热加载
  def _maybe_reload_cfg(self):
    self._cfg_n += 1
    # 20 Hz 下每 2 s 一次
    if (self._cfg_n % 40) == 0:
      try:
        self._ao_cfg, self._aa_cfg = read_cfg()
      except Exception:
        pass

  # ------------------------------------------------------------------- 主入口
  def update(self, *, sm, dh, v_ego, modelv2, lat_active, left_edge, right_edge, is_rhd=False):
    """每帧调用一次。永不抛异常。

    Args:
      sm:         modeld 的 SubMaster（必须含 radarState / radarTracks / amapNavi）
      dh:         DesireHelper 实例（读 lane_change_state）
      v_ego:      m/s
      modelv2:    本帧的 ModelV2 消息体（读 laneLineProbs）
      lat_active: carControl.latActive
      left_edge:  RELC 判定的左侧路沿
      right_edge: RELC 判定的右侧路沿
      is_rhd:     右舵

    Returns:
      BridgeOut
    """
    self._n += 1
    self._maybe_reload_cfg()

    ao_cfg = self._ao_cfg
    aa_cfg = self._aa_cfg

    # ---- 公共输入：车道可用性 ----
    lp_left, lp_right = 1.0, 1.0
    try:
      probs = list(modelv2.laneLineProbs)
      lp_left = float(probs[0]) if len(probs) > 0 else 1.0
      lp_right = float(probs[3]) if len(probs) > 3 else 1.0
    except Exception:
      pass
    left_ok = (not bool(left_edge)) and lp_left >= ao_cfg["lane_prob_min"]
    right_ok = (not bool(right_edge)) and lp_right >= ao_cfg["lane_prob_min"]

    # ---- 公共输入：amapNavi 位解码 + LiDAR 锁存 ----
    bits = decode_navi(sm)
    self.lidar.update(bits)
    # 语义：available=None -> "未接入"，门放行；available=True -> 严格判定
    lidar_avail_l = True if self.lidar.left else None
    lidar_avail_r = True if self.lidar.right else None
    lidar_valid_l = True if self.lidar.left else None
    lidar_valid_r = True if self.lidar.right else None
    lidar_age_l = 0.0 if self.lidar.left else None
    lidar_age_r = 0.0 if self.lidar.right else None
    lidar_obs_l = bits.left_lidar_obstacle if self.lidar.left else None
    lidar_obs_r = bits.right_lidar_obstacle if self.lidar.right else None
    lidar_free_l = (not bits.left_lidar_obstacle) if self.lidar.left else None
    lidar_free_r = (not bits.right_lidar_obstacle) if self.lidar.right else None

    # ---- 公共输入：MR76（本基线未实现，永远 fresh=0）----
    mr76_fresh = _read_num("AOMr76Fresh", 0.0) != 0.0
    mr76_lane = int(_read_num("AOMr76Lane", 0.0))
    mr76_gain_fresh = _read_num("AOMr76GainFresh", 0.0) != 0.0
    mr76_gain_lane = int(_read_num("AOMr76GainLane", 0.0))

    ao_dir = self._update_ao(sm, dh, v_ego, ao_cfg, left_ok, right_ok, is_rhd,
                             lp_left, lp_right, bits,
                             lidar_avail_l, lidar_avail_r, lidar_valid_l, lidar_valid_r,
                             lidar_age_l, lidar_age_r, lidar_obs_l, lidar_obs_r,
                             lidar_free_l, lidar_free_r,
                             mr76_fresh, mr76_lane, mr76_gain_fresh, mr76_gain_lane)
    aa_dir, brake = self._update_aa(sm, dh, v_ego, aa_cfg, left_ok, right_ok, is_rhd, bits,
                                    lidar_avail_l, lidar_avail_r, lidar_valid_l, lidar_valid_r,
                                    lidar_age_l, lidar_age_r, lidar_free_l, lidar_free_r,
                                    mr76_gain_fresh, mr76_gain_lane)

    # ---- 合并：AA（安全兜底）优先于 AO（舒适性）----
    direction = aa_dir if aa_dir != LaneChangeDirection.none else ao_dir

    return BridgeOut(
      direction=direction,
      brake_req=brake,
      confirm_delay_sec=float(ao_cfg["confirm_delay_sec"]),
      ao_state=self.ao.get_state(),
      aa_state=self.aa.get_state(),
    )

  # ---------------------------------------------------------------------- AO
  def _update_ao(self, sm, dh, v_ego, cfg, left_ok, right_ok, is_rhd,
                 lp_left, lp_right, bits,
                 l_avail, r_avail, l_valid, r_valid, l_age, r_age, l_obs, r_obs,
                 l_free, r_free,
                 mr76_fresh, mr76_lane, mr76_gain_fresh, mr76_gain_lane):
    fields = {"en": int(bool(cfg["enabled"]))}
    if not cfg["enabled"]:
      self.ao.reset()
      fields["why"] = "disabled"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

    if not sm.seen.get("radarState", False):
      self.ao.reset()
      fields["why"] = "no_radarstate"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

    try:
      cs = sm['carState']
      lead = sm['radarState'].leadOne
      v_cruise = float(cs.cruiseState.speed) if cs.cruiseState.enabled else 0.0
      # vRel 是本车最可靠的字段；vLead/vLeadK 噪声大
      v_lead = max(0.0, float(v_ego) + float(lead.vRel))

      # Ford BSM 后向接近区（carstate.py 曾写 AOBsmZone / AOBsmFault，
      # 本基线写入侧已丢失 -> 读到 None -> 门放行，属可接受的降级）
      lz, rz, rf = None, None, None
      try:
        packed = _read_raw("AOBsmZone")
        if packed:
          pv = int(float(packed))
          lz, rz = pv // 100, pv % 100
        f = _read_raw("AOBsmFault")
        if f:
          fv = int(float(f))
          rf = bool(fv % 10) or bool(fv // 10)
      except Exception:
        lz, rz, rf = None, None, None

      mr76_l_occ = bool((mr76_lane // 10) % 10) if mr76_fresh else None
      mr76_r_occ = bool(mr76_lane % 10) if mr76_fresh else None
      tgt_l = bool((mr76_gain_lane // 10) % 10) if mr76_gain_fresh else None
      tgt_r = bool(mr76_gain_lane % 10) if mr76_gain_fresh else None

      ao_dir = self.ao.update(
        enabled=True,
        lc_state=dh.lane_change_state,
        v_ego=v_ego,
        v_cruise=v_cruise,
        lead_present=bool(lead.present),
        lead_d=float(lead.dRel),
        v_lead=v_lead,
        left_ok=left_ok,
        right_ok=right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(cs.leftBlinker or cs.rightBlinker),
        bsm_available=True,
        left_bsm=bool(cs.leftBlindspot),
        right_bsm=bool(cs.rightBlindspot),
        left_rear_zone=lz,
        right_rear_zone=rz,
        rear_sensor_fault=rf,
        lane_preference=int(cfg["lane_preference"]),
        min_cruise_speed=float(cfg["min_cruise_kph"]) * CV.KPH_TO_MS,
        left_lane_prob=lp_left,
        right_lane_prob=lp_right,
        lane_prob_min=float(cfg["lane_prob_min"]),
        junction_dist=None,
        # MR76 相邻车道占用（fresh=0 -> None -> 门弃权）
        mr76_left_available=(True if mr76_fresh else None),
        mr76_right_available=(True if mr76_fresh else None),
        mr76_left_valid=(True if mr76_fresh else None),
        mr76_right_valid=(True if mr76_fresh else None),
        mr76_left_age=(0.0 if mr76_fresh else None),
        mr76_right_age=(0.0 if mr76_fresh else None),
        mr76_left_obstacle=mr76_l_occ,
        mr76_right_obstacle=mr76_r_occ,
        # 收益门（fail-closed；MR76 缺失时 require_lane_gain 必须为 0）
        left_target_lead_present=tgt_l,
        right_target_lead_present=tgt_r,
        require_lane_gain=bool(cfg["require_lane_gain"]),
        # LiDAR（amapNavi 位解码；未接入 -> None -> 门弃权）
        left_lidar_available=l_avail,
        right_lidar_available=r_avail,
        left_lidar_valid=l_valid,
        right_lidar_valid=r_valid,
        left_lidar_age=l_age,
        right_lidar_age=r_age,
        left_lidar_obstacle=l_obs,
        right_lidar_obstacle=r_obs,
        left_lidar_free=l_free,
        right_lidar_free=r_free,
        require_aux_sensors=True,
      )

      st = self.ao.get_state()
      fields.update({
        "v": round(v_ego * 3.6, 1), "vc": round(v_cruise * 3.6, 1),
        "lp": int(bool(lead.present)), "d": round(float(lead.dRel), 1), "vl": round(v_lead, 1),
        "lok": int(left_ok), "rok": int(right_ok),
        "lbsm": int(bool(cs.leftBlindspot)), "rbsm": int(bool(cs.rightBlindspot)),
        "lp0": round(lp_left, 2), "lp3": round(lp_right, 2),
        "ao": int(ao_dir), "mode": st["mode"], "why": st["last_reason"],
        "dh": int(dh.lane_change_state), "dhd": int(dh.lane_change_direction),
        "cf": int(bool(st.get("lane_conf_L"))) * 2 + int(bool(st.get("lane_conf_R"))),
        "jb": int(bool(st.get("junction_ban"))),
        "llid": int(bool(self.lidar.left)) * 2 + int(bits.left_lidar_obstacle),
        "rlid": int(bool(self.lidar.right)) * 2 + int(bits.right_lidar_obstacle),
        "gf": int(bool(cfg["require_lane_gain"])),
        "gL": int(bool(st.get("gain_ok_L"))), "gR": int(bool(st.get("gain_ok_R"))),
        "gr": str(st.get("last_gain_reason") or "-"),
        "mrf": int(mr76_fresh), "mrl": int(mr76_lane),
        "mgf": int(mr76_gain_fresh), "mgl": int(mr76_gain_lane),
        "nl": int(bits.left_lane) * 2 + int(bits.right_lane),
      })
      self.ao_log.write(_fmt(fields))
      return ao_dir

    except Exception:
      cloudlog.exception("auto overtake update failed")
      self.ao.reset()
      fields["why"] = "exception"
      self.ao_log.write(_fmt(fields))
      return LaneChangeDirection.none

  # ---------------------------------------------------------------------- AA
  def _update_aa(self, sm, dh, v_ego, cfg, left_ok, right_ok, is_rhd, bits,
                 l_avail, r_avail, l_valid, r_valid, l_age, r_age, l_free, r_free,
                 mr76_gain_fresh, mr76_gain_lane):
    fields = {"en": int(bool(cfg["enabled"]))}
    if not cfg["enabled"]:
      self.aa.reset()
      self._clear_obs()
      fields["why"] = "disabled"
      self.aa_log.write(_fmt(fields))
      return LaneChangeDirection.none, 0.0

    try:
      cs = sm['carState']
      lead = sm['radarState'].leadOne if sm.seen.get("radarState", False) else None

      # ---- 障碍物：radarTracks 前向点云，取本车道内最近者 ----
      obs_d, obs_vr, obs_in_path = None, None, False
      best = None
      if sm.seen.get("radarTracks", False):
        for p in sm['radarTracks'].points:
          d = float(p.dRel)
          y = float(p.yRel)
          vr = float(p.vRel)
          if d < 1.0 or d > 60.0:
            continue
          # 横向筛选 1.0 m（不是 1.8 m）：半个车道宽 ≈1.8 m，
          # 用 1.8 会把相邻车道的车当成本车道障碍物 -> 误触发变道。
          if abs(y) > 1.0:
            continue
          if best is None or d < best[0]:
            best = (d, y, vr)

      now = time.monotonic()
      hold_s = float(cfg.get("obs_hold_sec", 0.0))
      if best is not None:
        self._obs_hold["d"] = best[0]
        self._obs_hold["y"] = best[1]
        self._obs_hold["vr"] = best[2]
        self._obs_hold["t"] = now
        obs_d, obs_vr, obs_in_path = best[0], best[2], True
      elif hold_s > 0.0 and self._obs_hold["d"] is not None and (now - self._obs_hold["t"]) <= hold_s:
        obs_d = self._obs_hold["d"]
        obs_vr = self._obs_hold["vr"]
        obs_in_path = True
      else:
        self._clear_obs()

      aa_mr76_l = bool((mr76_gain_lane // 10) % 10) if mr76_gain_fresh else None
      aa_mr76_r = bool(mr76_gain_lane % 10) if mr76_gain_fresh else None

      aa_dir, brake, _hazard, _off = self.aa.update(
        enabled=True,
        obstacle_in_path=obs_in_path,
        lc_state=dh.lane_change_state,
        v_ego=v_ego,
        left_ok=left_ok,
        right_ok=right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(cs.leftBlinker or cs.rightBlinker),
        bsm_available=True,
        left_bsm_blocked=bool(cs.leftBlindspot),
        right_bsm_blocked=bool(cs.rightBlindspot),
        obstacle_dist=obs_d,
        obstacle_rel_speed=obs_vr,
        lead_dist=float(lead.dRel) if lead is not None else None,
        lead_rel_speed=float(lead.vRel) if lead is not None else None,
        # AA 的逃逸车道门刻意不传 require_aux_sensors_lidar：
        # AA 是 >80 km/h 的紧急兜底，若因 WiFi LiDAR 瞬时掉线就
        # fail-closed，会在最需要它的时候把避让整体锁死。
        left_lidar_free=l_free,
        right_lidar_free=r_free,
        left_lidar_available=l_avail,
        right_lidar_available=r_avail,
        left_lidar_valid=l_valid,
        right_lidar_valid=r_valid,
        left_lidar_age=l_age,
        right_lidar_age=r_age,
        require_aux_sensors=False,
        mr76_left_obstacle=aa_mr76_l,
        mr76_right_obstacle=aa_mr76_r,
        avoid_stable_mode=str(cfg["stable_mode"]),
        avoid_stable_ratio=float(cfg["stable_ratio"]),
        avoid_stable_window_sec=float(cfg["stable_window_sec"]),
        avoid_brake_gain=float(cfg["brake_gain"]),
        avoid_brake_max=float(cfg["brake_max"]),
        avoid_brake_ttc_ref=float(cfg["brake_ttc_ref"]),
      )

      st = self.aa.get_state()
      fields.update({
        "dir": int(aa_dir), "mode": st["mode"], "why": st["last_reason"],
        "risk": int(st["risk_level"]),
        "od": round(obs_d, 1) if obs_d is not None else -1,
        "ovr": round(obs_vr, 1) if obs_vr is not None else 0,
        "v": round(v_ego * 3.6, 1),
        "sm": str(cfg["stable_mode"]),
        "oh": round(float(cfg.get("obs_hold_sec", 0.0)), 2),
        "brk": round(float(brake), 3),
        "bmax": round(float(cfg.get("brake_max", 0.0)), 2),
        "llid": int(bool(self.lidar.left)) * 2 + int(bits.left_lidar_obstacle),
        "rlid": int(bool(self.lidar.right)) * 2 + int(bits.right_lidar_obstacle),
        "amf": int(mr76_gain_fresh),
        "aml": int(bool(aa_mr76_l)), "amr": int(bool(aa_mr76_r)),
      })
      self.aa_log.write(_fmt(fields))

      brake = 0.0 if brake != brake else max(0.0, float(brake))
      self._publish_brake(brake)
      return aa_dir, brake

    except Exception:
      cloudlog.exception("auto avoidance update failed")
      self.aa.reset()
      self._clear_obs()
      fields["why"] = "exception"
      self.aa_log.write(_fmt(fields))
      self._publish_brake(0.0)
      return LaneChangeDirection.none, 0.0

  def _clear_obs(self):
    self._obs_hold["d"] = None
    self._obs_hold["y"] = None
    self._obs_hold["vr"] = None

  # ---------------------------------------------------------- 刹车请求心跳写盘
  def _publish_brake(self, brake):
    """镜像到 /data/ford_params/AvoidBrakeReq，格式 "<值>:<monotonic秒>"。

    ★ 心跳语义：只要请求 > 0，**每帧都重写**（靠内嵌时间戳推进）。
      消费侧（controlsd 的 AABrakeApply）用时间戳是否推进判断写入侧是否
      还活着，从而区分【持续请求】与【写入侧已死】。若只在值变化时写，
      恒定请求会被 staleness 误杀 —— 这是必须避免的安全缺陷。
    """
    try:
      changed = abs(brake - self._brake_pub) >= 0.01
      if changed or brake > 0.0:
        self._brake_pub = brake
        write_param("AvoidBrakeReq", f"{brake:.3f}:{time.monotonic():.3f}")
    except Exception:
      pass
