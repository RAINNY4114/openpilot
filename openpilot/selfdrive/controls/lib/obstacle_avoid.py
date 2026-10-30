#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Avoidance Planner  —— 车道内障碍物横向偏移绕行
=========================================================

移植来源: gitee.com/xiansfyt/cpv9-corolla (CarrotPilot, toyota-19-corolla 分支)
  - selfdrive/controls/lib/lateral_planner.py::_calc_obstacle_avoid_param
  - selfdrive/controls/lib/lane_planner_2.py::update_obstacle_data / get_d_path 障碍段
  - selfdrive/controls/plannerd.py::_build_obstacles
  - selfdrive/controls/radard.py 侧向目标过滤

与设备现有模块的分层关系（互不替代）:
  L1 本模块        : 常规障碍物 -> 车道内小幅横向偏移绕行（不换道）
  L2 longitudinal  : 静态障碍/路边停车 -> 巡航限速下降
  L3 auto_avoidance: 紧急转向逃逸（躲不开时）
  L4 brake_request : 无可行间隙时的刹车请求

坐标系约定（openpilot 标准）:
  y 正 = 左, y 负 = 右; 障碍在右 -> 向左让(offset 正); 障碍在左 -> 向右让(offset 负)

安全设计:
  - 默认关闭 (ObstacleAvoidEnabled=0)
  - 默认 observe_only (ObstacleAvoidActive=0): 只计算 + 记日志, 不修改路径
  - 偏移上限默认 0.90 m, 单帧变化 <= 0.02 m (20Hz -> 0.4 m/s)
  - 车道边界硬约束, 不越线/不压路沿
  - 正前方同车道障碍不做横向偏移 (交给纵向), 避免与紧急转向抢方向
  - 任何异常 -> 返回 None, 行为与原生完全一致

配置: /data/ford_params/<key> 纯文本 (与 ao_aa_bridge 同约定, 不新增 Params 键)
  ObstacleAvoidEnabled   0/1     总开关            默认 0
  ObstacleAvoidActive    0/1     0=observe 1=生效  默认 0
  ObstacleAvoidMaxShift  float   最大横向偏移(m)   默认 0.90
  ObstacleAvoidMinKph    float   最低生效车速      默认 8
  ObstacleAvoidMaxKph    float   最高生效车速      默认 110
  ObstacleAvoidVruExtra  float   行人/两轮额外净空  默认 0.50
"""

import json
import os
import time

import numpy as np

# ============================ 可调常量 ============================
HALF_CAR_WIDTH = 0.90          # 车半宽 (m)
MIN_SAFETY_MARGIN = 0.25       # 远处最小净空 (m)
MAX_SAFETY_MARGIN = 0.60       # 近处最大净空 (m)
CLOSE_DIST = 10.0              # 净空插值: 近端 (m)
FAR_DIST = 30.0                # 净空插值: 远端 (m)
VRU_EXTRA_MARGIN = 0.50        # 行人/两轮额外净空 (m)

OBS_MAX_LONG = 40.0            # 障碍物最远生效距离 (m)
OBS_MIN_LONG = 0.5             # 最近生效距离 (m)
OBS_MAX_LAT = 3.2              # 最远横向距离 (m)

# 各类障碍半宽 (m) —— 与 Gitee OBS_HALF_WIDTH 一致
OBS_HALF_WIDTH = {
  "pedestrian": 0.25,
  "motorcycle": 0.35,
  "bicycle": 0.30,
  "parked": 0.90,
  "static": 0.60,
  "oncoming": 0.90,
  "unknown": 0.60,
}

# 同车道判定: |y| 小于此值视为"正前方", 不做横向偏移(交给纵向)
IN_PATH_HALF_WIDTH = 0.45
# 太近的障碍横向已来不及, 交给纵向/紧急层, 不做横向偏移
MIN_AVOID_DIST = 5.0

PROFILE_SPAN_IDX = 4           # 障碍物影响窗口(索引半径)
SMOOTH_ALPHA = 0.12            # 廓线一阶滤波
RATE_LIMIT_PER_FRAME = 0.02    # 单帧最大偏移变化 (m)
EDGE_SAFETY = 0.30             # 距车道边界安全余量 (m)

PARAM_DIRS = ("/data/ford_params", "/dev/shm/params", "/data/params/d")
LOG_PATH = "/data/media/0/obstacle_avoid.log"
CFG_REFRESH_SEC = 1.0

# ===== 辅助传感器接入 =====
# MR76 / 外接雷达统一走"文件桥": 任何外部进程(串口网关/ESP32/手机App/UDP 转发)
# 只需按下面的格式写 /dev/shm 下的 json, 本模块每帧读取。解耦协议, 不绑死硬件。
#
#   {"t": <epoch秒>, "targets": [[d, y, v_rel, "cls"], ...], "fresh": bool}
#
#   d     纵向距离 m (车头起算, 正=前方)
#   y     横向距离 m (openpilot 约定: 正=左, 负=右)
#   v_rel 相对速度 m/s (负=接近)
#   cls   可选: pedestrian/motorcycle/bicycle/static/parked/oncoming/unknown
#         省略或未知 -> "unknown"
#   fresh 可选: 写入方自报"这一路数据当前可信"。false -> 整源忽略。
#         缺省视为 true(兼容没有存活信息的外部写入方)。
#
# fresh 必须实现的原因: 如果雷达掉线/对象流停了, 写入方仍会不断刷新 t,
# 下游就会看到"文件新鲜 + 目标为空" = "车道空", 恰恰是最危险的失效方向。
# 参考实现 E:\openpilot\chestnut 2026-09-20 里把这一点单独标了 [FIX]。
AUX_PATH = "/dev/shm/mr76_targets.json"
AUX_STALE_SEC = 0.5            # 超过此时间视为失效, 不参与避让
AUX_MAX_TARGETS = 32           # 防御: 单帧最多采纳的目标数

# 两侧激光雷达(amapNavi bit0/bit2) -> 侧向"虚拟墙"
# 只有布尔量, 没有距离: 该侧被判占用时, 禁止向该侧偏移, 但不主动推开
LIDAR_WALL_Y = 1.75            # 虚拟墙横向位置 (m)
LIDAR_WALL_GAP = 0.50          # 虚拟墙所需净空 (m)


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


def _log(msg):
  """节流由调用方控制; 写盘失败静默忽略。"""
  try:
    with open(LOG_PATH, "a") as f:
      f.write(time.strftime("%m-%d %H:%M:%S ") + msg + "\n")
  except Exception:
    pass


def _aux_fresh(data):
  """解析文件桥的 fresh 字段。

  缺字段 -> True(兼容没有存活信息的外部写入方)。
  显式假值 -> False。**必须同时认 `false` / `0` / `"0"` / `"false"`** ——
  外部写入方很可能写 0/1 而不是 JSON 布尔, 只判断 `is False` 会静默放行,
  等于这条安全门对它们完全无效。
  """
  if "fresh" not in data:
    return True
  v = data["fresh"]
  if v is None:
    return True
  if isinstance(v, str):
    return v.strip().lower() not in ("0", "false", "no", "off", "")
  try:
    return bool(v)
  except Exception:
    return True


class ObstacleAvoidPlanner:
  def __init__(self):
    self._cfg = {}
    self._cfg_ts = 0.0
    self._log_ts = 0.0
    self._profile = None       # 平滑后的廓线 (N,)
    self._n = 0
    # LiDAR 会话级锁存: 只有本会话内曾置位过, 才认为该侧 LiDAR 真的接入。
    # 与 ao_aa_bridge.LidarLatch 同语义 —— amapNavi 无法区分"未安装"与"已装且干净",
    # 若一上来就按 bit 位约束, 未装车的用户会被永久锁死在一侧。
    self._lidar_seen = {"left": False, "right": False}
    self._aux_ok_ts = 0.0
    self._aux_n = 0
    self._aux_dead = False     # 辅助雷达自报不可信(掉线/对象流停)
    self.diag = {
      "enabled": False, "active": False, "n_obs": 0,
      "closest": float("inf"), "shift": 0.0,
      "conflict": False, "why": "init",
      "aux": 0, "aux_dead": False, "lidar": "-",
    }

  # ------------------------------------------------------------------
  # 配置
  # ------------------------------------------------------------------
  def _load_cfg(self):
    now = time.monotonic()
    if now - self._cfg_ts < CFG_REFRESH_SEC and self._cfg:
      return self._cfg
    self._cfg_ts = now
    self._cfg = {
      "enabled": _read_num("ObstacleAvoidEnabled", 0, int) != 0,
      "active": _read_num("ObstacleAvoidActive", 0, int) != 0,
      "max_shift": max(0.0, min(_read_num("ObstacleAvoidMaxShift", 0.90), 1.8)),
      "min_kph": _read_num("ObstacleAvoidMinKph", 8.0),
      "max_kph": _read_num("ObstacleAvoidMaxKph", 110.0),
      "vru_extra": max(0.0, min(_read_num("ObstacleAvoidVruExtra", VRU_EXTRA_MARGIN), 1.0)),
      "max_long": max(10.0, min(_read_num("ObstacleAvoidMaxLong", OBS_MAX_LONG), 60.0)),
    }
    return self._cfg

  # ------------------------------------------------------------------
  # 障碍物聚合
  # ------------------------------------------------------------------
  @staticmethod
  def _cluster(points):
    """把雷达点云聚成目标, 返回 [(d, y, v_rel), ...]。

    福特 Delphi MRR 单目标常出多点, 不聚类会让同一障碍重复计入廓线。
    点数很小(<64), O(n^2) 贪心足够。
    """
    clusters = []  # [d, y, vr, n]
    for d, y, vr in points:
      hit = None
      for c in clusters:
        if abs(c[0] - d) < 2.0 and abs(c[1] - y) < 1.0:
          hit = c
          break
      if hit is None:
        clusters.append([d, y, vr, 1])
      else:
        n = hit[3]
        hit[0] = (hit[0] * n + d) / (n + 1)
        hit[1] = (hit[1] * n + y) / (n + 1)
        hit[2] = (hit[2] * n + vr) / (n + 1)
        hit[3] = n + 1
    return [(c[0], c[1], c[2]) for c in clusters]

  @staticmethod
  def _classify(d, y, vr):
    """无视觉标签时的雷达启发式分类。"""
    if vr < -3.0:
      return "oncoming"                       # 迎面来车
    if abs(vr) < 1.0:
      # 静止: 贴边 -> 路边停放; 车道内 -> 静止障碍
      return "parked" if abs(y) > 1.0 else "static"
    if vr < 3.0 and abs(y) < 1.6:
      # 低速同向且靠近本车道: 保守按两轮/行人处理, 给额外净空
      return "motorcycle"
    return "unknown"

  def _collect(self, sm, max_long):
    obs = []
    # 主源: radarTracks 点云 (福特 MRR)
    try:
      if sm.seen.get("radarTracks", False):
        pts = []
        for p in sm["radarTracks"].points:
          d = float(p.dRel)
          y = float(p.yRel)
          vr = float(p.vRel)
          if OBS_MIN_LONG < d < max_long and abs(y) < OBS_MAX_LAT:
            pts.append((d, y, vr))
        obs = [(d, y, vr, None) for (d, y, vr) in self._cluster(pts)]
    except Exception:
      obs = []

    # 补充源: radarState 两个 lead (点云偶发丢目标时兜底)
    try:
      if sm.seen.get("radarState", False):
        for ld in (sm["radarState"].leadOne, sm["radarState"].leadTwo):
          if not ld.status:
            continue
          d = float(ld.dRel)
          y = float(ld.yRel)
          vr = float(ld.vRel)
          if not (OBS_MIN_LONG < d < max_long and abs(y) < OBS_MAX_LAT):
            continue
          # 与点云目标去重
          if any(abs(d - o[0]) < 2.0 and abs(y - o[1]) < 1.0 for o in obs):
            continue
          obs.append((d, y, vr, None))
    except Exception:
      pass
    return obs

  # ------------------------------------------------------------------
  # 辅助传感器: MR76 / 外接雷达 (文件桥)
  # ------------------------------------------------------------------
  def _read_aux(self):
    """读 /dev/shm/mr76_targets.json。外部进程写入, 本模块只读。

    返回 [(d, y, vr, cls), ...]; 文件不存在/过期/格式错误/自报不可信 -> []。
    """
    self._aux_n = 0
    self._aux_dead = False
    try:
      with open(AUX_PATH) as f:
        raw = f.read()
      if not raw:
        return []
      data = json.loads(raw)
      # 时间戳必须是近期的, 否则说明喂数据的进程已经挂了
      ts = float(data.get("t", 0.0))
      if ts <= 0.0 or (time.time() - ts) > AUX_STALE_SEC:
        return []
      # 写入方自报"这一路不可信"(雷达掉线 / 对象流停了) -> 整源忽略。
      # 绝不能当成"车道空": 那正是最危险的失效方向。
      if not _aux_fresh(data):
        self._aux_dead = True
        return []
      out = []
      for item in data.get("targets", [])[:AUX_MAX_TARGETS]:
        try:
          d = float(item[0])
          y = float(item[1])
          vr = float(item[2]) if len(item) > 2 else 0.0
        except (TypeError, ValueError, IndexError):
          continue
        if not (OBS_MIN_LONG < d < OBS_MAX_LONG and abs(y) < OBS_MAX_LAT):
          continue
        cls = None
        if len(item) > 3 and isinstance(item[3], str) and item[3] in OBS_HALF_WIDTH:
          cls = item[3]
        out.append((d, y, vr, cls))
      self._aux_n = len(out)
      if out:
        self._aux_ok_ts = time.time()
      return out
    except Exception:
      return []

  # ------------------------------------------------------------------
  # 辅助传感器: 两侧激光雷达 (amapNavi bit0/bit2) -> 侧向虚拟墙
  # ------------------------------------------------------------------
  def _lidar_walls(self, sm):
    """该侧被判占用时, 在那一侧立一道虚拟墙, 禁止向该侧偏移。

    只有布尔量, 没有距离/角度, 所以只做"禁止往那边让", 不主动推开。
    """
    walls = []
    try:
      if not (sm.seen.get("amapNavi", False) and sm.valid.get("amapNavi", False)):
        return walls
      n = sm["amapNavi"]
      lb = int(n.leftBlind)
      rb = int(n.rightBlind)
      # bit0 = LiDAR 该侧有障碍; bit2 = LiDAR 判定车侧有障碍
      if lb & 0b0101:
        self._lidar_seen["left"] = True
      if rb & 0b0101:
        self._lidar_seen["right"] = True
      # 只有该侧曾经置位过(说明真的装了)才施加约束
      if self._lidar_seen["left"] and (lb & 0b0101):
        walls.append((LIDAR_WALL_Y, LIDAR_WALL_GAP))
      if self._lidar_seen["right"] and (rb & 0b0101):
        walls.append((-LIDAR_WALL_Y, LIDAR_WALL_GAP))
    except Exception:
      pass
    return walls

  # ------------------------------------------------------------------
  # 廓线计算 (移植 Gitee _calc_obstacle_avoid_param, 坐标系翻转到 y 正=左)
  # ------------------------------------------------------------------
  def _build_profile(self, obs, t_idxs, v_ego, max_shift, vru_extra, walls=()):
    n = len(t_idxs)
    profile = np.zeros(n, dtype=np.float64)
    if not obs and not walls:
      return profile, float("inf"), False

    envelope = []   # (y, gap) 用于两侧冲突校验
    # 侧向虚拟墙(激光雷达占用): 只约束候选位置, 不产生推力
    envelope.extend(walls)
    min_dist = float("inf")
    for d, y, vr, cls_hint in obs:
      min_dist = min(min_dist, d)
      cls = cls_hint or self._classify(d, y, vr)

      # 净空插值: 越近要求越大
      if d < CLOSE_DIST:
        margin = MAX_SAFETY_MARGIN
      else:
        t = np.clip((d - CLOSE_DIST) / (FAR_DIST - CLOSE_DIST), 0.0, 1.0)
        margin = MAX_SAFETY_MARGIN + (MIN_SAFETY_MARGIN - MAX_SAFETY_MARGIN) * t

      is_vru = cls in ("pedestrian", "bicycle", "motorcycle")
      gap = OBS_HALF_WIDTH.get(cls, 0.60) + HALF_CAR_WIDTH + margin + (vru_extra if is_vru else 0.0)

      # 正前方同车道: 不做横向偏移 (交给纵向/紧急层), 也不参与两侧冲突校验,
      # 否则会被判成"无可行间隙"而把车硬推到某一侧。
      if abs(y) < IN_PATH_HALF_WIDTH:
        continue
      # 太近: 横向机动来不及, 交给纵向
      if d < MIN_AVOID_DIST:
        continue

      envelope.append((y, gap))

      # 所需偏移 = 净空需求 - 已有横向间距, 下限 0
      req = max(0.0, gap - abs(y))
      if req <= 0.0:
        continue
      # y 正(左) -> 向右让(负); y 负(右) -> 向左让(正)
      push = -np.sign(y) * req

      # 映射到时间索引
      t_obs = d / max(v_ego, 2.0)
      idx = int(np.clip(np.interp(t_obs, t_idxs, np.arange(n)), 0, n - 1))
      i0 = max(0, idx - PROFILE_SPAN_IDX)
      i1 = min(n, idx + PROFILE_SPAN_IDX + 1)
      for i in range(i0, i1):
        w = 1.0 - abs(i - idx) / (PROFILE_SPAN_IDX + 1.0)
        if push < 0:
          profile[i] = min(profile[i], push * w)
        else:
          profile[i] = max(profile[i], push * w)

    # 两侧冲突校验: 每个采样点取"到所有附近障碍最小净空最大"的位置
    conflict = False
    if envelope:
      cand = np.linspace(-1.8, 1.8, 73)
      for i in range(n):
        near = envelope  # 点云目标少, 全量参与即可(与 Gitee 的索引邻近筛选等效)
        raw = float(profile[i])
        best_y, best_score, max_mc = raw, -1e9, -1e9
        for y in cand:
          mc = min(abs(o_y - y) - g for (o_y, g) in near)
          max_mc = max(max_mc, mc)
          score = -abs(y - raw) if mc >= 0.0 else mc - 0.01 * abs(y - raw)
          if score > best_score:
            best_score, best_y = score, y
        profile[i] = best_y
        if max_mc < 0.0:
          conflict = True

    profile = np.clip(profile, -max_shift, max_shift)
    return profile, min_dist, conflict

  # ------------------------------------------------------------------
  # 主入口
  # ------------------------------------------------------------------
  def update(self, sm, plan, t_idxs, v_ego, lat_active):
    """返回需要叠加到路径 y 上的偏移数组 (N,) 或 None。

    plan: model_output['plan'][0], shape (N, 6+), 索引 1 为 y
    """
    try:
      cfg = self._load_cfg()
      self.diag["enabled"] = cfg["enabled"]
      self.diag["active"] = cfg["active"]
      if not cfg["enabled"]:
        self.diag["why"] = "disabled"
        self._profile = None
        return None

      v_kph = v_ego * 3.6
      if not lat_active:
        self.diag["why"] = "lat_inactive"
        self._decay()
        return self._emit(cfg)
      if v_kph < cfg["min_kph"]:
        self.diag["why"] = "slow"
        self._decay()
        return self._emit(cfg)
      if v_kph > cfg["max_kph"]:
        self.diag["why"] = "fast"
        self._decay()
        return self._emit(cfg)

      n = len(t_idxs)
      if self._profile is None or self._n != n:
        self._profile = np.zeros(n, dtype=np.float64)
        self._n = n

      obs = self._collect(sm, cfg["max_long"])
      # 辅助传感器: MR76/外接雷达(文件桥) + 两侧激光雷达(虚拟墙)
      aux = self._read_aux()
      for a in aux:
        # 与车载雷达去重: 同一目标不重复计入
        if any(abs(a[0] - o[0]) < 2.0 and abs(a[1] - o[1]) < 1.0 for o in obs):
          continue
        obs.append(a)
      walls = self._lidar_walls(sm)

      self.diag["n_obs"] = len(obs)
      self.diag["aux"] = self._aux_n
      self.diag["aux_dead"] = self._aux_dead
      self.diag["lidar"] = ("L" if self._lidar_seen["left"] else "-") + \
                           ("R" if self._lidar_seen["right"] else "-")
      target, min_dist, conflict = self._build_profile(
        obs, t_idxs, v_ego, cfg["max_shift"], cfg["vru_extra"], walls)
      self.diag["closest"] = min_dist
      self.diag["conflict"] = conflict

      # 一阶滤波 + 单帧变化限幅
      smooth = SMOOTH_ALPHA * target + (1.0 - SMOOTH_ALPHA) * self._profile
      delta = np.clip(smooth - self._profile, -RATE_LIMIT_PER_FRAME, RATE_LIMIT_PER_FRAME)
      self._profile = np.clip(self._profile + delta, -cfg["max_shift"], cfg["max_shift"])

      # 车道边界硬约束: 偏移后近端 y 不得越界
      self._profile = self._clamp_to_lane(sm, plan, self._profile)

      self.diag["shift"] = float(self._profile[0])
      self.diag["why"] = "run"
      return self._emit(cfg)
    except Exception as e:
      self.diag["why"] = f"err:{e}"
      return None

  def _decay(self):
    """条件不满足时把廓线平滑收回 0, 避免突然消失导致方向跳变。"""
    if self._profile is None:
      return
    self._profile = self._profile * 0.85
    if np.max(np.abs(self._profile)) < 1e-3:
      self._profile = None

  def _clamp_to_lane(self, sm, plan, profile):
    """用车道线/路沿把偏移后的近端位置夹在车道内。"""
    try:
      md = sm["modelV2"]
      y0 = float(plan[0, 1])
      left_bound = None
      right_bound = None
      if len(md.laneLines) >= 4:
        ly = float(np.array(md.laneLines[1].y)[0])
        ry = float(np.array(md.laneLines[2].y)[0])
        # 车道线可信度不足时不作为硬约束
        if md.laneLineProbs[1] > 0.5:
          left_bound = ly + EDGE_SAFETY
        if md.laneLineProbs[2] > 0.5:
          right_bound = ry - EDGE_SAFETY
      if left_bound is None and len(md.roadEdges) >= 2 and md.roadEdgeProbs[0] > 0.3:
        left_bound = float(np.array(md.roadEdges[0].y)[0]) + EDGE_SAFETY
      if right_bound is None and len(md.roadEdges) >= 2 and md.roadEdgeProbs[1] > 0.3:
        right_bound = float(np.array(md.roadEdges[1].y)[0]) - EDGE_SAFETY
      if left_bound is not None and right_bound is not None and left_bound < right_bound:
        lo = left_bound - y0
        hi = right_bound - y0
        profile = np.clip(profile, lo, hi)
    except Exception:
      pass
    return profile

  def _emit(self, cfg):
    now = time.monotonic()
    if now - self._log_ts >= 1.0:
      self._log_ts = now
      d = self.diag
      _log(f"[OA] why={d['why']} obs={d['n_obs']} aux={d['aux']} "
           f"auxdead={int(d['aux_dead'])} lidar={d['lidar']} "
           f"closest={d['closest']:.1f} shift={d['shift']:.2f} "
           f"conflict={int(d['conflict'])} active={int(cfg['active'])}")
    if not cfg["active"]:
      return None
    if self._profile is None:
      return None
    return self._profile.copy()
