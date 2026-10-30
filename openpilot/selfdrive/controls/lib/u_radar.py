#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MR76 毫米波雷达解析 (u_radar.dbc)
==================================

报文:
  0x60A (1546) Status     : NoOfObjects / MeasCount / InterfaceVersion
  0x60B (1547) ObjectData : 每帧一个目标, 8 字节

ObjectData 字段 (Motorola @0, start bit 为信号 MSB):
  ID        7|8    @0+ (1,0)           -> 0..255
  DistLong  15|13  @0+ (0.2,-500)      m   -> -500..1138.2
  DistLat   18|11  @0+ (0.2,-204.6)    m   -> -204.6..204.8
  VRelLong  39|10  @0+ (0.25,-128)     m/s -> -128..127.75
  VRelLat   45|9   @0+ (0.25,-64)      m/s -> -64..63.75
  DynProp   50|3   @0+ (1,0)              0 moving / 1 stationary / 2 oncoming
                                          3 crossing_left / 4 crossing_right
                                          5 unknown / 6 stopped
  Class     52|2   @0+ (1,0)              0 point / 1 vehicle
  RCS       63|8   @0+ (0.5,-64)       dBm2

位序说明: DBC 的 Motorola start bit 是信号最高位, 按 sawtooth 编号。
换算成 LSB0 位置: lsb = 8*(start//8) + (7 - start%8)。
按此换算后 8 个字段互不重叠且正好填满 64 bit —— 已用构造帧自检确认。

坐标系: 本模块对外统一输出 openpilot 约定 (y 正 = 左)。
雷达自身 DistLat 的正方向 DBC 未注明, 用 lat_sign 配置:
  -1 = 雷达右正(常见 ARS 系) -> 翻转为左正   默认
  +1 = 雷达本身就是左正
参考实现 E:\\openpilot\\chestnut 2026-09-20 的注释也确认:
  "DistLat -> yRel (metres, lateral, NEGATIVE = left, POSITIVE = right)"
即原始 DistLat 右正, 所以默认 -1 与之致。
若实车发现避让方向反了, 把 /data/ford_params/Mr76LatSign 改成 1。
"""

import time

ADDR_OBJECT = 0x60B
ADDR_STATUS = 0x60A

# ============================================================================
# 与原车报文区分 (MR76 与整车共用 CAN1)
# ============================================================================
# 参考 E:\openpilot\chestnut 2026-09-20 的实测做法:
#   - 独立 parser 只绑 bus1, 只解 0x60A/0x60B (0x201 配置报文不解析)
#   - MR76 数据完全隔离, 绝不进 RadarData / radarTracks / lead
#   - 靠物理合理性 + Class 白名单过滤, 而不是靠"频率"或"配对"猜测
#   - 实测帧率不定: Status 0.32~16.5Hz, ObjectData 86~287Hz,
#     所以任何基于固定周期的门控都是错的
#
# 这里保留配对门作为额外保险, 但窗口必须 >= Status 的最慢周期(3.1s),
# 否则会把正常数据大批误拒。
STATUS_GATE_SEC = 5.0
MAX_OBJECTS_HDR = 32          # NoOfObjects 合理上限(雷达最多几十个目标)
IFACE_VER_MIN = 1             # InterfaceVersion 0 视为无效

# 物理合理性 (取自参考实现的实车标定值)
D_LONG_RANGE = (-0.5, 150.0)
D_LAT_RANGE = (-25.0, 25.0)
VR_RANGE = (-80.0, 80.0)
VLAT_RANGE = (-50.0, 50.0)
VALID_CLASSES = frozenset({0, 1})   # 0=point, 1=vehicle

# 存活判定: 必须看"对象流"本身, 不能因为 Status 还在就认为雷达活着。
# 参考实现踩过的坑: 0x60A 还来、0x60B 停了 -> 缓存逐个过期清空 ->
# 报告 "fresh=True, 无目标" = "目标车道空", 恰是最危险的失效方向。
OBJECT_TIMEOUT_SEC = 0.50
STATUS_TIMEOUT_SEC = 1.00
MAX_STALE_UPDATES = 10

# ============================================================================
# 目标可信度门 —— 源自参考实现的实测结论, 不是拍脑袋
# ============================================================================
# 参考实现用 6 条路线 / 266,049 个 CAN batch / 1,568,057 个 MR76 目标实测:
#   朴素规则(Class==1 或 |vRel|>2)判"邻道有车"的比例 = 77.34%
#   原厂 BSM 真值                                   = 10.54%
#   => 朴素规则过于激进 7.34 倍
#   其中 85% 的误报是 Class==0 的 point 目标, 84% 是静止目标, 86% RCS<0,
#   典型特征是 |yRel| 2~5m / dRel 10~30m —— 就是路侧护栏、灯杆、树、标志牌。
#
# 本模块做的是"车道内小幅绕行", 不是邻道否决, 所以不能照搬参考的
# 距离/横向窗口; 但必须继承同一条教训: 路侧杂物不能触发横向偏移。
# 这里用几何关系区分, 不引入新猜的数字:
#   若目标横向距离已超过"车半宽 + 目标半宽 + 最小净空", 直行通过本来
#   就不会撞, 那就不该为它打方向。
POINT_CLASS = 0
VEHICLE_CLASS = 1
POINT_HALF_WIDTH = 0.60       # Class 0 目标按 0.60m 半宽保守估计
HALF_CAR_WIDTH = 0.90         # 车半宽
MIN_SAFETY_MARGIN = 0.25      # 最小净空
# = 1.75 m: |y| 超过它, 直行净空已为正, 无需偏移
POINT_RELEVANT_Y = HALF_CAR_WIDTH + POINT_HALF_WIDTH + MIN_SAFETY_MARGIN

# 横向相对速度: 沿车道稳定跟踪的目标 vLat 很小; 横扫而过的多半是
# 重新关联出来的假目标。参考实现用 1.0 m/s。
VLAT_STABLE = 1.0

# RCS 地板: 只用来丢掉明显的垃圾回波, 不是调参旋钮。
# 参考实现用 -5.0 dBsm 做邻道否决(Class-1 的 p50 恰好是 -5.0), 那对
# "否决变道"合适; 对"必须看见正前方障碍"太严 —— 会砍掉一半真车。
RCS_FLOOR = -20.0

# 连续命中确认: 单帧幽灵不参与决策。
# 参考实现用 10 帧(约 0.5s)做邻道否决; 车道内偏移本身还有平滑 + 限幅,
# 3 帧足够滤掉单帧跳变, 又不至于让真障碍晚 0.5s 才被处理。
MIN_HITS = 3

# DynProp 编码
DYN_MOVING = 0
DYN_STATIONARY = 1
DYN_ONCOMING = 2
DYN_CROSSING = (3, 4)
DYN_STOPPED = 6

# DynProp -> 障碍类型 (决定净空与半宽)
_DYN_MAP = {
  DYN_MOVING: "unknown",       # moving, 同向运动
  DYN_STATIONARY: "static",    # stationary
  DYN_ONCOMING: "oncoming",    # 迎面
  3: "pedestrian",             # crossing_left, 横穿最危险
  4: "pedestrian",             # crossing_right
  5: "unknown",
  DYN_STOPPED: "static",       # stopped
}


def _lsb0(start):
  return 8 * (start // 8) + (7 - (start % 8))


def _extract(data, start, length):
  """从 8 字节 Motorola 帧提取无符号原始值。"""
  lsb = _lsb0(start)
  val = 0
  for i in range(length):
    pos = lsb + i
    val |= ((data[pos >> 3] >> (pos & 7)) & 1) << i
  return val


def decode_object(data):
  """解码一帧 ObjectData。data 为 bytes/list, 长度 >= 8。"""
  if data is None or len(data) < 8:
    return None
  d = _extract(data, 15, 13) * 0.2 - 500.0
  lat = _extract(data, 18, 11) * 0.2 - 204.6
  vr = _extract(data, 39, 10) * 0.25 - 128.0
  return {
    "id": _extract(data, 7, 8),
    "d": d,
    "lat": lat,
    "vr": vr,
    "vr_lat": _extract(data, 45, 9) * 0.25 - 64.0,
    "dyn": _extract(data, 50, 3),
    "cls": _extract(data, 52, 2),
    "rcs": _extract(data, 63, 8) * 0.5 - 64.0,
  }


def classify(obj):
  """DynProp + Class -> obstacle_avoid 的类型标签。

  Class 只有 point / vehicle 两档, 比 DynProp 更能区分"是不是人"。
  point 目标里, 静止的多半是柱子/锥桶, 运动的多半是两轮, 横穿的按行人算。
  """
  if obj["cls"] == VEHICLE_CLASS:
    return _DYN_MAP.get(obj["dyn"], "unknown")
  dyn = obj["dyn"]
  if dyn in DYN_CROSSING:             # crossing_left / right
    return "pedestrian"
  if dyn == DYN_MOVING:               # moving point
    return "motorcycle"
  if dyn in (DYN_STATIONARY, DYN_STOPPED):
    return "static"
  return "unknown"


def decode_status(data):
  """解码 Status (0x60A): NoOfObjects / MeasCount / InterfaceVersion。"""
  if data is None or len(data) < 8:
    return None
  return {
    "n_objects": _extract(data, 7, 8),
    "meas_count": _extract(data, 15, 16),
    "iface_ver": _extract(data, 31, 4),
  }


def _plausible(obj):
  """ObjectData 物理合理性检查(硬门)。"""
  return (D_LONG_RANGE[0] < obj["d"] < D_LONG_RANGE[1]
          and D_LAT_RANGE[0] < obj["lat"] < D_LAT_RANGE[1]
          and VR_RANGE[0] < obj["vr"] < VR_RANGE[1]
          and VLAT_RANGE[0] < obj["vr_lat"] < VLAT_RANGE[1]
          and obj["cls"] in VALID_CLASSES)


class Mr76Tracker:
  """把逐帧的 ObjectData 聚合成目标列表。

    雷达每帧只发一个目标(ID 区分), 需要按 ID 维护短时记忆, 否则每帧
    只能看到一个目标, 避障会漏掉其它车。

    与原车共用总线时, require_status=True 会启用配对门:
    没有合法 Status 就不接受 ObjectData, 避免把原车同 ID 报文当目标。
  """

  def __init__(self, timeout=0.6, lat_sign=-1.0, max_targets=24,
               require_status=True, trust_class=True, min_hits=MIN_HITS):
    self.timeout = timeout
    self.lat_sign = lat_sign
    self.max_targets = max_targets
    self.require_status = require_status
    self.trust_class = trust_class
    self.min_hits = min_hits
    self._objs = {}          # id -> (t, obj)
    self._hits = {}          # id -> 连续命中帧数
    self.n_objects = 0       # Status 报文里的 NoOfObjects
    self.frames = 0          # 通过全部门的目标帧数
    self.object_frames = 0   # 收到的 0x60B 帧数(含被拒的)
    self.status_frames = 0
    self.rejected = 0        # 未通过配对门/内容门被丢弃的 0x60B
    self.last_rx = 0.0
    self.last_object_rx = 0.0   # 0x60B 帧到达(过了配对门), 存活判定依据
    self.last_status_rx = 0.0
    self._last_status = 0.0
    self._stale_updates = 0
    self._saw_batch = False

  # ------------------------------------------------------------------
  # 喂数据
  # ------------------------------------------------------------------
  def feed(self, src, address, data, t=None):
    """喂一帧 CAN。返回 True 表示是本雷达的报文(含被拒绝的)。"""
    t = time.monotonic() if t is None else t

    if address == ADDR_STATUS:
      self._saw_batch = True
      st = decode_status(data)
      if st is None:
        return True
      if 0 <= st["n_objects"] <= MAX_OBJECTS_HDR and st["iface_ver"] >= IFACE_VER_MIN:
        self.n_objects = st["n_objects"]
        self._last_status = t
        self.last_status_rx = t
        self.last_rx = t
        self.status_frames += 1
      else:
        self.rejected += 1
      return True

    if address != ADDR_OBJECT:
      return False

    self._saw_batch = True
    self.object_frames += 1

    # 配对门: 没有近期合法 Status -> 大概率是原车同 ID 报文。
    # 窗口取 5s, 因为实测 Status 最慢 0.32Hz(3.1s 周期)。
    if self.require_status and (t - self._last_status) > STATUS_GATE_SEC:
      self.rejected += 1
      return True

    # 到这里已确认"这帧是 MR76 发的", 所以先记存活时间戳, 再做内容校验。
    # 参考实现同样是先记 mr76_last_rx_time 再解内容 —— 存活 = "雷达在说话",
    # 内容有效 = "说的话可信", 两者必须分开。否则雷达只发垃圾帧时会被
    # 判成"雷达不在", 而不是"雷达坏了"。
    self.last_rx = t
    self.last_object_rx = t

    obj = decode_object(data)
    if obj is None or not _plausible(obj):
      self.rejected += 1
      return True

    oid = obj["id"]
    self._objs[oid] = (t, obj)
    self._hits[oid] = self._hits.get(oid, 0) + 1
    self.frames += 1
    return True

  def tick(self):
    """每个 CAN batch 调一次(与 feed 分离)。

    参考实现按 batch 统计 mr76_stale_updates: 连续 MAX_STALE_UPDATES 个
    batch 都没有 MR76 报文 -> 硬清空缓存。这个计数器若放在 feed() 里
    按帧清零是无效的 —— 同一条总线上原车报文也在喂, 计数器永远为 0,
    硬清空永不触发。
    """
    if self._saw_batch:
      self._saw_batch = False
      self._stale_updates = 0
      return
    self._stale_updates += 1
    if self._stale_updates > MAX_STALE_UPDATES:
      self._objs.clear()
      self._hits.clear()

  # ------------------------------------------------------------------
  # 目标输出
  # ------------------------------------------------------------------
  def prune(self, t=None):
    t = time.monotonic() if t is None else t
    dead = [k for k, (ts, _) in self._objs.items() if (t - ts) > self.timeout]
    for k in dead:
      self._objs.pop(k, None)
      self._hits.pop(k, None)
    if len(self._objs) > self.max_targets:
      # 按距离保留最近的若干个
      keep = sorted(self._objs.items(), key=lambda kv: kv[1][1]["d"])[:self.max_targets]
      self._objs = dict(keep)
      self._hits = {k: v for k, v in self._hits.items() if k in self._objs}

  def _trustworthy(self, obj, y):
    """可信度门: 决定这个目标值不值得为它打方向。

    三条规则, 每条都有出处:
      1. RCS 地板 —— 只丢垃圾回波
      2. 横穿目标豁免 —— 行人正在横穿, vLat 大是特征不是缺陷
      3. Class 0 + 非横穿 —— 只在横向真的会撞时才采纳(几何判定)
    """
    if obj["rcs"] < RCS_FLOOR:
      return False
    if obj["dyn"] in DYN_CROSSING:
      # 横穿: 无论横向在哪都要给净空。绝不能因为 vLat 大就丢掉 ——
      # 那正是行人朝我们走过来的样子。
      return True
    if abs(obj["vr_lat"]) > VLAT_STABLE:
      # 横向速度大但又不是横穿 -> 重新关联出来的假目标 / 擦身而过
      return False
    if not self.trust_class or obj["cls"] == VEHICLE_CLASS:
      return True
    # Class 0 (未分类点目标): 参考实现实测 85% 的误报在这一类。
    # 用几何判定 —— 横向距离已超过"车半宽+目标半宽+最小净空"时,
    # 直行本来就不会撞, 不该为它打方向。
    return abs(y) < POINT_RELEVANT_Y

  def targets(self, max_long=40.0):
    """返回 [(d, y, vr, cls), ...], y 已翻到 openpilot 约定(正=左)。"""
    self.prune()
    out = []
    for oid, (_, obj) in self._objs.items():
      if self._hits.get(oid, 0) < self.min_hits:
        continue
      d = obj["d"]
      if not (0.5 < d < max_long):
        continue
      y = obj["lat"] * self.lat_sign
      if abs(y) > 3.2:
        continue
      if not self._trustworthy(obj, y):
        continue
      out.append((d, y, obj["vr"], classify(obj)))
    return out

  # ------------------------------------------------------------------
  # 存活判定
  # ------------------------------------------------------------------
  def fresh(self, now=None):
    """雷达数据当前是否可用。三分支, 与参考实现一致。

    1. 缓存里有目标   -> 只要"最新目标"没过期就算新鲜
    2. 缓存为空       -> 必须对象流(0x60B)和 Status(0x60A)同时新鲜,
                         这才是"雷达活着且确实没东西"
    3. 其余           -> 不新鲜

    为什么不能只看 Status: 参考实现踩过的坑 —— 0x60A 还活着但 0x60B 停了,
    缓存会在 0.5s 内逐个过期清空, 随后模块报 "fresh=True, 无目标" =
    "目标车道空", 恰好是最危险的失效方向。
    """
    now = time.monotonic() if now is None else now
    self.prune(now)
    if self._objs:
      newest = max(ts for ts, _ in self._objs.values())
      return (now - newest) <= OBJECT_TIMEOUT_SEC
    return (self.last_object_rx > 0.0
            and (now - self.last_object_rx) <= OBJECT_TIMEOUT_SEC
            and self.last_status_rx > 0.0
            and (now - self.last_status_rx) <= STATUS_TIMEOUT_SEC)

  def is_stale(self):
    """True 表示"数据不可信"。下游应整源忽略, 而不是当成"空"。"""
    return not self.fresh()

  # ------------------------------------------------------------------
  # 诊断
  # ------------------------------------------------------------------
  def class_hist(self):
    """(Class 直方图, DynProp 直方图)。

    若 cls[0] 长期占 100%, 说明雷达固件没填 Class 字段, 此时可信度门
    会误杀真车 —— 把 /data/ford_params/Mr76TrustClass 设为 0 关掉该门。
    """
    ch, dh = {}, {}
    for _, obj in self._objs.values():
      ch[obj["cls"]] = ch.get(obj["cls"], 0) + 1
      dh[obj["dyn"]] = dh.get(obj["dyn"], 0) + 1
    return ch, dh

  def status_dict(self):
    """给文件桥用的快照, 下游据此区分"没东西"和"雷达没了"。"""
    ch, dh = self.class_hist()
    return {
      "fresh": self.fresh(),
      "objects": len(self._objs),
      "frames": self.frames,
      "object_frames": self.object_frames,
      "status_frames": self.status_frames,
      "rejected": self.rejected,
      "n_obj_hdr": self.n_objects,
      "cls": ch,
      "dyn": dh,
    }
