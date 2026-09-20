#!/usr/bin/env python3
"""原车前雷达的「前侧盲区」检测（左/右相邻车道是否有妨碍变道的目标）。

移植并重写自 cpv9-dev ``controls/lib/desire_helper.py`` 的
``leftFrontBlind`` / ``rightFrontBlind`` 计算（约 700~734 行）。

**原实现的几个问题（本模块已改掉）**：

1. 用 ``abs(vLead) > 2.8`` —— 拿**绝对车速**当判据。侧后方静止的车
   （vLead≈0）永远判不出，而远处同向快车（vLead 很大）却会误判。
   侧车道威胁只与**相对速度** vRel 有关，这里改用 vRel。
2. ``side_object_dist = dRel + vLead * 3.0`` —— 同样把绝对速度当成接近
   速度用。改为 ``dRel + vRel * horizon``（相对速度外推）。
3. 没有横向判定：不区分"本车道前车 / 相邻车道目标 / 再远一条车道的车"，
   任何被雷达认成 leadLeft/leadRight 的目标都算数。这里按车道宽做横向门限。
4. 单点阈值、没有去抖 → 临界处来回跳。这里加了进入/退出延时（迟滞）。
5. 阈值全部硬编码（3.0s / 2.8 / v_ego*4），这里全部参数化（见 config.py）。

判据（单侧）::

  1) 雷达目标有效（status）
  2) 横向：出本车道、且在 ~1.8 倍车道宽以内 → 只认相邻车道
  3) 纵向：dRel 落在 [-后向, +前向] 窗口内
  4) 风险（满足其一）：
       * 预测窗口内的最小纵向间距 < 安全间距(max(v_ego*时距, 最小间距))
       * TTC < 阈值（前方目标被追上 / 后方目标追上来）
  5) 去抖：连续 risky 达 on_time 才置位，连续不 risky 达 off_time 才清除

参数全部走 amapnavi 自己的参数模块（``config.py`` 的 ``UnifiedParams``），
不往 openpilot 的 ``params_keys.h`` 里加东西。
"""

from dataclasses import dataclass, field

from openpilot.selfdrive.carrot.amapnavi.config import unified_params

# 单位换算：参数以 0.1 为单位存储（与 amapnavi 其它参数一致）
_SCALE = 0.1

DEFAULT_LANE_WIDTH_M = 3.5


@dataclass
class StockFrontBlindConfig:
  """原车前侧盲区的判据参数。"""

  enable: bool = True
  lat_min_m: float = 1.2          # 横向下限（小于此值视为本车道内）
  lat_max_m: float = 5.0          # 横向上限（硬上限）
  lat_lane_factor: float = 1.8    # 横向上限 = factor * 车道宽
  drel_min_m: float = -20.0       # 纵向窗口：后向（负=在身后）
  drel_max_m: float = 80.0        # 纵向窗口：前向
  time_headway_s: float = 1.5     # 安全时距
  min_clearance_m: float = 4.0    # 绝对最小纵向间距
  ttc_threshold_s: float = 2.5    # TTC 阈值
  horizon_s: float = 3.0          # 预测窗口（一次完整变道）
  on_time_s: float = 0.3          # 置位需要持续 risky 的时间
  off_time_s: float = 0.8         # 清除需要持续不 risky 的时间

  @classmethod
  def from_params(cls, params=unified_params) -> "StockFrontBlindConfig":
    def i(key, default):
      try:
        value = int(params.get_int(key))
      except Exception:
        value = default
      return value

    return cls(
      enable=bool(i("StockFrontBlindEnable", 1)),
      lat_min_m=max(0.0, i("StockFrontLatMin", 12) * _SCALE),
      lat_max_m=max(0.1, i("StockFrontLatMax", 50) * _SCALE),
      lat_lane_factor=max(1.0, i("StockFrontLatLaneFactor", 18) * _SCALE),
      drel_min_m=float(i("StockFrontDrelMin", -200) * _SCALE),
      drel_max_m=float(i("StockFrontDrelMax", 800) * _SCALE),
      time_headway_s=max(0.1, i("StockFrontTimeHeadway", 15) * _SCALE),
      min_clearance_m=max(0.5, i("StockFrontMinClearance", 40) * _SCALE),
      ttc_threshold_s=max(0.1, i("StockFrontTtc", 25) * _SCALE),
      horizon_s=max(0.5, i("StockFrontHorizon", 30) * _SCALE),
      on_time_s=max(0.0, i("StockFrontOnTime", 3) * _SCALE),
      off_time_s=max(0.0, i("StockFrontOffTime", 8) * _SCALE),
    )


@dataclass
class SideDecision:
  """单侧一帧的判定明细（便于调试与单元测试）。"""

  active: bool = False
  raw: bool = False
  lateral_m: float = 0.0
  drel_m: float = 0.0
  vrel_mps: float = 0.0
  clearance_m: float = 999.0
  ttc_s: float | None = None
  reason: str = "no_target"


@dataclass
class _SideState:
  active: bool = False
  on_timer: float = 0.0
  off_timer: float = 0.0
  last: SideDecision = field(default_factory=SideDecision)


def lateral_offset(lead) -> float:
  """目标相对本车行驶轨迹的横向偏移（m，左正右负由调用方按侧别处理）。

  优先用 ``dPath``（用模型预测路径算的横向距离，弯道更准），
  没有时退回雷达自身的 ``yRel``。
  """
  d_path = float(getattr(lead, "dPath", 0.0) or 0.0)
  if abs(d_path) > 0.05:
    return d_path
  return float(getattr(lead, "yRel", 0.0) or 0.0)


def evaluate_side(lead, v_ego_mps: float, lane_width_m: float,
                  cfg: StockFrontBlindConfig) -> SideDecision:
  """评估单侧原车前雷达目标是否构成「前侧盲区」。"""
  if lead is None or not bool(getattr(lead, "status", False)):
    return SideDecision(reason="no_target")

  drel = float(getattr(lead, "dRel", 0.0) or 0.0)
  vrel = float(getattr(lead, "vRel", 0.0) or 0.0)
  lat = abs(lateral_offset(lead))

  # ---- 横向门限：必须出了本车道、且没有远到第二条车道之外 ----
  lane_width = lane_width_m if lane_width_m and lane_width_m > 0.5 else DEFAULT_LANE_WIDTH_M
  lat_min = max(cfg.lat_min_m, lane_width * 0.5)
  lat_max = min(cfg.lat_max_m, lane_width * cfg.lat_lane_factor)
  if lat < lat_min:
    return SideDecision(raw=False, lateral_m=lat, drel_m=drel, vrel_mps=vrel, reason="same_lane")
  if lat > lat_max:
    return SideDecision(raw=False, lateral_m=lat, drel_m=drel, vrel_mps=vrel, reason="far_lane")

  # ---- 纵向窗口 ----
  if drel < cfg.drel_min_m or drel > cfg.drel_max_m:
    return SideDecision(raw=False, lateral_m=lat, drel_m=drel, vrel_mps=vrel, reason="out_of_range")

  # ---- 风险：预测窗口内的最小间距 + TTC ----
  d_end = drel + vrel * cfg.horizon_s
  if drel * d_end < 0:                       # 窗口内会并肩通过 → 间距按 0 算
    clearance = 0.0
  else:
    clearance = min(abs(drel), abs(d_end))
  safe = max(v_ego_mps * cfg.time_headway_s, cfg.min_clearance_m)

  ttc = None
  if vrel < 0 and drel > 0:                  # 前方目标在被追上
    ttc = drel / -vrel
  elif vrel > 0 and drel < 0:                # 后方目标在追上来
    ttc = abs(drel) / vrel

  ttc_risk = ttc is not None and 0.0 <= ttc < cfg.ttc_threshold_s
  clearance_risk = clearance < safe
  raw = bool(ttc_risk or clearance_risk)

  reason = "clear"
  if ttc_risk:
    reason = "ttc"
  elif clearance_risk:
    reason = "clearance"

  return SideDecision(raw=raw, lateral_m=lat, drel_m=drel, vrel_mps=vrel,
                      clearance_m=clearance, ttc_s=ttc, reason=reason)


class StockFrontBlindMonitor:
  """带迟滞的原车前侧盲区监视器（左右各一份状态）。"""

  def __init__(self, cfg: StockFrontBlindConfig | None = None, params=unified_params):
    # 容错：把参数对象当 cfg 位置参数传进来时自动纠正（历史踩过的坑——
    # self.cfg 成了 UnifiedParams，每帧在 cfg.enable 上抛 AttributeError，
    # 而本模块跑在 amapnavi 主数据循环里，异常会连带跳过该帧后续所有逻辑）。
    if cfg is not None and not isinstance(cfg, StockFrontBlindConfig):
      params = cfg
      cfg = None
    self.params = params
    self.cfg = cfg or StockFrontBlindConfig.from_params(params)
    self._sides = {"left": _SideState(), "right": _SideState()}
    self._param_frame = 0

  # ------------------------------------------------------------------ 参数
  def refresh_config(self, every_n_frames: int = 20) -> None:
    """周期性刷新参数（默认每秒左右一次，DATA_HZ=20）。"""
    self._param_frame += 1
    if self._param_frame % max(1, every_n_frames) == 0:
      self.cfg = StockFrontBlindConfig.from_params(self.params)

  # ------------------------------------------------------------------ 主入口
  def update(self, dt: float, v_ego_mps: float, leads: dict,
             lane_width_left_m: float = DEFAULT_LANE_WIDTH_M,
             lane_width_right_m: float = DEFAULT_LANE_WIDTH_M) -> dict:
    """:param leads: ``{"left": LeadData|None, "right": LeadData|None}``
       :return: ``{"left": bool, "right": bool}``"""
    if not self.cfg.enable:
      for st in self._sides.values():
        st.active = False
        st.on_timer = 0.0
        st.off_timer = 0.0
        st.last = SideDecision(reason="disabled")
      return {"left": False, "right": False}

    widths = {"left": lane_width_left_m, "right": lane_width_right_m}
    out = {}
    for side, st in self._sides.items():
      decision = evaluate_side(leads.get(side), v_ego_mps,
                               widths.get(side, DEFAULT_LANE_WIDTH_M), self.cfg)
      st.last = decision

      if decision.raw:
        st.on_timer = min(st.on_timer + dt, max(self.cfg.on_time_s, dt))
        st.off_timer = 0.0
        if st.on_timer >= self.cfg.on_time_s:
          st.active = True
      else:
        st.off_timer = min(st.off_timer + dt, max(self.cfg.off_time_s, dt))
        st.on_timer = 0.0
        if st.off_timer >= self.cfg.off_time_s:
          st.active = False

      decision.active = st.active
      out[side] = st.active
    return out

  # ------------------------------------------------------------------ 诊断
  def detail(self, side: str) -> SideDecision:
    return self._sides[side].last


def _model_front_blind(sm, side: str) -> bool:
  """模型（视觉）给的前侧盲区：本 fork 的 modelV2.meta 未必有该字段，取不到按 False。"""
  if not sm.alive['modelV2']:
    return False
  meta = sm['modelV2'].meta
  key = f"{side}FrontBlind"
  if not hasattr(meta, key):
    return False
  return bool(getattr(meta, key))


def apply_stock_front_blind(shared_data, sm, dt: float,
                            monitor: "StockFrontBlindMonitor | None" = None,
                            v_ego_mps: float | None = None) -> StockFrontBlindMonitor:
  """用原车前雷达补算 ``shared_data.leftFrontBlind`` / ``rightFrontBlind``。

  与模型的视觉结果按 **OR** 合并（与本 fork "视觉盲区与 OEM 盲区 OR 合并、
  视觉结果永不清除 OEM 盲区" 的约定一致）：任一方判盲区即为盲区。
  功能关闭时直接返回，此时 messages.py 会退回模型自身的值。
  """
  monitor = monitor if monitor is not None else StockFrontBlindMonitor()
  monitor.refresh_config()
  if not monitor.cfg.enable:
    # 功能关闭时清掉标志，避免关掉之后图标/护栏还挂在关闭前的 True 上
    shared_data.leftFrontBlind = False
    shared_data.rightFrontBlind = False
    return monitor

  v_ego = v_ego_mps if v_ego_mps is not None else (shared_data.v_ego_m or 0.0)

  radar = sm['radarState'] if sm.alive['radarState'] else None
  leads = {"left": None, "right": None}
  if radar is not None:
    leads["left"] = getattr(radar, "leadLeft", None)
    leads["right"] = getattr(radar, "leadRight", None)

  lane_left = DEFAULT_LANE_WIDTH_M
  lane_right = DEFAULT_LANE_WIDTH_M
  if sm.alive['modelV2']:
    meta = sm['modelV2'].meta
    lane_left = float(getattr(meta, "laneWidthLeft", 0.0) or 0.0) or DEFAULT_LANE_WIDTH_M
    lane_right = float(getattr(meta, "laneWidthRight", 0.0) or 0.0) or DEFAULT_LANE_WIDTH_M

  result = monitor.update(dt, v_ego, leads, lane_left, lane_right)

  shared_data.leftFrontBlind = bool(result["left"]) or _model_front_blind(sm, "left")
  shared_data.rightFrontBlind = bool(result["right"]) or _model_front_blind(sm, "right")
  return monitor
