"""停车跟随时，前车「起步」必须由距离证据确认。

移植自 carrot-wip `openpilot/selfdrive/controls/lib/longitudinal_stopping_lead.py`。

【为什么需要】
停车跟停（longControlState == stopping）时，雷达测距存在 5 cm 量化台阶。单帧台阶
跳变会让 MPC 认为 vLead > 0，于是自车跟着「起步」——表现为停车时一阵阵往前窜、
或本该停住却慢慢溜车。本模块的做法是：在 planner 输入端把 lead 的速度/加速度
一律按 0 处理，只有当「距离连续张开 >= 15 cm 且持续 >= 0.1 s」（即连续跨过三个
5 cm 档位）才承认前车真的走了。

【设计边界】
- 只条件化 planner 输入副本，原始 radarState 与雷达测量完全不动；
- 距离 dRel 永不冻结（前车若继续靠近，MPC 必须看得到），只冻结速度/加速度；
- MPC 仍然独占停止/起步输出权，本模块不产生任何加速度指令。

【相对 carrot-wip 的适配（设备 cereal schema 差异）】
- carrot `LeadData.status`  -> 设备用 `lead.present`
- carrot `LeadData.jLead`    -> 设备 RadarState.LeadData 无此字段，去掉
- carrot `LeadData.aLead`    -> 设备已归入 deprecated group，不再置零（MPC 只读 aLeadK）
- `radar_state.as_builder()` 兼容分支保留：设备 `sm['radarState']` 返回
  `_DynamicStructReader`，实测支持 as_builder()（往返保真已验）
"""

from dataclasses import dataclass
import math


STOPPED_EGO_SPEED = 0.10
STOPPED_LEAD_SPEED = 0.30
STABLE_RANGE_SPAN = 0.10
STABLE_TIME = 0.20
# Three 5 cm range bins: a single quantized range step is not departure.
DEPARTURE_DISTANCE = 0.15
DEPARTURE_TIME = 0.10
MAX_FRAME_GAP = 0.20


@dataclass
class _LeadEvidence:
  track_id: int
  since: float
  min_distance: float
  max_distance: float
  anchor_distance: float
  held: bool = False
  departure_since: float | None = None
  departed: bool = False


class StoppingLeadFilter:
  def __init__(self):
    self._leads: dict[str, _LeadEvidence] = {}
    self._last_time: float | None = None
    self.held_mask = 0

  def _hold_lead(self, role: str, lead, v_ego: float, now: float, fresh: bool) -> bool:
    # 设备 schema：present / radar / radarTrackId / dRel / vRel / vLead / aLeadK
    valid = (
      bool(lead.present) and bool(lead.radar) and lead.radarTrackId >= 0
      and all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.vLead, lead.aLeadK))
      and lead.dRel > 0.2
    )
    if not valid:
      self._leads.pop(role, None)
      return False

    evidence = self._leads.get(role)
    if evidence is not None and evidence.track_id != lead.radarTrackId:
      self._leads.pop(role)
      evidence = None

    # 明确的接近/减速观测走原路径，不干预。
    if lead.vLead < -STOPPED_LEAD_SPEED or lead.aLeadK < -0.5:
      self._leads.pop(role, None)
      return False

    quiet = v_ego <= STOPPED_EGO_SPEED and abs(lead.vLead) <= STOPPED_LEAD_SPEED
    if evidence is None:
      if not quiet or not fresh:
        return False
      evidence = _LeadEvidence(int(lead.radarTrackId), now, lead.dRel, lead.dRel, lead.dRel)
      self._leads[role] = evidence

    if evidence.departed:
      return False
    if not fresh:
      return evidence.held

    if not evidence.held:
      evidence.min_distance = min(evidence.min_distance, lead.dRel)
      evidence.max_distance = max(evidence.max_distance, lead.dRel)
      if not quiet or evidence.max_distance - evidence.min_distance > STABLE_RANGE_SPAN + 1e-6:
        self._leads.pop(role)
        return False
      if now - evidence.since < STABLE_TIME - 1e-6:
        return False
      evidence.held = True
      evidence.anchor_distance = lead.dRel

    # 距离仍在缩小：绝不冻结障碍物距离，重新建立证据而不是把「距离回落」当起步。
    if lead.dRel < evidence.anchor_distance - STABLE_RANGE_SPAN - 1e-6:
      self._leads.pop(role)
      return False

    opening = (
      lead.dRel - evidence.anchor_distance >= DEPARTURE_DISTANCE - 1e-6
      and lead.vRel > 0.0 and lead.vLead > 0.0
    )
    if opening:
      if evidence.departure_since is None:
        evidence.departure_since = now
      if now - evidence.departure_since >= DEPARTURE_TIME - 1e-6:
        evidence.departed = True
        return False
    else:
      evidence.departure_since = None
    return True

  def update(self, radar_state, *, stopping: bool, v_ego: float, mono_time_ns: int, valid: bool = True):
    self.held_mask = 0
    now = mono_time_ns * 1e-9
    if not stopping or not valid or not math.isfinite(v_ego) or mono_time_ns <= 0:
      self._leads.clear()
      self._last_time = None
      return radar_state

    if self._last_time is not None and abs(now - self._last_time) > MAX_FRAME_GAP + 1e-6:
      self._leads.clear()
      self._last_time = None
    # planner 以 modelV2 为节拍，radarState 可能比上一拍更旧：既不确认起步，
    # 也不擦掉已 hold 的证据。
    fresh = self._last_time is None or now > self._last_time
    self._last_time = now if fresh else self._last_time

    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      if self._hold_lead(role, getattr(radar_state, role), abs(v_ego), now, fresh):
        self.held_mask |= mask
    if not self.held_mask:
      return radar_state

    output = radar_state.as_builder() if hasattr(radar_state, "as_builder") else radar_state.as_reader().as_builder()
    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      if self.held_mask & mask:
        lead = getattr(output, role)
        lead.vLead = lead.vLeadK = 0.0
        lead.aLeadK = 0.0
    return output
