#!/usr/bin/env python3
"""原车前侧盲区（stock_front_blind）的单元测试。

用合成的原车雷达目标（不依赖真实硬件）逐条验证判据，同时对比
cpv9-dev 旧算法，固化"为什么新算法更好"。

在设备上运行::

  cd /data/openpilot
  PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \\
    python3 -m openpilot.selfdrive.carrot.amapnavi.tests.test_stock_front_blind

也可直接被 pytest 收集（每个 check_* 都是独立用例）。
"""

from types import SimpleNamespace

from openpilot.selfdrive.carrot.amapnavi.stock_front_blind import (
  DEFAULT_LANE_WIDTH_M,
  StockFrontBlindConfig,
  StockFrontBlindMonitor,
  evaluate_side,
  lateral_offset,
)

DT = 0.05  # 数据线程 20Hz
LANE = 3.5


# ------------------------------------------------------------------ 工具
def lead(drel=0.0, vrel=0.0, lateral=LANE, vlead=None, status=True, use_dpath=True):
  """构造一个 LeadData 风格的合成目标。"""
  obj = SimpleNamespace(
    status=status,
    dRel=float(drel),
    yRel=float(lateral),
    vRel=float(vrel),
    vLead=float(vrel) if vlead is None else float(vlead),
    aRel=0.0,
    dPath=float(lateral) if use_dpath else 0.0,
    vLat=0.0,
  )
  return obj


def cfg(**kw) -> StockFrontBlindConfig:
  base = StockFrontBlindConfig()
  for k, v in kw.items():
    setattr(base, k, v)
  return base


def run(seconds, leads, v_ego=30.0, config=None, lane_left=LANE, lane_right=LANE):
  """连续喂若干帧，返回 (left, right) 与监视器。"""
  m = StockFrontBlindMonitor(config or cfg())
  left = right = False
  for _ in range(int(seconds / DT)):
    res = m.update(DT, v_ego, leads, lane_left, lane_right)
    left, right = res["left"], res["right"]
  return left, right, m


# ------------------------------------------------------------------ 用例
def test_no_target():
  left, right, _ = run(1.0, {"left": None, "right": None})
  assert not left and not right, "无目标时不该报盲区"


def test_target_in_own_lane_is_ignored():
  """本车道前车不算前侧盲区（旧算法不区分，会误判）。"""
  leads = {"left": lead(drel=20.0, vrel=-5.0, lateral=0.2), "right": None}
  left, _r, m = run(1.0, leads)
  assert not left, "本车道目标不应判为前侧盲区"
  assert m.detail("left").reason == "same_lane"


def test_far_lane_is_ignored():
  """隔两条车道的车不算（超过 1.8 倍车道宽）。"""
  leads = {"left": lead(drel=20.0, vrel=-5.0, lateral=LANE * 2.4), "right": None}
  left, _r, m = run(1.0, leads)
  assert not left, "隔车道的车不应判为前侧盲区"
  assert m.detail("left").reason == "far_lane"


def test_out_of_longitudinal_range():
  """纵向窗口之外（>80m）不算。"""
  leads = {"left": lead(drel=150.0, vrel=-5.0, lateral=LANE), "right": None}
  left, _r, m = run(1.0, leads)
  assert not left
  assert m.detail("left").reason == "out_of_range"


def test_alongside_target_is_blind():
  """相邻车道、纵向几乎并肩 → 间距为 0 → 盲区。"""
  leads = {"left": lead(drel=0.5, vrel=0.0, lateral=LANE), "right": None}
  left, right, _ = run(1.0, leads)
  assert left, "并肩目标应判为前侧盲区"
  assert not right, "右侧无目标"


def test_stationary_side_target_is_blind_but_old_algorithm_misses_it():
  """相邻车道的静止车：新算法能判出，旧算法（abs(vLead)>2.8）判不出。

  旧算法: side_object_block = (... ) and abs(vLead) > 2.8
    静止车 vLead = 0 → False → 漏检（明明变道过去会撞上）。
  """
  target = lead(drel=25.0, vrel=-30.0, lateral=LANE, vlead=0.0)  # 我方 30m/s 接近静止目标
  decision = evaluate_side(target, v_ego_mps=30.0, lane_width_m=LANE, cfg=cfg())
  assert decision.raw, "相邻车道的静止目标应判为风险"

  # 旧算法复算：必然 False
  old_block = ((target.dRel + target.vLead * 3.0 < 30.0 * (3.0 + 1.0)) or
               (target.dRel < 30.0 * 1.0)) and abs(target.vLead) > 2.8
  assert not old_block, "旧算法对静止目标应当漏检（这正是要修的问题）"


def test_fast_distant_same_direction_target_not_blind():
  """远处同向快车（vLead 很大）不该误判 —— 旧算法这里会误报。

  旧算法 abs(vLead) > 2.8 成立，且 dRel < v_ego*min_drel_vego_time 时误报。
  """
  # 前方 70m、同向、相对速度接近 0（一起跑）
  target = lead(drel=70.0, vrel=0.5, lateral=LANE, vlead=30.0)
  decision = evaluate_side(target, v_ego_mps=30.0, lane_width_m=LANE, cfg=cfg())
  assert not decision.raw, "同向等速的远处目标不应判为风险"


def test_ttc_risk_rear_approaching():
  """后方快速追上来的目标（dRel<0, vRel>0）应判风险。"""
  target = lead(drel=-15.0, vrel=8.0, lateral=LANE)  # 8m/s 追上来 → TTC≈1.9s
  decision = evaluate_side(target, v_ego_mps=30.0, lane_width_m=LANE, cfg=cfg())
  assert decision.raw, "后方快速接近的目标应判为风险"
  assert decision.ttc_s is not None and decision.ttc_s < 2.5


def test_debounce_on_and_off():
  """去抖：单帧风险不立即置位；风险消失后延时释放。"""
  m = StockFrontBlindMonitor(cfg(on_time_s=0.3, off_time_s=0.8))
  risky = {"left": lead(drel=0.5, vrel=0.0, lateral=LANE), "right": None}
  clear = {"left": None, "right": None}

  # 只喂 1 帧风险（0.05s < on_time 0.3s）→ 不置位
  res = m.update(DT, 30.0, risky, LANE, LANE)
  assert not res["left"], "单帧风险不应立即置位"

  # 持续喂够 on_time → 置位
  for _ in range(int(0.3 / DT) + 1):
    res = m.update(DT, 30.0, risky, LANE, LANE)
  assert res["left"], "持续风险应置位"

  # 目标消失 0.5s（< off_time 0.8s）→ 仍保持
  for _ in range(int(0.5 / DT)):
    res = m.update(DT, 30.0, clear, LANE, LANE)
  assert res["left"], "释放延时应保持盲区"

  # 再等够 off_time → 释放
  for _ in range(int(0.4 / DT) + 1):
    res = m.update(DT, 30.0, clear, LANE, LANE)
  assert not res["left"], "超过释放延时应清除盲区"


def test_disable():
  """关闭功能时双侧恒为 False。"""
  m = StockFrontBlindMonitor(cfg(enable=False))
  for _ in range(20):
    res = m.update(DT, 30.0, {"left": lead(drel=0.5, lateral=LANE), "right": None}, LANE, LANE)
  assert not res["left"] and not res["right"]


def test_lateral_offset_prefers_dpath():
  """优先用 dPath（模型路径算的横向偏移），没有时退回 yRel。"""
  obj = SimpleNamespace(dPath=-3.2, yRel=1.1)
  assert lateral_offset(obj) == -3.2
  obj2 = SimpleNamespace(dPath=0.0, yRel=1.1)
  assert lateral_offset(obj2) == 1.1


def test_lane_width_narrows_lateral_window():
  """窄车道时横向上限随车道宽收紧（1.8*2.0=3.6m）。"""
  target = lead(drel=0.5, lateral=4.2)  # 4.2m 在 3.5m 车道算相邻，2.0m 车道算隔道
  wide = evaluate_side(target, 30.0, LANE, cfg())
  narrow = evaluate_side(target, 30.0, 2.0, cfg())
  assert wide.raw, "宽车道下 4.2m 应算相邻车道"
  assert not narrow.raw and narrow.reason == "far_lane", "窄车道下 4.2m 应算隔道"


def test_default_lane_width_used_when_missing():
  """车道宽缺失（0）时退回默认车道宽。"""
  target = lead(drel=0.5, lateral=DEFAULT_LANE_WIDTH_M)
  decision = evaluate_side(target, 30.0, 0.0, cfg())
  assert decision.raw


# ------------------------------------------------------------------ 运行
CHECKS = (
    test_no_target,
    test_target_in_own_lane_is_ignored,
    test_far_lane_is_ignored,
    test_out_of_longitudinal_range,
    test_alongside_target_is_blind,
    test_stationary_side_target_is_blind_but_old_algorithm_misses_it,
    test_fast_distant_same_direction_target_not_blind,
    test_ttc_risk_rear_approaching,
    test_debounce_on_and_off,
    test_disable,
    test_lateral_offset_prefers_dpath,
    test_lane_width_narrows_lateral_window,
    test_default_lane_width_used_when_missing,
)


def main() -> int:
  failed = 0
  for fn in CHECKS:
    try:
      fn()
      print(f"  [PASS] {fn.__name__}")
    except AssertionError as e:
      failed += 1
      print(f"  [FAIL] {fn.__name__}: {e}")
    except Exception as e:  # noqa: BLE001
      failed += 1
      print(f"  [ERROR] {fn.__name__}: {type(e).__name__}: {e}")
  print("")
  print(f"===== 结果: {len(CHECKS) - failed}/{len(CHECKS)} 通过 =====")
  return 1 if failed else 0


if __name__ == "__main__":
  raise SystemExit(main())
