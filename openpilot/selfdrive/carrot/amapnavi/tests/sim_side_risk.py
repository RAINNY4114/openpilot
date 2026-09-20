#!/usr/bin/env python3
"""侧向碰撞风险(激光雷达变道判定)改版后的回归仿真。

覆盖面：
  A/B/C  三种接近场景的判定时机（vrel 时距做预测窗口 + drel 时距做危险距离）
  D/E    目标消失/设备静默后，标志是否在 1s 内复位（原来会一直残留）
  F      动态盲区判定区域（侧面车道宽度）对邻道车是否误屏蔽
  G      迟滞：临界点附近是否抖动

用法（设备本机）：
  cd /data/openpilot
  PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \
    python3 /tmp/c3_sim_risk2.py
"""
import os
import time

from openpilot.selfdrive.carrot.amapnavi.protocol import PacketHandler, CORNER_DATA_TIMEOUT_S
from openpilot.selfdrive.carrot.amapnavi.shared_state import SharedData
from openpilot.selfdrive.carrot.amapnavi.blindspot import side_object_risky
from openpilot.selfdrive.carrot.amapnavi.config import unified_params

DT = 0.1
IP = "127.0.0.9"
V_EGO = 10.0


def make_handler():
  shared = SharedData()
  shared.v_ego_m = V_EGO
  h = PacketHandler(shared, None)
  h.min_front_drel_vego_time = unified_params.get_int("LidarFrontVDistTime") * 0.1
  h.min_front_vrel_vego_time = unified_params.get_int("LidarFrontVRelDistTime") * 0.1
  h.min_behind_drel_vego_time = unified_params.get_int("LidarBehindVDistTime") * 0.1
  h.min_behind_vrel_vego_time = unified_params.get_int("LidarBehindVRelDistTime") * 0.1
  h.min_clearance_m = max(1.0, unified_params.get_int("LidarMinClearance") * 0.1)
  h.ttc_threshold_s = max(0.5, unified_params.get_int("LidarTtcThreshold") * 0.1)
  return shared, h


def packet(t_ms, corners):
  msg = {"device": "lidar", "resp": "blindspot", "lidar_id": 0, "detect_side": 3,
         "dist_time": t_ms}
  for k in ("lblind", "rblind", "lfblind", "rfblind", "lbblind", "rbblind"):
    msg["lidar_" + k] = False
  for c, d in corners.items():
    msg[c + "_drel"] = int(d)
    msg[c + "_xrel"] = 1500
  return msg


def run(name, corner, traj, silent_from=None, desc=""):
  shared, h = make_handler()
  behind = corner in ("lb", "rb")
  vrel_t = h.min_behind_vrel_vego_time if behind else h.min_front_vrel_vego_time
  drel_t = h.min_behind_drel_vego_time if behind else h.min_front_drel_vego_time
  print("\n--- %s %s ---" % (name, desc))
  print("    vrel时距=%.1fs(预测窗口)  drel时距=%.1f(%s)  TTC<%.1fs  数据超时复位=%.1fs" % (
    vrel_t, drel_t, ("绝对%.1fm" % abs(drel_t)) if drel_t < 0 else ("vEgo×%.1fs" % drel_t),
    h.ttc_threshold_s, CORNER_DATA_TIMEOUT_S))
  info = {}
  t0 = time.time()
  steps = int(traj[-1][0] / DT) + 1
  last = -99.0
  first_risky = None
  clear_at = None
  for i in range(steps):
    t = i * DT
    d = _interp(traj, t)
    ms = int((t0 + t) * 1000)
    if silent_from is not None and t >= silent_from:
      msg = {"device": "lidar", "resp": "blindspot", "lidar_id": 0, "detect_side": 3, "dist_time": ms}
    elif silent_from is not None and t >= silent_from - 1.0:
      msg = packet(ms, {})
    else:
      msg = packet(ms, {corner: d})
    info = h.handle(msg, IP, info, time.time())
    h.expire_corners()          # 模拟 amap_navi.lidar_object_blind() 每帧调用
    risky = h.object_detected[corner]
    if risky and first_risky is None:
      first_risky = (t, abs(d) / 1000.0)
    if first_risky is not None and not risky and clear_at is None and t > first_risky[0]:
      clear_at = t
    if t - last >= 1.0 or i == steps - 1:
      last = t
      print("  t=%4.1fs d=%7.2fm risky=%-5s" % (t, d / 1000.0, risky))
    if not os.environ.get("SIM_NOSLEEP"):
      time.sleep(DT)
  if first_risky:
    print("  >> 首次判危险: t=%.1fs 距离 %.1fm" % first_risky)
  else:
    print("  >> 全程未判危险")
  if clear_at is not None:
    print("  >> 判定清除: t=%.1fs" % clear_at)
  if silent_from is not None:
    # 用真实时间验证数据超时复位（主循环是无休眠加速跑的）
    time.sleep(CORNER_DATA_TIMEOUT_S + 0.2)
    h.expire_corners()
    print("  >> 静默 %.1fs 后复位: risky=%s (期望 False)" % (
      CORNER_DATA_TIMEOUT_S + 0.2, h.object_detected[corner]))
  return shared, h


def _interp(traj, t):
  if t <= traj[0][0]:
    return traj[0][1]
  for i in range(1, len(traj)):
    t0, d0 = traj[i - 1]
    t1, d1 = traj[i]
    if t <= t1:
      f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
      return d0 + (d1 - d0) * f
  return traj[-1][1]


def mask_test():
  print("\n--- F 动态盲区判定区域(侧面车道宽度) ---")
  try:
    from openpilot.selfdrive.carrot.amapnavi.amap_navi import side_region_limit_mm
  except Exception as e:
    print("  导入失败: %s" % e)
    return
  print("  车道宽读数 -> 判定区域上限(旧实现 1.2~3.5m 直取，新实现 1.5 倍且 3.5~5.5m 兜底)")
  for w in (2.0, 2.44, 2.98, 3.2, 3.5, 3.9, 5.0):
    new = side_region_limit_mm(w) / 1000.0
    old = max(1.2, min(3.5, round(w, 1)))
    print("    laneWidth=%.2fm -> 旧上限 %.2fm   新上限 %.2fm" % (w, old, new))
  print("  邻道车典型横向 3.0~3.5m：旧实现(2.44/2.98m 车道)会屏蔽，新实现保留")
  for xrel in (2500, 3000, 3300, 3500):
    for w in (2.44, 2.98):
      print("    xrel=%dmm laneWidth=%.2fm -> 旧%s 新%s" % (
        xrel, w,
        "屏蔽" if xrel > max(1.2, min(3.5, round(w, 1))) * 1000 else "保留",
        "屏蔽" if xrel > side_region_limit_mm(w) else "保留"))


def hysteresis_test():
  print("\n--- G 迟滞：距离在危险线附近小幅抖动 ---")
  shared = SharedData()
  shared.v_ego_m = V_EGO
  hits = []
  drel_t, vrel_t = -5.0, 4.0
  latch = False
  for k in range(30):
    d = 4600 if (k // 3) % 2 == 0 else 5400      # 4.6m <-> 5.4m 抖动
    res = side_object_risky(d, 0.0, V_EGO, vrel_time_s=vrel_t, drel_time_s=drel_t, latch=latch)
    latch = res.risky
    hits.append("1" if latch else "0")
  print("  距离 4.6m/5.4m 交替 -> 危险标志序列: %s" % "".join(hits))
  print("  (无迟滞时会 010101 抖；有迟滞进入后要求退出到危险距离×1.4=7m 才清除)")


def main():
  print("vEgo=%.1f m/s" % V_EGO)
  run("A 侧方静止车 8m→4m", "lf", [(0, 8000), (3, 8000), (5, 4000), (7, 4000)])
  run("B 后车 6m/s 快速接近", "lb", [(0, -40000), (5, -10000), (6.5, -1000)])
  run("B2 后车 2m/s 中速接近", "lb", [(0, -30000), (10, -10000)])
  run("C 本车 5m/s 逼近侧前方车", "lf", [(0, 40000), (5, 15000), (7, 5000)])
  run("C2 本车 2m/s 逼近侧前方车", "lf", [(0, 30000), (10, 10000)])
  run("D 目标消失(不再报该角)", "lf", [(0, 4000), (2, 4000), (10, 4000)], silent_from=2.0)
  run("E 设备完全静默", "lf", [(0, 4000), (2, 4000), (10, 4000)], silent_from=2.0)
  mask_test()
  hysteresis_test()


if __name__ == "__main__":
  main()
