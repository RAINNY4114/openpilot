#!/usr/bin/env python3
"""实机(回放)端到端验证：激光雷达侧向判定的三个修复点。

用 UDP 往 amap_navi(4211) 灌虚拟激光雷达数据，再读 amapNavi 消息看真实下发的
leftBlind/rightBlind 位（bit0=激光侧方 bit4=侧前 bit5=侧后）：

  1) 后车 6m/s 逼近 → 应在 ~34m 就置位（旧实现要 22m）
  2) 邻道车横向 3.3m → 动态盲区不应把它屏蔽（旧实现 laneWidth<3.3m 时必屏蔽）
  3) 停止上报 → 1.5s 内标志必须清零（旧实现会永久残留）

用法（设备本机）：
  cd /data/openpilot
  PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \
    python3 /tmp/c3_live_verify.py
"""
import json
import socket
import time

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.carrot.amapnavi.transport import LISTEN_PORT

SRC_IP = "127.0.0.50"
TARGET = ("127.0.0.1", LISTEN_PORT)
B_LIDAR, B_FRONT, B_REAR = 1, 16, 32
SIDE_BITS = B_LIDAR | B_FRONT | B_REAR

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SRC_IP, 0))
sm = messaging.SubMaster(["amapNavi", "carState", "modelV2"])


def wait_valid(timeout=20.0):
  t0 = time.time()
  while time.time() - t0 < timeout:
    sm.update(100)
    if sm.valid["amapNavi"] and sm.valid["carState"] and sm.valid["modelV2"]:
      return True
  return False


def read():
  sm.update(0)
  a = sm["amapNavi"] if sm.valid["amapNavi"] else None
  cs = sm["carState"] if sm.valid["carState"] else None
  return (a.leftBlind if a else None, a.rightBlind if a else None,
          (cs.vEgo if cs else 0.0), (cs.standstill if cs else None))


def send_lb(drel, xrel=1200):
  msg = {"device": "lidar", "resp": "blindspot", "lidar_id": 0, "detect_side": 1,
         "port": 4210, "dist_time": int(time.time() * 1000),
         "lidar_lblind": False, "lidar_rblind": False, "lidar_lfblind": False,
         "lidar_rfblind": False, "lidar_lbblind": False, "lidar_rbblind": False,
         "lb_drel": int(drel), "lb_xrel": int(xrel)}
  sock.sendto(json.dumps(msg).encode(), TARGET)


def main():
  if not wait_valid():
    print("[失败] 等不到 amapNavi/carState 数据（openpilot 没在跑？）")
    return
  l, r, v, ss = read()
  print("环境: vEgo=%.2f m/s standstill=%s leftBlind=%d rightBlind=%d" % (v, ss, l, r))
  if sm.valid["modelV2"]:
    meta = sm["modelV2"].meta
    try:
      from openpilot.selfdrive.carrot.amapnavi.amap_navi import side_region_limit_mm
      lim_l = side_region_limit_mm(getattr(meta, 'laneWidthLeft', 0.0)) / 1000.0
      lim_r = side_region_limit_mm(getattr(meta, 'laneWidthRight', 0.0)) / 1000.0
      lim_txt = "  新判定区域上限: 左 %.2fm 右 %.2fm" % (lim_l, lim_r)
    except Exception as e:
      lim_txt = "  (上限计算失败: %s)" % e
    print("       laneWidthLeft=%.2fm laneWidthRight=%.2fm%s" % (
      meta.laneWidthLeft, meta.laneWidthRight, lim_txt))
  if ss:
    print("[注意] standstill=True：动态盲区会把四角强制清零，本测试结论无效")

  # ---------------- 1) 后车快靠近：-40m → -6m，6m/s ----------------
  print("\n[1] 左后车以 6m/s 从 40m 逼近（xrel=1.2m）")
  first = None
  t0 = time.time()
  while True:
    t = time.time() - t0
    d = -40000 + min(34000, 6000 * t)
    if d >= -6000.0:
      break
    send_lb(d)
    lb, _rb, _v, _ss = read()
    if lb is not None and (lb & B_REAR) and first is None:
      first = (-d / 1000.0, t)
    if abs(t - round(t)) < 0.02:
      print("   t=%4.1fs d=%6.1fm leftBlind=%-3d%s" % (
        t, -d / 1000.0, lb if lb is not None else -1,
        "  <- 首次判定侧后" if (first and abs(first[0] - -d / 1000.0) < 0.6) else ""))
    time.sleep(0.1)
  print("   >> 首次判定侧后: %s" % (
    ("%.1fm (%.1fs)" % first) if first else "未判定 [FAIL]"))

  # ---------------- 2) 邻道车横向 3.3m 不应被屏蔽 ----------------
  print("\n[2] 左后 6m + 横向 3.3m（典型邻道车横向距离）")
  ok = False
  t0 = time.time()
  while time.time() - t0 < 3.0:
    send_lb(-6000, xrel=3300)
    lb, _rb, _v, _ss = read()
    if lb is not None and (lb & B_REAR):
      ok = True
    time.sleep(0.1)
  lb, _rb, _v, _ss = read()
  print("   leftBlind=%d -> %s" % (lb, "未被屏蔽 [PASS]" if ok else "被屏蔽或未判定 [FAIL]"))

  # ---------------- 2c) 反证：超宽目标必须被屏蔽 ----------------
  print("\n[2c] 左后 6m + 横向 6.0m（明显超出邻道 → 应被动态屏蔽）")
  t0 = time.time()
  while time.time() - t0 < 3.0:
    send_lb(-6000, xrel=6000)
    time.sleep(0.1)
  lb, _rb, _v, _ss = read()
  print("   leftBlind=%d -> %s" % (
    lb, "已屏蔽 [PASS]" if not (lb & B_REAR) else "未屏蔽 [FAIL]"))

  # ---------------- 3) 停止上报 → 标志必须清零 ----------------
  print("\n[3] 停止上报（模拟设备掉线/无目标）")
  t0 = time.time()
  cleared_at = None
  while time.time() - t0 < 3.0:
    lb, _rb, _v, _ss = read()
    if lb is not None and not (lb & SIDE_BITS) and cleared_at is None:
      cleared_at = time.time() - t0
    time.sleep(0.1)
  lb, _rb, _v, _ss = read()
  print("   leftBlind=%d -> %s" % (
    lb, ("%.1fs 内清零 [PASS]" % cleared_at) if cleared_at is not None else "仍未清零 [FAIL]"))


if __name__ == "__main__":
  main()
