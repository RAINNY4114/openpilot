#!/usr/bin/env python3
"""超车相关判定的多场景仿真（对运行中的 amap_navi 服务发合成雷达报文）。

超车（App 端决策）依赖 CP 下发的这些字段，本测试用合成的外挂雷达数据
逐场景校验它们是否正确::

  leftBlind / rightBlind 位定义（与 amap_navi 发布端一致）
    bit0(1)  外挂雷达侧方盲区
    bit1(2)  摄像头盲区
    bit2(4)  车身盲区（雷达判定目标进入本车侧面）
    bit3(8)  车道线（实线）阻止变道
    bit4(16) 侧前方有目标
    bit5(32) 侧后方有目标
  l_front_blind / r_front_blind  原车前雷达的「前侧盲区」
  lf/lb/rf/rb Drel(+Valid)       四角距离

用法（设备本机）::

  cd /data/openpilot
  PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \\
    python3 -m openpilot.selfdrive.carrot.amapnavi.tests.test_overtake_scenarios

注意：本机若已接真实外挂设备，它们的盲区会与本仿真的结果做 OR 合并，
因此"无目标"类断言会在检测到真实设备时自动跳过（只做正向断言）。
"""

import argparse
import sys
import time

from openpilot.selfdrive.carrot.amapnavi.tests.simulate_devices import (
  BROADCAST_PORT,
  LISTEN_PORT,
  AmapNaviWatcher,
  VirtualDevice,
  app_heartbeat,
)

# 位定义
B_LIDAR = 1
B_CAMERA = 2
B_BODY = 4
B_SOLID = 8
B_FRONT = 16
B_REAR = 32

HOLD_S = 3.0  # 每个场景保持时间（客户端超时 2s，留足余量）


def send_lidar(dev, side: int, port: int, *, side_blind=False, front=False, rear=False,
               drel_f=35000, drel_b=-12000, xrel=1200, lidar_id=0):
  """side: 1=左 2=右。前/后/侧方盲区可任意组合，并带四角距离。

  ``lidar_id`` 语义见 protocol._update_main_targets：
  0=前后角都报（本仿真默认），1=只报前角，2=只报后角。
  """
  now_ms = int(time.time() * 1000)
  left = side == 1
  msg = {
    "device": "lidar", "resp": "blindspot", "lidar_id": lidar_id,
    "detect_side": side, "port": port, "dist_time": now_ms,
    "lidar_lblind": bool(left and side_blind),
    "lidar_rblind": bool((not left) and side_blind),
    "lidar_lfblind": bool(left and front),
    "lidar_rfblind": bool((not left) and front),
    "lidar_lbblind": bool(left and rear),
    "lidar_rbblind": bool((not left) and rear),
  }
  if left:
    msg.update(lf_drel=drel_f, lf_xrel=xrel, lb_drel=drel_b, lb_xrel=xrel)
  else:
    msg.update(rf_drel=drel_f, rf_xrel=xrel, rb_drel=drel_b, rb_xrel=xrel)
  dev.send(LISTEN_PORT, **msg)


class Report:
  def __init__(self):
    self.rows = []
    self.skipped = 0

  def check(self, name, ok, detail=""):
    self.rows.append((bool(ok), name, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

  def skip(self, name, why=""):
    """真实外挂设备在线时无法成立的断言（结果会与真实设备取 OR），直接跳过。"""
    self.skipped += 1
    print(f"  [SKIP] {name}" + (f"  ({why})" if why else ""))

  def summary(self):
    bad = [r for r in self.rows if not r[0]]
    print("")
    print(f"===== 结果: {len(self.rows) - len(bad)}/{len(self.rows)} 通过"
          + (f"，跳过 {self.skipped} 项（真实外挂设备在线）" if self.skipped else "") + " =====")
    for _ok, name, detail in bad:
      print(f"  FAILED: {name} {detail}")
    return 1 if bad else 0


def run_scenario(watcher, app, lidar, seconds, fn):
  """持续发一种数据 seconds 秒，返回观察窗口内的字段采样。"""
  samples = []
  end = time.time() + seconds
  while time.time() < end:
    app_heartbeat(app, app.port)
    fn()
    if watcher.alive:
      samples.append(dict(watcher.state))
    time.sleep(0.1)
  return samples


def last(samples, key, default=0):
  return samples[-1].get(key, default) if samples else default


def main() -> int:
  parser = argparse.ArgumentParser(description="超车判定多场景仿真")
  parser.add_argument("--target", default="127.0.0.1", help="目标 CP 地址")
  parser.add_argument("--hold", type=float, default=HOLD_S, help="每个场景保持秒数")
  args = parser.parse_args()

  import openpilot.selfdrive.carrot.amapnavi.tests.simulate_devices as sd
  sd.TARGET_IP = args.target

  report = Report()
  app = VirtualDevice("app/overtake", "0.0.0.0", BROADCAST_PORT)
  lidar_l = VirtualDevice("lidar/left", "127.0.0.10")
  lidar_r = VirtualDevice("lidar/right", "127.0.0.11")
  watcher = AmapNaviWatcher()

  # ---------- 基线：先看看本机有没有真实外挂设备 ----------
  print("=== 基线：不发外挂雷达数据 ===")
  base_samples = []
  end = time.time() + 4.0
  while time.time() < end:
    app_heartbeat(app, app.port)
    if watcher.alive:
      base_samples.append(dict(watcher.state))
    time.sleep(0.1)
  base_left = last(base_samples, "leftBlind")
  base_right = last(base_samples, "rightBlind")
  real_device = bool((base_left | base_right) & (B_LIDAR | B_CAMERA))
  print(f"  基线 leftBlind={base_left} rightBlind={base_right}"
        f"{'（检测到真实外挂设备，无目标类断言将跳过）' if real_device else ''}")

  try:
    # ---------- 场景1：左侧前方有目标 ----------
    # 注意：动态模式(DynamicBlindRange=2)下设备自报的 lidar_*blind 会被 CP 自己的
    # 引擎结果替代，所以这里必须给出"真能判危险"的距离——当前参数前方危险距离是
    # 绝对 5m(LidarFrontVDistTime=-50)，8m 的静止目标不算危险，用 4m。
    print("=== 场景1：左前(侧前方)有目标 → 应阻左侧超车 ===")
    s = run_scenario(watcher, app, lidar_l, args.hold,
                     lambda: send_lidar(lidar_l, 1, app.port, side_blind=True, front=True, drel_f=4000))
    lb = last(s, "leftBlind")
    report.check("leftBlind 含 侧方(bit0)", bool(lb & B_LIDAR), f"leftBlind={lb}")
    report.check("leftBlind 含 侧前(bit4)", bool(lb & B_FRONT), f"leftBlind={lb}")
    report.check("左前距离已回填", last(s, "lfDrel") not in (None, 0) and last(s, "lfDrelValid") == 1,
                 f"lfDrel={last(s, 'lfDrel')} valid={last(s, 'lfDrelValid')}")
    if real_device:
      report.skip("右侧不受影响", "有真实外挂设备")
    else:
      report.check("右侧不受影响", not (last(s, "rightBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                   f"rightBlind={last(s, 'rightBlind')}")

    # ---------- 场景2：左侧后方有目标 ----------
    print("=== 场景2：左后有目标 → 应阻左侧超车 ===")
    s = run_scenario(watcher, app, lidar_l, args.hold,
                     lambda: send_lidar(lidar_l, 1, app.port, side_blind=True, rear=True, drel_b=-6000))
    lb = last(s, "leftBlind")
    report.check("leftBlind 含 侧后(bit5)", bool(lb & B_REAR), f"leftBlind={lb}")

    # ---------- 场景3：右侧前方有目标 ----------
    print("=== 场景3：右前有目标 → 应阻右侧超车、左侧放行 ===")
    s = run_scenario(watcher, app, lidar_r, args.hold,
                     lambda: send_lidar(lidar_r, 2, app.port, side_blind=True, front=True, drel_f=4000))
    rb = last(s, "rightBlind")
    report.check("rightBlind 含 侧方+侧前", bool(rb & B_LIDAR) and bool(rb & B_FRONT), f"rightBlind={rb}")
    report.check("右前距离已回填", last(s, "rfDrel") not in (None, 0) and last(s, "rfDrelValid") == 1,
                 f"rfDrel={last(s, 'rfDrel')} valid={last(s, 'rfDrelValid')}")
    if real_device:
      report.skip("左侧已清空(bit0/4/5 全无)", "有真实外挂设备")
    else:
      report.check("左侧已清空(bit0/4/5 全无)", not (last(s, "leftBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                   f"leftBlind={last(s, 'leftBlind')}")

    # ---------- 场景4：两侧同时有目标 ----------
    print("=== 场景4：两侧同时有目标 → 双向都应阻止 ===")
    s = run_scenario(watcher, app, lidar_l, args.hold, lambda: (
      send_lidar(lidar_l, 1, app.port, side_blind=True, front=True, drel_f=4000),
      send_lidar(lidar_r, 2, app.port, side_blind=True, rear=True, drel_b=-5000),
    ))
    report.check("leftBlind 有盲区", bool(last(s, "leftBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                 f"leftBlind={last(s, 'leftBlind')}")
    report.check("rightBlind 有盲区", bool(last(s, "rightBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                 f"rightBlind={last(s, 'rightBlind')}")

    # ---------- 场景6：邻道车横向距离不应被动态盲区误屏蔽 ----------
    # 判定区域 = 1.5×侧面车道宽（夹 3.5~5.5m）。城市里车道宽读数常只有 2.4~3.0m，
    # 旧实现直接拿 1 倍车道宽当上限，邻道车(横向 3.0~3.5m)会被全部屏蔽。
    print("=== 场景6：左后 6m + 横向 3.3m（典型邻道车）→ 不应被屏蔽 ===")
    s = run_scenario(watcher, app, lidar_l, 5.0,
                     lambda: send_lidar(lidar_l, 1, app.port, side_blind=True, rear=True,
                                        drel_b=-6000, xrel=3300))
    report.check("横向3.3m 仍判侧后盲区", bool(last(s, "leftBlind") & B_REAR),
                 f"leftBlind={last(s, 'leftBlind')}")

    # ---------- 场景6b：明显超宽的目标应被屏蔽（反证） ----------
    print("=== 场景6b：左后 6m + 横向 6.0m（超宽）→ 应被动态屏蔽 ===")
    s = run_scenario(watcher, app, lidar_l, 4.0,
                     lambda: send_lidar(lidar_l, 1, app.port, side_blind=True, rear=True,
                                        drel_b=-6000, xrel=6000))
    report.check("横向6.0m 被屏蔽", not (last(s, "leftBlind") & B_REAR),
                 f"leftBlind={last(s, 'leftBlind')}")

    # ---------- 场景7：相对速度时距（不是绝对距离）也要能判危险 ----------
    # 后车 8m/s 逼近，2.5s 只从 40m 走到 20m：光靠绝对 10m 判不出来，
    # 必须靠 LidarBehindVRelDistTime(4s) 的外推：d - 8*4 < 10 → d < 42m。
    print("=== 场景7：后车 8m/s 逼近（40m→20m）→ 相对速度时距应判危险 ===")
    st = {"d": -40000.0}

    def approach():
      st["d"] = max(-20000.0, st["d"] + 800)     # 每帧 100ms → 8 m/s
      send_lidar(lidar_l, 1, app.port, drel_b=int(st["d"]))

    s = run_scenario(watcher, app, lidar_l, 2.5, approach)
    got = any((x.get("leftBlind", 0) & B_REAR) for x in s)
    report.check("相对速度时距判出侧后危险", got,
                 f"最远只到 {st['d'] / 1000:.1f}m, leftBlind={last(s, 'leftBlind')}")

    # ---------- 场景5：停止上报 → 全部清空 ----------
    print("=== 场景5：停止上报 → 盲区与距离应清空 ===")
    end = time.time() + 8.0
    while time.time() < end:
      app_heartbeat(app, app.port)
      time.sleep(0.1)
    s = [dict(watcher.state)]
    # 停止上报后是否清空，只有在没有真实外挂设备时才能断言：真实设备还在报数的话，
    # 它们的盲区/距离会继续与仿真结果取 OR（基线里已能看到）。
    if real_device:
      report.skip("leftBlind 已清空", "有真实外挂设备")
      report.skip("rightBlind 已清空", "有真实外挂设备")
      report.skip("距离 valid 已复位", "有真实外挂设备")
    else:
      report.check("leftBlind 已清空", not (last(s, "leftBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                   f"leftBlind={last(s, 'leftBlind')}")
      report.check("rightBlind 已清空", not (last(s, "rightBlind") & (B_LIDAR | B_FRONT | B_REAR)),
                   f"rightBlind={last(s, 'rightBlind')}")
      report.check("距离 valid 已复位", last(s, "lfDrelValid") == 0 and last(s, "rfDrelValid") == 0,
                   f"lfValid={last(s, 'lfDrelValid')} rfValid={last(s, 'rfDrelValid')}")

  finally:
    for d in (app, lidar_l, lidar_r):
      d.close()
    watcher.close()

  return report.summary()


if __name__ == "__main__":
  sys.exit(main())
