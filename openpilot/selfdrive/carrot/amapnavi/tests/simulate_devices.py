#!/usr/bin/env python3
"""amapnavi 多端通讯仿真（可作为协议回归测试）。

按真实协议模拟 **App / 左侧雷达 / 右侧雷达 / 盲区摄像头 / 转向灯板**，
与本机 ``amap_navi`` 和 ``lane`` 服务通讯，并校验 C3 侧的响应。

端口与字段直接引用 :mod:`..transport` 里的真实常量，避免测试和实现各写一份。
仿真的字段格式对照 AmapNavi App 源码
(``KeepAliveService.kt`` / ``OvertakeCompat.kt`` / ``MainActivityLaneControl.kt``)：

  App  -> CP(4211) : {"resp":"heartbeat","device":"overtake","ip":..,"port":4210}
                     {"ctrlid":N,"ctrl":"blinker","state":"LEFT"}   ← 大写
  雷达 -> CP(4211) : {"device":"lidar","resp":"blindspot","lidar_id":..,
                      "detect_side":..,"dist_time":ms,"lf_drel":..,"lidar_lblind":..}
  摄像头->CP(4211) : {"device":"camera","resp":"cam_blind","left_blind":..,"right_blind":..}
  App  -> CP(4213) : {"resp":"lane","port":4212,"left_lane":..,"right_lane":..}

用法（在设备本机运行，用 127.0.0.x 作为多个虚拟设备的源地址）：

  cd /data/openpilot
  PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \\
    python3 -m openpilot.selfdrive.carrot.amapnavi.tests.simulate_devices

参数：
  --target IP     目标 CP 地址（默认 127.0.0.1）
  --no-cereal     不订阅 amapNavi，只做 UDP 层校验
  --duration SEC  每个阶段的时长（默认 4.0 秒）

退出码：全部检查通过返回 0，有失败返回 1。

注意：运行期间会向 amap_navi 注册若干虚拟设备（约 15 秒），
真实外挂硬件的数据不会被破坏，但同一时刻 ``ext_state`` 会包含虚拟设备数量。
"""
import argparse
import json
import socket
import sys
import threading
import time

from openpilot.selfdrive.carrot.amapnavi.transport import (
  BROADCAST_PORT,      # 4210  App 侧 overtake 兼容端口 / CP 广播目标
  LANE_PORT,           # 4213  lane 服务监听
  LANE_REMOTE_PORT,    # 4212  App 侧 lane 端口
  LISTEN_PORT,         # 4211  amap_navi 监听
  NAVI_REMOTE_PORT,    # 7705  App 侧 carrotMan 主通道
)

# 虚拟设备的源 IP（Linux 上 127.0.0.0/8 全部可用）
APP_IP = "0.0.0.0"        # App 绑 0.0.0.0 才能收到 255.255.255.255 的广播
LIDAR_LEFT_IP = "127.0.0.10"
LIDAR_RIGHT_IP = "127.0.0.11"
CAMERA_IP = "127.0.0.20"
BOARD_IP = "127.0.0.30"
BOARD_PORT = 4230

TARGET_IP = "127.0.0.1"


# ------------------------------------------------------------------ 工具
class VirtualDevice:
  """一个虚拟 UDP 端点：可发可收，收到的报文存在 self.rx 里。"""

  def __init__(self, name, ip, port=0):
    self.name = name
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.sock.bind((ip, port))
    self.sock.settimeout(0.2)
    self.port = self.sock.getsockname()[1]
    self.rx = []
    self._stop = threading.Event()
    threading.Thread(target=self._rx_loop, daemon=True).start()

  def _rx_loop(self):
    while not self._stop.is_set():
      try:
        data, _addr = self.sock.recvfrom(8192)
      except socket.timeout:
        continue
      except OSError:
        break
      try:
        self.rx.append(json.loads(data.decode(errors="replace")))
      except Exception:
        pass

  def send(self, dest_port, **kw):
    try:
      self.sock.sendto(json.dumps(kw).encode(), (TARGET_IP, dest_port))
    except OSError as e:
      print(f"  [{self.name}] 发送失败: {e}")

  def messages(self, **match):
    """筛出同时满足所有字段的报文"""
    out = []
    for m in self.rx:
      if all(m.get(k) == v for k, v in match.items()):
        out.append(m)
    return out

  def close(self):
    self._stop.set()
    try:
      self.sock.close()
    except OSError:
      pass


class AmapNaviWatcher:
  """订阅 amapNavi 消息，保存最新一帧的字段。"""

  FIELDS = ("leftBlind", "rightBlind", "leftLine", "rightLine", "lineValid",
            "leftDevice", "rightDevice", "lfDrel", "lbDrel", "rfDrel", "rbDrel",
            "lfDrelValid", "lbDrelValid", "rfDrelValid", "rbDrelValid", "extState",
            # 侧向标志拆成独立字段后新增（位图字段仅兼容保留）
            "blindLidarL", "blindLidarLf", "blindLidarLb", "blindCombinedL", "blindCarL", "laneBlindL",
            "blindLidarR", "blindLidarRf", "blindLidarRb", "blindCombinedR", "blindCarR", "laneBlindR",
            "extBlinker")

  def __init__(self):
    import openpilot.cereal.messaging as messaging

    self.sm = messaging.SubMaster(['amapNavi'])
    self.state = {}
    self.alive = False
    self._stop = threading.Event()
    threading.Thread(target=self._loop, daemon=True).start()

  def _loop(self):
    while not self._stop.is_set():
      self.sm.update(100)
      self.alive = bool(self.sm.alive['amapNavi'])
      if self.alive:
        m = self.sm['amapNavi']
        self.state = {f: getattr(m, f) for f in self.FIELDS}
      time.sleep(0.02)

  def close(self):
    self._stop.set()


class Report:
  def __init__(self):
    self.rows = []

  def check(self, name, ok, detail=""):
    self.rows.append((bool(ok), name, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

  @property
  def failed(self):
    return [r for r in self.rows if not r[0]]

  def summary(self):
    total = len(self.rows)
    bad = len(self.failed)
    print("")
    print(f"===== 结果: {total - bad}/{total} 通过 =====")
    if bad:
      for _ok, name, detail in self.failed:
        print(f"  FAILED: {name} {detail}")
    return 1 if bad else 0


# ------------------------------------------------------------------ 仿真报文
def app_heartbeat(app, overtake_port):
  app.send(LISTEN_PORT, resp="heartbeat", device="overtake", ip=TARGET_IP,
           port=overtake_port, timestamp=int(time.time()))
  app.send(LISTEN_PORT, resp="overtake", device="overtake", ip=TARGET_IP,
           port=overtake_port, timestamp=int(time.time()))


def lidar_report(dev, side, lidar_id, port):
  """side: 1=左 2=右"""
  now_ms = int(time.time() * 1000)
  msg = {
    "device": "lidar", "resp": "blindspot", "lidar_id": lidar_id,
    "detect_side": side, "port": port, "dist_time": now_ms,
    "lidar_lblind": side == 1, "lidar_rblind": side == 2,
    "lidar_lfblind": False, "lidar_lbblind": side == 1,
    "lidar_rfblind": False, "lidar_rbblind": False,
  }
  if side == 1:
    msg.update(lf_drel=35000, lf_xrel=1500, lb_drel=-12000, lb_xrel=1200)
  else:
    msg.update(rf_drel=30000, rf_xrel=1400, rb_drel=-15000, rb_xrel=1300)
  dev.send(LISTEN_PORT, **msg)


def camera_report(dev, port):
  dev.send(LISTEN_PORT, device="camera", resp="cam_blind", detect_side=3,
           port=port, left_blind=True, right_blind=True)


def blinker_command(dev):
  """App 下发的转向灯命令（注意是大写 LEFT）"""
  dev.send(LISTEN_PORT, ctrlid=7, ctrl="blinker", state="LEFT",
           port=BOARD_PORT, device="board")


def lane_query(dev):
  dev.send(LANE_PORT, resp="lane", port=LANE_REMOTE_PORT,
           left_lane=1, right_lane=2)


# ------------------------------------------------------------------ 主流程
def main():
  global TARGET_IP

  parser = argparse.ArgumentParser(description="amapnavi 多端通讯仿真")
  parser.add_argument("--target", default="127.0.0.1", help="目标 CP 地址")
  parser.add_argument("--no-cereal", action="store_true", help="不订阅 amapNavi")
  parser.add_argument("--duration", type=float, default=4.0, help="每阶段时长(秒)")
  args = parser.parse_args()
  TARGET_IP = args.target

  report = Report()

  # 虚拟设备
  app_overtake = VirtualDevice("app/overtake", APP_IP, BROADCAST_PORT)
  app_lane = VirtualDevice("app/lane", APP_IP, LANE_REMOTE_PORT)
  app_cm = VirtualDevice("app/carrotman", APP_IP, NAVI_REMOTE_PORT)
  lidar_l = VirtualDevice("lidar/left", LIDAR_LEFT_IP)
  lidar_r = VirtualDevice("lidar/right", LIDAR_RIGHT_IP)
  camera = VirtualDevice("camera", CAMERA_IP)
  board = VirtualDevice("board", BOARD_IP, BOARD_PORT)
  devices = [app_overtake, app_lane, app_cm, lidar_l, lidar_r, camera, board]

  watcher = None
  if not args.no_cereal:
    try:
      watcher = AmapNaviWatcher()
    except Exception as e:
      print(f"  跳过 amapNavi 订阅（cereal 不可用）: {e}")

  # 基线：本机可能已经接了真实的外挂设备（App / 雷达 / 摄像头），
  # 所以"掉线清理"类断言要相对基线判断，而不是绝对等于 0。
  base = {"extState": 0, "leftDevice": 0, "rightDevice": 0}
  if watcher is not None:
    time.sleep(1.5)
    for key in base:
      value = watcher.state.get(key)
      if value is not None:
        base[key] = value
    print(f"  基线（注册虚拟设备之前）: {base}")

  def loop(seconds, fn):
    end = time.time() + seconds
    while time.time() < end:
      fn()
      time.sleep(0.1)

  try:
    # ---- 阶段1: App 注册 ----
    print(f"=== 阶段1: App 注册（overtake 心跳）{args.duration}s ===")
    loop(args.duration, lambda: app_heartbeat(app_overtake, app_overtake.port))

    op_msgs = app_overtake.messages(device="op")
    ips = {m.get("ip") for m in op_msgs}
    report.check("CP 广播到 App:4210", bool(op_msgs),
                 f"{len(op_msgs)} 包")
    report.check("广播报文 ip 有效(非 0.0.0.0)",
                 bool(ips) and "0.0.0.0" not in ips, f"ip={ips or '无'}")

    # ---- 阶段2: 雷达 + 摄像头 + 转向灯 + 车道线 ----
    print(f"=== 阶段2: 雷达/摄像头/转向灯板/车道线 {args.duration}s ===")

    def phase2():
      app_heartbeat(app_overtake, app_overtake.port)
      lidar_report(lidar_l, 1, 1, app_overtake.port)
      lidar_report(lidar_r, 2, 2, app_overtake.port)
      camera_report(camera, app_overtake.port)
      blinker_command(board)
      lane_query(app_lane)

    loop(args.duration, phase2)

    lane_msgs = app_lane.messages(device="lane")
    lane_ips = {m.get("ip") for m in lane_msgs}
    report.check("lane 广播到 App:4212", bool(lane_msgs), f"{len(lane_msgs)} 包")
    report.check("lane 广播 ip 有效",
                 bool(lane_ips) and "0.0.0.0" not in lane_ips, f"ip={lane_ips or '无'}")

    blink = {m.get("blinker") for m in board.rx if "blinker" in m}
    report.check("转向灯板收到大写 LEFT 对应的下发",
                 "left" in blink, f"blinker 取值={blink or '无'}")

    if watcher is not None:
      st = dict(watcher.state)
      report.check("amapNavi 消息可用", watcher.alive and bool(st))
      report.check("左右雷达+摄像头同时注册 (leftDevice/rightDevice=3)",
                   st.get("leftDevice") == 3 and st.get("rightDevice") == 3,
                   f"leftDevice={st.get('leftDevice')} rightDevice={st.get('rightDevice')}")
      # 测试里摄像头报 left_blind/right_blind=True，对应综合盲区位(bit1)
      report.check("综合盲区位(bit1)已置位",
                   bool(st.get("leftBlind", 0) & 2) and bool(st.get("rightBlind", 0) & 2),
                   f"leftBlind={st.get('leftBlind')} rightBlind={st.get('rightBlind')}")

      # 侧向标志拆成独立字段后：按位序组装的结果必须与位图字段**完全一致**
      #   位序 1=激光侧方 2=综合 4=车身 8=实线 16=前角 32=后角
      def _assemble(side):
        s = side
        return ((1 if st.get(f"blindLidar{s}") else 0) | (2 if st.get(f"blindCombined{s}") else 0) |
                (4 if st.get(f"blindCar{s}") else 0) | (8 if st.get(f"laneBlind{s}") else 0) |
                (16 if st.get(f"blindLidar{s}f") else 0) | (32 if st.get(f"blindLidar{s}b") else 0))

      report.check("独立字段组装后与位图一致(左)", _assemble("L") == st.get("leftBlind", -1),
                   f"组装={_assemble('L')} 位图={st.get('leftBlind')}")
      report.check("独立字段组装后与位图一致(右)", _assemble("R") == st.get("rightBlind", -1),
                   f"组装={_assemble('R')} 位图={st.get('rightBlind')}")
      report.check("距离字段回填",
                   st.get("lfDrel") not in (None, 0) and st.get("lfDrelValid") == 1,
                   f"lfDrel={st.get('lfDrel')} valid={st.get('lfDrelValid')}")
      report.check("客户端数量(extState) 随注册增加",
                   st.get("extState", 0) >= 1, f"extState={st.get('extState')}")

    # ---- 阶段2.5: 只发 App 心跳，间隔 1.1s（实测 AmapNavi App 的真实节奏）----
    # 客户端超时如果和心跳周期同量级，客户端会被反复清掉再重新注册，
    # 表现为外挂客户端数量在 1/2 之间来回跳（旧代码超时 1.0s 时的现象）。
    print("=== 阶段2.5: App 心跳 1.1s 间隔，客户端不应被超时清掉 ===")
    min_ext = None
    end = time.time() + 8.0
    next_hb = 0.0
    while time.time() < end:
      if time.time() >= next_hb:
        app_heartbeat(app_overtake, app_overtake.port)
        next_hb = time.time() + 1.1
      if watcher is not None:
        ext = watcher.state.get("extState")
        if ext is not None:
          min_ext = ext if min_ext is None else min(min_ext, ext)
      time.sleep(0.05)
    if watcher is not None:
      # 虚拟 App 全程在线，数量至少应为 基线 + 1；旧代码（超时 1.0s）会周期性掉回基线
      report.check("App 心跳 1.1s 期间客户端不超时 (extState 不低于 基线+1)",
                   min_ext is not None and min_ext >= base["extState"] + 1,
                   f"基线={base['extState']} 窗口内最小 extState={min_ext}")

    # ---- 阶段3: 停止上报，等待超时清理 ----
    # 雷达/摄像头超时 2s，心跳型客户端 5s，lane 服务 3s，留足余量后再检查
    print("=== 阶段3: 停止上报，等待客户端超时(7s) ===")
    time.sleep(7.0)
    if watcher is not None:
      st = dict(watcher.state)
      report.check(f"客户端超时后被清理 (leftDevice 回到基线 {base['leftDevice']})",
                   st.get("leftDevice") == base["leftDevice"],
                   f"leftDevice={st.get('leftDevice')} 基线={base['leftDevice']}")
      report.check(f"客户端数量(extState) 回到基线 {base['extState']}",
                   st.get("extState", -1) == base["extState"],
                   f"extState={st.get('extState')} 基线={base['extState']}")

  finally:
    for d in devices:
      d.close()
    if watcher is not None:
      watcher.close()

  return report.summary()


if __name__ == "__main__":
  sys.exit(main())
