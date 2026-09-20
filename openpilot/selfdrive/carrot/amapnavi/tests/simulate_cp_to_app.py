#!/usr/bin/env python3
"""模拟 CP（amap_navi）→ AmapNavi App 的 7705 业务数据帧。

用途：在没有 C3 的情况下验证 App 超车页（activity_main 的 5 张雷达卡片）显示是否正确。

字段与 :meth:`openpilot.selfdrive.carrot.amapnavi.messages.NaviMessageBuilder.build_navi`
的产物一致；App 侧字段表见 ``OpState.kt`` 的 ``OP_FIELD_TABLE``
（缺失的键在 500ms 后被复位，所以每个场景都要把该场景里的键显式发全）。

用法::

  # 跑全部场景，每个场景保持 6s，并用 adb 自动读回卡片文字做校验
  python3 simulate_cp_to_app.py --ip 192.168.1.14

  # 只跑某个场景，保持 20 秒（方便肉眼看手机）
  python3 simulate_cp_to_app.py --ip 192.168.1.14 --scenario bug_right_rear_blind_only --hold 20

  # 只发包，不做 adb 校验
  python3 simulate_cp_to_app.py --ip 192.168.1.14 --no-adb
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time

try:
  sys.stdout.reconfigure(encoding='utf-8')
except Exception:
  pass

APP_PORT = 7705          # transport.NAVI_REMOTE_PORT: App 的 carrotMan 主通道
CP_PORT = 4211           # transport.LISTEN_PORT: CP 自己的监听端口(只作为回包目标)

ADB_CANDIDATES = (
  r"C:\Android\Sdk\platform-tools\adb.exe",
  os.path.expandvars(r"%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"),
  "adb",
)

# 界面上 10 个卡片的 id（布局见 App 的 activity_main.xml）
CARD_IDS = ("tvRadarMmwLeft", "tvRadarLidarLeftFront", "tvRadarLidarLeftRear",
            "tvRadarFront", "tvRadarEgo",
            "tvRadarMmwRight", "tvRadarLidarRightFront", "tvRadarLidarRightRear",
            "tvRadarLeftBlock", "tvRadarRightBlock")

# ---------------------------------------------------------------- 场景表
#   payload   : 除固定信封外的键(键名与 NaviMessageBuilder.build_navi 一致)
#   expect    : 卡片期望文本(子串匹配；str 或 list；缺省表示不断言)
#   not_expect: 断言不应出现的文本
#   卡片对应：左列=原车毫米波(tvRadarMmwLeft)/激光前(tvRadarLidarLeftFront)/激光后(tvRadarLidarLeftRear)
#             中列=正前车(tvRadarFront)、本车(tvRadarEgo)
#             右列同左列；第 4 行=是否禁止变道(tvRadarLeftBlock/tvRadarRightBlock)
SCENARIOS = [
  dict(
    name="clear", desc="全清: 全部无目标, 左右都允许变道",
    payload={},
    expect={"tvRadarFront": "无目标", "tvRadarMmwLeft": "无目标",
            "tvRadarLidarLeftFront": "无目标", "tvRadarLidarLeftRear": "无目标",
            "tvRadarMmwRight": "无目标", "tvRadarLidarRightFront": "无目标",
            "tvRadarLidarRightRear": "无目标",
            "tvRadarLeftBlock": "可左变道", "tvRadarRightBlock": "可右变道"},
  ),
  dict(
    name="front", desc="正前车卡片: 前车/距离/速度/速差 四行",
    payload={"lead1": True, "drel": 25, "vlead": 40, "vrel": -5},
    expect={"tvRadarFront": ["前车", "25.0m", "40km/h", "(-5km/h)"]},
  ),
  dict(
    name="left_front", desc="左列: 原车毫米波 + 激光前角 各自一张卡片",
    payload={"l_lead": True, "l_drel": 18, "l_vlead": 36, "l_vrel": -2,
             "lf_drel": 15000, "lf_xrel": 1600, "lf_vrel": 0},
    expect={"tvRadarMmwLeft": "18.0m", "tvRadarLidarLeftFront": ["15.0m", "/1.6m"],
            "tvRadarMmwRight": "无目标"},
  ),
  dict(
    name="right_front", desc="右列: 原车毫米波 + 激光前角",
    payload={"r_lead": True, "r_drel": 22, "r_vlead": 30, "r_vrel": 2,
             "rf_drel": 20000, "rf_xrel": -1500, "rf_vrel": -1},
    expect={"tvRadarMmwRight": "22.0m", "tvRadarLidarRightFront": ["20.0m", "/1.5m"]},
  ),
  dict(
    name="left_front_lidar_only", desc="左列: 只有激光前角(毫米波卡显示无目标)",
    payload={"lf_drel": 16000, "lf_xrel": 1500, "lf_vrel": 0},
    expect={"tvRadarLidarLeftFront": "16.0m", "tvRadarMmwLeft": "无目标"},
  ),
  dict(
    name="right_front_lidar_only", desc="右列: 只有激光前角",
    payload={"rf_drel": 21000, "rf_xrel": -1400, "rf_vrel": 0},
    expect={"tvRadarLidarRightFront": "21.0m", "tvRadarMmwRight": "无目标"},
  ),
  dict(
    name="front_mmw_lidar_both", desc="左右前角两个来源同时有数据(各一张卡片, 不互相挤)",
    payload={"l_lead": True, "l_drel": 15, "l_vlead": 33, "l_vrel": -1,
             "lf_drel": 16000, "lf_xrel": 1500, "lf_vrel": 0,
             "r_lead": True, "r_drel": 20, "r_vlead": 31, "r_vrel": 1,
             "rf_drel": 21000, "rf_xrel": -1400, "rf_vrel": 0},
    expect={"tvRadarMmwLeft": "15.0m", "tvRadarLidarLeftFront": "16.0m",
            "tvRadarMmwRight": "20.0m", "tvRadarLidarRightFront": "21.0m"},
  ),
  dict(
    name="left_rear", desc="左后激光有目标 + 激光后角盲区 -> 第4行红底写明原因",
    payload={"lb_drel": -12000, "lb_xrel": 1300, "lb_vrel": 3, "lidar_lbblind": True},
    expect={"tvRadarLidarLeftRear": ["12.0m", "/1.3m"],
            "tvRadarLeftBlock": ["禁止左变道", "激光盲区·后角"],
            "tvRadarRightBlock": "可右变道"},
  ),
  dict(
    name="right_rear", desc="右后激光有目标 + 激光侧盲区",
    payload={"rb_drel": -9000, "rb_xrel": -1100, "rb_vrel": 5, "lidar_rblind": True},
    expect={"tvRadarLidarRightRear": ["9.0m", "/1.1m"],
            "tvRadarRightBlock": ["禁止右变道", "激光盲区·侧方"]},
  ),
  dict(
    name="right_rear_dist_no_blind", desc="右后激光有目标、无任何盲区 -> 第4行仍是绿灯",
    payload={"rb_drel": -9000, "rb_xrel": -1100, "rb_vrel": 5},
    expect={"tvRadarLidarRightRear": "9.0m", "tvRadarRightBlock": "可右变道"},
    not_expect={"tvRadarRightBlock": "禁止"},
  ),
  dict(
    name="bug_right_rear_blind_only",
    desc="★原问题: 综合盲区(摄像头/实线等)为真但没有右后激光距离 -> 写明真实原因, 不再显示'盲区有车'",
    payload={"blind_r": True},
    # 只给综合标志、没给分解来源 -> 显示"综合盲区"(旧版这里写死成"摄像头盲区")
    expect={"tvRadarRightBlock": ["禁止右变道", "综合盲区"],
            "tvRadarLidarRightRear": "无目标"},
  ),
  dict(
    name="bug_left_rear_blind_only", desc="★同类: 左侧综合盲区为真但无左后激光距离",
    payload={"blind_l": True},
    expect={"tvRadarLeftBlock": ["禁止左变道", "综合盲区"],
            "tvRadarLidarLeftRear": "无目标"},
  ),
  dict(
    name="lidar_rbblind_only", desc="右后激光雷达自身盲区(无距离) -> 禁止右变道(激光盲区·后角)",
    payload={"lidar_rbblind": True},
    expect={"tvRadarRightBlock": ["禁止右变道", "激光盲区·后角"]},
  ),
  dict(
    name="front_blind_only", desc="原车前雷达侧前盲区为真 -> 左右都不能变道(前侧盲区·前雷达)",
    payload={"l_front_blind": True, "r_front_blind": True},
    expect={"tvRadarLeftBlock": ["禁止左变道", "前侧盲区·前雷达"],
            "tvRadarRightBlock": ["禁止右变道", "前侧盲区·前雷达"]},
  ),
  dict(
    name="front_blind_and_lidar", desc="左右前侧盲区 + 右侧综合盲区(查左右是否对称)",
    payload={"l_front_blind": True, "r_front_blind": True, "blind_r": True},
    expect={"tvRadarLeftBlock": "禁止左变道", "tvRadarRightBlock": "禁止右变道"},
  ),
  dict(
    name="blind_lane_only",
    desc="★旧会误显示成摄像头: CP 判为实线但 App 车道线类型不为实线 -> 应显示'车道实线'",
    payload={"blind_l": True, "blind_lane_l": True, "blind_r": True, "blind_lane_r": True},
    expect={"tvRadarLeftBlock": ["禁止左变道", "车道实线"],
            "tvRadarRightBlock": ["禁止右变道", "车道实线"]},
    not_expect={"tvRadarLeftBlock": "摄像头"},
  ),
  dict(
    name="blind_car_only",
    desc="★旧会误显示成摄像头: 车身盲区(目标进入本车侧面) -> 应显示'车身盲区'",
    payload={"blind_l": True, "blind_car_l": True},
    expect={"tvRadarLeftBlock": ["禁止左变道", "车身盲区"]},
    not_expect={"tvRadarLeftBlock": "摄像头"},
  ),
  dict(
    name="blind_camera_only",
    desc="确实有摄像头盲区 -> 仍显示'摄像头盲区'",
    payload={"blind_l": True, "blind_camera_l": True},
    expect={"tvRadarLeftBlock": ["禁止左变道", "摄像头盲区"]},
  ),
  dict(
    name="lidar_rear_approach",
    desc="激光在线 + 侧后车 14m 且以 20km/h 逼近(绝对距离判不出, 靠时距/TTC) -> 禁止左变道",
    payload={"lidar_l": True, "lb_drel": -14000, "lb_xrel": 1300, "lb_vrel": 20},
    expect={"tvRadarLeftBlock": ["禁止左变道", "侧后逼近·激光14.0m"]},
  ),
  dict(
    name="lidar_front_close",
    desc="激光在线 + 侧前车 9m 且本车以 10km/h 逼近 -> 禁止右变道",
    payload={"lidar_r": True, "rf_drel": 9000, "rf_xrel": -1400, "rf_vrel": -10},
    expect={"tvRadarRightBlock": ["禁止右变道", "侧前危险·激光9.0m"]},
  ),
  dict(
    name="lidar_offline_no_new_gate",
    desc="同样的侧后逼近数据但激光离线 -> App 自算判据不启用, 仍是可左变道",
    payload={"lb_drel": -14000, "lb_xrel": 1300, "lb_vrel": 20},
    expect={"tvRadarLeftBlock": "可左变道"},
    not_expect={"tvRadarLeftBlock": "侧后逼近"},
  ),
  dict(
    name="all", desc="全部有目标(综合)",
    payload={"lead1": True, "drel": 30, "vlead": 45, "vrel": -3,
             "l_lead": True, "l_drel": 15, "l_vlead": 33, "l_vrel": -1,
             "r_lead": True, "r_drel": 20, "r_vlead": 31, "r_vrel": 1,
             "lf_drel": 16000, "lf_xrel": 1500, "lf_vrel": 0,
             "rf_drel": 21000, "rf_xrel": -1400, "rf_vrel": 0,
             "lb_drel": -11000, "lb_xrel": 1200, "lb_vrel": 2,
             "rb_drel": -8000, "rb_xrel": -1000, "rb_vrel": 4,
             "lidar_lblind": True, "lidar_rblind": True,
             "l_front_blind": True, "r_front_blind": True},
    expect={"tvRadarFront": ["前车", "30.0m", "45km/h", "(-3km/h)"],
            "tvRadarMmwLeft": "15.0m", "tvRadarLidarLeftFront": "16.0m/1.5m",
            "tvRadarMmwRight": "20.0m", "tvRadarLidarRightFront": "21.0m/1.4m",
            "tvRadarLidarLeftRear": "11.0m/1.2m",
            "tvRadarLidarRightRear": "8.0m/1.0m",
            "tvRadarLeftBlock": ["禁止左变道", "前侧盲区·前雷达"],
            "tvRadarRightBlock": ["禁止右变道", "前侧盲区·前雷达"]},
  ),
]

# 固定信封: 这些键每个包都带(App 里大多 resetOnTimeout=false)
ENVELOPE = {
  "device": "op", "port": CP_PORT, "IsOnroad": True,
  "vego": 10.0, "v_ego_kph": 36, "v_cruise_kph": 40,
  "active": True, "engaged": True,
  # 布尔类: 显式发 false, 免得依赖 500ms 超时复位
  "lead1": False, "l_lead": False, "r_lead": False,
  # 激光雷达自身(纯激光)四角盲区
  "lidar_lblind": False, "lidar_rblind": False,
  "lidar_lfblind": False, "lidar_lbblind": False,
  "lidar_rfblind": False, "lidar_rbblind": False,
  # 激光雷达在线(设备级)：App 侧的时距/TTC 自算判据只在在线时启用
  "lidar_l": False, "lidar_r": False,
  # 侧向判定阈值(CP 下发, x0.1)：App 自算判据与 CP 同源
  "lidar_front_vdist_time": -50, "lidar_front_vrel_time": 40,
  "lidar_behind_vdist_time": -100, "lidar_behind_vrel_time": 40,
  # 综合盲区(车道线/车身/激光/摄像头)
  "blind_l": False, "blind_r": False,
  # 综合盲区的分解来源(摄像头/车身/实线)
  "blind_camera_l": False, "blind_camera_r": False,
  "blind_car_l": False, "blind_car_r": False,
  "blind_lane_l": False, "blind_lane_r": False,
  "l_front_blind": False, "r_front_blind": False,
  "left_blindspot": False, "right_blindspot": False,
  # 距离类: 场景里有就覆盖, 没有就不发(靠超时清空, 与 CP 行为一致)
}


def find_adb() -> str | None:
  for cand in ADB_CANDIDATES:
    path = shutil.which(cand) if os.path.basename(cand) == cand else (cand if os.path.exists(cand) else None)
    if path:
      return path
  return None


def screenshot(adb: str, path: str) -> int:
  """截图保存（数据流常驻时 uiautomator 取不到 idle，截图不受影响）。"""
  out = subprocess.run([adb, "exec-out", "screencap", "-p"], capture_output=True).stdout
  if not out:
    return 0
  os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
  with open(path, "wb") as f:
    f.write(out)
  return len(out)


def read_cards(adb: str, tries: int = 3) -> dict:
  """读回 activity_main 里几张卡片的文字。

  首选 App 自己输出的一行日志（不受 uiautomator idle 限制）：

    RADARCARDS tvRadarMmwLeft=15.0m/(-1km/h) tvRadarFront=... ...

  （App 侧在雷达页刷新、文本变化时输出，见 MainActivityUi.logRadarCardsOnce）
  取不到时再退回 uiautomator dump —— 页面上"决策详情"带计时/实时数值会一直刷新，
  uiautomator 经常拿不到 idle(ERROR: could not get idle state)。
  """
  try:
    raw = subprocess.run([adb, "logcat", "-d"], capture_output=True).stdout.decode("utf-8", "ignore")
    cards = {}
    for line in raw.splitlines():
      m = re.search(r"RADARCARDS (.*)$", line)
      if not m:
        continue
      parsed = {}
      for kv in m.group(1).split(" "):
        if "=" in kv:
          key, value = kv.split("=", 1)
          parsed[key] = value
      if parsed:
        cards = parsed          # 取最后一条（最新的状态）
    if cards:
      return cards
  except Exception:
    pass

  for attempt in range(tries):
    for extra in (["--windows"], []):
      subprocess.run([adb, "shell", "uiautomator", "dump"] + extra + ["/sdcard/ui.xml"],
                     capture_output=True)
      out = subprocess.run([adb, "shell", "cat", "/sdcard/ui.xml"], capture_output=True)
      xml = out.stdout.decode("utf-8", "ignore")
      cards = {}
      for m in re.finditer(r"<node[^>]*resource-id=\"com\.carrot\.amapnavi:id/(tvRadar\w+)\"[^>]*/?>", xml):
        tag, rid = m.group(0), m.group(1)
        tm = re.search(r'text="([^"]*)"', tag)
        cards[rid] = (tm.group(1) if tm else "").replace("&#10;", " / ").replace("\n", " / ")
      if cards:
        return cards
    if attempt < tries - 1:
      time.sleep(1.2)
  return {}


def wait_for_app(adb: str) -> bool:
  out = subprocess.run([adb, "shell", "ps", "-A"], capture_output=True).stdout.decode("utf-8", "ignore")
  return "com.carrot.amapnavi" in out


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--ip", default="192.168.1.14", help="手机 IP")
  ap.add_argument("--port", type=int, default=APP_PORT)
  ap.add_argument("--hz", type=float, default=10.0)
  ap.add_argument("--hold", type=float, default=6.0, help="每个场景保持秒数")
  ap.add_argument("--scenario", default=None, help="只跑这个场景(名字见 --list)")
  ap.add_argument("--no-adb", action="store_true", help="不做 adb 校验")
  ap.add_argument("--shot-dir", default=None,
                  help="每个场景结束时截图到此目录（推荐：数据流下 uiautomator 取不到 idle）")
  ap.add_argument("--list", action="store_true", help="列出所有场景")
  args = ap.parse_args()

  if args.list:
    for s in SCENARIOS:
      print("  %-26s %s" % (s["name"], s["desc"]))
    return 0

  todo = [s for s in SCENARIOS if args.scenario in (None, s["name"])]
  if not todo:
    print("没有匹配的场景:", args.scenario)
    return 1

  adb = None if args.no_adb else find_adb()
  if not args.no_adb:
    if adb is None:
      print("找不到 adb，改为只发包（--no-adb 可显式关闭校验）")
    elif not wait_for_app(adb):
      print("提示: App 进程没在跑，先执行 adb shell am start -n com.carrot.amapnavi/.MainActivity")

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  dst = (args.ip, args.port)
  period = 1.0 / max(args.hz, 1.0)
  print("发往 %s:%d @ %.0fHz, 场景数 %d, 每个 %.1fs\n" % (dst[0], dst[1], args.hz, len(todo), args.hold))

  all_pass = True
  for sc in todo:
    payload = dict(ENVELOPE)
    payload.update(sc["payload"])
    # 信封里不含距离键，距离只由场景提供；与 CP 一致：没有目标的角就不发该键
    raw = json.dumps(payload).encode()

    print("=" * 100)
    print("场景 %s — %s" % (sc["name"], sc["desc"]))
    keys = {k: v for k, v in sc["payload"].items()}
    print("  发送字段: %s" % (keys if keys else "(只有信封)"))
    print("  期望卡片: %s" % sc["expect"])

    deadline = time.monotonic() + args.hold
    sent = 0
    while time.monotonic() < deadline:
      sock.sendto(raw, dst)
      sent += 1
      time.sleep(period)
    print("  已发送 %d 包" % sent)

    if adb and args.shot_dir:
      shot = os.path.join(args.shot_dir, "scenario_%s.png" % sc["name"])
      size = screenshot(adb, shot)
      print("  截图: %s (%.0f KB)" % (shot, size / 1024.0))

    if adb:
      time.sleep(0.3)
      cards = read_cards(adb)
      if not cards:
        print("  [uiautomator 取不到界面] 以截图为准核对期望")
        continue
      for cid in CARD_IDS:
        txt = cards.get(cid, "(未在界面里)")
        exp = sc["expect"].get(cid)
        nexp = sc.get("not_expect", {}).get(cid)
        if exp is None and nexp is None:
          print("    %-18s = %s" % (cid, txt))
          continue
        wants = [exp] if isinstance(exp, str) else list(exp or [])
        checks = [(("含 " + w), w in txt) for w in wants]
        if nexp:
          checks.append((("不含 " + nexp), nexp not in txt))
        ok = all(c[1] for c in checks)
        all_pass &= ok
        print("    %-18s = %-42s %s (%s)" % (
          cid, txt, "PASS" if ok else "FAIL",
          ", ".join("%s%s" % ("" if good else "✗", name) for name, good in checks)))

  print("")
  print("所有断言通过" if all_pass else "存在 FAIL，见上面各场景")
  return 0 if all_pass else 2


if __name__ == "__main__":
  raise SystemExit(main())
