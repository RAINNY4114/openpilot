#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mr76d —— MR76 毫米波雷达 CAN 解析守护进程
==========================================

订阅 `can`, 按 u_radar.dbc 解 0x60A/0x60B, 把目标列表写到
/dev/shm/mr76_targets.json, 由 obstacle_avoid 读取。

为什么要独立进程:
  - modeld 是 20Hz 实时进程, 不适合再挂一路 100Hz CAN 解析
  - 崩了只影响辅助雷达, 不影响视觉/控车
  - 可以单独启停, 不动主链路

启停 (不写配置就不跑):
  echo 1 > /data/ford_params/Mr76Enabled
  echo 1 > /data/ford_params/Mr76CanSrc     # MR76 所在的 CAN 路, -1=任意
  echo -1 > /data/ford_params/Mr76LatSign   # 雷达 DistLat 右正 -> 翻转为左正; 若反了改 1
  echo 1 > /data/ford_params/Mr76TrustClass # Class 字段可信才用它过滤杂物

写入格式 (与 obstacle_avoid._read_aux 约定):
  {"t": <epoch>, "targets": [[d, y, v_rel, "cls"], ...], "fresh": bool, "stat": {...}}

fresh=False 表示 MR76 本身不可信(掉线/对象流停了)。下游看到 fresh=False
必须把这一路整源忽略, 绝不能当成"目标车道空" —— 这是参考实现
E:\openpilot\chestnut 2026-09-20 里明确记下的失效方向。
"""

import json
import os
import time

import cereal.messaging as messaging
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.u_radar import Mr76Tracker

OUT = "/dev/shm/mr76_targets.json"
TMP = OUT + ".tmp"
PARAM_DIR = "/data/ford_params"
LOG_PATH = "/data/media/0/mr76d.log"


def _num(key, default):
  for d in (PARAM_DIR, "/dev/shm/params", "/data/params/d"):
    try:
      with open(os.path.join(d, key)) as f:
        return float(f.read().strip())
    except Exception:
      continue
  return default


def _log(msg):
  try:
    with open(LOG_PATH, "a") as f:
      f.write(time.strftime("%m-%d %H:%M:%S ") + msg + "\n")
  except Exception:
    pass


def main():
  # 本机只有 4 核 (0-3); 绑核失败不致命, 退化为不绑核即可
  try:
    config_realtime_process(3, Priority.CTRL_LOW)
  except Exception:
    pass
  src_filter = int(_num("Mr76CanSrc", 1))   # MR76 接 CAN1(与整车同总线), 默认只看 CAN1
  lat_sign = _num("Mr76LatSign", -1.0)
  max_long = _num("Mr76MaxLong", 40.0)
  trust_class = int(_num("Mr76TrustClass", 1)) != 0
  cloudlog.info(f"mr76d start src={src_filter} lat_sign={lat_sign} "
                f"trust_class={trust_class}")

  tr = Mr76Tracker(lat_sign=lat_sign, trust_class=trust_class)
  sm = messaging.SubMaster(["can"])
  seen = {}            # src -> 0x60B 计数, 用于首次探测日志
  last_log = 0.0
  st = {"cls": {}, "dyn": {}}
  tg = []
  write_err = [None]   # 只报一次写盘失败, 不刷屏

  while True:
    sm.update(20)
    if sm.updated["can"]:
      for m in sm["can"]:
        if src_filter >= 0 and m.src != src_filter:
          continue
        try:
          if tr.feed(m.src, m.address, m.dat):
            seen[m.src] = seen.get(m.src, 0) + 1
        except Exception:
          continue
    # 每个 CAN batch 调一次: 连续多批没 MR76 报文 -> 硬清空缓存
    tr.tick()

    tg = tr.targets(max_long)
    # 每轮都刷新时间戳, 但 fresh 单独给, 让下游能区分三种状态:
    #   雷达在 + 有目标 / 雷达在 + 真没目标 / 雷达没了
    try:
      st = tr.status_dict()
      payload = {"t": time.time(), "targets": tg,
                 "fresh": st["fresh"], "stat": st}
      with open(TMP, "w") as f:
        json.dump(payload, f)
      os.replace(TMP, OUT)
    except Exception as e:
      # 典型原因: OUT 曾被 root 创建(/dev/shm 是 sticky 目录, comma 既删不掉也
      # 覆盖不了), 于是写盘永远失败、下游永远读到旧载荷 —— 必须留下痕迹。
      msg = f"{type(e).__name__}: {e}"
      if write_err[0] != msg:
        write_err[0] = msg
        _log(f"WRITE FAIL {OUT} -> {msg}; 若属主不是 comma 请 sudo rm -f {OUT}")

    now = time.monotonic()
    if now - last_log >= 5.0:
      last_log = now
      if seen:
        _log(f"rx={dict(seen)} objs={len(tg)} "
             f"n_obj_hdr={tr.n_objects} status={tr.status_frames} "
             f"obj_frames={tr.object_frames} rejected={tr.rejected} "
             f"fresh={int(bool(st.get('fresh', False)))} "
             f"cls={st.get('cls', {})} dyn={st.get('dyn', {})}")
      else:
        _log(f"no 0x60A/0x60B seen (src_filter={src_filter}); "
             f"检查 MR76 接线与 Mr76CanSrc")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
  except Exception:
    cloudlog.exception("mr76d failed")
    raise
