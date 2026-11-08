#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
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
  echo 180 > /data/ford_params/Mr76RssLimitMB  # 可选: 单进程 RSS 软上限

写入格式 (与 obstacle_avoid._read_aux 约定):
  {"t": <epoch>, "targets": [[d, y, v_rel, "cls"], ...], "fresh": bool, "stat": {...}}

fresh=False 表示 MR76 本身不可信(掉线/对象流停了)。下游看到 fresh=False
必须把这一路整源忽略, 绝不能当成"目标车道空" —— 这是参考实现
E:\openpilot\chestnut 2026-09-20 里明确记下的失效方向。

[MEM_FIX_20261108] 2026-11-06 行车记录: 本进程 RSS 从 64MB 线性涨到 1183MB
(~16MB/min), 把 deviceState.memoryUsagePercent 推到 94%, 触发 lowMemory ->
soft disable -> 无法再进入, 熄火/重启后内存掉回 33%。

设备端对照实验结论(独立 msgq 通道, 精确复现 SubMaster -> log_from_bytes):
  BASE(gc on)              0.53 KB/rx
  gc.disable()             0.53 KB/rx  <- 完全一致, 排除 GC 关闭为主因
  显式 del + gc.collect()  0.50 KB/rx  <- 噪声级, 排除 Python 层引用
即每收一帧 can 消息就有约 0.5KB 的确定性增长, 发生在 C 层
(glibc malloc arena / pycapnp reader 内存池), Python GC 与断引用都管不到,
只有替换/销毁整个进程映像才能归还 —— 与"熄火后归零"完全吻合。

因此主修复是 RSS 看门狗: 超过软上限就先把共享载荷标成 fresh=False, 然后用
os.execv 原地重载自身进程映像(走 manager.launcher 同一条启动路径)。这样
C 层内存整块归还, PID 与进程名不变, manager 无感 —— 不能用 os._exit(),
因为 manager.ensure_running() 对"proc 对象还在但已退出"的进程不会重新拉起,
直接退出会让 MR76 一直失效到下次点火。

顺带做三件低风险的事: 恢复被 config_realtime_process() 关掉的 GC(mr76d 不是
硬实时进程)、切断帧末循环变量引用、给 /data/media/0 上的日志加轮转(/data 已 90%)。
"""

import gc
import json
import os
import sys
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.u_radar import Mr76Tracker

OUT = "/dev/shm/mr76_targets.json"
TMP = OUT + ".tmp"
PARAM_DIR = "/data/ford_params"
LOG_PATH = "/data/media/0/mr76d.log"
MODULE = "openpilot.selfdrive.controls.mr76d"
DAEMON = "mr76d"

# [MEM_FIX_20261108] 内存与磁盘双护栏
LOG_MAX_BYTES = 256 * 1024    # 日志轮转上限, 防止长期运行堆积 (/data 已 90%)
RSS_LIMIT_DEFAULT_MB = 180.0  # 单进程 RSS 软上限默认值; 可用 Mr76RssLimitMB 覆盖
GC_INTERVAL_SEC = 5.0         # 周期性强制回收(常规卫生措施, 非本次主修复)
CHECK_INTERVAL_SEC = 5.0      # RSS 检查周期


def _num(key, default):
  for d in (PARAM_DIR, "/dev/shm/params", "/data/params/d"):
    try:
      with open(os.path.join(d, key)) as f:
        return float(f.read().strip())
    except Exception:
      continue
  return default


def _rss_mb():
  """当前进程 RSS(MB), 读 /proc/self/statm 第 2 列(页数)。"""
  try:
    with open("/proc/self/statm") as f:
      return int(f.read().split()[1]) * 4096 / (1024.0 * 1024.0)
  except Exception:
    return 0.0


def _log(msg):
  try:
    # [MEM_FIX_20261108] 单文件轮转: 超过上限就改名为 .old, 只保留一份历史
    try:
      if os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
        os.replace(LOG_PATH, LOG_PATH + ".old")
    except OSError:
      pass
    with open(LOG_PATH, "a") as f:
      f.write(time.strftime("%m-%d %H:%M:%S ") + msg + "\n")
  except Exception:
    pass


def _mark_stale(st):
  """看门狗重启前把共享载荷标成不可信, 避免下游在重启窗口误用旧目标。"""
  try:
    safe_stat = dict(st) if isinstance(st, dict) else {}
    safe_stat["fresh"] = False
    with open(TMP, "w") as f:
      json.dump({"t": time.time(), "targets": [],
                 "fresh": False, "stat": safe_stat}, f)
    os.replace(TMP, OUT)
  except Exception:
    pass


def _self_reexec():
  """[MEM_FIX_20261108] 原地重载进程映像, 整块归还 C 层内存。

  复用 manager 的 launcher(importlib + setproctitle + reset_context +
  cloudlog.bind), 保证重启后进程名/日志 tag 与原进程完全一致。
  PID 不变, manager 的 Process 对象仍 is_alive(), 不需要它参与重启。
  """
  py = sys.executable or "/usr/local/venv/bin/python3"
  code = ("from openpilot.system.manager.process import launcher; "
          f"launcher({MODULE!r}, {DAEMON!r})")
  try:
    # 关掉继承的 ZMQ/msgq fd, 避免多次重启后 fd 累积; 0/1/2 保留
    os.closerange(3, 1024)
    os.execv(py, [py, "-c", code])
  except Exception as e:
    _log(f"REEXEC FAIL {type(e).__name__}: {e}")
    os._exit(1)


def main():
  # 本机只有 4 核 (0-3); 绑核失败不致命, 退化为不绑核即可
  try:
    config_realtime_process(3, Priority.CTRL_LOW)
  except Exception:
    pass
  # [MEM_FIX_20261108] config_realtime_process() 第一行是 gc.disable(), 那是给
  # modeld/controlsd 这类硬实时进程用的。mr76d 只是 20Hz 轮询的雷达解析, 没有
  # 关 GC 的理由; 实测它不是本次泄漏的主因, 但恢复它没有副作用。
  gc.enable()

  src_filter = int(_num("Mr76CanSrc", 1))   # MR76 接 CAN1(与整车同总线), 默认只看 CAN1
  lat_sign = _num("Mr76LatSign", -1.0)
  max_long = _num("Mr76MaxLong", 40.0)
  trust_class = int(_num("Mr76TrustClass", 1)) != 0
  rss_limit_mb = _num("Mr76RssLimitMB", RSS_LIMIT_DEFAULT_MB)
  # 下限保护: 正常空载就 64MB, 设太低会导致频繁重启
  if rss_limit_mb < 100.0 or rss_limit_mb > 600.0:
    rss_limit_mb = RSS_LIMIT_DEFAULT_MB
  cloudlog.info(f"mr76d start src={src_filter} lat_sign={lat_sign} "
                f"trust_class={trust_class} gc={gc.isenabled()} "
                f"rss_limit={rss_limit_mb:.0f}MB")

  tr = Mr76Tracker(lat_sign=lat_sign, trust_class=trust_class)
  sm = messaging.SubMaster(["can"])
  seen = {}            # src -> 0x60B 计数, 用于首次探测日志
  last_log = 0.0
  last_gc = time.monotonic()
  last_check = last_gc
  st = {"cls": {}, "dyn": {}}
  tg = []
  m = None
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
      # [MEM_FIX_20261108] Python 的 for 循环变量在循环结束后仍存活, 会一直
      # 抓着最后一个 capnp struct reader(以及它背后的整条 CAN 消息 arena)。
      m = None
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
    payload = None

    now = time.monotonic()
    # 常规卫生措施: 低频全代回收。本进程是 CTRL_LOW 优先级, 5s 一次的几 ms
    # 抖动不影响雷达解析。注意这不是本次泄漏的主修复(见文件头实验结论)。
    if now - last_gc >= GC_INTERVAL_SEC:
      last_gc = now
      gc.collect()
    # [MEM_FIX_20261108] 主修复: 每帧 ~0.5KB 的 C 层增长无法在进程内回收,
    # 只能在它吃掉整机内存之前原地重载进程映像。重启窗口内下游读到
    # fresh=False, 不会把旧目标误当成"车道空"。
    if now - last_check >= CHECK_INTERVAL_SEC:
      last_check = now
      rss = _rss_mb()
      if rss > rss_limit_mb:
        _log(f"RSS WATCHDOG {rss:.1f}MB > {rss_limit_mb:.1f}MB; "
             f"frames={tr.object_frames} gc={gc.isenabled()} self-reexec")
        cloudlog.warning(f"mr76d rss watchdog {rss:.1f}MB, self-reexec")
        _mark_stale(st)
        _self_reexec()

    if now - last_log >= 5.0:
      last_log = now
      if seen:
        _log(f"rx={dict(seen)} objs={len(tg)} rss={_rss_mb():.1f}MB "
             f"n_obj_hdr={tr.n_objects} status={tr.status_frames} "
             f"obj_frames={tr.object_frames} rejected={tr.rejected} "
             f"fresh={int(bool(st.get('fresh', False)))} "
             f"cls={st.get('cls', {})} dyn={st.get('dyn', {})}")
      else:
        _log(f"no 0x60A/0x60B seen (src_filter={src_filter}) "
             f"rss={_rss_mb():.1f}MB; 检查 MR76 接线与 Mr76CanSrc")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
  except Exception:
    cloudlog.exception("mr76d failed")
    raise
