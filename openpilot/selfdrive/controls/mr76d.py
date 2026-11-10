#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

LOG_MAX_BYTES = 256 * 1024
RSS_LIMIT_DEFAULT_MB = 180.0
GC_INTERVAL_SEC = 5.0
CHECK_INTERVAL_SEC = 5.0

def _num(key, default):
  for d in (PARAM_DIR, "/dev/shm/params", "/data/params/d"):
    try:
      with open(os.path.join(d, key)) as f:
        return float(f.read().strip())
    except Exception:
      continue
  return default

def _rss_mb():
  try:
    with open("/proc/self/statm") as f:
      return int(f.read().split()[1]) * 4096 / (1024.0 * 1024.0)
  except Exception:
    return 0.0

def _log(msg):
  try:
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
  py = sys.executable or "/usr/local/venv/bin/python3"
  code = ("from openpilot.system.manager.process import launcher; "
          f"launcher({MODULE!r}, {DAEMON!r})")
  try:
    os.closerange(3, 1024)
    os.execv(py, [py, "-c", code])
  except Exception as e:
    _log(f"REEXEC FAIL {type(e).__name__}: {e}")
    os._exit(1)

def main():
  try:
    config_realtime_process(3, Priority.CTRL_LOW)
  except Exception:
    pass
  gc.enable()

  src_filter = int(_num("Mr76CanSrc", 1))
  lat_sign = _num("Mr76LatSign", -1.0)
  max_long = _num("Mr76MaxLong", 40.0)
  trust_class = int(_num("Mr76TrustClass", 1)) != 0
  rss_limit_mb = _num("Mr76RssLimitMB", RSS_LIMIT_DEFAULT_MB)
  if rss_limit_mb < 100.0 or rss_limit_mb > 600.0:
    rss_limit_mb = RSS_LIMIT_DEFAULT_MB
  cloudlog.info(f"mr76d start src={src_filter} lat_sign={lat_sign} "
                f"trust_class={trust_class} gc={gc.isenabled()} "
                f"rss_limit={rss_limit_mb:.0f}MB")

  tr = Mr76Tracker(lat_sign=lat_sign, trust_class=trust_class)
  sm = messaging.SubMaster(["can"])
  seen = {}
  last_log = 0.0
  last_gc = time.monotonic()
  last_check = last_gc
  st = {"cls": {}, "dyn": {}}
  tg = []
  m = None
  write_err = [None]

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
      m = None
    tr.tick()

    tg = tr.targets(max_long)
    try:
      st = tr.status_dict()
      payload = {"t": time.time(), "targets": tg,
                 "fresh": st["fresh"], "stat": st}
      with open(TMP, "w") as f:
        json.dump(payload, f)
      os.replace(TMP, OUT)
    except Exception as e:
      msg = f"{type(e).__name__}: {e}"
      if write_err[0] != msg:
        write_err[0] = msg
        _log(f"WRITE FAIL {OUT} -> {msg}; 若属主不是 comma 请 sudo rm -f {OUT}")
    payload = None

    now = time.monotonic()
    if now - last_gc >= GC_INTERVAL_SEC:
      last_gc = now
      gc.collect()
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
