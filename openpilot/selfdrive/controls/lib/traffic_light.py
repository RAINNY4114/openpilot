"""红绿灯（导航推送）状态读取 + 停车目标计算。

配套 `openpilot/carrot/traffic_light_recv.py`（接收端）。

【链路】
手机 App --HTTP POST :7713--> traffic_light_recv --写--> /dev/shm/params/d/TrafficLight
                                                          |
                                     本模块读 <------------+
                                                          |
                          longitudinal_planner: v_cruise = min(v_cruise, v_target)

【安全设计】
- mode 0=off / 1=shadow（只算不用，写日志）/ 2=active，默认 **1**。
  红绿灯数据来自外部导航推送，可能推错路口；先影子跑，确认灯与路口对得上再切 2。
- 推送超过 max_age_s 视为无效（导航断连时不会一直按红灯停车）。
- distance < min_distance_m 时不再规划（已到灯下，交给 MPC / 前车停车逻辑）。
- 只做 v_cruise 上限（速度目标），不直接给负加速度，减速度由 MPC 正常求解。
- 任何异常 -> 返回 v_target=None（不干预）。
"""

import json
import math
import os
import time

CONFIG_PATH = "/data/traffic_light.json"
PARAM_PATH = "/dev/shm/params/d/TrafficLight"

DEFAULTS = {
    "mode": 1,                 # 0=off 1=shadow 2=active
    "max_age_s": 10.0,         # 推送新鲜度
    "min_distance_m": 5.0,     # 小于此距离不再规划
    "max_distance_m": 120.0,   # 大于此距离不介入
    "stop_margin_m": 4.0,      # 停在灯前的余量
    "decel_ms2": 1.5,          # 规划减速度（速度目标曲线）
    "stop_lamp": ["red"],      # 需要停的灯
    "log_period_s": 5.0,
}


def _deep_merge(base, over):
  out = dict(base)
  for k, v in (over or {}).items():
    if isinstance(v, dict) and isinstance(out.get(k), dict):
      out[k] = _deep_merge(out[k], v)
    else:
      out[k] = v
  return out


class TrafficLightControl:
  def __init__(self, config_path=CONFIG_PATH, param_path=PARAM_PATH):
    self.config_path = config_path
    self.param_path = param_path
    self.cfg = dict(DEFAULTS)
    self._cfg_mtime = None
    self._cfg_check_t = 0.0
    self._param_mtime = None
    self._param = None
    self._last_log_t = 0.0
    self.last = {}

  # ------------------------------------------------------------------ config
  def _load_cfg(self, now):
    if (now - self._cfg_check_t) < 1.0:
      return
    self._cfg_check_t = now
    try:
      if not os.path.exists(self.config_path):
        return
      m = os.path.getmtime(self.config_path)
      if m == self._cfg_mtime:
        return
      with open(self.config_path, "r") as f:
        user = json.load(f)
      if isinstance(user, dict):
        self.cfg = _deep_merge(DEFAULTS, user)
        self._cfg_mtime = m
    except Exception:
      pass

  def _read_param(self):
    try:
      m = os.path.getmtime(self.param_path)
    except OSError:
      self._param = None
      self._param_mtime = None
      return None
    if m == self._param_mtime and self._param is not None:
      return self._param
    try:
      with open(self.param_path, "r") as f:
        obj = json.load(f)
      self._param = obj if isinstance(obj, dict) else None
      self._param_mtime = m
    except Exception:
      self._param = None
    return self._param

  # ------------------------------------------------------------------ update
  def update(self, v_ego, now=None):
    now = time.monotonic() if now is None else float(now)
    self._load_cfg(now)

    out = {"mode": int(self.cfg.get("mode", 1)), "lamp": None, "distance": 0,
           "remain": 0, "age": -1.0, "v_target": None, "active": False, "reason": "off"}
    if out["mode"] <= 0:
      self.last = out
      return out

    p = self._read_param()
    if not p:
      out["reason"] = "no_data"
      self.last = out
      return out

    try:
      ts = float(p.get("ts", 0.0))
      dist = float(p.get("distance", 0.0))
      remain = int(p.get("remain", 0))
      lamp = str(p.get("lamp", ""))
    except Exception:
      out["reason"] = "bad_param"
      self.last = out
      return out

    age = now - ts
    out.update(lamp=lamp, distance=dist, remain=remain, age=age)

    max_age = float(self.cfg.get("max_age_s", 10.0))
    if age < 0.0 or age > max_age:
      out["reason"] = "stale"
      self.last = out
      return out
    if remain <= 0:
      out["reason"] = "no_remain"
      self.last = out
      return out

    stop_lamp = self.cfg.get("stop_lamp", ["red"])
    if lamp not in stop_lamp:
      out["reason"] = "go"
      self.last = out
      return out

    dmin = float(self.cfg.get("min_distance_m", 5.0))
    dmax = float(self.cfg.get("max_distance_m", 120.0))
    if dist > dmax:
      out["reason"] = "too_far"
      self.last = out
      return out
    if dist < dmin:
      # 已到灯下：不再规划速度，交给 MPC / 前车停车逻辑（避免最后一米被速度目标锁死）
      out["reason"] = "too_close"
      self.last = out
      return out

    margin = float(self.cfg.get("stop_margin_m", 4.0))
    decel = max(0.2, float(self.cfg.get("decel_ms2", 1.5)))
    travel = max(0.0, dist - margin)
    v_target = math.sqrt(max(0.0, 2.0 * decel * travel))
    if v_ego <= 0.3 and travel <= 1.0:
      v_target = 0.0

    out["reason"] = "stop"
    out["active"] = out["mode"] >= 2
    out["v_target"] = v_target if out["mode"] >= 2 else None

    if (now - self._last_log_t) >= float(self.cfg.get("log_period_s", 5.0)):
      self._last_log_t = now
      out["_log"] = (f"[traffic_light] mode={out['mode']} lamp={lamp} d={dist:.0f}m "
                     f"remain={remain}s age={age:.1f}s v_tgt={v_target:.1f} "
                     f"v_ego={v_ego:.1f} -> {'ACTIVE' if out['active'] else 'shadow'}")
    self.last = out
    return out
