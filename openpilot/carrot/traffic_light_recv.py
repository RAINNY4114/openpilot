"""红绿灯导航推送接收（carrot-wip 协议移植）。

【数据源】
手机端 carrot / 高德导航 App 通过 HTTP POST 把路口信息推到设备：
    POST http://<device-ip>:7713/api/navi/<tmap_version>
    body: JSON，含 `sinf`（简版）或 `ssinf`（详版）字段

sinf 示例（carrot 注释原样保留）：
    {'distance': 120, 'redLightRemainTime': 15, 'greenLightRemainTime': 0,
     'leftLightRemainTime': 0, 'rightLightRemainTime': 0, 'uturnLightRemainTime': 0,
     'redLightOn': True, 'greenLightOn': False, 'leftLightOn': False,
     'rightLightOn': False, 'uturnLightOn': False,
     'location': {'latitude': 37.477..., 'longitude': 127.045...}}

ssinf 示例：{distance, straight: 'GREEN_LIGHT_ON'|'RED_LIGHT_ON',
             straight_remain_time, left/right/uturn(+_remain_time), lat, lon}

【落地】
    /dev/shm/params/d/TrafficLight = JSON {distance, lamp, remain, ts, lat, lon}
用内存参数（tmpfs）而不是 /data/params：manager 的 Params::clearAll() 会删掉所有
未注册进已编译 params_keys.h 的 key，内存参数不受影响；重启丢失无妨（导航会重推）。

【消费方】
openpilot/selfdrive/controls/lib/traffic_light.py（读同一路径，带超时与 shadow 模式）

【相对 carrot 的差异】
- 用标准库 http.server 而不是 aiohttp（设备 aiohttp 可用，但这条链路越轻越好；
  POST 是低频事件，单进程多线程足够）
- 不做 `_store_navi_event` / `_write_navi_debug_param` 那套事件落盘（调试用，非必需）
- remain<=0 一律丢弃，与 carrot `_put_traffic_light` 一致
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PARAM_DIR = "/dev/shm/params/d"
PARAM_KEY = "TrafficLight"
DEFAULT_PORT = 7713

LAMPS = ("red", "left", "green", "right", "uturn")

_state_lock = threading.Lock()
_last = {"lamp": None, "remain": 0, "distance": 0, "ts": 0.0}
_stats = {"rx": 0, "sinf": 0, "ssinf": 0, "drop_remain": 0, "drop_bad": 0, "last_rx_ts": 0.0}


def _write_param(obj):
  os.makedirs(PARAM_DIR, exist_ok=True)
  path = os.path.join(PARAM_DIR, PARAM_KEY)
  tmp = path + ".tmp"
  with open(tmp, "wb") as f:
    f.write(json.dumps(obj).encode())
  # os.replace 会丢权限位（umask 664）-> 显式设置，避免消费方读不到
  os.chmod(tmp, 0o644)
  os.replace(tmp, path)


def _put_traffic_light(lamp, remain, distance=0, lat=None, lon=None):
  try:
    remain_int = int(float(remain or 0))
  except Exception:
    remain_int = 0
  if remain_int <= 0:
    _stats["drop_remain"] += 1
    return False

  try:
    distance_int = int(float(distance or 0))
  except Exception:
    distance_int = 0

  obj = {
    "distance": distance_int,
    "lamp": lamp,
    "remain": remain_int,
    "ts": time.monotonic(),
  }
  try:
    if lat is not None:
      obj["lat"] = float(lat)
    if lon is not None:
      obj["lon"] = float(lon)
  except Exception:
    pass

  with _state_lock:
    _last.update(obj)
    _write_param(obj)
  return True


def handle_sinf(d):
  """简版：各方向 on 标志 + remain 秒。"""
  if not isinstance(d, dict):
    _stats["drop_bad"] += 1
    return False

  lamp = None
  remain = 0
  if d.get("redLightOn"):
    lamp, remain = "red", d.get("redLightRemainTime", 0)
  elif d.get("leftLightOn"):
    lamp, remain = "left", d.get("leftLightRemainTime", 0)
  elif d.get("greenLightOn"):
    lamp, remain = "green", d.get("greenLightRemainTime", 0)
  elif d.get("rightLightOn"):
    lamp, remain = "right", d.get("rightLightRemainTime", 0)
  elif d.get("uturnLightOn"):
    lamp, remain = "uturn", d.get("uturnLightRemainTime", 0)

  if lamp is None:
    _stats["drop_bad"] += 1
    return False

  lat = lon = None
  loc = d.get("location", {})
  try:
    if isinstance(loc, dict):
      if loc.get("latitude") is not None:
        lat = float(loc.get("latitude"))
      if loc.get("longitude") is not None:
        lon = float(loc.get("longitude"))
  except Exception:
    pass
  return _put_traffic_light(lamp, remain, d.get("distance", 0), lat, lon)


def handle_ssinf(d):
  """详版：每个方向一个枚举串（GREEN_LIGHT_ON / RED_LIGHT_ON）+ 各自 remain。"""
  if not isinstance(d, dict):
    _stats["drop_bad"] += 1
    return False

  green_checks = (
    ("left", "left", "left_remain_time"),
    ("straight", "green", "straight_remain_time"),
    ("right", "right", "right_remain_time"),
    ("uturn", "uturn", "uturn_remain_time"),
  )
  for field, lamp, remain_field in green_checks:
    if str(d.get(field, "")).upper() == "GREEN_LIGHT_ON":
      return _put_traffic_light(lamp, d.get(remain_field, 0),
                                d.get("distance", 0), d.get("lat"), d.get("lon"))

  red_remain = 0
  for field in ("straight", "left", "right", "uturn"):
    if str(d.get(field, "")).upper() == "RED_LIGHT_ON":
      try:
        red_remain = max(red_remain, int(d.get(f"{field}_remain_time", 0) or 0))
      except Exception:
        pass
  if red_remain > 0:
    return _put_traffic_light("red", red_remain, d.get("distance", 0),
                              d.get("lat"), d.get("lon"))
  _stats["drop_bad"] += 1
  return False


def dispatch(obj):
  """按 carrot `_dispatch_obj` 的顺序处理 sinf / ssinf。"""
  if not isinstance(obj, dict):
    _stats["drop_bad"] += 1
    return False
  hit = False
  if "ssinf" in obj:
    _stats["ssinf"] += 1
    hit = handle_ssinf(obj["ssinf"]) or hit
  if "sinf" in obj:
    _stats["sinf"] += 1
    hit = handle_sinf(obj["sinf"]) or hit
  return hit


class _Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"
  server_version = "chx-traffic-light/1.0"

  def log_message(self, fmt, *args):
    pass

  def _respond(self, code, body):
    self.send_response(code)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    try:
      self.wfile.write(body)
    except Exception:
      pass

  def do_GET(self):
    if self.path.startswith("/health"):
      with _state_lock:
        snap = dict(_last)
      body = json.dumps({"ok": True, "service": "traffic_light",
                         "stats": dict(_stats), "last": snap}).encode()
      self._respond(200, body)
    else:
      self._respond(404, b'{"ok":false,"error":"not found"}')

  def do_POST(self):
    if not self.path.startswith("/api/navi"):
      self._respond(404, b'{"ok":false,"error":"not found"}')
      return
    try:
      n = int(self.headers.get("Content-Length", 0) or 0)
      raw = self.rfile.read(n) if n > 0 else b""
      obj = json.loads(raw.decode())
    except Exception as e:
      _stats["drop_bad"] += 1
      self._respond(400, json.dumps({"ok": False, "error": f"invalid json: {e}"}).encode())
      return

    _stats["rx"] += 1
    _stats["last_rx_ts"] = time.monotonic()
    try:
      hit = dispatch(obj)
      self._respond(200, json.dumps({"ok": True, "accepted": bool(hit)}).encode())
    except Exception as e:
      self._respond(500, json.dumps({"ok": False, "error": str(e)}).encode())


def start(port=DEFAULT_PORT, host="0.0.0.0"):
  """起后台线程；返回 server（调用方可 shutdown）。端口被占用时抛异常，由调用方兜住。"""
  srv = ThreadingHTTPServer((host, port), _Handler)
  srv.daemon_threads = True
  th = threading.Thread(target=srv.serve_forever, daemon=True, name="traffic_light_http")
  th.start()
  print(f"[traffic_light] HTTP listening {host}:{port}  POST /api/navi/<ver>  GET /health",
        flush=True)
  return srv
