#!/usr/bin/env python3
"""手机上设置导航目的地的小网页。

背景
----
车机上**没有目的地选择界面**，openpilot 自己的说明是
"Set a destination from comma connect"（`mici/layouts/settings/developer.py:87`），
也就是只能靠手机 App 的 RPC。但那个入口依赖在线配对、而且要绕好几层。

这个脚本在车机上起一个只做一件事的小 HTTP 服务：
手机浏览器打开 → 在地图上点一下（或搜索）→ 点「设为目的地」→ 写 NavDestination。

    http://<车机IP>:8899

为什么用「地图上点一下」而不是搜索框为主
----------------------------------------
实测 Mapbox 对中文分词很差（「奥林匹克公园」会拆成单字、搜到悉尼），
所以点击选点才是可靠路径；搜索框只是辅助（英文/拼音还行）。

接口
----
    GET  /            页面
    GET  /api/state   当前状态（目的地、NavigationEnabled、车辆位置）
    POST /api/dest    {"lat":..,"lon":..}  设置目的地
    POST /api/clear   清除目的地

只接受中国范围内的经纬度，不做任何命令执行，纯参数读写。
"""
import json
import os
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.common.params import Params

PORT = 8899
TOKEN_PATH = "/data/ford_params/mapbox_token"

# 中国大致范围，防止误点到国外（Mapbox 中文搜索会把「奥林匹克公园」搜到悉尼）
CN_LAT = (3.0, 54.0)
CN_LON = (73.0, 136.0)


def read_token():
  try:
    with open(TOKEN_PATH) as f:
      return f.read().strip()
  except OSError:
    return os.environ.get("MAPBOX_TOKEN", "").strip()


def car_position(p):
  """车辆最后已知位置。没有就退回杭州（这台车的常驻区域）。"""
  raw = p.get("LastGPSPositionLLK")
  if raw:
    try:
      d = json.loads(raw) if isinstance(raw, str) else raw
      lat, lon = float(d["latitude"]), float(d["longitude"])
      if CN_LAT[0] <= lat <= CN_LAT[1] and CN_LON[0] <= lon <= CN_LON[1]:
        return lat, lon
    except Exception:
      pass
  return 30.0697, 119.9506


def state():
  p = Params()
  dest = p.get("NavDestination")
  lat, lon = car_position(p)
  return {
    "nav_enabled": bool(p.get_bool("NavigationEnabled")),
    "dest": dest if isinstance(dest, dict) else None,
    "car": {"lat": lat, "lon": lon},
  }


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>设置导航目的地</title>
<link href="https://api.mapbox.com/mapbox-gl-js/v3.6.0/mapbox-gl.css" rel="stylesheet">
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body { margin: 0; height: 100%; font: 15px/1.5 -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif; }
  #map { position: absolute; inset: 0; background: #e8eaed; }
  #panel {
    position: absolute; left: 0; right: 0; bottom: 0; padding: 12px 12px 18px;
    background: rgba(255,255,255,.96); border-radius: 16px 16px 0 0;
    box-shadow: 0 -2px 16px rgba(0,0,0,.18); max-height: 62%; overflow-y: auto;
  }
  #status { font-size: 13px; color: #555; margin-bottom: 8px; word-break: break-all; }
  #status b { color: #111; }
  .row { display: flex; gap: 8px; margin-bottom: 8px; }
  input {
    flex: 1; min-width: 0; padding: 11px 12px; font-size: 15px;
    border: 1px solid #ccc; border-radius: 10px; outline: none;
  }
  input:focus { border-color: #2b6cff; }
  button {
    padding: 11px 16px; font-size: 15px; border: 0; border-radius: 10px;
    background: #2b6cff; color: #fff; font-weight: 600; white-space: nowrap;
  }
  button:active { opacity: .75; }
  button.ghost { background: #eceff3; color: #333; }
  button:disabled { background: #c9d3e0; color: #fff; }
  #pick { font-size: 14px; color: #333; margin: 6px 0 10px; min-height: 22px; }
  #pick b { color: #2b6cff; }
  #results { margin-bottom: 6px; }
  .item {
    padding: 10px 12px; background: #f5f7fa; border-radius: 10px;
    margin-bottom: 6px; font-size: 14px; color: #333;
  }
  .item:active { background: #e6ecf5; }
  .hint { font-size: 12px; color: #888; margin-top: 4px; }
  #err { color: #d33; font-size: 13px; margin-top: 6px; white-space: pre-wrap; }
</style>
</head>
<body>
<div id="map"></div>
<div id="panel">
  <div id="status">加载中…</div>
  <div class="row">
    <input id="q" placeholder="搜索地点（中文分词较差，建议直接点地图）" autocomplete="off">
    <button id="search" class="ghost">搜索</button>
  </div>
  <div id="results"></div>
  <div id="pick">在地图上点一下选择终点</div>
  <div class="row">
    <button id="set" style="flex:1" disabled>设为目的地</button>
    <button id="clear" class="ghost">清除</button>
  </div>
  <div class="hint">停车时会自动清除目的地，每次开车前需重设。</div>
  <div id="err"></div>
</div>
<script src="https://api.mapbox.com/mapbox-gl-js/v3.6.0/mapbox-gl.js"></script>
<script>
const TOKEN = "__TOKEN__";
const CAR = { lat: __CAR_LAT__, lon: __CAR_LON__ };
let picked = null;
let destMarker = null;

const $ = id => document.getElementById(id);
const err = m => { $('err').textContent = m || ''; };
const fmt = d => (d < 1000 ? Math.round(d) + ' m' : (d / 1000).toFixed(1) + ' km');

function dist(a, b) {
  const R = 6371000, r = Math.PI / 180;
  const dLat = (b.lat - a.lat) * r, dLon = (b.lon - a.lon) * r;
  const s = Math.sin(dLat / 2) ** 2 +
            Math.cos(a.lat * r) * Math.cos(b.lat * r) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(s));
}

async function refresh() {
  try {
    const s = await (await fetch('/api/state')).json();
    const en = s.nav_enabled ? '开' : '关';
    let txt = '导航功能：<b>' + en + '</b>';
    if (s.dest) {
      const d = { lat: s.dest.latitude, lon: s.dest.longitude };
      txt += '　目的地：<b>' + d.lat.toFixed(5) + ', ' + d.lon.toFixed(5) + '</b>';
      txt += '　直线 ' + fmt(dist(s.car, d));
      if (map) setDestMarker(d);
    } else {
      txt += '　目的地：<b>未设置</b>';
    }
    $('status').innerHTML = txt;
  } catch (e) { err('读取状态失败: ' + e); }
}

let map = null;

function setDestMarker(p) {
  if (!map) return;
  if (!destMarker) {
    destMarker = new mapboxgl.Marker({ color: '#e8453c' }).setLngLat([p.lon, p.lat]).addTo(map);
  } else {
    destMarker.setLngLat([p.lon, p.lat]);
  }
}

function placePin(p) {
  picked = p;
  setDestMarker(p);
  $('pick').innerHTML = '终点：<b>' + p.lat.toFixed(5) + ', ' + p.lon.toFixed(5) + '</b>' +
                        '　距车 ' + fmt(dist(CAR, p));
  $('set').disabled = false;
}

try {
  mapboxgl.accessToken = TOKEN;
  map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v11',
    center: [CAR.lon, CAR.lat],
    zoom: 13,
    attributionControl: false,
  });
  map.addControl(new mapboxgl.NavigationControl({ showCompass: false }), 'top-right');
  new mapboxgl.Marker({ color: '#2b6cff' }).setLngLat([CAR.lon, CAR.lat])
    .setPopup(new mapboxgl.Popup().setText('车辆位置')).addTo(map);
  map.on('click', e => placePin({ lat: e.lngLat.lat, lon: e.lngLat.lng }));
} catch (e) {
  err('地图加载失败（手机需要能访问 api.mapbox.com）。\n可以直接用下面的搜索框，或手动输入经纬度。');
  map = null;
  $('q').placeholder = '输入 lat,lon 例如 30.0697,119.9506';
}

$('search').onclick = async () => {
  err('');
  const q = $('q').value.trim();
  if (!q) return;
  // 手动输入坐标
  const m = q.match(/^(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)$/);
  if (m) { placePin({ lat: +m[1], lon: +m[2] }); if (map) map.flyTo({ center: [+m[2], +m[1]], zoom: 14 }); return; }
  $('results').innerHTML = '<div class="item">搜索中…</div>';
  try {
    const url = 'https://api.mapbox.com/geocoding/v5/mapbox.places/' +
                encodeURIComponent(q) + '.json?access_token=' + TOKEN +
                '&limit=5&language=zh';
    const data = await (await fetch(url)).json();
    const fs = data.features || [];
    if (!fs.length) { $('results').innerHTML = '<div class="item">没找到，换个写法或直接点地图</div>'; return; }
    $('results').innerHTML = '';
    fs.forEach(f => {
      const el = document.createElement('div');
      el.className = 'item';
      el.textContent = f.place_name_zh || f.place_name || f.text;
      el.onclick = () => {
        placePin({ lat: f.center[1], lon: f.center[0] });
        if (map) map.flyTo({ center: f.center, zoom: 14 });
        $('results').innerHTML = '';
      };
      $('results').appendChild(el);
    });
  } catch (e) { err('搜索失败: ' + e); }
};

$('set').onclick = async () => {
  if (!picked) return;
  err('');
  $('set').disabled = true;
  try {
    const r = await fetch('/api/dest', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(picked),
    });
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || '失败');
    await refresh();
    $('pick').innerHTML = '<b style="color:#2f9e44">已设置</b>，起步后会自动算路';
  } catch (e) { err('设置失败: ' + e); $('set').disabled = false; }
};

$('clear').onclick = async () => {
  err('');
  try {
    await fetch('/api/clear', { method: 'POST' });
    if (destMarker) { destMarker.remove(); destMarker = null; }
    $('pick').textContent = '在地图上点一下选择终点';
    $('set').disabled = true;
    await refresh();
  } catch (e) { err('清除失败: ' + e); }
};

refresh();
setInterval(refresh, 10000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
  def log_message(self, fmt, *args):
    pass  # 安静点，别刷屏

  def _send(self, code, body, ctype="application/json; charset=utf-8"):
    data = body if isinstance(body, bytes) else body.encode("utf-8")
    self.send_response(code)
    self.send_header("Content-Type", ctype)
    self.send_header("Content-Length", str(len(data)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(data)

  def do_GET(self):
    if self.path in ("/", "/index.html"):
      p = Params()
      lat, lon = car_position(p)
      page = (PAGE.replace("__TOKEN__", read_token())
                  .replace("__CAR_LAT__", f"{lat:.6f}")
                  .replace("__CAR_LON__", f"{lon:.6f}"))
      self._send(200, page, "text/html; charset=utf-8")
    elif self.path.startswith("/api/state"):
      self._send(200, json.dumps(state()))
    else:
      self._send(404, json.dumps({"ok": False, "error": "not found"}))

  def do_POST(self):
    try:
      n = int(self.headers.get("Content-Length") or 0)
      body = json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
    except Exception:
      body = {}

    if self.path == "/api/dest":
      try:
        lat, lon = float(body["lat"]), float(body["lon"])
      except (KeyError, TypeError, ValueError):
        self._send(400, json.dumps({"ok": False, "error": "缺少 lat/lon"}))
        return
      if not (CN_LAT[0] <= lat <= CN_LAT[1] and CN_LON[0] <= lon <= CN_LON[1]):
        self._send(400, json.dumps({"ok": False, "error": "坐标不在中国范围内"}))
        return
      # ★ block=True 必须：非阻塞写之后立刻回读会命中旧缓存
      Params().put("NavDestination", {"latitude": lat, "longitude": lon}, block=True)
      self._send(200, json.dumps({"ok": True, "lat": lat, "lon": lon}))

    elif self.path == "/api/clear":
      Params().remove("NavDestination")
      self._send(200, json.dumps({"ok": True}))

    else:
      self._send(404, json.dumps({"ok": False, "error": "not found"}))


def local_ips():
  ips = []
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ips.append(s.getsockname()[0])
    s.close()
  except Exception:
    pass
  try:
    for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
      ip = info[4][0]
      if not ip.startswith("127.") and ip not in ips:
        ips.append(ip)
  except Exception:
    pass
  return ips


def main():
  # 端口被占用时不要退出（否则 manager 会一直报 Process Not Running 并反复重启），
  # 而是在进程内等一会儿再试。
  while True:
    try:
      srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
      break
    except OSError as e:
      print(f"[nav_web] bind {PORT} 失败: {e}，5s 后重试")
      time.sleep(5)
  print(f"[nav_web] 监听 0.0.0.0:{PORT}")

  # 把访问地址落到文件里，方便 SSH 时直接查（网络变了地址也会变）
  ips = local_ips()
  host = socket.gethostname()
  lines = [f"http://{ip}:{PORT}" for ip in ips] + [f"http://{host}.local:{PORT}"]
  for line in lines:
    print(f"[nav_web] 手机打开: {line}")
  try:
    with open("/data/nav_web.url", "w") as f:
      f.write("\n".join(lines) + "\n")
  except OSError:
    pass

  srv.serve_forever()


if __name__ == "__main__":
  main()
