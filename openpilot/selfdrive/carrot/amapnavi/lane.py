#!/usr/bin/env python3
"""amapnavi 车道线视频流服务。

由 cpv9-dev 的 ``openpilot/selfdrive/carrot/lane.py`` 移植而来，改动如下：

* **去掉 Flask**：改用标准库 ``http.server.ThreadingHTTPServer``，零第三方依赖
  （与 xiaoge / recovery 服务的做法一致）。对外 URL 完全保持兼容，
  原有 Android 端 ``/`` ``/status`` ``/roadrgb.jpg`` ``/roadgray.jpg`` 无需改动。
* **OpenCV 的处理方式参照 carrot/xiaoge**：
  ``cv2`` 由 ``launch_chffrplus.sh`` 从 ``opencv-python-headless==4.13.0.92``
  安装到 pydeps，**不会在运行时 pip install**；缺失时按 xiaoge 的写法给出明确提示后退出。
* NV12 读取用纯 numpy 实现（参照 ``xiaoge/nv12.py``），正确处理 stride / uv_offset。

单独进程运行：
    python -m openpilot.selfdrive.carrot.amapnavi.lane --port 8888
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

# 注意：不要在模块导入阶段 raise SystemExit。
# manager 的 PythonProcess.prepare() 会 importlib.import_module() 预导入本模块，
# 而 SystemExit 属于 BaseException，不会被它的 `except Exception` 捕获，
# 会导致整个 manager 启动失败。这里只记录错误，真正的退出放到 main() 里。
try:
  import cv2
  cv2.setNumThreads(1)
  CV2_IMPORT_ERROR = None
except ModuleNotFoundError as error:
  if error.name != "cv2":
    raise
  cv2 = None
  CV2_IMPORT_ERROR = (
    "lane 视频流服务需要 OpenCV。请通过正常启动器重启 openpilot，"
    "以便把内置的 wheel 安装到 pydeps；开发环境下可安装 xiaoge/requirements.txt。"
  )

DEFAULT_PORT = 8888
TARGET_WIDTH = 416
TARGET_HEIGHT = 416
JPEG_QUALITY = 50

# 请求统计窗口
REQUEST_WINDOW = 2.0
# 取流间隔（秒）：按客户端实际请求速率动态调整
# 统计窗口内没有任何请求时退回的保守间隔（2fps）
DEFAULT_FRAME_TIME = 0.5
MIN_FRAME_TIME = 0.05      # 最快 20fps
MAX_FRAME_TIME = 1.0       # 最慢 1fps
# 客户端超过该时间没有请求就停止取流，避免空转耗 CPU
IDLE_TIMEOUT = 2.0
# 取流失败后的退避
RECV_RETRY_SLEEP = 0.05
# 持续这么久收不到帧就认为 camerad 重启了，重新连接
CAMERA_RECONNECT_TIMEOUT = 5.0

# 说明：源码里 "/" 只是一个纯文本状态页（给 Android App 单帧抓取用），
# 直接打开看不到画面。这里改成「状态行 + 自动刷新的路面图像」，方便浏览器里调试。
HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Lane Detect</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            margin: 0;
            background: black;
            color: #0f0;
            font-family: monospace;
            font-size: 14px;
        }
        #text {
            padding: 6px 10px;
            white-space: pre-wrap;
        }
        #frame {
            display: block;
            width: 100%;
            max-width: 1280px;
            margin: 0 auto;
        }
    </style>
</head>
<body>
<div id="text">waiting app connect...</div>
<img id="frame" src="/roadrgb.jpg" alt="road camera">
<script>
    var el = document.getElementById("text");
    var img = document.getElementById("frame");
    function update() {
        fetch("/status")
            .then(function (r) { return r.text(); })
            .then(function (t) { el.textContent = t; })
            .catch(function () {});
    }
    function refresh() {
        // 加时间戳绕开浏览器缓存
        img.src = "/roadrgb.jpg?t=" + Date.now();
    }
    update();
    setInterval(update, 1000);   // 状态每秒刷新
    setInterval(refresh, 400);   // 图像约 2.5fps
</script>
</body>
</html>
"""


def nv12_y_plane(data, width, height, stride):
  """取出可见的 Y 平面（丢掉行尾 padding）。"""
  raw = np.frombuffer(data, dtype=np.uint8)
  y_size = stride * height
  if raw.size < y_size:
    raise ValueError(f"short NV12 Y plane: {raw.size} < {y_size}")
  return raw[:y_size].reshape(height, stride)[:, :width]


def pack_nv12(buf):
  """把 VisionIPC 缓冲区打包成紧凑 NV12（Y + UV），正确处理 stride / uv_offset。"""
  y, uv = nv12_planes(buf)
  return np.vstack((y, uv))


def nv12_planes(buf):
  """取出可见的 Y / UV 平面（去掉行尾 padding，正确处理 uv_offset）。"""
  width, height, stride = buf.width, buf.height, buf.stride
  y = nv12_y_plane(buf.data, width, height, stride)
  if width % 2 or height % 2:
    raise ValueError("NV12 width and height must be even")

  uv_offset = int(getattr(buf, "uv_offset", stride * height) or (stride * height))
  raw = np.frombuffer(buf.data, dtype=np.uint8)
  uv_end = uv_offset + stride * (height // 2)
  if raw.size < uv_end:
    raise ValueError(f"short NV12 UV plane: {raw.size} < {uv_end}")
  uv = raw[uv_offset:uv_end].reshape(height // 2, stride)[:, :width]
  return y, uv


def nv12_to_small_bgr(buf, target_w, target_h):
  """先把 NV12 整数倍抽稀，再转 BGR。

  直接对 1928x1208 全分辨率做 cvtColor + resize 要 50ms 左右；
  先抽稀到接近目标尺寸再转换，只有几毫秒，画质几乎无差别。

  抽稀规则：Y 和 UV 用同一个整数步长。UV 本身是半分辨率，
  所以 uv[::scale] 正好对应 y[::scale] 的色度位置。
  """
  y, uv = nv12_planes(buf)
  height, width = y.shape

  # 抽稀步长取「最接近目标尺寸」的整数倍，而不是「不小于目标」的整数倍。
  # 原因：剩下的缩放交给 cv2.resize 时，缩放比越大 INTER_AREA 越慢
  # （实测 964x604 -> 416x416 要 21ms，而 642x402 -> 416x416 只要 4.7ms）。
  # 中间图略小于目标也没关系，最后一步轻微放大对画质几乎无影响。
  scale = max(1, int(round(min(width / target_w, height / target_h))))
  y_small = y[::scale, ::scale]
  uv_small = uv[::scale, ::scale]

  # NV12 要求宽高均为偶数
  h = (y_small.shape[0] // 2) * 2
  w = (y_small.shape[1] // 2) * 2
  y_small = y_small[:h, :w]
  uv_small = uv_small[:h // 2, :w]

  return cv2.cvtColor(np.vstack((y_small, uv_small)), cv2.COLOR_YUV2BGR_NV12)


def y_plane_to_gray_jpeg(buf, target_w, target_h, quality):
  """灰度 JPEG：整数倍下采样后中心裁剪，避免 cv2.resize（低 CPU）。"""
  y_plane = nv12_y_plane(buf.data, buf.width, buf.height, buf.stride)

  scale_x = target_w / buf.width
  scale_y = target_h / buf.height
  scale = max(scale_x, scale_y)
  step_x = max(1, int(1 / scale))
  step_y = max(1, int(1 / scale))

  y_ds = y_plane[0:buf.height:step_y, 0:buf.width:step_x]
  ds_h, ds_w = y_ds.shape

  start_x = max(0, (ds_w - target_w) // 2)
  start_y = max(0, (ds_h - target_h) // 2)
  y_crop = y_ds[start_y:start_y + target_h, start_x:start_x + target_w]
  # 确保偶数尺寸（编码器要求）
  y_crop = y_crop[:target_h & ~1, :target_w & ~1]

  ok, jpg = cv2.imencode(".jpg", y_crop, [cv2.IMWRITE_JPEG_QUALITY, quality])
  return jpg.tobytes() if ok else None


def encode_jpeg(bgr, target_w, target_h, quality):
  """固定尺寸 JPEG 输出（比例被强制拉伸，与源实现保持一致）。"""
  if bgr.shape[1] != target_w or bgr.shape[0] != target_h:
    bgr = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
  ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
  return jpg.tobytes() if ok else None


def blank_jpeg(target_w, target_h, quality, gray):
  """没有可用帧时返回纯黑占位图。"""
  placeholder = np.zeros((target_h, target_w), dtype=np.uint8)
  ok, jpg = cv2.imencode(".jpg", placeholder, [cv2.IMWRITE_JPEG_QUALITY, quality])
  return jpg.tobytes() if ok else b""


class LaneStreamServer:
  """路面 JPEG 抓取服务（单帧抓取，客户端轮询）。"""

  def __init__(self, port=DEFAULT_PORT, target_width=TARGET_WIDTH,
               target_height=TARGET_HEIGHT, quality=JPEG_QUALITY):
    self.port = port
    self.target_width = target_width
    self.target_height = target_height
    self.quality = quality

    self.frame_lock = threading.Lock()
    self.req_lock = threading.Lock()
    self.status_lock = threading.Lock()

    self.latest_color = None
    self.latest_gray = None
    self.gray_mode = False
    self.last_snapshot_time = 0.0

    self.req_count = 0
    self.latest_req_count = 0
    self.req_window_start = time.monotonic()
    self.req_frame_time = 0.05

    self.status_text = "waiting app connect..."

  def _set_status(self, text):
    with self.status_lock:
      self.status_text = text

  # ------------------------------------------------------------------ 相机线程
  def camera_thread(self):
    from msgq.visionipc import VisionIpcClient, VisionStreamType

    vipc_client = None
    last_print = time.monotonic()
    frame_times = []
    retries = 0
    empty_since = None

    while True:
      try:
        # 本进程通常和 camerad 同时启动，camerad 可能还没就绪，
        # 所以这里必须一直重试，不能试几次就放弃。
        if vipc_client is None:
          client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, False)
          if client.connect(False):
            vipc_client = client
            retries = 0
            empty_since = None
            print(f"[lane] VisionIPC 已连接: {client.width}x{client.height} stride={client.stride}")
          else:
            retries += 1
            self._set_status(f"camera connecting... (retry {retries})")
            time.sleep(1.0)
            continue

        # 没有客户端请求时不干活
        if time.monotonic() - self.last_snapshot_time > IDLE_TIMEOUT:
          time.sleep(0.1)
          continue

        buf = vipc_client.recv()
        if buf is None:
          now = time.monotonic()
          if empty_since is None:
            empty_since = now
          elif now - empty_since > CAMERA_RECONNECT_TIMEOUT:
            # camerad 可能重启了，丢弃当前连接重建
            print("[lane] 长时间没有收到帧，重新连接 VisionIPC")
            vipc_client = None
            empty_since = None
            self._set_status("camera reconnecting...")
          time.sleep(RECV_RETRY_SLEEP)
          continue
        empty_since = None

        started = time.monotonic()

        with self.req_lock:
          gray_mode = self.gray_mode

        if gray_mode:
          jpeg = y_plane_to_gray_jpeg(buf, self.target_width, self.target_height, self.quality)
          if jpeg is None:
            continue
          with self.frame_lock:
            self.latest_gray = jpeg
        else:
          # 先抽稀再转 BGR，避免在全分辨率上做 cvtColor + resize
          bgr = nv12_to_small_bgr(buf, self.target_width, self.target_height)
          jpeg = encode_jpeg(bgr, self.target_width, self.target_height, self.quality)
          del bgr
          if jpeg is None:
            continue
          with self.frame_lock:
            self.latest_color = jpeg

        elapsed = time.monotonic() - started
        frame_times.append(elapsed * 1000)

        now = time.monotonic()
        if now - last_print > 2.0:
          if frame_times:
            with self.status_lock:
              self.status_text = (
                f"JPEG {self.target_width}x{self.target_height} | "
                f"avg {np.mean(frame_times):.1f} ms | "
                f"FPS {len(frame_times) / 2:.1f}"
              )
          frame_times.clear()
          last_print = now

        sleep_time = self.req_frame_time - elapsed
        if sleep_time > 0:
          time.sleep(sleep_time)
      except Exception as e:
        print(f"[lane] camera thread error: {e}")
        time.sleep(0.5)

  # ------------------------------------------------------------------ HTTP
  def create_handler(self):
    """创建请求处理器（闭包捕获 server 实例，替代 Flask 的全局变量）。"""
    server = self

    class LaneHTTPHandler(BaseHTTPRequestHandler):
      protocol_version = "HTTP/1.1"

      def log_message(self, format, *args):
        pass

      def _send_bytes(self, body, content_type):
        try:
          self.send_response(200)
          self.send_header("Content-Type", content_type)
          self.send_header("Content-Length", str(len(body)))
          self.send_header("Cache-Control", "no-store")
          self.send_header("Access-Control-Allow-Origin", "*")
          self.end_headers()
          self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
          pass

      def _send_text(self, text):
        self._send_bytes(text.encode("utf-8"), "text/plain; charset=utf-8")

      def do_GET(self):
        path = self.path.split("?", 1)[0]
        try:
          if path == "/":
            self._send_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
          elif path == "/status":
            with server.status_lock:
              self._send_text(server.status_text)
          elif path == "/roadrgb.jpg":
            self._send_bytes(server.snapshot_frame(gray=False), "image/jpeg")
          elif path == "/roadgray.jpg":
            self._send_bytes(server.snapshot_frame(gray=True), "image/jpeg")
          else:
            self.send_error(404, "not found")
        except (BrokenPipeError, ConnectionResetError):
          pass
        except Exception as e:
          print(f"[lane] request {path} failed: {e}")
          try:
            self.send_error(500, "internal error")
          except Exception:
            pass

    return LaneHTTPHandler

  def snapshot_frame(self, gray):
    """记录一次抓取请求并自适应调整取流间隔，返回最新 JPEG。"""
    self.last_snapshot_time = time.monotonic()
    with self.req_lock:
      self.gray_mode = gray

      now = time.monotonic()
      if now - self.req_window_start > REQUEST_WINDOW:
        self.req_window_start = now
        self.latest_req_count = self.req_count
        self.req_count = 0
        if self.latest_req_count >= 1:
          # 按客户端实际请求速率取流（留 2 倍余量，保证有新鲜帧）
          target = 2.0 / self.latest_req_count
          self.req_frame_time = max(MIN_FRAME_TIME, min(MAX_FRAME_TIME, target))
        else:
          # 本窗口没有任何请求：退回保守间隔。
          # 不重置的话会沿用上一次的间隔（可能很小），
          # 导致客户端只要 2fps 时相机线程仍按 20fps 编码，白白耗 CPU。
          self.req_frame_time = DEFAULT_FRAME_TIME
      self.req_count += 1

    with self.frame_lock:
      jpeg = self.latest_gray if gray else self.latest_color

    if jpeg is None:
      jpeg = blank_jpeg(self.target_width, self.target_height, self.quality, gray)
    return jpeg

  def serve(self, host="0.0.0.0"):
    """启动 HTTP 服务（阻塞）。"""
    threading.Thread(target=self.camera_thread, daemon=True).start()
    time.sleep(1)

    print("=" * 60)
    print("车道线服务程序")
    print(f"访问: http://{host}:{self.port}")
    print("=" * 60)

    httpd = ThreadingHTTPServer((host, self.port), self.create_handler())
    httpd.daemon_threads = True
    try:
      httpd.serve_forever()
    finally:
      httpd.server_close()


def main():
  if cv2 is None:
    raise SystemExit(f"[lane] {CV2_IMPORT_ERROR}")

  parser = argparse.ArgumentParser(description="amapnavi lane MJPEG 单帧抓取服务")
  parser.add_argument("--host", default="0.0.0.0")
  parser.add_argument("--port", type=int, default=DEFAULT_PORT)
  parser.add_argument("--width", type=int, default=TARGET_WIDTH)
  parser.add_argument("--height", type=int, default=TARGET_HEIGHT)
  parser.add_argument("--quality", type=int, default=JPEG_QUALITY)
  args = parser.parse_args()

  LaneStreamServer(
    port=args.port, target_width=args.width, target_height=args.height, quality=args.quality,
  ).serve(args.host)


if __name__ == "__main__":
  main()
