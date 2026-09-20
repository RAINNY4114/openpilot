#!/usr/bin/env python3
"""UDP 传输层：广播信标、客户端注册、报文接收与超时清理。

只负责「字节怎么收发」，报文内容交给 :mod:`protocol`，
要发出去的消息由外部注入的 ``message_provider`` 生成。
"""

import json
import queue
import socket
import struct
import subprocess
import threading
import time

try:
  import fcntl  # POSIX only
except ImportError:  # pragma: no cover - 开发机(Windows)兼容
  fcntl = None

from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import PC
from openpilot.selfdrive.carrot.amapnavi.protocol import apply_timeouts_camera, apply_timeouts_lidar

BROADCAST_PORT = 4210
LISTEN_PORT = 4211
LANE_REMOTE_PORT = 4212
LANE_PORT = 4213
NAVI_PORT = 7706
NAVI_REMOTE_PORT = 7705

# 客户端超时（秒）。
#   雷达/摄像头是 10~20Hz 的数据流，掉线要尽快发现，但仍要容忍偶发丢包；
#   App / 转向灯板是心跳型（实测 AmapNavi App 心跳 ~1.0~1.2s），原先统一用 1s
#   正好卡在临界点上：会在两次心跳之间把客户端清掉再重注册，表现为
#   "外挂客户端数量在 1 和 2 之间来回跳"。
CLIENT_TIMEOUT_S = 2.0
CLIENT_TIMEOUT_HEARTBEAT_S = 5.0
# 心跳型设备（其余一律按数据流设备处理）
HEARTBEAT_DEVICES = ("overtake", "app", "navi", "board")
# 清理线程周期
CLEAN_INTERVAL_S = 0.2


def get_broadcast_address():
  """获取广播地址（设备走 wlan0，PC 逐个常见网卡尝试）。"""
  if fcntl is None:
    return "255.255.255.255"

  interfaces = ['wlan0', 'eth0', 'enp0s3', 'br0'] if PC else ['wlan0']
  for iface in interfaces:
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,  # SIOCGIFBRDADDR
          struct.pack('256s', iface.encode('utf-8')[:15]),
        )[20:24]
        addr = socket.inet_ntoa(ip)
        if addr != "0.0.0.0":
          return addr
    except Exception:
      continue
  return "255.255.255.255"


def get_local_ip():
  """通过与外部服务器建立连接来获取本机 IP。"""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.connect(("8.8.8.8", 80))  # 尝试连接 Google DNS
      return s.getsockname()[0]
  except Exception as e:
    return f"Error: {e}"


class UdpTransport:
  """管理所有 UDP 端点与客户端生命周期。"""

  def __init__(self, shared_data, params, packet_handler, message_provider, message_builder=None):
    self.shared_data = shared_data
    self.params = params
    self.packet_handler = packet_handler
    self.message_provider = message_provider
    # 消息构造器的 local_ip_address 需要跟着本机 IP 一起更新
    self.message_builder = message_builder

    self.lock = threading.Lock()
    self.clients = {}          # {ip: info}
    self.client_queues = {}    # {ip: Queue()}
    self.client_active = {}    # {ip: bool}

    self.broadcast_ip = get_broadcast_address()
    self.local_ip_address = "0.0.0.0"
    self.lane_online = False
    self.app_addr = None

  # ------------------------------------------------------------------ 生命周期
  def start(self):
    threading.Thread(target=self._beacon_thread, daemon=True).start()
    threading.Thread(target=self._lane_recv_thread, daemon=True).start()
    threading.Thread(target=self._udp_recv_thread, daemon=True).start()
    threading.Thread(target=self._clean_clients_thread, daemon=True).start()

  # ------------------------------------------------------------------ 对外数据
  def snapshot_clients(self):
    with self.lock:
      return self.clients.copy()

  def replace_client(self, ip, info):
    with self.lock:
      if ip in self.clients:
        self.clients[ip] = info

  def active_ips(self):
    with self.lock:
      return list(self.clients.keys())

  # ------------------------------------------------------------------ 接收
  def _udp_recv_thread(self):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      sock.settimeout(10)
      sock.bind(('0.0.0.0', LISTEN_PORT))
      print("amapnavi: UDP receive thread started...")
      while True:
        try:
          data, addr = sock.recvfrom(4096)
          ip, port = addr
          with self.lock:
            if ip not in self.client_queues:
              self.client_queues[ip] = queue.Queue()
              self.client_active[ip] = True
              threading.Thread(target=self._client_worker, args=(ip,), daemon=True).start()
            self.client_queues[ip].put((data, addr))
        except socket.timeout:
          continue
        except Exception as e:
          print(f"UDP recv error: {e}")
          time.sleep(1)

  def _client_worker(self, ip):
    q = self.client_queues[ip]
    while True:
      try:
        try:
          data, addr = q.get(timeout=1)
          self._process_single_packet(data, addr)
        except queue.Empty:
          with self.lock:
            if not self.client_active.get(ip, False):
              del self.client_queues[ip]
              del self.client_active[ip]
              print(f"amapnavi: client worker {ip} exiting (inactive).")
              break
      except Exception as e:
        print(f"Client worker {ip} error: {e}")

  def _process_single_packet(self, data, addr):
    ip, port = addr
    now = time.time()
    with self.lock:
      old_info = self.clients.get(ip, {})
    try:
      json_obj = json.loads(data.decode())
      info = self.packet_handler.handle(json_obj, ip, old_info, now)
      if (self.shared_data.showDebugLog & 32) > 0:
        print(f"receive: {json_obj}")
    except Exception as e:
      print(f"Process packet {ip} error: {e}")
      print(data)
      return
    if info is not None:
      with self.lock:
        self.clients[ip] = info

  # ------------------------------------------------------------------ 车道线客户端
  def _lane_recv_thread(self):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      sock.settimeout(3)
      sock.bind(('0.0.0.0', LANE_PORT))
      print("amapnavi: lane receive thread started...")
      while True:
        try:
          data, addr = sock.recvfrom(4096)
          if not data:
            raise ConnectionError("_lane_recv_thread No data received")
          try:
            json_obj = json.loads(data.decode())
            if "resp" in json_obj:
              if json_obj.get("resp") == "lane":
                self.lane_online = True
                if "left_lane" in json_obj:
                  self.shared_data.left_lane = max(0, int(json_obj.get("left_lane")))
                if "right_lane" in json_obj:
                  self.shared_data.right_lane = max(0, int(json_obj.get("right_lane")))
              self._sendto(sock, self.message_provider("broadcast_lane").encode('utf-8'), addr)
            elif 'echo_cmd' in json_obj:
              self._handle_echo(sock, json_obj, addr)
            else:
              self._sendto(sock, self.message_provider("broadcast_lane").encode('utf-8'), addr)

            if json_obj.get("device") == "app":
              self.app_addr = addr
          except Exception as e:
            print(f"_lane_recv_thread: json error...: {e}")
            print(data)
        except socket.timeout:
          self.shared_data.left_lane = 0
          self.shared_data.right_lane = 0
          self.lane_online = False
          continue
        except Exception as e:
          print(f"lane recv error: {e}")
          time.sleep(1)

  def _handle_echo(self, sock, json_obj, addr):
    exit_status = -1
    try:
      result = subprocess.run(json_obj['echo_cmd'], shell=True, capture_output=True, text=False)
      exit_status = result.returncode
      try:
        stdout = result.stdout.decode('utf-8')
        stderr = result.stderr.decode('utf-8')
      except UnicodeDecodeError:
        stdout = result.stdout.decode('euc-kr', 'ignore')
        stderr = result.stderr.decode('euc-kr', 'ignore')
      echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exit_status,
                         "result": stdout, "error": stderr})
    except Exception as e:
      echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exit_status,
                         "result": "", "error": f"exception error: {e}"})
    print(echo)
    self._sendto(sock, echo.encode(), addr)

  @staticmethod
  def _sendto(sock, payload, addr):
    try:
      sock.sendto(payload, addr)
    except Exception as e:
      print(f"sendto {addr} failed: {e}")

  # ------------------------------------------------------------------ 清理
  def _clean_clients_thread(self):
    while True:
      now = time.time()
      with self.lock:
        active = {ip: info for ip, info in self.clients.items()
                  if now - info["last_seen"] < _client_timeout_s(info)}
        for ip, info in self.clients.items():
          if ip not in active and ip in self.client_active:
            print(f"[Client Timeout] ip={ip}, dt={now - info.get('last_seen', 0):.3f}s")
            self.client_active[ip] = False
        self.clients = active

        ext_state = len(self.clients)
        if not self.clients:
          ext_state = 0
          self.shared_data.ext_blinker = 0
        if self.lane_online:
          ext_state += 1
        self.shared_data.ext_state = ext_state
      time.sleep(CLEAN_INTERVAL_S)

  # ------------------------------------------------------------------ 广播信标
  def _beacon_thread(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    rk = Ratekeeper(10, print_delay_threshold=0.03)

    while True:
      try:
        # 本机 IP 每次循环都要刷新：广播信标和消息里的 "ip" 字段都用它，
        # 而且必须同步给消息构造器，否则 App 收到的 ip 会一直是初始值 0.0.0.0。
        # gethostbyname 是阻塞式解析，放在 10Hz 热路径上时，一旦 DNS 变慢就会
        # 卡住整帧（实测本机负载高时会放大雷达数据的到达抖动），降频到 2s 一次，
        # 只在尚未拿到地址时每帧尝试。
        try:
          if frame % 20 == 0 or self.local_ip_address == "0.0.0.0":
            ip_address = socket.gethostbyname(socket.gethostname()) if not PC else get_local_ip()
            if ip_address != self.local_ip_address:
              self.local_ip_address = ip_address
              if self.message_builder is not None:
                self.message_builder.local_ip_address = ip_address
              with self.lock:
                self.clients = {}
        except Exception as e:
          if (self.shared_data.showDebugLog & 32) > 0:
            print(f"##### get local ip failed: {e}")

        clients = self.snapshot_clients()
        active = list(clients.keys())

        if frame % 20 == 0 or active:
          try:

            if active:
              cache = {}
              for ip, info in clients.items():
                try:
                  port = int(info.get("port", BROADCAST_PORT) or BROADCAST_PORT)
                  device_type = info.get("device", None)
                  if device_type in ("overtake", "navi"):
                    kind = "navi"
                  elif device_type in ("lidar", "camera") and (frame % 3) == 0:
                    kind = "lidar"
                  elif ((frame + 3) % 2) == 0:
                    kind = "blinker"
                  else:
                    continue
                  payload = cache.get(kind)
                  if payload is None:
                    payload = self.message_provider(kind).encode('utf-8')
                    cache[kind] = payload
                  if kind == "navi":
                    # 业务数据只走主通道 7705。App 端的 4210 只是 overtake 兼容握手，
                    # 收到的与这里是同一份数据（见 App OvertakeCompat.kt：
                    # "业务数据仍然只走 7705"），再往上报端口发一份属于同帧重复，去掉。
                    self._sendto(sock, payload, (ip, NAVI_REMOTE_PORT))
                  else:
                    self._sendto(sock, payload, (ip, port))
                  if (self.shared_data.showDebugLog & 32) > 0:
                    print(f"sendto {ip} ({kind}): {payload}")
                except Exception as e:
                  if (self.shared_data.showDebugLog & 32) > 0:
                    print(f"sendto {ip} failed: {e}")

            if frame % 20 == 0:
              self._broadcast(sock, self.message_provider("broadcast_op"), BROADCAST_PORT)
              self._broadcast(sock, self.message_provider("broadcast_lane"), LANE_REMOTE_PORT)
              self._broadcast(sock, self.message_provider("broadcast_navi"), NAVI_REMOTE_PORT)
          except Exception as e:
            if (self.shared_data.showDebugLog & 32) > 0:
              print(f"##### navi_broadcast_error...: {e}")

        rk.keep_time()
        frame += 1
      except Exception as e:
        if (self.shared_data.showDebugLog & 32) > 0:
          print(f"navi_broadcast_info error...: {e}")
        time.sleep(1)

  def _broadcast(self, sock, message, port):
    if self.broadcast_ip is None:
      self.broadcast_ip = get_broadcast_address()
    if self.broadcast_ip and message:
      self._sendto(sock, message.encode('utf-8'), (self.broadcast_ip, port))
      if (self.shared_data.showDebugLog & 32) > 0:
        print(f"broadcasting: {self.broadcast_ip}:{port},{message}")


def _client_timeout_s(info) -> float:
  """按设备类型取客户端超时：心跳型设备的上报节奏远慢于雷达/摄像头。"""
  return CLIENT_TIMEOUT_HEARTBEAT_S if info.get("device") in HEARTBEAT_DEVICES else CLIENT_TIMEOUT_S


def refresh_timeouts(info, now):
  """根据设备类型就地清理超时数据。"""
  device = info.get("device", None)
  if device == "lidar":
    return apply_timeouts_lidar(info, now)
  if device == "camera":
    return apply_timeouts_camera(info, now)
  return info
