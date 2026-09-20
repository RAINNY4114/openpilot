#!/usr/bin/env python3
"""Web 界面模块 - amapnavi 参数设置页面。

由 cpv9-dev 的 ``openpilot/selfdrive/carrot/web_interface.py`` 移植而来（文件名改为 config_web.py）。
用于在 Web 页面查看/修改由 ``UnifiedParams``(nav_params.json) 管理的 amapnavi 参数。

注意：本 fork 的 ``openpilot.common.params.Params`` 是带类型校验的新版 API，
``get()`` 返回已解码的 Python 值，``put()`` 要求传入与 key 类型匹配的值。
因此源码里基于 bytes 的 get/put 不能直接使用，下面按类型分别做了转换。

本服务只管理 ``config.py``(nav_params.json) 这一套参数：
网页上的「导出配置 / 恢复配置」备份与恢复的也只是这套参数，
**不涉及 openpilot 系统参数**（原 cp 系统的参数不走这个渠道）。

接口列表：
  GET  /                       -> 重定向到 /nav_params
  GET  /nav_params             -> 参数设置页面 (nav_params.html)
  GET  /nav_params_data        -> 当前配置 (JSON)，供页面接口使用
  GET  /nav_params_download    -> 当前配置以 JSON 文件下载（备份，兼容 iOS Safari）
  GET  /get_default_nav_params -> 默认配置 (JSON)
  POST /nav_params             -> 保存配置；{"reset": "true"} 时恢复默认值
"""

import json
import os
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

from openpilot.selfdrive.carrot.amapnavi.config import UnifiedParams

DEFAULT_HTTP_PORT = 8088


class ConfigWeb:
    """amapnavi 参数配置 Web 服务处理器。

    :param controller: 拥有 ``.params``(UnifiedParams) 属性的对象，如 ``AmapNaviServ`` / ``CarrotMan``；
                       不传时直接使用 ``UnifiedParams()`` 单例。
    :param port: HTTP 端口（默认 8088）
    :param params: 也可以直接传入 UnifiedParams 实例。
    """

    def __init__(self, controller=None, port=DEFAULT_HTTP_PORT, params=None):
        self.controller = controller
        self.web_server = None
        self.port = port
        self.params = params
        if self.params is None:
          self.params = getattr(controller, "params", None)
        if self.params is None:
          self.params = UnifiedParams()

    def create_web_handler(self):
        """创建Web处理器"""
        unified_params = self.params
        controller = self.controller

        class ConfigWebRequestHandler(BaseHTTPRequestHandler):
            def safe_write(self, data):
                """安全写入数据，处理连接中断"""
                try:
                    if isinstance(data, str):
                        data = data.encode('utf-8')
                    self.wfile.write(data)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                except Exception as e:
                    print(f"写入数据时发生错误: {e}")

            def log_message(self, format, *args):
                """自定义日志格式，避免每次请求都打印到控制台"""
                pass

            def do_OPTIONS(self):
                """处理CORS预检请求"""
                try:
                    self.send_response(200)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, DELETE')
                    self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
                    self.send_header('Access-Control-Max-Age', '86400')
                    self.end_headers()
                except Exception as e:
                    print(f"OPTIONS请求处理错误: {e}")

            def do_GET(self):
                """处理GET请求"""
                try:
                    if self.path == '/':
                        self.send_response(301)
                        self.send_header('Location', '/nav_params')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                    elif self.path == '/radar':
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Content-type', 'text/html; charset=utf-8')
                        self.end_headers()
                        self.send_radar_page()
                    elif self.path == '/radar_data':
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Content-type', 'application/json; charset=utf-8')
                        self.send_header('Cache-Control', 'no-cache')
                        self.end_headers()
                        self.send_radar_data()
                    elif self.path == '/nav_params':
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Content-type', 'text/html; charset=utf-8')
                        self.end_headers()
                        self.send_nav_params_page()
                    elif self.path == '/nav_params_data':
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Content-type', 'application/json; charset=utf-8')
                        self.end_headers()
                        self.send_nav_params_data()
                    elif self.path == '/get_default_nav_params':
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Content-type', 'application/json; charset=utf-8')
                        self.end_headers()
                        self.send_default_nav_params()
                    elif self.path == '/nav_params_download':
                        self.send_nav_params_file()
                    else:
                        self.send_error(404, f"{self.path} Page not found")

                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端连接中断: {e}")
                except IOError as e:
                    print(f"IO错误: {e}")
                    self.send_error(500, "Internal server erro")
                except Exception as e:
                    print(f"未预期的错误: {e}")
                    self.send_error(500, f"Internal server erro: {str(e)}")

            def do_POST(self):
                """处理POST请求"""
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(post_data) if post_data else {}

                    if self.path == '/nav_params':
                        self.handle_nav_params(data)
                    else:
                        self.send_error(404, "API not found")
                except json.JSONDecodeError:
                    self.send_error(400, "JSON format error")
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端连接中断: {e}")
                except Exception as e:
                    print(f"请求处理错误: {e}")
                    self.send_error(500, f"Internal server erro: {str(e)}")

            def send_nav_params_file(self):
                """下载当前配置为 JSON 文件（带 Content-Disposition，兼容 iOS Safari）"""
                try:
                    unified_params._match_system_param()
                    payload = json.dumps(unified_params.nav_data, ensure_ascii=False, indent=2)
                    body = payload.encode('utf-8')
                    filename = time.strftime("amapnavi_params_%Y%m%d_%H%M%S.json")
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
                    self.send_header('Content-Length', str(len(body)))
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.safe_write(body)
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送配置文件错误: {e}")

            def send_nav_params_page(self):
                """发送 amapnavi 参数设置页面"""
                try:
                    html = self.get_html_content('nav_params.html')
                    self.safe_write(html)
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送参数页面错误: {e}")

            def send_default_nav_params(self):
                """发送默认的 amapnavi 参数数据"""
                try:
                    default_data = unified_params._get_default_nav_data()
                    self.safe_write(json.dumps(default_data, ensure_ascii=False))
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送默认参数错误: {e}")

            def send_nav_params_data(self):
                """发送 amapnavi 参数数据"""
                try:
                    unified_params._match_system_param()
                    nav_data = unified_params.nav_data
                    self.safe_write(json.dumps(nav_data, ensure_ascii=False))
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送参数数据错误: {e}")

            def handle_nav_params(self, data):
                """处理 amapnavi 参数更新 - 复用 UnifiedParams 逻辑"""
                try:
                    if not data:
                        self.send_json_response({'status': 'error', 'message': '无效的数据'})
                        return

                    if 'reset' in data and data['reset'] == 'true':
                        default_data = unified_params._get_default_nav_data()
                        unified_params.nav_data = default_data
                        unified_params._save_nav_data()
                        unified_params._save_system_param()
                        self.send_json_response({'status': 'success', 'message': '参数已恢复默认'})
                    else:
                        unified_params.nav_data = data
                        unified_params._save_nav_data()
                        unified_params._save_system_param()
                        self.send_json_response({'status': 'success', 'message': '参数已保存'})
                except Exception as e:
                    self.send_json_response({'status': 'error', 'message': f'保存失败: {str(e)}'})

            def send_json_response(self, data):
                """发送JSON响应"""
                try:
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json; charset=utf-8')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.safe_write(json.dumps(data, ensure_ascii=False))
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送JSON响应错误: {e}")

            def send_radar_page(self):
                """发送雷达数据显示页面（没有 radar.html 时返回提示信息）"""
                try:
                    html = self.get_html_content('radar.html')
                    self.safe_write(html)
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送雷达页面错误: {e}")

            def send_radar_data(self):
                """发送雷达数据JSON（仅当存在 controller.get_radar_data() 时）"""
                try:
                    if controller is None or not hasattr(controller, 'get_radar_data'):
                        radar_data = {'points': [], 'tracks': [], 'leads': [], 'timestamp': 0.0, 'error': 'no radar source'}
                    else:
                        radar_data = controller.get_radar_data()
                    self.safe_write(json.dumps(radar_data, ensure_ascii=False))
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"客户端断开连接: {e}")
                except Exception as e:
                    print(f"发送雷达数据错误: {e}")

            def get_html_content(self, filename):
                """HTML文件读取加载（与本模块同目录）"""
                try:
                    base_dir = os.path.abspath(os.path.dirname(__file__))
                    html_file_path = os.path.join(base_dir, filename)

                    if not os.path.exists(html_file_path):
                        return f"<html><body><h1>错误：文件 {filename} 不存在于路径 {html_file_path}</h1></body></html>"

                    with open(html_file_path, 'r', encoding='utf-8') as f:
                        return f.read()
                except FileNotFoundError:
                    return f"<html><body><h1>错误：未找到 {filename} 文件</h1></body></html>"
                except Exception as e:
                    return f"<html><body><h1>错误：读取文件失败 - {str(e)}</h1></body></html>"

        return ConfigWebRequestHandler

    def start_web_server(self):
        """启动Web服务器"""
        try:
            handler = self.create_web_handler()
            self.web_server = HTTPServer(('0.0.0.0', self.port), handler)
            self.web_server.timeout = 30
            print(f"🌐 amapnavi Web服务器启动在端口 {self.port}")
            self.web_server.serve_forever()
        except Exception as e:
            print(f"启动Web服务器失败: {e}")
            raise

    def start(self, daemon=True):
        """以后台线程方式启动 Web 服务器"""
        import threading

        thread = threading.Thread(target=self.start_web_server, daemon=daemon)
        thread.start()
        return thread

    def stop_web_server(self):
        """停止Web服务器"""
        if self.web_server:
            self.web_server.shutdown()
            self.web_server.server_close()
            print("🌐 amapnavi Web服务器已停止")
