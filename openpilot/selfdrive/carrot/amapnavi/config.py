#!/usr/bin/env python3
"""
统一的参数管理模块
提供统一的接口访问系统参数和自定义参数，支持fallback机制
"""

import json
import os
from openpilot.common.params import Params, UnknownKeyName

class UnifiedParams:
    """统一的参数管理类，同时处理系统参数和自定义参数"""

    _instance = None
    _initialized = False

    def __new__(cls, nav_json_file=None):
        if cls._instance is None:
            cls._instance = super(UnifiedParams, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, nav_json_file=None):
        """初始化统一的参数管理器"""
        if self._initialized:
            return

        self.system_params = Params()

        if nav_json_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            nav_json_file = os.path.join(current_dir, "nav_params.json")

        self.nav_json_file = os.path.realpath(nav_json_file)

        self.nav_data = {}
        self._load_nav_params()

        self._initialized = True

    def _match_system_param(self):
      # -----------------------------
      # 检查 system_params是否已有参数，有则使用系统参数
      # -----------------------------
      for key in list(self.nav_data.keys()):
        try:
          # 尝试从 system_params 读取，依类型猜测
          sys_val = None
          # 按 JSON 类型尝试读取
          if isinstance(self.nav_data[key], int):
            sys_val = self.system_params.get_int(key)
          elif isinstance(self.nav_data[key], float):
            sys_val = self.system_params.get_float(key)
          elif self.nav_data[key] in (0, 1):  # 布尔型（int表示）
            sys_val = self.system_params.get_bool(key)

          if sys_val is not None:
            self.nav_data[key] = sys_val  # 使用系统值覆盖 JSON 值
        except Exception:
          pass

    def _save_system_param(self):
      """
      尝试将 nav_data 中的参数写入 system_params。
      如果 system_params 中存在对应的 key，则覆盖系统参数。
      如果 system_params 中不存在该 key，则忽略（说明它是自定义参数）。
      """
      for key, value in list(self.nav_data.items()):
        try:
          # 根据 nav_data 的类型选择正确的 put 方法
          if isinstance(value, bool) or value in (0, 1):
            # bool（在 JSON 中通常表现为 0/1）
            self.system_params.put_bool(key, bool(value))
          elif isinstance(value, int):
            self.system_params.put_int(key, int(value))
          elif isinstance(value, float):
            self.system_params.put_float(key, float(value))
          else:
            # 原始值（字符串等）
            self.system_params.put(key, str(value))

        except (KeyError, AttributeError, UnknownKeyName):
          # system_params 不认识这个 key → 忽略
          pass
        except Exception:
          # 其他未知异常也忽略，避免影响主流程
          pass

    def _load_nav_params(self):
        """加载自定义参数数据。

        以 ``_get_default_nav_data()`` 为底稿，用文件里的值覆盖——这样以后
        新增参数（旧 JSON 里没有的键）也能拿到代码里的默认值，
        不会因为读不到而退化成 0。
        """
        defaults = self._get_default_nav_data()
        try:
            if os.path.exists(self.nav_json_file):
                with open(self.nav_json_file, 'r', encoding='utf-8') as f:
                    self.nav_data = json.load(f)
                for key, value in defaults.items():
                    self.nav_data.setdefault(key, value)
                self._match_system_param()
            else:
                self.nav_data = self._get_default_nav_data()
                self._match_system_param()
                self._save_nav_data()
        except json.JSONDecodeError as e:
            print(f"⚠️ 自定义配置文件格式错误，使用默认配置并重新创建: {e}")
            self.nav_data = self._get_default_nav_data()
            self._match_system_param()
            self._save_nav_data()
        except (OSError, IOError) as e:
            print(f"⚠️ 加载自定义参数{self.nav_json_file}失败: {e}")
            self.nav_data = self._get_default_nav_data()
            self._match_system_param()

    def _save_nav_data(self):
        """保存自定义参数到文件"""
        try:
            os.makedirs(os.path.dirname(self.nav_json_file), exist_ok=True)
            with open(self.nav_json_file, 'w', encoding='utf-8') as f:
                json.dump(self.nav_data, f, indent=2, ensure_ascii=False)
        except (OSError, IOError) as e:
            print(f"❌ 保存自定义参数失败: {e}")

    def _get_default_nav_data(self):
        """获取默认的自定义参数数据

        amapnavi(外部激光雷达/摄像头BSD模块)使用的配置项。
        其它参数由 openpilot Params(params_keys.h) 直接管理。
        """
        return {
            "ShowDebugLog": 0,
            "StockBlinkerCtrl": 0,
            "DynamicBlindRange": 0,
            "DynamicBlindDistance": 0,
            "DisableBlindSpot": 0,
            "EnableCruiseStateShow": 1,

            "LidarBsdDelayTime": 10,
            "LidarFrontVDistTime": -50,
            "LidarFrontVRelDistTime": 40,
            "LidarBehindVDistTime": -100,
            "LidarBehindVRelDistTime": 40,
            "LaneLineDelayTime": 10,

            # 风险评估（blindspot）：最小间距 4.0m / TTC 2.5s / 预测窗口 3.0s
            "LidarMinClearance": 40,
            "LidarTtcThreshold": 25,
            "LidarRiskHorizon": 30,

            # 让行速度规划（decel_advisor）
            "BsdTimeHeadway": 12,      # x0.1s -> 1.2s
            "BsdMinGap": 80,           # x0.1m -> 8.0m
            "BsdMergeMargin": 30,      # x0.1m -> 3.0m
            "BsdRearDangerGap": 60,    # x0.1m -> 6.0m
            "BsdSpeedMargin": 6,       # km/h
            "BsdMaxSpeedup": 12,       # km/h
            "BsdMaxSlowdown": 18,      # km/h
            "BsdAccelLimit": 15,       # x0.1 km/h/s -> 1.5
            "BsdDecelLimit": 25,       # x0.1 km/h/s -> 2.5
            "BsdCommitTime": 25,       # x0.1s -> 2.5s

            # 原车前雷达的「前侧盲区」(stock_front_blind)
            "StockFrontBlindEnable": 1,
            "StockFrontLatMin": 12,       # x0.1m -> 1.2m（横向下限，小于此值算本车道）
            "StockFrontLatMax": 50,       # x0.1m -> 5.0m（横向上限硬顶）
            "StockFrontLatLaneFactor": 18, # x0.1 -> 1.8 倍车道宽（横向上限）
            "StockFrontDrelMin": -200,    # x0.1m -> -20m（纵向窗口：身后）
            "StockFrontDrelMax": 800,     # x0.1m -> 80m（纵向窗口：前方）
            "StockFrontTimeHeadway": 15,  # x0.1s -> 1.5s
            "StockFrontMinClearance": 40, # x0.1m -> 4.0m
            "StockFrontTtc": 25,          # x0.1s -> 2.5s
            "StockFrontHorizon": 30,      # x0.1s -> 3.0s
            "StockFrontOnTime": 3,        # x0.1s -> 0.3s（置位延时）
            "StockFrontOffTime": 8,       # x0.1s -> 0.8s（清除延时）
        }

    def get_bool(self, key, default=False):
        """获取布尔值参数 - 统一接口"""
        # 优先尝试系统参数
        try:
            value = self.system_params.get_bool(key)
            if value is not None:
                return value
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        # 系统参数不存在，尝试自定义参数
        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return bool(int(value))
            except (ValueError, TypeError):
                return default

        return default

    def get_int(self, key, default=0):
        """获取整数值参数 - 统一接口"""
        try:
            value = self.system_params.get_int(key)
            if value is not None:
                return value
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return int(value)
            except (ValueError, TypeError):
                return default

        return default

    def get_float(self, key, default=0.0):
        """获取浮点数值参数 - 统一接口"""
        try:
            value = self.system_params.get_float(key)
            if value is not None:
                return value
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return float(value)
            except (ValueError, TypeError):
                return default

        return default

    def put_bool(self, key, value):
      """设置布尔值参数 - 统一接口"""
      bool_value = bool(value)
      int_value = int(bool_value)

      json_need_save = False

      # 先尝试保存系统参数
      try:
        self.system_params.put_bool(key, bool_value)
      except (KeyError, AttributeError, UnknownKeyName):
        # 系统参数没有 → 写入 nav_data
        self.nav_data[key] = int_value
        json_need_save = True
      else:
        # 系统参数写成功，但如果 key 存在于 nav_data，也要同步更新
        if key in self.nav_data:
          self.nav_data[key] = int_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put_int(self, key, value):
      """设置整数值参数 - 统一接口"""
      int_value = int(value)

      json_need_save = False

      try:
        self.system_params.put_int(key, int_value)
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = int_value
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = int_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put_float(self, key, value):
      """设置浮点数值参数 - 统一接口"""
      float_value = float(value)

      json_need_save = False

      try:
        self.system_params.put_float(key, float_value)
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = float_value
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = float_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put(self, key, dat):
      """设置原始数据 - 系统参数专用"""
      json_need_save = False

      try:
        self.system_params.put(key, dat)
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = dat
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = dat
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    # 为了保持兼容性，也提供原始的 get/put 方法
    def get(self, key, encoding='utf-8'):
        """获取原始数据 - 系统参数专用"""
        return self.system_params.get(key, encoding=encoding)

# 全局实例，便于导入使用
unified_params = UnifiedParams()
