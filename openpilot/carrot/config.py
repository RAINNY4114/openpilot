#!/usr/bin/env python3
"""
统一的参数管理模块

新版 SunnyPilot / openpilot Carrot 兼容版。

功能：
1. 优先使用 openpilot.common.params.Params
2. Params 中不存在的自定义参数使用 nav_params.json
3. 支持 bool / int / float / raw string 参数
4. 保留原有 UnifiedParams API
5. 不再依赖旧版 common.params_pyx / UnknownKeyName
6. Carrot 位于：
   /data/openpilot/openpilot/carrot/
"""

import json
import os

from openpilot.common.params import Params


class UnifiedParams:
    """统一的参数管理类，同时处理系统参数和 Carrot 自定义参数"""

    _instance = None
    _initialized = False

    def __new__(cls, nav_json_file=None):
        if cls._instance is None:
            cls._instance = super(UnifiedParams, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, nav_json_file=None):
        """初始化统一参数管理器"""

        if self._initialized:
            return

        self.system_params = Params()

        # Carrot 已经移动到 openpilot/carrot/
        # 因此 nav_params.json 默认就在当前目录
        if nav_json_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            nav_json_file = os.path.join(current_dir, "nav_params.json")

        self.nav_json_file = os.path.realpath(nav_json_file)

        self.nav_data = {}

        self._load_nav_params()

        self._initialized = True

    # ----------------------------------------------------------------------
    # System Params / JSON 参数同步
    # ----------------------------------------------------------------------

    def _match_system_param(self):
        """
        检查 nav_data 中的参数是否同时存在于系统 Params。

        如果系统 Params 可以读取，则优先使用系统 Params 的值。

        注意：
        不同版本 Params 对未知 key 的行为可能不同，因此这里
        故意使用宽松异常处理，避免 Carrot 因一个未知参数而退出。
        """

        for key in list(self.nav_data.keys()):
            try:
                value = self.nav_data[key]
                sys_val = None

                # bool 必须优先判断，因为 bool 是 int 的子类
                if isinstance(value, bool):
                    sys_val = self.system_params.get_bool(key)

                elif isinstance(value, int):
                    sys_val = self.system_params.get_int(key)

                elif isinstance(value, float):
                    sys_val = self.system_params.get_float(key)

                elif isinstance(value, str):
                    sys_val = self.system_params.get(key)

                if sys_val is not None:
                    self.nav_data[key] = sys_val

            except Exception:
                # 系统 Params 中不存在该 key：
                # 保留 nav_params.json 中的自定义参数
                pass

    def _save_system_param(self):
        """
        尝试将 nav_data 中的参数写入系统 Params。

        如果系统 Params 不支持对应 key，则忽略，
        该参数继续由 nav_params.json 管理。
        """

        for key, value in list(self.nav_data.items()):
            try:

                if isinstance(value, bool):
                    self.system_params.put_bool(
                        key,
                        bool(value),
                    )

                elif isinstance(value, int):
                    self.system_params.put_int(
                        key,
                        int(value),
                    )

                elif isinstance(value, float):
                    self.system_params.put_float(
                        key,
                        float(value),
                    )

                else:
                    self.system_params.put(
                        key,
                        str(value),
                    )

            except Exception:
                # 未知 key 或新版 Params API 不支持时忽略
                pass

    # ----------------------------------------------------------------------
    # JSON
    # ----------------------------------------------------------------------

    def _load_nav_params(self):
        """加载 Carrot 自定义参数"""

        try:

            if os.path.exists(self.nav_json_file):

                with open(
                    self.nav_json_file,
                    "r",
                    encoding="utf-8",
                ) as f:
                    self.nav_data = json.load(f)

                # JSON 必须是 dict
                if not isinstance(self.nav_data, dict):
                    raise ValueError(
                        "nav_params.json root object must be a dictionary"
                    )

                self._match_system_param()

            else:

                self.nav_data = self._get_default_nav_data()

                self._match_system_param()

                self._save_nav_data()

        except json.JSONDecodeError as e:

            print(
                f"⚠️ Carrot 配置文件格式错误，"
                f"使用默认配置并重新创建: {e}"
            )

            self.nav_data = self._get_default_nav_data()

            self._match_system_param()

            self._save_nav_data()

        except (OSError, IOError) as e:

            print(
                f"⚠️ 加载 Carrot 自定义参数 "
                f"{self.nav_json_file} 失败: {e}"
            )

            self.nav_data = self._get_default_nav_data()

            self._match_system_param()

    def _save_nav_data(self):
        """保存 Carrot 自定义参数到 nav_params.json"""

        try:

            parent_dir = os.path.dirname(self.nav_json_file)

            if parent_dir:
                os.makedirs(
                    parent_dir,
                    exist_ok=True,
                )

            # 临时文件写入，避免写文件过程中断导致 JSON 损坏
            temp_file = self.nav_json_file + ".tmp"

            with open(
                temp_file,
                "w",
                encoding="utf-8",
            ) as f:

                json.dump(
                    self.nav_data,
                    f,
                    indent=2,
                    ensure_ascii=False,
                )

                f.write("\n")

            os.replace(
                temp_file,
                self.nav_json_file,
            )

        except (OSError, IOError) as e:

            print(
                f"❌ 保存 Carrot 自定义参数失败: {e}"
            )

            try:
                if os.path.exists(temp_file):
                    os.remove(temp_file)
            except Exception:
                pass

    # ----------------------------------------------------------------------
    # 默认 Carrot 参数
    # ----------------------------------------------------------------------

    def _get_default_nav_data(self):
        """获取默认 Carrot 导航参数"""

        return {
            "AutoTurnDistOffset": 0,
            "AutoForkDistOffset": 30,
            "AutoDoForkBlinkerDist": 15,
            "AutoDoForkNavDist": 15,

            "AutoForkDistOffsetH": 1000,
            "AutoDoForkDecalDistH": 50,
            "AutoDoForkDecalDist": 20,

            "AutoDoForkBlinkerDistH": 30,
            "AutoDoForkNavDistH": 50,

            "AutoUpRoadLimit": 0,
            "AutoUpRoadLimit40KMH": 15,

            "AutoUpHighwayRoadLimit": 0,
            "AutoUpHighwayRoadLimit40KMH": 15,

            "RoadType": -1,

            "AutoForkDecalRateH": 80,
            "AutoForkSpeedMinH": 60,
            "AutoKeepForkSpeedH": 5,

            "AutoForkDecalRate": 80,
            "AutoForkSpeedMin": 45,
            "AutoKeepForkSpeed": 5,

            "ShowDebugLog": 0,

            "AutoCurveSpeedFactorH": 100,
            "AutoCurveSpeedAggressivenessH": 100,

            "SameSpiCamFilter": 1,

            "StockBlinkerCtrl": 0,
            "ExtBlinkerCtrlTest": 0,
            "BlinkerMode": 1,

            "LaneStabTime": 50,

            "DynamicBlindRange": 0,
            "DynamicBlindDistance": 0,
            "DisableBlindSpot": 0,

            "BsdDelayTime": 20,
            "SideBsdDelayTime": 20,

            "SideRelDistTime": 10,
            "SidevRelDistTime": 10,

            "SideRadarMinDist": 0,

            "AutoTurnInNotRoadEdge": 1,

            "ContinuousLaneChange": 1,
            "ContinuousLaneChangeCnt": 4,
            "ContinuousLaneChangeInterval": 2,

            "AutoTurnLeft": 1,

            "AutoEnTurnNewLaneTimeH": 0,
            "AutoEnTurnNewLaneTime": 0,

            "NewLaneWidthDiff": 8,
        }

    # ----------------------------------------------------------------------
    # GET
    # ----------------------------------------------------------------------

    def get_bool(self, key, default=False):
        """
        获取布尔参数。

        优先：
            system Params

        fallback：
            nav_params.json
        """

        try:

            value = self.system_params.get_bool(key)

            if value is not None:
                return bool(value)

        except Exception:
            pass

        if key in self.nav_data:

            value = self.nav_data[key]

            try:

                if isinstance(value, bool):
                    return value

                if isinstance(value, str):
                    value = value.strip().lower()

                    if value in ("true", "1", "yes", "on"):
                        return True

                    if value in ("false", "0", "no", "off"):
                        return False

                    return default

                return bool(int(value))

            except (ValueError, TypeError):

                return default

        return default

    def get_int(self, key, default=0):
        """
        获取整数参数。

        优先 system Params，
        不存在时 fallback 到 nav_params.json。
        """

        try:

            value = self.system_params.get_int(key)

            if value is not None:
                return int(value)

        except Exception:
            pass

        if key in self.nav_data:

            value = self.nav_data[key]

            try:
                return int(value)

            except (ValueError, TypeError):

                return default

        return default

    def get_float(self, key, default=0.0):
        """
        获取浮点参数。

        优先 system Params，
        不存在时 fallback 到 nav_params.json。
        """

        try:

            value = self.system_params.get_float(key)

            if value is not None:
                return float(value)

        except Exception:
            pass

        if key in self.nav_data:

            value = self.nav_data[key]

            try:
                return float(value)

            except (ValueError, TypeError):

                return default

        return default

    def get(self, key, default=None, encoding="utf-8"):
        """
        获取原始参数。

        优先从系统 Params 获取。
        如果系统 Params 中不存在，则从 nav_data 获取。
        """

        try:

            value = self.system_params.get(
                key,
                encoding=encoding,
            )

            if value is not None:
                return value

        except Exception:
            pass

        if key in self.nav_data:

            value = self.nav_data[key]

            if value is None:
                return default

            if isinstance(value, bytes):
                try:
                    return value.decode(encoding)
                except Exception:
                    return value

            return value

        return default

    # ----------------------------------------------------------------------
    # PUT
    # ----------------------------------------------------------------------

    def put_bool(self, key, value):
        """设置布尔参数"""

        bool_value = bool(value)

        json_need_save = False

        try:

            self.system_params.put_bool(
                key,
                bool_value,
            )

        except Exception:

            # 系统 Params 不认识该 key
            # → 使用 Carrot JSON 参数
            self.nav_data[key] = int(bool_value)
            json_need_save = True

        else:

            # 如果该参数原本存在于 nav_data，
            # 同步更新 JSON
            if key in self.nav_data:

                self.nav_data[key] = int(bool_value)
                json_need_save = True

        if json_need_save:

            self._save_nav_data()

    def put_int(self, key, value):
        """设置整数参数"""

        try:
            int_value = int(value)
        except (ValueError, TypeError):
            return

        json_need_save = False

        try:

            self.system_params.put_int(
                key,
                int_value,
            )

        except Exception:

            self.nav_data[key] = int_value
            json_need_save = True

        else:

            if key in self.nav_data:

                self.nav_data[key] = int_value
                json_need_save = True

        if json_need_save:

            self._save_nav_data()

    def put_float(self, key, value):
        """设置浮点参数"""

        try:
            float_value = float(value)
        except (ValueError, TypeError):
            return

        json_need_save = False

        try:

            self.system_params.put_float(
                key,
                float_value,
            )

        except Exception:

            self.nav_data[key] = float_value
            json_need_save = True

        else:

            if key in self.nav_data:

                self.nav_data[key] = float_value
                json_need_save = True

        if json_need_save:

            self._save_nav_data()

    def put(self, key, dat):
        """
        设置原始参数。

        系统 Params 支持时优先写系统 Params；
        否则写入 Carrot nav_params.json。
        """

        json_need_save = False

        try:

            self.system_params.put(
                key,
                dat,
            )

        except Exception:

            self.nav_data[key] = dat
            json_need_save = True

        else:

            if key in self.nav_data:

                self.nav_data[key] = dat
                json_need_save = True

        if json_need_save:

            self._save_nav_data()


# --------------------------------------------------------------------------
# 全局实例
# --------------------------------------------------------------------------

unified_params = UnifiedParams()
