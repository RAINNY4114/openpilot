#!/usr/bin/env python3

import json
import os

from openpilot.common.params import Params

class UnifiedParams:

    _instance = None
    _initialized = False

    def __new__(cls, nav_json_file=None):
        if cls._instance is None:
            cls._instance = super(UnifiedParams, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, nav_json_file=None):

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

        for key in list(self.nav_data.keys()):
            try:
                value = self.nav_data[key]
                sys_val = None

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
                pass

    def _save_system_param(self):

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
                pass

    def _load_nav_params(self):

        try:

            if os.path.exists(self.nav_json_file):

                with open(
                    self.nav_json_file,
                    "r",
                    encoding="utf-8",
                ) as f:
                    self.nav_data = json.load(f)

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

        try:

            parent_dir = os.path.dirname(self.nav_json_file)

            if parent_dir:
                os.makedirs(
                    parent_dir,
                    exist_ok=True,
                )

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

    def _get_default_nav_data(self):

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

    def get_bool(self, key, default=False):

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

    def put_bool(self, key, value):

        bool_value = bool(value)

        json_need_save = False

        try:

            self.system_params.put_bool(
                key,
                bool_value,
            )

        except Exception:

            self.nav_data[key] = int(bool_value)
            json_need_save = True

        else:

            if key in self.nav_data:

                self.nav_data[key] = int(bool_value)
                json_need_save = True

        if json_need_save:

            self._save_nav_data()

    def put_int(self, key, value):

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

unified_params = UnifiedParams()
