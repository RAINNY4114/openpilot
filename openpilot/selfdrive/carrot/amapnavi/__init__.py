"""amapnavi: 外部(外挂)激光雷达·摄像头 BSD 模块的 UDP 通信模块。

由 cpv9-dev 的 ``openpilot/selfdrive/carrot/amap_navi.py`` 移植而来，并按职责拆分：

* :mod:`.amap_navi`     - 主服务编排（``AmapNaviServ``）
* :mod:`.shared_state`  - 共享状态与常量（``SharedData``）
* :mod:`.blindspot`     - 侧向目标跟踪 / 风险评估
* :mod:`.decel_advisor` - 盲区受阻时的车速规划
* :mod:`.vehicle_state` - 车辆状态获取（自带 carState 订阅）
* :mod:`.protocol`      - UDP 报文解析
* :mod:`.transport`     - UDP 收发与客户端管理
* :mod:`.messages`      - 下发消息构造
* :mod:`.config`        - 参数管理（``UnifiedParams``）
* :mod:`.config_web`    - 参数配置 Web 服务（``ConfigWeb``）

``lane``（车道线视频流服务）需要 OpenCV，是独立进程入口，因此**不在**本包
``__init__`` 中导入，请用 ``python -m ...amapnavi.lane`` 单独启动。
"""

from openpilot.selfdrive.carrot.amapnavi.amap_navi import AmapNaviServ
from openpilot.selfdrive.carrot.amapnavi.blindspot import (
  OccupancyCounter,
  RiskConfig,
  RiskResult,
  TrackedTarget,
  TrackerConfig,
  assess_side_risk,
  resolve_safe_distance,
)
from openpilot.selfdrive.carrot.amapnavi.config import UnifiedParams, unified_params
from openpilot.selfdrive.carrot.amapnavi.decel_advisor import (
  HumanLikeConfig,
  Plan,
  SideTarget,
  SpeedAdvisor,
  MODE_BEHIND,
  MODE_FRONT,
  MODE_GIVEUP,
  MODE_NONE,
)
from openpilot.selfdrive.carrot.amapnavi.shared_state import (
  BLINKER_BOTH,
  BLINKER_LEFT,
  BLINKER_NONE,
  BLINKER_RIGHT,
  DT_BROADCAST,
  SharedData,
)
from openpilot.selfdrive.carrot.amapnavi.vehicle_state import apply_carstate

try:  # 参数配置 Web 服务依赖 http.server，缺失时 BSD 模块仍可正常工作
  from openpilot.selfdrive.carrot.amapnavi.config_web import ConfigWeb
except Exception:  # pragma: no cover
  ConfigWeb = None

__all__ = [
  "AmapNaviServ",
  "SharedData",
  "ConfigWeb",
  "UnifiedParams",
  "unified_params",
  "TrackedTarget",
  "TrackerConfig",
  "RiskConfig",
  "RiskResult",
  "assess_side_risk",
  "resolve_safe_distance",
  "OccupancyCounter",
  "SpeedAdvisor",
  "HumanLikeConfig",
  "Plan",
  "SideTarget",
  "MODE_NONE",
  "MODE_FRONT",
  "MODE_BEHIND",
  "MODE_GIVEUP",
  "apply_carstate",
  "BLINKER_NONE",
  "BLINKER_LEFT",
  "BLINKER_RIGHT",
  "BLINKER_BOTH",
  "DT_BROADCAST",
]
