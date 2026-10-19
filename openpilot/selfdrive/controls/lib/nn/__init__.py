"""
nn —— Ford/Lincoln 弯道控制的神经网络残差层（第二阶段）

部署位置
---------------------------------------------------------------------------
整个目录复制到::

    openpilot/selfdrive/controls/lib/nn/

之后即可::

    from openpilot.selfdrive.controls.lib.nn.ford_nn_curve_control import FordNNCurveControl

模块
---------------------------------------------------------------------------
ford_nn_features       特征抽取与归一化（纯 numpy，不依赖 cereal）
ford_nn_policy         前馈网络 + Adam 训练内核（纯 numpy，训练与推理同源）
ford_driver_style      4 维可解释驾驶风格向量 + 有界在线进化
ford_nn_curve_control  运行时：残差生成、四道安全边界、配置热加载
train_offline          离线训练脚本（含合成数据通路）

设计原则（改代码前请先读 04_NN弯道控制与个性化架构设计.md）
---------------------------------------------------------------------------
1. **残差形式**：NN 只输出"对基线的有界修正"，不输出执行器指令。
   输出层零初始化 ⇒ 权重为初始值时输出恒为 0 ⇒ 关闭 NN 与现状**逐位一致**。
2. **四道独立安全边界**：硬限幅 / 变化率限幅 / 饱和监控 / panda 固件。
   任何一道单独失效都不会让车进入不可接受状态。
3. **默认关闭**：`mode=0` 时 `update()` 直接返回全 0 且不推进任何内部状态。
4. **绝不抛异常到控制环**：任何异常都降级为"本帧输出 0"。

这里的 `__getattr__` 是**惰性**的：只有真正取属性时才 import 子模块，
避免 `import nn` 这个动作本身就拉起 numpy 与全部依赖（在 openpilot 的
import 顺序里有实际意义）。
"""

from __future__ import annotations

__all__ = [
  "FordNNCurveControl",
  "Policy",
  "PolicyBundle",
  "DriverStyle",
  "DrivingStatsWindow",
]

_LAZY = {
  "FordNNCurveControl": ("ford_nn_curve_control", "FordNNCurveControl"),
  "Policy": ("ford_nn_policy", "Policy"),
  "PolicyBundle": ("ford_nn_policy", "PolicyBundle"),
  "DriverStyle": ("ford_driver_style", "DriverStyle"),
  "DrivingStatsWindow": ("ford_driver_style", "DrivingStatsWindow"),
}

# 包内 import 前缀：作为包导入时是 ".xxx"，被平铺到 sys.path 时是 "xxx"
_PKG = __name__.rsplit(".", 1)[0] if "." in __name__ else ""


def __getattr__(name: str):
  if name not in _LAZY:
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
  mod_name, attr = _LAZY[name]
  if _PKG:
    import importlib
    mod = importlib.import_module(f".{mod_name}", __package__)
  else:  # pragma: no cover - 平铺部署（测试/离线）
    import importlib
    mod = importlib.import_module(mod_name)
  return getattr(mod, attr)


def __dir__() -> list[str]:
  return sorted(__all__)
