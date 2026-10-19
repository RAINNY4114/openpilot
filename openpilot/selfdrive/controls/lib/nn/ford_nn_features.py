"""
ford_nn_features.py —— 神经网络的输入特征抽取与归一化

设计约束（三条都是硬要求，改动前请先读完）
---------------------------------------------------------------------------
1) **核心不依赖 cereal / capnp。**
   本模块只用 numpy + 标准库，输入是普通浮点数/数组。这样离线训练脚本、
   单元测试、以及车端推理可以共用同一份特征代码，不会出现"训练时用的特征
   和车上算的不一样"这类静默偏差。与 openpilot 消息的对接隔离在文件末尾的
   `MessageAdapter` 里，用 try/except 包住。

2) **归一化必须有界。**
   每个特征显式声明 `scale`（归一化除数）与 `clip`（归一化后的裁剪半径）。
   OOD 输入（例如模型输出的离谱曲率）不允许让特征发散 —— 否则 NN 会在
   没见过的区域外推，这在安全关键系统里是不可接受的。

3) **特征顺序一经发布不可更改。**
   权重文件（.npz）按位置解释特征。任何顺序/维度改动都必须同时升
   `FEATURE_VERSION`，并让加载端拒绝版本不匹配的权重。

为什么是这些特征
---------------------------------------------------------------------------
上一轮已经查明：`FordCurveController._compute_path_angle()` 里的
`curvature_factor` 是一张手工标定的二维查表（`np.interp`，自变量只有
`|kappa|` 与 `v_ego`）。这张表**表达不了**下面这些对弯道品质真正重要的量：

  * `dkappa/dt` —— 曲率变化率。人类驾驶员靠它**预判**弯道收放，
    现有实现只能用 `curve_phase` 状态机粗糙近似。
  * `steer_lag` —— 指令 path_angle 与"方向盘实际位置等效的 path_angle"之差
    （统一到 path_angle 空间，见 `steering_path_angle`）。
    PSCM 有自己的角度闭环，滞后是福特/林肯平台的主要痛点，
    而查表对此完全无感。
    ⚠️ **当前默认关闭**（`FeatSpec.disabled=True`）：用 realdata 实测后发现，
    这份数据里它要么恒 0、要么是"人对指令的偏离"而非部署时的"PSCM 跟随
    误差"。完整证据链见 `LATERAL_FF_SPEC` 里该行的注释。
  * `a_lat` —— 实际横向加速度，舒适性的直接度量。
  * 驾驶员风格 —— 见 ford_driver_style.py。

所以 NN 的价值不是"替代一个已经很好的模块"，而是**补上查表原理上
表达不了的自由度**，同时保持残差形式（见 ford_nn_policy.py）以便随时退回。
"""

from __future__ import annotations

import math
from typing import NamedTuple, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# 版本
# ---------------------------------------------------------------------------

# v2 (2026-10-14)：`steer_lag` 的符号修正。
#   实测（以 `modelV2.position.y` 的预测路径几何航向角为基准，2720 帧 v>6 m/s）：
#     modelV2.orientationRate.z  同号 87.6% corr +0.858  → openpilot 约定（左正）
#     carState.steeringAngleDeg  同号 14.6% corr -0.801  → **反号**
#   旁证：openpilot 自己也是 `controlsd.py:91 self.curvature = -VM.calc_curvature(...)`。
#
# v3 (2026-10-14)：`steer_lag` 的**量纲**修正（v2 只修了符号，仍不对）。
#   v2 的式子是 `path_angle_last - STEERING_SIGN·radians(steer)·1.0`，
#   而 v2 的 `path_angle_last` 在 rlog 读取器里是
#   `radians(cc.actuators.steeringAngleDeg) / steer_ratio` —— 即**前轮转角**
#   （rad），量纲是"角"；`radians(steer)` 是**方向盘角**（rad），量纲也是"角"，
#   但两者差一个 `steer_ratio` 因子（16.8 倍）。相加/相减都会得到一个
#   被 `steer_ratio` 污染的伪量。实测（3 段 2398 帧）：
#     OLD  |steer_lag| p50=0.326  p90=6.50   → scale 0.05 时饱和 57%
#   ⇒ 统一到 **path_angle 空间**（`κ·v`，rad），这是唯一与 `path_angle_last`
#     同量纲、且物理上就是"车实际走的航向角"的量：
#     steer_lag = path_angle_last − STEERING_SIGN·radians(steer)·v/(steerRatio·wheelbase)
#   同时明确 `path_angle_last` 的**契约**：必须是 **openpilot 约定**（左正）的
#   path_angle，量纲为 rad。车端传的是 `angle_state["path_angle"]`（本来就是
#   openpilot 约定）；rlog 读取器用 `cc.actuators.steeringAngleDeg` 反算时必须
#   自己带上 `STEERING_SIGN`（见 train_offline.read_frames_from_rlogs）。
#   维度与顺序未变，但**语义变了** → 必须再升版本，让加载端拒绝 v1/v2 权重。
FEATURE_VERSION = 3

# 驾驶风格向量维度（与 ford_driver_style.py 的 STYLE_DIM 必须一致）
STYLE_DIM = 4

# `carState.steeringAngleDeg` / `carControl.actuators.steeringAngleDeg`
# 相对 openpilot 转向/曲率约定的符号（见上）。两者是**同一套**车体约定。
STEERING_SIGN = -1.0

# 默认车辆几何（FORD_EXPLORER_MK6，见 `16` 号文档 §4.1）。仅在调用方拿不到
# carParams 时兜底；车端一律传真实的 `CP.steerRatio` / `CP.wheelbase`。
STEER_RATIO_DEFAULT = 16.8
WHEELBASE_DEFAULT = 3.025

# ---------------------------------------------------------------------------
# 特征规格
# ---------------------------------------------------------------------------


class FeatSpec(NamedTuple):
  """单个特征的归一化规格。

  Attributes:
    name:  特征名（仅用于诊断/日志，不参与计算）。
    scale: 归一化除数。原始值 / scale 落在约 [-1, 1] 视为"典型工况"。
    clip:  归一化后的裁剪半径。默认 3.0，即允许到典型值的 3 倍。
    disabled: True 时归一化结果**恒为 0**（维度保留，权重列无梯度）。
      用途：某个特征在**当前这批数据上不可辨识或与标签泄漏**时，
      把它关掉。为什么用"置 0"而不是"从向量里删掉"：
        ① 维度不变 → 权重文件格式稳定，不需要为每次数据差异改架构；
        ② **训练与车端看到的是同一个 0** → 不会出现"训练时按 0 学、
           车端按真实值推理"这种最隐蔽的偏差（否则网络会给一个
           在训练中毫无梯度的权重列，却让它在线时贡献一个随机量）。
      默认值由数据决定，见 `LATERAL_FF_SPEC` 里 `steer_lag` 的说明。
  """

  name: str
  scale: float
  clip: float = 3.0
  disabled: bool = False


# --- 横向前馈（Lateral Feed-Forward）-----------------------------------------
# 10 个物理特征 + 4 个风格特征 = 14 维
LATERAL_FF_SPEC: tuple[FeatSpec, ...] = (
  FeatSpec("kappa",       0.02),    # 1/m，0.02 对应 50 m 半径
  FeatSpec("kappa_rate",  0.05),    # 1/m/s
  FeatSpec("v_ego",       30.0),    # m/s
  FeatSpec("a_lat",       3.6),     # m/s^2，ISO 11270 上限
  FeatSpec("kappa_lag1",  0.02),    # kappa(t-0.05)
  FeatSpec("kappa_lag2",  0.02),    # kappa(t-0.10)
  FeatSpec("kappa_lag3",  0.02),    # kappa(t-0.15)
  # steer_lag：**path_angle 空间**的"指令 − 实测"（rad），见 FEATURE_VERSION v3。
  # scale 由实测分位数定（3 段 / 700 个可用样本，v≥5 且 |κ|>8e-4）：
  #   |steer_lag| p50=0.0000  p90=0.0011  p99=0.0232  max=0.0265
  # 取 0.01 → p99/0.01 = 2.32、max/0.01 = 2.65，均在 clip 3.0 以内（零饱和）。
  # ⚠️ 这个 scale 只在"能取到非零 steer_lag 的那批帧"上有意义；本特征一旦在
  #    **部署工况**（latActive ∧ !override）下重新启用，必须用新数据重定标。
  #
  # ⚠️ **默认 disabled=True —— 2026-10-14 用 realdata 实测后的结论**。
  #   起初怀疑它是"标签泄漏"（override 帧上"实测角"就是人打的，而标签
  #   d_gain 也是人的偏离）。**实测推翻了该假设**：
  #       corr(steer_lag, d_gain 标签) = -0.0007   （3 段 700 样本）
  #   真正的问题不是泄漏，而是**语义不可迁移**，来自两条实测事实：
  #
  #   ① 上游 `selfdrive/controls/lib/latcontrol_angle.py:24-25`：
  #          if not active: angle_steers_des = float(CS.steeringAngleDeg)
  #      ⇒ **横向未激活时指令角直接回显实测角**，`steer_lag ≡ 0`。
  #      实测 1043/1626 帧（64%）|cc−cs| 严格 = 0.0000。
  #   ② 横向激活时 `angle_steers_des = degrees(VM.get_steer_from_curvature(
  #          -desired_curvature, vEgo, roll)) + angleOffsetDeg` —— 这才是
  #      部署时该特征的语义（**PSCM 跟随误差**）。
  #      但本数据集的训练样本里，latActive=True 的帧必然带 `steeringPressed`
  #      （人不接管时的在控帧 `human_driving=False`，根本不进训练集）
  #      ⇒ 训练时它算的是"**人对 openpilot 指令的偏离**"。
  #
  #   ⇒ 训练工况（人的偏离）与部署工况（PSCM 跟随误差）是**两个不同的物理量**，
  #     网络学到的映射不可能迁移。在这份数据上它要么恒 0、要么是错的那个量。
  #     保持关闭；等有了"latActive ∧ !override ∧ 有曲率"的样本再开。
  #   要打开：`ford_nn_features.set_feature_enabled("steer_lag", True)`，
  #   或训练时加 `--enable-feature steer_lag`。**维度不变，不必升版本**。
  FeatSpec("steer_lag",   0.01, disabled=True),
  FeatSpec("a_lat_filt",  3.6),     # 低通后的 a_lat
  FeatSpec("yaw_rate",    0.5),     # rad/s
) + tuple(FeatSpec(f"style{i}", 1.0, 1.0) for i in range(STYLE_DIM))

# --- 纵向弯道预算（Curve Speed Budget）---------------------------------------
# 7 个前瞻曲率 + 5 个上下文 + 4 个风格 = 16 维
CURVE_HORIZON_N = 7          # 前瞻采样点数
CURVE_HORIZON_DT = 0.5       # 采样间隔 (s)，覆盖 0 .. 3.0 s

CURVE_SPEED_SPEC: tuple[FeatSpec, ...] = (
  tuple(FeatSpec(f"kappa_h{i}", 0.02) for i in range(CURVE_HORIZON_N))
) + (
  FeatSpec("v_ego",          30.0),
  FeatSpec("dist_to_curve", 200.0),   # m
  FeatSpec("kappa_max",       0.02),
  FeatSpec("kappa_end",       0.02),
  FeatSpec("a_ego",           3.0),
) + tuple(FeatSpec(f"style{i}", 1.0, 1.0) for i in range(STYLE_DIM))

LATERAL_FF_DIM = len(LATERAL_FF_SPEC)
CURVE_SPEED_DIM = len(CURVE_SPEED_SPEC)


# ---------------------------------------------------------------------------
# 归一化
# ---------------------------------------------------------------------------


def normalize(raw: Sequence[float], spec: Sequence[FeatSpec]) -> np.ndarray:
  """把原始特征按 spec 归一化并裁剪。

  Args:
    raw:  原始特征序列，长度必须等于 len(spec)。
    spec: 特征规格。

  Returns:
    float32 数组，形状 (len(spec),)，每个元素已裁剪到 [-clip, clip]。

  Raises:
    ValueError: 长度不匹配，或出现 NaN/Inf（宁可报错也不要静默传播）。
  """
  if len(raw) != len(spec):
    raise ValueError(f"feature length mismatch: got {len(raw)}, want {len(spec)}")

  arr = np.asarray(raw, dtype=np.float64)
  if not np.all(np.isfinite(arr)):
    raise ValueError("non-finite feature value (NaN/Inf)")

  scales = np.array([s.scale for s in spec], dtype=np.float64)
  clips = np.array([s.clip for s in spec], dtype=np.float64)

  out = arr / scales
  np.clip(out, -clips, clips, out=out)

  # `disabled` 特征一律置 0（见 FeatSpec.disabled 的说明）：
  # 训练与车端共用本函数 ⇒ 两边看到同一个 0，不可能出现静默偏差。
  if any(s.disabled for s in spec):
    out[[i for i, s in enumerate(spec) if s.disabled]] = 0.0

  return out.astype(np.float32)


def feature_names(spec: Sequence[FeatSpec]) -> list[str]:
  """特征名列表（与归一化后的列一一对应），供诊断/泄漏审计使用。"""
  return [s.name for s in spec]


def set_feature_enabled(name: str, enabled: bool) -> bool:
  """按名字开关一个特征（改 `FeatSpec.disabled` 位）。

  **维度与顺序不变**，所以不需要升 `FEATURE_VERSION`；但会同时改变
  训练与车端的行为（因为两边都走 `normalize()`），这正是想要的一致性。

  Returns:
    True 表示确实改到了（名字存在），False 表示名字不存在（未做任何事）。
  """
  global LATERAL_FF_SPEC, CURVE_SPEED_SPEC
  hit = False
  for attr in ("LATERAL_FF_SPEC", "CURVE_SPEED_SPEC"):
    spec = globals()[attr]
    new = []
    for s in spec:
      if s.name == name:
        new.append(s._replace(disabled=not enabled))
        hit = True
      else:
        new.append(s)
    globals()[attr] = tuple(new)
  return hit


def denormalize_gain(value_norm: float, spec: FeatSpec) -> float:
  """把归一化值还原为物理值（用于诊断输出）。"""
  return float(value_norm) * spec.scale


# ---------------------------------------------------------------------------
# 曲率历史（提供 kappa_lag1..3）
# ---------------------------------------------------------------------------


class KappaHistory:
  """定长曲率历史环形缓冲。

  只保存最近 N 帧的曲率，供 NN 推断"曲率趋势"。不引入任何滤波 ——
  滤波是 controller 的职责，特征层只做搬运，避免两处滤波叠加。
  """

  def __init__(self, size: int = 4) -> None:
    self.size = int(size)
    self._buf = np.zeros(self.size, dtype=np.float64)
    self._filled = 0

  def reset(self) -> None:
    self._buf[:] = 0.0
    self._filled = 0

  def push(self, kappa: float) -> None:
    self._buf = np.roll(self._buf, 1)
    self._buf[0] = float(kappa)
    self._filled = min(self._filled + 1, self.size)

  def lag(self, n: int) -> float:
    """取 n 帧前的曲率。n 从 1 开始。历史不足时返回 0.0。"""
    if n < 1 or n >= self.size:
      raise ValueError(f"lag index out of range: {n}")
    if self._filled <= n:
      return 0.0
    return float(self._buf[n])

  def __len__(self) -> int:
    return self._filled


# ---------------------------------------------------------------------------
# modelV2 → 曲率
#
# 这一节是**本轮修正的重点**。chestnut 的 `modelV2.position.x` **不是**多项式
# 系数，而是模型预测轨迹在 33 个时间点上的**纵向距离**（单位 m）。要拿到曲率
# 必须配合另外两个输出：
#
#     v_denom = clip(velocity.x, max(1.0, 0.7 * v_ego), 100.0)
#     kappa   = clip(orientationRate.z / v_denom, -0.02, 0.02)
#
# 这套算法在 chestnut 里已有唯一实现：
#     openpilot/selfdrive/controls/lib/curve_bend.py::path_curvature()
# 本模块**逐位复刻**它（不直接 import，是为了维持"核心不依赖 openpilot"
# 这条约束，让离线训练 / 单元测试可用）。为什么必须一致：纵向弯道预算与
# 既有的弯道减速、HUD 弯道标志必须看到同一个曲率，否则会出现
# "HUD 亮了但 NN 没减速"这种无法向用户解释的不一致。
# ---------------------------------------------------------------------------


# 与 curve_bend.py / ford_curve_speed.py 相同的曲率裁剪半径 (1/m)
KAPPA_CLIP = 0.02

# ModelConstants.T_IDXS 的本地副本（10 s 视野、33 点、二次分布）。
# 车端优先用 openpilot 的 ModelConstants；离线环境用同一公式复算，
# 保证训练与车端看到同一组时间点。
_MODEL_T_IDXS: np.ndarray | None = None


def model_t_idxs() -> np.ndarray:
  """模型预测轨迹的时间索引 (s)，长度 33。"""
  global _MODEL_T_IDXS
  if _MODEL_T_IDXS is None:
    try:  # pragma: no cover - 车端路径
      from openpilot.selfdrive.modeld.constants import ModelConstants
      _MODEL_T_IDXS = np.asarray(ModelConstants.T_IDXS, dtype=np.float64)
    except Exception:  # pragma: no cover - 离线路径
      idx = np.arange(33, dtype=np.float64)
      _MODEL_T_IDXS = 10.0 * (idx / 32.0) ** 2
  return _MODEL_T_IDXS


def model_curvature(positions: Sequence[float],
                    turn_rates: Sequence[float],
                    v_preds: Sequence[float],
                    v_ego: float) -> tuple[np.ndarray, np.ndarray]:
  """把 modelV2 的轨迹换算成 (s, 带符号曲率)。

  与 `openpilot/selfdrive/controls/lib/curve_bend.py::path_curvature()`
  **逐位一致**（同一套分母下限与 clip）：

      v_denom = clip(velocity.x, max(1.0, 0.7 * v_ego), 100.0)
      kappa   = clip(orientationRate.z / v_denom, -0.02, 0.02)

  为什么不直接 import 那个函数：本模块有一条硬约束是"核心不依赖
  openpilot / cereal"，这样离线训练脚本与单元测试才能用同一份特征代码。
  代价是这里必须复刻 —— 所以一旦上游改了算法，**两处必须同步修改**；
  `verify/test_nn.py` 里加了与公式的对照断言来兜住这一点。

  Args:
    positions:  modelV2.position.x —— 沿预测路径的纵向距离 (m)，33 点。
    turn_rates: modelV2.orientationRate.z —— 横摆角速度 (rad/s)。
    v_preds:    modelV2.velocity.x —— 预测车速 (m/s)。
    v_ego:      当前实测车速 (m/s)，用于给分母设下限。

  Returns:
    (s, kappa)，两个同长数组；输入不可用时长度为 0。**绝不抛异常** ——
    控制环里任何异常都必须由调用方降级处理。
  """
  pos = np.asarray(positions, dtype=np.float64)
  tr = np.asarray(turn_rates, dtype=np.float64)
  vp = np.asarray(v_preds, dtype=np.float64)
  n = min(pos.size, tr.size, vp.size)
  if n < 2:
    return np.zeros(0, dtype=np.float64), np.zeros(0, dtype=np.float64)
  pos, tr, vp = pos[:n], tr[:n], vp[:n]
  if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(tr)) and np.all(np.isfinite(vp))):
    return np.zeros(0, dtype=np.float64), np.zeros(0, dtype=np.float64)
  v_denom = np.clip(vp, max(1.0, float(v_ego) * 0.7), 100.0)
  signed = np.clip(tr / v_denom, -KAPPA_CLIP, KAPPA_CLIP)
  return pos, signed


def sample_curvature_horizon(positions: Sequence[float],
                             turn_rates: Sequence[float],
                             v_preds: Sequence[float],
                             v_ego: float,
                             n: int = CURVE_HORIZON_N,
                             dt: float = CURVE_HORIZON_DT) -> np.ndarray:
  """在 t = 0, dt, ..., (n-1)dt 上采样曲率。

  为什么不能直接取 `orientationRate.z[i] / velocity.x[i]`：
  模型输出的 33 个点是**按 T_IDXS = 10*(idx/32)^2 不均匀分布**的
  （近处约 0.01 s 一个点，远处约 0.7 s 一个点）。直接按下标取会让
  特征在时间轴上严重畸变 —— 这正是"训练和车上不一致"最容易发生的地方。

  所以：先在 T_IDXS 上求曲率，再对时间做线性插值。超过模型视野
  （T_IDXS[-1] = 10 s）的部分由 `np.interp` 自动取末点值，
  即"末点曲率恒定外推"，与 `cumulative_bend_deg()` 的外推约定一致。

  Returns:
    长度 n 的曲率数组。模型不可用时返回全 0（直线假设，最保守）。
  """
  _s, kappa = model_curvature(positions, turn_rates, v_preds, v_ego)
  if kappa.size < 2:
    return np.zeros(n, dtype=np.float64)

  t_model = model_t_idxs()[:kappa.size]
  t_query = np.arange(n, dtype=np.float64) * float(dt)
  out = np.interp(t_query, t_model, kappa)
  if not np.all(np.isfinite(out)):
    return np.zeros(n, dtype=np.float64)
  return out.astype(np.float64)


def distance_to_curve_start_from_s(s: Sequence[float],
                                   kappa: Sequence[float],
                                   threshold: float = 0.0008) -> float:
  """用 (s, kappa) 序列求"到弯道起点"的距离 (m)。

  比 `distance_to_curve_start()` 的纯时间估算更准，因为模型输出的采样点
  本身就携带**沿路径的距离**，不需要再用 v*dt 近似。
  整个视野内都是直线时返回末点距离（= "看不到弯"）。
  """
  sv = np.asarray(s, dtype=np.float64)
  kv = np.asarray(kappa, dtype=np.float64)
  n = min(sv.size, kv.size)
  if n < 1:
    return 0.0
  sv, kv = sv[:n], kv[:n]
  hit = np.abs(kv) > float(threshold)
  if not np.any(hit):
    return float(max(sv[-1], 0.0))
  return float(max(sv[int(np.argmax(hit))], 0.0))


def curvature_from_poly(coeffs: Sequence[float], t: float) -> float:
  """由路径多项式系数求 t 时刻的曲率。

  **注意：chestnut 的 modelV2.position.x 不是多项式系数**（它是 33 点
  轨迹距离），所以本函数不参与车端路径。保留它只为两类场景：
    * 合成数据 / 单元测试里用多项式描述一条曲率已知的路径；
    * 若将来上游把 modelV2 换成多项式表示（部分 fork 如此），可直接复用。
  车端请用 `model_curvature()` / `sample_curvature_horizon()`。

  精确曲率 kappa = x'' / (1 + x'^2)^1.5。小转角下 x'^2 可忽略，
  但入弯初期 x' 可达 0.2，1.5 次方会带来约 6% 误差，故这里保留分母。
  """
  c = np.asarray(coeffs, dtype=np.float64)
  if c.size < 3:
    return 0.0
  c = np.pad(c, (0, max(0, 5 - c.size)))[:5]
  t = float(t)
  dx = c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t + 4.0 * c[4] * t ** 3
  ddx = 2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t * t
  denom = (1.0 + dx * dx) ** 1.5
  return float(ddx / max(denom, 1e-3))


def sample_horizon_from_poly(coeffs: Sequence[float],
                             n: int = CURVE_HORIZON_N,
                             dt: float = CURVE_HORIZON_DT) -> np.ndarray:
  """在 0 .. (n-1)*dt 上等间隔采样多项式路径的曲率（仅测试 / 合成用）。"""
  return np.array([curvature_from_poly(coeffs, i * dt) for i in range(n)],
                  dtype=np.float64)


def distance_to_curve_start(kappa_horizon: Sequence[float],
                            v_ego: float,
                            threshold: float = 0.0008) -> float:
  """由等间隔前瞻曲率估算"到弯道起点"的距离 (m)。

  这是**没有模型轨迹时**的退化路径（合成数据训练走这条）：
  第 i 个采样点对应距离约 i * CURVE_HORIZON_DT * v_ego。
  车端有 modelV2 时请优先用 `distance_to_curve_start_from_s()`。

  约定：第一个满足 |kappa| > threshold 的采样点即视为弯道起点
  （0.0008 1/m 约等于 1250 m 半径，低于此值人类基本不做减速准备）。
  整个前瞻窗都是直线时返回前瞻窗末端距离（即"看不到弯"）。
  """
  kh = np.asarray(kappa_horizon, dtype=np.float64)
  hit = np.abs(kh) > float(threshold)
  idx = int(np.argmax(hit)) if np.any(hit) else len(kh)
  return float(max(idx, 0)) * CURVE_HORIZON_DT * max(float(v_ego), 1.0)


# ---------------------------------------------------------------------------
# 特征构造
# ---------------------------------------------------------------------------


def steering_path_angle(steering_angle_deg: float,
                        v_ego: float,
                        steer_ratio: float = STEER_RATIO_DEFAULT,
                        wheelbase: float = WHEELBASE_DEFAULT) -> float:
  """把（车体约定的）方向盘角度折算为 **openpilot 约定**的 path_angle (rad)。

  与基线 `path_angle_base = κ · v` 同量纲（不含 `curvature_factor`）：

      κ_steer    = STEERING_SIGN · radians(steer) / (steer_ratio · wheelbase)
      path_angle = κ_steer · max(v, 0)

  为什么必须是这个量纲（v3 的核心修正）：`path_angle_last` 是"角速度量纲
  的航向角"（rad），而 `radians(steer)` 是"角度量纲的方向盘角"（rad）——
  两者差一个 `steer_ratio · wheelbase / v` 因子。v1/v2 直接把两者相减，
  得到的是一个被 `steer_ratio` 污染、且随车速反向漂移的伪量
  （实测 p90 达 6.5 rad，而 DBC 量程只有 ±0.5 rad）。

  符号：`carState.steeringAngleDeg` 与 openpilot 曲率约定反号（左转为负），
  故乘 `STEERING_SIGN`。**同一个 `STEERING_SIGN` 也适用于
  `carControl.actuators.steeringAngleDeg`** —— 两者是同一套车体约定。

  Args:
    steering_angle_deg: 方向盘转角 (deg)，车体约定。
    v_ego:              车速 (m/s)。取 max(v, 0)，倒车时退化为 0。
    steer_ratio:        转向比；非正/缺失时回落 `STEER_RATIO_DEFAULT`。
    wheelbase:          轴距 (m)；非正/过小时回落 `WHEELBASE_DEFAULT`。

  Returns:
    openpilot 约定的 path_angle (rad)。量级参考：κ=0.002、v=20 → 0.04 rad。
  """
  sr = float(steer_ratio) if steer_ratio and steer_ratio > 1.0 else STEER_RATIO_DEFAULT
  L = float(wheelbase) if wheelbase and wheelbase > 0.5 else WHEELBASE_DEFAULT
  kappa = STEERING_SIGN * math.radians(float(steering_angle_deg)) / (sr * L)
  return kappa * max(float(v_ego), 0.0)


def build_lateral_ff_features(kappa: float,
                              kappa_rate: float,
                              v_ego: float,
                              steering_angle_deg: float,
                              path_angle_last: float,
                              yaw_rate: float,
                              a_lat_filt: float,
                              kappa_hist: KappaHistory,
                              style: Sequence[float],
                              steer_ratio: float = STEER_RATIO_DEFAULT,
                              wheelbase: float = WHEELBASE_DEFAULT) -> np.ndarray:
  """构造横向前馈 NN 的输入向量。

  Args:
    kappa:                  当前规划曲率 (1/m)。
    kappa_rate:             曲率变化率 (1/m/s)。
    v_ego:                  车速 (m/s)。
    steering_angle_deg:     实测方向盘转角 (deg)，**车体约定**（左转为负）。
    path_angle_last:        上一帧下发的 path_angle (rad)，**openpilot 约定**
                            （左转为正）。契约见 `steering_path_angle` 的说明 ——
                            传进来之前必须已经统一到 openpilot 约定。
    yaw_rate:               实测横摆角速度 (rad/s)。
    a_lat_filt:             低通后的横向加速度 (m/s^2)。
    kappa_hist:             曲率历史（本函数不 push，由调用方维护）。
    style:                  风格向量，长度 STYLE_DIM。
    steer_ratio:            转向比（`CP.steerRatio`），用于把方向盘角度折算到
                            path_angle 空间。缺失时回落 16.8。
    wheelbase:              轴距 (m)（`CP.wheelbase`）。缺失时回落 3.025。

  Returns:
    归一化后的 float32 特征向量，形状 (LATERAL_FF_DIM,)。
  """
  v = max(float(v_ego), 0.0)
  k = float(kappa)

  a_lat = k * v * v

  # 指令 path_angle 与"方向盘实际所在位置等效的 path_angle"之差，**同量纲**。
  # 正号表示"指令已经给出去但车还没转到位"（PSCM 滞后 / 饱和的直接测量）。
  # 量纲与符号的来龙去脉见 FEATURE_VERSION 的 v2/v3 注释。
  steer_pa = steering_path_angle(steering_angle_deg, v, steer_ratio, wheelbase)
  steer_lag = float(path_angle_last) - steer_pa

  raw = [
    k,
    float(kappa_rate),
    v,
    a_lat,
    kappa_hist.lag(1),
    kappa_hist.lag(2),
    kappa_hist.lag(3),
    steer_lag,
    float(a_lat_filt),
    float(yaw_rate),
  ]
  raw.extend(float(s) for s in style)

  return normalize(raw, LATERAL_FF_SPEC)


def build_curve_speed_features(kappa_horizon: Sequence[float],
                               v_ego: float,
                               a_ego: float,
                               style: Sequence[float]) -> np.ndarray:
  """构造纵向弯道预算 NN 的输入向量。

  Args:
    kappa_horizon: 前瞻曲率序列，长度必须为 CURVE_HORIZON_N。
    v_ego:         车速 (m/s)。
    a_ego:         实测纵向加速度 (m/s^2)。
    style:         风格向量，长度 STYLE_DIM。

  Returns:
    归一化后的 float32 特征向量，形状 (CURVE_SPEED_DIM,)。
  """
  kh = np.asarray(kappa_horizon, dtype=np.float64)
  if kh.size != CURVE_HORIZON_N:
    raise ValueError(f"kappa_horizon must have {CURVE_HORIZON_N} samples, got {kh.size}")

  v = max(float(v_ego), 0.0)
  abs_kh = np.abs(kh)

  raw = list(kh)
  raw.extend([
    v,
    distance_to_curve_start(kh, v),
    float(abs_kh.max()),
    float(kh[-1]),
    float(a_ego),
  ])
  raw.extend(float(s) for s in style)

  return normalize(raw, CURVE_SPEED_SPEC)


# ---------------------------------------------------------------------------
# openpilot 消息适配（可选，仅在车端可用）
# ---------------------------------------------------------------------------


class MessageAdapter:
  """把 openpilot 的 cereal 消息折算成特征层需要的标量。

  刻意做成薄薄一层：**只做单位换算与字段取值，不做任何状态或决策**。
  状态（曲率历史、滤波）由 ford_nn_curve_control.py 持有。

  在没有 cereal 的环境（离线训练/测试）里，本类不可用 —— 导入时才尝试，
  失败则 `MessageAdapter.available` 为 False。
  """

  available = False

  def __init__(self, CP=None) -> None:
    self.CP = CP
    # 从车辆参数取转向比与轴距；取不到才回落常量（见 FEATURE_VERSION v3：
    # 这两个量直接决定 steer_lag 的量纲，不能用 1.0 糊弄过去）。
    self.steer_ratio = STEER_RATIO_DEFAULT
    self.wheelbase = WHEELBASE_DEFAULT
    try:
      if CP is not None:
        self.steer_ratio = float(getattr(CP, "steerRatio", 0.0) or 0.0) or STEER_RATIO_DEFAULT
        self.wheelbase = float(getattr(CP, "wheelbase", 0.0) or 0.0) or WHEELBASE_DEFAULT
    except Exception:
      self.steer_ratio = STEER_RATIO_DEFAULT
      self.wheelbase = WHEELBASE_DEFAULT

  @staticmethod
  def model_trajectory(model_v2) -> tuple[list[float], list[float], list[float]]:
    """取 modelV2 预测轨迹的 (position.x, orientationRate.z, velocity.x)。

    三者都是 33 点数组，**必须成对使用**才能算出曲率
    （见 `model_curvature`）。任何字段缺失都返回三个空列表，
    由调用方走"无模型输入"的降级路径。
    """
    try:
      return (list(model_v2.position.x),
              list(model_v2.orientationRate.z),
              list(model_v2.velocity.x))
    except Exception:  # noqa: BLE001
      return [], [], []

  @staticmethod
  def extract_lateral(model_v2, car_state, path_angle_last: float,
                      a_lat_filt: float) -> dict:
    """从消息里取出横向特征所需的原始标量。

    曲率走 `model_curvature()`（与 `curve_bend.path_curvature` 同一算法），
    `kappa_rate` 用模型前两个采样点做差分 —— 那两个点在 T_IDXS 上分别对应
    0 s 与约 0.0098 s，所以必须除以**真实的** Δt，不能假设 0.05 s。

    Returns:
      dict，键与 build_lateral_ff_features 的参数名对应。
    """
    v_ego = float(car_state.vEgo)
    _s, kappa = model_curvature(model_v2.position.x,
                                model_v2.orientationRate.z,
                                model_v2.velocity.x,
                                v_ego)
    k0 = float(kappa[0]) if kappa.size else 0.0
    if kappa.size > 1:
      t = model_t_idxs()
      dt = max(float(t[1] - t[0]), 1e-3)
      k_rate = (float(kappa[1]) - k0) / dt
    else:
      k_rate = 0.0

    return {
      "kappa": k0,
      "kappa_rate": k_rate,
      "v_ego": v_ego,
      "steering_angle_deg": float(car_state.steeringAngleDeg),
      "path_angle_last": float(path_angle_last),
      "yaw_rate": float(car_state.yawRate),
      "a_lat_filt": float(a_lat_filt),
    }
