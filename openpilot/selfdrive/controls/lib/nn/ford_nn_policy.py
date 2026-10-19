"""
ford_nn_policy.py —— 纯 numpy 的小型 MLP：推理 + 训练 + 权重序列化

为什么是纯 numpy 而不是 PyTorch / tinygrad
---------------------------------------------------------------------------
* **可离线训练、可车端推理、可单元测试，同一份代码。** 不引入训练框架意味着
  不会出现"训练图和部署图不一致"这一类最难查的静默偏差。
* openpilot 运行时**本来就带 numpy**，车端零额外依赖。
* 本模型规模很小（14→32→32→2，约 1.5k 参数），纯 numpy 的前向开销在
  20 Hz 的控制环里可以忽略（实测见 verify/test_nn.py）。
* 需要放大到 CNN/GRU 时，把 `Policy.forward` 换成 tinygrad 实现即可，
  权重布局（见 `state_dict()`）保持不变。

残差范式 —— 本文件最重要的设计
---------------------------------------------------------------------------
**所有输出都是"相对基线的有界残差"，且输出层零初始化。**

    out = r_max * tanh(z)

* 输出层权重初始化为 0 → `z = 0` → `tanh(0) = 0` → **out 恒等于 0**。
* 因此"未加载权重 / NN 关闭"时，输出精确为 0，上层控制器退化为原有行为，
  **不是"接近一致"而是逐位一致**（0.0 与 0.0 相加仍为 0.0）。
* 训练后输出被 `tanh` 硬性限制在 `[-r_max, +r_max]`，任何输入（含 OOD、
  含极端值）都不可能超出这个范围。这是安全关键系统里必须有的一层保证：
  **NN 的失效模式是"不生效"，而不是"输出离谱值"。**

真正的物理/安全限幅在 ford_nn_curve_control.py 里做第二道（见那里的
`_clamp_residual`），panda 固件里的 safety 是第三道。三道边界互相独立，
任何一道单独失效都不会导致不可接受的输出。
"""

from __future__ import annotations

import json
import os
from typing import Sequence

import numpy as np

# ---------------------------------------------------------------------------
# 权重文件格式版本
# ---------------------------------------------------------------------------

WEIGHTS_FORMAT_VERSION = 1


class Policy:
  """一个小型全连接网络：tanh 隐层 + tanh 有界输出。

  网络结构::

      x (in_dim)
        -> Linear(in_dim, hidden[0]) -> tanh
        -> Linear(hidden[0], hidden[1]) -> tanh
        -> ...
        -> Linear(hidden[-1], out_dim)
        -> r_max * tanh(.)            # 有界残差输出

  Attributes:
    in_dim:  输入维度。
    hidden:  隐层宽度元组。
    out_dim: 输出维度。
    r_max:   每个输出的残差半径，长度 out_dim，必须全为正。
  """

  def __init__(self, in_dim: int, hidden: Sequence[int] = (32, 32),
               out_dim: int = 1, r_max: Sequence[float] = (1.0,),
               seed: int = 0) -> None:
    self.in_dim = int(in_dim)
    self.hidden = tuple(int(h) for h in hidden)
    self.out_dim = int(out_dim)

    r = np.asarray(r_max, dtype=np.float64).reshape(-1)
    if r.size != self.out_dim:
      raise ValueError(f"r_max must have {self.out_dim} entries, got {r.size}")
    if np.any(r <= 0):
      raise ValueError("r_max entries must be positive")
    self.r_max = r

    dims = [self.in_dim, *self.hidden, self.out_dim]
    rng = np.random.default_rng(seed)

    self.W: list[np.ndarray] = []
    self.b: list[np.ndarray] = []
    for i in range(len(dims) - 1):
      fan_in, fan_out = dims[i], dims[i + 1]
      if i == len(dims) - 2:
        # 输出层零初始化 —— 保证初始输出恒为 0（见模块 docstring）
        self.W.append(np.zeros((fan_in, fan_out), dtype=np.float64))
      else:
        # Xavier/Glorot
        limit = np.sqrt(6.0 / (fan_in + fan_out))
        self.W.append(rng.uniform(-limit, limit, size=(fan_in, fan_out)))
      self.b.append(np.zeros(fan_out, dtype=np.float64))

    # Adam 状态
    self._m = [np.zeros_like(w) for w in self.W] + [np.zeros_like(b) for b in self.b]
    self._v = [np.zeros_like(w) for w in self.W] + [np.zeros_like(b) for b in self.b]
    self._t = 0

  # -------------------------------------------------------------------------
  # 推理
  # -------------------------------------------------------------------------

  @property
  def n_layers(self) -> int:
    return len(self.W)

  def _forward_cache(self, X: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
    """前向并缓存每层激活（供反向传播复用）。

    Args:
      X: (N, in_dim) 或 (in_dim,)。

    Returns:
      (acts, z_last)。acts[0] 是输入，acts[i+1] 是第 i 层 tanh 后的输出；
      z_last 是输出层**仿射**结果（尚未过 tanh）。
    """
    X = np.atleast_2d(np.asarray(X, dtype=np.float64))
    acts = [X]
    h = X
    for i in range(self.n_layers - 1):
      h = np.tanh(h @ self.W[i] + self.b[i])
      acts.append(h)
    z_last = h @ self.W[-1] + self.b[-1]
    return acts, z_last

  def forward(self, X: np.ndarray) -> np.ndarray:
    """前向推理。

    Args:
      X: (N, in_dim) 或 (in_dim,)。

    Returns:
      若输入是 1 维则返回 (out_dim,)，否则返回 (N, out_dim)。
      取值范围恒在 [-r_max, +r_max] 内。
    """
    single = np.asarray(X).ndim == 1
    _, z_last = self._forward_cache(X)
    out = self.r_max * np.tanh(z_last)
    return out[0] if single else out

  def is_zero_output(self, atol: float = 1e-12) -> bool:
    """输出是否恒为 0（即输出层仍是零初始化 / 权重已被清零）。"""
    return bool(np.all(np.abs(self.W[-1]) <= atol) and np.all(np.abs(self.b[-1]) <= atol))

  def zero_output_layer(self) -> None:
    """把输出层清零，使网络退回"恒输出 0"的基线行为。"""
    self.W[-1][...] = 0.0
    self.b[-1][...] = 0.0

  # -------------------------------------------------------------------------
  # 训练
  # -------------------------------------------------------------------------

  def loss(self, X: np.ndarray, Y: np.ndarray,
           l2_weight: float = 0.0, l2_output: float = 0.0) -> float:
    """计算带正则的 MSE 损失（仅用于监控，不参与反传）。"""
    pred = self.forward(X)
    mse = float(np.mean((pred - Y) ** 2))
    reg_w = l2_weight * sum(float(np.sum(w * w)) for w in self.W)
    reg_o = l2_output * float(np.mean(pred ** 2))
    return mse + reg_w + reg_o

  def fit(self, X: np.ndarray, Y: np.ndarray,
          epochs: int = 400,
          lr: float = 3e-3,
          batch_size: int = 256,
          l2_weight: float = 1e-5,
          l2_output: float = 0.0,
          seed: int = 0,
          verbose: bool = False) -> list[float]:
    """用 Adam + 手写反向传播训练。

    Args:
      X:          (N, in_dim) 特征。
      Y:          (N, out_dim) 目标残差（**已含符号的物理量**，不是归一化值）。
      epochs:     训练轮数。
      lr:         学习率。
      batch_size: mini-batch 大小。
      l2_weight:  权重衰减（抑制过拟合）。
      l2_output:  输出收缩项。>0 时鼓励网络"不确定就别偏离基线"，
                  对安全关键场景很有用。
      seed:       打乱顺序的随机种子。
      verbose:    每 10% 打印一次损失。

    Returns:
      每个 epoch 的平均损失列表。
    """
    X = np.atleast_2d(np.asarray(X, dtype=np.float64))
    Y = np.atleast_2d(np.asarray(Y, dtype=np.float64))
    if X.shape[0] != Y.shape[0]:
      raise ValueError(f"X/Y row mismatch: {X.shape[0]} vs {Y.shape[0]}")
    if X.shape[1] != self.in_dim:
      raise ValueError(f"X has {X.shape[1]} cols, policy expects {self.in_dim}")
    if Y.shape[1] != self.out_dim:
      raise ValueError(f"Y has {Y.shape[1]} cols, policy expects {self.out_dim}")

    n = X.shape[0]
    bs = int(min(max(batch_size, 1), n))
    rng = np.random.default_rng(seed)
    beta1, beta2, eps = 0.9, 0.999, 1e-8

    history: list[float] = []
    for epoch in range(int(epochs)):
      order = rng.permutation(n)
      epoch_loss = 0.0
      n_batches = 0

      for start in range(0, n, bs):
        idx = order[start:start + bs]
        xb, yb = X[idx], Y[idx]
        m = xb.shape[0]

        grads = self._backward(xb, yb, l2_weight, l2_output)
        epoch_loss += self._batch_loss(xb, yb, l2_weight, l2_output)
        n_batches += 1

        self._t += 1
        # Adam
        flat_params = self.W + self.b
        for pi, g in enumerate(grads):
          self._m[pi] = beta1 * self._m[pi] + (1 - beta1) * g
          self._v[pi] = beta2 * self._v[pi] + (1 - beta2) * (g * g)
          m_hat = self._m[pi] / (1 - beta1 ** self._t)
          v_hat = self._v[pi] / (1 - beta2 ** self._t)
          flat_params[pi] -= lr * m_hat / (np.sqrt(v_hat) + eps)

      avg = epoch_loss / max(n_batches, 1)
      history.append(avg)
      if verbose and (epoch % max(1, epochs // 10) == 0 or epoch == epochs - 1):
        print(f"  epoch {epoch:5d}  loss={avg:.6e}")

    return history

  def _batch_loss(self, xb: np.ndarray, yb: np.ndarray,
                  l2_weight: float, l2_output: float) -> float:
    pred = self.forward(xb)
    mse = float(np.mean((pred - yb) ** 2))
    reg_w = l2_weight * sum(float(np.sum(w * w)) for w in self.W)
    reg_o = l2_output * float(np.mean(pred ** 2))
    return mse + reg_w + reg_o

  def _backward(self, xb: np.ndarray, yb: np.ndarray,
                l2_weight: float, l2_output: float) -> list[np.ndarray]:
    """手写反向传播。返回 [dW0, dW1, ..., db0, db1, ...]（与 W+b 顺序一致）。"""
    m = xb.shape[0]
    acts, z_last = self._forward_cache(xb)
    pred = self.r_max * np.tanh(z_last)

    # dL/dpred：MSE 对 pred 的梯度。loss = mean((pred-yb)^2) 是对**全部元素**取均值，
    # 故分母是 m * out_dim，不是 m。
    denom = float(m * self.out_dim)
    dpred = 2.0 * (pred - yb) / denom
    # 输出收缩项 d(l2_output * mean(pred^2))/dpred
    if l2_output > 0.0:
      dpred = dpred + l2_output * 2.0 * pred / denom

    # 过输出层 tanh 与 r_max
    dz = dpred * self.r_max * (1.0 - np.tanh(z_last) ** 2)

    dW: list[np.ndarray] = [None] * self.n_layers  # type: ignore[list-item]
    db: list[np.ndarray] = [None] * self.n_layers  # type: ignore[list-item]

    h_prev = acts[-1]
    dW[-1] = h_prev.T @ dz + 2.0 * l2_weight * self.W[-1]
    db[-1] = dz.sum(axis=0)

    dh = dz @ self.W[-1].T
    for i in range(self.n_layers - 2, -1, -1):
      # 过隐层 tanh
      dz_i = dh * (1.0 - acts[i + 1] ** 2)
      h_in = acts[i]
      dW[i] = h_in.T @ dz_i + 2.0 * l2_weight * self.W[i]
      db[i] = dz_i.sum(axis=0)
      dh = dz_i @ self.W[i].T

    return dW + db

  # -------------------------------------------------------------------------
  # 序列化
  # -------------------------------------------------------------------------

  def state_dict(self) -> dict:
    """导出为纯 numpy 字典（**部署端的契约**，与训练框架无关）。"""
    d: dict[str, object] = {
      "in_dim": self.in_dim,
      "hidden": list(self.hidden),
      "out_dim": self.out_dim,
      "r_max": self.r_max.copy(),
    }
    for i, (w, b) in enumerate(zip(self.W, self.b)):
      d[f"W{i}"] = w.copy()
      d[f"b{i}"] = b.copy()
    return d

  def load_state_dict(self, d: dict) -> None:
    """从 state_dict 载入并校验形状（形状不符直接报错，不静默降级）。"""
    if int(d["in_dim"]) != self.in_dim or int(d["out_dim"]) != self.out_dim:
      raise ValueError("state_dict dimension mismatch")
    if tuple(int(h) for h in d["hidden"]) != self.hidden:
      raise ValueError("state_dict hidden layer mismatch")
    r = np.asarray(d["r_max"], dtype=np.float64)
    if r.shape != self.r_max.shape:
      raise ValueError("state_dict r_max mismatch")
    self.r_max = r

    for i in range(self.n_layers):
      w = np.asarray(d[f"W{i}"], dtype=np.float64)
      b = np.asarray(d[f"b{i}"], dtype=np.float64)
      if w.shape != self.W[i].shape or b.shape != self.b[i].shape:
        raise ValueError(f"layer {i} shape mismatch: {w.shape} vs {self.W[i].shape}")
      self.W[i] = w.copy()
      self.b[i] = b.copy()

  def save(self, path: str) -> None:
    """保存为 .npz（含 meta）。"""
    sd = self.state_dict()
    payload = {k: v for k, v in sd.items() if isinstance(v, np.ndarray)}
    payload["meta"] = np.array(json.dumps({
      "format": WEIGHTS_FORMAT_VERSION,
      "in_dim": self.in_dim,
      "hidden": list(self.hidden),
      "out_dim": self.out_dim,
      "r_max": self.r_max.tolist(),
    }))
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    np.savez_compressed(path, **payload)

  @classmethod
  def load(cls, path: str) -> "Policy":
    """从 .npz 载入。格式版本不符直接报错。"""
    with np.load(path, allow_pickle=False) as z:
      meta = json.loads(str(z["meta"]))
      if int(meta.get("format", -1)) != WEIGHTS_FORMAT_VERSION:
        raise ValueError(f"unsupported weights format: {meta.get('format')}")
      p = cls(int(meta["in_dim"]), tuple(meta["hidden"]), int(meta["out_dim"]),
              meta["r_max"])
      # 维度信息存在 meta 里，数组在其余键上；合并成完整 state_dict
      d = {k: z[k] for k in z.files if k != "meta"}
      d.update({
        "in_dim": meta["in_dim"],
        "hidden": meta["hidden"],
        "out_dim": meta["out_dim"],
        "r_max": np.asarray(meta["r_max"], dtype=np.float64),
      })
      p.load_state_dict(d)
    return p


# ---------------------------------------------------------------------------
# 组合：一次加载两个模型
# ---------------------------------------------------------------------------


class PolicyBundle:
  """把横向前馈与纵向弯道预算两个模型打包在一起。

  分开建模的理由：两者的输入时间尺度差一个数量级（横向 20 Hz 逐帧，
  纵向 0.5 s × 7 点前瞻 3 s），混在一个网络里会让横向被纵向的慢变量主导。
  """

  def __init__(self,
               lateral_ff: Policy | None = None,
               curve_speed: Policy | None = None) -> None:
    self.lateral_ff = lateral_ff
    self.curve_speed = curve_speed

  @property
  def ready(self) -> bool:
    """两个模型都加载了才认为可用（宁可不用，也不要半套）。"""
    return self.lateral_ff is not None and self.curve_speed is not None

  def save(self, path: str) -> None:
    payload: dict[str, np.ndarray] = {}
    for name, pol in (("lateral", self.lateral_ff), ("longitudinal", self.curve_speed)):
      if pol is None:
        continue
      # 维度信息是标量/元组，np.savez 只接受数组，必须显式转换 ——
      # 否则加载端会 KeyError（这个坑踩过一次）。
      payload[f"{name}/in_dim"] = np.int64(pol.in_dim)
      payload[f"{name}/hidden"] = np.asarray(pol.hidden, dtype=np.int64)
      payload[f"{name}/out_dim"] = np.int64(pol.out_dim)
      payload[f"{name}/r_max"] = np.asarray(pol.r_max, dtype=np.float64)
      for i, (w, b) in enumerate(zip(pol.W, pol.b)):
        payload[f"{name}/W{i}"] = w
        payload[f"{name}/b{i}"] = b

    payload["meta"] = np.array(json.dumps({
      "format": WEIGHTS_FORMAT_VERSION,
      "has_lateral": self.lateral_ff is not None,
      "has_longitudinal": self.curve_speed is not None,
    }))
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    np.savez_compressed(path, **payload)

  @classmethod
  def load(cls, path: str) -> "PolicyBundle":
    with np.load(path, allow_pickle=False) as z:
      meta = json.loads(str(z["meta"]))
      if int(meta.get("format", -1)) != WEIGHTS_FORMAT_VERSION:
        raise ValueError(f"unsupported weights format: {meta.get('format')}")

      def _build(prefix: str) -> Policy | None:
        keys = [k for k in z.files if k.startswith(prefix + "/")]
        if not keys:
          return None
        d = {k[len(prefix) + 1:]: z[k] for k in keys}
        d["r_max"] = np.asarray(d["r_max"], dtype=np.float64)
        # 注意用 Policy 而不是 cls —— cls 在这里是 PolicyBundle
        p = Policy(
          int(d["in_dim"]), tuple(int(h) for h in d["hidden"]),
          int(d["out_dim"]), d["r_max"])
        p.load_state_dict(d)
        return p

      return cls(_build("lateral"), _build("longitudinal"))
