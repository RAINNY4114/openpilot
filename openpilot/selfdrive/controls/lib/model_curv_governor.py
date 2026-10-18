#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
model_curv_governor.py — 模型直出曲率(modelV2.action.desiredCurvature)的整型器

为什么需要它
------------
2026-10-18 实测（route 00000048--12b95d5a3e，seg8，>50 km/h）：

    modelV2.action.desiredCurvature  std = 6.0e-4 1/m
      谱峰 0.03 Hz(37%)  ← 真实道路曲率，必须原样通过
          0.24 Hz( 6%)  ← 大模型输出噪声
          0.49 Hz( 4%)  ← 大模型输出噪声
    整条指令的功率：0.2~0.7 Hz 占 29%，>0.7 Hz 占 19%
    方向盘转角 pp50 = 4.7°（指令）/ 3.2°（实际），指令->实际滞后 280 ms

也就是"高速左右摇晃"的指令侧来源：在 18.5 m/s 上 6e-4 1/m 就是
0.2 m/s² 的侧向加速度；>0.7 Hz 的分量纯粹是采样/感知抖动，没有任何
道路信息，只会让舵角高频哆嗦。

不能动模型本身（大模型加载链路是禁区），所以只在命令侧整型。

设计（在真实日志上扫参后定，见 _sim_tune.py）
--------------------------------------------
    LP 2 Hz 预滤波  ->  「速率限制 6e-4 (1/m)/s + 加加速度限制 2e-2 (1/m)/s^3」

在 seg8 >50 km/h 的实测结果：

                        0.2~0.7Hz   >0.7Hz    std        对 1s 慢参考的跟踪误差
    原始                 29.4%      18.8%    6.00e-4     RMS 1.2e-4 / max 6.1e-4
    LP2Hz+lim 6e-4       38.8%       6.2%    4.77e-4     RMS 1.7e-4 / max 8.4e-4

即：整体抖动 -21%、高频哆嗦降到 1/3，而跟随真实道路曲率的代价只是
额外 <0.3° 的等效舵角滞后（同一段路上模型延迟本来就是 280 ms）。

为什么不按"曲率幅值"分区：实测噪声幅度 1.3e-4 与真实道路曲率 5.7e-4
严重重叠，幅值门限分不开；而斜率/加加速度可以分开（噪声是来回变号）。
所以只按"是否正在变道 / 是否急弯(|κ|>knee)"直通，其余一律整型。

安全设计
--------
- 默认 enabled=true；配置缺失用保守默认值；任何异常直接返回原值
- 未激活 / 低速 / 正在变道 / 急弯 -> 直通且不积累状态
- 稳态无偏置：低通/限速/限加速度的稳态都等于输入，不会把车持续带偏
- 配置热加载（/data/model_curv.json，按 mtime），改完即生效
"""

import json
import math
import os
import time

CONFIG_PATH = "/data/model_curv.json"

DEFAULTS = {
    "enabled": True,
    # 预低通：只杀 13 Hz 模型采样在 100 Hz 控制环里的混叠，不影响任何道路信息
    "lp_hz": 2.0,           # 一阶低通截止频率 (Hz)，<=0 关闭
    # 速率限制（一阶）：超过这个变化率就是抖动而不是道路
    "rate": 6.0e-4,         # (1/m)/s
    # 加加速度限制（二阶）：让限速过程是 S 曲线而不是折线，进一步去掉来回变号
    "jerk": 2.0e-2,         # (1/m)/s^3
    # 直通门限：变道中、或曲率大于此值（急弯/变道起步）时完全直通
    "knee_curv": 6.0e-3,    # 1/m
    "min_speed_ms": 3.0,
    "log": False,
}


class ModelCurvatureGovernor:
  """整型模型直出曲率。update() 永远返回有限 float 或原值。"""

  def __init__(self, config_path: str = CONFIG_PATH):
    self.config_path = config_path
    self.cfg = dict(DEFAULTS)
    self._cfg_mtime = None
    self._last_cfg_check = 0.0
    self._load_config(force=True)
    # 状态：预滤波值 / 输出 / 输出变化率
    self.z = 0.0
    self.out = 0.0
    self.rate_state = 0.0
    self.raw = 0.0
    self.bypass = 0
    self._last_log_t = 0.0

  # ------------------------------------------------------------------ config
  def _load_config(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and (now - self._last_cfg_check) < 1.0:
      return
    self._last_cfg_check = now
    try:
      st = os.stat(self.config_path)
      if not force and self._cfg_mtime is not None and st.st_mtime == self._cfg_mtime:
        return
      with open(self.config_path, "r") as f:
        user = json.load(f)
      cfg = dict(DEFAULTS)
      if isinstance(user, dict):
        for k, v in user.items():
          if k in cfg:
            cfg[k] = v
      self.cfg = cfg
      self._cfg_mtime = st.st_mtime
    except FileNotFoundError:
      self.cfg = dict(DEFAULTS)
      self._cfg_mtime = None
    except Exception:
      pass

  def _f(self, key: str, default: float) -> float:
    try:
      val = float(self.cfg[key])
      return val if math.isfinite(val) else float(default)
    except Exception:
      return float(default)

  def _b(self, key: str, default: bool) -> bool:
    try:
      return bool(self.cfg[key])
    except Exception:
      return bool(default)

  # ------------------------------------------------------------------ update
  def reset(self) -> None:
    self.z = 0.0
    self.out = 0.0
    self.rate_state = 0.0
    self.raw = 0.0
    self.bypass = 0

  def update(self, k_des: float, v_ego: float, active: bool, lane_change: bool,
             dt: float) -> float:
    try:
      self._load_config()
      x = float(k_des)
      if not math.isfinite(x):
        self.reset()
        return x
      self.raw = x

      knee = max(0.0, self._f("knee_curv", 6.0e-3))

      # ---- 直通：不积累状态，回到整型时不会跳变 ----
      if (not self._b("enabled", True)) or (not active) or bool(lane_change) or \
         abs(x) > knee or float(v_ego) < self._f("min_speed_ms", 3.0):
        self.z = x
        self.out = x
        self.rate_state = 0.0
        self.bypass = 1
        return x

      dt = max(1e-3, min(0.1, float(dt)))
      self.bypass = 0

      # ---- 1) 预低通（杀 13 Hz 采样混叠）----
      fc = self._f("lp_hz", 2.0)
      if fc > 0.0:
        a = 1.0 - math.exp(-2.0 * math.pi * fc * dt)
        self.z += a * (x - self.z)
        x = self.z
      else:
        self.z = x

      # ---- 2) 速率限制（一阶）----
      rate = max(0.0, self._f("rate", 6.0e-4))
      d_target = (x - self.out) / dt
      if rate > 0.0:
        d_target = max(-rate, min(rate, d_target))

      # ---- 3) 加加速度限制（二阶）----
      # 变化率只能以 jerk 的加速度逼近目标速率，于是输出是 S 曲线而不是
      # 折线；来回变号的噪声（需要很高的加加速度）被显著衰减，
      # 而单调变化的真实道路曲率几乎不受影响。
      jerk = max(0.0, self._f("jerk", 2.0e-2))
      if jerk > 0.0:
        max_dr = jerk * dt
        self.rate_state += max(-max_dr, min(max_dr, d_target - self.rate_state))
      else:
        self.rate_state = d_target

      y = self.out + self.rate_state * dt

      if not math.isfinite(y):
        self.reset()
        return x
      self.out = y

      if self._b("log", False):
        now = time.monotonic()
        if now - self._last_log_t > 2.0:
          self._last_log_t = now
          print(f"[model_curv] raw={self.raw:+.6f} out={y:+.6f} "
                f"rate={self.rate_state:.2e} v={float(v_ego):.1f}")
      return float(y)
    except Exception:
      self.reset()
      return x

  # ------------------------------------------------------------------ diag
  def status(self) -> dict:
    return {
      "raw": round(self.raw, 6),
      "out": round(self.out, 6),
      "rate": round(self.rate_state, 9),
      "bypass": int(self.bypass),
    }
