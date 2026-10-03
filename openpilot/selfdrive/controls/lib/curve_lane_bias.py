#!/usr/bin/env python3
"""
curve_lane_bias.py — 弯道内圈偏移（车道居中略偏向弯道内侧）

需求
----
"高速路上出现弯道的时候…保持车道居中略偏向内圈"：直道上完全居中（偏置 = 0），
进入弯道后随曲率大小**按比例**把车往弯道内侧挪一点，最内侧约 0.20 m
（约半个车宽以内，不会压线）。

为什么用"位置闭环"而不是"恒定曲率偏置"
--------------------------------------
本机横向链路是「端到端模型直出曲率」：

    modelV2.action.desiredCurvature -> clip_curvature -> actuators.curvature
      -> Ford carcontroller -> CAN

这条链路**没有横向位置积分器**，所以单纯加一个恒定曲率偏置会被模型闭环抵消掉
（车只是短暂偏移一下又回到中心）。正确做法是加位置闭环：

    lane_center_y = (laneLines[1].y[0] + laneLines[2].y[0]) / 2
    error = lane_center_y + bias
    trim  = clamp(kp*error + ki*∫error, -max_curv, +max_curv)

注入点与 lateral_clearance.py 相同：controlsd.py 在 clip_curvature 之前叠加。

符号约定（与 lateral_clearance.py / 本仓库 ldw.py 一致）
    modelV2 车体系里 +y = 右（左线 y<0，右线 y>0）
    lane_center_y > 0  <=>  车偏左
    desiredCurvature > 0  <=>  右转（orientationRate.z > 0 也是右转）
    bias > 0  <=>  希望车向右挪
=> 右转(κ>0)时内圈在右 -> bias > 0；左转(κ<0)时 bias < 0。即 bias = sign(κ)*|偏置|。

安全设计
--------
- 输出硬限幅 max_curv（默认 0.00040 1/m，与 lateral_clearance 同量级）
- 总偏置限幅 bias_max_m（默认 0.20 m）
- 四重闸门：latActive / 车速 >= min_speed_ms / 车道线可信 / 司机未在打方向
- **变道中一律输出 0**（不跟变道、不跟 lane_turn_controller 抢方向）
- **人工转向让权**：司机正在打方向时输出 0 并清积分（见下）
- 车道线不可信时输出 0 并清积分（防 windup）
- 整个 update 包 try/except：任何异常都返回 0，绝不把异常抛进 controlsd
- 配置热加载（按 mtime），可边开边调：/data/curve_lane_bias.json

与 hwh-kavin/openpilot cf5bb735 的对照（2026-09-28 分析）
--------------------------------------------------------
参考 commit：hwh-kavin/openpilot cf5bb735 "ford: extract curvature lateral into
its own module, angle-primary infra split"。那份 commit 是**横向架构重构**
（曲率为主 -> 角度为主），与"弯道纵向减速"不是同一件事，本模块**并未**参考它
（我们的曲线减速/HUD 来自 RAINNY4114/openpilot 264abad @ ford）。

但它有两条经验对本模块直接适用，本 fork 已经在下游具备：

1) **实测曲率偏差钳位（deviation clip）**
   kavin 在 angle 模式把命令曲率钳到 `current_curvature ± angle_deviation_clip`
   （默认 0.005；试过 0.002 -> 弯道入口 understeer，0.008 -> 切弯/跑出车道）。
   本 fork 等价物在 carcontroller 的 `FordCurveController._current_curvature_limit()`：
   车速 > cc_min_speed 时把请求曲率钳到 `current_curvature ± dp_ford_curvature_error`
   （默认 0.002）。我们的 max_curv=0.00040 远小于 0.002，天然落在带内 ——
   所以这里**不重复实现**钳位，避免两处限幅互相打架。

2) **人工转向让权（human-turn override）**
   kavin 的 HumanTurnDetector：司机持续手动转向时横向强制失活（mode 0），
   松手后经软限速率斜坡恢复（否则 PSCM 要 2-3 s 才能重新接管）。
   本 fork 等价物同样在 `FordCurveController`：human_turn_active 时直接 return 0.0，
   松手后走 `_apply_htd_ramp()`。**下游已经把命令清零了**，
   所以本模块加这道闸门不是为了"安全"（下游已兜底），而是为了**模块自身状态正确**：
   司机打方向时车的横向位置是司机控制的，此时若继续跑位置环，积分会累积在
   一个无意义的误差上；提前 reset 可让松手瞬间没有残留，且 status() 不会误报。

   注意：`CC.latActive` 在司机打方向时**仍然是 True**（它只反映 steerFault /
   standstill / get_lat_active），所以**必须**单独用 `CS.steeringPressed` 判据。

⚠️ 风险提示（来自 kavin 参考的负面经验）
   kavin 的 lateral_angle_ext.py 文档写明：曲率模式下的车道定位 PID
   （`LC_PID_controller`）"never actually tracked lane center correctly in this mode
   and was removed; only the DBC-required zero c0 remains"。
   本模块属于同一族做法（位置环输出曲率修正），bluepilot 最终放弃了它。
   本 fork 是 angle 模式（`SteerControlType.angle`），曲率经
   `VM.get_steer_from_curvature()` 转成转向角下发，机理与纯曲率模式不完全相同，
   但**必须实车验证**：重点看能否稳定保持内圈偏移、松手/出弯是否回正、有无振荡。
   `enabled` 可随时置 false 完全回退原生行为。
"""

import json
import math
import os
import time

import numpy as np

from openpilot.selfdrive.controls.lib import curve_bend

CONFIG_PATH = "/data/curve_lane_bias.json"

DEFAULTS = {
    "enabled": True,        # 默认开启（写成 false 即可完全关闭，行为回到原生）
    "bias_max_m": 0.20,     # 最大横向偏置（米），>0 = 往弯道内侧挪
    "k_ref": 0.004,         # 达到 bias_max_m 所需的曲率 (1/m)，0.004 = 半径 250 m
    "kp": 0.0010,           # 位置环比例增益（曲率 / 米）
    "ki": 0.0004,           # 位置环积分增益
    "max_curv": 0.00040,    # 输出硬限幅 (1/m)
    "min_speed_ms": 8.0,    # 低于此速度不介入
    "lane_prob_thr": 0.50,  # 车道线可信度阈值
    "smooth_alpha": 0.15,   # 偏置低通系数
    "window_m": 60.0,       # 用前方多远内的曲率判定"弯道方向/大小"
    "require_bend_deg": 0.0,  # >0 时额外要求累计转角 >= 该值（0 = 不要求，见 README/报告）
    "steer_override_gate": True,  # 司机打方向时让权（清状态、输出 0）
    "log": False,
}


class CurveLaneBias:
  """弯道内圈偏移。update() 永远返回一个有限 float（曲率修正量，1/m）。"""

  def __init__(self, config_path: str = CONFIG_PATH):
    self.config_path = config_path
    self.cfg = dict(DEFAULTS)
    self._cfg_mtime = None
    self._last_cfg_check = 0.0

    self.integral = 0.0
    self.bias_filtered = 0.0
    self.last_error = 0.0
    self.last_trim = 0.0
    self.last_reason = "off"
    self.last_k = 0.0
    self.last_bend_deg = 0.0
    self._last_log_t = 0.0

    self._load_config(force=True)

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
      # 配置损坏 -> 保持上一次的有效配置，绝不让异常外泄
      pass

  def _f(self, key: str, default: float) -> float:
    try:
      return float(self.cfg[key])
    except Exception:
      return float(default)

  def _b(self, key: str, default: bool) -> bool:
    try:
      return bool(self.cfg[key])
    except Exception:
      return bool(default)

  # ------------------------------------------------------------------ signals
  @staticmethod
  def _lane_center_y(model_v2, prob_thr: float):
    """返回 (lane_center_y, ok)。lane_center_y > 0 表示车偏左。"""
    try:
      lines = model_v2.laneLines
      probs = model_v2.laneLineProbs
      if len(lines) < 4 or len(probs) < 4:
        return 0.0, False
      if probs[1] < prob_thr or probs[2] < prob_thr:
        return 0.0, False
      y_left = float(lines[1].y[0])    # 左线为负
      y_right = float(lines[2].y[0])   # 右线为正
      if not (math.isfinite(y_left) and math.isfinite(y_right)):
        return 0.0, False
      width = y_right - y_left
      if not (2.0 <= width <= 6.0):    # 车道宽度合理性检查
        return 0.0, False
      return (y_left + y_right) / 2.0, True
    except Exception:
      return 0.0, False

  def _curve_k(self, model_v2, v_ego: float, window_m: float):
    """前方 window_m 内的最大 |κ| 及其带符号值。返回 (k_abs, k_signed)。"""
    try:
      pos = list(model_v2.position.x)
      tr = list(model_v2.orientationRate.z)
      vp = list(model_v2.velocity.x)
      if len(pos) < 2 or len(pos) != len(tr) or len(tr) != len(vp):
        return 0.0, 0.0
      positions, abs_k, signed_k = curve_bend.path_curvature(pos, tr, vp, v_ego)
      if positions.size < 2:
        return 0.0, 0.0
      mask = positions <= float(window_m)
      if not np.any(mask):
        return 0.0, 0.0
      k_w = abs_k[mask]
      s_w = signed_k[mask]
      idx = int(np.argmax(k_w))
      return float(k_w[idx]), float(s_w[idx])
    except Exception:
      return 0.0, 0.0

  # ------------------------------------------------------------------ update
  def update(self, active: bool, model_v2, v_ego: float, dt: float,
             lane_change_active: bool = False, steering_pressed: bool = False) -> float:
    """返回要加到 desired_curvature 上的曲率修正量 (1/m)。

    steering_pressed: 司机是否正在打方向（CS.steeringPressed）。
      注意 CC.latActive 在司机打方向时仍为 True，所以必须单独传这个判据。
    """
    try:
      self._load_config()

      if (not self._b("enabled", True)) or (not active) or lane_change_active:
        self._reset("off")
        return 0.0
      # 人工转向让权：司机正在打方向时不介入。
      # 下游 FordCurveController 的 HTD 会把曲率命令清零，这里提前让权是为了
      # 不让位置环在"司机控制的横向位置"上继续积分（避免松手瞬间残留）。
      if self._b("steer_override_gate", True) and steering_pressed:
        self._reset("steer_override")
        return 0.0
      if v_ego < self._f("min_speed_ms", 8.0):
        self._reset("slow")
        return 0.0

      lane_center_y, ok = self._lane_center_y(model_v2, self._f("lane_prob_thr", 0.50))
      if not ok:
        self._reset("no_lane")
        return 0.0

      # ---- 弯道方向与大小 -> 目标偏置（往内圈） ----
      window_m = self._f("window_m", 60.0)
      k_abs, k_signed = self._curve_k(model_v2, v_ego, window_m)
      self.last_k = k_abs

      if self._f("require_bend_deg", 0.0) > 0.0:
        bend_deg = curve_bend.bend_from_model(model_v2, v_ego)
        self.last_bend_deg = bend_deg
        if bend_deg < self._f("require_bend_deg", 0.0):
          self._reset("bend_low")
          return 0.0

      k_ref = max(1e-4, self._f("k_ref", 0.004))
      bias_max = max(0.0, self._f("bias_max_m", 0.20))
      amount = bias_max * min(1.0, k_abs / k_ref)
      # 右转 κ>0 -> 内圈在右 -> bias>0
      bias_target = math.copysign(amount, k_signed) if k_abs > 1e-4 else 0.0
      if not math.isfinite(bias_target):
        bias_target = 0.0

      a = max(0.01, min(1.0, self._f("smooth_alpha", 0.15)))
      self.bias_filtered += a * (bias_target - self.bias_filtered)

      # ---- PI 位置闭环 ----
      error = lane_center_y + self.bias_filtered
      kp = self._f("kp", 0.0010)
      ki = self._f("ki", 0.0004)
      max_curv = self._f("max_curv", 0.00040)

      p_term = kp * error
      self.integral += ki * error * dt
      if ki > 1e-9:
        lim = max_curv / ki
        self.integral = max(-lim, min(lim, self.integral))

      trim = p_term + self.integral
      trim = max(-max_curv, min(max_curv, trim))
      if ki > 1e-9 and abs(p_term + self.integral) > max_curv:
        self.integral = max(-max_curv, min(max_curv, trim - p_term))

      self.last_error = error
      self.last_trim = trim
      self.last_reason = "curve_L" if bias_target < 0 else ("curve_R" if bias_target > 0 else "straight")

      if self._b("log", False):
        now = time.monotonic()
        if now - self._last_log_t > 2.0:
          self._last_log_t = now
          print(f"[curve_lane_bias] k={k_abs:.4f} bias={self.bias_filtered:+.3f} "
                f"e={error:+.3f} trim={trim:+.5f} why={self.last_reason} v={v_ego:.1f}")

      if not math.isfinite(trim):
        self._reset("nan")
        return 0.0
      return float(trim)
    except Exception:
      # 绝不把异常抛进 controlsd
      self._reset("exc")
      return 0.0

  def _reset(self, reason: str) -> None:
    self.integral = 0.0
    self.bias_filtered = 0.0
    self.last_trim = 0.0
    self.last_reason = reason

  # ------------------------------------------------------------------ diag
  def status(self) -> dict:
    return {
      "enabled": self._b("enabled", True),
      "bias_m": round(self.bias_filtered, 3),
      "k": round(self.last_k, 5),
      "error_m": round(self.last_error, 3),
      "trim_curv": round(self.last_trim, 6),
      "reason": self.last_reason,
    }
