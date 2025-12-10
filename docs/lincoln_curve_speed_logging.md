# 林肯/Ford 弯道限速与调试日志一览（可复刻指南）

## 功能概述
- **弯道限速**：基于视觉模型曲率，按舒适侧向上限计算目标速度，提前收紧纵向加速度和 MPC 约束，避免“画线准但车不减速”。
- **调试日志**：将曲率/限速/约束、实际加速度、接管/故障/告警等关键状态写入文件，便于离线分析（WinSCP 可直接拷贝）。

## 开关与参数
- 开关位置：`selfdrive/ui/layouts/settings/lincoln.py`
  - `动态弯道限速` → Param `dp_lincoln_curve_speed`（PERSISTENT，bool）
  - `弯道限速调试日志` → Param `dp_lincoln_curve_log`（PERSISTENT，bool）
- Param 声明：`common/params_keys.h`
  - `{"dp_lincoln_curve_speed", {PERSISTENT, BOOL, "0"}}`
  - `{"dp_lincoln_curve_log", {PERSISTENT, BOOL, "0"}}`

## 控制逻辑入口
- Planner：`selfdrive/controls/plannerd.py`
  - 周期读取 Params，按品牌为 Ford/Lincoln 时将 `dp_lincoln_curve_speed`、`dp_lincoln_curve_log` 传入 `LongitudinalPlanner.update(...)`。
- 核心算法：`selfdrive/controls/lib/longitudinal_planner.py`
  - `_apply_lincoln_curve_speed`：弯道限速计算与 MPC 约束收紧。
  - `_log_curve`：调试日志写入。

## 弯道限速算法要点
- 取模型前方 ≤80m 曲率，限制在信号上限 0.02 m⁻¹；平滑 + 进入/退出滞回（`k_enter≈0.006，k_exit≈0.0036`）。
- 目标侧向上限 ≈1.44 m/s²（舒适 1.6×安全系数 0.9），限速 `v_limit = sqrt(a_lat_limit / k_smooth)`。
- 等效减速度 `a = (v_f² - v_i²)/(2d)`，封顶不超过 -3.0 m/s²，收紧全局 `accel_clip[1]` 并逐时间步写入 MPC 上限 `mpc.params[:,1]`。
- 防抖/早退：
  - 模型无效、车速 <1 m/s、无前瞻点、曲率/距离过小、当前已低于限速都会早退。
  - 退出滞回 + 定时器（0.5s）避免出口抖动。

## 调试日志
- 路径：`/data/media/0/lincoln_curve_logs/`，按日命名 `curve_YYYYMMDD.log`，默认 0.5s 节流；横向关闭/告警事件强制写入（不节流）。
- 记录字段（常规/事件）：
  - 触发原因 `reason`
  - 车速 `v_ego`、限速 `v_limit`、平滑/最大曲率 `k_smooth/k_max`、前瞻距离 `dist`
  - 等效减速度 `a_req`、收紧后加速度上限 `accel_clip_max`、MPC 上限最小值 `mpc_a_max_min`
  - 实际加速度 `accel_actual`、指令加速度/曲率 `accel_cmd/curv_cmd`、实测曲率 `curv_current=-yawRate/v`
  - 转向扭矩 `steer_torque`、临时/永久故障 `steer_fault_*`、传感器状态 `sensors_invalid`
  - 接管状态 `steer_pressed/gas_pressed/brake_pressed`、横向使能 `lat_active`、告警文本 `alerts`
- 早退原因日志：`model_invalid/low_speed/no_window_points/k_too_small/below_enter/under_limit` 等。

## 复刻步骤
1) UI 开关：在你的分支添加同名 toggle，写 Params `dp_lincoln_curve_speed`、`dp_lincoln_curve_log`。
2) Params 声明：在 `common/params_keys.h` 增加上述键。
3) Planner 传参：在品牌判定后把开关传入 `LongitudinalPlanner.update`。
4) 控制逻辑：复制 `_apply_lincoln_curve_speed` 与 `_log_curve` 的实现，确保前瞻、滞回、MPC 约束收紧和日志字段一致。
5) 日志目录：保证 `/data/media/0/lincoln_curve_logs/` 可写；如需大小限制可自行添加轮转。

## 已知物理/安全限制
- Ford 非 CAN FD 曲率信号上限 ±0.02 m⁻¹、EPS 侧向能力 ~2 m/s²；弯前必须降速，否则会触发转向不足/Take Control。
- 方向盘接管阈值 1 Nm，EPS 质量码/故障会导致横向退出；日志已记录相关状态用于排查。

