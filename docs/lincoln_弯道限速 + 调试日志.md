# Lincoln 面板「弯道限速 + 调试日志 + 可调参数」复刻指引（1:1 全量版）
目标：在其他分支完整还原弯道限速、调试日志与转向响应调节。默认行为与原版一致，用户不调整时无变化。
路径逐项迁移即可。

## 1) 参数键（common/params_keys.h）
开关：
```cpp
{"dp_lincoln_curve_speed", {PERSISTENT, BOOL, "0"}},  // 弯道限速开关
{"dp_lincoln_curve_log",   {PERSISTENT, BOOL, "0"}},  // 调试日志开关
```
弯道限速可调（默认值）：
```cpp
{"dp_lincoln_curve_window_m", {PERSISTENT, INT, "130"}}, // 前瞻 50–190 m
{"dp_lincoln_curve_k_enter",  {PERSISTENT, INT, "4"}},   // 触发曲率 0.002–0.010（存 1e-3，默认 0.004）
{"dp_lincoln_curve_alat",     {PERSISTENT, INT, "120"}}, // 横向上限 1.20 m/s²（存 cm/s²）
{"dp_lincoln_curve_decel",    {PERSISTENT, INT, "-320"}},// 最强减速度 -3.20 m/s²（存 cm/s²）
{"dp_lincoln_curve_exit_h",   {PERSISTENT, INT, "70"}},  // 退出滞回 0.70 s（存 1e-2 s）
```
转向速率可调（全速域等比，保护范围 80–120%）：
```cpp
{"dp_lincoln_steer_rate_up_pct",   {PERSISTENT, INT, "100"}}, // 入弯速率倍率
{"dp_lincoln_steer_rate_down_pct", {PERSISTENT, INT, "100"}}, // 回正速率倍率
```

## 2) UI（selfdrive/ui/layouts/settings/lincoln.py）
- 弯道限速区：
  - Dynamic Turn Speed Control → `dp_lincoln_curve_speed`
  - Curve speed debug log → `dp_lincoln_curve_log`
  - 旋钮：前瞻/曲率阈值/横向上限/最大减速度/退出滞回，绑定对应 Params，范围/默认见上，描述含调高/调低效果与推荐。
- 转向响应区（全速域等比）：
  - Steer rate (turn-in) → `dp_lincoln_steer_rate_up_pct`（建议 ±5% 小步调，调高更快但更易警告，调低更顺更慢）
  - Steer rate (return)  → `dp_lincoln_steer_rate_down_pct`（同上）
翻译文件 app_zh-CHS.po 已同步。

## 3) 控制入口（selfdrive/controls/plannerd.py）
仅 Ford/Lincoln 时传参：
```python
longitudinal_planner.update(sm, dp_flags,
  lincoln_curve_speed=lincoln_curve_speed_enabled(),
  lincoln_curve_log=lincoln_curve_log_enabled())
```
旧 DTSC 逻辑已移除。

## 4) 弯道算法细节（selfdrive/controls/lib/longitudinal_planner.py）
- 前瞻曲率：窗口默认 130 m，夹到 50–190 m；取最大曲率，截到 0.02 m⁻¹。
- 滞回：`k_enter` 默认 0.004（下限 0.002），`k_exit = 0.6 * k_enter`，退出延时默认 0.70 s。
- 限速：横向上限默认 1.20 m/s²，`v_limit = sqrt(a_lat / k_smooth)`；若已低于 `v_limit * 1.05` 则不动作。
- 减速：等效 `a = (v_f² - v_i²) / (2*d)`，封顶默认 -3.20 m/s²，收紧 `accel_clip[1]` 并写入 `mpc.params[:,1]`，确保 MPC 在关键距离前受限。
- 早退条件：模型无效、车速 <1 m/s、无前瞻点、曲率/距离过小、未达阈值、当前已低于限速等直接返回。

## 5) 调试日志（同文件 `_log_curve`）
- 路径：`/data/media/0/lincoln_curve_logs/` 按日 CSV（`curve_YYYYMMDD.log`）。
- 节流：常规 0.5s；横向失效/告警（alert 或 latActive=false）强制写一条（不节流）。
- 主要字段：时间戳、reason、v_ego、v_limit、k_smooth/k_max/k_model_max/k_model_std、k_enter、a_lat_limit、window_m、exit_h、decel_setting、distance、required_decel、accel_clip_max、mpc_a_max_min、curv_cmd/curv_current、accel_cmd/accel_actual、v_desired/v_plan0、yaw_rate/steer_angle/steer_torque、steer_pressed/gas_pressed/brake_pressed、steer_fault_temp/perm、sensors_invalid、ctrl/lat/long_active、alert_type/size/sound、cs_enabled/available/speed、alerts 文本。列顺序与代码一致，便于用表头解析。

## 6) 转向速率调节（opendbc_repo/opendbc/car/ford/values.py, carcontroller.py）
- 原始速率表：入弯 [0.00045 → 0.00010]，回正 [0.00045 → 0.00015]，断点 [5,25] m/s，曲率硬上限 0.02。
- `CarControllerParams.get_angle_limits()`：读取倍率，夹到 0.8–1.2，等比缩放 rate_up/rate_down，缓存 1s，硬上限不变。
- 车控使用动态限幅：`apply_std_steer_angle_limits(..., CarControllerParams.get_angle_limits())`。

## 7) 复刻步骤速览
1. 拷贝参数键（第 1 节）。
2. 拷贝 UI 控件（弯道限速 + 转向响应）。
3. Planner 入口传参（第 3 节）。
4. 弯道算法与日志（第 4、5 节）。
5. 转向速率动态限幅（第 6 节）。
6. 确保 `/data/media/0/lincoln_curve_logs/` 可写。默认值不变，未调节时行为与原版一致。

## 8) 安全与调参提示
- 曲率硬上限 ±0.02 m⁻¹，速率倍率 0.8–1.2；调高可能增加 “Turn Exceeds Steering Limit” 警告，建议每次仅 ±5% 小步路测。
- 弯道参数：调高前瞻/敏感度/减速度 → 更早更狠减速；调低 → 更流畅但介入更晚。
- 转向速率：调高入弯/回正更快但风险高；调低更稳但响应慢。
- 路测：一次只改一项，小步调整，出现警告/抖动即恢复默认。
