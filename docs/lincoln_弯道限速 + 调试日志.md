# Lincoln 弯道限速 + 调试日志 + 可调参数（1:1 复刻版）

目标：在其他分支完整复刻弯道限速、调试日志，保留少量核心可调项，默认行为与原版一致。

## 可调参数（仅保留这 4 项 + 日志）
- 前瞻距离 window：默认 130 m（50–190），越大越早减速。
- 进入曲率阈值 k_enter：默认 0.004（0.002–0.010），越低越敏感。
- 最大预刹力度 decel cap：默认 -3.20 m/s²（-3.5 至 -2.0），越负减速越猛。
- 日志开关：默认关，仅用于调试。
- 已移除/固定：退出滞回时间、曲率退出阈值、转向速率倍率、舒适横向上限（固定 1.0 m/s²）。

## 控制入口
`selfdrive/controls/plannerd.py` 传入：
```python
longitudinal_planner.update(sm, dp_flags,
  lincoln_curve_speed=lincoln_curve_speed_enabled(),
  lincoln_curve_log=lincoln_curve_log_enabled())
```

## 弯道算法要点（`selfdrive/controls/lib/longitudinal_planner.py`）
- 前瞻曲率窗口：默认 130 m，限制 50–190 m，曲率截到 0.02 m⁻¹。
- 快速触发：窗口内任一点 ≥ k_enter 即激活；平滑 alpha=0.6，退出仍用 k_exit=0.6*k_enter + 固定退出 0.70 s。
- 限速计算：`v_limit = sqrt(a_lat / k_smooth)`；若 v_ego ≤ 1.05*v_limit 则不动作。
- 预刹距离：`d_use = max(trigger_distance, v_ego * 1.5 s)`。
- 减速上限：等效 `a = (v_f² - v_i²)/(2*d_use)`，高速可到 -3.5~-4.0 m/s²；写入 `accel_clip` 和 `mpc.params[:,1]`，确保 MPC 前瞻受限。
- 早退条件：模型无效、车速 <1 m/s、无前瞻点、曲率/距离太小、未达阈值或已低于限速等直接返回。

## 调试日志（同文件 `_log_curve`）
- 路径：`/data/media/0/lincoln_curve_logs/`，按日 CSV（`curve_YYYYMMDD.log`）。
- 节流：常规 0.5 s；若出现横向失效/报警强制写一条。
- 首行写表头+单位，主要字段：
  - 基本：timestamp, reason, v_ego, v_limit
  - 曲率：k_smooth, k_max, k_model_max, k_model_std, k_enter, k_exit, k_p80
  - 触发：distance_to_peak, trigger_distance/curv/type, bend_cum/bend_thresh, d_use
  - 减速：a_lat_limit, decel_setting, required_decel, accel_clip_max, mpc_a_max_min
  - 轨迹/指令：curv_cmd/current, accel_cmd/actual, v_desired, v_plan0
  - 车身/状态：yaw_rate, steer_angle/torque, 按键/传感器状态，ctrl/lat/long_active，alert_type/size/sound，巡航状态与 alerts 文本。

## 复刻步骤速览
1) 拷贝参数键（仅上方 5 个）到 `common/params_keys.h`。
2) 拷贝 UI 控件（仅弯道 4 个滑条 + 日志开关）到 `selfdrive/ui/layouts/settings/lincoln.py`。
3) 拷贝 planner 入口调用。
4) 拷贝弯道算法与日志实现。
5) 确保 `/data/media/0/lincoln_curve_logs/` 可写。

## 使用提示
- 增大前瞻距离/降低 k_enter：更早减速，弯中更稳，但可能直路略早刹。
- 提高 decel cap：弯前更狠、更早减速（注意舒适性）。
- 一次只改一项，小步路测；出现警告/抖动即可恢复默认。
