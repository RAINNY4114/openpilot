# Lincoln 面板「弯道限速 + 调试日志」复刻指南

> 目标：在其它分支 1:1 还原弯道限速和调试日志功能。按文件路径逐项移植即可完整复制。

---

### 1. 参数键

**文件**：`common/params_keys.h`

新增 2 个持久化参数：

```cpp
{"dp_lincoln_curve_speed", {PERSISTENT, BOOL, "0"}},
{"dp_lincoln_curve_log",   {PERSISTENT, BOOL, "0"}},
```

分别用于弯道限速开关、调试日志开关。

---

### 2. Lincoln 设置面板

**文件**：`selfdrive/ui/layouts/settings/lincoln.py`

新增 2 个控件：

```python
toggle_item(
  title=lambda: tr("Dynamic Turn Speed Control"),
  description=lambda: tr("Slow down automatically in curves for Lincoln/Ford (model-based, comfort tuned)."),
  initial_state=self._params.get_bool("dp_lincoln_curve_speed"),
  callback=lambda val: self._params.put_bool("dp_lincoln_curve_speed", val),
),
toggle_item(
  title=lambda: tr("Curve speed debug log"),
  description=lambda: tr("Write curve speed details (model curvature, speed limit, decel) to /data/media/0/lincoln_curve_logs/."),
  initial_state=self._params.get_bool("dp_lincoln_curve_log"),
  callback=lambda val: self._params.put_bool("dp_lincoln_curve_log", val),
),
```

---

### 3. 控制逻辑入口

**文件**：`selfdrive/controls/plannerd.py`

- 仅 Ford/Lincoln 品牌时读取上述参数并传入 `LongitudinalPlanner.update(sm, dp_flags, lincoln_curve_speed=..., lincoln_curve_log=...)`。
- DPFlags 仅包含 ACM/AEM，已移除旧 DTSC 逻辑。

---

### 4. 核心算法与日志

**文件**：`selfdrive/controls/lib/longitudinal_planner.py`

#### 4.1 弯道限速 `_apply_lincoln_curve_speed`

- 前瞻：取模型前方 ≤80m 的曲率，截断到信号上限 0.02 m⁻¹。
- 平滑/滞回：曲率平滑 + 进入/退出滞回（k_enter≈0.006，k_exit≈0.0036，退出延时 0.5s）。
- 限速：目标侧向上限约 1.44 m/s²（1.6×0.9），`v_limit = sqrt(a_lat_limit / k_smooth)`。
- 减速度：等效 `a = (v_f² - v_i²)/(2d)`，封顶不超过 -3.0 m/s²，收紧全局 `accel_clip[1]` 并逐时间步写入 `mpc.params[:,1]`。
- 早退：模型无效、车速<1 m/s、无前瞻点、曲率/距离过小、未达触发阈值、当前已低于限速都会提前返回。

#### 4.2 调试日志 `_log_curve`

- 路径：`/data/media/0/lincoln_curve_logs/`，按日命名 `curve_YYYYMMDD.log`。
- 频率：常规 0.5s 节流；横向关闭/告警事件强制写一条（不节流）。
- 字段（常规/事件）：
  - `reason`（active/alert_or_lat_off/早退原因）
  - `v_ego`、`v_limit`、`k_smooth/k_max`、`dist`
  - 模型曲率信息：`k_model_max`（原始未截断曲率最大值）、`k_model_std`（曲率标准差/置信度近似）
  - `a_req`、`accel_clip_max`、`mpc_a_max_min`
  - `accel_cmd/accel_actual`、`curv_cmd/curv_current=-yawRate/v`
  - `steer_torque`、`steer_fault_temp/perm`、`sensors_invalid`
  - `steer_pressed/gas_pressed/brake_pressed`、`lat_active`、`alerts`
  - 控制/巡航状态：`ctrl_active/ctrl_long_active/ctrl_lat_active`、`ctrl_alert_type/size/sound`、`cs_enabled/cs_available/cs_speed`
  - `v_desired`、`v_plan0`（规划首元素）
- 输出格式：CSV，每行逗号分隔；第一列为时间戳，第二列为 reason，后续字段顺序与上述列表一致，便于直接用表头解析。

---

### 5. 复刻步骤

1. 参数：在 `common/params_keys.h` 添加 `dp_lincoln_curve_speed`、`dp_lincoln_curve_log`。
2. UI：在 Lincoln 面板加入对应 toggles。
3. Planner：品牌判定后传参到 `LongitudinalPlanner.update`。
4. 控制：复制 `_apply_lincoln_curve_speed` 与 `_log_curve`（包含前瞻、滞回、MPC 约束、日志字段）。
5. 日志：确保 `/data/media/0/lincoln_curve_logs/` 可写；如需可自定义大小/轮转策略。

---

### 6. 已知物理/安全限制

- Ford 非 CAN FD 曲率信号上限 ±0.02 m⁻¹、EPS 侧向能力 ~2 m/s²，弯前需降速，否则易转向不足/Take Control。
- 方向盘接管阈值 1 Nm，EPS 质量码/故障会导致横向退出；日志已记录相关状态便于排查。
