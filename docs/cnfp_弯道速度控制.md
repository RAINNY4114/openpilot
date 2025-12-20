# openpilotCP-cnfp（FrogPilot）弯道速度控制（MTSC/VTSC）

本文档整理 `F:\openpilotCP-cnfp\openpilotCP-cnfp` 中的“弯道速度控制”实现，目标是让其他 AI/开发者可以按此 1:1 复刻相同机制（包括参数开关、数据流、触发条件、核心公式）。

---

## 1. 功能总览

openpilotCP-cnfp（FrogPilot）把“弯道限速/弯道降速”拆成两套独立控制器，并在同一处合成最终的 `vCruise`：

1) **VTSC（Vision Turn Speed Controller，视觉弯道限速）**
- 数据来源：`modelV2`（视觉模型输出）
- 作用方式：直接计算一个“弯道允许目标速度 `vtsc_target`”，并把它作为候选 `vCruise` 上限（更低者更保守）。

2) **MTSC（Map Turn Speed Controller，地图弯道限速）**
- 数据来源：地图预先生成的“路径点目标速度序列” + 当前 GPS 位置
- 作用方式：在“到弯前的距离足够让车用 jerk/减速度模型降到目标速度”时，触发并锁定一个 `mtsc_target`，直到到达对应目标点才释放。

两者最终都不是直接“踩刹车/给制动”，而是通过 **降低 `vCruise`（纵向规划的期望速度）** 来让纵向 MPC 提前持续减速。

---

## 2. 代码位置（关键入口）

### 2.1 VTSC / MTSC 计算主入口
- `F:\openpilotCP-cnfp\openpilotCP-cnfp\selfdrive\frogpilot\controls\frogpilot_planner.py`
  - `FrogPilotPlanner.update(...)`
  - `FrogPilotPlanner.update_v_cruise(...)` ← 这里合成 MTSC/VTSC/限速等多种目标

### 2.2 MTSC 控制器本体
- `F:\openpilotCP-cnfp\openpilotCP-cnfp\selfdrive\frogpilot\controls\lib\map_turn_speed_controller.py`
  - `class MapTurnSpeedController`
  - `target_speed(v_ego, a_ego) -> float`

### 2.3 “路面曲率”计算（给 VTSC 用）
- `F:\openpilotCP-cnfp\openpilotCP-cnfp\selfdrive\frogpilot\controls\lib\frogpilot_functions.py`
  - `calculate_road_curvature(modelData, v_ego)`

### 2.4 参数开关读取（面板参数→toggles）
- `F:\openpilotCP-cnfp\openpilotCP-cnfp\selfdrive\frogpilot\controls\lib\frogpilot_variables.py`
  - `FrogPilotVariables.update_frogpilot_params(...)`

---

## 3. 数据流（SubMaster → FrogPilotPlanner → frogpilotPlan → longitudinal_planner）

### 3.1 FrogPilot 进程周期
`F:\openpilotCP-cnfp\openpilotCP-cnfp\selfdrive\frogpilot\frogpilot_process.py`
- 订阅：`carState`, `controlsState`, `modelV2`, `radarState`, `liveLocationKalman` 等
- 调用：`frogpilot_planner.update(...)`
- 发布：`frogpilotPlan`

### 3.2 纵向规划使用 frogpilotPlan
在该分支中，纵向规划会读取 `frogpilotPlan.vCruise` 等字段作为 MPC 目标/约束来源（代码中可见 `sm['frogpilotPlan']...` 的用法）。

---

## 4. 参数开关与用户可调项

这些参数在 `frogpilot_variables.py` 被读取并转换为 `frogpilot_toggles`：

### 4.1 VTSC（视觉弯道限速）
- `VisionTurnControl`（bool）：是否启用 VTSC
- `CurveSensitivity`（int，百分比）：曲率灵敏度缩放
  - `curve_sensitivity = CurveSensitivity / 100`
  - 在实现中用于：`adjusted_road_curvature = road_curvature * curve_sensitivity`
  - 直观效果：数值越大 → 认为弯更“急” → `vtsc_target` 更低（更保守）
- `TurnAggressiveness`（int，百分比）：弯道“横向舒适度”倍率
  - `turn_aggressiveness = TurnAggressiveness / 100`
  - 在实现中用于：`adjusted_target_lat_a = TARGET_LAT_A * turn_aggressiveness`
  - 直观效果：数值越大 → 允许更大横向加速度 → `vtsc_target` 更高（更积极）

### 4.2 MTSC（地图弯道限速）
- `MTSCEnabled`（bool）：是否启用 MTSC
- `MTSCCurvatureCheck`（bool）：是否启用“曲率检查”（注意：该分支实现存在明显的阈值问题，见“已知缺陷”）
- `MTSCAggressiveness`（int，百分比）：写入 `MapTargetLatA`（但在本代码树里未看到被其它模块使用）
  - `MapTargetLatA = 2 * (MTSCAggressiveness/100)` 写入 `/dev/shm/params`

---

## 5. VTSC（Vision Turn Speed Controller）算法细节

### 5.1 计算“路面曲率” road_curvature
位置：`frogpilot_functions.py: calculate_road_curvature(modelData, v_ego)`

核心逻辑：
1) 取模型输出：
   - `orientation_rate = abs(modelData.orientationRate.z)`（偏航角速度预测序列）
   - `velocity = modelData.velocity.x`（速度预测序列）
2) 计算前方预测的“最大横向需求”：
   - `max_pred_lat_acc = max(orientation_rate * velocity)`
   - 直观：`yawRate * v ≈ a_lat`
3) 换算成曲率量：
   - `road_curvature = max_pred_lat_acc / (v_ego ** 2)`

### 5.2 由曲率算目标速度
位置：`frogpilot_planner.py: update_v_cruise()`

触发条件：
- `vision_turn_controller == True`
- `v_ego > CRUISING_SPEED`（常量 5 m/s）
- `controlsState.enabled == True`

计算：
- `adjusted_road_curvature = road_curvature * curve_sensitivity`
- `adjusted_target_lat_a = TARGET_LAT_A * turn_aggressiveness`（常量 `TARGET_LAT_A=1.9 m/s^2`）
- `vtsc_target = sqrt(adjusted_target_lat_a / adjusted_road_curvature)`
- `vtsc_target = clip(vtsc_target, CRUISING_SPEED, v_cruise)`

特点（实现层面）：
- 只使用“最大值”，没有分位数/平滑/滞回；对模型尖峰更敏感（更容易保守或抖动）。

---

## 6. MTSC（Map Turn Speed Controller）算法细节

### 6.1 数据依赖（Params）
位置：`map_turn_speed_controller.py: MapTurnSpeedController.target_speed()`

它会从 `/dev/shm/params` 读取：
- `LastGPSPosition`：当前 GPS（json）
- `MapTargetVelocities`：地图路径点序列（json）

`LastGPSPosition` 示例（字段名来自读取逻辑）：
```json
{"latitude": 39.9, "longitude": 116.3}
```

`MapTargetVelocities` 期望格式（list[dict]）：
```json
[
  {"latitude": 39.9001, "longitude": 116.3002, "velocity": 18.0},
  {"latitude": 39.9002, "longitude": 116.3003, "velocity": 16.0}
]
```
其中 `velocity` 单位为 m/s（从控制器直接与 `v_ego` 比较可知）。

重要提示：在本代码树中只看到“读取 `MapTargetVelocities`”，未看到任何“生成/写入 `MapTargetVelocities`”的实现；如果没有外部模块持续写入，该控制器会直接返回 0 并等效不工作。

### 6.2 距离模型（刹到目标速度需要多远）
MTSC 使用 jerk/减速度约束来估算“从当前速度/加速度，降到 tv 需要的最短距离”，常量：
- `TARGET_JERK = -0.6`（m/s^3）
- `TARGET_ACCEL = -1.2`（m/s^2）
- `TARGET_OFFSET = 1.0`（秒，用于额外提前量）

步骤概括：
1) 用 Haversine 计算当前位置到每个目标点的距离，找到最近点 index（认为“自己在路径上的当前位置”）。
2) 只考虑“当前位置之后”的点（forward_points）。
3) 对每个 forward 点：
   - 若 `tv > v_ego` 则跳过（只做降速，不做加速）。
   - 用 jerk 把加速度从 `a_ego` 拉到 `TARGET_ACCEL`，并求出该阶段后的 `min_accel_v` 与距离。
   - 如果 `tv` 还没降到：继续以 `TARGET_ACCEL` 匀减速补足到 `tv`，并累加距离。
   - 得到总距离 `max_d` 后，若 `d < max_d + tv * TARGET_OFFSET`，认为“来得及刹到 tv”，该 tv 为有效候选。
4) 从所有候选中取 **最小 tv**（最保守）作为 `min_v`。

### 6.3 “锁定/滞回”（防止提前放开）
若上一帧已经锁定了一个更低的 `self.target_v`，则在到达该目标点之前不会因为新的计算结果变高就提前释放：它会检查上一次的 `(lat,lon,velocity)` 是否仍在 forward_points 中；若在，则继续返回旧的更低目标速度。

### 6.4 MTSC 触发门控
位置：`frogpilot_planner.py: update_v_cruise()`

触发条件：
- `MTSCEnabled == True`
- `v_ego > CRUISING_SPEED`
- `controlsState.enabled == True`
- `gps_check == True`（要求 `liveLocationKalman` 状态有效且 gpsOK）

并会进行：
- `self.mtsc_target = clip(mtsc.target_speed(...), CRUISING_SPEED, v_cruise)`

---

## 7. 多目标合成（最终 vCruise）
位置：`frogpilot_planner.py: update_v_cruise()`

该分支把以下目标放入列表并取最小值：
- `mtsc_target`（地图弯道）
- `slc_target / overridden_speed`（限速控制相关）
- `vtsc_target`（视觉弯道）

最终返回：
- `return min(filtered_targets)`

并写入发布：
- `frogpilotPlan.vCruise = self.v_cruise`
- `frogpilotPlan.adjustedCruise = min(mtsc_target, vtsc_target)`（用于 UI 展示/调试）

---

## 8. 已知缺陷/实现风险（按代码“原样复刻”必须知晓）

1) **MTSC 依赖 `MapTargetVelocities`，本代码树中未见生成链路**
- 只要没有外部模块往 `/dev/shm/params` 写入 `MapTargetVelocities`，MTSC 就基本不会起作用。

2) **`MTSCCurvatureCheck` 的阈值实现很可疑**
- 逻辑为：`if MTSCCurvatureCheck and road_curvature < 1.0 and not mtsc_active: mtsc_target = v_cruise`
- 由于 `road_curvature` 来自 `max_pred_lat_acc / v_ego^2`，典型值远小于 1.0（几乎恒成立），会导致 MTSC 在“尚未激活”的情况下被强制关闭，进而可能“永远无法首次激活”。
- 如果要 1:1 复刻，就必须保留这个行为；如果要可用性，需重新审视该阈值与维度含义。

3) **VTSC 使用 max 尖峰，缺少平滑/滞回**
- 容易出现：短暂尖峰 → `vtsc_target` 立刻变很低 → 车辆“过度保守/抖动/长时间低速”。

---

## 9. 复刻检查清单（快速自检）

要确认“复刻成功且可触发”，最少需要：
- VTSC：能读到 `modelV2.orientationRate.z`、`modelV2.velocity.x`，并在弯道时 `frogpilotPlan.adjustedCruise` 明显低于 `controlsState.vCruise`。
- MTSC：`LastGPSPosition` 持续更新 + `MapTargetVelocities` 非空且点序列覆盖当前位置之后路段；否则 MTSC 将始终返回 0 或等效不触发。

