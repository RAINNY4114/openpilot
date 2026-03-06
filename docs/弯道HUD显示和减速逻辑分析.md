# 弯道 HUD 显示和减速逻辑分析（更新版）

## 1. 目标与范围

本文件用于对照：
- 当前分支：`E:\openpilot`
- 参考分支：`Z:\openpilot-fp-focus-lite\openpilot-fp-focus-lite`

目标：
1. 明确 FP 体感更好的真实原因。
2. 校正“更早发现/立即减速/HUD自动同步”等容易误解的说法。
3. 形成可直接执行的优化准备清单。

---

## 2. 已确认结论（本次定稿）

1. 当前分支减速偏慢的主因是：没有 FP 的 `taco_tune` 轨迹裁剪层。
2. 当前分支是“压 `v_cruise` + MPC追踪”，不是“直接压模型 `v[]` 轨迹”。
3. `taco_tune` 带来的是“更早响应”，不是“更早发现弯道”。
4. “0秒延迟”不成立；只能说会更早开始减速。
5. 只加 `taco_tune` 不能保证 HUD 同步；`curveSpeedSource` / `vTargetDEPRECATED` 仍需联动处理。
6. 截至当前代码状态：阶段A+B代码已落地（`parse_model` 已接入 taco 裁剪，HUD source/target 已联动），当前重点转为实车验证与参数校准。

---

## 3. 关键代码链路对照（逐行）

### 3.1 当前分支：纵向减速链路

文件：`E:\openpilot\selfdrive\controls\lib\longitudinal_planner.py`

1. `parse_model()` 无 taco 裁剪：
- `L262-L291` 仅插值 `x/v/a/j`，未做基于曲率的 `v[]` 限制。

2. 曲率检测与目标速度：
- `L518-L528`：`road_curvature_detected`（检测阈值逻辑）
- `L559-L560`：`map_speed`
- `L565-L568`：`vtsc_speed`
- `L571-L573`：只压 `v_cruise`，不改 `v[]`

3. MPC 输入：
- `L721-L723`：`self.mpc.update(..., v_cruise, x, v, a, j, ...)`
- 这里 `v` 仍是原始模型速度轨迹。

4. 减速度约束强化（仍非 taco）：
- `L709-L719`：`cruise_min_accel`
- `L781-L787`：`accel_clip` 下限变化率

结论：当前分支属于“目标追踪型减速”，体感通常偏缓。

当前状态确认（关键）：
1. `parse_model` 已扩展为 taco 可选裁剪路径。
2. `update()` 已按 `curve_speed_control` 传入 taco 开关并接收裁剪目标。
3. 当前已进入“taco + 旧map/vision并行”的过渡架构。

### 3.2 当前分支：HUD 弯道图标链路

文件：`E:\openpilot\selfdrive\ui\onroad\hud_renderer.py`

1. 显示开关：
- `L402-L405`：`CurveSpeedControl` + `ShowCSCStatus`

2. HUD 输入源：
- `L442-L449`：读取 `curveSpeedSource`
- `L467-L474`：优先 `vTargetDEPRECATED`，否则回退到 `speeds` 最小值

3. 显示条件：
- `L459`：`curveSpeedSource in (1, 2)` 才进入主逻辑
- `L477`：`v_target_disp < set_speed` 才显示

结论：HUD 并非直接读取“是否 taco 裁剪生效”，而是依赖 source/target 产出状态。

补充：
1. 只要 `curveSpeedSource` 不在 `(1,2)`，HUD 侧不会进入弯道控件主显示逻辑。
2. 当前已把 HUD 发布源改为 map/vision/taco 三方候选取最小目标，避免“减速了但HUD不显示”的主要链路问题。

### 3.3 参考 FP 分支：taco 裁剪链路

文件：`Z:\openpilot-fp-focus-lite\openpilot-fp-focus-lite\selfdrive\controls\lib\longitudinal_planner.py`

1. `parse_model(..., taco_tune)`：
- `L140-L144`：直接计算曲率上限速度并执行 `v = min(max_v, v)`

2. MPC 使用裁剪后 `v`：
- `L225-L226`

结论：FP（TacoTune开启时）把降速“前移”到模型轨迹层，MPC更早开始规划减速。

### 3.4 关于“阈值”的边界说明

1. FP 的 `taco_tune` 裁剪是每帧执行。
2. 但 FP 的 VTSC/MTSC 仍有 `road_curvature_detected` 门控，不是全链路“完全无阈值”。

文件：
- `frogpilot/controls/frogpilot_planner.py:L109`
- `frogpilot/controls/lib/frogpilot_vcruise.py:L60-L71, L92-L96`

---

## 4. 对 4 个常见问题的最终校正

### 4.1 能“更早发现弯道”吗？

结论：`❌ 不准确`

准确表述：
1. `taco_tune` 不改变模型感知输入，不是“更早发现”。
2. 它是“更早响应”，因为把降速动作前移到 `parse_model` 的 `v[]` 裁剪。
3. 极小曲率时可能几乎不降速（`k` 很小时 `max_v` 接近原 `v`）。

### 4.2 能“马上减速（0秒）”吗？

结论：`⚠️ 过度承诺`

准确表述：
1. 会更早开始减速，但不是 0 秒。
2. 仍受 `DT_MDL`、执行器延迟、加减速/jerk 约束影响。
3. 参考约束位置：`longitudinal_planner.py:L709-L723`

### 4.3 HUD 目标速度会自动同步吗？

结论：`❌ 只加 taco 不够`

准确表述：
1. HUD 依赖 `curveSpeedSource` 与 `vTargetDEPRECATED`，并要求 `source in (1,2)`。
2. 如果只加 `taco_tune`，但不联动 source/target 发布，可能出现：
  - 车在减速但 HUD 不显示；
  - HUD 显示与实际减速不同步。
3. 需补 planner 侧发布逻辑，确保 HUD 的 source/target 反映当前实际控制。

### 4.4 能“固定 1-2 秒达标”吗？

结论：`❌ 不能保证固定时长`

准确表述：
1. 一般会比当前更果断。
2. 但达标时间取决于车速、曲率、约束和执行器特性。

---

## 5. `A_CRUISE_MAX_VALS_FORD` 的作用边界

文件：`E:\openpilot\selfdrive\controls\lib\longitudinal_planner.py`

- `L29-L30` 与 `L77-L80` 主要约束正向加速上限（舒适性、抑制高转速）。
- 它会影响弯道前后“再加速”体感，但不是提前减速差异的决定性因素。

---

## 6. 优化实施准备（建议顺序）

### 阶段 A：先补“轨迹裁剪层”

目标：把“压 `v_cruise`”升级为“压 `v[]` 轨迹 + MPC”。

建议：
1. 扩展当前 `parse_model` 签名，接入 `v_ego` 与开关。
2. 引入 FP 等价的 taco 裁剪公式。
3. 保留现有 map/vision 功能，不做一次性大删改。

### 阶段 B：补 HUD 同步链路

目标：让图标/目标速度与实际减速来源一致。

建议：
1. 明确 `curveSpeedSource` 在 taco 生效时的标注策略（建议归类为 vision source）。
2. 让 `vTargetDEPRECATED` 与当前有效限速目标一致（优先采用实际约束目标）。
3. 保持与 `longitudinalPlan.speeds` 的一致性，避免“显示值与执行值”分叉。

阶段约束（必须遵守）：
1. 阶段A与阶段B必须成对落地，不建议只上线A。
2. 若只上线A，必须接受“减速先变好但HUD可能不同步”的过渡风险。
3. 对外验收标准应以“减速行为 + HUD显示一致性”双指标共同通过为准。

### 阶段 C：再收敛旧阈值逻辑

目标：避免双重限速冲突、避免过强限速。

建议：
1. 先保留阈值逻辑观察。
2. 路测后再决定是否弱化或删除部分 vision 阈值路径。

---

## 7. 验证清单（优化前先定义）

1. 响应时序：
- 记录“HUD图标出现时刻”与“`aTarget`开始明显为负时刻”的差值。

2. HUD一致性：
- 检查 `curveSpeedSource`、`vTargetDEPRECATED`、`longitudinalPlan.speeds` 最小值是否一致。

3. 体感与安全：
- 小曲率长弯是否仍有有效预减速。
- 大曲率急弯是否避免“突然大减速”或过冲。

4. 舒适性：
- 检查 jerk 峰值与连续弯道中的速度波动。

---

## 8. 一句话总结

当前分支慢在“只压巡航目标，不压模型轨迹”；优化应先补 `taco` 轨迹裁剪，再补 HUD source/target 同步，最后再收敛旧阈值逻辑。

---

## 9. 当前状态与验证风险（新增）

### 9.1 当前状态（截至本文更新）

1. 当前代码已接入 `taco` 轨迹裁剪，并接入主纵向流程。
2. 当前HUD发布已联动 `curveSpeedSource` + `vTargetDEPRECATED`，并保留 `speeds` 回退。
3. 当前进入“代码已落地、待实车验证”阶段。
4. 已增加边界收敛措施：
- 仅在 `taco` 对轨迹产生有效裁剪（裁剪量超过最小阈值）时，才让其参与 HUD 目标候选。
- 对 map/vision 近值候选增加来源滞回，减少 `curveSpeedSource` 在边界场景频繁切换。
- `taco_hud_active` 使用开/关双阈值滞回（避免长弯边界反复亮灭）。
- `curveSpeedSource` 使用最小保持时间（避免短时间来回切换来源标签）。

### 9.2 当前剩余风险（即使A+B已落地）

1. map/vision 与 taco 并行时，目标速度来源切换可能在边界场景出现抖动。
2. 低曲率长弯场景可能出现“轻微减速但显示不稳定”的体验波动（取决于阈值与候选目标差值）。
3. 实车需重点核验 HUD 目标值与 `longitudinalPlan.speeds` 最小值的一致性。
4. 目前滞回阈值与最小裁剪阈值为工程初值，仍需按实车日志调参。
5. 当前工程初值：
- `taco` 裁剪量阈值：开 `0.20 m/s`，关 `0.10 m/s`
- `taco` 目标差阈值：开 `0.10 m/s`，关 `0.03 m/s`
- `curveSpeedSource` 最小保持时间：`0.6 s`

### 9.3 最重要提醒

只加 taco 不够。必须同步处理 HUD 发布逻辑（阶段B）。本次代码已同步落地，但仍需实车验证确认“减速行为 + HUD显示”一致通过。
