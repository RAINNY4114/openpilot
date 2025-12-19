# 林肯面板：HUD 显示增强（dp_lincoln_hud_enhanced）

本功能是**纯 UI 可视化增强**：只改变 HUD 的绘制效果，不改变任何控制/规划逻辑。

## 1. 用户开关

- **面板位置**：`Lincoln` → `### HUD & Visualization ###` → `HUD drawing enhancements`
- **Param**：`dp_lincoln_hud_enhanced`（`bool`，默认 `0`）

相关代码：

- `common/params_keys.h`：加入 `dp_lincoln_hud_enhanced`
- `selfdrive/ui/layouts/settings/lincoln.py`：面板开关写入 Params
- `selfdrive/ui/ui_state.py`：缓存读取 `ui_state.dp_lincoln_hud_enhanced`

## 2. 启用后的绘制改动（与 CP 无关、全新设计）

### 2.1 转向灯 / 盲点提示（边框 + 侧向“风险区域”）

数据来源：`carState.leftBlinker/rightBlinker`、`carState.leftBlindspot/rightBlindspot`

启用增强后，左右两侧使用一致的颜色/闪烁规则：

- **打灯**：黄闪
- **盲点**：红闪
- **打灯 + 盲点**：红快闪

实现位置：

- `selfdrive/ui/onroad/augmented_road_view.py`
  - `_update_dp_indicator_side_state()`：颜色/闪烁逻辑
  - `_draw_hud_enhancements()`：在画面左右绘制红/黄的半透明**渐变带**“风险区域”（随闪烁同步）

### 2.2 车道线更稳、更清晰（粗细稳定、主车道更突出）

数据来源：`modelV2.laneLines`、`modelV2.laneLineProbs`、`modelV2.roadEdges`、`modelV2.roadEdgeStds`

启用增强后：

- **车道线粗细不再随 `laneLineProb` 大幅变细**（降低“时粗时细”的观感问题）
- **4 条车道线统一使用绿色体系**（主车道两条边界线更亮、更显眼；外侧线更淡）
- **路沿（roadEdges）略加粗**（更容易分辨道路边界）

实现位置：

- `selfdrive/ui/onroad/model_renderer.py`
  - `_update_model()`：调整 laneLines/roadEdges 的绘制宽度策略
  - `_draw_lane_lines()`：4 条车道线统一绿色体系（engaged 时主车道更亮）

### 2.3 前车信息更简洁（距离 + 前车速度）

数据来源：`radarState.leadOne/leadTwo`

启用增强后，即使不打开其它调试/显示项，也会在前车 chevron 下方显示**距离**（m/ft），避免 UI 太“空”但又不堆数字。

同时显示**前车速度**（km/h / mph），并且不显示 TTC 等额外数字。

实现位置：

- `selfdrive/ui/onroad/model_renderer.py` → `_draw_lead_indicator()`

### 2.3.1 前车黄色占位框（自动缩放 + 滤波跟踪）

这个“框”是**基于雷达/融合 lead 的占位框**（UI 提示用），不是相机目标检测那种“像素级贴边的真实轮廓框”。

数据来源：

- 优先 `radarState.leadOne`：`dRel/yRel/status`（锚定前车相对位置）
- 回退 `modelV2.leadsV3[0]`：当雷达 lead 缺失/很短暂时，使用 `prob/x/y/v`（UI 兜底，不保证和雷达一样稳定）
- `modelV2.position.z`：在 `dRel` 处取路面高度，用于坡度/俯仰下的投影一致性

实现思路（参考 carrot2 的做法）：

- 在车体坐标系假设一组横向边界：`y = -yRel ± halfWidth`（近距离更窄、远距离略宽，避免框住相邻车辆）
- 将左右边界点投影到屏幕，得到 `width_px = |xR - xL|`，从而实现**随距离自动缩放**
- 对 `x/y/w/h` 做指数平滑滤波：远距离更稳、近距离更灵敏，减少跳动同时降低滞后
- 只画黄色描边、框内全透明

显示/强度规则（默认值）：

- 距离触发：`dRel <= 130m` 开始逐渐显示，`<= 30m` 达到最强
- 追近触发：当 `vRel < 0` 时，按 TTC 计算；`TTC <= 5s` 开始显示，`<= 2.5s` 达到最强

尺寸/滤波策略（默认值）：

- 宽度：近距离 `halfWidth≈0.90m` → 远距离 `halfWidth≈1.10m`
- 高度比例：近距离 `≈0.70` → 远距离 `≈0.62`
- 滤波：远距离更稳（平滑系数 `≈0.85`），近距离更灵敏（平滑系数 `≈0.55`）

实现位置：

- `selfdrive/ui/onroad/model_renderer.py`
  - `_update_lead_box()`：计算投影框并滤波
  - `_draw_lead_box()`：绘制黄色描边框

### 2.4 刹车提示（整框变色）

数据来源优先级：

1. `carState.aEgo`（实际纵向加速度，<0 表示减速；可覆盖原车 ACC 自己刹车的场景）
2. `carOutput.actuatorsOutput.brake`（openpilot 的制动指令，0~1，用作补充）
3. `carState.brakePressed`（人工踩刹车时用于“立即点亮/最低亮度”保障）

启用增强后，HUD **整圈边框**会随制动强度“渐变变红”：

- 常规减速：边框颜色随**减速度/制动强度**从当前状态色逐渐过渡到红色（带滤波，避免闪烁）。
- 急刹车：当检测到**大减速度/大制动**（或模型 `hardBrakePredicted`）时，边框改为**红色快闪**作为强提醒。
- 人工踩刹车：`brakePressed` 会让边框**立即开始变红**（避免“踩了但不亮/亮得太慢”）。

实现位置：

- `selfdrive/ui/onroad/augmented_road_view.py` → `_draw_border()`（边框颜色动态更新）

## 3. 重要说明

- 本功能只影响**标准 UI（MainLayout / `selfdrive/ui/onroad/*`）** 的绘制；如需同步到 `mici` UI，需要单独实现。
- 新增 Param 需要重新编译 `common/params_pyx`（正常 `scons` 构建会自动处理），否则会出现 `UnknownKeyName: b'dp_lincoln_hud_enhanced'`。
