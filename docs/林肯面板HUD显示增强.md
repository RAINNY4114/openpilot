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
  - `_draw_hud_enhancements()`：在画面左右绘制半透明的侧向“风险区域”（随闪烁同步）

### 2.2 车道线更稳、更清晰（粗细稳定、主车道更突出）

数据来源：`modelV2.laneLines`、`modelV2.laneLineProbs`、`modelV2.roadEdges`、`modelV2.roadEdgeStds`

启用增强后：

- **车道线粗细不再随 `laneLineProb` 大幅变细**（降低“时粗时细”的观感问题）
- **主车道两条边界线（laneLines[1]/laneLines[2]）在 engaged 时使用更“高级”的绿色**（更符合驾驶状态直觉）
- **路沿（roadEdges）略加粗**（更容易分辨道路边界）

实现位置：

- `selfdrive/ui/onroad/model_renderer.py`
  - `_update_model()`：调整 laneLines/roadEdges 的绘制宽度策略
  - `_draw_lane_lines()`：主车道边界在 engaged 时改为绿色

### 2.3 前车信息更简洁（只显示距离）

数据来源：`radarState.leadOne/leadTwo`

启用增强后，即使不打开其它调试/显示项，也会在前车 chevron 下方显示**距离**（m/ft），避免 UI 太“空”但又不堆数字。

实现位置：

- `selfdrive/ui/onroad/model_renderer.py` → `_draw_lead_indicator()`

### 2.3.1 前车黄色占位框（自动缩放 + 滤波跟踪）

这个“框”是**基于雷达/融合 lead 的占位框**（UI 提示用），不是相机目标检测那种“像素级贴边的真实轮廓框”。

数据来源：

- `radarState.leadOne`：`dRel/yRel/status`（锚定前车相对位置）
- `modelV2.position.z`：在 `dRel` 处取路面高度，用于坡度/俯仰下的投影一致性

实现思路（参考 carrot2 的做法）：

- 在车体坐标系假设一组横向边界：`y = -yRel ± halfWidth`（近距离更窄、远距离略宽，避免框住相邻车辆）
- 将左右边界点投影到屏幕，得到 `width_px = |xR - xL|`，从而实现**随距离自动缩放**
- 用 `alpha=0.85` 的指数平滑对 `x/y/w/h` 滤波，减少跳动
- 只画黄色描边、框内全透明

显示/强度规则（默认值）：

- 距离触发：`leadOne.dRel <= 70m` 开始逐渐显示，`<= 20m` 达到最强
- 追近触发：当 `vRel < 0` 时，按 TTC 计算；`TTC <= 5s` 开始显示，`<= 2.5s` 达到最强

尺寸/滤波策略（默认值）：

- 宽度：近距离 `halfWidth≈0.90m` → 远距离 `halfWidth≈1.10m`
- 高度比例：近距离 `≈0.70` → 远距离 `≈0.62`
- 滤波：远距离更稳（更大平滑系数），近距离更灵敏（更小平滑系数）

实现位置：

- `selfdrive/ui/onroad/model_renderer.py`
  - `_update_lead_box()`：计算投影框并滤波
  - `_draw_lead_box()`：绘制黄色描边框

### 2.4 刹车提示条（随制动强度逐渐变红）

数据来源优先级：

1. `carState.aEgo`（实际纵向加速度，<0 表示减速；可覆盖原车 ACC 自己刹车的场景）
2. `carOutput.actuatorsOutput.brake`（openpilot 的制动指令，0~1，用作补充）
3. `carState.brakePressed`（人工踩刹车时用于“立即点亮/最低亮度”保障）

启用增强后，在 HUD 底部绘制一条细条：

- 常规减速：颜色随**减速度/制动强度**从绿→红渐变（带滤波，避免闪烁）。
- 急刹车：当检测到**大减速度/大制动**（或模型 `hardBrakePredicted`）时，改为**红色快闪**作为强提醒。
- 人工踩刹车：`brakePressed` 会让条形**立即点亮**（避免“踩了但不亮/亮得太慢”）。

实现位置：

- `selfdrive/ui/onroad/augmented_road_view.py` → `_draw_hud_enhanced_top_overlays()`（**最顶层绘制**，贴最底部，覆盖底边框区域）

## 3. 重要说明

- 本功能只影响**标准 UI（MainLayout / `selfdrive/ui/onroad/*`）** 的绘制；如需同步到 `mici` UI，需要单独实现。
- 新增 Param 需要重新编译 `common/params_pyx`（正常 `scons` 构建会自动处理），否则会出现 `UnknownKeyName: b'dp_lincoln_hud_enhanced'`。
