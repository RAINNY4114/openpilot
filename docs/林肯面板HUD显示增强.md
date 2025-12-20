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

### 2.1 转向灯 / 盲点提示（黄灯渐变带 + 红色盲点墙）

数据来源：`carState.leftBlinker/rightBlinker`、`carState.leftBlindspot/rightBlindspot`

启用增强后，左右两侧使用一致的提示规则：

- **打灯**：对应侧绘制**黄色渐变带**（按闪烁节奏闪烁）
- **盲点**：对应侧绘制**红色盲点墙**（覆盖相邻车道，红色渐变，常亮；对齐 FrogPilot 的表现）
- **打灯 + 盲点**：仍以**盲点墙优先**（不再叠加黄色渐变带，避免画面过于复杂）

实现位置：

- `selfdrive/ui/onroad/augmented_road_view.py`
  - `_update_dp_indicator_side_state()`：颜色/闪烁逻辑
  - `_draw_hud_enhancements()`：按 blinker/blindspot 状态选择绘制“黄色渐变带”或“红色盲点墙”
  - `_draw_hud_fp_blindspot_wall()`：FrogPilot 风格盲点墙（红色渐变填充相邻车道）
  - `_fp_get_adjacent_lane_polygon()`：用 `modelV2.laneLines/roadEdges` 合成相邻车道多边形（本仓库标准模型只有 4 条 laneLines，因此用左右两条边界线的**几何中线**近似 FrogPilot 的 `laneLines[4/5]` 相邻车道中心线）

### 2.1.1 盲点墙（对齐 FrogPilot 的实现细节）

**FrogPilot 参考实现**

- 盲点墙绘制：`frogpilot/ui/qt/onroad/frogpilot_annotated_camera.cc` → `paintBlindSpotPath()`
  - 使用 `track_adjacent_vertices[0/1]` 直接填充多边形
  - 颜色：红色渐变（底部 α=0.6 → 中部 α=0.4 → 顶部 α=0.2）
- `track_adjacent_vertices` 生成：`selfdrive/ui/ui.cc` → `update_line_data(...)`
  - 左：`update_line_data(..., lane_lines[4], lane_width_left/2, ..., &track_adjacent_vertices[0], max_idx, allow_invert=false)`
  - 右：`update_line_data(..., lane_lines[5], lane_width_right/2, ..., &track_adjacent_vertices[1], max_idx, allow_invert=false)`
  - 绘制长度：`MIN_DRAW_DISTANCE=10m` ~ `MAX_DRAW_DISTANCE=100m`，并且会按 `radarState.leadOne.dRel` 提前缩短（与主轨迹一致）

**本仓库标准 UI 的对齐方式**

- 由于本仓库 `modelV2.laneLines` 只有 4 条（0/1/2/3），没有 FrogPilot 的 `laneLines[4/5]` “相邻车道中心线”，因此采用：
  - 左侧相邻车道中心线 ≈ `laneLines[0]` 与 `laneLines[1]` 的几何中线
  - 右侧相邻车道中心线 ≈ `laneLines[3]` 与 `laneLines[2]` 的几何中线
- 相邻车道宽度：按 FrogPilot 同款 `calculate_lane_width()` 思路（并做“roadEdge 更近则置 0”门控）
- 多边形生成：复用 `selfdrive/ui/onroad/model_renderer.py` 的投影/裁剪逻辑 `_map_line_to_polygon(..., y_off=lane_width/2, allow_invert=False)`
- 绘制长度：同样使用 10~100m + leadOne 缩短，确保“墙的长度/跟随前车缩短”与 FrogPilot 一致

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

### 2.5 弯道限速提示（1:1 复刻 FrogPilot 的曲线提示控件）

> 说明：这是“弯道限速（`dp_lincoln_curve_speed`）”的 HUD 可视化提示，**不依赖** `dp_lincoln_hud_enhanced` 开关（即：就算不启用 HUD 显示增强，只要弯道限速在工作，这个提示也会出现）。

**显示内容**
- 弯道图标（根据弯道方向左右镜像）
- 弯道目标速度（`km/h`/`mph`）

**显示条件（与 FrogPilot 一致：只在“正在控速”时出现）**
- `carParams.brand == "ford"`
- 已设定巡航速度（左上角 `MAX` 有效数值）
- `dp_lincoln_curve_speed == 1`
- 预测弯道目标速度 `< 当前设定巡航速度`

**数据来源 / 计算**
- 使用 `modelV2.position.x / modelV2.velocity.x / modelV2.orientationRate.z` 计算前方窗口内曲率峰值并平滑
- 目标速度：`v_limit = sqrt(a_lat / k_smooth)`（`a_lat` 固定 1.0 m/s²）
- 参数沿用弯道限速设置：`dp_lincoln_curve_window_m`、`dp_lincoln_curve_k_enter`

**资源与代码位置**
- 图标 PNG：`selfdrive/assets/icons/curve_speed.png`
- 计算与绘制：`selfdrive/ui/onroad/hud_renderer.py`
  - `_update_curve_speed_widget()`：生成目标速度文本 + 镜像方向
  - `_draw_curve_speed_control()`：绘制图标与蓝色速度条

**HUD 位置（1:1 对齐 FrogPilot 的相对布局）**
- 参考 FrogPilot：`curveSpeedRect.x = setSpeedRect.right() + UI_BORDER_SIZE`
  - FrogPilot 参考文件：`frogpilot/ui/qt/onroad/frogpilot_annotated_camera.cc`（`paintCurveSpeedControl`）
  - FrogPilot 图标资源：`frogpilot/assets/other_images/curve_speed.png`
- 本仓库实现：
  - 图标左上角：`x = set_speed_rect.x + set_speed_rect.width + UI_CONFIG.border_size`，`y = set_speed_rect.y`
  - 容器尺寸：`widget_size = UI_CONFIG.button_size * 1.25`
  - 蓝色速度条：`y + widget_size + 10`，高度 `100`

### 2.6 HUD 底部性能条（道路名称/逆地理）

> 说明：这是一条**独立开关**的 HUD 叠加（`dp_lincoln_perf_info_enabled`），用于在底部展示运行状态与“道路信息”。

**显示内容（从左到右）**
- `Curvature`：曲率/方向盘角/方向盘扭矩
- `Direction`：方向（N/NE/E/…）
- `Road`：优先显示 `RoadName`（mapd 离线 OSM 匹配/逆地理结果），否则回退显示 `lat,lon`（保留 5 位小数）；完全无效则显示 `--`
- `Control`：自动/人工接管
- `Memory`、`CPU Temp`

**布局/样式（用于 1:1 复刻）**
- 字号：`PERF_FONT_SIZE = 32`
- 内边距：`PERF_PADDING = 12`
- 底部留白：`PERF_MARGIN_BOTTOM = UI_BORDER_SIZE // 2`（当前等于 `15px`）
- 字段间距：`PERF_ITEM_GAP = 140`（超宽时会自动收缩）
- 背景色：`PERF_BG_COLOR = rl.Color(0, 0, 0, 120)`（半透明黑，更浅）

**关键代码**
- 绘制：`selfdrive/ui/onroad/augmented_road_view.py` → `_draw_performance_info()` / `_get_road_location_text()`
- 进程启动条件：`system/manager/process_config.py` → `mapd(...)`
  - 下载离线地图进行中：始终运行
  - 上路：开启 `dp_lincoln_osm_realtime_cruise` 或开启性能条（`dp_lincoln_perf_info_enabled`）时运行

## 3. 重要说明

- 本功能只影响**标准 UI（MainLayout / `selfdrive/ui/onroad/*`）** 的绘制；如需同步到 `mici` UI，需要单独实现。
- 新增 Param 需要重新编译 `common/params_pyx`（正常 `scons` 构建会自动处理），否则会出现 `UnknownKeyName: b'dp_lincoln_hud_enhanced'`。
