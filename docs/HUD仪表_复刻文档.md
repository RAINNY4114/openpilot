# HUD仪表_复刻文档

目标：基于当前代码完整复刻 onroad HUD（含相机画面、路径/车道线、速度/按钮、警报、驾驶员监控、检测框、边框指示等）。所有尺寸、颜色、开关、渲染顺序均来自现有实现，确保 1:1 复现。

适用版本：本仓库当前代码（参考 `selfdrive/ui/onroad/*.py`, `system/ui/lib/application.py`, `selfdrive/ui/ui_state.py` 等）。

---

**1. 入口与渲染流程**

- onroad 界面入口：`selfdrive/ui/layouts/main.py`
  - `MainLayout` 在 `ui_state.started == True` 时切换到 `AugmentedRoadView`。
  - onroad 时隐藏侧边栏（除非用户点击触发）。
- 主渲染类：`selfdrive/ui/onroad/augmented_road_view.py`
  - 仅在 `ui_state.started` 时渲染（未 onroad 直接 return）。
  - 渲染顺序（严格）：
    1) 更新 DP 指示灯状态 `_update_dp_indicator_states`
    2) 相机流切换 `_switch_stream_if_needed`
    3) 更新标定 `_update_calibration`
    4) 计算 `content_rect` 并开启 scissor 裁剪
    5) 相机画面（继承 `CameraView`）
    6) `ModelRenderer`（车道线/路径/lead）
    7) HUD 增强层 `_draw_hud_enhancements`（盲区墙/意图高亮）
    8) 性能信息条 `_draw_performance_info`（可选）
    9) `HudRenderer`（速度/按钮/曲线速度控件）
    10) `AlertRenderer`（告警条）
    11) `DriverStateRenderer`（驾驶员监控图标，非 LITE 且未隐藏）
    12) 目标检测框 `_draw_object_detections`
    13) 关闭 scissor
    14) 绘制外框 `_draw_border` + 侧边指示条

渲染被隐藏条件：
- `dp_ui_hide_hud_speed_kph > 0` 且 `vEgo > dp_ui_hide_hud_speed_ms` 时：隐藏 `HudRenderer` 与 `DriverStateRenderer`（其余层仍渲染）。
- `LITE` 环境变量存在时：不渲染 `DriverStateRenderer`。

---

**2. 基础画布与缩放**

- UI 默认分辨率：`system/ui/lib/application.py`
  - Big UI：2160 x 1080
  - Small UI：536 x 240
  - `gui_app.width/height` 为当前 UI 尺寸。
- UI 边框：`UI_BORDER_SIZE = 30` (`selfdrive/ui/__init__.py`)
- 字体缩放：`FONT_SCALE = 1.242` (BIG_UI) / `1.16` (SMALL)
  - `rl.draw_text_ex` 被全局 patch，所有字体尺寸都会乘以 `FONT_SCALE`。
- HUD 全部坐标以 `rect`（onroad 全屏）和 `content_rect`（去边框区域）为基准：
  - `content_rect = (rect.x+30, rect.y+30, rect.w-60, rect.h-60)`
  - Camera/模型/检测框全部在 `content_rect` 内渲染并 scissor 裁剪。

---

**3. 相机画面（前视）**

文件：`selfdrive/ui/onroad/cameraview.py` + `selfdrive/ui/onroad/augmented_road_view.py`

数据源：
- `VisionIpcClient("camerad", VISION_STREAM_ROAD)`
- 可切换 `VISION_STREAM_WIDE_ROAD`

切换逻辑（仅实验模式且 wide 流存在）：
- vEgo < 10.0 m/s -> WIDE
- vEgo > 15.0 m/s -> ROAD
- 10~15 m/s：保持当前流（迟滞区）

相机渲染流程（`CameraView._render`）：
- 收帧：非阻塞 `recv(0)`, 有帧则更新纹理。
- TICI：EGL 零拷贝 + `samplerExternalOES` shader。
- 非 TICI：Y/UV 分离纹理 + YUV->RGB shader。
- Driver camera 时 `src_rect.width` 取负值实现水平翻转。

AugmentedRoadView 自定义相机矩阵（对齐标定 & 视角）：
- 取 `DeviceCameraConfig` 中 `intrinsics`，配合 `view_from_calib`。
- zoom：ROAD=1.1, WIDE=2.0
- 计算 vanishing point `kep = intrinsic @ calib @ INF_POINT`
- 限制 `x_offset/y_offset`（margin=5px）
- 生成 `cached_matrix`：
  ```
  [
    [zoom * 2 * cx / w, 0, -x_offset / w * 2],
    [0, zoom * 2 * cy / h, -y_offset / h * 2],
    [0, 0, 1]
  ]
  ```
- 同时生成 `video_transform` 给 `ModelRenderer`：
  ```
  [
    [zoom, 0, (w/2 + x - x_offset) - (cx * zoom)],
    [0, zoom, (h/2 + y - y_offset) - (cy * zoom)],
    [0, 0, 1]
  ]
  ```

占位色：
- 无帧时显示 `BORDER_COLORS[DISENGAGED]`。

---

**4. 边框与状态颜色**

文件：`selfdrive/ui/onroad/augmented_road_view.py`

颜色：
- DISENGAGED: (0x12, 0x28, 0x39, 0xFF)
- OVERRIDE:   (0x89, 0x92, 0x8D, 0xFF)
- ENGAGED:    (0x16, 0x7F, 0x40, 0xFF)
- ALKA:       (0x22, 0xA0, 0xDC, 0xF1)

绘制：
- 外框黑色线：`draw_rectangle_lines_ex(rect, UI_BORDER_SIZE, BLACK)`
- 内部圆角线框：`roundness=0.12`, `thickness=UI_BORDER_SIZE`
- 当 `dp_alka_active` 且 `status==DISENGAGED` 时用 ALKA 颜色。

Lincoln HUD 增强（dp_lincoln_hud_enhanced）：
- 边框颜色随制动强度变化（减速度 or brake_cmd）。
- decel: `aEgo` -> `decel = max(0, -aEgo)`
- brake_cmd: `carOutput.actuatorsOutput.brake`
- intensity = max(decel_intensity, brake_intensity)，低通滤波 `FirstOrderFilter(0.0, 0.3, 1/fps)`
- 若硬刹车：`decel >= 3.5` 或 `brake_cmd >= 0.7` 或 `modelV2.meta.hardBrakePredicted`
  - 以 4Hz 红色闪烁
- 否则按 intensity 从基色渐变到红色（R=255,G=0,B=0）。

侧边指示条：
- 左/右矩形条：宽=UI_BORDER_SIZE，高=rect.h-8*UI_BORDER_SIZE，起始 y=rect.y+4*UI_BORDER_SIZE
- 颜色/闪烁：
  - `DP_INDICATOR_BLINK_RATE_FAST = fps*0.25`
  - `DP_INDICATOR_BLINK_RATE_STD = fps*0.5`
  - 非增强逻辑：blinker/BSM 组合决定闪烁与颜色
  - 增强逻辑：blinker=黄闪，BSM=红闪，组合用红快闪

---

**5. HUD 速度区（顶部）**

文件：`selfdrive/ui/onroad/hud_renderer.py`

尺寸（UIConfig）：
- header_height=300
- border_size=30
- button_size=192
- wheel_icon_size=144
- set_speed_width_metric=200
- set_speed_width_imperial=172
- set_speed_height=204

字体尺寸（FontSizes）：
- current_speed=176
- speed_unit=66
- max_speed=40
- set_speed=90

颜色（RGBA）：
- engaged: (128,216,166,255)
- disengaged/override: (145,155,149,255)
- engaged_bg: (128,216,166,204)
- disengaged_bg: (0,0,0,153)
- black_translucent: (0,0,0,166)
- white_translucent: (255,255,255,200)
- border_translucent: (255,255,255,75)
- header_gradient_start: (0,0,0,114)
- header_gradient_end: transparent

绘制：
- 顶部渐变条：`draw_rectangle_gradient_v(rect.x, rect.y, rect.width, 300, start, end)`

MAX/设定速度框：
- x = rect.x + 60 + (set_speed_width_imperial - set_speed_width)/2
- y = rect.y + 45
- 圆角背景：`roundness=0.35`, `segments=10`
- 边框线：厚度 6，颜色 `border_translucent`
- “MAX”文字：
  - 字体 SemiBold, size=40
  - x 居中，y = y + 27
- 设定速度：
  - 字体 Bold, size=90
  - x 居中，y = y + 77
  - 未设定：显示 `CRUISE_DISABLED_CHAR`（实际为 U+2013 EN DASH "–"）

当前速度：
- 数字字体 Bold size=176
- x 居中，y = 180（以文字高度居中对齐）
- 单位（km/h 或 mph）字体 Medium size=66，y=290

速度单位：
- `ui_state.is_metric` 决定 km/h 或 mph
- 设定速度在英制时乘 `KM_TO_MILE=0.621371`

---

**6. HUD 按钮**

Exp 按钮（右上）：
- 位置：`x=rect.x + rect.w - border_size - button_size`, `y=rect.y + border_size`
- 圆形背景：alpha 166
- 图标纹理：
  - `dragonpilot/selfdrive/assets/icons/chffr_wheel.png`
  - `selfdrive/assets/icons/experimental.png`
- 按下透明度：alpha=180；未 engageable 时也降 alpha。
- 点击切换 `ExperimentalMode`（需 `ExperimentalModeConfirmed` 为 True）。

Lane Pref 按钮（左侧中间）：
- 尺寸：`button_size * 0.85`（约 163）
- 垂直位置：位于 “MAX 框底部” 与 “驾驶员监控按钮顶部” 中点
- 水平位置：与驾驶员监控 icon 的中心对齐
- 视觉：圆形半透明背景 + 环形描边 + 字母 “A/L/R”
- 状态色：
  - A: 白色 120 alpha
  - L: 蓝色 160 alpha
  - R: 橙色 160 alpha
- 开关：
  - `dp_lincoln_lane_preference` (0/1/2)
  - 仅在 Ford 且 `dp_lincoln_auto_overtake` 或 `dp_lincoln_auto_avoid` 开启时显示

Torque Bar（底部弧形扭矩条）：
- 文件：`dragonpilot/selfdrive/ui/onroad/torque_bar.py`
- SCALE=2.5, TORQUE_ANGLE_SPAN=12.7
- 主要参数：
  - `torque_line_radius=1200 * SCALE`
  - `torque_line_offset = interp(|x|, [0.5,1], [22,26]) * SCALE`
  - `torque_line_height = interp(|x|, [0.5,1], [14,56]) * SCALE`
  - 中心点：`cx = rect.x + rect.w/2 + 8*SCALE`, `cy = rect.y + rect.h + radius - offset`
- 颜色：白->黄->橙渐变，engaged 时更亮；非 engaged 时降低 alpha。
- 仅在 `controlsState.lateralControlState != angleState` 时绘制。

---

**7. 曲线速度控件（Curve Speed Widget）**

文件：`selfdrive/ui/onroad/hud_renderer.py`

显示条件：
- `CurveSpeedControl` == True
- `ShowCSCStatus` == True
- cruise 已设定
- `modelV2` 与 `longitudinalPlan` 已更新

位置与尺寸：
- icon 区域：`widget_size = button_size * 1.25`（约 240）
- icon 位置：紧贴 MAX 框右侧
  - x = set_speed_rect.x + set_speed_rect.width + border_size
  - y = set_speed_rect.y
- 文本卡片：
  - y = y + widget_size + 10
  - height = 100
  - width = max(2*widget_size, max(text_width) + 40)
  - 圆角：0.35，描边 10
  - 蓝色半透明背景 + 实线蓝边

文字：
- 上行距离文本 size=34
- 下行目标速度文本 size=50
- 默认内容（按源码中的中文字符串）：
  - 目标速度：`"目标 {round(v_target_disp)} {unit}"`
  - 距离：`"前方弯道 {dist} m 路 {source}"`
  - source：vision / map（`curveSpeedSource` in {1,2}）

L/R icon 方向：
- 根据预测曲率符号决定
- 死区 `2e-4`，最小保持 0.25s 防抖

---

**8. Model 渲染（路径/车道线/道路边缘/lead）**

文件：`selfdrive/ui/onroad/model_renderer.py`

基础参数：
- `CLIP_MARGIN=500`
- `MIN_DRAW_DISTANCE=10.0`
- `MAX_DRAW_DISTANCE=160.0`

输入：
- `modelV2`, `liveCalibration`, `radarState`, `controlsState`, `carState`, `carParams`

映射流程：
- 模型点（x,y,z）投影到屏幕：使用 `video_transform @ calib_transform`
- 每条线用 `_map_line_to_polygon` 生成封闭多边形
- `path` 使用 `y_off=0.9`, `z_off=path_offset_z`

车道线绘制：
- 四条 laneLines（0:左远,1:左近,2:右近,3:右远）
- 线宽：
  - 默认：`0.025 * prob`
  - Lincoln HUD 增强：近线 base=0.055，远线 base=0.050，最终 `base + 0.015*prob`
- 颜色：
  - 默认：白色，alpha = clip(prob, 0~0.7)
  - 增强：绿色系
    - 主边界(1,2)：alpha >= 0.80(engaged)/0.55(非)
    - 辅边界(0,3)：alpha >= 0.55(engaged)/0.45(非)

道路边缘：
- 线宽：增强=0.08，默认=0.025
- 颜色：红色 (255,0,0)，alpha=confidence 或 sqrt(confidence)

路径填充：
- 普通模式：油门/无油门渐变
  - `THROTTLE_COLORS` = [(13,248,122,102), (114,255,92,89), (114,255,92,0)]
  - `NO_THROTTLE_COLORS` = [(242,242,242,102), (242,242,242,89), (242,242,242,0)]
  - `allowThrottle` 通过 `longitudinalPlan.allowThrottle` 与 `openpilotLongitudinalControl` 决定
- 实验模式：加速度渐变（HSL 转 RGB）
- 曲线速度预警：在路径上叠加红色渐变

Lead 绘制：
- 车头 chevron：三角扇面（黄 glow + 红填充）
- Lincoln HUD 增强模式：
  - 显示 `距离(米/英尺)` + `lead 速度(km/h|mph)`
  - 字体大小：56 / 48
  - 额外绘制 “雷达/模型 lead box”：
    - 根据 `dRel` 计算宽度、高度、透明度
    - 颜色由远到近：金黄 -> 红色

LiveTracks（DP 调试）：
- `dp_ui_lead` in [radar, all] 时绘制
- 每个点：红色圆 + 文字 (d/y/dV)

---

**9. Alert 提示框**

文件：`selfdrive/ui/onroad/alert_renderer.py`

尺寸参数：
- ALERT_MARGIN=40
- ALERT_PADDING=60
- ALERT_LINE_SPACING=45
- ALERT_BORDER_RADIUS=30
- 小/中高度：271 / 420

字体：
- small=66
- medium=74
- big=88

颜色：
- normal: #151515 alpha F1
- userPrompt: #DA6F25 alpha F1
- critical: #C92231 alpha F1

布局：
- small：单行居中
- mid：上行大字 + 下行小字
- full：全屏遮罩，标题/副标题位置固定

---

**10. Driver Monitoring 图标**

文件：`selfdrive/ui/onroad/driver_state.py`

尺寸与位置：
- BTN_SIZE=192
- IMG_SIZE=144
- 中心位置：
  - 左驾：`x = rect.x + UI_BORDER_SIZE + BTN_SIZE/2`
  - 右驾：`x = rect.x + rect.w - (UI_BORDER_SIZE + BTN_SIZE/2)`
  - y = rect.y + rect.h - (UI_BORDER_SIZE + BTN_SIZE/2)

绘制层：
- 背景圆：黑色 alpha=70
- face icon：`icons/driver_face.png`，alpha=0.65(active)/0.2(inactive)
- 线条：`draw_spline_linear`，thickness=5.2
- 水平/垂直弧线：ARC_LENGTH=133，ARC_THICKNESS_DEFAULT=6.7，EXTEND=12.0
- engaged 状态颜色：绿(26,242,66)，否则灰(139,139,139)

可见性：
- 仅当 `selfdriveState.alertSize == none` 且 `driverStateV2` 已更新。

---

**11. 目标检测框（锥桶/行人/车辆）**

文件：`selfdrive/ui/onroad/augmented_road_view.py` + `object_tracker.py`

开关：
- Param: `dp_lat_cone_detection` 必须为 True
- Env:
  - `DP_DET_SHOW_TRACK_ID`
  - `DP_DET_SHOW_DISTANCE`
  - `DP_DET_DRAW_ALL`

数据源：
- `customReservedRawData0`（coned 输出）
- `decode_cone_detections` 解析字段：`imgW/imgH/objsR/objs/cones`

显示逻辑：
- 使用 `ObjectTracker` 做 IoU 追踪平滑：
  - iou_thres=0.30, max_missed=5
  - bbox/vel/score EMA smoothing
- stale 超时 2s 后清空
- 坐标映射：
  ```
  sx1 = cam_dst.x + (x1 / img_w) * cam_dst.w
  sy1 = cam_dst.y + (y1 / img_h) * cam_dst.h
  sx2 = cam_dst.x + (x2 / img_w) * cam_dst.w
  sy2 = cam_dst.y + (y2 / img_h) * cam_dst.h
  ```

形状：
- 车辆：角框 + 车道方向箭头
- 行人：简笔人图标
- 锥桶：三角锥图标
- 其他：矩形框

标签：
- 条件：车辆类 或 `SHOW_TRACK_ID` 或 `SHOW_DISTANCE`
- 字体 size：14~22（随 box 尺寸）
- 背景色：与 box 同色，alpha=180

颜色分类：
- 行人：蓝色 (0,170,255,220)
- 锥桶：橙色 (255,149,0,220)
- 车辆：红色 (255,60,60,220)
- 标志牌：黄 (255,220,0,220)
- 动物：紫 (180,120,255,220)
- 其他：灰 (220,220,220,200)

---

**12. HUD 增强层（盲区墙/车道占用）**

文件：`selfdrive/ui/onroad/augmented_road_view.py`

条件：
- 仅 `dp_lincoln_hud_enhanced == True`

盲区墙：
- 左/右盲区时绘制 adjacent lane polygon
- 颜色：红色梯度 (223,32,32) alpha 0.2/0.4/0.6

车道占用墙（orange block）：
- 基于 coned 检测 + lane lines 估计车道占用
- intensity 依距离变化，alpha 渐变
- 颜色：`DET_COLOR_LANE_BLOCK` (255,149,0,255)

意图高亮（非盲区）：
- 左右转向灯时显示斜面高亮带
- 梯度透明度：alpha 70 -> 25
- 斜边参数：
  - inset = rect.w * 0.01
  - top_y = rect.y + min(320, rect.h*0.35)
  - bottom_y = rect.y + rect.h
  - slant = rect.h * 0.10
  - top_w = rect.w * 0.10
  - bottom_w = rect.w * 0.16

---

**13. 性能信息条（底部）**

文件：`selfdrive/ui/onroad/augmented_road_view.py`

开关：
- `dp_lincoln_perf_info_enabled`

布局：
- 字体 `FontWeight.MEDIUM`, size=32
- padding=12, gap=140, bottom margin=15
- 背景：黑色 alpha=120
- 宽度：根据内容自适应，最大 `min(rect.w-40, rect.w*0.98)`
- 文本包含：
  - Curvature / Direction / Road / Control / Memory / CPU Temp

---

**14. 参数/环境变量总表**

Params（影响 HUD）：
- `IsMetric`：单位
- `ExperimentalMode` / `ExperimentalModeConfirmed`
- `CurveSpeedControl`
- `ShowCSCStatus`
- `dp_lincoln_hud_enhanced`
- `dp_lincoln_perf_info_enabled`
- `dp_lincoln_lane_preference`
- `dp_lincoln_auto_overtake`
- `dp_lincoln_auto_avoid`
- `dp_lat_cone_detection`
- `dp_ui_hide_hud_speed_kph`
- `dp_ui_rainbow`
- `dp_ui_lead`
- `GPSQualityOK`, `RoadName`, `MapTargetVelocities`

环境变量：
- `LITE`
- `DP_DET_SHOW_TRACK_ID`
- `DP_DET_SHOW_DISTANCE`
- `DP_DET_DRAW_ALL`
- `DP_LINCOLN_AUTO_LC_TGAP_S`
- `DP_LINCOLN_AUTO_LC_MIN_DIST_M`
- `DP_LINCOLN_AUTO_LC_MAX_DIST_M`
- `DP_LINCOLN_AUTO_LC_LANE_LINE_PROB_MIN`
- `DP_LINCOLN_AUTO_LC_LANE_MARGIN_M`
- `DP_LINCOLN_AUTO_LC_SIDE_CLOSE_DIST_M`

---

**15. 资源/字体**

字体（`system/ui/lib/application.py`）：
- `Inter-Regular.fnt`, `Inter-Medium.fnt`, `Inter-Bold.fnt`, `Inter-SemiBold.fnt`
- 字符缺失时回退 `unifont.fnt`

图标：
- `selfdrive/assets/icons/curve_speed.png`
- `selfdrive/assets/icons/curveR_speed.png`
- `selfdrive/assets/icons/experimental.png`
- `dragonpilot/selfdrive/assets/icons/chffr_wheel.png`
- `selfdrive/assets/icons/driver_face.png`

---

**16. 复刻检查清单**

- 相机流切换阈值、zoom、vanishing point 对齐一致
- scissor 裁剪区为去边框 content_rect
- HUD 顶部速度与 MAX 框尺寸、位置完全一致
- HUD 按钮位置与尺寸一致（右上 EXP、左侧 Lane Pref）
- 车道线/路径宽度与渐变颜色一致
- Lead 车框/chevron/文字一致
- 边框颜色与制动强度映射一致
- Alert 提示框尺寸/颜色/字体一致
- Driver Monitoring 图标位置与弧线参数一致
- 目标检测框样式与过滤逻辑一致
- 性能信息条布局/宽度/字体一致
