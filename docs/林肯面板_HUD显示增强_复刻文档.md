# 林肯面板：HUD 显示增强（dp_lincoln_hud_enhanced）复刻文档
## 0. 目标与范围
- 目标：增强 HUD 视觉提示清晰度（车道线/路沿/前车框/制动边框/盲区墙/侧向提示）。
- 不改变控制算法，仅改变 UI 绘制。
- 该开关只影响 HUD 视觉模块；检测/自动避让/曲线控制逻辑不在此开关内。

## 1. 参数与开关
- `dp_lincoln_hud_enhanced`：PERSISTENT BOOL，默认 `0`。
  - 定义位置：`common/params_keys.h`。
- 相关但不属于本开关的参数（路径联动提示依赖）：
  - `CurveSpeedControl`（曲线速度控制主开关）。

## 2. 面板设置入口（UI）
- 文件：`selfdrive/ui/layouts/settings/lincoln.py`
- 分组：`### HUD & Visualization ###`
- Toggle 文案：
  - 标题：`HUD drawing enhancements`
  - 描述：`Enable enhanced HUD visuals (blindspot zones, clearer lane lines, brake cues).`
- 回调：`self._params.put_bool("dp_lincoln_hud_enhanced", val)`

## 3. 状态缓存与刷新
- 文件：`selfdrive/ui/ui_state.py`
- 缓存字段：`ui_state.dp_lincoln_hud_enhanced`
- 刷新频率：每 1s 从 `Params` 读取一次（`_update_settings_params()`）。

## 4. 依赖的实时消息
- `carState`：`vEgo`、`aEgo`、`leftBlinker/rightBlinker`、`leftBlindspot/rightBlindspot`、`brakePressed`。
- `radarState`：`leadOne/leadTwo`（前车状态、距离、相对速度、横向偏移）。
- `modelV2`：`laneLines`/`roadEdges`、`laneLineProbs`、`roadEdgeStds`、`leadsV3`（模型前车兜底）。
- `carOutput`：`actuatorsOutput.brake`（纵向制动指令，边框制动提示用）。
- `carParams`：`openpilotLongitudinalControl`（决定默认是否绘制前车提示）。

## 5. 关键常量（必须一致）
### 5.1 前车框与颜色（`selfdrive/ui/onroad/model_renderer.py`）
- `DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_NEAR = 0.90`
- `DP_LINCOLN_LEAD_BOX_HALF_WIDTH_M_FAR = 1.10`
- `DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_NEAR = 0.70`
- `DP_LINCOLN_LEAD_BOX_HEIGHT_RATIO_FAR = 0.62`
- `DP_LINCOLN_LEAD_BOX_SMOOTHING_FAR = 0.85`
- `DP_LINCOLN_LEAD_BOX_SMOOTHING_NEAR = 0.55`
- `DP_LINCOLN_LEAD_EFFECT_START_M = 130.0`
- `DP_LINCOLN_LEAD_EFFECT_FULL_M = 30.0`
- `DP_LINCOLN_LEAD_EFFECT_TTC_START_S = 5.0`
- `DP_LINCOLN_LEAD_EFFECT_TTC_FULL_S = 2.5`
- `DP_LINCOLN_LEAD_COLOR_FAR = rl.Color(255, 215, 0, 255)`
- `DP_LINCOLN_LEAD_COLOR_NEAR = rl.Color(255, 60, 0, 255)`

### 5.2 车道线/路沿（`selfdrive/ui/onroad/model_renderer.py`）
- 车道线宽度（增强模式）：
  - 主边界（索引 1、2）：`base = 0.055`
  - 次边界（索引 0、3）：`base = 0.050`
  - `line_width = base + 0.015 * lane_line_prob`
- 车道线宽度（非增强）：`line_width = 0.025 * lane_line_prob`
- 路沿宽度：增强 `0.08`，非增强 `0.025`

### 5.3 制动边框与侧边指示（`selfdrive/ui/onroad/augmented_road_view.py`）
- `DP_DECEL_BAR_MIN_MS2 = 0.25`
- `DP_DECEL_BAR_MAX_MS2 = 3.0`
- `DP_HARD_BRAKE_DECEL_MS2 = 3.5`
- `DP_HARD_BRAKE_BRAKE_CMD = 0.7`
- `DP_HARD_BRAKE_FLASH_HZ = 4.0`
- `DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)`
- `DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)`
- `DP_INDICATOR_COLOR_BSM_ENHANCED = rl.Color(255, 0, 0, 255)`
- `DP_INDICATOR_COLOR_BLINKER_ENHANCED = rl.Color(255, 255, 0, 255)`
- 盲区墙颜色（硬编码）：`r=223, g=32, b=32`，alpha 渐变 0.2/0.4/0.6。

## 6. 视觉逻辑详解（完整 1:1）
### 6.1 前车框（Lead Box）
- 入口：`ModelRenderer._update_lead_box()` + `_draw_lead_box()`。
- 前车来源：
  - 优先：`radarState.leadOne`（`status == True`）。
  - 兜底：`modelV2.leadsV3[0]`，要求 `prob >= 0.55` 且 `x0 > 0`。
  - 模型前车 y 轴符号与 radar 相反：使用 `y_rel = -y0` 以保持一致。
- 强度计算：
  - 距离强度：`dist_intensity = interp(d_rel, [30, 130] -> [1, 0])`。
  - TTC 强度（仅 `v_rel < -0.1`）：
    - `ttc = d_rel / max(0.1, -v_rel)`
    - `ttc_intensity = interp(ttc, [2.5, 5.0] -> [1, 0])`
  - `intensity = max(dist_intensity, ttc_intensity)`
- 若 `intensity <= 0.01`：逐步淡出，`_lead_box_alpha -= 0.10`。
- 无有效 lead（雷达/模型都无）：`_lead_box_alpha -= 0.20`，降到 0 后 `self._lead_box_valid=False`。
- 有效 lead 时：`_lead_box_alpha += 0.25`（上限 1.0）。
- 框几何：
  - 宽度：`half_width_m` 在 30m~130m 线性插值 `0.90 -> 1.10`。
  - 高度：`height_ratio` 在 30m~130m 线性插值 `0.70 -> 0.62`。
  - `height_px = width_px * height_ratio`。
  - margin：`max(6, width_px * 0.03)`；总宽加 margin。
- 平滑：
  - `smoothing = interp(intensity, [0, 1] -> [0.85, 0.55])`。
  - 有效时采用指数平滑更新 box 坐标；首次直接赋值。
- 颜色/透明度：
  - 颜色随 `intensity` 在 `DP_LINCOLN_LEAD_COLOR_FAR -> NEAR` 间插值。
  - alpha：`_lead_box_alpha * (110 + 145 * intensity)`。
  - 绘制：圆角矩形线框（厚度 `3`）。

### 6.2 前车三角 + 距离/速度文本
- 入口：`ModelRenderer._draw_lead_indicator()`。
- 触发条件：
  - `radarState` 有效，且 `(openpilotLongitudinalControl == True) OR (dp_lincoln_hud_enhanced == True)`。
- 文本显示（增强模式）：
  - 仅显示 **距离 + 前车速度**（不显示 TTC）。
  - 距离：`{d_rel:.1f}m` / `ft`。
  - 速度：`{v_lead*3.6:.0f}km/h` / `mph`。

### 6.3 车道线/路沿增强
- 车道线透明度（增强模式）：
  - `alpha = clip(prob * 1.35, 0..1)`。
  - `prob < 0.05` 直接 `alpha = 0`。
  - 否则 `alpha >= 0.40`。
  - 主边界（索引 1/2）：
    - ENGAGED：`alpha >= 0.80`
    - 非 ENGAGED：`alpha >= 0.55`
    - 颜色：`(0,255,80)`
  - 次边界（索引 0/3）：
    - ENGAGED：`alpha >= 0.55`
    - 非 ENGAGED：`alpha >= 0.45`
    - 颜色：`(0,220,70)`
- 路沿线透明度（增强模式）：
  - `confidence = clip(1 - roadEdgeStd, 0..1)`
  - `alpha = max(0.45, sqrt(confidence))`
  - 颜色：红色 `(255,0,0)`

### 6.4 道路带宽染色（与前车接近联动）
- 入口：`ModelRenderer._draw_path()`。
- 条件：`dp_lincoln_hud_enhanced == True` 且 `_lead_box_intensity > 0.01`。
- 颜色：同前车框颜色插值（黄 -> 红）。
- 渐变：从底部透明到顶部 `alpha = 20 + intensity * 120`。

### 6.5 制动边框（整屏红框提示）
- 入口：`AugmentedRoadView._draw_border()`。
- 条件：`dp_lincoln_hud_enhanced == True`。
- 强度来源：
  1) 实际减速度（`carState.aEgo`）：
     - `decel = max(0, -aEgo)`
     - `decel_intensity = interp(decel, [0.25, 3.0] -> [0, 1])`
  2) 指令制动（`carOutput.actuatorsOutput.brake`）：
     - `brake_intensity = interp(brake_cmd, [0.02, 0.6] -> [0, 1])`
- 合成：`intensity_raw = max(decel_intensity, brake_intensity)`。
  - 若 `brakePressed == True`，强制 `intensity_raw >= 0.20`。
  - 使用 `FirstOrderFilter(0.0, 0.3, 1/fps)` 平滑。
- 硬刹提示：
  - 条件：`modelV2.meta.hardBrakePredicted == True` 或 `decel >= 3.5` 或 `brake_cmd >= 0.7`。
  - 动作：红色闪烁（4Hz）。
- 非硬刹：将基础边框颜色线性拉向红色。

### 6.6 盲区墙与侧向提示
- 入口：`AugmentedRoadView._draw_hud_enhancements()`。
- 盲区墙：
  - 当 `leftBlindspot/rightBlindspot == True` 时绘制。
  - 区域：基于模型车道线构建相邻车道多边形（`_fp_get_adjacent_lane_polygon()`）。
  - 多边形长度：`max_distance = clip(path_end, 10..100m)`；若有 lead → `lead_d*2` 并缩短 `max_distance = clip(lead_d - min(lead_d*0.35, 10.0), 0, max_distance)`。
  - 车道宽度计算：若道路边缘比车道线更近，则视为无可用车道（不画墙）。
  - 渐变：红色自下而上渐弱（alpha 0.6/0.4/0.2）。
- 侧向提示（仅打灯，无盲区）：
  - `_draw_hud_enhanced_side_zone()` 绘制侧向黄色半透明梯形渐变。
  - 几何参数：
    - `inset = rect.width * 0.01`
    - `top_y = rect.y + min(320, rect.height * 0.35)`
    - `bottom_y = rect.y + rect.height`
    - `slant = rect.height * 0.10`
    - `top_w = rect.width * 0.10`
    - `bottom_w = rect.width * 0.16`
  - alpha 基准：70（盲区+打灯 180，盲区 140）。

### 6.7 侧边指示条（边框内侧）
- 入口：`AugmentedRoadView._draw_border()`。
- 逻辑：`_update_dp_indicator_side_state()` 维护闪烁计数。
- 增强模式：
  - 盲区（含打灯）：红色闪烁；打灯时更快。
  - 仅打灯：黄色闪烁。
- 非增强模式：
  - 盲区：黄色常亮或快闪（与打灯叠加）。
  - 打灯：绿色闪烁。

## 7. 文件路径清单
- `common/params_keys.h`
- `selfdrive/ui/layouts/settings/lincoln.py`
- `selfdrive/ui/ui_state.py`
- `selfdrive/ui/onroad/model_renderer.py`
- `selfdrive/ui/onroad/augmented_road_view.py`

## 8. 复刻步骤（一步不省）
1. 在 `common/params_keys.h` 注册 `dp_lincoln_hud_enhanced`，默认值 `0`。
2. 在 `selfdrive/ui/layouts/settings/lincoln.py` 添加开关项（标题/描述与回调一致）。
3. 在 `selfdrive/ui/ui_state.py` 中：
   - 添加 `dp_lincoln_hud_enhanced` 成员；
   - 在 `_update_settings_params()` 内读取 `Params`；
   - `update()` 中保证 1s 刷新。
4. 在 `selfdrive/ui/onroad/model_renderer.py`：
   - 增加前车框（lead box）逻辑与常量；
   - 车道线宽度/透明度/颜色增强；
   - 路沿线加粗与强制红色；
   - 前车接近时的道路带宽染色。
5. 在 `selfdrive/ui/onroad/augmented_road_view.py`：
   - 增强边框制动提示（整屏红框与硬刹闪烁）；
   - 盲区墙与侧向提示；
   - 侧边指示条颜色与闪烁逻辑。

## 9. 验证清单
- 开关关闭：车道线恢复白色、路沿线细、无整屏红框、无盲区墙。
- 开关打开：
  - 车道线更粗更亮（主边界更亮）。
  - 路沿线加粗为红色。
  - 有前车时显示前车框，且距离越近颜色越红。
  - 制动/急刹时边框变红或红色闪烁。
  - 盲区出现时相邻车道有红色“墙”。
  - 打灯时侧边条闪烁显示。

## 10. 排查要点
- 无前车框：`radarState` 未有效或 `leadOne.status == False`；模型兜底仅在 radarState 存在时才会启用。
- 盲区墙不出现：`carState.leftBlindspot/rightBlindspot` 未触发或车道线不足以构造相邻车道。
- 制动红框不明显：`aEgo`/`brake_cmd` 无明显负值或 `carOutput` 未有效。
