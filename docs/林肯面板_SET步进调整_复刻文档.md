# 林肯面板：SET+/SET- 步进调整复刻文档

目标：不读源码即可 1:1 实现“SET+/SET- 步进调整”功能与 UI。

## 1. 必须修改/新增的文件清单
- `common/params_keys.h`
- `selfdrive/ui/layouts/settings/lincoln.py`
- `selfdrive/car/cruise.py`

## 2. 参数定义（`common/params_keys.h`）
- `dp_lincoln_set_speed_step_enabled`：PERSISTENT BOOL，默认 `"1"`（默认开启，无 UI 开关）
- `dp_lincoln_set_speed_step_short_kph`：PERSISTENT INT，默认 `"1"`（范围 1–5）
- `dp_lincoln_set_speed_step_long_kph`：PERSISTENT INT，默认 `"5"`（范围 5–30，步进 5）

## 3. 面板 UI（`selfdrive/ui/layouts/settings/lincoln.py`）
- 分组标题：`"### SET+/SET- Step Adjustment ###"`
- 无 UI 开关，参数默认开启。
- Spin：`"Short press step (km/h)"` → `dp_lincoln_set_speed_step_short_kph`
  - description：`"Speed change per short press of SET+/SET-."`
  - initial_value：`clamp(dp_lincoln_set_speed_step_short_kph, 1..5)`
  - min=1, max=5, step=1, suffix=" km/h"
- Spin：`"Long press step (km/h)"` → `dp_lincoln_set_speed_step_long_kph`
  - description：`"Speed change per long press of SET+/SET-."`
  - initial_value：`round-to-5(dp_lincoln_set_speed_step_long_kph)`
  - min=5, max=30, step=5, suffix=" km/h"

## 4. 运行时逻辑（`selfdrive/car/cruise.py`）
- 仅在 `CP.pcmCruise == False`（openpilot 自控巡航）时生效。
- 仅在 `is_metric == True` 时启用自定义步进（英制保持原逻辑）。
- 参数读取与 clamp：
  - short：`clamp(dp_lincoln_set_speed_step_short_kph, 1..5)`
  - long：`round-to-5(clamp(dp_lincoln_set_speed_step_long_kph, 5..30))`
- 按键逻辑：
  - 短按使用 short 步进。
  - 长按使用 long 步进。
  - 其它条件下保持默认：公制短按 1 km/h、长按 5 km/h；英制短按 1 mph、长按 5 mph。

## 5. 复刻步骤
1. `common/params_keys.h` 注册 3 个参数键（含默认值，默认开启）。
2. `lincoln.py` 添加分组标题与短按/长按步进 Spin，并绑定参数。
3. `cruise.py` 在非 PCM 巡航逻辑中读取参数并替换步进计算。

## 6. 验证要点
- 短按/长按步进随设置变化（开机即生效）。
- 长按步进仅出现 5 的倍数。
- `CP.pcmCruise == True` 时不受影响。
