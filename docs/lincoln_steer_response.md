# Lincoln 转向响应调节（复刻指南 1:1）

目标：为福特/林肯非 CAN-FD 车型提供可调的转向速率（入弯/回正）倍率，默认行为保持不变，用户可在 UI 调整，范围受保护（0.8x–1.2x）。

## 1. 参数键（持久化）
文件：`common/params_keys.h`
新增：
- `dp_lincoln_steer_rate_up_pct`：入弯速率倍率，百分比，默认 `100`（1.00x），范围 80–120。
- `dp_lincoln_steer_rate_down_pct`：回正速率倍率，百分比，默认 `100`（1.00x），范围 80–120。

## 2. 控制逻辑
文件：`opendbc_repo/opendbc/car/ford/values.py`
- 保留原始速率表：
  - 入弯（rate_up）：breakpoints `[5, 25]` → values `[0.00045, 0.00010]`
  - 回正（rate_down）：breakpoints `[5, 25]` → values `[0.00045, 0.00015]`
- 新增 `CarControllerParams.get_angle_limits()`：
  - 从 Params 读取倍率（缺省 100）；`_scaled_rates` 将倍率夹到 0.8–1.2，避免超出 EPS 能力。
  - 按倍率等比缩放 rate_up / rate_down，曲率硬上限仍为 0.02 m⁻¹。
  - 结果缓存 1s，减少 Params 读取。
- 缓存字段：`_ANGLE_LIMITS_CACHE`、`_ANGLE_LIMITS_CACHE_T`。

文件：`opendbc_repo/opendbc/car/ford/carcontroller.py`
- 将转向限幅调用改为使用动态限幅：  
  `apply_std_steer_angle_limits(..., CarControllerParams.get_angle_limits())`

## 3. UI 设置入口
文件：`selfdrive/ui/layouts/settings/lincoln.py`
新增区块 “### Steering Response ###” 含两项 spin 控件（步进 2、范围 80–120、建议小步 ±5%）：
- `Steer rate (turn-in)` → 绑定 `dp_lincoln_steer_rate_up_pct`
- `Steer rate (return)` → 绑定 `dp_lincoln_steer_rate_down_pct`
描述说明：默认 100%（全速域），调高更快但警告风险高，调低更顺/更慢，建议小步调整。前瞻/曲率等其他控件不受影响。

## 4. 翻译
文件：`selfdrive/ui/translations/app_zh-CHS.po`
对应新增的 UI 文案，中文已写明小步调整和警告风险。

## 5. 默认行为与安全
- 默认值保持原始速率表，未调整时转向行为完全一致。
- 倍率被限制在 0.8x–1.2x，且曲率硬上限 0.02 m⁻¹ 未改，防止超 EPS 限制。
- 全速域生效：5–25 m/s 按表插值，<5 m/s 用首值，>25 m/s 用末值，随后乘以倍率。
- 调高可能增加 “Turn Exceeds Steering Limit” 警告概率，建议用户每次仅 ±5% 微调并路测验证。

## 6. 路测/验证提示
- 建议在安全路段微调（+5%），观察是否出现警告或抖动；如有，回到 100% 或降低。
- 如需更细的分速段调节，需要扩展速率表断点或分段倍率，本方案为全域统一倍率。 
