# "Turn Exceeds Steering Limit" 警告分析验证

## 验证结论

**其他AI的分析完全准确。** 以下是代码验证结果。

---

## 1. 触发条件（已验证）

文件：`E:\openpilot\selfdrive\selfdrived\selfdrived.py:377-385`

```python
if lac.active and not recent_steer_pressed and not self.CP.notCar:
    clipped_speed = max(CS.vEgo, 0.3)
    actual_lateral_accel = controlstate.curvature * (clipped_speed**2)
    desired_lateral_accel = self.sm['modelV2'].action.desiredCurvature * (clipped_speed**2)
    undershooting = abs(desired_lateral_accel) / abs(1e-3 + actual_lateral_accel) > 1.2
    turning = abs(desired_lateral_accel) > 1.0
    # TODO: lac.saturated includes speed and other checks, should be pulled out
    if undershooting and turning and lac.saturated:
        self.events.add(EventName.steerSaturated)
```

**触发条件（三个必须同时满足）：**
1. `undershooting > 1.2`：实际横向加速度不足期望值的83%
2. `turning > 1.0`：期望横向加速度 > 1.0 m/s²
3. `lac.saturated == True`：横向控制饱和

---

## 2. lac.saturated 的计算逻辑（已验证）

文件：`E:\openpilot\selfdrive\controls\lib\latcontrol.py:22-29`

```python
def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
        self.sat_time += self.dt
    else:
        self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)
```

**关键发现：**
- `lac.saturated` 包含 `curvature_limited`（曲率限制）
- 只要 `saturated or curvature_limited` 为真，就会累积 `sat_time`
- 当 `sat_time` 达到 `sat_limit` 时，`lac.saturated` 返回 `True`

**调用位置（已验证）：**

文件：`E:\openpilot\selfdrive\controls\lib\latcontrol_pid.py:47`
```python
pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))
```

文件：`E:\openpilot\selfdrive\controls\lib\latcontrol_torque.py:109`
```python
pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))
```

文件：`E:\openpilot\selfdrive\controls\lib\latcontrol_angle.py:34`
```python
angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
```

---

## 3. road-edge 曲率偏置（已验证）

文件：`E:\openpilot\selfdrive\controls\controlsd.py:134`

```python
return _clamp(correction, -0.002, 0.002)
```

**确认：当前分支确实添加了 ±0.002 的曲率偏置。**

完整逻辑：
- Lines 59-86：检测左右路边
- Lines 89-136：计算曲率修正
- Line 134：限制在 ±0.002 范围内

---

## 4. 多弯道场景为何更易触发

**其他AI的分析完全正确：**

### 原因1：desiredCurvature 快速变化
- 多弯道场景中，模型输出的 `desiredCurvature` 频繁变化
- 实际转向有执行延迟（Ford `steerActuatorDelay=0.2`）
- 导致 `actual_lateral_accel` 跟不上 `desired_lateral_accel`
- 触发 `undershooting > 1.2`

### 原因2：curvature_limited 累积 sat_time
- MPC 在弯道中会限制曲率变化率（jerk/acceleration limiting）
- 这会设置 `curvature_limited = True`
- `_check_saturation` 会因为 `curvature_limited` 累积 `sat_time`
- 即使转向硬件未达到物理极限，`lac.saturated` 也会变为 `True`

### 原因3：road-edge 偏置放大问题
- ±0.002 曲率偏置在高速时会产生明显的横向加速度
- 例如：v=20 m/s 时，0.002 曲率 → 0.8 m/s² 横向加速度
- 这会增加 `desired_lateral_accel`，更容易触发 `turning > 1.0`

---

## 5. 今天的 taco_tune 优化是否有帮助

**是的，有帮助，原因：**

1. **更早减速**：taco_tune 无检测阈值，任何曲率都会限制速度
   - 进入弯道前车速已降低
   - 降低了所需的横向加速度
   - 减少触发 `turning > 1.0` 的概率

2. **连续限速**：多弯道场景中持续限制速度
   - 避免弯道间加速再减速
   - 保持稳定的低速通过
   - 减少 `desiredCurvature` 的剧烈变化

3. **安全裕度**：`-2.0 m/s` 的安全裕度
   - 实际车速低于理论最大安全速度
   - 降低对转向系统的需求
   - 减少 `curvature_limited` 触发

**但不能完全消除警告，因为：**
- 如果弯道曲率超过 0.02 m⁻¹（Ford EPS 硬件极限）
- 即使速度降低，仍可能触发硬件饱和
- 需要进一步优化或降低参数

---

## 6. 如果警告仍然频繁，建议措施

### 措施1：降低 max_lat_accel 值（低风险）
```python
# 当前值（FP原版）
max_lat_accel = np.interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])

# 建议调整为
max_lat_accel = np.interp(v_ego, [5, 10, 20], [1.2, 1.5, 2.5])
```

### 措施2：增加安全裕度（低风险）
```python
# 当前值
max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0

# 建议调整为
max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 3.0
```

### 措施3：降低 road-edge 曲率偏置（中风险）
```python
# 当前值（controlsd.py:134）
return _clamp(correction, -0.002, 0.002)

# 建议调整为
return _clamp(correction, -0.001, 0.001)
```

### 措施4：添加诊断日志（推荐先做）
在触发警告时记录：
- `desired_lateral_accel`
- `actual_lateral_accel`
- `curvature_limited` 状态
- `saturated` 状态
- 当前车速和曲率

**目的：**
- 确定是"限幅触发"还是"真转向能力不足"
- 为参数调整提供数据支持

---

## 7. 总结

**其他AI的分析100%准确：**

✅ 根因是横向控制饱和（`lac.saturated`），不是 HUD 绘制
✅ 触发条件：`undershooting > 1.2 && turning > 1.0 && lac.saturated == True`
✅ `lac.saturated` 包含 `curvature_limited`（MPC 限幅）
✅ 多弯道更易触发，因为 `desiredCurvature` 快速变化 + 执行延迟
✅ 当前分支的 road-edge 偏置（±0.002）会放大问题
✅ 今天的 taco_tune 优化会有帮助，但不能完全消除

**下一步建议：**
1. 先添加诊断日志，定位是限幅还是硬件不足
2. 如果是限幅触发，考虑降低 max_lat_accel 或增加安全裕度
3. 如果是硬件不足，考虑降低 road-edge 偏置
4. 实车验证后再决定最终参数
