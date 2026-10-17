# ford-chestnut：左右侧激光雷达融合接线说明

> 分支 `ford-chestnut`，代码提交 `9e14580`，基线 `staging-chestnut` `99fc3ce`
> 更新：2026-10-12

本文说明本版相对基线 `staging-chestnut` 做了什么、左右两侧激光雷达信号
如何进入变道 / 超车 / 避让的判定，以及调试与验证方法。

---

## 1. 信号源分工（硬约束）

| 功能 | 允许使用的传感器 |
|---|---|
| **左右两侧有车警报**（语音 / HUD 图标） | **只认原车 SOD/BSM**：`carState.leftBlindspot` / `rightBlindspot` |
| **变道阻挡**（`laneChangeBlocked`） | 原车 BSM（上游行为，未改） |
| **主动超车**（AutoOvertake, AO） | 原车雷达 `radarState.leadOne` + BSM + MR76 + **左右激光雷达** |
| **紧急转向避让**（AutoAvoidance, AA） | `radarTracks` 点云 + BSM + MR76 + **左右激光雷达** |

「左右两侧有车警报」**不得**并入 MR76 或激光雷达结果——这是为了避免
辅助传感器误报导致原厂盲区警报语义被污染。

---

## 2. 数据通道：`amapNavi`

左右两侧激光雷达经 carrot 客户端 UDP 汇总，由 `openpilot/carrot/amap_navi.py`
（`process_config.py` 中 `always_run`）发布为 `amapNavi` 消息。

### 2.1 位域 `leftBlind` / `rightBlind`

| bit | 值 | 含义 |
|---|---|---|
| bit0 | 1 | **激光雷达**盲区 |
| bit1 | 2 | **原车盲点**（BSM） |
| bit2 | 4 | **车身雷达**盲区 |
| bit3 | 8 | **车道线** ← 不是障碍物！ |

> ⚠️ 融合侧取「有障碍」时必须排除 bit3，掩码用 `0b101`（bit0 | bit2）。
> 否则「有车道线」会被误判成盲区，把变道永久堵死。

### 2.2 独立字段（本版新增）

`custom.capnp` 的 `AmapNavi` 结构新增 11 个字段，避免消费方去猜位域：

| 字段 | 类型 | 含义 |
|---|---|---|
| `lidarLeftPresent` / `lidarRightPresent` | Bool | 该侧激光雷达**是否接入过**（锁存，掉线不会翻回 False） |
| `lidarLeftValid` / `lidarRightValid` | Bool | 数据是否新鲜（`age <= 2.0 s`） |
| `lidarLeftAge` / `lidarRightAge` | Float32 | 距上次有效数据的秒数；`< 0` 表示从未收到 |
| `leftLidarObstacle` / `rightLidarObstacle` | Bool | 激光雷达盲区（= bit0） |
| `leftBodyRadarObstacle` / `rightBodyRadarObstacle` | Bool | 车身雷达盲区（= bit2） |
| `stamp` | Float64 | `time.monotonic()` 时间戳 |

`present` 采用**锁存**语义是刻意的：如果把「掉线」表达成「未接入」，
安全门会走「放行」分支，等于 WiFi 一抖就失去保护。

---

## 3. 接线：`sunnypilot/modeld_v2/modeld.py`

真正在跑的模型进程是 `modeld_tinygrad`（`NativeProcess` →
`openpilot/sunnypilot/modeld_v2/modeld.py`）。它在原有 `SubMaster` 基础上
增加订阅 `radarState` / `radarTracks` / `amapNavi`，然后：

```
amapNavi ──► 提取 present/valid/age/obstacle
                │
                ├─► AO.update( ... require_aux_sensors=True )
                │      自动超车：激光雷达与 MR76 同时参与门控
                │
                └─► AA.update( ... require_aux_sensors=False, lidar_free=... )
                       紧急避让：障碍物无条件否决，但不因掉线而整体锁死
                │
                └─► _merged_dir（AA 优先，其次 AO）──► DH.update(...)
                       最终变成一次正常的、可见转向灯、可被驾驶员取消的变道
```

### 3.1 传参映射（关键）

```python
available = True  if present else None     # 未接入 -> None（门放行）
valid     = valid   if present else None
age       = age     if present else None
obstacle  = obs     if present else None
free      = (not obs) if (present and valid) else None
```

三种情形对应的门行为：

| 情形 | 传参 | `_sensor_fresh` | 结果 |
|---|---|---|---|
| 该侧未接入 | `available=None` | — | **放行**（AO 的 `_lidar_lane_safe` 在 `available is None` 时提前 `return True`） |
| 已接入但数据过期 | `available=True, valid=False` | `False` | AO：`require_aux_sensors=True` → **fail-closed 否决** |
| 已接入且有效 | `available=True, valid=True` | `True` | 用 `obstacle` / `age` 严格判定 |

> **必须传 `None` 而不是 `False` 表示「未接入」**：`_sensor_fresh()` 把
> `available=False` 解释为「已接入但不可用」，会直接 fail-closed 把变道锁死。

### 3.2 AO 与 AA 的门为什么不一样

- **AO（自动超车）**：`require_aux_sensors=True`。超车是舒适性动作，
  传感器不健康时**不应该**做，fail-closed 是正确的。
  AO 的 `_lidar_lane_safe` 有 `available is None` 提前返回，所以没装
  激光雷达的车不会被锁死。

- **AA（紧急避让）**：`require_aux_sensors=False`，只传 `lidar_free`。
  AA 是 >80 km/h 的紧急转向兜底，若因 WiFi 激光雷达瞬时掉线就 fail-closed，
  会在最需要它的时刻把避让整体锁死。这里依赖 AA 的
  `_escape_lane_safe` 逻辑：
  - `available=None` → `lidar_ok=False` → 两道激光雷达门都弃权（行为与旧版一致）
  - `available=True, valid=True, obstacle=True` → `lidar_free=False` →
    `lidar_free is not True` → **无条件否决**

---

## 4. 本版修正的两个缺陷

### 4.1 AA 在 AO 关闭时永不生效

原 `ford-chestnut` 把 `left_ok` / `right_ok` 定义在
`if ao_cfg["enabled"] and sm.seen['radarState']:` 的 `try` 块**内部**，
而 `AutoOvertakeEnabled` 出厂默认是 `0`（关闭）。此时 AA 分支引用
`left_ok` 会抛 `NameError`，被 AA 自己的 `except Exception` 吞掉 →
`AA.reset()` → **紧急避让永久失效**，日志里只留一行
`auto avoidance update failed`。

本版把这两个变量提到 AO 分支之外计算，AA 不再依赖 AO 是否使能。

### 4.2 stock 路径的 `DH.update` 实参错位

`openpilot/selfdrive/modeld/modeld.py`（仅 `is_stock_model` 为真时运行）
在原 `ford-chestnut` 里被改成：

```python
DH.update(sm['amapNavi'], sm['carState'], sm['carControl'].latActive, ...)
```

但 `DesireHelper.update` 的签名是：

```python
update(self, carstate, lateral_active, lane_change_prob,
       left_edge_detected=False, right_edge_detected=False,
       auto_lane_change_direction=None, auto_confirm_delay_sec=None)
```

不接受 `amapNavi`，实参会整体错位并在 `carstate.vEgo` 处抛
`AttributeError`。因为本车不走 stock 路径，上游一直没暴露。
本版只在该文件补上 `amapNavi` 订阅，保留上游正确的 `DH.update` 调用。

---

## 5. 参数持久化：`/data/ford_params`

### 5.1 问题

本版沿用基线预编译的 `libparams_c.so`。`params_keys.h` 只是**声明**；
真正决定行为的是**编译进 `.so` 的那张键表**。

`openpilot/common/params.cc`：

```cpp
void Params::clearAll(ParamKeyFlag key_flag) {
  ...
  auto it = keys.find(de->d_name);
  if (it == keys.end() || (it->second.flags & key_flag)) {
    unlink(getParamPath(de->d_name).c_str());   // ← 键不在表里就删
  }
}
```

`openpilot/system/manager/manager.py` 在启动、onroad、offroad、点火时各调一次：

```python
params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)
```

所以放在 `/data/params/d/` 的福特配置**每次开机都会被静默清空**。

### 5.2 解决

`clearAll()` 只遍历**普通文件**（`de->d_type != DT_DIR` 才删），
且只遍历 params 目录。因此把福特参数放到 `/data/ford_params/`
（既不是 params 目录，也不在其下）就完全免疫。

所有福特参数的读写都把 `/data/ford_params` 排在候选列表**第一位**，
并保留 `/dev/shm/params`、`/data/params/d` 作为回退：

```python
("/data/ford_params", "/dev/shm/params", "/data/params/d")
```

涉及文件：

| 文件 | 键 |
|---|---|
| `openpilot/selfdrive/selfdrived/selfdrived.py` | `dp_radar_loss_no_disable` |
| `opendbc_repo/opendbc/car/ford/radar_interface.py` | `dp_radar_loss_no_disable` |
| `openpilot/selfdrive/car/card.py` | `AOMr76Fresh` / `AOMr76Lane`（读写）、`dp_mr76_lane_enable` 等 |
| `opendbc_repo/opendbc/car/ford/carstate.py` | `AOBsmZone` / `AOBsmFault`（写） |
| `openpilot/selfdrive/ui/onroad/hud_renderer.py` | `dp_lincoln_curve_*` |
| `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` | `dp_lincoln_stop_distance_m` |
| `openpilot/selfdrive/controls/lib/aa_brake_apply.py` | `AvoidBrakeReq`（读） |
| `openpilot/sunnypilot/modeld_v2/modeld.py` | `AutoOvertake*` / `AutoAvoidanceEnabled` / `Avoid*` / `AOMr76*` |
| `openpilot/selfdrive/controls/lib/ford_curve_speed.py`、`ford_curve_controller.py`、`ui/soundd.py`、`opendbc/.../fordcan.py`、`interface.py`、`apa_controller.py` | 原有实现，本已用 fork 目录 |

`install_scripts/install-ford-chestnut.sh` 会自动 `mkdir -p /data/ford_params`
并把 `/data/params/d` 下的历史福特配置迁移过来。

---

## 6. 常用参数

全部写在 `/data/ford_params/<键名>`，纯文本，热重载（多数 1–2 s 生效）。

### 6.1 自动超车（AO）

| 键 | 默认 | 说明 |
|---|---|---|
| `AutoOvertakeEnabled` | `0` | 总开关 |
| `AutoOvertakeLanePref` | `0` | 0=自动 1=保持左 2=保持右 |
| `AutoOvertakeConfirmSec` | `3.0` | 决策后确认延时（转向灯可见、可取消） |
| `AutoOvertakeMinCruiseKph` | `60.0` | 最低巡航速度 |
| `AutoOvertakeLaneProbMin` | `0.20` | 目标车道线概率下限 |

### 6.2 紧急避让（AA）

| 键 | 默认 | 说明 |
|---|---|---|
| `AutoAvoidanceEnabled` | `0` | 总开关 |
| `AvoidStableMode` | `ratio` | `strict` / `ratio`（容抖） |
| `AvoidObsHold` | `0.45` | 障碍物读数短时保持秒数（对抗 radarTracks 8.3 Hz） |
| `AvoidBrakeMax` | `0.0` | 纵向兜底请求上限，`0` = 关闭 |
| `AvoidBrakeGain` / `AvoidBrakeTtcRef` | `1.0` / `3.0` | 兜底缩放 / TTC 参考 |

### 6.3 雷达 / 辅助传感器

| 键 | 默认 | 说明 |
|---|---|---|
| `dp_radar_loss_no_disable` | `1` | 雷达总线掉线时**不**退出 openpilot |
| `dp_mr76_lane_enable` | `0` | MR76 相邻车道占用发布 |
| `dp_mr76_log_enable` | `0` | MR76 日志 |
| `dp_amap_lidar_enable` | `0` | carrot 激光雷达通道 |
| `dp_amap_lidar_mask` | `5` | 位域掩码（`0b101`：bit0 激光雷达 \| bit2 车身雷达，**排除 bit3 车道线**） |

---

## 7. 调试与验证

### 7.1 日志

| 路径 | 内容 | 频率 |
|---|---|---|
| `/data/media/0/auto_overtake.log` | AO 每帧决策：`en/v/vc/lp/d/vl/lok/rok/lbsm/rbsm/lp0/lp3/ao/mode/why/dh/dhd/cf/jb/js/llid/rlid/llob/rlob` | 1 Hz |
| `/data/media/0/auto_avoidance.log` | AA 每帧决策：`en/dir/mode/why/risk/od/ovr/v/sm/oh/brk/bmax/llid/rlid/llob/rlob` | 1 Hz |
| `/data/media/0/aa_brake_apply.log` | 纵向兜底落地情况 | 1 Hz |

激光雷达诊断字段：

- `llid` / `rlid` = `present * 2 + valid` → `0` 未接入、`1` 接入但过期、`3` 接入且有效
- `llob` / `rlob` = 该侧激光雷达是否报障碍

### 7.2 快速自检

```bash
# 1) 激光雷达是否在发布？
ls -l /data/ford_params/                     # 福特配置目录
grep -a amapNavi /data/params/d/../*.log 2>/dev/null

# 2) 看 AO/AA 是否在跑（mode 不应长期停在 disabled / no_radarstate）
tail -f /data/media/0/auto_overtake.log
tail -f /data/media/0/auto_avoidance.log

# 3) 确认 param 文件没有被 clearAll 清掉
ls -l /data/ford_params/dp_radar_loss_no_disable
```

### 7.3 已完成的静态验证

- 全树 **1104** 个 `.py` 语法编译通过
- `capnp` 实测加载：
  - `custom.capnp` → `AmapNavi` 13 字段全部就位
  - `log.capnp` → `Event` 155 字段，`which() == 'amapNavi'`，
    13 字段 72 字节往返值完全一致
  - 122 个类型 ID 无重复；`@154` 在 `log.capnp` 中唯一

> 动态验证（实车跑一轮并回看上面三个日志）仍需在设备上完成。
