# AI 入门指南（openpilot / dragonpilot）

本文面向需要在此分支中加入新功能的 AI 助手与开发者，梳理硬件假设、显示规格、UI 结构、控制/模型守护进程以及扩展注意事项，并列出对应代码文件以便交叉查阅。

---

## 1. 平台范围与能力

- **基础功能**：openpilot 支持自适应巡航（ACC）、自动居中（ALC）、前碰撞预警（FCW）、车道偏离预警（LDW）以及基于摄像头的驾驶员监控（docs/getting-started/what-is-openpilot.md:3-6）。
- **目标硬件**：系统默认运行在 `comma 3X` 设备上，并通过 panda 安全 MCU 与车辆 CAN 通信（docs/getting-started/what-is-openpilot.md:8；docs/concepts/glossary.md:5-9）。
- **dragonpilot 增强**：该分支增加了本地化 UI、额外安全/娱乐开关、设备管理工具等，所有新功能需兼容这些自定义参数（dragonpilot/selfdrive/ui/layouts/settings/dragonpilot.py）。

---

## 2. 硬件概览

### 2.1 设备与传感器
- `HardwareBase` 提供统一接口，涵盖 OS 信息、调制解调器/SIM、功耗、显示与热区读取等（system/hardware/base.py）。
- `system/hardware/tici/hardware.py` 实现了 comma 3X 设备：通过 dbus 控制 Wi-Fi/WWAN、管理 eSIM（`TiciLPA`）、驱动红外补光/功放等（system/hardware/tici/hardware.py:1-140）。
- panda 安全 MCU 不在本仓库内，但其状态通过 `pandaStates` 服务被 `hardwared` 监控（system/hardware/hardwared.py:1-69）。

### 2.2 硬件守护进程 `hardwared`
- 负责发布 `deviceState`、汇聚触摸输入、统计蜂窝流量、执行热管理并决定 onroad/offroad 状态（system/hardware/hardwared.py:1-190）。
- 关键线程：
  - `touch_thread`：从 `/dev/input` 读取触摸事件并写入 `touch` 消息（system/hardware/hardwared.py:46-94）。
  - `hw_state_thread`：约每 10 秒刷新网络与调制解调器状态，必要时重启 ModemManager（system/hardware/hardwared.py:96-154）。
  - `hardware_thread`：融合 panda/GPS/自驾状态，计算点火、热状态、日志空间等并触发告警（system/hardware/hardwared.py:156-315）。

### 2.3 显示、背光与触摸
- UI 缺省分辨率为 **2160 × 1080**（system/ui/lib/application.py:123-150,464；selfdrive/ui/installer/installer.cc:214）。
- `GuiApplication` 会根据 `SCALE` 环境变量缩放，点亮显示并将亮度设为 65 后再渲染帧（system/ui/lib/application.py:94-146）。
- 可选调试变量：`SHOW_FPS`、`STRICT_MODE`、`ENABLE_VSYNC`（system/ui/lib/application.py:24-43）。
- 触摸采样频率 140 Hz，由 `MouseState` 与 `hardwared` 协同处理（system/ui/lib/application.py:30-76；system/hardware/hardwared.py:46-94）。

### 2.4 联网与 SIM
- `tici.hardware` 通过 NetworkManager/ModemManager 识别以太网、Wi-Fi、蜂窝制式并获取调制解调器温度（system/hardware/tici/hardware.py:54-175）。
- eSIM 下载/切换/重命名基于 `LPABase`（system/hardware/tici/esim.py）。
- UI 网络面板 `NetworkUI + AdvancedNetworkSettings` 支持热点、APN、蜂窝/ Wi-Fi 计费策略，自身依赖 `WifiManager`（system/ui/widgets/network.py:67-275）。

### 2.5 热与功耗
- `THERMAL_BANDS` 定义了绿色/黄色/红色/危险温区（system/hardware/hardwared.py:33-60）。
- 通过 `ThermalZone` 读取温度并与 `PowerMonitoring` 结合监控设备功耗（system/hardware/base.py:12-53；system/hardware/power_monitoring.py）。
- 过热会设置 offroad 告警并阻止重新 onroad，直到温度恢复安全范围（system/hardware/hardwared.py:60-190）。

---

## 3. 软件与消息体系

### 3.1 基础通信
- 所有进程通过 `cereal.messaging` 的 Pub/Sub 服务交换数据；服务定义在 `cereal` 模块中。
- `ui_state` 订阅 `deviceState`、`carState`、`selfdriveState` 等，并向所有 widgets 提供一致的数据视图（selfdrive/ui/ui_state.py）。
- `Params`（common/params）作为持久化 KV 存储，被 UI 设置、控制器、模型等大量使用。

### 3.2 关键守护进程
- **controlsd**：融合车辆状态、规划与模型输出，计算转向/加速度并发布 `carControl`、`controlsState`、`dpControlsState`（selfdrive/controls/controlsd.py:1-205）。
- **modeld**：使用 tinygrad 执行视觉/策略模型，通过 vision IPC 读取相机帧并发出 `modelV2`、`liveParameters`（selfdrive/modeld/modeld.py:1-210）。
- **hardwared**：见上；确保硬件和系统状态可被 UI/控制模块信赖。
- 其他守护进程（loggerd、camerad、manager.py）沿用上游逻辑，负责日志、摄像头、进程编排。

### 3.3 参数与开关
- 大量 Params 直接影响控制器。例如 `dp_lat_alka` 可让控制器在非自驾状态下保持横向控制，并通过 HUD 显示（selfdrive/controls/controlsd.py:61-104,188-190）。
- `dp_lat_road_edge_detection` 等参数会传入模型守护进程构造 `RoadEdgeDetector`（selfdrive/modeld/modeld.py:33,299）。
- UI 切换项（如 Experimental Mode、记录音频、驾驶性格等）集中在 `TogglesLayout`，并根据 Param 状态控制显示/可点击性（selfdrive/ui/layouts/settings/toggles.py:17-265）。

---

## 4. UI 体系

### 4.1 渲染循环
- `selfdrive/ui/ui.py` 初始化 `GuiApplication`，进入渲染循环：每帧更新 `ui_state`、渲染布局、处理模态对话（selfdrive/ui/ui.py:4-29）。
- `gui_app` 负责字体/纹理缓存、渲染目标缩放以及触摸事件路由（system/ui/lib/application.py:123-344）。

### 4.2 MainLayout 状态机
- 包含 HOME、SETTINGS、ONROAD 三种模式，并负责侧边栏显示、点击回调与 onboarding 弹窗（selfdrive/ui/layouts/main.py:14-110）。
- 旗帜按钮会发布 `bookmarkButton` 消息；设置入口可跳至指定 `PanelType`（selfdrive/ui/layouts/main.py:24-110）。
- Onboarding 流程（条款 + 培训）使用 `OnboardingWindow` 作为模态覆盖层（selfdrive/ui/layouts/main.py:38-47；selfdrive/ui/layouts/onboarding.py:169-220）。

### 4.3 Sidebar
- 显示网络、温度、车辆连接状态、麦克风提示以及设置/书签按钮，颜色/图标由实时数据驱动（selfdrive/ui/layouts/sidebar.py:65-211）。
- 当 `dp_dev_disable_connect` 为真时会隐藏连接状态指标（dragonpilot/selfdrive/ui/layouts/settings/dragonpilot.py:275-281）。

### 4.4 Home / Offroad
- `HomeLayout` 提供通知头、Prime 广告、Firehose 引导、Experimental 模式入口等（selfdrive/ui/layouts/home.py:27-211）。
- `UpdateAlert` 负责版本更新/发布说明，`OffroadAlert` 汇总离线警报并提供延迟按钮（selfdrive/ui/widgets/offroad_alerts.py:78-337）。
- `ExperimentalModeButton` 与 `SetupWidget` 分别控制模式切换与配对/Firehose 快捷入口（selfdrive/ui/widgets/exp_mode_button.py:8-53；selfdrive/ui/widgets/setup.py:14-86）。

### 4.5 Settings 面板
- `SettingsLayout` 左侧为导航按钮，右侧载入不同 panel（selfdrive/ui/layouts/settings/settings.py:37-171）：
  - **Device**：配对、校准、驾驶员摄像头、On/Off Road 强制切换、车型选择、重启/关机等（selfdrive/ui/layouts/settings/device.py:44-309）。
  - **Network / Advanced**：Wi-Fi 列表、隐藏网络、APN、蜂窝/ Wi-Fi 计费、热点开关等（system/ui/widgets/network.py:67-274）。
  - **Toggles**：总开关、Experimental Mode、LDW、监控、录音/录像、单位、驾驶性格等（selfdrive/ui/layouts/settings/toggles.py:17-265）。
  - **Software**：检查/下载/安装更新、分支切换、卸载（selfdrive/ui/layouts/settings/software.py:48-170）。
  - **Firehose**：解释数据上传策略、统计段数与 FAQ（selfdrive/ui/layouts/settings/firehose.py:18-134）。
  - **Developer**：ADB/SSH、Joystick/Longitudinal Maneuver、Alpha Longitudinal、错误日志查看器（selfdrive/ui/layouts/settings/developer.py:45-188）。
  - **Dragonpilot**：提供以下细分功能，全部通过 `Params` 持久化，UI 由 `toggle_item` / `spin_button_item` 等生成（dragonpilot/selfdrive/ui/layouts/settings/dragonpilot.py:34-306）：
    - **车厂专属**：Toyota（门锁自动锁/解锁、TSS1 SnG、原厂纵向切换）、Volkswagen（MQB A0 SnG、PQ Steering Patch、EPS Lockout 保护）、Mazda 预留标题。实现方式是按品牌匹配 `ui_state.CP.brand` 后注册对应 toggles，并写入 `dp_toyota_* / dp_vag_*` 等参数（同文件第 51-101 行）。
    - **横向控制**：Always-on LKA（`dp_lat_alka`）、LCA 触发速度与自动超时（`dp_lat_lca_speed`、`dp_lat_lca_auto_sec`）、Road Edge Detection（`dp_lat_road_edge_detection`）。这些值直接影响 `controlsd` 是否在非 selfdrive 状态下保持转向，以及 `modeld` 中的 `RoadEdgeDetector`（selfdrive/controls/controlsd.py:61-109；selfdrive/modeld/modeld.py:33,299）。
    - **纵向控制**：外置雷达（`dp_lon_ext_radar`）、Adaptive Coasting Mode（`dp_lon_acm`）、Adaptive Experimental Mode（`dp_lon_aem`）。弯道减速已改为 Lincoln/Ford 专用开关 `dp_lincoln_curve_speed`，直接收紧纵向加速度上限实现预减速（selfdrive/controls/lib/longitudinal_planner.py）。
    - **UI 扩展**：HUD 隐藏阈值（`dp_ui_hide_hud_speed_kph`，与 Onroad HUD 渲染挂钩 selfdrive/ui/onroad/augmented_road_view.py:99-111）、彩虹轨迹（`dp_ui_rainbow`）、Lead/Radar 显示模式（`dp_ui_lead`）、显示亮灭逻辑（`dp_ui_display_mode`）、告警音模式（`dp_dev_audible_alert_mode`）。这些参数在 UI 渲染阶段通过 `ui_state.params` 读取。
    - **设备/服务**：右舵模式（`dp_dev_is_rhd`，与 DriverStateRenderer 同步）、禁用监控（`dp_dev_monitoring_disabled`）、蜂鸣器（`dp_dev_beep`）、自动关机（`dp_dev_auto_shutdown_in`）、dashy 模式（`dp_dev_dashy`）、loggerd 延迟（`dp_dev_delay_loggerd`）、禁用 connect（`dp_dev_disable_connect`，Sidebar 中有条件隐藏连接指标）、一键重置（按钮 `btn_reset_dp_conf` 触发 ConfirmDialog，写入 `dp_dev_reset_conf` 后请求重启）。
  - **Lincoln**：新近增加的品牌专属面板，集中处理美版车型需求（selfdrive/ui/layouts/settings/lincoln.py:30-210）：
    - **盲点语音提示**：左右盲点触发 `dp_lincoln_bsm_voice_enabled`、间隔与音量 (`dp_lincoln_bsm_voice_interval_sec/pct`)，后端由 `soundd.py` 播放 JetBrains mono 语音包（selfdrive/ui/soundd.py:78-170）。
    - **人工转弯检测 (HTD)**：开关与触发角度写入 `dp_htd_*`，控制器通过 `HumanTurnDetection` 状态机监控驾驶员扭矩并在大弯道优雅让渡/恢复横向控制（dragonpilot/selfdrive/controls/lib/human_turn_detection.py; selfdrive/controls/controlsd.py:60-140）。
    - **HUD 性能条**：`dp_lincoln_perf_info_enabled` 触发底部性能条，显示曲率/方向盘/扭矩、方位、控制状态、内存、CPU 温度，渲染逻辑在 `AugmentedRoadView._draw_performance_info`（selfdrive/ui/onroad/augmented_road_view.py:250-410，详见 docs/林肯面板性能显示.md）。
    - **NAS 管理**：通过 NAS (Synology) 配置、状态、上传/删除按钮写入 `LincolnNAS*`、`NasSsh*` 参数，后台调用 `selfdrive/ui/tools/lincoln_media_manager.py` 完成 SCP 上传（selfdrive/ui/layouts/settings/lincoln.py:90-210）。

### 4.6 Onroad 视图
- `AugmentedRoadView` 合成摄像头画面、模型轨迹、HUD、驾驶员监控、告警以及 DP 边框指示灯（selfdrive/ui/onroad/augmented_road_view.py:40-301）。
+- `HudRenderer` 显示 MAX/当前速度、单位、实验模式按钮以及车距人格信息（selfdrive/ui/onroad/hud_renderer.py:21-140）。
- `AlertRenderer` 渲染多尺寸告警并监控自驾状态超时（selfdrive/ui/onroad/alert_renderer.py:14-150）。
- `DriverStateRenderer` 展示驾驶员面部关键点、左右弧线，并支持右舵模式（selfdrive/ui/onroad/driver_state.py:11-205）。
- `ExpButton`（onroad）允许驾驶员在满足条件时切换 Experimental Mode（selfdrive/ui/onroad/exp_button.py:8-66）。

### 4.7 Widgets 与资源
- 通用组件位于 `system/ui/widgets/`，所有 UI 类均继承 `Widget`，方便统一管理输入与渲染。
- 字体基于 Inter 系列，并在需要时回退至 Unifont（system/ui/lib/application.py:49-87）。

---

## 5. 控制与模型流水线

### 5.1 `controlsd`
- 初始化 `CarParams` 与车辆接口，订阅 planner/model 服务（selfdrive/controls/controlsd.py:1-60）。
- 根据 `VehicleModel` 计算曲率，将纵向/横向控制结果写入 `carControl`，并处理 Panda 安全限幅（selfdrive/controls/controlsd.py:60-170）。
- 附带 `HumanTurnDetection`：当 `dp_htd_enabled` 打开时会在角度控制前调用 `HumanTurnDetection.update()`，若检测到驾驶员扭矩超过阈值则暂时关闭 `latActive` 并扩展状态到 `dpControlsState`（dragonpilot/selfdrive/controls/lib/human_turn_detection.py；selfdrive/controls/controlsd.py:61-118）。
- 额外发布 `dpControlsState`，向 UI/HUD 广播 ALKA 激活等状态（selfdrive/controls/controlsd.py:173-205）。

### 5.2 `modeld`
- 根据硬件环境选择计算后端（QCOM/CPU/AMD），加载 tinygrad 序列化模型（selfdrive/modeld/modeld.py:1-80）。
- 维护输入缓冲、平滑期望加速度/曲率并生成 `modelV2` 消息供 planner/controls/UI 使用（selfdrive/modeld/modeld.py:95-210）。
- 若 `dp_lat_road_edge_detection` 启用，将实例化 `RoadEdgeDetector` 以辅助 LCA/提醒（selfdrive/modeld/modeld.py:33,299）。

### 5.3 Param 钩子
- 某些开关（Experimental Mode、Alpha Longitudinal 等）会弹出确认框，确认后写入 Param，并可能请求 `OnroadCycleRequested` 或重新引导（selfdrive/ui/layouts/settings/toggles.py:202-265；selfdrive/ui/layouts/settings/developer.py:138-188）。
- Dragonpilot 面板写入的设备/UI 参数（如 dashy、显示模式、禁用 connect、右舵模式等）均由 UI 渲染层通过 `ui_state.params` 读取；横纵向参数则被 `controlsd`、`modeld`、HUD 等模块消费（dragonpilot/selfdrive/ui/layouts/settings/dragonpilot.py:205-281；selfdrive/controls/controlsd.py:61-109；selfdrive/ui/onroad/driver_state.py:22-75）。
- 新增 Param 时需同步：UI 输入、控制/模型使用方以及任何依赖该状态的守护进程。

### 5.4 Ford/Lincoln 非 CAN FD 弯道与安全要点
- **覆盖车型（仅非 CAN FD）**：Bronco Sport 21-24、Escape/Kuga 20-22、Explorer 20-24 / Lincoln Aviator、Focus Mk4 18、Maverick 22-24（opendbc_repo/opendbc/car/ford/values.py:90-118,129-151）。这些平台共用 Q3 线束与 `ford_lincoln_base_pt` DBC，默认雷达 Delphi MRR。
- **命令链路**：Ford 控制在 20 Hz 发送 `LateralMotionControl`，只用偏移/角度/曲率三系数（opendbc_repo/opendbc/car/ford/carcontroller.py:101-129），对应 DBC 的 `LatCtlCurv_No_Actl`/`LatCtlCurv_NoRate_Actl`（opendbc_repo/opendbc/dbc/ford_lincoln_base_pt.dbc:3374-3383）。非 CAN FD 不用 `LateralMotionControl2`。
- **硬性曲率上限**：信号量程 ±0.02 m⁻¹（半径≈50 m，DBC 同上），控制参数 `ANGLE_LIMITS.max_curvature` 也固定 0.02（opendbc_repo/opendbc/car/ford/values.py:26-36），模型更大曲率会被截断。
- **额外限幅**：速度 >9 m/s 时，曲率被夹到当前测得曲率 ±0.002 m⁻¹，再应用 EPS 曲率/曲率变化率限幅（opendbc_repo/opendbc/car/ford/carcontroller.py:35-51）；Bronco/F-150 还有抗过冲滤波（同文件:101-109）。
- **安全策略/锁死**：非 CAN FD 平台启动时检查 EPS 固件是否开放 TJA/LCA，缺失直接 `dashcamOnly`（opendbc_repo/opendbc/car/ford/interface.py:46-79）。CarState 会把方向柱补偿质量码异常或 `EPAS_INFO` 报错标记成转向故障（opendbc_repo/opendbc/car/ford/carstate.py:29-53），事件层转换成 `steerTempUnavailable` 等告警（selfdrive/car/car_specific.py:136-160）。
- **纵向可用性**：默认使用原厂 ACC，若启用 Alpha Long 或外接雷达时会在 Panda 安全配置中加入 `FordSafetyFlags.LONG_CONTROL` 以允许 openpilot 纵向（opendbc_repo/opendbc/car/ford/interface.py:32-56）。
- **用户感知问题**：紧弯/匝道（曲率>0.02 m⁻¹）会饱和并触发上述限幅，表现为提前退出横向或频繁告警，这是物理量程与安全保护共同作用。
- **信息性改进方向**：若需更大弯道能力，需硬件/DBC 支持更大曲率或在非 CAN FD 上组合路径角度/偏移；任何提升前要复核 Panda 安全限幅与 EPS 保护。
- **安全策略补充**：
  - Panda 安全：默认 `SafetyModel.ford`，若主干在 bus>=4 先放置 `noOutput` 禁止对摄像头发送；开启纵向或 CANFD 时分别置位 `FordSafetyFlags.LONG_CONTROL/CANFD`（opendbc_repo/opendbc/car/ford/interface.py:46-59）。
  - SecOC 拦截：CAN FD 平台若检测到摄像头总线报文长度非 8，视为 SecOC，直接 `dashcamOnly`（同文件:60-66）。
  - EPS 功能位校验：非 CAN FD 读取 EPS 配置字节，若 TJA/LCA 未打开或固件长度异常则锁定为行车记录仪模式（同文件:67-79）。
  - 转向容忍：驾驶员反扭矩阈值 1 Nm 视为接管（opendbc_repo/opendbc/car/ford/values.py:24；carstate.py:48-51），配合曲率/曲率变化率限幅减少过控。
