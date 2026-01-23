# SP 模型管理体系复刻文档（当前落地版 · 100% 复刻清单）

本文件是 **我们项目中已落地的模型管理体系** 的完整复刻说明（以 `e:\openpilot` 当前工作区为准）。
目标：任何开发者或 AI **仅按本文档即可 1:1 复刻全部功能**，不遗漏文件、参数、行为或边界条件。

- 参考源：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt`
- 当前模型清单源（临时）：
  - `https://raw.githubusercontent.com/sunnypilot/sunnypilot-docs/refs/heads/gh-pages/docs/driving_models_v10.json`
- 原则：**不复用 SP 代码**，只复刻结构、协议与行为。

---

## 0. 复刻范围（必须全部包含）

必须实现的模块：
- 模型清单服务（JSON 协议 + 拉取/缓存）
- 模型下载与校验（artifact + metadata）
- 模型切换与回退（ActiveBundle + Runner 选择）
- 多运行器进程调度（stock / tinygrad / snpe）
- UI 模型管理面板（选择、下载进度、取消、清缓存、收藏/搜索）
- 高级控制项（LaneTurn + Lagd）

---

## 1. 文件清单（一个都不能遗漏）

### 1.1 协议与服务
- `cereal/custom.capnp`：新增 `ModelManagerSP` 结构体与枚举。
- `cereal/log.capnp`：新增 `modelManagerSP @109 :Custom.ModelManagerSP`。
- `cereal/services.py`：新增服务 `modelManagerSP`，频率 1Hz（`should_log=False`）。

### 1.2 参数键
- `common/params_keys.h`：新增所有 `ModelManager_*`、`ModelRunnerTypeCache`、`ShowAdvancedControls`、`Lagd*`、`LaneTurn*` 键。

### 1.3 模型管理核心
- `selfdrive/modeld/model_manager.py`：模型管理主进程（下载、缓存、状态发布）。
- `selfdrive/modeld/model_manager_helpers.py`：兼容性判断、路径解析、Runner 选择、bundle 解析。

### 1.4 模型运行器与调度
- `system/manager/process_config.py`：选择运行器，启动 `modeld` / `modeld_snpe` / `models_manager`。
- `selfdrive/modeld/modeld.py`：tinygrad split 模型加载与回退（stock 与 tinygrad 都使用此进程）。
- `selfdrive/modeld/modeld_snpe.py`：supercombo(THNEED/ONNX) 加载与回退。
- `selfdrive/modeld/runners/__init__.py`：`ModelRunner` 选择 THNEED/ONNX。
- `selfdrive/modeld/runners/onnxmodel.py`：ONNXRuntime 运行器。
- `selfdrive/modeld/runners/run.h`：C++ runner 头。

### 1.5 构建系统
- `selfdrive/modeld/SConscript`：thneed 编译、QCOM2 宏、tinygrad 编译模型。

### 1.6 UI 与交互
- `selfdrive/ui/layouts/settings/settings.py`：设置面板增加 “Models”。
- `selfdrive/ui/layouts/settings/model_manager.py`：模型面板 UI（完整功能）。
- `selfdrive/ui/ui_state.py`：订阅 `modelManagerSP` 与 `liveDelay`。
- `system/ui/widgets/tree_dialog.py`：模型树选择 + 收藏 + 搜索。
- `system/ui/widgets/progress_bar.py`：进度条 UI。
- `system/ui/widgets/input_dialog.py`：输入对话框（搜索用）。
- `selfdrive/ui/translations/app_zh-CHS.po`：模型面板中文翻译（必须完整，避免 “???”）。

### 1.7 高级控制项（模型面板内的附属功能）
- `selfdrive/controls/lib/lane_turn_desire.py`
- `selfdrive/controls/lib/desire_helper.py`
- `selfdrive/livedelay/helpers.py`
- `selfdrive/livedelay/lagd_toggle.py`
- `selfdrive/locationd/lagd.py`
- `selfdrive/locationd/torqued.py`
- `selfdrive/controls/controlsd.py`
- `selfdrive/modeld/modeld.py`
- `selfdrive/modeld/modeld_snpe.py`

### 1.8 路径扩展
- `system/hardware/hw.py`：新增 `Paths.model_root()`。

---

## 2. 模型清单 JSON 协议（driving_models_v10.json）

### 2.1 顶层结构
- `tinygrad_ref`：tinygrad 编译基线哈希（只做对齐提示）
- `bundles`：模型包列表（核心）

### 2.2 Bundle 字段（JSON）
- `index`：下载索引（UI 选择后写入 `ModelManager_DownloadIndex`）
- `short_name` / `display_name`
- `runner`：`snpe` / `tinygrad` / `stock`（必须与 Capnp 枚举名一致）
- `generation` / `environment` / `is_20hz` / `minimum_selector_version`
- `ref`：唯一标识（UI Tree 选择与收藏 key）
- `overrides`：字典（常用 `folder`、`lat`、`long`）
- `models`：模型列表（`supercombo` / `vision` / `policy` / `navigation`）

### 2.3 Model 字段（JSON）
- `type`：`supercombo` / `vision` / `policy` / `navigation`
- `artifact.file_name`
- `artifact.download_uri.url`
- `artifact.download_uri.sha256`
- `metadata.*` 同上（可选，但 tinygrad split 和 supercombo 必须提供）

### 2.4 JSON -> Capnp 字段映射（必须严格一致）
- `download_uri.url` → `DownloadUri.uri`
- `file_name` → `Artifact.fileName`
- `minimum_selector_version` → 写入 Capnp 后变为 `minimumSelectorVersion`
- `short_name` → `internalName`
- `display_name` → `displayName`

> 注意：`ModelManager_ActiveBundle` 保存的是 **Capnp 的 `to_dict()` 结果**，字段是 camelCase（如 `minimumSelectorVersion`），不是 JSON 的 snake_case。

### 2.5 JSON 示例（骨架）
```json
{
  "tinygrad_ref": "855f5e4d...",
  "bundles": [
    {
      "index": 63,
      "short_name": "WMIV10",
      "display_name": "WMI v10 (January 09, 2026)",
      "runner": "tinygrad",
      "generation": 12,
      "environment": "development",
      "is_20hz": true,
      "minimum_selector_version": 12,
      "ref": "855f5e4d...",
      "overrides": {"folder": "2026 World Models", "lat": ".1", "long": ".3"},
      "models": [
        {
          "type": "vision",
          "artifact": {"file_name": "driving_vision_wmiv10_tinygrad.pkl", "download_uri": {"url": "...", "sha256": "..."}},
          "metadata": {"file_name": "driving_vision_wmiv10_metadata.pkl", "download_uri": {"url": "...", "sha256": "..."}}
        },
        {
          "type": "policy",
          "artifact": {"file_name": "driving_policy_wmiv10_tinygrad.pkl", "download_uri": {"url": "...", "sha256": "..."}},
          "metadata": {"file_name": "driving_policy_wmiv10_metadata.pkl", "download_uri": {"url": "...", "sha256": "..."}}
        }
      ]
    }
  ]
}
```

---

## 3. Cap'n Proto 结构（ModelManagerSP）

### 3.1 结构体与枚举（`cereal/custom.capnp`）
- `ModelManagerSP`
  - `activeBundle` / `selectedBundle` / `availableBundles`
- `DownloadUri`：`uri`, `sha256`
- `DownloadStatus`：`notDownloading=0`, `downloading=1`, `downloaded=2`, `cached=3`, `failed=4`
- `DownloadProgress`：`status`, `progress`(0~100), `eta`(秒)
- `Model`：`type`, `artifact`, `metadata`
  - `Type`：`supercombo=0`, `navigation=1`, `vision=2`, `policy=3`
- `Runner`：`snpe=0`, `tinygrad=1`, `stock=2`
- `ModelBundle`：`index`, `internalName`, `displayName`, `models`, `status`, `generation`, `environment`,
  `runner`, `is20hz`, `ref`, `minimumSelectorVersion`, `overrides`

### 3.2 事件挂载（`cereal/log.capnp`）
- `modelManagerSP @109 :Custom.ModelManagerSP;`

### 3.3 Service（`cereal/services.py`）
- `"modelManagerSP": (False, 1.),`

---

## 4. Params 键与含义（必须全部存在）

**模型管理相关**（`common/params_keys.h`）：
- `ModelManager_ModelsCache` (JSON, PERSISTENT)
- `ModelManager_LastSyncTime` (INT, PERSISTENT, 默认 0，单位 ns，`time.monotonic()`)
- `ModelManager_ActiveBundle` (JSON, PERSISTENT，保存 capnp.to_dict 结果)
- `ModelManager_DownloadIndex` (INT, CLEAR_ON_MANAGER_START)
- `ModelManager_ClearCache` (BOOL, CLEAR_ON_MANAGER_START)
- `ModelManager_DeleteBundleRef` (STRING, CLEAR_ON_MANAGER_START)
- `ModelManager_Favs` (STRING, PERSISTENT，`;` 分隔)
- `ModelRunnerTypeCache` (INT, PERSISTENT, 默认 2=stock)

**UI 高级控制相关**（模型面板内显示）：
- `ShowAdvancedControls` (BOOL, PERSISTENT, 默认 1)
- `LagdToggle` (BOOL, PERSISTENT, 默认 1)
- `LagdToggleDelay` (FLOAT, PERSISTENT, 默认 0.2)
- `LagdValueCache` (FLOAT, PERSISTENT)
- `LaneTurnDesire` (BOOL, PERSISTENT, 默认 0)
- `LaneTurnValue` (FLOAT, PERSISTENT, 默认 19.0，单位 mph)

---

## 5. 模型存储路径

- `system/hardware/hw.py` 新增：`Paths.model_root()`
- 设备端：`/data/models`
- PC：`~/.comma/models`

说明：下载文件全部直接写入该目录，不分子目录。

---

## 6. Model Manager 进程（`selfdrive/modeld/model_manager.py`）

### 6.1 基本行为
- 进程名：`models_manager`（见 `process_config.py`）
- 仅 offroad 运行（`only_offroad`）
- 1Hz 循环（`Ratekeeper(1)`）

### 6.2 关键常量
- `MODEL_URL`：模型清单 JSON 地址
- `CACHE_TIMEOUT_NS = 3600 * 1e9`（1 小时）
- `CHUNK_SIZE = 128 * 1024`

### 6.3 JSON -> Capnp 解析（ModelParser）
- `download_uri.url` → `DownloadUri.uri`
- `file_name` → `Artifact.fileName`
- `runner` / `type` 必须是 **枚举名字符串**（不匹配会导致 capnp 赋值异常）
- 未提供 `runner` → 默认 `snpe`（注意：这会触发 `modeld_snpe`）

### 6.4 缓存逻辑（ModelCache）
- JSON 缓存在 `ModelManager_ModelsCache`
- 时间写入 `ModelManager_LastSyncTime`（monotonic ns）
- 超时后拉取新清单，否则复用缓存

### 6.5 拉取逻辑（ModelFetcher）
- 缓存未过期 → 直接解析
- 缓存过期 → `requests.get(MODEL_URL, timeout=10)`
- 拉取失败 → 回退到旧缓存
- 404 → 报错并进入异常分支

### 6.6 下载流程（串行、阻塞）
- 触发：`ModelManager_DownloadIndex`
- **索引为 0 不会触发**（当前使用 `if index_to_download := ...` 的真值判断）
- 顺序：先下载 `metadata`，再下载 `artifact`
- 每个 artifact 维护自己的 `DownloadProgress`
- sha256 不匹配 → 抛错并标记 `failed`
- 取消下载：删除 `ModelManager_DownloadIndex`（每个 chunk 轮询）

### 6.7 下载进度细节
- 进度计算依赖 `content-length`；缺失时不会更新进度（仍可下载完成）
- `progress` 为 0~100 百分比
- `eta` 为秒，无法估算时默认 60
- 文件存在且 hash 匹配 → `status=cached`, `progress=100`, `eta=0`

### 6.8 Active/Selected 状态
- `selectedBundle`：下载中（失败时仍保留，供 UI 显示）
- `activeBundle`：下载完成后设置
- 成功后：`params.put(ModelManager_ActiveBundle, bundle.to_dict())`

### 6.9 清理逻辑
- `ModelManager_ClearCache`：删除所有非 active bundle 文件
- `ModelManager_DeleteBundleRef`：删除指定 bundle 文件；
  若删除的是 active bundle → 清空 `ModelManager_ActiveBundle` 并把 `ModelRunnerTypeCache` 置回 stock
- 删除仅针对 **文件**，不会删除目录

### 6.10 主循环顺序（每秒）
1) 拉取/解析可用 bundles
2) 读取 active bundle
3) 若有 `ModelManager_DownloadIndex` → 执行下载
4) 若 `ModelManager_ClearCache` → 清缓存
5) 若 `ModelManager_DeleteBundleRef` → 删除指定 bundle
6) 发布 `modelManagerSP`

---

## 7. Helpers（`selfdrive/modeld/model_manager_helpers.py`）

### 7.1 版本兼容
- `CURRENT_SELECTOR_VERSION = 13`
- `REQUIRED_MIN_SELECTOR_VERSION = 12`
- `minimumSelectorVersion` 不在范围内的 bundle 会被过滤

### 7.2 Bundle 解析
- `_coerce_bundle`：支持 dict 与 capnp 对象（禁止 `isinstance(capnp_struct)`）
- `_enum_raw`：处理 `capnp._DynamicEnum` 的 `.raw`

### 7.3 Runner 选择
- `get_active_model_runner()`：
  - 优先 `ModelRunnerTypeCache`（onroad）
  - offroad 时强制重算（`force_check=True`）
  - 未设置 active → stock

### 7.4 路径解析
- `get_tinygrad_bundle_paths()`：vision/policy split
- `get_supercombo_bundle_paths()`：supercombo

---

## 8. 运行器选择与进程调度（`system/manager/process_config.py`）

### 8.1 运行器类型
- `custom.ModelManagerSP.Runner.snpe`
- `custom.ModelManagerSP.Runner.tinygrad`
- `custom.ModelManagerSP.Runner.stock`

### 8.2 进程调度
- `models_manager`：只在 offroad 运行
- `modeld`：在 onroad 且 runner=stock 或 tinygrad 时运行
- `modeld_snpe`：在 onroad 且 runner=snpe 时运行

> 当前工程 **没有独立的 `modeld_tinygrad` 进程**，stock 与 tinygrad 均由 `modeld` 处理。

---

## 9. Modeld 接入（tinygrad / snpe）

### 9.1 tinygrad 路线（`selfdrive/modeld/modeld.py`）
- `_resolve_tinygrad_paths()`：优先使用下载模型（bundle）
- 若加载失败（pickle/metadata 不兼容） → 清空 `ModelManager_ActiveBundle` 并回退内置模型
- 内置默认：
  - `selfdrive/modeld/models/driving_vision_tinygrad.pkl`
  - `selfdrive/modeld/models/driving_policy_tinygrad.pkl`
  - `selfdrive/modeld/models/driving_vision_metadata.pkl`
  - `selfdrive/modeld/models/driving_policy_metadata.pkl`

> 注意：tinygrad `.pkl` 必须与当前 tinygrad 版本兼容，否则会出现 `Ops.VIEW` / `tinygrad.shape` 等错误。

### 9.2 snpe 路线（`selfdrive/modeld/modeld_snpe.py`）
- `_resolve_supercombo_paths()`：优先 bundle；否则 fallback 本地 `supercombo.thneed`/`supercombo.onnx`
- 若无可用模型 → 清空 `ModelManager_ActiveBundle` 并抛错
- 使用 `overrides`：`lat`/`long` 替换平滑参数

---

## 10. Runner 实现（`selfdrive/modeld/runners`）

- `__init__.py`：`ModelRunner` 在 THNEED 与 ONNX 之间选择
  - `USE_THNEED` 默认在 TICI 设备开启
- `onnxmodel.py`：ONNXRuntime
  - 自动选择 OpenVINO/CUDA/CPU
  - FP16 → FP32 转换
- `thneedmodel_pyx`：由 `SConscript` 构建

---

## 11. 构建系统（`selfdrive/modeld/SConscript`）

### 11.1 tinygrad 编译
- 依赖 `tinygrad_repo/examples/openpilot/compile3.py`
- 生成 `*_tinygrad.pkl`
- flags：
  - `larch64`: `DEV=QCOM FLOAT16=1 NOLOCALS=1 IMAGE=2 JIT_BATCH_SIZE=0`
  - `Darwin`: `DEV=CPU HOME=...`
  - 其他：`DEV=CPU CPU_LLVM=1`

### 11.2 thneed 构建
- `arch == larch64` 时添加 `-DQCOM2`（修复 thneed_qcom2.cc 缺字段）
- `GetOption('pc_thneed')` 可在 PC 编译

---

## 12. UI 模型管理面板（完整功能清单）

### 12.1 面板注册
- `selfdrive/ui/layouts/settings/settings.py`：添加 “Models” 入口

### 12.2 面板控件顺序（从上到下）
1) `Current Model`（按钮）
2) `Cancel Download`（按钮，下载中才显示）
3) `Driving Model`（进度行）
4) `Vision Model`（进度行）
5) `Policy Model`（进度行）
6) `Refresh Model List`（按钮）
7) `Clear Model Cache`（按钮，右侧显示缓存大小）
8) `Use Lane Turn Desires`（开关）
9) `Adjust Lane Turn Speed`（数值）
10) `Live Learning Steer Delay`（开关）
11) `Adjust Software Delay`（数值）

### 12.3 关键行为（1:1）

**1) 当前模型选择**
- 点击 “Current Model” 弹出 TreeOptionDialog
- 选 “Default” → 清空 `ModelManager_ActiveBundle` + `ModelRunnerTypeCache` 置 stock，并触发“重置校准”提示
- 选 bundle → 写 `ModelManager_DownloadIndex`
- 若 `generation` 与当前 active 不一致 → 弹出重置校准确认

**2) 下载进度显示**
- 行项目：`Driving Model` / `Vision Model` / `Policy Model`
- 仅显示 `artifact` 进度（metadata 不展示）
- 文案状态：
  - `pending - <displayName>`
  - `<pct>% - <displayName>`
  - `<displayName> - downloaded / from cache / ready`
  - `download failed - <displayName>`
- 进入下载状态会 `device.reset_interactive_timeout(300)` 防止休眠

**3) 取消下载**
- 按钮 `Cancel Download`
- 行为：删除 `ModelManager_DownloadIndex`

**4) 刷新模型列表**
- 按钮 `Refresh Model List`
- 行为：`ModelManager_LastSyncTime = 0`
- 弹窗：`Fetching Latest Models`

**5) 清理缓存**
- 按钮 `Clear Model Cache`
- 行为：设置 `ModelManager_ClearCache = True`
- 确认框：删除除 active 以外的模型
- UI 右侧显示当前缓存总占用（每 0.5s 刷新）

**6) 收藏与搜索**
- TreeDialog 支持收藏（星标）
- 收藏保存到 `ModelManager_Favs`（`;` 分隔）
- 搜索弹出 InputDialog（匹配 display/short name）

**7) Onroad 限制**
- Onroad 时禁用“Current Model”按钮
- 提示文案：`Only available when vehicle is off, or always offroad mode is on`

**8) 重置校准提示**
- 弹窗文案：`Model download has started in the background. We suggest resetting calibration. Would you like to do that now?`
- 用户确认才会删除 `CalibrationParams` 与 `LiveTorqueParameters`

### 12.4 TreeDialog 行为细节（`system/ui/widgets/tree_dialog.py`）
- 分组依据：`overrides.folder`
- `Favorites` 组插入第 2 位
- `Default Model` 固定存在，ref = `Default`
- 搜索逻辑：NFKD 归一化 → 仅保留 a-z0-9 → 所有关键词必须匹配

---

## 13. UI 英文字符串清单（必须完全一致）

来自 `selfdrive/ui/layouts/settings/model_manager.py` 与 `tree_dialog.py`：
- `Current Model`
- `SELECT`
- `Driving Model`
- `Vision Model`
- `Policy Model`
- `Refresh Model List`
- `REFRESH`
- `Fetching Latest Models`
- `Clear Model Cache`
- `CLEAR`
- `Cancel Download`
- `Cancel`
- `This will delete ALL downloaded models from the cache except the currently active model. Are you sure?`
- `Clear Cache`
- `Use Lane Turn Desires`
- `If you're driving at 20 mph (32 km/h) or below and have your blinker on, the car will plan a turn at the nearest drivable path.`
- `Adjust Lane Turn Speed`
- `Set the maximum speed for lane turn desires. Default is 19 mph.`
- `Live Learning Steer Delay`
- `Adjust Software Delay`
- `Adjust the software delay when Live Learning Steer Delay is toggled off. The default software delay value is 0.2`
- `Select a Model`
- `Default Model`
- `Favorites`
- `Search`
- `Enter search query`
- `Only available when vehicle is off, or always offroad mode is on`
- `pending - {displayName}`
- `{pct}% - {displayName}`
- `{displayName} - downloaded`
- `{displayName} - from cache`
- `{displayName} - ready`
- `download failed - {displayName}`
- `Reset Calibration`
- `Model download has started in the background. We suggest resetting calibration. Would you like to do that now?`
- `Live Steer Delay:`
- `Actuator Delay:`
- `Software Delay:`
- `Total Delay:`

> 以上字符串必须在 `selfdrive/ui/translations/app_zh-CHS.po` 中有完整翻译。

---

## 14. 高级控制项（模型面板内功能）

### 14.1 LaneTurn
- 参数：`LaneTurnDesire` / `LaneTurnValue`
- UI：
  - `Use Lane Turn Desires` 开关
  - `Adjust Lane Turn Speed` 数值控件
- 范围：5~20 mph
- 默认：19 mph
- 逻辑：低速 + 打灯 + 无盲区 → `turnLeft/turnRight`
- 文件：
  - `selfdrive/controls/lib/lane_turn_desire.py`
  - `selfdrive/controls/lib/desire_helper.py`

### 14.2 Lagd
- 参数：`LagdToggle` / `LagdToggleDelay` / `LagdValueCache`
- UI：
  - `Live Learning Steer Delay` 开关
  - `Adjust Software Delay` 数值控件
- 范围：0.05~0.50 s
- 默认：0.20 s
- 逻辑：
  - 开启 → 使用 `liveDelay` 的估计值
  - 关闭 → `steerActuatorDelay + LagdToggleDelay`
- 文件：
  - `selfdrive/livedelay/helpers.py`
  - `selfdrive/livedelay/lagd_toggle.py`
  - `selfdrive/locationd/lagd.py`
  - `selfdrive/locationd/torqued.py`
  - `selfdrive/controls/controlsd.py`
  - `selfdrive/modeld/modeld.py`
  - `selfdrive/modeld/modeld_snpe.py`

### 14.3 高级控件显示规则
- `ShowAdvancedControls = True` 才显示数值调节项

---

## 15. 已修复的关键错误（必须记录）

1) **Capnp enum 转换错误**
- 现象：`TypeError: int() argument must be a string... not capnp._DynamicEnum`
- 修复：使用 `.raw` / `_enum_raw()` 统一处理
- 文件：`selfdrive/modeld/model_manager_helpers.py`

2) **isinstance + capnp 类型崩溃**
- 现象：`TypeError: isinstance() arg 2 must be a type`
- 修复：避免 `isinstance(bundle, capnp_struct)`
- 文件：`selfdrive/modeld/model_manager_helpers.py`

3) **thneed_qcom2 编译失败**
- 现象：`Thneed` 缺 `cmds/fd/ram`
- 修复：larch64 编译加 `-DQCOM2`
- 文件：`selfdrive/modeld/SConscript`

4) **UI 崩溃：ShowAdvancedControls 未注册**
- 现象：`UnknownKeyName: ShowAdvancedControls`
- 修复：在 `common/params_keys.h` 增加该 key

---

## 16. 已知边界与注意事项（必须写清楚）

- `ModelManager_DownloadIndex` 为 0 不会触发下载（当前代码真值判断）。
- `navigation` 类型模型会下载，但 UI 无进度行显示（无 label 对应）。
- 下载失败或 hash 失败不会自动删除残留文件。
- `ModelManager_ActiveBundle` 若不兼容版本，`get_active_bundle()` 会返回 None，但 **不会自动删除该参数**。
- `ModelManager_ActiveBundle` 必须是 capnp `to_dict()` 的 camelCase 格式，不可直接使用 JSON 的 snake_case。

---

## 17. 运行/验证清单（必须逐项核对）

1) `modelManagerSP` 能发布（UI 列表可见）
2) 选择模型后自动下载（offroad）
3) 下载进度每行显示
4) 下载完成后 `ModelManager_ActiveBundle` 正确写入
5) 重启后仍保持当前模型
6) Runner 选择正确（stock/tinygrad/snpe）
7) 加载失败自动回退默认模型
8) onroad 时 UI 禁止切换
9) 清缓存只删除非 active bundle
10) “Reset Calibration” 仅在用户确认时清除参数

---

## 18. 将来替换数据源（自建服务）

替换为自建源时必须满足：
1) JSON schema 完全一致（字段名必须匹配 `ModelParser`）
2) `download_uri` 必须包含 `url` 与 `sha256`
3) 更新 `MODEL_URL`（`selfdrive/modeld/model_manager.py`）
4) 保持 `minimum_selector_version` 与 `CURRENT_SELECTOR_VERSION` 兼容
5) 所有模型文件名在 `/data/models` 下不可冲突

---

## 19. 结论

本文档已经覆盖当前落地版模型管理体系的 **全部代码路径、参数、协议、UI 组件与构建细节**。
任何开发者严格依照本文档实现，即可 **100% 1:1 复刻完整模型功能且不会遗漏**。
