# SP 模型管理体系复刻文档（基于 99fc67a2）

本文档总结 `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt` 的实现细节，用于在我们项目中复刻完整的“模型清单服务 + 多运行器进程 + 预编译模型仓库”。不复用 SP 代码，只复刻结构与协议。

## 1. 目标与范围
- 目标：实现与 SP 等效的模型下载、切换、缓存、进度展示、兼容性回退、多运行器调度。
- 范围：模型清单服务协议、下载管理器、运行器选择、进程启动、UI 面板。
- 基线版本：`e:\openpilot` 当前 commit `99fc67a2`。

## 2. SP 架构总览（组件与职责）
- **模型清单拉取/缓存**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\fetcher.py`
  - 逻辑：从 `driving_models_v10.json` 拉取 bundles，缓存到 Params（`ModelManager_ModelsCache`），缓存失效时间 1 小时。
- **模型下载与状态**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\manager.py`
  - 逻辑：按 bundle 下载 artifact/metadata，校验 sha256，写 `ModelManager_ActiveBundle`。
  - 通过 `modelManagerSP` 消息向 UI 发布进度。
- **运行器选择/缓存**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\helpers.py`
  - 逻辑：读取 `ModelManager_ActiveBundle`，根据 `bundle.runner` 决定 Runner，并缓存到 `ModelRunnerTypeCache`。
- **进程调度**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\system\manager\process_config.py`
  - 逻辑：根据 Runner 启动不同 modeld 进程（`modeld_snpe`、`modeld_tinygrad`、`selfdrive.modeld.modeld`）。
- **模型运行器实现**
  - SNPE/THNEED 路线：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\modeld\modeld.py`
  - Tinygrad 路线：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\modeld_v2\modeld.py`
  - Tinygrad Runner：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\runners\tinygrad\tinygrad_runner.py`
- **UI 面板**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\selfdrive\ui\sunnypilot\layouts\settings\models.py`
  - 逻辑：模型选择对话框、下载进度、清缓存、取消下载。
- **数据结构（Cap’n Proto）**
  - 文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\cereal\custom.capnp`
  - 结构体：`ModelManagerSP`（Bundle、Model、Artifact、DownloadProgress、Runner 等）。

## 3. 模型清单协议（driving_models_v10.json）
来源：`https://raw.githubusercontent.com/sunnypilot/sunnypilot-docs/refs/heads/gh-pages/docs/driving_models_v10.json`

顶层结构：
- `tinygrad_ref`：tinygrad 编译基线（用于兼容性提示）
- `bundles`：模型包列表

Bundle 结构（关键字段）：
- `index`：下载索引（UI 选择后写入 `ModelManager_DownloadIndex`）
- `short_name` / `display_name`
- `runner`：`snpe` / `tinygrad` / `stock`
- `generation`、`environment`、`is_20hz`、`minimum_selector_version`
- `ref`：唯一标识（UI 选项树的 key）
- `overrides`：字典（如 `folder`、`lat`、`long`）
- `models`：模型列表（supercombo / vision / policy）

Model 结构：
- `type`：`supercombo` / `vision` / `policy`
- `artifact`: `file_name` + `download_uri { url, sha256 }`
- `metadata`: `file_name` + `download_uri { url, sha256 }`

JSON 结构示例（骨架）：
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

## 4. 参数与消息（Params + modelManagerSP）
核心参数键（Params）：
- `ModelManager_ModelsCache`：模型清单 JSON 缓存
- `ModelManager_LastSyncTime`：上次同步时间（纳秒）
- `ModelManager_ActiveBundle`：当前激活 bundle（序列化 dict）
- `ModelManager_DownloadIndex`：触发下载的 bundle index
- `ModelManager_ClearCache`：清理模型缓存
- `ModelManager_Favs`：模型收藏（UI Tree favorites）
- `ModelRunnerTypeCache`：runner 缓存（snpe/tinygrad/stock）

消息（Cap’n Proto）：
`modelManagerSP`（`ModelManagerSP`），包含：
- `availableBundles`
- `selectedBundle`
- `activeBundle`
- `Artifact.downloadProgress`：`status` / `progress` / `eta`

## 5. 下载与校验策略
实现文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\manager.py`
- 下载粒度：`artifact` + `metadata`，每个文件独立进度。
- 进度与 ETA：`DownloadProgress` 按文件更新，单文件 ETA 由 “耗时 / 进度” 估算。
- 缓存：若本地文件存在且 sha256 匹配，标记 `cached`。
- 失败处理：下载异常或校验失败则删除文件，标记 `failed`。
- 成功后：`ModelManager_ActiveBundle` 写入 Params。

## 6. 运行器与进程调度
运行器决定逻辑：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\helpers.py`
- `bundle.runner` 决定 `ModelRunnerTypeCache`
- `stock` → `selfdrive.modeld.modeld`
- `snpe` → `modeld_snpe`（SP 的 thneed/onnx 运行器）
- `tinygrad` → `modeld_tinygrad`（SP 的 tinygrad split/mono）

进程调度：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\system\manager\process_config.py`
- `modeld_snpe`: `sunnypilot/modeld`（THNEED/ONNX）
- `modeld_tinygrad`: `sunnypilot/modeld_v2`（Tinygrad）

## 7. Tinygrad Split/Mono 选择
实现文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\runners\helpers.py`
- 若 bundle 包含 `vision` 或 `policy` → `TinygradSplitRunner`
- 否则使用 `TinygradRunner`（supercombo）

## 8. UI 行为（模型面板）
实现文件：`Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\selfdrive\ui\sunnypilot\layouts\settings\models.py`
- “当前模型”弹出 TreeOptionDialog（带 favorites 分组）
- 选择 “Default” 会清除 `ModelManager_ActiveBundle`
- 选择 bundle 会写 `ModelManager_DownloadIndex`
- 下载进度：按 `vision`/`policy` 分行显示文件进度
- 取消下载：移除 `ModelManager_DownloadIndex`
- 清理缓存：删除除 active bundle 外所有文件

## 9. 模型仓库与文件布局要求
SP 下载目标路径：`Paths.model_root()`（通常 `/data/models`）
文件命名由 JSON `artifact.file_name` / `metadata.file_name` 决定：
- Tinygrad split：`driving_vision_*_tinygrad.pkl`、`driving_policy_*_tinygrad.pkl` + metadata
- Supercombo：`supercombo.*`（`thneed` / `onnx` / `dlc` 视 runner）

## 10. 复刻实施清单（在 99fc67a2 上）
1) **Cap’n Proto**
   - 添加 `ModelManagerSP` 结构（参考 `Z:\...\cereal\custom.capnp`）。
2) **Model Fetcher**
   - 实现清单拉取 + Params 缓存 + 版本兼容性判断（selector version）。
3) **Model Manager 进程**
   - 下载/校验/进度/缓存清理/ActiveBundle 写入。
   - 发布 `modelManagerSP` 消息供 UI 使用。
4) **多运行器进程调度**
   - `modeld_snpe`、`modeld_tinygrad`、`stock modeld` 互斥运行。
5) **Runner 实现**
   - Tinygrad split/mono runner。
   - THNEED/ONNX runner（或 SNPE runner，视模型来源）。
6) **UI 面板**
   - 当前模型 + 模型选择 + 下载进度 + 清缓存 + 取消下载。
7) **模型仓库**
   - 建立 JSON + 文件仓库，提供 `url + sha256`。

## 11. 关键差异与风险
- 没有预编译模型仓库 → 无法完整复刻 SP 功能。
- Runner 不齐全（无 thneed/SNPE）→ 只能支持 tinygrad。
- selector version 不一致 → 必须过滤不兼容 bundle。

## 12. 参考文件清单（SP）
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\fetcher.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\manager.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\helpers.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\models\runners\helpers.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\modeld\modeld.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\sunnypilot\modeld_v2\modeld.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\system\manager\process_config.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\selfdrive\ui\sunnypilot\layouts\settings\models.py`
- `Z:\sunnypilot-master-prebuilt\sunnypilot-master-prebuilt\cereal\custom.capnp`

## 13. ??????????????
- ?????????????Settings -> Model Manager??
- ???????????/????????????????????????/????/??????????
- ???? `ModelManager_DeleteBundleRef` ???????/?????????????????
- ????????????????????????

