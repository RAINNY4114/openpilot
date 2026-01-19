# 林肯面板：NAS 管理（Synology）复刻文档
## 0. 功能定义
- 将 `/data/media/0/realdata` 录制目录通过 SCP 上传到 NAS，或删除本地录制。
- UI 内提供配置、状态展示与执行按钮。

## 1. 参数与默认值（`common/params_keys.h`）
- `NasSshDest`：PERSISTENT STRING，默认 `NAS@192.168.50.200:/volume1/openpilot`
- `NasSshPort`：PERSISTENT STRING，默认 `22`
- `NasSshKey`：PERSISTENT STRING，默认空字符串（系统默认 key）
- `LincolnNASAddress`：PERSISTENT STRING（解析后的 host）
- `LincolnNASUsername`：PERSISTENT STRING（解析后的 user）
- `LincolnNASPassword`：PERSISTENT STRING（当前未使用，固定清空）
- `LincolnNASLastResult`：PERSISTENT STRING（状态显示）

## 2. UI 入口与文案（`selfdrive/ui/layouts/settings/lincoln.py`）
### 2.1 配置入口
- 按钮标题：`NAS (Synology) configuration`
- 按钮文字：`Edit`
- 描述（动态）：`Destination: {dest} · Port: {port} · Key: {key}`
  - `key` 为空时显示：`System default (copy custom keys to /data/openpilot/ssh/ and enter full path)`

### 2.2 状态显示
- 标题：`NAS status`
- 描述：`Last result of NAS upload/delete operations.`
- 默认状态：`Waiting for action`（首次读取为空时写入）。

### 2.3 执行按钮
- 左键：`Upload recordings`
- 右键：`Delete local recordings`
- 描述：`Upload recordings from /data/media/0/realdata to NAS via SCP, or delete them locally. Files are read-only to other users (0600) by default.`

## 3. UI 交互逻辑（完整）
### 3.1 编辑流程（3 步键盘输入）
- 入口：`_on_nas_configure()` → `_show_next_nas_keyboard()`
- 步骤顺序：`dest` → `port` → `key`
- 键盘输入长度：`dest/port` 最小 1 字符，`key` 允许空（min_text_size=0）。
- 文案：
  - dest 标题：`Edit NAS destination`
  - port 标题：`Edit NAS SSH port`
  - key 标题：`Edit NAS SSH key path`
  - dest 提示：`Use format user@host:/volume/path`
  - port 提示：`Default 22 if empty.`
  - key 提示：`Leave empty for system default...`
- 校验：
  - dest：必须同时包含 `@` 和 `:`，否则提示 `Invalid destination format. Use user@host:/volume/path.`
  - port：空则设为 `22`，必须为数字，否则提示 `Port must be a number.`
  - key：不校验，可空
- 提交后：调用 `_nas_summary()` 刷新展示与写入解析参数。

### 3.2 配置归一化与解析
- `_nas_summary()`：
  - 确保 `NasSshDest/NasSshPort/NasSshKey` 有默认值。
  - `parse_dest()` 解析 `user@host:/path`，写入：
    - `LincolnNASAddress = host`
    - `LincolnNASUsername = user`
    - `LincolnNASPassword = ""`
  - 返回展示串：`{host}:{remote_path} (user: {user})`

## 4. 执行流程（UI 调用脚本）
- 入口：`_run_nas_command()`
- 互斥：`_nas_action_inflight` 防止并发。
- 若任务已在运行：弹窗提示 `NAS task already running.` 并直接返回。
- 脚本路径：`selfdrive/ui/tools/lincoln_media_manager.py`
- 若脚本缺失：写入 `LincolnNASLastResult = "NAS helper script missing."`
- 执行中：写入 `LincolnNASLastResult = "Processing NAS request..."`
- 子线程执行：`subprocess.run([python, NAS_SCRIPT, --upload/--delete])`
- 异常处理：
  - `CalledProcessError` → `NAS command failed (code X).`
  - 其它异常 → `NAS command failed: ...`

## 5. 脚本逻辑（`selfdrive/ui/tools/lincoln_media_manager.py`）
### 5.1 基本常量
- `LOCAL_ROOT = /data/media/0/realdata`
- `DEFAULT_DEST = NAS@192.168.50.200:/volume1/openpilot`
- `DEFAULT_PORT = 22`
- `DEFAULT_KEY = ""`

### 5.2 SSH / SCP 选项
- SSH：`ssh -p {port} -o StrictHostKeyChecking=no -o BatchMode=yes [-i key]`
- SCP：`scp -r -P {port} -o StrictHostKeyChecking=no -o BatchMode=yes [-i key]`
- `BatchMode=yes` 表示 **不弹密码**，必须配置免密或 key。

### 5.3 Upload 逻辑（`--upload`）
1. 读取并写回默认参数 `NasSshDest/NasSshPort/NasSshKey`。
2. 检查本地目录是否存在、是否为空。
3. 远端目录准备：`ssh mkdir -p remote_path`。
4. 遍历本地子目录/文件：
   - 远端存在即跳过：`ssh test -e remote_entry`。
5. 上传：对每个未上传条目执行 `scp -r`。
6. 状态更新：
   - `Upload progress: idx/total`
   - 失败即写入错误并返回。
   - 成功写入：`Upload finished: uploaded X item(s), skipped Y.`

### 5.4 Delete 逻辑（`--delete`）
- 遍历 `LOCAL_ROOT` 下所有目录/文件：
  - 目录：`shutil.rmtree`
  - 文件：`unlink`
- 写入状态：`Local recordings deleted (processed N item(s)).`
- 失败：`Failed to delete <name>: <error>`

## 6. 文件路径清单
- `common/params_keys.h`
- `selfdrive/ui/layouts/settings/lincoln.py`
- `selfdrive/ui/tools/lincoln_media_manager.py`

## 7. 复刻步骤（一步不省）
1. 注册 NAS 相关参数键（含 `LincolnNASLastResult`）。
2. UI 添加 3 个入口：配置按钮、状态文本、双按钮（上传/删除）。
3. 完整实现 3 步键盘输入与校验。
4. 在 UI 端实现脚本调用与并发互斥。
5. 实现 `lincoln_media_manager.py` 脚本：默认值、SSH/SCP 逻辑、状态写入。

## 8. 验证清单
- 配置流程能依次弹出 dest/port/key 键盘。
- 输入错误格式时弹出提示并阻断保存。
- 点击上传后状态应滚动更新进度。
- 远端已存在文件应被跳过并统计 `skipped`。
- 删除后本地 `/data/media/0/realdata` 为空。
