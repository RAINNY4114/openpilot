# ford-chestnut 系统安装说明

> 基于 C3X 设备上已验证的 sunnypilot-chestnut 系统生成
> 仓库：https://github.com/RAINNY4114/openpilot （分支 `ford-chestnut`）
> 版本：sunnypilot **2026.003.000** ｜ AGNOS **19.7** ｜ 提交 `fdb5323`

---

## 一、这是什么

本系统由 C3X 设备上已安装并调校完善的 sunnypilot-chestnut 系统完整导出，已全部推送到
GitHub 的 `ford-chestnut` 分支，并额外打包了一份**离线安装包**。

**包含完整可运行内容**（不只源码）：

- 全部 Python 源码（含 33 个修改文件 + 20 个新增文件）
- 全部预编译二进制：`libparams_c.so`、acados 求解器、`libcar.so` / `libpose.so` / `liblive.so`
- 全部模型文件：`big_driving_tinygrad.pkl`（18 分片）、`dmonitoring_model`、`dm_warp` 等
- `prebuilt` 标记 → 部署后**无需重新编译**，重启即可生效

---

## 二、主要功能特性

| 类别 | 内容 |
|---|---|
| **Ford 车控** | MR76 辅助雷达、弯道控制器（curve controller）、雷达静止判定修复、ACCDATA 修复（`AccPrpl_A_Pred`、`AccVeh_V_Trg`）、盲区语音提示 |
| **Cereal 协议** | `custom.capnp` / `log.capnp` / `services.py` 扩展（`amapNavi`、`AudibleAlertSP`、`OnroadEventSP`、自动超车字段） |
| **Carrot 模块** | `amap_navi.py`（高德导航）、`web_interface.py`、`nav_params` |
| **控制库** | `auto_overtake`、`auto_avoidance`、`curve_bend`、`curve_lane_bias`、`ford_curve_controller`、`ford_curve_speed`、`lateral_clearance` |
| **UI** | HUD 弯道组件、盲区变道语音（`left.wav` / `right.wav` 等）、soundd 混音修复 |
| **驾驶员监控** | 放宽的分心判定阈值 |
| **参数系统** | `libparams_c.so` 重新构建，284 个参数键（含 13 个自定义 `dp_*`） |
| **Panda 安全** | `ford.h` 曲率限制调整 |
| **地图/定位** | `osm_map_data` 实时巡航速度、`pigeond` GPS 处理调整 |

---

## 三、安装方式

### 方式 A：离线安装包（推荐，无需网络）

**1. 上传安装包到设备**

```bash
scp -i <你的密钥> openpilot-ford-chestnut-installable.tar.gz \
    comma@<设备IP>:/data/
```

**2. 在设备上解压**

```bash
ssh comma@<设备IP>
cd /data
tar xzf openpilot-ford-chestnut-installable.tar.gz
```

解压后会得到 `/data/openpilot/` 目录。

**3. 执行部署脚本**

```bash
bash /data/install-ford-chestnut.sh /data/openpilot
```

脚本会自动：备份现有系统 → 替换文件 → 重建符号链接 → 打 prebuilt 标记 → 清理 overlay 标记。

**4. 重启生效**

```bash
sudo reboot
```

---

### 方式 B：从 GitHub 直接安装（设备可联网时更快）

```bash
ssh comma@<设备IP>
cd /data
# 上传并运行脚本
bash install-from-github.sh
sudo reboot
```

脚本会把设备上已有的 openpilot 仓库切到 `ford-chestnut` 分支。
由于分支内含预编译文件，切换后无需 `build.py`。

---

### 方式 C：设备上已有仓库，只做拉取更新（增量）

```bash
ssh comma@<设备IP>
cd /data/openpilot
git remote set-url origin https://github.com/RAINNY4114/openpilot.git

# 若有本地修改，先备份
git stash push -u -m "before-update"

git fetch origin ford-chestnut --depth 1
git checkout -B ford-chestnut origin/ford-chestnut

# 重建符号链接 + 标记
ln -sfn msgq_repo/msgq msgq; ln -sfn opendbc_repo/opendbc opendbc
ln -sfn rednose_repo/rednose rednose; ln -sfn teleoprtc_repo/teleoprtc teleoprtc
ln -sfn tinygrad_repo/tinygrad tinygrad
touch prebuilt
sudo reboot
```

---

## 四、文件校验

| 项目 | 值 |
|---|---|
| 安装包文件名 | `openpilot-ford-chestnut-installable.tar.gz` |
| 大小 | 928 MB |
| MD5 | `72549d9dcc02783df78c1150d2ce05f0` |
| 归档条目数 | 5020 |
| Git 提交 | `fdb5323dda1fa64ac1b2a34221ebff76cb512d0a` |

校验命令：

```bash
md5sum openpilot-ford-chestnut-installable.tar.gz
# 应输出 72549d9dcc02783df78c1150d2ce05f0
```

---

## 五、回滚

部署脚本会自动备份原系统到 `/data/openpilot.bak.<时间戳>`。

```bash
cd /data
ls -d openpilot.bak.*              # 找到备份目录
rm -rf /data/openpilot
mv /data/openpilot.bak.<时间戳> /data/openpilot
sudo reboot
```

> 注意：设备 `/data` 分区 88 GB，使用率约 90%。备份前请确认剩余空间充足，
> 必要时先清理 `/data/openpilot.bak.*` 等旧备份。

---

## 六、注意事项

1. **AGNOS 版本**：本系统适配 AGNOS 19.7。设备 `/VERSION` 与
   `launch_chffrplus.sh` 中的 `AGNOS_VERSION` 不一致时，开机脚本会自动触发 AGNOS 更新并重启，属正常行为。

2. **prebuilt 标记很关键**：`/data/openpilot/prebuilt` 文件存在时，
   启动脚本会跳过 `build.py`。若误删，首次启动会进行完整编译（耗时较长）。

3. **overlay 标记**：若 `/data/openpilot/.overlay_init` 比 `.git` 内容更新，
   启动脚本会输出 "has been modified, skipping overlay update installation"。
   本部署流程已清理该标记，确保使用新代码。

4. **首次启动**：建议在设备上原地观察一轮启动日志，确认 manager 正常拉起：
   ```bash
   tmux attach -t comma    # 或查看 /tmp/launch_log
   ```

5. **安全提示**：本包中含 `libparams_c.so` 等预编译二进制，与源码配套。
   请勿单独替换源码而不更新该二进制，否则会出现参数键数量不一致的问题。

---

## 七、目录结构速览

```
/data/openpilot/
├── prebuilt                    # 存在则跳过编译
├── launch_chffrplus.sh         # 主启动脚本（含 C3X 恢复逻辑）
├── openpilot/                  # 主程序
│   ├── carrot/                 # 高德导航 / Web 界面
│   ├── cereal/                 # 消息协议定义
│   ├── selfdrive/              # 控制、UI、监控、modeld
│   │   ├── controls/lib/       # 新增曲线控制 / 自动超车 / 避让
│   │   ├── ui/soundd.py        # 语音（盲区提示）
│   │   └── modeld_v2/modeld    # 模型推理二进制
│   ├── sunnypilot/             # sunnypilot 扩展
│   └── common/libparams_c.so   # 参数系统（284 键）
├── opendbc_repo/               # 车辆接口（Ford 强化）
│   └── opendbc/
│       ├── car/ford/           # carstate / carcontroller / radar_interface
│       ├── dbc/u_radar.dbc     # MR76 雷达报文
│       └── safety/modes/ford.h # Panda 安全
├── msgq_repo/  rednose_repo/  teleoprtc_repo/  tinygrad_repo/
└── tools/  system/  panda/
```
