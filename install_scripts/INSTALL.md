# ford-chestnut 系统安装说明

> 基线：**staging-chestnut**（`99fc3ce`，2026-10-11，sunnypilot chestnut 最新系统）
> 在其之上叠加 **ford-chestnut 的福特控制方法**，并把**左右两侧激光雷达**真正接入
> 变道阻挡 / 主动超车 / 紧急避让的融合链路。
> 仓库：https://github.com/RAINNY4114/openpilot （分支 `ford-chestnut`）
> 分支最新提交：**`6fd4a9a`**（离线包按此提交打包；上一条 `eac44fe` = 25fca3a + 重新打包）
> 更新：**2026-10-12**
>
> 上一版（福特自己那条线，基线更旧）已备份为分支：
> `ford-chestnut-backup-2026-10-12`（= `1964475`）与
> `ford-chestnut-lidar-fusion-2026-10-12`（= `6c55810`）。

---

## 零、这一版和上一版的区别（重要）

| 项目 | 上一版 `1964475` | 本版 `6fd4a9a` |
|---|---|---|
| 基线 | ford-chestnut 自己的旧基线 | **staging-chestnut `99fc3ce`**（最新） |
| 左右侧激光雷达 | schema 有字段但**未接入**（`lidar_available=None` 恒放行） | **真正接入** AO/AA 融合，含 present/valid/age/obstacle |
| 激光雷达接入点 | — | 变道阻挡 / 主动超车 / 紧急转向避让 |
| 参数持久化 | `/data/params/d`（依赖 FC 自己重编的 `.so`） | **`/data/ford_params`**（不依赖 `.so`，见第六节） |
| 自动避让 AA | AO 关闭时因 `NameError` 被静默吞掉，**永不生效** | 已修（`left_ok/right_ok` 提前计算） |

详细接线与语义见 [`docs/ford-chestnut-lidar-fusion.md`](../docs/ford-chestnut-lidar-fusion.md)。

### 信号源分工（硬约束，勿改）

- **左右两侧有车警报**（语音 / 图标）**只认原车 SOD/BSM**，
  即 `carState.leftBlindspot` / `rightBlindspot`，**不并入** MR76 与激光雷达。
- **变道阻挡 / 主动超车 / 紧急避让** 使用
  原车雷达（`radarState.leadOne`）+ 左右盲点检测（BSM）+ MR76 + 左右激光雷达。

---

## 一、这是什么

本系统以 C3X 设备上已调校完善的 sunnypilot-chestnut 最新系统为底，
叠加 `ford-chestnut` 的福特控制方法（MR76 融合雷达、弯道控制器、自动超车、
紧急避让、盲区语音、Ford APA 等），并新增左右两侧激光雷达融合。

**包含完整可运行内容**（不只源码）：

- 全部 Python 源码（含盲区修复、弯道控制器、自动超车、紧急避让、激光雷达融合）
- 全部预编译二进制：`libparams_c.so`、acados 求解器、`libcar.so` / `libpose.so` / `liblive.so`
- 全部模型文件：`big_driving_tinygrad.pkl`、`dmonitoring_model`、`dm_warp` 等
- `prebuilt` 标记 → 部署后**无需重新编译**，重启即可生效

---

## 二、主要功能特性

| 类别 | 内容 |
|---|---|
| **Ford 车控** | MR76 辅助雷达、弯道控制器（curve controller）、雷达静止判定修复、ACCDATA 修复（`AccPrpl_A_Pred`、`AccVeh_V_Trg`）、盲区语音提示 |
| **激光雷达融合** | `amapNavi` 消息通道承载左右侧激光雷达健康度（present/valid/age）与盲区；接入自动超车（AO）与紧急避让（AA）的门控 |
| **Cereal 协议** | `custom.capnp` 新增 `AmapNavi`（13 字段）；`log.capnp` 增加 `amapNavi @154`；`services.py` / `services.h` 注册服务 |
| **Carrot 模块** | `amap_navi.py`（高德导航 + 外接雷达/激光雷达汇总）、`web_interface.py`、`nav_params` |
| **控制库** | `auto_overtake`、`auto_avoidance`、`curve_bend`、`curve_lane_bias`、`ford_curve_controller`、`ford_curve_speed`、`lateral_clearance` |
| **UI** | HUD 弯道组件、盲区变道语音（`left.wav` / `right.wav` 等）、soundd 混音修复 |
| **驾驶员监控** | 放宽的分心判定阈值 |
| **参数系统** | 沿用基线预编译 `libparams_c.so`；福特自定义键改为持久化到 fork 自有目录（见第六节） |
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

本版离线安装包**已重新打包**，对应分支 `ford-chestnut` 的提交 **`6fd4a9a`**。

| 项目 | 本版值 |
|---|---|
| 安装包文件名 | `openpilot-ford-chestnut-installable.tar.gz` |
| 大小 | 947,422,597 字节（903 MB） |
| MD5 | `01f91db34e6ee9e6932b1d5e2d22f899` |
| SHA256 | `fd3894837bd9d1ded38714b617ddb90f511f1feacf1ff37e9666fc19f5cdb13e` |
| 归档条目数 | 5046 |
| 归档顶层目录 | `openpilot/`（解压到 `/data` 即得 `/data/openpilot/`） |
| Git 提交 | `6fd4a9a3fb334dbaf9a074e08ab7e30dae46162f` |
| 打包日期 | 2026-10-12 |

校验命令：

```bash
md5sum openpilot-ford-chestnut-installable.tar.gz
# 期望: 01f91db34e6ee9e6932b1d5e2d22f899

sha256sum openpilot-ford-chestnut-installable.tar.gz
# 期望: fd3894837bd9d1ded38714b617ddb90f511f1feacf1ff37e9666fc19f5cdb13e

tar tzf openpilot-ford-chestnut-installable.tar.gz | wc -l
# 期望: 5046
```

**⚠️ 打包必须用 LF（重要，勿改）**：

本仓库 `core.autocrlf=true`，工作树里的文本文件是 **CRLF**。
**默认的 `git archive` 也会套用这个转换**，所以必须显式关掉：

```bash
git -c core.autocrlf=false archive --format=tar --prefix=openpilot/ HEAD > base.tar
tar -rf base.tar -C <repo> --transform='s|^|openpilot/|' -T pyclist.txt   # 44 个 .pyc
gzip -6 -c base.tar > openpilot-ford-chestnut-installable.tar.gz
```

若从**工作树**直接打包（上一版 `25fca3a` 的做法），`launch_chffrplus.sh`
的 shebang 会变成 `#!/usr/bin/env bash\r`，全树 193/194 个 `.sh` 带 CRLF，
装到设备后 `bad interpreter: No such file or directory`，**系统无法启动**。
本版已改用上面的方式，全部 `.sh` / `.py` 均为 LF。

**包内容说明**：

- 归档内**不含 `.git`**（约 3.9 GB，对运行无必要）。运行时读取版本的
  `common/git.py` 走 `run_cmd_default`，取不到 git 信息时返回空串，
  **不会报错**；UI 的版本号可能显示为空，属预期行为。
- 五个运行时符号链接 `msgq` / `opendbc` / `rednose` / `teleoprtc` / `tinygrad`
  **不在包内**，由 `install-ford-chestnut.sh` 部署时创建。
- 包内**保留** 44 个 `*.py.pyc` 预编译产物（fork 自带，可加速启动；
  Python 会按源码 mtime 自行校验，失效时自动重编，无副作用）。
- 部署后 `/data/openpilot` **不是 git 仓库**。若后续要用 git 增量更新
  （方式 C）或查看提交历史，请改用方式 B / C 从 GitHub 安装。

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

5. **参数目录 `/data/ford_params` 不要删**：本版沿用基线的预编译
   `libparams_c.so`，它的键表里**没有**福特自定义键（`dp_*` /
   `AutoOvertake*` / `AutoAvoidanceEnabled` / `AvoidBrakeReq` …）。
   `params.cc::clearAll()` 会在 manager 启动、onroad/offroad 切换、点火时
   把 `/data/params/d` 下所有"不在 `.so` 键表里"的文件 unlink 掉。
   `/data/ford_params` 不在 params 目录下，`clearAll()` 永不触碰它，因此
   所有福特配置都持久化在这里（安装脚本会自动创建并迁移历史配置）。
   详见 [`docs/ford-chestnut-lidar-fusion.md`](../docs/ford-chestnut-lidar-fusion.md) 第 5 节。

6. **`params_keys.h` 里的福特键**只是声明，方便将来重编 `.so`；
   在当前预编译 `.so` 下它们不生效，代码一律走文件 I/O。

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
