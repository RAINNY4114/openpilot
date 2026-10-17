#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# ---- chestnut 大模型：tinygrad 编译/下载缓存落持久分区 ----
# /home 是 100MB overlay（upperdir=/rwtmp/home_upper），重启即清空且容量上限 100MB；
# tinygrad 的缓存默认在 $XDG_CACHE_HOME/tinygrad（helpers.py:399），落在 /home 里
# 每次开机都要重编大模型内核。挪到 /data 后跨重启复用，缩短加载窗口。
# 注：上游 sp0920xl 用 TINYGRAD_CACHE_DIR=/data/tg_cache/tinygrad，但本版 tinygrad
# **不读**该变量（tinygrad_repo/tinygrad/helpers.py:399 只认 XDG_CACHE_HOME；
# 全仓 grep TINYGRAD_CACHE_DIR = 0 命中）⇒ 换成它反而会让缓存落回 /home，故保留本行。
export XDG_CACHE_HOME=/data/.cache

# ---- chestnut eGPU(ASM24) 功率上限 ----
# 默认走显卡 SMU PPT 上限(182W)。上游 sp0920xl 压到 120W，本机用 100W：
# 当前主诉是 USB/PCIe 枚举与加载超时，峰值电流越低越稳，故保持更保守的 100W。
# 若要给大模型更多算力余量：改成 120（代价：峰值电流上升，链路抖动风险增加）。
export AM_POWER_LIMIT=100

# ---- C3XL IFE 硬件缩图（修 CTM 大模型掉帧 / USB 5G 带宽不足）----
# 移植自 onemiless/openpilot@dev-sp-egpu 提交 2a69709f06（上游 sp0920xl 同款）。
# road 相机在 IFE 硬件里直接出 1344x760（非裁剪，全视场），像素 -43.8%、
# NV12 单帧 -56.2%，两路 road @20Hz 的 USB 占用从 139.6MB/s 降到 61.3MB/s。
# camerad 侧门控 = getenv(C3XL_IFE_ROAD_SIZE)="1344x760" AND /data/hardware_profile=="c3xl"，
# 而 modeld 侧只认 env。只导出 env、不建 hardware_profile ⇒ camerad 照旧出 1928x1208、
# modeld 却按 1344x760 校验 ⇒ RuntimeError 崩溃（2026-09-24：大模型完全起不来）。
# 故把 env 与 opt-in 文件绑死：没有该文件就不导出，让两侧同时休眠，杜绝「半启用」。
# 本机现状：/data/hardware_profile 不存在 且 全仓 grep C3XL_IFE_ROAD_SIZE = 0 命中
# ⇒ 该块当前完全惰性（camerad/modeld 尚未移植支持代码），保留以便日后同步上游时直接生效。
# 启用：echo c3xl > /data/hardware_profile 后重启。回滚：rm /data/hardware_profile。
if [ -f /data/hardware_profile ] && [ "$(cat /data/hardware_profile 2>/dev/null)" = "c3xl" ]; then
  export C3XL_IFE_ROAD_SIZE=1344x760
fi

# ---- 开机自动校时（版本化，见 system/time_seed.sh 头部注释）----
# 本机 RTC(qpnp_rtc) 坏：timedatectl 每次开机都读到 1970-01-01，断电不保留；
# NTP 同步前系统时间会退回 2026-07-28，导致 swaglog 同一文件内时间戳倒流。
# 该脚本开机即用锚点兜底（不依赖网络）+ NTP + HTTPS Date 头三重校时，
# 并把「最后一次已知正确时间」写入 /data/time_anchor，保证时间永不倒退。
# 放在仓库内而不是 /data/continue.sh：后者由
# openpilot/selfdrive/ui/installer/continue_openpilot.sh 生成，重装/OTA 会覆盖，
# 钩子会静默消失。脚本内部会等网络（实测约 55~100s 才就绪）并自带 flock 单实例保护，
# 所以必须 setsid + 后台执行，绝不能阻塞 openpilot 启动。
# 注：launch_env.sh 会被 openpilot/system/updated/updated.py:199 反复 source，
# flock 保证不会重复拉起。日志见 /data/time_seed.log。
if [ -x "${DIR:-/data/openpilot}/system/time_seed.sh" ]; then
  setsid "${DIR:-/data/openpilot}/system/time_seed.sh" >>/data/time_seed.log 2>&1 &
fi

# ---- chestnut(eGPU) USB 节点权限守护（冷启动 EACCES 竞态）----
# devtmpfs 以 0600 root:root 建 /dev/bus/usb/<bus>/<dev>，udev 要读完描述符才按
# 50-udev-default.rules:72 放宽成 0664；eGPU 底座与车机同时上电时 ASM2464 描述符
# 响应慢，这个窗口能拖到几十秒，而 modeld 以 comma 身份在 T+34s 就 libusb_open
# -> EACCES -> load_big() 只试一次即放弃 -> 静默回退小模型。
# 这里开机即后台轮询、抢在 udev 前把节点 chmod 666；脚本自带自提权。
# 必须放仓库内（与 time_seed.sh 同理）：写 /data/continue.sh 会被重装覆盖。
# 本文件会被 updated.py 反复 source；egpu_usb_perm.sh 自带 /tmp/egpu_usb_perm.pid
# 单实例锁，重复拉起会立即 exit 0，故此处不做额外判断（与原设计一致）。
# 用 ${DIR:-/data/openpilot} 而非硬编码：launch_chffrplus.sh 带 DIR 加载，
# 而 updated.py / tools/op.sh 不带 DIR 加载，必须靠默认值兜底。
if [ -x "${DIR:-/data/openpilot}/system/egpu_usb_perm.sh" ]; then
  setsid "${DIR:-/data/openpilot}/system/egpu_usb_perm.sh" >/dev/null 2>&1 &
fi

# ---- 未采纳的上游项：SKIP_TINYGRAD_COMPILE ----
# 上游 sp0920xl 导出 SKIP_TINYGRAD_COMPILE=1，理由是 SConscript 会尝试编译
# big_driving_supercombo.onnx。但本 fork 仓库根没有 SConstruct/SConscript
# （find . -name "SConscript*" 全空），该变量全仓 0 命中 ⇒ 导出是 no-op，
# 故不导出，避免造成「已跳过编译」的错觉。
