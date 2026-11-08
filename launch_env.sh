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
  export AGNOS_VERSION="19.9"
fi

export STAGING_ROOT="/data/safe_staging"

# [AM_POWER_LIMIT_20261027] eGPU (chestnut) power ceiling, in watts.
#
# modeld.py 只在环境变量缺失时 setdefault('AM_POWER_LIMIT', '120') 兜底，所以这里
# 的 export 才是实际生效值，两者默认值必须保持一致。tinygrad turns it into a real SMU
# cap: amdev.py -> ip.py set_power_limit() -> PPSMC_MSG_SetPptLimit(int watts).
#
# 120 W only raises the ceiling.  Measured median draw on this car is ~33 W, so
# behaviour is unchanged unless a workload actually asks for more.
#
# Runtime override, no repo edit needed:
#   printf '100' > /data/ford_params/am_power_limit && sudo systemctl restart comma
# The value MUST be numeric: tinygrad does float(os.getenv(...)) and would raise
# at AMDev boot on anything else, which takes the big model down with it.
AM_POWER_LIMIT_FILE="/data/ford_params/am_power_limit"
if [ -z "${AM_POWER_LIMIT:-}" ] && [ -r "$AM_POWER_LIMIT_FILE" ]; then
  AM_POWER_LIMIT="$(tr -cd '0-9.' < "$AM_POWER_LIMIT_FILE" 2>/dev/null)"
fi
case "${AM_POWER_LIMIT:-}" in
  ''|*[!0-9.]*|*.*.*) AM_POWER_LIMIT=120 ;;
esac
export AM_POWER_LIMIT
unset AM_POWER_LIMIT_FILE

# [CHX_USB_ENUM_20261027] chestnut (ASM2464) eGPU 的 USB 节点权限守护。
#
# 为什么需要：devtmpfs 以 0600 root:root 建 /dev/bus/usb/<bus>/<dev>，udev 要读完
# 描述符才按 50-udev-default.rules 放宽成 0664。eGPU 与车机同时上电时 ASM2464
# 描述符响应慢，这个窗口可达数十秒；而 modeld 以 comma 身份在 T+34s 就
# libusb_open -> EACCES -> "chestnut load failed" -> 静默回退小模型。
# 本钩子抢在 udev 前面 chmod 666，让 modeld 不必等。脚本同时负责 unbind
# usb-storage（否则 USBDEVFS_SETCONFIGURATION 返回 -EBUSY）。
#
# 幂等：脚本自带单实例保护（/tmp/egpu_usb_perm.pid）。updated.py 会反复
# `bash -c "source launch_env.sh && ..."`，没有这个保护会累积几百个进程。
if [ -x /data/openpilot/system/egpu_usb_perm.sh ]; then
  setsid bash /data/openpilot/system/egpu_usb_perm.sh >/dev/null 2>&1 &
fi


# [NAV_MAPBOX_20261031] navd 的 Mapbox access token（public, pk.*）。
#
# 为什么从文件读而不是写在这里：launch_env.sh 是 git 跟踪的文件，凭证写进来会被
# commit。也不能用 Params —— 新增的 param key 会被 libparams_c.so 的 clearAll 清掉
# （curve_bend.py 里对同一问题有详细说明）。所以用普通文件。
#
# 生效逻辑（navd.py）：
#   MAPBOX_TOKEN 非空 -> mapbox_host = https://api.mapbox.com
#   为空             -> mapbox_host = API_HOST + "/v1/nav"（本机 API_HOST=api.konik.ai，
#                      该端点实测 404，所以为空时 navd 算不出路）
#
# 设置方式（token 不要带引号或换行）：
#   printf "%s" "pk.xxxxx" > /data/ford_params/mapbox_token
#   sudo systemctl restart comma
MAPBOX_TOKEN_FILE="/data/ford_params/mapbox_token"
if [ -z "${MAPBOX_TOKEN:-}" ] && [ -r "$MAPBOX_TOKEN_FILE" ]; then
  MAPBOX_TOKEN="$(tr -d "[:space:]" < "$MAPBOX_TOKEN_FILE" 2>/dev/null)"
fi
if [ -n "${MAPBOX_TOKEN:-}" ]; then
  export MAPBOX_TOKEN
else
  unset MAPBOX_TOKEN
fi
unset MAPBOX_TOKEN_FILE

# [IFE_ROAD_RESIZE_20261102] 参考 onemiless/openpilot commit e0814246c918439f8d05d4518efaa649753e3f3b
# camerad hw.h 已把 WIDE_ROAD 的 IFE 输出从 1928x1208 降到 1344x760，
# camera.py 据此缩放内参矩阵 K（fx/fy 独立）。这个变量给 Python 侧做开关。
export C3XL_IFE_ROAD_SIZE=1344x760
