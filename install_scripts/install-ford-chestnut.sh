#!/usr/bin/env bash
#
# install-ford-chestnut.sh
# ----------------------------------------------------------------------------
# 将 ford-chestnut 系统文件部署到 comma three / C3X 设备
#
# 用法（在设备上以 comma 用户运行）：
#   bash install-ford-chestnut.sh                       # 从当前目录的 openpilot/ 安装
#   bash install-ford-chestnut.sh /path/to/openpilot    # 指定源目录
#
# 原理：
#   1. 备份现有 /data/openpilot 到 /data/openpilot.bak.<时间戳>
#   2. 用新系统文件替换 /data/openpilot
#   3. 重建运行时符号链接（msgq / opendbc / rednose / teleoprtc / tinygrad）
#   4. 确保 prebuilt 标记存在（跳过重新编译）
#   5. 清理 overlay 标记，使下次启动直接使用新代码
#
# 注意：全新 C3X 也可以用本包覆盖安装，但需要设备已刷好 AGNOS 19.7。
# ----------------------------------------------------------------------------

set -euo pipefail

TARGET="/data/openpilot"
SOURCE="${1:-$(pwd)/openpilot}"
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="/data/openpilot.bak.${STAMP}"

echo "=============================================="
echo " ford-chestnut 系统部署脚本"
echo "=============================================="
echo "源目录  : ${SOURCE}"
echo "目标目录: ${TARGET}"
echo "备份到  : ${BACKUP}"
echo ""

# --- 前置检查 -------------------------------------------------------------
if [ ! -d "${SOURCE}" ]; then
  echo "[错误] 源目录不存在: ${SOURCE}"
  echo "       请先解压安装包，例如："
  echo "       tar xzf openpilot-ford-chestnut-installable.tar.gz"
  exit 1
fi

if [ ! -f "${SOURCE}/openpilot/sunnypilot/common/version.h" ]; then
  echo "[错误] 源目录结构不正确，缺少 sunnypilot 版本文件。"
  exit 1
fi

VER="$(grep -o '"[^"]*"' "${SOURCE}/openpilot/sunnypilot/common/version.h" | head -1 | tr -d '"')"
echo "[信息] 待安装版本: ${VER}"
echo ""

read -r -p "确认继续部署？将覆盖 ${TARGET} (y/N): " CONFIRM
if [ "${CONFIRM}" != "y" ] && [ "${CONFIRM}" != "Y" ]; then
  echo "已取消。"
  exit 0
fi

# --- 备份 -----------------------------------------------------------------
if [ -d "${TARGET}" ]; then
  echo "[1/7] 备份现有系统到 ${BACKUP} ..."
  # 优先用 mv（快且原子）；磁盘不足时退化为 cp
  if mv "${TARGET}" "${BACKUP}" 2>/dev/null; then
    echo "      备份完成 (mv)"
  else
    echo "      mv 失败，改用复制备份 ..."
    mkdir -p "${BACKUP}"
    cp -a "${TARGET}/." "${BACKUP}/" || {
      echo "[错误] 备份失败，磁盘空间可能不足。已中止。"
      exit 1
    }
    rm -rf "${TARGET}"
    echo "      备份完成 (cp)"
  fi
else
  echo "[1/7] 目标不存在，跳过备份（全新安装）。"
fi

# --- 部署 -----------------------------------------------------------------
echo "[2/7] 复制新系统文件 ..."
mkdir -p "${TARGET}"
cp -a "${SOURCE}/." "${TARGET}/"
sudo chown -R comma:comma "${TARGET}"

# --- 运行时符号链接 -------------------------------------------------------
echo "[3/7] 重建运行时符号链接 ..."
cd "${TARGET}"
ln -sfn msgq_repo/msgq        msgq
ln -sfn opendbc_repo/opendbc  opendbc
ln -sfn rednose_repo/rednose  rednose
ln -sfn teleoprtc_repo/teleoprtc teleoprtc
ln -sfn tinygrad_repo/tinygrad tinygrad

# --- prebuilt 标记 --------------------------------------------------------
echo "[4/7] 确保 prebuilt 标记存在（跳过重新编译）..."
touch "${TARGET}/prebuilt"

# --- 清理 overlay 标记 ----------------------------------------------------
echo "[5/7] 清理 overlay 更新标记 ..."
rm -f "${TARGET}/.overlay_init" "${TARGET}/.overlay_consistent"

# --- fork 自有参数目录 ----------------------------------------------------
# 本系统沿用的 libparams_c.so 是预编译产物，键表里**没有**福特自定义键
# （dp_* / AutoOvertake* / AutoAvoidanceEnabled / AvoidBrakeReq 等）。
# params.cc::clearAll() 会在 manager 启动、onroad/offroad 切换、点火时
# 把 /data/params/d 下所有"不在 .so 键表里"的文件 unlink 掉。
# /data/ford_params 不在 params 目录下，clearAll() 永远不碰它，因此
# 福特配置一律持久化到这里（代码侧候选列表已把它排在第一位）。
echo "[6/7] 准备 fork 自有参数目录 /data/ford_params ..."
sudo mkdir -p /data/ford_params
sudo chown comma:comma /data/ford_params

# 迁移历史位置里已有的福特配置（若存在），避免升级后配置丢失
for _k in dp_radar_loss_no_disable dp_mr76_lane_enable dp_mr76_log_enable \
          dp_amap_lidar_enable dp_amap_lidar_mask \
          dp_lincoln_curve_speed dp_lincoln_curve_window_m dp_lincoln_curve_k_enter \
          dp_lincoln_curve_decel dp_lincoln_osm_realtime_cruise dp_lincoln_stop_distance_m \
          dp_bsm_voice_enabled dp_bsm_voice_interval_sec dp_bsm_voice_volume_pct \
          dp_ford_ramp_type dp_ford_precision_type dp_ford_apa_panda_alloutput \
          dp_ford_apa_enable dp_ford_apa_rate_scale dp_ford_apa_shadow_log_path \
          AutoOvertakeEnabled AutoOvertakeLanePref AutoOvertakeConfirmSec \
          AutoOvertakeMinCruiseKph AutoOvertakeLaneProbMin AutoAvoidanceEnabled \
          AvoidStableMode AvoidObsHold AvoidBrakeMax AvoidBrakeGain AvoidBrakeTtcRef; do
  if [ -f "/data/params/d/${_k}" ] && [ ! -f "/data/ford_params/${_k}" ]; then
    sudo cp -a "/data/params/d/${_k}" "/data/ford_params/${_k}" || true
    echo "      迁移 ${_k}"
  fi
done
sudo chown -R comma:comma /data/ford_params

# --- pythonpath -----------------------------------------------------------
echo "[7/7] 更新 pythonpath 链接 ..."
ln -sfn "${TARGET}" /data/pythonpath

echo ""
echo "=============================================="
echo " 部署完成"
echo "=============================================="
echo "版本    : ${VER}"
echo "备份位于: ${BACKUP}"
echo ""
echo "接下来请："
echo "  1) 重启设备:   sudo reboot"
echo "  2) 或仅重启进程: (在设备 tmux 中) Ctrl-C 后重新运行 launch_chffrplus.sh"
echo ""
echo "如需回滚："
echo "  rm -rf ${TARGET} && mv ${BACKUP} ${TARGET} && sudo reboot"
