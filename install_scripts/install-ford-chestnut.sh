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
  echo "[1/6] 备份现有系统到 ${BACKUP} ..."
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
  echo "[1/6] 目标不存在，跳过备份（全新安装）。"
fi

# --- 部署 -----------------------------------------------------------------
echo "[2/6] 复制新系统文件 ..."
mkdir -p "${TARGET}"
cp -a "${SOURCE}/." "${TARGET}/"
sudo chown -R comma:comma "${TARGET}"

# --- 运行时符号链接 -------------------------------------------------------
echo "[3/6] 重建运行时符号链接 ..."
cd "${TARGET}"
ln -sfn msgq_repo/msgq        msgq
ln -sfn opendbc_repo/opendbc  opendbc
ln -sfn rednose_repo/rednose  rednose
ln -sfn teleoprtc_repo/teleoprtc teleoprtc
ln -sfn tinygrad_repo/tinygrad tinygrad

# --- prebuilt 标记 --------------------------------------------------------
echo "[4/6] 确保 prebuilt 标记存在（跳过重新编译）..."
touch "${TARGET}/prebuilt"

# --- 清理 overlay 标记 ----------------------------------------------------
echo "[5/6] 清理 overlay 更新标记 ..."
rm -f "${TARGET}/.overlay_init" "${TARGET}/.overlay_consistent"

# --- pythonpath -----------------------------------------------------------
echo "[6/6] 更新 pythonpath 链接 ..."
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
