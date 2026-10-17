#!/usr/bin/env bash
#
# install-from-github.sh
# ----------------------------------------------------------------------------
# 直接在 C3X 设备上从 GitHub 拉取 ford-chestnut 分支并切换到该分支。
# 适合设备已有 openpilot 且能联网的场景（比传输 900MB 安装包更快）。
#
# 用法（在设备上以 comma 用户运行）：
#   bash install-from-github.sh
#
# 说明：
#   - 仓库: https://github.com/RAINNY4114/openpilot  分支: ford-chestnut
#   - 该分支包含完整的可运行系统文件（含预编译 .so 与模型文件），
#     因此切换分支后无需 build.py，直接重启即可生效。
# ----------------------------------------------------------------------------

set -euo pipefail

REPO="https://github.com/RAINNY4114/openpilot.git"
BRANCH="ford-chestnut"
TARGET="/data/openpilot"

echo "=============================================="
echo " 从 GitHub 安装 ford-chestnut"
echo "=============================================="
echo "仓库: ${REPO}"
echo "分支: ${BRANCH}"
echo ""

if [ ! -d "${TARGET}/.git" ]; then
  echo "[错误] ${TARGET} 不是 git 仓库。"
  echo "       请改用离线安装包 + install-ford-chestnut.sh"
  exit 1
fi

cd "${TARGET}"

echo "[1/5] 检查本地修改 ..."
if ! git diff --quiet HEAD 2>/dev/null; then
  echo "     检测到本地未提交修改。"
  read -r -p "     是否 stash 保存这些修改？(y/N): " STASH
  if [ "${STASH}" = "y" ] || [ "${STASH}" = "Y" ]; then
    git stash push -u -m "backup-before-ford-chestnut-$(date +%s)"
    echo "     已 stash 保存。"
  fi
fi

echo "[2/5] 配置远程仓库 ..."
git remote set-url origin "${REPO}"
git fetch origin "${BRANCH}" --depth 1

echo "[3/5] 切换到 ${BRANCH} ..."
git checkout -B "${BRANCH}" "origin/${BRANCH}"

echo "[4/5] 重建运行时符号链接 ..."
ln -sfn msgq_repo/msgq        msgq
ln -sfn opendbc_repo/opendbc  opendbc
ln -sfn rednose_repo/rednose  rednose
ln -sfn teleoprtc_repo/teleoprtc teleoprtc
ln -sfn tinygrad_repo/tinygrad tinygrad

echo "[5/5] 确保 prebuilt 标记与 overlay 清理 ..."
touch "${TARGET}/prebuilt"
rm -f "${TARGET}/.overlay_init" "${TARGET}/.overlay_consistent"
ln -sfn "${TARGET}" /data/pythonpath

VER="$(cat openpilot/sunnypilot/common/version.h 2>/dev/null || echo 'unknown')"
echo ""
echo "=============================================="
echo " 安装完成"
echo "=============================================="
echo "版本: ${VER}"
echo "当前提交: $(git rev-parse --short HEAD)"
echo ""
echo "请重启设备生效: sudo reboot"
