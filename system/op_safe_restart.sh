#!/usr/bin/env bash
# ============================================================================
# 安全重启 openpilot —— 规避 comma.sh 的「触摸恢复出厂重置」地雷
#
# 背景（2026-10-12 踩到）：
#   /usr/comma/comma.sh:58
#     if [ ! -f /tmp/booted ]; then
#       touch /tmp/booted
#       ...
#       elif (( "$(cat /sys/class/input/input2/device/touch_count)" > 4 )); then
#         echo "launching system reset, got taps"
#         $RESET --tap-reset          # <-- 恢复出厂重置！
#
#   /tmp 是 tmpfs（开机清空），而 touch_count 是触摸驱动的 sysfs 计数器，
#   会随使用一直累加（实测开机 5 小时后 = 57）。只要 /tmp/booted 不存在，
#   comma.sh 就会拿这个累加值去判断「用户开机时敲了 5 下屏幕」，
#   于是 systemctl restart comma 变成一次恢复出厂重置。
#
#   实测：2026-10-12 05:35 一次 `sudo systemctl restart comma` 就触发了
#   `/usr/comma/reset --tap-reset`（幸好及时 pkill 掉，/data 未受损）。
#
# 原理：
#   `[ ! -f /tmp/booted ]` 为假 => 整段重置检查被跳过。所以重启前先把
#   这个文件建出来，100% 规避。本脚本就是这么做的。
#
# 用法：
#   bash /data/openpilot/system/op_safe_restart.sh
#
# 想直接重启整机（而非只重启服务）时，本脚本也先补 /tmp/booted；
# 但注意整机重启后 /tmp 会清空，comma.sh 仍会读 touch_count —— 那时
# 计数器由驱动重新从 0 开始，通常安全；若担心，可先用本脚本重启服务。
# ============================================================================
set -u

TOUCH_COUNT_SYS=/sys/class/input/input2/device/touch_count

tc=$(cat "$TOUCH_COUNT_SYS" 2>/dev/null || echo "?")
echo "[op_safe_restart] touch_count=$tc"

# 关键一步：让 comma.sh 跳过整段恢复出厂重置检查
touch /tmp/booted
echo "[op_safe_restart] /tmp/booted 已就位 -> comma.sh 不会走 tap-reset 分支"

echo "[op_safe_restart] restarting comma.service ..."
sudo systemctl restart comma

sleep 3
echo "[op_safe_restart] comma.service: $(systemctl is-active comma)"
echo "[op_safe_restart] 完成。查看输出：tmux capture-pane -p -t comma -S -40"
