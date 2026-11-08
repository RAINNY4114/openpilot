#!/bin/bash
# UT3G 刷机前置：止血 -> 打补丁 -> 诊断 -> 生成白名单
#
# 本脚本**不会**执行 flash_ut3g_adapted.py install。
# 原因：install 会要求输入 FLASH-xxxxxxxx 人工确认，这是安装器刻意的安全闸门，
# 也是「是否接受刷砖风险」的决策点，必须由人来做。脚本只把前面所有步骤做完，
# 最后把那条命令打印出来。
#
# 用法（设备上，会自动 sudo 提权）：
#   bash /data/openpilot/openpilot/_chk/ut3g2/ut3g_run.sh
#   bash .../ut3g_run.sh --stage1     # 只止血+打补丁
#   bash .../ut3g_run.sh --stage2     # 只诊断+白名单（需先 stage1）
#   bash .../ut3g_run.sh --status     # 只看当前状态，不改任何东西
#
# 退出码：0 成功走到「可以刷」/ 1 有阶段失败 / 2 诊断结论是「不该刷」

set -u

REPO=/data/openpilot/openpilot
CHK="$REPO/_chk"
INST=/data/ut3g-install
PY=/usr/local/venv/bin/python3
SYS_PY=python3

SELF="$(readlink -f "$0")"
SRC_DIR="$(dirname "$SELF")"

# ---------- 输出 ----------
c_ok()   { printf '\033[32m%s\033[0m\n' "$*"; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*"; }
c_err()  { printf '\033[31m%s\033[0m\n' "$*"; }
c_hdr()  { printf '\n\033[36m=== %s ===\033[0m\n' "$*"; }

# ---------- 提权 ----------
if [ "$(id -u)" -ne 0 ]; then
  c_warn "需要 root，正在自提权..."
  exec sudo -n bash "$SELF" "$@"
fi

STAGE1=0; STAGE2=0; STAGE3=0; STATUS=0
case "${1:-}" in
  --stage1) STAGE1=1 ;;
  --stage2) STAGE2=1 ;;
  --status) STATUS=1 ;;
  --stage3) STAGE3=1 ;;
  "")       STAGE1=1; STAGE2=1; STAGE3=1 ;;
  *)        echo "未知参数: $1"; exit 2 ;;
esac

# =====================================================================
# 状态查看
# =====================================================================
if [ "$STATUS" -eq 1 ]; then
  c_hdr "当前状态"
  echo "comma.service : $(systemctl is-active comma.service 2>/dev/null)"
  echo "flash.py 进程 : $(pgrep -af 'chestnut/flash.py' | tr '\n' ';' || true)"
  echo "hardwared     : $(pgrep -af 'system/hardware/hardwared.py' | tr '\n' ';' || true)"
  echo "install 目录  : $(ls -1 "$INST" 2>/dev/null | tr '\n' ' ' || echo '<不存在>')"
  echo "identity.json : $([ -f "$INST/identity.json" ] && echo 存在 || echo 不存在)"
  echo "state 目录    : $(ls -1 /data/ut3g-sp8516 2>/dev/null | tr '\n' ' ' || echo '<不存在>')"
  echo
  echo "--- 移植状态 ---"
  [ -f "$CHK/apply_sptest1026.py" ] && \
    "$PY" "$CHK/apply_sptest1026.py" --status 2>&1 | head -20
  exit 0
fi

# =====================================================================
# 前置检查
# =====================================================================
preflight() {
  c_hdr "前置检查"
  local fail=0
  for f in "$CHK/apply_sptest1026.py" "$CHK/egpu_bringup.py" \
           "$SRC_DIR/ut3g_precheck.py" "$SRC_DIR/flash_ut3g_adapted.py" \
           "$SRC_DIR/UT3G_SP8516_UNBOUND_R1.bin"; do
    if [ -f "$f" ]; then c_ok "  OK   $f"; else c_err "  缺失 $f"; fail=1; fi
  done
  if [ ! -x "$PY" ]; then c_err "  缺失 $PY"; fail=1; fi
  return $fail
}

preflight || { c_err "前置检查失败，终止"; exit 1; }

# =====================================================================
# 阶段 1：止血 + 打补丁
# =====================================================================
if [ "$STAGE1" -eq 1 ]; then
  c_hdr "阶段 1/3：止血 + 打补丁"

  echo "[1.1] 停止 comma.service（防止 hardwared 继续刷固件）"
  systemctl stop comma.service 2>/dev/null
  sleep 1
  st=$(systemctl is-active comma.service 2>/dev/null)
  if [ "$st" = "inactive" ]; then c_ok "  comma.service = inactive"; else c_err "  comma.service = $st（继续，但安装器会拒绝）"; fi

  echo "[1.2] 杀掉正在断 VBUS 的刷写进程"
  n1=$(pgrep -fc 'chestnut/flash.py' || true)
  n2=$(pgrep -fc 'system/hardware/hardwared.py' || true)
  pkill -f 'chestnut/flash.py' 2>/dev/null
  pkill -f 'system/hardware/hardwared.py' 2>/dev/null
  sleep 1
  left=$(pgrep -fc 'chestnut/flash.py' || true)
  echo "  已终止 flash.py=$n1 hardwared=$n2，剩余 flash.py=$left"
  [ "${left:-0}" -eq 0 ] && c_ok "  刷写进程已清空" || c_warn "  仍有残留，后续可能争抢 USB"

  echo "[1.3] 预检 sptest1026 补丁锚点"
  if ! "$PY" "$CHK/apply_sptest1026.py" --check >/tmp/_sp1026_chk.log 2>&1; then
    c_err "  锚点预检失败，补丁未打（先看 /tmp/_sp1026_chk.log）"
    tail -20 /tmp/_sp1026_chk.log
    c_warn "  继续诊断，但不刷机"
    STAGE2=1
  else
    c_ok "  锚点全部命中"
    echo "[1.4] 应用补丁（锁死自动刷写 + 解 fmt 死锁）"
    "$PY" "$CHK/apply_sptest1026.py" --apply 2>&1 | sed 's/^/  /'
  fi
fi

# =====================================================================
# 阶段 2：诊断 + 白名单
# =====================================================================
DIAG_RC=99
if [ "$STAGE2" -eq 1 ]; then
  c_hdr "阶段 2/3：诊断底座"

  mkdir -p "$INST"
  cp -f "$SRC_DIR"/ut3g_precheck.py "$SRC_DIR"/flash_ut3g_adapted.py \
        "$SRC_DIR"/UT3G_SP8516_UNBOUND_R1.bin "$INST"/ 2>/dev/null
  chmod +x "$INST"/ut3g_precheck.py "$INST"/flash_ut3g_adapted.py
  c_ok "  运行文件已就位: $INST"

  echo
  "$PY" "$INST/ut3g_precheck.py"
  DIAG_RC=$?

  # 若只有 12Mbps，尝试发 0xF3 把链路带到 USB3，再诊断一次
  if [ "$DIAG_RC" -eq 1 ]; then
    c_hdr "尝试 PCIe 上电（0xF3）把链路带到 USB3"
    c_warn "  当前身份不满足安装器门槛，先尝试上电重枚举"
    "$PY" "$CHK/egpu_bringup.py" --kill-flash --timeout 60 --power-timeout 40 2>&1 | tail -25 | sed 's/^/  /'
    echo
    c_hdr "上电后复诊"
    "$PY" "$INST/ut3g_precheck.py"
    DIAG_RC=$?
  fi

  if [ "$DIAG_RC" -eq 0 ] || [ "$DIAG_RC" -eq 2 ]; then
    c_hdr "生成实测身份白名单"
    "$PY" "$INST/ut3g_precheck.py" --emit-identity "$INST/identity.json" 2>&1 | sed 's/^/  /'
  fi
fi

# =====================================================================
# 阶段 3：结论 + 下一步
# =====================================================================
if [ "$STAGE3" -eq 1 ]; then
  c_hdr "阶段 3/3：结论"

  if [ "$DIAG_RC" -eq 0 ]; then
    c_ok "诊断 GO：身份满足安装器门槛，白名单已生成。"
    echo
    echo "  下一步（需要你亲手输入确认码，脚本不代劳）："
    echo "    cd $INST"
    echo "    sudo $SYS_PY flash_ut3g_adapted.py install"
    echo
    echo "  install 会显示 UID / 变化扇区 / 备份路径，然后要求输入 FLASH-xxxxxxxx。"
    echo "  核对无误再输入；输入错了就是取消，零擦写。"
    echo
    echo "  刷完看到 WRITTEN_VERIFIED_PENDING_POWER_CYCLE 后："
    echo "    1) 完整切断底座 USB 和外部供电，再上电"
    echo "    2) sudo $SYS_PY flash_ut3g_adapted.py verify"
    echo "    3) 成功后再 sudo systemctl start comma.service"
    exit 0
  elif [ "$DIAG_RC" -eq 2 ]; then
    c_warn "诊断 NO_GO_ENV：硬件身份 OK，但环境不满足。"
    echo
    echo "  先处理："
    echo "    sudo systemctl stop comma.service"
    echo "    systemctl is-active comma.service   # 必须是 inactive"
    echo "  并确认已停车、IsOffroad=1，然后重跑本脚本 --stage2"
    exit 2
  elif [ "$DIAG_RC" -eq 1 ]; then
    c_err "诊断 NO_GO_HW：底座身份不满足，安装器会拒绝且不打开设备。"
    echo
    echo "  这通常意味着底座当前不在 USB3(5000Mbps)，或 manufacturer/product 不在已知集合。"
    echo "  在「连 5000Mbps 都上不去」的情况下，安装器无法工作 —— 它不是救砖工具。"
    echo
    echo "  可选："
    echo "    a) 换一根确定支持 USB3/USB4 的线，换直连口（不经 hub）再试"
    echo "    b) 底座与车机两端同时断电 >=30s，先插好线再一起上电，然后重跑 --stage2"
    echo "    c) 若始终读不出描述符：只能外部 SPI 编程器（CH341A + SOP8 夹子）离线写"
    echo "    d) 用 bash $SELF --status 查看细节"
    exit 2
  else
    c_err "诊断 NO_DEVICE：连 chestnut/ROM 设备都看不到。"
    echo
    echo "  先解决 USB 枚举，再谈刷固件："
    echo "    a) 两端断电 >=30s，先插线再上电"
    echo "    b) 换线 / 换口，避开 hub"
    echo "    c) bash $SELF --status 看进程是否还有残留"
    exit 2
  fi
fi

c_hdr "完成"
echo "如需恢复 openpilot 运行：sudo systemctl start comma.service"
