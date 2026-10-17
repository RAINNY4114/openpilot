#!/usr/bin/env bash
# ============================================================================
# chestnut (ASM2464) eGPU USB 链路诊断 / 修复
#
# 背景（2026-10-13 定案）：
#   大模型加载失败的直接异常是
#     ExceptionGroup: No interface for AMD:0 is available
#       +- RuntimeError: libusb_set_configuration: Resource busy
#   而 USBDEVFS_SETCONFIGURATION 在内核里(drivers/usb/core/devio.c:proc_setconfig)
#   只要「当前配置里**任意**一个接口有内核驱动绑定」就返回 -EBUSY。
#   tinygrad 只 detach 了 interface 0，所以只要驱动落在别的接口上就永远起不来。
#   ASM2464 会暴露一个 mass-storage 接口，而 AGNOS 开机就注册 usb-storage。
#
#   本脚本：打印 eGPU 的链路/接口/驱动绑定，并可用 --fix 把内核驱动从这些接口上解绑。
#
# 用法：
#   bash system/egpu_usb_diag.sh          # 只诊断
#   sudo bash system/egpu_usb_diag.sh --fix   # 诊断 + 解绑内核驱动
# ============================================================================
FIX=0
[ "$1" = "--fix" ] && FIX=1

echo "===== chestnut eGPU 诊断 $(date -u +%FT%TZ) ====="

found=0
for d in /sys/bus/usb/devices/*/; do
  vid=$(cat "${d}idVendor" 2>/dev/null)
  pid=$(cat "${d}idProduct" 2>/dev/null)
  case "${vid}:${pid}" in
    3801:0001|add1:0001|174c:2464|174c:2463) ;;
    *) continue ;;
  esac
  found=1
  name=$(basename "$d")
  echo
  echo "--- $name  vid:pid=${vid}:${pid} ---"
  printf "  speed        : %s Mbps%s\n" "$(cat ${d}speed 2>/dev/null)" \
    "$( [ "$(cat ${d}speed 2>/dev/null)" -lt 5000 ] 2>/dev/null && echo '   <== 慢！应为 5000/10000' )"
  printf "  product      : %s\n" "$(cat ${d}product 2>/dev/null)"
  printf "  manufacturer : %s\n" "$(cat ${d}manufacturer 2>/dev/null)"
  printf "  busnum/devnum: %s / %s\n" "$(cat ${d}busnum 2>/dev/null)" "$(cat ${d}devnum 2>/dev/null)"
  printf "  bNumConfigs  : %s   active cfg: %s\n" "$(cat ${d}bNumConfigurations 2>/dev/null)" "$(cat ${d}bConfigurationValue 2>/dev/null)"
  printf "  typec cc     : %s\n" "$(cat /sys/class/power_supply/usb/typec_cc_orientation 2>/dev/null)"
  printf "  devnode      : %s\n" "$(printf '/dev/bus/usb/%03d/%03d' "$(cat ${d}busnum 2>/dev/null)" "$(cat ${d}devnum 2>/dev/null)")"

  # 接口与驱动绑定（关键！）
  echo "  interfaces:"
  for i in "${d}"*/; do
    [ -e "${i}bInterfaceNumber" ] || continue
    ifn=$(cat "${i}bInterfaceNumber" 2>/dev/null)
    cls=$(cat "${i}bInterfaceClass" 2>/dev/null)
    drv=$(basename "$(readlink -f "${i}driver" 2>/dev/null)" 2>/dev/null)
    [ -z "$drv" ] && drv="(none)"
    mark=""
    [ "$drv" != "(none)" ] && mark="   <== 会导致 set_configuration EBUSY"
    printf "    iface %-2s class=%-4s driver=%s%s\n" "$ifn" "$cls" "$drv" "$mark"
    if [ "$FIX" = "1" ] && [ "$drv" != "(none)" ]; then
      echo "$(basename "$i")" > "/sys/bus/usb/drivers/${drv}/unbind" 2>/dev/null \
        && echo "       -> 已 unbind $(basename "$i") from ${drv}" \
        || echo "       -> unbind 失败（需要 root）"
    fi
  done

  # 内核驱动目录里是否有本设备
  echo "  bound drivers (sysfs scan):"
  for b in /sys/bus/usb/drivers/*/; do
    for l in "${b}"*; do
      [ -L "$l" ] || continue
      [ "$(basename "$l")" = "$name" ] && echo "    $(basename "$b") <- $name"
    done
  done
done

if [ "$found" = "0" ]; then
  echo "没有检测到 chestnut eGPU（VID 3801/ADD1 或 ROM 模式 174C:2464/2463）。"
  echo "检查：USB-C 线、12V 供电、底座是否上电。"
  exit 1
fi

echo
echo "===== 提示 ====="
echo "speed < 5000 Mbps 时 ASM2464 无法作为 PCIe 隧道，大模型必然加载失败："
echo "  1) 换一根支持 USB3/Thunderbolt 的线（不是充电线）"
echo "  2) 把 USB-C 插头翻转 180 度再插（换到另一组 SS 差分对）"
echo "  3) 确认 eGPU 底座的 12V 供电"
echo "  4) 若 interfaces 里有 driver != (none)：用 --fix 解绑，或重启后由 egpu_usb_perm.sh 处理"
