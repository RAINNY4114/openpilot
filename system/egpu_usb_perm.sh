#!/usr/bin/env bash
# ============================================================================
# chestnut (ASM2464 / VID 3801) eGPU USB 节点权限守护
#
# 背景（2026-09-23 定案）:
#   冷启动时 devtmpfs 以 0600 root:root 创建 /dev/bus/usb/<bus>/<dev>，
#   udev 要等 builtin usb_id 把设备描述符读完，才按
#   /usr/lib/udev/rules.d/50-udev-default.rules:72 设成 0664 root:root。
#   当 eGPU 底座和 C3XL 同时上电时，ASM2464 刚上电、描述符响应慢，
#   这个窗口能拖到几十秒；而 modeld 以 comma 身份在开机 T+34s 就
#   libusb_open 了 -> EACCES -> "libusb_open: Access denied" ->
#   load_big() 只试一次即放弃 -> 静默回退小模型（ChestnutModelError=True）。
#   comma 在 root 组，所以只要节点到 0664 就能打开 —— 这解释了
#   "开机后先 OFFROAD 再 ONROAD 必然成功"（那时 udev 早处理完了）。
#
#   本脚本抢在 udev 前面把节点权限放开，让 modeld 根本不必等。
#
# 幂等：可重复执行；同一个 (bus,devnum) 只处理一次。
# 停止：pkill -f "egpu_usb_per[m]"
# ============================================================================
LOG=/data/egpu_usb_perm.log
VENDOR=3801

# comma.service 是 User=comma（ExecStart=tmux ... /usr/comma/comma.sh），所以
# continue.sh 及其子进程都以 comma 身份跑；而 /dev/bus/usb/*/* 属 root，
# comma 直接 chmod 会 EPERM（现象：日志里只有 [start]、没有 chmod 行）。
# comma 有 NOPASSWD sudo，这里自提权到 root 再干活。
if [ "$(id -u)" -ne 0 ]; then
  if sudo -n true 2>/dev/null; then
    exec sudo -n "$0" "$@"
  fi
  echo "$(date -u +%FT%TZ 2>/dev/null) [warn] uid=$(id -u) 非 root 且 sudo 不可用 -> chmod 会失败" >> "$LOG"
fi

# ---- 单实例保护（2026-09-24 补）----
# launch_env.sh 不只在开机时被 source：仓库里 openpilot/system/updated/updated.py:199
# 每次检查 AGNOS 版本都会 `bash -c "source launch_env.sh && echo $AGNOS_VERSION"`，
# 而 updated 在版本不符时会一直重试（失败 5 分钟一轮）。于是下面这个 setsid 钩子
# 被反复拉起：实测 uptime 820 / 1003 / 1095 / 1175s 各起一个，一趟长途能累积几百个
# 进程，每个都在 0.5s 轮询 sysfs。加锁只保留一个。
# /tmp 是 tmpfs，重启自然清空；持有者已死则自动接管。
PIDFILE=/tmp/egpu_usb_perm.pid
if [ -f "$PIDFILE" ]; then
  holder=$(cat "$PIDFILE" 2>/dev/null)
  if [ -n "$holder" ] && kill -0 "$holder" 2>/dev/null; then
    exit 0
  fi
fi
echo $$ > "$PIDFILE" 2>/dev/null || true
trap 'rm -f "$PIDFILE"' EXIT INT TERM

echo "$(date -u +%FT%TZ 2>/dev/null) [start] pid=$$ uid=$(id -u) uptime=$(cut -d' ' -f1 /proc/uptime)s" >> "$LOG"

seen=" "
while :; do
  for d in /sys/bus/usb/devices/*/; do
    # [egpu-busyfix] 原来的 = "$VENDOR" 只认 3801，漏掉 add1:0001（同一颗 ASM2464
    # 的另一组 VID:PID，CHESTNUT_USB_IDS 里两组都算 chestnut）。
    case "$(cat "${d}idVendor" 2>/dev/null)" in 3801|add1) ;; *) continue ;; esac
    b=$(cat "${d}busnum" 2>/dev/null)
    n=$(cat "${d}devnum" 2>/dev/null)
    [ -n "$b" ] && [ -n "$n" ] || continue
    key="$b-$n"
    case "$seen" in *" $key "*) continue ;; esac
    node=$(printf '/dev/bus/usb/%03d/%03d' "$b" "$n")
    [ -e "$node" ] || continue
    before=$(stat -c '%a %U:%G' "$node" 2>/dev/null)
    if chmod 666 "$node" 2>/dev/null; then
      seen="$seen$key "
      echo "$(date -u +%FT%TZ 2>/dev/null) chmod 666 $node  (was: $before)  uptime=$(cut -d' ' -f1 /proc/uptime)s" >> "$LOG"
    fi
  done

  # [egpu-busyfix] 解绑内核驱动 —— 每轮都复查，不能只做一次。
  # 内核 USBDEVFS_SETCONFIGURATION 的实现 (drivers/usb/core/devio.c:proc_setconfig)
  # 只要「当前配置里任意一个接口有内核驱动绑定」就返回 -EBUSY；而 tinygrad 的
  # USB3.__init__ 只 detach interface 0。ASM2464 会暴露一个 mass-storage 接口，
  # AGNOS 开机就注册 usb-storage，并且它可能在 udev 处理完、甚至 modeld 起来之后
  # 才绑上。结果是：
  #   ExceptionGroup: No interface for AMD:0 is available
  #     +- RuntimeError: libusb_set_configuration: Resource busy
  # 大模型加载直接失败。这里以 root 身份在设备被打开前把驱动摘掉。
  for d in /sys/bus/usb/devices/*/; do
    case "$(cat "${d}idVendor" 2>/dev/null):$(cat "${d}idProduct" 2>/dev/null)" in
      3801:0001|add1:0001|174c:2464|174c:2463) ;;
      *) continue ;;
    esac
    for intf in "${d}"*/; do
      [ -e "${intf}bInterfaceNumber" ] || continue
      drv=$(basename "$(readlink -f "${intf}driver" 2>/dev/null)" 2>/dev/null)
      [ -n "$drv" ] || continue
      # [egpu-usbfs-fix 20261017] 绝不 unbind usbfs。usbfs = 用户态驱动占用：
      # modeld 的 tinygrad USB3.__init__ 在 libusb_claim_interface(0) 后正处于
      # AMDev bring-up（control/bulk 传输 + 链路训练重试）。unbind usbfs 会把
      # 已 claim 的接口从该进程脚下踢走 -> 后续 USB 操作失败/挂起 ->
      # bring-up 无异常卡死 -> join(BIG_MODEL_TIMEOUT)=150s 超时才回退小模型
      # （2026-10-17 实测：unbind 风暴与大模型加载窗口重叠后必然挂死）。
      # 本钩子的设计目标只是摘掉 usb-storage 这类内核驱动（避免
      # USBDEVFS_SETCONFIGURATION 返回 -EBUSY），usbfs 一律跳过。
      [ "$drv" = "usbfs" ] && continue
      if echo "$(basename "$intf")" > "/sys/bus/usb/drivers/${drv}/unbind" 2>/dev/null; then
        echo "$(date -u +%FT%TZ 2>/dev/null) unbind $(basename "$intf") from $drv  uptime=$(cut -d' ' -f1 /proc/uptime)s" >> "$LOG"
      fi
    done
  done

  sleep 0.5
done
