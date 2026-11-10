#!/bin/bash
set -u

SELF="$(readlink -f "$0")"
PIDFILE=/data/egpu_usb_perm.pid
LOG=/data/egpu_usb_perm.log

if [ "$(id -u)" -ne 0 ]; then
  exec sudo -n "$SELF" "$@"
fi

if [ -f "$PIDFILE" ]; then
  OLD=$(cat "$PIDFILE" 2>/dev/null || echo 0)
  if [ -n "$OLD" ] && [ "$OLD" != "$$" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "$(date -u +%FT%TZ) already running pid=$OLD, exit" >>"$LOG"
    exit 0
  fi
fi
echo $$ >"$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT INT TERM

echo "$(date -u +%FT%TZ) egpu_usb_perm start pid=$$" >>"$LOG"

while true; do
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    v=$(cat "$d/idVendor" 2>/dev/null)
    [ "$v" = "3801" ] || [ "$v" = "add1" ] || continue
    b=$(cat "$d/busnum" 2>/dev/null)
    n=$(cat "$d/devnum" 2>/dev/null)
    [ -n "$b" ] && [ -n "$n" ] || continue
    node=$(printf "/dev/bus/usb/%03d/%03d" "$b" "$n")
    if [ -e "$node" ]; then
      cur=$(stat -c '%a' "$node" 2>/dev/null)
      if [ "$cur" != "666" ]; then
        chmod 666 "$node" 2>/dev/null && \
          echo "$(date -u +%FT%TZ) chmod 666 $node (was $cur) speed=$(cat "$d/speed" 2>/dev/null)" >>"$LOG"
      fi
    fi
  done
  sleep 0.5
done
