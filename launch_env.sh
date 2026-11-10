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

AM_POWER_LIMIT_FILE="/data/ford_params/am_power_limit"
if [ -z "${AM_POWER_LIMIT:-}" ] && [ -r "$AM_POWER_LIMIT_FILE" ]; then
  AM_POWER_LIMIT="$(tr -cd '0-9.' < "$AM_POWER_LIMIT_FILE" 2>/dev/null)"
fi
case "${AM_POWER_LIMIT:-}" in
  ''|*[!0-9.]*|*.*.*) AM_POWER_LIMIT=120 ;;
esac
export AM_POWER_LIMIT
unset AM_POWER_LIMIT_FILE

if [ -x /data/openpilot/system/egpu_usb_perm.sh ]; then
  setsid bash /data/openpilot/system/egpu_usb_perm.sh >/dev/null 2>&1 &
fi


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

export C3XL_IFE_ROAD_SIZE=1344x760
