#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/openpilot/common/hardware/comma/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    while true; do
      $DIR/openpilot/common/hardware/comma/updater $AGNOS_PY $MANIFEST
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD:/data/pylibs"

  # submodule package symlinks for PYTHONPATH imports on device.
  # on PC these come from editable installs via pyproject.toml / uv.
  ln -sfn msgq_repo/msgq msgq
  ln -sfn opendbc_repo/opendbc opendbc
  ln -sfn rednose_repo/rednose rednose
  ln -sfn teleoprtc_repo/teleoprtc teleoprtc
  ln -sfn tinygrad_repo/tinygrad tinygrad

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # ---- Auto-restart manager with crash protection (C3X Recovery) ----
  cd openpilot/system/manager
  MANAGER_RESTART_COUNT=0
  MAX_FAST_RESTARTS=5
  while true; do
    MANAGER_START_TIME=$(date +%s)

    # rebuild if needed
    if [ ! -f $DIR/prebuilt ]; then
      ./build.py
    fi

    # start manager
    if [ -f /AGNOS ]; then
      taskset -c 0-5 ./manager.py
    else
      ./manager.py
    fi
    MANAGER_EXIT_CODE=$?

    # check for shutdown/reboot/uninstall signals
    if [ -f /data/params/d/DoShutdown ] || [ -f /data/params/d/DoReboot ] || [ -f /data/params/d/DoUninstall ]; then
      echo "Shutdown/reboot/uninstall requested, exiting..."
      break
    fi

    # crash protection logic
    MANAGER_END_TIME=$(date +%s)
    MANAGER_RUNTIME=$((MANAGER_END_TIME - MANAGER_START_TIME))

    if [ $MANAGER_RUNTIME -lt 10 ]; then
      MANAGER_RESTART_COUNT=$((MANAGER_RESTART_COUNT + 1))
    else
      MANAGER_RESTART_COUNT=0
    fi

    if [ $MANAGER_RESTART_COUNT -ge $MAX_FAST_RESTARTS ]; then
      echo "Too many fast restarts ($MANAGER_RESTART_COUNT), stopping..."
      break
    fi

    # exponential backoff
    BACKOFF=$((MANAGER_RESTART_COUNT * 5 + 2))
    if [ $BACKOFF -gt 30 ]; then BACKOFF=30; fi
    echo "Manager exited (code=$MANAGER_EXIT_CODE), restarting in ${BACKOFF}s... (attempt $((MANAGER_RESTART_COUNT + 1))/$MAX_FAST_RESTARTS)"
    sleep $BACKOFF
  done

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
