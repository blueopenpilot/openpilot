#!/usr/bin/bash
#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The BASEDIR init file has to exist, with a newer modtime than anything in
  #    the BASEDIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${BASEDIR}/.overlay_init" ]; then
    find ${BASEDIR}/.git -newer ${BASEDIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${BASEDIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $BASEDIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $BASEDIR
          cd $BASEDIR

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
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log
  tmux new-window -n htop
  tmux new-window -n tail_build
  tmux new-window -n tail_launch
  tmux new-window -n nano_build
  tmux new-window -n nano_launch
  tmux new-window -n shell
  tmux send-keys -t htop "htop" Enter
  tmux send-keys -t tail_build "./tail_build_log_last.sh"
  tmux send-keys -t tail_launch "./tail_launch_log_last.sh"
  tmux send-keys -t nano_build "./nano_build_log_last.sh"
  tmux send-keys -t nano_launch "./nano_launch_log_last.sh"
  mv /data/media/build_log_last.txt /data/media/build_log_last2.txt
  mv /data/media/launch_log_last.txt /data/media/launch_log_last2.txt

  # start manager
  cd system/manager
  #Pon: Dump log to file, add rebuild option
  if [ -d "/data/media/0" ]; then
    if [ -d "/data/media/0/log" ]; then
      ##Pon: Waiting for time sync
      #while [ ! $(date +"%Y") == 2024 ]
      #do
      #  date
      #  sleep 1
      #done
      if [ -d "/data/media/0/build" ]; then
        ./build.py > /data/media/0/log/build_log_$(date +"%Y%m%d_%H%M%S").txt && ./manager.py > /data/media/0/log/launch_log_$(date +"%Y%m%d_%H%M%S").txt
      else
        ./manager.py > /data/media/0/log/launch_log_$(date +"%Y%m%d_%H%M%S").txt
      fi
    fi
  else
    echo "[Warning] No SSD to save log!"
  fi
  if [ -d "/data/media/0/build" ]; then
    ./build.py > /data/media/build_log_last.txt && ./manager.py > /data/media/launch_log_last.txt
  else
    ./manager.py > /data/media/launch_log_last.txt
  fi

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
