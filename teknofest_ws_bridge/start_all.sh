#!/usr/bin/env bash
# Brings up the whole perception stack on the Orange Pi.
#
# Safe to run unattended: nothing here can command motion. The ESP32 only
# drives the motors when SBUS is bound AND armed, and the serial bridge only
# forwards commands that something else publishes.
#
# Each service is launched with setsid because a plain background job dies
# with the SSH session that started it.
set -u
WS="$HOME/teknofest_ws"

start() {  # start <script> <logname>
  script="$1"; name="$2"
  if pgrep -f "[/]$script" > /dev/null 2>&1; then
    echo "  $name already running"
    return
  fi
  setsid "$WS/$script" > "/tmp/$name.log" 2>&1 < /dev/null &
  disown 2>/dev/null || true
  echo "  started $name"
}

echo "bringing up teknofest stack..."
start start_cameras.sh       cameras
sleep 4
start start_lidar.sh         lidar
start start_serial_bridge.sh serial_bridge
sleep 3
start start_sign_stack.sh    sign_stack
start start_segmentation.sh  segmentation
start start_aim_tracker.sh   aim_tracker
start start_dashboard.sh     dashboard
start start_foxglove.sh      foxglove
sleep 6

IP=$(hostname -I | awk '{print $1}')
echo
echo "dashboard : http://$IP:8080/"
echo "foxglove  : ws://$IP:8765"
echo
echo "check with: ~/teknofest_ws/status_all.sh"
