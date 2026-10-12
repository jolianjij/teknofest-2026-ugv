#!/usr/bin/env bash
# One-shot health check of the whole stack. Read-only.
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/teknofest_ws/install/setup.bash" 2>/dev/null

echo "=== processes ==="
for p in camera_manager_node rplidar_node serial_bridge.py \
         stage_sign_detector_node stage_manager_node \
         path_segmentation_node web_dashboard.py foxglove_bridge; do
  if pgrep -f "[${p:0:1}]${p:1}" > /dev/null 2>&1; then
    printf '  %-28s UP\n' "$p"
  else
    printf '  %-28s DOWN\n' "$p"
  fi
done

echo
echo "=== devices ==="
for d in /dev/esp32 /dev/rplidar /dev/cam_front /dev/cam_aim /dev/cam_rear; do
  if [ -e "$d" ]; then
    printf '  %-16s -> %s\n' "$d" "$(readlink -f "$d")"
  else
    printf '  %-16s MISSING\n' "$d"
  fi
done

echo
echo "=== topic rates (3s each) ==="
for t in /front/image_raw /scan /esp32/imu /front/drivable_mask /front/ground_obstacles; do
  r=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 average)
  printf '  %-28s %s\n' "$t" "${r:-no data}"
done

echo
echo "=== ESP32 state ==="
timeout 5 ros2 topic echo /esp32/diagnostics --once 2>/dev/null | tr '\n' ' ' \
  | sed 's/^/  [loop_hz sbus_ok armed] /' ; echo
