#!/usr/bin/env bash
# Deploys everything changed today (2026-09-09) to the Orange Pi and restarts
# the affected services. Run from Git Bash on Windows, from the repo root.
#
# The Pi's connection has been flaky - each phase below is independent and
# safe to re-run on its own if one drops mid-way (scp overwrites, services
# check pgrep before starting). Run phase by phase if the whole thing won't
# go through in one shot: `bash deploy_updates.sh phase1`, `phase2`, etc.
# With no argument, runs everything in order.
set -u
PI="${PI:-orangepi}"
WS=teknofest_ws_bridge

phase1_python_files() {
  echo "== phase 1: teknofest_ws_bridge python files =="
  scp \
    "$WS/serial_bridge.py" \
    "$WS/aim_tracker.py" \
    "$WS/target_detection.py" \
    "$WS/camera_manager_node.py" \
    "$WS/test_aim_pipeline.py" \
    "$WS/web_dashboard.py" \
    "$WS/mission_recorder.py" \
    "$WS/heading_reference.py" \
    "$WS/mission_manager.py" \
    "$WS/mission_logic.py" \
    "$WS/test_mission_logic.py" \
    "$WS/nav2_goal_feeder.py" \
    "$WS/door_tracking.py" \
    "$WS/door_direction_node.py" \
    "$WS/start_heading_reference.sh" \
    "$WS/start_mission_manager.sh" \
    "$WS/start_door_direction.sh" \
    "$PI:~/teknofest_ws/" || return 1
  ssh "$PI" "chmod +x ~/teknofest_ws/start_heading_reference.sh ~/teknofest_ws/start_mission_manager.sh ~/teknofest_ws/start_door_direction.sh"

  # Systemd unit files (added 2026-09-11, after the whole stack moved off
  # the old setsid/start_all.sh pattern) - previously had no deploy path at
  # all, meaning a unit-file edit needed a manual scp+cp+daemon-reload
  # remembered separately. Installing them is a no-op if nothing changed
  # (daemon-reload is cheap and idempotent), so this is safe to run every
  # time, not just when a unit file actually changed.
  scp "$WS/systemd/"*.service "$PI:~/teknofest_ws/systemd/" || return 1
  ssh "$PI" "sudo cp ~/teknofest_ws/systemd/*.service /etc/systemd/system/ && sudo systemctl daemon-reload"

  # teknofest-door-direction.service is NEW this deploy (2026-09-11) - the
  # code existed since 2026-09-03 but had no unit file until
  # mission_manager.py's SlidingDoorController gave /door/state an actual
  # consumer. The other services above were already enabled from an earlier
  # session, so phase4's `systemctl restart` is enough for them; this one
  # needs an explicit first-time enable so it also survives a reboot like
  # everything else. Idempotent - safe to re-run.
  ssh "$PI" "sudo systemctl enable teknofest-door-direction.service"
}

phase2_stage_sign_package() {
  echo "== phase 2: ugv_stage_sign_detector package (mission.py, stage_manager_node.py, tests) =="
  SRC="/c/Users/jolian/Downloads/UGV_Stage_Sign_Detector_v0.3.0/ugv-stage-sign-detector/ros2_ws/src/ugv_stage_sign_detector"
  scp \
    "$SRC/ugv_stage_sign_detector/mission.py" \
    "$SRC/ugv_stage_sign_detector/stage_manager_node.py" \
    "$PI:~/teknofest_ws/src/ugv_stage_sign_detector/ugv_stage_sign_detector/" || return 1
  scp \
    "$SRC/test/test_mission.py" \
    "$PI:~/teknofest_ws/src/ugv_stage_sign_detector/test/" || return 1
  echo "rebuilding ugv_stage_sign_detector (colcon)..."
  ssh "$PI" "cd ~/teknofest_ws && source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select ugv_stage_sign_detector --symlink-install"
}

phase3_camera_udev_check() {
  echo "== phase 3: verify the camera front/rear udev fix is still in place =="
  # This was already applied live on the Pi via SSH earlier - not a file to
  # push, just confirming it survived (a reboot or a fresh udev reload could
  # theoretically need it redone, though it's written to disk so it shouldn't).
  ssh "$PI" "grep SYMLINK /etc/udev/rules.d/99-teknofest-cameras.rules"
}

phase4_restart_services() {
  echo "== phase 4: restart everything so new code actually takes effect =="
  # REWRITTEN 2026-09-11: the whole stack moved to systemd services
  # (teknofest-*.service, all Restart=always and enabled for boot) earlier
  # this session. The old pkill+setsid dance below is now actively
  # dangerous here - systemd would just relaunch anything pkilled almost
  # instantly (Restart=always), racing this script's own setsid launches
  # and recreating exactly the duplicate-process chaos that caused the
  # lidar-starving incident this same approach was originally hardened
  # against. `systemctl restart` is the correct, race-free equivalent now.
  ssh "$PI" "sudo systemctl restart teknofest-serial-bridge.service teknofest-dashboard.service teknofest-aim-tracker.service teknofest-cameras.service teknofest-mission-manager.service teknofest-heading-reference.service teknofest-door-direction.service; sleep 3; echo restarted"
}

phase5_status() {
  echo "== phase 5: status check =="
  ssh "$PI" "systemctl is-active teknofest-cameras teknofest-lidar teknofest-serial-bridge teknofest-sign-stack teknofest-aim-tracker teknofest-dashboard teknofest-foxglove teknofest-heading-reference teknofest-mission-manager teknofest-door-direction; \
    echo '--- duplicate check (anything with count > 1 is a real problem - systemd should never do this, but confirm) ---'; \
    ps -eo args | grep -E 'python3 (serial_bridge|web_dashboard|aim_tracker|camera_manager_node|mission_manager|heading_reference|door_direction_node)\.py' | sort | uniq -c; \
    echo '--- load average ---'; uptime; \
    echo '--- lidar (SL_RESULT_OPERATION_TIMEOUT here means check the physical unit is spinning) ---'; \
    tail -5 /tmp/lidar.log"
}

# NOT part of "all" below - these two are explicit-only. phase6 pushes the
# Nav2/URDF package (robot_description/ - never had an automated deploy
# path before 2026-09-11, unlike everything else in this script); phase7
# actually launches Nav2 on real hardware and hands it driving authority,
# which is consequential enough that it should never fire from a routine
# `bash deploy_updates.sh` with no arguments.
phase6_robot_description() {
  echo "== phase 6: robot_description package (URDF, Nav2/EKF sim+hardware configs) =="
  ssh "$PI" "mkdir -p ~/teknofest_ws/src/teknofest_description" || return 1
  scp -r robot_description/. "$PI:~/teknofest_ws/src/teknofest_description/" || return 1
  echo "checking rf2o_laser_odometry is installed (nav2_hardware.launch.py's odometry source - not a core ROS2 package, easy to be missing)..."
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && ros2 pkg list 2>/dev/null | grep -q rf2o_laser_odometry && echo 'rf2o_laser_odometry: present' || echo 'rf2o_laser_odometry: MISSING - install with: sudo apt install ros-jazzy-rf2o-laser-odometry'"
  echo "rebuilding teknofest_description (colcon)..."
  ssh "$PI" "cd ~/teknofest_ws && source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select teknofest_description --symlink-install"
}

phase7_launch_nav2_hardware() {
  echo "== phase 7: launch Nav2 on real hardware (NEVER RUN BEFORE, EVEN IN SIM) =="
  echo "This hands driving authority to Nav2 - stopping mission_manager.py first"
  echo "so the two don't fight over /cmd_vel (see CLAUDE.md's Stage 5 entry)."
  ssh "$PI" "sudo systemctl stop teknofest-mission-manager.service" || return 1
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && source ~/teknofest_ws/install/setup.bash && \
    setsid ros2 launch teknofest_description nav2_hardware.launch.py > /tmp/nav2_hardware.log 2>&1 < /dev/null & disown"
  sleep 5
  # nav2_goal_feeder.py (2026-09-11) - without this, every Nav2 server above
  # comes up correctly but the robot never moves: there's no persistent map,
  # so nothing sends bt_navigator/NavigateToPose an actual goal. This feeds
  # it a continuous "drive forward from wherever you are" goal instead - see
  # that file's own docstring. Started separately (plain python3, not part
  # of the ament launch file) since it isn't a colcon-installed executable,
  # matching mission_manager.py/heading_reference.py's own deploy pattern.
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && source ~/teknofest_ws/install/setup.bash && \
    cd ~/teknofest_ws && setsid python3 nav2_goal_feeder.py > /tmp/nav2_goal_feeder.log 2>&1 < /dev/null & disown"
  sleep 2
  ssh "$PI" "pgrep -af 'controller_server|planner_server|bt_navigator|ekf_node|rf2o_laser_odometry|nav2_goal_feeder' || echo 'NOTHING RUNNING - check /tmp/nav2_hardware.log'; \
    echo '--- nav2 log tail ---'; tail -30 /tmp/nav2_hardware.log; \
    echo '--- goal feeder log tail ---'; tail -15 /tmp/nav2_goal_feeder.log"
}

phase8_stop_nav2_restore_mission_manager() {
  echo "== phase 8: stop Nav2, hand driving back to mission_manager.py =="
  ssh "$PI" "pkill -9 -f 'nav2_hardware.launch.py'; \
    pkill -9 -f 'nav2_goal_feeder.py'; \
    pkill -9 -f 'controller_server'; pkill -9 -f 'planner_server'; pkill -9 -f 'behavior_server'; \
    pkill -9 -f 'bt_navigator'; pkill -9 -f 'waypoint_follower'; pkill -9 -f 'velocity_smoother'; \
    pkill -9 -f 'collision_monitor'; pkill -9 -f 'lifecycle_manager'; \
    pkill -9 -f 'rf2o_laser_odometry'; pkill -9 -f 'ekf_node'; pkill -9 -f 'robot_state_publisher'; \
    sleep 2; echo stopped"
  ssh "$PI" "sudo systemctl start teknofest-mission-manager.service; sleep 2; systemctl is-active teknofest-mission-manager.service"
}

case "${1:-all}" in
  phase1) phase1_python_files ;;
  phase2) phase2_stage_sign_package ;;
  phase3) phase3_camera_udev_check ;;
  phase4) phase4_restart_services ;;
  phase5) phase5_status ;;
  phase6) phase6_robot_description ;;
  phase7) phase7_launch_nav2_hardware ;;
  phase8) phase8_stop_nav2_restore_mission_manager ;;
  all)
    phase1_python_files && \
    phase2_stage_sign_package && \
    phase3_camera_udev_check && \
    phase4_restart_services && \
    phase5_status
    ;;
  *) echo "usage: $0 [phase1|phase2|phase3|phase4|phase5|phase6|phase7|phase8|all]"; exit 1 ;;
esac
