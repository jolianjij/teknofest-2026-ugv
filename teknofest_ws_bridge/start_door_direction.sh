#!/usr/bin/env bash
# Starts door_direction_node.py - sliding-door direction/period tracking
# from the front camera, feeding mission_manager.py's SlidingDoorController
# (Stage 6). New 2026-09-11, never yet run on the Pi - the code has existed
# since 2026-09-03 but had no systemd unit/start script until this session
# wired the commit logic that actually consumes /door/state.
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source "$HOME/teknofest_ws/install/setup.bash" 2>/dev/null || true
exec python3 door_direction_node.py --ros-args --log-level info
