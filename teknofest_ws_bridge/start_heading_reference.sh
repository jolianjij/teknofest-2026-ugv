#!/usr/bin/env bash
# Starts heading_reference.py - gyro-zero-on-autonomous-engagement +
# /mission/active latch. New 2026-09-08, never yet run on the Pi.
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source "$HOME/teknofest_ws/install/setup.bash" 2>/dev/null || true
exec python3 heading_reference.py --ros-args --log-level info
