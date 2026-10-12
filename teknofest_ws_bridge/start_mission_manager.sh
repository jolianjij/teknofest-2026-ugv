#!/usr/bin/env bash
# Starts mission_manager.py - Phase 1 corridor-following + ExecuteStage
# action server. New 2026-09-08, never yet run on the Pi. Needs
# ugv_stage_interfaces built and sourced (it imports ExecuteStage from it).
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source "$HOME/teknofest_ws/install/setup.bash"
exec python3 mission_manager.py --ros-args --log-level info
