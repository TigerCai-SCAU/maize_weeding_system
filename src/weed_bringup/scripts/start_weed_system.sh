#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/weed_ws}"

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$WORKSPACE/install/setup.bash"

ros2 launch weed_bringup weed_system.launch.py "$@"
