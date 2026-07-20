#!/usr/bin/env bash
# Copyright 2026 Maize Weeding System Contributors
# SPDX-License-Identifier: MIT

set -euo pipefail

if (( $# < 1 )); then
  echo "usage: $0 OUTPUT_BAG [TOPIC ...]" >&2
  echo "default topics: /livox/lidar /livox/imu" >&2
  exit 2
fi

output_bag=$1
shift
if (( $# == 0 )); then
  topics=(/livox/lidar /livox/imu)
else
  topics=("$@")
fi
if [[ -e "$output_bag" ]]; then
  echo "refusing to overwrite existing bag: $output_bag" >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -n "${LIVOX_WORKSPACE_SETUP:-}" ]]; then
  source "$LIVOX_WORKSPACE_SETUP"
fi
set -u

package_prefix=$(ros2 pkg prefix livox_ros_driver2)
package_share=$(ros2 pkg prefix --share livox_ros_driver2)
health_check="$package_prefix/lib/livox_ros_driver2/check_livox_health.py"
qos_override="$package_share/config/rosbag_livox_qos.yaml"

echo "Running Livox health gate before formal recording..."
"$health_check" --duration 10

echo "Health gate passed. Recording topics: ${topics[*]}"
exec ros2 bag record \
  --max-cache-size 536870912 \
  --qos-profile-overrides-path "$qos_override" \
  -o "$output_bag" \
  "${topics[@]}"
