#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
binary="$root/localization/karto_slam_dora_node/build/karto_slam_dora_node"
test -x "$binary"
if ldd "$binary" | grep -E '/opt/ros|lib(rcl|ros|tf2|ament|rcutils)'; then echo "ROS library linked" >&2; exit 1; fi
if grep -R -E '(^|[^A-Za-z])(rclcpp|sensor_msgs|nav_msgs|geometry_msgs|tf2_ros)([^A-Za-z]|$)' \
  "$root/localization/karto_slam_dora_node" "$root/third_party/slam_toolbox_core" \
  --exclude-dir=build --exclude='*.md'; then echo "ROS source dependency found" >&2; exit 1; fi
if grep -R -E 'scan_matching_dora_node|pose_graph_slam_dora_node|map_rebuilder|map_localization' \
  "$root/apps/adora_nano_navigation"/adora_nano_{mapping,navigation,slam_navigation,localization_navigation}.yml; then
  echo "legacy SLAM node remains in production YAML" >&2; exit 1
fi
if grep -R '/opt/ros' "$root/localization/karto_slam_dora_node/build/CMakeCache.txt"; then echo "ROS CMake path found" >&2; exit 1; fi
if pgrep -fa '(^|/)(roscore|rosmaster|ros2)( |$)' >/dev/null; then echo "ROS process running" >&2; exit 1; fi
echo "ROS_FREE=PASS"
