#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
for topic in /cmd_vel /odom /scan /center_goal /amr2/cmd_vel /amr2/odom /amr2/scan /amr2/center_goal /elevator/amr_arrived /elevator/status; do
  printf '%-34s ' "$topic"
  ros2 topic info "$topic" 2>/dev/null | head -1 || true
done
echo "[DOMAIN] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
