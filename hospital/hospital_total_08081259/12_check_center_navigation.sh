#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "--- nodes ---"
ros2 node list | grep -E 'pose_lock|centerline|controller_server|map_server|rviz' || true
echo "--- lock ---"
timeout 3 ros2 topic echo /initial_pose_locked --once || true
echo "--- essential topics ---"
ros2 topic list | grep -E '^/map$|^/scan$|^/odom$|^/center_goal$|^/centerline_path$|costmap' || true
echo "--- lifecycle ---"
ros2 lifecycle get /map_server || true
ros2 lifecycle get /controller_server || true
