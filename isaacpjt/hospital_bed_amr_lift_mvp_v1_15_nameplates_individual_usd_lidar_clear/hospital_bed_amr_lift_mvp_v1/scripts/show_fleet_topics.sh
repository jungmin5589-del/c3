#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ros2 topic list | grep -E '^/(clock|tf|tf_static)$|^/(amr1|amr2)/' | sort
