#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
ros2 topic list | grep -E '^/(amr1|amr2)/(camera/front/color/image_raw|ocr/request|ocr/result|ocr/control|align/status)$' | sort
