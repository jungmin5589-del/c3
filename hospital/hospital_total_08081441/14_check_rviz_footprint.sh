#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

echo "[CHECK] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[CHECK] footprint topics"
ros2 topic list | grep -E '^/(local|global)_costmap/published_footprint$' || true

echo "[CHECK] local footprint one message (max 8 sec)"
if timeout 8 ros2 topic echo /local_costmap/published_footprint --once; then
  echo "[PASS] local footprint is being published. RViz display: AMR Footprint (Local)"
else
  echo "[FAIL] no local footprint message. Start terminal 2 first: ./09_run_nav2_amr1.sh" >&2
  exit 1
fi
