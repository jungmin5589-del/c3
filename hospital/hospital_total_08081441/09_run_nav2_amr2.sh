#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] 먼저 ./02_build_ros_ws.sh 실행" >&2; exit 1; }
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"; fi

echo "============================================================"
echo "[NAV2 AMR2 - STANDALONE] AMR1과 완전 분리 실행"
echo "[DOMAIN] ROS_DOMAIN_ID=115"
echo "[POSE] x=-47.2788 y=26.5713 yaw=0.0 -> AMR1과 같은 자동 Pose Lock"
echo "[MAP] /amr2/map + /amr2/map_server/load_map (AMR1 맵 전환과 독립)"
echo "[LOCAL] /amr2/local_costmap/*"
echo "[NAV] /amr2/controller_server /amr2/follow_path /amr2/center_goal"
echo "[TF] /amr2/tf, /amr2/tf_static"
echo "============================================================"
exec ros2 launch hospital_nav2 hospital_amr2_navigation.launch.py
