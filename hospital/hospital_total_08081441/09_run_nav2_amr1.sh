#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_SOURCE="$ROOT/ros2_ws/src/hospital_nav2/maps/hospital_map.yaml"
IMAGE_SOURCE="$ROOT/ros2_ws/src/hospital_nav2/maps/hospital_map.png"

[[ -f "$MAP_SOURCE" ]] || { echo "[ERROR] Missing map YAML: $MAP_SOURCE" >&2; exit 1; }
[[ -f "$IMAGE_SOURCE" ]] || { echo "[ERROR] Missing map image: $IMAGE_SOURCE" >&2; exit 1; }
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || {
  echo "[ERROR] Workspace is not built. Run: ./02_build_ros_ws.sh" >&2
  exit 1
}

grep -qE '^image:[[:space:]]*hospital_map\.png[[:space:]]*$' "$MAP_SOURCE" || {
  echo "[ERROR] hospital_map.yaml must contain: image: hospital_map.png" >&2
  exit 1
}

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

echo "============================================================"
echo "[NAV2] Pose Lock + Corridor/Door Center Navigation"
echo "[NAV2] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ORDER] 1) Isaac PLAY  2) 자동 초기 Pose 잠금 확인  3) 2D Goal Pose"
echo "[POSE] AMR1 x=-45.0467 y=31.8558 yaw=-1.566514 (자동 적용)"
echo "============================================================"
exec ros2 launch hospital_nav2 hospital_amr1_navigation.launch.py
