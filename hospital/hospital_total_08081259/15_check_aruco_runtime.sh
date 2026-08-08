#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AMR="${1:-amr1}"
[[ "$AMR" == "amr1" || "$AMR" == "amr2" ]] || { echo "Usage: $0 [amr1|amr2]" >&2; exit 2; }
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] ./02_build_ros_ws.sh 먼저 실행'; exit 1; }
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

echo "=== ${AMR^^} OCR + ArUco topics ==="
TOPICS="$(ros2 topic list 2>/dev/null || true)"
for T in "/$AMR/camera/front/color/image_raw" "/$AMR/ocr/result" "/$AMR/aruco/result" "/$AMR/aruco/debug_image" "/$AMR/magnet/status"; do
  if grep -Fxq "$T" <<<"$TOPICS"; then echo "[OK] $T"; else echo "[MISS] $T"; fi
done

echo
echo '=== one paired-ArUco result ==='
if grep -Fxq "/$AMR/aruco/result" <<<"$TOPICS"; then
  timeout 4 ros2 topic echo "/$AMR/aruco/result" --once || true
else
  echo "[INFO] 해당 ${AMR^^} OCR/미션 launch를 먼저 실행하세요."
fi

echo
echo "실시간 박스/ID/중앙선 보기: ./17_view_aruco_debug.sh $AMR"
