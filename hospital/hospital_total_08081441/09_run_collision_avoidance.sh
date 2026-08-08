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
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

pkill -f path_conflict_manager 2>/dev/null || true
sleep 0.3

echo "============================================================"
echo "[TRAFFIC] AMR1/AMR2 실제 centerline path 충돌 회피"
echo "[DOMAIN] ROS_DOMAIN_ID=115"
echo "[RULE 1] OCR/결합/해체/강제직선/엘리베이터 특수동작 AMR = 절대 우선"
echo "[RULE 2] 그 외에는 같은 층 + 미래 path 1.0m 이내 겹침 -> 우선권 결정"
echo "[YIELD] 후순위 AMR은 충돌구간 약 4m 전 정지"
echo "[RESUME] 특수동작 종료 또는 선행 AMR 충돌구간 통과 후 기존 최종 목표 재계산"
echo "[STATUS] ros2 topic echo /traffic_conflict/status"
echo "============================================================"
exec ros2 run hospital_nav2 path_conflict_manager --ros-args \
  -p overlap_distance_m:=1.0 \
  -p hold_trigger_distance_m:=4.0 \
  -p release_clearance_m:=1.2 \
  -p release_delay_sec:=2.0 \
  -p tie_distance_m:=0.35
