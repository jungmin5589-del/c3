#!/usr/bin/env bash
set -u

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

pass=0
fail=0

ok() { echo "[PASS] $*"; pass=$((pass+1)); }
bad() { echo "[FAIL] $*"; fail=$((fail+1)); }

has_topic() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

check_topic() {
  local topic="$1"
  if has_topic "$topic"; then ok "topic exists: $topic"; else bad "topic missing: $topic"; fi
}

rate_once() {
  local topic="$1"
  echo "[RATE] $topic (about 4 seconds)"
  timeout 4s ros2 topic hz "$topic" 2>&1 | tail -n 4 || true
}

check_tf() {
  local parent="$1"
  local child="$2"
  echo "[TF] $parent -> $child"
  if timeout 5s ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -qE 'Translation:|At time'; then
    ok "TF available: $parent -> $child"
  else
    bad "TF unavailable: $parent -> $child"
  fi
}

echo "========== Navigation Stage 1-6 Check =========="
check_topic /clock
check_topic /amr1/odom
check_topic /amr2/odom
check_topic /amr1/scan
check_topic /amr2/scan
check_topic /amr1/camera/front/color/image_raw
check_topic /amr1/camera/rear/color/image_raw
check_topic /amr2/camera/front/color/image_raw
check_topic /amr2/camera/rear/color/image_raw
check_topic /tf
check_topic /tf_static
check_topic /amr1/cmd_vel
check_topic /amr2/cmd_vel

rate_once /clock
rate_once /amr1/odom
rate_once /amr1/scan

echo "[SUBSCRIBERS] /amr1/cmd_vel"
ros2 topic info /amr1/cmd_vel -v 2>/dev/null | sed -n '1,80p'
echo "[SUBSCRIBERS] /amr2/cmd_vel"
ros2 topic info /amr2/cmd_vel -v 2>/dev/null | sed -n '1,80p'

check_tf amr1/odom amr1/base_link
check_tf amr1/base_link amr1/lidar_link
check_tf amr1/base_link amr1/front_camera_link
check_tf amr1/base_link amr1/rear_camera_link
check_tf amr2/odom amr2/base_link
check_tf amr2/base_link amr2/lidar_link
check_tf amr2/base_link amr2/front_camera_link
check_tf amr2/base_link amr2/rear_camera_link

if has_topic /clock; then
  echo "[CLOCK SAMPLE]"
  timeout 3s ros2 topic echo /clock --once 2>/dev/null | sed -n '1,8p' || true
fi
if has_topic /amr1/odom; then
  echo "[ODOM SAMPLE]"
  timeout 3s ros2 topic echo /amr1/odom --once 2>/dev/null | sed -n '1,26p' || true
fi

echo "================================================="
echo "PASS=$pass FAIL=$fail"
if (( fail > 0 )); then
  echo "일부 항목이 실패했습니다. Isaac Sim 통합 실행 창이 켜져 있는지 확인하세요."
  exit 1
fi
