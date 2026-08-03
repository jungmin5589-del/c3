#!/usr/bin/env bash
set -u

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

TOPICS=(
  /amr1/camera/front/color/image_raw
  /amr1/camera/rear/color/image_raw
  /amr2/camera/front/color/image_raw
  /amr2/camera/rear/color/image_raw
)

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo

for topic in "${TOPICS[@]}"; do
  echo "===== $topic ====="
  if ! ros2 topic type "$topic" >/tmp/amr_camera_type.$$ 2>/dev/null; then
    echo "[FAIL] topic not discovered"
    continue
  fi
  echo "type: $(cat /tmp/amr_camera_type.$$)"
  ros2 topic info -v "$topic" | sed -n '1,35p'
  echo "-- waiting up to 8 s for one Image message --"
  if timeout 8s ros2 topic echo "$topic" --once --field header >/tmp/amr_camera_echo.$$ 2>&1; then
    cat /tmp/amr_camera_echo.$$
    echo "[PASS] Image message received"
  else
    cat /tmp/amr_camera_echo.$$ | tail -n 10
    echo "[FAIL] topic exists but no Image message arrived"
  fi
  echo
done

rm -f /tmp/amr_camera_type.$$ /tmp/amr_camera_echo.$$
