#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "=== REQUIRED TOPICS ==="
for topic in /cmd_vel /odom /scan /tf /tf_static /clock /map; do
  if ros2 topic list | grep -qx "$topic"; then
    echo "[OK] $topic"
  else
    echo "[MISSING] $topic"
  fi
done

echo
echo "=== RATES ==="
timeout 4 ros2 topic hz /odom 2>/dev/null || true
timeout 4 ros2 topic hz /scan 2>/dev/null || true
