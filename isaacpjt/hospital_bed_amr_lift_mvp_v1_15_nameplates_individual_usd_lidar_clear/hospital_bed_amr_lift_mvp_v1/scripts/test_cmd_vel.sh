#!/usr/bin/env bash
set -euo pipefail
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

ROBOT="${1:-amr1}"
MODE="${2:-forward}"
DURATION="${3:-2}"
TOPIC="/${ROBOT}/cmd_vel"

case "$MODE" in
  forward)  X=0.70; Y=0.0;  Z=0.0 ;;
  reverse)  X=-0.70; Y=0.0; Z=0.0 ;;
  left)     X=0.0;  Y=0.90; Z=0.0 ;;
  right)    X=0.0;  Y=-0.90; Z=0.0 ;;
  turn_left)  X=0.0; Y=0.0; Z=1.2 ;;
  turn_right) X=0.0; Y=0.0; Z=-1.2 ;;
  arc_right)  X=0.70; Y=0.0; Z=-0.90 ;;
  *) echo "사용법: $0 amr1|amr2 forward|reverse|left|right|turn_left|turn_right|arc_right [seconds]"; exit 2 ;;
esac

echo "[TEST] $TOPIC mode=$MODE duration=${DURATION}s"
timeout "${DURATION}s" ros2 topic pub -r 20 "$TOPIC" geometry_msgs/msg/Twist \
"{linear: {x: $X, y: $Y, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $Z}}" || true
ros2 topic pub --once "$TOPIC" geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "[STOP] zero Twist sent"
