#!/usr/bin/env bash
set -euo pipefail
PATIENT="${1:-}"
COMMAND="${2:-}"
if [[ -z "$PATIENT" || -z "$COMMAND" ]]; then
  echo "사용법: $0 patient1 TO_MRI"
  echo "명령: TO_MRI | TO_TRANSPORT | TOGGLE | RESET_ARM | STATUS"
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 topic pub --once "/patient_transfer/${PATIENT}/command" std_msgs/msg/String "{data: '${COMMAND}'}"
