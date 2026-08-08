#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 topic echo /patient_transfer/status std_msgs/msg/String
