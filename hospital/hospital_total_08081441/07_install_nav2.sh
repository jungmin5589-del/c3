#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools
echo "[DONE] Nav2 packages installed"
