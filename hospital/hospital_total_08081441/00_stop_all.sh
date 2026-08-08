#!/usr/bin/env bash
set -eo pipefail
pkill -f rviz2 2>/dev/null || true
pkill -f pose_lock_localizer 2>/dev/null || true
pkill -f centerline_navigator 2>/dev/null || true
pkill -f "ros2 launch hospital_nav2" 2>/dev/null || true
pkill -f map_server 2>/dev/null || true
pkill -f controller_server 2>/dev/null || true
pkill -f planner_server 2>/dev/null || true
pkill -f bt_navigator 2>/dev/null || true
pkill -f lifecycle_manager 2>/dev/null || true
pkill -f velocity_smoother 2>/dev/null || true
pkill -f hospital_ocr_node 2>/dev/null || true
pkill -f patient_transport_manager.py 2>/dev/null || true
pkill -f path_conflict_manager 2>/dev/null || true
pkill -f "ros2 launch hospital_ocr_bridge" 2>/dev/null || true
pkill -f isaac_amr_ros.py 2>/dev/null || true
pkill -f isaac-sim 2>/dev/null || true
sleep 2
echo "[DONE] Isaac Sim / Nav2 / RViz / OCR / mission processes stopped."
