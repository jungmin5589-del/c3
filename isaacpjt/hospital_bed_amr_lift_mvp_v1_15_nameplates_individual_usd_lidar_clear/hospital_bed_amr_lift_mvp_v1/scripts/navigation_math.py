#!/usr/bin/env python3
"""Small ROS/Isaac-independent helpers for planar odometry and simulation time."""
from __future__ import annotations

import math


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for a planar Z-axis rotation."""
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)


def relative_planar_pose(
    current_position: tuple[float, float, float],
    current_yaw: float,
    origin_position: tuple[float, float, float],
    origin_yaw: float,
) -> tuple[float, float, float, float]:
    """Express a world pose in an odom frame anchored at the startup pose."""
    dx = float(current_position[0]) - float(origin_position[0])
    dy = float(current_position[1]) - float(origin_position[1])
    dz = float(current_position[2]) - float(origin_position[2])
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    x = c * dx + s * dy
    y = -s * dx + c * dy
    yaw = normalize_angle(float(current_yaw) - float(origin_yaw))
    return x, y, dz, yaw


def split_sim_time(seconds: float) -> tuple[int, int]:
    """Convert nonnegative floating-point seconds to ROS sec/nanosec fields."""
    value = max(0.0, float(seconds))
    sec = int(math.floor(value))
    nanosec = int(round((value - sec) * 1_000_000_000.0))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec
