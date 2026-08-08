#!/usr/bin/env python3
"""ROS 2 Nav2 bridge for AMR1 in the hospital Isaac Sim project.

The bridge keeps the existing USD and controller intact. It adds the standard
interfaces expected by Nav2:
- /cmd_vel subscriber
- /odom publisher
- odom -> base_link TF publisher
- /scan publisher from a PhysX rotating lidar
- /clock publisher
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import omni.kit.commands
import omni.timeline
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, Usd, UsdGeom


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _yaw_from_matrix(matrix: Gf.Matrix4d) -> float:
    quat = matrix.ExtractRotationQuat()
    qw = float(quat.GetReal())
    qx, qy, qz = (float(v) for v in quat.GetImaginary())
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _set_stamp(stamp: Any, seconds: float) -> None:
    safe = max(0.0, float(seconds))
    sec = int(safe)
    nanosec = int(round((safe - sec) * 1_000_000_000.0))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


class Nav2Bridge:
    def __init__(self, stage: Usd.Stage, controller: Any, node: Any, cfg: dict[str, Any]) -> None:
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        self.stage = stage
        self.controller = controller
        self.node = node
        self.cfg = cfg
        self.Twist = Twist
        self.Odometry = Odometry
        self.TransformStamped = TransformStamped
        self.TFMessage = TFMessage
        self.LaserScan = LaserScan
        self.Clock = Clock

        self.cmd_vel_topic = str(cfg.get("cmd_vel_topic", "/cmd_vel"))
        self.odom_topic = str(cfg.get("odom_topic", "/odom"))
        self.scan_topic = str(cfg.get("scan_topic", "/scan"))
        self.clock_topic = str(cfg.get("clock_topic", "/clock"))
        self.odom_frame = str(cfg.get("odom_frame", "odom"))
        self.base_frame = str(cfg.get("base_frame", "base_link"))
        self.scan_frame = str(cfg.get("scan_frame", "base_scan"))
        self.tf_topic = str(cfg.get("tf_topic", "/tf"))
        self.publish_clock_enabled = bool(cfg.get("publish_clock", True))
        self.command_timeout = float(cfg.get("command_timeout_sec", 0.5))
        self.odom_period = 1.0 / max(1.0, float(cfg.get("odom_publish_hz", 30.0)))
        self.scan_period = 1.0 / max(1.0, float(cfg.get("scan_publish_hz", 10.0)))
        self.clock_period = 1.0 / max(1.0, float(cfg.get("clock_publish_hz", 60.0)))

        self.latest_command = (0.0, 0.0, 0.0)
        self.last_command_wall_time = -1.0
        self.last_odom_sim_time = -1.0
        self.last_scan_sim_time = -1.0
        self.last_clock_sim_time = -1.0
        self.clock_start_wall_time = time.monotonic()
        self.scan_warning_printed = False

        self.cmd_subscription = node.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._on_cmd_vel,
            20,
        )
        self.odom_publisher = node.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_publisher = node.create_publisher(TFMessage, self.tf_topic, 20)
        self.scan_publisher = node.create_publisher(LaserScan, self.scan_topic, 10)
        self.clock_publisher = node.create_publisher(Clock, self.clock_topic, 10)

        self.timeline = omni.timeline.get_timeline_interface()
        initial_matrix = self._base_world_matrix()
        self.initial_position = initial_matrix.ExtractTranslation()
        self.initial_yaw = _yaw_from_matrix(initial_matrix)

        # Stable odometry anchors. While no motion command is active, small
        # PhysX pose jitter is absorbed instead of being published to RViz.
        self._raw_anchor = (0.0, 0.0, 0.0)
        self._odom_anchor = (0.0, 0.0, 0.0)
        self._odom_output = (0.0, 0.0, 0.0)
        self._was_moving = False

        self.lidar_path = self._create_lidar()
        self.lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

        print(f"[NAV2 SUB] {self.cmd_vel_topic}")
        print(f"[NAV2 PUB] {self.odom_topic} frame={self.odom_frame}->{self.base_frame}")
        print(f"[NAV2 PUB] {self.scan_topic} frame={self.scan_frame}")
        if self.publish_clock_enabled:
            print(f"[NAV2 PUB] {self.clock_topic}")
        else:
            print(f"[NAV2 CLOCK] shared clock; publisher disabled for {self.controller.name}")
        print(f"[NAV2 PUB] {self.tf_topic} frame={self.odom_frame}->{self.base_frame}")
        print(f"[NAV2 LIDAR] {self.lidar_path}")

    def _base_world_matrix(self) -> Gf.Matrix4d:
        return UsdGeom.Xformable(self.controller.base_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

    def _create_lidar(self) -> str:
        lidar_cfg = dict(self.cfg.get("lidar", {}))
        prim_name = str(lidar_cfg.get("prim_name", "nav_lidar"))
        full_path = f"{self.controller.base_path}/{prim_name}"
        if self.stage.GetPrimAtPath(full_path).IsValid():
            self.stage.RemovePrim(full_path)

        translation = lidar_cfg.get("translation", [0.0, 0.0, 0.32])
        result, _prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=f"/{prim_name}",
            parent=self.controller.base_path,
            translation=Gf.Vec3d(float(translation[0]), float(translation[1]), float(translation[2])),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            min_range=float(lidar_cfg.get("min_range_m", 0.15)),
            max_range=float(lidar_cfg.get("max_range_m", 12.0)),
            draw_points=bool(lidar_cfg.get("draw_points", False)),
            draw_lines=bool(lidar_cfg.get("draw_lines", False)),
            horizontal_fov=360.0,
            vertical_fov=float(lidar_cfg.get("vertical_fov_deg", 1.0)),
            horizontal_resolution=float(lidar_cfg.get("horizontal_resolution_deg", 0.5)),
            vertical_resolution=float(lidar_cfg.get("vertical_resolution_deg", 1.0)),
            rotation_rate=float(lidar_cfg.get("rotation_rate_hz", 0.0)),
            high_lod=False,
            yaw_offset=float(lidar_cfg.get("yaw_offset_deg", 0.0)),
            enable_semantics=False,
        )
        if not result:
            raise RuntimeError(f"Could not create PhysX lidar: {full_path}")
        return full_path

    def _on_cmd_vel(self, msg: Any) -> None:
        self.latest_command = (
            float(msg.linear.x),
            0.0,
            float(msg.angular.z),
        )
        self.last_command_wall_time = time.monotonic()

    def get_fresh_command(self) -> tuple[float, float, float] | None:
        if self.last_command_wall_time < 0.0:
            return None
        if time.monotonic() - self.last_command_wall_time > self.command_timeout:
            return None
        max_linear = float(self.cfg.get("max_linear_speed_mps", 0.75))
        max_lateral = float(self.cfg.get("max_lateral_speed_mps", 0.55))
        max_angular = float(self.cfg.get("max_angular_speed_rad_s", 1.2))
        vx, vy, wz = self.latest_command
        return (
            max(-max_linear, min(max_linear, vx)),
            max(-max_lateral, min(max_lateral, vy)),
            max(-max_angular, min(max_angular, wz)),
        )

    def _sim_time(self) -> float:
        return max(0.0, time.monotonic() - self.clock_start_wall_time)

    def publish(self) -> None:
        sim_time = self._sim_time()
        if self.publish_clock_enabled and (
            self.last_clock_sim_time < 0.0 or sim_time - self.last_clock_sim_time >= self.clock_period
        ):
            self._publish_clock(sim_time)
            self.last_clock_sim_time = sim_time
        if self.last_odom_sim_time < 0.0 or sim_time - self.last_odom_sim_time >= self.odom_period:
            self._publish_odom_and_tf(sim_time)
            self.last_odom_sim_time = sim_time
        if self.last_scan_sim_time < 0.0 or sim_time - self.last_scan_sim_time >= self.scan_period:
            self._publish_scan(sim_time)
            self.last_scan_sim_time = sim_time

    def _publish_clock(self, sim_time: float) -> None:
        msg = self.Clock()
        _set_stamp(msg.clock, sim_time)
        self.clock_publisher.publish(msg)

    def _relative_pose(self) -> tuple[float, float, float, float]:
        matrix = self._base_world_matrix()
        position = matrix.ExtractTranslation()
        world_yaw = _yaw_from_matrix(matrix)
        dx = float(position[0] - self.initial_position[0])
        dy = float(position[1] - self.initial_position[1])
        c = math.cos(self.initial_yaw)
        s = math.sin(self.initial_yaw)
        x = c * dx + s * dy
        y = -s * dx + c * dy
        yaw = _normalize_angle(world_yaw - self.initial_yaw)
        return x, y, float(position[2]), yaw

    @staticmethod
    def _pose_delta(anchor: tuple[float, float, float], current: tuple[float, float, float]) -> tuple[float, float, float]:
        ax, ay, ayaw = anchor
        cx, cy, cyaw = current
        dx_world = cx - ax
        dy_world = cy - ay
        c = math.cos(ayaw)
        s = math.sin(ayaw)
        return (
            c * dx_world + s * dy_world,
            -s * dx_world + c * dy_world,
            _normalize_angle(cyaw - ayaw),
        )

    @staticmethod
    def _pose_compose(anchor: tuple[float, float, float], delta: tuple[float, float, float]) -> tuple[float, float, float]:
        ax, ay, ayaw = anchor
        dx, dy, dyaw = delta
        c = math.cos(ayaw)
        s = math.sin(ayaw)
        return (
            ax + c * dx - s * dy,
            ay + s * dx + c * dy,
            _normalize_angle(ayaw + dyaw),
        )

    def _stable_relative_pose(self) -> tuple[float, float, float]:
        raw_x, raw_y, _raw_z, raw_yaw = self._relative_pose()
        raw = (raw_x, raw_y, raw_yaw)
        moving = (
            abs(float(self.controller.current_vx)) > 0.006
            or abs(float(self.controller.current_vy)) > 0.006
            or abs(float(self.controller.current_wz)) > 0.008
        )

        if moving:
            if not self._was_moving:
                self._raw_anchor = raw
                self._odom_anchor = self._odom_output
                self._was_moving = True
            delta = self._pose_delta(self._raw_anchor, raw)
            self._odom_output = self._pose_compose(self._odom_anchor, delta)
        else:
            if self._was_moving:
                delta = self._pose_delta(self._raw_anchor, raw)
                self._odom_output = self._pose_compose(self._odom_anchor, delta)
            # Continuously absorb tiny stationary PhysX movement.
            self._raw_anchor = raw
            self._odom_anchor = self._odom_output
            self._was_moving = False
        return self._odom_output

    def _publish_odom_and_tf(self, sim_time: float) -> None:
        x, y, yaw = self._stable_relative_pose()
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        odom = self.Odometry()
        _set_stamp(odom.header.stamp, sim_time)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.controller.current_vx)
        odom.twist.twist.linear.y = float(self.controller.current_vy)
        odom.twist.twist.angular.z = float(self.controller.current_wz)
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.04
        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[7] = 0.03
        odom.twist.covariance[35] = 0.05
        self.odom_publisher.publish(odom)

        transform = self.TransformStamped()
        _set_stamp(transform.header.stamp, sim_time)
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_publisher.publish(self.TFMessage(transforms=[transform]))

    def _publish_scan(self, sim_time: float) -> None:
        try:
            depth = np.asarray(
                self.lidar_interface.get_linear_depth_data(self.lidar_path),
                dtype=np.float32,
            )
            azimuth = np.asarray(
                self.lidar_interface.get_azimuth_data(self.lidar_path),
                dtype=np.float32,
            )
            if depth.size == 0:
                return

            if depth.ndim > 1:
                depth = depth[depth.shape[0] // 2]
            depth = depth.reshape(-1)

            if azimuth.size == depth.size:
                angles = azimuth.reshape(-1).astype(np.float64)
                if np.nanmax(np.abs(angles)) > 7.0:
                    angles = np.deg2rad(angles)
                angles = np.arctan2(np.sin(angles), np.cos(angles))
                order = np.argsort(angles)
                angles = angles[order]
                ranges = depth[order]
                angle_min = float(angles[0])
                angle_max = float(angles[-1])
                angle_increment = float((angle_max - angle_min) / max(1, len(ranges) - 1))
            else:
                ranges = depth
                angle_min = -math.pi
                angle_max = math.pi
                angle_increment = float((angle_max - angle_min) / max(1, len(ranges) - 1))

            min_range = float(self.cfg.get("lidar", {}).get("min_range_m", 0.15))
            max_range = float(self.cfg.get("lidar", {}).get("max_range_m", 12.0))
            valid = np.isfinite(ranges) & (ranges >= min_range) & (ranges <= max_range)
            clean_ranges = np.where(valid, ranges, np.inf).astype(np.float32)

            msg = self.LaserScan()
            _set_stamp(msg.header.stamp, sim_time)
            msg.header.frame_id = self.scan_frame
            msg.angle_min = angle_min
            msg.angle_max = angle_max
            msg.angle_increment = angle_increment
            msg.time_increment = 0.0
            msg.scan_time = self.scan_period
            msg.range_min = min_range
            msg.range_max = max_range
            msg.ranges = clean_ranges.tolist()
            self.scan_publisher.publish(msg)
        except Exception as exc:
            if not self.scan_warning_printed:
                print(f"[NAV2 WARNING] Lidar scan is not ready yet: {exc}")
                self.scan_warning_printed = True

    def close(self) -> None:
        self.latest_command = (0.0, 0.0, 0.0)
