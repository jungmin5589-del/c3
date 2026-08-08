#!/usr/bin/env python3
"""Publish AMR2 initialpose from the absolute Isaac world pose until pose lock succeeds."""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class WorldPoseInitializer(Node):
    def __init__(self) -> None:
        super().__init__('world_pose_initializer')
        self.declare_parameter('world_pose_topic', '/amr2/world_pose')
        self.declare_parameter('initialpose_topic', '/amr2/initialpose')
        self.declare_parameter('lock_topic', '/amr2/initial_pose_locked')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('fixed_pose_enabled', False)
        self.declare_parameter('fixed_x', 0.0)
        self.declare_parameter('fixed_y', 0.0)
        self.declare_parameter('fixed_yaw', 0.0)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initialpose_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('world_pose_topic').value),
            self._on_world_pose,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('lock_topic').value),
            self._on_lock,
            latched,
        )
        self.fixed_pose_enabled = bool(self.get_parameter('fixed_pose_enabled').value)
        self.latest: dict | None = None
        if self.fixed_pose_enabled:
            self.latest = {
                'x': float(self.get_parameter('fixed_x').value),
                'y': float(self.get_parameter('fixed_y').value),
                'yaw': float(self.get_parameter('fixed_yaw').value),
            }
        self.locked = False
        period = max(0.1, float(self.get_parameter('publish_period_sec').value))
        self.create_timer(period, self._tick)
        if self.fixed_pose_enabled:
            self.get_logger().info(
                f"AMR2 fixed initialpose waiting: x={self.latest['x']:.4f}, "
                f"y={self.latest['y']:.4f}, yaw={self.latest['yaw']:.4f}"
            )
        else:
            self.get_logger().info('AMR2 Isaac world pose -> automatic initialpose waiting')

    def _on_world_pose(self, msg: String) -> None:
        if self.fixed_pose_enabled:
            return
        try:
            payload = json.loads(msg.data)
            self.latest = {
                'x': float(payload['x']),
                'y': float(payload['y']),
                'yaw': float(payload['yaw']),
            }
        except Exception as exc:
            self.get_logger().warning(f'invalid world pose: {exc}')

    def _on_lock(self, msg: Bool) -> None:
        self.locked = bool(msg.data)
        if self.locked:
            self.get_logger().info('AMR2 initial pose locked')

    def _tick(self) -> None:
        if self.locked or self.latest is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.latest['x']
        msg.pose.pose.position.y = self.latest['y']
        yaw = self.latest['yaw']
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldPoseInitializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
