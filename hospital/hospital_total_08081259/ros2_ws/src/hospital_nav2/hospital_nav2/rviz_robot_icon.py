#!/usr/bin/env python3
"""Publish a lightweight RViz marker for AMR2 without touching Nav2/TF."""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RvizRobotIcon(Node):
    def __init__(self) -> None:
        super().__init__('amr2_rviz_icon')
        self.declare_parameter('world_pose_topic', '/amr2/world_pose')
        self.declare_parameter('marker_topic', '/amr2/rviz_icon')
        self.declare_parameter('frame_id', 'map')
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('world_pose_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        self.pub = self.create_publisher(Marker, marker_topic, 10)
        self.create_subscription(String, topic, self._on_pose, 10)
        self.get_logger().info(f'AMR2 RViz icon: {topic} -> {marker_topic} frame={self.frame_id}')

    def _on_pose(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            x = float(data['x'])
            y = float(data['y'])
            yaw = float(data.get('yaw', 0.0))
        except Exception as exc:
            self.get_logger().warning(f'invalid AMR2 world_pose: {exc}')
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'amr2_icon'
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.12
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)
        marker.scale.x = 0.80
        marker.scale.y = 0.32
        marker.scale.z = 0.20
        marker.color.r = 0.15
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizRobotIcon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
