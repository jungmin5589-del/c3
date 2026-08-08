#!/usr/bin/env python3
"""Lock map->odom from the first RViz 2D Pose Estimate.

This intentionally does not run AMCL. The first /initialpose is treated as the
true AMR pose in the map. After that, map->odom stays fixed and odom->base_link
from Isaac Sim carries the motion. A second pose estimate is ignored until this
node is restarted.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class PoseLockLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("pose_lock_localizer")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("broadcast_hz", 30.0)
        self.declare_parameter("auto_initial_pose", True)
        self.declare_parameter("initial_x", -45.0467)
        self.declare_parameter("initial_y", 31.8558)
        self.declare_parameter("initial_yaw", -1.566514)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        hz = max(5.0, float(self.get_parameter("broadcast_hz").value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.lock_pub = self.create_publisher(Bool, "/initial_pose_locked", latched)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, 10)
        self.timer = self.create_timer(1.0 / hz, self._tick)

        self.pending_pose: Optional[PoseWithCovarianceStamped] = None
        self.is_locked = False
        self.locked_transform = TransformStamped()
        self.locked_transform.header.frame_id = self.global_frame
        self.locked_transform.child_frame_id = self.odom_frame
        self.locked_transform.transform.rotation.w = 1.0
        self._publish_lock(False)

        if bool(self.get_parameter("auto_initial_pose").value):
            initial_x = float(self.get_parameter("initial_x").value)
            initial_y = float(self.get_parameter("initial_y").value)
            initial_yaw = float(self.get_parameter("initial_yaw").value)
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = self.global_frame
            initial.pose.pose.position.x = initial_x
            initial.pose.pose.position.y = initial_y
            initial.pose.pose.orientation.z = math.sin(initial_yaw * 0.5)
            initial.pose.pose.orientation.w = math.cos(initial_yaw * 0.5)
            self.pending_pose = initial
            self.get_logger().info(
                f"AMR1 자동 초기 위치 대기: x={initial_x:.4f}, "
                f"y={initial_y:.4f}, yaw={initial_yaw:.6f}rad"
            )
        else:
            self.get_logger().info(
                "첫 2D Pose Estimate 1회만 사용합니다. AMCL/ParticleCloud는 실행하지 않습니다."
            )

    def _publish_lock(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.lock_pub.publish(msg)

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.is_locked:
            self.get_logger().warning(
                "이미 초기 위치가 잠겼습니다. 다시 지정하려면 Nav2를 재시작하세요."
            )
            return
        self.pending_pose = msg
        self.get_logger().info("초기 위치를 받았습니다. odom TF와 결합해 위치를 잠급니다.")

    def _try_lock(self) -> None:
        if self.pending_pose is None or self.is_locked:
            return
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warning(f"odom->base_link 대기 중: {exc}", throttle_duration_sec=2.0)
            return

        desired = self.pending_pose.pose.pose
        yaw_map_base = yaw_from_quaternion(
            desired.orientation.x,
            desired.orientation.y,
            desired.orientation.z,
            desired.orientation.w,
        )
        odom_pose = odom_to_base.transform
        yaw_odom_base = yaw_from_quaternion(
            odom_pose.rotation.x,
            odom_pose.rotation.y,
            odom_pose.rotation.z,
            odom_pose.rotation.w,
        )

        yaw_map_odom = normalize_angle(yaw_map_base - yaw_odom_base)
        c = math.cos(yaw_map_odom)
        s = math.sin(yaw_map_odom)
        rotated_odom_x = c * odom_pose.translation.x - s * odom_pose.translation.y
        rotated_odom_y = s * odom_pose.translation.x + c * odom_pose.translation.y
        tx = desired.position.x - rotated_odom_x
        ty = desired.position.y - rotated_odom_y

        transform = TransformStamped()
        transform.header.frame_id = self.global_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(tx)
        transform.transform.translation.y = float(ty)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(yaw_map_odom * 0.5)
        transform.transform.rotation.w = math.cos(yaw_map_odom * 0.5)
        self.locked_transform = transform
        self.is_locked = True
        self.pending_pose = None
        self._publish_lock(True)
        self.get_logger().info(
            f"초기 위치 잠금 완료: map->odom x={tx:.3f}, y={ty:.3f}, yaw={math.degrees(yaw_map_odom):.1f}deg"
        )

    def _tick(self) -> None:
        self._try_lock()
        self.locked_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.locked_transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseLockLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
