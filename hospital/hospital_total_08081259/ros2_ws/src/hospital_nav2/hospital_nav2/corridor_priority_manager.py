#!/usr/bin/env python3
"""Goal broker for two-AMR narrow-corridor priority without changing Nav2 planners/controllers.

The public goal/status API remains unchanged. In the dual launch only, each centerline
navigator is wired to an internal goal/status topic and this node brokers those topics.
The first generated path that intersects the configured corridor owns it. A second goal
is held at a safe waiting pose. After the owner actually exits the corridor and stays out
for the configured delay, the waiting robot's original goal is re-issued.
"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _make_pose(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(float(yaw) * 0.5)
    msg.pose.orientation.w = math.cos(float(yaw) * 0.5)
    return msg


@dataclass
class Robot:
    name: str
    public_goal: str
    internal_goal: str
    public_status: str
    internal_status: str
    path_topic: str
    world_pose_topic: str
    goal_pub: object = None
    status_pub: object = None
    last_external_goal: Optional[PoseStamped] = None
    pending_goal: Optional[PoseStamped] = None
    candidate_goal: Optional[PoseStamped] = None
    waiting_original_goal: Optional[PoseStamped] = None
    waiting: bool = False
    wait_goal_active: bool = False
    x: Optional[float] = None
    y: Optional[float] = None


class CorridorPriorityManager(Node):
    def __init__(self) -> None:
        super().__init__("corridor_priority_manager")
        defaults = {
            "enabled": True,
            "min_x": -38.5620,
            "max_x": -32.0796,
            "min_y": 10.5645,
            "max_y": 14.2866,
            "path_margin_m": 0.20,
            "exit_margin_m": 0.25,
            "release_delay_sec": 3.0,
            "room_wait_x": -40.0,
            "room_wait_y": 8.0,
            "room_wait_yaw": math.radians(90.0),
            "elevator_wait_x": -30.0,
            "elevator_wait_y": 18.0,
            "elevator_wait_yaw": math.radians(-90.0),
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.path_margin = float(self.get_parameter("path_margin_m").value)
        self.exit_margin = float(self.get_parameter("exit_margin_m").value)
        self.release_delay = max(0.0, float(self.get_parameter("release_delay_sec").value))
        self.wait_poses = {
            "room": (
                float(self.get_parameter("room_wait_x").value),
                float(self.get_parameter("room_wait_y").value),
                float(self.get_parameter("room_wait_yaw").value),
            ),
            "elevator": (
                float(self.get_parameter("elevator_wait_x").value),
                float(self.get_parameter("elevator_wait_y").value),
                float(self.get_parameter("elevator_wait_yaw").value),
            ),
        }

        self.robots = {
            "amr1": Robot(
                "amr1", "/center_goal", "/corridor_priority/amr1/goal",
                "/center_goal/status", "/corridor_priority/amr1/status_raw",
                "/centerline_path", "/amr1/world_pose",
            ),
            "amr2": Robot(
                "amr2", "/amr2/center_goal", "/corridor_priority/amr2/goal",
                "/amr2/center_goal/status", "/corridor_priority/amr2/status_raw",
                "/amr2/centerline_path", "/amr2/world_pose",
            ),
        }

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        for robot in self.robots.values():
            robot.goal_pub = self.create_publisher(PoseStamped, robot.internal_goal, 10)
            robot.status_pub = self.create_publisher(String, robot.public_status, latched)
            self.create_subscription(
                PoseStamped, robot.public_goal,
                lambda msg, name=robot.name: self._on_external_goal(name, msg), 10,
            )
            self.create_subscription(
                String, robot.internal_status,
                lambda msg, name=robot.name: self._on_internal_status(name, msg), latched,
            )
            self.create_subscription(
                Path, robot.path_topic,
                lambda msg, name=robot.name: self._on_path(name, msg), latched,
            )
            self.create_subscription(
                String, robot.world_pose_topic,
                lambda msg, name=robot.name: self._on_world_pose(name, msg), 10,
            )

        self.status_pub = self.create_publisher(String, "/corridor_priority/status", latched)
        self.owner: Optional[str] = None
        self.candidate: Optional[str] = None
        self.owner_entered = False
        self.owner_outside_since: Optional[float] = None
        self.create_timer(0.10, self._tick)
        self._publish_manager_status("READY")
        self.get_logger().info(
            "복도 우선순위 준비: centerline/Nav2 알고리즘은 그대로 두고 goal/status만 중계합니다. "
            f"corridor=({self.min_x:.4f},{self.min_y:.4f})~({self.max_x:.4f},{self.max_y:.4f}), "
            f"release={self.release_delay:.1f}s"
        )

    def _publish_manager_status(self, state: str, detail: str = "") -> None:
        msg = String()
        msg.data = json.dumps(
            {"state": state, "owner": self.owner or "", "candidate": self.candidate or "", "detail": detail},
            ensure_ascii=False, separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _publish_public_status(self, robot: Robot, value: str) -> None:
        msg = String()
        msg.data = value
        robot.status_pub.publish(msg)

    def _on_world_pose(self, name: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.robots[name].x = float(payload["x"])
            self.robots[name].y = float(payload["y"])
        except Exception:
            return

    def _on_external_goal(self, name: str, msg: PoseStamped) -> None:
        robot = self.robots[name]
        robot.last_external_goal = copy.deepcopy(msg)
        if not self.enabled:
            robot.goal_pub.publish(copy.deepcopy(msg))
            return

        # While another robot owns the corridor, preserve the requested destination and
        # send only a safe waiting goal. It will be restored after the 3-second release.
        if self.owner is not None and self.owner != name:
            robot.pending_goal = copy.deepcopy(msg)
            robot.waiting_original_goal = copy.deepcopy(msg)
            self._send_wait_goal(robot)
            return

        # Only one not-yet-classified path is allowed at a time. This prevents two
        # simultaneous paths from entering the corridor before ownership is decided.
        if self.candidate is not None and self.candidate != name:
            robot.pending_goal = copy.deepcopy(msg)
            self._publish_public_status(robot, "ACTIVE:CORRIDOR_PRIORITY_QUEUE")
            self.get_logger().info(f"[{name.upper()}] 다른 AMR 경로 판정까지 목표를 잠시 보류합니다.")
            return

        self._forward_for_classification(robot, msg)

    def _forward_for_classification(self, robot: Robot, msg: PoseStamped) -> None:
        robot.candidate_goal = copy.deepcopy(msg)
        robot.pending_goal = None
        robot.waiting = False
        robot.wait_goal_active = False
        self.candidate = robot.name
        robot.goal_pub.publish(copy.deepcopy(msg))
        self.get_logger().info(f"[{robot.name.upper()}] 목표 전달 -> 생성 경로의 복도 교차 여부 판정")

    def _path_crosses(self, msg: Path, margin: float) -> bool:
        min_x = self.min_x - margin
        max_x = self.max_x + margin
        min_y = self.min_y - margin
        max_y = self.max_y + margin
        return any(
            min_x <= pose.pose.position.x <= max_x
            and min_y <= pose.pose.position.y <= max_y
            for pose in msg.poses
        )

    def _on_path(self, name: str, msg: Path) -> None:
        if not self.enabled or self.candidate != name:
            return
        robot = self.robots[name]
        if robot.wait_goal_active:
            return
        crosses = self._path_crosses(msg, self.path_margin)
        robot.candidate_goal = None
        self.candidate = None
        if crosses:
            self.owner = name
            self.owner_entered = False
            self.owner_outside_since = None
            self.get_logger().info(f"[CORRIDOR OWNER] {name.upper()}가 복도 통행권을 획득했습니다.")
            self._publish_manager_status("RESERVED", f"{name} path crosses corridor")
            self._hold_other_if_pending(name)
        else:
            self.get_logger().info(f"[{name.upper()}] 경로가 지정 복도를 지나지 않습니다. 예약 없음.")
            self._publish_manager_status("FREE", f"{name} path does not cross corridor")
            self._dispatch_next_pending()

    def _hold_other_if_pending(self, owner_name: str) -> None:
        other = self.robots["amr2" if owner_name == "amr1" else "amr1"]
        if other.pending_goal is None:
            return
        other.waiting_original_goal = copy.deepcopy(other.pending_goal)
        other.pending_goal = None
        self._send_wait_goal(other)

    def _send_wait_goal(self, robot: Robot) -> None:
        robot.waiting = True
        if robot.wait_goal_active:
            self._publish_public_status(robot, "ACTIVE:CORRIDOR_WAITING")
            return
        x, y, yaw = self._choose_wait_pose(robot)
        robot.wait_goal_active = True
        robot.goal_pub.publish(_make_pose(self, x, y, yaw))
        self._publish_public_status(robot, "ACTIVE:CORRIDOR_WAIT")
        self.get_logger().info(
            f"[{robot.name.upper()}][WAIT] 복도 통행 대기 -> ({x:.2f}, {y:.2f}); 원래 목표는 보존했습니다."
        )
        self._publish_manager_status("WAITING", f"{robot.name} waits for {self.owner}")

    def _choose_wait_pose(self, robot: Robot) -> tuple[float, float, float]:
        room = self.wait_poses["room"]
        elevator = self.wait_poses["elevator"]
        if robot.x is None or robot.y is None:
            # Fall back to goal side if absolute pose has not arrived yet.
            goal = robot.waiting_original_goal or robot.pending_goal
            if goal is not None:
                gx = float(goal.pose.position.x)
                gy = float(goal.pose.position.y)
                dr = math.hypot(gx - room[0], gy - room[1])
                de = math.hypot(gx - elevator[0], gy - elevator[1])
                return room if dr <= de else elevator
            return room
        dr = math.hypot(robot.x - room[0], robot.y - room[1])
        de = math.hypot(robot.x - elevator[0], robot.y - elevator[1])
        return room if dr <= de else elevator

    def _on_internal_status(self, name: str, msg: String) -> None:
        robot = self.robots[name]
        status = str(msg.data)
        # Do not let a waiting-pose SUCCEEDED fool an external mission manager into
        # thinking its original destination was reached.
        if robot.waiting:
            if status.startswith("FAILED"):
                self._publish_public_status(robot, "ACTIVE:CORRIDOR_WAITING")
            elif status == "SUCCEEDED":
                robot.wait_goal_active = False
                self._publish_public_status(robot, "ACTIVE:CORRIDOR_WAITING")
                self.get_logger().info(f"[{name.upper()}][WAITING] 대기 위치 도착, 복도 해제 대기")
            else:
                self._publish_public_status(robot, "ACTIVE:CORRIDOR_WAIT")
            return
        self._publish_public_status(robot, status)
        if name == self.owner and status.startswith("FAILED") and not self.owner_entered:
            self.get_logger().warning(f"[{name.upper()}] 복도 진입 전 경로 실패 -> 예약 해제")
            self._release_corridor("owner failed before entering")

    def _inside(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.min_x - margin <= x <= self.max_x + margin
            and self.min_y - margin <= y <= self.max_y + margin
        )

    def _tick(self) -> None:
        if not self.enabled or self.owner is None:
            return
        robot = self.robots[self.owner]
        if robot.x is None or robot.y is None:
            return
        now = time.monotonic()
        inside = self._inside(robot.x, robot.y, self.exit_margin)
        if not self.owner_entered:
            if inside:
                self.owner_entered = True
                self.owner_outside_since = None
                self.get_logger().info(f"[{self.owner.upper()}] 실제 복도 진입 확인")
                self._publish_manager_status("OCCUPIED")
            return
        if inside:
            self.owner_outside_since = None
            return
        if self.owner_outside_since is None:
            self.owner_outside_since = now
            self.get_logger().info(
                f"[{self.owner.upper()}] 복도 이탈 확인 -> {self.release_delay:.1f}초 후 대기 AMR 재개"
            )
            return
        if now - self.owner_outside_since >= self.release_delay:
            self._release_corridor("owner stayed outside for release delay")

    def _release_corridor(self, reason: str) -> None:
        old_owner = self.owner
        self.owner = None
        self.owner_entered = False
        self.owner_outside_since = None
        self.candidate = None
        self.get_logger().info(f"[CORRIDOR FREE] {reason}")
        self._publish_manager_status("FREE", reason)

        # Re-issue exactly the waiting robot's original goal after the release delay.
        if old_owner is not None:
            other = self.robots["amr2" if old_owner == "amr1" else "amr1"]
            if other.waiting_original_goal is not None:
                goal = copy.deepcopy(other.waiting_original_goal)
                other.waiting_original_goal = None
                other.waiting = False
                other.wait_goal_active = False
                self.get_logger().info(f"[{other.name.upper()}][RESUME] 원래 목표를 다시 발행합니다.")
                self._forward_for_classification(other, goal)
                return
        self._dispatch_next_pending()

    def _dispatch_next_pending(self) -> None:
        if self.owner is not None or self.candidate is not None:
            return
        for name in ("amr1", "amr2"):
            robot = self.robots[name]
            if robot.pending_goal is not None:
                goal = copy.deepcopy(robot.pending_goal)
                robot.pending_goal = None
                self._forward_for_classification(robot, goal)
                return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorPriorityManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
