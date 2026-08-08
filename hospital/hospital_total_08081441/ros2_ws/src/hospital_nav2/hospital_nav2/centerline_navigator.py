#!/usr/bin/env python3
"""Plan wall-clear, corridor-centered, door-centered paths and follow them segment by segment.

RViz publishes /center_goal. This node:
1. snaps start and goal to safe, high-clearance cells,
2. runs 4-connected A* with wall-clearance and turn penalties,
3. splits the route into straight segments,
4. sends each segment to Nav2 FollowPath,
5. publishes zero velocity at every corner and for two seconds at the final goal.
"""
from __future__ import annotations

from collections import deque
import copy
import heapq
import math
import time
from typing import Iterable, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def make_pose(frame: str, stamp, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose


class CenterlineNavigator(Node):
    DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]

    def __init__(self) -> None:
        super().__init__("centerline_navigator")
        defaults = {
            "global_frame": "map",
            "base_frame": "base_link",
            "goal_topic": "/center_goal",
            "path_topic": "/centerline_path",
            "cmd_vel_topic": "/cmd_vel",
            "follow_path_action": "/follow_path",
            "downsample_factor": 2,
            "robot_safe_radius_m": 0.52,
            "goal_snap_radius_m": 2.0,
            "center_weight": 0.55,
            "turn_penalty": 2.5,
            "path_point_spacing_m": 0.10,
            "corner_stop_sec": 0.55,
            "final_stop_sec": 2.0,
            "map_topic": "/map",
            "pose_lock_topic": "/initial_pose_locked",
            "status_topic": "/center_goal/status",
            "traffic_pause_topic": "",
            "rotate_before_segment": True,
            "rotate_max_speed_rad_s": 0.45,
            "rotate_min_speed_rad_s": 0.10,
            "rotate_kp": 1.25,
            "rotate_tolerance_rad": 0.035,
            "rotate_stable_cycles": 5,
            "retry_delay_sec": 0.8,
            "max_follow_path_retries": 0,
            "near_segment_success_tolerance_m": 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.action_name = str(self.get_parameter("follow_path_action").value)
        self.factor = max(1, int(self.get_parameter("downsample_factor").value))
        self.robot_safe_radius = float(self.get_parameter("robot_safe_radius_m").value)
        self.safe_radius = self.robot_safe_radius
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pose_lock_topic = str(self.get_parameter("pose_lock_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.traffic_pause_topic = str(self.get_parameter("traffic_pause_topic").value).strip()
        self.snap_radius = float(self.get_parameter("goal_snap_radius_m").value)
        self.center_weight = float(self.get_parameter("center_weight").value)
        self.turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.spacing = max(0.05, float(self.get_parameter("path_point_spacing_m").value))
        self.corner_stop_sec = float(self.get_parameter("corner_stop_sec").value)
        self.final_stop_sec = float(self.get_parameter("final_stop_sec").value)
        self.rotate_before_segment = bool(self.get_parameter("rotate_before_segment").value)
        self.rotate_max_speed = max(0.05, float(self.get_parameter("rotate_max_speed_rad_s").value))
        self.rotate_min_speed = max(0.02, float(self.get_parameter("rotate_min_speed_rad_s").value))
        self.rotate_kp = max(0.1, float(self.get_parameter("rotate_kp").value))
        self.rotate_tolerance = max(0.005, float(self.get_parameter("rotate_tolerance_rad").value))
        self.rotate_stable_cycles = max(1, int(self.get_parameter("rotate_stable_cycles").value))
        self.retry_delay_sec = max(0.1, float(self.get_parameter("retry_delay_sec").value))
        configured_retries = int(self.get_parameter("max_follow_path_retries").value)
        # This project historically used 0 to mean unlimited. For the final demo,
        # never allow an unbounded retry loop: 0 now falls back to two retries.
        self.max_follow_path_retries = 2 if configured_retries <= 0 else configured_retries
        self.near_segment_success_tolerance = max(0.05, float(self.get_parameter("near_segment_success_tolerance_m").value))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(Bool, self.pose_lock_topic, self._on_pose_lock, map_qos)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        if self.traffic_pause_topic:
            self.create_subscription(Bool, self.traffic_pause_topic, self._on_traffic_pause, map_qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, map_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        self.status_pub = self.create_publisher(String, self.status_topic, map_qos)
        self.action_client = ActionClient(self, FollowPath, self.action_name)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self._tick)

        self.pose_locked = False
        self.map_msg: Optional[OccupancyGrid] = None
        # 2F map is 1512x841 in this project. Keep 2F on the proven exact-corner behavior.
        self.is_2f_map = False
        self.coarse_free: Optional[np.ndarray] = None
        self.clearance_m: Optional[np.ndarray] = None
        self.coarse_resolution = 0.0
        self.segments: list[list[tuple[float, float]]] = []
        self.segment_index = 0
        self.final_yaw = 0.0
        self.keep_arrival_heading = False
        self.active_goal_handle = None
        self.pending_goal: Optional[PoseStamped] = None
        self.next_segment_time = 0.0
        self.stop_until = 0.0
        self.busy = False
        self.final_goal_msg: Optional[PoseStamped] = None
        self.retry_count = 0
        self.retry_plan_time = float("inf")
        self.traffic_paused = False
        self.pause_cancel_inflight = False
        self.resume_replan_pending = False
        self.rotate_active = False
        self.rotate_target_yaw = 0.0
        self.rotate_mode = ""
        self.rotate_stable_count = 0
        self.segment_rotation_done = False
        self.get_logger().info(
            f"중앙 경로 노드 준비: AMR footprint radius={self.safe_radius:.2f}m, "
            f"회전 progress 제약=OFF, FollowPath 자동 재시도={self.max_follow_path_retries}회, "
            f"segment 근접 성공={self.near_segment_success_tolerance:.2f}m"
        )
        self._publish_status("READY:WAITING_POSE")

    def _publish_status(self, value: str) -> None:
        msg = String()
        msg.data = str(value)
        self.status_pub.publish(msg)

    def _on_pose_lock(self, msg: Bool) -> None:
        self.pose_locked = bool(msg.data)
        self._publish_status("READY" if self.pose_locked else "READY:WAITING_POSE")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        h, w = msg.info.height, msg.info.width
        # The floor maps are unchanged from the proven baseline: 1F=1528x841, 2F=1512x841.
        # Detect 2F from the loaded OccupancyGrid itself; no new topic/TF is introduced.
        self.is_2f_map = (int(w) == 1512 and int(h) == 841)
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        free = grid == 0
        f = self.factor
        hc, wc = h // f, w // f
        cropped = free[: hc * f, : wc * f]
        # A coarse cell is free only when every underlying cell is free.
        coarse = cropped.reshape(hc, f, wc, f).all(axis=(1, 3))
        self.coarse_free = coarse
        self.coarse_resolution = float(msg.info.resolution) * f
        self.clearance_m = self._distance_transform(coarse) * self.coarse_resolution
        self.get_logger().info(
            f"지도 준비: {w}x{h}, 중앙 경로 격자={wc}x{hc}, 해상도={self.coarse_resolution:.2f}m, "
            f"floor={'2F exact' if self.is_2f_map else '1F responsive'}"
        )

    @staticmethod
    def _distance_transform(free: np.ndarray) -> np.ndarray:
        try:
            import cv2

            image = (free.astype(np.uint8) * 255)
            return cv2.distanceTransform(image, cv2.DIST_L2, 5).astype(np.float32)
        except Exception:
            # Dependency-free Manhattan-distance fallback.
            h, w = free.shape
            inf = np.iinfo(np.int32).max // 4
            dist = np.full((h, w), inf, dtype=np.int32)
            queue: deque[tuple[int, int]] = deque()
            obstacle_rows, obstacle_cols = np.where(~free)
            for r, c in zip(obstacle_rows.tolist(), obstacle_cols.tolist()):
                dist[r, c] = 0
                queue.append((r, c))
            while queue:
                r, c = queue.popleft()
                nd = dist[r, c] + 1
                for dr, dc in CenterlineNavigator.DIRS:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        queue.append((nr, nc))
            return dist.astype(np.float32)

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        assert self.map_msg is not None
        ox = self.map_msg.info.origin.position.x
        oy = self.map_msg.info.origin.position.y
        c = int(math.floor((x - ox) / self.coarse_resolution))
        r = int(math.floor((y - oy) / self.coarse_resolution))
        return r, c

    def _cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        assert self.map_msg is not None
        r, c = cell
        ox = self.map_msg.info.origin.position.x
        oy = self.map_msg.info.origin.position.y
        return (
            ox + (c + 0.5) * self.coarse_resolution,
            oy + (r + 0.5) * self.coarse_resolution,
        )

    def _current_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _snap_cell(self, requested: tuple[int, int], safe_radius: float) -> Optional[tuple[int, int]]:
        if self.coarse_free is None or self.clearance_m is None:
            return None
        h, w = self.coarse_free.shape
        rr, cc = requested
        max_cells = max(1, int(math.ceil(self.snap_radius / self.coarse_resolution)))
        best = None
        best_score = float("inf")
        for r in range(max(0, rr - max_cells), min(h, rr + max_cells + 1)):
            for c in range(max(0, cc - max_cells), min(w, cc + max_cells + 1)):
                clearance = float(self.clearance_m[r, c])
                if not self.coarse_free[r, c] or clearance < safe_radius:
                    continue
                d = math.hypot(r - rr, c - cc)
                if d > max_cells:
                    continue
                # Prefer a nearby point, then the local maximum-clearance center.
                score = d + 1.2 / max(clearance, 0.05)
                if score < best_score:
                    best_score = score
                    best = (r, c)
        return best

    def _astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        safe_radius: float,
    ) -> Optional[list[tuple[int, int]]]:
        assert self.coarse_free is not None and self.clearance_m is not None
        h, w = self.coarse_free.shape
        # State includes incoming direction, allowing an explicit turn penalty.
        start_state = (start[0], start[1], 4)
        g_score = {start_state: 0.0}
        parent: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        heap = [(abs(start[0] - goal[0]) + abs(start[1] - goal[1]), 0.0, start_state)]
        visited: set[tuple[int, int, int]] = set()
        final_state = None

        while heap:
            _f, current_g, state = heapq.heappop(heap)
            if state in visited:
                continue
            visited.add(state)
            r, c, incoming = state
            if (r, c) == goal:
                final_state = state
                break
            for direction, (dr, dc) in enumerate(self.DIRS):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                clearance = float(self.clearance_m[nr, nc])
                if not self.coarse_free[nr, nc] or clearance < safe_radius:
                    continue
                effective_clearance = min(clearance, 1.50)
                center_cost = self.center_weight / max(effective_clearance * effective_clearance, 0.04)
                turn_cost = self.turn_penalty if incoming != 4 and incoming != direction else 0.0
                ng = current_g + 1.0 + center_cost + turn_cost
                nxt = (nr, nc, direction)
                if ng >= g_score.get(nxt, float("inf")):
                    continue
                g_score[nxt] = ng
                parent[nxt] = state
                heuristic = abs(nr - goal[0]) + abs(nc - goal[1])
                heapq.heappush(heap, (ng + heuristic, ng, nxt))

        if final_state is None:
            return None
        states = [final_state]
        while states[-1] != start_state:
            states.append(parent[states[-1]])
        states.reverse()
        return [(r, c) for r, c, _direction in states]

    @staticmethod
    def _compress_cells(cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        result = [cells[0]]
        previous_direction = None
        for i in range(1, len(cells)):
            direction = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
            if previous_direction is not None and direction != previous_direction:
                result.append(cells[i - 1])
            previous_direction = direction
        result.append(cells[-1])
        # Remove zero-length duplicates.
        clean = [result[0]]
        for cell in result[1:]:
            if cell != clean[-1]:
                clean.append(cell)
        return clean

    def _build_segments(
        self,
        current_xy: tuple[float, float],
        corners: list[tuple[int, int]],
    ) -> list[list[tuple[float, float]]]:
        world_corners = [self._cell_to_world(cell) for cell in corners]
        if world_corners:
            world_corners[0] = current_xy
        segments: list[list[tuple[float, float]]] = []
        for a, b in zip(world_corners, world_corners[1:]):
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            if distance < 0.04:
                continue
            count = max(2, int(math.ceil(distance / self.spacing)) + 1)
            points = [
                (a[0] + (b[0] - a[0]) * i / (count - 1), a[1] + (b[1] - a[1]) * i / (count - 1))
                for i in range(count)
            ]
            segments.append(points)
        return segments

    def _publish_full_path(self, segments: list[list[tuple[float, float]]], final_yaw: float) -> None:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        flat: list[tuple[float, float]] = []
        for segment in segments:
            if flat and segment and segment[0] == flat[-1]:
                flat.extend(segment[1:])
            else:
                flat.extend(segment)
        for i, (x, y) in enumerate(flat):
            if i + 1 < len(flat):
                nx, ny = flat[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = final_yaw
            path.poses.append(make_pose(self.global_frame, path.header.stamp, x, y, yaw))
        self.path_pub.publish(path)

    def _reset_motion_state(self) -> None:
        self.segments = []
        self.segment_index = 0
        self.next_segment_time = float("inf")
        self.retry_plan_time = float("inf")
        self.rotate_active = False
        self.rotate_mode = ""
        self.rotate_stable_count = 0
        self.segment_rotation_done = False
        self.cmd_pub.publish(Twist())

    def _on_traffic_pause(self, msg: Bool) -> None:
        pause = bool(msg.data)
        if pause == self.traffic_paused:
            return

        if pause:
            self.traffic_paused = True
            self.resume_replan_pending = False
            self.retry_plan_time = float("inf")
            self.rotate_active = False
            self.rotate_mode = ""
            self.segment_rotation_done = False
            # Resume is always a fresh replan from the actual current pose.  Keep only
            # final_goal_msg; old straight segments are intentionally discarded.
            self.segments = []
            self.segment_index = 0
            self.next_segment_time = float("inf")
            if self.active_goal_handle is not None:
                self.pause_cancel_inflight = True
                self.active_goal_handle.cancel_goal_async()
            self.cmd_pub.publish(Twist())
            self._publish_status("PAUSED:TRAFFIC")
            self.get_logger().warning("교차 충돌 회피: TRAFFIC PAUSE -> 현재 Nav2 구간 취소 후 정지")
            return

        self.traffic_paused = False
        self.resume_replan_pending = self.final_goal_msg is not None
        self.cmd_pub.publish(Twist())
        self._publish_status("ACTIVE:TRAFFIC_RESUME_WAIT")
        self.get_logger().info("교차 충돌 회피: TRAFFIC RESUME -> 현재 위치에서 기존 최종 목표 재계산")

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self.pose_locked:
            self.get_logger().error("자동 초기 Pose가 아직 잠기지 않았습니다.")
            self._publish_status("FAILED:POSE_NOT_LOCKED")
            self._stop_for(self.final_stop_sec)
            return
        if self.map_msg is None or self.coarse_free is None or self.clearance_m is None:
            self.get_logger().error("아직 /map을 받지 못했습니다.")
            self._publish_status("FAILED:MAP_NOT_READY")
            return

        self.final_goal_msg = copy.deepcopy(msg)
        self.retry_count = 0
        if self.traffic_paused:
            self.pending_goal = None
            self.resume_replan_pending = True
            self._publish_status("PAUSED:TRAFFIC:GOAL_STORED")
            self.get_logger().info("TRAFFIC PAUSE 중 새 목표 저장: 해제 후 현재 위치에서 실행합니다.")
            return
        if self.active_goal_handle is not None or self.busy:
            self.pending_goal = copy.deepcopy(msg)
            if self.active_goal_handle is not None:
                self.get_logger().warning("새 목표를 받았습니다. 현재 경로를 취소합니다.")
                self.active_goal_handle.cancel_goal_async()
            return
        if self.rotate_active or self.segments or self.retry_plan_time != float("inf"):
            self.get_logger().warning("새 목표로 기존 회전/재시도 상태를 교체합니다.")
            self._reset_motion_state()
        self._plan_goal(msg, is_retry=False)

    def _plan_goal(self, msg: PoseStamped, is_retry: bool = False) -> None:
        self.busy = True
        self._publish_status("ACTIVE:PLANNING")
        try:
            current_x, current_y, _current_yaw = self._current_pose()
        except TransformException as exc:
            self.busy = False
            self.get_logger().error(f"현재 map->base_link 위치를 찾지 못했습니다: {exc}")
            self._publish_status("FAILED:TF_UNAVAILABLE")
            return

        q = msg.pose.orientation
        q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        # A zero quaternion is an internal mission sentinel meaning: preserve the
        # heading of the final straight segment and do not rotate again at arrival.
        # Normal RViz/Nav2 goals always carry a unit quaternion and are unchanged.
        self.keep_arrival_heading = q_norm < 1.0e-6
        self.final_yaw = _current_yaw if self.keep_arrival_heading else yaw_from_quaternion(q.x, q.y, q.z, q.w)
        requested_start = self._world_to_cell(current_x, current_y)
        requested_goal = self._world_to_cell(msg.pose.position.x, msg.pose.position.y)

        route = None
        used_radius = self.safe_radius
        start = goal = None
        # Narrow doors are handled by controlled relaxation, while Nav2 local costmap
        # remains responsible for live collision avoidance.
        for radius in (self.safe_radius, max(0.45, self.safe_radius - 0.07), 0.38):
            start = self._snap_cell(requested_start, radius)
            goal = self._snap_cell(requested_goal, radius)
            if start is None or goal is None:
                continue
            route = self._astar(start, goal, radius)
            if route is not None:
                used_radius = radius
                break

        if route is None or start is None or goal is None:
            self.busy = False
            self._stop_for(self.final_stop_sec)
            self.get_logger().error("안전한 중앙 경로를 찾지 못했습니다.")
            self._publish_status("FAILED:NO_SAFE_PATH")
            return

        # 요청 좌표를 그대로 따라가지 않고, 항상 가장 여유가 큰 복도 중앙 셀로
        # 보정한 뒤 4방향 중앙 경로만 사용한다. 대각선 지름길도 만들지 않는다.
        corners = self._compress_cells(route)
        self.segments = self._build_segments((current_x, current_y), corners)
        if not self.segments:
            self.busy = False
            if self.keep_arrival_heading:
                self.get_logger().info("목적지 위치에 있습니다. 도착 heading 유지 요청이라 추가 회전 없이 완료합니다.")
                self._finish_success()
            else:
                self.get_logger().info("목적지 위치에 있습니다. 최종 yaw만 별도 회전으로 맞춥니다.")
                self._begin_rotation(self.final_yaw, "final")
            return
        self.segment_index = 0
        self.segment_rotation_done = False
        self._publish_full_path(self.segments, self.final_yaw)
        snapped_x, snapped_y = self._cell_to_world(goal)
        self._publish_status(f"ACTIVE:PATH_READY:{len(self.segments)}")
        self.get_logger().info(
            f"강제 중앙 경로 생성: 요청=({msg.pose.position.x:.2f},{msg.pose.position.y:.2f}) "
            f"보정=({snapped_x:.2f},{snapped_y:.2f}), 직선구간={len(self.segments)}, "
            f"벽 여유={used_radius:.2f}m"
        )
        self.busy = False
        self.next_segment_time = time.monotonic()

    def _segment_path(self, points: list[tuple[float, float]], is_final: bool) -> Path:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if len(points) >= 2:
            segment_yaw = math.atan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
        else:
            segment_yaw = self.final_yaw
        for x, y in points:
            # Final yaw is handled by the node's own rotation controller after the
            # last straight segment. FollowPath therefore only translates along
            # a pre-aligned straight segment and never waits on rotation progress.
            path.poses.append(make_pose(self.global_frame, path.header.stamp, x, y, segment_yaw))
        return path

    def _segment_yaw(self, points: list[tuple[float, float]]) -> float:
        if len(points) >= 2:
            return math.atan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
        return self.final_yaw

    def _begin_rotation(self, target_yaw: float, mode: str) -> None:
        self.rotate_target_yaw = normalize_angle(target_yaw)
        self.rotate_mode = mode
        self.rotate_stable_count = 0
        self.rotate_active = True
        self.next_segment_time = float("inf")
        self._publish_status(
            f"ACTIVE:ROTATING_{mode.upper()}:{self.segment_index + 1}_OF_{max(1, len(self.segments))}"
        )
        self.get_logger().info(
            f"회전 시작({mode}): 목표 yaw={self.rotate_target_yaw:.4f}rad, "
            "위치 progress 조건 없이 cmd_vel을 계속 재전송합니다."
        )

    def _rotation_tick(self) -> None:
        try:
            _x, _y, current_yaw = self._current_pose()
        except TransformException:
            self.cmd_pub.publish(Twist())
            return
        error = normalize_angle(self.rotate_target_yaw - current_yaw)
        if abs(error) <= self.rotate_tolerance:
            self.rotate_stable_count += 1
            self.cmd_pub.publish(Twist())
            if self.rotate_stable_count < self.rotate_stable_cycles:
                return
            mode = self.rotate_mode
            self.rotate_active = False
            self.rotate_mode = ""
            self.rotate_stable_count = 0
            self.get_logger().info(
                f"회전 완료({mode}): yaw 오차={math.degrees(error):.2f}도"
            )
            if mode == "final":
                self._finish_success()
            else:
                self.segment_rotation_done = True
                self.next_segment_time = time.monotonic() + 0.15
            return

        self.rotate_stable_count = 0
        speed = min(self.rotate_max_speed, max(self.rotate_min_speed, abs(error) * self.rotate_kp))
        cmd = Twist()
        cmd.angular.z = math.copysign(speed, error)
        self.cmd_pub.publish(cmd)

    def _send_next_segment(self) -> None:
        if self.segment_index >= len(self.segments):
            if self.keep_arrival_heading:
                if self.is_2f_map and self.segments:
                    # 2F MRI/엘리베이터: do not keep a residual DWB yaw error.
                    # Re-lock the robot to the exact heading of the last axis-aligned
                    # centerline segment, then finish. This keeps the following
                    # forced reverse on the same X/Y axis instead of drifting ~45 deg.
                    final_segment_yaw = self._segment_yaw(self.segments[-1])
                    self.get_logger().info(
                        f"2F 마지막 직선 heading 정밀 고정: yaw={final_segment_yaw:.4f}rad"
                    )
                    self._begin_rotation(final_segment_yaw, "final")
                else:
                    self.get_logger().info("마지막 직선 heading 유지: 최종 추가 회전 없이 완료합니다.")
                    self._finish_success()
            else:
                self._begin_rotation(self.final_yaw, "final")
            return
        if self.rotate_before_segment and not self.segment_rotation_done:
            self._begin_rotation(self._segment_yaw(self.segments[self.segment_index]), "segment")
            return
        if not self.action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning("FollowPath action 서버 대기 중...", throttle_duration_sec=2.0)
            self.next_segment_time = time.monotonic() + 0.5
            return
        goal = FollowPath.Goal()
        goal.path = self._segment_path(
            self.segments[self.segment_index],
            self.segment_index == len(self.segments) - 1,
        )
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.next_segment_time = float("inf")
        self._publish_status(
            f"ACTIVE:SEGMENT_{self.segment_index + 1}_OF_{len(self.segments)}"
        )
        self.get_logger().info(
            f"직선 구간 {self.segment_index + 1}/{len(self.segments)} 이동 시작"
        )

    def _goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.get_logger().error("FollowPath가 경로를 거절했습니다. 같은 최종 목표를 다시 전송합니다.")
            self._schedule_retry("FOLLOW_PATH_REJECTED")
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        result = future.result()
        self.active_goal_handle = None
        status = result.status
        if self.pause_cancel_inflight:
            self.pause_cancel_inflight = False
            self.pending_goal = None
            self.segments = []
            self.segment_index = 0
            self.next_segment_time = float("inf")
            self.retry_plan_time = float("inf")
            self.cmd_pub.publish(Twist())
            if self.traffic_paused:
                self._publish_status("PAUSED:TRAFFIC")
            else:
                self.resume_replan_pending = self.final_goal_msg is not None
            return
        if self.traffic_paused:
            self.pending_goal = None
            self.resume_replan_pending = self.final_goal_msg is not None
            self.cmd_pub.publish(Twist())
            self._publish_status("PAUSED:TRAFFIC")
            return
        if self.pending_goal is not None:
            pending = self.pending_goal
            self.pending_goal = None
            self.segments = []
            self._stop_for(0.6)
            self._plan_goal(pending)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(result.result, "error_code", 0)
            error_msg = getattr(result.result, "error_msg", "")

            # Nav2 ProgressChecker can abort even when the AMR is already very close
            # to the end of the current straight segment.  In that case, do not
            # rebuild the whole centerline path: accept the segment as reached and
            # continue to the next forced turn/straight segment.
            # 1F keeps the 0.30 m near-success shortcut that fixed the long retry loop.
            # 2F is narrower and the proven baseline required reaching the real corner;
            # accepting 30 cm early can cut the corner and shift the next straight path
            # toward a wall, so the shortcut is disabled only on 2F.
            if (not self.is_2f_map) and 0 <= self.segment_index < len(self.segments) and self.segments[self.segment_index]:
                try:
                    current_x, current_y, _ = self._current_pose()
                    end_x, end_y = self.segments[self.segment_index][-1]
                    remaining = math.hypot(end_x - current_x, end_y - current_y)
                    if remaining <= self.near_segment_success_tolerance:
                        self.get_logger().warning(
                            f"FollowPath status={status}지만 segment 끝점까지 {remaining:.3f}m 남음 "
                            f"(<= {self.near_segment_success_tolerance:.2f}m). 도착으로 인정하고 계속 진행합니다."
                        )
                        self._stop_for(self.corner_stop_sec)
                        self.segment_index += 1
                        self.segment_rotation_done = False
                        if self.segment_index >= len(self.segments):
                            self.get_logger().info(
                                "마지막 직선 구간 근접 도착: 재계산 없이 최종 yaw 단계로 진행합니다."
                            )
                        else:
                            self.get_logger().info(
                                "직선 구간 근접 도착: 재계산 없이 다음 강제회전으로 진행합니다."
                            )
                        self.next_segment_time = time.monotonic()
                        return
                except TransformException as exc:
                    self.get_logger().warning(
                        f"실패 후 segment 근접거리 확인 TF 실패: {exc}. 기존 재시도 절차를 사용합니다."
                    )

            self.get_logger().error(
                f"직선 구간 이동 실패/취소: status={status}, error_code={error_code}, error_msg={error_msg}. "
                "현재 위치에서 중앙 경로를 다시 계산합니다."
            )
            self._schedule_retry(
                f"FOLLOW_PATH_STATUS_{status}:ERROR_{error_code}:{error_msg}"
            )
            return
        # Stop exactly once at a segment boundary.  _stop_for() already gates the
        # tick loop until the stop interval expires, so adding corner_stop_sec again
        # to next_segment_time caused every corner pause to be doubled.
        self._stop_for(self.corner_stop_sec)
        self.segment_index += 1
        self.segment_rotation_done = False
        if self.segment_index >= len(self.segments):
            self.get_logger().info("마지막 직선 구간 도착: 짧게 정지 후 최종 yaw를 별도 회전으로 맞춥니다.")
        else:
            self.get_logger().info("코너 도착: 짧게 정지 후 다음 방향으로 바로 회전합니다.")
        self.next_segment_time = time.monotonic()

    def _schedule_retry(self, reason: str) -> None:
        self._stop_for(self.retry_delay_sec)
        self.segments = []
        self.segment_index = 0
        self.segment_rotation_done = False
        self.rotate_active = False
        self.rotate_mode = ""
        self.retry_count += 1
        if self.max_follow_path_retries > 0 and self.retry_count > self.max_follow_path_retries:
            self.retry_plan_time = float("inf")
            self.get_logger().error(
                f"자동 재시도 한도 초과: {self.max_follow_path_retries}회, reason={reason}"
            )
            self._publish_status(f"FAILED:RETRY_LIMIT:{reason}")
            return
        if self.final_goal_msg is None:
            self.retry_plan_time = float("inf")
            self._publish_status(f"FAILED:NO_RETRY_GOAL:{reason}")
            return
        self.retry_plan_time = time.monotonic() + self.retry_delay_sec
        self._publish_status(f"ACTIVE:RETRY_{self.retry_count}:{reason}")
        self.get_logger().warning(
            f"중앙 경로 자동 재시도 {self.retry_count}회: {self.retry_delay_sec:.1f}초 후 "
            "현재 위치에서 동일 최종 목적지로 다시 명령합니다."
        )

    def _finish_success(self) -> None:
        self.segments = []
        self.segment_index = 0
        self.next_segment_time = float("inf")
        self.retry_plan_time = float("inf")
        self.rotate_active = False
        self.rotate_mode = ""
        self.segment_rotation_done = False
        self.keep_arrival_heading = False
        self._stop_for(self.final_stop_sec)
        self._publish_status("SUCCEEDED")
        self.get_logger().info("목적지 도착 완료: 추가 동작 없이 정지합니다.")

    def _stop_for(self, seconds: float) -> None:
        self.stop_until = max(self.stop_until, time.monotonic() + max(0.1, seconds))
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        now = time.monotonic()
        if self.traffic_paused:
            self.cmd_pub.publish(Twist())
            return
        if (
            self.resume_replan_pending
            and self.active_goal_handle is None
            and not self.busy
            and not self.pause_cancel_inflight
        ):
            self.resume_replan_pending = False
            if self.final_goal_msg is not None:
                goal = copy.deepcopy(self.final_goal_msg)
                self._reset_motion_state()
                self._plan_goal(goal, is_retry=True)
            return
        if now < self.stop_until:
            self.cmd_pub.publish(Twist())
            return
        if self.rotate_active:
            self._rotation_tick()
            return
        if (
            self.retry_plan_time != float("inf")
            and now >= self.retry_plan_time
            and self.active_goal_handle is None
            and not self.busy
        ):
            self.retry_plan_time = float("inf")
            if self.final_goal_msg is not None:
                self._plan_goal(copy.deepcopy(self.final_goal_msg), is_retry=True)
            return
        if (
            self.active_goal_handle is None
            and self.segments
            and now >= self.next_segment_time
        ):
            self._send_next_segment()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CenterlineNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
