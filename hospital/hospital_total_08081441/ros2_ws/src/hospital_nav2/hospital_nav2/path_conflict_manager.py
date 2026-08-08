#!/usr/bin/env python3
"""Two-AMR path conflict coordinator.

This node does not replace either robot's Nav2 stack.  It only watches the two
already-generated centerline paths and publishes a per-robot traffic pause Bool.
A tiny optional pause hook in centerline_navigator cancels the current FollowPath
segment, holds zero velocity, and replans the same final goal after release.

Rules:
* A robot doing non-Nav special motion (OCR/coupling/forced motion/elevator) has absolute priority.
* Otherwise compare paths only when both robots are on the same loaded floor.
* Future centerline points closer than overlap_distance_m form the next conflict zone.
* A robot already inside the conflict zone has priority.
* Otherwise the robot with the shorter path distance to the conflict entry wins.
  Near ties use AMR1 deterministically.
* The loser is paused only as it approaches the zone (hold_trigger_distance_m).
* After the winner passes the conflict zone, keep a release delay, then resume.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass
class RobotState:
    name: str
    path: list[tuple[float, float]] = field(default_factory=list)
    pose: Optional[tuple[float, float]] = None
    floor: str = "unknown"
    nav_active: bool = False
    path_stamp: float = 0.0
    status: str = ""


@dataclass
class ConflictSession:
    winner: str
    loser: str
    winner_start: int
    winner_end: int
    loser_start: int
    loser_end: int
    created: float
    loser_paused: bool = False
    release_started: Optional[float] = None


class PathConflictManager(Node):
    def __init__(self) -> None:
        super().__init__("path_conflict_manager")
        self.declare_parameter("overlap_distance_m", 1.00)
        self.declare_parameter("hold_trigger_distance_m", 4.00)
        self.declare_parameter("release_clearance_m", 1.20)
        self.declare_parameter("release_delay_sec", 2.00)
        self.declare_parameter("tie_distance_m", 0.35)
        self.declare_parameter("path_sample_step", 2)
        # centerline_path is intentionally latched for the whole active goal.
        # Do not expire a valid route only because the trip lasts >30 seconds.
        self.declare_parameter("path_stale_sec", 30.0)  # kept only for launch compatibility
        self.declare_parameter("special_stale_sec", 1.0)
        self.declare_parameter("head_on_enabled", True)
        self.declare_parameter("head_on_trigger_distance_m", 1.25)
        self.declare_parameter("head_on_dot_threshold", -0.70)
        self.declare_parameter("head_on_lateral_distance_m", 1.0)
        self.declare_parameter("head_on_forward_distance_m", 1.0)
        self.declare_parameter("head_on_timeout_sec", 25.0)

        self.overlap_distance = max(0.3, float(self.get_parameter("overlap_distance_m").value))
        self.hold_trigger_distance = max(0.5, float(self.get_parameter("hold_trigger_distance_m").value))
        self.release_clearance = max(0.2, float(self.get_parameter("release_clearance_m").value))
        self.release_delay = max(0.0, float(self.get_parameter("release_delay_sec").value))
        self.tie_distance = max(0.0, float(self.get_parameter("tie_distance_m").value))
        self.sample_step = max(1, int(self.get_parameter("path_sample_step").value))
        self.path_stale_sec = max(2.0, float(self.get_parameter("path_stale_sec").value))
        self.special_stale_sec = max(0.5, float(self.get_parameter("special_stale_sec").value))
        self.head_on_enabled = bool(self.get_parameter("head_on_enabled").value)
        self.head_on_trigger_distance = max(0.8, float(self.get_parameter("head_on_trigger_distance_m").value))
        self.head_on_dot_threshold = min(-0.10, float(self.get_parameter("head_on_dot_threshold").value))
        self.head_on_lateral_distance = max(0.1, float(self.get_parameter("head_on_lateral_distance_m").value))
        self.head_on_forward_distance = max(0.1, float(self.get_parameter("head_on_forward_distance_m").value))
        self.head_on_timeout = max(5.0, float(self.get_parameter("head_on_timeout_sec").value))

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.robots = {
            "amr1": RobotState("amr1"),
            "amr2": RobotState("amr2"),
        }
        self.pause_pub = {
            "amr1": self.create_publisher(Bool, "/traffic_pause", latched),
            "amr2": self.create_publisher(Bool, "/amr2/traffic_pause", latched),
        }
        self.special_pause_pub = {
            "amr1": self.create_publisher(Bool, "/amr1/special_motion_pause", latched),
            "amr2": self.create_publisher(Bool, "/amr2/special_motion_pause", latched),
        }
        # Direct, agreed passing maneuver. Nav2 is paused first; Isaac then executes
        # local-frame E 1m -> forward 1m -> Q 1m for BOTH robots.
        self.maneuver_pub = {
            "amr1": self.create_publisher(String, "/amr1/traffic_maneuver/command", latched),
            "amr2": self.create_publisher(String, "/amr2/traffic_maneuver/command", latched),
        }
        self.status_pub = self.create_publisher(String, "/traffic_conflict/status", latched)

        self.create_subscription(Path, "/centerline_path", lambda m: self._on_path("amr1", m), latched)
        self.create_subscription(Path, "/amr2/centerline_path", lambda m: self._on_path("amr2", m), latched)
        self.create_subscription(String, "/amr1/world_pose", lambda m: self._on_pose("amr1", m), 20)
        self.create_subscription(String, "/amr2/world_pose", lambda m: self._on_pose("amr2", m), 20)
        self.create_subscription(String, "/center_goal/status", lambda m: self._on_nav_status("amr1", m), latched)
        self.create_subscription(String, "/amr2/center_goal/status", lambda m: self._on_nav_status("amr2", m), latched)
        self.create_subscription(OccupancyGrid, "/map", lambda m: self._on_map("amr1", m), latched)
        self.create_subscription(OccupancyGrid, "/amr2/map", lambda m: self._on_map("amr2", m), latched)
        self.create_subscription(Bool, "/amr1/special_motion_active", lambda m: self._on_special("amr1", m), 10)
        self.create_subscription(Bool, "/amr2/special_motion_active", lambda m: self._on_special("amr2", m), 10)
        self.create_subscription(String, "/amr1/traffic_maneuver/status", lambda m: self._on_maneuver_status("amr1", m), 10)
        self.create_subscription(String, "/amr2/traffic_maneuver/status", lambda m: self._on_maneuver_status("amr2", m), 10)

        self.special_active = {"amr1": False, "amr2": False}
        self.special_stamp = {"amr1": 0.0, "amr2": 0.0}
        self.special_since = {"amr1": float("inf"), "amr2": float("inf")}
        self.special_owner: Optional[str] = None

        self.passing_request_id = ""
        self.passing_started = 0.0
        self.passing_command_sent = False
        self.passing_status = {"amr1": "", "amr2": ""}
        self.passing_cooldown_until = 0.0

        self.session: Optional[ConflictSession] = None
        self.last_pause = {"amr1": None, "amr2": None}
        self.last_special_pause = {"amr1": None, "amr2": None}
        self.last_status = ""
        self.create_timer(0.10, self._tick)
        self._set_pause("amr1", False)
        self._set_pause("amr2", False)
        self._set_special_pause("amr1", False)
        self._set_special_pause("amr2", False)
        self._publish_status("READY", "실제 AMR1/AMR2 centerline path 비교 대기")
        self.get_logger().info(
            f"경로 충돌 회피 준비: overlap<={self.overlap_distance:.2f}m, "
            f"hold_trigger={self.hold_trigger_distance:.1f}m, release_delay={self.release_delay:.1f}s, "
            f"special-priority=ON, head-on-pass={'ON' if self.head_on_enabled else 'OFF'}"
        )

    @staticmethod
    def _floor_from_map(msg: OccupancyGrid) -> str:
        w, h = int(msg.info.width), int(msg.info.height)
        if (w, h) == (1528, 841):
            return "1f"
        if (w, h) == (1512, 841):
            return "2f"
        return f"map:{w}x{h}"

    def _on_map(self, robot: str, msg: OccupancyGrid) -> None:
        floor = self._floor_from_map(msg)
        if self.robots[robot].floor != floor:
            self.robots[robot].floor = floor
            self.get_logger().info(f"{robot.upper()} floor={floor}")
            # A floor transition invalidates any old cross-floor conflict session.
            if self.session is not None:
                self._clear_session("FLOOR_CHANGED")

    def _on_path(self, robot: str, msg: Path) -> None:
        pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.robots[robot].path = pts
        self.robots[robot].path_stamp = time.monotonic()
        if len(pts) >= 2:
            self.robots[robot].nav_active = True

    def _on_pose(self, robot: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.robots[robot].pose = (float(data["x"]), float(data["y"]))
        except Exception:
            return

    def _on_special(self, robot: str, msg: Bool) -> None:
        now = time.monotonic()
        active = bool(msg.data)
        if active and not self.special_active[robot]:
            self.special_since[robot] = now
        if not active:
            self.special_since[robot] = float("inf")
        self.special_active[robot] = active
        self.special_stamp[robot] = now

    def _special_now(self, robot: str, now: float) -> bool:
        return bool(
            self.special_active[robot]
            and (now - self.special_stamp[robot]) <= self.special_stale_sec
        )

    def _on_maneuver_status(self, robot: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if str(payload.get("request_id", "")) != self.passing_request_id:
            return
        self.passing_status[robot] = str(payload.get("state", "")).upper()

    @staticmethod
    def _unit_heading(path: list[tuple[float, float]], index: int) -> Optional[tuple[float, float]]:
        if len(path) < 2:
            return None
        i0 = max(0, min(int(index), len(path) - 1))
        i1 = min(len(path) - 1, i0 + 6)
        if i1 == i0:
            i0 = max(0, i0 - 6)
        dx = float(path[i1][0] - path[i0][0])
        dy = float(path[i1][1] - path[i0][1])
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return dx / norm, dy / norm

    def _head_on_geometry(self, a: RobotState, b: RobotState) -> tuple[bool, float, float]:
        if not self.head_on_enabled or a.pose is None or b.pose is None:
            return False, float("inf"), 1.0
        ai = self._nearest_index(a.path, a.pose)
        bi = self._nearest_index(b.path, b.pose)
        ha = self._unit_heading(a.path, ai)
        hb = self._unit_heading(b.path, bi)
        if ha is None or hb is None:
            return False, float("inf"), 1.0
        ax, ay = a.pose
        bx, by = b.pose
        dx, dy = bx - ax, by - ay
        separation = math.hypot(dx, dy)
        if separation < 1e-6:
            return True, separation, -1.0
        ux, uy = dx / separation, dy / separation
        heading_dot = ha[0] * hb[0] + ha[1] * hb[1]
        # Both robots must actually face toward each other, not merely cross at an intersection.
        a_faces_b = ha[0] * ux + ha[1] * uy
        b_faces_a = -(hb[0] * ux + hb[1] * uy)
        head_on = bool(
            heading_dot <= self.head_on_dot_threshold
            and a_faces_b >= 0.55
            and b_faces_a >= 0.55
        )
        return head_on, separation, heading_dot

    def _publish_maneuver_command(self, robot: str, command: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "request_id": self.passing_request_id,
            "command": command,
            "robot": robot,
            "lateral_distance_m": self.head_on_lateral_distance,
            "forward_distance_m": self.head_on_forward_distance,
        }, ensure_ascii=False, separators=(",", ":"))
        self.maneuver_pub[robot].publish(msg)

    def _start_head_on_passing(self, now: float, separation: float, heading_dot: float) -> None:
        self.session = None
        self.passing_request_id = f"headon-{int(time.time() * 1000)}"
        self.passing_started = now
        self.passing_command_sent = False
        self.passing_status = {"amr1": "", "amr2": ""}
        # First cancel/pause BOTH Nav2 controllers. Direct motion starts only after
        # both centerline navigators report PAUSED:TRAFFIC.
        self._set_pause("amr1", True)
        self._set_pause("amr2", True)
        self._publish_status(
            "HEAD_ON_PAUSE",
            "head-on single-path encounter: pause both Nav2 before agreed right-side pass",
            request_id=self.passing_request_id,
            separation_m=round(separation, 2),
            heading_dot=round(heading_dot, 3),
        )

    def _handle_head_on_passing(self, now: float) -> bool:
        if not self.passing_request_id:
            return False
        self._set_pause("amr1", True)
        self._set_pause("amr2", True)

        elapsed = now - self.passing_started
        failed = [r for r, state in self.passing_status.items() if state.startswith("FAILED") or state.startswith("REJECTED")]
        if failed or elapsed > self.head_on_timeout:
            if self.passing_command_sent:
                self._publish_maneuver_command("amr1", "CANCEL")
                self._publish_maneuver_command("amr2", "CANCEL")
            # Fail-safe: never silently resume into the same face-to-face conflict.
            self._publish_status(
                "HEAD_ON_FAILED_HOLD",
                "passing maneuver failed/timed out; both AMRs remain traffic-paused for safety",
                request_id=self.passing_request_id,
                failed=failed,
                elapsed_s=round(elapsed, 1),
            )
            return True

        if not self.passing_command_sent:
            both_paused = all(self.robots[r].status.startswith("PAUSED:TRAFFIC") for r in ("amr1", "amr2"))
            if not both_paused:
                return True
            self._publish_maneuver_command("amr1", "START")
            self._publish_maneuver_command("amr2", "START")
            self.passing_command_sent = True
            self._publish_status(
                "HEAD_ON_PASSING",
                "both Nav2 paused -> both execute E 1m, forward 1m, Q 1m",
                request_id=self.passing_request_id,
            )
            return True

        if all(self.passing_status[r] == "COMPLETE" for r in ("amr1", "amr2")):
            request_id = self.passing_request_id
            self.passing_request_id = ""
            self.passing_command_sent = False
            self.passing_status = {"amr1": "", "amr2": ""}
            self.passing_cooldown_until = now + 2.0
            # Releasing traffic_pause makes each centerline navigator replan the
            # SAME final goal from its new lane position.
            self._set_pause("amr1", False)
            self._set_pause("amr2", False)
            self._publish_status(
                "HEAD_ON_COMPLETE",
                "agreed pass complete; both AMRs replan their original final goals",
                request_id=request_id,
            )
            return True
        return True

    def _on_nav_status(self, robot: str, msg: String) -> None:
        status = str(msg.data)
        state = self.robots[robot]
        state.status = status
        if status.startswith("ACTIVE") or status.startswith("PAUSED"):
            state.nav_active = True
        elif status.startswith("SUCCEEDED") or status.startswith("FAILED") or status.startswith("READY"):
            state.nav_active = False
            if status.startswith("SUCCEEDED") or status.startswith("FAILED"):
                state.path = []

    def _set_pause(self, robot: str, pause: bool) -> None:
        pause = bool(pause)
        if self.last_pause[robot] == pause:
            return
        msg = Bool()
        msg.data = pause
        self.pause_pub[robot].publish(msg)
        self.last_pause[robot] = pause
        self.get_logger().warning(f"{robot.upper()} TRAFFIC_PAUSE={pause}") if pause else self.get_logger().info(
            f"{robot.upper()} TRAFFIC_PAUSE={pause}"
        )

    def _set_special_pause(self, robot: str, pause: bool) -> None:
        pause = bool(pause)
        if self.last_special_pause[robot] == pause:
            return
        msg = Bool()
        msg.data = pause
        self.special_pause_pub[robot].publish(msg)
        self.last_special_pause[robot] = pause
        if pause:
            self.get_logger().warning(f"{robot.upper()} SPECIAL_MOTION_PAUSE=True")
        else:
            self.get_logger().info(f"{robot.upper()} SPECIAL_MOTION_PAUSE=False")

    def _publish_status(self, state: str, detail: str, **extra) -> None:
        payload = {"state": state, "detail": detail, **extra}
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def _nearest_index(path: list[tuple[float, float]], pose: Optional[tuple[float, float]]) -> int:
        if not path or pose is None:
            return 0
        px, py = pose
        best_i = 0
        best_d2 = float("inf")
        for i, (x, y) in enumerate(path):
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    @staticmethod
    def _distance_along(path: list[tuple[float, float]], start: int, end: int) -> float:
        if not path or end <= start:
            return 0.0
        end = min(end, len(path) - 1)
        start = max(0, start)
        total = 0.0
        for i in range(start + 1, end + 1):
            total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        return total

    def _find_conflict(self, a: RobotState, b: RobotState) -> Optional[tuple[int, int, int, int]]:
        if len(a.path) < 2 or len(b.path) < 2:
            return None
        ai0 = self._nearest_index(a.path, a.pose)
        bi0 = self._nearest_index(b.path, b.pose)
        threshold = self.overlap_distance
        cell = threshold

        # Hash remaining B path points so the comparison stays cheap even for long paths.
        buckets: dict[tuple[int, int], list[tuple[int, float, float]]] = {}
        for j in range(bi0, len(b.path), self.sample_step):
            x, y = b.path[j]
            key = (math.floor(x / cell), math.floor(y / cell))
            buckets.setdefault(key, []).append((j, x, y))

        pairs: list[tuple[int, int]] = []
        threshold2 = threshold * threshold
        for i in range(ai0, len(a.path), self.sample_step):
            x, y = a.path[i]
            gx, gy = math.floor(x / cell), math.floor(y / cell)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j, bx, by in buckets.get((gx + dx, gy + dy), ()): 
                        if (x - bx) ** 2 + (y - by) ** 2 <= threshold2:
                            pairs.append((i, j))
        if not pairs:
            return None

        # Do not merge disconnected future intersections into one huge hold zone.
        # Select only the nearest contiguous conflict island on A's remaining path.
        a_indices = sorted({i for i, _ in pairs})
        max_gap = max(2, self.sample_step * 3)
        groups: list[list[int]] = []
        current = [a_indices[0]]
        for idx in a_indices[1:]:
            if idx - current[-1] <= max_gap:
                current.append(idx)
            else:
                groups.append(current)
                current = [idx]
        groups.append(current)
        group = min(groups, key=lambda g: self._distance_along(a.path, ai0, g[0]))
        a_start, a_end = group[0], group[-1]
        island_pairs = [(i, j) for i, j in pairs if a_start <= i <= a_end]
        return (
            a_start,
            a_end,
            min(j for _, j in island_pairs),
            max(j for _, j in island_pairs),
        )

    def _current_progress(self, robot: str) -> int:
        state = self.robots[robot]
        return self._nearest_index(state.path, state.pose)

    def _select_priority(
        self,
        a: RobotState,
        b: RobotState,
        a_start: int,
        a_end: int,
        b_start: int,
        b_end: int,
    ) -> tuple[str, str, float, float]:
        ai = self._nearest_index(a.path, a.pose)
        bi = self._nearest_index(b.path, b.pose)
        a_inside = a_start <= ai <= a_end
        b_inside = b_start <= bi <= b_end
        da = self._distance_along(a.path, ai, a_start) if ai < a_start else 0.0
        db = self._distance_along(b.path, bi, b_start) if bi < b_start else 0.0

        if a_inside and not b_inside:
            return "amr1", "amr2", da, db
        if b_inside and not a_inside:
            return "amr2", "amr1", da, db
        if a_inside and b_inside:
            # Late detection fallback: let the robot closer to leaving finish first.
            a_exit = self._distance_along(a.path, ai, a_end)
            b_exit = self._distance_along(b.path, bi, b_end)
            if b_exit + self.tie_distance < a_exit:
                return "amr2", "amr1", da, db
            return "amr1", "amr2", da, db
        if db + self.tie_distance < da:
            return "amr2", "amr1", da, db
        return "amr1", "amr2", da, db

    def _loser_distance_to_entry(self, session: ConflictSession) -> float:
        state = self.robots[session.loser]
        idx = self._nearest_index(state.path, state.pose)
        entry = session.loser_start
        if idx >= entry:
            return 0.0
        return self._distance_along(state.path, idx, entry)

    def _winner_cleared(self, session: ConflictSession) -> bool:
        state = self.robots[session.winner]
        if not state.nav_active and not state.path:
            return True
        if not state.path:
            return True
        idx = self._nearest_index(state.path, state.pose)
        if idx <= session.winner_end:
            return False
        # Require extra path progress after the last shared point.
        after = self._distance_along(state.path, session.winner_end, idx)
        return after >= self.release_clearance

    def _clear_session(self, reason: str) -> None:
        self._set_pause("amr1", False)
        self._set_pause("amr2", False)
        self.session = None
        self._publish_status("FREE", reason)

    def _tick(self) -> None:
        now = time.monotonic()
        a = self.robots["amr1"]
        b = self.robots["amr2"]

        # Absolute priority for non-Nav special motion.  The other AMR pauses its
        # centerline Nav2 and automatically replans the same final goal on release.
        special = [r for r in ("amr1", "amr2") if self._special_now(r, now)]
        if special:
            if self.special_owner not in special:
                self.special_owner = min(special, key=lambda r: self.special_since[r])
            winner = self.special_owner
            loser = "amr2" if winner == "amr1" else "amr1"
            self.session = None
            self._set_pause(winner, False)
            self._set_pause(loser, True)
            # Nav pause alone cannot stop direct OCR/ArUco/forced/elevator motion.
            # Keep the winner's special motion free and hold the other AMR's special
            # state machine before it starts (or at its next safe direct-motion loop).
            self._set_special_pause(winner, False)
            self._set_special_pause(loser, True)
            self._publish_status(
                "SPECIAL_PRIORITY",
                f"{winner.upper()} non-Nav special motion has absolute priority; {loser.upper()} Nav/special motion waits",
                winner=winner,
                loser=loser,
            )
            return
        if self.special_owner is not None:
            previous = self.special_owner
            self.special_owner = None
            self.session = None
            self._set_pause("amr1", False)
            self._set_pause("amr2", False)
            self._set_special_pause("amr1", False)
            self._set_special_pause("amr2", False)
            self._publish_status("FREE", f"{previous.upper()} special motion finished; paused Nav/special motion resumed")
            return

        if self._handle_head_on_passing(now):
            return

        # Never compare coordinates from different floor maps.
        if a.floor == "unknown" or b.floor == "unknown" or a.floor != b.floor:
            if self.session is not None or self.last_pause["amr1"] or self.last_pause["amr2"]:
                self._clear_session(f"DIFFERENT_FLOORS:{a.floor}/{b.floor}")
            return

        if not a.nav_active or not b.nav_active:
            if self.session is not None:
                # If the winner completed its route, this will release below.
                if self._winner_cleared(self.session):
                    if self.session.release_started is None:
                        self.session.release_started = now
                    elif now - self.session.release_started >= self.release_delay:
                        self._clear_session("WINNER_FINISHED")
                return
            return

        if self.session is None:
            conflict = self._find_conflict(a, b)
            if conflict is None:
                self._publish_status("FREE", f"same floor {a.floor}, future paths do not overlap")
                return
            a_start, a_end, b_start, b_end = conflict
            head_on, separation, heading_dot = self._head_on_geometry(a, b)
            if head_on:
                if now < self.passing_cooldown_until:
                    return
                if separation <= self.head_on_trigger_distance:
                    self._start_head_on_passing(now, separation, heading_dot)
                    return
                # It is the same corridor but they are still far apart. Keep normal
                # Nav2 running until the agreed pass trigger distance is reached.
                self._publish_status(
                    "HEAD_ON_APPROACHING",
                    "opposite-direction same-path encounter detected; waiting for pass trigger distance",
                    separation_m=round(separation, 2),
                    trigger_m=round(self.head_on_trigger_distance, 2),
                    heading_dot=round(heading_dot, 3),
                )
                return
            winner, loser, da, db = self._select_priority(a, b, a_start, a_end, b_start, b_end)
            self.session = ConflictSession(
                winner=winner,
                loser=loser,
                winner_start=a_start if winner == "amr1" else b_start,
                winner_end=a_end if winner == "amr1" else b_end,
                loser_start=b_start if loser == "amr2" else a_start,
                loser_end=b_end if loser == "amr2" else a_end,
                created=now,
            )
            self._publish_status(
                "CONFLICT_DETECTED",
                f"{winner.upper()} priority / {loser.upper()} yields before shared path",
                floor=a.floor,
                amr1_distance_to_conflict_m=round(da, 2),
                amr2_distance_to_conflict_m=round(db, 2),
                winner=winner,
                loser=loser,
            )
            self.get_logger().warning(
                f"충돌 경로 감지({a.floor}): winner={winner.upper()}, loser={loser.upper()}, "
                f"AMR1 entry≈{da:.2f}m, AMR2 entry≈{db:.2f}m"
            )

        session = self.session
        if session is None:
            return

        loser_distance = self._loser_distance_to_entry(session)
        if not session.loser_paused and loser_distance <= self.hold_trigger_distance:
            self._set_pause(session.loser, True)
            self._set_pause(session.winner, False)
            session.loser_paused = True
            self._publish_status(
                "YIELDING",
                f"{session.loser.upper()} paused before conflict; {session.winner.upper()} passes",
                floor=a.floor,
                winner=session.winner,
                loser=session.loser,
                loser_distance_to_conflict_m=round(loser_distance, 2),
            )

        if self._winner_cleared(session):
            if session.release_started is None:
                session.release_started = now
                self._publish_status(
                    "CLEARANCE_DELAY",
                    f"{session.winner.upper()} cleared; wait {self.release_delay:.1f}s before release",
                    winner=session.winner,
                    loser=session.loser,
                )
            elif now - session.release_started >= self.release_delay:
                self._set_pause(session.loser, False)
                self._set_pause(session.winner, False)
                self.get_logger().info(
                    f"충돌 구간 해제: {session.winner.upper()} 통과 완료 -> {session.loser.upper()} 재출발"
                )
                self.session = None
                self._publish_status("FREE", "conflict cleared; yielded AMR resumed")
        else:
            session.release_started = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathConflictManager()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._set_pause("amr1", False)
            node._set_pause("amr2", False)
            node._set_special_pause("amr1", False)
            node._set_special_pause("amr2", False)
            if node.passing_request_id:
                node._publish_maneuver_command("amr1", "CANCEL")
                node._publish_maneuver_command("amr2", "CANCEL")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
