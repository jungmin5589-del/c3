#!/usr/bin/env python3
"""3-patient hospital bed transport mission manager.

Patient-specific OCR pose -> OCR identity verification -> paired ArUco bed-centre alignment
-> fixed patient-specific approach -> magnetic bed lock -> half-distance room reverse
-> elevator service UP -> MRI -> 3 s wait -> 11 m straight reverse -> K/service gate
-> 11 m straight forward back to MRI -> 3 s wait -> fixed 2F elevator goal
-> elevator service DOWN -> 1F elevator goal -> original bed position -> release
-> same half-distance reverse -> initial docking pose.

OCR is used only to verify patient identity.  Docking position control deliberately ignores
the OCR bounding box and uses the centre of the patient's left/right ArUco marker pair.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
import select
import termios
import tty
from pathlib import Path
import shlex
import sys
import time
import uuid


def _restart_with_ros_environment() -> None:
    if os.environ.get("KIMSEOUL_MISSION_ROS_READY") == "1":
        return
    root = Path(__file__).resolve().parent
    ros_setup = Path("/opt/ros/humble/setup.bash")
    ws_setup = root / "ros2_ws" / "install" / "setup.bash"
    if not ros_setup.is_file():
        raise SystemExit("[오류] ROS 2 Humble setup.bash가 없습니다.")
    if not ws_setup.is_file():
        raise SystemExit("[오류] 먼저 ./02_build_ros_ws.sh를 실행하세요.")
    command = " ".join([
        f"source {shlex.quote(str(ros_setup))}",
        f"&& source {shlex.quote(str(ws_setup))}",
        "&& export ROS_DOMAIN_ID=117",
        "&& export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}",
        "&& export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}",
        "&& export KIMSEOUL_MISSION_ROS_READY=1",
        "&& if [ -f \"$HOME/.ros/fastdds_whitelist.xml\" ]; then export FASTRTPS_DEFAULT_PROFILES_FILE=\"$HOME/.ros/fastdds_whitelist.xml\"; fi",
        "&& exec python3",
        shlex.quote(str(Path(__file__).resolve())),
        *(shlex.quote(arg) for arg in sys.argv[1:]),
    ])
    os.execv("/bin/bash", ["bash", "-lc", command])


_restart_with_ros_environment()

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.srv import LoadMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


PATIENTS = {
    "1": {
        "name": "김서울",
        "birth_date": "2000-11-02",
        "ocr_pose": {"x": -43.6377, "y": 11.4166, "yaw": -1.565538},
        # Reference ArUco project's measured overshoot correction (3.328 -> 2.87 m).
        "approach_distance_m": 2.87,
    },
    "2": {
        "name": "박인천",
        "birth_date": "1960-07-23",
        "ocr_pose": {"x": -43.4339, "y": 11.2432, "yaw": 3.111352},
        "approach_distance_m": 3.5221,
    },
    "3": {
        "name": "서수원",
        "birth_date": "1990-02-10",
        "ocr_pose": {"x": -43.8482, "y": 15.1898, "yaw": 3.132669},
        "approach_distance_m": 3.1554,
    },
}

AMR1_INITIAL_POSE = {"x": -45.0467, "y": 31.8558, "yaw": -1.566514}
AMR2_INITIAL_POSE = {"x": -47.2788, "y": 26.5713, "yaw": 0.0}
AMR_INITIAL_POSES = {"amr1": AMR1_INITIAL_POSE, "amr2": AMR2_INITIAL_POSE}
ELEVATOR_1F = {"x": -26.208667755126953, "y": 21.487224578857422, "keep_arrival_heading": True}
ELEVATOR_2F_XY = {"x": ELEVATOR_1F["x"], "y": ELEVATOR_1F["y"]}
MRI_2F = {"x": 6.28808, "y": 6.5918, "keep_arrival_heading": True}
MRI_WAIT_S = 3.0
MRI_REVERSE_DISTANCE_M = 11.0

# Paired-ArUco docking values copied from the proven reference project.  Keep these
# conservative even though other integrated-project motion speeds are doubled.
ARUCO_PAIRS = {"김서울": (10, 11), "박인천": (20, 21), "서수원": (30, 31)}
ARUCO_X_TOLERANCE_PX = 12.0
ARUCO_STABLE_MESSAGES = 2
ARUCO_YAW_THRESHOLD_PX = 70.0
ARUCO_LATERAL_KP = 1.6
ARUCO_MAX_LATERAL_MPS = 0.10
ARUCO_YAW_KP = 0.9
ARUCO_MAX_YAW_RAD_S = 0.18
ARUCO_MIN_YAW_RAD_S = 0.05
ARUCO_RESULT_STALE_SEC = 0.75
ARUCO_APPROACH_SPEED_MPS = 0.32

# 전체 로봇 속도 2배 적용에 맞춘 강제 직선/엘리베이터 구간 속도.
FORCED_STRAIGHT_SPEED_MPS = 0.44
ELEVATOR_RETURN_REVERSE_DISTANCE_M = 10.0
ELEVATOR_RETURN_REVERSE_SPEED_MPS = 0.25
ELEVATOR_RETURN_EXIT_DISTANCE_M = 5.0
ELEVATOR_RETURN_EXIT_SPEED_MPS = 0.25

INSPECTION_COMPLETE_SERVICE = "/inspection/complete"
ELEVATOR_UP_SERVICE = "/amr1/elevator/request_up"
ELEVATOR_DOWN_SERVICE = "/amr1/elevator/request_down"

CENTER_GOAL_TOPIC = "/center_goal"
CENTER_STATUS_TOPIC = "/center_goal/status"
INITIAL_LOCK_TOPIC = "/initial_pose_locked"
AUTO_COMMAND_TOPIC = "/amr1/auto_approach/command"
ALIGN_STATUS_TOPIC = "/amr1/align/status"
OCR_REQUEST_TOPIC = "/amr1/ocr/request"
MAGNET_COMMAND_TOPIC = "/amr1/magnet/command"
MAGNET_STATUS_TOPIC = "/amr1/magnet/status"
CMD_VEL_TOPIC = "/cmd_vel"
ELEVATOR_ARRIVAL_TOPIC = "/elevator/amr_arrived"
ELEVATOR_STATUS_TOPIC = "/elevator/status"
ELEVATOR_MAP_ACK_TOPIC = "/elevator/map_switch_ack"
ELEVATOR_FINAL_TOPIC = "/amr1/elevator/arrived"
MAP_LOAD_SERVICE = "/map_server/load_map"


class KimSeoulMission(Node):
    def __init__(self, patient: dict, amr_id: str = "amr1") -> None:
        amr_id = str(amr_id).strip().lower()
        if amr_id not in {"amr1", "amr2"}:
            raise ValueError(f"unsupported AMR id: {amr_id}")
        node_cli_args = None
        if amr_id == "amr2":
            node_cli_args = [
                "--ros-args",
                "-r", "/tf:=/amr2/tf",
                "-r", "/tf_static:=/amr2/tf_static",
            ]
        super().__init__(f"{amr_id}_patient_transport_manager", cli_args=node_cli_args)
        self.amr_id = amr_id
        self.patient = dict(patient)
        self.initial_pose = dict(AMR_INITIAL_POSES[amr_id])
        self.odom_frame = "odom" if amr_id == "amr1" else "amr2/odom"
        self.base_frame = "base_link" if amr_id == "amr1" else "amr2/base_link"
        self.center_goal_topic = "/center_goal" if amr_id == "amr1" else "/amr2/center_goal"
        self.center_status_topic = "/center_goal/status" if amr_id == "amr1" else "/amr2/center_goal/status"
        self.initial_lock_topic = "/initial_pose_locked" if amr_id == "amr1" else "/amr2/initial_pose_locked"
        self.cmd_vel_topic = "/cmd_vel" if amr_id == "amr1" else "/amr2/cmd_vel"
        self.auto_command_topic = f"/{amr_id}/auto_approach/command"
        self.align_status_topic = f"/{amr_id}/align/status"
        self.ocr_request_topic = f"/{amr_id}/ocr/request"
        self.ocr_result_topic = f"/{amr_id}/ocr/result"
        self.ocr_control_topic = f"/{amr_id}/ocr/control"
        self.aruco_result_topic = f"/{amr_id}/aruco/result"
        self.magnet_command_topic = f"/{amr_id}/magnet/command"
        self.magnet_status_topic = f"/{amr_id}/magnet/status"
        self.elevator_up_service = f"/{amr_id}/elevator/request_up"
        self.elevator_down_service = f"/{amr_id}/elevator/request_down"
        self.elevator_final_topic = f"/{amr_id}/elevator/arrived"
        self.inspection_complete_service = f"/{amr_id}/inspection/complete"
        self.map_load_service = "/map_server/load_map" if amr_id == "amr1" else "/amr2/map_server/load_map"
        self.patient_name = str(self.patient["name"])
        self.patient_birth_date = str(self.patient["birth_date"])
        self.ocr_pose = dict(self.patient["ocr_pose"])
        self.approach_distance_m = float(self.patient["approach_distance_m"])
        self.room_reverse_distance_m = self.approach_distance_m * 0.5
        self.inspection_complete = False

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.goal_pub = self.create_publisher(PoseStamped, self.center_goal_topic, 10)
        # Kept for compatibility with the Isaac process, but the mission no longer
        # delegates docking geometry to Isaac AutoApproach/OCR bbox tracking.
        self.auto_pub = self.create_publisher(String, self.auto_command_topic, 10)
        self.ocr_request_pub = self.create_publisher(String, self.ocr_request_topic, 10)
        self.ocr_control_pub = self.create_publisher(String, self.ocr_control_topic, 10)
        self.magnet_pub = self.create_publisher(String, self.magnet_command_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        # Legacy arrival topic publisher is retained for compatibility, but the
        # mission itself uses request/response services for elevator commands.
        self.elevator_arrival_pub = self.create_publisher(String, ELEVATOR_ARRIVAL_TOPIC, 10)
        self.elevator_map_ack_pub = self.create_publisher(String, ELEVATOR_MAP_ACK_TOPIC, 10)
        self.map_client = self.create_client(LoadMap, self.map_load_service)
        self.elevator_up_client = self.create_client(Trigger, self.elevator_up_service)
        self.elevator_down_client = self.create_client(Trigger, self.elevator_down_service)
        self.inspection_service = self.create_service(
            Trigger, self.inspection_complete_service, self._on_inspection_complete_service
        )

        self.create_subscription(String, self.center_status_topic, self._on_nav, latched)
        self.create_subscription(String, self.align_status_topic, self._on_align, 10)
        self.create_subscription(String, self.ocr_result_topic, self._on_ocr_result, 10)
        self.create_subscription(String, self.aruco_result_topic, self._on_aruco_result, 10)
        self.create_subscription(String, self.magnet_status_topic, self._on_magnet, 10)
        self.create_subscription(Bool, self.initial_lock_topic, self._on_lock, latched)
        self.create_subscription(String, ELEVATOR_STATUS_TOPIC, self._on_elevator_status, 10)
        self.create_subscription(Bool, self.elevator_final_topic, self._on_elevator_final, latched)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_locked = False
        self.nav_status = ""
        self.align_payload: dict = {}
        self.ocr_payload: dict = {}
        self.aruco_payload: dict = {}
        self.aruco_received_at = 0.0
        self.magnet_payload: dict = {}
        self.elevator_payload: dict = {}
        self.elevator_final = False

        # Traffic priority signal: only true while this AMR is doing a non-Nav special motion
        # (OCR alignment/forward, bed coupling/release, forced straight motion, elevator sequence).
        # A short heartbeat lets the traffic manager auto-release if this mission process dies.
        self.special_motion_active = False
        self.special_motion_reason = ""
        self.special_motion_pub = self.create_publisher(Bool, f"/{self.amr_id}/special_motion_active", 10)
        self.create_timer(0.25, self._publish_special_motion_state)
        self._publish_special_motion_state()

    def _publish_special_motion_state(self) -> None:
        msg = Bool()
        msg.data = bool(self.special_motion_active)
        self.special_motion_pub.publish(msg)

    def _set_special_motion(self, active: bool, reason: str = "") -> None:
        active = bool(active)
        changed = active != self.special_motion_active or (active and reason != self.special_motion_reason)
        self.special_motion_active = active
        self.special_motion_reason = str(reason) if active else ""
        self._publish_special_motion_state()
        if changed:
            state = "ON" if active else "OFF"
            suffix = f" ({self.special_motion_reason})" if self.special_motion_reason else ""
            print(f"[TRAFFIC SPECIAL] {self.amr_id.upper()} special priority {state}{suffix}")

    @contextmanager
    def special_motion(self, reason: str):
        self._set_special_motion(True, reason)
        try:
            yield
        finally:
            self._set_special_motion(False)


    def _on_inspection_complete_service(self, request, response):
        del request
        self.inspection_complete = True
        response.success = True
        response.message = "inspection complete accepted; MRI re-entry and return scenario released"
        print(f"[검사완료 SERVICE] {self.inspection_complete_service} 요청 수신 -> MRI 11m 재진입 후 복귀 허용")
        return response

    def _on_nav(self, msg: String) -> None:
        self.nav_status = str(msg.data)

    def _on_align(self, msg: String) -> None:
        try:
            self.align_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.align_payload = {"state": "ERROR", "reason": "invalid align status JSON"}

    def _on_ocr_result(self, msg: String) -> None:
        try:
            self.ocr_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_aruco_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(payload.get("amr", self.amr_id)).lower() != self.amr_id:
            return
        self.aruco_payload = payload
        self.aruco_received_at = time.monotonic()

    def _on_magnet(self, msg: String) -> None:
        try:
            self.magnet_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_lock(self, msg: Bool) -> None:
        self.initial_locked = bool(msg.data)

    def _on_elevator_status(self, msg: String) -> None:
        try:
            self.elevator_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_elevator_final(self, msg: Bool) -> None:
        self.elevator_final = bool(msg.data)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self) -> None:
        msg = Twist()
        for _ in range(8):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

    def wait_for_system(self, timeout: float = 45.0) -> bool:
        print("[준비] 자동 초기 Pose, Nav2, Isaac, OCR + ArUco launch 연결을 확인합니다.")
        end = time.monotonic() + timeout
        last = 0.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            nav_ready = self.goal_pub.get_subscription_count() > 0 and not self.nav_status.startswith("ACTIVE")
            isaac_ready = self.auto_pub.get_subscription_count() > 0
            magnet_ready = self.magnet_pub.get_subscription_count() > 0
            ocr_ready = self.ocr_request_pub.get_subscription_count() > 0
            aruco_ready = self.count_publishers(self.aruco_result_topic) > 0
            tf_ready = self.lookup_pose("map", self.base_frame) is not None
            if self.initial_locked and nav_ready and isaac_ready and magnet_ready and ocr_ready and aruco_ready and tf_ready:
                print(
                    f"[준비 완료] 자동 Pose + {self.amr_id.upper()} Nav2 + Isaac + "
                    "OCR 신원확인 + ArUco 중심검출 연결 완료"
                )
                return True
            now = time.monotonic()
            if now - last > 2.0:
                print(
                    "[대기] "
                    f"pose={self.initial_locked}, nav={nav_ready}, tf={tf_ready}, "
                    f"isaac={isaac_ready}, magnet={magnet_ready}, "
                    f"ocr_launch={ocr_ready}, aruco={aruco_ready}"
                )
                last = now
        print("[실패] 시스템 준비 제한시간 초과")
        print(
            f"       OCR 터미널에 '[{self.amr_id}] OCR model ready'와 "
            "'paired ArUco ready'가 모두 보이는지 확인하세요."
        )
        return False

    def lookup_pose(self, parent: str, child: str):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), float(yaw)

    def make_goal(self, target: dict[str, float]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(target["x"])
        msg.pose.position.y = float(target["y"])
        if bool(target.get("keep_arrival_heading", False)):
            # Internal centerline sentinel: zero quaternion = no extra final yaw turn.
            # The last straight segment heading is kept for the following reverse.
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
        else:
            if "yaw" in target:
                yaw = float(target["yaw"])
            else:
                pose = self.lookup_pose("map", self.base_frame)
                yaw = float(pose[2]) if pose is not None else 0.0
            msg.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def navigate(self, label: str, target: dict[str, float], timeout: float = 240.0) -> bool:
        if self.goal_pub.get_subscription_count() < 1:
            print(f"[Nav2 실패] {self.center_goal_topic} 구독자가 없습니다.")
            return False
        self.nav_status = ""
        goal = self.make_goal(target)
        self.goal_pub.publish(goal)
        print(
            f"[Nav2 시작] {label}: "
            f"x={target['x']:.4f}, y={target['y']:.4f}"
            + (
                ", 도착 직선 heading 유지"
                if bool(target.get("keep_arrival_heading", False))
                else (f", yaw={target['yaw']:.6f}" if "yaw" in target else ", 현재 yaw 유지")
            )
        )
        active_seen = False
        end = None if timeout <= 0.0 else time.monotonic() + timeout
        last_print = 0.0
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.nav_status
            if status.startswith("ACTIVE"):
                active_seen = True
            if active_seen and status == "SUCCEEDED":
                self.stop()
                pose = self.lookup_pose("map", self.base_frame)
                if pose:
                    print(f"[Nav2 완료] {label}: 현재=({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.6f})")
                else:
                    print(f"[Nav2 완료] {label}")
                return True
            if status.startswith("FAILED"):
                self.stop()
                print(f"[Nav2 실패] {label}: {status}")
                return False
            now = time.monotonic()
            if now - last_print > 2.0:
                pose = self.lookup_pose("map", self.base_frame)
                if pose:
                    remain = math.hypot(float(target["x"]) - pose[0], float(target["y"]) - pose[1])
                    print(f"[Nav2 이동] {label}: 상태={status or 'WAIT'}, 남은 거리≈{remain:.2f}m")
                last_print = now
        self.stop()
        print(f"[Nav2 실패] {label} 제한시간 초과")
        return False

    def start_ocr_approach(self, timeout: float = 120.0) -> bool:
        """Verify identity with OCR, then centre only on the paired ArUco markers."""
        if self.ocr_request_pub.get_subscription_count() < 1:
            print(f"[OCR 실패] {self.ocr_request_topic} 구독자가 없습니다.")
            return False
        if self.count_publishers(self.aruco_result_topic) < 1:
            print(f"[ArUco 실패] {self.aruco_result_topic} publisher가 없습니다.")
            return False

        request_id = f"{self.amr_id}-{self.patient_name}-{uuid.uuid4().hex[:10]}"
        self.ocr_payload = {}
        req = String()
        req.data = json.dumps(
            {
                "protocol_version": 1,
                "command": "VERIFY_AND_TRACK",
                "request_id": request_id,
                "amr": self.amr_id,
                "expected_name": self.patient_name,
                "expected_birth_date": self.patient_birth_date,
                "frames_to_check": 2,
                "candidates": [
                    {"name": item["name"], "birth_date": item["birth_date"]}
                    for item in PATIENTS.values()
                ],
                "timestamp": time.time(),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.ocr_request_pub.publish(req)
        print(f"[OCR 신원확인] {self.patient_name} {self.patient_birth_date} 확인 요청")

        def stop_ocr(reason: str) -> None:
            control = String()
            control.data = json.dumps(
                {
                    "protocol_version": 1,
                    "action": "STOP_TRACKING",
                    "request_id": request_id,
                    "amr": self.amr_id,
                    "reason": reason,
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.ocr_control_pub.publish(control)

        verify_timeout = min(60.0, timeout)
        verify_end = time.monotonic() + verify_timeout
        verified = False
        last_state = ""
        while rclpy.ok() and time.monotonic() < verify_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if str(self.ocr_payload.get("request_id", "")) != request_id:
                continue
            state = str(self.ocr_payload.get("state", ""))
            if state and state != last_state:
                print(f"[OCR 상태] {state}")
                last_state = state
            if state in {"REJECTED", "ERROR"}:
                print(f"[OCR 실패] {self.ocr_payload.get('reason', state)}")
                stop_ocr("OCR verification failed")
                self.stop()
                return False
            if state in {"VERIFIED", "TRACKING"} and bool(self.ocr_payload.get("verified", False)):
                if str(self.ocr_payload.get("selected_name", "")) != self.patient_name:
                    print("[OCR 실패] 이름 불일치")
                    stop_ocr("patient name mismatch")
                    return False
                if str(self.ocr_payload.get("selected_birth_date", "")) != self.patient_birth_date:
                    print("[OCR 실패] 생년월일 불일치")
                    stop_ocr("patient birth date mismatch")
                    return False
                verified = True
                break
        if not verified:
            print("[OCR 실패] 신원확인 제한시간 초과")
            stop_ocr("OCR verification timeout")
            self.stop()
            return False

        print("[OCR 성공] 환자 신원 확인 완료 — OCR bbox는 위치 제어에 사용하지 않습니다.")
        stop_ocr("identity verified; switch to paired ArUco centre")

        expected_left, expected_right = ARUCO_PAIRS[self.patient_name]
        print(
            f"[ArUco 중심정렬] {self.patient_name} 침대 pair={expected_left}/{expected_right} "
            "두 마커 중점을 카메라 중심으로 맞춥니다."
        )
        stable = 0
        wrong_pair_count = 0
        last_msg_stamp = None
        last_direction = ""
        align_timeout = max(20.0, timeout - verify_timeout)
        align_end = time.monotonic() + align_timeout
        while rclpy.ok() and time.monotonic() < align_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if not self.aruco_payload or now - self.aruco_received_at > ARUCO_RESULT_STALE_SEC:
                self.stop()
                continue
            stamp = self.aruco_payload.get("timestamp")
            pair = self.aruco_payload.get("pairs", {}).get(self.patient_name)
            visible = self.aruco_payload.get("visible_ids", [])
            complete = [str(item) for item in self.aruco_payload.get("complete_patients", [])]
            if pair is None:
                self.stop()
                wrong = [name for name in complete if name != self.patient_name]
                if wrong:
                    wrong_pair_count += 1
                    if wrong_pair_count >= 5:
                        print(f"[ArUco 실패] OCR={self.patient_name}, 다른 침대 pair={wrong} 검출")
                        return False
                else:
                    wrong_pair_count = 0
                if stamp != last_msg_stamp:
                    print(
                        f"\r[ArUco 대기] expected={expected_left}/{expected_right}, visible={visible}",
                        end="",
                        flush=True,
                    )
                    last_msg_stamp = stamp
                continue
            wrong_pair_count = 0

            try:
                error_px = float(pair["center_error_px"])
                image_width = float(self.aruco_payload["image_width"])
            except (KeyError, TypeError, ValueError):
                self.stop()
                continue
            is_new = stamp != last_msg_stamp
            if is_new:
                last_msg_stamp = stamp

            if abs(error_px) <= ARUCO_X_TOLERANCE_PX:
                self.stop()
                if is_new:
                    stable += 1
                    print(
                        f"\r[ArUco CENTER] error={error_px:+.1f}px "
                        f"stable={stable}/{ARUCO_STABLE_MESSAGES}",
                        end="",
                        flush=True,
                    )
                if stable >= ARUCO_STABLE_MESSAGES:
                    print("\n[ArUco 중심정렬 완료] 좌/우 마커 중점 = 침대 도킹 중심선")
                    break
                continue

            if is_new:
                stable = 0
            normalized = error_px / max(1.0, image_width * 0.5)
            cmd = Twist()
            if abs(error_px) > ARUCO_YAW_THRESHOLD_PX:
                raw_w = -ARUCO_YAW_KP * normalized
                raw_w = max(-ARUCO_MAX_YAW_RAD_S, min(ARUCO_MAX_YAW_RAD_S, raw_w))
                if 0.0 < abs(raw_w) < ARUCO_MIN_YAW_RAD_S:
                    raw_w = math.copysign(ARUCO_MIN_YAW_RAD_S, raw_w)
                cmd.angular.z = raw_w
                direction = "ROTATE_LEFT" if raw_w > 0 else "ROTATE_RIGHT"
            else:
                raw_y = -ARUCO_LATERAL_KP * normalized
                raw_y = max(-ARUCO_MAX_LATERAL_MPS, min(ARUCO_MAX_LATERAL_MPS, raw_y))
                cmd.linear.y = raw_y
                direction = "Q-equivalent" if raw_y > 0 else "E-equivalent"
            self.cmd_pub.publish(cmd)
            if is_new and direction != last_direction:
                print(
                    f"\n[ArUco ALIGN] {direction}: pair_center={float(pair['pair_center_x']):.1f}px "
                    f"screen_center={image_width * 0.5:.1f}px error={error_px:+.1f}px"
                )
                last_direction = direction
        else:
            self.stop()
            print("\n[ArUco 실패] 중심정렬 제한시간 초과")
            return False

        self.stop()
        self.spin_for(0.25)
        print(f"[ArUco 기준 전진] 환자 설정 거리 {self.approach_distance_m:.4f}m 전진 시작")
        start_pose = self.lookup_pose(self.odom_frame, self.base_frame)
        if start_pose is None:
            print(f"[ArUco 전진 실패] {self.odom_frame}->{self.base_frame} TF 없음")
            return False
        forward_end = time.monotonic() + 30.0
        last_print = 0.0
        cmd = Twist()
        cmd.linear.x = ARUCO_APPROACH_SPEED_MPS
        while rclpy.ok() and time.monotonic() < forward_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose(self.odom_frame, self.base_frame)
            if current is None:
                self.stop()
                continue
            moved = math.hypot(current[0] - start_pose[0], current[1] - start_pose[1])
            if moved >= self.approach_distance_m:
                self.stop()
                print(
                    f"\n[ArUco 자동 접근 완료] OCR 신원확인 + ArUco pair 중심정렬 + "
                    f"{moved:.3f}m 전진"
                )
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(
                    f"\r[ArUco 전진] {moved:.3f}/{self.approach_distance_m:.3f}m",
                    end="",
                    flush=True,
                )
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[ArUco 전진 실패] 제한시간 초과")
        return False

    def lock_bed(self) -> bool:
        if self.magnet_pub.get_subscription_count() < 1:
            print("[결합 실패] Isaac magnet command 구독자가 없습니다.")
            return False
        for attempt in range(1, 4):
            request_id = f"lock-{uuid.uuid4().hex[:10]}"
            self.magnet_payload = {}
            msg = String()
            msg.data = json.dumps(
                {"command": "C", "request_id": request_id},
                separators=(",", ":"),
            )
            self.magnet_pub.publish(msg)
            print(f"[C 결합 {attempt}/3] C 키와 동일한 LOCK 명령 전송")
            end = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.magnet_payload.get("request_id") != request_id:
                    continue
                if bool(self.magnet_payload.get("success")):
                    print(f"[결합 성공] {self.magnet_payload.get('attached_bed', '')}")
                    print("[자동 리프트] R 상승과 동일하게 침대 캐스터를 바닥에서 분리합니다.")
                    print("[Nav2 footprint] AMR 정사각형 0.74m x 0.74m 적용, 침대 확장 footprint는 사용하지 않습니다.")
                    return True
                print(f"[결합 응답 실패] {self.magnet_payload.get('state', '')}")
                break
            time.sleep(0.3)
        print("[결합 실패 종료]")
        return False

    def forced_reverse(self, distance_m: float, speed_mps: float = FORCED_STRAIGHT_SPEED_MPS) -> bool:
        start = self.lookup_pose(self.odom_frame, self.base_frame)
        if start is None:
            print(f"[후진 실패] {self.odom_frame}->{self.base_frame} TF 없음")
            return False
        print(
            f"[강제 직선 후진] {distance_m:.3f}m, linear.x={-speed_mps:.2f}, "
            "linear.y=0, angular.z=0"
        )
        cmd = Twist()
        cmd.linear.x = -abs(float(speed_mps))
        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose(self.odom_frame, self.base_frame)
            if current is None:
                self.cmd_pub.publish(Twist())
                continue
            moved = math.hypot(current[0] - start[0], current[1] - start[1])
            if moved >= distance_m:
                self.stop()
                print(f"[강제 후진 완료] 실제 이동={moved:.3f}m")
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(f"\r[강제 후진] {moved:.3f}/{distance_m:.3f}m", end="", flush=True)
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[강제 후진 중단] ROS 종료")
        return False

    def forced_forward(self, distance_m: float, speed_mps: float = FORCED_STRAIGHT_SPEED_MPS) -> bool:
        start = self.lookup_pose(self.odom_frame, self.base_frame)
        if start is None:
            print(f"[전진 실패] {self.odom_frame}->{self.base_frame} TF 없음")
            return False
        print(
            f"[강제 직선 전진] {distance_m:.3f}m, linear.x={speed_mps:.2f}, "
            "linear.y=0, angular.z=0"
        )
        cmd = Twist()
        cmd.linear.x = abs(float(speed_mps))
        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose(self.odom_frame, self.base_frame)
            if current is None:
                self.cmd_pub.publish(Twist())
                continue
            moved = math.hypot(current[0] - start[0], current[1] - start[1])
            if moved >= distance_m:
                self.stop()
                print(f"[강제 전진 완료] 실제 이동={moved:.3f}m")
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(f"\r[강제 전진] {moved:.3f}/{distance_m:.3f}m", end="", flush=True)
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[강제 전진 중단] ROS 종료")
        return False

    def capture_map_pose(self, label: str) -> dict[str, float] | None:
        pose = self.lookup_pose("map", self.base_frame)
        if pose is None:
            print(f"[위치 저장 실패] {label}: map->base_link TF 없음")
            return None
        saved = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        print(
            f"[위치 저장] {label}: x={saved['x']:.4f}, y={saved['y']:.4f}, "
            f"yaw={saved['yaw']:.6f}"
        )
        return saved

    def switch_to_floor_map(self, floor: str, request_id: str) -> bool:
        floor = str(floor).strip().lower()
        if floor not in {"1f", "2f"}:
            reason = f"invalid floor map request: {floor}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        map_yaml = (
            Path(__file__).resolve().parent
            / f"ros2_ws/src/hospital_nav2/maps/hospital_map_{floor}.yaml"
        )
        if not map_yaml.is_file():
            reason = f"{floor.upper()} map missing: {map_yaml}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        if not self.map_client.wait_for_service(timeout_sec=10.0):
            reason = f"service unavailable: {self.map_load_service}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        req = LoadMap.Request()
        req.map_url = str(map_yaml)
        future = self.map_client.call_async(req)
        end = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done() or future.result() is None:
            reason = f"{floor.upper()} LoadMap timeout/no response"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        response = future.result()
        success = int(response.result) == int(getattr(response, "RESULT_SUCCESS", 0))
        reason = "" if success else f"LoadMap result={int(response.result)}"
        print(f"[맵 전환] -> {floor.upper()} {'완료' if success else '실패'}: {map_yaml}")
        self._publish_map_ack(request_id, success, floor, reason)
        return success

    def _publish_map_ack(
        self,
        request_id: str,
        success: bool,
        floor: str,
        reason: str = "",
    ) -> None:
        msg = String()
        msg.data = json.dumps({
            "request_id": request_id,
            "success": bool(success),
            "floor": floor,
            "reason": reason,
        }, ensure_ascii=False, separators=(",", ":"))
        self.elevator_map_ack_pub.publish(msg)

    def _call_elevator_service(self, direction: str, timeout: float = 60.0) -> str | None:
        direction = str(direction).strip().lower()
        client = self.elevator_up_client if direction == "up" else self.elevator_down_client
        service_name = self.elevator_up_service if direction == "up" else self.elevator_down_service
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=min(10.0, timeout)):
            print(f"[엘리베이터 SERVICE 실패] {service_name} 없음")
            return None

        while rclpy.ok() and time.monotonic() < deadline:
            future = client.call_async(Trigger.Request())
            response_deadline = min(deadline, time.monotonic() + 10.0)
            while rclpy.ok() and time.monotonic() < response_deadline and not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done() or future.result() is None:
                print(f"[엘리베이터 SERVICE 실패] {service_name} 응답 timeout")
                return None

            response = future.result()
            payload = {}
            try:
                payload = json.loads(str(response.message))
            except Exception:
                payload = {}

            if bool(response.success):
                request_id = str(payload.get("request_id", "")) or str(response.message).strip()
                print(f"[엘리베이터 SERVICE 승인] {service_name} request_id={request_id}")
                return request_id or None

            state = str(payload.get("state", "")).upper()
            if state == "BUSY":
                owner = str(payload.get("owner", "")) or "other AMR"
                print(f"[엘리베이터 대기] 현재 {owner} 사용 중 -> 1초 후 SERVICE 재요청")
                wait_end = min(deadline, time.monotonic() + 1.0)
                while rclpy.ok() and time.monotonic() < wait_end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                continue

            print(f"[엘리베이터 SERVICE 거절] {response.message}")
            return None

        print(f"[엘리베이터 SERVICE 실패] {service_name} BUSY 대기 제한시간 초과")
        return None

    def run_elevator_sequence(self, direction: str = "up", timeout: float = 240.0) -> bool:
        direction = str(direction).strip().lower()
        if direction not in {"up", "down"}:
            print(f"[엘리베이터 실패] invalid direction: {direction}")
            return False
        is_up = direction == "up"
        self.elevator_payload = {}
        self.elevator_final = False
        request_id = self._call_elevator_service(direction)
        if not request_id:
            return False
        if is_up:
            print("[엘리베이터 상승 시작] SERVICE 응답 승인 -> 도착 heading 그대로 전진 탑승 -> 2F -> 5m 후진 하차")
        else:
            print("[엘리베이터 하강 시작] 2F Nav2 종료 상태 -> 오른쪽 90도 -> 문 열림 -> Y=25.91 탑승 -> 1F 문 열림")

        end = time.monotonic() + timeout
        last_state = ""
        map_switched = False
        sequence_started = False
        expected_floor = "2f" if is_up else "1f"
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            payload = self.elevator_payload
            payload_request = str(payload.get("request_id", ""))
            if payload_request and payload_request != request_id:
                continue
            state = str(payload.get("state", ""))
            if payload_request == request_id and state:
                sequence_started = True
            if state and state != last_state:
                print(f"[엘리베이터 상태] {state}")
                last_state = state
            if state == "MAP_SWITCH_REQUIRED" and not map_switched:
                requested_floor = str(payload.get("floor", expected_floor)).strip().lower() or expected_floor
                if not self.switch_to_floor_map(requested_floor, request_id):
                    return False
                map_switched = True
            if state in {"FAILED", "BUSY"}:
                print(f"[엘리베이터 실패] {payload.get('reason', state)}")
                return False
            if sequence_started and (state == "COMPLETE" or self.elevator_final):
                print(f"[엘리베이터 {'상승' if is_up else '하강'} 완료] {expected_floor.upper()} 맵 + {'5m 하차' if is_up else '1F 엘리베이터 외부 좌표 하차'} 완료")
                return True
        print(f"[엘리베이터 실패] {direction} 자동 시퀀스 제한시간 초과")
        return False

    def release_bed(self) -> bool:
        if self.magnet_pub.get_subscription_count() < 1:
            print("[결합해체 실패] Isaac magnet command 구독자가 없습니다.")
            return False
        request_id = f"release-{uuid.uuid4().hex[:10]}"
        self.magnet_payload = {}
        msg = String()
        msg.data = json.dumps({"command": "X", "request_id": request_id}, separators=(",", ":"))
        self.magnet_pub.publish(msg)
        print("[X 결합해체] 병실 결합 위치에서 RELEASE 명령 전송")
        end = time.monotonic() + 4.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.magnet_payload.get("request_id") != request_id:
                continue
            if bool(self.magnet_payload.get("success")):
                print("[결합해체 완료] 침대 분리 + 리프트 하강")
                return True
            print(f"[결합해체 실패] {self.magnet_payload.get('state', '')}")
            return False
        print("[결합해체 실패] 응답 timeout")
        return False

    def wait_for_inspection_complete(self) -> bool:
        self.inspection_complete = False
        print("\n===== MRI 검사 완료 대기 =====")
        print("OCR/미션 터미널에서 K 키를 누르면 11m 전진 -> MRI 3초 대기 -> 병실 복귀를 시작합니다.")
        print(f"외부 장비는 SERVICE 호출 가능: ros2 service call {self.inspection_complete_service} std_srvs/srv/Trigger '{{}}'")
        fd = None
        old_term = None
        try:
            fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
            old_term = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception as exc:
            print(f"[K 입력 경고] /dev/tty 직접 입력 사용 불가: {exc}")
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
                fd = None
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.inspection_complete:
                    print("[검사완료] SERVICE 신호 확인 -> 11m 전진/MRI 3초 대기 후 복귀 시작")
                    return True
                if fd is not None:
                    ready, _, _ = select.select([fd], [], [], 0.0)
                    if ready:
                        try:
                            key = os.read(fd, 1).decode(errors="ignore").lower()
                        except BlockingIOError:
                            key = ""
                        if key == "k":
                            self.inspection_complete = True
                            print("\n[검사완료 K] K 입력 확인 -> 11m 전진/MRI 3초 대기 후 복귀 시작")
                            return True
            return False
        finally:
            if fd is not None:
                if old_term is not None:
                    try:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
                    except Exception:
                        pass
                try:
                    os.close(fd)
                except Exception:
                    pass

    def run(self) -> bool:
        print(f"===== {self.patient_name} 침대 이송 최종 시나리오 =====")
        print(
            "자동 시작 Pose: "
            f"({self.initial_pose['x']}, {self.initial_pose['y']}, {self.initial_pose['yaw']})"
        )
        print(
            f"[환자 설정] OCR=({self.ocr_pose['x']:.4f}, {self.ocr_pose['y']:.4f}, {self.ocr_pose['yaw']:.6f}), "
            f"전진={self.approach_distance_m:.4f}m, 결합 후 후진={self.room_reverse_distance_m:.4f}m"
        )
        if not self.wait_for_system():
            return False
        if not self.navigate(f"{self.patient_name} OCR 위치", self.ocr_pose):
            return False
        self.stop()
        # 기존 0.5초보다 짧게. Nav2가 SUCCEEDED를 준 뒤 즉시 OCR 준비로 넘어간다.
        self.spin_for(0.15)
        # 서수원은 빠른 2-frame OCR에서 간헐적으로 첫 판독이 실패할 수 있으므로
        # 같은 OCR 위치/자세에서 최초 1회 + 추가 2회, 총 3회까지만 재시도한다.
        # 김서울/박인천도 첫 실패 시 같은 위치에서 1회만 추가 재판독한다.
        with self.special_motion("PATIENT_PICKUP_OCR_ALIGN_COUPLE"):
            ocr_attempts = 3 if self.patient_name == "서수원" else 2
            ocr_ok = False
            for ocr_attempt in range(1, ocr_attempts + 1):
                if ocr_attempts > 1:
                    print(f"[OCR 판독 {ocr_attempt}/{ocr_attempts}] {self.patient_name}")
                if self.start_ocr_approach():
                    ocr_ok = True
                    break
                if ocr_attempt < ocr_attempts:
                    self.stop()
                    print(f"[OCR 재시도] 첫 판독 실패 — 같은 위치에서 다시 판독합니다. ({ocr_attempt + 1}/{ocr_attempts})")
                    self.spin_for(0.4)
            if not ocr_ok:
                return False
            self.stop()
            self.spin_for(0.4)
            if not self.lock_bed():
                return False
            self.stop()
            print("[자동 리프트 대기] 목표 높이 0.035m 상승을 위해 3.5초 대기")
            self.spin_for(3.5)
            if not self.forced_reverse(self.room_reverse_distance_m):
                return False
            self.stop()
            self.spin_for(0.6)

        # 이 위치를 저장해 두면 귀환 시 같은 yaw로 맞춘 뒤 절반 거리만 전진해
        # 최초 결합 위치를 정확히 재현할 수 있다.
        room_return_staging = self.capture_map_pose("병실 복귀용 결합 전진 시작 위치")
        if room_return_staging is None:
            return False

        if not self.navigate("1층 엘리베이터 도달", ELEVATOR_1F, timeout=0.0):
            return False
        self.stop()
        with self.special_motion("ELEVATOR_UP_SEQUENCE"):
            if not self.run_elevator_sequence("up"):
                return False
        self.stop()
        self.spin_for(0.6)

        # 2층 엘리베이터 목표는 X/Y만 사용한다. 이 좌표 도착 즉시 Nav2는 종료된다.
        # 이후 Isaac 엘리베이터 시퀀스가 닫힌 문 앞에서 오른쪽 90도 회전을 먼저 끝내고,
        # 그 다음 2F 문을 연 뒤 기존 내부 목표 Y=25.91까지 직접 전진한다.
        elevator_2f_return = {
            "x": float(ELEVATOR_2F_XY["x"]),
            "y": float(ELEVATOR_2F_XY["y"]),
            "keep_arrival_heading": True,
        }

        print(f"[2F MRI 이동] 목표 x={MRI_2F['x']:.5f}, y={MRI_2F['y']:.5f}")
        if not self.navigate("2층 MRI 도착", MRI_2F, timeout=0.0):
            return False
        self.stop()
        print(f"[MRI 도착] 들어온 방향 그대로 {MRI_WAIT_S:.1f}초 정지")
        self.spin_for(MRI_WAIT_S)
        with self.special_motion("MRI_FORCED_REVERSE"):
            if not self.forced_reverse(MRI_REVERSE_DISTANCE_M):
                return False
        self.stop()
        print(f"[MRI 검사 대기 위치] 같은 방향으로 {MRI_REVERSE_DISTANCE_M:.1f}m 후진 완료")
        if not self.wait_for_inspection_complete():
            return False

        # K는 즉시 엘리베이터 복귀 신호가 아니다. 검사 대기 위치에서 MRI까지
        # 들어왔던 방향 그대로 11 m 전진하여 되돌아온 뒤 3초 정지한다.
        # 이 3초 동안 기존 PatientTransfer auto-cycle이 MRI -> 침대로 환자를 복귀시킨다.
        print(f"[K 이후 MRI 복귀] 들어왔던 방향 그대로 {MRI_REVERSE_DISTANCE_M:.1f}m 전진")
        with self.special_motion("MRI_FORCED_FORWARD"):
            if not self.forced_forward(MRI_REVERSE_DISTANCE_M):
                return False
        self.stop()
        print(f"[MRI 재도착] 환자 침대 복귀를 위해 {MRI_WAIT_S:.1f}초 정지")
        self.spin_for(MRI_WAIT_S)

        print(
            f"[2F 엘리베이터 복귀] 고정 좌표 x={elevator_2f_return['x']:.5f}, "
            f"y={elevator_2f_return['y']:.5f}, 도착 즉시 Nav2 OFF -> 오른쪽 90도 완료 -> 문 열림 -> Y=25.91 직접 탑승"
        )
        if not self.navigate("2층 엘리베이터 도달", elevator_2f_return, timeout=0.0):
            return False
        self.stop()
        self.spin_for(0.4)
        with self.special_motion("ELEVATOR_DOWN_SEQUENCE"):
            if not self.run_elevator_sequence("down"):
                return False
        self.stop()
        self.spin_for(0.6)

        # DOWN 엘리베이터 시퀀스 자체가 1F 문이 열린 뒤 외부 이탈까지 담당한다.
        # 여기서부터 다시 Nav2를 켜 병실 복귀를 시작한다.
        print(
            f"[1F 엘리베이터 좌표 도착] x={ELEVATOR_1F['x']:.5f}, y={ELEVATOR_1F['y']:.5f} "
            "-> 여기서부터 Nav2 병실 복귀 시작"
        )
        self.stop()
        self.spin_for(0.3)

        print("[1F 복귀/Nav2 ON] 결합 직후 절반 후진이 끝났던 병실 복귀 위치로 이동합니다.")
        if not self.navigate("병실 복귀 대기 위치", room_return_staging, timeout=0.0):
            return False
        self.stop()
        self.spin_for(0.3)
        with self.special_motion("PATIENT_DROPOFF_RELEASE"):
            if not self.forced_forward(self.room_reverse_distance_m):
                return False
            self.stop()
            if not self.release_bed():
                return False
            self.spin_for(1.0)
    
            # X 해체 후에도 방금 침대로 들어간 yaw를 유지한다. 최초 결합 뒤 빠져나왔던
            # 거리와 동일한 1/2 거리만큼 다시 후진한 다음 초기 Pose로 자율주행한다.
            print(
                f"[X 이후 병실 이탈] 들어갔던 방향 그대로 {self.room_reverse_distance_m:.4f}m "
                "후진(환자별 최초 전진거리의 1/2)"
            )
            if not self.forced_reverse(self.room_reverse_distance_m):
                return False
            self.stop()
            self.spin_for(0.4)

        print(f"[도킹 복귀] 병실 1/2 후진 완료 -> {self.amr_id.upper()} 초기 Pose로 복귀")
        if not self.navigate("초기 도킹 스테이션", self.initial_pose, timeout=0.0):
            return False
        self.stop()
        print(
            f"[전체 성공] {self.patient_name} 병실 -> MRI 검사대기/K -> MRI 재진입 -> "
            "2F->1F 엘리베이터 -> 병실 원위치 -> 결합해체 -> 1/2 후진 -> 초기 도킹 스테이션 복귀 완료"
        )
        return True



def select_patient(argument: str | None) -> dict | None:
    aliases = {profile["name"]: key for key, profile in PATIENTS.items()}
    if argument is not None:
        value = argument.strip()
        key = value if value in PATIENTS else aliases.get(value)
        return dict(PATIENTS[key]) if key else None
    print("\n===== 이송할 환자를 선택하세요 =====")
    print("1. 김서울")
    print("2. 박인천")
    print("3. 서수원")
    try:
        selected = input("번호 입력 (1/2/3): ").strip()
    except EOFError:
        return None
    return dict(PATIENTS[selected]) if selected in PATIENTS else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("patient", nargs="?", help="1 김서울 / 2 박인천 / 3 서수원")
    parser.add_argument("--amr", choices=("amr1", "amr2"), default="amr1")
    args = parser.parse_args()
    patient = select_patient(args.patient)
    if patient is None:
        print("[종료] 환자 선택은 1.김서울 / 2.박인천 / 3.서수원 중 하나여야 합니다.")
        return 2

    rclpy.init()
    node = KimSeoulMission(patient, amr_id=args.amr)
    try:
        return 0 if node.run() else 1
    except KeyboardInterrupt:
        print("\n[사용자 중단] AMR 정지")
        return 130
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
