#!/usr/bin/env python3
"""Robust paired-ArUco detector for AMR bed docking.

Patient mapping is fixed and interpreted from the FRONT of the bed:
  김서울: left 10 / right 11
  박인천: left 20 / right 21
  서수원: left 30 / right 31

The detector publishes both centre alignment error and a signed yaw proxy based on
left/right apparent marker size.  A larger right marker means the bed's right side
is closer to the camera, so the AMR should rotate right to square itself to the bed.
"""
from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

PAIR_MAP: dict[str, tuple[int, int]] = {
    "김서울": (10, 11),
    "박인천": (20, 21),
    "서수원": (30, 31),
}
EXPECTED_IDS = {10, 11, 20, 21, 30, 31}


def _image_to_bgr(msg: Image) -> np.ndarray:
    enc = str(msg.encoding).lower()
    h, w = int(msg.height), int(msg.width)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channels is None:
        raise RuntimeError(f"unsupported image encoding: {msg.encoding}")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape(h, int(msg.step))[:, : w * channels]
    if channels == 1:
        return cv2.cvtColor(rows.reshape(h, w), cv2.COLOR_GRAY2BGR)
    image = rows.reshape(h, w, channels)
    if enc == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def _marker_side_px(points: np.ndarray) -> float:
    pts = points.reshape(4, 2).astype(np.float32)
    return float(sum(np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)) / 4.0)


class ArucoPairNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_pair_node")
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "cv2.aruco is unavailable. Install opencv-contrib-python==4.10.0.84 in the OCR venv."
            )

        self.declare_parameter("amr_id", "amr1")
        self.declare_parameter("image_topic", "/amr1/camera/front/color/image_raw")
        self.declare_parameter("result_topic", "/amr1/aruco/result")
        self.declare_parameter("debug_image_topic", "/amr1/aruco/debug_image")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_hz", 10.0)

        self.amr_id = str(self.get_parameter("amr_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_period = 1.0 / max(1.0, float(self.get_parameter("publish_hz").value))

        dictionary_name = str(self.get_parameter("dictionary").value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.params = cv2.aruco.DetectorParameters()
        # Small-marker friendly, but still restricted to the six known IDs below.
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 53
        self.params.adaptiveThreshWinSizeStep = 4
        self.params.minMarkerPerimeterRate = 0.01
        self.params.maxMarkerPerimeterRate = 4.0
        self.params.minCornerDistanceRate = 0.01
        self.params.minDistanceToBorder = 2
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params.cornerRefinementWinSize = 5
        self.params.cornerRefinementMaxIterations = 50
        self.params.cornerRefinementMinAccuracy = 0.01
        if hasattr(self.params, "detectInvertedMarker"):
            self.params.detectInvertedMarker = True

        try:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        except AttributeError:
            self.detector = None

        self.last_publish = -1.0
        self.pub = self.create_publisher(String, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"[{self.amr_id}] paired ArUco V2 ready: RGB={self.image_topic} OUT={self.result_topic} "
            "pairs=김서울 L10/R11, 박인천 L20/R21, 서수원 L30/R31"
        )

    def _detect_once(self, gray: np.ndarray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

    def _collect_markers(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        """Detect at native scale and, when needed, an upscaled contrast-enhanced view.

        Returned coordinates are always converted back to the original image pixels.
        If the same ID is seen more than once, the observation with the larger apparent
        side is kept because its corners are usually the cleaner estimate.
        """
        observations: dict[int, tuple[float, np.ndarray]] = {}

        def consume(img: np.ndarray, scale: float) -> None:
            corners, ids, _ = self._detect_once(img)
            if ids is None:
                return
            for marker_corners, raw_id in zip(corners, ids.flatten()):
                marker_id = int(raw_id)
                if marker_id not in EXPECTED_IDS:
                    continue
                pts = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2) / float(scale)
                side = _marker_side_px(pts)
                previous = observations.get(marker_id)
                if previous is None or side > previous[0]:
                    observations[marker_id] = (side, pts)

        consume(gray, 1.0)

        # If a complete patient pair was not found at native scale, make a second pass.
        native_ids = set(observations)
        has_pair = any(l in native_ids and r in native_ids for l, r in PAIR_MAP.values())
        if not has_pair:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            consume(clahe, 1.0)
            enlarged = cv2.resize(clahe, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            consume(enlarged, 2.0)

        return {marker_id: pts for marker_id, (_, pts) in observations.items()}

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.last_publish > 0.0 and now - self.last_publish < self.publish_period:
            return
        self.last_publish = now

        try:
            frame = _image_to_bgr(msg)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected = self._collect_markers(gray)
        except Exception as exc:
            self.get_logger().warning(f"ArUco image processing failed: {exc}")
            return

        marker_data: dict[int, dict[str, Any]] = {}
        for marker_id, pts in detected.items():
            center = pts.mean(axis=0)
            marker_data[marker_id] = {
                "id": marker_id,
                "center_x": float(center[0]),
                "center_y": float(center[1]),
                "side_px": _marker_side_px(pts),
                "corners": pts,
            }

        h, w = frame.shape[:2]
        visible_ids = sorted(marker_data)
        pairs: dict[str, dict[str, Any]] = {}
        complete_patients: list[str] = []

        for patient, (left_id, right_id) in PAIR_MAP.items():
            left = marker_data.get(left_id)
            right = marker_data.get(right_id)
            if left is None or right is None:
                continue

            lx, ly = float(left["center_x"]), float(left["center_y"])
            rx, ry = float(right["center_x"]), float(right["center_y"])
            left_size = max(1e-6, float(left["side_px"]))
            right_size = max(1e-6, float(right["side_px"]))
            size_mean = max(1.0, 0.5 * (left_size + right_size))
            center_x = 0.5 * (lx + rx)
            center_y = 0.5 * (ly + ry)
            # Signed yaw proxy. >0 means RIGHT marker appears closer/larger.
            yaw_error_ratio = float(math.log(right_size / left_size))
            pair_order_ok = bool(lx < rx)

            complete_patients.append(patient)
            pairs[patient] = {
                "patient": patient,
                "left_id": left_id,
                "right_id": right_id,
                "left_center_x": lx,
                "left_center_y": ly,
                "right_center_x": rx,
                "right_center_y": ry,
                "pair_center_x": center_x,
                "pair_center_y": center_y,
                "camera_center_x": float(w) * 0.5,
                "camera_center_y": float(h) * 0.5,
                "center_error_px": center_x - float(w) * 0.5,
                "pair_spacing_px": float(math.hypot(rx - lx, ry - ly)),
                "horizontal_spacing_px": float(rx - lx),
                "left_side_px": left_size,
                "right_side_px": right_size,
                "size_error_ratio": float((right_size - left_size) / size_mean),
                "yaw_error_ratio": yaw_error_ratio,
                "line_angle_deg": float(math.degrees(math.atan2(ry - ly, rx - lx))),
                "pair_order_ok": pair_order_ok,
                "min_marker_side_px": float(min(left_size, right_size)),
            }

        debug = frame.copy()
        camera_cx = int(round(w * 0.5))
        cv2.line(debug, (camera_cx, 0), (camera_cx, h - 1), (255, 255, 0), 2)
        cv2.putText(debug, "CAM CENTER", (max(4, camera_cx - 62), 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

        for marker_id, data in marker_data.items():
            pts = np.asarray(data["corners"], dtype=np.float32)
            poly = np.round(pts).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [poly], True, (0, 220, 0), 2, cv2.LINE_AA)
            cx, cy = np.round(pts.mean(axis=0)).astype(int)
            cv2.circle(debug, (int(cx), int(cy)), 4, (0, 220, 0), -1)
            cv2.putText(debug, f"ID {marker_id}", (int(cx) - 24, max(15, int(cy) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA)

        text_y = 42
        for patient, pair in pairs.items():
            lx, ly = int(round(pair["left_center_x"])), int(round(pair["left_center_y"]))
            rx, ry = int(round(pair["right_center_x"])), int(round(pair["right_center_y"]))
            pcx, pcy = int(round(pair["pair_center_x"])), int(round(pair["pair_center_y"]))
            center_err = float(pair["center_error_px"])
            yaw_err = float(pair["yaw_error_ratio"])
            order_ok = bool(pair["pair_order_ok"])
            pair_color = (0, 255, 0) if order_ok else (0, 0, 255)
            cv2.line(debug, (lx, ly), (rx, ry), pair_color, 2, cv2.LINE_AA)
            cv2.circle(debug, (pcx, pcy), 7, pair_color, 2)
            cv2.putText(
                debug,
                f"L{pair['left_id']}/R{pair['right_id']} x={center_err:+.1f}px yaw={yaw_err:+.3f}",
                (8, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, pair_color, 2, cv2.LINE_AA,
            )
            text_y += 23

        cv2.putText(debug, f"visible={visible_ids}", (8, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = int(h)
        debug_msg.width = int(w)
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = int(w * 3)
        debug_msg.data = debug.tobytes()
        self.debug_pub.publish(debug_msg)

        out = String()
        out.data = json.dumps(
            {
                "state": "TRACKING",
                "amr": self.amr_id,
                "timestamp": time.time(),
                "image_width": int(w),
                "image_height": int(h),
                "visible_ids": visible_ids,
                "complete_patients": complete_patients,
                "pairs": pairs,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoPairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
