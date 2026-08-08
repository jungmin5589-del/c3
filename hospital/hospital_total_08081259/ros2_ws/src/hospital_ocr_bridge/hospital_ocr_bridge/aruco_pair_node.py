#!/usr/bin/env python3
"""RGB-only paired ArUco detector used after OCR identity verification.

The two markers are physically separate PNG cards attached to the bed, not baked
into the nameplate.  OCR remains responsible only for patient identity.  This
node publishes the geometric centre of each expected left/right pair so docking
alignment never depends on the OCR nameplate bounding box.
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
        self.declare_parameter("amr_id", "amr1")
        self.declare_parameter("image_topic", "/amr1/camera/front/color/image_raw")
        self.declare_parameter("result_topic", "/amr1/aruco/result")
        self.declare_parameter("debug_image_topic", "/amr1/aruco/debug_image")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_hz", 12.0)

        self.amr_id = str(self.get_parameter("amr_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_period = 1.0 / max(1.0, float(self.get_parameter("publish_hz").value))

        dictionary_name = str(self.get_parameter("dictionary").value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        params = cv2.aruco.DetectorParameters()
        # Favor small marker detection in the 640x360 Isaac camera stream.
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 33
        params.adaptiveThreshWinSizeStep = 10
        params.minMarkerPerimeterRate = 0.02
        params.maxMarkerPerimeterRate = 4.0
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        try:
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            self.legacy_dictionary = None
            self.legacy_params = None
        except AttributeError:
            self.detector = None
            self.legacy_dictionary = dictionary
            self.legacy_params = params

        self.last_publish = -1.0
        self.pub = self.create_publisher(String, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"[{self.amr_id}] paired ArUco ready: RGB={self.image_topic} OUT={self.result_topic} "
            f"DEBUG={self.debug_image_topic} pairs=김서울10/11,박인천20/21,서수원30/31"
        )

    def _detect(self, gray: np.ndarray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.legacy_dictionary, parameters=self.legacy_params)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.last_publish > 0.0 and now - self.last_publish < self.publish_period:
            return
        self.last_publish = now
        try:
            frame = _image_to_bgr(msg)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detect(gray)
        except Exception as exc:
            self.get_logger().warning(f"ArUco image processing failed: {exc}")
            return

        marker_data: dict[int, dict[str, Any]] = {}
        visible_ids: list[int] = []
        draw_markers: list[tuple[int, np.ndarray]] = []
        if ids is not None:
            for marker_corners, raw_id in zip(corners, ids.flatten()):
                marker_id = int(raw_id)
                pts = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
                center = pts.mean(axis=0)
                marker_data[marker_id] = {
                    "id": marker_id,
                    "center_x": float(center[0]),
                    "center_y": float(center[1]),
                    "side_px": _marker_side_px(pts),
                }
                visible_ids.append(marker_id)
                draw_markers.append((marker_id, pts.copy()))

        h, w = frame.shape[:2]
        pairs: dict[str, dict[str, Any]] = {}
        complete_patients: list[str] = []
        for patient, (left_id, right_id) in PAIR_MAP.items():
            left = marker_data.get(left_id)
            right = marker_data.get(right_id)
            if left is None or right is None:
                continue
            complete_patients.append(patient)
            lx, ly = float(left["center_x"]), float(left["center_y"])
            rx, ry = float(right["center_x"]), float(right["center_y"])
            center_x = 0.5 * (lx + rx)
            center_y = 0.5 * (ly + ry)
            left_size = float(left["side_px"])
            right_size = float(right["side_px"])
            size_mean = max(1.0, 0.5 * (left_size + right_size))
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
                "left_side_px": left_size,
                "right_side_px": right_size,
                "size_error_ratio": float((right_size - left_size) / size_mean),
                "line_angle_deg": float(math.degrees(math.atan2(ry - ly, rx - lx))),
            }

        # Presentation/debug view: draw detected marker boxes, pair center, camera center
        # and current horizontal alignment error on top of the real AMR camera image.
        debug = frame.copy()
        camera_cx = int(round(w * 0.5))
        cv2.line(debug, (camera_cx, 0), (camera_cx, h - 1), (255, 255, 0), 2)
        cv2.putText(debug, "CAM CENTER", (max(4, camera_cx - 62), 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

        complete_ids = {mid for patient in complete_patients for mid in PAIR_MAP[patient]}
        for marker_id, pts in draw_markers:
            poly = np.round(pts).astype(np.int32).reshape((-1, 1, 2))
            color = (0, 220, 0) if marker_id in complete_ids else (0, 180, 255)
            cv2.polylines(debug, [poly], True, color, 2, cv2.LINE_AA)
            cx, cy = np.round(pts.mean(axis=0)).astype(int)
            cv2.circle(debug, (int(cx), int(cy)), 4, color, -1)
            cv2.putText(debug, f"ID {marker_id}", (int(cx) - 24, max(15, int(cy) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        text_y = 42
        for patient, pair in pairs.items():
            lx, ly = int(round(pair["left_center_x"])), int(round(pair["left_center_y"]))
            rx, ry = int(round(pair["right_center_x"])), int(round(pair["right_center_y"]))
            pcx, pcy = int(round(pair["pair_center_x"])), int(round(pair["pair_center_y"]))
            err = float(pair["center_error_px"])
            centered = abs(err) <= 12.0
            pair_color = (0, 255, 0) if centered else (0, 100, 255)
            cv2.line(debug, (lx, ly), (rx, ry), pair_color, 2, cv2.LINE_AA)
            cv2.circle(debug, (pcx, pcy), 7, pair_color, 2)
            cv2.line(debug, (pcx, max(0, pcy - 18)), (pcx, min(h - 1, pcy + 18)), pair_color, 2)
            state = "CENTERED" if centered else ("PAIR RIGHT" if err > 0 else "PAIR LEFT")
            cv2.putText(debug,
                        f"PAIR {pair['left_id']}/{pair['right_id']} err={err:+.1f}px {state}",
                        (8, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, pair_color, 2, cv2.LINE_AA)
            text_y += 24

        cv2.putText(debug, f"visible={sorted(visible_ids)}", (8, h - 12),
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
                "visible_ids": sorted(visible_ids),
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
