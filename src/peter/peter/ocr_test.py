#!/usr/bin/env python3
"""Run PaddleOCR on the AMR1 front-camera ROS 2 image stream."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Iterable, Sequence

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


DEFAULT_PADDLE_PYTHON = Path(
    '/home/peter-msi/DoosanPjt/doosan_cob2/cobot_ws/'
    '.venv_paddleocr/bin/python'
)


def restart_in_paddle_environment_if_needed() -> None:
    """Re-exec with the existing PaddleOCR venv when ros2 run uses system Python."""
    if importlib.util.find_spec('paddleocr') is not None:
        return

    configured = os.environ.get('PADDLEOCR_PYTHON')
    paddle_python = Path(configured).expanduser() if configured else DEFAULT_PADDLE_PYTHON

    if os.environ.get('PETER_OCR_REEXEC') == '1':
        raise RuntimeError(
            'PaddleOCR is still unavailable after switching Python environments.'
        )
    if not paddle_python.is_file():
        raise RuntimeError(
            'PaddleOCR is not installed in the current Python environment, and '
            f'the configured interpreter does not exist: {paddle_python}. '
            'Set PADDLEOCR_PYTHON to a Python executable containing paddleocr.'
        )

    environment = os.environ.copy()
    environment['PETER_OCR_REEXEC'] = '1'
    os.execve(
        str(paddle_python),
        [str(paddle_python), str(Path(__file__).resolve()), *sys.argv[1:]],
        environment,
    )


def result_as_dict(result: Any) -> dict[str, Any]:
    """Convert a PaddleOCR 3.x result object to a plain dictionary."""
    if isinstance(result, dict):
        data = result
    else:
        data: dict[str, Any] | None = None

        json_value = getattr(result, 'json', None)
        if callable(json_value):
            json_value = json_value()
        if isinstance(json_value, str):
            try:
                decoded = json.loads(json_value)
                data = decoded if isinstance(decoded, dict) else None
            except json.JSONDecodeError:
                data = None
        elif isinstance(json_value, dict):
            data = json_value

        if data is None:
            to_dict = getattr(result, 'to_dict', None)
            if callable(to_dict):
                converted = to_dict()
                data = converted if isinstance(converted, dict) else None

        if data is None:
            try:
                converted = dict(result)
                data = converted if isinstance(converted, dict) else {}
            except (TypeError, ValueError):
                data = {}

    nested = data.get('res')
    return nested if isinstance(nested, dict) else data


def box_to_polygon(box: Sequence[float]) -> list[list[float]]:
    """Convert a PaddleOCR [x1, y1, x2, y2] box into four polygon points."""
    values = np.asarray(box, dtype=np.float64).reshape(-1)
    if values.size < 4:
        return []
    x1, y1, x2, y2 = values[:4]
    return [
        [float(x1), float(y1)],
        [float(x2), float(y1)],
        [float(x2), float(y2)],
        [float(x1), float(y2)],
    ]


def parse_ocr_results(raw_results: Iterable[Any]) -> list[dict[str, Any]]:
    """Extract text, confidence, and polygon from PaddleOCR 3.x results."""
    detections: list[dict[str, Any]] = []

    for result in raw_results:
        data = result_as_dict(result)
        texts = data.get('rec_texts')
        scores = data.get('rec_scores')
        polygons = data.get('rec_polys')
        boxes = data.get('rec_boxes')

        texts = [] if texts is None else texts
        scores = [] if scores is None else scores

        for index, raw_text in enumerate(texts):
            if polygons is not None and index < len(polygons):
                points = np.asarray(
                    polygons[index],
                    dtype=np.float64,
                ).reshape(-1, 2)
                polygon = points.tolist()
            elif boxes is not None and index < len(boxes):
                polygon = box_to_polygon(boxes[index])
            else:
                polygon = []

            confidence = float(scores[index]) if index < len(scores) else 0.0
            detections.append(
                {
                    'text': str(raw_text),
                    'confidence': confidence,
                    'polygon': polygon,
                }
            )

    return detections


class PaddleOcrCameraNode(Node):
    """Subscribe to the front camera and OCR only the newest available frame."""

    def __init__(self) -> None:
        super().__init__('paddle_ocr_camera')

        self.declare_parameter(
            'image_topic',
            '/amr1/camera/front/color/image_raw',
        )
        self.declare_parameter('text_topic', '/amr1/ocr/text')
        self.declare_parameter(
            'annotated_image_topic',
            '/amr1/ocr/image_annotated',
        )
        self.declare_parameter('language', 'korean')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('ocr_interval_sec', 1.0)
        self.declare_parameter('preprocess', 'none')
        self.declare_parameter('use_textline_orientation', False)
        self.declare_parameter('publish_annotated_image', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.text_topic = str(self.get_parameter('text_topic').value)
        self.annotated_topic = str(
            self.get_parameter('annotated_image_topic').value
        )
        self.language = str(self.get_parameter('language').value)
        self.device = str(self.get_parameter('device').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.ocr_interval = float(
            self.get_parameter('ocr_interval_sec').value
        )
        self.preprocess_mode = str(self.get_parameter('preprocess').value)
        self.use_textline_orientation = bool(
            self.get_parameter('use_textline_orientation').value
        )
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value
        )

        self._validate_parameters()

        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.latest_header = None
        self.latest_frame_number = 0
        self.last_processed_frame_number = 0

        self.get_logger().info(
            'Loading PaddleOCR model. The first run may download model files.'
        )
        self.ocr = self._create_ocr_engine()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.text_publisher = self.create_publisher(
            String,
            self.text_topic,
            10,
        )
        self.annotated_publisher = self.create_publisher(
            Image,
            self.annotated_topic,
            image_qos,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )
        self.ocr_timer = self.create_timer(
            self.ocr_interval,
            self.process_latest_frame,
        )

        self.get_logger().info(
            'PaddleOCR camera node ready: '
            f'{self.image_topic} -> {self.text_topic}, '
            f'language={self.language}, device={self.device}, '
            f'interval={self.ocr_interval:.2f}s'
        )

    def _validate_parameters(self) -> None:
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0.0 and 1.0')
        if self.ocr_interval <= 0.0:
            raise ValueError('ocr_interval_sec must be greater than zero')
        if self.preprocess_mode not in {'none', 'gray', 'clahe', 'threshold'}:
            raise ValueError(
                'preprocess must be one of: none, gray, clahe, threshold'
            )

    def _create_ocr_engine(self):
        try:
            import paddle
            from paddleocr import PaddleOCR
        except ImportError as exc:
            raise RuntimeError(
                'Unable to import PaddleOCR. Run with the PaddleOCR Python '
                'environment or set PADDLEOCR_PYTHON.'
            ) from exc

        if self.device.startswith('gpu') and not paddle.device.is_compiled_with_cuda():
            raise RuntimeError(
                'GPU was requested, but the installed PaddlePaddle build is CPU-only.'
            )

        options: dict[str, Any] = {
            'lang': self.language,
            'device': self.device,
            'use_doc_orientation_classify': False,
            'use_doc_unwarping': False,
            'use_textline_orientation': self.use_textline_orientation,
        }
        if self.language == 'korean':
            options['ocr_version'] = 'PP-OCRv5'

        return PaddleOCR(**options)

    def image_callback(self, message: Image) -> None:
        """Convert and retain only the newest camera frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        self.latest_frame = np.asarray(frame).copy()
        self.latest_header = copy.deepcopy(message.header)
        self.latest_frame_number += 1

    def preprocess_image(self, frame: np.ndarray) -> np.ndarray:
        if self.preprocess_mode == 'none':
            return frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.preprocess_mode == 'gray':
            processed = gray
        elif self.preprocess_mode == 'clahe':
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            processed = clahe.apply(gray)
        else:
            processed = cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                11,
            )
        return cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

    def accepted_detections(
        self,
        detections: Sequence[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        return [
            detection
            for detection in detections
            if float(detection['confidence']) >= self.confidence_threshold
        ]

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: Sequence[dict[str, Any]],
        elapsed: float,
    ) -> np.ndarray:
        canvas = frame.copy()

        for index, detection in enumerate(detections, start=1):
            polygon = detection['polygon']
            if len(polygon) < 3:
                continue

            points = np.rint(polygon).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(
                canvas,
                [points],
                True,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            anchor_x = max(0, int(np.min(points[:, 0, 0])))
            anchor_y = max(18, int(np.min(points[:, 0, 1])) - 5)
            label = f'{index}: {float(detection["confidence"]):.2f}'
            cv2.putText(
                canvas,
                label,
                (anchor_x, anchor_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        status = f'OCR {elapsed:.2f}s | accepted {len(detections)}'
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            status,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def process_latest_frame(self) -> None:
        """Run one OCR pass without building a queue of stale camera frames."""
        if self.latest_frame is None:
            return
        if self.latest_frame_number == self.last_processed_frame_number:
            return

        frame_number = self.latest_frame_number
        frame = self.latest_frame.copy()
        header = copy.deepcopy(self.latest_header)
        self.last_processed_frame_number = frame_number

        ocr_input = self.preprocess_image(frame)
        started = time.perf_counter()
        try:
            raw_results = self.ocr.predict(ocr_input)
            detections = parse_ocr_results(raw_results)
        except Exception as exc:
            self.get_logger().error(
                f'OCR inference failed: {type(exc).__name__}: {exc}'
            )
            return
        elapsed = time.perf_counter() - started

        accepted = self.accepted_detections(detections)
        recognized_text = '\n'.join(
            str(detection['text']) for detection in accepted
        )
        self.text_publisher.publish(String(data=recognized_text))

        if accepted:
            summary = ' | '.join(
                f'{detection["text"]} '
                f'({float(detection["confidence"]):.2f})'
                for detection in accepted
            )
            self.get_logger().info(
                f'OCR {elapsed:.2f}s: {summary}'
            )
        else:
            self.get_logger().info(
                f'OCR {elapsed:.2f}s: no text above '
                f'{self.confidence_threshold:.2f}'
            )

        if self.publish_annotated:
            annotated = self.draw_detections(frame, accepted, elapsed)
            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding='bgr8',
            )
            if header is not None:
                annotated_message.header = header
            self.annotated_publisher.publish(annotated_message)


def main(args: list[str] | None = None) -> None:
    restart_in_paddle_environment_if_needed()
    rclpy.init(args=args)
    node: PaddleOcrCameraNode | None = None
    try:
        node = PaddleOcrCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
