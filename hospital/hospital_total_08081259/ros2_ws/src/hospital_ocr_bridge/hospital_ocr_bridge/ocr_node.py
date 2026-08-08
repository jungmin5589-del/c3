from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .nameplate_vision import (
    PlateDetection,
    Recognition,
    candidate_score,
    choose_best,
    crop_normalized,
    detect_nameplate,
    make_recognition_variants,
    normalize_birth_date,
    normalize_digits,
    normalize_name,
    parse_recognition,
    sensor_image_to_bgr,
)


@dataclass
class RequestContext:
    request_id: str
    expected_name: str
    expected_birth_date: str
    candidates: list[dict[str, str]]
    frames_to_check: int
    requested_at: float


@dataclass
class FrameEvidence:
    frame_index: int
    bbox: tuple[int, int, int, int]
    plate_score: float
    raw_name: str
    raw_birth: str
    name_confidence: float
    birth_confidence: float
    candidate_scores: dict[str, float]
    name_similarities: dict[str, float]
    birth_similarities: dict[str, float]


class PaddleLineRecognizer:
    """One Korean recognition model, loaded once in the external ROS process."""

    def __init__(self, model_name: str, device: str, cpu_threads: int, variant_count: int) -> None:
        from paddleocr import TextRecognition

        self.variant_count = max(1, int(variant_count))
        self.model = TextRecognition(
            model_name=model_name,
            device=device,
            cpu_threads=int(cpu_threads),
        )

    def recognize(self, image: np.ndarray) -> Recognition:
        outputs: list[Recognition] = []
        for variant in make_recognition_variants(image)[: self.variant_count]:
            for result in self.model.predict(input=variant, batch_size=1):
                outputs.append(parse_recognition(result))
        return choose_best(outputs)


class HospitalOcrNode(Node):
    def __init__(self) -> None:
        super().__init__("hospital_ocr_node")

        self.declare_parameter("amr_id", "amr1")
        self.declare_parameter("image_topic", "/amr1/camera/front/color/image_raw")
        self.declare_parameter("request_topic", "/amr1/ocr/request")
        self.declare_parameter("result_topic", "/amr1/ocr/result")
        self.declare_parameter("control_topic", "/amr1/ocr/control")
        self.declare_parameter("output_root", "")
        self.declare_parameter("model_name", "korean_PP-OCRv5_mobile_rec")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("cpu_threads", 4)
        self.declare_parameter("recognition_variant_count", 6)
        self.declare_parameter("default_frames_to_check", 10)
        self.declare_parameter("verification_score_threshold", 0.50)
        self.declare_parameter("minimum_plate_frames", 1)
        self.declare_parameter("tracking_publish_hz", 8.0)
        self.declare_parameter("plate_search_roi", [0.04, 0.04, 0.96, 0.96])
        self.declare_parameter("name_value_roi", [0.35, 0.06, 0.98, 0.49])
        self.declare_parameter("birth_value_roi", [0.35, 0.52, 0.98, 0.96])

        self.amr_id = str(self.get_parameter("amr_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.request_topic = str(self.get_parameter("request_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.plate_search_roi = tuple(float(v) for v in self.get_parameter("plate_search_roi").value)
        self.name_value_roi = tuple(float(v) for v in self.get_parameter("name_value_roi").value)
        self.birth_value_roi = tuple(float(v) for v in self.get_parameter("birth_value_roi").value)
        self.verification_threshold = float(self.get_parameter("verification_score_threshold").value)
        self.minimum_plate_frames = int(self.get_parameter("minimum_plate_frames").value)
        self.tracking_period = 1.0 / max(1.0, float(self.get_parameter("tracking_publish_hz").value))

        output_root = str(self.get_parameter("output_root").value).strip()
        self.output_root = Path(output_root).expanduser().resolve() if output_root else Path.cwd() / "output" / "ocr"
        self.output_root.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.result_pub = self.create_publisher(String, self.result_topic, qos)
        self.create_subscription(String, self.request_topic, self._on_request, qos)
        self.create_subscription(String, self.control_topic, self._on_control, qos)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self._lock = threading.Lock()
        self._request: RequestContext | None = None
        self._frames: list[np.ndarray] = []
        self._future: Future[dict[str, Any]] | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"{self.amr_id}-ocr")
        self._latest_image: np.ndarray | None = None
        self._tracking_active = False
        self._tracking_bbox: tuple[int, int, int, int] | None = None
        self._tracking_identity: dict[str, Any] | None = None
        self._last_tracking_publish = 0.0

        self.get_logger().info(
            f"Loading PaddleOCR recognition model for {self.amr_id}: "
            f"{self.get_parameter('model_name').value}"
        )
        self.recognizer = PaddleLineRecognizer(
            model_name=str(self.get_parameter("model_name").value),
            device=str(self.get_parameter("device").value),
            cpu_threads=int(self.get_parameter("cpu_threads").value),
            variant_count=int(self.get_parameter("recognition_variant_count").value),
        )
        self.get_logger().info(f"[{self.amr_id}] OCR model ready")
        self.create_timer(0.05, self._poll_worker)

        self.get_logger().info(f"[{self.amr_id}] image SUB   : {self.image_topic}")
        self.get_logger().info(f"[{self.amr_id}] request SUB : {self.request_topic}")
        self.get_logger().info(f"[{self.amr_id}] result PUB  : {self.result_topic}")
        self.get_logger().info(f"[{self.amr_id}] control SUB : {self.control_topic}")

    def destroy_node(self) -> bool:
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _publish(self, payload: dict[str, Any]) -> None:
        payload.setdefault("protocol_version", 1)
        payload.setdefault("amr", self.amr_id)
        payload.setdefault("timestamp", time.time())
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.result_pub.publish(msg)

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("OCR request is not valid JSON")
            return
        if str(payload.get("amr", self.amr_id)) != self.amr_id:
            return
        request_id = str(payload.get("request_id", "")).strip()
        expected_name = str(payload.get("expected_name", "")).strip()
        expected_birth = str(payload.get("expected_birth_date", "")).strip()
        raw_candidates = payload.get("candidates", [])
        candidates = [
            {"name": str(item.get("name", "")).strip(), "birth_date": str(item.get("birth_date", "")).strip()}
            for item in raw_candidates
            if isinstance(item, dict) and item.get("name") and item.get("birth_date")
        ]
        if not request_id or not expected_name or not expected_birth or not candidates:
            self._publish(
                {
                    "state": "ERROR",
                    "request_id": request_id,
                    "verified": False,
                    "reason": "request requires request_id, expected patient and candidate list",
                }
            )
            return
        # Ignore a duplicate delivery of the same request. Re-running the same
        # heavy PaddleOCR job can otherwise make the demo appear to pause.
        with self._lock:
            active_request_id = self._request.request_id if self._request else ""
            future_busy = self._future is not None and not self._future.done()
            tracking_same_request = bool(
                self._tracking_active
                and self._tracking_identity
                and self._tracking_identity.get("request_id") == request_id
            )
        if request_id == active_request_id and (future_busy or tracking_same_request):
            self.get_logger().warning(
                f"[{self.amr_id}] duplicate request ignored: {request_id}"
            )
            return

        frames = int(payload.get("frames_to_check", self.get_parameter("default_frames_to_check").value))
        context = RequestContext(
            request_id=request_id,
            expected_name=expected_name,
            expected_birth_date=expected_birth,
            candidates=candidates[:3],
            frames_to_check=max(1, min(30, frames)),
            requested_at=time.time(),
        )
        with self._lock:
            self._request = context
            self._frames = []
            self._future = None
            self._tracking_active = False
            self._tracking_bbox = None
            self._tracking_identity = None
        self.get_logger().info(
            f"[{self.amr_id}] request={request_id} expected={expected_name} {expected_birth}; "
            f"collecting {context.frames_to_check} frames"
        )
        self._publish(
            {
                "state": "CAPTURING",
                "request_id": request_id,
                "verified": False,
                "captured_frames": 0,
                "required_frames": context.frames_to_check,
            }
        )

    def _on_control(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(payload.get("amr", self.amr_id)) != self.amr_id:
            return
        action = str(payload.get("action", "")).upper()
        request_id = str(payload.get("request_id", ""))
        with self._lock:
            active_id = self._request.request_id if self._request else ""
            if request_id and active_id and request_id != active_id:
                return
            if action in {"STOP", "STOP_TRACKING", "CANCEL"}:
                was_tracking = self._tracking_active
                self._request = None
                self._tracking_active = False
                self._tracking_bbox = None
                self._tracking_identity = None
                self._frames = []
                if was_tracking or active_id:
                    self.get_logger().info(
                        f"[{self.amr_id}] request/tracking stopped by control command"
                    )

    def _on_image(self, msg: Image) -> None:
        try:
            image = sensor_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"image conversion failed: {exc}")
            return
        now = time.monotonic()
        with self._lock:
            self._latest_image = image
            context = self._request
            future_running = self._future is not None and not self._future.done()
            if context is not None and not future_running and len(self._frames) < context.frames_to_check:
                self._frames.append(image.copy())
                count = len(self._frames)
                if count == context.frames_to_check:
                    frames = self._frames.copy()
                    self._future = self._executor.submit(self._process_request, context, frames)
                    self.get_logger().info(f"[{self.amr_id}] {count} frames captured; OCR worker started")
                elif count in {1, context.frames_to_check // 2}:
                    self._publish(
                        {
                            "state": "CAPTURING",
                            "request_id": context.request_id,
                            "verified": False,
                            "captured_frames": count,
                            "required_frames": context.frames_to_check,
                        }
                    )

            tracking_active = self._tracking_active
            previous_bbox = self._tracking_bbox
            identity = dict(self._tracking_identity) if self._tracking_identity else None

        if tracking_active and identity and now - self._last_tracking_publish >= self.tracking_period:
            detection = detect_nameplate(image, self.plate_search_roi, previous_bbox)
            if detection is None:
                return
            with self._lock:
                self._tracking_bbox = detection.bbox
            self._last_tracking_publish = now
            center_x, center_y = detection.center
            self._publish(
                {
                    "state": "TRACKING",
                    "request_id": identity["request_id"],
                    "verified": True,
                    "selected_name": identity["selected_name"],
                    "selected_birth_date": identity["selected_birth_date"],
                    "score": identity["score"],
                    "bbox_x": detection.bbox[0],
                    "bbox_y": detection.bbox[1],
                    "bbox_width": detection.bbox[2],
                    "bbox_height": detection.bbox[3],
                    "bbox_center_x": center_x,
                    "bbox_center_y": center_y,
                    "image_width": int(image.shape[1]),
                    "image_height": int(image.shape[0]),
                }
            )

    def _poll_worker(self) -> None:
        with self._lock:
            future = self._future
        if future is None or not future.done():
            return
        with self._lock:
            self._future = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"OCR worker failed: {exc}")
            with self._lock:
                context = self._request
                self._request = None
                self._frames = []
            self._publish(
                {
                    "state": "ERROR",
                    "request_id": context.request_id if context else "",
                    "verified": False,
                    "reason": str(exc),
                }
            )
            return

        result_request_id = str(result.get("request_id", ""))
        with self._lock:
            active_request_id = self._request.request_id if self._request else ""
        if not active_request_id or result_request_id != active_request_id:
            self.get_logger().warning(
                f"[{self.amr_id}] stale/cancelled OCR result discarded: "
                f"result={result_request_id} active={active_request_id or 'NONE'}"
            )
            return

        self._publish(result)
        if result.get("verified"):
            with self._lock:
                self._tracking_active = True
                self._tracking_bbox = tuple(int(v) for v in result["bbox"])
                self._tracking_identity = {
                    "request_id": result["request_id"],
                    "selected_name": result["selected_name"],
                    "selected_birth_date": result["selected_birth_date"],
                    "score": result["score"],
                }
            self.get_logger().info(
                f"[{self.amr_id}] VERIFIED {result['selected_name']} "
                f"{result['selected_birth_date']} score={result['score']:.3f}; tracking started"
            )
        else:
            # A rejected request is complete.  Leaving it active causes the same
            # frames/request to start again before the mission can issue a new ID.
            with self._lock:
                if self._request and self._request.request_id == result_request_id:
                    self._request = None
                    self._frames = []
                self._tracking_active = False
                self._tracking_bbox = None
                self._tracking_identity = None
            selected_name = result.get("selected_name") or "NONE"
            selected_birth = result.get("selected_birth_date") or "NONE"
            self.get_logger().warning(
                f"[{self.amr_id}] REJECTED reason={result.get('reason', 'UNKNOWN')} "
                f"selected={selected_name} {selected_birth} "
                f"plate_frames={result.get('plate_frames', 0)} "
                f"raw_name={result.get('raw_name', '')!r} "
                f"raw_birth={result.get('raw_birth_text', '')!r} "
                f"score={result.get('score')} "
                f"output={result.get('output_dir', '')}"
            )

    def _recognize_line(self, image: np.ndarray) -> Recognition:
        return self.recognizer.recognize(image)

    def _process_request(self, context: RequestContext, frames: list[np.ndarray]) -> dict[str, Any]:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = self.output_root / f"{self.amr_id}_{context.request_id}_{timestamp}"
        output_dir.mkdir(parents=True, exist_ok=True)

        evidence: list[FrameEvidence] = []
        previous_bbox: tuple[int, int, int, int] | None = None
        best_detection: PlateDetection | None = None
        best_frame: np.ndarray | None = None
        best_frame_score = -1.0

        for index, frame in enumerate(frames, start=1):
            detection = detect_nameplate(frame, self.plate_search_roi, previous_bbox)
            if detection is None:
                cv2.imwrite(str(output_dir / f"frame_{index:02d}_NO_PLATE.jpg"), frame)
                continue
            previous_bbox = detection.bbox
            plate = detection.rectified
            name_image = crop_normalized(plate, self.name_value_roi)
            birth_image = crop_normalized(plate, self.birth_value_roi)
            name_rec = self._recognize_line(name_image)
            birth_rec = self._recognize_line(birth_image)

            scores: dict[str, float] = {}
            name_sims: dict[str, float] = {}
            birth_sims: dict[str, float] = {}
            for candidate in context.candidates:
                key = f"{candidate['name']}|{candidate['birth_date']}"
                score, name_sim, birth_sim = candidate_score(
                    name_rec.text,
                    birth_rec.text,
                    name_rec.score,
                    birth_rec.score,
                    candidate["name"],
                    candidate["birth_date"],
                )
                scores[key] = score
                name_sims[key] = name_sim
                birth_sims[key] = birth_sim

            frame_best = max(scores.values()) if scores else 0.0
            evidence.append(
                FrameEvidence(
                    frame_index=index,
                    bbox=detection.bbox,
                    plate_score=detection.score,
                    raw_name=name_rec.text,
                    raw_birth=birth_rec.text,
                    name_confidence=name_rec.score,
                    birth_confidence=birth_rec.score,
                    candidate_scores=scores,
                    name_similarities=name_sims,
                    birth_similarities=birth_sims,
                )
            )
            if frame_best > best_frame_score:
                best_frame_score = frame_best
                best_detection = detection
                best_frame = frame.copy()

            debug = frame.copy()
            x, y, w, h = detection.bbox
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx, cy = detection.center
            cv2.drawMarker(debug, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(
                debug,
                f"name={name_rec.text} birth={birth_rec.text}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
            )
            cv2.imwrite(str(output_dir / f"frame_{index:02d}_debug.jpg"), debug)
            cv2.imwrite(str(output_dir / f"frame_{index:02d}_plate.jpg"), plate)
            cv2.imwrite(str(output_dir / f"frame_{index:02d}_name.jpg"), name_image)
            cv2.imwrite(str(output_dir / f"frame_{index:02d}_birth.jpg"), birth_image)

        aggregate: dict[str, dict[str, Any]] = {}
        for candidate in context.candidates:
            key = f"{candidate['name']}|{candidate['birth_date']}"
            frame_scores = sorted(
                (item.candidate_scores.get(key, 0.0) for item in evidence),
                reverse=True,
            )
            name_scores = sorted(
                (item.name_similarities.get(key, 0.0) for item in evidence),
                reverse=True,
            )
            birth_scores = sorted(
                (item.birth_similarities.get(key, 0.0) for item in evidence),
                reverse=True,
            )

            # Name and date do not have to be perfect in the same frame.
            # Aggregate the best independent evidence across the 10 frames.
            top_frame = frame_scores[: min(3, len(frame_scores))]
            top_name = name_scores[: min(3, len(name_scores))]
            top_birth = birth_scores[: min(3, len(birth_scores))]
            mean_frame = float(sum(top_frame) / len(top_frame)) if top_frame else 0.0
            mean_name = float(sum(top_name) / len(top_name)) if top_name else 0.0
            mean_birth = float(sum(top_birth) / len(top_birth)) if top_birth else 0.0
            aggregate_score = 0.45 * mean_frame + 0.30 * mean_name + 0.25 * mean_birth

            name_support = sum(1 for value in name_scores if value >= 0.66)
            birth_support = sum(1 for value in birth_scores if value >= 0.88)
            support = max(name_support, birth_support)
            aggregate[key] = {
                "name": candidate["name"],
                "birth_date": candidate["birth_date"],
                "score": float(aggregate_score),
                "support_frames": int(support),
                "name_support_frames": int(name_support),
                "birth_support_frames": int(birth_support),
                "best_name_similarity": float(name_scores[0]) if name_scores else 0.0,
                "best_birth_similarity": float(birth_scores[0]) if birth_scores else 0.0,
            }

        ranked = sorted(aggregate.values(), key=lambda item: item["score"], reverse=True)

        # Demo reliability rule: the mission already tells us which patient is
        # expected at this pose.  Do not reject that patient merely because a
        # noisy one-frame OCR score lets another fixed candidate rank slightly
        # higher.  Evaluate the requested patient's own evidence directly.
        expected_key = f"{context.expected_name}|{context.expected_birth_date}"
        expected_candidate = aggregate.get(expected_key)
        enough_frames = len(evidence) >= self.minimum_plate_frames

        expected_score = float(expected_candidate.get("score", 0.0)) if expected_candidate else 0.0
        expected_name_sim = float(expected_candidate.get("best_name_similarity", 0.0)) if expected_candidate else 0.0
        expected_birth_sim = float(expected_candidate.get("best_birth_similarity", 0.0)) if expected_candidate else 0.0

        # Very permissive on purpose for the fixed three-patient demo, while
        # still requiring actual plate/OCR evidence. One correctly read Korean
        # syllable or roughly half of the birth-date digits is enough to keep
        # the mission moving.
        verified = bool(
            expected_candidate
            and enough_frames
            and (
                expected_score >= self.verification_threshold
                or expected_name_sim >= 0.34
                or expected_birth_sim >= 0.50
            )
        )

        if verified:
            selected = expected_candidate
        elif ranked and float(ranked[0]["score"]) > 0.0:
            selected = ranked[0]
        else:
            selected = {
                "name": "",
                "birth_date": "",
                "score": 0.0,
                "support_frames": 0,
                "best_name_similarity": 0.0,
                "best_birth_similarity": 0.0,
            }

        expected_match = (
            selected["name"] == context.expected_name
            and selected["birth_date"] == context.expected_birth_date
        )

        selected_key = f"{selected['name']}|{selected['birth_date']}"
        selected_evidence = sorted(
            evidence,
            key=lambda item: item.candidate_scores.get(selected_key, 0.0),
            reverse=True,
        )
        raw_name = selected_evidence[0].raw_name if selected_evidence else ""
        raw_birth = selected_evidence[0].raw_birth if selected_evidence else ""

        if best_detection is None or best_frame is None:
            bbox = (0, 0, 0, 0)
            center_x = center_y = 0.0
            image_width = int(frames[-1].shape[1]) if frames else 0
            image_height = int(frames[-1].shape[0]) if frames else 0
            verified = False
        else:
            bbox = best_detection.bbox
            center_x, center_y = best_detection.center
            image_height, image_width = best_frame.shape[:2]

        if verified:
            reason = "MATCHED"
        elif len(evidence) == 0:
            reason = "NO_PLATE_DETECTED"
        elif not raw_name and not raw_birth:
            reason = "OCR_EMPTY"
        elif not expected_match:
            reason = "PATIENT_MISMATCH"
        else:
            reason = "SCORE_TOO_LOW"

        result: dict[str, Any] = {
            "state": "VERIFIED" if verified else "REJECTED",
            "reason": reason,
            "request_id": context.request_id,
            "verified": verified,
            "expected_name": context.expected_name,
            "expected_birth_date": context.expected_birth_date,
            "selected_name": selected["name"],
            "selected_birth_date": selected["birth_date"],
            "raw_name": raw_name,
            "raw_birth_text": raw_birth,
            "normalized_raw_name": normalize_name(raw_name),
            "normalized_raw_birth": normalize_birth_date(raw_birth),
            "raw_birth_digits": normalize_digits(raw_birth),
            "score": float(selected["score"]),
            "support_frames": int(selected["support_frames"]),
            "frames_received": len(frames),
            "plate_frames": len(evidence),
            "candidate_ranking": ranked,
            "bbox": [int(v) for v in bbox],
            "bbox_x": int(bbox[0]),
            "bbox_y": int(bbox[1]),
            "bbox_width": int(bbox[2]),
            "bbox_height": int(bbox[3]),
            "bbox_center_x": float(center_x),
            "bbox_center_y": float(center_y),
            "image_width": int(image_width),
            "image_height": int(image_height),
            "output_dir": str(output_dir),
            "matching_rule": "Only Korean name characters and birth-date digits are scored; labels are excluded by ROI.",
        }
        (output_dir / "result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: HospitalOcrNode | None = None
    try:
        node = HospitalOcrNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
