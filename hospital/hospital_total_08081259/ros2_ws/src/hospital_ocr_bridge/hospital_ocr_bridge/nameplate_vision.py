from __future__ import annotations

from dataclasses import dataclass
from difflib import SequenceMatcher
import ast
import math
import re
from typing import Any, Iterable, Mapping, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class Recognition:
    text: str
    score: float


@dataclass
class PlateDetection:
    bbox: tuple[int, int, int, int]
    corners: np.ndarray
    rectified: np.ndarray
    score: float

    @property
    def center(self) -> tuple[float, float]:
        x, y, w, h = self.bbox
        return x + w * 0.5, y + h * 0.5


def normalize_name(text: str) -> str:
    return re.sub(r"[^가-힣]", "", text or "")


def normalize_digits(text: str) -> str:
    normalized = (
        (text or "")
        .strip()
        .replace(" ", "")
        .replace(".", "-")
        .replace("/", "-")
        .replace("_", "-")
        .replace("—", "-")
        .replace("–", "-")
        .replace("O", "0")
        .replace("o", "0")
        .replace("I", "1")
        .replace("l", "1")
        .replace("|", "1")
    )
    return re.sub(r"[^0-9]", "", normalized)


def normalize_birth_date(text: str) -> str:
    digits = normalize_digits(text)
    if len(digits) != 8:
        return ""
    try:
        year, month, day = int(digits[:4]), int(digits[4:6]), int(digits[6:8])
    except ValueError:
        return ""
    if not (1900 <= year <= 2099 and 1 <= month <= 12 and 1 <= day <= 31):
        return ""
    return f"{year:04d}-{month:02d}-{day:02d}"


def string_similarity(observed: str, expected: str) -> float:
    if not observed or not expected:
        return 0.0
    return SequenceMatcher(None, observed, expected).ratio()


def candidate_score(
    observed_name: str,
    observed_birth_text: str,
    name_confidence: float,
    birth_confidence: float,
    candidate_name: str,
    candidate_birth_date: str,
) -> tuple[float, float, float]:
    observed_name_norm = normalize_name(observed_name)
    candidate_name_norm = normalize_name(candidate_name)
    observed_birth_digits = normalize_digits(observed_birth_text)
    candidate_birth_digits = normalize_digits(candidate_birth_date)

    name_similarity = string_similarity(observed_name_norm, candidate_name_norm)
    birth_similarity = string_similarity(observed_birth_digits, candidate_birth_digits)

    # Exact 8-digit birth date is the strongest discriminator between the
    # three fixed patients.  A nearly correct Korean name can still pass when
    # PaddleOCR splits a syllable, but a wrong date never gets promoted.
    birth_exact = float(
        len(observed_birth_digits) == 8
        and observed_birth_digits == candidate_birth_digits
    )
    name_exact = float(observed_name_norm == candidate_name_norm and bool(candidate_name_norm))
    confidence = 0.5 * max(0.0, name_confidence) + 0.5 * max(0.0, birth_confidence)
    total = (
        0.46 * name_similarity
        + 0.30 * birth_similarity
        + 0.12 * birth_exact
        + 0.06 * name_exact
        + 0.06 * confidence
    )
    return float(total), float(name_similarity), float(birth_similarity)


def _order_quad(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    ordered[0] = pts[np.argmin(sums)]  # top-left
    ordered[2] = pts[np.argmax(sums)]  # bottom-right
    ordered[1] = pts[np.argmin(diffs)]  # top-right
    ordered[3] = pts[np.argmax(diffs)]  # bottom-left
    return ordered


def _warp_plate(image: np.ndarray, corners: np.ndarray, output_size: tuple[int, int] = (900, 624)) -> np.ndarray:
    src = _order_quad(corners)
    width, height = output_size
    dst = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_CUBIC)


def _bbox_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def detect_nameplate(
    image: np.ndarray,
    search_roi: Sequence[float] = (0.04, 0.04, 0.96, 0.96),
    previous_bbox: tuple[int, int, int, int] | None = None,
) -> PlateDetection | None:
    """Detect the rectangular patient nameplate and return a rectified crop.

    The actual Isaac Sim camera renders the white plate rather dark.  A
    Canny-only detector can therefore miss the outer rectangle.  This version
    combines ordinary edges with an adaptive-threshold rectangle detector.
    """
    if image is None or image.size == 0:
        return None

    height, width = image.shape[:2]
    rx1 = int(clamp01(float(search_roi[0])) * width)
    ry1 = int(clamp01(float(search_roi[1])) * height)
    rx2 = int(clamp01(float(search_roi[2])) * width)
    ry2 = int(clamp01(float(search_roi[3])) * height)
    if rx2 <= rx1 or ry2 <= ry1:
        return None

    crop = image[ry1:ry2, rx1:rx2]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    image_area = float(width * height)
    target_aspect = 1.60
    candidates: list[PlateDetection] = []

    def add_contours(mask: np.ndarray, source_bonus: float) -> None:
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area < image_area * 0.002 or contour_area > image_area * 0.55:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            rect = cv2.minAreaRect(contour)
            (_, _), (rw, rh), _ = rect
            if min(rw, rh) < 24:
                continue

            aspect = max(rw, rh) / max(1.0, min(rw, rh))
            if not (1.20 <= aspect <= 2.10):
                continue

            rect_area = float(rw * rh)
            rectangularity = contour_area / max(1.0, rect_area)
            if rectangularity < 0.48:
                continue

            if len(approx) == 4 and cv2.isContourConvex(approx):
                local_corners = approx.reshape(4, 2).astype(np.float32)
                quad_quality = 1.0
            else:
                local_corners = cv2.boxPoints(rect).astype(np.float32)
                quad_quality = max(0.0, 1.0 - min(1.0, abs(len(approx) - 4) / 8.0))

            global_corners = local_corners + np.array([rx1, ry1], dtype=np.float32)
            x, y, bw, bh = cv2.boundingRect(global_corners.astype(np.float32))
            x = max(0, x)
            y = max(0, y)
            bw = min(width - x, bw)
            bh = min(height - y, bh)
            if bw < 70 or bh < 40:
                continue

            rectified = _warp_plate(image, global_corners)
            rect_gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
            mean_brightness = float(np.mean(rect_gray)) / 255.0
            contrast = float(np.std(rect_gray)) / 64.0
            dark_ratio = float(np.mean(rect_gray < max(25.0, np.mean(rect_gray) - 12.0)))
            if mean_brightness < 0.18 or contrast < 0.07 or dark_ratio < 0.015:
                continue

            aspect_quality = math.exp(-2.0 * abs(math.log(aspect / target_aspect)))
            area_quality = min(1.0, rect_area / max(1.0, image_area * 0.075))
            prior_quality = _bbox_iou((x, y, bw, bh), previous_bbox) if previous_bbox else 0.0
            center_distance = math.hypot(
                (x + bw * 0.5) - width * 0.5,
                (y + bh * 0.5) - height * 0.38,
            )
            center_quality = 1.0 - min(
                1.0,
                center_distance / max(1.0, math.hypot(width, height) * 0.70),
            )
            score = (
                0.24 * aspect_quality
                + 0.28 * area_quality
                + 0.22 * min(1.0, rectangularity)
                + 0.12 * quad_quality
                + 0.07 * center_quality
                + 0.05 * prior_quality
                + source_bonus
            )
            candidates.append(
                PlateDetection(
                    bbox=(int(x), int(y), int(bw), int(bh)),
                    corners=global_corners.astype(np.float32),
                    rectified=rectified,
                    score=float(score),
                )
            )

    edge = cv2.Canny(blurred, 30, 105)
    edge = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=2)
    edge = cv2.dilate(edge, np.ones((3, 3), np.uint8), iterations=1)
    add_contours(edge, source_bonus=0.00)

    # Do not blur this branch: the thin table border disappears after blur in
    # the real 640x360 Isaac image.
    adaptive = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31,
        5,
    )
    adaptive = cv2.morphologyEx(
        adaptive,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), np.uint8),
        iterations=1,
    )
    add_contours(adaptive, source_bonus=0.08)

    if not candidates:
        return None
    return max(candidates, key=lambda item: item.score)

def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def crop_normalized(image: np.ndarray, roi: Sequence[float]) -> np.ndarray:
    h, w = image.shape[:2]
    x1 = int(clamp01(float(roi[0])) * w)
    y1 = int(clamp01(float(roi[1])) * h)
    x2 = int(clamp01(float(roi[2])) * w)
    y2 = int(clamp01(float(roi[3])) * h)
    return image[y1:y2, x1:x2].copy()


def make_recognition_variants(image: np.ndarray) -> list[np.ndarray]:
    """Generate OCR inputs robust to small, dim Isaac Sim text."""
    if image is None or image.size == 0:
        return []

    # Keep character height large enough for the mobile Korean recognizer.
    scale = max(2.0, 180.0 / max(1, image.shape[0]))
    enlarged = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(enlarged, cv2.COLOR_BGR2GRAY)
    gray = cv2.fastNlMeansDenoising(gray, None, 7, 7, 21)
    clahe = cv2.createCLAHE(clipLimit=2.8, tileGridSize=(8, 8)).apply(gray)
    sharpen = cv2.addWeighted(clahe, 1.7, cv2.GaussianBlur(clahe, (0, 0), 1.2), -0.7, 0)
    otsu = cv2.threshold(sharpen, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    adaptive = cv2.adaptiveThreshold(
        sharpen, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 7
    )
    # Add a small white margin so edge characters are not clipped.
    def bordered(src: np.ndarray) -> np.ndarray:
        if src.ndim == 2:
            src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        return cv2.copyMakeBorder(src, 18, 18, 24, 24, cv2.BORDER_CONSTANT, value=(255, 255, 255))

    return [
        bordered(enlarged),
        bordered(clahe),
        bordered(sharpen),
        bordered(otsu),
        bordered(adaptive),
        bordered(cv2.bitwise_not(otsu)),
    ]


def _mapping_from_result(result: Any) -> Mapping[str, Any] | None:
    if isinstance(result, Mapping):
        return result
    for attr_name in ("json", "res", "to_dict"):
        if not hasattr(result, attr_name):
            continue
        value = getattr(result, attr_name)
        try:
            value = value() if callable(value) else value
        except TypeError:
            continue
        if isinstance(value, str):
            try:
                value = ast.literal_eval(value)
            except (SyntaxError, ValueError):
                continue
        if isinstance(value, Mapping):
            return value
    try:
        converted = dict(result)
        return converted if isinstance(converted, Mapping) else None
    except (TypeError, ValueError):
        return None


def parse_recognition(result: Any) -> Recognition:
    mapping = _mapping_from_result(result)
    if mapping:
        payload = mapping.get("res") if isinstance(mapping.get("res"), Mapping) else mapping
        text = payload.get("rec_text")
        score = payload.get("rec_score")
        if text is not None:
            try:
                return Recognition(str(text), float(score or 0.0))
            except (TypeError, ValueError):
                return Recognition(str(text), 0.0)
    representation = str(result)
    text_match = re.search(r"['\"]rec_text['\"]\s*:\s*['\"]([^'\"]*)", representation)
    score_match = re.search(r"['\"]rec_score['\"]\s*:\s*([0-9.]+)", representation)
    return Recognition(
        text_match.group(1) if text_match else "",
        float(score_match.group(1)) if score_match else 0.0,
    )


def choose_best(items: Iterable[Recognition]) -> Recognition:
    usable = [item for item in items if item.text]
    return max(usable, key=lambda item: item.score) if usable else Recognition("", 0.0)


def sensor_image_to_bgr(msg: Any) -> np.ndarray:
    encoding = str(msg.encoding).lower()
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "8uc3": 3,
        "8uc4": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
    packed = row[:, : int(msg.width) * channels].reshape(int(msg.height), int(msg.width), channels)
    if encoding in {"rgb8", "8uc3"}:
        return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return packed.copy()
    if encoding == "rgba8":
        return cv2.cvtColor(packed, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(packed, cv2.COLOR_BGRA2BGR)
