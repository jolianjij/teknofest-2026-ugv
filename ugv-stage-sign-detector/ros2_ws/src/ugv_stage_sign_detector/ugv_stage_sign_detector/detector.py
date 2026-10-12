"""Frame-by-frame hybrid computer-vision stage-sign detector.

Pipeline:

1. Resize and crop to a configurable region of interest.
2. Segment the red sign border in HSV colour space.
3. reject non-circular contours and implausible sizes.
4. Normalize each candidate to a square image.
5. Verify red-ring, white-centre, and black-digit structure.
6. Classify the rectified crop with ONNX or the classical fallback.

Temporal confirmation is intentionally kept in ``temporal.py``. This class
answers only "what is visible in this frame?" and never triggers vehicle motion.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

import cv2
import numpy as np

from .classification import OnnxSignClassifier, SignClassifier, UNKNOWN_LABEL
from .config import DetectorConfig
from .templates import TemplateRepository, TemplateSignClassifier, normalize_foreground


@dataclass(frozen=True, slots=True)
class BoundingBox:
    """Pixel coordinates in the original, unscaled camera frame."""
    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True, slots=True)
class SignDetection:
    """One candidate that passed colour, geometry, and digit classification."""
    sign_id: int
    confidence: float
    bounding_box: BoundingBox
    circularity: float
    red_fraction: float
    white_fraction: float
    classification_margin: float
    confirmed: bool = False


@dataclass(frozen=True, slots=True)
class SignCandidate:
    """Geometrically valid sign crop before number classification."""

    patch_bgr: np.ndarray
    digit_mask: np.ndarray
    bounding_box: BoundingBox
    circularity: float
    red_fraction: float
    white_fraction: float


class StageSignDetector:
    """Detect numbered red-ring signs in BGR images."""
    def __init__(
        self,
        config: DetectorConfig | None = None,
        templates: TemplateRepository | None = None,
        classifier: SignClassifier | None = None,
    ) -> None:
        self.config = config or DetectorConfig()
        self.config.validate()
        self.templates: TemplateRepository | None = templates
        if classifier is not None:
            self.classifier = classifier
        elif self.config.classifier.mode == "onnx":
            self.classifier = OnnxSignClassifier(
                self.config.classifier.model_path,
                self.config.classifier.labels,
                self.config.classifier.input_size,
            )
        else:
            self.templates = self.templates or TemplateRepository.generate(self.config.templates)
            self.classifier = TemplateSignClassifier(self.templates)
        self.last_red_mask: np.ndarray | None = None

    def detect(self, frame_bgr: np.ndarray) -> list[SignDetection]:
        """Detect all valid signs and return the highest-confidence item first."""
        candidates = self.extract_candidates(frame_bgr)
        detections: list[SignDetection] = []

        for candidate in candidates:
            classification = self.classifier.classify(
                candidate.patch_bgr,
                candidate.digit_mask,
            )
            if self.config.classifier.mode == "onnx":
                minimum_confidence = self.config.classifier.min_confidence
                minimum_margin = self.config.classifier.min_margin
                classification_weight = 0.88
            else:
                minimum_confidence = self.config.templates.min_match_confidence
                minimum_margin = self.config.templates.min_classification_margin
                classification_weight = 0.72

            geometry_score = float(
                np.clip(
                    0.45 * candidate.circularity
                    + 0.30 * min(1.0, candidate.red_fraction / 0.65)
                    + 0.25 * min(1.0, candidate.white_fraction / 0.75),
                    0.0,
                    1.0,
                )
            )

            if classification.label == UNKNOWN_LABEL:
                # Fail safe. This candidate passed the red-annulus, white-centre
                # and circularity checks, so a sign really is there - the model
                # just cannot say which one. Dropping it silently means driving
                # past an unread instruction at full speed, so publish it as
                # sign 0 and let the mission layer stop the vehicle.
                #
                # The classifier confidence/margin gates are deliberately NOT
                # applied here: failing them is precisely what "unknown" means.
                # Geometry is the only evidence available, so it alone decides
                # whether this is a real sign or a red barrier fragment, and it
                # becomes the confidence that temporal confirmation debounces.
                if not self.config.classifier.unknown_stops_vehicle:
                    continue
                if geometry_score < self.config.classifier.unknown_min_geometry:
                    continue
                detections.append(
                    SignDetection(
                        sign_id=UNKNOWN_LABEL,
                        confidence=geometry_score,
                        bounding_box=candidate.bounding_box,
                        circularity=candidate.circularity,
                        red_fraction=candidate.red_fraction,
                        white_fraction=candidate.white_fraction,
                        classification_margin=classification.margin,
                    )
                )
                continue

            # Any value outside the 1-12 competition labels is rejected before
            # it can reach ROS.
            if classification.label not in self.config.templates.labels:
                continue
            if classification.confidence < minimum_confidence:
                continue
            if classification.margin < minimum_margin:
                continue

            confidence = float(
                np.clip(
                    classification_weight * classification.confidence
                    + (1.0 - classification_weight) * geometry_score,
                    0.0,
                    1.0,
                )
            )
            detections.append(
                SignDetection(
                    sign_id=classification.label,
                    confidence=confidence,
                    bounding_box=candidate.bounding_box,
                    circularity=candidate.circularity,
                    red_fraction=candidate.red_fraction,
                    white_fraction=candidate.white_fraction,
                    classification_margin=classification.margin,
                )
            )

        return sorted(detections, key=lambda item: item.confidence, reverse=True)

    def extract_candidates(self, frame_bgr: np.ndarray) -> list[SignCandidate]:
        """Return validated red-ring crops without deciding their number.

        The real-data capture tool uses this method so it can save sign ``1``
        samples even when the old template classifier would call them ``2``.
        """

        if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("Expected a non-empty BGR image")

        # Process a capped width to keep Raspberry Pi latency predictable. The
        # result boxes are converted back to original camera coordinates below.
        original_height, original_width = frame_bgr.shape[:2]
        process_scale = min(1.0, self.config.process_width / original_width)
        if process_scale < 1.0:
            working = cv2.resize(
                frame_bgr,
                (self.config.process_width, int(round(original_height * process_scale))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            working = frame_bgr

        height, width = working.shape[:2]
        # Search only the configured portion of the frame. A wide Camera Module
        # 3 may need a narrower horizontal ROI after the physical mount is known.
        roi = self.config.roi
        x0 = int(round(roi.x_min * width))
        y0 = int(round(roi.y_min * height))
        x1 = int(round(roi.x_max * width))
        y1 = int(round(roi.y_max * height))
        roi_image = working[y0:y1, x0:x1]
        red_mask = self._red_mask(roi_image)
        self.last_red_mask = red_mask

        # RETR_EXTERNAL gives the outside of the red annulus. Its white centre
        # is validated later using pixel fractions rather than contour nesting.
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area = max(1, roi_image.shape[0] * roi_image.shape[1])
        candidates: list[SignCandidate] = []
        inverse_scale = 1.0 / process_scale

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.geometry.min_area_px:
                continue
            if area / roi_area > self.config.geometry.max_area_fraction:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            # Circularity is 1.0 for an ideal circle and lower for rectangles,
            # barrier stripes, broken rings, and irregular red objects.
            circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
            if circularity < self.config.geometry.min_circularity:
                continue

            x, y, box_width, box_height = cv2.boundingRect(contour)
            aspect = box_width / max(1, box_height)
            if not (
                self.config.geometry.min_aspect_ratio
                <= aspect
                <= self.config.geometry.max_aspect_ratio
            ):
                continue

            # Stretching the candidate box to a square compensates for moderate
            # horizontal perspective compression when the sign is viewed at an angle.
            patch = self._rectify(roi_image, x, y, box_width, box_height)
            validation = self._validate_and_extract(patch)
            if validation is None:
                continue
            digit_mask, red_fraction, white_fraction = validation
            original_box = BoundingBox(
                x=int(round((x + x0) * inverse_scale)),
                y=int(round((y + y0) * inverse_scale)),
                width=int(round(box_width * inverse_scale)),
                height=int(round(box_height * inverse_scale)),
            )
            candidates.append(
                SignCandidate(
                    patch_bgr=patch,
                    digit_mask=digit_mask,
                    bounding_box=original_box,
                    circularity=circularity,
                    red_fraction=red_fraction,
                    white_fraction=white_fraction,
                )
            )

        return sorted(
            candidates,
            key=lambda item: item.circularity + item.red_fraction + item.white_fraction,
            reverse=True,
        )

    def _red_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        """Create a cleaned mask containing both OpenCV HSV red intervals."""
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        red = self.config.red
        first = cv2.inRange(
            hsv,
            np.array([red.low_hue_1, red.min_saturation, red.min_value], dtype=np.uint8),
            np.array([red.high_hue_1, 255, 255], dtype=np.uint8),
        )
        second = cv2.inRange(
            hsv,
            np.array([red.low_hue_2, red.min_saturation, red.min_value], dtype=np.uint8),
            np.array([red.high_hue_2, 255, 255], dtype=np.uint8),
        )
        mask = cv2.bitwise_or(first, second)
        kernel_size = red.morphology_kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        # Opening removes isolated red noise; closing reconnects small gaps in
        # the ring created by compression, reflections, or low image quality.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _rectify(
        self,
        roi_image: np.ndarray,
        x: int,
        y: int,
        width: int,
        height: int,
    ) -> np.ndarray:
        """Crop with slight padding and normalize a candidate to a square."""
        padding = max(2, int(round(max(width, height) * 0.025)))
        left = max(0, x - padding)
        top = max(0, y - padding)
        right = min(roi_image.shape[1], x + width + padding)
        bottom = min(roi_image.shape[0], y + height + padding)
        crop = roi_image[top:bottom, left:right]
        size = self.config.templates.rectified_size
        return cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)

    def _validate_and_extract(
        self,
        patch_bgr: np.ndarray,
    ) -> tuple[np.ndarray, float, float] | None:
        """Validate sign structure and return a normalized black-digit mask."""
        size = patch_bgr.shape[0]
        hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
        red = self.config.red
        red_mask = cv2.bitwise_or(
            cv2.inRange(
                hsv,
                np.array([red.low_hue_1, red.min_saturation, red.min_value], dtype=np.uint8),
                np.array([red.high_hue_1, 255, 255], dtype=np.uint8),
            ),
            cv2.inRange(
                hsv,
                np.array([red.low_hue_2, red.min_saturation, red.min_value], dtype=np.uint8),
                np.array([red.high_hue_2, 255, 255], dtype=np.uint8),
            ),
        )

        # Radial masks express the expected sign layout independently of the
        # candidate's pixel dimensions: red annulus outside, white/digit inside.
        yy, xx = np.ogrid[:size, :size]
        center = (size - 1) / 2.0
        radius = np.sqrt((xx - center) ** 2 + (yy - center) ** 2) / size
        geometry = self.config.geometry
        annulus = (radius >= geometry.annulus_inner_radius) & (
            radius <= geometry.annulus_outer_radius
        )
        inner = radius <= geometry.digit_region_radius
        red_fraction = float(np.count_nonzero((red_mask > 0) & annulus) / max(1, annulus.sum()))
        if red_fraction < geometry.min_red_annulus_fraction:
            return None

        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        white = (saturation < 105) & (value > 105)
        white_fraction = float(np.count_nonzero(white & inner) / max(1, inner.sum()))
        if white_fraction < geometry.min_white_inner_fraction:
            return None

        # CLAHE improves local contrast when half of the sign is shaded. Otsu
        # then chooses a per-candidate black/white threshold automatically.
        gray = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2GRAY)
        equalized = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
        _, digit = cv2.threshold(
            equalized,
            0,
            255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
        )
        digit[~inner] = 0
        digit = self._remove_small_components(digit, max(5, int(size * size * 0.00035)))
        foreground_fraction = float(np.count_nonzero(digit) / max(1, inner.sum()))
        if not (
            geometry.min_digit_fraction
            <= foreground_fraction
            <= geometry.max_digit_fraction
        ):
            return None

        normalized = normalize_foreground(
            digit,
            self.config.templates.normalized_digit_size,
            self.config.templates.normalized_margin,
        )
        return normalized, red_fraction, white_fraction

    @staticmethod
    def _remove_small_components(mask: np.ndarray, minimum_area: int) -> np.ndarray:
        """Discard dirt, compression blocks, and other tiny black components."""
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        cleaned = np.zeros_like(mask)
        for component in range(1, count):
            if stats[component, cv2.CC_STAT_AREA] >= minimum_area:
                cleaned[labels == component] = 255
        return cleaned


def annotate_frame(
    frame_bgr: np.ndarray,
    detections: Iterable[SignDetection],
    status: str = "",
) -> np.ndarray:
    """Return a copy with operator-friendly boxes, labels, and status text."""
    output = frame_bgr.copy()
    for detection in detections:
        box = detection.bounding_box
        color = (0, 220, 0) if detection.confirmed else (0, 180, 255)
        cv2.rectangle(output, (box.x, box.y), (box.x + box.width, box.y + box.height), color, 2)
        label = f"sign {detection.sign_id}  {detection.confidence:.2f}"
        cv2.putText(
            output,
            label,
            (box.x, max(22, box.y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
    if status:
        cv2.putText(
            output,
            status,
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return output
