"""Generate and compare digit templates without a trained model.

The competition specification fixes the typeface to Arial Black. On Windows,
the resolver uses the installed Arial Black file. On Linux, a compatible bold
font is a fallback; for best competition accuracy, generate templates on the
Windows laptop and copy the resulting PNG masks to the Raspberry Pi.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .classification import Classification
from .config import TemplateConfig


# The physical sign can be tilted in the camera image. Comparing only with an
# upright glyph made acceptance depend too much on the exact font rasterizer
# used by Windows or Linux. These modest variants cover the detector's tested
# in-plane rotation range without making unrelated digits rotationally valid.
TEMPLATE_ROTATION_ANGLES = (-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0)


class TemplateRepository:
    """In-memory reference masks for labels 1 through 12."""
    def __init__(self, masks: dict[int, np.ndarray], config: TemplateConfig):
        if not masks:
            raise ValueError("No sign templates were supplied")
        self.masks = masks
        self.config = config
        # Build these once at startup rather than rotating templates for every
        # camera frame. Each label still receives one score: its best supported
        # orientation. The runner-up therefore remains a different digit.
        self.rotated_masks = {
            label: tuple(
                rotate_and_normalize_template(mask, angle, config)
                for angle in TEMPLATE_ROTATION_ANGLES
            )
            for label, mask in masks.items()
        }
        # Whole-label matching is useful, but it can confuse visually related
        # pairs such as 1/2 and 11/12 after blur or perspective distortion.
        # Build reusable references for each individual character as a second,
        # independent signal. Samples are learned only from the supplied font
        # templates; no trained model or external dataset is involved.
        self.glyph_masks = collect_glyph_samples(masks, config)
        self.rotated_glyph_masks = {
            digit: tuple(
                rotate_and_normalize_template(sample, angle, config)
                for sample in samples
                for angle in TEMPLATE_ROTATION_ANGLES
            )
            for digit, samples in self.glyph_masks.items()
        }

    @classmethod
    def generate(
        cls,
        config: TemplateConfig,
        font_path: str | Path | None = None,
    ) -> "TemplateRepository":
        resolved_font = resolve_font_path(font_path or config.font_path)
        masks: dict[int, np.ndarray] = {}
        for label in config.labels:
            rendered = render_digit_mask(
                str(label),
                canvas_size=config.rectified_size,
                font_path=resolved_font,
            )
            masks[label] = normalize_foreground(
                rendered,
                config.normalized_digit_size,
                config.normalized_margin,
            )
        return cls(masks, config)

    @classmethod
    def load_or_generate(
        cls,
        config: TemplateConfig,
        directory: str | Path | None = None,
        font_path: str | Path | None = None,
    ) -> "TemplateRepository":
        if directory:
            root = Path(directory)
            if not root.is_dir():
                raise FileNotFoundError(f"Digit-template directory does not exist: {root.resolve()}")
            expected = [root / f"sign_{label}.png" for label in config.labels]
            missing = [path.name for path in expected if not path.is_file()]
            if missing:
                raise FileNotFoundError(
                    f"Digit-template directory is incomplete: {root.resolve()}; "
                    f"missing {', '.join(missing)}"
                )
            loaded: dict[int, np.ndarray] = {}
            for label, path in zip(config.labels, expected):
                image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise OSError(f"Could not read digit template: {path.resolve()}")
                loaded[label] = normalize_foreground(
                    image,
                    config.normalized_digit_size,
                    config.normalized_margin,
                )
            return cls(loaded, config)
        return cls.generate(config, font_path)

    def save(self, directory: str | Path) -> None:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        for label, mask in self.masks.items():
            if not cv2.imwrite(str(root / f"sign_{label}.png"), mask):
                raise OSError(f"Could not write template for sign {label}")

    def classify(self, observed: np.ndarray) -> Classification:
        """Return the closest template and a confidence/uniqueness estimate.

        ``binary_shape_score`` handles the primary shape similarity. The
        difference between the best and second-best labels is also considered:
        a high raw score is not trusted when two labels look almost identical.
        """
        normalized = normalize_foreground(
            observed,
            self.config.normalized_digit_size,
            self.config.normalized_margin,
        )
        observed_glyphs = split_digit_components(normalized, self.config)
        glyph_score_cache: list[dict[int, float]] = []
        for observed_glyph in observed_glyphs:
            glyph_score_cache.append(
                {
                    digit: max(
                        glyph_shape_score(observed_glyph, variant)
                        for variant in variants
                    )
                    for digit, variants in self.rotated_glyph_masks.items()
                    if variants
                }
            )

        scored_labels: list[tuple[float, int]] = []
        for label in self.masks:
            expected_digits = [int(character) for character in str(label)]

            if len(observed_glyphs) != len(expected_digits):
                # A one-character observation cannot be sign 10-12, and a
                # two-character observation cannot be sign 1-9. Fail closed
                # instead of allowing whole-mask similarity to cross groups.
                score = 0.0
            else:
                whole_score = max(
                    binary_shape_score(normalized, variant)
                    for variant in self.rotated_masks[label]
                )
                per_glyph_scores = [
                    glyph_score_cache[index].get(expected_digit, 0.0)
                    for index, expected_digit in enumerate(expected_digits)
                ]
                glyph_score = (
                    float(np.mean(per_glyph_scores))
                    if per_glyph_scores
                    else 0.0
                )
                score = 0.40 * whole_score + 0.60 * glyph_score
            scored_labels.append((score, label))

        scores = sorted(scored_labels, reverse=True)
        best_score, best_label = scores[0]
        second_score, second_label = scores[1] if len(scores) > 1 else (0.0, best_label)
        margin = max(0.0, best_score - second_score)
        uniqueness = min(1.0, margin / 0.12)
        confidence = float(np.clip(0.88 * best_score + 0.12 * uniqueness, 0.0, 1.0))
        return Classification(best_label, confidence, second_label, margin)


class TemplateSignClassifier:
    """Adapt the legacy digit-mask repository to the common classifier API."""

    def __init__(self, repository: TemplateRepository):
        self.repository = repository

    def classify(
        self,
        patch_bgr: np.ndarray,
        digit_mask: np.ndarray | None = None,
    ) -> Classification:
        del patch_bgr
        if digit_mask is None:
            raise ValueError("Template classification requires an extracted digit mask")
        return self.repository.classify(digit_mask)


def collect_glyph_samples(
    masks: dict[int, np.ndarray],
    config: TemplateConfig,
) -> dict[int, tuple[np.ndarray, ...]]:
    """Extract labelled single-character samples from templates 1 through 12."""
    samples: dict[int, list[np.ndarray]] = {}
    canonical: dict[int, np.ndarray] = {}
    for label, mask in masks.items():
        digits = [int(character) for character in str(label)]
        components = split_digit_components(mask, config)
        if len(components) != len(digits):
            continue
        for digit, component in zip(digits, components):
            samples.setdefault(digit, []).append(component)
        # A direct one-character label is the cleanest reference for digits
        # 1-9. Digit 0 is obtained from sign 10 because sign 0 does not exist.
        if len(digits) == 1:
            canonical[digits[0]] = components[0]
    return {
        digit: (canonical[digit] if digit in canonical else items[0],)
        for digit, items in samples.items()
    }


def split_digit_components(
    mask: np.ndarray,
    config: TemplateConfig,
) -> list[np.ndarray]:
    """Split a normalized label into left-to-right connected digit glyphs."""
    binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
    total_foreground = int(np.count_nonzero(binary))
    if total_foreground == 0:
        return []

    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    minimum_area = max(8, int(round(total_foreground * 0.08)))
    components: list[tuple[int, np.ndarray]] = []
    for component in range(1, count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area < minimum_area:
            continue
        component_mask = np.zeros_like(binary)
        component_mask[labels == component] = 255
        normalized = normalize_foreground(
            component_mask,
            config.normalized_digit_size,
            config.normalized_margin,
        )
        x_position = int(stats[component, cv2.CC_STAT_LEFT])
        components.append((x_position, normalized))
    return [component for _, component in sorted(components, key=lambda item: item[0])]


def glyph_shape_score(observed: np.ndarray, template: np.ndarray) -> float:
    """Combine mask similarity with horizontal/vertical stroke profiles."""
    pixel_score = binary_shape_score(observed, template)
    profile_score = projection_profile_score(observed, template)
    return float(np.clip(0.70 * pixel_score + 0.30 * profile_score, 0.0, 1.0))


def projection_profile_score(first: np.ndarray, second: np.ndarray) -> float:
    """Compare row/column occupancy, which strongly separates 1 from 2."""
    first_profile = _projection_profile(first)
    second_profile = _projection_profile(second)
    if first_profile is None or second_profile is None:
        return 0.0
    first_rows, first_columns = first_profile
    second_rows, second_columns = second_profile
    difference = float(
        np.mean(np.abs(first_rows - second_rows))
        + np.mean(np.abs(first_columns - second_columns))
    )
    return float(np.clip(1.0 - difference, 0.0, 1.0))


def _projection_profile(
    mask: np.ndarray,
    size: int = 48,
) -> tuple[np.ndarray, np.ndarray] | None:
    binary = np.asarray(mask) > 0
    ys, xs = np.nonzero(binary)
    if len(xs) == 0:
        return None
    crop = binary[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1].astype(np.uint8)
    normalized = cv2.resize(crop, (size, size), interpolation=cv2.INTER_NEAREST) > 0
    return normalized.mean(axis=1), normalized.mean(axis=0)


def rotate_and_normalize_template(
    mask: np.ndarray,
    angle_degrees: float,
    config: TemplateConfig,
) -> np.ndarray:
    """Return one centered digit template at a supported image rotation."""
    if angle_degrees == 0.0:
        return mask.copy()
    height, width = mask.shape[:2]
    center = ((width - 1) / 2.0, (height - 1) / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle_degrees, 1.0)
    rotated = cv2.warpAffine(
        mask,
        matrix,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return normalize_foreground(
        rotated,
        config.normalized_digit_size,
        config.normalized_margin,
    )


def resolve_font_path(requested: str | Path | None = None) -> Path:
    candidates: list[Path] = []
    environment_font = os.getenv("UGV_SIGN_FONT", "")
    for item in (requested, environment_font):
        if item:
            candidates.append(Path(item))

    candidates.extend(
        [
            Path("C:/Windows/Fonts/ariblk.ttf"),
            Path("C:/Windows/Fonts/Arialbd.ttf"),
            Path("/usr/share/fonts/truetype/msttcorefonts/Arial_Black.ttf"),
            Path("/usr/share/fonts/truetype/msttcorefonts/Arial_Bold.ttf"),
            Path("/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"),
            Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
        ]
    )
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        "No bold TrueType font was found. Set UGV_SIGN_FONT or templates.font_path."
    )


def render_digit_mask(text: str, canvas_size: int, font_path: str | Path) -> np.ndarray:
    image = Image.new("L", (canvas_size, canvas_size), 0)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(str(font_path), int(canvas_size * 0.52))
    box = draw.textbbox((0, 0), text, font=font, stroke_width=0)
    width = box[2] - box[0]
    height = box[3] - box[1]
    x = (canvas_size - width) / 2 - box[0]
    y = (canvas_size - height) / 2 - box[1]
    draw.text((x, y), text, fill=255, font=font)
    return np.asarray(image, dtype=np.uint8)


def normalize_foreground(mask: np.ndarray, size: int, margin: int) -> np.ndarray:
    """Crop a digit and place it on a fixed square while preserving aspect.

    This removes distance and small centering differences while preserving the
    distinguishing geometry of narrow digits, wide multi-digit labels, and
    enclosed holes such as those in 6, 8, and 9.
    """
    binary = np.asarray(mask) > 0
    ys, xs = np.nonzero(binary)
    canvas = np.zeros((size, size), dtype=np.uint8)
    if len(xs) == 0:
        return canvas

    crop = binary[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1].astype(np.uint8) * 255
    available = max(1, size - 2 * margin)
    scale = min(available / crop.shape[1], available / crop.shape[0])
    width = max(1, int(round(crop.shape[1] * scale)))
    height = max(1, int(round(crop.shape[0] * scale)))
    resized = cv2.resize(crop, (width, height), interpolation=cv2.INTER_NEAREST)
    x0 = (size - width) // 2
    y0 = (size - height) // 2
    canvas[y0 : y0 + height, x0 : x0 + width] = resized
    return canvas


def binary_shape_score(observed: np.ndarray, template: np.ndarray) -> float:
    """Combine pixel overlap and symmetric chamfer distance.

    Dice overlap rewards matching foreground pixels. Chamfer distance remains
    tolerant of small edge shifts caused by blur, resizing, and font rendering.
    The final score is bounded to 0..1.
    """
    first = observed > 0
    second = template > 0
    count_first = int(first.sum())
    count_second = int(second.sum())
    if count_first == 0 or count_second == 0:
        return 0.0

    intersection = int(np.logical_and(first, second).sum())
    dice = (2.0 * intersection) / (count_first + count_second)

    distance_to_second = cv2.distanceTransform((~second).astype(np.uint8), cv2.DIST_L2, 3)
    distance_to_first = cv2.distanceTransform((~first).astype(np.uint8), cv2.DIST_L2, 3)
    chamfer = 0.5 * (
        float(distance_to_second[first].mean()) + float(distance_to_first[second].mean())
    )
    chamfer_score = math.exp(-chamfer / 3.5)
    return float(np.clip(0.52 * dice + 0.48 * chamfer_score, 0.0, 1.0))
