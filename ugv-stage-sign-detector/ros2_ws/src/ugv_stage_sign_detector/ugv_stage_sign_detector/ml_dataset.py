"""Synthetic dataset generation and dataset-folder utilities.

Synthetic images bootstrap the classifier, but they do not replace validation
with the real laptop camera and Raspberry Pi Camera Module 3.  Keep complete
recording sessions in only one split (train, validation, or test).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable, Iterator

import cv2
import numpy as np

from .synthetic import render_sign
from .templates import resolve_font_path


CLASS_NAMES = ("unknown",) + tuple(str(label) for label in range(1, 13))
CLASS_LABELS = tuple(range(0, 13))
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}


def class_name(label: int) -> str:
    if label not in CLASS_LABELS:
        raise ValueError(f"Unsupported dataset label: {label}")
    return "unknown" if label == 0 else str(label)


def parse_class_name(value: str) -> int:
    normalized = value.strip().lower()
    if normalized in {"unknown", "reject", "0"}:
        return 0
    label = int(normalized)
    if label not in CLASS_LABELS:
        raise ValueError("Dataset label must be unknown or an integer from 1 through 12")
    return label


def iter_dataset_images(
    roots: Iterable[str | Path],
    split: str,
) -> Iterator[tuple[Path, int]]:
    """Yield image paths from ``ROOT/SPLIT/CLASS`` in deterministic order."""

    for root_value in roots:
        root = Path(root_value).expanduser()
        for label in CLASS_LABELS:
            directory = root / split / class_name(label)
            if not directory.is_dir():
                continue
            for path in sorted(directory.rglob("*")):
                if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
                    yield path, label


def generate_synthetic_dataset(
    output: str | Path,
    counts: dict[str, int],
    fonts: list[str | Path] | None = None,
    seed: int = 2026,
    patch_size: int = 160,
) -> int:
    """Generate augmented 1-12 and unknown rectified sign patches."""

    root = Path(output).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    resolved_fonts = [resolve_font_path(path) for path in fonts] if fonts else [resolve_font_path()]
    random = np.random.default_rng(seed)
    manifest_rows: list[tuple[str, str, int, str]] = []
    written = 0

    for split, count in counts.items():
        if count < 0:
            raise ValueError(f"Image count for {split} cannot be negative")
        for label in CLASS_LABELS:
            destination = root / split / class_name(label)
            destination.mkdir(parents=True, exist_ok=True)
            for index in range(count):
                font = resolved_fonts[int(random.integers(0, len(resolved_fonts)))]
                rendered_label = _unknown_source_label(random) if label == 0 else label
                image = render_sign(rendered_label, diameter=patch_size, font_path=font)
                if label == 0 and random.random() < 0.35:
                    image = _corrupt_unknown(image, random)
                image = augment_sign_patch(image, random)
                filename = f"synthetic_{seed}_{index:06d}.jpg"
                path = destination / filename
                if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                    raise OSError(f"Could not write generated image: {path}")
                manifest_rows.append(
                    (str(path.relative_to(root)), split, label, str(font))
                )
                written += 1

    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("path", "split", "label", "font"))
        writer.writerows(manifest_rows)
    return written


def augment_sign_patch(image_bgr: np.ndarray, random: np.random.Generator) -> np.ndarray:
    """Apply camera-like geometric, lighting, blur, noise, and screen artifacts."""

    height, width = image_bgr.shape[:2]
    background_level = int(random.integers(185, 256))
    destination = np.array(
        [
            [random.uniform(0.00, 0.10) * width, random.uniform(0.00, 0.10) * height],
            [random.uniform(0.90, 1.00) * width, random.uniform(0.00, 0.10) * height],
            [random.uniform(0.90, 1.00) * width, random.uniform(0.90, 1.00) * height],
            [random.uniform(0.00, 0.10) * width, random.uniform(0.90, 1.00) * height],
        ],
        dtype=np.float32,
    )
    # Independent horizontal compression simulates an oblique camera view.
    compression = float(random.uniform(0.68, 1.0))
    center_x = width / 2.0
    destination[:, 0] = center_x + (destination[:, 0] - center_x) * compression
    source = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(
        image_bgr,
        transform,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(background_level,) * 3,
    )

    angle = float(random.uniform(-22.0, 22.0))
    rotation = cv2.getRotationMatrix2D(((width - 1) / 2, (height - 1) / 2), angle, 1.0)
    warped = cv2.warpAffine(
        warped,
        rotation,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(background_level,) * 3,
    )

    alpha = float(random.uniform(0.65, 1.35))
    beta = float(random.uniform(-45.0, 45.0))
    warped = cv2.convertScaleAbs(warped, alpha=alpha, beta=beta)

    if random.random() < 0.45:
        overlay = warped.copy()
        axes = (int(random.uniform(0.20, 0.55) * width), int(random.uniform(0.04, 0.16) * height))
        center = (int(random.uniform(0.2, 0.8) * width), int(random.uniform(0.2, 0.8) * height))
        cv2.ellipse(overlay, center, axes, float(random.uniform(-45, 45)), 0, 360, (255, 255, 255), -1)
        warped = cv2.addWeighted(overlay, float(random.uniform(0.10, 0.32)), warped, 1.0, 0)

    if random.random() < 0.35:
        spacing = int(random.integers(3, 9))
        line_value = int(random.integers(170, 245))
        for row in range(int(random.integers(0, spacing)), height, spacing):
            cv2.line(warped, (0, row), (width - 1, row), (line_value,) * 3, 1)

    blur_choice = int(random.integers(0, 4))
    if blur_choice:
        kernel = 2 * blur_choice + 1
        warped = cv2.GaussianBlur(warped, (kernel, kernel), float(random.uniform(0.4, 2.0)))

    noise_sigma = float(random.uniform(0.0, 12.0))
    noise = random.normal(0.0, noise_sigma, warped.shape).astype(np.float32)
    warped = np.clip(warped.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    if random.random() < 0.55:
        quality = int(random.integers(45, 96))
        ok, encoded = cv2.imencode(".jpg", warped, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if decoded is not None:
                warped = decoded
    return warped


def _unknown_source_label(random: np.random.Generator) -> int:
    possibilities = [0] + list(range(13, 100))
    return int(possibilities[int(random.integers(0, len(possibilities)))])


def _corrupt_unknown(image_bgr: np.ndarray, random: np.random.Generator) -> np.ndarray:
    output = image_bgr.copy()
    height, width = output.shape[:2]
    for _ in range(int(random.integers(1, 4))):
        start = (int(random.integers(0, width)), int(random.integers(0, height)))
        end = (int(random.integers(0, width)), int(random.integers(0, height)))
        colour = int(random.integers(0, 80))
        cv2.line(output, start, end, (colour,) * 3, int(random.integers(3, 10)))
    return output


def generate_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate an augmented stage-sign ML dataset")
    parser.add_argument("output", help="Dataset root to create")
    parser.add_argument("--font", action="append", help="Repeat to train with several TrueType fonts")
    parser.add_argument("--train-per-class", type=int, default=1200)
    parser.add_argument("--val-per-class", type=int, default=200)
    parser.add_argument("--test-per-class", type=int, default=200)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args(argv)
    total = generate_synthetic_dataset(
        args.output,
        {
            "train": args.train_per_class,
            "val": args.val_per_class,
            "test": args.test_per_class,
        },
        fonts=args.font,
        seed=args.seed,
    )
    print(f"Generated {total} images under: {Path(args.output).resolve()}", flush=True)
    print("Classes: unknown, 1, 2, ... 12", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(generate_main())
