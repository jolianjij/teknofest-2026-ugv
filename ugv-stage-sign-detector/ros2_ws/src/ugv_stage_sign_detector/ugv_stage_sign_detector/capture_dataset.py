"""Capture rectified real-camera candidates for ML training.

The operator supplies the true label.  The current template prediction is not
used, which is important when collecting the exact 1/2 and 11/12 failures that
the new model must learn to fix.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

import cv2

from .classification import Classification
from .config import DetectorConfig
from .detector import StageSignDetector
from .ml_dataset import class_name, parse_class_name


class _UnusedClassifier:
    """Candidate extraction needs no label classifier."""

    def classify(self, patch_bgr, digit_mask=None) -> Classification:  # pragma: no cover
        raise RuntimeError("The capture tool must not classify dataset samples")


def capture_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture real rectified stage-sign samples")
    parser.add_argument("output", help="Dataset root")
    parser.add_argument("--label", required=True, help="unknown or 1 through 12")
    parser.add_argument("--split", choices=("train", "val", "test"), required=True)
    parser.add_argument("--session", required=True, help="Unique lighting/view recording name")
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--config", help="Detector YAML used for red-ring extraction")
    parser.add_argument("--auto", action="store_true", help="Save automatically instead of Space")
    parser.add_argument("--interval", type=int, default=10, help="Frames between automatic saves")
    parser.add_argument("--max-images", type=int, default=300)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)

    if args.headless and not args.auto:
        parser.error("--headless requires --auto")
    if args.interval < 1 or args.max_images < 1:
        parser.error("--interval and --max-images must be positive")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.session):
        parser.error("--session may contain only letters, numbers, underscores, and hyphens")

    label = parse_class_name(args.label)
    destination = (
        Path(args.output).expanduser().resolve()
        / args.split
        / class_name(label)
    )
    destination.mkdir(parents=True, exist_ok=True)

    config = DetectorConfig.from_yaml(args.config) if args.config else DetectorConfig()
    detector = StageSignDetector(config, classifier=_UnusedClassifier())
    source: int | str = int(args.source) if args.source.isdigit() else args.source
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        print(f"Could not open video source: {args.source}", file=sys.stderr)
        return 2

    frame_index = 0
    saved = 0
    try:
        while saved < args.max_images:
            ok, frame = capture.read()
            if not ok:
                break
            candidates = detector.extract_candidates(frame)
            best = candidates[0] if candidates else None
            preview = frame.copy()
            if best is not None:
                box = best.bounding_box
                cv2.rectangle(
                    preview,
                    (box.x, box.y),
                    (box.x + box.width, box.y + box.height),
                    (0, 220, 0),
                    2,
                )
            instruction = (
                f"label={class_name(label)} split={args.split} saved={saved}/{args.max_images}"
            )
            if not args.auto:
                instruction += " | SPACE save | Q quit"
            cv2.putText(
                preview,
                instruction,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            key = -1
            if not args.headless:
                cv2.imshow("UGV ML dataset capture", preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            should_save = best is not None and (
                (args.auto and frame_index % args.interval == 0)
                or (not args.auto and key == ord(" "))
            )
            if should_save:
                filename = (
                    f"{args.session}_{time.time_ns()}_{saved:05d}.jpg"
                )
                path = destination / filename
                if not cv2.imwrite(str(path), best.patch_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise OSError(f"Could not save camera sample: {path}")
                saved += 1
                print(f"Saved {saved}: {path}", flush=True)
            frame_index += 1
    finally:
        capture.release()
        if not args.headless:
            cv2.destroyAllWindows()

    print(f"Captured {saved} images in: {destination}", flush=True)
    print("Keep this session out of the other train/val/test splits.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(capture_main())
