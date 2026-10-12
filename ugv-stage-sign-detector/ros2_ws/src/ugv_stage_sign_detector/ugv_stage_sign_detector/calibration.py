from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path

import cv2
import yaml

from .config import DetectorConfig
from .detector import StageSignDetector, annotate_frame
from .templates import TemplateRepository


WINDOW = "UGV sign calibration"


def _nothing(_: int) -> None:
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Interactively tune red sign thresholds")
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--config", help="Existing detector YAML")
    parser.add_argument("--font", help="Arial Black-compatible TrueType font")
    parser.add_argument("--templates", help="Optional generated digit-template directory")
    parser.add_argument("--save", help="Write calibrated settings to this YAML file")
    args = parser.parse_args(argv)

    config = DetectorConfig.from_yaml(args.config) if args.config else DetectorConfig()
    repository = TemplateRepository.load_or_generate(
        config.templates,
        directory=args.templates,
        font_path=args.font,
    )
    detector = StageSignDetector(config, repository)

    source: int | str = int(args.source) if args.source.isdigit() else args.source
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise OSError(f"Could not open source: {args.source}")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    red = config.red
    controls = {
        "low hue 1": (red.low_hue_1, 179),
        "high hue 1": (red.high_hue_1, 179),
        "low hue 2": (red.low_hue_2, 179),
        "high hue 2": (red.high_hue_2, 179),
        "min saturation": (red.min_saturation, 255),
        "min value": (red.min_value, 255),
    }
    for name, (value, maximum) in controls.items():
        cv2.createTrackbar(name, WINDOW, value, maximum, _nothing)

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            red.low_hue_1 = cv2.getTrackbarPos("low hue 1", WINDOW)
            red.high_hue_1 = cv2.getTrackbarPos("high hue 1", WINDOW)
            red.low_hue_2 = cv2.getTrackbarPos("low hue 2", WINDOW)
            red.high_hue_2 = cv2.getTrackbarPos("high hue 2", WINDOW)
            red.min_saturation = cv2.getTrackbarPos("min saturation", WINDOW)
            red.min_value = cv2.getTrackbarPos("min value", WINDOW)

            detections = detector.detect(frame)
            annotated = annotate_frame(
                frame,
                detections,
                "Adjust HSV | S: save | Q: quit",
            )
            cv2.imshow(WINDOW, annotated)
            if detector.last_red_mask is not None:
                cv2.imshow("Red mask", detector.last_red_mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                _save(config, args.save or "calibrated_detector.yaml")
    finally:
        capture.release()
        cv2.destroyAllWindows()

    if args.save:
        _save(config, args.save)
    return 0


def _save(config: DetectorConfig, output: str) -> None:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump({"detector": asdict(config)}, stream, sort_keys=False)
    print(f"Saved calibration: {path.resolve()}")


if __name__ == "__main__":
    raise SystemExit(main())
