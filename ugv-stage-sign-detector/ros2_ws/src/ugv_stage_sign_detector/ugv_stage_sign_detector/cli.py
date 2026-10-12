from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from .config import DetectorConfig
from .detector import StageSignDetector, annotate_frame
from .templates import TemplateRepository, resolve_font_path
from .temporal import TemporalConfirmer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect numbered UGV stage signs")
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--config", help="Detector YAML configuration")
    parser.add_argument("--templates", help="Directory containing sign_1.png ... sign_12.png")
    parser.add_argument("--font", help="Arial Black-compatible TrueType font")
    parser.add_argument(
        "--classifier",
        choices=("template", "onnx"),
        help="Override classifier.mode from the YAML configuration",
    )
    parser.add_argument("--model", help="Override path to the ONNX classifier model")
    parser.add_argument("--headless", action="store_true", help="Disable the preview window")
    parser.add_argument("--output", help="Optional annotated output video")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames; 0 is unlimited")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = DetectorConfig.from_yaml(args.config) if args.config else DetectorConfig()
    if args.classifier:
        config.classifier.mode = args.classifier
    if args.model:
        config.classifier.model_path = str(Path(args.model).expanduser().resolve())
    config.validate()

    templates: TemplateRepository | None = None
    if config.classifier.mode == "onnx":
        print(
            f"Loading ONNX sign classifier: {Path(config.classifier.model_path).resolve()}",
            flush=True,
        )
    else:
        if args.templates:
            print(f"Loading digit templates from: {Path(args.templates).resolve()}", flush=True)
        else:
            resolved_font = resolve_font_path(args.font or config.templates.font_path)
            print(f"Generating digit templates with font: {resolved_font}", flush=True)
        templates = TemplateRepository.load_or_generate(
            config.templates,
            directory=args.templates,
            font_path=args.font,
        )
    try:
        detector = StageSignDetector(config, templates)
    except (FileNotFoundError, OSError, RuntimeError, cv2.error) as exc:
        print(f"Could not initialize sign classifier: {exc}", file=sys.stderr)
        return 2
    confirmer = TemporalConfirmer(config.temporal)

    source: int | str = int(args.source) if args.source.isdigit() else args.source
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        print(f"Could not open video source: {args.source}", file=sys.stderr)
        return 2

    writer: cv2.VideoWriter | None = None
    frame_count = 0
    fps_average = 0.0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = time.perf_counter()
            detections = detector.detect(frame)
            best = detections[0] if detections else None
            confirmed = confirmer.update(best)
            elapsed = max(1e-6, time.perf_counter() - started)
            instantaneous_fps = 1.0 / elapsed
            fps_average = instantaneous_fps if frame_count == 0 else 0.9 * fps_average + 0.1 * instantaneous_fps

            shown_detections = detections
            if confirmed is not None:
                shown_detections = [confirmed] + [
                    item for item in detections if item.bounding_box != confirmed.bounding_box
                ]
                print(
                    json.dumps(
                        {
                            "event": "confirmed_stage_sign",
                            "sign_id": confirmed.sign_id,
                            "confidence": round(confirmed.confidence, 4),
                            "bbox": {
                                "x": confirmed.bounding_box.x,
                                "y": confirmed.bounding_box.y,
                                "width": confirmed.bounding_box.width,
                                "height": confirmed.bounding_box.height,
                            },
                        }
                    ),
                    flush=True,
                )

            status = f"CV {fps_average:4.1f} FPS"
            if confirmer.locked_sign is not None:
                status += f" | locked sign {confirmer.locked_sign}"
            annotated = annotate_frame(frame, shown_detections, status)

            if args.output:
                if writer is None:
                    input_fps = capture.get(cv2.CAP_PROP_FPS)
                    if input_fps <= 1 or input_fps > 240:
                        input_fps = 20.0
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(
                        args.output,
                        fourcc,
                        input_fps,
                        (annotated.shape[1], annotated.shape[0]),
                    )
                    if not writer.isOpened():
                        raise OSError(f"Could not create output video: {args.output}")
                writer.write(annotated)

            if not args.headless:
                cv2.imshow("UGV stage sign detector", annotated)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            frame_count += 1
            if args.max_frames and frame_count >= args.max_frames:
                break
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
