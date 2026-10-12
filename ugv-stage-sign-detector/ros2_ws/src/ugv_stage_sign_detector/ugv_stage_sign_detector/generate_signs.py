from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from .config import DetectorConfig
from .synthetic import make_scene, render_sign
from .templates import TemplateRepository, resolve_font_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate stage-sign templates and test scenes")
    parser.add_argument("output", help="Output directory")
    parser.add_argument("--font", help="Arial Black-compatible TrueType font")
    parser.add_argument("--diameter", type=int, default=600)
    args = parser.parse_args(argv)

    output = Path(args.output).resolve()
    resolved_font = resolve_font_path(args.font)
    signs = output / "printable_signs"
    scenes = output / "test_scenes"
    templates = output / "digit_templates"
    signs.mkdir(parents=True, exist_ok=True)
    scenes.mkdir(parents=True, exist_ok=True)
    config = DetectorConfig()
    TemplateRepository.generate(config.templates, resolved_font).save(templates)
    for sign_id in range(1, 13):
        sign = render_sign(sign_id, args.diameter, resolved_font)
        scene = make_scene(sign_id, font_path=resolved_font)
        if not cv2.imwrite(str(signs / f"sign_{sign_id}.png"), sign):
            raise OSError(f"Could not write sign {sign_id}")
        if not cv2.imwrite(str(scenes / f"scene_{sign_id}.png"), scene):
            raise OSError(f"Could not write scene {sign_id}")
    print(f"Font used: {resolved_font}")
    print(f"Generated digit templates: {templates}")
    print(f"Generated printable signs: {signs}")
    print(f"Generated test scenes: {scenes}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
