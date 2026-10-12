from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .templates import resolve_font_path


def render_sign(
    label: int,
    diameter: int = 240,
    font_path: str | Path | None = None,
) -> np.ndarray:
    """Render a specification-like red-ring sign for tests and printing."""

    if label < 0:
        raise ValueError("Sign label must be non-negative")
    resolved_font = resolve_font_path(font_path)
    image = Image.new("RGB", (diameter, diameter), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    inset = max(2, int(round(diameter * 0.025)))
    ring_width = max(4, int(round(diameter * 0.115)))
    draw.ellipse(
        (inset, inset, diameter - inset - 1, diameter - inset - 1),
        fill=(235, 0, 0),
    )
    inner = inset + ring_width
    draw.ellipse(
        (inner, inner, diameter - inner - 1, diameter - inner - 1),
        fill=(255, 255, 255),
    )

    text = str(label)
    font = ImageFont.truetype(str(resolved_font), int(round(diameter * 0.43)))
    box = draw.textbbox((0, 0), text, font=font)
    width = box[2] - box[0]
    height = box[3] - box[1]
    position = ((diameter - width) / 2 - box[0], (diameter - height) / 2 - box[1])
    draw.text(position, text, font=font, fill=(0, 0, 0))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def make_scene(
    label: int,
    frame_size: tuple[int, int] = (1280, 720),
    sign_diameter: int = 210,
    center: tuple[int, int] = (980, 235),
    angle_degrees: float = 0.0,
    horizontal_scale: float = 1.0,
    font_path: str | Path | None = None,
) -> np.ndarray:
    width, height = frame_size
    scene = np.full((height, width, 3), (150, 175, 150), dtype=np.uint8)
    cv2.rectangle(scene, (0, int(height * 0.62)), (width, height), (90, 105, 80), -1)
    cv2.rectangle(scene, (0, int(height * 0.53)), (width, int(height * 0.62)), (235, 235, 235), -1)
    for start in range(0, width, 240):
        cv2.rectangle(
            scene,
            (start, int(height * 0.53)),
            (min(width - 1, start + 120), int(height * 0.62)),
            (20, 20, 210),
            -1,
        )

    sign = render_sign(label, sign_diameter, font_path)
    matrix = cv2.getRotationMatrix2D(
        ((sign_diameter - 1) / 2, (sign_diameter - 1) / 2),
        angle_degrees,
        1.0,
    )
    rotated = cv2.warpAffine(
        sign,
        matrix,
        (sign_diameter, sign_diameter),
        flags=cv2.INTER_LINEAR,
        borderValue=(150, 175, 150),
    )
    if horizontal_scale != 1.0:
        rotated = cv2.resize(
            rotated,
            (max(1, int(round(sign_diameter * horizontal_scale))), sign_diameter),
            interpolation=cv2.INTER_AREA,
        )

    sign_height, sign_width = rotated.shape[:2]
    x0 = center[0] - sign_width // 2
    y0 = center[1] - sign_height // 2
    x1 = x0 + sign_width
    y1 = y0 + sign_height
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        raise ValueError("Synthetic sign does not fit inside the frame")
    scene[y0:y1, x0:x1] = rotated
    return scene
