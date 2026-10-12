"""
ROS-independent safe-terrain and cone extraction for the front camera.

Stage 5 (traffic cones) is worth 50 points, loses 5 per contact, and is the
one stage the autonomous run may NOT pass on - so it deserves a second
opinion alongside lidar. Lidar sees a 75 cm cone easily and stays
authoritative; the camera adds two things it cannot give:

1. Drivable ground. Lidar in a 2D plane cannot tell tarmac from a ditch or
   a soft verge. The ground-plane heuristic marks terrain that looks like
   the surface the vehicle is already driving on.
2. Where a cone meets the ground. A cone's base pixel, under a flat-ground
   assumption, converts to a real range and bearing - which is what a
   planner actually consumes.

The obstacle rule is deliberately simple and needs no training data: an
obstacle is a region that is NOT drivable but is surrounded by, or sits
directly on, terrain that is. That catches cones without a cone detector,
and without the colour assumptions that keep breaking under real lighting.

Geometry note: `pixel_to_ground` assumes flat ground, a known camera height
and pitch, and a known vertical FOV. On the 20% side slope and the 45%
inclines that assumption is wrong, which is fine - cones only appear on
the flat middle lane. Do not reuse this projection on the hills.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

# Camera mounting - PLACEHOLDERS, must be measured on the real vehicle before
# any range estimate is trusted (same caveat as the URDF's dimensions).
CAMERA_HEIGHT_M = 0.45
CAMERA_PITCH_DEG = -8.0        # negative = tilted down toward the ground
CAMERA_VFOV_DEG = 48.0
CAMERA_HFOV_DEG = 62.0

MIN_OBSTACLE_AREA_PX = 250     # ignore speckle
MAX_RANGE_M = 12.0             # beyond this the flat-ground projection is noise


@dataclass(frozen=True)
class GroundObstacle:
    """One obstacle standing on the drivable surface."""

    range_m: float
    bearing_deg: float          # positive = to the right of centre
    base_x_px: int
    base_y_px: int
    area_px: int
    width_px: int


def pixel_to_ground(
    x_px: int,
    y_px: int,
    image_width: int,
    image_height: int,
    camera_height_m: float = CAMERA_HEIGHT_M,
    camera_pitch_deg: float = CAMERA_PITCH_DEG,
    vfov_deg: float = CAMERA_VFOV_DEG,
    hfov_deg: float = CAMERA_HFOV_DEG,
) -> tuple[float, float] | None:
    """Project a ground-contact pixel to (range_m, bearing_deg).

    Returns None when the pixel is at or above the horizon, where a
    flat-ground projection has no finite solution.
    """
    # Angle of this row below the optical axis, then below horizontal.
    row_offset = (y_px - image_height / 2.0) / (image_height / 2.0)
    angle_below_axis = math.radians(row_offset * vfov_deg / 2.0)
    depression = angle_below_axis - math.radians(camera_pitch_deg)

    if depression <= 1e-4:
        return None  # at or above the horizon

    range_m = camera_height_m / math.tan(depression)
    if not math.isfinite(range_m) or range_m <= 0 or range_m > MAX_RANGE_M:
        return None

    col_offset = (x_px - image_width / 2.0) / (image_width / 2.0)
    bearing_deg = col_offset * hfov_deg / 2.0
    return float(range_m), float(bearing_deg)


def find_ground_obstacles(
    drivable_mask: np.ndarray,
    min_area_px: int = MIN_OBSTACLE_AREA_PX,
) -> list[GroundObstacle]:
    """Find non-drivable regions standing on the drivable surface.

    `drivable_mask` is the uint8 output of the ground-plane heuristic
    (255 = drivable). An obstacle is a hole in that surface: a non-drivable
    blob whose lowest row has drivable terrain immediately beneath it, which
    is exactly what a cone standing on the road looks like from a
    forward-facing camera. A non-drivable region with nothing drivable below
    it is just the sky, a wall, or terrain beyond the road edge.
    """
    if drivable_mask is None or drivable_mask.size == 0:
        return []

    height, width = drivable_mask.shape[:2]
    drivable = drivable_mask > 0
    holes = (~drivable).astype(np.uint8)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(holes, connectivity=8)
    obstacles: list[GroundObstacle] = []

    for index in range(1, count):  # 0 is the background label
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area_px:
            continue

        top = int(stats[index, cv2.CC_STAT_TOP])
        left = int(stats[index, cv2.CC_STAT_LEFT])
        blob_width = int(stats[index, cv2.CC_STAT_WIDTH])
        blob_height = int(stats[index, cv2.CC_STAT_HEIGHT])
        bottom = top + blob_height - 1

        # Anything touching the bottom edge is the vehicle's own nose or the
        # unviewable near field, not an obstacle ahead.
        if bottom >= height - 2:
            continue

        # Anything touching the top edge is sky, background or a wall. Such a
        # region also has drivable ground along its lower boundary, so without
        # this rule the entire above-horizon background passes the support test
        # below and is reported as one enormous obstacle.
        if top <= 1:
            continue

        # A cone occupies a small part of the view. A blob spanning most of the
        # frame is a shadow band, a lighting artefact, or terrain beyond the
        # road edge - not something to plan around as a discrete obstacle.
        if blob_width > width * 0.6:
            continue

        component = labels[:, left:left + blob_width] == index
        bottom_row = component[bottom]
        if not bottom_row.any():
            continue
        base_x = left + int(np.flatnonzero(bottom_row).mean())

        # Require drivable ground directly beneath the base: that is what
        # distinguishes a cone standing on the road from the sky or a wall.
        support_band = drivable[bottom + 1:min(height, bottom + 6), base_x]
        if support_band.size == 0 or not support_band.any():
            continue

        projected = pixel_to_ground(base_x, bottom, width, height)
        if projected is None:
            continue
        range_m, bearing_deg = projected

        obstacles.append(
            GroundObstacle(
                range_m=range_m,
                bearing_deg=bearing_deg,
                base_x_px=base_x,
                base_y_px=bottom,
                area_px=area,
                width_px=blob_width,
            )
        )

    obstacles.sort(key=lambda item: item.range_m)
    return obstacles
