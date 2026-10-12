"""
ROS-independent sliding-door tracking core.

Separated from door_direction_node.py for the same reason the stage-sign
package splits detector from node: the decision this makes - which way the
door is travelling, and whether the gate is clear - gates a 50-point stage
where the failure mode is a collision. It has to be testable without a
robot, a camera, or a real door.

See door_direction_node.py's module docstring for why this is frame
differencing rather than a learned model.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

MOTION_THRESHOLD = 18
MIN_MOTION_FRACTION = 0.015
# The competition door is 1 m wide in a 3 m corridor, so it can never occlude
# more than about a third of the gate. Thresholds must sit well below that
# ceiling or the door never registers as present at all and no period is ever
# measured. Hysteresis keeps a door hovering near the edge from chattering.
BLOCKED_FRACTION_ENTER = 0.15
BLOCKED_FRACTION_EXIT = 0.07
HISTORY = 12
MIN_SPEED_PX_S = 25.0
# Per-column vertical standard deviation below which a column is treated as
# occluded by the door panel. Absolute (not relative) so that a fully blocked
# gate, where every column is uniform, still reads as blocked rather than
# self-normalising back to 0.5. Tune on real footage of the actual door.
UNIFORM_COLUMN_STD = 12.0

DIRECTION_LEFT = -1.0
DIRECTION_RIGHT = 1.0
DIRECTION_UNKNOWN = 0.0


@dataclass(frozen=True)
class DoorState:
    """One frame's reading. `direction` is the value the mission acts on."""

    direction: float
    centroid_x_norm: float
    velocity_px_s: float
    blocked_fraction: float
    period_s: float
    clear_elapsed_s: float
    blocked: bool

    def as_list(self) -> list[float]:
        return [
            self.direction,
            self.centroid_x_norm,
            self.velocity_px_s,
            self.blocked_fraction,
            self.period_s,
            self.clear_elapsed_s,
            # Additive (2026-09-11, for mission_manager.py's
            # SlidingDoorController): the exact hysteresis-debounced
            # blocked/clear decision this tracker already makes internally -
            # a consumer re-deriving it from blocked_fraction alone would
            # need to duplicate BLOCKED_FRACTION_ENTER/EXIT's hysteresis to
            # avoid chatter right at the boundary. Existing consumers
            # reading only indices 0-5 are unaffected.
            1.0 if self.blocked else 0.0,
        ]


class DoorTracker:
    """Track a horizontally sliding door in a fixed gate ROI."""

    def __init__(
        self,
        motion_threshold: int = MOTION_THRESHOLD,
        min_motion_fraction: float = MIN_MOTION_FRACTION,
        min_speed_px_s: float = MIN_SPEED_PX_S,
        history: int = HISTORY,
    ) -> None:
        self.motion_threshold = motion_threshold
        self.min_motion_fraction = min_motion_fraction
        self.min_speed_px_s = min_speed_px_s
        self.prev_gray: np.ndarray | None = None
        self.centroids: deque[tuple[float, float]] = deque(maxlen=history)
        self.blocked = False
        self.clear_since: float | None = None
        self.last_clear_start: float | None = None
        self.periods: deque[float] = deque(maxlen=5)
        self.last_motion_mask: np.ndarray | None = None

    def update(self, roi_bgr: np.ndarray, timestamp: float) -> DoorState:
        gray = cv2.GaussianBlur(cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)

        direction = DIRECTION_UNKNOWN
        centroid_norm = 0.0
        velocity = 0.0

        if self.prev_gray is not None and self.prev_gray.shape == gray.shape:
            diff = cv2.absdiff(gray, self.prev_gray)
            _, motion = cv2.threshold(diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
            motion = cv2.morphologyEx(motion, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            self.last_motion_mask = motion

            if float(motion.mean()) / 255.0 > self.min_motion_fraction:
                columns = motion.sum(axis=0).astype(np.float64)
                total = columns.sum()
                if total > 0:
                    width = motion.shape[1]
                    centroid_px = float((columns * np.arange(width)).sum() / total)
                    centroid_norm = centroid_px / width
                    self.centroids.append((timestamp, centroid_px))
                    velocity = self._velocity_px_s()
                    if abs(velocity) >= self.min_speed_px_s:
                        direction = DIRECTION_RIGHT if velocity > 0 else DIRECTION_LEFT
            else:
                # A stale centroid history would keep reporting a direction for
                # a door that has already stopped or left the frame.
                self.centroids.clear()

        self.prev_gray = gray

        blocked_fraction = self.blocked_fraction(gray)
        self._update_cycle(blocked_fraction, timestamp)

        clear_elapsed = (
            0.0 if (self.blocked or self.clear_since is None) else timestamp - self.clear_since
        )
        period = float(np.mean(self.periods)) if self.periods else 0.0

        return DoorState(
            direction=direction,
            centroid_x_norm=centroid_norm,
            velocity_px_s=velocity,
            blocked_fraction=blocked_fraction,
            period_s=period,
            clear_elapsed_s=clear_elapsed,
            blocked=self.blocked,
        )

    def _velocity_px_s(self) -> float:
        """Least-squares slope of centroid-x against time.

        Fitted over several frames rather than differencing two, so a single
        noisy centroid cannot flip the reported direction - which is the worst
        available failure, since direction decides whether the vehicle commits
        into the gate.
        """
        if len(self.centroids) < 4:
            return 0.0
        times = np.array([t for t, _ in self.centroids], dtype=np.float64)
        xs = np.array([x for _, x in self.centroids], dtype=np.float64)
        times -= times[0]
        if times[-1] <= 0:
            return 0.0
        slope, _ = np.polyfit(times, xs, 1)
        return float(slope)

    def blocked_fraction(self, gray: np.ndarray) -> float:
        """Fraction of gate columns occluded by the door panel.

        Physical model: an open gate looks *through* to the corridor beyond,
        so each column spans real depth structure - ground, horizon, barriers -
        and varies a lot from top to bottom. A solid door panel occludes all of
        that and flattens the column to near-uniform. So occlusion reads as LOW
        vertical variance, not high edge density.

        Deliberately a variance measure rather than colour or edges: the door's
        colour is unknown, and an edge-density test gets this backwards - a
        panel's only edges are its two vertical boundaries.

        Lidar remains authoritative for whether the gate is actually
        passable. This is the camera's cross-check and the input to the period
        estimate; the camera's unique contribution is direction, not occupancy.
        """
        column_variation = gray.astype(np.float32).std(axis=0)
        return float((column_variation < UNIFORM_COLUMN_STD).mean())

    def _update_cycle(self, blocked_fraction: float, timestamp: float) -> None:
        if not self.blocked and blocked_fraction >= BLOCKED_FRACTION_ENTER:
            self.blocked = True
            self.clear_since = None
        elif self.blocked and blocked_fraction <= BLOCKED_FRACTION_EXIT:
            self.blocked = False
            self.clear_since = timestamp
            if self.last_clear_start is not None:
                period = timestamp - self.last_clear_start
                # Reject implausible periods rather than poisoning the mean.
                if 0.5 <= period <= 30.0:
                    self.periods.append(period)
            self.last_clear_start = timestamp
