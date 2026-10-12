"""Repeated-frame confirmation for safe one-shot stage events."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import replace

from .config import TemporalConfig
from .detector import SignDetection


class TemporalConfirmer:
    """Debounce detections and emit one event per physical sign.

    A single frame can be wrong because of glare, vibration, blur, or a red
    barrier pattern. Confirmation requires a majority-like number of hits in a
    short history. After emitting, that label is locked until it has disappeared
    for several frames, preventing hundreds of repeated ROS action goals.
    """

    def __init__(self, config: TemporalConfig | None = None) -> None:
        self.config = config or TemporalConfig()
        self.history: deque[SignDetection | None] = deque(maxlen=self.config.history_size)
        self.locked_sign: int | None = None
        self._locked_absent_frames = 0

    def update(self, detection: SignDetection | None) -> SignDetection | None:
        """Add one frame observation and possibly return a confirmed detection."""
        valid = detection if detection and detection.confidence >= self.config.min_confidence else None
        self.history.append(valid)

        # While locked, do not emit again. Rearm only after the physical sign is
        # no longer visible (or a different sign has occupied the view) long enough.
        if self.locked_sign is not None:
            if valid is None or valid.sign_id != self.locked_sign:
                self._locked_absent_frames += 1
            else:
                self._locked_absent_frames = 0
            if self._locked_absent_frames >= self.config.disappear_frames_to_rearm:
                self.locked_sign = None
                self._locked_absent_frames = 0
            else:
                return None

        candidates = [item for item in self.history if item is not None]
        if not candidates:
            return None
        # Count label agreement within the sliding window, then return the best
        # bounding box/confidence among the agreeing frames.
        counts = Counter(item.sign_id for item in candidates)
        sign_id, hits = counts.most_common(1)[0]
        if hits < self.config.required_hits:
            return None

        matching = [item for item in candidates if item.sign_id == sign_id]
        best = max(matching, key=lambda item: item.confidence)
        self.locked_sign = sign_id
        self._locked_absent_frames = 0
        self.history.clear()
        return replace(best, confirmed=True)

    def reset(self) -> None:
        self.history.clear()
        self.locked_sign = None
        self._locked_absent_frames = 0
