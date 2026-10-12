from __future__ import annotations

from ugv_stage_sign_detector.config import TemporalConfig
from ugv_stage_sign_detector.detector import BoundingBox, SignDetection
from ugv_stage_sign_detector.temporal import TemporalConfirmer


def detection(sign_id: int, confidence: float = 0.9) -> SignDetection:
    return SignDetection(
        sign_id=sign_id,
        confidence=confidence,
        bounding_box=BoundingBox(10, 20, 100, 100),
        circularity=0.9,
        red_fraction=0.7,
        white_fraction=0.8,
        classification_margin=0.2,
    )


def test_confirms_four_of_six_frames_and_emits_once() -> None:
    confirmer = TemporalConfirmer(
        TemporalConfig(history_size=6, required_hits=4, min_confidence=0.7)
    )
    outputs = [
        confirmer.update(detection(4)),
        confirmer.update(None),
        confirmer.update(detection(4)),
        confirmer.update(detection(3, 0.5)),
        confirmer.update(detection(4)),
        confirmer.update(detection(4)),
    ]
    confirmed = [item for item in outputs if item is not None]
    assert len(confirmed) == 1
    assert confirmed[0].sign_id == 4
    assert confirmed[0].confirmed is True
    assert confirmer.update(detection(4)) is None


def test_rearms_after_sign_disappears() -> None:
    config = TemporalConfig(
        history_size=4,
        required_hits=3,
        min_confidence=0.7,
        disappear_frames_to_rearm=3,
    )
    confirmer = TemporalConfirmer(config)
    assert confirmer.update(detection(2)) is None
    assert confirmer.update(detection(2)) is None
    assert confirmer.update(detection(2)) is not None
    for _ in range(3):
        assert confirmer.update(None) is None
    assert confirmer.update(detection(3)) is None
    assert confirmer.update(detection(3)) is None
    result = confirmer.update(detection(3))
    assert result is not None
    assert result.sign_id == 3


def test_default_threshold_does_not_lock_weak_classification() -> None:
    """A repeated 0.71 observation must not be allowed to trigger an action."""
    confirmer = TemporalConfirmer(TemporalConfig())
    outputs = [confirmer.update(detection(2, 0.71)) for _ in range(8)]
    assert all(item is None for item in outputs)
    assert confirmer.locked_sign is None
