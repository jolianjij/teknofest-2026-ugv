from __future__ import annotations

from ugv_stage_sign_detector.mission import StageSequence


def test_strict_sequence_accepts_only_expected_stage() -> None:
    gate = StageSequence([1, 2, 3], ["water", "rock", "slope"])
    rejected = gate.observe(2)
    assert rejected.accepted is False
    assert rejected.reason == "out_of_order_expected_1"
    first = gate.observe(1)
    assert first.accepted is True
    assert first.behavior == "water"
    duplicate = gate.observe(1)
    assert duplicate.reason == "duplicate"
    second = gate.observe(2)
    assert second.accepted is True
    assert second.previous_stage == 1
    assert second.current_stage == 2


def test_non_strict_sequence_supports_mid_course_restart() -> None:
    gate = StageSequence([1, 2, 3], ["water", "rock", "slope"], strict=False)
    result = gate.observe(3)
    assert result.accepted is True
    assert result.behavior == "slope"


def test_reset_returns_to_initial_stage_for_a_second_attempt() -> None:
    gate = StageSequence([1, 2, 3], ["water", "rock", "slope"])
    gate.observe(1)
    gate.observe(2)
    assert gate.current_stage == 2

    gate.reset()
    assert gate.current_stage == 0

    # A second attempt must be able to start from sign 1 again, not be
    # rejected as out-of-order against the stage the previous attempt reached.
    result = gate.observe(1)
    assert result.accepted is True
    assert result.behavior == "water"


def test_reset_honors_a_non_zero_initial_stage() -> None:
    gate = StageSequence([1, 2, 3], ["water", "rock", "slope"], initial_stage=1)
    gate.observe(2)
    gate.reset()
    assert gate.current_stage == 1
