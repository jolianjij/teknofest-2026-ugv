"""Course-order gate shared by ROS and unit tests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TransitionDecision:
    accepted: bool
    previous_stage: int
    current_stage: int
    behavior: str
    reason: str


class StageSequence:
    """ROS-independent sequential gate for confirmed sign events.

    Sign classification and temporal confirmation are necessary but not enough:
    accepting sign 8 directly after sign 2 could launch a dangerous wrong
    behavior. Strict mode only accepts the next configured stage. Non-strict
    mode exists for bench testing or intentionally restarting mid-course.
    """

    def __init__(
        self,
        sequence: list[int],
        behaviors: list[str],
        strict: bool = True,
        initial_stage: int = 0,
    ) -> None:
        if not sequence:
            raise ValueError("Stage sequence cannot be empty")
        if len(sequence) != len(behaviors):
            raise ValueError("One behavior is required for each stage")
        if len(set(sequence)) != len(sequence):
            raise ValueError("Stage sequence values must be unique")
        self.sequence = list(sequence)
        self.behavior_by_stage = dict(zip(sequence, behaviors, strict=True))
        self.strict = strict
        self._initial_stage = initial_stage
        self.current_stage = initial_stage

    def reset(self) -> None:
        """Return to the configured initial stage, e.g. at the start of a
        fresh autonomous-run attempt. Without this, a vehicle that is
        disarmed, repositioned, and re-armed for a second attempt keeps
        whatever stage it reached on the previous attempt and (in strict
        mode) rejects sign 1 as out of order."""
        self.current_stage = self._initial_stage

    def observe(self, sign_id: int) -> TransitionDecision:
        """Evaluate a confirmed sign without performing any vehicle action."""
        previous = self.current_stage
        if sign_id == previous:
            return TransitionDecision(False, previous, previous, "", "duplicate")
        if sign_id not in self.behavior_by_stage:
            return TransitionDecision(False, previous, previous, "", "unknown_stage")
        if self.strict:
            expected = self.expected_next()
            if expected is None:
                return TransitionDecision(False, previous, previous, "", "sequence_complete")
            if sign_id != expected:
                return TransitionDecision(
                    False,
                    previous,
                    previous,
                    "",
                    f"out_of_order_expected_{expected}",
                )
        self.current_stage = sign_id
        return TransitionDecision(
            True,
            previous,
            sign_id,
            self.behavior_by_stage[sign_id],
            "accepted",
        )

    def expected_next(self) -> int | None:
        if self.current_stage == 0:
            return self.sequence[0]
        try:
            index = self.sequence.index(self.current_stage)
        except ValueError:
            return self.sequence[0]
        return self.sequence[index + 1] if index + 1 < len(self.sequence) else None
