"""Convert confirmed signs into validated mission-stage transitions."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, UInt8

from ugv_stage_interfaces.msg import StageSign, StageTransition

from .mission import StageSequence


DEFAULT_BEHAVIORS = [
    "fording_and_shower",
    "aggregate_road",
    "side_slope",
    "vertical_obstacle",
    "traffic_cones",
    "sliding_doors",
    "bumped_road",
    "incline_stop",
    "fire_zone",
    "decline_stop",
]

# Label zero is the classifier's reject class. The detector only publishes it
# for candidates that already passed the red-ring geometry checks, so a
# confirmed zero means "a sign is there and we could not read it" - not "no
# sign". Driving on would mean ignoring an instruction we know exists.
UNKNOWN_SIGN_ID = 0
UNKNOWN_BEHAVIOR = "stop_unknown_sign"


class StageManagerNode(Node):
    """Validate sign order and expose a stable behavior-transition event.

    This node owns mission state but not behavior implementation. Its transition
    topic is stable even if the teammate later changes motor controllers,
    navigation software, or the specific ROS Action server.
    """

    def __init__(self) -> None:
        super().__init__("stage_manager")
        self.declare_parameter("stage_sequence", list(range(1, 11)))
        self.declare_parameter("stage_behaviors", DEFAULT_BEHAVIORS)
        self.declare_parameter("strict_sequence", True)
        self.declare_parameter("initial_stage", 0)
        self.declare_parameter("unknown_stops_vehicle", True)
        self.declare_parameter("unknown_behavior", UNKNOWN_BEHAVIOR)

        self.unknown_stops_vehicle = bool(
            self.get_parameter("unknown_stops_vehicle").value
        )
        self.unknown_behavior = str(self.get_parameter("unknown_behavior").value)

        sequence = [int(value) for value in self.get_parameter("stage_sequence").value]
        behaviors = [str(value) for value in self.get_parameter("stage_behaviors").value]
        self.gate = StageSequence(
            sequence,
            behaviors,
            strict=bool(self.get_parameter("strict_sequence").value),
            initial_stage=int(self.get_parameter("initial_stage").value),
        )

        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # TRANSIENT_LOCAL makes the latest current stage immediately available
        # to dashboards or control nodes that start after the mission began.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.transition_publisher = self.create_publisher(
            StageTransition,
            "/mission/stage_transition",
            event_qos,
        )
        self.current_stage_publisher = self.create_publisher(
            UInt8,
            "/mission/current_stage",
            state_qos,
        )
        self.subscription = self.create_subscription(
            StageSign,
            "/perception/stage_sign/confirmed",
            self._confirmed_callback,
            event_qos,
        )
        # Vehicle-control side (mission_manager.py) publishes this once per
        # fresh autonomous engagement, so a second attempt in the same
        # process lifetime (disarm, reposition, re-arm) doesn't get stuck
        # thinking it's still on whatever stage the previous attempt reached.
        # This does NOT publish a StageTransition - a reset is not a
        # sign-triggered behavior change, so it must not cause the action
        # dispatcher to send an ExecuteStage goal for a nonexistent "stage 0".
        self.reset_subscription = self.create_subscription(
            Empty,
            "/mission/reset_stage_sequence",
            self._reset_callback,
            event_qos,
        )
        self._publish_current_stage()

    def _reset_callback(self, _message: Empty) -> None:
        previous = self.gate.current_stage
        self.gate.reset()
        if self.gate.current_stage == previous:
            return
        self.get_logger().info(f"Resetting stage sequence: {previous} -> {self.gate.current_stage}")
        self._publish_current_stage()

    def _confirmed_callback(self, message: StageSign) -> None:
        observed = int(message.sign_id)

        if observed == UNKNOWN_SIGN_ID:
            if not self.unknown_stops_vehicle:
                self.get_logger().warning(
                    "Unreadable sign confirmed; unknown_stops_vehicle is disabled, continuing"
                )
                return
            # Deliberately does NOT advance the sequence: we still do not know
            # which stage this is, so the mission stays where it was and the
            # vehicle halts until an operator or a readable sign resolves it.
            self._publish_transition(
                header=message.header,
                previous_stage=self.gate.current_stage,
                current_stage=self.gate.current_stage,
                behavior=self.unknown_behavior,
                confidence=message.confidence,
            )
            self.get_logger().error(
                "Unreadable sign confirmed -> stopping "
                f"(geometry confidence={message.confidence:.3f}, "
                f"stage stays {self.gate.current_stage})"
            )
            return

        decision = self.gate.observe(observed)
        if not decision.accepted:
            if decision.reason not in ("duplicate", "sequence_complete"):
                self.get_logger().warning(
                    f"Ignoring sign {observed}: {decision.reason.replace('_', ' ')}"
                )
            return
        self._publish_transition(
            header=message.header,
            previous_stage=decision.previous_stage,
            current_stage=decision.current_stage,
            behavior=decision.behavior,
            confidence=message.confidence,
        )
        self._publish_current_stage()
        self.get_logger().info(
            f"Stage {decision.previous_stage} -> {decision.current_stage}: {decision.behavior} "
            f"(confidence={message.confidence:.3f})"
        )

    def _publish_transition(
        self,
        header,
        previous_stage: int,
        current_stage: int,
        behavior: str,
        confidence: float,
    ) -> None:
        transition = StageTransition()
        transition.header = header
        transition.previous_stage = previous_stage
        transition.current_stage = current_stage
        transition.behavior = behavior
        transition.confidence = confidence
        self.transition_publisher.publish(transition)

    def _publish_current_stage(self) -> None:
        state = UInt8()
        state.data = self.gate.current_stage
        self.current_stage_publisher.publish(state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StageManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
