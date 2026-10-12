"""Dispatch validated stage transitions through a ROS 2 Action client.

The stage manager publishes a transport-independent ``StageTransition`` event.
This adapter translates that event into the generic ``ExecuteStage`` Action.
If a new stage arrives while the previous action is active, the previous goal
is cancelled before the newest goal is sent. This matches a course where each
sign changes the vehicle's active driving behavior.

The vehicle-control teammate may implement an ``ExecuteStage`` server directly
or replace this adapter with one targeting their own action types.
"""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ugv_stage_interfaces.action import ExecuteStage
from ugv_stage_interfaces.msg import StageTransition


class StageActionDispatcherNode(Node):
    """Turn each validated mission transition into one ROS Action goal."""

    def __init__(self) -> None:
        super().__init__("stage_action_dispatcher")
        self.declare_parameter("action_name", "/mission/execute_stage")
        self.declare_parameter("retry_period_seconds", 0.25)
        self.declare_parameter("cancel_previous_goal", True)

        action_name = str(self.get_parameter("action_name").value)
        retry_period = float(self.get_parameter("retry_period_seconds").value)
        self.cancel_previous = bool(self.get_parameter("cancel_previous_goal").value)

        self.client = ActionClient(self, ExecuteStage, action_name)
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            StageTransition,
            "/mission/stage_transition",
            self._transition_callback,
            event_qos,
        )

        # ``pending`` always stores the newest transition. A course sign should
        # not be blocked forever merely because the action server starts later.
        self.pending: StageTransition | None = None
        self.dispatched: StageTransition | None = None
        self.active_goal_handle: Any | None = None
        self.send_future: Any | None = None
        self.cancel_requested = False
        self.server_warning_emitted = False
        self.timer = self.create_timer(max(0.05, retry_period), self._dispatch_if_ready)
        self.get_logger().info(f"Dispatching stage actions to {action_name}")

    def _transition_callback(self, message: StageTransition) -> None:
        """Keep the latest stage and cancel an obsolete active behavior."""

        self.pending = message
        if (
            self.cancel_previous
            and self.active_goal_handle is not None
            and not self.cancel_requested
        ):
            self.cancel_requested = True
            self.get_logger().info(
                f"Cancelling active stage before dispatching stage {message.current_stage}"
            )
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_response_callback)
        self._dispatch_if_ready()

    def _dispatch_if_ready(self) -> None:
        """Send a queued transition when no goal/send operation is active."""

        if self.pending is None:
            return
        if self.active_goal_handle is not None or self.send_future is not None:
            return
        if not self.client.server_is_ready():
            if not self.server_warning_emitted:
                self.get_logger().warning(
                    "Stage action server is unavailable; the latest transition remains queued"
                )
                self.server_warning_emitted = True
            return

        self.server_warning_emitted = False
        transition = self.pending
        self.pending = None
        self.dispatched = transition
        goal = ExecuteStage.Goal()
        goal.stage_id = transition.current_stage
        goal.behavior = transition.behavior
        self.get_logger().info(
            f"Sending action goal for stage {goal.stage_id}: {goal.behavior}"
        )
        self.send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        self.send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: Any) -> None:
        """Store an accepted goal handle and begin waiting for its result."""

        self.send_future = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Sending the stage action failed: {exc}")
            self.dispatched = None
            return
        if not goal_handle.accepted:
            self.get_logger().error("Stage action server rejected the goal")
            self.dispatched = None
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

        # A newer transition may have arrived while the goal request was in
        # flight. Cancel this now-obsolete goal as soon as its handle exists.
        if self.pending is not None and self.cancel_previous and not self.cancel_requested:
            self.cancel_requested = True
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_response_callback)

    def _feedback_callback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        self.get_logger().debug(
            f"Stage action feedback: {feedback.progress:.0%} - {feedback.state}"
        )

    def _cancel_response_callback(self, future: Any) -> None:
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warning("The active stage action could not be cancelled")
        except Exception as exc:
            self.get_logger().error(f"Stage action cancellation failed: {exc}")

    def _result_callback(self, future: Any) -> None:
        """Clear the active goal, report its outcome, then send any queued stage."""

        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            stage = self.dispatched.current_stage if self.dispatched else 0
            log = self.get_logger().info if result.success else self.get_logger().warning
            log(f"Stage {stage} action finished: {result.message}")
        except Exception as exc:
            self.get_logger().error(f"Reading the stage action result failed: {exc}")
        finally:
            self.active_goal_handle = None
            self.dispatched = None
            self.cancel_requested = False
            self._dispatch_if_ready()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StageActionDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
