"""Safe mock ``ExecuteStage`` Action server for integration testing.

This node does not control hardware. It simulates progress for each behavior so
the complete camera -> detection -> stage -> ROS Action chain can be tested
before the vehicle-control teammate's real action server is available.
"""

from __future__ import annotations

import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from ugv_stage_interfaces.action import ExecuteStage


class MockStageActionServer(Node):
    def __init__(self) -> None:
        super().__init__("mock_stage_action_server")
        self.declare_parameter("action_name", "/mission/execute_stage")
        self.declare_parameter("duration_seconds", 2.0)
        action_name = str(self.get_parameter("action_name").value)
        self.duration = max(0.1, float(self.get_parameter("duration_seconds").value))
        self.server = ActionServer(
            self,
            ExecuteStage,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.get_logger().warning(
            "Mock stage server is active: no motor, brake, steering, or laser commands are sent"
        )

    def _accept_goal(self, goal_request: ExecuteStage.Goal) -> GoalResponse:
        if not goal_request.behavior:
            self.get_logger().warning("Rejecting a stage goal with an empty behavior")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"Mock accepted stage {goal_request.stage_id}: {goal_request.behavior}"
        )
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_: object) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle: object) -> ExecuteStage.Result:
        steps = 20
        for step in range(steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteStage.Result()
                result.success = False
                result.message = "Cancelled because a newer stage became active"
                return result

            feedback = ExecuteStage.Feedback()
            feedback.progress = step / steps
            feedback.state = f"simulating {goal_handle.request.behavior}"
            goal_handle.publish_feedback(feedback)
            if step < steps:
                await asyncio.sleep(self.duration / steps)

        goal_handle.succeed()
        result = ExecuteStage.Result()
        result.success = True
        result.message = f"Mock completed {goal_handle.request.behavior}"
        return result

    def destroy_node(self) -> bool:
        self.server.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockStageActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
