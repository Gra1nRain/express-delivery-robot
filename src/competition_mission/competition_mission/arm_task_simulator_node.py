#!/usr/bin/env python3
"""No-motion ArmTask server for offline mission integration only."""

from __future__ import annotations

import asyncio

from competition_interfaces.action import ArmTask
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node


class ArmTaskSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_task_simulator")
        self._phase_delay_s = float(
            self.declare_parameter("phase_delay_s", 0.05).value
        )
        if self._phase_delay_s < 0.0:
            raise ValueError("phase_delay_s must be non-negative")
        self._outcome_name = str(
            self.declare_parameter("outcome", "SUCCESS").value
        ).strip().upper()
        self._target_type = str(
            self.declare_parameter("target_type", "green_bottle").value
        ).strip()
        self._active = False
        self._server = ActionServer(
            self,
            ArmTask,
            str(self.declare_parameter("action_name", "/mission/arm_task").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self.get_logger().warning(
            "ArmTask simulator ready: it never commands the physical arm"
        )

    def _goal(self, goal_request) -> GoalResponse:
        if self._active:
            return GoalResponse.REJECT
        if goal_request.task_type not in {
            ArmTask.Goal.PICKUP,
            ArmTask.Goal.DROP,
        }:
            return GoalResponse.REJECT
        self._active = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        try:
            phases = [
                ArmTask.Feedback.MOVING_TO_INSTRUCTION_POSE,
                ArmTask.Feedback.RECOGNIZING_INSTRUCTION,
                ArmTask.Feedback.TARGET_TYPE_LOCKED,
                ArmTask.Feedback.SEARCHING_TARGET_OBJECT,
                ArmTask.Feedback.OPERATING,
                ArmTask.Feedback.VERIFYING_OPERATION,
            ]
            target_type = (
                goal_handle.request.target_type_hint or self._target_type
            )
            for phase in phases:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(ArmTask.Result.TIMEOUT, target_type)
                feedback = ArmTask.Feedback()
                feedback.phase = phase
                feedback.target_type = target_type
                feedback.attempt = 1
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(self._phase_delay_s)

            outcome = {
                "SUCCESS": ArmTask.Result.SUCCESS,
                "INSTRUCTION_NOT_FOUND": ArmTask.Result.INSTRUCTION_NOT_FOUND,
                "TARGET_NOT_FOUND": ArmTask.Result.TARGET_NOT_FOUND,
                "OPERATION_FAILED": ArmTask.Result.OPERATION_FAILED,
                "TIMEOUT": ArmTask.Result.TIMEOUT,
            }.get(self._outcome_name, ArmTask.Result.OPERATION_FAILED)
            if outcome == ArmTask.Result.SUCCESS:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return self._result(outcome, target_type)
        finally:
            self._active = False

    @staticmethod
    def _result(outcome: int, target_type: str):
        result = ArmTask.Result()
        result.outcome = outcome
        result.target_type = target_type
        result.detail = "offline_simulator_no_physical_motion"
        return result

    def destroy_node(self) -> bool:
        self._server.destroy()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArmTaskSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
