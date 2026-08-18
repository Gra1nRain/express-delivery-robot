#!/usr/bin/env python3
"""Persistent real-Piper ArmTask action server for the competition mission."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import subprocess
import sys
from threading import Lock

from competition_interfaces.action import ArmTask
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from competition_mission.arm_task_runner import (
    ArmTaskOutcome,
    ArmTaskPhase,
    ArmTaskRequest,
    ArmTaskRunner,
    ArmTaskType,
)
from competition_mission.piper_arm_backend import (
    PiperMigrationBackend,
    create_competition_piper_controller,
    load_piper_modules,
)


_PHASE_TO_ACTION = {
    ArmTaskPhase.MOVING_TO_INSTRUCTION_POSE: (
        ArmTask.Feedback.MOVING_TO_INSTRUCTION_POSE
    ),
    ArmTaskPhase.RECOGNIZING_INSTRUCTION: (
        ArmTask.Feedback.RECOGNIZING_INSTRUCTION
    ),
    ArmTaskPhase.TARGET_TYPE_LOCKED: ArmTask.Feedback.TARGET_TYPE_LOCKED,
    ArmTaskPhase.SEARCHING_TARGET_OBJECT: (
        ArmTask.Feedback.SEARCHING_TARGET_OBJECT
    ),
    ArmTaskPhase.OPERATING: ArmTask.Feedback.OPERATING,
    ArmTaskPhase.VERIFYING_OPERATION: ArmTask.Feedback.VERIFYING_OPERATION,
}

_OUTCOME_TO_ACTION = {
    ArmTaskOutcome.SUCCESS: ArmTask.Result.SUCCESS,
    ArmTaskOutcome.INSTRUCTION_NOT_FOUND: ArmTask.Result.INSTRUCTION_NOT_FOUND,
    ArmTaskOutcome.TARGET_NOT_FOUND: ArmTask.Result.TARGET_NOT_FOUND,
    ArmTaskOutcome.OPERATION_FAILED: ArmTask.Result.OPERATION_FAILED,
    ArmTaskOutcome.TIMEOUT: ArmTask.Result.TIMEOUT,
}

_OVERLAY_PATH_VARIABLES = (
    "AMENT_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PYTHONPATH",
)


def _migration_root_from_environment() -> Path:
    competition_ws = os.environ.get(
        "COMPETITION_WS",
        "/home/agilex/competition_ws",
    )
    return Path(
        os.environ.get(
            "PIPER_MIGRATION_ROOT",
            os.path.join(
                competition_ws,
                "Piper_Grasp_Humble_Migration_20260723",
            ),
        )
    ).expanduser()


def _ensure_piper_overlay() -> None:
    if os.environ.get("COMPETITION_PIPER_OVERLAY_READY") == "1":
        return
    migration_root = _migration_root_from_environment()
    piper_setup = Path(
        os.environ.get(
            "PIPER_SETUP",
            str(
                migration_root
                / "drivers"
                / "piper_ros"
                / "install"
                / "setup.bash"
            ),
        )
    ).expanduser()
    if not piper_setup.is_file():
        raise FileNotFoundError(
            "Piper ROS overlay is not built: "
            f"{piper_setup}; run build_piper_humble.sh first"
        )

    resolved_prefix = str(piper_setup.parent.resolve())
    prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    if resolved_prefix in prefixes:
        os.environ["COMPETITION_PIPER_OVERLAY_READY"] = "1"
        return

    ros_setup = Path(
        os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
    ).expanduser()
    if not ros_setup.is_file():
        raise FileNotFoundError(f"ROS setup not found: {ros_setup}")
    command = (
        f"source {shlex.quote(str(ros_setup))} && "
        f"source {shlex.quote(str(piper_setup))} && env -0"
    )
    completed = subprocess.run(
        ["bash", "-lc", command],
        check=True,
        capture_output=True,
    )
    environment = os.environ.copy()
    for item in completed.stdout.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        environment[key.decode()] = value.decode(errors="surrogateescape")
    for variable in _OVERLAY_PATH_VARIABLES:
        entries = environment.get(variable, "").split(":")
        environment[variable] = ":".join(
            entry for entry in entries if "/agx_arm_ws/" not in entry
        )
    environment["COMPETITION_PIPER_OVERLAY_READY"] = "1"
    os.execve(
        sys.executable,
        [
            sys.executable,
            "-m",
            "competition_mission.piper_arm_task_node",
            *sys.argv[1:],
        ],
        environment,
    )


class PiperArmTaskNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_arm_task")
        competition_ws = os.environ.get(
            "COMPETITION_WS",
            "/home/agilex/competition_ws",
        )
        migration_root = Path(
            str(
                self.declare_parameter(
                    "migration_root",
                    os.path.join(
                        competition_ws,
                        "Piper_Grasp_Humble_Migration_20260723",
                    ),
                ).value
            )
        )
        grasp_module, place_module = load_piper_modules(migration_root)
        self.piper_controller = create_competition_piper_controller(
            grasp_module,
            manage_camera=bool(
                self.declare_parameter("manage_camera", True).value
            ),
        )
        backend = PiperMigrationBackend(
            self.piper_controller,
            grasp_module,
            place_module,
            readiness_timeout_s=float(
                self.declare_parameter("readiness_timeout_s", 30.0).value
            ),
            feedback_timeout_s=float(
                self.declare_parameter("gripper_feedback_timeout_s", 2.0).value
            ),
            min_pickup_opening_m=float(
                self.declare_parameter(
                    "min_pickup_opening_m",
                    0.002,
                ).value
            ),
            min_drop_opening_m=float(
                self.declare_parameter(
                    "min_drop_opening_m",
                    0.030,
                ).value
            ),
            post_instruction_clear_delay_s=float(
                self.declare_parameter(
                    "post_instruction_clear_delay_s",
                    0.0,
                ).value
            ),
        )
        self._runner = ArmTaskRunner(backend)
        self._active = False
        self._active_lock = Lock()
        self._action_server = ActionServer(
            self,
            ArmTask,
            str(
                self.declare_parameter(
                    "action_name",
                    "/mission/arm_task",
                ).value
            ),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            "Persistent Piper ArmTask server ready; "
            "PICKUP and DROP are independently supervised"
        )

    def _goal(self, request) -> GoalResponse:
        valid = (
            request.task_type in {ArmTask.Goal.PICKUP, ArmTask.Goal.DROP}
            and request.station in {ArmTask.Goal.FRONT, ArmTask.Goal.REAR}
            and bool(str(request.task_id).strip())
            and int(request.max_attempts) > 0
            and float(request.timeout_s) > 0.0
        )
        if not valid:
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        request = goal_handle.request
        task_type = (
            ArmTaskType.PICKUP
            if request.task_type == ArmTask.Goal.PICKUP
            else ArmTaskType.DROP
        )
        try:
            result = self._runner.run(
                ArmTaskRequest(
                    task_type=task_type,
                    target_type_hint=str(request.target_type_hint),
                    max_attempts=int(request.max_attempts),
                    timeout_s=float(request.timeout_s),
                ),
                publish_feedback=lambda phase, target, attempt: (
                    self._publish_feedback(
                        goal_handle,
                        phase,
                        target,
                        attempt,
                    )
                ),
                cancel_requested=lambda: bool(
                    goal_handle.is_cancel_requested
                ),
            )
            if result.outcome == ArmTaskOutcome.SUCCESS:
                goal_handle.succeed()
            elif (
                result.outcome == ArmTaskOutcome.TIMEOUT
                and goal_handle.is_cancel_requested
            ):
                goal_handle.canceled()
            else:
                goal_handle.abort()
            message = ArmTask.Result()
            message.outcome = _OUTCOME_TO_ACTION[result.outcome]
            message.target_type = result.target_type
            message.detail = (
                f"attempts={result.attempts};{result.detail}"
            )
            return message
        finally:
            with self._active_lock:
                self._active = False

    @staticmethod
    def _publish_feedback(
        goal_handle,
        phase: ArmTaskPhase,
        target_type: str,
        attempt: int,
    ) -> None:
        feedback = ArmTask.Feedback()
        feedback.phase = _PHASE_TO_ACTION[phase]
        feedback.target_type = target_type
        feedback.attempt = int(attempt)
        goal_handle.publish_feedback(feedback)

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()


def main() -> None:
    _ensure_piper_overlay()
    rclpy.init()
    node = PiperArmTaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.piper_controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.piper_controller.cleanup()
        node.piper_controller.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
