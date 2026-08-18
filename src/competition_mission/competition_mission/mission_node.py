#!/usr/bin/env python3
"""ROS adapter for the pure competition mission state machine."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from competition_interfaces.action import ArmTask
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
import yaml

from competition_mission.mission_config import mission_config_from_dict
from competition_mission.mission_state_machine import (
    ArmOutcome,
    ArmResult,
    ArmTaskRequest,
    CheckpointReady,
    CommandType,
    CompetitionMissionStateMachine,
    FlagDetected,
    LightObservation,
    LightSample,
    MarkerPassed,
    MissionCommand,
    MissionDecision,
    StableLight,
    Tick,
)


def _state_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class CompetitionMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("competition_mission")
        config_file = Path(
            str(self.declare_parameter("mission_config_file", "").value)
        ).expanduser()
        if not config_file.is_file():
            raise FileNotFoundError(
                f"mission_config_file does not exist: {config_file}"
            )
        with config_file.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("mission configuration must be a YAML mapping")
        self._machine = CompetitionMissionStateMachine(
            mission_config_from_dict(document)
        )
        self._last_ready_key: tuple[str, str] | None = None
        self._last_reason = "waiting_for_start_flag"
        self._arm_feedback: dict[str, Any] = {}
        self._goal_handles: dict[str, Any] = {}
        self._cancelled_task_ids: set[str] = set()

        self._route_enable_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "route_enable_topic", "/mission/route_enable"
                ).value
            ),
            _state_qos(),
        )
        self._checkpoint_release_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "checkpoint_release_topic",
                    "/mission/checkpoint_release",
                ).value
            ),
            10,
        )
        self._traffic_enable_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "traffic_light_enable_topic",
                    "/perception/traffic_light_enable",
                ).value
            ),
            10,
        )
        self._traffic_stop_enable_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "traffic_stop_enable_topic",
                    "/perception/traffic_stop_enable",
                ).value
            ),
            _state_qos(),
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "mission_status_topic", "/mission/status"
                ).value
            ),
            _state_qos(),
        )

        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "flag_event_topic", "/perception/flag_wave_detected"
                ).value
            ),
            self._flag_callback,
            _state_qos(),
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "marker_topic", "/mission/marker_passed"
                ).value
            ),
            self._marker_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "control_status_topic", "/control/status"
                ).value
            ),
            self._control_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "traffic_detection_topic",
                    "/perception/traffic_light_detection",
                ).value
            ),
            self._traffic_detection_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "traffic_state_topic", "/perception/traffic_light_state"
                ).value
            ),
            self._traffic_state_callback,
            10,
        )

        self._arm_client = ActionClient(
            self,
            ArmTask,
            str(
                self.declare_parameter(
                    "arm_action_name", "/mission/arm_task"
                ).value
            ),
        )
        tick_period_s = float(self.declare_parameter("tick_period_s", 0.10).value)
        if tick_period_s <= 0.0:
            raise ValueError("tick_period_s must be positive")
        self.create_timer(tick_period_s, self._tick)

        self._route_enable_publisher.publish(Bool(data=False))
        self._traffic_enable_publisher.publish(Bool(data=False))
        self._traffic_stop_enable_publisher.publish(Bool(data=False))
        self._publish_status(self._machine.snapshot)
        self.get_logger().info(
            f"Mission ready in {self._machine.snapshot.state.value}; "
            f"config={config_file}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _flag_callback(self, message: Bool) -> None:
        if message.data:
            self._handle_event(FlagDetected(now_s=self._now_s()))

    def _marker_callback(self, message: String) -> None:
        marker_ref = str(message.data).strip()
        if marker_ref:
            self._handle_event(
                MarkerPassed(now_s=self._now_s(), marker_ref=marker_ref)
            )

    def _control_status_callback(self, message: String) -> None:
        payload = _json_mapping(message.data)
        if payload is None:
            self.get_logger().warning("Ignored malformed control status")
            return
        phase = str(payload.get("mission_phase", ""))
        checkpoint_ref = str(payload.get("active_checkpoint_ref", "")).strip()
        if phase not in {"WAIT_RELEASE", "ROUTE_COMPLETED"} or not checkpoint_ref:
            self._last_ready_key = None
            return
        ready_key = (phase, checkpoint_ref)
        if ready_key == self._last_ready_key:
            return
        self._last_ready_key = ready_key
        self._handle_event(
            CheckpointReady(
                now_s=self._now_s(),
                checkpoint_ref=checkpoint_ref,
            )
        )

    def _traffic_detection_callback(self, message: String) -> None:
        payload = _json_mapping(message.data)
        if payload is None or not bool(payload.get("active", False)):
            return
        self._handle_event(
            LightSample(
                now_s=self._now_s(),
                light=_light_observation(payload.get("class_name")),
            )
        )

    def _traffic_state_callback(self, message: String) -> None:
        light = _light_observation(message.data)
        if light == LightObservation.GREEN:
            self._handle_event(StableLight(now_s=self._now_s(), light=light))

    def _tick(self) -> None:
        self._handle_event(Tick(now_s=self._now_s()))

    def _handle_event(self, event) -> None:
        decision = self._machine.handle(event)
        self._execute_commands(decision.commands)
        self._publish_status(self._machine.snapshot)

    def _execute_commands(self, commands: tuple[MissionCommand, ...]) -> None:
        for command in commands:
            if command.reason:
                self._last_reason = command.reason
            if command.command_type == CommandType.SET_ROUTE_ENABLED:
                self._route_enable_publisher.publish(
                    Bool(data=bool(command.enabled))
                )
            elif command.command_type == CommandType.SET_TRAFFIC_LIGHT_ENABLED:
                self._traffic_enable_publisher.publish(
                    Bool(data=bool(command.enabled))
                )
            elif command.command_type == CommandType.SET_TRAFFIC_STOP_ENABLED:
                self._traffic_stop_enable_publisher.publish(
                    Bool(data=bool(command.enabled))
                )
            elif command.command_type == CommandType.RELEASE_TO_CHECKPOINT:
                assert command.checkpoint_ref is not None
                self._checkpoint_release_publisher.publish(
                    String(data=command.checkpoint_ref)
                )
            elif command.command_type == CommandType.START_ARM_TASK:
                assert command.arm_task is not None
                self._start_arm_task(command.arm_task)
            elif command.command_type == CommandType.CANCEL_ARM_TASK:
                assert command.task_id is not None
                self._cancel_arm_task(command.task_id)
            elif command.command_type == CommandType.MISSION_FINISHED:
                self._route_enable_publisher.publish(Bool(data=False))
                self._traffic_enable_publisher.publish(Bool(data=False))
                self._traffic_stop_enable_publisher.publish(Bool(data=False))

    def _start_arm_task(self, request: ArmTaskRequest) -> None:
        if not self._arm_client.server_is_ready():
            self.get_logger().error(
                f"ArmTask server unavailable for {request.task_id}"
            )
            self._handle_event(
                ArmResult(
                    now_s=self._now_s(),
                    task_id=request.task_id,
                    outcome=ArmOutcome.OPERATION_FAILED,
                    target_type=request.target_type_hint,
                )
            )
            return
        goal = ArmTask.Goal()
        goal.task_type = (
            ArmTask.Goal.PICKUP
            if request.task_type.value == "PICKUP"
            else ArmTask.Goal.DROP
        )
        goal.station = (
            ArmTask.Goal.FRONT
            if request.station.value == "FRONT"
            else ArmTask.Goal.REAR
        )
        goal.task_id = request.task_id
        goal.target_type_hint = request.target_type_hint
        goal.max_attempts = request.max_attempts
        goal.timeout_s = request.timeout_s
        future = self._arm_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, task_id=request.task_id: (
                self._arm_feedback_callback(task_id, feedback)
            ),
        )
        future.add_done_callback(
            lambda result, task_id=request.task_id: self._arm_goal_response(
                task_id, result
            )
        )

    def _arm_goal_response(self, task_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"ArmTask goal failed: {exc}")
            self._arm_failed(task_id, "goal_request_failed")
            return
        if not goal_handle.accepted:
            self._arm_failed(task_id, "goal_rejected")
            return
        self._goal_handles[task_id] = goal_handle
        if (
            task_id in self._cancelled_task_ids
            or self._machine.snapshot.active_arm_task_id != task_id
        ):
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, accepted_task_id=task_id: self._arm_result_callback(
                accepted_task_id, result
            )
        )

    def _arm_feedback_callback(self, task_id: str, feedback_message) -> None:
        feedback = feedback_message.feedback
        self._arm_feedback = {
            "task_id": task_id,
            "phase": int(feedback.phase),
            "target_type": str(feedback.target_type),
            "attempt": int(feedback.attempt),
        }

    def _arm_result_callback(self, task_id: str, future) -> None:
        self._goal_handles.pop(task_id, None)
        self._cancelled_task_ids.discard(task_id)
        try:
            result = future.result().result
            outcome = _arm_outcome(int(result.outcome))
            target_type = str(result.target_type)
        except Exception as exc:
            self.get_logger().error(f"ArmTask result failed: {exc}")
            outcome = ArmOutcome.OPERATION_FAILED
            target_type = ""
        self._handle_event(
            ArmResult(
                now_s=self._now_s(),
                task_id=task_id,
                outcome=outcome,
                target_type=target_type,
            )
        )

    def _arm_failed(self, task_id: str, reason: str) -> None:
        self.get_logger().error(f"ArmTask {task_id} failed: {reason}")
        self._handle_event(
            ArmResult(
                now_s=self._now_s(),
                task_id=task_id,
                outcome=ArmOutcome.OPERATION_FAILED,
            )
        )

    def _cancel_arm_task(self, task_id: str) -> None:
        self._cancelled_task_ids.add(task_id)
        goal_handle = self._goal_handles.get(task_id)
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def _publish_status(self, decision: MissionDecision) -> None:
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": decision.state.value,
                        "has_cargo": decision.has_cargo,
                        "target_type": decision.target_type,
                        "active_arm_task_id": decision.active_arm_task_id,
                        "arm_feedback": self._arm_feedback,
                        "finished": decision.finished,
                        "reason": self._last_reason,
                    },
                    separators=(",", ":"),
                )
            )
        )


def _json_mapping(text: str) -> dict[str, Any] | None:
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


def _light_observation(value: Any) -> LightObservation:
    try:
        return LightObservation(str(value or "").strip().lower())
    except ValueError:
        return LightObservation.UNKNOWN


def _arm_outcome(value: int) -> ArmOutcome:
    return {
        ArmTask.Result.SUCCESS: ArmOutcome.SUCCESS,
        ArmTask.Result.INSTRUCTION_NOT_FOUND: ArmOutcome.INSTRUCTION_NOT_FOUND,
        ArmTask.Result.TARGET_NOT_FOUND: ArmOutcome.TARGET_NOT_FOUND,
        ArmTask.Result.OPERATION_FAILED: ArmOutcome.OPERATION_FAILED,
        ArmTask.Result.TIMEOUT: ArmOutcome.TIMEOUT,
    }.get(value, ArmOutcome.OPERATION_FAILED)


def main() -> None:
    rclpy.init()
    node = CompetitionMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._route_enable_publisher.publish(Bool(data=False))
            node._traffic_enable_publisher.publish(Bool(data=False))
            node._traffic_stop_enable_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
