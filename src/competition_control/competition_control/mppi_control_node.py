#!/usr/bin/env python3
"""ROS adapter from FAST-LIO/Ranger state to MPPI body commands."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)
from competition_control.mppi_controller import (
    BodyCommand,
    ControlTrajectory,
    MPPIController,
    MPPIParams,
    VehicleState,
)
from competition_localization.planar_transform import yaw_from_quaternion
from competition_localization.state_estimator import (
    Pose2D,
    StateEstimator,
    StateEstimatorLimits,
    StateObservation,
    Velocity2D,
)


class MPPIControlNode(Node):
    def __init__(self) -> None:
        super().__init__("mppi_control")
        trajectory_file = str(self.declare_parameter("trajectory_file", "").value)
        if not trajectory_file:
            raise ValueError("trajectory_file parameter is required")
        with Path(trajectory_file).open("r", encoding="utf-8") as stream:
            artifact = yaml.safe_load(stream)
        if not isinstance(artifact, dict):
            raise ValueError(f"trajectory_file is not a YAML mapping: {trajectory_file}")
        source_paths = resolve_trajectory_source_paths(
            route_file=str(self.declare_parameter("route_file", "").value),
            semantic_map_file=str(
                self.declare_parameter("semantic_map_file", "").value
            ),
            planning_params_file=str(
                self.declare_parameter("planning_params_file", "").value
            ),
            optimizer_params_file=str(
                self.declare_parameter("optimizer_params_file", "").value
            ),
        )
        validate_source_manifest(artifact, source_paths)
        if not artifact.get("ok", False):
            raise ValueError(f"trajectory_file is not a successful artifact: {trajectory_file}")

        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body").value)
        self._control_period_s = 1.0 / float(
            self.declare_parameter("frequency_hz", 20.0).value
        )
        trajectory = ControlTrajectory.from_dict(artifact)
        if trajectory.frame_id != self._map_frame:
            raise ValueError(
                f"trajectory frame_id={trajectory.frame_id} does not match "
                f"map_frame={self._map_frame}"
            )
        params = MPPIParams(
            control_dt_s=self._control_period_s,
            horizon_steps=int(self.declare_parameter("horizon_steps", 30).value),
            rollout_count=int(self.declare_parameter("rollout_count", 768).value),
            iterations=int(self.declare_parameter("iterations", 2).value),
            temperature=float(self.declare_parameter("temperature", 0.35).value),
            speed_noise_std_mps=float(
                self.declare_parameter("speed_noise_std_mps", 0.05).value
            ),
            curvature_noise_std_1pm=float(
                self.declare_parameter("curvature_noise_std_1pm", 0.25).value
            ),
            max_speed_mps=float(self.declare_parameter("max_speed_mps", 0.20).value),
            max_acceleration_mps2=float(
                self.declare_parameter("max_acceleration_mps2", 0.20).value
            ),
            max_deceleration_mps2=float(
                self.declare_parameter("max_deceleration_mps2", 0.30).value
            ),
            min_turning_radius_m=float(
                self.declare_parameter("min_turning_radius_m", 0.81).value
            ),
            max_curvature_rate_1pmps=float(
                self.declare_parameter("max_curvature_rate_1pmps", 0.80).value
            ),
            goal_position_tolerance_m=float(
                self.declare_parameter("goal_position_tolerance_m", 0.10).value
            ),
            recovery_lateral_error_m=float(
                self.declare_parameter("recovery_lateral_error_m", 0.30).value
            ),
            recovery_heading_error_rad=math.radians(
                float(self.declare_parameter("recovery_heading_error_deg", 65.0).value)
            ),
            progress_search_window_points=int(
                self.declare_parameter("progress_search_window_points", 40).value
            ),
            max_progress_advance_points=int(
                self.declare_parameter("max_progress_advance_points", 3).value
            ),
            lateral_feedback_gain_1pm_per_m=float(
                self.declare_parameter("lateral_feedback_gain_1pm_per_m", 1.5).value
            ),
            heading_feedback_gain_1pm_per_rad=float(
                self.declare_parameter("heading_feedback_gain_1pm_per_rad", 1.0).value
            ),
            feedback_blend=float(self.declare_parameter("feedback_blend", 0.35).value),
        )
        self._controller = MPPIController(
            trajectory,
            params,
            random_seed=int(self.declare_parameter("random_seed", 7).value),
        )
        self._state_estimator = StateEstimator(
            StateEstimatorLimits(
                pose_timeout_s=float(self.declare_parameter("pose_timeout_s", 0.20).value),
                velocity_timeout_s=float(
                    self.declare_parameter("velocity_timeout_s", 0.20).value
                ),
                max_position_jump_m=float(
                    self.declare_parameter("max_position_jump_m", 0.25).value
                ),
                max_heading_jump_rad=math.radians(
                    float(self.declare_parameter("max_heading_jump_deg", 20.0).value)
                ),
            )
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_velocity: Velocity2D | None = None
        self._latest_velocity_stamp_s = 0.0
        odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        self._odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            20,
        )
        self._command_publisher = self.create_publisher(
            TwistStamped,
            str(self.declare_parameter("body_command_topic", "/control/body_cmd").value),
            10,
        )
        self._error_publisher = self.create_publisher(
            Vector3Stamped,
            str(
                self.declare_parameter(
                    "tracking_error_topic",
                    "/control/tracking_error",
                ).value
            ),
            10,
        )
        self._valid_publisher = self.create_publisher(
            Bool,
            str(self.declare_parameter("state_valid_topic", "/control/state_valid").value),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.declare_parameter("status_topic", "/control/status").value),
            10,
        )
        self._timer = self.create_timer(self._control_period_s, self._control_cycle)
        self.get_logger().info(
            f"MPPI ready: trajectory={trajectory_file}, "
            f"frames={self._map_frame}->{self._base_frame}"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_velocity = Velocity2D(
            linear_x_mps=float(message.twist.twist.linear.x),
            yaw_rate_radps=float(message.twist.twist.angular.z),
        )
        self._latest_velocity_stamp_s = _stamp_to_seconds(message.header.stamp)

    def _control_cycle(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        command: BodyCommand
        valid = False
        state_reasons: tuple[str, ...] = ()
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            if self._latest_velocity is None:
                raise RuntimeError("waiting_for_odometry")
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pose = Pose2D(
                x=float(translation.x),
                y=float(translation.y),
                yaw=yaw_from_quaternion(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
            )
            estimate = self._state_estimator.update(
                StateObservation(
                    pose=pose,
                    velocity=self._latest_velocity,
                    pose_stamp_s=_stamp_to_seconds(transform.header.stamp),
                    velocity_stamp_s=self._latest_velocity_stamp_s,
                ),
                now_s=now_s,
            )
            valid = estimate.valid
            state_reasons = estimate.reasons
            if estimate.valid:
                command = self._controller.compute_command(
                    VehicleState(
                        x=estimate.pose.x,
                        y=estimate.pose.y,
                        yaw=estimate.pose.yaw,
                        linear_speed_mps=estimate.velocity.linear_x_mps,
                    )
                )
            else:
                command = BodyCommand.hold(
                    target_index=0,
                    lateral_error_m=0.0,
                    heading_error_rad=0.0,
                    status="INVALID_STATE",
                )
        except (TransformException, RuntimeError) as exc:
            state_reasons = (str(exc),)
            command = BodyCommand.hold(
                target_index=0,
                lateral_error_m=0.0,
                heading_error_rad=0.0,
                status="INVALID_STATE",
            )

        command_message = TwistStamped()
        command_message.header.stamp = now.to_msg()
        command_message.header.frame_id = self._base_frame
        command_message.twist.linear.x = command.linear_x_mps
        command_message.twist.angular.z = command.yaw_rate_radps
        self._command_publisher.publish(command_message)

        error_message = Vector3Stamped()
        error_message.header = command_message.header
        error_message.vector.x = command.lateral_error_m
        error_message.vector.y = command.heading_error_rad
        error_message.vector.z = float(command.target_index)
        self._error_publisher.publish(error_message)
        self._valid_publisher.publish(Bool(data=valid))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "status": command.status,
                        "target_index": command.target_index,
                        "state_reasons": list(state_reasons),
                    },
                    separators=(",", ":"),
                )
            )
        )


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = MPPIControlNode()
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
