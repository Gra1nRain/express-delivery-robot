#!/usr/bin/env python3
"""ROS adapter from FAST-LIO/Ranger state to MPPI body commands."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path
from competition_control.local_plan_continuity import (
    checkpoint_errors,
    checkpoint_longitudinal_error,
    local_paths_are_equivalent,
    nearest_path_point_index,
    nearest_stop_line_path_point_index,
    stop_line_lengths_excluding_docks,
)
from competition_control.mission_checkpoints import mission_checkpoints_from_route
from competition_control.mppi_controller import (
    BodyCommand,
    ControlTrajectory,
    ControlTrajectoryPoint,
    MPPIController,
    MPPIParams,
    VehicleState,
    control_trajectories_from_dict,
    shape_checkpoint_approach_command,
)
from competition_control.segmented_route_state_machine import (
    SegmentedRouteConfig,
    SegmentedRouteObservation,
    SegmentedRouteStateMachine,
    state_failure_requires_rearm,
)
from competition_localization.planar_transform import yaw_from_quaternion
from competition_localization.state_estimator import (
    Pose2D,
    StateEstimator,
    StateEstimatorLimits,
    StateObservation,
    Velocity2D,
    predict_observation_to_time,
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
        self._route = _load_yaml(source_paths["route"])
        self._semantic_map = _load_yaml(source_paths["semantic_map"])
        self._stop_line_lengths_by_ref = stop_line_lengths_excluding_docks(
            self._semantic_map
        )
        self._local_optimizer_config = _load_yaml(source_paths["optimizer_params"])
        self._replanning_enabled = bool(
            self.declare_parameter("replanning_enabled", False).value
        )
        self._local_trajectory_timeout_s = float(
            self.declare_parameter("local_trajectory_timeout_s", 1.0).value
        )
        if self._local_trajectory_timeout_s <= 0.0:
            raise ValueError("local_trajectory_timeout_s must be positive")
        self._latest_local_plan_stamp_s: float | None = None
        self._local_plan_error: str | None = None
        self._accepted_local_geometry: tuple[PathPoint, ...] | None = None
        self._local_plan_update_mode = "waiting"
        self._local_plan_reuse_count = 0
        self._local_plan_replace_count = 0
        self._local_plan_error_count = 0
        self._local_plan_reuse_position_tolerance_m = float(
            self.declare_parameter(
                "local_plan_reuse_position_tolerance_m",
                0.05,
            ).value
        )
        self._local_plan_reuse_heading_tolerance_rad = math.radians(
            float(
                self.declare_parameter(
                    "local_plan_reuse_heading_tolerance_deg",
                    5.0,
                ).value
            )
        )
        if (
            self._local_plan_reuse_position_tolerance_m < 0.0
            or self._local_plan_reuse_heading_tolerance_rad < 0.0
        ):
            raise ValueError("local plan reuse tolerances must be non-negative")
        self._local_stop_requested = self._replanning_enabled
        self._control_period_s = 1.0 / float(
            self.declare_parameter("frequency_hz", 20.0).value
        )
        trajectories = control_trajectories_from_dict(artifact)
        if len(trajectories) != 1:
            raise ValueError(
                "mppi_control requires one continuous trajectory artifact; "
                "segmented artifacts must be regenerated"
            )
        self._global_trajectory = trajectories[0]
        self._mission_checkpoints = mission_checkpoints_from_route(
            self._route,
            self._semantic_map,
            self._global_trajectory,
        )
        self._checkpoint_route_enabled = bool(self._mission_checkpoints)
        self._active_checkpoint_index = 0
        trajectory = self._global_trajectory
        if trajectory.frame_id != self._map_frame:
            raise ValueError(
                f"trajectory frame_id={trajectory.frame_id} does not match "
                f"map_frame={self._map_frame}"
            )
        goal_position_tolerance_m = float(
            self.declare_parameter("goal_position_tolerance_m", 0.10).value
        )
        self._checkpoint_match_tolerance_m = goal_position_tolerance_m
        goal_heading_tolerance_deg = float(
            self.declare_parameter(
                "goal_heading_tolerance_deg",
                4.0 if self._checkpoint_route_enabled else 180.0,
            ).value
        )
        self._checkpoint_heading_tolerance_rad = math.radians(
            goal_heading_tolerance_deg
        )
        self._checkpoint_slowdown_distance_m = float(
            self.declare_parameter(
                "checkpoint_slowdown_distance_m",
                1.0,
            ).value
        )
        self._checkpoint_min_speed_mps = float(
            self.declare_parameter("checkpoint_min_speed_mps", 0.05).value
        )
        self._checkpoint_max_speed_mps = float(
            self.declare_parameter("checkpoint_max_speed_mps", 0.08).value
        )
        if not (
            self._checkpoint_slowdown_distance_m
            > goal_position_tolerance_m
            and 0.0
            < self._checkpoint_min_speed_mps
            <= self._checkpoint_max_speed_mps
        ):
            raise ValueError("invalid checkpoint approach speed limits")
        controller_goal_position_tolerance_m = float(
            self.declare_parameter(
                "controller_goal_position_tolerance_m",
                0.03,
            ).value
        )
        controller_goal_heading_tolerance_deg = float(
            self.declare_parameter(
                "controller_goal_heading_tolerance_deg",
                2.0,
            ).value
        )
        if not (
            0.0 < controller_goal_position_tolerance_m < goal_position_tolerance_m
        ):
            raise ValueError(
                "controller goal position tolerance must be positive and smaller "
                "than checkpoint position tolerance"
            )
        if not (
            0.0
            < controller_goal_heading_tolerance_deg
            < goal_heading_tolerance_deg
        ):
            raise ValueError(
                "controller goal heading tolerance must be positive and smaller "
                "than checkpoint heading tolerance"
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
            command_speed_memory_limit_mps=float(
                self.declare_parameter(
                    "command_speed_memory_limit_mps",
                    0.05,
                ).value
            ),
            goal_position_tolerance_m=controller_goal_position_tolerance_m,
            goal_heading_tolerance_rad=math.radians(
                controller_goal_heading_tolerance_deg
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
        self._route_enabled = False
        self._avoidance_stop_requested = self._checkpoint_route_enabled
        self._mission_phase = (
            "MISSION_DISARMED"
            if self._checkpoint_route_enabled
            else "CONTINUOUS_ROUTE"
        )
        self._segmented_state_machine = None
        if self._checkpoint_route_enabled:
            self._segmented_state_machine = SegmentedRouteStateMachine(
                segment_count=len(self._mission_checkpoints),
                config=SegmentedRouteConfig(
                    goal_position_tolerance_m=goal_position_tolerance_m,
                    goal_heading_tolerance_rad=math.radians(
                        goal_heading_tolerance_deg
                    ),
                    goal_overshoot_tolerance_m=float(
                        self.declare_parameter(
                            "checkpoint_overshoot_tolerance_m",
                            0.02,
                        ).value
                    ),
                    stop_speed_tolerance_mps=float(
                        self.declare_parameter(
                            "segment_stop_speed_tolerance_mps",
                            0.03,
                        ).value
                    ),
                    dock_hold_s=float(
                        self.declare_parameter("segment_dock_hold_s", 2.0).value
                    ),
                ),
            )
        local_optimizer = self._local_optimizer_config.setdefault(
            "trajectory_optimizer",
            {},
        )
        local_optimizer.update(
            {
                "max_speed_mps": params.max_speed_mps,
                "max_acceleration_mps2": params.max_acceleration_mps2,
                "max_deceleration_mps2": params.max_deceleration_mps2,
                "max_curvature_1pm": 1.0 / params.min_turning_radius_m,
            }
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
        self._max_pose_prediction_s = float(
            self.declare_parameter("max_pose_prediction_s", 0.0).value
        )
        self._initial_pose_settle_s = float(
            self.declare_parameter("initial_pose_settle_s", 0.5).value
        )
        if self._initial_pose_settle_s < 0.0:
            raise ValueError("initial_pose_settle_s must be non-negative")
        self._initial_pose_settle_until_s: float | None = None

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
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._reference_path_publisher = self.create_publisher(
            NavPath,
            str(
                self.declare_parameter(
                    "reference_path_topic",
                    "/planning/optimized_trajectory",
                ).value
            ),
            path_qos,
        )
        self._active_checkpoint_publisher = self.create_publisher(
            UInt32,
            str(
                self.declare_parameter(
                    "active_checkpoint_topic",
                    "/mission/active_checkpoint_index",
                ).value
            ),
            path_qos,
        )
        self._active_checkpoint_ref_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "active_checkpoint_ref_topic",
                    "/mission/active_checkpoint_ref",
                ).value
            ),
            path_qos,
        )
        self._executed_path_publisher = self.create_publisher(
            NavPath,
            str(
                self.declare_parameter(
                    "executed_path_topic",
                    "/control/executed_path",
                ).value
            ),
            1,
        )
        self._executed_path_min_separation_m = float(
            self.declare_parameter("executed_path_min_separation_m", 0.05).value
        )
        self._executed_path_max_points = int(
            self.declare_parameter("executed_path_max_points", 2000).value
        )
        if self._executed_path_min_separation_m <= 0.0:
            raise ValueError("executed_path_min_separation_m must be positive")
        if self._executed_path_max_points < 2:
            raise ValueError("executed_path_max_points must be at least 2")
        self._executed_poses: list[PoseStamped] = []
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            10,
        )
        if self._checkpoint_route_enabled:
            self.create_subscription(
                Bool,
                str(
                    self.declare_parameter(
                        "route_enable_topic",
                        "/mission/route_enable",
                    ).value
                ),
                self._route_enable_callback,
                10,
            )
            self.create_subscription(
                Bool,
                str(
                    self.declare_parameter(
                        "avoidance_stop_request_topic",
                        "/avoidance/stop_request",
                    ).value
                ),
                self._avoidance_stop_callback,
                10,
            )
        self._reference_path_publisher.publish(
            _control_trajectory_path(trajectory, self.get_clock().now().to_msg())
        )
        self._active_checkpoint_publisher.publish(UInt32(data=0))
        if self._checkpoint_route_enabled:
            self._active_checkpoint_ref_publisher.publish(
                String(data=self._mission_checkpoints[0].ref_id)
            )
        if self._replanning_enabled:
            self.create_subscription(
                NavPath,
                str(
                    self.declare_parameter(
                        "local_trajectory_topic",
                        "/planning/local_trajectory",
                    ).value
                ),
                self._local_trajectory_callback,
                1,
            )
            self.create_subscription(
                Bool,
                str(
                    self.declare_parameter(
                        "local_stop_request_topic",
                        "/planning/local_stop_request",
                    ).value
                ),
                self._local_stop_callback,
                10,
            )
        self._timer = self.create_timer(self._control_period_s, self._control_cycle)
        self.get_logger().info(
            f"MPPI ready: trajectory={trajectory_file}, "
            f"continuous_points={len(self._global_trajectory.points)}, "
            f"mission_checkpoints={len(self._mission_checkpoints)}, "
            f"frames={self._map_frame}->{self._base_frame}"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_velocity = Velocity2D(
            linear_x_mps=float(message.twist.twist.linear.x),
            yaw_rate_radps=float(message.twist.twist.angular.z),
        )
        self._latest_velocity_stamp_s = _stamp_to_seconds(message.header.stamp)

    def _initialpose_callback(self, message: PoseWithCovarianceStamped) -> None:
        frame_id = message.header.frame_id or self._map_frame
        if frame_id != self._map_frame:
            self.get_logger().warning(
                f"Ignoring /initialpose in {frame_id}; expected {self._map_frame}"
            )
            return
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning("Ignoring non-finite /initialpose")
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        self._initial_pose_settle_until_s = now_s + self._initial_pose_settle_s
        self._controller.reset()
        if self._segmented_state_machine is not None:
            self._route_enabled = False
            self._segmented_state_machine.reset()
            self._controller.replace_trajectory(self._global_trajectory)
            self._activate_checkpoint(0)
        self._accepted_local_geometry = None
        self._local_plan_update_mode = "waiting"
        self._executed_poses.clear()
        self.get_logger().info(
            "Received /initialpose; holding zero command for "
            f"{self._initial_pose_settle_s:.2f}s before resetting state continuity"
        )

    def _local_trajectory_callback(self, message: NavPath) -> None:
        if message.header.frame_id != self._map_frame:
            self._local_plan_error = (
                f"frame_mismatch:{message.header.frame_id}->{self._map_frame}"
            )
            self._local_plan_update_mode = "error"
            self._local_plan_error_count += 1
            return
        try:
            geometry = tuple(
                PathPoint(
                    x=float(pose.pose.position.x),
                    y=float(pose.pose.position.y),
                    yaw=yaw_from_quaternion(
                        float(pose.pose.orientation.x),
                        float(pose.pose.orientation.y),
                        float(pose.pose.orientation.z),
                        float(pose.pose.orientation.w),
                    ),
                )
                for pose in message.poses
            )
            geometry = self._annotate_active_checkpoint(geometry)
            if self._accepted_local_geometry is not None and local_paths_are_equivalent(
                self._accepted_local_geometry,
                geometry,
                position_tolerance_m=self._local_plan_reuse_position_tolerance_m,
                heading_tolerance_rad=self._local_plan_reuse_heading_tolerance_rad,
            ):
                self._latest_local_plan_stamp_s = _stamp_to_seconds(
                    message.header.stamp
                )
                self._local_plan_error = None
                self._local_plan_update_mode = "reused"
                self._local_plan_reuse_count += 1
                return
            parameterized = parameterize_local_path(
                geometry,
                self._semantic_map,
                self._local_optimizer_config,
            )
            trajectory = ControlTrajectory(
                frame_id=self._map_frame,
                route_name="online_local_replan",
                points=tuple(
                    ControlTrajectoryPoint(
                        x=point.x,
                        y=point.y,
                        yaw=point.yaw,
                        s=point.s,
                        curvature=point.curvature,
                        v=point.v,
                        t=point.t,
                        ref_id=point.ref_id,
                    )
                    for point in parameterized.points
                ),
            )
            self._controller.replace_trajectory(trajectory)
        except (KeyError, TypeError, ValueError) as exc:
            self._local_plan_error = str(exc)
            self._local_plan_update_mode = "error"
            self._local_plan_error_count += 1
            return
        self._accepted_local_geometry = geometry
        self._latest_local_plan_stamp_s = _stamp_to_seconds(message.header.stamp)
        self._local_plan_error = None
        self._local_plan_update_mode = "replaced"
        self._local_plan_replace_count += 1

    def _annotate_active_checkpoint(
        self,
        geometry: tuple[PathPoint, ...],
    ) -> tuple[PathPoint, ...]:
        if not self._checkpoint_route_enabled:
            return geometry
        checkpoint = self._mission_checkpoints[self._active_checkpoint_index]
        stop_line_length_m = self._stop_line_lengths_by_ref.get(checkpoint.ref_id)
        if stop_line_length_m is None:
            checkpoint_index = nearest_path_point_index(
                geometry,
                checkpoint,
                max_distance_m=self._checkpoint_match_tolerance_m,
            )
        else:
            checkpoint_index = nearest_stop_line_path_point_index(
                geometry,
                checkpoint,
                line_length_m=stop_line_length_m,
                max_longitudinal_distance_m=self._checkpoint_match_tolerance_m,
            )
        if checkpoint_index is None:
            return geometry
        return tuple(
            PathPoint(
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                ref_id=checkpoint.ref_id if index == checkpoint_index else point.ref_id,
            )
            for index, point in enumerate(geometry)
        )

    def _local_stop_callback(self, message: Bool) -> None:
        self._local_stop_requested = bool(message.data)

    def _route_enable_callback(self, message: Bool) -> None:
        self._route_enabled = bool(message.data)

    def _avoidance_stop_callback(self, message: Bool) -> None:
        self._avoidance_stop_requested = bool(message.data)

    def _activate_checkpoint(self, checkpoint_index: int) -> None:
        self._active_checkpoint_index = checkpoint_index
        self._accepted_local_geometry = None
        self._latest_local_plan_stamp_s = None
        if self._replanning_enabled:
            self._local_stop_requested = True
        self._active_checkpoint_publisher.publish(UInt32(data=checkpoint_index))
        checkpoint = self._mission_checkpoints[checkpoint_index]
        self._active_checkpoint_ref_publisher.publish(
            String(data=checkpoint.ref_id)
        )
        self.get_logger().info(
            f"Activated mission checkpoint {checkpoint_index + 1}/"
            f"{len(self._mission_checkpoints)}: {checkpoint.ref_id}"
        )

    def _segmented_command(
        self,
        state: VehicleState,
        now_s: float,
        local_plan_age_s: float | None,
    ) -> BodyCommand:
        assert self._segmented_state_machine is not None
        goal = self._mission_checkpoints[self._active_checkpoint_index]
        position_error_m, heading_error_rad = checkpoint_errors(
            state,
            goal,
            stop_line_length_m=self._stop_line_lengths_by_ref.get(goal.ref_id),
        )
        longitudinal_error_m = checkpoint_longitudinal_error(state, goal)
        local_plan_unavailable = self._replanning_enabled and (
            self._local_stop_requested
            or local_plan_age_s is None
            or local_plan_age_s > self._local_trajectory_timeout_s
        )
        decision = self._segmented_state_machine.update(
            SegmentedRouteObservation(
                now_s=now_s,
                enabled=self._route_enabled,
                state_valid=True,
                stop_requested=(
                    self._avoidance_stop_requested or local_plan_unavailable
                ),
                position_error_m=position_error_m,
                heading_error_rad=heading_error_rad,
                speed_mps=state.linear_speed_mps,
                longitudinal_error_m=longitudinal_error_m,
            )
        )
        self._mission_phase = decision.phase.value
        if decision.segment_changed:
            self._activate_checkpoint(decision.active_segment_index)
        if decision.allow_tracking:
            return shape_checkpoint_approach_command(
                self._controller.compute_command(state),
                longitudinal_error_m=longitudinal_error_m,
                checkpoint_heading_error_rad=heading_error_rad,
                checkpoint_heading_tolerance_rad=(
                    self._checkpoint_heading_tolerance_rad
                ),
                capture_distance_m=self._checkpoint_match_tolerance_m,
                slowdown_distance_m=self._checkpoint_slowdown_distance_m,
                min_speed_mps=self._checkpoint_min_speed_mps,
                max_speed_mps=self._checkpoint_max_speed_mps,
            )
        return BodyCommand.hold(
            target_index=0,
            lateral_error_m=0.0,
            heading_error_rad=0.0,
            status=decision.phase.value,
        )

    def _segmented_invalid_command(
        self,
        now_s: float,
        state_reasons: tuple[str, ...],
    ) -> BodyCommand:
        assert self._segmented_state_machine is not None
        requires_rearm = state_failure_requires_rearm(state_reasons)
        decision = self._segmented_state_machine.update(
            SegmentedRouteObservation(
                now_s=now_s,
                enabled=self._route_enabled,
                state_valid=not requires_rearm,
                stop_requested=(
                    self._avoidance_stop_requested or not requires_rearm
                ),
                position_error_m=math.inf,
                heading_error_rad=math.inf,
                speed_mps=math.inf,
            )
        )
        self._mission_phase = decision.phase.value
        return BodyCommand.hold(
            target_index=0,
            lateral_error_m=0.0,
            heading_error_rad=0.0,
            status=decision.phase.value,
        )

    def _control_cycle(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        command: BodyCommand
        valid = False
        state_reasons: tuple[str, ...] = ()
        pose_delay_s: float | None = None
        pose_prediction_s = 0.0
        try:
            if self._initial_pose_settle_until_s is not None:
                if now_s < self._initial_pose_settle_until_s:
                    raise RuntimeError("initial_pose_settling")
                self._state_estimator.reset()
                self._initial_pose_settle_until_s = None
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
            raw_pose_stamp_s = _stamp_to_seconds(transform.header.stamp)
            pose_delay_s = now_s - raw_pose_stamp_s
            observation = StateObservation(
                pose=pose,
                velocity=self._latest_velocity,
                pose_stamp_s=raw_pose_stamp_s,
                velocity_stamp_s=self._latest_velocity_stamp_s,
            )
            observation = predict_observation_to_time(
                observation,
                now_s=now_s,
                max_prediction_s=self._max_pose_prediction_s,
            )
            pose_prediction_s = observation.pose_stamp_s - raw_pose_stamp_s
            estimate = self._state_estimator.update(
                observation,
                now_s=now_s,
            )
            valid = estimate.valid
            state_reasons = estimate.reasons
            if estimate.valid:
                vehicle_state = VehicleState(
                    x=estimate.pose.x,
                    y=estimate.pose.y,
                    yaw=estimate.pose.yaw,
                    linear_speed_mps=estimate.velocity.linear_x_mps,
                )
                self._publish_executed_pose(vehicle_state, now.to_msg())
                local_plan_age_s = self._local_plan_age_s(now_s)
                if self._segmented_state_machine is not None:
                    command = self._segmented_command(
                        vehicle_state,
                        now_s,
                        local_plan_age_s,
                    )
                elif self._replanning_enabled and self._local_stop_requested:
                    command = BodyCommand.hold(
                        target_index=0,
                        lateral_error_m=0.0,
                        heading_error_rad=0.0,
                        status="LOCAL_PLANNER_STOP",
                    )
                elif self._replanning_enabled and (
                    local_plan_age_s is None
                    or local_plan_age_s > self._local_trajectory_timeout_s
                ):
                    command = BodyCommand.hold(
                        target_index=0,
                        lateral_error_m=0.0,
                        heading_error_rad=0.0,
                        status="LOCAL_PLAN_STALE",
                    )
                else:
                    command = self._controller.compute_command(vehicle_state)
            else:
                if self._segmented_state_machine is not None:
                    command = self._segmented_invalid_command(now_s, state_reasons)
                else:
                    command = BodyCommand.hold(
                        target_index=0,
                        lateral_error_m=0.0,
                        heading_error_rad=0.0,
                        status="INVALID_STATE",
                    )
        except (TransformException, RuntimeError) as exc:
            state_reasons = (str(exc),)
            if self._segmented_state_machine is not None:
                command = self._segmented_invalid_command(now_s, state_reasons)
            else:
                command = BodyCommand.hold(
                    target_index=0,
                    lateral_error_m=0.0,
                    heading_error_rad=0.0,
                    status="INVALID_STATE",
                )

        if command.status != "TRACKING":
            self._controller.reset()

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
                        "pose_delay_s": pose_delay_s,
                        "pose_prediction_s": pose_prediction_s,
                        "local_plan_age_s": self._local_plan_age_s(now_s),
                        "local_plan_error": self._local_plan_error,
                        "local_plan_update_mode": self._local_plan_update_mode,
                        "local_plan_reuse_count": self._local_plan_reuse_count,
                        "local_plan_replace_count": self._local_plan_replace_count,
                        "local_plan_error_count": self._local_plan_error_count,
                        "mission_phase": self._mission_phase,
                        "route_enabled": self._route_enabled,
                        "active_checkpoint_index": self._active_checkpoint_index,
                        "active_checkpoint_ref": (
                            self._mission_checkpoints[
                                self._active_checkpoint_index
                            ].ref_id
                            if self._mission_checkpoints
                            else None
                        ),
                        "checkpoint_count": len(self._mission_checkpoints),
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _local_plan_age_s(self, now_s: float) -> float | None:
        if self._latest_local_plan_stamp_s is None:
            return None
        return now_s - self._latest_local_plan_stamp_s

    def _publish_executed_pose(self, state: VehicleState, stamp) -> None:
        if self._executed_poses:
            last = self._executed_poses[-1].pose.position
            if (
                math.hypot(state.x - float(last.x), state.y - float(last.y))
                < self._executed_path_min_separation_m
            ):
                return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = state.x
        pose.pose.position.y = state.y
        pose.pose.orientation.z = math.sin(state.yaw * 0.5)
        pose.pose.orientation.w = math.cos(state.yaw * 0.5)
        self._executed_poses.append(pose)
        if len(self._executed_poses) > self._executed_path_max_points:
            del self._executed_poses[: -self._executed_path_max_points]
        message = NavPath()
        message.header = pose.header
        message.poses = list(self._executed_poses)
        self._executed_path_publisher.publish(message)


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def _control_trajectory_path(trajectory: ControlTrajectory, stamp) -> NavPath:
    message = NavPath()
    message.header.stamp = stamp
    message.header.frame_id = trajectory.frame_id
    for point in trajectory.points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(point.yaw * 0.5)
        pose.pose.orientation.w = math.cos(point.yaw * 0.5)
        message.poses.append(pose)
    return message


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
