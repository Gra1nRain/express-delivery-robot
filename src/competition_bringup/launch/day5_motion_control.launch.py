#!/usr/bin/env python3
"""Day 5 FAST-LIO -> MPPI -> safety bringup with two explicit motion gates."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _load_yaml(path: str) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def _bool_override(value: str, *, default: bool) -> bool:
    normalized = value.strip().lower()
    if normalized in ("", "auto"):
        return default
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"invalid boolean launch override: {value!r}")


def _launch_setup(context, *args, **kwargs):
    control_config = _load_yaml(LaunchConfiguration("control_params_file").perform(context))
    planning_config = _load_yaml(
        LaunchConfiguration("planning_params_file").perform(context)
    )
    safety_config = _load_yaml(LaunchConfiguration("safety_params_file").perform(context))
    tracker = control_config["trajectory_tracker"]
    mppi = tracker["mppi"]
    motion = control_config["motion"]
    estimator = control_config["state_estimator"]
    visualization = control_config["visualization"]
    localization_base_frame = str(
        estimator.get("localization_base_frame", estimator["base_frame"])
    )
    tracking_base_frame = str(
        estimator.get("tracking_base_frame", estimator["base_frame"])
    )
    global_planner = planning_config["global_planner"]
    replanning = planning_config["replanning"]
    replanning_enabled = _bool_override(
        LaunchConfiguration("replanning_enabled").perform(context),
        default=bool(replanning["enabled"]),
    )
    safety = safety_config["safety"]
    scan_projection = safety_config["pointcloud_to_laserscan"]
    scan_projection_parameters = {
        key: value
        for key, value in scan_projection.items()
        if key not in {"input_topic", "output_topic"}
    }
    proximity_stop = safety_config["proximity_stop"]
    bringup_pkg = get_package_share_directory("competition_bringup")
    mapping_launch = os.path.join(bringup_pkg, "launch", "day1_mapping.launch.py")

    actions = []
    tracking_transform = estimator.get("tracking_frame_transform")
    if isinstance(tracking_transform, dict):
        parent_frame = str(tracking_transform.get("parent_frame", estimator["base_frame"]))
        child_frame = str(tracking_transform.get("child_frame", tracking_base_frame))
        if child_frame != parent_frame:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{parent_frame}_to_{child_frame}_static_tf",
                    output="screen",
                    arguments=[
                        "--x",
                        str(float(tracking_transform.get("x_m", 0.0))),
                        "--y",
                        str(float(tracking_transform.get("y_m", 0.0))),
                        "--z",
                        str(float(tracking_transform.get("z_m", 0.0))),
                        "--yaw",
                        str(float(tracking_transform.get("yaw_rad", 0.0))),
                        "--pitch",
                        str(float(tracking_transform.get("pitch_rad", 0.0))),
                        "--roll",
                        str(float(tracking_transform.get("roll_rad", 0.0))),
                        "--frame-id",
                        parent_frame,
                        "--child-frame-id",
                        child_frame,
                    ],
                )
            )

    actions.extend(
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "start_livox": LaunchConfiguration("start_livox"),
                "force_livox_host_timestamps": "true",
                "start_fast_lio": LaunchConfiguration("start_fast_lio"),
                "start_base": LaunchConfiguration("start_base"),
                "publish_odom_tf": "false",
                "start_scan": "false",
                "start_slam": "false",
                "start_anchor": "true",
                "fast_lio_config": LaunchConfiguration("fast_lio_config"),
                "anchor_map_frame": str(estimator["map_frame"]),
                "anchor_odom_frame": "camera_init",
                "anchor_base_frame": localization_base_frame,
                "port_name": LaunchConfiguration("port_name"),
                "robot_model": "ranger_mini_v3",
                "rviz": "false",
            }.items(),
        ),
        Node(
            package="competition_control",
            executable="mppi_control_node",
            name="mppi_control",
            output="screen",
            parameters=[
                {
                    "trajectory_file": LaunchConfiguration("trajectory_file"),
                    "route_file": LaunchConfiguration("route_file"),
                    "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                    "planning_params_file": LaunchConfiguration(
                        "planning_params_file"
                    ),
                    "optimizer_params_file": LaunchConfiguration(
                        "optimizer_params_file"
                    ),
                    "map_frame": estimator["map_frame"],
                    "base_frame": tracking_base_frame,
                    "frequency_hz": tracker["frequency_hz"],
                    "horizon_steps": mppi["horizon_steps"],
                    "rollout_count": mppi["rollout_count"],
                    "iterations": mppi["iterations"],
                    "temperature": mppi["temperature"],
                    "speed_noise_std_mps": mppi["speed_noise_std_mps"],
                    "curvature_noise_std_1pm": mppi["curvature_noise_std_1pm"],
                    "progress_search_window_points": mppi[
                        "progress_search_window_points"
                    ],
                    "max_progress_advance_points": mppi[
                        "max_progress_advance_points"
                    ],
                    "lateral_feedback_gain_1pm_per_m": mppi[
                        "lateral_feedback_gain_1pm_per_m"
                    ],
                    "heading_feedback_gain_1pm_per_rad": mppi[
                        "heading_feedback_gain_1pm_per_rad"
                    ],
                    "feedback_blend": mppi["feedback_blend"],
                    "max_speed_mps": motion["max_speed_mps"],
                    "max_acceleration_mps2": motion["max_acceleration_mps2"],
                    "max_deceleration_mps2": motion["max_deceleration_mps2"],
                    "min_turning_radius_m": motion["min_turning_radius_m"],
                    "max_curvature_rate_1pmps": motion["max_curvature_rate_1pmps"],
                    "pose_timeout_s": estimator["pose_timeout_s"],
                    "velocity_timeout_s": estimator["velocity_timeout_s"],
                    "max_pose_prediction_s": estimator["max_pose_prediction_s"],
                    "max_position_jump_m": estimator["max_position_jump_m"],
                    "max_heading_jump_deg": estimator["max_heading_jump_deg"],
                    "reference_path_topic": visualization[
                        "reference_path_topic"
                    ],
                    "executed_path_topic": visualization["executed_path_topic"],
                    "executed_path_min_separation_m": visualization[
                        "executed_path_min_separation_m"
                    ],
                    "executed_path_max_points": visualization[
                        "executed_path_max_points"
                    ],
                    "replanning_enabled": replanning_enabled,
                    "local_trajectory_topic": visualization[
                        "local_trajectory_topic"
                    ],
                    "local_trajectory_timeout_s": replanning[
                        "local_trajectory_timeout_s"
                    ],
                    "local_stop_request_topic": replanning[
                        "local_stop_request_topic"
                    ],
                }
            ],
        ),
        Node(
            package="competition_planning",
            executable="dwa_local_planner_node",
            name="dwa_local_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_local_replanner")),
            parameters=[
                {
                    "trajectory_file": LaunchConfiguration("trajectory_file"),
                    "map_frame": estimator["map_frame"],
                    "base_frame": tracking_base_frame,
                    "frequency_hz": replanning["frequency_hz"],
                    "obstacle_source": replanning["obstacle_source"],
                    "costmap_topic": replanning["costmap_topic"],
                    "expected_obstacle_frame": replanning[
                        "expected_obstacle_frame"
                    ],
                    "costmap_occupancy_threshold": replanning[
                        "costmap_occupancy_threshold"
                    ],
                    "odom_topic": replanning["odom_topic"],
                    "max_obstacle_age_s": replanning["max_obstacle_age_s"],
                    "max_odom_age_s": replanning["max_odom_age_s"],
                    "obstacle_x_min_m": replanning["obstacle_x_min_m"],
                    "obstacle_x_max_m": replanning["obstacle_x_max_m"],
                    "obstacle_y_half_width_m": replanning[
                        "obstacle_y_half_width_m"
                    ],
                    "max_obstacle_points": replanning["max_obstacle_points"],
                    "min_speed_mps": replanning["min_speed_mps"],
                    "max_speed_mps": replanning["max_speed_mps"],
                    "max_acceleration_mps2": replanning[
                        "max_acceleration_mps2"
                    ],
                    "max_deceleration_mps2": replanning[
                        "max_deceleration_mps2"
                    ],
                    "max_yaw_rate_radps": replanning["max_yaw_rate_radps"],
                    "max_yaw_acceleration_radps2": replanning[
                        "max_yaw_acceleration_radps2"
                    ],
                    "min_turning_radius_m": motion["min_turning_radius_m"],
                    "prediction_horizon_s": replanning["prediction_horizon_s"],
                    "simulation_step_s": replanning["simulation_step_s"],
                    "speed_sample_count": replanning["speed_sample_count"],
                    "yaw_rate_sample_count": replanning[
                        "yaw_rate_sample_count"
                    ],
                    "obstacle_clearance_m": replanning[
                        "obstacle_clearance_m"
                    ],
                    "reference_lookahead_m": replanning[
                        "reference_lookahead_m"
                    ],
                    "max_reference_deviation_m": replanning[
                        "max_reference_deviation_m"
                    ],
                    "reference_search_window_points": replanning[
                        "reference_search_window_points"
                    ],
                    "progress_weight": replanning["progress_weight"],
                    "path_distance_weight": replanning[
                        "path_distance_weight"
                    ],
                    "goal_distance_weight": replanning[
                        "goal_distance_weight"
                    ],
                    "heading_weight": replanning["heading_weight"],
                    "clearance_weight": replanning["clearance_weight"],
                    "speed_weight": replanning["speed_weight"],
                    "yaw_rate_weight": replanning["yaw_rate_weight"],
                    "local_trajectory_topic": replanning[
                        "local_trajectory_topic"
                    ],
                    "local_stop_request_topic": replanning[
                        "local_stop_request_topic"
                    ],
                    "status_topic": replanning["status_topic"],
                }
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="day5_pointcloud_to_laserscan",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_proximity_stop")),
            parameters=[scan_projection_parameters],
            remappings=[
                ("cloud_in", scan_projection["input_topic"]),
                ("scan", scan_projection["output_topic"]),
            ],
        ),
        Node(
            package="competition_safety",
            executable="proximity_stop_node",
            name="proximity_stop",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_proximity_stop")),
            parameters=[
                {
                    "input_type": proximity_stop["input_type"],
                    "input_scan_topic": proximity_stop["input_scan_topic"],
                    "scan_qos_reliability": proximity_stop[
                        "scan_qos_reliability"
                    ],
                    "scan_qos_depth": proximity_stop["scan_qos_depth"],
                    "stop_request_topic": proximity_stop["stop_request_topic"],
                    "status_topic": proximity_stop["status_topic"],
                    "costmap_topic": proximity_stop["costmap_topic"],
                    "scan_topic": proximity_stop["scan_topic"],
                    "marker_topic": proximity_stop["marker_topic"],
                    "visualization_rate_hz": proximity_stop[
                        "visualization_rate_hz"
                    ],
                    "expected_frame_id": proximity_stop["expected_frame_id"],
                    "max_scan_age_s": proximity_stop["max_scan_age_s"],
                    "x_min_m": proximity_stop["x_min_m"],
                    "stop_distance_m": proximity_stop["stop_distance_m"],
                    "front_half_angle_rad": proximity_stop["front_half_angle_rad"],
                    "lateral_half_width_m": proximity_stop[
                        "lateral_half_width_m"
                    ],
                    "z_min_m": proximity_stop["z_min_m"],
                    "z_max_m": proximity_stop["z_max_m"],
                    "min_points": proximity_stop["min_points"],
                    "grid_resolution_m": proximity_stop["grid_resolution_m"],
                    "grid_x_min_m": proximity_stop["grid_x_min_m"],
                    "grid_x_max_m": proximity_stop["grid_x_max_m"],
                    "grid_y_min_m": proximity_stop["grid_y_min_m"],
                    "grid_y_max_m": proximity_stop["grid_y_max_m"],
                    "grid_inflation_radius_m": proximity_stop[
                        "grid_inflation_radius_m"
                    ],
                    "scan_bin_count": proximity_stop["scan_bin_count"],
                    "scan_range_min_m": proximity_stop["scan_range_min_m"],
                    "scan_range_max_m": proximity_stop["scan_range_max_m"],
                    "vehicle_length_m": proximity_stop["vehicle_length_m"],
                    "vehicle_width_m": proximity_stop["vehicle_width_m"],
                }
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="day5_map_server",
            output="screen",
            parameters=[{"yaml_filename": LaunchConfiguration("map_file")}],
            condition=IfCondition(LaunchConfiguration("start_map_server")),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="day5_map_lifecycle_manager",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["day5_map_server"],
                }
            ],
            condition=IfCondition(LaunchConfiguration("start_map_server")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_day5_motion_control",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="competition_safety",
            executable="safety_node",
            name="competition_safety",
            output="screen",
            parameters=[
                {
                    "frequency_hz": tracker["frequency_hz"],
                    "command_output_topic": LaunchConfiguration("command_output_topic"),
                    "command_timeout_s": safety["command_timeout_s"],
                    "state_timeout_s": safety["state_timeout_s"],
                    "system_state_timeout_s": safety["system_state_timeout_s"],
                    "max_speed_mps": safety["max_speed_mps"],
                    "max_acceleration_mps2": safety["max_acceleration_mps2"],
                    "max_deceleration_mps2": safety["max_deceleration_mps2"],
                    "min_turning_radius_m": safety["min_turning_radius_m"],
                    "recovery_lateral_error_m": safety.get(
                        "recovery_lateral_error_m",
                        0.10,
                    ),
                    "recovery_heading_error_deg": safety.get(
                        "recovery_heading_error_deg",
                        10.0,
                    ),
                    "recovery_clear_lateral_error_m": safety.get(
                        "recovery_clear_lateral_error_m",
                        0.06,
                    ),
                    "recovery_clear_heading_error_deg": safety.get(
                        "recovery_clear_heading_error_deg",
                        5.0,
                    ),
                    "recovery_speed_mps": safety.get("recovery_speed_mps", 0.06),
                    "max_lateral_error_m": safety["max_lateral_error_m"],
                    "max_heading_error_deg": safety["max_heading_error_deg"],
                    "tracking_error_timeout_s": safety.get(
                        "tracking_error_timeout_s",
                        1.0,
                    ),
                    "avoidance_stop_topic": safety["avoidance_stop_topic"],
                    "require_avoidance_source": safety["require_avoidance_source"],
                    "avoidance_timeout_s": safety["avoidance_timeout_s"],
                }
            ],
        ),
        Node(
            package="competition_control",
            executable="ranger_twist_adapter_node",
            name="ranger_twist_adapter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_chassis_adapter")),
            parameters=[
                {
                    "input_topic": LaunchConfiguration("chassis_adapter_input_topic"),
                    "output_topic": LaunchConfiguration("chassis_adapter_output_topic"),
                    "wheelbase_m": motion["wheelbase_m"],
                    "track_width_m": motion["track_width_m"],
                    "driver_min_turn_radius_m": motion[
                        "ranger_driver_min_turn_radius_m"
                    ],
                }
            ],
        ),
        ]
    )
    return actions


def generate_launch_description():
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    bringup_share = get_package_share_directory("competition_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory_file",
                default_value=os.path.join(
                    competition_ws,
                    "docs",
                    "evidence",
                    "day5",
                    "debug_continuous_trajectory.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "route_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "routes",
                    "debug_route.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "semantic_map_file",
                default_value=os.path.join(
                    competition_ws,
                    "maps",
                    "debug",
                    "semantic_map.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "planning_params_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "planning",
                    "planning_params.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "optimizer_params_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "planning",
                    "optimizer_params.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "control_params_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "control",
                    "control_params.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "safety",
                    "safety_params.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=os.path.join(
                    competition_ws,
                    "maps",
                    "debug",
                    "map.yaml",
                ),
            ),
            DeclareLaunchArgument("port_name", default_value="can3"),
            DeclareLaunchArgument(
                "fast_lio_config",
                default_value="fast_lio_mid360_day5_control.yaml",
            ),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_proximity_stop", default_value="true"),
            DeclareLaunchArgument("start_local_replanner", default_value="true"),
            DeclareLaunchArgument(
                "replanning_enabled",
                default_value="auto",
                description=(
                    "Override MPPI local-trajectory dependency; auto uses "
                    "planning_params.yaml replanning.enabled."
                ),
            ),
            DeclareLaunchArgument("start_map_server", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    bringup_share,
                    "rviz",
                    "day5_motion_control.rviz",
                ),
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="false",
                description="First motion gate: start the Ranger CAN driver.",
            ),
            DeclareLaunchArgument(
                "command_output_topic",
                default_value="/cmd_vel_safe",
                description=(
                    "Second motion gate: keep /cmd_vel_safe for no-motion checks; "
                    "enable start_chassis_adapter during an approved supervised "
                    "field run."
                ),
            ),
            DeclareLaunchArgument(
                "start_chassis_adapter",
                default_value="false",
                description=(
                    "Final motion gate: adapt /cmd_vel_safe to Ranger /cmd_vel "
                    "during an approved supervised field run."
                ),
            ),
            DeclareLaunchArgument(
                "chassis_adapter_input_topic",
                default_value="/cmd_vel_safe",
            ),
            DeclareLaunchArgument(
                "chassis_adapter_output_topic",
                default_value="/cmd_vel",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
