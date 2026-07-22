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


def _launch_setup(context, *args, **kwargs):
    control_config = _load_yaml(LaunchConfiguration("control_params_file").perform(context))
    safety_config = _load_yaml(LaunchConfiguration("safety_params_file").perform(context))
    tracker = control_config["trajectory_tracker"]
    mppi = tracker["mppi"]
    motion = control_config["motion"]
    estimator = control_config["state_estimator"]
    safety = safety_config["safety"]
    proximity_stop = safety_config["proximity_stop"]
    bringup_pkg = get_package_share_directory("competition_bringup")
    mapping_launch = os.path.join(bringup_pkg, "launch", "day1_mapping.launch.py")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "start_livox": LaunchConfiguration("start_livox"),
                "start_fast_lio": LaunchConfiguration("start_fast_lio"),
                "start_base": LaunchConfiguration("start_base"),
                "start_scan": "false",
                "start_slam": "false",
                "start_anchor": "true",
                "fast_lio_config": LaunchConfiguration("fast_lio_config"),
                "anchor_map_frame": str(estimator["map_frame"]),
                "anchor_odom_frame": "camera_init",
                "anchor_base_frame": str(estimator["base_frame"]),
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
                    "base_frame": estimator["base_frame"],
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
                }
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
                    "cloud_topic": proximity_stop["cloud_topic"],
                    "cloud_qos_reliability": proximity_stop[
                        "cloud_qos_reliability"
                    ],
                    "cloud_qos_depth": proximity_stop["cloud_qos_depth"],
                    "stop_request_topic": proximity_stop["stop_request_topic"],
                    "status_topic": proximity_stop["status_topic"],
                    "expected_frame_id": proximity_stop["expected_frame_id"],
                    "max_cloud_age_s": proximity_stop["max_cloud_age_s"],
                    "x_min_m": proximity_stop["x_min_m"],
                    "stop_distance_m": proximity_stop["stop_distance_m"],
                    "front_half_angle_rad": proximity_stop["front_half_angle_rad"],
                    "lateral_half_width_m": proximity_stop[
                        "lateral_half_width_m"
                    ],
                    "z_min_m": proximity_stop["z_min_m"],
                    "z_max_m": proximity_stop["z_max_m"],
                    "min_points": proximity_stop["min_points"],
                }
            ],
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
                    "max_lateral_error_m": safety["max_lateral_error_m"],
                    "max_heading_error_deg": safety["max_heading_error_deg"],
                    "avoidance_stop_topic": safety["avoidance_stop_topic"],
                    "require_avoidance_source": safety["require_avoidance_source"],
                    "avoidance_timeout_s": safety["avoidance_timeout_s"],
                }
            ],
        ),
    ]


def generate_launch_description():
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
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
            DeclareLaunchArgument("port_name", default_value="can3"),
            DeclareLaunchArgument(
                "fast_lio_config",
                default_value="fast_lio_mid360_day5_control.yaml",
            ),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_proximity_stop", default_value="true"),
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
                    "use /cmd_vel only during an approved supervised field run."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
