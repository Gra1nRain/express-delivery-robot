#!/usr/bin/env python3
"""Integrated indoor mission launch with all physical motion gates off by default."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    bringup_share = get_package_share_directory("competition_bringup")
    realsense_share = get_package_share_directory("realsense2_camera")
    day5_launch = os.path.join(
        bringup_share,
        "launch",
        "day5_motion_control.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_base", default_value="false"),
            DeclareLaunchArgument("start_chassis_adapter", default_value="false"),
            DeclareLaunchArgument("start_wrist_camera", default_value="true"),
            DeclareLaunchArgument("start_real_arm", default_value="false"),
            DeclareLaunchArgument("start_arm_simulator", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, "launch", "rs_launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_wrist_camera")),
                launch_arguments={
                    "camera_namespace": "left_wrist_camera",
                    "camera_name": "camera",
                    "usb_port_id": "2-3.3.2",
                    "initial_reset": "true",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_infra": "false",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "enable_motion": "false",
                    "enable_sync": "true",
                    "align_depth.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "hole_filling_filter.enable": "true",
                    "pointcloud.enable": "false",
                    "publish_tf": "false",
                    "diagnostics_period": "0.0",
                    "rgb_camera.color_profile": "640,480,15",
                    "depth_module.depth_profile": "640,480,15",
                    "log_level": "warn",
                }.items(),
            ),
            Node(
                package="competition_mission",
                executable="piper_arm_task_node",
                name="piper_arm_task",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_real_arm")),
                parameters=[
                    {
                        "migration_root": os.path.join(
                            competition_ws,
                            "Piper_Grasp_Humble_Migration_20260723",
                        ),
                        "manage_camera": False,
                    }
                ],
                additional_env={
                    "PIPER_MIGRATION_ROOT": os.path.join(
                        competition_ws,
                        "Piper_Grasp_Humble_Migration_20260723",
                    )
                },
            ),
            Node(
                package="competition_mission",
                executable="arm_task_simulator_node",
                name="arm_task_simulator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_arm_simulator")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(day5_launch),
                launch_arguments={
                    "trajectory_file": os.path.join(
                        competition_ws,
                        "docs",
                        "evidence",
                        "day5",
                        "indoor_competition_mission_trajectory.yaml",
                    ),
                    "route_file": os.path.join(
                        competition_ws,
                        "config",
                        "routes",
                        "indoor_competition_mission_route.yaml",
                    ),
                    "semantic_map_file": os.path.join(
                        competition_ws,
                        "maps",
                        "debug",
                        "semantic_map.yaml",
                    ),
                    "planning_params_file": os.path.join(
                        competition_ws,
                        "config",
                        "planning",
                        "planning_params.yaml",
                    ),
                    "optimizer_params_file": os.path.join(
                        competition_ws,
                        "config",
                        "planning",
                        "optimizer_params.yaml",
                    ),
                    "mission_config_file": os.path.join(
                        competition_ws,
                        "config",
                        "mission",
                        "indoor_competition_mission.yaml",
                    ),
                    "start_wrist_traffic_perception": "true",
                    "start_competition_mission": "true",
                    # Mission checkpoint holds implement red/yellow/no-result logic.
                    # The old direct safety stop would halt at the pre-trigger marker.
                    "require_traffic_rules": "false",
                    "start_base": LaunchConfiguration("start_base"),
                    "start_chassis_adapter": LaunchConfiguration(
                        "start_chassis_adapter"
                    ),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
