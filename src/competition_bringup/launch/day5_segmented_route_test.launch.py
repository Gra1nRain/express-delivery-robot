#!/usr/bin/env python3
"""Bring up the state-driven indoor one-lap route on the Day 5 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    bringup_share = get_package_share_directory("competition_bringup")
    motion_launch = os.path.join(
        bringup_share,
        "launch",
        "day5_motion_control.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("port_name", default_value="can3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(motion_launch),
                launch_arguments={
                    "trajectory_file": os.path.join(
                        competition_ws,
                        "docs",
                        "evidence",
                        "day4",
                        "debug_indoor_one_lap_trajectory.yaml",
                    ),
                    "route_file": os.path.join(
                        competition_ws,
                        "config",
                        "routes",
                        "debug_indoor_one_lap_route.yaml",
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
                    "start_livox": LaunchConfiguration("start_livox"),
                    "start_fast_lio": LaunchConfiguration("start_fast_lio"),
                    "start_base": LaunchConfiguration("start_base"),
                    "port_name": LaunchConfiguration("port_name"),
                    "rviz": LaunchConfiguration("rviz"),
                    "start_proximity_stop": "true",
                    "start_local_replanner": "true",
                    "replanning_enabled": "true",
                    "start_map_server": "true",
                    "command_output_topic": "/cmd_vel_safe",
                    "start_chassis_adapter": "false",
                }.items(),
            ),
        ]
    )
