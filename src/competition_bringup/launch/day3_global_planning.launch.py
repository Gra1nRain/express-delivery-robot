#!/usr/bin/env python3
"""Day 3 semantic global planning bringup.

This launch file publishes nav_msgs/Path only. It does not start controllers,
the chassis driver, or any mechanical-arm action.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    competition_ws_default = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file",
                default_value=os.path.join(
                    competition_ws_default, "config", "routes", "debug_route.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "semantic_map_file",
                default_value=os.path.join(
                    competition_ws_default, "maps", "debug", "semantic_map.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "planning_params_file",
                default_value=os.path.join(
                    competition_ws_default,
                    "config",
                    "planning",
                    "planning_params.yaml",
                ),
            ),
            DeclareLaunchArgument("step_id", default_value="go_traffic_light_1"),
            DeclareLaunchArgument("publish_topic", default_value="/planning/global_path"),
            DeclareLaunchArgument("publish_period_s", default_value="1.0"),
            Node(
                package="competition_planning",
                executable="semantic_global_path_node",
                name="semantic_global_path",
                output="screen",
                parameters=[
                    {
                        "route_file": LaunchConfiguration("route_file"),
                        "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                        "planning_params_file": LaunchConfiguration("planning_params_file"),
                        "step_id": LaunchConfiguration("step_id"),
                        "publish_topic": LaunchConfiguration("publish_topic"),
                        "publish_period_s": LaunchConfiguration("publish_period_s"),
                    }
                ],
            ),
        ]
    )
