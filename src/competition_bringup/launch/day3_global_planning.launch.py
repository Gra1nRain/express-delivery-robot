#!/usr/bin/env python3
"""Day 3 semantic global planning bringup.

This launch file publishes nav_msgs/Path only. It does not start controllers,
the chassis driver, or any mechanical-arm action.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    competition_ws_default = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    bringup_share = get_package_share_directory("competition_bringup")

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
            DeclareLaunchArgument("publish_viz_anchor", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    bringup_share,
                    "rviz",
                    "day3_global_planning.rviz",
                ),
            ),
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="day3_map_viz_anchor",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "map",
                    "day3_viz_anchor",
                ],
                condition=IfCondition(LaunchConfiguration("publish_viz_anchor")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_day3_global_planning",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
