#!/usr/bin/env python3
"""Day 3 execution bringup using the proven Ranger Nav2 frame chain.

This launch reuses the frame convention from ``agilex_ws``:
``map -> odom -> base_link`` for localization and control.  FAST-LIO's
``camera_init -> body`` chain is intentionally not part of this execution path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    competition_ws_default = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    ranger_nav_pkg = get_package_share_directory("ranger_nav")
    ranger_bringup_pkg = get_package_share_directory("ranger_bringup")
    bringup_pkg = get_package_share_directory("competition_bringup")

    description_launch = os.path.join(ranger_nav_pkg, "launch", "description.launch.py")
    livox_launch = os.path.join(ranger_nav_pkg, "launch", "livox_mid360_pointcloud2.launch.py")
    ranger_launch = os.path.join(ranger_bringup_pkg, "launch", "ranger_mini_v3.launch.py")
    planning_launch = os.path.join(bringup_pkg, "launch", "day3_global_planning.launch.py")

    map_file = LaunchConfiguration("map_file")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    scan_config = LaunchConfiguration("scan_config")
    port_name = LaunchConfiguration("port_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("port_name", default_value="can3"),
            DeclareLaunchArgument(
                "map_file",
                default_value=os.path.join(competition_ws_default, "maps", "debug", "map.yaml"),
            ),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=os.path.join(
                    competition_ws_default, "config", "localization", "day3_amcl.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scan_config",
                default_value=os.path.join(
                    competition_ws_default,
                    "config",
                    "mapping",
                    "pointcloud_to_laserscan_day3_nav.yaml",
                ),
            ),
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
                    competition_ws_default, "config", "planning", "planning_params.yaml"
                ),
            ),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_scan", default_value="true"),
            DeclareLaunchArgument("start_planning", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                condition=IfCondition(LaunchConfiguration("start_description")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ranger_launch),
                condition=IfCondition(LaunchConfiguration("start_base")),
                launch_arguments={
                    "port_name": port_name,
                    "robot_model": "ranger_mini_v3",
                    "update_rate": "50",
                    "publish_odom_tf": "true",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "odom_topic_name": "odom",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
                condition=IfCondition(LaunchConfiguration("start_livox")),
                launch_arguments={"frame_id": "livox_frame"}.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_scan")),
                parameters=[scan_config],
                remappings=[
                    ("cloud_in", "/livox/lidar"),
                    ("scan", "/scan"),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[amcl_params_file],
            ),
            Node(
                package="ranger_nav",
                executable="amcl_tf_keepalive_node.py",
                name="amcl_tf_keepalive",
                output="screen",
                parameters=[
                    {
                        "global_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="day3_localization_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planning_launch),
                condition=IfCondition(LaunchConfiguration("start_planning")),
                launch_arguments={
                    "route_file": LaunchConfiguration("route_file"),
                    "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                    "planning_params_file": LaunchConfiguration("planning_params_file"),
                    "map_file": map_file,
                    "map": "false",
                    "rviz": LaunchConfiguration("rviz"),
                    "publish_viz_anchor": "false",
                    "show_all_steps": "true",
                }.items(),
            ),
        ]
    )
