#!/usr/bin/env python3
"""Start only the additive avoidance adapter on an existing Day5 chain."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    dry_run = _is_true(LaunchConfiguration("dry_run").perform(context))
    chassis_output = _is_true(
        LaunchConfiguration("enable_chassis_output").perform(context)
    )
    operation_mode = LaunchConfiguration("operation_mode").perform(context)
    if not dry_run or chassis_output or operation_mode != "dry_run":
        raise RuntimeError(
            "This additive launch is dry-run only; chassis output remains blocked."
        )

    return [
        Node(
            package="competition_avoidance",
            executable="avoidance_manager_node",
            name="avoidance_manager",
            output="screen",
            parameters=[LaunchConfiguration("avoidance_params_file")],
        ),
    ]


def generate_launch_description():
    competition_ws = os.environ.get("COMPETITION_WS", "/home/agilex/competition_ws")
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("enable_chassis_output", default_value="false"),
            DeclareLaunchArgument("operation_mode", default_value="dry_run"),
            DeclareLaunchArgument(
                "avoidance_params_file",
                default_value=os.path.join(
                    competition_ws,
                    "config",
                    "avoidance",
                    "avoidance_params.yaml",
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
