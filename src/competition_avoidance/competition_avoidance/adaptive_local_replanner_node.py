#!/usr/bin/env python3
"""ROS entry point that adds adaptive lookahead without changing planning code."""

from __future__ import annotations

import rclpy

from competition_avoidance.adaptive_local_replanner import (
    AdaptiveLocalTrajectoryPlanner,
)
from competition_planning.local_replanner_node import LocalReplannerNode
from competition_planning.occupancy_grid_planner import OccupancyGridMap


class AdaptiveLocalReplannerNode(LocalReplannerNode):
    """Reuse LocalReplannerNode while replacing only its pure planning seam."""

    def __init__(self) -> None:
        super().__init__()
        fallback_lookahead = float(
            self.declare_parameter("fallback_lookahead_distance_m", 3.5).value
        )
        map_file = str(self.get_parameter("map_file").value)
        self._planner = AdaptiveLocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(map_file),
            self._config,
            (fallback_lookahead,),
        )
        self.get_logger().info(
            "Adaptive local replanner ready: "
            f"lookaheads={self._config.lookahead_distance_m:.2f}m,"
            f"{fallback_lookahead:.2f}m"
        )


def main() -> None:
    rclpy.init()
    node = AdaptiveLocalReplannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
