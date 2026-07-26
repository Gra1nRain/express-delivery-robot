import math
import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_avoidance.obstacle_memory import MapObstacleMemory
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint


class AvoidancePlanningMemoryTest(unittest.TestCase):
    def test_sparse_bucket_remains_available_for_next_replanning_cycle(self) -> None:
        trajectory_data = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_continuous_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                float(point["x"]),
                float(point["y"]),
                float(point["yaw"]),
            )
            for point in trajectory_data["points"]
        )
        planner = LocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / "maps" / "debug" / "map.yaml"),
            LocalReplanConfig(
                lookahead_distance_m=3.0,
                inflation_radius_m=0.45,
                search_padding_m=1.5,
                sample_spacing_m=0.10,
                min_turning_radius_m=0.81,
                step_length_m=0.20,
                curvature_bins=9,
                heading_bins=72,
                goal_position_tolerance_m=0.25,
                goal_heading_tolerance_rad=math.radians(15.0),
                reference_deviation_weight=2.0,
                max_expansions=250_000,
                reference_search_window_points=120,
            ),
        )
        memory = MapObstacleMemory(ttl_s=1.5, resolution_m=0.10)

        first_pose = PathPoint(0.346, 0.125, 0.0)
        memory.update(
            ((2.000, -0.050, -0.070), (2.020, 0.020, -0.070)),
            translation_x_m=first_pose.x,
            translation_y_m=first_pose.y,
            yaw_rad=first_pose.yaw,
            timestamp_s=10.0,
        )

        next_pose = PathPoint(0.546, 0.129, 0.02)
        without_memory = planner.plan(
            reference_path=reference,
            current_pose=next_pose,
            dynamic_obstacle_points=(),
        )
        retained_points = memory.update(
            (),
            translation_x_m=next_pose.x,
            translation_y_m=next_pose.y,
            yaw_rad=next_pose.yaw,
            timestamp_s=10.2,
        )
        result = planner.plan(
            reference_path=reference,
            current_pose=next_pose,
            dynamic_obstacle_points=(
                (
                    next_pose.x
                    + math.cos(next_pose.yaw) * point[0]
                    - math.sin(next_pose.yaw) * point[1],
                    next_pose.y
                    + math.sin(next_pose.yaw) * point[0]
                    + math.cos(next_pose.yaw) * point[1],
                )
                for point in retained_points
            ),
        )

        self.assertEqual(without_memory.status, "REFERENCE_CLEAR")
        self.assertGreaterEqual(len(retained_points), 1)
        self.assertEqual(result.status, "REPLANNED")
        self.assertTrue(result.path_is_navigable)


if __name__ == "__main__":
    unittest.main()
