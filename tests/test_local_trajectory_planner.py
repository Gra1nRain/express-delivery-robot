import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path


def _empty_map() -> OccupancyGridMap:
    width = 100
    height = 80
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=0.10,
        origin_x=-1.0,
        origin_y=-4.0,
        occupied=tuple(False for _ in range(width * height)),
    )


def _straight_reference(length_m: float = 8.0) -> tuple[PathPoint, ...]:
    return tuple(
        PathPoint(x=index * 0.10, y=0.0, yaw=0.0)
        for index in range(round(length_m / 0.10) + 1)
    )


def _table_points() -> tuple[tuple[float, float], ...]:
    return tuple(
        (1.50 + x_index * 0.10, -0.40 + y_index * 0.10)
        for x_index in range(6)
        for y_index in range(9)
    )


class LocalTrajectoryPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = LocalReplanConfig(
            lookahead_distance_m=5.0,
            inflation_radius_m=0.30,
            search_padding_m=3.0,
            sample_spacing_m=0.10,
            min_turning_radius_m=0.81,
            step_length_m=0.20,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.25,
            goal_heading_tolerance_rad=math.radians(15.0),
            reference_deviation_weight=2.0,
            max_expansions=250_000,
        )
        self.reference = _straight_reference()

    def test_clear_reference_is_reused_without_unnecessary_detour(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=(),
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertEqual(result.reference_start_index, 0)
        self.assertEqual(result.rejoin_index, 50)
        self.assertEqual(result.path, self.reference[:51])
        self.assertLess(result.planning_grid_cell_count, 100 * 80)

    def test_table_is_avoided_before_rejoining_global_reference(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=_table_points(),
        )

        self.assertEqual(result.status, "REPLANNED")
        self.assertEqual(result.rejoin_index, 50)
        self.assertEqual(result.dynamic_obstacle_count, len(_table_points()))
        self.assertTrue(result.path_is_navigable)
        self.assertGreater(max(abs(point.y) for point in result.path), 0.70)
        self.assertLess(max(abs(point.y) for point in result.path), 1.50)
        self.assertLess(math.hypot(result.path[-1].x - 5.0, result.path[-1].y), 0.26)
        self.assertLess(abs(result.path[-1].yaw), math.radians(15.1))

    def test_replanned_geometry_reuses_day4_time_parameterization(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)
        local_plan = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=_table_points(),
        )

        trajectory = parameterize_local_path(
            local_plan.path,
            semantic_map={"frame_id": "map"},
            optimizer_config={
                "trajectory_optimizer": {
                    "max_speed_mps": 0.20,
                    "max_acceleration_mps2": 0.20,
                    "max_deceleration_mps2": 0.30,
                    "max_lateral_acceleration_mps2": 0.20,
                }
            },
        )

        self.assertEqual(len(trajectory.points), len(local_plan.path))
        self.assertTrue(
            all(
                following.s > previous.s and following.t > previous.t
                for previous, following in zip(
                    trajectory.points,
                    trajectory.points[1:],
                )
            )
        )
        self.assertLessEqual(
            max(abs(point.curvature) for point in trajectory.points),
            1.0 / 0.81 + 0.02,
        )
        self.assertLessEqual(max(point.v for point in trajectory.points), 0.20)


if __name__ == "__main__":
    unittest.main()
