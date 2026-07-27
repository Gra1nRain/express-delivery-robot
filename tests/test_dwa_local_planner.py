import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.dwa_local_planner import (
    DWAConfig,
    DWALocalPlanner,
    DWAPlanningError,
    DWAVelocity,
    filter_and_downsample_obstacle_points,
    occupied_grid_cell_centers,
)
from competition_planning.semantic_planner import PathPoint


def _straight_reference(length_m: float = 8.0) -> tuple[PathPoint, ...]:
    return tuple(
        PathPoint(index * 0.10, 0.0, 0.0)
        for index in range(round(length_m / 0.10) + 1)
    )


class DWALocalPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = DWAConfig()
        self.planner = DWALocalPlanner(self.config)
        self.reference = _straight_reference()
        self.current_pose = PathPoint(0.0, 0.0, 0.0)
        self.cruise = DWAVelocity(0.20, 0.0)

    def test_clear_reference_selects_a_straight_forward_path(self) -> None:
        result = self.planner.plan(
            reference_path=self.reference,
            current_pose=self.current_pose,
            current_velocity=self.cruise,
            obstacle_points_body=(),
        )

        self.assertEqual(result.status, "DWA_TRACKING")
        self.assertIsNone(result.minimum_clearance_m)
        self.assertGreater(result.path[-1].x, 1.0)
        self.assertLess(max(abs(point.y) for point in result.path), 1e-6)
        self.assertLess(abs(result.selected_velocity.yaw_rate_radps), 1e-9)

    def test_static_obstacle_on_reference_selects_a_collision_free_arc(self) -> None:
        obstacle = tuple(
            (1.40 + dx, dy)
            for dx in (-0.05, 0.0, 0.05)
            for dy in (-0.05, 0.0, 0.05)
        )

        result = self.planner.plan(
            reference_path=self.reference,
            current_pose=self.current_pose,
            current_velocity=self.cruise,
            obstacle_points_body=obstacle,
        )

        self.assertEqual(result.status, "DWA_AVOIDING")
        self.assertGreater(
            abs(result.selected_velocity.yaw_rate_radps),
            1e-3,
        )
        self.assertIsNotNone(result.minimum_clearance_m)
        self.assertGreater(
            result.minimum_clearance_m,
            self.config.obstacle_clearance_m,
        )
        self.assertGreater(max(abs(point.y) for point in result.path), 0.20)

    def test_close_wall_has_no_feasible_path_and_requests_hold_upstream(self) -> None:
        wall = tuple((0.45, -1.50 + index * 0.05) for index in range(61))

        with self.assertRaisesRegex(
            DWAPlanningError,
            "no collision-free",
        ):
            self.planner.plan(
                reference_path=self.reference,
                current_pose=self.current_pose,
                current_velocity=self.cruise,
                obstacle_points_body=wall,
            )

    def test_selected_arc_respects_ranger_turning_radius(self) -> None:
        result = self.planner.plan(
            reference_path=self.reference,
            current_pose=self.current_pose,
            current_velocity=self.cruise,
            obstacle_points_body=((1.40, 0.0),),
        )

        velocity = result.selected_velocity
        if abs(velocity.yaw_rate_radps) > 1e-9:
            radius = velocity.linear_mps / abs(velocity.yaw_rate_radps)
            self.assertGreaterEqual(
                radius + 1e-9,
                self.config.min_turning_radius_m,
            )
        self.assertLessEqual(
            abs(velocity.yaw_rate_radps),
            self.config.max_yaw_rate_radps,
        )

    def test_rejects_invalid_dynamic_window_configuration(self) -> None:
        with self.assertRaises(ValueError):
            DWAConfig(min_turning_radius_m=0.0)
        with self.assertRaises(ValueError):
            DWAConfig(min_speed_mps=0.2, max_speed_mps=0.1)
        with self.assertRaises(ValueError):
            DWAConfig(speed_sample_count=1)

    def test_obstacle_crop_runs_before_point_limit(self) -> None:
        irrelevant_points = tuple(
            (-2.0, -3.0 + 0.01 * index, 0.0) for index in range(300)
        )
        front_obstacle = (1.40, 0.0, 0.0)

        points = filter_and_downsample_obstacle_points(
            (*irrelevant_points, front_obstacle),
            z_min_m=-0.25,
            z_max_m=0.80,
            x_min_m=0.05,
            x_max_m=4.0,
            y_half_width_m=2.5,
            point_stride=1,
            voxel_size_m=0.01,
            max_points=20,
        )

        self.assertEqual(points, (front_obstacle,))

    def test_inflated_costmap_cells_become_dwa_obstacles(self) -> None:
        points = occupied_grid_cell_centers(
            [-1, 50, 100, -1, 49, 50],
            width=3,
            height=2,
            resolution_m=0.10,
            origin_x_m=-0.10,
            origin_y_m=-0.20,
            occupancy_threshold=50,
            x_min_m=0.0,
            x_max_m=1.0,
            y_half_width_m=1.0,
            max_points=10,
        )

        self.assertEqual(len(points), 3)
        for actual, expected in zip(
            points,
            ((0.05, -0.15), (0.15, -0.15), (0.15, -0.05)),
        ):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])


if __name__ == "__main__":
    unittest.main()
