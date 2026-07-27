import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_safety.proximity_stop import (
    LocalGridConfig,
    ProximityStopConfig,
    evaluate_local_clearance,
    laser_scan_points,
)


class LocalClearanceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.stop_config = ProximityStopConfig(
            x_min_m=0.25,
            stop_distance_m=0.85,
            front_half_angle_rad=0.4363,
            lateral_half_width_m=0.45,
            z_min_m=-0.25,
            z_max_m=0.80,
            min_points=3,
        )
        self.grid_config = LocalGridConfig(
            resolution_m=0.10,
            x_min_m=-0.50,
            x_max_m=2.00,
            y_min_m=-1.00,
            y_max_m=1.00,
            inflation_radius_m=0.20,
            scan_bin_count=180,
            scan_range_min_m=0.10,
            scan_range_max_m=3.00,
        )

    def test_table_face_stops_and_is_visible_in_costmap_and_scan(self) -> None:
        result = evaluate_local_clearance(
            [
                (0.70, -0.20, 0.30),
                (0.70, -0.10, 0.30),
                (0.70, 0.00, 0.30),
                (0.70, 0.10, 0.30),
                (0.70, 0.20, 0.30),
            ],
            self.stop_config,
            self.grid_config,
        )

        self.assertTrue(result.stop)
        self.assertEqual(result.point_count, 5)
        self.assertAlmostEqual(result.nearest_obstacle_distance_m, 0.70)
        self.assertIn(100, result.costmap.data)
        self.assertIn(50, result.costmap.data)
        self.assertTrue(any(math.isfinite(value) for value in result.scan_ranges_m))

    def test_side_obstacle_is_visible_without_false_corridor_stop(self) -> None:
        result = evaluate_local_clearance(
            [
                (0.70, 0.80, 0.30),
                (0.75, 0.80, 0.30),
                (0.80, 0.80, 0.30),
            ],
            self.stop_config,
            self.grid_config,
        )

        self.assertFalse(result.stop)
        self.assertEqual(result.point_count, 0)
        self.assertIsNone(result.nearest_obstacle_distance_m)
        self.assertIn(100, result.costmap.data)
        self.assertTrue(any(math.isfinite(value) for value in result.scan_ranges_m))

    def test_laser_scan_ranges_become_planar_obstacle_points(self) -> None:
        points = laser_scan_points(
            [math.inf, 1.0, float("nan"), 0.05, 3.5],
            angle_min_rad=-math.pi / 2.0,
            angle_increment_rad=math.pi / 4.0,
            range_min_m=0.10,
            range_max_m=3.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], math.sqrt(0.5))
        self.assertAlmostEqual(points[0][1], -math.sqrt(0.5))
        self.assertEqual(points[0][2], 0.0)


if __name__ == "__main__":
    unittest.main()
