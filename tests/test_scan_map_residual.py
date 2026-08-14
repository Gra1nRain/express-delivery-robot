import math
import pathlib
import sys
import unittest

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.scan_map_residual import (
    OccupancyDistanceField,
    ScanMatchConfig,
    StationaryResidualSample,
    classify_stationary_residuals,
    correction_about_sensor_as_transform,
    laser_scan_points,
    match_scan_to_map,
    velocity_is_stationary,
)


class ScanMapResidualTest(unittest.TestCase):
    def test_converts_sensor_pivot_correction_to_global_transform(self) -> None:
        correction = correction_about_sensor_as_transform(
            sensor_xy_m=(2.0, 3.0),
            dx_m=0.10,
            dy_m=-0.20,
            dyaw_rad=math.pi / 2.0,
        )

        self.assertAlmostEqual(correction.x, 5.10)
        self.assertAlmostEqual(correction.y, 0.80)
        self.assertAlmostEqual(correction.yaw, math.pi / 2.0)

    def test_ros_monitor_is_observation_only(self) -> None:
        node_text = (
            REPO_ROOT
            / "src"
            / "competition_localization"
            / "competition_localization"
            / "scan_map_residual_monitor_node.py"
        ).read_text(encoding="utf-8")
        setup_text = (
            REPO_ROOT / "src" / "competition_localization" / "setup.py"
        ).read_text(encoding="utf-8")

        self.assertIn("OccupancyGrid", node_text)
        self.assertIn("LaserScan", node_text)
        self.assertIn("scan_map_residual_monitor_node", setup_text)
        self.assertNotIn("TransformBroadcaster", node_text)
        self.assertNotIn("Twist", node_text)
        self.assertNotIn("/cmd_vel", node_text)

    def test_recovers_translation_offset_against_static_map(self) -> None:
        resolution = 0.05
        width = height = 200
        occupancy = np.zeros((height, width), dtype=np.int16)
        origin_x = origin_y = -5.0
        wall_x = round((2.0 - origin_x) / resolution)
        wall_y = round((3.0 - origin_y) / resolution)
        wall_x_m = origin_x + (wall_x + 0.5) * resolution
        wall_y_m = origin_y + (wall_y + 0.5) * resolution
        occupancy[20:181, wall_x] = 100
        occupancy[wall_y, 20:181] = 100
        field = OccupancyDistanceField.from_occupancy(
            occupancy.ravel(),
            width=width,
            height=height,
            resolution_m=resolution,
            origin_x_m=origin_x,
            origin_y_m=origin_y,
            origin_yaw_rad=0.0,
        )
        true_points = np.vstack(
            (
                np.column_stack((np.full(61, wall_x_m), np.linspace(-1.0, 2.0, 61))),
                np.column_stack((np.linspace(-1.0, 2.0, 61), np.full(61, wall_y_m))),
            )
        )
        observed_points = true_points + np.array([0.15, -0.10])

        result = match_scan_to_map(
            field,
            observed_points,
            sensor_xy_m=(0.0, 0.0),
            config=ScanMatchConfig(
                translation_window_m=0.30,
                translation_step_m=0.05,
                yaw_window_rad=0.0,
                yaw_step_rad=0.01,
                fine_translation_window_m=0.05,
                fine_translation_step_m=0.01,
                fine_yaw_window_rad=0.0,
                fine_yaw_step_rad=0.01,
                max_residual_m=0.50,
                inlier_threshold_m=0.10,
                min_points=30,
            ),
        )

        self.assertAlmostEqual(result.correction_x_m, -0.15, delta=0.03)
        self.assertAlmostEqual(result.correction_y_m, 0.10, delta=0.03)
        self.assertLess(result.best_median_residual_m, 0.03)
        self.assertGreater(result.inlier_ratio, 0.90)
        self.assertFalse(result.search_boundary_hit)

    def test_classifies_correction_growth_while_stationary_as_drift(self) -> None:
        samples = (
            StationaryResidualSample(0.0, 0.01, -0.01, 0.0),
            StationaryResidualSample(60.0, 0.05, -0.02, 0.01),
            StationaryResidualSample(120.0, 0.10, -0.03, 0.02),
            StationaryResidualSample(180.0, 0.15, -0.04, 0.03),
        )

        assessment = classify_stationary_residuals(samples)

        self.assertEqual(assessment.classification, "stationary_drift")
        self.assertGreater(assessment.translation_change_m, 0.10)
        self.assertGreater(assessment.duration_s, 120.0)

    def test_recovers_yaw_offset_about_lidar_origin(self) -> None:
        resolution = 0.05
        width = height = 200
        origin_x = origin_y = -5.0
        occupancy = np.zeros((height, width), dtype=np.int16)
        wall_x = 140
        wall_y = 160
        wall_x_m = origin_x + (wall_x + 0.5) * resolution
        wall_y_m = origin_y + (wall_y + 0.5) * resolution
        occupancy[80:161, wall_x] = 100
        occupancy[wall_y, 80:141] = 100
        field = OccupancyDistanceField.from_occupancy(
            occupancy.ravel(),
            width=width,
            height=height,
            resolution_m=resolution,
            origin_x_m=origin_x,
            origin_y_m=origin_y,
            origin_yaw_rad=0.0,
        )
        true_points = np.vstack(
            (
                np.column_stack(
                    (np.full(81, wall_x_m), np.linspace(-0.975, 3.025, 81))
                ),
                np.column_stack(
                    (np.linspace(-0.975, 2.025, 61), np.full(61, wall_y_m))
                ),
            )
        )
        yaw_error = math.radians(4.0)
        rotation = np.array(
            (
                (math.cos(yaw_error), -math.sin(yaw_error)),
                (math.sin(yaw_error), math.cos(yaw_error)),
            )
        )
        observed_points = true_points @ rotation.T

        result = match_scan_to_map(
            field,
            observed_points,
            sensor_xy_m=(0.0, 0.0),
            config=ScanMatchConfig(
                translation_window_m=0.0,
                translation_step_m=0.05,
                yaw_window_rad=math.radians(8.0),
                yaw_step_rad=math.radians(1.0),
                fine_translation_window_m=0.0,
                fine_translation_step_m=0.01,
                fine_yaw_window_rad=math.radians(1.0),
                fine_yaw_step_rad=math.radians(0.25),
                min_points=30,
            ),
        )

        self.assertAlmostEqual(math.degrees(result.correction_yaw_rad), -4.0, delta=0.5)

    def test_classifies_stable_nonzero_correction_as_fixed_anchor_offset(self) -> None:
        samples = (
            StationaryResidualSample(0.0, 0.12, -0.02, math.radians(0.5)),
            StationaryResidualSample(90.0, 0.13, -0.02, math.radians(0.6)),
            StationaryResidualSample(180.0, 0.12, -0.01, math.radians(0.4)),
        )

        assessment = classify_stationary_residuals(samples)

        self.assertEqual(assessment.classification, "fixed_anchor_offset")

    def test_laser_scan_conversion_filters_invalid_ranges(self) -> None:
        points = laser_scan_points(
            (float("nan"), 0.05, 1.0, float("inf"), 2.0),
            angle_min_rad=-0.2,
            angle_increment_rad=0.1,
            range_min_m=0.10,
            range_max_m=1.50,
            max_points=10,
        )

        self.assertEqual(points.shape, (1, 2))
        self.assertAlmostEqual(points[0, 0], 1.0)
        self.assertAlmostEqual(points[0, 1], 0.0)

    def test_search_boundary_result_is_not_reported_as_confident(self) -> None:
        occupancy = np.zeros((100, 100), dtype=np.int16)
        occupancy[:, 60] = 100
        field = OccupancyDistanceField.from_occupancy(
            occupancy.ravel(),
            width=100,
            height=100,
            resolution_m=0.05,
            origin_x_m=-2.5,
            origin_y_m=-2.5,
            origin_yaw_rad=0.0,
        )
        wall_x_m = -2.5 + (60.5 * 0.05)
        points = np.column_stack(
            (np.full(80, wall_x_m + 1.0), np.linspace(-1.5, 1.5, 80))
        )

        result = match_scan_to_map(
            field,
            points,
            sensor_xy_m=(0.0, 0.0),
            config=ScanMatchConfig(
                translation_window_m=0.30,
                translation_step_m=0.05,
                yaw_window_rad=0.0,
                yaw_step_rad=0.1,
                fine_translation_window_m=0.05,
                fine_translation_step_m=0.01,
                fine_yaw_window_rad=0.0,
                fine_yaw_step_rad=0.1,
                max_residual_m=1.5,
                inlier_threshold_m=1.5,
                min_points=30,
            ),
        )

        self.assertTrue(result.search_boundary_hit)
        self.assertFalse(result.confident)

    def test_stale_zero_velocity_cannot_create_stationary_evidence(self) -> None:
        self.assertFalse(
            velocity_is_stationary(
                linear_speed_mps=0.0,
                yaw_rate_radps=0.0,
                odom_age_s=1.0,
                max_odom_age_s=0.5,
                linear_threshold_mps=0.01,
                yaw_rate_threshold_radps=0.01,
            )
        )


if __name__ == "__main__":
    unittest.main()
