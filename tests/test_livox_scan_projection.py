import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_safety.livox_scan_projection import (
    ScanProjectionConfig,
    project_points_to_scan_ranges,
)


class LivoxScanProjectionTest(unittest.TestCase):
    def test_projects_height_filtered_points_into_nearest_2d_ranges(self) -> None:
        config = ScanProjectionConfig(
            min_height_m=-0.25,
            max_height_m=0.80,
            angle_min_rad=-math.pi,
            angle_max_rad=math.pi,
            angle_increment_rad=math.pi / 4.0,
            range_min_m=0.10,
            range_max_m=6.0,
        )

        ranges = project_points_to_scan_ranges(
            [
                (1.00, 0.00, 0.00),
                (0.60, 0.00, 0.10),
                (0.00, 1.00, 0.00),
                (0.20, 0.00, 1.20),
            ],
            config,
        )

        self.assertEqual(len(ranges), 8)
        self.assertAlmostEqual(ranges[4], 0.60)
        self.assertAlmostEqual(ranges[6], 1.00)
        self.assertTrue(math.isinf(ranges[5]))

    def test_applies_lidar_to_body_translation_before_height_crop(self) -> None:
        config = ScanProjectionConfig(
            min_height_m=-0.25,
            max_height_m=0.80,
            angle_increment_rad=math.pi / 2.0,
            sensor_to_body_x_m=-0.01,
            sensor_to_body_y_m=-0.02,
            sensor_to_body_z_m=0.04,
        )

        ranges = project_points_to_scan_ranges([(1.01, 0.02, -0.04)], config)

        self.assertAlmostEqual(ranges[2], 1.00)


if __name__ == "__main__":
    unittest.main()
