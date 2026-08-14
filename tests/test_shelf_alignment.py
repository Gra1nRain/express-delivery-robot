import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.shelf_alignment import (
    ShelfAlignmentConfig,
    estimate_shelf_from_scan,
)


def synthetic_right_shelf_scan(
    *,
    distance_m: float,
    heading_deg: float = 0.0,
) -> tuple[list[float], float, float]:
    angle_min = -math.pi / 2.0
    angle_increment = math.radians(0.5)
    ranges = [math.inf] * 361
    heading = math.radians(heading_deg)
    tangent = (math.cos(heading), math.sin(heading))
    normal = (-tangent[1], tangent[0])
    point_on_line = (0.0, -distance_m)
    line_offset = normal[0] * point_on_line[0] + normal[1] * point_on_line[1]
    for index in range(len(ranges)):
        angle = angle_min + index * angle_increment
        ray = (math.cos(angle), math.sin(angle))
        denominator = normal[0] * ray[0] + normal[1] * ray[1]
        if abs(denominator) < 1e-9:
            continue
        distance = line_offset / denominator
        if distance <= 0.0:
            continue
        x = distance * ray[0]
        if -0.60 <= x <= 0.80:
            ranges[index] = distance
    return ranges, angle_min, angle_increment


class ShelfAlignmentEstimatorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ShelfAlignmentConfig(
            side="RIGHT",
            min_range_m=0.10,
            max_range_m=1.50,
            min_longitudinal_m=-0.60,
            max_longitudinal_m=0.80,
            min_side_distance_m=0.20,
            max_side_distance_m=1.00,
            min_points=12,
            min_span_m=0.30,
            max_residual_m=0.025,
            max_heading_error_rad=math.radians(15.0),
        )

    def test_recovers_right_shelf_distance_and_heading_with_outliers(self) -> None:
        ranges, angle_min, increment = synthetic_right_shelf_scan(
            distance_m=0.53,
            heading_deg=3.0,
        )
        for index in (8, 23, 51, 76):
            ranges[index] = 0.31 + index * 0.001

        observation = estimate_shelf_from_scan(
            ranges,
            angle_min_rad=angle_min,
            angle_increment_rad=increment,
            config=self.config,
        )

        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertAlmostEqual(observation.side_distance_m, 0.53, delta=0.02)
        self.assertAlmostEqual(observation.heading_error_rad, math.radians(3.0), delta=math.radians(1.0))
        self.assertGreaterEqual(observation.point_count, self.config.min_points)
        self.assertLessEqual(observation.residual_rms_m, self.config.max_residual_m)

    def test_rejects_sparse_returns_instead_of_guessing(self) -> None:
        ranges = [math.inf] * 361
        for index in range(0, 24, 4):
            ranges[index] = 0.53

        observation = estimate_shelf_from_scan(
            ranges,
            angle_min_rad=-math.pi / 2.0,
            angle_increment_rad=math.radians(0.5),
            config=self.config,
        )

        self.assertIsNone(observation)


if __name__ == "__main__":
    unittest.main()
