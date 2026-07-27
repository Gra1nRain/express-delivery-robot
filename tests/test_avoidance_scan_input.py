import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.perception import ObstacleDetection
from competition_avoidance.scan_input import (
    mark_planar_detection,
    scan_ranges_to_points,
)


class AvoidanceScanInputTest(unittest.TestCase):
    def test_converts_only_finite_in_range_samples_to_body_points(self) -> None:
        points = scan_ranges_to_points(
            ranges=(1.0, math.inf, 0.05, 2.0, math.nan, 7.0),
            angle_min_rad=-math.pi / 2.0,
            angle_increment_rad=math.pi / 4.0,
            range_min_m=0.10,
            range_max_m=6.0,
        )

        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], -1.0, places=6)
        self.assertEqual(points[0][2], 0.0)
        self.assertAlmostEqual(points[1][0], math.sqrt(2.0), places=6)
        self.assertAlmostEqual(points[1][1], math.sqrt(2.0), places=6)
        self.assertEqual(points[1][2], 0.0)

    def test_rejects_invalid_scan_metadata(self) -> None:
        with self.assertRaises(ValueError):
            scan_ranges_to_points(
                ranges=(1.0,),
                angle_min_rad=0.0,
                angle_increment_rad=0.0,
                range_min_m=0.10,
                range_max_m=6.0,
            )
        with self.assertRaises(ValueError):
            scan_ranges_to_points(
                ranges=(1.0,),
                angle_min_rad=0.0,
                angle_increment_rad=0.1,
                range_min_m=6.0,
                range_max_m=0.10,
            )

    def test_marks_planar_cluster_as_motion_candidate_without_changing_geometry(
        self,
    ) -> None:
        detection = ObstacleDetection(
            x=2.0,
            y=0.3,
            z=0.0,
            length_m=0.2,
            width_m=0.3,
            height_m=0.0,
            point_count=4,
            classification="UNKNOWN",
            confidence=0.25,
        )

        candidate = mark_planar_detection(detection)

        self.assertEqual(candidate.x, detection.x)
        self.assertEqual(candidate.y, detection.y)
        self.assertEqual(candidate.radius_m, detection.radius_m)
        self.assertEqual(candidate.point_count, detection.point_count)
        self.assertEqual(candidate.classification, "SCAN_CANDIDATE")
        self.assertEqual(candidate.confidence, 0.40)


if __name__ == "__main__":
    unittest.main()
