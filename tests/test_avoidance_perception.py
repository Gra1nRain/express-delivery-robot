import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.perception import PerceptionConfig, cluster_points


class AvoidancePerceptionTest(unittest.TestCase):
    def test_cone_cluster_survives_ground_and_self_filtering(self) -> None:
        cone_points = [
            (2.0 + dx, dy, z)
            for dx in (-0.08, 0.0, 0.08)
            for dy in (-0.08, 0.0, 0.08)
            for z in (0.05, 0.25, 0.45, 0.60)
        ]
        ground_points = [
            (0.5 + 0.1 * index, -1.0 + 0.1 * index, -0.30)
            for index in range(20)
        ]
        self_points = [
            (0.1, 0.1, 0.1),
            (0.2, -0.1, 0.2),
            (-0.1, 0.0, 0.0),
        ]

        detections = cluster_points(
            [*cone_points, *ground_points, *self_points],
            PerceptionConfig(
                voxel_size_m=0.05,
                cluster_tolerance_m=0.24,
                min_cluster_points=8,
            ),
        )

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertAlmostEqual(detection.x, 2.0, places=2)
        self.assertAlmostEqual(detection.y, 0.0, places=2)
        self.assertEqual(detection.classification, "CONE_CANDIDATE")
        self.assertGreaterEqual(detection.confidence, 0.5)


if __name__ == "__main__":
    unittest.main()
