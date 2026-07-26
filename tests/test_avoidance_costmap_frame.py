import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.costmap_frame import (
    transform_grid_origin,
    yaw_quaternion,
)


class AvoidanceCostmapFrameTest(unittest.TestCase):
    def test_transforms_body_grid_origin_at_cloud_pose(self) -> None:
        x, y = transform_grid_origin(
            origin_x_m=-0.5,
            origin_y_m=-2.5,
            translation_x_m=10.0,
            translation_y_m=20.0,
            yaw_rad=math.pi / 2.0,
        )

        self.assertAlmostEqual(x, 12.5)
        self.assertAlmostEqual(y, 19.5)

    def test_yaw_quaternion_is_planar_and_normalized(self) -> None:
        x, y, z, w = yaw_quaternion(math.pi / 3.0)

        self.assertEqual(x, 0.0)
        self.assertEqual(y, 0.0)
        self.assertAlmostEqual(z * z + w * w, 1.0)


if __name__ == "__main__":
    unittest.main()
