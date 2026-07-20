import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.planar_transform import PlanarTransform, wrap_angle


class PlanarTransformTest(unittest.TestCase):
    def assertTransformAlmostEqual(
        self,
        actual: PlanarTransform,
        expected: PlanarTransform,
        places: int = 12,
    ) -> None:
        self.assertAlmostEqual(actual.x, expected.x, places=places)
        self.assertAlmostEqual(actual.y, expected.y, places=places)
        self.assertAlmostEqual(wrap_angle(actual.yaw - expected.yaw), 0.0, places=places)

    def test_inverse_composes_to_identity(self) -> None:
        transform = PlanarTransform(x=2.5, y=-1.25, yaw=1.2)

        self.assertTransformAlmostEqual(
            transform.compose(transform.inverse()),
            PlanarTransform(x=0.0, y=0.0, yaw=0.0),
        )

    def test_anchor_derivation_matches_known_pose(self) -> None:
        map_to_base = PlanarTransform(x=2.98, y=-0.77, yaw=0.43)
        odom_to_base = PlanarTransform(x=1.20, y=0.10, yaw=0.05)

        map_to_odom = map_to_base.compose(odom_to_base.inverse())

        self.assertTransformAlmostEqual(map_to_odom.compose(odom_to_base), map_to_base)


if __name__ == "__main__":
    unittest.main()
