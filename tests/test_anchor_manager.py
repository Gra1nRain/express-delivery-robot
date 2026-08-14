import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.anchor_manager import (
    AnchorCorrectionMode,
    AnchorManager,
    AnchorSafetyState,
)
from competition_localization.planar_transform import PlanarTransform, wrap_angle


class AnchorManagerTest(unittest.TestCase):
    def assertTransformAlmostEqual(
        self,
        actual: PlanarTransform,
        expected: PlanarTransform,
    ) -> None:
        self.assertAlmostEqual(actual.x, expected.x)
        self.assertAlmostEqual(actual.y, expected.y)
        self.assertAlmostEqual(wrap_angle(actual.yaw - expected.yaw), 0.0)

    def test_versioned_correction_can_be_rolled_back_without_reusing_revision(
        self,
    ) -> None:
        manager = AnchorManager()
        coarse = manager.set_coarse_anchor(
            target_map_base=PlanarTransform(2.0, 1.0, math.radians(20.0)),
            odom_to_base=PlanarTransform(0.5, -0.2, math.radians(5.0)),
            stationary=True,
        )
        self.assertTrue(coarse.applied)
        self.assertEqual(coarse.revision, 1)

        correction = PlanarTransform(0.10, -0.04, math.radians(1.0))
        refined = manager.apply_correction(
            correction=correction,
            expected_revision=1,
            mode=AnchorCorrectionMode.STARTUP,
            safety=AnchorSafetyState(
                stationary=True,
                route_enabled=False,
                checkpoint_hold=False,
            ),
        )
        self.assertTrue(refined.applied)
        self.assertEqual(refined.revision, 2)
        self.assertTransformAlmostEqual(
            refined.transform,
            correction.compose(coarse.transform),
        )

        rolled_back = manager.rollback(
            expected_revision=2,
            safety=AnchorSafetyState(
                stationary=True,
                route_enabled=False,
                checkpoint_hold=False,
            ),
        )
        self.assertTrue(rolled_back.applied)
        self.assertEqual(rolled_back.revision, 3)
        self.assertTransformAlmostEqual(rolled_back.transform, coarse.transform)

    def test_hard_limit_uses_vehicle_displacement_for_pivoted_transform(self) -> None:
        manager = AnchorManager()
        manager.set_coarse_anchor(
            target_map_base=PlanarTransform(10.0, 0.0, 0.0),
            odom_to_base=PlanarTransform(0.0, 0.0, 0.0),
            stationary=True,
        )

        update = manager.apply_correction(
            correction=PlanarTransform(0.10, -0.35, math.radians(2.0)),
            displacement_correction=PlanarTransform(
                0.05,
                0.0,
                math.radians(2.0),
            ),
            expected_revision=1,
            mode=AnchorCorrectionMode.CHECKPOINT,
            safety=AnchorSafetyState(True, True, True),
        )

        self.assertTrue(update.applied)

    def test_stale_revision_and_unsafe_contexts_cannot_change_anchor(self) -> None:
        manager = AnchorManager()
        coarse = manager.set_coarse_anchor(
            target_map_base=PlanarTransform(1.0, 2.0, 0.0),
            odom_to_base=PlanarTransform(0.0, 0.0, 0.0),
            stationary=True,
        )
        assert coarse.transform is not None

        stale = manager.apply_correction(
            correction=PlanarTransform(0.05, 0.0, 0.0),
            expected_revision=0,
            mode=AnchorCorrectionMode.STARTUP,
            safety=AnchorSafetyState(True, False, False),
        )
        moving = manager.apply_correction(
            correction=PlanarTransform(0.05, 0.0, 0.0),
            expected_revision=1,
            mode=AnchorCorrectionMode.STARTUP,
            safety=AnchorSafetyState(False, False, False),
        )
        checkpoint_without_hold = manager.apply_correction(
            correction=PlanarTransform(0.05, 0.0, 0.0),
            expected_revision=1,
            mode=AnchorCorrectionMode.CHECKPOINT,
            safety=AnchorSafetyState(True, True, False),
        )

        self.assertFalse(stale.applied)
        self.assertFalse(moving.applied)
        self.assertFalse(checkpoint_without_hold.applied)
        self.assertEqual(manager.revision, 1)
        self.assertTransformAlmostEqual(manager.transform, coarse.transform)


if __name__ == "__main__":
    unittest.main()
