import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.planar_transform import PlanarTransform
from competition_localization.startup_alignment import (
    AlignmentConfig,
    AlignmentMode,
    AlignmentObservation,
    AlignmentPhase,
    StartupAlignment,
)


def observation(
    stamp_s: float,
    x_m: float,
    y_m: float,
    yaw_deg: float,
) -> AlignmentObservation:
    return AlignmentObservation(
        stamp_s=stamp_s,
        stationary=True,
        confident=True,
        search_boundary_hit=False,
        correction=PlanarTransform(x_m, y_m, math.radians(yaw_deg)),
        best_median_residual_m=0.02,
        inlier_ratio=0.75,
    )


class StartupAlignmentTest(unittest.TestCase):
    def test_limits_use_vehicle_displacement_not_global_pivot_transform(self) -> None:
        alignment = StartupAlignment(AlignmentConfig(required_samples=2))
        alignment.begin(
            mode=AlignmentMode.CHECKPOINT,
            reference="far_checkpoint",
            anchor_revision=3,
        )
        sample = AlignmentObservation(
            stamp_s=1.0,
            stationary=True,
            confident=True,
            search_boundary_hit=False,
            correction=PlanarTransform(0.10, -0.35, math.radians(2.0)),
            residual_correction=PlanarTransform(0.05, 0.0, math.radians(2.0)),
            best_median_residual_m=0.02,
            inlier_ratio=0.75,
        )

        alignment.observe(sample)
        decision = alignment.observe(
            AlignmentObservation(**{**sample.__dict__, "stamp_s": 2.0})
        )

        self.assertEqual(decision.phase, AlignmentPhase.APPLYING)
        self.assertEqual(decision.correction, sample.correction)

    def test_stable_startup_samples_produce_one_robust_correction(self) -> None:
        alignment = StartupAlignment(
            AlignmentConfig(
                required_samples=3,
                verification_samples=2,
                max_sample_translation_spread_m=0.03,
                max_sample_yaw_spread_rad=math.radians(0.5),
            )
        )
        alignment.begin(
            mode=AlignmentMode.STARTUP,
            reference="startup",
            anchor_revision=4,
        )

        alignment.observe(observation(1.0, 0.10, -0.04, 1.0))
        alignment.observe(observation(2.0, 0.11, -0.05, 0.8))
        decision = alignment.observe(observation(3.0, 0.09, -0.05, 1.2))

        self.assertEqual(decision.phase, AlignmentPhase.APPLYING)
        self.assertEqual(decision.expected_anchor_revision, 4)
        self.assertIsNotNone(decision.correction)
        assert decision.correction is not None
        self.assertAlmostEqual(decision.correction.x, 0.10)
        self.assertAlmostEqual(decision.correction.y, -0.05)
        self.assertAlmostEqual(decision.correction.yaw, math.radians(1.0))

    def test_applied_correction_requires_clean_verification_before_ready(self) -> None:
        alignment = StartupAlignment(
            AlignmentConfig(
                required_samples=2,
                verification_samples=2,
                max_sample_translation_spread_m=0.03,
                max_sample_yaw_spread_rad=math.radians(0.5),
            )
        )
        alignment.begin(
            mode=AlignmentMode.STARTUP,
            reference="startup",
            anchor_revision=8,
        )
        alignment.observe(observation(1.0, 0.10, -0.04, 1.0))
        alignment.observe(observation(2.0, 0.11, -0.04, 1.1))

        applying = alignment.applied(new_anchor_revision=9)
        self.assertEqual(applying.phase, AlignmentPhase.VERIFYING)

        alignment.observe(observation(3.0, 0.01, -0.01, 0.2))
        ready = alignment.observe(observation(4.0, 0.02, -0.01, 0.3))

        self.assertEqual(ready.phase, AlignmentPhase.READY)
        self.assertEqual(ready.expected_anchor_revision, 9)

    def test_motion_clears_partial_evidence_before_a_correction_can_apply(self) -> None:
        alignment = StartupAlignment(AlignmentConfig(required_samples=3))
        alignment.begin(
            mode=AlignmentMode.STARTUP,
            reference="startup",
            anchor_revision=1,
        )
        alignment.observe(observation(1.0, 0.10, -0.04, 1.0))
        alignment.observe(observation(2.0, 0.10, -0.04, 1.0))
        moving = observation(3.0, 0.10, -0.04, 1.0)
        moving = AlignmentObservation(**{**moving.__dict__, "stationary": False})

        alignment.observe(moving)
        alignment.observe(observation(4.0, 0.10, -0.04, 1.0))
        decision = alignment.observe(observation(5.0, 0.10, -0.04, 1.0))

        self.assertEqual(decision.phase, AlignmentPhase.COLLECTING)

    def test_checkpoint_correction_over_safe_limit_is_rejected(self) -> None:
        alignment = StartupAlignment(AlignmentConfig(required_samples=2))
        alignment.begin(
            mode=AlignmentMode.CHECKPOINT,
            reference="drop_front",
            anchor_revision=3,
        )
        alignment.observe(observation(1.0, 0.25, 0.0, 1.0))
        decision = alignment.observe(observation(2.0, 0.25, 0.0, 1.0))

        self.assertEqual(decision.phase, AlignmentPhase.REJECTED)
        self.assertIsNone(decision.correction)

    def test_failed_post_apply_verification_requires_rollback(self) -> None:
        alignment = StartupAlignment(
            AlignmentConfig(required_samples=2, verification_samples=2)
        )
        alignment.begin(
            mode=AlignmentMode.STARTUP,
            reference="startup",
            anchor_revision=5,
        )
        alignment.observe(observation(1.0, 0.10, 0.0, 1.0))
        alignment.observe(observation(2.0, 0.10, 0.0, 1.0))
        alignment.applied(new_anchor_revision=6)
        alignment.observe(observation(3.0, 0.06, 0.0, 1.0))
        decision = alignment.observe(observation(4.0, 0.06, 0.0, 1.0))

        self.assertEqual(decision.phase, AlignmentPhase.REJECTED)
        self.assertTrue(decision.rollback_required)

    def test_lock_aborts_collection_and_prevents_late_correction(self) -> None:
        alignment = StartupAlignment(AlignmentConfig(required_samples=2))
        alignment.begin(
            mode=AlignmentMode.STARTUP,
            reference="startup",
            anchor_revision=2,
        )
        alignment.observe(observation(1.0, 0.10, 0.0, 1.0))

        locked = alignment.lock("route enabled")
        ignored = alignment.observe(observation(2.0, 0.10, 0.0, 1.0))

        self.assertEqual(locked.phase, AlignmentPhase.LOCKED)
        self.assertEqual(ignored.phase, AlignmentPhase.LOCKED)
        self.assertIsNone(ignored.correction)

    def test_anchor_writer_rejection_fails_closed(self) -> None:
        alignment = StartupAlignment(AlignmentConfig(required_samples=2))
        alignment.begin(
            mode=AlignmentMode.CHECKPOINT,
            reference="pickup_front",
            anchor_revision=7,
        )
        alignment.observe(observation(1.0, 0.05, 0.0, 0.5))
        alignment.observe(observation(2.0, 0.05, 0.0, 0.5))

        rejected = alignment.reject("anchor revision mismatch")

        self.assertEqual(rejected.phase, AlignmentPhase.REJECTED)
        self.assertIn("revision mismatch", rejected.reason)


if __name__ == "__main__":
    unittest.main()
