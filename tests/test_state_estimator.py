import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_localization"))

from competition_localization.state_estimator import (
    Pose2D,
    StateEstimator,
    StateEstimatorLimits,
    StateObservation,
    Velocity2D,
    predict_observation_to_time,
)


class StateEstimatorTest(unittest.TestCase):
    def test_rejects_tf_jump_without_replacing_last_continuous_state(self) -> None:
        estimator = StateEstimator(
            StateEstimatorLimits(
                pose_timeout_s=0.20,
                velocity_timeout_s=0.20,
                max_position_jump_m=0.25,
                max_heading_jump_rad=math.radians(20.0),
                max_linear_speed_mps=2.0,
                max_yaw_rate_radps=3.259,
                rate_margin_m=0.03,
                rate_margin_rad=math.radians(3.0),
            )
        )
        first = estimator.update(
            StateObservation(
                pose=Pose2D(0.0, 0.0, 0.0),
                velocity=Velocity2D(0.10, 0.0),
                pose_stamp_s=1.00,
                velocity_stamp_s=1.00,
            ),
            now_s=1.05,
        )
        continuous = estimator.update(
            StateObservation(
                pose=Pose2D(0.02, 0.0, 0.01),
                velocity=Velocity2D(0.10, 0.01),
                pose_stamp_s=1.10,
                velocity_stamp_s=1.10,
            ),
            now_s=1.12,
        )
        jumped = estimator.update(
            StateObservation(
                pose=Pose2D(0.03, 0.0, -1.48),
                velocity=Velocity2D(0.10, 0.0),
                pose_stamp_s=1.20,
                velocity_stamp_s=1.20,
            ),
            now_s=1.22,
        )

        self.assertTrue(first.valid)
        self.assertTrue(continuous.valid)
        self.assertFalse(jumped.valid)
        self.assertIn("heading_jump", jumped.reasons)
        self.assertEqual(jumped.pose, continuous.pose)

    def test_rejects_stale_and_time_reversed_observations(self) -> None:
        estimator = StateEstimator(StateEstimatorLimits())
        accepted = estimator.update(
            StateObservation(
                pose=Pose2D(1.0, 2.0, 0.1),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=5.0,
                velocity_stamp_s=5.0,
            ),
            now_s=5.0,
        )
        stale = estimator.update(
            StateObservation(
                pose=Pose2D(1.0, 2.0, 0.1),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=5.0,
                velocity_stamp_s=5.0,
            ),
            now_s=6.0,
        )
        reversed_time = estimator.update(
            StateObservation(
                pose=Pose2D(1.1, 2.0, 0.1),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=4.9,
                velocity_stamp_s=5.1,
            ),
            now_s=5.1,
        )

        self.assertTrue(accepted.valid)
        self.assertIn("stale_pose", stale.reasons)
        self.assertIn("stale_velocity", stale.reasons)
        self.assertIn("pose_time_reversed", reversed_time.reasons)

    def test_reset_accepts_a_new_relocalized_pose_as_the_continuity_baseline(
        self,
    ) -> None:
        estimator = StateEstimator(StateEstimatorLimits())
        initial = estimator.update(
            StateObservation(
                pose=Pose2D(0.0, 0.0, 0.0),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=1.0,
                velocity_stamp_s=1.0,
            ),
            now_s=1.0,
        )
        rejected_jump = estimator.update(
            StateObservation(
                pose=Pose2D(1.0, 0.5, 0.0),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=1.1,
                velocity_stamp_s=1.1,
            ),
            now_s=1.1,
        )

        estimator.reset()
        reanchored = estimator.update(
            StateObservation(
                pose=Pose2D(1.0, 0.5, 0.0),
                velocity=Velocity2D(0.0, 0.0),
                pose_stamp_s=1.2,
                velocity_stamp_s=1.2,
            ),
            now_s=1.2,
        )

        self.assertTrue(initial.valid)
        self.assertFalse(rejected_jump.valid)
        self.assertIn("position_jump", rejected_jump.reasons)
        self.assertTrue(reanchored.valid)
        self.assertEqual(reanchored.pose, Pose2D(1.0, 0.5, 0.0))

    def test_predicts_bounded_delayed_pose_to_current_time(self) -> None:
        observation = StateObservation(
            pose=Pose2D(1.0, 2.0, 0.0),
            velocity=Velocity2D(0.20, 0.0),
            pose_stamp_s=10.0,
            velocity_stamp_s=11.48,
        )

        predicted = predict_observation_to_time(
            observation,
            now_s=11.5,
            max_prediction_s=2.0,
        )
        estimate = StateEstimator(StateEstimatorLimits()).update(
            predicted,
            now_s=11.5,
        )

        self.assertAlmostEqual(predicted.pose.x, 1.30)
        self.assertAlmostEqual(predicted.pose.y, 2.0)
        self.assertAlmostEqual(predicted.pose_stamp_s, 11.5)
        self.assertTrue(estimate.valid)
        self.assertNotIn("stale_pose", estimate.reasons)

    def test_keeps_over_limit_pose_stale(self) -> None:
        observation = StateObservation(
            pose=Pose2D(1.0, 2.0, 0.0),
            velocity=Velocity2D(0.20, 0.0),
            pose_stamp_s=10.0,
            velocity_stamp_s=12.98,
        )

        unchanged = predict_observation_to_time(
            observation,
            now_s=13.0,
            max_prediction_s=2.0,
        )
        estimate = StateEstimator(StateEstimatorLimits()).update(
            unchanged,
            now_s=13.0,
        )

        self.assertEqual(unchanged, observation)
        self.assertFalse(estimate.valid)
        self.assertIn("stale_pose", estimate.reasons)


if __name__ == "__main__":
    unittest.main()
