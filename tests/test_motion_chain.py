import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
for package in (
    "competition_bringup",
    "competition_control",
    "competition_localization",
    "competition_planning",
    "competition_safety",
):
    sys.path.insert(0, str(REPO_ROOT / "src" / package))

from competition_bringup.motion_validation import SimulationConfig, simulate_motion_chain
from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    MPPIParams,
)
from competition_safety.supervisor import SafetyLimits


class MotionChainTest(unittest.TestCase):
    def test_closed_loop_reaches_goal_through_safety_exit(self) -> None:
        points = tuple(
            ControlTrajectoryPoint(
                x=index * 0.05,
                y=0.0,
                yaw=0.0,
                s=index * 0.05,
                curvature=0.0,
                v=0.0 if index in {0, 30} else 0.15,
                t=index / 3.0,
                ref_id="finish" if index == 30 else None,
            )
            for index in range(31)
        )
        trajectory = ControlTrajectory("map", "integration_straight", points)
        dt = 0.05

        report = simulate_motion_chain(
            trajectory,
            MPPIParams(
                control_dt_s=dt,
                horizon_steps=20,
                rollout_count=256,
                iterations=2,
                max_speed_mps=0.20,
                max_acceleration_mps2=0.20,
                max_deceleration_mps2=0.30,
                min_turning_radius_m=0.81,
                max_curvature_rate_1pmps=0.80,
            ),
            SafetyLimits(
                max_speed_mps=0.20,
                max_acceleration_mps2=0.20,
                max_deceleration_mps2=0.30,
                min_turning_radius_m=0.81,
                nominal_period_s=dt,
            ),
            SimulationConfig(dt_s=dt, max_steps=1_000, random_seed=19),
        )

        self.assertTrue(report.completed)
        self.assertEqual(report.safe_hold_count, 0)
        self.assertLessEqual(report.max_speed_mps, 0.20 + 1e-9)
        self.assertLess(report.final_position_error_m, 0.10)
        self.assertEqual(report.final_speed_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
