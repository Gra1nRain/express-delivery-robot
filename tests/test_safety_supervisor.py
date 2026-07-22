import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_control.mppi_controller import BodyCommand
from competition_safety.supervisor import SafetyContext, SafetyLimits, SafetySupervisor


def _command(*, speed: float = 0.10, yaw_rate: float = 0.05) -> BodyCommand:
    curvature = yaw_rate / speed if abs(speed) > 1e-9 else 0.0
    return BodyCommand(
        linear_x_mps=speed,
        yaw_rate_radps=yaw_rate,
        curvature_1pm=curvature,
        target_index=10,
        lateral_error_m=0.02,
        heading_error_rad=math.radians(2.0),
        status="TRACKING",
    )


def _healthy_context(**overrides) -> SafetyContext:
    values = {
        "now_s": 10.0,
        "command_stamp_s": 9.98,
        "state_stamp_s": 9.98,
        "measured_speed_mps": 0.10,
        "estop_ready": True,
        "remote_ready": True,
        "state_valid": True,
        "avoidance_stop": False,
        "chassis_fault": False,
    }
    values.update(overrides)
    return SafetyContext(**values)


class SafetySupervisorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.supervisor = SafetySupervisor(
            SafetyLimits(
                command_timeout_s=0.15,
                state_timeout_s=0.15,
                max_speed_mps=0.20,
                max_acceleration_mps2=0.20,
                max_deceleration_mps2=0.30,
                min_turning_radius_m=0.81,
                max_lateral_error_m=0.15,
                max_heading_error_rad=math.radians(20.0),
                nominal_period_s=0.05,
            )
        )

    def test_healthy_command_passes_the_safety_exit(self) -> None:
        output = self.supervisor.filter_command(_command(), _healthy_context())

        self.assertEqual(output.status, "SAFE_ACTIVE")
        self.assertAlmostEqual(output.linear_x_mps, 0.10)
        self.assertAlmostEqual(output.yaw_rate_radps, 0.05)
        self.assertEqual(output.reasons, ())

    def test_invalid_or_stale_state_forces_safe_hold(self) -> None:
        invalid = self.supervisor.filter_command(
            _command(),
            _healthy_context(state_valid=False),
        )
        stale = self.supervisor.filter_command(
            _command(),
            _healthy_context(now_s=11.0),
        )

        self.assertEqual(invalid.status, "SAFE_HOLD")
        self.assertEqual((invalid.linear_x_mps, invalid.yaw_rate_radps), (0.0, 0.0))
        self.assertIn("invalid_state", invalid.reasons)
        self.assertIn("stale_command", stale.reasons)
        self.assertIn("stale_state", stale.reasons)

    def test_limits_acceleration_and_curvature_before_chassis(self) -> None:
        unsafe = _command(speed=0.50, yaw_rate=1.0)

        output = self.supervisor.filter_command(
            unsafe,
            _healthy_context(measured_speed_mps=0.0),
        )

        self.assertEqual(output.status, "SAFE_LIMITED")
        self.assertLessEqual(output.linear_x_mps, 0.20 * 0.05 + 1e-9)
        self.assertLessEqual(
            abs(output.yaw_rate_radps),
            abs(output.linear_x_mps) / 0.81 + 1e-9,
        )
        self.assertIn("speed_limited", output.reasons)
        self.assertIn("acceleration_limited", output.reasons)
        self.assertIn("curvature_limited", output.reasons)

    def test_goal_stop_respects_deceleration_limit_before_safe_stop(self) -> None:
        self.supervisor.filter_command(
            _command(speed=0.20, yaw_rate=0.0),
            _healthy_context(now_s=10.00, measured_speed_mps=0.20),
        )
        goal = _command(speed=0.0, yaw_rate=0.0)
        goal = BodyCommand(
            linear_x_mps=goal.linear_x_mps,
            yaw_rate_radps=goal.yaw_rate_radps,
            curvature_1pm=goal.curvature_1pm,
            target_index=goal.target_index,
            lateral_error_m=goal.lateral_error_m,
            heading_error_rad=goal.heading_error_rad,
            status="GOAL_REACHED",
        )

        first = self.supervisor.filter_command(
            goal,
            _healthy_context(now_s=10.05, measured_speed_mps=0.20),
        )

        self.assertEqual(first.status, "SAFE_LIMITED")
        self.assertAlmostEqual(first.linear_x_mps, 0.185)
        self.assertEqual(first.yaw_rate_radps, 0.0)
        self.assertIn("goal_decelerating", first.reasons)


if __name__ == "__main__":
    unittest.main()
