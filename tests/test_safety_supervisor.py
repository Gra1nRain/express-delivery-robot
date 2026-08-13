import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_control.mppi_controller import BodyCommand
from competition_safety.supervisor import SafetyContext, SafetyLimits, SafetySupervisor


def _command(
    *,
    speed: float = 0.10,
    yaw_rate: float = 0.05,
    lateral_error_m: float = 0.02,
    heading_error_deg: float = 2.0,
    lateral_speed: float = 0.0,
) -> BodyCommand:
    curvature = yaw_rate / speed if abs(speed) > 1e-9 else 0.0
    return BodyCommand(
        linear_x_mps=speed,
        yaw_rate_radps=yaw_rate,
        curvature_1pm=curvature,
        target_index=10,
        lateral_error_m=lateral_error_m,
        heading_error_rad=math.radians(heading_error_deg),
        status="TRACKING",
        linear_y_mps=lateral_speed,
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
        "avoidance_ready": True,
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
                recovery_lateral_error_m=0.10,
                recovery_heading_error_rad=math.radians(10.0),
                recovery_clear_lateral_error_m=0.06,
                recovery_clear_heading_error_rad=math.radians(5.0),
                recovery_speed_mps=0.06,
                max_lateral_error_m=0.40,
                max_heading_error_rad=math.radians(45.0),
                tracking_error_timeout_s=1.0,
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

    def test_missing_avoidance_source_forces_safe_hold(self) -> None:
        missing_avoidance = self.supervisor.filter_command(
            _command(),
            _healthy_context(avoidance_ready=False),
        )

        self.assertEqual(missing_avoidance.status, "SAFE_HOLD")
        self.assertEqual(
            (missing_avoidance.linear_x_mps, missing_avoidance.yaw_rate_radps),
            (0.0, 0.0),
        )
        self.assertIn("avoidance_stale", missing_avoidance.reasons)

    def test_avoidance_stop_forces_safe_hold(self) -> None:
        obstacle_stop = self.supervisor.filter_command(
            _command(),
            _healthy_context(avoidance_stop=True),
        )

        self.assertEqual(obstacle_stop.status, "SAFE_HOLD")
        self.assertEqual(
            (obstacle_stop.linear_x_mps, obstacle_stop.yaw_rate_radps),
            (0.0, 0.0),
        )
        self.assertIn("avoidance_stop", obstacle_stop.reasons)

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

    def test_moderate_tracking_error_slows_down_without_holding(self) -> None:
        output = self.supervisor.filter_command(
            _command(lateral_error_m=0.17),
            _healthy_context(measured_speed_mps=0.05),
        )

        self.assertEqual(output.status, "SAFE_RECOVERY")
        self.assertGreater(output.linear_x_mps, 0.0)
        self.assertLessEqual(output.linear_x_mps, 0.06)
        self.assertIn("tracking_recovery", output.reasons)

    def test_tracking_recovery_uses_hysteresis(self) -> None:
        entered = self.supervisor.filter_command(
            _command(speed=0.06, lateral_error_m=0.12),
            _healthy_context(now_s=10.00, command_stamp_s=9.98, state_stamp_s=9.98),
        )
        retained = self.supervisor.filter_command(
            _command(speed=0.06, lateral_error_m=0.08),
            _healthy_context(now_s=10.05, command_stamp_s=10.03, state_stamp_s=10.03),
        )
        cleared = self.supervisor.filter_command(
            _command(speed=0.06, lateral_error_m=0.04),
            _healthy_context(now_s=10.10, command_stamp_s=10.08, state_stamp_s=10.08),
        )

        self.assertEqual(entered.status, "SAFE_RECOVERY")
        self.assertEqual(retained.status, "SAFE_RECOVERY")
        self.assertEqual(cleared.status, "SAFE_ACTIVE")

    def test_large_tracking_error_must_persist_before_hard_hold(self) -> None:
        first = self.supervisor.filter_command(
            _command(lateral_error_m=0.45),
            _healthy_context(now_s=10.00, command_stamp_s=9.98, state_stamp_s=9.98),
        )
        brief = self.supervisor.filter_command(
            _command(lateral_error_m=0.45),
            _healthy_context(now_s=10.50, command_stamp_s=10.48, state_stamp_s=10.48),
        )
        persistent = self.supervisor.filter_command(
            _command(lateral_error_m=0.45),
            _healthy_context(now_s=11.01, command_stamp_s=10.99, state_stamp_s=10.99),
        )

        self.assertEqual(first.status, "SAFE_RECOVERY")
        self.assertEqual(brief.status, "SAFE_RECOVERY")
        self.assertEqual(persistent.status, "SAFE_HOLD")
        self.assertEqual(
            (persistent.linear_x_mps, persistent.yaw_rate_radps),
            (0.0, 0.0),
        )
        self.assertIn("lateral_error_persistent", persistent.reasons)

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

    def test_parallel_trim_is_bounded_and_keeps_hard_obstacle_stop(self) -> None:
        parallel = _command(speed=0.06, yaw_rate=0.0, lateral_speed=0.08)
        output = self.supervisor.filter_command(
            parallel,
            _healthy_context(
                measured_speed_mps=0.0,
                ackermann_mode=False,
                motion_mode="parallel",
                precision_motion_allowed=True,
            ),
        )
        stopped = self.supervisor.filter_command(
            parallel,
            _healthy_context(
                now_s=10.05,
                command_stamp_s=10.03,
                state_stamp_s=10.03,
                measured_speed_mps=0.0,
                avoidance_stop=True,
                ackermann_mode=False,
                motion_mode="parallel",
                precision_motion_allowed=True,
            ),
        )

        self.assertGreater(output.linear_y_mps, 0.0)
        self.assertLessEqual(math.hypot(output.linear_x_mps, output.linear_y_mps), 0.20)
        self.assertEqual(stopped.status, "SAFE_HOLD")
        self.assertEqual(stopped.linear_y_mps, 0.0)
        self.assertIn("avoidance_stop", stopped.reasons)

    def test_spin_trim_is_allowed_only_for_matching_motion_mode(self) -> None:
        spin = _command(speed=0.0, yaw_rate=0.12)

        matching = self.supervisor.filter_command(
            spin,
            _healthy_context(
                measured_speed_mps=0.0,
                ackermann_mode=False,
                motion_mode="spin",
                precision_motion_allowed=True,
            ),
        )
        moving_mismatch = self.supervisor.filter_command(
            spin,
            _healthy_context(
                now_s=10.05,
                command_stamp_s=10.03,
                state_stamp_s=10.03,
                measured_speed_mps=0.10,
                ackermann_mode=True,
                motion_mode="dual_ackermann",
                precision_motion_allowed=True,
            ),
        )

        self.assertEqual(matching.linear_x_mps, 0.0)
        self.assertGreater(matching.yaw_rate_radps, 0.0)
        self.assertEqual(moving_mismatch.status, "SAFE_HOLD")
        self.assertIn("unexpected_motion_mode", moving_mismatch.reasons)

    def test_motion_mode_transition_must_complete_within_timeout(self) -> None:
        spin = _command(speed=0.0, yaw_rate=0.12)

        entering = self.supervisor.filter_command(
            spin,
            _healthy_context(
                now_s=10.00,
                command_stamp_s=9.98,
                state_stamp_s=9.98,
                measured_speed_mps=0.0,
                precision_motion_allowed=True,
            ),
        )
        timed_out = self.supervisor.filter_command(
            spin,
            _healthy_context(
                now_s=10.60,
                command_stamp_s=10.58,
                state_stamp_s=10.58,
                measured_speed_mps=0.0,
                precision_motion_allowed=True,
            ),
        )

        self.assertIn("motion_mode_transition", entering.reasons)
        self.assertEqual(timed_out.status, "SAFE_HOLD")
        self.assertIn("motion_mode_transition_timeout", timed_out.reasons)

    def test_non_ackermann_command_is_rejected_outside_precision_phase(self) -> None:
        spin = _command(speed=0.0, yaw_rate=0.12)

        output = self.supervisor.filter_command(
            spin,
            _healthy_context(
                measured_speed_mps=0.0,
                ackermann_mode=False,
                motion_mode="spin",
                precision_motion_allowed=False,
            ),
        )

        self.assertEqual(output.status, "SAFE_HOLD")
        self.assertIn("non_ackermann_outside_precision", output.reasons)


if __name__ == "__main__":
    unittest.main()
