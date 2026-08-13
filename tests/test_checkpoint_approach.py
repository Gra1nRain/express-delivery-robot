import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mppi_controller import (
    BodyCommand,
    shape_checkpoint_approach_command,
)


class CheckpointApproachCommandTest(unittest.TestCase):
    def command(self, speed_mps: float) -> BodyCommand:
        return BodyCommand(
            linear_x_mps=speed_mps,
            yaw_rate_radps=0.5 * speed_mps,
            curvature_1pm=0.5,
            target_index=1,
            lateral_error_m=0.0,
            heading_error_rad=math.radians(3.0),
            status="TRACKING",
        )

    def test_near_checkpoint_replaces_micro_command_with_stable_speed(self) -> None:
        shaped = shape_checkpoint_approach_command(
            self.command(0.003),
            longitudinal_error_m=-0.50,
            checkpoint_heading_error_rad=math.radians(3.0),
            checkpoint_heading_tolerance_rad=math.radians(4.0),
            capture_distance_m=0.10,
            slowdown_distance_m=1.00,
            min_speed_mps=0.05,
            max_speed_mps=0.08,
        )

        self.assertGreaterEqual(shaped.linear_x_mps, 0.05)
        self.assertLessEqual(shaped.linear_x_mps, 0.08)
        self.assertAlmostEqual(
            shaped.yaw_rate_radps,
            shaped.linear_x_mps * shaped.curvature_1pm,
        )

    def test_near_checkpoint_caps_an_excessive_command(self) -> None:
        shaped = shape_checkpoint_approach_command(
            self.command(0.15),
            longitudinal_error_m=-0.10,
            checkpoint_heading_error_rad=math.radians(3.0),
            checkpoint_heading_tolerance_rad=math.radians(4.0),
            capture_distance_m=0.10,
            slowdown_distance_m=1.00,
            min_speed_mps=0.05,
            max_speed_mps=0.08,
        )

        self.assertAlmostEqual(shaped.linear_x_mps, 0.05)

    def test_heading_outside_final_tolerance_uses_minimum_speed(self) -> None:
        shaped = shape_checkpoint_approach_command(
            self.command(0.15),
            longitudinal_error_m=-0.80,
            checkpoint_heading_error_rad=math.radians(4.6),
            checkpoint_heading_tolerance_rad=math.radians(4.0),
            capture_distance_m=0.10,
            slowdown_distance_m=1.00,
            min_speed_mps=0.05,
            max_speed_mps=0.08,
        )

        self.assertAlmostEqual(shaped.linear_x_mps, 0.05)

    def test_crossing_checkpoint_plane_commands_zero(self) -> None:
        shaped = shape_checkpoint_approach_command(
            self.command(0.05),
            longitudinal_error_m=0.001,
            checkpoint_heading_error_rad=math.radians(4.6),
            checkpoint_heading_tolerance_rad=math.radians(4.0),
            capture_distance_m=0.10,
            slowdown_distance_m=1.00,
            min_speed_mps=0.05,
            max_speed_mps=0.08,
        )

        self.assertEqual(shaped.linear_x_mps, 0.0)
        self.assertEqual(shaped.yaw_rate_radps, 0.0)
        self.assertEqual(shaped.status, "CHECKPOINT_PLANE_HOLD")

    def test_far_from_checkpoint_preserves_controller_command(self) -> None:
        command = self.command(0.12)

        shaped = shape_checkpoint_approach_command(
            command,
            longitudinal_error_m=-1.20,
            checkpoint_heading_error_rad=math.radians(3.0),
            checkpoint_heading_tolerance_rad=math.radians(4.0),
            capture_distance_m=0.10,
            slowdown_distance_m=1.00,
            min_speed_mps=0.05,
            max_speed_mps=0.08,
        )

        self.assertIs(shaped, command)


if __name__ == "__main__":
    unittest.main()
