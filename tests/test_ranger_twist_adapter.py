import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.ranger_twist_adapter import (  # noqa: E402
    RangerMiniV3Geometry,
    adapt_yaw_rate_for_ranger_driver,
)


GEOMETRY = RangerMiniV3Geometry()


def _ranger_driver_realized_radius(command_radius_m: float) -> float:
    inner_angle = math.atan((GEOMETRY.wheelbase_m / 2.0) / command_radius_m)
    central_angle = math.atan(
        GEOMETRY.wheelbase_m
        * math.sin(inner_angle)
        / (
            GEOMETRY.wheelbase_m * math.cos(inner_angle)
            + GEOMETRY.track_width_m * math.sin(inner_angle)
        )
    )
    curvature = 2.0 * math.sin(central_angle) / GEOMETRY.wheelbase_m
    return 1.0 / curvature


class RangerTwistAdapterTest(unittest.TestCase):
    def test_unadapted_ranger_command_turns_too_wide(self) -> None:
        desired_radius_m = 0.81

        realized_radius = _ranger_driver_realized_radius(desired_radius_m)

        self.assertGreater(realized_radius, desired_radius_m + 0.20)
        self.assertAlmostEqual(realized_radius, 1.022, places=3)

    def test_adapter_maps_desired_081m_body_radius_to_ranger_command(self) -> None:
        linear_x = 0.08
        desired_yaw_rate = linear_x / 0.81

        adapted_yaw_rate = adapt_yaw_rate_for_ranger_driver(
            linear_x_mps=linear_x,
            desired_yaw_rate_radps=desired_yaw_rate,
            geometry=GEOMETRY,
        )
        adapted_command_radius = abs(linear_x / adapted_yaw_rate)
        realized_radius = _ranger_driver_realized_radius(adapted_command_radius)

        self.assertAlmostEqual(adapted_command_radius, 0.589, places=3)
        self.assertAlmostEqual(realized_radius, 0.81, places=6)

    def test_adapter_preserves_explicit_spin_commands(self) -> None:
        adapted_yaw_rate = adapt_yaw_rate_for_ranger_driver(
            linear_x_mps=0.0,
            desired_yaw_rate_radps=0.25,
            geometry=GEOMETRY,
        )

        self.assertEqual(adapted_yaw_rate, 0.25)

    def test_adapter_avoids_driver_spin_threshold_for_tight_arcs(self) -> None:
        linear_x = 0.08
        desired_yaw_rate = linear_x / 0.60

        adapted_yaw_rate = adapt_yaw_rate_for_ranger_driver(
            linear_x_mps=linear_x,
            desired_yaw_rate_radps=desired_yaw_rate,
            geometry=GEOMETRY,
        )
        adapted_command_radius = abs(linear_x / adapted_yaw_rate)

        self.assertGreaterEqual(
            adapted_command_radius,
            GEOMETRY.driver_min_turn_radius_m - 1e-12,
        )


if __name__ == "__main__":
    unittest.main()
