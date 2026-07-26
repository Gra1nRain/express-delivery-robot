import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_safety.proximity_stop import (
    ProximityStopConfig,
    should_stop_for_points,
)


class AvoidanceCollisionRegressionTest(unittest.TestCase):
    def test_contact_returns_trigger_vehicle_proximity_stop(self) -> None:
        """Reproduce the 2026-07-27 bucket-contact false-clear decision."""

        params = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "avoidance"
                / "avoidance_params.yaml"
            ).read_text(encoding="utf-8")
        )["avoidance_manager"]["ros__parameters"]
        config = ProximityStopConfig(
            x_min_m=float(params["proximity_x_min_m"]),
            stop_distance_m=float(params["proximity_stop_distance_m"]),
            front_half_angle_rad=float(
                params["proximity_front_half_angle_rad"]
            ),
            lateral_half_width_m=float(
                params["proximity_lateral_half_width_m"]
            ),
            z_min_m=float(params["proximity_z_min_m"]),
            z_max_m=float(params["proximity_z_max_m"]),
            min_points=int(params["proximity_min_points"]),
        )

        stop, count = should_stop_for_points(
            (
                (1.056, -0.039, -0.071),
                (1.060, -0.065, -0.072),
            ),
            config,
        )

        self.assertTrue(
            stop,
            "Physical-contact Livox returns must never be classified as clear.",
        )
        self.assertGreaterEqual(count, 2)


if __name__ == "__main__":
    unittest.main()
