import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_safety"))

from competition_safety.proximity_stop import (
    ProximityStopConfig,
    should_stop_for_points,
)


class ProximityStopTest(unittest.TestCase):
    def test_points_inside_stop_box_trigger_stop(self) -> None:
        config = ProximityStopConfig(
            x_min_m=0.25,
            stop_distance_m=0.55,
            front_half_angle_rad=0.4363,
            z_min_m=-0.25,
            z_max_m=0.80,
            min_points=3,
        )

        stop, count = should_stop_for_points(
            [
                (0.40, 0.00, 0.10),
                (0.45, 0.10, 0.20),
                (0.50, -0.15, 0.30),
                (0.70, 0.00, 0.10),
            ],
            config,
        )

        self.assertTrue(stop)
        self.assertEqual(count, 3)

    def test_side_front_points_inside_vehicle_corridor_trigger_stop(self) -> None:
        config = ProximityStopConfig(
            x_min_m=0.25,
            stop_distance_m=0.55,
            front_half_angle_rad=0.4363,
            z_min_m=-0.25,
            z_max_m=0.80,
            min_points=2,
        )

        stop, count = should_stop_for_points(
            [
                (0.40, 0.38, 0.10),
                (0.45, -0.42, 0.20),
                (0.40, 0.60, 0.10),
            ],
            config,
        )

        self.assertTrue(stop)
        self.assertEqual(count, 2)

    def test_points_outside_stop_box_do_not_trigger_stop(self) -> None:
        config = ProximityStopConfig(min_points=2)

        stop, count = should_stop_for_points(
            [
                (0.10, 0.00, 0.10),
                (0.40, 0.60, 0.10),
                (0.40, 0.00, 1.20),
                (0.70, 0.00, 0.10),
            ],
            config,
        )

        self.assertFalse(stop)
        self.assertEqual(count, 0)


if __name__ == "__main__":
    unittest.main()
