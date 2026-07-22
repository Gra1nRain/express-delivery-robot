import math
import pathlib
import sys
import unittest
from dataclasses import replace


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    MPPIController,
    MPPIParams,
    VehicleState,
)


def _straight_trajectory() -> ControlTrajectory:
    points = []
    for index in range(41):
        x = index * 0.10
        points.append(
            ControlTrajectoryPoint(
                x=x,
                y=0.0,
                yaw=0.0,
                s=x,
                curvature=0.0,
                v=0.0 if index in {0, 40} else 0.20,
                t=index * 0.50,
                ref_id="finish" if index == 40 else None,
            )
        )
    return ControlTrajectory(frame_id="map", route_name="straight", points=tuple(points))


def _constant_curvature_trajectory(radius_m: float = 1.0) -> ControlTrajectory:
    points = []
    for index in range(41):
        s = index * 0.10
        yaw = s / radius_m
        points.append(
            ControlTrajectoryPoint(
                x=radius_m * math.sin(yaw),
                y=radius_m * (1.0 - math.cos(yaw)),
                yaw=yaw,
                s=s,
                curvature=1.0 / radius_m,
                v=0.0 if index in {0, 40} else 0.20,
                t=index * 0.50,
                ref_id="finish" if index == 40 else None,
            )
        )
    return ControlTrajectory(frame_id="map", route_name="arc", points=tuple(points))


class MPPIControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.params = MPPIParams(
            control_dt_s=0.05,
            horizon_steps=24,
            rollout_count=512,
            iterations=2,
            temperature=0.35,
            speed_noise_std_mps=0.06,
            curvature_noise_std_1pm=0.30,
            max_speed_mps=0.20,
            max_acceleration_mps2=0.20,
            max_deceleration_mps2=0.30,
            min_turning_radius_m=0.81,
            max_curvature_rate_1pmps=0.80,
        )

    def test_tracks_straight_path_without_requesting_spin_mode(self) -> None:
        controller = MPPIController(_straight_trajectory(), self.params, random_seed=7)

        command = controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.0)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertIs(type(command.lateral_error_m), float)
        self.assertIs(type(command.heading_error_rad), float)
        self.assertGreater(command.linear_x_mps, 0.0)
        self.assertLessEqual(command.linear_x_mps, 0.20)
        self.assertLess(abs(command.yaw_rate_radps), 0.02)
        if abs(command.yaw_rate_radps) > 1e-9:
            self.assertGreaterEqual(
                abs(command.linear_x_mps / command.yaw_rate_radps),
                0.81,
            )

    def test_lateral_error_generates_corrective_curvature(self) -> None:
        controller = MPPIController(_straight_trajectory(), self.params, random_seed=11)

        command = controller.compute_command(
            VehicleState(x=0.5, y=0.12, yaw=0.0, linear_speed_mps=0.10)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertGreater(command.linear_x_mps, 0.0)
        self.assertLess(command.curvature_1pm, 0.0)
        self.assertAlmostEqual(
            command.yaw_rate_radps,
            command.linear_x_mps * command.curvature_1pm,
            places=9,
        )

    def test_curved_reference_preserves_feedforward_without_tracking_error(self) -> None:
        params = replace(
            self.params,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
            max_curvature_rate_1pmps=100.0,
        )
        controller = MPPIController(
            _constant_curvature_trajectory(),
            params,
            random_seed=13,
        )

        command = controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.10)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertGreater(command.curvature_1pm, 0.80)

    def test_rejects_trajectory_outside_runtime_turning_radius(self) -> None:
        with self.assertRaisesRegex(ValueError, "turning-radius envelope"):
            MPPIController(
                _constant_curvature_trajectory(radius_m=1.0),
                replace(self.params, min_turning_radius_m=2.0),
                random_seed=17,
            )

    def test_repeated_geometry_does_not_jump_to_later_route_pass(self) -> None:
        points = []
        for index in range(21):
            x = index * 0.10
            points.append(
                ControlTrajectoryPoint(x, 0.05, 0.0, index * 0.10, 0.0, 0.15, index * 0.5)
            )
        for offset, index in enumerate(range(19, -1, -1), start=21):
            x = index * 0.10
            points.append(
                ControlTrajectoryPoint(
                    x,
                    0.0,
                    math.pi,
                    offset * 0.10,
                    0.0,
                    0.0 if index == 0 else 0.15,
                    offset * 0.5,
                    "finish" if index == 0 else None,
                )
            )
        controller = MPPIController(
            ControlTrajectory("map", "out_and_back", tuple(points)),
            self.params,
            random_seed=5,
        )

        first_pass = controller.compute_command(
            VehicleState(x=1.0, y=0.05, yaw=0.0, linear_speed_mps=0.10)
        )
        command = controller.compute_command(
            VehicleState(x=1.0, y=0.0, yaw=0.0, linear_speed_mps=0.10)
        )

        self.assertLess(first_pass.target_index, 15)
        self.assertEqual(command.status, "TRACKING")
        self.assertLess(command.target_index, 15)


if __name__ == "__main__":
    unittest.main()
