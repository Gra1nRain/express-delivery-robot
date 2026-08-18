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


def _curvature_ramp_trajectory() -> ControlTrajectory:
    return ControlTrajectory(
        frame_id="map",
        route_name="curvature_ramp",
        points=(
            ControlTrajectoryPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0),
            ControlTrajectoryPoint(0.1, 0.0, 0.0, 0.1, 1.0, 0.15, 2.0 / 3.0),
            ControlTrajectoryPoint(0.2, 0.0, 0.0, 0.2, 1.0, 0.15, 4.0 / 3.0),
        ),
    )


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

    def test_field_stop_cannot_finish_before_entering_checkpoint_gate(self) -> None:
        field_stop = VehicleState(
            x=8.315,
            y=0.015,
            yaw=0.003,
            linear_speed_mps=0.0,
        )
        exact_checkpoint_trajectory = ControlTrajectory(
            frame_id="map",
            route_name="pickup_front_gate",
            points=(
                ControlTrajectoryPoint(8.265, -0.053, 0.014, 0.0, 0.0, 0.06, 0.0),
                ControlTrajectoryPoint(
                    8.413,
                    -0.081,
                    0.015,
                    0.1506,
                    0.0,
                    0.0,
                    2.51,
                    "pickup_front",
                ),
            ),
        )
        controller = MPPIController(
            exact_checkpoint_trajectory,
            replace(
                self.params,
                goal_position_tolerance_m=0.03,
                goal_heading_tolerance_rad=math.radians(2.0),
            ),
            random_seed=43,
        )

        command = controller.compute_command(field_stop)

        self.assertEqual(command.status, "TRACKING")
        self.assertGreater(
            math.hypot(field_stop.x - 8.413, field_stop.y + 0.081),
            0.10,
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

    def test_sparse_reference_curvature_is_interpolated_at_control_time(self) -> None:
        params = replace(
            self.params,
            horizon_steps=8,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
            max_curvature_rate_1pmps=100.0,
            feedback_blend=1.0,
        )
        controller = MPPIController(
            _curvature_ramp_trajectory(),
            params,
            random_seed=29,
        )

        command = controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.15)
        )

        # At 50 ms the reference lies only 7.5% of the way to the next
        # 0.10 m point. Selecting the whole next point would command 1.0 1/m.
        self.assertAlmostEqual(command.curvature_1pm, 0.075, places=3)

    def test_large_tracking_error_still_returns_a_corrective_command(self) -> None:
        controller = MPPIController(_straight_trajectory(), self.params, random_seed=31)

        command = controller.compute_command(
            VehicleState(x=0.5, y=0.35, yaw=0.0, linear_speed_mps=0.06)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertGreater(command.linear_x_mps, 0.0)
        self.assertLess(command.curvature_1pm, 0.0)

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

    def test_replaces_global_reference_with_latest_local_trajectory(self) -> None:
        params = replace(
            self.params,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
            max_curvature_rate_1pmps=100.0,
        )
        controller = MPPIController(_straight_trajectory(), params, random_seed=23)
        controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.10)
        )

        controller.replace_trajectory(_constant_curvature_trajectory())
        command = controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.10)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertLess(command.target_index, 4)
        self.assertGreater(command.curvature_1pm, 0.80)

    def test_replanning_keeps_command_speed_ramping_through_odom_deadband(self) -> None:
        params = replace(
            self.params,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
        )
        controller = MPPIController(
            _straight_trajectory(),
            params,
            random_seed=37,
        )
        local_trajectory = replace(
            _straight_trajectory(),
            points=tuple(
                replace(point, v=0.20)
                for point in _straight_trajectory().points
            ),
        )
        state = VehicleState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_speed_mps=0.0,
        )

        speeds = []
        for _ in range(4):
            speeds.append(controller.compute_command(state).linear_x_mps)
            controller.replace_trajectory(local_trajectory)

        self.assertGreater(speeds[-1], speeds[0] + 0.02)
        self.assertEqual(speeds, sorted(speeds))

    def test_tracking_speed_changes_respect_jerk_limit_through_odom_deadband(self) -> None:
        params = replace(
            self.params,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
            max_speed_mps=1.0,
            max_acceleration_mps2=0.50,
            max_deceleration_mps2=0.80,
            command_speed_memory_limit_mps=1.0,
        )
        controller = MPPIController(
            _straight_trajectory(),
            params,
            random_seed=47,
        )
        state = VehicleState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_speed_mps=0.0,
        )

        speeds = [controller.compute_command(state).linear_x_mps for _ in range(60)]
        slow_trajectory = replace(
            _straight_trajectory(),
            points=tuple(
                replace(point, v=0.05)
                for point in _straight_trajectory().points
            ),
        )
        controller.replace_trajectory(slow_trajectory)
        speeds.extend(
            controller.compute_command(state).linear_x_mps for _ in range(60)
        )
        accelerations = [
            (current - previous) / params.control_dt_s
            for previous, current in zip([0.0, *speeds[:-1]], speeds)
        ]
        jerks = [
            (current - previous) / params.control_dt_s
            for previous, current in zip([0.0, *accelerations[:-1]], accelerations)
        ]

        self.assertLessEqual(max(abs(jerk) for jerk in jerks), 2.0 + 1e-9)
        self.assertTrue(all(speed > 0.0 for speed in speeds))
        self.assertLess(speeds[-1], max(speeds))

    def test_normal_curve_keeps_forward_speed(self) -> None:
        params = replace(
            self.params,
            speed_noise_std_mps=0.0,
            curvature_noise_std_1pm=0.0,
            command_speed_memory_limit_mps=self.params.max_speed_mps,
        )
        controller = MPPIController(
            _constant_curvature_trajectory(),
            params,
            random_seed=53,
        )

        command = controller.compute_command(
            VehicleState(x=0.0, y=0.0, yaw=0.0, linear_speed_mps=0.10)
        )

        self.assertEqual(command.status, "TRACKING")
        self.assertGreater(command.linear_x_mps, 0.05)
        self.assertGreater(abs(command.yaw_rate_radps), 0.0)

    def test_goal_requires_heading_tolerance_when_configured(self) -> None:
        params = replace(
            self.params,
            goal_position_tolerance_m=0.10,
            goal_heading_tolerance_rad=math.radians(5.0),
            progress_search_window_points=100,
            max_progress_advance_points=100,
        )
        controller = MPPIController(_straight_trajectory(), params, random_seed=41)

        misaligned = controller.compute_command(
            VehicleState(
                x=4.0,
                y=0.0,
                yaw=math.radians(8.0),
                linear_speed_mps=0.0,
            )
        )
        aligned = controller.compute_command(
            VehicleState(
                x=4.0,
                y=0.0,
                yaw=math.radians(2.0),
                linear_speed_mps=0.0,
            )
        )

        self.assertEqual(misaligned.status, "TRACKING")
        self.assertEqual(aligned.status, "GOAL_REACHED")


if __name__ == "__main__":
    unittest.main()
