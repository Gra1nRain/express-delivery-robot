import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mppi_controller import control_trajectories_from_dict
from competition_control.segmented_route_state_machine import (
    SegmentedRouteConfig,
    SegmentedRouteObservation,
    SegmentedRoutePhase,
    SegmentedRouteStateMachine,
)


class SegmentedRouteStateMachineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = SegmentedRouteStateMachine(
            segment_count=2,
            config=SegmentedRouteConfig(
                goal_position_tolerance_m=0.10,
                goal_heading_tolerance_rad=math.radians(5.0),
                stop_speed_tolerance_mps=0.03,
                dock_hold_s=1.0,
            ),
        )

    def observation(
        self,
        now_s: float,
        *,
        enabled: bool = True,
        state_valid: bool = True,
        stop_requested: bool = False,
        position_error_m: float = 1.0,
        heading_error_rad: float = 0.0,
        speed_mps: float = 0.0,
    ) -> SegmentedRouteObservation:
        return SegmentedRouteObservation(
            now_s=now_s,
            enabled=enabled,
            state_valid=state_valid,
            stop_requested=stop_requested,
            position_error_m=position_error_m,
            heading_error_rad=heading_error_rad,
            speed_mps=speed_mps,
        )

    def test_requires_explicit_enable_before_tracking(self) -> None:
        decision = self.machine.update(self.observation(0.0, enabled=False))

        self.assertEqual(decision.phase, SegmentedRoutePhase.DISARMED)
        self.assertFalse(decision.allow_tracking)
        self.assertEqual(decision.active_segment_index, 0)

        decision = self.machine.update(self.observation(0.1))

        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)
        self.assertTrue(decision.allow_tracking)

    def test_advances_only_after_pose_heading_speed_and_hold_are_satisfied(self) -> None:
        self.machine.update(self.observation(0.0))

        decision = self.machine.update(
            self.observation(
                1.0,
                position_error_m=0.05,
                heading_error_rad=math.radians(2.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(decision.phase, SegmentedRoutePhase.DOCK_HOLD)
        self.assertFalse(decision.allow_tracking)

        decision = self.machine.update(
            self.observation(
                1.5,
                position_error_m=0.05,
                heading_error_rad=math.radians(2.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(decision.active_segment_index, 0)
        self.assertFalse(decision.segment_changed)

        decision = self.machine.update(
            self.observation(
                2.0,
                position_error_m=0.05,
                heading_error_rad=math.radians(2.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)
        self.assertEqual(decision.active_segment_index, 1)
        self.assertTrue(decision.segment_changed)
        self.assertFalse(decision.allow_tracking)

    def test_dock_hold_returns_to_tracking_if_pose_drifts(self) -> None:
        self.machine.update(self.observation(0.0))
        self.machine.update(
            self.observation(
                1.0,
                position_error_m=0.05,
                heading_error_rad=0.0,
                speed_mps=0.0,
            )
        )

        decision = self.machine.update(
            self.observation(
                1.5,
                position_error_m=0.14,
                heading_error_rad=0.0,
                speed_mps=0.0,
            )
        )

        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)
        self.assertTrue(decision.allow_tracking)
        self.assertEqual(decision.active_segment_index, 0)

    def test_safety_stop_auto_resumes_but_invalid_state_requires_rearm(self) -> None:
        self.machine.update(self.observation(0.0))

        decision = self.machine.update(self.observation(0.1, stop_requested=True))
        self.assertEqual(decision.phase, SegmentedRoutePhase.SAFETY_HOLD)
        self.assertFalse(decision.allow_tracking)

        decision = self.machine.update(self.observation(0.2))
        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)
        self.assertTrue(decision.allow_tracking)

        decision = self.machine.update(self.observation(0.3, state_valid=False))
        self.assertEqual(decision.phase, SegmentedRoutePhase.FAULT_HOLD)

        decision = self.machine.update(self.observation(0.4, state_valid=True))
        self.assertEqual(decision.phase, SegmentedRoutePhase.FAULT_HOLD)
        self.assertFalse(decision.allow_tracking)

        decision = self.machine.update(self.observation(0.5, enabled=False))
        self.assertEqual(decision.phase, SegmentedRoutePhase.DISARMED)

        decision = self.machine.update(self.observation(0.6, enabled=True))
        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)

    def test_final_segment_finishes_and_reset_returns_to_disarmed_start(self) -> None:
        self.machine.update(self.observation(0.0))
        self.machine.update(
            self.observation(1.0, position_error_m=0.0, speed_mps=0.0)
        )
        self.machine.update(
            self.observation(2.0, position_error_m=0.0, speed_mps=0.0)
        )
        self.machine.update(
            self.observation(2.1, position_error_m=0.0, speed_mps=0.0)
        )
        decision = self.machine.update(
            self.observation(3.1, position_error_m=0.0, speed_mps=0.0)
        )

        self.assertEqual(decision.phase, SegmentedRoutePhase.COMPLETED)
        self.assertEqual(decision.active_segment_index, 1)
        self.assertFalse(decision.allow_tracking)

        self.machine.reset()
        decision = self.machine.update(self.observation(4.0, enabled=False))
        self.assertEqual(decision.phase, SegmentedRoutePhase.DISARMED)
        self.assertEqual(decision.active_segment_index, 0)


class SegmentedTrajectoryArtifactTest(unittest.TestCase):
    def test_loads_current_ten_segment_artifact(self) -> None:
        import yaml

        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day4"
                / "debug_optimized_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )

        trajectories = control_trajectories_from_dict(artifact)

        self.assertEqual(len(trajectories), 10)
        self.assertEqual(trajectories[0].route_name, "go_traffic_light_1")
        self.assertEqual(trajectories[-1].route_name, "finish_park")
        for trajectory in trajectories:
            self.assertEqual(trajectory.points[-1].v, 0.0)

    def test_keeps_continuous_artifact_compatibility(self) -> None:
        artifact = {
            "frame_id": "map",
            "route_name": "one_segment",
            "points": [
                {
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "s": 0.0,
                    "curvature": 0.0,
                    "v": 0.1,
                    "t": 0.0,
                },
                {
                    "x": 0.1,
                    "y": 0.0,
                    "yaw": 0.0,
                    "s": 0.1,
                    "curvature": 0.0,
                    "v": 0.0,
                    "t": 1.0,
                },
            ],
        }

        trajectories = control_trajectories_from_dict(artifact)

        self.assertEqual(len(trajectories), 1)
        self.assertEqual(trajectories[0].route_name, "one_segment")

    def test_indoor_one_lap_finishes_after_first_drop_rear(self) -> None:
        import yaml

        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day4"
                / "debug_indoor_one_lap_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )

        trajectories = control_trajectories_from_dict(artifact)

        self.assertEqual(
            [trajectory.route_name for trajectory in trajectories],
            [
                "go_traffic_light_1",
                "random_obstacle_1",
                "pickup_1_rear",
                "cone_lane_change_1",
                "drop_1_rear",
                "finish_park",
            ],
        )
        self.assertEqual(trajectories[-2].points[-1].ref_id, "drop_rear")
        self.assertEqual(trajectories[-1].points[-1].ref_id, "finish_park")


class SegmentedRouteLaunchTopologyTest(unittest.TestCase):
    def test_dedicated_launch_keeps_all_motion_gates_closed(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_segmented_route_test.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("debug_indoor_one_lap_trajectory.yaml", launch_text)
        self.assertIn("debug_indoor_one_lap_route.yaml", launch_text)
        self.assertIn('"start_local_replanner": "false"', launch_text)
        self.assertIn('"replanning_enabled": "false"', launch_text)
        self.assertIn('"start_chassis_adapter": "false"', launch_text)
        self.assertIn('"command_output_topic": "/cmd_vel_safe"', launch_text)
        self.assertNotIn("restart.sh", launch_text)


if __name__ == "__main__":
    unittest.main()
