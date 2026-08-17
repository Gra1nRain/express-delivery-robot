import math
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mppi_controller import control_trajectories_from_dict
from competition_control.mission_checkpoints import mission_checkpoints_from_route
from competition_control.segmented_route_state_machine import (
    SegmentedRouteConfig,
    SegmentedRouteObservation,
    SegmentedRoutePhase,
    SegmentedRouteStateMachine,
    state_failure_requires_rearm,
)


class SegmentedRouteStateMachineTest(unittest.TestCase):
    def test_default_final_heading_tolerance_is_four_degrees(self) -> None:
        self.assertAlmostEqual(
            SegmentedRouteConfig().goal_heading_tolerance_rad,
            math.radians(4.0),
        )
        self.assertAlmostEqual(
            SegmentedRouteConfig().goal_overshoot_tolerance_m,
            0.02,
        )

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
        longitudinal_error_m: float = -1.0,
        release_segment_index: int | None = None,
    ) -> SegmentedRouteObservation:
        return SegmentedRouteObservation(
            now_s=now_s,
            enabled=enabled,
            state_valid=state_valid,
            stop_requested=stop_requested,
            position_error_m=position_error_m,
            heading_error_rad=heading_error_rad,
            speed_mps=speed_mps,
            longitudinal_error_m=longitudinal_error_m,
            release_segment_index=release_segment_index,
        )

    def test_overshot_checkpoint_holds_instead_of_micro_crawling(self) -> None:
        machine = SegmentedRouteStateMachine(
            segment_count=2,
            config=SegmentedRouteConfig(
                goal_position_tolerance_m=0.10,
                goal_heading_tolerance_rad=math.radians(4.0),
                goal_overshoot_tolerance_m=0.02,
            ),
        )
        machine.update(self.observation(0.0))

        in_window = machine.update(
            self.observation(
                1.0,
                position_error_m=0.09,
                heading_error_rad=math.radians(4.6),
                longitudinal_error_m=-0.02,
            )
        )
        self.assertEqual(in_window.phase, SegmentedRoutePhase.TRACKING)
        self.assertTrue(in_window.allow_tracking)

        overshot = machine.update(
            self.observation(
                2.0,
                position_error_m=0.263,
                heading_error_rad=math.radians(3.39),
                longitudinal_error_m=0.263,
            )
        )
        self.assertEqual(overshot.phase, SegmentedRoutePhase.OVERSHOOT_HOLD)
        self.assertFalse(overshot.allow_tracking)

        still_held = machine.update(
            self.observation(
                3.0,
                position_error_m=0.263,
                heading_error_rad=math.radians(3.39),
                longitudinal_error_m=0.263,
            )
        )
        self.assertEqual(still_held.phase, SegmentedRoutePhase.OVERSHOOT_HOLD)
        self.assertFalse(still_held.allow_tracking)

    def test_requires_explicit_enable_before_tracking(self) -> None:
        decision = self.machine.update(self.observation(0.0, enabled=False))

        self.assertEqual(decision.phase, SegmentedRoutePhase.DISARMED)
        self.assertFalse(decision.allow_tracking)
        self.assertEqual(decision.active_segment_index, 0)

        decision = self.machine.update(self.observation(0.1))

        self.assertEqual(decision.phase, SegmentedRoutePhase.TRACKING)
        self.assertTrue(decision.allow_tracking)

    def test_only_transient_freshness_failures_can_auto_resume(self) -> None:
        self.assertFalse(state_failure_requires_rearm(("stale_velocity",)))
        self.assertFalse(
            state_failure_requires_rearm(("stale_pose", "stale_velocity"))
        )
        self.assertTrue(state_failure_requires_rearm(("position_jump",)))
        self.assertTrue(
            state_failure_requires_rearm(("stale_velocity", "position_jump"))
        )
        self.assertTrue(state_failure_requires_rearm(("future_velocity_stamp",)))
        self.assertTrue(state_failure_requires_rearm(("unknown_failure",)))
        self.assertTrue(state_failure_requires_rearm(()))

    def test_waits_for_explicit_release_after_stable_stop(self) -> None:
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
        self.assertEqual(decision.phase, SegmentedRoutePhase.WAIT_RELEASE)
        self.assertEqual(decision.active_segment_index, 0)
        self.assertFalse(decision.segment_changed)
        self.assertFalse(decision.allow_tracking)

        still_waiting = self.machine.update(
            self.observation(
                20.0,
                position_error_m=0.05,
                heading_error_rad=math.radians(2.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(still_waiting.phase, SegmentedRoutePhase.WAIT_RELEASE)
        self.assertFalse(still_waiting.segment_changed)

        released = self.machine.update(
            self.observation(
                20.1,
                position_error_m=0.05,
                heading_error_rad=math.radians(2.0),
                speed_mps=0.02,
                release_segment_index=1,
            )
        )
        self.assertEqual(released.phase, SegmentedRoutePhase.TRACKING)
        self.assertEqual(released.active_segment_index, 1)
        self.assertTrue(released.segment_changed)
        self.assertFalse(released.allow_tracking)

    def test_captures_dock_pose_before_vehicle_has_fully_stopped(self) -> None:
        self.machine.update(self.observation(0.0))

        decision = self.machine.update(
            self.observation(
                1.0,
                position_error_m=0.08,
                heading_error_rad=math.radians(4.0),
                speed_mps=0.05,
            )
        )

        self.assertEqual(decision.phase, SegmentedRoutePhase.DOCK_HOLD)
        self.assertFalse(decision.allow_tracking)

        decision = self.machine.update(
            self.observation(
                1.5,
                position_error_m=0.07,
                heading_error_rad=math.radians(4.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(decision.active_segment_index, 0)
        self.assertFalse(decision.segment_changed)

        decision = self.machine.update(
            self.observation(
                2.5,
                position_error_m=0.07,
                heading_error_rad=math.radians(4.0),
                speed_mps=0.02,
            )
        )
        self.assertEqual(decision.phase, SegmentedRoutePhase.WAIT_RELEASE)
        self.assertEqual(decision.active_segment_index, 0)
        self.assertFalse(decision.segment_changed)

        decision = self.machine.update(
            self.observation(
                2.6,
                position_error_m=0.07,
                heading_error_rad=math.radians(4.0),
                speed_mps=0.02,
                release_segment_index=1,
            )
        )
        self.assertEqual(decision.active_segment_index, 1)
        self.assertTrue(decision.segment_changed)

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
            self.observation(
                2.1,
                position_error_m=0.0,
                speed_mps=0.0,
                release_segment_index=1,
            )
        )
        self.machine.update(
            self.observation(2.2, position_error_m=0.0, speed_mps=0.0)
        )
        decision = self.machine.update(
            self.observation(3.2, position_error_m=0.0, speed_mps=0.0)
        )

        self.assertEqual(decision.phase, SegmentedRoutePhase.COMPLETED)
        self.assertEqual(decision.active_segment_index, 1)
        self.assertFalse(decision.allow_tracking)

        self.machine.reset()
        decision = self.machine.update(self.observation(4.0, enabled=False))
        self.assertEqual(decision.phase, SegmentedRoutePhase.DISARMED)
        self.assertEqual(decision.active_segment_index, 0)

    def test_release_can_skip_intermediate_checkpoint(self) -> None:
        machine = SegmentedRouteStateMachine(
            segment_count=4,
            config=SegmentedRouteConfig(dock_hold_s=0.5),
        )
        machine.update(self.observation(0.0))
        machine.update(
            self.observation(1.0, position_error_m=0.0, speed_mps=0.0)
        )
        ready = machine.update(
            self.observation(1.5, position_error_m=0.0, speed_mps=0.0)
        )
        self.assertEqual(ready.phase, SegmentedRoutePhase.WAIT_RELEASE)

        released = machine.update(
            self.observation(
                1.6,
                position_error_m=0.0,
                speed_mps=0.0,
                release_segment_index=3,
            )
        )

        self.assertEqual(released.phase, SegmentedRoutePhase.TRACKING)
        self.assertEqual(released.active_segment_index, 3)
        self.assertTrue(released.segment_changed)


class SegmentedTrajectoryArtifactTest(unittest.TestCase):
    def test_semantic_dock_heading_tolerances_match_final_gate(self) -> None:
        import yaml

        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )

        self.assertTrue(semantic_map["dock_poses"])
        self.assertEqual(
            {float(dock["yaw_tolerance_deg"]) for dock in semantic_map["dock_poses"]},
            {4.0},
        )

    def test_indoor_route_stops_only_at_task_checkpoints(self) -> None:
        import yaml

        route = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "routes"
                / "debug_indoor_one_lap_route.yaml"
            ).read_text(encoding="utf-8")
        )
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )
        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_indoor_one_lap_continuous_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        trajectories = control_trajectories_from_dict(artifact)

        self.assertEqual(len(trajectories), 1)
        checkpoints = mission_checkpoints_from_route(
            route,
            semantic_map,
            trajectories[0],
        )
        self.assertEqual(
            [checkpoint.ref_id for checkpoint in checkpoints],
            [
                "pickup_front",
                "pickup_rear",
                "drop_front",
                "drop_rear",
                "finish_park",
            ],
        )
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
                "pickup_transit_1",
                "pickup_1_rear",
                "pickup_departure_wide",
                "drop_transit_1",
                "drop_1_rear",
                "finish_park",
            ],
        )
        self.assertEqual(trajectories[-2].points[-1].ref_id, "drop_rear")
        self.assertEqual(trajectories[-1].points[-1].ref_id, "finish_park")


class SegmentedRouteLaunchTopologyTest(unittest.TestCase):
    def test_one_lap_launch_reuses_full_local_avoidance_stack(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_segmented_route_test.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn("debug_indoor_one_lap_continuous_trajectory.yaml", launch_text)
        self.assertIn("debug_indoor_one_lap_route.yaml", launch_text)
        self.assertIn('"start_local_replanner": "true"', launch_text)
        self.assertIn('"replanning_enabled": "true"', launch_text)
        self.assertIn('"start_chassis_adapter": "false"', launch_text)
        self.assertIn('"command_output_topic": "/cmd_vel_safe"', launch_text)
        self.assertNotIn("restart.sh", launch_text)

    def test_checkpoint_state_does_not_switch_the_global_reference(self) -> None:
        control_node = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")
        replanner_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn(
            "segmented route test requires replanning_enabled=false",
            control_node,
        )
        self.assertNotIn("_active_segment_callback", replanner_node)
        self.assertNotIn('"active_segment_topic"', replanner_node)
        self.assertIn("_initialpose_callback", replanner_node)
        self.assertIn("_active_checkpoint_publisher", control_node)
        self.assertIn("_active_checkpoint_ref_publisher", control_node)
        self.assertIn("_active_checkpoint_ref_callback", replanner_node)
        self.assertIn("straight_followup_anchors_from_config", control_node)
        self.assertIn("calibrated_straight_reference", control_node)
        self.assertIn('self._precision_phase = "CALIBRATED_STRAIGHT"', control_node)
        self.assertIn("reference_prefix_to_checkpoint(", replanner_node)
        self.assertIn("controller_goal_position_tolerance_m", control_node)
        self.assertIn("controller_goal_heading_tolerance_deg", control_node)
        self.assertIn("concatenate_reference_paths", replanner_node)
        self.assertNotIn("_activate_segment", control_node)
        self.assertIn("mission_checkpoints_from_route", control_node)
        self.assertIn("nearest_path_point_index", control_node)
        self.assertIn("nearest_stop_line_path_point_index", control_node)
        self.assertIn("stop_line_lengths_excluding_docks", control_node)
        self.assertIn("checkpoint_errors(", control_node)
        self.assertIn("self._local_stop_requested", control_node)

    def test_one_lap_route_marks_obstacle_segment_for_avoidance(self) -> None:
        import yaml

        route = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "routes"
                / "debug_indoor_one_lap_route.yaml"
            ).read_text(encoding="utf-8")
        )
        steps = {step["id"]: step for step in route["steps"]}

        self.assertTrue(steps["pickup_transit_1"]["avoidance_required"])
        self.assertTrue(steps["pickup_transit_1"]["soft_intermediate_refs"])
        self.assertTrue(steps["drop_transit_1"]["soft_intermediate_refs"])

        precision_refs = set(route["precision_docking"]["checkpoints"])
        self.assertEqual(
            precision_refs,
            {"pickup_front", "pickup_rear", "drop_front", "drop_rear"},
        )

    def test_precision_docking_is_loaded_from_a_site_specific_config_file(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"dock_params_file"', launch_text)
        self.assertIn('"precision_docking_config_file"', launch_text)


if __name__ == "__main__":
    unittest.main()
