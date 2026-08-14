import math
import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mission_checkpoints import MissionCheckpoint
from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    VehicleState,
)
from competition_control.precision_docking import (
    PrecisionDockingController,
    PrecisionDockingPhase,
    RequestedMotionMode,
    calibrated_straight_reference,
    fixed_reference_to_checkpoint,
    precision_docking_configs_from_dict,
    straight_followup_anchors_from_config,
)
from competition_control.shelf_alignment import ShelfObservation


class PrecisionDockingConfigurationTest(unittest.TestCase):
    def test_shipped_route_declares_calibrated_straight_followups(self) -> None:
        dock_params = yaml.safe_load(
            (REPO_ROOT / "config" / "docking" / "debug_dock_params.yaml")
            .read_text(encoding="utf-8")
        )

        followups = straight_followup_anchors_from_config(
            dock_params,
            ("pickup_front", "pickup_rear", "drop_front", "drop_rear"),
        )

        self.assertEqual(
            followups,
            {
                "pickup_rear": "pickup_front",
                "drop_rear": "drop_front",
            },
        )

    def test_calibrated_followup_starts_at_actual_front_pose_and_stays_straight(
        self,
    ) -> None:
        anchor_state = VehicleState(
            x=8.50,
            y=-1.20,
            yaw=math.radians(1.5),
            linear_speed_mps=0.0,
        )
        anchor = MissionCheckpoint("pickup_front", 8.413, -0.081, 0.015)
        followup = MissionCheckpoint("pickup_rear", 9.013, -0.091, 0.022)

        target, trajectory = calibrated_straight_reference(
            anchor_state,
            anchor,
            followup,
            speed_mps=0.08,
        )

        semantic_distance = (
            math.cos(anchor.yaw) * (followup.x - anchor.x)
            + math.sin(anchor.yaw) * (followup.y - anchor.y)
        )
        self.assertAlmostEqual(
            math.hypot(target.x - anchor_state.x, target.y - anchor_state.y),
            semantic_distance,
            delta=1e-9,
        )
        self.assertAlmostEqual(target.yaw, anchor_state.yaw, delta=1e-9)
        self.assertAlmostEqual(trajectory.points[0].x, anchor_state.x, delta=1e-9)
        self.assertAlmostEqual(trajectory.points[0].y, anchor_state.y, delta=1e-9)
        self.assertEqual(trajectory.points[-1].ref_id, "pickup_rear")
        self.assertTrue(all(point.curvature == 0.0 for point in trajectory.points))
        self.assertTrue(
            all(
                math.isclose(point.yaw, anchor_state.yaw, abs_tol=1e-9)
                for point in trajectory.points
            )
        )

    def test_shipped_pickup_and_drop_followup_distances_are_valid(self) -> None:
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )

        distances = {}
        for anchor_ref, followup_ref in (
            ("pickup_front", "pickup_rear"),
            ("drop_front", "drop_rear"),
        ):
            anchor_raw = semantic_map["points"][anchor_ref]
            followup_raw = semantic_map["points"][followup_ref]
            anchor = MissionCheckpoint(
                anchor_ref,
                float(anchor_raw["x"]),
                float(anchor_raw["y"]),
                float(anchor_raw["yaw"]),
            )
            followup = MissionCheckpoint(
                followup_ref,
                float(followup_raw["x"]),
                float(followup_raw["y"]),
                float(followup_raw["yaw"]),
            )
            _, trajectory = calibrated_straight_reference(
                VehicleState(anchor.x, anchor.y, anchor.yaw, 0.0),
                anchor,
                followup,
                speed_mps=0.08,
            )
            distances[followup_ref] = trajectory.points[-1].s

        self.assertAlmostEqual(distances["pickup_rear"], 0.60, delta=0.01)
        self.assertAlmostEqual(distances["drop_rear"], 0.581, delta=0.01)

    def test_calibrated_followup_rejects_non_straight_semantic_pair(self) -> None:
        with self.assertRaisesRegex(ValueError, "not a straight forward pair"):
            calibrated_straight_reference(
                VehicleState(0.0, 0.0, 0.0, 0.0),
                MissionCheckpoint("front", 0.0, 0.0, 0.0),
                MissionCheckpoint("rear", 0.50, 0.20, 0.0),
                speed_mps=0.08,
            )

    def test_route_maps_semantic_refs_to_reusable_site_profiles(self) -> None:
        route = {
            "precision_docking": {
                "checkpoints": {
                    "pickup_front": "shelf",
                    "drop_front": "shelf",
                }
            }
        }
        dock_params = {
            "precision_profiles": {
                "shelf": {
                    "activation_distance_m": 1.5,
                    "trim_entry_distance_m": 0.10,
                    "final_position_tolerance_m": 0.03,
                    "heading_realign_tolerance_deg": 4.0,
                    "heading_trim_target_deg": 2.0,
                }
            }
        }

        configs = precision_docking_configs_from_dict(route, dock_params)

        self.assertEqual(set(configs), {"pickup_front", "drop_front"})
        self.assertAlmostEqual(configs["pickup_front"].activation_distance_m, 1.5)
        self.assertIsNot(configs["pickup_front"], configs["drop_front"])

    def test_missing_profile_is_rejected_before_motion(self) -> None:
        route = {"precision_docking": {"checkpoints": {"pickup_front": "unknown"}}}

        with self.assertRaisesRegex(ValueError, "unknown precision profile"):
            precision_docking_configs_from_dict(route, {"precision_profiles": {}})

    def test_trim_entry_dead_zone_is_rejected_before_motion(self) -> None:
        route = {"precision_docking": {"checkpoints": {"dock": "shelf"}}}
        dock_params = {
            "precision_profiles": {
                "shelf": {
                    "activation_distance_m": 1.5,
                    "trim_entry_distance_m": 0.10,
                    "final_position_tolerance_m": 0.03,
                    "heading_realign_tolerance_deg": 4.0,
                    "heading_trim_target_deg": 2.0,
                    "max_parallel_correction_m": 0.08,
                }
            }
        }

        with self.assertRaisesRegex(ValueError, "cover trim entry distance"):
            precision_docking_configs_from_dict(route, dock_params)

    def test_shipped_profiles_trim_the_observed_nine_centimeter_error(self) -> None:
        route = {"precision_docking": {"checkpoints": {"dock": "shelf_dock"}}}
        for filename in ("debug_dock_params.yaml", "competition_dock_params.yaml"):
            dock_params = yaml.safe_load(
                (REPO_ROOT / "config" / "docking" / filename).read_text(
                    encoding="utf-8"
                )
            )
            config = precision_docking_configs_from_dict(route, dock_params)["dock"]
            controller = PrecisionDockingController(config)
            checkpoint = MissionCheckpoint("dock", 0.0, 0.0, 0.0)
            state = VehicleState(
                x=-0.015,
                y=0.0944,
                yaw=math.radians(-3.47),
                linear_speed_mps=0.0,
            )
            shelf_observation = ShelfObservation(
                side_distance_m=0.44,
                heading_error_rad=0.0,
                point_count=30,
                residual_rms_m=0.01,
                span_m=0.60,
            )

            controller.update(
                now_s=0.0,
                state=state,
                checkpoint=checkpoint,
                yaw_rate_radps=0.0,
                shelf_observation=shelf_observation,
            )
            decision = controller.update(
                now_s=0.31,
                state=state,
                checkpoint=checkpoint,
                yaw_rate_radps=0.0,
                shelf_observation=shelf_observation,
            )

            self.assertEqual(
                decision.phase,
                PrecisionDockingPhase.POSITION_TRIM,
                filename,
            )
            self.assertEqual(decision.motion_mode, RequestedMotionMode.PARALLEL)
            self.assertGreater(
                math.hypot(decision.linear_x_mps, decision.linear_y_mps),
                0.0,
            )

    def test_fixed_reference_is_sliced_from_vehicle_to_semantic_checkpoint(
        self,
    ) -> None:
        trajectory = ControlTrajectory(
            frame_id="map",
            route_name="field_route",
            points=tuple(
                ControlTrajectoryPoint(
                    x=index * 0.10,
                    y=0.0,
                    yaw=0.0,
                    s=index * 0.10,
                    curvature=0.0,
                    v=0.10,
                    t=index * 0.50,
                    ref_id="dock" if index == 20 else None,
                )
                for index in range(31)
            ),
        )

        fixed = fixed_reference_to_checkpoint(
            trajectory,
            VehicleState(x=0.80, y=0.02, yaw=0.0, linear_speed_mps=0.0),
            "dock",
        )

        self.assertLessEqual(fixed.points[0].x, 0.80)
        self.assertEqual(fixed.points[-1].ref_id, "dock")
        self.assertEqual(fixed.points[0].s, 0.0)
        self.assertEqual(fixed.points[0].t, 0.0)
        self.assertTrue(
            all(
                right.s > left.s and right.t > left.t
                for left, right in zip(fixed.points, fixed.points[1:])
            )
        )


class PrecisionDockingControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        configs = precision_docking_configs_from_dict(
            {"precision_docking": {"checkpoints": {"dock": "shelf"}}},
            {
                "precision_profiles": {
                    "shelf": {
                        "activation_distance_m": 1.5,
                        "trim_entry_distance_m": 0.10,
                        "final_position_tolerance_m": 0.03,
                        "heading_realign_tolerance_deg": 4.0,
                        "heading_trim_target_deg": 2.0,
                        "settle_time_s": 0.30,
                        "stable_time_s": 0.50,
                        "max_spin_correction_deg": 10.0,
                        "max_parallel_correction_m": 0.10,
                        "spin_gain": 1.0,
                        "spin_min_yaw_rate_radps": 0.06,
                        "spin_max_yaw_rate_radps": 0.15,
                        "parallel_gain": 0.8,
                        "parallel_min_speed_mps": 0.04,
                        "parallel_max_speed_mps": 0.06,
                    }
                }
            },
        )
        self.controller = PrecisionDockingController(configs["dock"])
        self.goal = MissionCheckpoint("dock", 0.0, 0.0, 0.0)

    def update(
        self,
        now_s: float,
        *,
        x: float,
        y: float = 0.0,
        yaw_deg: float = 0.0,
        speed_mps: float = 0.0,
        yaw_rate_radps: float = 0.0,
    ):
        return self.controller.update(
            now_s=now_s,
            state=VehicleState(
                x=x,
                y=y,
                yaw=math.radians(yaw_deg),
                linear_speed_mps=speed_mps,
            ),
            checkpoint=self.goal,
            yaw_rate_radps=yaw_rate_radps,
        )

    def test_far_from_dock_keeps_normal_dynamic_replanning(self) -> None:
        decision = self.update(0.0, x=-2.0)

        self.assertEqual(decision.phase, PrecisionDockingPhase.NORMAL_NAV)
        self.assertTrue(decision.use_dynamic_replanning)
        self.assertFalse(decision.use_fixed_reference)

    def test_precision_approach_uses_fixed_reference_before_inflation_deadlock(
        self,
    ) -> None:
        decision = self.update(0.0, x=-1.40)

        self.assertEqual(decision.phase, PrecisionDockingPhase.PRECISION_APPROACH)
        self.assertFalse(decision.use_dynamic_replanning)
        self.assertTrue(decision.use_fixed_reference)
        self.assertEqual(decision.motion_mode, RequestedMotionMode.DUAL_ACKERMANN)

    def test_settles_then_spins_only_when_heading_needs_correction(self) -> None:
        settling = self.update(0.0, x=-0.08, yaw_deg=-5.0)
        still_settling = self.update(0.20, x=-0.08, yaw_deg=-5.0)
        spinning = self.update(0.31, x=-0.08, yaw_deg=-5.0)

        self.assertEqual(settling.phase, PrecisionDockingPhase.STOP_SETTLE)
        self.assertEqual(still_settling.phase, PrecisionDockingPhase.STOP_SETTLE)
        self.assertEqual(spinning.phase, PrecisionDockingPhase.HEADING_TRIM)
        self.assertEqual(spinning.motion_mode, RequestedMotionMode.SPIN)
        self.assertGreater(spinning.yaw_rate_radps, 0.0)
        self.assertEqual((spinning.linear_x_mps, spinning.linear_y_mps), (0.0, 0.0))

    def test_parallel_mode_corrects_longitudinal_and_lateral_error_together(
        self,
    ) -> None:
        self.update(0.0, x=-0.05, y=-0.04, yaw_deg=3.0)
        decision = self.update(0.31, x=-0.05, y=-0.04, yaw_deg=3.0)

        self.assertEqual(decision.phase, PrecisionDockingPhase.POSITION_TRIM)
        self.assertEqual(decision.motion_mode, RequestedMotionMode.PARALLEL)
        self.assertGreater(decision.linear_x_mps, 0.0)
        self.assertGreater(decision.linear_y_mps, 0.0)
        self.assertEqual(decision.yaw_rate_radps, 0.0)

    def test_heading_inside_four_degrees_does_not_trigger_spin(self) -> None:
        self.update(0.0, x=-0.02, yaw_deg=3.0)
        decision = self.update(0.31, x=-0.02, yaw_deg=3.0)

        self.assertEqual(decision.phase, PrecisionDockingPhase.DOCK_READY)
        self.assertEqual(decision.motion_mode, RequestedMotionMode.DUAL_ACKERMANN)
        self.assertEqual(decision.yaw_rate_radps, 0.0)

    def test_large_alignment_error_holds_instead_of_attempting_risky_trim(self) -> None:
        self.update(0.0, x=-0.08, yaw_deg=-12.0)
        decision = self.update(0.31, x=-0.08, yaw_deg=-12.0)

        self.assertEqual(decision.phase, PrecisionDockingPhase.ALIGNMENT_HOLD)
        self.assertEqual(
            (decision.linear_x_mps, decision.linear_y_mps, decision.yaw_rate_radps),
            (0.0, 0.0, 0.0),
        )

    def test_final_pose_must_remain_stable_before_ready(self) -> None:
        self.update(0.0, x=-0.02, y=0.0, yaw_deg=1.0)
        stable_started = self.update(0.31, x=-0.02, y=0.0, yaw_deg=1.0)
        not_ready = self.update(0.70, x=-0.02, y=0.0, yaw_deg=1.0)
        ready = self.update(0.82, x=-0.02, y=0.0, yaw_deg=1.0)

        self.assertEqual(stable_started.phase, PrecisionDockingPhase.DOCK_READY)
        self.assertFalse(not_ready.pose_ready)
        self.assertTrue(ready.pose_ready)


class ShelfRelativePrecisionDockingTest(unittest.TestCase):
    def setUp(self) -> None:
        configs = precision_docking_configs_from_dict(
            {"precision_docking": {"checkpoints": {"dock": "shelf"}}},
            {
                "precision_profiles": {
                    "shelf": {
                        "activation_distance_m": 1.5,
                        "trim_entry_distance_m": 0.10,
                        "final_position_tolerance_m": 0.03,
                        "heading_realign_tolerance_deg": 4.0,
                        "heading_trim_target_deg": 2.0,
                        "max_parallel_correction_m": 0.35,
                        "shelf_relative": {
                            "enabled": True,
                            "side": "RIGHT",
                            "vehicle_half_width_m": 0.25,
                            "target_side_clearance_m": 0.28,
                            "minimum_side_clearance_m": 0.12,
                            "capture_distance_m": 0.20,
                        },
                    }
                }
            },
        )
        self.controller = PrecisionDockingController(configs["dock"])
        self.goal = MissionCheckpoint("dock", 0.0, 0.0, 0.0)

    @staticmethod
    def observation(distance_m: float, heading_deg: float = 0.0) -> ShelfObservation:
        return ShelfObservation(
            side_distance_m=distance_m,
            heading_error_rad=math.radians(heading_deg),
            point_count=30,
            residual_rms_m=0.01,
            span_m=0.60,
        )

    def update(self, now_s: float, *, x: float, y: float, observation=None):
        return self.controller.update(
            now_s=now_s,
            state=VehicleState(x=x, y=y, yaw=0.0, linear_speed_mps=0.0),
            checkpoint=self.goal,
            yaw_rate_radps=0.0,
            shelf_observation=observation,
        )

    def test_map_lateral_error_is_replaced_by_measured_shelf_clearance(self) -> None:
        observation = self.observation(0.53)
        first = self.update(0.0, x=-0.05, y=0.18, observation=observation)
        ready = self.update(0.31, x=-0.05, y=0.18, observation=observation)

        self.assertEqual(first.phase, PrecisionDockingPhase.STOP_SETTLE)
        self.assertEqual(ready.phase, PrecisionDockingPhase.POSITION_TRIM)
        self.assertAlmostEqual(ready.linear_y_mps, 0.0, delta=1e-9)
        self.assertGreater(ready.linear_x_mps, 0.0)
        self.assertTrue(ready.shelf_relative_active)

    def test_too_close_vehicle_moves_away_from_right_shelf(self) -> None:
        observation = self.observation(0.34)
        self.update(0.0, x=-0.05, y=0.0, observation=observation)
        decision = self.update(0.31, x=-0.05, y=0.0, observation=observation)

        self.assertEqual(decision.phase, PrecisionDockingPhase.POSITION_TRIM)
        self.assertGreater(decision.linear_y_mps, 0.0)
        self.assertEqual(decision.linear_x_mps, 0.0)

    def test_emergency_clearance_retreat_is_not_blocked_by_trim_limit(self) -> None:
        observation = self.observation(0.20)
        self.update(0.0, x=-0.15, y=0.0, observation=observation)
        first_retreat = self.update(0.31, x=-0.15, y=0.0, observation=observation)
        continuing_retreat = self.update(
            0.36,
            x=-0.15,
            y=0.0,
            observation=observation,
        )

        self.assertGreater(first_retreat.linear_y_mps, 0.0)
        self.assertGreater(continuing_retreat.linear_y_mps, 0.0)
        self.assertEqual(continuing_retreat.phase, PrecisionDockingPhase.POSITION_TRIM)

    def test_missing_observation_holds_after_shelf_capture(self) -> None:
        decision = self.update(0.0, x=-0.05, y=0.0, observation=None)

        self.assertEqual(decision.phase, PrecisionDockingPhase.SHELF_OBSERVATION_HOLD)
        self.assertEqual(
            (decision.linear_x_mps, decision.linear_y_mps, decision.yaw_rate_radps),
            (0.0, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
