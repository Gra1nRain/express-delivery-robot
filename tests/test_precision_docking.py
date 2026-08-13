import math
import pathlib
import sys
import unittest


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
    fixed_reference_to_checkpoint,
    precision_docking_configs_from_dict,
)


class PrecisionDockingConfigurationTest(unittest.TestCase):
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
                        "max_parallel_correction_m": 0.08,
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


if __name__ == "__main__":
    unittest.main()
