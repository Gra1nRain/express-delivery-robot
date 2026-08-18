import copy
import os
import pathlib
import sys
import unittest

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
MIGRATION_ROOT = REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723"
MISSION_SRC = REPO_ROOT / "src" / "competition_mission"


class BlockGraspIkReplayTest(unittest.TestCase):
    def test_replays_blue_block_outer_workspace_ik_failure(self):
        previous_environment = os.environ.copy()
        try:
            sys.path.insert(0, str(MISSION_SRC))
            sys.path.insert(0, str(MIGRATION_ROOT))
            try:
                from competition_mission.piper_arm_backend import (
                    apply_migration_shell_defaults,
                )

                apply_migration_shell_defaults(
                    MIGRATION_ROOT / "run_grasp_single.sh"
                )
                import grasp_single as grasp
            except ModuleNotFoundError as exc:
                self.skipTest(f"Piper runtime dependency unavailable: {exc}")

            class _Logger:
                def info(self, _message):
                    pass

            class _PlannerHarness:
                get_grasp_config = grasp.PiperController.get_grasp_config
                build_grasp_waypoints = (
                    grasp.PiperController.build_grasp_waypoints
                )
                plan_adaptive_block_grasp_path = (
                    grasp.PiperController.plan_adaptive_block_grasp_path
                )
                gripper_center_offset = (
                    grasp.DEFAULT_GRIPPER_CENTER_OFFSET.copy()
                )

                def get_logger(self):
                    return _Logger()

            object_result = {
                "depth_grasp_center": np.array(
                    [-0.1199837221, -0.4343592544, 0.0410993095]
                ),
                "top_center": np.array(
                    [-0.1199837221, -0.4343592544, 0.0641386170]
                ),
                "robot_rpy_deg": np.array([-179.449, 25.0, 96.904]),
                "target_class_id": 1,
                "target_model_class_name": "blue_block",
                "target_prompt": "blue_block",
                "grasp_yaw_deg": -26.6649702075,
                "object_axes": {"axis_ratio": 1.1689914724},
            }
            start_joints = np.array(
                [-1.5437, 0.5933, -0.7181, -0.0726, 0.7851, 0.0]
            )

            plan = _PlannerHarness().plan_adaptive_block_grasp_path(
                copy.deepcopy(object_result),
                start_joints,
            )

            self.assertEqual(plan["yaw_source"], "calibrated_fallback")
            self.assertIn(abs(plan["yaw_offset_deg"]), (0.0, 20.0))
            self.assertAlmostEqual(
                plan["grasp_open"]["gripper"],
                grasp.GRASP_CONFIG[1]["gripper_open"],
            )
            self.assertAlmostEqual(
                grasp.BLOCK_GRASP_LEFT_SHIFT_M,
                grasp.BOTTLE_GRASP_LEFT_SHIFT_M,
            )
            self.assertAlmostEqual(grasp.BLOCK_FORWARD_EXTRA_M, 0.040)
            self.assertAlmostEqual(grasp.BOTTLE_FORWARD_EXTRA_M, 0.055)
            selected_rotation = grasp.rotation_from_rpy_xyz(
                plan["grasp_open"]["roll"],
                plan["grasp_open"]["pitch"],
                plan["grasp_open"]["yaw"],
            )
            expected_left_shift = selected_rotation @ np.array(
                [0.0, grasp.BLOCK_GRASP_LEFT_SHIFT_M, 0.0]
            )
            expected_left_shift[2] = 0.0
            forward_xy = selected_rotation[:, 2].copy()
            forward_xy[2] = 0.0
            expected_forward_shift = (
                forward_xy
                / np.linalg.norm(forward_xy)
                * grasp.BLOCK_FORWARD_EXTRA_M
            )
            expected_target_center = (
                object_result["depth_grasp_center"]
                + expected_left_shift
                + expected_forward_shift
            )
            np.testing.assert_allclose(
                plan["waypoints"][4],
                expected_target_center,
            )
            self.assertGreaterEqual(
                plan["minimum_joint_margin_rad"],
                grasp.BLOCK_MIN_JOINT_MARGIN_RAD,
            )

            red_start_joints = np.array(
                [-1.1961, 0.5944, -0.7180, -0.0717, 0.7852, 0.0]
            )
            red_start_transform = grasp.piper_forward_kinematics(
                red_start_joints
            )
            red_object_result = {
                "depth_grasp_center": np.array(
                    [0.1821311043, -0.3824944375, 0.0482452629]
                ),
                "top_center": np.array(
                    [0.1821311043, -0.3824944375, 0.0706522656]
                ),
                "robot_rpy_deg": grasp.ScipyRotation.from_matrix(
                    red_start_transform[:3, :3]
                ).as_euler("xyz", degrees=True),
                "target_class_id": 1,
                "target_model_class_name": "red_block",
                "target_prompt": "red_block",
                "grasp_yaw_deg": None,
                "object_axes": None,
            }
            red_plan = _PlannerHarness().plan_adaptive_block_grasp_path(
                copy.deepcopy(red_object_result),
                red_start_joints,
            )

            self.assertGreater(
                np.linalg.norm(
                    red_plan["waypoints"][4][:2]
                    - red_object_result["depth_grasp_center"][:2]
                ),
                0.05,
            )
            self.assertGreaterEqual(
                red_plan["minimum_joint_margin_rad"],
                grasp.BLOCK_MIN_JOINT_MARGIN_RAD,
            )
        finally:
            os.environ.clear()
            os.environ.update(previous_environment)


if __name__ == "__main__":
    unittest.main()
