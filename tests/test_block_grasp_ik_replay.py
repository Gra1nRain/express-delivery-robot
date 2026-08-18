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
                grasp.OPEN_GRIPPER_M,
            )
            np.testing.assert_allclose(
                plan["waypoints"][4],
                object_result["depth_grasp_center"],
            )
            self.assertGreaterEqual(
                plan["minimum_joint_margin_rad"],
                grasp.BLOCK_MIN_JOINT_MARGIN_RAD,
            )
        finally:
            os.environ.clear()
            os.environ.update(previous_environment)


if __name__ == "__main__":
    unittest.main()
