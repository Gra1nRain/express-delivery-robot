import math
import sys
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
MIGRATION_ROOT = REPO_ROOT / "Piper_Grasp_Humble_Migration_20260723"
sys.path.insert(0, str(MIGRATION_ROOT))

from block_grasp_planner import (  # noqa: E402
    build_block_rpy_candidates,
    build_tool_axis_pregrasp_pose,
    build_world_yz_pregrasp_pose,
    choose_reachable_block_candidate,
    project_yolo_bbox_center_with_robust_depth,
    robust_point_cloud_box_center,
)


class BlockGraspPlannerTest(unittest.TestCase):
    def test_uses_robust_box_center_for_uneven_block_point_density(self):
        dense_left_face = np.repeat(
            [[0.08, -0.42, 0.03]],
            60,
            axis=0,
        )
        sparse_right_face = np.repeat(
            [[0.12, -0.38, 0.07]],
            40,
            axis=0,
        )
        gross_outliers = np.array(
            [
                [-1.0, -1.0, -1.0],
                [-0.8, -0.9, -0.7],
                [0.9, 0.8, 0.9],
                [1.0, 1.0, 1.0],
            ]
        )
        points = np.vstack(
            [dense_left_face, sparse_right_face, gross_outliers]
        )

        center, bounds_min, bounds_max = robust_point_cloud_box_center(
            points,
            lower_quantile=0.05,
            upper_quantile=0.95,
        )

        np.testing.assert_allclose(bounds_min, [0.08, -0.42, 0.03])
        np.testing.assert_allclose(bounds_max, [0.12, -0.38, 0.07])
        np.testing.assert_allclose(center, [0.10, -0.40, 0.05])
        self.assertFalse(
            np.allclose(np.median(points, axis=0), center)
        )

    def test_yolo_bbox_center_uses_central_depth_not_cloud_edges(self):
        camera_matrix = np.array(
            [
                [500.0, 0.0, 320.0],
                [0.0, 500.0, 240.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        target_bbox = [340.0, 200.0, 380.0, 280.0]
        central_pixels = np.array(
            [
                [u, v]
                for u in range(354, 367, 2)
                for v in range(232, 249, 4)
            ],
            dtype=np.int32,
        )
        central_depths = np.linspace(
            0.495,
            0.505,
            central_pixels.shape[0],
        )
        central_points = np.column_stack(
            [
                np.zeros(central_pixels.shape[0]),
                np.zeros(central_pixels.shape[0]),
                central_depths,
            ]
        )
        edge_pixels = np.array(
            [[342, v] for v in range(202, 279, 2)]
            + [[378, v] for v in range(202, 279, 2)],
            dtype=np.int32,
        )
        edge_points = np.column_stack(
            [
                np.zeros(edge_pixels.shape[0]),
                np.zeros(edge_pixels.shape[0]),
                np.full(edge_pixels.shape[0], 0.9),
            ]
        )

        point_cam, diagnostics = (
            project_yolo_bbox_center_with_robust_depth(
                object_points_cam=np.vstack(
                    [central_points, edge_points]
                ),
                object_pixels=np.vstack(
                    [central_pixels, edge_pixels]
                ),
                target_bbox=target_bbox,
                camera_matrix=camera_matrix,
            )
        )

        np.testing.assert_allclose(point_cam, [0.04, 0.0, 0.5])
        self.assertEqual(diagnostics["pixel_center"], [360.0, 240.0])
        self.assertEqual(
            diagnostics["depth_source"],
            "central_object_points",
        )
        self.assertEqual(
            diagnostics["depth_points"],
            central_pixels.shape[0],
        )

    def test_builds_near_vertical_candidates_with_axis_aligned_yaw(self):
        candidates = build_block_rpy_candidates(
            roll_deg=-179.234,
            fallback_yaw_deg=90.144,
            object_yaw_deg=41.339,
            pitch_candidates_deg=(30.0, 35.0, 40.0),
        )

        self.assertEqual(len(candidates), 6)
        self.assertEqual(candidates[0]["pitch_deg"], 30.0)
        self.assertAlmostEqual(candidates[0]["yaw_deg"], 41.339)
        self.assertAlmostEqual(candidates[1]["yaw_deg"], -138.661)

    def test_builds_near_axis_fallback_yaws(self):
        candidates = build_block_rpy_candidates(
            roll_deg=-179.449,
            fallback_yaw_deg=90.183,
            object_yaw_deg=-26.665,
            pitch_candidates_deg=(30.0,),
            yaw_offset_candidates_deg=(0.0, -20.0, 20.0),
        )

        self.assertEqual(len(candidates), 6)
        self.assertEqual(
            [candidate["yaw_offset_deg"] for candidate in candidates],
            [0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
        )
        self.assertAlmostEqual(candidates[2]["yaw_deg"], -46.665)
        self.assertAlmostEqual(candidates[3]["yaw_deg"], 133.335)

    def test_pregrasp_backs_away_along_tool_axis(self):
        grasp_pose = {
            "x": 0.0822,
            "y": -0.4171,
            "z": 0.1409,
            "roll": math.radians(-179.234),
            "pitch": math.radians(35.0),
            "yaw": math.radians(90.144),
            "gripper": 0.08,
        }

        pregrasp = build_tool_axis_pregrasp_pose(
            grasp_pose,
            backoff_m=0.060,
        )
        displacement = np.array(
            [
                pregrasp["x"] - grasp_pose["x"],
                pregrasp["y"] - grasp_pose["y"],
                pregrasp["z"] - grasp_pose["z"],
            ]
        )

        self.assertAlmostEqual(float(np.linalg.norm(displacement)), 0.060)
        self.assertGreater(pregrasp["y"], grasp_pose["y"])
        self.assertGreater(pregrasp["z"], grasp_pose["z"])
        for key in ("roll", "pitch", "yaw", "gripper"):
            self.assertEqual(pregrasp[key], grasp_pose[key])

    def test_block_pregrasp_moves_only_in_world_y_and_z(self):
        grasp_pose = {
            "x": 0.0822,
            "y": -0.4171,
            "z": 0.1409,
            "roll": math.radians(-179.234),
            "pitch": math.radians(35.0),
            "yaw": math.radians(-26.665),
            "gripper": 0.08,
        }

        pregrasp = build_world_yz_pregrasp_pose(
            grasp_pose,
            backoff_y_m=0.030,
            lift_z_m=0.050,
        )

        self.assertEqual(pregrasp["x"], grasp_pose["x"])
        self.assertAlmostEqual(pregrasp["y"], grasp_pose["y"] + 0.030)
        self.assertAlmostEqual(pregrasp["z"], grasp_pose["z"] + 0.050)
        for key in ("roll", "pitch", "yaw", "gripper"):
            self.assertEqual(pregrasp[key], grasp_pose[key])

    def test_selects_smallest_tilt_with_required_joint_margin(self):
        evaluations = [
            {
                "name": "pitch_30",
                "pitch_deg": 30.0,
                "minimum_joint_margin_rad": 0.086,
                "max_joint_step_rad": 0.029,
                "max_joint_travel_rad": 0.40,
            },
            {
                "name": "pitch_35",
                "pitch_deg": 35.0,
                "minimum_joint_margin_rad": 0.202,
                "max_joint_step_rad": 0.027,
                "max_joint_travel_rad": 0.42,
            },
            {
                "name": "pitch_40",
                "pitch_deg": 40.0,
                "minimum_joint_margin_rad": 0.317,
                "max_joint_step_rad": 0.026,
                "max_joint_travel_rad": 0.45,
            },
        ]

        selected = choose_reachable_block_candidate(
            evaluations,
            minimum_joint_margin_rad=0.15,
        )

        self.assertEqual(selected["name"], "pitch_35")

    def test_rejects_candidates_without_required_joint_margin(self):
        with self.assertRaisesRegex(RuntimeError, "关节余量"):
            choose_reachable_block_candidate(
                [
                    {
                        "name": "old_pose",
                        "pitch_deg": 25.674,
                        "minimum_joint_margin_rad": 0.020,
                        "max_joint_step_rad": 0.026,
                        "max_joint_travel_rad": 0.41,
                    }
                ],
                minimum_joint_margin_rad=0.15,
            )

    def test_prefers_smallest_safe_yaw_offset_before_extra_margin(self):
        selected = choose_reachable_block_candidate(
            [
                {
                    "name": "axis_aligned_but_unsafe",
                    "pitch_deg": 30.0,
                    "yaw_offset_deg": 0.0,
                    "minimum_joint_margin_rad": 0.10,
                    "max_joint_step_rad": 0.010,
                    "max_joint_travel_rad": 1.0,
                },
                {
                    "name": "offset_20_safe",
                    "pitch_deg": 30.0,
                    "yaw_offset_deg": -20.0,
                    "minimum_joint_margin_rad": 0.20,
                    "max_joint_step_rad": 0.010,
                    "max_joint_travel_rad": 1.2,
                },
                {
                    "name": "offset_30_more_margin",
                    "pitch_deg": 30.0,
                    "yaw_offset_deg": -30.0,
                    "minimum_joint_margin_rad": 0.40,
                    "max_joint_step_rad": 0.008,
                    "max_joint_travel_rad": 1.1,
                },
            ],
            minimum_joint_margin_rad=0.15,
        )

        self.assertEqual(selected["name"], "offset_20_safe")


if __name__ == "__main__":
    unittest.main()
