import math
import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5OnlineReplanningTopologyTest(unittest.TestCase):
    def test_day5_runs_local_hybrid_astar_without_dwa(self) -> None:
        planning = yaml.safe_load(
            (REPO_ROOT / "config" / "planning" / "planning_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        control = yaml.safe_load(
            (REPO_ROOT / "config" / "control" / "control_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        safety = yaml.safe_load(
            (REPO_ROOT / "config" / "safety" / "safety_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "local_hybrid_astar_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["local_hybrid_astar_runtime"]
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")
        planning_setup = (
            REPO_ROOT / "src" / "competition_planning" / "setup.py"
        ).read_text(encoding="utf-8")
        local_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")
        local_algorithm = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_trajectory_planner.py"
        ).read_text(encoding="utf-8")

        replanning = planning["replanning"]
        self.assertTrue(replanning["enabled"])
        self.assertEqual(
            runtime["plugin"],
            "reference_aware_hybrid_astar",
        )
        self.assertEqual(replanning["obstacle_source"], "costmap")
        self.assertEqual(
            replanning["costmap_topic"],
            safety["proximity_stop"]["costmap_topic"],
        )
        self.assertEqual(replanning["expected_obstacle_frame"], "body")
        self.assertEqual(
            replanning["local_trajectory_topic"],
            control["visualization"]["local_trajectory_topic"],
        )
        self.assertEqual(
            replanning["local_stop_request_topic"],
            "/planning/local_stop_request",
        )

        self.assertEqual(runtime["frequency_hz"], 1.0)
        self.assertEqual(runtime["max_obstacle_age_s"], 2.0)
        self.assertEqual(runtime["max_odom_age_s"], 0.50)
        self.assertEqual(runtime["inflation_radius_m"], 0.04)
        self.assertEqual(runtime["lookahead_distance_m"], 3.00)
        self.assertEqual(runtime["search_padding_m"], 1.50)
        self.assertEqual(runtime["reference_deviation_weight"], 2.0)
        self.assertEqual(runtime["relaxed_segment_entry_ref"], "random_obstacle_entry")
        self.assertEqual(runtime["relaxed_segment_exit_ref"], "random_obstacle_exit")
        self.assertEqual(runtime["relaxed_activation_distance_m"], 1.0)
        self.assertEqual(runtime["relaxed_reference_deviation_weight"], 0.5)
        self.assertEqual(runtime["relaxed_corridor_half_width_m"], 0.85)
        self.assertEqual(runtime["relaxed_step_length_m"], 0.30)
        self.assertEqual(runtime["relaxed_extension_curvature_bins"], 7)
        self.assertEqual(runtime["relaxed_goal_heading_tolerance_deg"], 20.0)
        self.assertEqual(runtime["trajectory_switch_improvement_ratio"], 0.15)
        self.assertEqual(runtime["obstacle_clearance_distance_m"], 0.20)
        self.assertEqual(runtime["obstacle_clearance_weight"], 0.8)
        self.assertEqual(runtime["search_heuristic_weight"], 1.2)
        self.assertEqual(
            control["trajectory_tracker"]["mppi"][
                "local_plan_reuse_position_tolerance_m"
            ],
            0.05,
        )
        self.assertEqual(
            control["trajectory_tracker"]["mppi"][
                "local_plan_reuse_heading_tolerance_deg"
            ],
            5.0,
        )
        self.assertEqual(
            control["trajectory_tracker"]["mppi"][
                "controller_goal_position_tolerance_m"
            ],
            0.03,
        )
        self.assertEqual(
            control["trajectory_tracker"]["mppi"][
                "controller_goal_heading_tolerance_deg"
            ],
            2.0,
        )
        alternate_control = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "control"
                / "control_params_day5_chassis_center_yneg020.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(
            alternate_control["trajectory_tracker"]["mppi"][
                "local_plan_reuse_position_tolerance_m"
            ],
            0.05,
        )
        self.assertEqual(
            alternate_control["trajectory_tracker"]["mppi"][
                "local_plan_reuse_heading_tolerance_deg"
            ],
            5.0,
        )
        self.assertEqual(safety["proximity_stop"]["grid_inflation_radius_m"], 0.44)
        vehicle_corner_radius_m = math.hypot(
            safety["proximity_stop"]["vehicle_length_m"] * 0.5,
            safety["proximity_stop"]["vehicle_width_m"] * 0.5,
        )
        effective_hard_clearance_m = (
            safety["proximity_stop"]["grid_inflation_radius_m"]
            + runtime["inflation_radius_m"]
        )
        self.assertGreaterEqual(
            effective_hard_clearance_m,
            vehicle_corner_radius_m + 0.03,
        )
        self.assertEqual(runtime["docking_activation_distance_m"], 1.5)
        self.assertEqual(runtime["docking_costmap_occupancy_threshold"], 100)
        self.assertEqual(runtime["docking_vehicle_length_m"], 0.72)
        self.assertEqual(runtime["docking_vehicle_width_m"], 0.50)
        self.assertEqual(runtime["docking_work_side_clearance_m"], 0.03)
        self.assertGreater(
            runtime["docking_non_work_side_clearance_m"],
            runtime["docking_work_side_clearance_m"],
        )
        self.assertEqual(runtime["planning_timeout_s"], 0.75)
        self.assertEqual(runtime["relaxed_extension_timeout_s"], 0.75)
        self.assertLess(
            runtime["relaxed_extension_timeout_s"],
            replanning["local_trajectory_timeout_s"],
        )
        self.assertLessEqual(
            runtime["planning_timeout_s"],
            replanning["local_trajectory_timeout_s"],
        )
        self.assertEqual(runtime["reference_search_window_points"], 160)
        self.assertEqual(
            control["motion"]["min_turning_radius_m"],
            0.60,
        )

        self.assertIn('executable="local_replanner_node"', launch_text)
        self.assertNotIn("dwa_local_planner", launch_text)
        self.assertNotIn("dwa_runtime", launch_text)
        self.assertIn('"map_file": LaunchConfiguration("map_file")', launch_text)
        self.assertIn(
            '"min_turning_radius_m": motion["min_turning_radius_m"]',
            launch_text,
        )
        self.assertIn('"local_planner_runtime_params_file"', launch_text)
        self.assertIn(
            '"planning_timeout_s": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"relaxed_extension_timeout_s": local_runtime[',
            launch_text,
        )
        self.assertIn('"relaxed_segment_entry_ref": local_runtime[', launch_text)
        self.assertIn(
            '"relaxed_goal_heading_tolerance_deg": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"relaxed_extension_curvature_bins": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"obstacle_clearance_distance_m": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"obstacle_clearance_weight": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"search_heuristic_weight": local_runtime[',
            launch_text,
        )
        self.assertIn(
            '"local_plan_reuse_position_tolerance_m": mppi[',
            launch_text,
        )
        self.assertIn(
            '"local_plan_reuse_heading_tolerance_deg": mppi[',
            launch_text,
        )
        self.assertIn(
            '"controller_goal_position_tolerance_m": mppi[',
            launch_text,
        )
        self.assertIn(
            '"controller_goal_heading_tolerance_deg": mppi[',
            launch_text,
        )
        self.assertIn(
            '"semantic_map_file": LaunchConfiguration("semantic_map_file")',
            launch_text,
        )
        self.assertIn('"trajectory_switch_improvement_ratio": local_runtime[', launch_text)
        self.assertIn(
            'local_runtime.get("plugin") != "reference_aware_hybrid_astar"',
            launch_text,
        )
        self.assertIn("local_replanner_node", planning_setup)
        self.assertNotIn("dwa_local_planner_node", planning_setup)
        self.assertIn("LocalTrajectoryPlanner", local_node)
        self.assertIn("local_stop_request_topic", local_node)
        self.assertIn("HYBRID_ASTAR_NO_FEASIBLE_PATH", local_node)
        self.assertIn("HYBRID_ASTAR_TIMEOUT", local_node)
        self.assertIn("planning_timeout_s", local_node)
        self.assertIn("relaxed_extension_timeout_s", local_node)
        self.assertIn("relaxed_extension_curvature_bins", local_node)
        self.assertIn("HybridAStarPlanner", local_algorithm)
        self.assertFalse(
            (
                REPO_ROOT
                / "src"
                / "competition_planning"
                / "competition_planning"
                / "dwa_local_planner.py"
            ).exists()
        )
        self.assertFalse(
            (
                REPO_ROOT
                / "src"
                / "competition_planning"
                / "competition_planning"
                / "dwa_local_planner_node.py"
            ).exists()
        )

    def test_ranger_adapter_keeps_safety_before_final_cmd_vel(self) -> None:
        control = yaml.safe_load(
            (REPO_ROOT / "config" / "control" / "control_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")
        control_setup = (
            REPO_ROOT / "src" / "competition_control" / "setup.py"
        ).read_text(encoding="utf-8")

        self.assertIn("ranger_twist_adapter_node", control_setup)
        self.assertIn("ranger_twist_adapter_node", launch_text)
        self.assertIn("start_chassis_adapter", launch_text)
        self.assertIn('"command_output_topic"', launch_text)
        self.assertIn('default_value="/cmd_vel_safe"', launch_text)
        self.assertIn('"chassis_adapter_input_topic"', launch_text)
        self.assertIn('default_value="/cmd_vel_safe"', launch_text)
        self.assertIn('"chassis_adapter_output_topic"', launch_text)
        self.assertIn('default_value="/cmd_vel"', launch_text)
        self.assertEqual(
            control["motion"]["ranger_driver_min_turn_radius_m"],
            0.47644,
        )

    def test_day5_runtime_turning_radius_is_consistent_across_profiles(self) -> None:
        control_paths = (
            REPO_ROOT / "config" / "control" / "control_params.yaml",
            REPO_ROOT
            / "config"
            / "control"
            / "control_params_day5_chassis_center_yneg020.yaml",
        )
        safety_paths = (
            REPO_ROOT / "config" / "safety" / "safety_params.yaml",
            *sorted(
                (REPO_ROOT / "config" / "safety").glob(
                    "safety_params_day5_*.yaml"
                )
            ),
        )

        for path in control_paths:
            with self.subTest(path=path.name):
                control = yaml.safe_load(path.read_text(encoding="utf-8"))
                motion = control["motion"]
                self.assertEqual(motion["min_turning_radius_m"], 0.60)
                self.assertGreater(
                    motion["min_turning_radius_m"],
                    motion["ranger_driver_min_turn_radius_m"],
                )
                self.assertLessEqual(
                    control["trajectory_tracker"]["mppi"][
                        "curvature_noise_std_1pm"
                    ],
                    0.15,
                )
        for path in safety_paths:
            with self.subTest(path=path.name):
                safety = yaml.safe_load(
                    path.read_text(encoding="utf-8")
                )["safety"]
                self.assertEqual(safety["min_turning_radius_m"], 0.60)

    def test_local_costmap_transform_uses_latest_tf(self) -> None:
        replanner_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")

        callback_body = replanner_node.split(
            "    def _costmap_callback(self, message: OccupancyGrid) -> None:",
            maxsplit=1,
        )[1].split("    def _odom_callback", maxsplit=1)[0]
        self.assertIn("rclpy.time.Time()", callback_body)
        self.assertNotIn(
            "rclpy.time.Time.from_msg(message.header.stamp)",
            callback_body,
        )

    def test_local_costmap_freshness_uses_receipt_time(self) -> None:
        replanner_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")

        callback_body = replanner_node.split(
            "    def _costmap_callback(self, message: OccupancyGrid) -> None:",
            maxsplit=1,
        )[1].split("    def _odom_callback", maxsplit=1)[0]
        self.assertIn("received_s = self._now_s()", callback_body)
        self.assertIn("self._obstacle_received_s = received_s", callback_body)
        self.assertIn(
            "self._obstacle_header_stamp_s = _stamp_to_seconds",
            callback_body,
        )


if __name__ == "__main__":
    unittest.main()
