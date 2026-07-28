import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5OnlineReplanningTopologyTest(unittest.TestCase):
    def test_inflated_2d_costmap_drives_dwa_local_trajectory(self) -> None:
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
        dwa_runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "dwa_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["dwa_runtime"]
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
        dwa_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "dwa_local_planner_node.py"
        ).read_text(encoding="utf-8")
        control_node = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        replanning = planning["replanning"]
        global_turning_radius = planning["global_planner"][
            "min_turning_radius_m"
        ]
        runtime_turning_radius = control["motion"]["min_turning_radius_m"]
        self.assertTrue(replanning["enabled"])
        self.assertEqual(replanning["plugin"], "dwa")
        self.assertEqual(global_turning_radius, 0.81)
        self.assertEqual(runtime_turning_radius, 0.60)
        self.assertEqual(
            safety["safety"]["min_turning_radius_m"],
            runtime_turning_radius,
        )
        self.assertGreater(
            runtime_turning_radius,
            control["motion"]["ranger_driver_min_turn_radius_m"],
        )
        self.assertEqual(replanning["obstacle_source"], "costmap")
        self.assertEqual(
            replanning["costmap_topic"],
            safety["proximity_stop"]["costmap_topic"],
        )
        self.assertEqual(replanning["costmap_topic"], "/avoidance/local_costmap")
        self.assertEqual(replanning["expected_obstacle_frame"], "body")
        self.assertEqual(replanning["costmap_occupancy_threshold"], 50)
        self.assertGreaterEqual(replanning["obstacle_clearance_m"], 0.30)
        self.assertLess(replanning["obstacle_clearance_m"], 0.45)
        self.assertEqual(safety["proximity_stop"]["input_type"], "laser_scan")
        self.assertEqual(safety["proximity_stop"]["input_scan_topic"], "/scan")
        self.assertEqual(
            safety["pointcloud_to_laserscan"]["output_topic"],
            "/scan",
        )
        self.assertGreater(replanning["prediction_horizon_s"], 0.0)
        self.assertEqual(
            replanning["local_trajectory_topic"],
            control["visualization"]["local_trajectory_topic"],
        )
        self.assertEqual(
            replanning["local_stop_request_topic"],
            "/planning/local_stop_request",
        )
        self.assertIn("dwa_local_planner_node", planning_setup)
        self.assertIn('executable="dwa_local_planner_node"', launch_text)
        self.assertNotIn('executable="local_replanner_node"', launch_text)
        self.assertIn("start_local_replanner", launch_text)
        self.assertIn("OccupancyGrid", dwa_node)
        self.assertNotIn("PointCloud2", dwa_node)
        self.assertIn("DWALocalPlanner", dwa_node)
        self.assertIn("local_stop_request_topic", dwa_node)
        dwa_launch = launch_text.split(
            'executable="dwa_local_planner_node"',
            maxsplit=1,
        )[1].split(
            'executable="proximity_stop_node"',
            maxsplit=1,
        )[0]
        self.assertIn(
            '"min_turning_radius_m": motion["min_turning_radius_m"]',
            dwa_launch,
        )
        self.assertNotIn(
            '"min_turning_radius_m": global_planner[',
            dwa_launch,
        )
        self.assertIn("local_trajectory_topic", control_node)
        self.assertIn("local_stop_request_topic", control_node)
        self.assertIn("LOCAL_PLANNER_STOP", control_node)
        self.assertIn("parameterize_local_path", control_node)
        self.assertIn("replace_trajectory", control_node)
        self.assertIn("LOCAL_PLAN_STALE", control_node)
        self.assertEqual(dwa_runtime["frequency_hz"], 1.0)
        self.assertLessEqual(
            dwa_runtime["max_reference_deviation_m"],
            0.65,
        )
        self.assertGreater(
            dwa_runtime["path_distance_weight"],
            replanning["path_distance_weight"],
        )
        self.assertGreater(dwa_runtime["direction_switch_penalty"], 0.0)
        self.assertLessEqual(
            control["trajectory_tracker"]["mppi"][
                "curvature_noise_std_1pm"
            ],
            0.15,
        )
        self.assertIn('"dwa_runtime_params_file"', launch_text)
        self.assertIn(
            '"frequency_hz": dwa_runtime["frequency_hz"]',
            dwa_launch,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n'
            '                "dwa_runtime_params_file",',
            launch_text,
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
                control = yaml.safe_load(
                    path.read_text(encoding="utf-8")
                )
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
        )[1].split("    def _planning_cycle", maxsplit=1)[0]
        self.assertIn("rclpy.time.Time()", callback_body)
        self.assertNotIn("rclpy.time.Time.from_msg(message.header.stamp)", callback_body)

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
        )[1].split("    def _planning_cycle", maxsplit=1)[0]
        self.assertIn("received_at_s = self.get_clock().now().nanoseconds / 1e9", callback_body)
        self.assertIn("self._costmap_stamp_s = received_at_s", callback_body)
        self.assertNotIn("_stamp_to_seconds(message.header.stamp)", callback_body)


if __name__ == "__main__":
    unittest.main()
