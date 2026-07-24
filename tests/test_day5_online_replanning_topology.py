import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5OnlineReplanningTopologyTest(unittest.TestCase):
    def test_live_costmap_drives_reference_aware_local_trajectory(self) -> None:
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
        replanner_node = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "local_replanner_node.py"
        ).read_text(encoding="utf-8")
        control_node = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        replanning = planning["replanning"]
        self.assertTrue(replanning["enabled"])
        self.assertEqual(replanning["plugin"], "reference_aware_hybrid_astar")
        self.assertGreaterEqual(replanning["lookahead_distance_m"], 3.0)
        self.assertGreater(replanning["reference_deviation_weight"], 0.0)
        self.assertLessEqual(replanning["search_padding_m"], 1.5)
        self.assertEqual(
            replanning["costmap_topic"],
            "/avoidance/local_costmap",
        )
        self.assertEqual(
            replanning["local_trajectory_topic"],
            control["visualization"]["local_trajectory_topic"],
        )
        self.assertGreaterEqual(
            safety["proximity_stop"]["grid_x_max_m"],
            replanning["lookahead_distance_m"],
        )
        self.assertGreaterEqual(
            safety["proximity_stop"]["grid_y_max_m"],
            2.0,
        )
        self.assertIn("local_replanner_node", planning_setup)
        self.assertIn("local_replanner_node", launch_text)
        self.assertIn("start_local_replanner", launch_text)
        self.assertIn("OccupancyGrid", replanner_node)
        self.assertIn("LocalTrajectoryPlanner", replanner_node)
        self.assertIn("local_trajectory_topic", control_node)
        self.assertIn("parameterize_local_path", control_node)
        self.assertIn("replace_trajectory", control_node)
        self.assertIn("LOCAL_PLAN_STALE", control_node)

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
