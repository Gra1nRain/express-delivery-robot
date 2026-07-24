import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class AvoidanceTopologyTest(unittest.TestCase):
    def test_global_conservative_limits_are_frozen_in_avoidance_profile(self) -> None:
        config = yaml.safe_load(
            (
                REPO_ROOT / "config" / "avoidance" / "avoidance_params.yaml"
            ).read_text(encoding="utf-8")
        )["avoidance_manager"]["ros__parameters"]

        self.assertTrue(config["dry_run"])
        self.assertFalse(config["enable_chassis_output"])
        self.assertEqual(config["operation_mode"], "dry_run")
        self.assertEqual(config["planning_min_turning_radius_m"], 0.81)
        self.assertEqual(config["maximum_speed_mps"], 0.15)
        self.assertEqual(config["maximum_acceleration_mps2"], 0.20)
        self.assertEqual(config["maximum_deceleration_mps2"], 0.30)
        self.assertEqual(config["proximity_stop_distance_m"], 0.85)
        self.assertGreaterEqual(config["dynamic_safety_margin_m"], 0.40)

    def test_new_launch_keeps_all_existing_motion_gates_closed(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "launch"
            / "vehicle_avoidance_bringup.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"start_base": "false"', launch_text)
        self.assertIn('"start_chassis_adapter": "false"', launch_text)
        self.assertIn('"command_output_topic": "/cmd_vel_safe"', launch_text)
        self.assertIn('"start_proximity_stop": "false"', launch_text)
        self.assertNotIn('"/cmd_vel"', launch_text)

    def test_avoidance_adapter_owns_only_avoidance_topics(self) -> None:
        node_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "competition_avoidance"
            / "avoidance_manager_node.py"
        ).read_text(encoding="utf-8")

        for topic in (
            "/avoidance/status",
            "/avoidance/objects",
            "/avoidance/corridor_update",
            "/avoidance/stop_request",
            "/avoidance/local_costmap",
        ):
            self.assertIn(topic, node_text)
        self.assertNotIn('"/cmd_vel"', node_text)
        self.assertNotIn('"/planning/local_trajectory"', node_text)

    def test_odometry_adapter_bridges_fast_lio_without_touching_navigation(self) -> None:
        setup_text = (
            REPO_ROOT / "src" / "competition_avoidance" / "setup.py"
        ).read_text(encoding="utf-8")
        startup_text = (
            REPO_ROOT / "scripts" / "start_navigation_prerequisites.sh"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "odometry_adapter_node = "
            "competition_avoidance.odometry_adapter_node:main",
            setup_text,
        )
        self.assertIn(
            "ros2 run competition_avoidance odometry_adapter_node",
            startup_text,
        )
        self.assertIn("wait_for_topic /odom", startup_text)
        self.assertNotIn("topic_tools relay", startup_text)

    def test_livox_release_rebuild_is_scoped_and_guarded(self) -> None:
        rebuild_text = (
            REPO_ROOT / "scripts" / "rebuild_livox_release.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("--packages-select livox_ros_driver2", rebuild_text)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", rebuild_text)
        self.assertIn("pgrep -f", rebuild_text)
        self.assertNotIn("rm -", rebuild_text)

    def test_livox_acceptance_keeps_strict_latency_limits(self) -> None:
        acceptance_text = (
            REPO_ROOT / "scripts" / "livox_latency_acceptance.py"
        ).read_text(encoding="utf-8")

        self.assertIn("default=120.0", acceptance_text)
        self.assertIn("default=0.30", acceptance_text)
        self.assertIn("default=0.50", acceptance_text)

    def test_prerequisite_frontend_starts_project_rviz_configuration(self) -> None:
        startup_text = (
            REPO_ROOT / "scripts" / "start_navigation_prerequisites.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("day5_motion_control.rviz", startup_text)
        self.assertIn("__node:=rviz2_day5_motion_control", startup_text)
        self.assertIn("wait_for_node /rviz2_day5_motion_control", startup_text)
        self.assertIn('start_base:=false', startup_text)
        self.assertIn('/cmd_vel absent', startup_text)


if __name__ == "__main__":
    unittest.main()
