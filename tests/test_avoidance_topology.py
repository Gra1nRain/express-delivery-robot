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
        self.assertEqual(config["maximum_processing_frequency_hz"], 10.0)
        self.assertEqual(config["association_gate_m"], 0.35)
        self.assertEqual(config["moving_speed_mps"], 0.35)
        self.assertEqual(config["static_speed_mps"], 0.18)
        self.assertEqual(config["moving_confirmation_count"], 3)
        self.assertFalse(config["allow_unknown_dynamic"])
        self.assertEqual(config["maximum_unknown_dynamic_radius_m"], 0.80)
        self.assertEqual(config["proximity_stop_distance_m"], 1.20)
        self.assertEqual(config["proximity_min_points"], 2)
        self.assertEqual(config["planning_obstacle_memory_ttl_s"], 1.50)
        self.assertEqual(config["planning_obstacle_memory_resolution_m"], 0.10)
        self.assertGreaterEqual(config["dynamic_safety_margin_m"], 0.40)

    def test_new_launch_is_incremental_and_keeps_motion_topics_out(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "launch"
            / "vehicle_avoidance_bringup.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('executable="avoidance_manager_node"', launch_text)
        self.assertIn(
            'executable="pointcloud_to_laserscan_node"',
            launch_text,
        )
        self.assertIn('executable="odometry_adapter_node"', launch_text)
        self.assertIn('("cloud_in", "/cloud_registered_body")', launch_text)
        self.assertIn('("scan", "/avoidance/scan")', launch_text)
        self.assertNotIn("IncludeLaunchDescription", launch_text)
        self.assertNotIn("day5_motion_control.launch.py", launch_text)
        self.assertNotIn('"/cmd_vel"', launch_text)
        self.assertNotIn('"/cmd_vel_safe"', launch_text)

    def test_scan_converter_dependency_is_explicit(self) -> None:
        package_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "package.xml"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "<exec_depend>pointcloud_to_laserscan</exec_depend>",
            package_text,
        )

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
            "/avoidance/scan",
        ):
            self.assertIn(topic, node_text)
        self.assertNotIn('"/cmd_vel"', node_text)
        self.assertNotIn('"/planning/local_trajectory"', node_text)

    def test_scan_mode_subscribes_to_live_scan_without_republishing_it(
        self,
    ) -> None:
        node_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "competition_avoidance"
            / "avoidance_manager_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('self._perception_input_type == "scan"', node_text)
        self.assertIn(
            "self.create_subscription(\n"
            "                LaserScan,\n"
            "                self._scan_input_topic,\n"
            "                self._scan_callback,",
            node_text,
        )
        self.assertIn("self._scan_publisher = None", node_text)
        self.assertIn("if self._scan_publisher is not None:", node_text)

    def test_avoidance_profile_freezes_odometry_semantics(self) -> None:
        config = yaml.safe_load(
            (
                REPO_ROOT / "config" / "avoidance" / "avoidance_params.yaml"
            ).read_text(encoding="utf-8")
        )["avoidance_manager"]["ros__parameters"]

        self.assertEqual(config["odometry_topic"], "/odom")
        self.assertEqual(config["expected_odometry_frame"], "camera_init")
        self.assertEqual(config["perception_input_type"], "scan")
        self.assertEqual(config["scan_input_topic"], "/avoidance/scan")

    def test_scan_source_profile_keeps_canonical_publishers_unique(self) -> None:
        safety_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "safety"
                / "safety_params_day5_avoidance_scan.yaml"
            ).read_text(encoding="utf-8")
        )
        avoidance_config = yaml.safe_load(
            (
                REPO_ROOT / "config" / "avoidance" / "avoidance_params.yaml"
            ).read_text(encoding="utf-8")
        )["avoidance_manager"]["ros__parameters"]
        proximity = safety_config["proximity_stop"]

        self.assertEqual(proximity["scan_topic"], "/avoidance/scan")
        self.assertEqual(
            proximity["stop_request_topic"],
            "/avoidance/proximity_stop_request",
        )
        self.assertEqual(
            proximity["costmap_topic"],
            "/avoidance/proximity_costmap",
        )
        self.assertEqual(
            safety_config["safety"]["avoidance_stop_topic"],
            "/avoidance/stop_request",
        )
        self.assertEqual(
            avoidance_config["stop_request_topic"],
            "/avoidance/stop_request",
        )
        self.assertEqual(
            avoidance_config["local_costmap_topic"],
            "/avoidance/local_costmap",
        )

    def test_scan_source_profile_only_remaps_proximity_outputs(self) -> None:
        base_config = yaml.safe_load(
            (
                REPO_ROOT / "config" / "safety" / "safety_params.yaml"
            ).read_text(encoding="utf-8")
        )
        scan_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "safety"
                / "safety_params_day5_avoidance_scan.yaml"
            ).read_text(encoding="utf-8")
        )
        expected_proximity = dict(base_config["proximity_stop"])
        expected_proximity["stop_request_topic"] = (
            "/avoidance/proximity_stop_request"
        )
        expected_proximity["costmap_topic"] = "/avoidance/proximity_costmap"

        self.assertEqual(scan_config["safety"], base_config["safety"])
        self.assertEqual(scan_config["proximity_stop"], expected_proximity)

    def test_odometry_adapter_bridges_fast_lio_without_touching_navigation(self) -> None:
        setup_text = (
            REPO_ROOT / "src" / "competition_avoidance" / "setup.py"
        ).read_text(encoding="utf-8")
        node_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "competition_avoidance"
            / "odometry_adapter_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "odometry_adapter_node = "
            "competition_avoidance.odometry_adapter_node:main",
            setup_text,
        )
        self.assertIn('"/Odometry"', node_text)
        self.assertIn('"/odom"', node_text)
        self.assertNotIn('"/cmd_vel"', node_text)

    def test_livox_latest_frame_adapter_is_additive_and_motion_free(self) -> None:
        setup_text = (
            REPO_ROOT / "src" / "competition_avoidance" / "setup.py"
        ).read_text(encoding="utf-8")
        package_text = (
            REPO_ROOT / "src" / "competition_avoidance" / "package.xml"
        ).read_text(encoding="utf-8")
        adapter_text = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "competition_avoidance"
            / "livox_latest_frame_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "livox_latest_frame_adapter_node = "
            "competition_avoidance.livox_latest_frame_adapter_node:main",
            setup_text,
        )
        self.assertIn("<exec_depend>livox_ros_driver2</exec_depend>", package_text)
        self.assertIn('"/livox/lidar"', adapter_text)
        self.assertIn('"/avoidance/livox_latest"', adapter_text)
        self.assertIn("raw=True", adapter_text)
        self.assertIn(
            "self._publisher.publish(serialized_message)",
            adapter_text,
        )
        self.assertNotIn('"/cmd_vel"', adapter_text)

    def test_latest_frame_fast_lio_profile_does_not_replace_raw_profile(self) -> None:
        raw_config = yaml.safe_load(
            (
                REPO_ROOT / "config" / "mapping" / "fast_lio_mid360_day1.yaml"
            ).read_text(encoding="utf-8")
        )["/**"]["ros__parameters"]
        adapter_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "mapping"
                / "fast_lio_mid360_avoidance_latest.yaml"
            ).read_text(encoding="utf-8")
        )["/**"]["ros__parameters"]
        self.assertEqual(raw_config["common"]["lid_topic"], "/livox/lidar")
        self.assertEqual(
            adapter_config["common"]["lid_topic"],
            "/avoidance/livox_latest",
        )
        self.assertEqual(adapter_config["preprocess"]["scan_rate"], 10)

    def test_avoidance_nodes_do_not_shutdown_ros_context_twice(self) -> None:
        node_dir = (
            REPO_ROOT
            / "src"
            / "competition_avoidance"
            / "competition_avoidance"
        )
        for filename in (
            "avoidance_manager_node.py",
            "livox_latest_frame_adapter_node.py",
            "odometry_adapter_node.py",
        ):
            node_text = (node_dir / filename).read_text(encoding="utf-8")
            self.assertIn(
                "if rclpy.ok():",
                node_text,
                msg=filename,
            )

    def test_livox_release_rebuild_is_scoped_and_guarded(self) -> None:
        rebuild_text = (
            REPO_ROOT / "scripts" / "rebuild_livox_release.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("--packages-select livox_ros_driver2", rebuild_text)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", rebuild_text)
        self.assertIn("pgrep -f", rebuild_text)
        self.assertIn("livox_ros_driver2_bounded_packet_queue.patch", rebuild_text)
        self.assertIn("livox_ros_driver2_sensor_qos.patch", rebuild_text)
        self.assertIn("apply --check", rebuild_text)
        self.assertIn("apply --check --ignore-space-change", rebuild_text)
        self.assertNotIn("rm -", rebuild_text)

        qos_patch = (
            REPO_ROOT / "patches" / "livox_ros_driver2_sensor_qos.patch"
        ).read_text(encoding="utf-8")
        self.assertIn("rclcpp::SensorDataQoS()", qos_patch)
        self.assertIn("qos.keep_last(1)", qos_patch)
        self.assertIn(
            "create_publisher<CustomMsg>(topic_name, qos)",
            qos_patch,
        )

    def test_livox_acceptance_keeps_strict_latency_limits(self) -> None:
        acceptance_text = (
            REPO_ROOT / "scripts" / "livox_latency_acceptance.py"
        ).read_text(encoding="utf-8")

        self.assertIn("default=120.0", acceptance_text)
        self.assertIn("default=0.30", acceptance_text)
        self.assertIn("default=0.50", acceptance_text)

    def test_localization_rviz_shows_map_and_body_cloud_without_avoidance(self) -> None:
        rviz_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "rviz"
            / "day5_localization.rviz"
        ).read_text(encoding="utf-8")

        self.assertIn("Name: Map", rviz_text)
        self.assertIn("Value: /map", rviz_text)
        self.assertIn("Name: Body Cloud", rviz_text)
        self.assertIn("Value: /cloud_registered_body", rviz_text)
        self.assertIn("Fixed Frame: map", rviz_text)
        self.assertIn("Value: /initialpose", rviz_text)
        self.assertNotIn("/avoidance/", rviz_text)


if __name__ == "__main__":
    unittest.main()
