import math
import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5ObstacleSafetyTopologyTest(unittest.TestCase):
    def test_tracking_error_uses_recovery_band_before_persistent_hold(self) -> None:
        safety = yaml.safe_load(
            (REPO_ROOT / "config" / "safety" / "safety_params.yaml").read_text(
                encoding="utf-8"
            )
        )["safety"]

        self.assertLess(
            safety["recovery_clear_lateral_error_m"],
            safety["recovery_lateral_error_m"],
        )
        self.assertLess(
            safety["recovery_lateral_error_m"],
            safety["max_lateral_error_m"],
        )
        self.assertLess(
            safety["recovery_clear_heading_error_deg"],
            safety["recovery_heading_error_deg"],
        )
        self.assertLess(
            safety["recovery_heading_error_deg"],
            safety["max_heading_error_deg"],
        )
        self.assertGreater(safety["recovery_speed_mps"], 0.0)
        self.assertGreater(safety["tracking_error_timeout_s"], 0.0)

    def test_day5_motion_launch_has_a_live_obstacle_stop_source(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")
        fast_lio_config = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "mapping"
                / "fast_lio_mid360_day5_control.yaml"
            ).read_text(encoding="utf-8")
        )
        safety_config = yaml.safe_load(
            (REPO_ROOT / "config" / "safety" / "safety_params.yaml").read_text(
                encoding="utf-8"
            )
        )

        publish_config = fast_lio_config["/**"]["ros__parameters"]["publish"]
        preprocess_config = fast_lio_config["/**"]["ros__parameters"]["preprocess"]
        proximity_config = safety_config["proximity_stop"]
        projection_config = safety_config["pointcloud_to_laserscan"]
        self.assertEqual(
            fast_lio_config["/**"]["ros__parameters"]["point_filter_num"],
            10,
        )
        self.assertLessEqual(
            preprocess_config["blind"],
            0.20,
            "Lowered Day5 lidar must retain near-field obstacle returns.",
        )
        self.assertTrue(
            publish_config["scan_publish_en"],
            "FAST-LIO scan_publish_en is the master switch for point cloud outputs.",
        )
        self.assertTrue(
            publish_config["scan_bodyframe_pub_en"],
            "Day5 motion must expose body-frame FAST-LIO points for obstacle stop.",
        )
        self.assertTrue(
            publish_config["dense_publish_en"],
            "Day5 obstacle projection needs the dense de-skewed body cloud.",
        )
        self.assertIn("proximity_stop_node", launch_text)
        self.assertIn('package="pointcloud_to_laserscan"', launch_text)
        self.assertIn('executable="pointcloud_to_laserscan_node"', launch_text)
        self.assertTrue(safety_config["safety"]["require_avoidance_source"])
        self.assertIn("proximity_stop", safety_config)
        self.assertEqual(proximity_config["input_type"], "laser_scan")
        self.assertEqual(proximity_config["input_scan_topic"], "/scan")
        self.assertEqual(
            projection_config["input_topic"],
            "/cloud_registered_body",
        )
        self.assertEqual(projection_config["output_topic"], "/scan")
        self.assertEqual(projection_config["target_frame"], "body")
        self.assertAlmostEqual(
            projection_config["angle_min"],
            -math.pi / 2.0,
        )
        self.assertAlmostEqual(
            projection_config["angle_max"],
            math.pi / 2.0,
        )
        self.assertGreaterEqual(
            proximity_config["stop_distance_m"],
            preprocess_config["blind"] + 0.25,
            "Proximity stop must look far enough beyond FAST-LIO's blind range.",
        )
        self.assertAlmostEqual(
            proximity_config["stop_distance_m"],
            preprocess_config["blind"]
            + proximity_config["vehicle_width_m"] / 2.0,
        )
        self.assertGreaterEqual(
            proximity_config["lateral_half_width_m"],
            0.45,
            "Proximity stop must cover the vehicle half-width plus clearance.",
        )
        self.assertEqual(proximity_config["scan_qos_reliability"], "best_effort")
        self.assertEqual(proximity_config["scan_qos_depth"], 1)
        self.assertLessEqual(proximity_config["max_scan_age_s"], 0.25)
        self.assertEqual(
            proximity_config["costmap_topic"],
            "/avoidance/local_costmap",
        )
        self.assertEqual(proximity_config["scan_topic"], "/avoidance/scan")
        self.assertEqual(proximity_config["marker_topic"], "/avoidance/markers")
        self.assertEqual(proximity_config["visualization_rate_hz"], 5.0)
        self.assertEqual(proximity_config["fusion_frame_count"], 10)
        self.assertIn(
            '"fusion_frame_count": proximity_stop["fusion_frame_count"]',
            launch_text,
        )
        self.assertGreaterEqual(
            proximity_config["grid_x_max_m"],
            proximity_config["stop_distance_m"],
        )
        self.assertLessEqual(
            proximity_config["grid_y_min_m"],
            -proximity_config["lateral_half_width_m"],
        )
        self.assertGreaterEqual(
            proximity_config["grid_y_max_m"],
            proximity_config["lateral_half_width_m"],
        )


if __name__ == "__main__":
    unittest.main()
