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
        self.assertFalse(
            publish_config["scan_publish_en"],
            "Day5 must not build the unused world-frame cloud in the control loop.",
        )
        self.assertTrue(
            publish_config["scan_bodyframe_pub_en"],
            "The FAST-LIO patch keeps the body cloud available independently.",
        )
        self.assertIn("proximity_stop_node", launch_text)
        self.assertTrue(safety_config["safety"]["require_avoidance_source"])
        self.assertIn("proximity_stop", safety_config)
        self.assertGreaterEqual(
            proximity_config["stop_distance_m"],
            preprocess_config["blind"] + 0.25,
            "Proximity stop must look far enough beyond FAST-LIO's blind range.",
        )
        self.assertGreaterEqual(
            proximity_config["lateral_half_width_m"],
            0.45,
            "Proximity stop must cover the vehicle half-width plus clearance.",
        )
        self.assertEqual(
            proximity_config["cloud_qos_reliability"],
            "best_effort",
            "Dense sensor clouds must not backpressure FAST-LIO through reliable delivery.",
        )
        self.assertEqual(
            proximity_config["cloud_qos_depth"],
            1,
            "Proximity safety must process the newest cloud instead of queued old clouds.",
        )
        self.assertGreaterEqual(
            proximity_config["max_cloud_age_s"],
            1.5,
            "Day5 FAST-LIO body clouds have shown >1.4s header delay while still "
            "arriving live; proximity freshness must not false-stop that stream.",
        )
        self.assertEqual(
            proximity_config["costmap_topic"],
            "/avoidance/local_costmap",
        )
        self.assertEqual(proximity_config["scan_topic"], "/avoidance/scan")
        self.assertEqual(proximity_config["marker_topic"], "/avoidance/markers")
        self.assertGreater(proximity_config["visualization_rate_hz"], 0.0)
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
