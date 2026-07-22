import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5ObstacleSafetyTopologyTest(unittest.TestCase):
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
        self.assertTrue(
            publish_config["scan_publish_en"],
            "FAST-LIO scan_publish_en is the master switch for point cloud outputs.",
        )
        self.assertTrue(
            publish_config["scan_bodyframe_pub_en"],
            "Day5 motion must expose body-frame FAST-LIO points for obstacle stop.",
        )
        self.assertIn("proximity_stop_node", launch_text)
        self.assertTrue(safety_config["safety"]["require_avoidance_source"])
        self.assertIn("proximity_stop", safety_config)


if __name__ == "__main__":
    unittest.main()
