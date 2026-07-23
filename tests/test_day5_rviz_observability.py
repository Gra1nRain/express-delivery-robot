import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5RvizObservabilityTest(unittest.TestCase):
    def test_day5_exposes_planned_actual_obstacle_and_sensor_views(self) -> None:
        control_config = yaml.safe_load(
            (REPO_ROOT / "config" / "control" / "control_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        safety_config = yaml.safe_load(
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
        rviz_path = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "rviz"
            / "day5_motion_control.rviz"
        )
        rviz_text = rviz_path.read_text(encoding="utf-8")

        visualization = control_config["visualization"]
        proximity = safety_config["proximity_stop"]
        expected_topics = {
            visualization["reference_path_topic"],
            visualization["local_trajectory_topic"],
            visualization["executed_path_topic"],
            proximity["costmap_topic"],
            proximity["scan_topic"],
            proximity["marker_topic"],
            proximity["cloud_topic"],
            "/map",
        }

        self.assertIn('DeclareLaunchArgument("rviz"', launch_text)
        self.assertIn("day5_motion_control.rviz", launch_text)
        self.assertIn("nav2_map_server", launch_text)
        for topic in expected_topics:
            self.assertIn(topic, rviz_text)
        self.assertIn("Reliability Policy: Best Effort", rviz_text)
        rviz_config = yaml.safe_load(rviz_text)
        displays = rviz_config["Visualization Manager"]["Displays"]
        body_cloud = next(display for display in displays if display["Name"] == "Body Cloud")
        self.assertFalse(body_cloud["Enabled"])
        self.assertFalse(body_cloud["Value"])


if __name__ == "__main__":
    unittest.main()
