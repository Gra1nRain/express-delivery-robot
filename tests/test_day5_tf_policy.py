import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5TfPolicyTest(unittest.TestCase):
    def test_day1_keeps_ranger_odom_tf_configurable(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day1_mapping.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("publish_odom_tf")', launch_text)
        self.assertIn('"publish_odom_tf": publish_odom_tf', launch_text)
        self.assertIn(
            'DeclareLaunchArgument("publish_odom_tf", default_value="true")',
            launch_text,
        )

    def test_day5_disables_only_the_disconnected_ranger_tf_tree(self) -> None:
        launch_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"publish_odom_tf": "false"', launch_text)
        self.assertIn('"start_base": LaunchConfiguration("start_base")', launch_text)
        self.assertIn('"anchor_odom_frame": "camera_init"', launch_text)

    def test_livox_time_rebasing_is_day5_specific(self) -> None:
        day1_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day1_mapping.launch.py"
        ).read_text(encoding="utf-8")
        day5_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day5_motion_control.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            '"rebase_livox_timestamps", default_value="false"', day1_text
        )
        self.assertIn('dst="/livox/lidar_raw"', day1_text)
        self.assertIn('dst="/livox/imu_raw"', day1_text)
        self.assertIn('"rebase_livox_timestamps": "true"', day5_text)


if __name__ == "__main__":
    unittest.main()
