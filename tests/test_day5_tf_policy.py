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

    def test_livox_host_time_policy_is_day5_specific(self) -> None:
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
            '"force_livox_host_timestamps", default_value="false"', day1_text
        )
        self.assertIn('"LIVOX_ROS_FORCE_HOST_TIMESTAMP", "1"', day1_text)
        self.assertIn('"force_livox_host_timestamps": "true"', day5_text)

        driver_patch = (
            REPO_ROOT / "patches" / "livox_ros_driver2_force_host_timestamp.patch"
        ).read_text(encoding="utf-8")
        self.assertIn('std::getenv("LIVOX_ROS_FORCE_HOST_TIMESTAMP")', driver_patch)
        self.assertIn(
            "!ForceHostTimestamp() && data->time_type != kTimestampTypeNoSync",
            driver_patch,
        )

    def test_livox_raw_packet_queue_is_bounded_for_vehicle_bringup(self) -> None:
        day1_text = (
            REPO_ROOT
            / "src"
            / "competition_bringup"
            / "launch"
            / "day1_mapping.launch.py"
        ).read_text(encoding="utf-8")
        driver_patch = (
            REPO_ROOT / "patches" / "livox_ros_driver2_bounded_packet_queue.patch"
        ).read_text(encoding="utf-8")

        self.assertIn(
            '"livox_publish_frequency_hz", default_value="10.0"', day1_text
        )
        self.assertIn(
            '"livox_raw_packet_queue_limit", default_value="256"', day1_text
        )
        self.assertIn('"publish_freq": float(', day1_text)
        self.assertIn('"LIVOX_ROS_MAX_PACKET_QUEUE"', day1_text)
        self.assertIn('std::getenv("LIVOX_ROS_MAX_PACKET_QUEUE")', driver_patch)
        self.assertIn("raw_packet_queue_.pop_front()", driver_patch)
        self.assertIn("raw_packet_queue_.push_back(std::move(packet))", driver_patch)


if __name__ == "__main__":
    unittest.main()
