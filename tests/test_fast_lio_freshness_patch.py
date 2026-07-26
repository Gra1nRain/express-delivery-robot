import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class FastLioFreshnessPatchTest(unittest.TestCase):
    def test_livox_subscription_keeps_only_the_latest_sensor_sample(self) -> None:
        patch_text = (
            REPO_ROOT / "patches" / "fast_lio_latest_lidar_qos.patch"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "auto lidar_qos = rclcpp::SensorDataQoS();",
            patch_text,
        )
        self.assertIn("lidar_qos.keep_last(1);", patch_text)
        self.assertIn(
            "create_subscription<livox_ros_driver2::msg::CustomMsg>("
            "lid_topic, lidar_qos, livox_pcl_cbk)",
            patch_text,
        )
        self.assertIn(
            "-            sub_pcl_livox_ = this->"
            "create_subscription<livox_ros_driver2::msg::CustomMsg>"
            "(lid_topic, 20, livox_pcl_cbk);",
            patch_text,
        )
        self.assertNotIn(
            "+            sub_pcl_livox_ = this->"
            "create_subscription<livox_ros_driver2::msg::CustomMsg>"
            "(lid_topic, 20, livox_pcl_cbk);",
            patch_text,
        )

    def test_rebuild_is_scoped_guarded_and_does_not_use_python_adapter(self) -> None:
        rebuild_text = (
            REPO_ROOT / "scripts" / "rebuild_fast_lio_release.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("--packages-select fast_lio", rebuild_text)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", rebuild_text)
        self.assertIn("fast_lio_latest_lidar_qos.patch", rebuild_text)
        self.assertIn("apply --reverse --check", rebuild_text)
        self.assertIn("apply --check", rebuild_text)
        self.assertIn("pgrep -f", rebuild_text)
        self.assertNotIn("livox_latest_frame_adapter_node", rebuild_text)
        self.assertNotIn("rm -", rebuild_text)


if __name__ == "__main__":
    unittest.main()
