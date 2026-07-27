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

    def test_sync_discards_waiting_lidar_frames_before_selecting_next(self) -> None:
        patch_text = (
            REPO_ROOT
            / "patches"
            / "fast_lio_latest_internal_buffer.patch"
        ).read_text(encoding="utf-8")

        discard_loop = "while (lidar_buffer.size() > 1)"
        select_latest = "meas.lidar = lidar_buffer.front();"
        self.assertIn(discard_loop, patch_text)
        self.assertIn("lidar_buffer.pop_front();", patch_text)
        self.assertIn("time_buffer.pop_front();", patch_text)
        self.assertLess(
            patch_text.index(discard_loop),
            patch_text.index(select_latest),
        )

    def test_day5_mapping_timer_preserves_fast_lio_executor_headroom(self) -> None:
        patch_text = (
            REPO_ROOT / "patches" / "fast_lio_mapping_timer_rate.patch"
        ).read_text(encoding="utf-8")
        config_text = (
            REPO_ROOT
            / "config"
            / "mapping"
            / "fast_lio_mid360_day5_control.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'declare_parameter<int>("mapping.timer_hz", 100);',
            patch_text,
        )
        self.assertIn(
            'get_parameter_or<int>("mapping.timer_hz", mapping_timer_hz, 100);',
            patch_text,
        )
        self.assertIn(
            "1000.0 / static_cast<double>(mapping_timer_hz)",
            patch_text,
        )
        self.assertIn("mapping_timer_hz must be positive", patch_text)
        self.assertRegex(config_text, r"(?m)^      timer_hz: 100$")

    def test_fast_lio_separates_sensor_and_mapping_callbacks(self) -> None:
        patch_text = (
            REPO_ROOT
            / "patches"
            / "fast_lio_executor_callback_groups.patch"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "rclcpp::executors::MultiThreadedExecutor executor(",
            patch_text,
        )
        self.assertIn("executor.add_node(node);", patch_text)
        self.assertIn("executor.spin();", patch_text)
        self.assertIn("lidar_callback_group_", patch_text)
        self.assertIn("imu_callback_group_", patch_text)
        self.assertIn("mapping_callback_group_", patch_text)
        self.assertIn("lidar_options.callback_group", patch_text)
        self.assertIn("imu_options.callback_group", patch_text)
        self.assertIn(
            "std::lock_guard<std::mutex> lock(mtx_buffer);",
            patch_text,
        )
        self.assertNotIn(
            "+    rclcpp::spin(std::make_shared<LaserMappingNode>());",
            patch_text,
        )

    def test_livox_preprocess_does_not_hold_the_sync_buffer_lock(self) -> None:
        patch_text = (
            REPO_ROOT
            / "patches"
            / "fast_lio_preprocess_lock_scope.patch"
        ).read_text(encoding="utf-8")

        preprocess = "+    p_pre->process(msg, ptr);"
        short_lock = "+        std::lock_guard<std::mutex> lock(mtx_buffer);"
        self.assertIn(preprocess, patch_text)
        self.assertIn(short_lock, patch_text)
        self.assertLess(
            patch_text.index(preprocess),
            patch_text.index(short_lock),
        )
        self.assertNotIn("+    mtx_buffer.lock();", patch_text)

    def test_rebuild_is_scoped_guarded_and_does_not_use_python_adapter(self) -> None:
        rebuild_text = (
            REPO_ROOT / "scripts" / "rebuild_fast_lio_release.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("--packages-select fast_lio", rebuild_text)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", rebuild_text)
        self.assertIn("fast_lio_latest_lidar_qos.patch", rebuild_text)
        self.assertIn("fast_lio_latest_internal_buffer.patch", rebuild_text)
        self.assertIn("fast_lio_mapping_timer_rate.patch", rebuild_text)
        self.assertIn("fast_lio_executor_callback_groups.patch", rebuild_text)
        self.assertIn("fast_lio_preprocess_lock_scope.patch", rebuild_text)
        self.assertIn("apply --reverse --check", rebuild_text)
        self.assertIn("apply --check", rebuild_text)
        self.assertIn("pgrep -f", rebuild_text)
        self.assertNotIn("livox_latest_frame_adapter_node", rebuild_text)
        self.assertNotIn("rm -", rebuild_text)


if __name__ == "__main__":
    unittest.main()
