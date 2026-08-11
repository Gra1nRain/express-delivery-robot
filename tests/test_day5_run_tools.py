import importlib.util
import pathlib
import sys
import tempfile
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
POLICY_PATH = REPO_ROOT / "scripts" / "day5_run_policy.py"


def _load_policy_module():
    spec = importlib.util.spec_from_file_location("day5_run_policy", POLICY_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Day5RunPolicyTest(unittest.TestCase):
    def test_loads_duration_and_finish_from_trajectory(self) -> None:
        module = _load_policy_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = pathlib.Path(temp_dir) / "trajectory.yaml"
            path.write_text(
                "duration_s: 217.482\n"
                "points:\n"
                "- {x: 0.0, y: 0.0}\n"
                "- {x: 1.5, y: -2.0}\n",
                encoding="utf-8",
            )

            metadata = module.load_route_metadata(path)

            self.assertEqual(metadata.point_count, 2)
            self.assertEqual(metadata.finish_xy, (1.5, -2.0))
            self.assertEqual(metadata.duration_s, 217.482)

    def test_default_watchdog_scales_with_planned_duration(self) -> None:
        module = _load_policy_module()

        timeout_s = module.resolve_watchdog_timeout_s(None, 217.482)

        self.assertAlmostEqual(timeout_s, 603.705)

    def test_explicit_watchdog_override_is_preserved(self) -> None:
        module = _load_policy_module()

        timeout_s = module.resolve_watchdog_timeout_s(720.0, 217.482)

        self.assertEqual(timeout_s, 720.0)

    def test_missing_duration_uses_legacy_fallback(self) -> None:
        module = _load_policy_module()

        timeout_s = module.resolve_watchdog_timeout_s(None, None)

        self.assertEqual(timeout_s, 420.0)


class Day5RunScriptTest(unittest.TestCase):
    def test_environment_loader_does_not_enable_interactive_shell_exit(self) -> None:
        text = (REPO_ROOT / "scripts" / "car_source_env.sh").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("set -eo pipefail", text)
        self.assertIn("_car_source_env_restore_nounset", text)

    def test_manual_adapter_command_preserves_terminal_on_exit(self) -> None:
        text = (REPO_ROOT / "docs" / "day5_manual_test.md").read_text(
            encoding="utf-8"
        )
        terminal_7 = text.split("## 终端7：", 1)[1].split("## 停车方法", 1)[0]

        self.assertIn("set +e", terminal_7)
        self.assertIn("adapter_rc=$?", terminal_7)
        self.assertIn("终端7保持打开", terminal_7)

    def test_sequential_bringup_gates_livox_before_fast_lio_and_cloud(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_sequential_bringup.sh"
        ).read_text(encoding="utf-8")

        sensor_start = text.index(
            "setsid ros2 launch competition_bringup day1_mapping.launch.py"
        )
        livox_gate = text.index("--mode livox")
        fast_lio_start = text.index("ros2 launch fast_lio mapping.launch.py")
        first_cloud_gate = text.index("--mode cloud")
        navigation_start = text.index(
            "setsid ros2 launch competition_bringup day5_motion_control.launch.py"
        )
        second_cloud_gate = text.index("--mode cloud", first_cloud_gate + 1)

        self.assertLess(sensor_start, livox_gate)
        self.assertLess(livox_gate, fast_lio_start)
        self.assertLess(fast_lio_start, first_cloud_gate)
        self.assertLess(first_cloud_gate, navigation_start)
        self.assertLess(navigation_start, second_cloud_gate)
        self.assertEqual(text.count("--mode cloud"), 2)
        self.assertIn("start_livox:=false", text)
        self.assertIn("start_fast_lio:=false", text)
        self.assertIn("No chassis relay was enabled", text)

    def test_sequential_bringup_owns_and_terminates_launch_process_groups(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_sequential_bringup.sh"
        ).read_text(encoding="utf-8")

        self.assertEqual(text.count("setsid ros2 launch"), 3)
        self.assertIn('kill -TERM -- "-$process_group_pid"', text)
        self.assertIn('kill -KILL -- "-$process_group_pid"', text)

    def test_freshness_gate_uses_latest_only_qos(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_sensor_freshness_gate.py"
        ).read_text(encoding="utf-8")

        self.assertIn("history=HistoryPolicy.KEEP_LAST", text)
        self.assertIn("depth=1", text)
        self.assertIn("message.points[-1].offset_time", text)

    def test_day5_bag_topics_include_tf_static_with_transient_qos(self) -> None:
        topics = {
            line.strip()
            for line in (
                REPO_ROOT / "config" / "day5" / "bag_topics.txt"
            ).read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        qos = yaml.safe_load(
            (
                REPO_ROOT / "config" / "day5" / "bag_qos_overrides.yaml"
            ).read_text(encoding="utf-8")
        )
        record_text = (
            REPO_ROOT / "scripts" / "day5_record_motion.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("/tf_static", topics)
        self.assertEqual(qos["/tf_static"]["durability"], "transient_local")
        self.assertIn("--qos-profile-overrides-path", record_text)

    def test_relay_uses_adaptive_watchdog_policy(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_full_route_relay.py"
        ).read_text(encoding="utf-8")

        self.assertIn("resolve_watchdog_timeout_s(", text)
        self.assertIn("default=None", text)
        self.assertIn("elapsed_s >= watchdog_timeout_s", text)

    def test_relay_accepts_dwa_ready_states(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_full_route_relay.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"DWA_TRACKING"', text)
        self.assertIn('"DWA_AVOIDING"', text)

    def test_relay_explicitly_arms_segmented_route_before_ready_gate(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_full_route_relay.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"--enable-segmented-route"', text)
        arm_index = text.index("node.publish_route_enable(True)")
        ready_gate_index = text.index("ready_start = time.time()")
        self.assertLess(arm_index, ready_gate_index)
        self.assertIn("node.publish_route_enable(False)", text)

    def test_relay_debounces_temporary_proximity_stop(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_full_route_relay.py"
        ).read_text(encoding="utf-8")

        self.assertIn("proximity_blocked = bool(", text)
        self.assertIn('"proximity_stop",', text)

    def test_relay_debounces_temporary_safe_command_staleness(self) -> None:
        text = (
            REPO_ROOT / "scripts" / "day5_full_route_relay.py"
        ).read_text(encoding="utf-8")

        self.assertIn("cmd_vel_safe_stale = (", text)
        self.assertIn('"cmd_vel_safe_stale",', text)

    def test_disabled_scan_stop_ignores_single_near_scan_return(self) -> None:
        module = _load_policy_module()

        self.assertIsNone(module.scan_stop_reason(0.2163, 0.0))
        self.assertEqual(
            module.scan_stop_reason(0.2163, 0.34),
            "scan_min_under_0.34m",
        )


if __name__ == "__main__":
    unittest.main()
