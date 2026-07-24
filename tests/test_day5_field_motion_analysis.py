import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "analyze_day5_field_motion.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("analyze_day5_field_motion", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_jsonl(
    path: pathlib.Path,
    samples: list[dict],
    *,
    stop_reason: str = "route_complete",
) -> None:
    lines = [
        json.dumps({"event": "script_start", "label": "unit_day5"}, ensure_ascii=False)
    ]
    lines.extend(json.dumps(sample, ensure_ascii=False) for sample in samples)
    lines.append(
        json.dumps(
            {
                "event": "stop_begin",
                "reason": stop_reason,
                "snapshot": samples[-1],
            },
            ensure_ascii=False,
        )
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _sample(index: int, *, heading_deg: float = 1.0) -> dict:
    return {
        "phase": "run",
        "elapsed_s": index * 0.05,
        "relay_active": True,
        "rx_age": {
            "body_cmd": 0.02,
            "cmd_vel_safe": 0.02,
            "odom": 0.02,
            "tracking": 0.02,
        },
        "body_cmd_x": 0.10,
        "safe_cmd_x": 0.10,
        "relay_cmd_x": 0.10,
        "odom_vx": 0.10,
        "odom_x": index * 0.02,
        "odom_y": 0.0,
        "lateral_error_m": 0.03,
        "heading_error_deg": heading_deg,
        "tracking_target_index": index,
        "control_status_value": "TRACKING",
    }


def _write_scenario_trajectory(path: pathlib.Path) -> None:
    points = []
    for index in range(60):
        if index < 20:
            curvature = 0.0
            speed = 0.20
            acceleration = 0.0
        elif index < 40:
            curvature = 0.40
            speed = 0.15
            acceleration = 0.0
        else:
            curvature = 0.0
            speed = 0.20 - (index - 39) * 0.006
            acceleration = -0.08
        points.append(
            {
                "x": index * 0.1,
                "y": 0.0,
                "yaw": 0.0,
                "curvature": curvature,
                "v": max(0.02, speed),
                "a": acceleration,
            }
        )
    path.write_text(
        "route_name: unit_scenario\npoints:\n"
        + "\n".join(
            [
                f"- x: {point['x']}\n"
                f"  y: {point['y']}\n"
                f"  yaw: {point['yaw']}\n"
                f"  curvature: {point['curvature']}\n"
                f"  v: {point['v']}\n"
                f"  a: {point['a']}"
                for point in points
            ]
        )
        + "\n",
        encoding="utf-8",
    )


class Day5FieldMotionAnalysisTest(unittest.TestCase):
    def test_accepts_stable_low_speed_tracking_log(self) -> None:
        module = _load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            jsonl_path = pathlib.Path(temp_dir) / "run.jsonl"
            _write_jsonl(jsonl_path, [_sample(index) for index in range(25)])

            report = module.analyze_jsonl(
                jsonl_path,
                module.AcceptanceLimits(min_samples=20, min_odom_distance_m=0.20),
            )

            self.assertTrue(report.passed, report.failed_checks)
            self.assertEqual(report.stop_reason, "route_complete")
            self.assertLessEqual(report.max_abs_lateral_error_m, 0.15)
            self.assertLessEqual(report.max_abs_heading_error_deg, 5.0)

    def test_rejects_heading_error_above_day5_acceptance_limit(self) -> None:
        module = _load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            jsonl_path = pathlib.Path(temp_dir) / "run.jsonl"
            _write_jsonl(jsonl_path, [_sample(index, heading_deg=6.2) for index in range(25)])

            report = module.analyze_jsonl(
                jsonl_path,
                module.AcceptanceLimits(min_samples=20, min_odom_distance_m=0.20),
            )

            self.assertFalse(report.passed)
            self.assertTrue(
                any("max_abs_heading_error" in item for item in report.failed_checks),
                report.failed_checks,
            )

    def test_reports_required_straight_turn_and_decel_scenarios(self) -> None:
        module = _load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            jsonl_path = temp_path / "run.jsonl"
            trajectory_path = temp_path / "trajectory.yaml"
            _write_jsonl(jsonl_path, [_sample(index) for index in range(60)])
            _write_scenario_trajectory(trajectory_path)

            report = module.analyze_jsonl(
                jsonl_path,
                module.AcceptanceLimits(
                    min_samples=20,
                    min_odom_distance_m=0.20,
                    required_scenarios=("straight", "turn", "decel"),
                ),
                trajectory_path=trajectory_path,
            )

            self.assertTrue(report.passed, report.failed_checks)
            self.assertGreaterEqual(report.scenario_metrics["straight"]["samples"], 5)
            self.assertGreaterEqual(report.scenario_metrics["turn"]["samples"], 5)
            self.assertGreaterEqual(report.scenario_metrics["decel"]["samples"], 5)

    def test_manual_index_range_can_accept_narrow_area_scenario(self) -> None:
        module = _load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            jsonl_path = pathlib.Path(temp_dir) / "run.jsonl"
            _write_jsonl(jsonl_path, [_sample(index) for index in range(30)])

            report = module.analyze_jsonl(
                jsonl_path,
                module.AcceptanceLimits(
                    min_samples=20,
                    min_odom_distance_m=0.20,
                    required_scenarios=("narrow",),
                ),
                scenario_ranges=("narrow:10:20",),
            )

            self.assertTrue(report.passed, report.failed_checks)
            self.assertEqual(report.scenario_metrics["narrow"]["samples"], 11)

    def test_missing_required_scenario_fails_acceptance(self) -> None:
        module = _load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            jsonl_path = pathlib.Path(temp_dir) / "run.jsonl"
            _write_jsonl(jsonl_path, [_sample(index) for index in range(30)])

            report = module.analyze_jsonl(
                jsonl_path,
                module.AcceptanceLimits(
                    min_samples=20,
                    min_odom_distance_m=0.20,
                    required_scenarios=("turn",),
                ),
            )

            self.assertFalse(report.passed)
            self.assertIn("missing_required_scenario=turn", report.failed_checks)

    def test_relay_monitor_records_body_command_topic(self) -> None:
        relay_text = (REPO_ROOT / "scripts" / "day5_full_route_relay.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"/control/body_cmd"', relay_text)
        self.assertIn("body_cmd_x", relay_text)
        self.assertIn("max_abs_body_cmd_x_mps", relay_text)

    def test_relay_can_preserve_manual_rviz_initialpose(self) -> None:
        relay_text = (REPO_ROOT / "scripts" / "day5_full_route_relay.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"--skip-initialpose"', relay_text)
        self.assertIn("initialpose_skipped", relay_text)
        self.assertIn("using_existing_map_tf", relay_text)
        self.assertIn("are required unless", relay_text)
        self.assertIn("--skip-initialpose is set", relay_text)

    def test_relay_can_apply_ranger_twist_semantics_after_safety(self) -> None:
        relay_text = (REPO_ROOT / "scripts" / "day5_full_route_relay.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"--adapt-ranger-twist"', relay_text)
        self.assertIn("adapt_yaw_rate_for_ranger_driver", relay_text)
        self.assertIn("command = self._relay_command(self.latest_safe_cmd)", relay_text)
        self.assertIn("self.cmd_pub.publish(command)", relay_text)


if __name__ == "__main__":
    unittest.main()
