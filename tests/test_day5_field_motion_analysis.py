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


def _write_jsonl(path: pathlib.Path, samples: list[dict]) -> None:
    lines = [
        json.dumps({"event": "script_start", "label": "unit_day5"}, ensure_ascii=False)
    ]
    lines.extend(json.dumps(sample, ensure_ascii=False) for sample in samples)
    lines.append(
        json.dumps(
            {
                "event": "stop_begin",
                "reason": "route_complete",
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
        "control_status_value": "TRACKING",
    }


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

    def test_relay_monitor_records_body_command_topic(self) -> None:
        relay_text = (REPO_ROOT / "scripts" / "day5_full_route_relay.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"/control/body_cmd"', relay_text)
        self.assertIn("body_cmd_x", relay_text)
        self.assertIn("max_abs_body_cmd_x_mps", relay_text)


if __name__ == "__main__":
    unittest.main()
