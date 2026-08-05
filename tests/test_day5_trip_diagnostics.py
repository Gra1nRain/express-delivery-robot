import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Day5TripDiagnosticsTest(unittest.TestCase):
    def test_diagnostics_is_passive_and_does_not_store_sensor_frames(self) -> None:
        script = REPO_ROOT / "scripts" / "day5_trip_diagnostics.py"
        source = script.read_text(encoding="utf-8")

        self.assertNotIn("create_publisher", source)
        self.assertNotIn("message.ranges", source)
        self.assertNotIn("list(message.data)", source)
        self.assertIn("/tmp/day5_trip_trace.jsonl", source)
        self.assertIn("/planning/local_replan_status", source)
        self.assertIn("/planning/local_trajectory", source)
        self.assertIn("/control/status", source)
        self.assertIn("/control/body_cmd", source)
        self.assertIn("/control/tracking_error", source)
        self.assertIn("/control/state_valid", source)
        self.assertIn("/cmd_vel_safe", source)
        self.assertIn("/cmd_vel", source)
        self.assertIn("/avoidance/local_costmap", source)
        self.assertIn("/avoidance/proximity_status", source)
        self.assertIn("/scan", source)
        self.assertIn("/odom", source)
        self.assertIn("if rclpy.ok():", source)


if __name__ == "__main__":
    unittest.main()
