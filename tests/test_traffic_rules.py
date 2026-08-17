import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_perception"))

from competition_perception.traffic_rules import (
    FlagWaveDetector,
    LightState,
    TrafficRuleController,
    WaveConfig,
)


class FlagWaveDetectorTest(unittest.TestCase):
    def test_fast_downward_wave_triggers_once(self) -> None:
        detector = FlagWaveDetector(
            WaveConfig(
                min_downward_displacement_px=20.0,
                direct_min_speed_pxps=40.0,
                min_total_travel_px=80.0,
            )
        )
        triggered = [
            detector.update(centroid_y=y, timestamp_s=index * 0.10)
            for index, y in enumerate([100, 90, 80, 70, 60, 90, 120, 150, 180])
        ]

        self.assertEqual(sum(triggered), 1)

    def test_static_red_region_does_not_trigger(self) -> None:
        detector = FlagWaveDetector()

        triggered = [
            detector.update(centroid_y=120.0, timestamp_s=index * 0.10)
            for index in range(30)
        ]

        self.assertFalse(any(triggered))


class TrafficRuleControllerTest(unittest.TestCase):
    def test_flag_starts_then_red_stops_and_green_resumes(self) -> None:
        rules = TrafficRuleController(confirm_frames=2)
        self.assertTrue(rules.decision.stop_required)

        started = rules.observe_flag_wave()
        self.assertFalse(started.stop_required)

        rules.observe_light("red")
        stopped = rules.observe_light("red")
        self.assertTrue(stopped.stop_required)
        self.assertEqual(stopped.light, LightState.RED)

        rules.observe_light("green")
        resumed = rules.observe_light("green")
        self.assertFalse(resumed.stop_required)
        self.assertEqual(resumed.light, LightState.GREEN)

    def test_green_light_cannot_bypass_start_flag(self) -> None:
        rules = TrafficRuleController(confirm_frames=1)

        decision = rules.observe_light("green")

        self.assertTrue(decision.stop_required)
        self.assertFalse(decision.started)
        self.assertEqual(decision.reason, "waiting_for_flag")

    def test_single_frame_false_positive_is_ignored(self) -> None:
        rules = TrafficRuleController(confirm_frames=3)
        rules.observe_flag_wave()

        rules.observe_light("red")
        decision = rules.observe_light(None)

        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.light, LightState.UNKNOWN)

    def test_yellow_and_off_are_fail_safe_stops(self) -> None:
        for observation in ("yellow", "off"):
            with self.subTest(observation=observation):
                rules = TrafficRuleController(confirm_frames=1)
                rules.observe_flag_wave()
                decision = rules.observe_light(observation)
                self.assertTrue(decision.stop_required)


if __name__ == "__main__":
    unittest.main()
