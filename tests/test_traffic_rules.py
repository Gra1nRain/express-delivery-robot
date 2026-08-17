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
    def test_horizontal_red_motion_triggers(self) -> None:
        detector = FlagWaveDetector(WaveConfig(min_displacement_px=30.0))
        triggered = [
            detector.update(
                centroid_x=x,
                centroid_y=120.0,
                timestamp_s=index * 0.10,
            )
            for index, x in enumerate([100, 100, 101, 130, 150, 170])
        ]

        self.assertEqual(sum(triggered), 1)

    def test_upward_red_motion_triggers(self) -> None:
        detector = FlagWaveDetector(WaveConfig(min_displacement_px=30.0))
        triggered = [
            detector.update(
                centroid_x=100.0,
                centroid_y=y,
                timestamp_s=index * 0.10,
            )
            for index, y in enumerate([180, 180, 179, 150, 120, 90])
        ]

        self.assertEqual(sum(triggered), 1)

    def test_static_red_region_does_not_trigger(self) -> None:
        detector = FlagWaveDetector()

        triggered = [
            detector.update(
                centroid_x=100.0 + index % 3,
                centroid_y=120.0 + index % 2,
                timestamp_s=index * 0.10,
            )
            for index in range(30)
        ]

        self.assertFalse(any(triggered))

    def test_wave_survives_brief_motion_blur_gap(self) -> None:
        detector = FlagWaveDetector()
        triggered = []

        for index, x in enumerate([100.0, 100.0, 100.0]):
            triggered.append(
                detector.update(
                    centroid_x=x,
                    centroid_y=100.0,
                    timestamp_s=index * 0.10,
                )
            )
        for index in range(3, 11):
            triggered.append(
                detector.update(
                    centroid_x=None,
                    centroid_y=None,
                    timestamp_s=index * 0.10,
                )
            )
        for index, x in enumerate([110.0, 140.0, 170.0], start=11):
            triggered.append(
                detector.update(
                    centroid_x=x,
                    centroid_y=100.0,
                    timestamp_s=index * 0.10,
                )
            )

        self.assertEqual(sum(triggered), 1)


class TrafficRuleControllerTest(unittest.TestCase):
    def test_flag_starts_then_red_stops_and_green_resumes(self) -> None:
        rules = TrafficRuleController(confirm_frames=2)
        self.assertTrue(rules.decision.stop_required)

        started = rules.observe_flag_wave()
        self.assertFalse(started.stop_required)

        pending = rules.set_traffic_active(True)
        self.assertTrue(pending.stop_required)
        self.assertEqual(pending.reason, "traffic_pending")

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
        rules.set_traffic_active(True)

        decision = rules.observe_light("green")

        self.assertTrue(decision.stop_required)
        self.assertFalse(decision.started)
        self.assertEqual(decision.reason, "waiting_for_flag")

    def test_single_frame_false_positive_is_ignored(self) -> None:
        rules = TrafficRuleController(confirm_frames=3)
        rules.observe_flag_wave()
        rules.set_traffic_active(True)

        rules.observe_light("red")
        decision = rules.observe_light(None)

        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.light, LightState.UNKNOWN)
        self.assertEqual(decision.reason, "traffic_pending")

    def test_yellow_and_off_are_fail_safe_stops(self) -> None:
        for observation in ("yellow", "off"):
            with self.subTest(observation=observation):
                rules = TrafficRuleController(confirm_frames=1)
                rules.observe_flag_wave()
                rules.set_traffic_active(True)
                decision = rules.observe_light(observation)
                self.assertTrue(decision.stop_required)

    def test_disabled_traffic_recognition_cannot_change_start_decision(self) -> None:
        rules = TrafficRuleController(confirm_frames=1)
        rules.observe_flag_wave()

        ignored = rules.observe_light("red")

        self.assertFalse(ignored.stop_required)
        self.assertEqual(ignored.light, LightState.UNKNOWN)

    def test_disabling_traffic_check_releases_a_started_vehicle(self) -> None:
        rules = TrafficRuleController(confirm_frames=1)
        rules.observe_flag_wave()
        rules.set_traffic_active(True)
        rules.observe_light("red")

        disabled = rules.set_traffic_active(False)

        self.assertFalse(disabled.stop_required)
        self.assertEqual(disabled.light, LightState.UNKNOWN)
        self.assertEqual(disabled.reason, "traffic_inactive")


if __name__ == "__main__":
    unittest.main()
