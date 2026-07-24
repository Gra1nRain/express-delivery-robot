import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.perception import ObstacleDetection
from competition_avoidance.tracker import ObstacleTracker, TrackerConfig


def _person(x: float, y: float) -> ObstacleDetection:
    return ObstacleDetection(
        x=x,
        y=y,
        z=0.9,
        length_m=0.45,
        width_m=0.45,
        height_m=1.70,
        point_count=40,
        classification="PERSON_CANDIDATE",
        confidence=0.75,
    )


class AvoidanceTrackerTest(unittest.TestCase):
    def test_crossing_person_keeps_id_and_becomes_dynamic(self) -> None:
        tracker = ObstacleTracker(
            TrackerConfig(
                association_gate_m=0.80,
                minimum_confirmed_hits=2,
                moving_confirmation_count=2,
            )
        )

        first = tracker.update((_person(2.0, -0.60),), timestamp_s=10.0)
        second = tracker.update((_person(2.0, -0.30),), timestamp_s=10.5)
        third = tracker.update((_person(2.0, 0.00),), timestamp_s=11.0)

        self.assertEqual(first[0].track_id, second[0].track_id)
        self.assertEqual(second[0].track_id, third[0].track_id)
        self.assertTrue(third[0].confirmed)
        self.assertEqual(third[0].motion_state, "DYNAMIC")
        self.assertGreater(third[0].vy_mps, 0.20)

    def test_stale_track_expires_and_time_cannot_move_backwards(self) -> None:
        tracker = ObstacleTracker(TrackerConfig(track_timeout_s=0.50))
        tracker.update((_person(1.0, 0.0),), timestamp_s=1.0)

        self.assertEqual(tracker.update((), timestamp_s=1.6), ())
        with self.assertRaisesRegex(ValueError, "timestamps must increase"):
            tracker.update((), timestamp_s=1.5)


if __name__ == "__main__":
    unittest.main()
