import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.engine import AvoidanceEngine
from competition_avoidance.perception import ObstacleDetection
from competition_avoidance.risk import EgoState
from competition_avoidance.tracker import TrackerConfig


def _detection(x: float, y: float, classification: str) -> ObstacleDetection:
    return ObstacleDetection(
        x=x,
        y=y,
        z=0.5,
        length_m=0.40,
        width_m=0.40,
        height_m=0.60 if classification == "CONE_CANDIDATE" else 1.70,
        point_count=30,
        classification=classification,
        confidence=0.7,
    )


class AvoidanceEngineTest(unittest.TestCase):
    def test_static_cone_requests_replanning_without_dynamic_stop(self) -> None:
        engine = AvoidanceEngine(
            tracker_config=TrackerConfig(
                static_confirmation_count=2,
                minimum_confirmed_hits=2,
            )
        )
        ego = EgoState(0.0, 0.0, 0.15, 0.0)

        engine.update(
            (_detection(2.0, 0.0, "CONE_CANDIDATE"),),
            timestamp_s=1.0,
            ego=ego,
            proximity_stop=False,
        )
        decision = engine.update(
            (_detection(2.0, 0.0, "CONE_CANDIDATE"),),
            timestamp_s=1.2,
            ego=ego,
            proximity_stop=False,
        )

        self.assertEqual(decision.mode, "STATIC_REPLAN")
        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.static_track_count, 1)

    def test_dynamic_track_becoming_cone_immediately_returns_to_static_replan(
        self,
    ) -> None:
        engine = AvoidanceEngine(
            tracker_config=TrackerConfig(
                association_gate_m=1.0,
                minimum_confirmed_hits=2,
                moving_speed_mps=0.35,
                static_speed_mps=0.18,
                moving_confirmation_count=2,
                static_confirmation_count=3,
            )
        )
        ego = EgoState(0.0, 0.0, 0.0, 0.0)

        for timestamp_s, x in ((1.0, 0.40), (1.1, 0.50), (1.2, 0.60)):
            decision = engine.update(
                (_detection(x, 0.0, "PERSON_CANDIDATE"),),
                timestamp_s=timestamp_s,
                ego=ego,
                proximity_stop=False,
            )
        self.assertEqual(decision.mode, "DYNAMIC_STOP")

        decision = engine.update(
            (_detection(0.70, 0.0, "CONE_CANDIDATE"),),
            timestamp_s=1.3,
            ego=ego,
            proximity_stop=False,
        )

        self.assertEqual(decision.mode, "STATIC_REPLAN")
        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.dynamic_track_count, 0)
        self.assertEqual(decision.static_track_count, 1)

    def test_low_confidence_unknown_motion_uses_static_replanning(
        self,
    ) -> None:
        engine = AvoidanceEngine(
            tracker_config=TrackerConfig(
                association_gate_m=1.0,
                minimum_confirmed_hits=2,
                moving_speed_mps=0.35,
                static_speed_mps=0.18,
                moving_confirmation_count=2,
                static_confirmation_count=3,
            )
        )
        ego = EgoState(0.0, 0.0, 0.0, 0.0)

        decisions = tuple(
            engine.update(
                (_detection(x, 0.0, "UNKNOWN"),),
                timestamp_s=2.0 + 0.1 * index,
                ego=ego,
                proximity_stop=False,
            )
            for index, x in enumerate((0.50, 0.60, 0.70, 0.80))
        )

        self.assertNotIn(
            "DYNAMIC_STOP",
            {decision.mode for decision in decisions},
        )
        self.assertEqual(decisions[-1].mode, "STATIC_REPLAN")
        self.assertEqual(decisions[-1].dynamic_track_count, 0)
        self.assertEqual(decisions[-1].static_track_count, 1)

    def test_proximity_stop_overrides_other_decisions(self) -> None:
        engine = AvoidanceEngine()

        decision = engine.update(
            (),
            timestamp_s=2.0,
            ego=EgoState(0.0, 0.0, 0.0, 0.0),
            proximity_stop=True,
        )

        self.assertEqual(decision.mode, "EMERGENCY_HOLD")
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.reason, "proximity_stop")


if __name__ == "__main__":
    unittest.main()
