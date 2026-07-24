import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))

from competition_avoidance.risk import (
    EgoState,
    RiskConfig,
    evaluate_dynamic_risk,
    stopping_distance_m,
)
from competition_avoidance.tracker import TrackedObstacle


def _dynamic_person() -> TrackedObstacle:
    return TrackedObstacle(
        track_id=7,
        x=1.0,
        y=-1.0,
        vx_mps=0.0,
        vy_mps=0.8,
        radius_m=0.30,
        classification="PERSON_CANDIDATE",
        confidence=0.8,
        motion_state="DYNAMIC",
        confirmed=True,
        age_s=1.0,
        last_seen_s=5.0,
    )


class AvoidanceRiskTest(unittest.TestCase):
    def test_crossing_person_requests_stop_before_emergency_distance(self) -> None:
        assessment = evaluate_dynamic_risk(
            (_dynamic_person(),),
            EgoState(x=0.0, y=0.0, vx_mps=0.15, vy_mps=0.0),
            RiskConfig(),
        )

        self.assertEqual(assessment.level, "STOP")
        self.assertEqual(assessment.track_id, 7)
        self.assertGreater(assessment.current_distance_m, 0.85)
        self.assertLess(assessment.time_to_cpa_s, 2.0)
        self.assertLess(assessment.distance_at_cpa_m, 1.0)

    def test_stopping_distance_uses_global_conservative_limits(self) -> None:
        distance = stopping_distance_m(
            speed_mps=0.20,
            reaction_time_s=0.35,
            max_deceleration_mps2=0.30,
            safety_margin_m=0.40,
        )

        self.assertAlmostEqual(distance, 0.5366666667, places=6)


if __name__ == "__main__":
    unittest.main()
