import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mission_markers import (
    MissionMarkerTracker,
    mission_markers_from_route,
)


class MissionMarkerTrackerTest(unittest.TestCase):
    def test_marker_fires_once_near_its_checkpoint(self) -> None:
        markers = mission_markers_from_route(
            {
                "mission_markers": [
                    {
                        "id": "traffic_light_vision_on",
                        "before_checkpoint_ref": "traffic_light_stop_line",
                        "trigger_distance_m": 1.0,
                    }
                ]
            },
            checkpoint_refs=("traffic_light_stop_line", "pickup_front"),
        )
        tracker = MissionMarkerTracker(markers)

        self.assertEqual(
            tracker.update(
                active_checkpoint_ref="traffic_light_stop_line",
                distance_to_checkpoint_m=1.5,
            ),
            (),
        )
        self.assertEqual(
            tracker.update(
                active_checkpoint_ref="traffic_light_stop_line",
                distance_to_checkpoint_m=0.9,
            ),
            ("traffic_light_vision_on",),
        )
        self.assertEqual(
            tracker.update(
                active_checkpoint_ref="traffic_light_stop_line",
                distance_to_checkpoint_m=0.2,
            ),
            (),
        )

    def test_marker_does_not_fire_for_another_active_checkpoint(self) -> None:
        markers = mission_markers_from_route(
            {
                "mission_markers": [
                    {
                        "id": "traffic_light_vision_on",
                        "before_checkpoint_ref": "traffic_light_stop_line",
                        "trigger_distance_m": 1.0,
                    }
                ]
            },
            checkpoint_refs=("traffic_light_stop_line", "pickup_front"),
        )
        tracker = MissionMarkerTracker(markers)

        self.assertEqual(
            tracker.update(
                active_checkpoint_ref="pickup_front",
                distance_to_checkpoint_m=0.5,
            ),
            (),
        )

    def test_unknown_checkpoint_reference_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown checkpoint"):
            mission_markers_from_route(
                {
                    "mission_markers": [
                        {
                            "id": "traffic_light_vision_on",
                            "before_checkpoint_ref": "missing",
                            "trigger_distance_m": 1.0,
                        }
                    ]
                },
                checkpoint_refs=("traffic_light_stop_line",),
            )


if __name__ == "__main__":
    unittest.main()
