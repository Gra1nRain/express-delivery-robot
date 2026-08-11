import math
import pathlib
import sys
import unittest
from dataclasses import dataclass


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.local_plan_continuity import (
    local_paths_are_equivalent,
    nearest_path_point_index,
)


@dataclass(frozen=True)
class _Point:
    x: float
    y: float
    yaw: float = 0.0
    ref_id: str | None = None


class LocalPlanContinuityTest(unittest.TestCase):
    def test_mppi_node_reuses_equivalent_plans_but_refreshes_freshness(self) -> None:
        source = (
            REPO_ROOT
            / "src"
            / "competition_control"
            / "competition_control"
            / "mppi_control_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn("local_paths_are_equivalent(", source)
        self.assertIn("self._accepted_local_geometry = geometry", source)
        self.assertIn('self._local_plan_update_mode = "reused"', source)
        self.assertIn('self._local_plan_update_mode = "replaced"', source)
        reuse_block = source.split(
            "if self._accepted_local_geometry is not None", 1
        )[1].split("parameterized = parameterize_local_path", 1)[0]
        self.assertIn("self._latest_local_plan_stamp_s", reuse_block)
        self.assertNotIn("replace_trajectory", reuse_block)

    def test_reuses_same_future_geometry_with_jittered_current_pose(self) -> None:
        previous = (
            _Point(0.00, 0.00),
            _Point(0.30, 0.05, 0.10),
            _Point(0.60, 0.12, 0.15),
        )
        current = (
            _Point(0.02, -0.01, 0.01),
            _Point(0.32, 0.04, 0.11),
            _Point(0.61, 0.11, 0.16),
        )

        self.assertTrue(local_paths_are_equivalent(previous, current))

    def test_reuses_a_consumed_suffix_without_cumulative_drift(self) -> None:
        previous = (
            _Point(0.00, 0.00),
            _Point(0.25, 0.00),
            _Point(0.50, 0.05),
            _Point(0.75, 0.10),
        )
        current = (
            _Point(0.24, 0.01),
            _Point(0.51, 0.04),
            _Point(0.74, 0.11),
        )

        self.assertTrue(local_paths_are_equivalent(previous, current))

    def test_replaces_a_real_lateral_route_change(self) -> None:
        previous = (
            _Point(0.00, 0.00),
            _Point(0.30, 0.00),
            _Point(0.60, 0.00),
        )
        current = (
            _Point(0.02, 0.00),
            _Point(0.30, 0.12),
            _Point(0.60, 0.18),
        )

        self.assertFalse(local_paths_are_equivalent(previous, current))

    def test_replaces_a_heading_change(self) -> None:
        previous = (
            _Point(0.00, 0.00),
            _Point(0.30, 0.00, 0.0),
            _Point(0.60, 0.00, 0.0),
        )
        current = (
            _Point(0.02, 0.00),
            _Point(0.30, 0.00, math.radians(9.0)),
            _Point(0.60, 0.00, math.radians(9.0)),
        )

        self.assertFalse(local_paths_are_equivalent(previous, current))

    def test_replaces_geometry_when_checkpoint_annotation_changes(self) -> None:
        previous = (
            _Point(0.0, 0.0),
            _Point(0.5, 0.0),
            _Point(1.0, 0.0),
        )
        current = (
            _Point(0.0, 0.0),
            _Point(0.5, 0.0),
            _Point(1.0, 0.0, ref_id="traffic_light_stop_line"),
        )

        self.assertFalse(local_paths_are_equivalent(previous, current))

    def test_does_not_reuse_a_two_point_plan(self) -> None:
        previous = (_Point(0.00, 0.00), _Point(0.30, 0.00))
        current = (_Point(0.01, 0.00), _Point(0.30, 0.00))

        self.assertFalse(local_paths_are_equivalent(previous, current))

    def test_finds_checkpoint_on_local_bypass_path(self) -> None:
        path = (
            _Point(0.0, 0.0),
            _Point(0.5, 0.02),
            _Point(1.0, 0.08),
            _Point(1.5, 0.20),
        )

        self.assertEqual(
            nearest_path_point_index(
                path,
                _Point(1.0, 0.0),
                max_distance_m=0.10,
            ),
            2,
        )

    def test_rejects_checkpoint_not_reached_by_local_path(self) -> None:
        path = (_Point(0.0, 0.0), _Point(0.5, 0.0), _Point(1.0, 0.4))

        self.assertIsNone(
            nearest_path_point_index(
                path,
                _Point(1.0, 0.0),
                max_distance_m=0.10,
            )
        )


if __name__ == "__main__":
    unittest.main()
