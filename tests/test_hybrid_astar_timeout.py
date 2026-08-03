import pathlib
import sys
import unittest
from unittest import mock


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.hybrid_astar_planner import (  # noqa: E402
    HybridAStarPlanner,
    HybridAStarTimeout,
)
from competition_planning.occupancy_grid_planner import (  # noqa: E402
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint  # noqa: E402


def _empty_map() -> OccupancyGridMap:
    width = 80
    height = 80
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=0.10,
        origin_x=-2.0,
        origin_y=-4.0,
        occupied=tuple(False for _ in range(width * height)),
    )


class HybridAStarTimeoutTest(unittest.TestCase):
    def test_search_aborts_when_wall_clock_deadline_expires(self) -> None:
        planner = HybridAStarPlanner(
            _empty_map(),
            inflation_radius_m=0.04,
            search_padding_m=1.50,
            sample_spacing_m=0.10,
            min_turning_radius_m=0.60,
            step_length_m=0.20,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.15,
            goal_heading_tolerance_rad=0.14,
            max_expansions=250_000,
            planning_timeout_s=0.05,
        )

        clock = iter((10.00, 10.06))
        with mock.patch(
            "competition_planning.hybrid_astar_planner.time.monotonic",
            side_effect=lambda: next(clock),
        ):
            with self.assertRaisesRegex(
                HybridAStarTimeout,
                "exceeded 0.050 s",
            ):
                planner.plan(
                    (
                        PathPoint(0.0, 0.0, 0.0),
                        PathPoint(3.0, 0.0, 0.0),
                    )
                )


if __name__ == "__main__":
    unittest.main()
