import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_avoidance"))
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_avoidance.adaptive_local_replanner import (
    AdaptiveLocalTrajectoryPlanner,
)
from competition_planning.local_trajectory_planner import (
    LocalPlan,
    LocalReplanConfig,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


class _FakePlanner:
    def __init__(self, lookahead: float, outcomes: dict[float, object]) -> None:
        self._lookahead = lookahead
        self._outcomes = outcomes

    def plan(self, **_: object) -> LocalPlan:
        outcome = self._outcomes[self._lookahead]
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


class AdaptiveLocalReplannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.grid = OccupancyGridMap(
            width=2,
            height=2,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            occupied=(False,) * 4,
        )
        self.reference = (PathPoint(0.0, 0.0, 0.0), PathPoint(1.0, 0.0, 0.0))
        self.pose = self.reference[0]
        self.result = LocalPlan(
            path=self.reference,
            reference_start_index=0,
            rejoin_index=1,
            dynamic_obstacle_count=0,
            status="REFERENCE_CLEAR",
            path_is_navigable=True,
            planning_grid_cell_count=4,
        )

    def _planner(self, outcomes: dict[float, object]) -> AdaptiveLocalTrajectoryPlanner:
        return AdaptiveLocalTrajectoryPlanner(
            self.grid,
            LocalReplanConfig(lookahead_distance_m=1.5),
            (3.5,),
            planner_factory=lambda _, config: _FakePlanner(
                config.lookahead_distance_m,
                outcomes,
            ),
        )

    def test_primary_success_does_not_use_fallback(self) -> None:
        planner = self._planner(
            {
                1.5: self.result,
                3.5: AssertionError("fallback should not run"),
            }
        )

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.pose,
            dynamic_obstacle_points=(),
        )

        self.assertIs(result, self.result)
        self.assertEqual(planner.last_selected_lookahead_distance_m, 1.5)

    def test_fallback_is_used_after_primary_has_no_path(self) -> None:
        planner = self._planner(
            {
                1.5: GridPlanningError("short horizon has no path"),
                3.5: self.result,
            }
        )

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.pose,
            dynamic_obstacle_points=((0.5, 0.0),),
        )

        self.assertIs(result, self.result)
        self.assertEqual(planner.last_selected_lookahead_distance_m, 3.5)

    def test_all_failures_are_reported(self) -> None:
        planner = self._planner(
            {
                1.5: GridPlanningError("short"),
                3.5: GridPlanningError("long"),
            }
        )

        with self.assertRaisesRegex(
            GridPlanningError,
            r"1\.50m: short.*3\.50m: long",
        ):
            planner.plan(
                reference_path=self.reference,
                current_pose=self.pose,
                dynamic_obstacle_points=(),
            )

        self.assertIsNone(planner.last_selected_lookahead_distance_m)

    def test_field_profile_holds_each_plan_for_two_seconds(self) -> None:
        profile = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "avoidance"
                / "adaptive_local_replanner_params.yaml"
            ).read_text(encoding="utf-8")
        )["local_replanner"]["ros__parameters"]

        self.assertEqual(profile["frequency_hz"], 0.5)
        self.assertEqual(profile["lookahead_distance_m"], 1.5)
        self.assertEqual(profile["fallback_lookahead_distance_m"], 3.5)


if __name__ == "__main__":
    unittest.main()
