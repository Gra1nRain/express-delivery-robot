import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.footprint_checker import (
    FootprintConfig,
    check_trajectory_footprint,
    poses_from_trajectory_artifact,
)
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import load_yaml_file, plan_continuous_route


class FootprintCheckerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.grid_map = OccupancyGridMap.from_yaml(REPO_ROOT / "maps" / "debug" / "map.yaml")
        self.config = FootprintConfig(
            vehicle_length_m=0.72,
            vehicle_width_m=0.50,
            clearance_m=0.20,
        )

    def test_existing_drop_dock_control_trajectory_is_too_close_to_static_obstacles(self) -> None:
        result = check_trajectory_footprint(
            poses_from_trajectory_artifact(
                REPO_ROOT / "docs" / "evidence" / "day5" / "debug_drop_dock_trajectory.yaml"
            ),
            self.grid_map,
            self.config,
        )

        self.assertFalse(result.ok)
        self.assertIsNotNone(result.first_violation)

    def test_control_validation_route_uses_pass_points_not_precision_docks(self) -> None:
        route = load_yaml_file(
            REPO_ROOT / "config" / "routes" / "debug_control_validation_route.yaml"
        )
        semantic_map = load_yaml_file(
            REPO_ROOT / "maps" / "debug" / "semantic_map_control_validation.yaml"
        )
        params = load_yaml_file(REPO_ROOT / "config" / "planning" / "planning_params.yaml")
        params["global_planner"]["planning_timeout_ms"] = 30_000.0

        plan = plan_continuous_route(route, semantic_map, params)

        self.assertTrue(plan.ok, plan.failures)
        refs = [point.ref_id for point in plan.path if point.ref_id]
        self.assertIn("pickup_pass", refs)
        self.assertIn("drop_pass", refs)
        self.assertNotIn("pickup_dock", refs)
        self.assertNotIn("drop_dock", refs)

    def test_control_validation_trajectory_passes_padded_footprint_check(self) -> None:
        result = check_trajectory_footprint(
            poses_from_trajectory_artifact(
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_control_validation_trajectory.yaml"
            ),
            self.grid_map,
            self.config,
        )

        self.assertTrue(result.ok, result.to_dict())


if __name__ == "__main__":
    unittest.main()
