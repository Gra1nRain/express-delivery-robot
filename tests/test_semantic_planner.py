import copy
import pathlib
import sys
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

from competition_planning.semantic_planner import load_yaml_file, plan_route


class SemanticPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.route = load_yaml_file(REPO_ROOT / "config" / "routes" / "debug_route.yaml")
        self.semantic_map = load_yaml_file(
            REPO_ROOT / "maps" / "debug" / "semantic_map.yaml"
        )
        self.params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "planning_params.yaml"
        )

    def test_debug_route_generates_paths_for_plannable_steps(self) -> None:
        result = plan_route(self.route, self.semantic_map, self.params)

        self.assertTrue(result.ok, result.failures)
        self.assertEqual(
            [plan.step_id for plan in result.plans],
            [
                "go_traffic_light_1",
                "random_obstacle_1",
                "cone_lane_change_1",
                "return_to_pickup_area",
                "cone_lane_change_2",
                "finish_park",
            ],
        )
        self.assertEqual(result.frame_id, "map")
        for plan in result.plans:
            self.assertGreater(len(plan.path), 1)
            self.assertGreater(plan.path_length_m, 0.0)
            self.assertLess(plan.planning_time_ms, 500.0)

    def test_cone_lane_change_without_target_uses_corridor_end(self) -> None:
        result = plan_route(self.route, self.semantic_map, self.params)
        cone_plan = next(plan for plan in result.plans if plan.step_id == "cone_lane_change_1")

        self.assertEqual(cone_plan.target_ref, "drop_dock")
        self.assertEqual(cone_plan.target_source, "corridor_end")

    def test_rejects_step_not_allowed_by_corridor(self) -> None:
        semantic_map = copy.deepcopy(self.semantic_map)
        for corridor in semantic_map["route_corridors"]:
            if corridor["id"] == "go_to_pickup":
                corridor["allowed_steps"] = ["random_obstacle_1"]

        result = plan_route(self.route, semantic_map, self.params)

        self.assertFalse(result.ok)
        self.assertIn(
            ("go_traffic_light_1", "step_not_allowed_in_corridor"),
            [(failure.step_id, failure.reason) for failure in result.failures],
        )

    def test_rejects_unknown_target_ref(self) -> None:
        route = copy.deepcopy(self.route)
        route["steps"][1]["target_ref"] = "missing_point"

        result = plan_route(route, self.semantic_map, self.params)

        self.assertFalse(result.ok)
        self.assertIn(
            ("go_traffic_light_1", "route_ref_not_on_centerline"),
            [(failure.step_id, failure.reason) for failure in result.failures],
        )

    def test_rejects_corridor_too_narrow_for_vehicle_footprint(self) -> None:
        semantic_map = copy.deepcopy(self.semantic_map)
        for centerline in semantic_map["lane_centerlines"]:
            if centerline["id"] == "start_to_pickup_lane":
                centerline["width_m"] = 1.0

        result = plan_route(self.route, semantic_map, self.params)

        self.assertFalse(result.ok)
        self.assertIn(
            ("go_traffic_light_1", "corridor_too_narrow"),
            [(failure.step_id, failure.reason) for failure in result.failures],
        )

    def test_rejects_curvature_when_turning_radius_is_too_small(self) -> None:
        params = copy.deepcopy(self.params)
        params["global_planner"]["min_turning_radius_m"] = 1.0

        result = plan_route(self.route, self.semantic_map, params)

        self.assertFalse(result.ok)
        self.assertIn(
            ("cone_lane_change_1", "curvature_exceeded"),
            [(failure.step_id, failure.reason) for failure in result.failures],
        )


if __name__ == "__main__":
    unittest.main()
