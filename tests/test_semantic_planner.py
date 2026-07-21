import copy
import pathlib
import sys
import tempfile
import textwrap
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
        self.params["global_planner"]["plugin"] = "semantic_corridor"

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

    def test_debug_route_uses_configured_occupancy_grid_astar(self) -> None:
        params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "planning_params.yaml"
        )

        result = plan_route(self.route, self.semantic_map, params)

        self.assertTrue(result.ok, result.failures)
        self.assertEqual(
            {plan.planner_plugin for plan in result.plans},
            {"occupancy_grid_astar"},
        )
        self.assertEqual(len(result.plans), 6)

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

    def test_occupancy_grid_astar_detours_around_blocked_cells(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = pathlib.Path(tmp_dir)
            map_path = tmp_path / "test_map.yaml"
            pgm_path = tmp_path / "test_map.pgm"
            pixels = [254] * 100
            for y in range(9):
                row = 9 - y
                pixels[row * 10 + 4] = 0
            pgm_path.write_bytes(
                b"P5\n10 10\n255\n" + bytes(pixels)
            )
            map_path.write_text(
                textwrap.dedent(
                    f"""\
                    image: {pgm_path.name}
                    mode: trinary
                    resolution: 1.0
                    origin: [0.0, 0.0, 0.0]
                    negate: 0
                    occupied_thresh: 0.65
                    free_thresh: 0.196
                    """
                ),
                encoding="utf-8",
            )
            route = {
                "route_name": "grid_test",
                "steps": [
                    {
                        "id": "cross_wall",
                        "type": "RUN_SEGMENT",
                        "corridor_ref": "test_corridor",
                        "target_ref": "goal",
                    }
                ],
            }
            semantic_map = {
                "frame_id": "map",
                "points": {
                    "start": {"x": 1.5, "y": 1.5, "yaw": 0.0},
                    "goal": {"x": 8.5, "y": 1.5, "yaw": 0.0},
                },
                "effective_area": {
                    "vertices": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
                },
                "lane_centerlines": [
                    {
                        "id": "test_centerline",
                        "width_m": 20.0,
                        "points": ["start", "goal"],
                    }
                ],
                "route_corridors": [
                    {
                        "id": "test_corridor",
                        "centerline_ref": "test_centerline",
                        "allowed_steps": ["cross_wall"],
                    }
                ],
            }
            params = {
                "global_planner": {
                    "plugin": "occupancy_grid_astar",
                    "map_file": str(map_path),
                    "grid_inflation_radius_m": 0.0,
                    "grid_search_padding_m": 10.0,
                    "path_sample_spacing_m": 1.0,
                    "min_turning_radius_m": 0.0,
                }
            }

            result = plan_route(route, semantic_map, params)

        self.assertTrue(result.ok, result.failures)
        path = result.plans[0].path
        self.assertGreater(len(path), 2)
        self.assertGreater(max(point.y for point in path), 8.0)

    def test_occupancy_grid_astar_preserves_semantic_waypoints(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = pathlib.Path(tmp_dir)
            map_path = tmp_path / "test_map.yaml"
            pgm_path = tmp_path / "test_map.pgm"
            pgm_path.write_bytes(b"P5\n10 10\n255\n" + bytes([254] * 100))
            map_path.write_text(
                textwrap.dedent(
                    f"""\
                    image: {pgm_path.name}
                    mode: trinary
                    resolution: 1.0
                    origin: [0.0, 0.0, 0.0]
                    negate: 0
                    occupied_thresh: 0.65
                    free_thresh: 0.196
                    """
                ),
                encoding="utf-8",
            )
            route = {
                "route_name": "grid_waypoint_test",
                "steps": [
                    {
                        "id": "visit_midpoint",
                        "type": "RUN_SEGMENT",
                        "corridor_ref": "test_corridor",
                        "target_ref": "goal",
                    }
                ],
            }
            semantic_map = {
                "frame_id": "map",
                "points": {
                    "start": {"x": 1.5, "y": 1.5, "yaw": 0.0},
                    "midpoint": {"x": 5.5, "y": 5.5, "yaw": 0.0},
                    "goal": {"x": 8.5, "y": 1.5, "yaw": 0.0},
                },
                "effective_area": {
                    "vertices": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
                },
                "lane_centerlines": [
                    {
                        "id": "test_centerline",
                        "width_m": 20.0,
                        "points": ["start", "midpoint", "goal"],
                    }
                ],
                "route_corridors": [
                    {
                        "id": "test_corridor",
                        "centerline_ref": "test_centerline",
                        "allowed_steps": ["visit_midpoint"],
                    }
                ],
            }
            params = {
                "global_planner": {
                    "plugin": "occupancy_grid_astar",
                    "map_file": str(map_path),
                    "grid_inflation_radius_m": 0.0,
                    "grid_search_padding_m": 10.0,
                    "path_sample_spacing_m": 1.0,
                    "min_turning_radius_m": 0.0,
                }
            }

            result = plan_route(route, semantic_map, params)

        self.assertTrue(result.ok, result.failures)
        self.assertIn("midpoint", [point.ref_id for point in result.plans[0].path])


if __name__ == "__main__":
    unittest.main()
