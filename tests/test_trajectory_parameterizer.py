import math
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))

PROFILE_8_14_2_SEMANTIC_MAP = (
    REPO_ROOT / "maps" / "debug" / "semantic_map_8_14_2.yaml"
)
PROFILE_8_14_2_ROUTE = (
    REPO_ROOT / "config" / "routes" / "debug_indoor_one_lap_route_8_14_2.yaml"
)
PROFILE_8_14_2_PLANNING = (
    REPO_ROOT / "config" / "planning" / "planning_params_8_14_2.yaml"
)

from competition_planning.semantic_planner import PathPoint, StepPlan, load_yaml_file
from competition_planning.trajectory_parameterizer import (
    optimize_route_trajectory,
    optimize_continuous_route_trajectory,
    parameterize_step_plan,
    retime_continuous_trajectory,
)


class TrajectoryParameterizerTest(unittest.TestCase):
    def test_debug_precision_docks_are_shifted_outward_by_27_cm(self) -> None:
        semantic_map = load_yaml_file(PROFILE_8_14_2_SEMANTIC_MAP)
        original_points = {
            "pickup_front": (8.4130, -0.0810),
            "pickup_rear": (9.0130, -0.0910),
            "drop_front": (2.2660, 3.4560),
            "drop_rear": (1.6850, 3.4670),
        }
        groups = {
            "pickup": tuple(
                ref for ref in original_points if ref.startswith("pickup_")
            ),
            "drop": tuple(
                ref for ref in original_points if ref.startswith("drop_")
            ),
        }

        for prefix, refs in groups.items():
            old_front = original_points[f"{prefix}_front"]
            old_rear = original_points[f"{prefix}_rear"]
            shelf_yaw = math.atan2(
                old_rear[1] - old_front[1], old_rear[0] - old_front[0]
            )
            tangent = (math.cos(shelf_yaw), math.sin(shelf_yaw))
            outward = (-tangent[1], tangent[0])
            for ref in refs:
                actual = semantic_map["points"][ref]
                delta = (
                    float(actual["x"]) - original_points[ref][0],
                    float(actual["y"]) - original_points[ref][1],
                )
                self.assertAlmostEqual(
                    delta[0] * tangent[0] + delta[1] * tangent[1],
                    0.0,
                    places=3,
                    msg=ref,
                )
                self.assertAlmostEqual(
                    delta[0] * outward[0] + delta[1] * outward[1],
                    0.27,
                    places=3,
                    msg=ref,
                )

    def test_debug_alignment_anchors_follow_shifted_dock_poses(self) -> None:
        semantic_map = load_yaml_file(PROFILE_8_14_2_SEMANTIC_MAP)
        for anchor_ref, dock_ref, distance_m in (
            ("pickup_approach_align", "pickup_front", 1.00),
            ("pickup_front_terminal_align", "pickup_front", 0.50),
            ("drop_approach_align", "drop_front", 1.00),
            ("drop_front_terminal_align", "drop_front", 0.50),
        ):
            anchor = semantic_map["points"][anchor_ref]
            dock = semantic_map["points"][dock_ref]
            yaw = float(dock["yaw"])
            self.assertAlmostEqual(
                float(anchor["x"]),
                float(dock["x"]) - distance_m * math.cos(yaw),
                places=3,
                msg=anchor_ref,
            )
            self.assertAlmostEqual(
                float(anchor["y"]),
                float(dock["y"]) - distance_m * math.sin(yaw),
                places=3,
                msg=anchor_ref,
            )
            self.assertAlmostEqual(float(anchor["yaw"]), yaw, msg=anchor_ref)

    def test_pickup_to_drop_turn_is_one_soft_global_planning_interval(self) -> None:
        route = load_yaml_file(PROFILE_8_14_2_ROUTE)
        semantic_map = load_yaml_file(PROFILE_8_14_2_SEMANTIC_MAP)
        steps = {step["id"]: step for step in route["steps"]}

        self.assertNotIn("pickup_departure_wide", steps)
        turn = steps["drop_transit_1"]
        self.assertFalse(turn["docking_approach"])
        self.assertTrue(turn["precision_goal_connection"])
        self.assertTrue(turn["soft_intermediate_refs"])
        self.assertEqual(turn["corridor_ref"], "pickup_to_drop_aligned")
        self.assertEqual(turn["target_ref"], "drop_front")
        self.assertNotIn("drop_final_approach", steps)
        pickup_rear = semantic_map["points"]["pickup_rear"]
        shelf_clear = semantic_map["points"]["pickup_shelf_clear_terminal_align"]
        rear_yaw = float(pickup_rear["yaw"])
        delta_x = float(shelf_clear["x"]) - float(pickup_rear["x"])
        delta_y = float(shelf_clear["y"]) - float(pickup_rear["y"])
        self.assertAlmostEqual(
            delta_x * math.cos(rear_yaw) + delta_y * math.sin(rear_yaw),
            0.60,
            places=3,
        )
        self.assertAlmostEqual(
            -delta_x * math.sin(rear_yaw) + delta_y * math.cos(rear_yaw),
            0.0,
            places=3,
        )
        self.assertAlmostEqual(float(shelf_clear["yaw"]), rear_yaw)
        turn_exit = semantic_map["points"]["pickup_turn_exit_guide"]
        self.assertAlmostEqual(
            float(turn_exit["x"]),
            float(shelf_clear["x"]) - 2.0 * 0.81 * math.sin(rear_yaw),
            places=3,
        )
        self.assertAlmostEqual(
            float(turn_exit["y"]),
            float(shelf_clear["y"]) + 2.0 * 0.81 * math.cos(rear_yaw),
            places=3,
        )
        centerlines = {
            item["id"]: item for item in semantic_map["lane_centerlines"]
        }
        corridors = {
            item["id"]: item for item in semantic_map["route_corridors"]
        }
        self.assertEqual(
            centerlines[corridors["pickup_to_drop_aligned"]["centerline_ref"]][
                "points"
            ],
            [
                "pickup_rear",
                "pickup_shelf_clear_terminal_align",
                "pickup_turn_exit_guide",
                "drop_approach_align",
                "drop_front_terminal_align",
                "drop_front",
            ],
        )

    def test_indoor_one_lap_is_one_continuous_trajectory(self) -> None:
        route = load_yaml_file(PROFILE_8_14_2_ROUTE)
        semantic_map = load_yaml_file(PROFILE_8_14_2_SEMANTIC_MAP)
        planning = load_yaml_file(PROFILE_8_14_2_PLANNING)
        optimizer = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
        )

        result = optimize_continuous_route_trajectory(
            route,
            semantic_map,
            planning,
            optimizer,
        )

        self.assertTrue(result.ok, result.failures)
        self.assertGreaterEqual(max(point.v for point in result.points), 0.070)
        refs = [point.ref_id for point in result.points if point.ref_id]
        for ref in (
            "traffic_light_stop_line",
            "random_obstacle_entry",
            "random_obstacle_exit",
            "pickup_approach_align",
            "pickup_front_terminal_align",
            "pickup_front",
            "pickup_rear_terminal_align",
            "pickup_rear",
            "pickup_shelf_clear_terminal_align",
            "pickup_turn_exit_guide",
            "drop_approach_align",
            "drop_front_terminal_align",
            "drop_front",
            "drop_rear_terminal_align",
            "drop_rear",
            "finish_park",
        ):
            self.assertIn(ref, refs)
        points_by_ref = {
            point.ref_id: point for point in result.points if point.ref_id
        }
        for ref in ("pickup_front", "pickup_rear", "drop_front", "drop_rear"):
            expected = semantic_map["points"][ref]
            self.assertAlmostEqual(points_by_ref[ref].x, float(expected["x"]))
            self.assertAlmostEqual(points_by_ref[ref].y, float(expected["y"]))
            terminal_ref = f"{ref}_terminal_align"
            terminal = points_by_ref[terminal_ref]
            terminal_expected = semantic_map["points"][terminal_ref]
            self.assertLessEqual(
                math.hypot(
                    terminal.x - float(terminal_expected["x"]),
                    terminal.y - float(terminal_expected["y"]),
                ),
                0.03,
            )

            terminal_index = next(
                index
                for index, point in enumerate(result.points)
                if point.ref_id == terminal_ref
            )
            dock_index = next(
                index
                for index, point in enumerate(result.points)
                if point.ref_id == ref and index > terminal_index
            )
            self.assertLess(terminal_index, dock_index)
            self.assertAlmostEqual(
                math.hypot(
                    float(expected["x"]) - float(terminal_expected["x"]),
                    float(expected["y"]) - float(terminal_expected["y"]),
                ),
                0.50 if ref.endswith("_front") else 0.30,
                places=3,
            )

            if ref.endswith("_front"):
                terminal_heading = float(expected["yaw"])
            else:
                prefix = ref.split("_", maxsplit=1)[0]
                shelf_front = semantic_map["points"][f"{prefix}_front"]
                shelf_rear = semantic_map["points"][f"{prefix}_rear"]
                terminal_heading = math.atan2(
                    float(shelf_rear["y"]) - float(shelf_front["y"]),
                    float(shelf_rear["x"]) - float(shelf_front["x"]),
                )
            terminal_tail = result.points[terminal_index : dock_index + 1]
            tangent_errors_deg = []
            for first, second in zip(terminal_tail, terminal_tail[1:]):
                tangent = math.atan2(second.y - first.y, second.x - first.x)
                tangent_errors_deg.append(
                    abs(
                        math.degrees(
                            (tangent - terminal_heading + math.pi)
                            % (2.0 * math.pi)
                            - math.pi
                        )
                    )
                )
            self.assertTrue(tangent_errors_deg, ref)
            self.assertLessEqual(max(tangent_errors_deg), 1.0, ref)
            first, second, third = terminal_tail[-3:]
            ab = math.hypot(second.x - first.x, second.y - first.y)
            bc = math.hypot(third.x - second.x, third.y - second.y)
            ca = math.hypot(first.x - third.x, first.y - third.y)
            cross = (second.x - first.x) * (third.y - first.y) - (
                second.y - first.y
            ) * (third.x - first.x)
            incoming_curvature = 2.0 * cross / (ab * bc * ca)
            self.assertLessEqual(abs(incoming_curvature), 0.01)
        ordinary_soft_offsets_m = []
        for ref in (
            "traffic_light_stop_line",
            "random_obstacle_entry",
            "random_obstacle_exit",
        ):
            expected = semantic_map["points"][ref]
            ordinary_soft_offsets_m.append(
                math.hypot(
                    points_by_ref[ref].x - float(expected["x"]),
                    points_by_ref[ref].y - float(expected["y"]),
                )
            )
        self.assertLessEqual(max(ordinary_soft_offsets_m), 1.20 + 1e-6)
        self.assertGreater(max(ordinary_soft_offsets_m), 0.80)
        for ref in ("pickup_approach_align", "drop_approach_align"):
            expected = semantic_map["points"][ref]
            actual = points_by_ref[ref]
            self.assertLessEqual(
                math.hypot(
                    actual.x - float(expected["x"]),
                    actual.y - float(expected["y"]),
                ),
                0.10 + 1e-6,
            )
            # The 1 m marker is a soft pose-shaping guide, not a stop or a
            # zero-curvature hard waypoint. The exact final 0.5 m tail above
            # still guarantees a straight dock entry.
            self.assertLessEqual(abs(actual.curvature), 0.20)
        self.assertEqual(
            [point.ref_id for point in result.points[1:] if point.v == 0.0],
            ["finish_park"],
        )
        self.assertLessEqual(
            max(abs(point.curvature) for point in result.points),
            1.0 / 0.60 + 1e-6,
        )

        pickup_rear_index = next(
            index
            for index, point in enumerate(result.points)
            if point.ref_id == "pickup_rear"
        )
        departure_tail_end_index = next(
            index
            for index, point in enumerate(result.points)
            if index > pickup_rear_index
            and point.ref_id == "pickup_shelf_clear_terminal_align"
        )
        drop_approach_index = next(
            index
            for index, point in enumerate(result.points)
            if point.ref_id == "drop_approach_align"
            and index > departure_tail_end_index
        )
        pickup_front = semantic_map["points"]["pickup_front"]
        pickup_rear = semantic_map["points"]["pickup_rear"]
        shelf_yaw = math.atan2(
            float(pickup_rear["y"]) - float(pickup_front["y"]),
            float(pickup_rear["x"]) - float(pickup_front["x"]),
        )
        departure_yaw = float(pickup_rear["yaw"])
        departure_tail = result.points[
            pickup_rear_index : departure_tail_end_index + 1
        ]
        self.assertLessEqual(abs(departure_tail[0].curvature), 0.01)
        self.assertGreaterEqual(
            departure_tail[-1].s - departure_tail[0].s,
            0.50,
        )
        for first, second in zip(departure_tail, departure_tail[1:]):
            tangent = math.atan2(second.y - first.y, second.x - first.x)
            heading_error = abs(
                math.degrees(
                    (tangent - departure_yaw + math.pi) % (2.0 * math.pi) - math.pi
                )
            )
            self.assertLessEqual(heading_error, 1.0)
        departure_turn = result.points[
            departure_tail_end_index : drop_approach_index + 1
        ]
        self.assertLessEqual(
            max(abs(point.curvature) for point in departure_turn),
            1.0 / 0.81 + 1e-6,
        )
        for prefix, distance_band_m, max_heading_error_deg in (
            ("pickup", (0.55, 0.70), 5.0),
            ("drop", (0.52, 0.68), 5.0),
        ):
            front = semantic_map["points"][f"{prefix}_front"]
            rear = semantic_map["points"][f"{prefix}_rear"]
            shelf_yaw = math.atan2(
                float(rear["y"]) - float(front["y"]),
                float(rear["x"]) - float(front["x"]),
            )
            direction_x = math.cos(shelf_yaw)
            direction_y = math.sin(shelf_yaw)
            heading_errors = []
            for point in result.points:
                distance_before_front_m = -(
                    (point.x - float(front["x"])) * direction_x
                    + (point.y - float(front["y"])) * direction_y
                )
                lateral_distance_m = abs(
                    -(point.x - float(front["x"])) * direction_y
                    + (point.y - float(front["y"])) * direction_x
                )
                if (
                    distance_band_m[0]
                    <= distance_before_front_m
                    <= distance_band_m[1]
                    and lateral_distance_m <= 0.35
                ):
                    heading_errors.append(
                        abs(
                            math.degrees(
                                (point.yaw - shelf_yaw + math.pi)
                                % (2.0 * math.pi)
                                - math.pi
                            )
                        )
                    )
            self.assertTrue(heading_errors, prefix)
            self.assertLessEqual(
                max(heading_errors),
                max_heading_error_deg,
                prefix,
            )

    def test_dock_approach_anchors_follow_precision_pose_axes(self) -> None:
        semantic_map = load_yaml_file(PROFILE_8_14_2_SEMANTIC_MAP)

        for prefix in ("pickup", "drop"):
            front = semantic_map["points"][f"{prefix}_front"]
            rear = semantic_map["points"][f"{prefix}_rear"]
            align = semantic_map["points"][f"{prefix}_approach_align"]
            front_terminal = semantic_map["points"][
                f"{prefix}_front_terminal_align"
            ]
            rear_terminal = semantic_map["points"][
                f"{prefix}_rear_terminal_align"
            ]
            shelf_yaw = math.atan2(
                float(rear["y"]) - float(front["y"]),
                float(rear["x"]) - float(front["x"]),
            )

            self.assertAlmostEqual(
                float(align["yaw"]), float(front["yaw"]), places=3
            )
            self.assertAlmostEqual(
                float(front_terminal["yaw"]), float(front["yaw"]), places=3
            )
            self.assertAlmostEqual(
                float(rear_terminal["yaw"]), shelf_yaw, places=3
            )
            self.assertAlmostEqual(
                math.hypot(
                    float(front["x"]) - float(align["x"]),
                    float(front["y"]) - float(align["y"]),
                ),
                1.00,
                places=3,
            )

        planning = load_yaml_file(PROFILE_8_14_2_PLANNING)["global_planner"]
        self.assertEqual(
            planning["hybrid_alignment_waypoint_heading_tolerance_deg"],
            5.0,
        )
        self.assertEqual(
            planning["hybrid_alignment_waypoint_position_tolerance_m"],
            0.10,
        )
        self.assertEqual(
            planning["hybrid_soft_waypoint_position_tolerance_m"],
            1.20,
        )

    def test_continuous_control_validation_route_uses_only_final_stop(self) -> None:
        route = load_yaml_file(
            REPO_ROOT / "config" / "routes" / "debug_control_validation_route.yaml"
        )
        semantic_map = load_yaml_file(
            REPO_ROOT / "maps" / "debug" / "semantic_map_control_validation.yaml"
        )
        planning = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "planning_params.yaml"
        )
        planning["global_planner"].update(
            {
                "plugin": "hybrid_astar",
                "min_turning_radius_m": 0.81,
                "path_sample_spacing_m": 0.10,
                "hybrid_step_length_m": 0.10,
                "hybrid_heading_bins": 72,
                "hybrid_goal_position_tolerance_m": 0.10,
                "hybrid_goal_heading_tolerance_deg": 5.0,
                "planning_timeout_ms": 10_000.0,
            }
        )
        planning["trajectory_smoother"]["enabled"] = False
        optimizer = {
            "continuous_trajectory_optimizer": {
                "plugin": "jerk_limited_s_curve",
                "max_speed_mps": 0.20,
                "max_acceleration_mps2": 0.20,
                "max_deceleration_mps2": 0.30,
                "max_jerk_mps3": 0.40,
                "max_lateral_acceleration_mps2": 0.20,
                "max_curvature_rate_1pmps": 0.80,
            }
        }

        result = optimize_continuous_route_trajectory(
            route,
            semantic_map,
            planning,
            optimizer,
        )

        self.assertTrue(result.ok, result.failures)
        self.assertEqual(result.planner_plugin, "hybrid_astar")
        self.assertEqual(result.optimizer_plugin, "jerk_limited_s_curve")
        self.assertEqual(result.points[0].v, 0.0)
        self.assertEqual(result.points[-1].v, 0.0)
        self.assertTrue(all(point.v > 0.0 for point in result.points[1:-1]))
        self.assertLessEqual(max(point.v for point in result.points), 0.20 + 1e-9)
        self.assertLessEqual(max(abs(point.a) for point in result.points), 0.20 + 1e-9)
        self.assertLessEqual(max(abs(point.jerk) for point in result.points), 0.40 + 1e-9)
        curvature_rates = [
            abs(current.curvature - previous.curvature) / (current.t - previous.t)
            for previous, current in zip(result.points, result.points[1:])
        ]
        self.assertLessEqual(max(curvature_rates), 0.80 + 1e-9)
        self.assertTrue(
            all(current.t > previous.t for previous, current in zip(result.points, result.points[1:]))
        )
        stopped_refs = [point.ref_id for point in result.points[1:] if point.v == 0.0]
        self.assertEqual(stopped_refs, ["finish_park"])

        staged = optimize_continuous_route_trajectory(
            route,
            semantic_map,
            planning,
            optimizer,
            end_ref="traffic_light_stop_line",
        )

        self.assertTrue(staged.ok, staged.failures)
        self.assertEqual(staged.points[-1].ref_id, "traffic_light_stop_line")
        self.assertEqual(staged.points[-1].v, 0.0)
        self.assertTrue(all(point.v > 0.0 for point in staged.points[1:-1]))
        self.assertLessEqual(max(abs(point.a) for point in staged.points), 0.20 + 1e-9)
        self.assertLessEqual(max(abs(point.jerk) for point in staged.points), 0.40 + 1e-9)

    def test_spatial_speed_envelope_accelerates_straights_without_relaxing_turn_rate(
        self,
    ) -> None:
        reference = load_yaml_file(
            REPO_ROOT
            / "docs"
            / "evidence"
            / "day5"
            / "debug_indoor_one_lap_continuous_trajectory_8_14_1.yaml"
        )
        optimizer = load_yaml_file(
            REPO_ROOT
            / "config"
            / "planning"
            / "optimizer_params_day5_speed_2x.yaml"
        )

        result = retime_continuous_trajectory(reference, optimizer)

        self.assertTrue(result.ok, result.failures)
        self.assertAlmostEqual(max(point.v for point in result.points), 0.20)
        self.assertGreaterEqual(
            sum(abs(point.v - 0.20) <= 1e-9 for point in result.points),
            200,
        )
        self.assertLess(result.duration_s, 130.0)
        self.assertEqual(result.points[0].v, 0.0)
        self.assertEqual(result.points[-1].v, 0.0)
        self.assertTrue(all(point.v > 0.0 for point in result.points[1:-1]))
        self.assertLessEqual(max(point.a for point in result.points), 0.16 + 1e-9)
        self.assertGreaterEqual(min(point.a for point in result.points), -0.12 - 1e-9)
        self.assertLessEqual(
            max(abs(point.jerk) for point in result.points),
            0.40 + 1e-9,
        )
        curvature_rates = [
            abs(current.curvature - previous.curvature) / (current.t - previous.t)
            for previous, current in zip(result.points, result.points[1:])
        ]
        self.assertLessEqual(max(curvature_rates), 0.80 + 1e-9)
        for original, retimed in zip(reference["points"], result.points):
            self.assertEqual(float(original["x"]), retimed.x)
            self.assertEqual(float(original["y"]), retimed.y)
            self.assertEqual(float(original["yaw"]), retimed.yaw)
            self.assertEqual(float(original["s"]), retimed.s)
            self.assertEqual(float(original["curvature"]), retimed.curvature)
            self.assertEqual(original.get("ref_id"), retimed.ref_id)

    def test_indoor_speed_envelope_is_jerk_limited_without_stopping_in_turns(
        self,
    ) -> None:
        reference = load_yaml_file(
            REPO_ROOT
            / "docs"
            / "evidence"
            / "day5"
            / "indoor_competition_mission_trajectory.yaml"
        )
        optimizer = load_yaml_file(
            REPO_ROOT
            / "config"
            / "planning"
            / "optimizer_params_indoor_competition.yaml"
        )
        optimizer["continuous_trajectory_optimizer"]["max_jerk_mps3"] = 2.0

        result = retime_continuous_trajectory(reference, optimizer)

        self.assertTrue(result.ok, result.failures)
        self.assertGreaterEqual(max(point.v for point in result.points), 0.999)
        self.assertTrue(all(point.v > 0.05 for point in result.points[1:-1]))
        self.assertLessEqual(
            max(abs(point.jerk) for point in result.points),
            2.0 + 1e-9,
        )
        curvature_rates = [
            abs(current.curvature - previous.curvature) / (current.t - previous.t)
            for previous, current in zip(result.points, result.points[1:])
        ]
        self.assertLessEqual(max(curvature_rates), 3.00 + 1e-9)
        for original, retimed in zip(reference["points"], result.points):
            self.assertEqual(float(original["x"]), retimed.x)
            self.assertEqual(float(original["y"]), retimed.y)
            self.assertEqual(float(original["yaw"]), retimed.yaw)
            self.assertEqual(float(original["s"]), retimed.s)
            self.assertEqual(float(original["curvature"]), retimed.curvature)
            self.assertEqual(original.get("ref_id"), retimed.ref_id)

    def test_semantic_stops_and_obstacle_zone_speed_caps_are_applied(self) -> None:
        plan = StepPlan(
            step_id="go_pickup",
            step_type="RUN_SEGMENT",
            corridor_ref="go_to_pickup",
            target_ref="pickup_dock",
            target_source="target_ref",
            planning_time_ms=1.0,
            path=(
                PathPoint(0.0, 0.0, 0.0, ref_id="start"),
                PathPoint(1.0, 0.0, 0.0, ref_id="traffic_light_stop_line"),
                PathPoint(2.0, 0.0, 0.0, ref_id="random_obstacle_entry"),
                PathPoint(3.0, 0.0, 0.0),
                PathPoint(4.0, 0.0, 0.0, ref_id="random_obstacle_exit"),
                PathPoint(5.0, 0.0, 0.0, ref_id="pickup_dock"),
            ),
        )
        semantic_map = {
            "stop_lines": [
                {"id": "traffic_light_stop_line", "point_ref": "traffic_light_stop_line"},
                {"id": "pickup_dock_stop_line", "point_ref": "pickup_dock"},
            ],
            "dock_poses": [
                {"id": "pickup_dock", "point_ref": "pickup_dock", "dock_type": "PICKUP"}
            ],
            "obstacle_zones": [
                {
                    "id": "random_obstacle_zone_1",
                    "semantic_type": "RANDOM_OBSTACLE",
                    "boundary": [[2.0, -1.0], [4.0, -1.0], [4.0, 1.0], [2.0, 1.0]],
                }
            ],
        }
        optimizer_config = {
            "trajectory_optimizer": {
                "max_speed_mps": 0.50,
                "max_acceleration_mps2": 0.30,
                "max_deceleration_mps2": 0.50,
                "max_lateral_acceleration_mps2": 0.20,
                "obstacle_zone_speed_limits_mps": {"RANDOM_OBSTACLE": 0.30},
            }
        }

        trajectory = parameterize_step_plan(plan, semantic_map, optimizer_config)

        by_ref = {point.ref_id: point for point in trajectory.points if point.ref_id}
        self.assertEqual(by_ref["traffic_light_stop_line"].v, 0.0)
        self.assertEqual(by_ref["pickup_dock"].v, 0.0)
        self.assertGreater(by_ref["random_obstacle_entry"].v, 0.0)
        self.assertGreater(by_ref["random_obstacle_exit"].v, 0.0)
        self.assertLessEqual(by_ref["random_obstacle_entry"].v, 0.30)
        self.assertLessEqual(by_ref["random_obstacle_exit"].v, 0.30)
        self.assertEqual([round(point.s, 3) for point in trajectory.points], [0, 1, 2, 3, 4, 5])
        self.assertTrue(all(point.curvature == 0.0 for point in trajectory.points))
        self.assertTrue(all(point.yaw_rate == 0.0 for point in trajectory.points))
        self.assertTrue(
            all(current.t > previous.t for previous, current in zip(trajectory.points, trajectory.points[1:]))
        )

        for previous, current in zip(trajectory.points, trajectory.points[1:]):
            ds = current.s - previous.s
            acceleration = (current.v * current.v - previous.v * previous.v) / (2.0 * ds)
            self.assertLessEqual(acceleration, 0.30 + 1e-9)
            self.assertGreaterEqual(acceleration, -0.50 - 1e-9)
            self.assertFalse(math.isnan(current.t))

    def test_curve_speed_cap_uses_lateral_acceleration_limit(self) -> None:
        plan = StepPlan(
            step_id="turn",
            step_type="RUN_SEGMENT",
            corridor_ref="turn_corridor",
            target_ref="goal",
            target_source="target_ref",
            planning_time_ms=1.0,
            path=(
                PathPoint(0.0, 0.0, 0.0, ref_id="start"),
                PathPoint(1.0, 0.0, 0.0),
                PathPoint(1.0, 1.0, math.pi / 2.0, ref_id="goal"),
            ),
        )
        optimizer_config = {
            "trajectory_optimizer": {
                "max_speed_mps": 1.00,
                "max_acceleration_mps2": 2.00,
                "max_deceleration_mps2": 2.00,
                "max_lateral_acceleration_mps2": 0.20,
            }
        }

        trajectory = parameterize_step_plan(plan, {}, optimizer_config)

        for point in trajectory.points:
            self.assertNotEqual(point.curvature, 0.0)
            expected_cap = math.sqrt(0.20 / abs(point.curvature))
            self.assertLessEqual(point.v, expected_cap + 1e-9)
            self.assertAlmostEqual(point.yaw_rate, point.v * point.curvature)

    def test_high_curvature_speed_cap_does_not_slow_straight_paths(self) -> None:
        optimizer = {
            "trajectory_optimizer": {
                "max_speed_mps": 0.12,
                "max_acceleration_mps2": 0.30,
                "max_deceleration_mps2": 0.50,
                "max_lateral_acceleration_mps2": 0.20,
                "high_curvature_threshold_1pm": 0.80,
                "high_curvature_speed_limit_mps": 0.07,
            }
        }
        straight = StepPlan(
            step_id="straight",
            step_type="RUN_SEGMENT",
            corridor_ref="test",
            target_ref="end",
            target_source="test",
            path=tuple(PathPoint(0.2 * index, 0.0, 0.0) for index in range(8)),
            planning_time_ms=0.0,
            planner_plugin="test",
            smoother_plugin="none",
        )
        curve = StepPlan(
            step_id="curve",
            step_type="RUN_SEGMENT",
            corridor_ref="test",
            target_ref="end",
            target_source="test",
            path=(
                PathPoint(0.0, 0.0, 0.0),
                PathPoint(0.1, 0.0, 0.0),
                PathPoint(0.2, 0.1, math.pi / 4.0),
                PathPoint(0.2, 0.2, math.pi / 2.0),
            ),
            planning_time_ms=0.0,
            planner_plugin="test",
            smoother_plugin="none",
        )

        straight_result = parameterize_step_plan(straight, {}, optimizer)
        curve_result = parameterize_step_plan(curve, {}, optimizer)

        self.assertGreater(max(point.v for point in straight_result.points), 0.09)
        self.assertLessEqual(max(point.v for point in curve_result.points), 0.07 + 1e-9)

    def test_public_route_optimizer_generates_stop_bounded_trajectories(self) -> None:
        route = load_yaml_file(REPO_ROOT / "config" / "routes" / "debug_route.yaml")
        semantic_map = load_yaml_file(REPO_ROOT / "maps" / "debug" / "semantic_map.yaml")
        planning_params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "planning_params.yaml"
        )
        optimizer_params = load_yaml_file(
            REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
        )

        result = optimize_route_trajectory(route, semantic_map, planning_params, optimizer_params)

        self.assertTrue(result.ok, result.failures)
        self.assertEqual(
            [trajectory.step_id for trajectory in result.trajectories],
            [
                "go_traffic_light_1",
                "random_obstacle_1",
                "pickup_1_rear",
                "cone_lane_change_1",
                "drop_1_rear",
                "return_to_pickup_area",
                "pickup_2_rear",
                "cone_lane_change_2",
                "drop_2_rear",
                "finish_park",
            ],
        )
        self.assertEqual(result.frame_id, "map")
        self.assertEqual(result.route_name, "debug_route")

        traffic = _point_by_ref(result, "go_traffic_light_1", "traffic_light_stop_line")
        pickup_front = _point_by_ref(result, "random_obstacle_1", "pickup_front")
        pickup_rear = _point_by_ref(result, "pickup_1_rear", "pickup_rear")
        drop_front = _point_by_ref(result, "cone_lane_change_1", "drop_front")
        drop_rear = _point_by_ref(result, "drop_1_rear", "drop_rear")
        finish = _point_by_ref(result, "finish_park", "finish_park")
        self.assertEqual(traffic.v, 0.0)
        self.assertEqual(pickup_front.v, 0.0)
        self.assertEqual(pickup_rear.v, 0.0)
        self.assertEqual(drop_front.v, 0.0)
        self.assertEqual(drop_rear.v, 0.0)
        self.assertEqual(finish.v, 0.0)

        random_entry = _point_by_ref(result, "random_obstacle_1", "random_obstacle_entry")
        random_exit = _point_by_ref(result, "random_obstacle_1", "random_obstacle_exit")
        cone_entry = _point_by_ref(result, "cone_lane_change_1", "cone_lane_change_entry")
        cone_exit = _point_by_ref(result, "cone_lane_change_1", "cone_lane_change_exit")
        for point in (random_entry, random_exit, cone_entry, cone_exit):
            self.assertGreater(point.v, 0.0)
            self.assertLessEqual(point.v, 0.30 + 1e-9)

        for trajectory in result.trajectories:
            self.assertGreater(trajectory.duration_s, 0.0)
            self.assertGreater(trajectory.path_length_m, 0.0)
            self.assertLessEqual(max(point.v for point in trajectory.points), 0.50 + 1e-9)
            for previous, current in zip(trajectory.points, trajectory.points[1:]):
                self.assertGreaterEqual(current.s, previous.s)
                self.assertGreaterEqual(current.t, previous.t)
                ds = current.s - previous.s
                if ds > 1e-9:
                    acceleration = (current.v * current.v - previous.v * previous.v) / (2.0 * ds)
                    self.assertLessEqual(acceleration, 0.30 + 1e-9)
                    self.assertGreaterEqual(acceleration, -0.50 - 1e-9)
            sample = trajectory.points[0].to_dict()
            for key in ("x", "y", "yaw", "s", "curvature", "v", "yaw_rate", "t"):
                self.assertIn(key, sample)

    def test_offline_cli_writes_yaml_and_per_step_csv(self) -> None:
        script = (
            REPO_ROOT
            / "src"
            / "competition_planning"
            / "competition_planning"
            / "offline_optimized_trajectory.py"
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = pathlib.Path(tmp_dir)
            output_yaml = tmp_path / "optimized_trajectory.yaml"
            csv_dir = tmp_path / "csv"
            report_path = tmp_path / "summary.md"
            svg_path = tmp_path / "overview.svg"
            env = os.environ.copy()
            python_path = str(REPO_ROOT / "src" / "competition_planning")
            env["PYTHONPATH"] = (
                python_path
                if not env.get("PYTHONPATH")
                else python_path + os.pathsep + env["PYTHONPATH"]
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--route",
                    str(REPO_ROOT / "config" / "routes" / "debug_route.yaml"),
                    "--semantic-map",
                    str(REPO_ROOT / "maps" / "debug" / "semantic_map.yaml"),
                    "--planning-params",
                    str(REPO_ROOT / "config" / "planning" / "planning_params.yaml"),
                    "--optimizer-params",
                    str(REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"),
                    "--output",
                    str(output_yaml),
                    "--csv-dir",
                    str(csv_dir),
                    "--report",
                    str(report_path),
                    "--svg",
                    str(svg_path),
                ],
                capture_output=True,
                env=env,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = load_yaml_file(output_yaml)
            self.assertEqual(len(output["trajectories"]), 10)
            self.assertEqual(
                set(output["source_manifest"]),
                {
                    "route",
                    "semantic_map",
                    "planning_params",
                    "optimizer_params",
                    "occupancy_map",
                    "occupancy_image",
                },
            )
            self.assertIn(
                "route=debug_route ok=True trajectories=10 failures=0",
                completed.stdout,
            )
            csv_files = sorted(path.name for path in csv_dir.glob("*.csv"))
            self.assertEqual(
                csv_files,
                [
                    "cone_lane_change_1.csv",
                    "cone_lane_change_2.csv",
                    "drop_1_rear.csv",
                    "drop_2_rear.csv",
                    "finish_park.csv",
                    "go_traffic_light_1.csv",
                    "pickup_1_rear.csv",
                    "pickup_2_rear.csv",
                    "random_obstacle_1.csv",
                    "return_to_pickup_area.csv",
                ],
            )
            header = (csv_dir / "go_traffic_light_1.csv").read_text(
                encoding="utf-8"
            ).splitlines()[0]
            self.assertEqual(header, "x,y,yaw,s,curvature,v,yaw_rate,t,ref_id")
            self.assertIn(
                "Day 4 优化轨迹摘要",
                report_path.read_text(encoding="utf-8"),
            )
            self.assertIn("<svg", svg_path.read_text())


def _point_by_ref(result, step_id: str, ref_id: str):
    trajectory = next(item for item in result.trajectories if item.step_id == step_id)
    return next(point for point in trajectory.points if point.ref_id == ref_id)


if __name__ == "__main__":
    unittest.main()
