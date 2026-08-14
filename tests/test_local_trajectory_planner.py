from dataclasses import replace
import math
import pathlib
import sys
import unittest
from unittest.mock import patch

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_planning"))
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_control"))

from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    MPPIController,
    MPPIParams,
)
from competition_planning.hybrid_astar_planner import (
    AsymmetricFootprint,
    HybridAStarPlanner,
    HybridAStarTimeout,
)
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
    concatenate_reference_paths,
    docking_mode_is_active,
    docking_shelf_filter_is_active,
    filter_expected_docking_shelf_points,
    occupied_grid_cell_centers,
    precision_docking_work_sides,
    reference_prefix_to_checkpoint,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path


def _empty_map(*, width: int = 100) -> OccupancyGridMap:
    height = 80
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=0.10,
        origin_x=-1.0,
        origin_y=-4.0,
        occupied=tuple(False for _ in range(width * height)),
    )


def _map_with_obstacle(x: float, y: float) -> OccupancyGridMap:
    grid_map = OccupancyGridMap(
        width=80,
        height=80,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=-2.0,
        occupied=tuple(False for _ in range(80 * 80)),
    )
    occupied = list(grid_map.occupied)
    occupied[grid_map.index(grid_map.world_to_cell(x, y))] = True
    return replace(grid_map, occupied=tuple(occupied))


def _docking_planner(obstacle_x: float, obstacle_y: float) -> HybridAStarPlanner:
    return HybridAStarPlanner(
        _map_with_obstacle(obstacle_x, obstacle_y),
        inflation_radius_m=0.0,
        search_padding_m=1.0,
        sample_spacing_m=0.05,
        min_turning_radius_m=0.60,
        step_length_m=0.10,
        curvature_bins=9,
        heading_bins=72,
        goal_position_tolerance_m=0.10,
        goal_heading_tolerance_rad=math.radians(5.0),
        footprint=AsymmetricFootprint(
            vehicle_length_m=0.72,
            vehicle_width_m=0.50,
            front_clearance_m=0.10,
            rear_clearance_m=0.10,
            left_clearance_m=0.10,
            right_clearance_m=0.05,
        ),
    )


class DockingCollisionModeTest(unittest.TestCase):
    def test_shelf_filter_requires_final_distance_and_aligned_heading(self) -> None:
        checkpoint = PathPoint(8.413, -0.081, 0.0, "pickup_front")

        self.assertTrue(
            docking_shelf_filter_is_active(
                current_pose=PathPoint(7.90, -0.081, math.radians(4.9)),
                checkpoint=checkpoint,
                activation_distance_m=1.00,
                heading_tolerance_rad=math.radians(5.0),
            )
        )
        self.assertFalse(
            docking_shelf_filter_is_active(
                current_pose=PathPoint(7.90, -0.081, math.radians(5.1)),
                checkpoint=checkpoint,
                activation_distance_m=1.00,
                heading_tolerance_rad=math.radians(5.0),
            )
        )
        self.assertFalse(
            docking_shelf_filter_is_active(
                current_pose=PathPoint(7.40, -0.081, 0.0),
                checkpoint=checkpoint,
                activation_distance_m=1.00,
                heading_tolerance_rad=math.radians(5.0),
            )
        )

    def test_aligned_final_approach_filters_only_expected_right_shelf_echoes(self) -> None:
        checkpoint = PathPoint(8.413, -0.081, 0.0, "pickup_front")
        points = (
            (8.10, -0.611),  # Expected shelf face, 0.53 m right of centerline.
            (8.10, -0.361),  # Too close to the physical body; must remain blocked.
            (8.10, 0.449),  # Non-work side obstacle; must remain blocked.
            (9.10, -0.611),  # Beyond the final approach corridor; must remain.
        )

        filtered, removed_count = filter_expected_docking_shelf_points(
            points,
            checkpoint=checkpoint,
            work_side="RIGHT",
            vehicle_length_m=0.72,
            vehicle_width_m=0.50,
            front_clearance_m=0.10,
            approach_distance_m=1.00,
            physical_guard_m=0.05,
        )

        self.assertEqual(removed_count, 1)
        self.assertEqual(filtered, points[1:])

    def test_left_work_side_filter_is_mirrored(self) -> None:
        checkpoint = PathPoint(0.0, 0.0, 0.0, "drop_front")
        points = ((-0.20, 0.53), (-0.20, -0.53))

        filtered, removed_count = filter_expected_docking_shelf_points(
            points,
            checkpoint=checkpoint,
            work_side="LEFT",
            vehicle_length_m=0.72,
            vehicle_width_m=0.50,
            front_clearance_m=0.10,
            approach_distance_m=1.00,
            physical_guard_m=0.05,
        )

        self.assertEqual(removed_count, 1)
        self.assertEqual(filtered, (points[1],))

    def test_only_pickup_and_drop_semantics_enable_precision_docking(self) -> None:
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )

        work_sides = precision_docking_work_sides(semantic_map["dock_poses"])

        self.assertEqual(
            work_sides,
            {
                "pickup_front": "RIGHT",
                "pickup_rear": "RIGHT",
                "drop_front": "RIGHT",
                "drop_rear": "RIGHT",
            },
        )

    def test_right_work_side_accepts_measured_28_cm_shelf_gap(self) -> None:
        planner = _docking_planner(0.0, -(0.25 + 0.28))

        self.assertTrue(planner.path_is_navigable((PathPoint(0.0, 0.0, 0.0),)))
        path = planner.plan(
            (
                PathPoint(-0.50, 0.0, 0.0),
                PathPoint(0.0, 0.0, 0.0, "pickup_front"),
            )
        )

        self.assertEqual(path[-1].ref_id, "pickup_front")

    def test_all_measured_dock_poses_clear_the_static_map_footprint(self) -> None:
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )
        runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "local_hybrid_astar_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["local_hybrid_astar_runtime"]
        planner = HybridAStarPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / semantic_map["source_map"]),
            inflation_radius_m=0.0,
            search_padding_m=1.0,
            sample_spacing_m=0.05,
            min_turning_radius_m=0.60,
            step_length_m=0.10,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.10,
            goal_heading_tolerance_rad=math.radians(5.0),
            footprint=AsymmetricFootprint(
                vehicle_length_m=runtime["docking_vehicle_length_m"],
                vehicle_width_m=runtime["docking_vehicle_width_m"],
                front_clearance_m=runtime["docking_front_clearance_m"],
                rear_clearance_m=runtime["docking_rear_clearance_m"],
                left_clearance_m=runtime["docking_non_work_side_clearance_m"],
                right_clearance_m=runtime["docking_work_side_clearance_m"],
            ),
        )

        points = semantic_map["points"]
        for ref_id in ("pickup_front", "pickup_rear", "drop_front", "drop_rear"):
            point = points[ref_id]
            with self.subTest(ref_id=ref_id):
                self.assertTrue(
                    planner.path_is_navigable(
                        (
                            PathPoint(
                                point["x"],
                                point["y"],
                                point["yaw"],
                                ref_id,
                            ),
                        )
                    )
                )

    def test_non_work_side_keeps_larger_clearance(self) -> None:
        planner = _docking_planner(0.0, 0.25 + 0.08)

        self.assertFalse(planner.path_is_navigable((PathPoint(0.0, 0.0, 0.0),)))

    def test_footprint_rotates_with_candidate_vehicle_pose(self) -> None:
        planner = _docking_planner(-(0.25 + 0.08), 0.0)

        self.assertFalse(
            planner.path_is_navigable((PathPoint(0.0, 0.0, math.pi / 2.0),))
        )

    def test_semantic_mode_requires_dock_ref_and_activation_distance(self) -> None:
        checkpoint = PathPoint(8.413, -0.081, 0.015, "pickup_front")

        self.assertTrue(
            docking_mode_is_active(
                current_pose=PathPoint(7.15, -0.081, 0.0),
                checkpoint=checkpoint,
                active_checkpoint_ref="pickup_front",
                docking_refs={"pickup_front", "pickup_rear"},
                activation_distance_m=1.5,
            )
        )
        self.assertFalse(
            docking_mode_is_active(
                current_pose=PathPoint(6.80, -0.081, 0.0),
                checkpoint=checkpoint,
                active_checkpoint_ref="pickup_front",
                docking_refs={"pickup_front", "pickup_rear"},
                activation_distance_m=1.5,
            )
        )
        self.assertFalse(
            docking_mode_is_active(
                current_pose=PathPoint(8.0, -0.081, 0.0),
                checkpoint=checkpoint,
                active_checkpoint_ref="finish_park",
                docking_refs={"pickup_front", "pickup_rear"},
                activation_distance_m=1.5,
            )
        )


def _straight_reference(length_m: float = 8.0) -> tuple[PathPoint, ...]:
    return tuple(
        PathPoint(x=index * 0.10, y=0.0, yaw=0.0)
        for index in range(round(length_m / 0.10) + 1)
    )


def _table_points() -> tuple[tuple[float, float], ...]:
    return tuple(
        (1.50 + x_index * 0.10, -0.40 + y_index * 0.10)
        for x_index in range(6)
        for y_index in range(9)
    )


class ReferencePathConcatenationTest(unittest.TestCase):
    def test_current_continuous_route_stops_at_each_exact_checkpoint_gate(self) -> None:
        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_indoor_one_lap_continuous_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                float(point["x"]),
                float(point["y"]),
                float(point["yaw"]),
                point.get("ref_id"),
            )
            for point in artifact["points"]
        )
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )
        optimizer = yaml.safe_load(
            (
                REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
            ).read_text(encoding="utf-8")
        )
        optimizer["trajectory_optimizer"]["max_curvature_1pm"] = 1.0 / 0.60

        for checkpoint_ref in (
            "pickup_front",
            "pickup_rear",
            "drop_front",
            "drop_rear",
            "finish_park",
        ):
            record = semantic_map["points"][checkpoint_ref]
            semantic_checkpoint = PathPoint(
                float(record["x"]),
                float(record["y"]),
                float(record["yaw"]),
                checkpoint_ref,
            )
            prefix = reference_prefix_to_checkpoint(
                reference,
                semantic_checkpoint,
                exact_pose=checkpoint_ref != "finish_park",
            )
            trajectory = parameterize_local_path(prefix, semantic_map, optimizer)

            if checkpoint_ref != "finish_park":
                self.assertEqual(prefix[-1], semantic_checkpoint)
            else:
                self.assertLessEqual(
                    math.hypot(
                        prefix[-1].x - semantic_checkpoint.x,
                        prefix[-1].y - semantic_checkpoint.y,
                    ),
                    0.10,
                )
            self.assertEqual(trajectory.points[-1].ref_id, checkpoint_ref)
            self.assertEqual(trajectory.points[-1].v, 0.0)

    def test_checkpoint_gate_exposes_only_prefix_of_one_continuous_reference(self) -> None:
        reference = (
            PathPoint(0.0, 0.0, 0.0, "start"),
            PathPoint(1.0, 0.0, 0.0, "pickup_front"),
            PathPoint(2.0, 0.0, 0.0, "pickup_rear"),
            PathPoint(3.0, 0.0, 0.0, "finish_park"),
        )

        pickup_front = reference_prefix_to_checkpoint(
            reference,
            PathPoint(1.1, -0.1, 0.05, "pickup_front"),
        )
        pickup_rear = reference_prefix_to_checkpoint(
            reference,
            PathPoint(2.1, -0.1, 0.05, "pickup_rear"),
        )

        self.assertEqual(pickup_front[:-1], reference[:1])
        self.assertEqual(pickup_front[-1], PathPoint(1.1, -0.1, 0.05, "pickup_front"))
        self.assertEqual(pickup_rear[:-1], reference[:2])
        self.assertEqual(pickup_rear[-1], PathPoint(2.1, -0.1, 0.05, "pickup_rear"))
        self.assertEqual(len(reference), 4)

    def test_checkpoint_gate_fails_closed_when_ref_is_missing(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing_checkpoint"):
            reference_prefix_to_checkpoint(
                (PathPoint(0.0, 0.0, 0.0, "start"),),
                PathPoint(1.0, 0.0, 0.0, "missing_checkpoint"),
            )

    def test_joins_segments_without_dropping_stop_or_obstacle_refs(self) -> None:
        first = (
            PathPoint(0.0, 0.0, 0.0, "start"),
            PathPoint(1.0, 0.0, 0.0, "traffic_light_stop_line"),
        )
        second = (
            PathPoint(1.0, 0.0, 0.0, "traffic_light_stop_line"),
            PathPoint(2.0, 0.1, 0.1, "random_obstacle_entry"),
            PathPoint(3.0, 0.0, 0.0, "random_obstacle_exit"),
        )

        joined = concatenate_reference_paths((first, second))

        self.assertEqual(len(joined), 4)
        self.assertEqual(
            [point.ref_id for point in joined if point.ref_id],
            [
                "start",
                "traffic_light_stop_line",
                "random_obstacle_entry",
                "random_obstacle_exit",
            ],
        )

    def test_rejects_disconnected_reference_segments(self) -> None:
        with self.assertRaisesRegex(ValueError, "disconnected"):
            concatenate_reference_paths(
                (
                    (PathPoint(0.0, 0.0, 0.0), PathPoint(1.0, 0.0, 0.0)),
                    (PathPoint(1.5, 0.0, 0.0), PathPoint(2.0, 0.0, 0.0)),
                )
            )

    def test_whole_route_join_preserves_downstream_precision_geometry(self) -> None:
        first = (
            PathPoint(0.0, 0.0, 0.0, "start"),
            PathPoint(1.0, 0.0, 0.0, "drop_approach_align"),
        )
        second = (
            PathPoint(1.08, 0.04, 0.05, "drop_approach_align"),
            PathPoint(1.58, 0.04, 0.05, "drop_front_terminal_align"),
            PathPoint(2.08, 0.04, 0.05, "drop_front"),
        )

        joined = concatenate_reference_paths(
            (first, second),
            preserve_next_path=True,
        )

        self.assertEqual(joined[-2:], second[-2:])
        self.assertEqual(joined[1], second[0])

    def test_indoor_one_lap_segments_form_one_planning_reference(self) -> None:
        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day4"
                / "debug_indoor_one_lap_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        paths = tuple(
            tuple(
                PathPoint(
                    float(point["x"]),
                    float(point["y"]),
                    float(point["yaw"]),
                    point.get("ref_id"),
                )
                for point in trajectory["points"]
            )
            for trajectory in artifact["trajectories"]
        )

        joined = concatenate_reference_paths(paths)
        refs = {point.ref_id for point in joined if point.ref_id}

        self.assertEqual(len(joined), sum(map(len, paths)) - (len(paths) - 1))
        self.assertTrue(
            {
                "traffic_light_stop_line",
                "random_obstacle_entry",
                "random_obstacle_exit",
                "pickup_front",
                "pickup_rear",
                "drop_front",
                "drop_rear",
                "finish_park",
            }.issubset(refs)
        )

    def test_indoor_route_boundary_is_valid_for_runtime_parameterization(self) -> None:
        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day4"
                / "debug_indoor_one_lap_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        paths = tuple(
            tuple(
                PathPoint(
                    float(point["x"]),
                    float(point["y"]),
                    float(point["yaw"]),
                    point.get("ref_id"),
                )
                for point in trajectory["points"]
            )
            for trajectory in artifact["trajectories"]
        )
        joined = concatenate_reference_paths(paths)
        local_path = joined[:31]

        trajectory = parameterize_local_path(
            local_path,
            yaml.safe_load(
                (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                    encoding="utf-8"
                )
            ),
            yaml.safe_load(
                (
                    REPO_ROOT / "config" / "planning" / "optimizer_params.yaml"
                ).read_text(encoding="utf-8")
            ),
        )

        self.assertLessEqual(
            max(abs(point.curvature) for point in trajectory.points),
            1.0 / 0.60 + 1e-9,
        )
        for path in paths:
            endpoint = path[-1]
            joined_endpoint = next(
                point for point in joined if point.ref_id == endpoint.ref_id
            )
            self.assertAlmostEqual(joined_endpoint.x, endpoint.x)
            self.assertAlmostEqual(joined_endpoint.y, endpoint.y)


def _relaxed_reference(*, repeated: bool = False) -> tuple[PathPoint, ...]:
    markers = {
        30: "random_obstacle_entry",
        60: "random_obstacle_exit",
    }
    if repeated:
        markers.update(
            {
                90: "random_obstacle_entry",
                120: "random_obstacle_exit",
            }
        )
    length = 140 if repeated else 80
    return tuple(
        PathPoint(
            x=index * 0.10,
            y=0.0,
            yaw=0.0,
            ref_id=markers.get(index),
        )
        for index in range(length + 1)
    )


def _relaxed_config(**changes: object) -> LocalReplanConfig:
    return replace(
        LocalReplanConfig(
            lookahead_distance_m=3.0,
            inflation_radius_m=0.10,
            search_padding_m=1.5,
            sample_spacing_m=0.10,
            min_turning_radius_m=0.60,
            step_length_m=0.20,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.15,
            goal_heading_tolerance_rad=math.radians(8.0),
            reference_deviation_weight=2.0,
            max_expansions=250_000,
            relaxed_segment_entry_ref="random_obstacle_entry",
            relaxed_segment_exit_ref="random_obstacle_exit",
            relaxed_activation_distance_m=1.0,
            relaxed_reference_deviation_weight=0.5,
            relaxed_corridor_half_width_m=0.85,
            relaxed_goal_heading_tolerance_rad=math.radians(20.0),
            trajectory_switch_improvement_ratio=0.15,
        ),
        **changes,
    )


class LocalTrajectoryPlannerTest(unittest.TestCase):
    def test_pickup_front_docking_mode_waits_for_the_alignment_zone(self) -> None:
        runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "local_hybrid_astar_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["local_hybrid_astar_runtime"]
        semantic_map = yaml.safe_load(
            (REPO_ROOT / "maps" / "debug" / "semantic_map.yaml").read_text(
                encoding="utf-8"
            )
        )
        artifact = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_indoor_one_lap_continuous_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                float(point["x"]),
                float(point["y"]),
                float(point["yaw"]),
                point.get("ref_id"),
            )
            for point in artifact["points"]
        )
        record = semantic_map["points"]["pickup_front"]
        checkpoint = PathPoint(
            float(record["x"]),
            float(record["y"]),
            float(record["yaw"]),
            "pickup_front",
        )
        current_pose = PathPoint(7.019, -0.075, -0.116)

        self.assertFalse(
            docking_mode_is_active(
                current_pose=current_pose,
                checkpoint=checkpoint,
                active_checkpoint_ref="pickup_front",
                docking_refs={"pickup_front", "pickup_rear"},
                activation_distance_m=runtime["docking_activation_distance_m"],
            )
        )

        planner = LocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / semantic_map["source_map"]),
            LocalReplanConfig(
                lookahead_distance_m=runtime["lookahead_distance_m"],
                inflation_radius_m=runtime["inflation_radius_m"],
                search_padding_m=runtime["search_padding_m"],
                sample_spacing_m=runtime["sample_spacing_m"],
                min_turning_radius_m=runtime["min_turning_radius_m"],
                step_length_m=runtime["step_length_m"],
                curvature_bins=runtime["curvature_bins"],
                heading_bins=runtime["heading_bins"],
                goal_position_tolerance_m=runtime["goal_position_tolerance_m"],
                goal_heading_tolerance_rad=math.radians(
                    runtime["goal_heading_tolerance_deg"]
                ),
                reference_deviation_weight=runtime[
                    "reference_deviation_weight"
                ],
                max_expansions=runtime["max_expansions"],
                planning_timeout_s=runtime["planning_timeout_s"],
                reference_search_window_points=runtime[
                    "reference_search_window_points"
                ],
            ),
        )
        result = planner.plan(
            reference_path=reference_prefix_to_checkpoint(
                reference,
                checkpoint,
                exact_pose=True,
            ),
            current_pose=current_pose,
            dynamic_obstacle_points=(),
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertTrue(result.path_is_navigable)

    def setUp(self) -> None:
        self.config = LocalReplanConfig(
            lookahead_distance_m=5.0,
            inflation_radius_m=0.30,
            search_padding_m=3.0,
            sample_spacing_m=0.10,
            min_turning_radius_m=0.81,
            step_length_m=0.20,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.25,
            goal_heading_tolerance_rad=math.radians(15.0),
            reference_deviation_weight=2.0,
            max_expansions=250_000,
        )
        self.reference = _straight_reference()

    def test_clear_reference_is_reused_without_unnecessary_detour(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=(),
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertEqual(result.reference_start_index, 0)
        self.assertEqual(result.rejoin_index, 50)
        self.assertEqual(result.path, self.reference[:51])
        self.assertLess(result.planning_grid_cell_count, 100 * 80)

    def test_checkpoint_endpoint_keeps_a_valid_terminal_tracking_segment(self) -> None:
        checkpoint_reference = (
            *self.reference[:50],
            replace(self.reference[50], ref_id="pickup_front"),
        )
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=checkpoint_reference,
            current_pose=checkpoint_reference[-1],
            dynamic_obstacle_points=(),
            previous_reference_index=len(checkpoint_reference) - 2,
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertEqual(result.reference_start_index, 49)
        self.assertEqual(result.rejoin_index, 50)
        self.assertEqual(result.path, checkpoint_reference[-2:])

    def test_checkpoint_endpoint_does_not_bypass_collision_validation(self) -> None:
        checkpoint_reference = (
            *self.reference[:50],
            replace(self.reference[50], ref_id="pickup_front"),
        )
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        with self.assertRaises(GridPlanningError):
            planner.plan(
                reference_path=checkpoint_reference,
                current_pose=checkpoint_reference[-1],
                dynamic_obstacle_points=((5.0, 0.0),),
                previous_reference_index=len(checkpoint_reference) - 2,
            )

    def test_random_obstacle_segment_uses_rolling_goal_without_reusing_blue_line(
        self,
    ) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.10,
                y=0.0,
                yaw=0.0,
                ref_id={
                    40: "random_obstacle_entry",
                    70: "random_obstacle_exit",
                }.get(index),
            )
            for index in range(81)
        )
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )

        self.assertEqual(result.status, "RELAXED_REPLANNED")
        self.assertEqual(result.reference_start_index, 20)
        self.assertEqual(result.rejoin_index, 50)
        self.assertLess(
            math.hypot(
                result.path[-1].x - reference[50].x,
                result.path[-1].y - reference[50].y,
            ),
            0.16,
        )
        self.assertNotEqual(result.path, reference[20:61])

    def test_random_obstacle_segment_uses_relaxed_heading_tolerance(self) -> None:
        config = _relaxed_config(
            goal_heading_tolerance_rad=math.radians(8.0),
            relaxed_goal_heading_tolerance_rad=math.radians(20.0),
        )
        planner = LocalTrajectoryPlanner(_empty_map(), config)

        strict = planner._planner(_empty_map(), self.reference, relaxed=False)
        relaxed = planner._planner(_empty_map(), self.reference, relaxed=True)

        self.assertAlmostEqual(
            strict._goal_heading_tolerance_rad,
            math.radians(8.0),
        )
        self.assertAlmostEqual(
            relaxed._goal_heading_tolerance_rad,
            math.radians(20.0),
        )

    def test_random_obstacle_segment_holds_an_equally_good_safe_path(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )

        advanced_pose = first.path[3]
        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=AssertionError("safe held path must skip fresh search"),
        ):
            second = planner.plan(
                reference_path=reference,
                current_pose=advanced_pose,
                dynamic_obstacle_points=(),
                previous_reference_index=20,
            )

        self.assertEqual(first.status, "RELAXED_REPLANNED")
        self.assertEqual(second.status, "RELAXED_HOLD")
        self.assertEqual(second.path, first.path[3:])

    def test_relaxed_hold_replans_when_pose_splice_breaks_turning_radius(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )
        held_point = first.path[3]
        offset_pose = PathPoint(
            held_point.x,
            held_point.y + 0.05,
            held_point.yaw,
            held_point.ref_id,
        )

        second = planner.plan(
            reference_path=reference,
            current_pose=offset_pose,
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )

        self.assertEqual(second.status, "RELAXED_REPLANNED")
        trajectory = parameterize_local_path(
            second.path,
            semantic_map={"frame_id": "map"},
            optimizer_config={
                "trajectory_optimizer": {
                    "max_curvature_1pm": 1.0 / 0.60,
                    "curvature_overshoot_tolerance_ratio": 0.0,
                }
            },
        )
        self.assertLessEqual(
            max(abs(point.curvature) for point in trajectory.points),
            1.0 / 0.60 + 1e-9,
        )

    def test_relaxed_policy_holds_safe_path_when_replan_has_no_forward_solution(
        self,
    ) -> None:
        reference = _relaxed_reference()
        obstacles = tuple(
            (2.25 + x_index * 0.05, -0.3875 + y_index * 0.05)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.30 + 1e-9
        )
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[0],
            dynamic_obstacle_points=obstacles,
        )
        held_index = 15
        held_point = first.path[held_index]
        angled_pose = PathPoint(
            held_point.x,
            held_point.y,
            math.radians(-7.0),
            held_point.ref_id,
        )

        with self.assertRaises(GridPlanningError):
            LocalTrajectoryPlanner(_empty_map(), _relaxed_config()).plan(
                reference_path=reference,
                current_pose=angled_pose,
                dynamic_obstacle_points=obstacles,
                previous_reference_index=first.reference_start_index,
            )

        second = planner.plan(
            reference_path=reference,
            current_pose=angled_pose,
            dynamic_obstacle_points=obstacles,
            previous_reference_index=first.reference_start_index,
        )

        self.assertEqual(second.status, "RELAXED_HOLD")
        self.assertEqual(second.path[0], angled_pose)
        self.assertEqual(second.path[1:], first.path[held_index + 1 :])
        self.assertTrue(second.path_is_navigable)

        blocking_wall = tuple(
            (1.80 + x_offset, y_index * 0.05)
            for x_offset in (-0.05, 0.0, 0.05)
            for y_index in range(-20, 21)
        )
        with self.assertRaises(GridPlanningError):
            planner.plan(
                reference_path=reference,
                current_pose=angled_pose,
                dynamic_obstacle_points=blocking_wall,
                previous_reference_index=second.reference_start_index,
            )

    def test_random_obstacle_segment_replans_when_held_path_becomes_blocked(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )

        second = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=((4.50, 0.0),),
            previous_reference_index=20,
        )

        self.assertEqual(first.status, "RELAXED_REPLANNED")
        self.assertEqual(second.status, "RELAXED_REPLANNED")
        self.assertNotEqual(second.path, first.path)
        self.assertGreater(max(abs(point.y) for point in second.path), 0.10)

    def test_relaxed_policy_does_not_change_tracking_before_activation_window(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(
            _empty_map(),
            _relaxed_config(lookahead_distance_m=1.0),
        )

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[0],
            dynamic_obstacle_points=(),
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertEqual(result.path, reference[:11])

    def test_planning_horizon_activates_relaxed_approach_before_obstacle_zone(
        self,
    ) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.10,
                y=0.0,
                yaw=0.0,
                ref_id={
                    37: "random_obstacle_entry",
                    67: "random_obstacle_exit",
                }.get(index),
            )
            for index in range(81)
        )
        obstacles = tuple(
            (2.80 + x_index * 0.05, y_index * 0.05)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.30 + 1e-9
        )
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[0],
            dynamic_obstacle_points=obstacles,
        )

        self.assertEqual(result.status, "RELAXED_REPLANNED")
        self.assertEqual(result.rejoin_index, 67)
        self.assertTrue(result.path_is_navigable)

    def test_relaxed_policy_applies_to_the_second_route_lap(self) -> None:
        reference = _relaxed_reference(repeated=True)
        planner = LocalTrajectoryPlanner(_empty_map(width=180), _relaxed_config())

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[80],
            dynamic_obstacle_points=(),
            previous_reference_index=80,
        )

        self.assertEqual(result.status, "RELAXED_REPLANNED")
        self.assertEqual(result.reference_start_index, 80)
        self.assertEqual(result.rejoin_index, 120)

    def test_relaxed_exit_extends_before_local_controller_reaches_goal(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())

        result = planner.plan(
            reference_path=reference,
            current_pose=PathPoint(5.85, 0.0, 0.0),
            dynamic_obstacle_points=(),
            previous_reference_index=58,
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertGreater(result.rejoin_index, 60)
        self.assertGreater(len(result.path), 2)

    def test_relaxed_exit_waits_for_pose_alignment_and_clear_reference(self) -> None:
        reference = _relaxed_reference()
        obstacles = tuple(
            (6.80 + x_index * 0.05, y_index * 0.05)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.30 + 1e-9
        )
        planner = LocalTrajectoryPlanner(
            _empty_map(),
            _relaxed_config(planning_timeout_s=5.0),
        )

        approach = planner.plan(
            reference_path=reference,
            current_pose=reference[50],
            dynamic_obstacle_points=obstacles,
            previous_reference_index=50,
        )
        result = planner.plan(
            reference_path=reference,
            current_pose=approach.path[9],
            dynamic_obstacle_points=obstacles,
            previous_reference_index=58,
        )

        self.assertGreater(approach.rejoin_index, 60)
        self.assertTrue(result.status.startswith("RELAXED_"))
        self.assertGreater(result.rejoin_index, 60)
        self.assertTrue(result.path_is_navigable)

    def test_relaxed_exit_accepts_small_recovery_offset_on_clear_route(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        planner._pending_relaxed_span = (30, 60)

        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=AssertionError("clear exit must not invoke Hybrid A*"),
        ):
            result = planner.plan(
                reference_path=reference,
                current_pose=PathPoint(6.50, 0.20, 0.0),
                dynamic_obstacle_points=(),
                previous_reference_index=65,
            )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertIsNone(planner._pending_relaxed_span)

    def test_timeout_reuses_last_collision_free_trajectory(self) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.40,
                y=0.0,
                yaw=0.0,
                ref_id={5: "random_obstacle_entry", 10: "random_obstacle_exit"}.get(
                    index
                ),
            )
            for index in range(30)
        )
        planner = LocalTrajectoryPlanner(
            _empty_map(width=140),
            _relaxed_config(),
        )
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[15],
            dynamic_obstacle_points=(),
            previous_reference_index=15,
        )
        planner._pending_relaxed_span = (5, 10)

        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=HybridAStarTimeout("synthetic post-exit timeout"),
        ):
            second = planner.plan(
                reference_path=reference,
                current_pose=PathPoint(6.0, 0.35, 0.0),
                dynamic_obstacle_points=(),
                previous_reference_index=first.reference_start_index,
            )

        self.assertEqual(second.status, "PLANNING_TIMEOUT_HOLD")
        self.assertEqual(second.path[0], PathPoint(6.0, 0.35, 0.0))
        self.assertTrue(second.path_is_navigable)

    def test_timeout_rejects_last_trajectory_blocked_by_new_obstacle(self) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.40,
                y=0.0,
                yaw=0.0,
                ref_id={5: "random_obstacle_entry", 10: "random_obstacle_exit"}.get(
                    index
                ),
            )
            for index in range(30)
        )
        planner = LocalTrajectoryPlanner(
            _empty_map(width=140),
            _relaxed_config(),
        )
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[15],
            dynamic_obstacle_points=(),
            previous_reference_index=15,
        )
        planner._pending_relaxed_span = (5, 10)

        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=HybridAStarTimeout("synthetic blocked hold timeout"),
        ), self.assertRaises(HybridAStarTimeout):
            planner.plan(
                reference_path=reference,
                current_pose=PathPoint(6.0, 0.35, 0.0),
                dynamic_obstacle_points=((6.40, 0.0),),
                previous_reference_index=first.reference_start_index,
            )

    def test_relaxed_checkpoint_timeout_keeps_validated_safe_tail(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )
        obstacle_after_exit = tuple(
            (6.80 + x_index * 0.05, y_index * 0.05)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.30 + 1e-9
        )
        advanced_pose = first.path[-2]

        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            autospec=True,
            side_effect=HybridAStarTimeout("synthetic extension timeout"),
        ) as plan_mock:
            second = planner.plan(
                reference_path=reference,
                current_pose=advanced_pose,
                dynamic_obstacle_points=obstacle_after_exit,
                previous_reference_index=first.reference_start_index,
            )

        self.assertEqual(second.status, "RELAXED_HOLD")
        self.assertEqual(second.rejoin_index, first.rejoin_index)
        self.assertEqual(second.path[0], advanced_pose)
        self.assertTrue(second.path_is_navigable)
        self.assertEqual(
            plan_mock.call_args.args[0]._planning_timeout_s,
            _relaxed_config().relaxed_extension_timeout_s,
        )

    def test_relaxed_extension_uses_smaller_curvature_lattice(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())

        strict = planner._planner(_empty_map(), _relaxed_reference(), relaxed=False)
        relaxed = planner._planner(_empty_map(), _relaxed_reference(), relaxed=True)
        extension = planner._planner(
            _empty_map(),
            _relaxed_reference(),
            relaxed=True,
            planning_timeout_s=_relaxed_config().relaxed_extension_timeout_s,
            reduced_curvature_lattice=True,
        )

        self.assertEqual(len(strict._curvatures), 9)
        self.assertEqual(len(relaxed._curvatures), 9)
        self.assertEqual(len(extension._curvatures), 7)

    def test_relaxed_exit_transition_commits_only_after_strict_plan_succeeds(
        self,
    ) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )
        distant_obstacle = tuple(
            (7.50 + x_index * 0.05, y_index * 0.05)
            for x_index in range(-4, 5)
            for y_index in range(-4, 5)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.20 + 1e-9
        )
        advanced_pose = first.path[-2]

        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=HybridAStarTimeout("synthetic strict transition timeout"),
        ):
            result = planner.plan(
                reference_path=reference,
                current_pose=advanced_pose,
                dynamic_obstacle_points=distant_obstacle,
                previous_reference_index=58,
            )

        self.assertEqual(result.status, "RELAXED_HOLD")
        self.assertEqual(result.rejoin_index, first.rejoin_index)
        self.assertTrue(result.path_is_navigable)

    def test_relaxed_blocked_exit_rejoins_after_stable_clear_run(self) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.10,
                y=0.0,
                yaw=0.0,
                ref_id={
                    40: "random_obstacle_entry",
                    70: "random_obstacle_exit",
                }.get(index),
            )
            for index in range(101)
        )
        obstacles = tuple(
            (6.30 + x_index * 0.05, y_index * 0.05)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(x_index * 0.05, y_index * 0.05) <= 0.30 + 1e-9
        )
        planner = LocalTrajectoryPlanner(
            _empty_map(width=120),
            _relaxed_config(),
        )

        result = planner.plan(
            reference_path=reference,
            current_pose=PathPoint(3.90, 0.70, 0.10),
            dynamic_obstacle_points=obstacles,
            previous_reference_index=39,
        )

        self.assertEqual(result.status, "RELAXED_REPLANNED")
        self.assertEqual(result.reference_start_index, 39)
        self.assertEqual(result.rejoin_index, 79)
        self.assertTrue(result.path_is_navigable)

    def test_relaxed_approach_checkpoint_extends_before_goal_reached(self) -> None:
        reference = tuple(
            PathPoint(
                x=index * 0.10,
                y=0.0,
                yaw=0.0,
                ref_id={
                    40: "random_obstacle_entry",
                    70: "random_obstacle_exit",
                }.get(index),
            )
            for index in range(91)
        )
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[20],
            dynamic_obstacle_points=(),
            previous_reference_index=20,
        )

        second = planner.plan(
            reference_path=reference,
            current_pose=PathPoint(4.85, 0.0, 0.0),
            dynamic_obstacle_points=(),
            previous_reference_index=48,
        )

        self.assertEqual(first.rejoin_index, 50)
        self.assertEqual(second.rejoin_index, 70)
        self.assertGreater(len(second.path), 2)

    def test_relaxed_path_rejoins_early_after_blockage_is_behind(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        obstacle_center = reference[38]
        obstacles = tuple(
            (obstacle_center.x + dx, obstacle_center.y + dy)
            for x_index in range(-5, 6)
            for y_index in range(-5, 6)
            if math.hypot(
                (dx := x_index * 0.05),
                (dy := y_index * 0.05),
            )
            <= 0.25 + 1e-9
        )
        first = planner.plan(
            reference_path=reference,
            current_pose=reference[25],
            dynamic_obstacle_points=obstacles,
            previous_reference_index=25,
        )
        self.assertEqual(first.rejoin_index, 60)
        before_trigger_planner = LocalTrajectoryPlanner(
            _empty_map(),
            _relaxed_config(),
        )
        before_trigger_planner.plan(
            reference_path=reference,
            current_pose=reference[25],
            dynamic_obstacle_points=obstacles,
            previous_reference_index=25,
        )
        before_trigger = before_trigger_planner.plan(
            reference_path=reference,
            current_pose=reference[35],
            dynamic_obstacle_points=(),
            previous_reference_index=35,
        )
        self.assertNotEqual(before_trigger.status, "EARLY_REJOIN_SPLICED")
        self.assertEqual(before_trigger.rejoin_index, 60)
        advanced_pose = first.path[-15]
        with patch(
            "competition_planning.local_trajectory_planner."
            "HybridAStarPlanner.plan",
            side_effect=AssertionError(
                "safe early rejoin must not run a blocking Hybrid A* search"
            ),
        ):
            rejoin = planner.plan(
                reference_path=reference,
                current_pose=advanced_pose,
                dynamic_obstacle_points=(),
                previous_reference_index=45,
            )

        self.assertEqual(rejoin.status, "EARLY_REJOIN_SPLICED")
        self.assertTrue(
            any(reference[index] in rejoin.path for index in range(46, 60))
        )
        resumed = planner.plan(
            reference_path=reference,
            current_pose=rejoin.path[-1],
            dynamic_obstacle_points=(),
            previous_reference_index=rejoin.rejoin_index,
        )
        self.assertEqual(resumed.status, "REFERENCE_CLEAR")
        self.assertGreater(resumed.rejoin_index, 60)

    def test_early_rejoin_deduplicates_an_exact_reference_join(self) -> None:
        reference = _relaxed_reference()
        planner = LocalTrajectoryPlanner(_empty_map(), _relaxed_config())
        collision_planner = planner._planner(  # noqa: SLF001 - regression seam
            _empty_map(),
            reference,
        )

        spliced = planner._splice_held_path_to_reference(  # noqa: SLF001
            held_path=(reference[1], reference[2]),
            planner=collision_planner,
            reference_path=reference,
            start_index=1,
            segment_exit_index=6,
        )

        self.assertTrue(spliced)
        self.assertTrue(
            all(
                math.hypot(following.x - previous.x, following.y - previous.y)
                > 1e-9
                for previous, following in zip(spliced, spliced[1:])
            )
        )
        trajectory = parameterize_local_path(
            spliced,
            semantic_map={"frame_id": "map"},
            optimizer_config={
                "trajectory_optimizer": {
                    "max_speed_mps": 0.20,
                    "max_acceleration_mps2": 0.20,
                    "max_deceleration_mps2": 0.30,
                    "max_lateral_acceleration_mps2": 0.20,
                }
            },
        )
        self.assertTrue(
            all(
                following.s > previous.s and following.t > previous.t
                for previous, following in zip(
                    trajectory.points,
                    trajectory.points[1:],
                )
            )
        )

    def test_soft_obstacle_edge_cost_increases_clearance(self) -> None:
        reference = _relaxed_reference()
        obstacle_center = reference[42]
        obstacles = tuple(
            (obstacle_center.x + dx, obstacle_center.y + dy)
            for x_index in range(-5, 6)
            for y_index in range(-5, 6)
            if math.hypot(
                (dx := x_index * 0.05),
                (dy := y_index * 0.05),
            )
            <= 0.25 + 1e-9
        )

        def planned_clearance(weight: float) -> float:
            planner = LocalTrajectoryPlanner(
                _empty_map(),
                _relaxed_config(
                    obstacle_clearance_distance_m=0.20,
                    obstacle_clearance_weight=weight,
                ),
            )
            result = planner.plan(
                reference_path=reference,
                current_pose=reference[20],
                dynamic_obstacle_points=obstacles,
                previous_reference_index=20,
            )
            return min(
                math.hypot(
                    point.x - obstacle_center.x,
                    point.y - obstacle_center.y,
                )
                for point in result.path
            )

        baseline_clearance = planned_clearance(0.0)
        softened_clearance = planned_clearance(4.0)

        self.assertGreaterEqual(
            softened_clearance,
            baseline_clearance + 0.06,
        )

    def test_inflated_costmap_cells_are_bounded_and_downsampled(self) -> None:
        points = occupied_grid_cell_centers(
            (100, 100, 100, 100, 100, 100),
            width=3,
            height=2,
            resolution_m=0.10,
            origin_x_m=0.0,
            origin_y_m=-0.10,
            occupancy_threshold=50,
            x_min_m=0.05,
            x_max_m=0.25,
            y_half_width_m=0.10,
            max_points=2,
        )

        self.assertEqual(len(points), 2)
        self.assertTrue(all(0.05 <= x <= 0.25 for x, _ in points))
        self.assertTrue(all(abs(y) <= 0.10 for _, y in points))

    def test_raw_costmap_threshold_excludes_inflation_only_cells(self) -> None:
        points = occupied_grid_cell_centers(
            (50, 100),
            width=2,
            height=1,
            resolution_m=0.10,
            origin_x_m=0.0,
            origin_y_m=-0.05,
            occupancy_threshold=100,
            x_min_m=0.0,
            x_max_m=0.20,
            y_half_width_m=0.10,
            max_points=2,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.15)
        self.assertAlmostEqual(points[0][1], 0.0)

    def test_day5_runtime_horizon_can_replan_around_a_route_obstacle(self) -> None:
        runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "local_hybrid_astar_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["local_hybrid_astar_runtime"]
        control = yaml.safe_load(
            (
                REPO_ROOT / "config" / "control" / "control_params.yaml"
            ).read_text(encoding="utf-8")
        )
        trajectory_data = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_control_validation_to_drop_pass_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point["yaw"]),
                ref_id=point.get("ref_id"),
            )
            for point in trajectory_data["points"]
        )
        planner = LocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / "maps" / "debug" / "map.yaml"),
            LocalReplanConfig(
                lookahead_distance_m=runtime["lookahead_distance_m"],
                inflation_radius_m=runtime["inflation_radius_m"],
                search_padding_m=runtime["search_padding_m"],
                sample_spacing_m=runtime["sample_spacing_m"],
                min_turning_radius_m=control["motion"]["min_turning_radius_m"],
                step_length_m=runtime["step_length_m"],
                curvature_bins=runtime["curvature_bins"],
                heading_bins=runtime["heading_bins"],
                goal_position_tolerance_m=runtime[
                    "goal_position_tolerance_m"
                ],
                goal_heading_tolerance_rad=math.radians(
                    runtime["goal_heading_tolerance_deg"]
                ),
                reference_deviation_weight=runtime[
                    "reference_deviation_weight"
                ],
                max_expansions=runtime["max_expansions"],
                reference_search_window_points=runtime[
                    "reference_search_window_points"
                ],
            ),
        )
        obstacle_center = reference[14]
        obstacles = tuple(
            (obstacle_center.x + dx, obstacle_center.y + dy)
            for dx in (-0.10, -0.05, 0.0, 0.05, 0.10)
            for dy in (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15)
        )

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[0],
            dynamic_obstacle_points=obstacles,
        )

        self.assertEqual(result.status, "REPLANNED")
        self.assertTrue(result.path_is_navigable)
        self.assertEqual(result.dynamic_obstacle_count, len(obstacles))

    def test_day5_random_obstacle_zone_uses_relaxed_checkpoint_policy(self) -> None:
        runtime = yaml.safe_load(
            (
                REPO_ROOT
                / "config"
                / "planning"
                / "local_hybrid_astar_runtime_params_day5.yaml"
            ).read_text(encoding="utf-8")
        )["local_hybrid_astar_runtime"]
        control = yaml.safe_load(
            (REPO_ROOT / "config" / "control" / "control_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        trajectory_data = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_control_validation_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point["yaw"]),
                ref_id=point.get("ref_id"),
            )
            for point in trajectory_data["points"]
        )
        entry_index = next(
            index
            for index, point in enumerate(reference)
            if point.ref_id == runtime["relaxed_segment_entry_ref"]
        )
        exit_index = next(
            index
            for index in range(entry_index + 1, len(reference))
            if reference[index].ref_id == runtime["relaxed_segment_exit_ref"]
        )
        start_index = entry_index
        distance_to_entry = 0.0
        while start_index > 0 and distance_to_entry < 0.90:
            previous = reference[start_index - 1]
            current = reference[start_index]
            distance_to_entry += math.hypot(
                current.x - previous.x,
                current.y - previous.y,
            )
            start_index -= 1
        obstacle_center = reference[(entry_index + exit_index) // 2]
        obstacles = tuple(
            (obstacle_center.x + dx, obstacle_center.y + dy)
            for x_index in range(-6, 7)
            for y_index in range(-6, 7)
            if math.hypot(
                (dx := x_index * 0.05),
                (dy := y_index * 0.05),
            )
            <= 0.30 + 1e-9
        )
        planner = LocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / "maps" / "debug" / "map.yaml"),
            LocalReplanConfig(
                lookahead_distance_m=runtime["lookahead_distance_m"],
                inflation_radius_m=runtime["inflation_radius_m"],
                search_padding_m=runtime["search_padding_m"],
                sample_spacing_m=runtime["sample_spacing_m"],
                min_turning_radius_m=control["motion"]["min_turning_radius_m"],
                step_length_m=runtime["step_length_m"],
                curvature_bins=runtime["curvature_bins"],
                heading_bins=runtime["heading_bins"],
                goal_position_tolerance_m=runtime["goal_position_tolerance_m"],
                goal_heading_tolerance_rad=math.radians(
                    runtime["goal_heading_tolerance_deg"]
                ),
                reference_deviation_weight=runtime["reference_deviation_weight"],
                max_expansions=runtime["max_expansions"],
                planning_timeout_s=runtime["planning_timeout_s"],
                reference_search_window_points=runtime[
                    "reference_search_window_points"
                ],
                relaxed_segment_entry_ref=runtime["relaxed_segment_entry_ref"],
                relaxed_segment_exit_ref=runtime["relaxed_segment_exit_ref"],
                relaxed_activation_distance_m=runtime[
                    "relaxed_activation_distance_m"
                ],
                relaxed_reference_deviation_weight=runtime[
                    "relaxed_reference_deviation_weight"
                ],
                relaxed_corridor_half_width_m=runtime[
                    "relaxed_corridor_half_width_m"
                ],
                relaxed_step_length_m=runtime["relaxed_step_length_m"],
                relaxed_goal_heading_tolerance_rad=math.radians(
                    runtime["relaxed_goal_heading_tolerance_deg"]
                ),
                trajectory_switch_improvement_ratio=runtime[
                    "trajectory_switch_improvement_ratio"
                ],
                obstacle_clearance_distance_m=runtime[
                    "obstacle_clearance_distance_m"
                ],
                obstacle_clearance_weight=runtime["obstacle_clearance_weight"],
                search_heuristic_weight=runtime["search_heuristic_weight"],
            ),
        )

        result = planner.plan(
            reference_path=reference,
            current_pose=reference[start_index],
            dynamic_obstacle_points=obstacles,
            previous_reference_index=start_index,
        )

        self.assertEqual(result.status, "RELAXED_REPLANNED")
        self.assertGreater(result.rejoin_index, exit_index)
        self.assertTrue(result.path_is_navigable)
        local_reference = reference[start_index : result.rejoin_index + 1]
        self.assertGreaterEqual(
            min(
                math.hypot(
                    point.x - obstacle_center.x,
                    point.y - obstacle_center.y,
                )
                for point in result.path
            ),
            0.45,
        )
        self.assertLessEqual(
            max(
                min(
                    math.hypot(point.x - ref.x, point.y - ref.y)
                    for ref in local_reference
                )
                for point in result.path
            ),
            0.86,
        )

    def test_clear_day5_turn_reference_is_kept_after_small_tracking_offset(self) -> None:
        trajectory_data = yaml.safe_load(
            (
                REPO_ROOT
                / "docs"
                / "evidence"
                / "day5"
                / "debug_control_validation_to_drop_pass_trajectory.yaml"
            ).read_text(encoding="utf-8")
        )
        reference = tuple(
            PathPoint(
                float(point["x"]),
                float(point["y"]),
                float(point["yaw"]),
            )
            for point in trajectory_data["points"]
        )
        planner = LocalTrajectoryPlanner(
            OccupancyGridMap.from_yaml(REPO_ROOT / "maps" / "debug" / "map.yaml"),
            LocalReplanConfig(
                lookahead_distance_m=3.0,
                inflation_radius_m=0.45,
                search_padding_m=3.0,
                sample_spacing_m=0.10,
                min_turning_radius_m=0.81,
                step_length_m=0.20,
                curvature_bins=9,
                heading_bins=72,
                goal_position_tolerance_m=0.25,
                goal_heading_tolerance_rad=math.radians(15.0),
                reference_deviation_weight=2.0,
                max_expansions=250_000,
                reference_search_window_points=120,
            ),
        )

        result = planner.plan(
            reference_path=reference,
            current_pose=PathPoint(
                6.6946821194394195,
                0.3988649623402306,
                0.02126558917458654,
            ),
            dynamic_obstacle_points=(),
            previous_reference_index=70,
        )

        self.assertEqual(result.status, "REFERENCE_CLEAR")
        self.assertEqual(result.reference_start_index, 71)
        self.assertEqual(result.rejoin_index, 102)
        self.assertEqual(result.path, reference[71:103])

    def test_table_is_avoided_before_rejoining_global_reference(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=_table_points(),
        )

        self.assertEqual(result.status, "REPLANNED")
        self.assertEqual(result.rejoin_index, 50)
        self.assertEqual(result.dynamic_obstacle_count, len(_table_points()))
        self.assertTrue(result.path_is_navigable)
        self.assertGreater(max(abs(point.y) for point in result.path), 0.70)
        self.assertLess(max(abs(point.y) for point in result.path), 1.50)
        self.assertLess(math.hypot(result.path[-1].x - 5.0, result.path[-1].y), 0.26)
        self.assertLess(abs(result.path[-1].yaw), math.radians(15.1))

    def test_blocked_rejoin_point_uses_nearby_global_reference_target(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)

        result = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=((5.0, 0.0),),
        )

        self.assertEqual(result.status, "REPLANNED")
        self.assertGreater(result.rejoin_index, 50)
        self.assertTrue(result.path_is_navigable)
        self.assertLess(
            math.hypot(
                result.path[-1].x - self.reference[result.rejoin_index].x,
                result.path[-1].y - self.reference[result.rejoin_index].y,
            ),
            0.26,
        )

    def test_replanned_geometry_reuses_day4_time_parameterization(self) -> None:
        planner = LocalTrajectoryPlanner(_empty_map(), self.config)
        local_plan = planner.plan(
            reference_path=self.reference,
            current_pose=self.reference[0],
            dynamic_obstacle_points=_table_points(),
        )

        trajectory = parameterize_local_path(
            local_plan.path,
            semantic_map={"frame_id": "map"},
            optimizer_config={
                "trajectory_optimizer": {
                    "max_speed_mps": 0.20,
                    "max_acceleration_mps2": 0.20,
                    "max_deceleration_mps2": 0.30,
                    "max_lateral_acceleration_mps2": 0.20,
                }
            },
        )

        self.assertEqual(len(trajectory.points), len(local_plan.path))
        self.assertTrue(
            all(
                following.s > previous.s and following.t > previous.t
                for previous, following in zip(
                    trajectory.points,
                    trajectory.points[1:],
                )
            )
        )
        self.assertLessEqual(
            max(abs(point.curvature) for point in trajectory.points),
            1.0 / 0.81 + 0.02,
        )
        self.assertLessEqual(max(point.v for point in trajectory.points), 0.20)

    def test_checkpoint_annotation_stops_without_ending_continuous_path(self) -> None:
        path = tuple(
            PathPoint(
                x=index * 0.10,
                y=0.0,
                yaw=0.0,
                ref_id="traffic_light_stop_line" if index == 10 else None,
            )
            for index in range(21)
        )

        trajectory = parameterize_local_path(
            path,
            semantic_map={
                "frame_id": "map",
                "stop_lines": [
                    {"point_ref": "traffic_light_stop_line"},
                ],
            },
            optimizer_config={
                "trajectory_optimizer": {
                    "max_speed_mps": 0.20,
                    "max_acceleration_mps2": 0.20,
                    "max_deceleration_mps2": 0.30,
                    "max_lateral_acceleration_mps2": 0.20,
                }
            },
        )

        self.assertEqual(trajectory.points[10].v, 0.0)
        self.assertGreater(max(point.v for point in trajectory.points[11:-1]), 0.0)

    def test_bagged_local_path_with_minor_curvature_overshoot_is_mppi_acceptable(self) -> None:
        path = (
            PathPoint(6.6183, 0.3383, 0.0200),
            PathPoint(6.7183, 0.3403, 0.0200),
            PathPoint(6.8183, 0.3423, 0.0200),
            PathPoint(6.9183, 0.3443, 0.0200),
            PathPoint(7.0183, 0.3463, 0.0200),
            PathPoint(7.1182, 0.3483, 0.0200),
            PathPoint(7.2182, 0.3503, 0.0200),
            PathPoint(7.3182, 0.3523, 0.0200),
            PathPoint(7.4182, 0.3543, 0.0200),
            PathPoint(7.5182, 0.3563, 0.0200),
            PathPoint(7.6181, 0.3583, 0.0200),
            PathPoint(7.7181, 0.3603, 0.0200),
            PathPoint(7.8181, 0.3623, 0.0200),
            PathPoint(7.9181, 0.3643, 0.0200),
            PathPoint(8.0181, 0.3663, 0.0200),
            PathPoint(8.1180, 0.3683, 0.0200),
            PathPoint(8.2180, 0.3703, 0.0200),
            PathPoint(8.3180, 0.3723, 0.0200),
            PathPoint(8.4180, 0.3743, 0.0200),
            PathPoint(8.5180, 0.3763, 0.0200),
            PathPoint(8.6179, 0.3783, 0.0200),
            PathPoint(8.7179, 0.3803, 0.0200),
            PathPoint(8.8179, 0.3823, 0.0200),
            PathPoint(8.9178, 0.3858, 0.0509),
            PathPoint(9.0176, 0.3925, 0.0817),
            PathPoint(9.1170, 0.4037, 0.1435),
            PathPoint(9.2154, 0.4210, 0.2052),
            PathPoint(9.3122, 0.4459, 0.2978),
            PathPoint(9.4063, 0.4796, 0.3904),
            PathPoint(9.4962, 0.5233, 0.5138),
            PathPoint(9.5801, 0.5777, 0.6373),
            PathPoint(9.6566, 0.6420, 0.7607),
        )
        runtime_curvature_limit = 1.0 / 0.81

        trajectory = parameterize_local_path(
            path,
            semantic_map={"frame_id": "map"},
            optimizer_config={
                "trajectory_optimizer": {
                    "max_speed_mps": 0.15,
                    "max_acceleration_mps2": 0.20,
                    "max_deceleration_mps2": 0.30,
                    "max_lateral_acceleration_mps2": 0.20,
                    "max_curvature_1pm": runtime_curvature_limit,
                }
            },
        )
        control_trajectory = ControlTrajectory(
            frame_id="map",
            route_name="bagged_local_replan",
            points=tuple(
                ControlTrajectoryPoint(
                    point.x,
                    point.y,
                    point.yaw,
                    point.s,
                    point.curvature,
                    point.v,
                    point.t,
                    point.ref_id,
                )
                for point in trajectory.points
            ),
        )

        MPPIController(
            control_trajectory,
            MPPIParams(min_turning_radius_m=0.81),
            random_seed=31,
        )
        self.assertLessEqual(
            max(abs(point.curvature) for point in trajectory.points),
            runtime_curvature_limit + 1e-9,
        )

    def test_local_path_far_outside_runtime_curvature_envelope_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "turning-radius envelope"):
            parameterize_local_path(
                (
                    PathPoint(0.0, 0.0, 0.0),
                    PathPoint(0.1, 0.0, 0.0),
                    PathPoint(0.1, 0.1, math.pi / 2.0),
                ),
                semantic_map={"frame_id": "map"},
                optimizer_config={
                    "trajectory_optimizer": {
                        "max_speed_mps": 0.15,
                        "max_acceleration_mps2": 0.20,
                        "max_deceleration_mps2": 0.30,
                        "max_lateral_acceleration_mps2": 0.20,
                        "max_curvature_1pm": 1.0,
                        "curvature_overshoot_tolerance_ratio": 0.0,
                    }
                },
            )


if __name__ == "__main__":
    unittest.main()
