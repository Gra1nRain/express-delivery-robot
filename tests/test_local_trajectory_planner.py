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
from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.hybrid_astar_planner import HybridAStarPlanner
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint
from competition_planning.trajectory_parameterizer import parameterize_local_path


def _empty_map() -> OccupancyGridMap:
    width = 100
    height = 80
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=0.10,
        origin_x=-1.0,
        origin_y=-4.0,
        occupied=tuple(False for _ in range(width * height)),
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


class LocalTrajectoryPlannerTest(unittest.TestCase):
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

    def test_hybrid_search_honors_local_wall_clock_timeout(self) -> None:
        planner = HybridAStarPlanner(
            _empty_map(),
            inflation_radius_m=0.30,
            search_padding_m=1.5,
            sample_spacing_m=0.10,
            min_turning_radius_m=0.81,
            step_length_m=0.20,
            curvature_bins=9,
            heading_bins=72,
            goal_position_tolerance_m=0.25,
            goal_heading_tolerance_rad=math.radians(15.0),
            max_expansions=250_000,
            planning_timeout_s=1.5,
        )

        with patch(
            "competition_planning.hybrid_astar_planner.time.perf_counter",
            side_effect=(10.0, 11.6),
        ):
            with self.assertRaisesRegex(
                GridPlanningError,
                "planning timeout",
            ):
                planner.plan((PathPoint(0.0, 0.0, 0.0), PathPoint(5.0, 0.0, 0.0)))

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
