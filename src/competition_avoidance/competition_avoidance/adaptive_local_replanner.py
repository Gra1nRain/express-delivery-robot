"""Adaptive lookahead seam around the existing local trajectory planner."""

from __future__ import annotations

from dataclasses import replace
from typing import Callable, Iterable

from competition_planning.local_trajectory_planner import (
    LocalPlan,
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


PlannerFactory = Callable[
    [OccupancyGridMap, LocalReplanConfig],
    LocalTrajectoryPlanner,
]


class AdaptiveLocalTrajectoryPlanner:
    """Try a short horizon first, then a longer safe rejoin horizon."""

    def __init__(
        self,
        static_map: OccupancyGridMap,
        primary_config: LocalReplanConfig,
        fallback_lookahead_distances_m: Iterable[float],
        *,
        planner_factory: PlannerFactory = LocalTrajectoryPlanner,
    ) -> None:
        lookaheads = [primary_config.lookahead_distance_m]
        for distance in fallback_lookahead_distances_m:
            value = float(distance)
            if value <= 0.0:
                raise ValueError("fallback lookahead distances must be positive")
            if value not in lookaheads:
                lookaheads.append(value)
        self._attempts = tuple(
            (
                lookahead,
                planner_factory(
                    static_map,
                    replace(primary_config, lookahead_distance_m=lookahead),
                ),
            )
            for lookahead in lookaheads
        )
        self.last_selected_lookahead_distance_m: float | None = None

    def plan(
        self,
        *,
        reference_path: tuple[PathPoint, ...],
        current_pose: PathPoint,
        dynamic_obstacle_points: Iterable[tuple[float, float]],
        previous_reference_index: int = 0,
    ) -> LocalPlan:
        failures: list[str] = []
        obstacle_points = tuple(dynamic_obstacle_points)
        for lookahead, planner in self._attempts:
            try:
                result = planner.plan(
                    reference_path=reference_path,
                    current_pose=current_pose,
                    dynamic_obstacle_points=obstacle_points,
                    previous_reference_index=previous_reference_index,
                )
            except GridPlanningError as exc:
                failures.append(f"{lookahead:.2f}m: {exc}")
                continue
            self.last_selected_lookahead_distance_m = lookahead
            return result
        self.last_selected_lookahead_distance_m = None
        raise GridPlanningError(
            "adaptive local replanning exhausted lookaheads: " + " | ".join(failures)
        )
