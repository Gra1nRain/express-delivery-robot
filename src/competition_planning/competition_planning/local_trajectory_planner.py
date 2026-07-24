"""Reference-aware short-horizon replanning over static and live obstacles."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from competition_planning.hybrid_astar_planner import HybridAStarPlanner
from competition_planning.occupancy_grid_planner import OccupancyGridMap
from competition_planning.semantic_planner import PathPoint


@dataclass(frozen=True)
class LocalReplanConfig:
    lookahead_distance_m: float = 5.0
    inflation_radius_m: float = 0.30
    search_padding_m: float = 3.0
    sample_spacing_m: float = 0.10
    min_turning_radius_m: float = 0.81
    step_length_m: float = 0.20
    curvature_bins: int = 9
    heading_bins: int = 72
    goal_position_tolerance_m: float = 0.25
    goal_heading_tolerance_rad: float = math.radians(15.0)
    reference_deviation_weight: float = 2.0
    max_expansions: int = 250_000
    planning_timeout_s: float = 1.5
    reference_search_window_points: int = 120

    def __post_init__(self) -> None:
        if self.lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if self.reference_deviation_weight < 0.0:
            raise ValueError("reference_deviation_weight must be non-negative")
        if self.planning_timeout_s <= 0.0:
            raise ValueError("planning_timeout_s must be positive")
        if self.reference_search_window_points < 2:
            raise ValueError("reference_search_window_points must be at least 2")


@dataclass(frozen=True)
class LocalPlan:
    path: tuple[PathPoint, ...]
    reference_start_index: int
    rejoin_index: int
    dynamic_obstacle_count: int
    status: str
    path_is_navigable: bool
    planning_grid_cell_count: int


class LocalTrajectoryPlanner:
    """Produce one local path that stays near and rejoins a global reference."""

    def __init__(
        self,
        static_map: OccupancyGridMap,
        config: LocalReplanConfig,
    ) -> None:
        self._static_map = static_map
        self._config = config

    def plan(
        self,
        *,
        reference_path: Sequence[PathPoint],
        current_pose: PathPoint,
        dynamic_obstacle_points: Iterable[tuple[float, float]],
        previous_reference_index: int = 0,
    ) -> LocalPlan:
        if len(reference_path) < 2:
            raise ValueError("local replanning requires at least two reference points")

        start_index = _nearest_reference_index(
            reference_path,
            current_pose,
            previous_reference_index=previous_reference_index,
            search_window_points=self._config.reference_search_window_points,
        )
        rejoin_index = _lookahead_index(
            reference_path,
            start_index,
            self._config.lookahead_distance_m,
        )
        if rejoin_index <= start_index:
            raise ValueError("global reference has no forward local replanning horizon")

        local_reference = tuple(reference_path[start_index : rejoin_index + 1])
        planning_map = _crop_planning_map(
            self._static_map,
            current_pose,
            local_reference,
            margin_m=(
                self._config.search_padding_m
                + self._config.inflation_radius_m
                + self._config.step_length_m
            ),
        )
        live_map, obstacle_count = _overlay_obstacles(
            planning_map,
            dynamic_obstacle_points,
        )
        planner = self._planner(live_map, local_reference)
        rejoin_index = _select_navigable_rejoin_index(
            reference_path,
            planner,
            start_index=start_index,
            preferred_rejoin_index=rejoin_index,
        )
        local_reference = tuple(reference_path[start_index : rejoin_index + 1])
        planner = self._planner(live_map, local_reference)
        if planner.path_is_navigable(local_reference):
            return LocalPlan(
                path=local_reference,
                reference_start_index=start_index,
                rejoin_index=rejoin_index,
                dynamic_obstacle_count=obstacle_count,
                status="REFERENCE_CLEAR",
                path_is_navigable=True,
                planning_grid_cell_count=live_map.width * live_map.height,
            )

        path = planner.plan(
            (
                PathPoint(
                    current_pose.x,
                    current_pose.y,
                    current_pose.yaw,
                    ref_id=local_reference[0].ref_id,
                ),
                local_reference[-1],
            )
        )
        return LocalPlan(
            path=path,
            reference_start_index=start_index,
            rejoin_index=rejoin_index,
            dynamic_obstacle_count=obstacle_count,
            status="REPLANNED",
            path_is_navigable=planner.path_is_navigable(path),
            planning_grid_cell_count=live_map.width * live_map.height,
        )

    def _planner(
        self,
        grid_map: OccupancyGridMap,
        reference_path: Sequence[PathPoint],
    ) -> HybridAStarPlanner:
        config = self._config
        return HybridAStarPlanner(
            grid_map,
            inflation_radius_m=config.inflation_radius_m,
            search_padding_m=config.search_padding_m,
            sample_spacing_m=config.sample_spacing_m,
            min_turning_radius_m=config.min_turning_radius_m,
            step_length_m=config.step_length_m,
            curvature_bins=config.curvature_bins,
            heading_bins=config.heading_bins,
            goal_position_tolerance_m=config.goal_position_tolerance_m,
            goal_heading_tolerance_rad=config.goal_heading_tolerance_rad,
            max_expansions=config.max_expansions,
            reference_path=reference_path,
            reference_deviation_weight=config.reference_deviation_weight,
            planning_timeout_s=config.planning_timeout_s,
        )


def _nearest_reference_index(
    reference_path: Sequence[PathPoint],
    current_pose: PathPoint,
    *,
    previous_reference_index: int,
    search_window_points: int,
) -> int:
    start = max(0, min(previous_reference_index, len(reference_path) - 2) - 3)
    end = min(len(reference_path), start + search_window_points)
    return min(
        range(start, end),
        key=lambda index: (
            (reference_path[index].x - current_pose.x) ** 2
            + (reference_path[index].y - current_pose.y) ** 2
            + 0.04
            * _wrap_angle(reference_path[index].yaw - current_pose.yaw) ** 2,
            index,
        ),
    )


def _lookahead_index(
    reference_path: Sequence[PathPoint],
    start_index: int,
    lookahead_distance_m: float,
) -> int:
    distance = 0.0
    for index in range(start_index + 1, len(reference_path)):
        previous = reference_path[index - 1]
        current = reference_path[index]
        distance += math.hypot(current.x - previous.x, current.y - previous.y)
        if distance + 1e-9 >= lookahead_distance_m:
            return index
    return len(reference_path) - 1


def _select_navigable_rejoin_index(
    reference_path: Sequence[PathPoint],
    planner: HybridAStarPlanner,
    *,
    start_index: int,
    preferred_rejoin_index: int,
) -> int:
    """Pick a reachable forward reference point near the requested rejoin point.

    Live point-cloud obstacles can temporarily occupy the exact lookahead point.
    Failing the whole local plan in that case causes the vehicle to stop even
    though a nearby point on the same global route is still usable.
    """

    if planner.path_is_navigable((reference_path[preferred_rejoin_index],)):
        return preferred_rejoin_index

    forward_candidates = range(preferred_rejoin_index + 1, len(reference_path))
    backward_candidates = range(preferred_rejoin_index - 1, start_index, -1)
    for candidates in (forward_candidates, backward_candidates):
        for index in candidates:
            if planner.path_is_navigable((reference_path[index],)):
                return index
    return preferred_rejoin_index


def _overlay_obstacles(
    static_map: OccupancyGridMap,
    points: Iterable[tuple[float, float]],
) -> tuple[OccupancyGridMap, int]:
    occupied = list(static_map.occupied)
    count = 0
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        cell = static_map.world_to_cell(x, y)
        if not static_map.contains(cell):
            continue
        occupied[static_map.index(cell)] = True
        count += 1
    return (
        OccupancyGridMap(
            width=static_map.width,
            height=static_map.height,
            resolution=static_map.resolution,
            origin_x=static_map.origin_x,
            origin_y=static_map.origin_y,
            occupied=tuple(occupied),
        ),
        count,
    )


def _crop_planning_map(
    static_map: OccupancyGridMap,
    current_pose: PathPoint,
    reference_path: Sequence[PathPoint],
    *,
    margin_m: float,
) -> OccupancyGridMap:
    x_values = [current_pose.x, *(point.x for point in reference_path)]
    y_values = [current_pose.y, *(point.y for point in reference_path)]
    resolution = static_map.resolution
    min_col = max(
        0,
        math.floor((min(x_values) - margin_m - static_map.origin_x) / resolution),
    )
    max_col = min(
        static_map.width - 1,
        math.floor((max(x_values) + margin_m - static_map.origin_x) / resolution),
    )
    min_bottom = max(
        0,
        math.floor((min(y_values) - margin_m - static_map.origin_y) / resolution),
    )
    max_bottom = min(
        static_map.height - 1,
        math.floor((max(y_values) + margin_m - static_map.origin_y) / resolution),
    )
    if min_col > max_col or min_bottom > max_bottom:
        raise ValueError("local planning window is outside the static occupancy map")

    width = max_col - min_col + 1
    height = max_bottom - min_bottom + 1
    occupied: list[bool] = []
    first_source_row = static_map.height - 1 - max_bottom
    for row in range(first_source_row, first_source_row + height):
        start = row * static_map.width + min_col
        occupied.extend(static_map.occupied[start : start + width])
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=static_map.origin_x + min_col * resolution,
        origin_y=static_map.origin_y + min_bottom * resolution,
        occupied=tuple(occupied),
    )


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
