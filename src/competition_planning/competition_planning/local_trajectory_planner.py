"""Reference-aware short-horizon replanning over static and live obstacles."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from competition_planning.hybrid_astar_planner import HybridAStarPlanner
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
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
    planning_timeout_s: float = 2.0
    reference_search_window_points: int = 120
    relaxed_segment_entry_ref: str = ""
    relaxed_segment_exit_ref: str = ""
    relaxed_activation_distance_m: float = 0.0
    relaxed_reference_deviation_weight: float = 0.5
    relaxed_corridor_half_width_m: float = 0.85
    relaxed_step_length_m: float = 0.30
    relaxed_goal_heading_tolerance_rad: float = math.radians(20.0)
    trajectory_switch_improvement_ratio: float = 0.15
    obstacle_clearance_distance_m: float = 0.0
    obstacle_clearance_weight: float = 0.0
    search_heuristic_weight: float = 1.0

    def __post_init__(self) -> None:
        if self.lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if self.reference_deviation_weight < 0.0:
            raise ValueError("reference_deviation_weight must be non-negative")
        if self.planning_timeout_s <= 0.0:
            raise ValueError("planning_timeout_s must be positive")
        if self.reference_search_window_points < 2:
            raise ValueError("reference_search_window_points must be at least 2")
        if bool(self.relaxed_segment_entry_ref) != bool(self.relaxed_segment_exit_ref):
            raise ValueError("relaxed segment entry and exit refs must be configured together")
        if self.relaxed_activation_distance_m < 0.0:
            raise ValueError("relaxed_activation_distance_m must be non-negative")
        if self.relaxed_reference_deviation_weight < 0.0:
            raise ValueError("relaxed_reference_deviation_weight must be non-negative")
        if self.relaxed_corridor_half_width_m <= 0.0:
            raise ValueError("relaxed_corridor_half_width_m must be positive")
        if self.relaxed_step_length_m <= 0.0:
            raise ValueError("relaxed_step_length_m must be positive")
        if self.relaxed_goal_heading_tolerance_rad <= 0.0:
            raise ValueError("relaxed_goal_heading_tolerance_rad must be positive")
        if not 0.0 <= self.trajectory_switch_improvement_ratio < 1.0:
            raise ValueError("trajectory_switch_improvement_ratio must be in [0, 1)")
        if self.obstacle_clearance_distance_m < 0.0:
            raise ValueError("obstacle_clearance_distance_m must be non-negative")
        if self.obstacle_clearance_weight < 0.0:
            raise ValueError("obstacle_clearance_weight must be non-negative")
        if self.search_heuristic_weight < 1.0:
            raise ValueError("search_heuristic_weight must be at least 1.0")


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
        self._held_relaxed_path: tuple[PathPoint, ...] = ()
        self._held_rejoin_index: int | None = None
        self._early_rejoin_exit_index: int | None = None
        self._early_rejoin_path_active = False

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
        relaxed_span = _active_relaxed_segment(
            reference_path,
            start_index=start_index,
            entry_ref=self._config.relaxed_segment_entry_ref,
            exit_ref=self._config.relaxed_segment_exit_ref,
            activation_distance_m=self._config.relaxed_activation_distance_m,
            planning_horizon_m=self._config.lookahead_distance_m,
        )
        checkpoint_advance_distance_m = max(
            self._config.relaxed_step_length_m,
            2.0 * self._config.goal_position_tolerance_m,
        )
        held_rejoin_index = self._held_rejoin_index
        if held_rejoin_index is not None and (
            math.hypot(
                reference_path[held_rejoin_index].x - current_pose.x,
                reference_path[held_rejoin_index].y - current_pose.y,
            )
            <= checkpoint_advance_distance_m + 1e-9
        ):
            self._held_relaxed_path = ()
            self._held_rejoin_index = None
            self._early_rejoin_path_active = False
        early_rejoin_exit_index = self._early_rejoin_exit_index
        if early_rejoin_exit_index is not None:
            if start_index >= early_rejoin_exit_index:
                self._early_rejoin_exit_index = None
                self._early_rejoin_path_active = False
            elif (
                relaxed_span is not None
                and relaxed_span[1] == early_rejoin_exit_index
                and not self._early_rejoin_path_active
            ):
                relaxed_span = None
        if relaxed_span is not None:
            segment_exit = reference_path[relaxed_span[1]]
            if (
                math.hypot(
                    segment_exit.x - current_pose.x,
                    segment_exit.y - current_pose.y,
                )
                <= checkpoint_advance_distance_m + 1e-9
            ):
                self._held_relaxed_path = ()
                self._held_rejoin_index = None
                self._early_rejoin_exit_index = None
                self._early_rejoin_path_active = False
                relaxed_span = None
        rolling_rejoin_index = _lookahead_index(
            reference_path,
            start_index,
            self._config.lookahead_distance_m,
        )
        if relaxed_span is None:
            rejoin_index = rolling_rejoin_index
        else:
            segment_entry_index, segment_exit_index = relaxed_span
            held_rejoin_index = self._held_rejoin_index
            if (
                held_rejoin_index is not None
                and start_index < held_rejoin_index <= segment_exit_index
            ):
                rejoin_index = held_rejoin_index
            else:
                distance_to_entry_m = sum(
                    math.hypot(
                        reference_path[index].x - reference_path[index - 1].x,
                        reference_path[index].y - reference_path[index - 1].y,
                    )
                    for index in range(start_index + 1, segment_entry_index + 1)
                )
                use_approach_checkpoint = (
                    start_index < segment_entry_index < rolling_rejoin_index
                    and distance_to_entry_m
                    > self._config.relaxed_activation_distance_m + 1e-9
                )
                rejoin_index = (
                    min(segment_exit_index, rolling_rejoin_index)
                    if use_approach_checkpoint
                    else segment_exit_index
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
        relaxed = relaxed_span is not None
        planner = self._planner(live_map, local_reference, relaxed=relaxed)
        selected_rejoin_index = _select_navigable_rejoin_index(
            reference_path,
            planner,
            start_index=start_index,
            preferred_rejoin_index=rejoin_index,
        )
        if selected_rejoin_index != rejoin_index:
            rejoin_index = selected_rejoin_index
            local_reference = tuple(reference_path[start_index : rejoin_index + 1])
            planner = self._planner(live_map, local_reference, relaxed=relaxed)
        if (
            relaxed
            and relaxed_span is not None
            and self._held_rejoin_index is not None
            and self._early_rejoin_exit_index is None
        ):
            segment_entry_index, segment_exit_index = relaxed_span
            distance_from_entry_m = sum(
                math.hypot(
                    reference_path[index].x - reference_path[index - 1].x,
                    reference_path[index].y - reference_path[index - 1].y,
                )
                for index in range(segment_entry_index + 1, start_index + 1)
            )
            reference_is_clear = planner.path_is_navigable(local_reference)
            if (
                distance_from_entry_m
                >= self._config.relaxed_activation_distance_m - 1e-9
                and reference_is_clear
            ):
                early_rejoin_distance_m = max(
                    self._config.relaxed_activation_distance_m,
                    2.0 * self._config.min_turning_radius_m,
                )
                early_rejoin_index = min(
                    segment_exit_index,
                    _lookahead_index(
                        reference_path,
                        start_index,
                        early_rejoin_distance_m,
                    ),
                )
                early_reference = tuple(
                    reference_path[start_index : early_rejoin_index + 1]
                )
                early_planner = self._planner(
                    live_map,
                    early_reference,
                    relaxed=False,
                )
                try:
                    early_path = early_planner.plan(
                        (current_pose, early_reference[-1])
                    )
                except GridPlanningError:
                    early_path = ()
                if early_path and early_planner.path_is_navigable(early_path):
                    self._held_relaxed_path = tuple(early_path)
                    self._held_rejoin_index = early_rejoin_index
                    self._early_rejoin_exit_index = segment_exit_index
                    self._early_rejoin_path_active = True
                    return LocalPlan(
                        path=tuple(early_path),
                        reference_start_index=start_index,
                        rejoin_index=early_rejoin_index,
                        dynamic_obstacle_count=obstacle_count,
                        status="EARLY_REJOIN_REPLANNED",
                        path_is_navigable=True,
                        planning_grid_cell_count=live_map.width * live_map.height,
                    )
        if relaxed:
            held_path = self._safe_held_path(
                current_pose=current_pose,
                planner=planner,
                rejoin_index=rejoin_index,
            )
            if held_path:
                self._held_relaxed_path = held_path
                return LocalPlan(
                    path=held_path,
                    reference_start_index=start_index,
                    rejoin_index=rejoin_index,
                    dynamic_obstacle_count=obstacle_count,
                    status="RELAXED_HOLD",
                    path_is_navigable=True,
                    planning_grid_cell_count=live_map.width * live_map.height,
                )
        if not relaxed and planner.path_is_navigable(local_reference):
            self._held_relaxed_path = ()
            self._held_rejoin_index = None
            return LocalPlan(
                path=local_reference,
                reference_start_index=start_index,
                rejoin_index=rejoin_index,
                dynamic_obstacle_count=obstacle_count,
                status="REFERENCE_CLEAR",
                path_is_navigable=True,
                planning_grid_cell_count=live_map.width * live_map.height,
            )

        try:
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
        except GridPlanningError:
            held_path = (
                self._safe_held_tail(
                    current_pose=current_pose,
                    planner=planner,
                    rejoin_index=rejoin_index,
                )
                if relaxed
                else ()
            )
            if not held_path:
                raise
            self._held_relaxed_path = held_path
            return LocalPlan(
                path=held_path,
                reference_start_index=start_index,
                rejoin_index=rejoin_index,
                dynamic_obstacle_count=obstacle_count,
                status="RELAXED_HOLD",
                path_is_navigable=True,
                planning_grid_cell_count=live_map.width * live_map.height,
            )
        status = "REPLANNED"
        if relaxed:
            held_path = self._safe_held_path(
                current_pose=current_pose,
                planner=planner,
                rejoin_index=rejoin_index,
            )
            if held_path and not _path_is_materially_better(
                path,
                held_path,
                reference_path=local_reference,
                reference_deviation_weight=(
                    self._config.relaxed_reference_deviation_weight
                ),
                improvement_ratio=self._config.trajectory_switch_improvement_ratio,
            ):
                path = held_path
                status = "RELAXED_HOLD"
            else:
                status = "RELAXED_REPLANNED"
            self._held_relaxed_path = tuple(path)
            self._held_rejoin_index = rejoin_index
        else:
            self._held_relaxed_path = ()
            self._held_rejoin_index = None
        return LocalPlan(
            path=path,
            reference_start_index=start_index,
            rejoin_index=rejoin_index,
            dynamic_obstacle_count=obstacle_count,
            status=status,
            path_is_navigable=planner.path_is_navigable(path),
            planning_grid_cell_count=live_map.width * live_map.height,
        )

    def _safe_held_path(
        self,
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
        rejoin_index: int,
    ) -> tuple[PathPoint, ...]:
        held_tail = self._safe_held_tail(
            current_pose=current_pose,
            planner=planner,
            rejoin_index=rejoin_index,
        )
        if not held_tail:
            return ()
        candidate = (current_pose, *held_tail[1:])
        if not _splice_respects_turning_radius(
            candidate,
            min_turning_radius_m=self._config.min_turning_radius_m,
        ):
            return ()
        return candidate

    def _safe_held_tail(
        self,
        *,
        current_pose: PathPoint,
        planner: HybridAStarPlanner,
        rejoin_index: int,
    ) -> tuple[PathPoint, ...]:
        """Return held geometry only while it remains collision-free."""

        if not self._held_relaxed_path or self._held_rejoin_index != rejoin_index:
            return ()
        nearest_index = min(
            range(len(self._held_relaxed_path)),
            key=lambda index: math.hypot(
                self._held_relaxed_path[index].x - current_pose.x,
                self._held_relaxed_path[index].y - current_pose.y,
            ),
        )
        nearest = self._held_relaxed_path[nearest_index]
        if math.hypot(nearest.x - current_pose.x, nearest.y - current_pose.y) > 0.50:
            return ()
        held_tail = self._held_relaxed_path[nearest_index:]
        if len(held_tail) < 2 or not planner.path_is_navigable(
            (current_pose, *held_tail)
        ):
            return ()
        return held_tail

    def _planner(
        self,
        grid_map: OccupancyGridMap,
        reference_path: Sequence[PathPoint],
        *,
        relaxed: bool = False,
    ) -> HybridAStarPlanner:
        config = self._config
        return HybridAStarPlanner(
            grid_map,
            inflation_radius_m=config.inflation_radius_m,
            search_padding_m=config.search_padding_m,
            sample_spacing_m=config.sample_spacing_m,
            min_turning_radius_m=config.min_turning_radius_m,
            step_length_m=(
                config.relaxed_step_length_m if relaxed else config.step_length_m
            ),
            curvature_bins=config.curvature_bins,
            heading_bins=config.heading_bins,
            goal_position_tolerance_m=config.goal_position_tolerance_m,
            goal_heading_tolerance_rad=(
                config.relaxed_goal_heading_tolerance_rad
                if relaxed
                else config.goal_heading_tolerance_rad
            ),
            max_expansions=config.max_expansions,
            planning_timeout_s=config.planning_timeout_s,
            reference_path=reference_path,
            reference_deviation_weight=(
                config.relaxed_reference_deviation_weight
                if relaxed
                else config.reference_deviation_weight
            ),
            corridor_half_width_m=(
                config.relaxed_corridor_half_width_m if relaxed else None
            ),
            obstacle_clearance_distance_m=config.obstacle_clearance_distance_m,
            obstacle_clearance_weight=config.obstacle_clearance_weight,
            search_heuristic_weight=config.search_heuristic_weight,
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


def _active_relaxed_segment(
    reference_path: Sequence[PathPoint],
    *,
    start_index: int,
    entry_ref: str,
    exit_ref: str,
    activation_distance_m: float,
    planning_horizon_m: float,
) -> tuple[int, int] | None:
    """Activate once the local planning horizon reaches the approach window."""

    if not entry_ref or not exit_ref:
        return None

    exit_search_start = 0
    for entry_index, point in enumerate(reference_path):
        if point.ref_id != entry_ref:
            continue
        exit_index = next(
            (
                index
                for index in range(max(entry_index + 1, exit_search_start), len(reference_path))
                if reference_path[index].ref_id == exit_ref
            ),
            None,
        )
        if exit_index is None:
            return None
        exit_search_start = exit_index + 1
        if entry_index <= start_index < exit_index:
            return entry_index, exit_index
        if start_index < entry_index:
            distance = sum(
                math.hypot(
                    reference_path[index].x - reference_path[index - 1].x,
                    reference_path[index].y - reference_path[index - 1].y,
                )
                for index in range(start_index + 1, entry_index + 1)
            )
            if distance <= activation_distance_m + 1e-9:
                return entry_index, exit_index
            if distance <= activation_distance_m + planning_horizon_m + 1e-9:
                return entry_index, exit_index
    return None


def _path_is_materially_better(
    candidate: Sequence[PathPoint],
    held: Sequence[PathPoint],
    *,
    reference_path: Sequence[PathPoint],
    reference_deviation_weight: float,
    improvement_ratio: float,
) -> bool:
    candidate_cost = _path_cost(
        candidate,
        reference_path=reference_path,
        reference_deviation_weight=reference_deviation_weight,
    )
    held_cost = _path_cost(
        held,
        reference_path=reference_path,
        reference_deviation_weight=reference_deviation_weight,
    )
    return candidate_cost < held_cost * (1.0 - improvement_ratio)


def _splice_respects_turning_radius(
    path: Sequence[PathPoint],
    *,
    min_turning_radius_m: float,
) -> bool:
    """Validate the only new bend introduced by prepending the current pose."""

    if len(path) < 3:
        return True
    first, second, third = path[:3]
    ab = math.hypot(second.x - first.x, second.y - first.y)
    bc = math.hypot(third.x - second.x, third.y - second.y)
    ca = math.hypot(first.x - third.x, first.y - third.y)
    denominator = ab * bc * ca
    if denominator <= 1e-12:
        return True
    cross = (second.x - first.x) * (third.y - first.y) - (
        second.y - first.y
    ) * (third.x - first.x)
    curvature = 2.0 * cross / denominator
    return abs(curvature) <= 1.0 / min_turning_radius_m + 1e-6


def _path_cost(
    path: Sequence[PathPoint],
    *,
    reference_path: Sequence[PathPoint],
    reference_deviation_weight: float,
) -> float:
    cost = 0.0
    reference_xy = tuple((point.x, point.y) for point in reference_path)
    for previous, point in zip(path, path[1:]):
        distance = math.hypot(point.x - previous.x, point.y - previous.y)
        deviation_squared = min(
            (point.x - x) ** 2 + (point.y - y) ** 2
            for x, y in reference_xy
        )
        cost += distance * (
            1.0 + reference_deviation_weight * deviation_squared
        )
    return cost


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


def occupied_grid_cell_centers(
    data: Iterable[int],
    *,
    width: int,
    height: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    occupancy_threshold: int,
    x_min_m: float,
    x_max_m: float,
    y_half_width_m: float,
    max_points: int,
) -> tuple[tuple[float, float], ...]:
    """Extract bounded obstacle cells from an already-inflated local costmap."""

    if width < 1 or height < 1 or resolution_m <= 0.0:
        raise ValueError("costmap geometry is invalid")
    if not 0 <= occupancy_threshold <= 100:
        raise ValueError("occupancy_threshold must be in [0, 100]")
    if x_max_m <= x_min_m or y_half_width_m <= 0.0 or max_points < 1:
        raise ValueError("costmap obstacle bounds are invalid")

    occupied: list[tuple[float, float]] = []
    for index, value in enumerate(data):
        if index >= width * height:
            break
        if int(value) < occupancy_threshold:
            continue
        row, column = divmod(index, width)
        x = origin_x_m + (column + 0.5) * resolution_m
        y = origin_y_m + (row + 0.5) * resolution_m
        if not (x_min_m <= x <= x_max_m and abs(y) <= y_half_width_m):
            continue
        occupied.append((x, y))

    if len(occupied) <= max_points:
        return tuple(occupied)
    stride = math.ceil(len(occupied) / max_points)
    return tuple(occupied[::stride][:max_points])
