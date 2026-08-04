"""Forward-only Hybrid A* for pose-constrained Ranger route segments."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
import time
from typing import Sequence

from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


_MISSING_POINT_COST = object()


class HybridAStarTimeout(GridPlanningError):
    """Raised when a Hybrid A* search exceeds its wall-clock budget."""


@dataclass(frozen=True)
class _Node:
    x: float
    y: float
    yaw: float
    curvature_index: int


class HybridAStarPlanner:
    """Search collision-free forward motions while preserving pose and radius.

    The external interface intentionally matches ``GridAStarPlanner``. Search
    states include planar position, heading, and the previous curvature
    primitive so steering changes can be constrained inside the module.
    """

    def __init__(
        self,
        grid_map: OccupancyGridMap,
        *,
        inflation_radius_m: float,
        search_padding_m: float,
        sample_spacing_m: float,
        min_turning_radius_m: float,
        step_length_m: float,
        curvature_bins: int,
        heading_bins: int,
        goal_position_tolerance_m: float,
        goal_heading_tolerance_rad: float,
        max_expansions: int = 250_000,
        planning_timeout_s: float | None = None,
        reference_path: Sequence[PathPoint] = (),
        reference_deviation_weight: float = 0.0,
        corridor_half_width_m: float | None = None,
        obstacle_clearance_distance_m: float = 0.0,
        obstacle_clearance_weight: float = 0.0,
    ) -> None:
        if min_turning_radius_m <= 0.0:
            raise GridPlanningError("hybrid_astar requires a positive min_turning_radius_m")
        self._map = grid_map
        self._blocked = grid_map.inflated_blocked(inflation_radius_m)
        self._padding_cells = int(
            math.ceil(max(0.0, search_padding_m) / grid_map.resolution)
        )
        self._sample_spacing_m = max(0.03, sample_spacing_m)
        self._step_length_m = max(self._sample_spacing_m, step_length_m)
        self._heading_bins = max(16, heading_bins)
        self._goal_position_tolerance_m = max(
            grid_map.resolution * 0.5,
            goal_position_tolerance_m,
        )
        self._goal_heading_tolerance_rad = max(
            math.pi / self._heading_bins,
            goal_heading_tolerance_rad,
        )
        self._max_expansions = max(1, max_expansions)
        if planning_timeout_s is not None and planning_timeout_s <= 0.0:
            raise GridPlanningError("planning_timeout_s must be positive")
        self._planning_timeout_s = planning_timeout_s
        if reference_deviation_weight < 0.0:
            raise GridPlanningError("reference_deviation_weight must be non-negative")
        self._reference_xy = tuple((point.x, point.y) for point in reference_path)
        self._reference_deviation_weight = reference_deviation_weight
        self._reference_distance_sq_cache: dict[tuple[int, int], float] = {}
        if corridor_half_width_m is not None and corridor_half_width_m <= 0.0:
            raise GridPlanningError("corridor_half_width_m must be positive")
        self._corridor_half_width_sq = (
            None if corridor_half_width_m is None else corridor_half_width_m**2
        )
        if obstacle_clearance_distance_m < 0.0:
            raise GridPlanningError("obstacle_clearance_distance_m must be non-negative")
        if obstacle_clearance_weight < 0.0:
            raise GridPlanningError("obstacle_clearance_weight must be non-negative")
        self._obstacle_clearance_distance_m = obstacle_clearance_distance_m
        self._obstacle_clearance_weight = obstacle_clearance_weight
        self._obstacle_clearance_cost_by_cell: tuple[float, ...] | None = (
            None
            if obstacle_clearance_distance_m > 0.0 and obstacle_clearance_weight > 0.0
            else ()
        )
        self._navigable_cell_cache: dict[tuple[int, int], bool] = {}
        self._point_cost_cache: dict[tuple[int, int], float | None] = {}
        max_curvature = 1.0 / min_turning_radius_m
        if curvature_bins < 3 or curvature_bins % 2 == 0:
            raise GridPlanningError(
                "hybrid_astar curvature_bins must be an odd integer >= 3"
            )
        self._curvatures = tuple(
            -max_curvature + 2.0 * max_curvature * index / (curvature_bins - 1)
            for index in range(curvature_bins)
        )

    def plan(self, waypoints: Sequence[PathPoint]) -> tuple[PathPoint, ...]:
        if len(waypoints) < 2:
            return tuple(waypoints)

        deadline_s = (
            None
            if self._planning_timeout_s is None
            else time.monotonic() + self._planning_timeout_s
        )
        path: list[PathPoint] = []
        segment_start = waypoints[0]
        curvature_index = len(self._curvatures) // 2
        for goal in waypoints[1:]:
            segment, curvature_index = self._search(
                segment_start,
                goal,
                start_curvature_index=curvature_index,
                deadline_s=deadline_s,
            )
            if path:
                path.extend(segment[1:])
            else:
                path.extend(segment)
            reached = segment[-1]
            segment_start = PathPoint(
                reached.x,
                reached.y,
                reached.yaw,
                ref_id=goal.ref_id,
            )
        return tuple(path)

    def path_is_navigable(self, path: Sequence[PathPoint]) -> bool:
        return all(self._point_is_navigable(point.x, point.y) for point in path)

    def _search(
        self,
        start: PathPoint,
        goal: PathPoint,
        *,
        start_curvature_index: int,
        deadline_s: float | None,
    ) -> tuple[list[PathPoint], int]:
        self._require_navigable(start.x, start.y, "start")
        self._require_navigable(goal.x, goal.y, "goal")
        bounds = self._bounds(start, goal)
        start_node = _Node(
            start.x,
            start.y,
            _wrap_angle(start.yaw),
            start_curvature_index,
        )
        start_key = self._key(start_node)
        states = {start_key: start_node}
        came_from: dict[tuple[int, int, int, int], tuple[int, int, int, int]] = {}
        g_score = {start_key: 0.0}
        open_heap: list[tuple[float, int, tuple[int, int, int, int]]] = [
            (self._heuristic(start_node, goal), 0, start_key)
        ]
        closed: set[tuple[int, int, int, int]] = set()
        sequence = 0

        for expansion_index in range(self._max_expansions):
            if (
                deadline_s is not None
                and expansion_index % 64 == 0
                and time.monotonic() >= deadline_s
            ):
                timeout_s = self._planning_timeout_s
                if timeout_s is None:
                    raise AssertionError("deadline requires planning_timeout_s")
                raise HybridAStarTimeout(
                    "hybrid_astar search exceeded "
                    f"{timeout_s:.3f} s"
                )
            if not open_heap:
                break
            _, _, current_key = heapq.heappop(open_heap)
            if current_key in closed:
                continue
            current = states[current_key]
            if self._at_goal(current, goal):
                return (
                    self._reconstruct(
                        came_from,
                        states,
                        current_key,
                        start_ref=start.ref_id,
                        goal_ref=goal.ref_id,
                    ),
                    current.curvature_index,
                )
            closed.add(current_key)

            for curvature_index in self._successor_curvatures(current.curvature_index):
                successor = self._advance(
                    current,
                    curvature_index,
                    self._step_length_m,
                )
                if not self._inside_bounds(successor, bounds):
                    continue
                motion_clearance_cost = self._motion_clearance_cost(
                    current,
                    curvature_index,
                )
                if motion_clearance_cost is None:
                    continue
                successor_key = self._key(successor)
                if successor_key in closed:
                    continue
                steering_change = abs(curvature_index - current.curvature_index)
                curvature_fraction = abs(self._curvatures[curvature_index]) / abs(
                    self._curvatures[-1]
                )
                tentative = g_score[current_key] + self._step_length_m * (
                    1.0 + 0.08 * curvature_fraction + 0.12 * steering_change
                    + self._reference_deviation_cost(successor)
                    + motion_clearance_cost
                )
                if tentative >= g_score.get(successor_key, math.inf):
                    continue
                states[successor_key] = successor
                came_from[successor_key] = current_key
                g_score[successor_key] = tentative
                sequence += 1
                priority = tentative + self._heuristic(successor, goal)
                heapq.heappush(open_heap, (priority, sequence, successor_key))

        raise GridPlanningError(
            "hybrid_astar found no forward pose-constrained path from "
            f"({start.x:.3f}, {start.y:.3f}, {start.yaw:.3f}) to "
            f"({goal.x:.3f}, {goal.y:.3f}, {goal.yaw:.3f})"
        )

    def _reference_deviation_cost(self, node: _Node) -> float:
        if not self._reference_xy or self._reference_deviation_weight <= 0.0:
            return 0.0
        return self._reference_deviation_weight * self._reference_distance_squared(
            node.x,
            node.y,
        )

    def _reference_distance_squared(self, x: float, y: float) -> float:
        cell = self._map.world_to_cell(x, y)
        distance_squared = self._reference_distance_sq_cache.get(cell)
        if distance_squared is None:
            cell_x, cell_y = self._map.cell_to_world(cell)
            distance_squared = min(
                (cell_x - x) ** 2 + (cell_y - y) ** 2
                for x, y in self._reference_xy
            )
            self._reference_distance_sq_cache[cell] = distance_squared
        return distance_squared

    def _obstacle_clearance_cost(self, cell: tuple[int, int]) -> float:
        costs = self._obstacle_clearance_cost_by_cell
        if costs is None:
            distances = _clearance_distance_field(
                self._blocked,
                width=self._map.width,
                height=self._map.height,
                resolution=self._map.resolution,
                max_distance_m=self._obstacle_clearance_distance_m,
            )
            costs = tuple(
                self._obstacle_clearance_weight
                * max(
                    0.0,
                    1.0 - clearance_m / self._obstacle_clearance_distance_m,
                )
                ** 2
                for clearance_m in distances
            )
            self._obstacle_clearance_cost_by_cell = costs
        if not costs:
            return 0.0
        if not self._map.contains(cell):
            return self._obstacle_clearance_weight
        return costs[self._map.index(cell)]

    def _successor_curvatures(self, previous_index: int) -> range:
        return range(
            max(0, previous_index - 1),
            min(len(self._curvatures), previous_index + 2),
        )

    def _advance(self, node: _Node, curvature_index: int, distance: float) -> _Node:
        curvature = self._curvatures[curvature_index]
        if abs(curvature) <= 1e-12:
            x = node.x + distance * math.cos(node.yaw)
            y = node.y + distance * math.sin(node.yaw)
            yaw = node.yaw
        else:
            yaw = node.yaw + curvature * distance
            x = node.x + (math.sin(yaw) - math.sin(node.yaw)) / curvature
            y = node.y - (math.cos(yaw) - math.cos(node.yaw)) / curvature
        return _Node(x=x, y=y, yaw=_wrap_angle(yaw), curvature_index=curvature_index)

    def _motion_clearance_cost(
        self,
        node: _Node,
        curvature_index: int,
    ) -> float | None:
        samples = max(2, math.ceil(self._step_length_m / (self._map.resolution * 0.5)))
        clearance_cost = 0.0
        for index in range(1, samples + 1):
            point = self._advance(
                node,
                curvature_index,
                self._step_length_m * index / samples,
            )
            point_cost = self._point_navigation_cost(point.x, point.y)
            if point_cost is None:
                return None
            clearance_cost += point_cost
        return clearance_cost / samples

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int, int, int], tuple[int, int, int, int]],
        states: dict[tuple[int, int, int, int], _Node],
        current_key: tuple[int, int, int, int],
        *,
        start_ref: str | None,
        goal_ref: str | None,
    ) -> list[PathPoint]:
        keys = [current_key]
        while current_key in came_from:
            current_key = came_from[current_key]
            keys.append(current_key)
        keys.reverse()

        points: list[PathPoint] = []
        for parent_key, child_key in zip(keys, keys[1:]):
            parent = states[parent_key]
            child = states[child_key]
            samples = max(1, math.ceil(self._step_length_m / self._sample_spacing_m))
            for index in range(samples):
                if points and index == 0:
                    continue
                sample = self._advance(
                    parent,
                    child.curvature_index,
                    self._step_length_m * index / samples,
                )
                points.append(PathPoint(sample.x, sample.y, sample.yaw))
            points.append(PathPoint(child.x, child.y, child.yaw))

        if not points:
            node = states[keys[0]]
            points = [PathPoint(node.x, node.y, node.yaw)]
        first = points[0]
        last = points[-1]
        points[0] = PathPoint(first.x, first.y, first.yaw, ref_id=start_ref)
        points[-1] = PathPoint(last.x, last.y, last.yaw, ref_id=goal_ref)
        return points

    def _key(self, node: _Node) -> tuple[int, int, int, int]:
        col, row = self._map.world_to_cell(node.x, node.y)
        yaw_fraction = (_wrap_angle(node.yaw) + math.pi) / (2.0 * math.pi)
        yaw_bin = int(round(yaw_fraction * self._heading_bins)) % self._heading_bins
        return col, row, yaw_bin, node.curvature_index

    def _heuristic(self, node: _Node, goal: PathPoint) -> float:
        distance = math.hypot(goal.x - node.x, goal.y - node.y)
        heading = abs(_wrap_angle(goal.yaw - node.yaw))
        min_radius = 1.0 / abs(self._curvatures[-1])
        return distance + 0.25 * min_radius * heading

    def _at_goal(self, node: _Node, goal: PathPoint) -> bool:
        return (
            math.hypot(goal.x - node.x, goal.y - node.y)
            <= self._goal_position_tolerance_m
            and abs(_wrap_angle(goal.yaw - node.yaw))
            <= self._goal_heading_tolerance_rad
            and node.curvature_index == len(self._curvatures) // 2
        )

    def _bounds(
        self,
        start: PathPoint,
        goal: PathPoint,
    ) -> tuple[int, int, int, int]:
        start_cell = self._map.world_to_cell(start.x, start.y)
        goal_cell = self._map.world_to_cell(goal.x, goal.y)
        return (
            max(0, min(start_cell[0], goal_cell[0]) - self._padding_cells),
            min(self._map.width - 1, max(start_cell[0], goal_cell[0]) + self._padding_cells),
            max(0, min(start_cell[1], goal_cell[1]) - self._padding_cells),
            min(self._map.height - 1, max(start_cell[1], goal_cell[1]) + self._padding_cells),
        )

    def _inside_bounds(self, node: _Node, bounds: tuple[int, int, int, int]) -> bool:
        col, row = self._map.world_to_cell(node.x, node.y)
        return bounds[0] <= col <= bounds[1] and bounds[2] <= row <= bounds[3]

    def _point_is_navigable(self, x: float, y: float) -> bool:
        cell = self._map.world_to_cell(x, y)
        return self._cell_is_navigable(cell, x=x, y=y)

    def _point_navigation_cost(self, x: float, y: float) -> float | None:
        cell = self._map.world_to_cell(x, y)
        cached = self._point_cost_cache.get(cell, _MISSING_POINT_COST)
        if cached is not _MISSING_POINT_COST:
            return cached
        cost = (
            self._obstacle_clearance_cost(cell)
            if self._cell_is_navigable(cell, x=x, y=y)
            else None
        )
        self._point_cost_cache[cell] = cost
        return cost

    def _cell_is_navigable(
        self,
        cell: tuple[int, int],
        *,
        x: float,
        y: float,
    ) -> bool:
        cached = self._navigable_cell_cache.get(cell)
        if cached is not None:
            return cached
        navigable = self._map.contains(cell) and not self._blocked[
            self._map.index(cell)
        ]
        if navigable and self._corridor_half_width_sq is not None:
            navigable = (
                self._reference_distance_squared(x, y)
                <= self._corridor_half_width_sq
            )
        self._navigable_cell_cache[cell] = navigable
        return navigable

    def _require_navigable(self, x: float, y: float, label: str) -> None:
        if not self._point_is_navigable(x, y):
            raise GridPlanningError(f"{label} pose ({x:.3f}, {y:.3f}) is blocked")


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _clearance_distance_field(
    blocked: Sequence[bool],
    *,
    width: int,
    height: int,
    resolution: float,
    max_distance_m: float,
) -> tuple[float, ...]:
    """Approximate distance to the inflated edge with a bounded 8-neighbor BFS."""

    max_steps = max(1, int(math.ceil(max_distance_m / resolution)))
    unknown = max_steps + 1
    steps = [unknown] * len(blocked)
    queue: deque[int] = deque()
    for index, is_blocked in enumerate(blocked):
        if is_blocked:
            steps[index] = 0
            queue.append(index)

    while queue:
        index = queue.popleft()
        distance_steps = steps[index]
        if distance_steps >= max_steps:
            continue
        row, col = divmod(index, width)
        next_distance = distance_steps + 1
        for row_delta in (-1, 0, 1):
            neighbor_row = row + row_delta
            if not 0 <= neighbor_row < height:
                continue
            for col_delta in (-1, 0, 1):
                if row_delta == 0 and col_delta == 0:
                    continue
                neighbor_col = col + col_delta
                if not 0 <= neighbor_col < width:
                    continue
                neighbor_index = neighbor_row * width + neighbor_col
                if steps[neighbor_index] <= next_distance:
                    continue
                steps[neighbor_index] = next_distance
                queue.append(neighbor_index)

    return tuple(
        min(max_distance_m, distance_steps * resolution)
        for distance_steps in steps
    )
