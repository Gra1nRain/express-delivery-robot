"""Dynamic-window local path selection around a global reference."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from competition_planning.semantic_planner import PathPoint


class DWAPlanningError(RuntimeError):
    """Raised when no collision-free dynamic-window candidate exists."""


@dataclass(frozen=True)
class DWAVelocity:
    linear_mps: float
    yaw_rate_radps: float


@dataclass(frozen=True)
class DWAConfig:
    min_speed_mps: float = 0.05
    max_speed_mps: float = 0.20
    max_acceleration_mps2: float = 0.20
    max_deceleration_mps2: float = 0.30
    max_yaw_rate_radps: float = 0.30
    max_yaw_acceleration_radps2: float = 0.60
    min_turning_radius_m: float = 0.81
    control_interval_s: float = 0.50
    prediction_horizon_s: float = 6.0
    simulation_step_s: float = 0.20
    speed_sample_count: int = 4
    yaw_rate_sample_count: int = 11
    obstacle_clearance_m: float = 0.55
    reference_lookahead_m: float = 1.50
    max_reference_deviation_m: float = 1.20
    reference_search_window_points: int = 160
    progress_weight: float = 6.0
    path_distance_weight: float = 2.0
    goal_distance_weight: float = 1.0
    heading_weight: float = 0.3
    clearance_weight: float = 0.8
    speed_weight: float = 1.0
    yaw_rate_weight: float = 0.15

    def __post_init__(self) -> None:
        positive = {
            "min_speed_mps": self.min_speed_mps,
            "max_speed_mps": self.max_speed_mps,
            "max_acceleration_mps2": self.max_acceleration_mps2,
            "max_deceleration_mps2": self.max_deceleration_mps2,
            "max_yaw_rate_radps": self.max_yaw_rate_radps,
            "max_yaw_acceleration_radps2": self.max_yaw_acceleration_radps2,
            "min_turning_radius_m": self.min_turning_radius_m,
            "control_interval_s": self.control_interval_s,
            "prediction_horizon_s": self.prediction_horizon_s,
            "simulation_step_s": self.simulation_step_s,
            "obstacle_clearance_m": self.obstacle_clearance_m,
            "reference_lookahead_m": self.reference_lookahead_m,
            "max_reference_deviation_m": self.max_reference_deviation_m,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f"{', '.join(invalid)} must be positive")
        if self.max_speed_mps < self.min_speed_mps:
            raise ValueError("max_speed_mps must be at least min_speed_mps")
        if self.simulation_step_s > self.prediction_horizon_s:
            raise ValueError("simulation_step_s must not exceed prediction_horizon_s")
        if self.speed_sample_count < 2 or self.yaw_rate_sample_count < 3:
            raise ValueError("DWA requires at least 2 speed and 3 yaw-rate samples")
        if self.reference_search_window_points < 2:
            raise ValueError("reference_search_window_points must be at least 2")


@dataclass(frozen=True)
class DWAPlan:
    path: tuple[PathPoint, ...]
    reference_start_index: int
    reference_target_index: int
    selected_velocity: DWAVelocity
    minimum_clearance_m: float | None
    obstacle_count: int
    status: str


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
    """Extract occupied cell centres from an already-inflated local costmap."""

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


def filter_and_downsample_obstacle_points(
    points_xyz: Iterable[Sequence[float]],
    *,
    z_min_m: float,
    z_max_m: float,
    x_min_m: float,
    x_max_m: float,
    y_half_width_m: float,
    point_stride: int,
    voxel_size_m: float,
    max_points: int,
) -> tuple[tuple[float, float, float], ...]:
    """Keep the local obstacle window before applying the point-count cap."""
    points: list[tuple[float, float, float]] = []
    occupied_voxels: set[tuple[int, int, int]] = set()
    for index, point in enumerate(points_xyz):
        if index % point_stride:
            continue
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        if not (
            z_min_m <= z <= z_max_m
            and x_min_m <= x <= x_max_m
            and abs(y) <= y_half_width_m
        ):
            continue
        voxel = (
            math.floor(x / voxel_size_m),
            math.floor(y / voxel_size_m),
            math.floor(z / voxel_size_m),
        )
        if voxel in occupied_voxels:
            continue
        occupied_voxels.add(voxel)
        points.append((x, y, z))
        if len(points) >= max_points:
            break
    return tuple(points)


class DWALocalPlanner:
    """Select a short forward arc that is safe and remains near Hybrid A*."""

    def __init__(self, config: DWAConfig) -> None:
        self._config = config

    def plan(
        self,
        *,
        reference_path: Sequence[PathPoint],
        current_pose: PathPoint,
        current_velocity: DWAVelocity,
        obstacle_points_body: Iterable[tuple[float, float]],
        previous_reference_index: int = 0,
    ) -> DWAPlan:
        if len(reference_path) < 2:
            raise ValueError("DWA requires at least two global reference points")

        start_index = _nearest_reference_index(
            reference_path,
            current_pose,
            previous_reference_index,
            self._config.reference_search_window_points,
        )
        target_index = _lookahead_index(
            reference_path,
            start_index,
            self._config.reference_lookahead_m,
        )
        reference_end = min(
            len(reference_path),
            start_index + self._config.reference_search_window_points,
        )
        local_reference = reference_path[max(0, start_index - 3) : reference_end]
        reference_distances = _cumulative_distances(reference_path)
        obstacles = tuple(
            (float(x), float(y))
            for x, y in obstacle_points_body
            if math.isfinite(x) and math.isfinite(y)
        )
        obstacle_index = _ObstacleIndex(
            obstacles,
            cell_size_m=self._config.obstacle_clearance_m,
        )

        best: tuple[
            float,
            tuple[PathPoint, ...],
            DWAVelocity,
            float | None,
        ] | None = None
        for velocity in self._velocity_candidates(current_velocity):
            body_path = _simulate_arc(
                velocity,
                self._config.prediction_horizon_s,
                self._config.simulation_step_s,
            )
            clearance = _path_clearance(
                body_path,
                obstacle_index,
                self._config.obstacle_clearance_m,
            )
            if clearance is not None and clearance <= self._config.obstacle_clearance_m:
                continue
            map_path = _to_map_path(body_path, current_pose)
            deviations = tuple(
                _distance_to_reference(point, local_reference)
                for point in map_path[1:]
            )
            if not deviations or max(deviations) > self._config.max_reference_deviation_m:
                continue

            endpoint = map_path[-1]
            endpoint_index = min(
                range(start_index, reference_end),
                key=lambda index: _distance_squared(endpoint, reference_path[index]),
            )
            target = reference_path[target_index]
            progress_m = max(
                0.0,
                reference_distances[endpoint_index]
                - reference_distances[start_index],
            )
            goal_distance_m = math.hypot(endpoint.x - target.x, endpoint.y - target.y)
            heading_error_rad = abs(_wrap_angle(endpoint.yaw - target.yaw))
            clearance_for_score = (
                self._config.obstacle_clearance_m * 3.0
                if clearance is None
                else min(clearance, self._config.obstacle_clearance_m * 3.0)
            )
            score = (
                self._config.progress_weight * progress_m
                - self._config.path_distance_weight
                * (sum(deviations) / len(deviations))
                - self._config.goal_distance_weight * goal_distance_m
                - self._config.heading_weight * heading_error_rad
                + self._config.clearance_weight * clearance_for_score
                + self._config.speed_weight * velocity.linear_mps
                - self._config.yaw_rate_weight * abs(velocity.yaw_rate_radps)
            )
            if best is None or score > best[0]:
                best = (score, map_path, velocity, clearance)

        if best is None:
            raise DWAPlanningError("no collision-free DWA trajectory")

        _, path, selected_velocity, clearance = best
        avoiding = bool(obstacles) and abs(selected_velocity.yaw_rate_radps) > 1e-3
        return DWAPlan(
            path=path,
            reference_start_index=start_index,
            reference_target_index=target_index,
            selected_velocity=selected_velocity,
            minimum_clearance_m=clearance,
            obstacle_count=len(obstacles),
            status="DWA_AVOIDING" if avoiding else "DWA_TRACKING",
        )

    def _velocity_candidates(
        self,
        current: DWAVelocity,
    ) -> tuple[DWAVelocity, ...]:
        config = self._config
        current_speed = max(0.0, float(current.linear_mps))
        speed_min = max(
            config.min_speed_mps,
            current_speed - config.max_deceleration_mps2 * config.control_interval_s,
        )
        speed_max = min(
            config.max_speed_mps,
            max(
                config.min_speed_mps,
                current_speed
                + config.max_acceleration_mps2 * config.control_interval_s,
            ),
        )
        if speed_min > speed_max:
            speed_min = speed_max

        candidates: list[DWAVelocity] = []
        for speed in _linspace(speed_min, speed_max, config.speed_sample_count):
            turning_limit = speed / config.min_turning_radius_m
            yaw_min = max(
                -config.max_yaw_rate_radps,
                current.yaw_rate_radps
                - config.max_yaw_acceleration_radps2 * config.control_interval_s,
                -turning_limit,
            )
            yaw_max = min(
                config.max_yaw_rate_radps,
                current.yaw_rate_radps
                + config.max_yaw_acceleration_radps2 * config.control_interval_s,
                turning_limit,
            )
            if yaw_min > yaw_max:
                continue
            for yaw_rate in _linspace(
                yaw_min,
                yaw_max,
                config.yaw_rate_sample_count,
            ):
                candidates.append(DWAVelocity(speed, yaw_rate))
        return tuple(candidates)


class _ObstacleIndex:
    def __init__(
        self,
        points: Sequence[tuple[float, float]],
        *,
        cell_size_m: float,
    ) -> None:
        self._cell_size_m = cell_size_m
        cells: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for point in points:
            cells.setdefault(self._cell(point[0], point[1]), []).append(point)
        self._cells = cells

    def nearest_distance(
        self,
        x: float,
        y: float,
        *,
        search_radius_cells: int,
    ) -> float | None:
        center_x, center_y = self._cell(x, y)
        nearest_squared: float | None = None
        for cell_y in range(
            center_y - search_radius_cells,
            center_y + search_radius_cells + 1,
        ):
            for cell_x in range(
                center_x - search_radius_cells,
                center_x + search_radius_cells + 1,
            ):
                for obstacle_x, obstacle_y in self._cells.get((cell_x, cell_y), ()):
                    distance_squared = (x - obstacle_x) ** 2 + (y - obstacle_y) ** 2
                    if nearest_squared is None or distance_squared < nearest_squared:
                        nearest_squared = distance_squared
        return math.sqrt(nearest_squared) if nearest_squared is not None else None

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            math.floor(x / self._cell_size_m),
            math.floor(y / self._cell_size_m),
        )


def _path_clearance(
    path: Sequence[tuple[float, float, float]],
    obstacles: _ObstacleIndex,
    required_clearance_m: float,
) -> float | None:
    minimum: float | None = None
    for x, y, _ in path:
        distance = obstacles.nearest_distance(x, y, search_radius_cells=3)
        if distance is None:
            continue
        minimum = distance if minimum is None else min(minimum, distance)
        if minimum <= required_clearance_m:
            return minimum
    return minimum


def _simulate_arc(
    velocity: DWAVelocity,
    horizon_s: float,
    step_s: float,
) -> tuple[tuple[float, float, float], ...]:
    x = 0.0
    y = 0.0
    yaw = 0.0
    path = [(x, y, yaw)]
    steps = max(1, math.ceil(horizon_s / step_s))
    dt = horizon_s / steps
    for _ in range(steps):
        next_yaw = yaw + velocity.yaw_rate_radps * dt
        if abs(velocity.yaw_rate_radps) < 1e-9:
            x += velocity.linear_mps * math.cos(yaw) * dt
            y += velocity.linear_mps * math.sin(yaw) * dt
        else:
            radius = velocity.linear_mps / velocity.yaw_rate_radps
            x += radius * (math.sin(next_yaw) - math.sin(yaw))
            y -= radius * (math.cos(next_yaw) - math.cos(yaw))
        yaw = next_yaw
        path.append((x, y, yaw))
    return tuple(path)


def _to_map_path(
    body_path: Sequence[tuple[float, float, float]],
    current_pose: PathPoint,
) -> tuple[PathPoint, ...]:
    cos_yaw = math.cos(current_pose.yaw)
    sin_yaw = math.sin(current_pose.yaw)
    return tuple(
        PathPoint(
            x=current_pose.x + cos_yaw * x - sin_yaw * y,
            y=current_pose.y + sin_yaw * x + cos_yaw * y,
            yaw=_wrap_angle(current_pose.yaw + yaw),
        )
        for x, y, yaw in body_path
    )


def _nearest_reference_index(
    reference_path: Sequence[PathPoint],
    current_pose: PathPoint,
    previous_reference_index: int,
    search_window_points: int,
) -> int:
    start = max(0, min(previous_reference_index, len(reference_path) - 2) - 3)
    end = min(len(reference_path), start + search_window_points)
    return min(
        range(start, end),
        key=lambda index: (_distance_squared(current_pose, reference_path[index]), index),
    )


def _lookahead_index(
    reference_path: Sequence[PathPoint],
    start_index: int,
    lookahead_m: float,
) -> int:
    distance = 0.0
    for index in range(start_index + 1, len(reference_path)):
        distance += math.hypot(
            reference_path[index].x - reference_path[index - 1].x,
            reference_path[index].y - reference_path[index - 1].y,
        )
        if distance >= lookahead_m:
            return index
    return len(reference_path) - 1


def _cumulative_distances(reference_path: Sequence[PathPoint]) -> tuple[float, ...]:
    distances = [0.0]
    for previous, current in zip(reference_path, reference_path[1:]):
        distances.append(
            distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    return tuple(distances)


def _distance_to_reference(
    point: PathPoint,
    reference_path: Sequence[PathPoint],
) -> float:
    return math.sqrt(
        min(_distance_squared(point, reference) for reference in reference_path)
    )


def _distance_squared(left: PathPoint, right: PathPoint) -> float:
    return (left.x - right.x) ** 2 + (left.y - right.y) ** 2


def _linspace(start: float, stop: float, count: int) -> tuple[float, ...]:
    if count == 1 or abs(stop - start) < 1e-12:
        return (start,)
    step = (stop - start) / (count - 1)
    return tuple(start + step * index for index in range(count))


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
