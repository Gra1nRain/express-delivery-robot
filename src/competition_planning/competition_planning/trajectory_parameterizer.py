"""Offline speed and time parameterization for semantic global paths."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

from competition_planning.semantic_planner import (
    PathPoint,
    PlanFailure,
    RoutePlan,
    StepPlan,
    plan_route,
)


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    v: float
    yaw_rate: float
    t: float
    ref_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 4),
            "s": round(self.s, 4),
            "curvature": round(self.curvature, 6),
            "v": round(self.v, 4),
            "yaw_rate": round(self.yaw_rate, 4),
            "t": round(self.t, 4),
        }
        if self.ref_id:
            result["ref_id"] = self.ref_id
        return result


@dataclass(frozen=True)
class OptimizedStepTrajectory:
    step_id: str
    step_type: str
    corridor_ref: str
    target_ref: str
    target_source: str
    planner_plugin: str
    smoother_plugin: str
    points: tuple[TrajectoryPoint, ...]

    @property
    def duration_s(self) -> float:
        return self.points[-1].t if self.points else 0.0

    @property
    def path_length_m(self) -> float:
        return self.points[-1].s if self.points else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "step_id": self.step_id,
            "type": self.step_type,
            "corridor_ref": self.corridor_ref,
            "target_ref": self.target_ref,
            "target_source": self.target_source,
            "planner_plugin": self.planner_plugin,
            "smoother_plugin": self.smoother_plugin,
            "point_count": len(self.points),
            "path_length_m": round(self.path_length_m, 3),
            "duration_s": round(self.duration_s, 3),
            "points": [point.to_dict() for point in self.points],
        }


@dataclass(frozen=True)
class OptimizedRouteTrajectory:
    frame_id: str
    route_name: str
    trajectories: tuple[OptimizedStepTrajectory, ...]
    failures: tuple[PlanFailure, ...]

    @property
    def ok(self) -> bool:
        return not self.failures

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_name": self.route_name,
            "frame_id": self.frame_id,
            "ok": self.ok,
            "trajectories": [trajectory.to_dict() for trajectory in self.trajectories],
            "failures": [failure.to_dict() for failure in self.failures],
        }


def optimize_route_trajectory(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    planning_config: dict[str, Any] | None = None,
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedRouteTrajectory:
    """Plan a semantic route and return its optimized trajectory artifact.

    This is the external seam for offline tools and future ROS adapters. Callers
    should not need to orchestrate global planning and speed/time
    parameterization separately.
    """

    route_plan = plan_route(route, semantic_map, planning_config)
    return parameterize_route_plan(route_plan, semantic_map, optimizer_config)


def parameterize_route_plan(
    route_plan: RoutePlan,
    semantic_map: dict[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedRouteTrajectory:
    trajectories: list[OptimizedStepTrajectory] = []
    failures = list(route_plan.failures)
    for plan in route_plan.plans:
        try:
            trajectories.append(parameterize_step_plan(plan, semantic_map, optimizer_config))
        except ValueError as exc:
            failures.append(
                PlanFailure(
                    step_id=plan.step_id,
                    step_type=plan.step_type,
                    reason="trajectory_parameterization_failed",
                    detail=str(exc),
                )
            )
    return OptimizedRouteTrajectory(
        frame_id=route_plan.frame_id,
        route_name=route_plan.route_name,
        trajectories=tuple(trajectories),
        failures=tuple(failures),
    )


def parameterize_step_plan(
    plan: StepPlan,
    semantic_map: dict[str, Any],
    optimizer_config: dict[str, Any] | None = None,
) -> OptimizedStepTrajectory:
    """Convert one geometric global path into a time-parameterized trajectory."""

    if not plan.path:
        raise ValueError(f"{plan.step_id} has no path points")

    params = (optimizer_config or {}).get("trajectory_optimizer", {})
    max_speed_mps = float(params.get("max_speed_mps", 0.50))
    max_acceleration_mps2 = _positive_float(params, "max_acceleration_mps2", 0.30)
    max_deceleration_mps2 = _positive_float(params, "max_deceleration_mps2", 0.50)
    max_lateral_acceleration_mps2 = _positive_float(
        params,
        "max_lateral_acceleration_mps2",
        0.20,
    )
    zone_speed_limits = {
        str(key): float(value)
        for key, value in params.get("obstacle_zone_speed_limits_mps", {}).items()
    }

    distances = _cumulative_distances(plan.path)
    curvatures = _path_curvatures(plan.path)
    stop_refs = _semantic_stop_refs(semantic_map)
    obstacle_zones = _obstacle_zones(semantic_map)

    speed_caps = [
        _speed_cap_for_point(
            point,
            curvature,
            max_speed_mps,
            max_lateral_acceleration_mps2,
            stop_refs,
            obstacle_zones,
            zone_speed_limits,
        )
        for point, curvature in zip(plan.path, curvatures)
    ]
    speeds = _apply_acceleration_limits(
        speed_caps,
        distances,
        max_acceleration_mps2,
        max_deceleration_mps2,
    )
    times = _integrate_times(distances, speeds)

    points = tuple(
        TrajectoryPoint(
            x=path_point.x,
            y=path_point.y,
            yaw=path_point.yaw,
            s=distance,
            curvature=curvature,
            v=speed,
            yaw_rate=speed * curvature,
            t=timestamp,
            ref_id=path_point.ref_id,
        )
        for path_point, distance, curvature, speed, timestamp in zip(
            plan.path,
            distances,
            curvatures,
            speeds,
            times,
        )
    )
    return OptimizedStepTrajectory(
        step_id=plan.step_id,
        step_type=plan.step_type,
        corridor_ref=plan.corridor_ref,
        target_ref=plan.target_ref,
        target_source=plan.target_source,
        planner_plugin=plan.planner_plugin,
        smoother_plugin=plan.smoother_plugin,
        points=points,
    )


def _positive_float(params: dict[str, Any], key: str, default: float) -> float:
    value = float(params.get(key, default))
    if value <= 0.0:
        raise ValueError(f"trajectory_optimizer.{key} must be positive")
    return value


def _cumulative_distances(path: Sequence[PathPoint]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(path, path[1:]):
        distances.append(
            distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    return distances


def _path_curvatures(path: Sequence[PathPoint]) -> list[float]:
    if len(path) < 3:
        return [0.0 for _ in path]

    curvatures = [0.0 for _ in path]
    for index, (first, second, third) in enumerate(zip(path, path[1:], path[2:]), start=1):
        curvatures[index] = _signed_curvature(first, second, third)
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]
    return curvatures


def _signed_curvature(first: PathPoint, second: PathPoint, third: PathPoint) -> float:
    ab = math.hypot(second.x - first.x, second.y - first.y)
    bc = math.hypot(third.x - second.x, third.y - second.y)
    ca = math.hypot(first.x - third.x, first.y - third.y)
    denominator = ab * bc * ca
    if denominator <= 1e-12:
        return 0.0
    cross = (second.x - first.x) * (third.y - first.y) - (
        second.y - first.y
    ) * (third.x - first.x)
    return 2.0 * cross / denominator


def _semantic_stop_refs(semantic_map: dict[str, Any]) -> set[str]:
    refs: set[str] = set()
    for stop_line in semantic_map.get("stop_lines", []):
        if isinstance(stop_line, dict) and stop_line.get("point_ref"):
            refs.add(str(stop_line["point_ref"]))
    for dock_pose in semantic_map.get("dock_poses", []):
        if isinstance(dock_pose, dict) and dock_pose.get("point_ref"):
            refs.add(str(dock_pose["point_ref"]))
    return refs


def _obstacle_zones(
    semantic_map: dict[str, Any],
) -> list[tuple[str, list[tuple[float, float]]]]:
    zones: list[tuple[str, list[tuple[float, float]]]] = []
    for zone in semantic_map.get("obstacle_zones", []):
        if not isinstance(zone, dict):
            continue
        polygon = _polygon(zone.get("boundary"))
        if polygon:
            zones.append((str(zone.get("semantic_type", "")), polygon))
    return zones


def _polygon(raw: Any) -> list[tuple[float, float]]:
    if not isinstance(raw, list):
        return []
    polygon: list[tuple[float, float]] = []
    for vertex in raw:
        if not isinstance(vertex, list | tuple) or len(vertex) < 2:
            return []
        polygon.append((float(vertex[0]), float(vertex[1])))
    return polygon


def _speed_cap_for_point(
    point: PathPoint,
    curvature: float,
    max_speed_mps: float,
    max_lateral_acceleration_mps2: float,
    stop_refs: set[str],
    obstacle_zones: list[tuple[str, list[tuple[float, float]]]],
    zone_speed_limits: dict[str, float],
) -> float:
    if point.ref_id in stop_refs:
        return 0.0

    cap = max_speed_mps
    for semantic_type, polygon in obstacle_zones:
        if semantic_type in zone_speed_limits and _point_in_polygon(point.x, point.y, polygon):
            cap = min(cap, zone_speed_limits[semantic_type])
    if abs(curvature) > 1e-9:
        cap = min(cap, math.sqrt(max_lateral_acceleration_mps2 / abs(curvature)))
    return max(0.0, cap)


def _apply_acceleration_limits(
    speed_caps: Sequence[float],
    distances: Sequence[float],
    max_acceleration_mps2: float,
    max_deceleration_mps2: float,
) -> list[float]:
    speeds = list(speed_caps)
    for index in range(1, len(speeds)):
        ds = distances[index] - distances[index - 1]
        if ds <= 1e-12:
            speeds[index] = min(speeds[index], speeds[index - 1])
            continue
        reachable = math.sqrt(speeds[index - 1] ** 2 + 2.0 * max_acceleration_mps2 * ds)
        speeds[index] = min(speeds[index], reachable)
    for index in range(len(speeds) - 2, -1, -1):
        ds = distances[index + 1] - distances[index]
        if ds <= 1e-12:
            speeds[index] = min(speeds[index], speeds[index + 1])
            continue
        reachable = math.sqrt(speeds[index + 1] ** 2 + 2.0 * max_deceleration_mps2 * ds)
        speeds[index] = min(speeds[index], reachable)
    return speeds


def _integrate_times(distances: Sequence[float], speeds: Sequence[float]) -> list[float]:
    times = [0.0]
    for previous_index, current_index in zip(range(len(speeds) - 1), range(1, len(speeds))):
        ds = distances[current_index] - distances[previous_index]
        if ds <= 1e-12:
            times.append(times[-1])
            continue
        average_speed = (speeds[previous_index] + speeds[current_index]) * 0.5
        if average_speed <= 1e-12:
            raise ValueError("trajectory contains a non-moving interval with positive distance")
        times.append(times[-1] + ds / average_speed)
    return times


def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if _point_on_segment_xy(x, y, x1, y1, x2, y2):
            return True
        crosses = (y1 > y) != (y2 > y)
        if crosses:
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
    return inside


def _point_on_segment_xy(
    x: float,
    y: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> bool:
    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
    if abs(cross) > 1e-9:
        return False
    dot = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)
    if dot < -1e-9:
        return False
    length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    return dot <= length_sq + 1e-9
