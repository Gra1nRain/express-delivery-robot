"""Path smoothing utilities for global trajectory handoff."""

from __future__ import annotations

import math
from typing import Sequence

from competition_planning.semantic_planner import PathPoint


class CubicBezierSmoother:
    """Smooth semantic anchor paths with piecewise cubic Bezier curves.

    Points with ``ref_id`` are treated as hard anchors. The curve passes through
    every anchor exactly, uses each anchor yaw as the local tangent direction
    when it is compatible with the route direction, and samples intermediate
    points at a stable spacing.
    """

    def __init__(self, *, sample_spacing_m: float, tangent_scale: float) -> None:
        self._sample_spacing_m = max(0.05, sample_spacing_m)
        self._tangent_scale = max(0.0, tangent_scale)

    def smooth(self, path: Sequence[PathPoint]) -> tuple[PathPoint, ...]:
        anchors = [point for point in path if point.ref_id]
        if len(anchors) < 3:
            return tuple(path)

        tangents = _anchor_tangents(anchors, self._tangent_scale)
        smoothed: list[PathPoint] = []
        for index, (start, end) in enumerate(zip(anchors, anchors[1:])):
            start_tangent = tangents[index]
            end_tangent = tangents[index + 1]
            controls = (
                (start.x, start.y),
                (start.x + start_tangent[0] / 3.0, start.y + start_tangent[1] / 3.0),
                (end.x - end_tangent[0] / 3.0, end.y - end_tangent[1] / 3.0),
                (end.x, end.y),
            )
            samples = max(
                1,
                math.ceil(_control_polygon_length(controls) / self._sample_spacing_m),
            )
            for sample_index in range(samples + 1):
                if smoothed and sample_index == 0:
                    continue
                t = sample_index / samples
                x, y = _bezier_xy(controls, t)
                dx, dy = _bezier_derivative_xy(controls, t)
                yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-12 else start.yaw
                ref_id = (
                    start.ref_id
                    if sample_index == 0
                    else end.ref_id
                    if sample_index == samples
                    else None
                )
                smoothed.append(PathPoint(x=x, y=y, yaw=yaw, ref_id=ref_id))
        return tuple(smoothed)


def _anchor_tangents(
    anchors: Sequence[PathPoint],
    tangent_scale: float,
) -> list[tuple[float, float]]:
    tangents: list[tuple[float, float]] = []
    for index, point in enumerate(anchors):
        if index == 0:
            next_point = anchors[index + 1]
            route_tangent = (next_point.x - point.x, next_point.y - point.y)
        elif index == len(anchors) - 1:
            previous = anchors[index - 1]
            route_tangent = (point.x - previous.x, point.y - previous.y)
        else:
            previous = anchors[index - 1]
            next_point = anchors[index + 1]
            route_tangent = (
                (next_point.x - previous.x) * 0.5,
                (next_point.y - previous.y) * 0.5,
            )
        tangents.append(_compatible_yaw_tangent(point, route_tangent, tangent_scale))
    return tangents


def _compatible_yaw_tangent(
    point: PathPoint,
    route_tangent: tuple[float, float],
    tangent_scale: float,
) -> tuple[float, float]:
    length = math.hypot(route_tangent[0], route_tangent[1])
    if length <= 1e-9:
        return (0.0, 0.0)

    route_yaw = math.atan2(route_tangent[1], route_tangent[0])
    if _angle_delta(point.yaw, route_yaw) <= math.radians(120.0):
        return (
            math.cos(point.yaw) * length * tangent_scale,
            math.sin(point.yaw) * length * tangent_scale,
        )
    return (route_tangent[0] * tangent_scale, route_tangent[1] * tangent_scale)


def _angle_delta(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + math.pi) % (2.0 * math.pi) - math.pi)


def _control_polygon_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


def _bezier_xy(
    controls: Sequence[tuple[float, float]],
    t: float,
) -> tuple[float, float]:
    p0, p1, p2, p3 = controls
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def _bezier_derivative_xy(
    controls: Sequence[tuple[float, float]],
    t: float,
) -> tuple[float, float]:
    p0, p1, p2, p3 = controls
    u = 1.0 - t
    return (
        3.0 * u * u * (p1[0] - p0[0])
        + 6.0 * u * t * (p2[0] - p1[0])
        + 3.0 * t * t * (p3[0] - p2[0]),
        3.0 * u * u * (p1[1] - p0[1])
        + 6.0 * u * t * (p2[1] - p1[1])
        + 3.0 * t * t * (p3[1] - p2[1]),
    )
