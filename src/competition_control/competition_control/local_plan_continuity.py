"""Geometry-only continuity checks for online local trajectories."""

from __future__ import annotations

import math
from typing import Protocol, Sequence


class PathPose(Protocol):
    x: float
    y: float
    yaw: float


def local_paths_are_equivalent(
    previous: Sequence[PathPose],
    current: Sequence[PathPose],
    *,
    position_tolerance_m: float = 0.05,
    heading_tolerance_rad: float = math.radians(5.0),
) -> bool:
    """Return whether ``current`` keeps a geometrically identical future path.

    Local replans prepend the latest vehicle pose, so point zero is deliberately
    ignored.  A consumed suffix is reusable only when at least two future points,
    including the endpoint, still match.  Real bypass, rejoin, or extension changes
    therefore continue to replace the controller trajectory.
    """

    if position_tolerance_m < 0.0 or heading_tolerance_rad < 0.0:
        raise ValueError("path equivalence tolerances must be non-negative")
    if len(previous) < 3 or len(current) < 3:
        return False

    current_future = current[1:]
    if len(current_future) < 2:
        return False
    if not _finite_path(current_future):
        return False

    for start_index in range(1, len(previous)):
        previous_future = previous[start_index:]
        if len(previous_future) != len(current_future):
            continue
        if not _finite_path(previous_future):
            continue
        if all(
            _poses_match(
                old,
                new,
                position_tolerance_m=position_tolerance_m,
                heading_tolerance_rad=heading_tolerance_rad,
            )
            for old, new in zip(previous_future, current_future)
        ):
            return True
    return False


def nearest_path_point_index(
    path: Sequence[PathPose],
    checkpoint: PathPose,
    *,
    max_distance_m: float,
) -> int | None:
    """Find a checkpoint on local geometry without snapping from far away."""

    if max_distance_m < 0.0:
        raise ValueError("max_distance_m must be non-negative")
    if not path or not _finite_path(path):
        return None
    distances = tuple(
        math.hypot(
            float(point.x) - float(checkpoint.x),
            float(point.y) - float(checkpoint.y),
        )
        for point in path
    )
    index = min(range(len(path)), key=distances.__getitem__)
    return index if distances[index] <= max_distance_m else None


def _finite_path(path: Sequence[PathPose]) -> bool:
    return all(
        math.isfinite(float(value))
        for point in path
        for value in (point.x, point.y, point.yaw)
    )


def _poses_match(
    first: PathPose,
    second: PathPose,
    *,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
) -> bool:
    position_error_m = math.hypot(
        float(first.x) - float(second.x),
        float(first.y) - float(second.y),
    )
    heading_error_rad = abs(
        math.atan2(
            math.sin(float(first.yaw) - float(second.yaw)),
            math.cos(float(first.yaw) - float(second.yaw)),
        )
    )
    return (
        position_error_m <= position_tolerance_m
        and heading_error_rad <= heading_tolerance_rad
        and getattr(first, "ref_id", None) == getattr(second, "ref_id", None)
    )
