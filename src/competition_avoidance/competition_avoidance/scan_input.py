"""Convert a two-dimensional LaserScan into body-frame obstacle points."""

from __future__ import annotations

import math
from typing import Iterable

from competition_avoidance.perception import ObstacleDetection


def scan_ranges_to_points(
    *,
    ranges: Iterable[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
) -> tuple[tuple[float, float, float], ...]:
    """Return finite scan samples as planar XYZ points."""

    if not math.isfinite(angle_increment_rad) or angle_increment_rad == 0.0:
        raise ValueError("angle_increment_rad must be finite and non-zero")
    if (
        not math.isfinite(range_min_m)
        or not math.isfinite(range_max_m)
        or range_min_m < 0.0
        or range_max_m <= range_min_m
    ):
        raise ValueError("scan range bounds are invalid")

    points: list[tuple[float, float, float]] = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or distance < range_min_m
            or distance > range_max_m
        ):
            continue
        angle = angle_min_rad + index * angle_increment_rad
        points.append(
            (
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.0,
            )
        )
    return tuple(points)


def mark_planar_detection(
    detection: ObstacleDetection,
) -> ObstacleDetection:
    """Mark a 2D cluster as eligible for motion-based classification."""

    return ObstacleDetection(
        x=detection.x,
        y=detection.y,
        z=detection.z,
        length_m=detection.length_m,
        width_m=detection.width_m,
        height_m=detection.height_m,
        point_count=detection.point_count,
        classification="SCAN_CANDIDATE",
        confidence=0.40,
    )
