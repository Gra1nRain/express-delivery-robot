"""Near-field point-cloud stop gate for supervised low-speed runs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class ProximityStopConfig:
    x_min_m: float = 0.25
    stop_distance_m: float = 0.55
    front_half_angle_rad: float = 0.4363
    lateral_half_width_m: float = 0.45
    z_min_m: float = -0.25
    z_max_m: float = 0.80
    min_points: int = 3

    def __post_init__(self) -> None:
        if self.x_min_m < 0.0:
            raise ValueError("x_min_m must be non-negative")
        if self.stop_distance_m <= self.x_min_m:
            raise ValueError("stop_distance_m must be greater than x_min_m")
        if not (0.0 < self.front_half_angle_rad < math.pi / 2.0):
            raise ValueError("front_half_angle_rad must be in (0, pi/2)")
        if self.lateral_half_width_m <= 0.0:
            raise ValueError("lateral_half_width_m must be positive")
        if self.z_max_m <= self.z_min_m:
            raise ValueError("z_max_m must be greater than z_min_m")
        if self.min_points < 1:
            raise ValueError("min_points must be at least 1")


def count_points_in_stop_box(
    points: Iterable[tuple[float, float, float]],
    config: ProximityStopConfig,
) -> int:
    count = 0
    for x, y, z in points:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        range_xy = math.hypot(x, y)
        if not (config.z_min_m <= z <= config.z_max_m):
            continue
        in_front_sector = (
            config.x_min_m <= range_xy <= config.stop_distance_m
            and x > 0.0
            and abs(math.atan2(y, x)) <= config.front_half_angle_rad
        )
        in_body_corridor = (
            config.x_min_m <= x <= config.stop_distance_m
            and abs(y) <= config.lateral_half_width_m
        )
        if not (in_front_sector or in_body_corridor):
            continue
        count += 1
    return count


def should_stop_for_points(
    points: Iterable[tuple[float, float, float]],
    config: ProximityStopConfig,
) -> tuple[bool, int]:
    count = count_points_in_stop_box(points, config)
    return count >= config.min_points, count
