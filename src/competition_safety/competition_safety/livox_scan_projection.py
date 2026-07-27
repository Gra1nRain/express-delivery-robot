"""Pure Livox point projection into a planar laser scan."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class ScanProjectionConfig:
    min_height_m: float = -0.25
    max_height_m: float = 0.80
    angle_min_rad: float = -math.pi
    angle_max_rad: float = math.pi
    angle_increment_rad: float = math.radians(0.5)
    range_min_m: float = 0.10
    range_max_m: float = 6.00
    sensor_to_body_x_m: float = 0.0
    sensor_to_body_y_m: float = 0.0
    sensor_to_body_z_m: float = 0.0
    sensor_to_body_yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.max_height_m <= self.min_height_m:
            raise ValueError("max_height_m must exceed min_height_m")
        if self.angle_max_rad <= self.angle_min_rad:
            raise ValueError("angle_max_rad must exceed angle_min_rad")
        if self.angle_increment_rad <= 0.0:
            raise ValueError("angle_increment_rad must be positive")
        if self.range_min_m < 0.0 or self.range_max_m <= self.range_min_m:
            raise ValueError("scan range limits are invalid")

    @property
    def bin_count(self) -> int:
        return max(
            1,
            int(
                math.ceil(
                    (self.angle_max_rad - self.angle_min_rad)
                    / self.angle_increment_rad
                )
            ),
        )


def project_points_to_scan_ranges(
    points: Iterable[tuple[float, float, float]],
    config: ScanProjectionConfig,
) -> tuple[float, ...]:
    """Project finite height-filtered sensor points into nearest planar ranges."""

    ranges = [math.inf] * config.bin_count
    cos_yaw = math.cos(config.sensor_to_body_yaw_rad)
    sin_yaw = math.sin(config.sensor_to_body_yaw_rad)
    angle_span = config.angle_max_rad - config.angle_min_rad

    for sensor_x, sensor_y, sensor_z in points:
        if not (
            math.isfinite(sensor_x)
            and math.isfinite(sensor_y)
            and math.isfinite(sensor_z)
        ):
            continue
        body_x = (
            config.sensor_to_body_x_m
            + cos_yaw * float(sensor_x)
            - sin_yaw * float(sensor_y)
        )
        body_y = (
            config.sensor_to_body_y_m
            + sin_yaw * float(sensor_x)
            + cos_yaw * float(sensor_y)
        )
        body_z = config.sensor_to_body_z_m + float(sensor_z)
        if not config.min_height_m <= body_z <= config.max_height_m:
            continue
        distance = math.hypot(body_x, body_y)
        if not config.range_min_m <= distance <= config.range_max_m:
            continue
        angle = math.atan2(body_y, body_x)
        relative_angle = (angle - config.angle_min_rad) % (2.0 * math.pi)
        if relative_angle >= angle_span:
            continue
        index = min(
            int(relative_angle / config.angle_increment_rad),
            config.bin_count - 1,
        )
        ranges[index] = min(ranges[index], distance)

    return tuple(ranges)
