"""Pure 2D helpers for anchoring a body-frame grid in a map frame."""

from __future__ import annotations

import math


def transform_grid_origin(
    *,
    origin_x_m: float,
    origin_y_m: float,
    translation_x_m: float,
    translation_y_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """Transform one body-frame grid origin into the map frame."""

    return (
        translation_x_m
        + math.cos(yaw_rad) * origin_x_m
        - math.sin(yaw_rad) * origin_y_m,
        translation_y_m
        + math.sin(yaw_rad) * origin_x_m
        + math.cos(yaw_rad) * origin_y_m,
    )


def yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return a planar quaternion as x, y, z, w."""

    half_yaw = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
