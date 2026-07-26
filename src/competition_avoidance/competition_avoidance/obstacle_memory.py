"""Short-lived map-frame memory for sparse live obstacle returns."""

from __future__ import annotations

import math
from typing import Iterable


class MapObstacleMemory:
    """Retain recent obstacle cells while returning them in the current body frame."""

    def __init__(self, *, ttl_s: float, resolution_m: float) -> None:
        if ttl_s <= 0.0:
            raise ValueError("ttl_s must be positive")
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        self._ttl_s = ttl_s
        self._resolution_m = resolution_m
        self._cells: dict[tuple[int, int], tuple[float, float, float, float]] = {}
        self._latest_timestamp_s: float | None = None

    def update(
        self,
        points_body: Iterable[tuple[float, float, float]],
        *,
        translation_x_m: float,
        translation_y_m: float,
        yaw_rad: float,
        timestamp_s: float,
    ) -> tuple[tuple[float, float, float], ...]:
        if not all(
            math.isfinite(value)
            for value in (
                translation_x_m,
                translation_y_m,
                yaw_rad,
                timestamp_s,
            )
        ):
            raise ValueError("pose and timestamp must be finite")
        if self._latest_timestamp_s is not None and timestamp_s < self._latest_timestamp_s:
            self._cells.clear()
        self._latest_timestamp_s = timestamp_s

        oldest_allowed_s = timestamp_s - self._ttl_s
        self._cells = {
            cell: sample
            for cell, sample in self._cells.items()
            if sample[3] >= oldest_allowed_s
        }

        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        for x_body, y_body, z_body in points_body:
            if not all(math.isfinite(value) for value in (x_body, y_body, z_body)):
                continue
            x_map = translation_x_m + cosine * x_body - sine * y_body
            y_map = translation_y_m + sine * x_body + cosine * y_body
            cell = (
                math.floor(x_map / self._resolution_m),
                math.floor(y_map / self._resolution_m),
            )
            self._cells[cell] = (x_map, y_map, z_body, timestamp_s)

        retained: list[tuple[float, float, float]] = []
        for x_map, y_map, z_body, _ in self._cells.values():
            delta_x = x_map - translation_x_m
            delta_y = y_map - translation_y_m
            retained.append(
                (
                    cosine * delta_x + sine * delta_y,
                    -sine * delta_x + cosine * delta_y,
                    z_body,
                )
            )
        return tuple(sorted(retained, key=lambda point: (point[0], point[1], point[2])))
