from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from competition_localization.planar_transform import PlanarTransform
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class OccupancyDistanceField:
    distances_m: np.ndarray
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float

    @classmethod
    def from_occupancy(
        cls,
        data: Sequence[int] | np.ndarray,
        *,
        width: int,
        height: int,
        resolution_m: float,
        origin_x_m: float,
        origin_y_m: float,
        origin_yaw_rad: float,
        occupied_threshold: int = 65,
    ) -> "OccupancyDistanceField":
        if width <= 0 or height <= 0 or resolution_m <= 0.0:
            raise ValueError(
                "occupancy grid dimensions and resolution must be positive"
            )
        occupancy = np.asarray(data, dtype=np.int16)
        if occupancy.size != width * height:
            raise ValueError("occupancy data size does not match width * height")
        occupied = occupancy.reshape((height, width)) >= occupied_threshold
        if not np.any(occupied):
            raise ValueError("occupancy grid contains no occupied cells")
        distances_m = distance_transform_edt(~occupied) * resolution_m
        return cls(
            distances_m=distances_m,
            resolution_m=resolution_m,
            origin_x_m=origin_x_m,
            origin_y_m=origin_y_m,
            origin_yaw_rad=origin_yaw_rad,
        )

    def residuals(self, points_xy_m: np.ndarray, max_residual_m: float) -> np.ndarray:
        points = np.asarray(points_xy_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points_xy_m must have shape (N, 2)")
        dx = points[:, 0] - self.origin_x_m
        dy = points[:, 1] - self.origin_y_m
        cos_yaw = math.cos(self.origin_yaw_rad)
        sin_yaw = math.sin(self.origin_yaw_rad)
        grid_x = (cos_yaw * dx + sin_yaw * dy) / self.resolution_m - 0.5
        grid_y = (-sin_yaw * dx + cos_yaw * dy) / self.resolution_m - 0.5
        x0 = np.floor(grid_x).astype(int)
        y0 = np.floor(grid_y).astype(int)
        valid = (
            (x0 >= 0)
            & (y0 >= 0)
            & (x0 + 1 < self.distances_m.shape[1])
            & (y0 + 1 < self.distances_m.shape[0])
        )
        residuals = np.full(points.shape[0], max_residual_m, dtype=float)
        if not np.any(valid):
            return residuals
        xv = x0[valid]
        yv = y0[valid]
        fx = grid_x[valid] - xv
        fy = grid_y[valid] - yv
        d00 = self.distances_m[yv, xv]
        d10 = self.distances_m[yv, xv + 1]
        d01 = self.distances_m[yv + 1, xv]
        d11 = self.distances_m[yv + 1, xv + 1]
        residuals[valid] = (
            d00 * (1.0 - fx) * (1.0 - fy)
            + d10 * fx * (1.0 - fy)
            + d01 * (1.0 - fx) * fy
            + d11 * fx * fy
        )
        return np.minimum(residuals, max_residual_m)


@dataclass(frozen=True)
class ScanMatchConfig:
    translation_window_m: float = 0.40
    translation_step_m: float = 0.05
    yaw_window_rad: float = math.radians(10.0)
    yaw_step_rad: float = math.radians(1.0)
    fine_translation_window_m: float = 0.05
    fine_translation_step_m: float = 0.01
    fine_yaw_window_rad: float = math.radians(1.0)
    fine_yaw_step_rad: float = math.radians(0.25)
    max_residual_m: float = 0.50
    inlier_threshold_m: float = 0.10
    min_points: int = 60


@dataclass(frozen=True)
class ScanMatchResult:
    correction_x_m: float
    correction_y_m: float
    correction_yaw_rad: float
    baseline_mean_residual_m: float
    baseline_median_residual_m: float
    baseline_inlier_ratio: float
    best_mean_residual_m: float
    best_median_residual_m: float
    best_p90_residual_m: float
    inlier_ratio: float
    point_count: int
    search_boundary_hit: bool
    confident: bool


@dataclass(frozen=True)
class StationaryResidualSample:
    stamp_s: float
    correction_x_m: float
    correction_y_m: float
    correction_yaw_rad: float


@dataclass(frozen=True)
class StationaryResidualAssessment:
    classification: str
    sample_count: int
    duration_s: float
    translation_change_m: float
    yaw_change_rad: float
    translation_rate_mps: float
    yaw_rate_radps: float


def velocity_is_stationary(
    *,
    linear_speed_mps: float,
    yaw_rate_radps: float,
    odom_age_s: float,
    max_odom_age_s: float,
    linear_threshold_mps: float,
    yaw_rate_threshold_radps: float,
) -> bool:
    return (
        0.0 <= odom_age_s <= max_odom_age_s
        and abs(linear_speed_mps) <= linear_threshold_mps
        and abs(yaw_rate_radps) <= yaw_rate_threshold_radps
    )


def laser_scan_points(
    ranges_m: Sequence[float] | np.ndarray,
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    max_points: int,
) -> np.ndarray:
    if max_points <= 0:
        raise ValueError("max_points must be positive")
    ranges = np.asarray(ranges_m, dtype=float)
    angles = angle_min_rad + np.arange(ranges.size) * angle_increment_rad
    valid = np.isfinite(ranges) & (ranges >= range_min_m) & (ranges <= range_max_m)
    ranges = ranges[valid]
    angles = angles[valid]
    if ranges.size > max_points:
        indices = np.linspace(0, ranges.size - 1, max_points, dtype=int)
        ranges = ranges[indices]
        angles = angles[indices]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def classify_stationary_residuals(
    samples: Sequence[StationaryResidualSample],
    *,
    min_duration_s: float = 120.0,
    drift_translation_m: float = 0.08,
    drift_yaw_rad: float = math.radians(2.0),
    stable_translation_span_m: float = 0.05,
    stable_yaw_span_rad: float = math.radians(1.5),
    fixed_offset_translation_m: float = 0.08,
    fixed_offset_yaw_rad: float = math.radians(2.0),
) -> StationaryResidualAssessment:
    if len(samples) < 2:
        return StationaryResidualAssessment(
            "insufficient_data", len(samples), 0.0, 0.0, 0.0, 0.0, 0.0
        )
    ordered = sorted(samples, key=lambda sample: sample.stamp_s)
    duration_s = max(0.0, ordered[-1].stamp_s - ordered[0].stamp_s)
    translation_change_m = math.hypot(
        ordered[-1].correction_x_m - ordered[0].correction_x_m,
        ordered[-1].correction_y_m - ordered[0].correction_y_m,
    )
    yaw_values = np.unwrap(
        np.array([sample.correction_yaw_rad for sample in ordered], dtype=float)
    )
    yaw_change_rad = abs(float(yaw_values[-1] - yaw_values[0]))
    translation_span_m = math.hypot(
        max(sample.correction_x_m for sample in ordered)
        - min(sample.correction_x_m for sample in ordered),
        max(sample.correction_y_m for sample in ordered)
        - min(sample.correction_y_m for sample in ordered),
    )
    yaw_span_rad = float(np.ptp(yaw_values))
    if duration_s < min_duration_s:
        classification = "insufficient_data"
    elif translation_change_m >= drift_translation_m or yaw_change_rad >= drift_yaw_rad:
        classification = "stationary_drift"
    elif (
        translation_span_m <= stable_translation_span_m
        and yaw_span_rad <= stable_yaw_span_rad
    ):
        initial = ordered[0]
        if (
            math.hypot(initial.correction_x_m, initial.correction_y_m)
            >= fixed_offset_translation_m
            or abs(initial.correction_yaw_rad) >= fixed_offset_yaw_rad
        ):
            classification = "fixed_anchor_offset"
        else:
            classification = "stationary_stable"
    else:
        classification = "stationary_unstable"
    return StationaryResidualAssessment(
        classification=classification,
        sample_count=len(ordered),
        duration_s=duration_s,
        translation_change_m=translation_change_m,
        yaw_change_rad=yaw_change_rad,
        translation_rate_mps=(
            translation_change_m / duration_s if duration_s > 0.0 else 0.0
        ),
        yaw_rate_radps=yaw_change_rad / duration_s if duration_s > 0.0 else 0.0,
    )


def match_scan_to_map(
    field: OccupancyDistanceField,
    points_xy_m: np.ndarray,
    *,
    sensor_xy_m: tuple[float, float],
    config: ScanMatchConfig = ScanMatchConfig(),
) -> ScanMatchResult:
    points = np.asarray(points_xy_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points_xy_m must have shape (N, 2)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < config.min_points:
        raise ValueError(
            f"scan has {len(points)} usable points; need at least {config.min_points}"
        )
    baseline = field.residuals(points, config.max_residual_m)
    coarse = _search(
        field,
        points,
        sensor_xy_m,
        center=(0.0, 0.0, 0.0),
        translation_window_m=config.translation_window_m,
        translation_step_m=config.translation_step_m,
        yaw_window_rad=config.yaw_window_rad,
        yaw_step_rad=config.yaw_step_rad,
        max_residual_m=config.max_residual_m,
    )
    best = _search(
        field,
        points,
        sensor_xy_m,
        center=coarse,
        translation_window_m=config.fine_translation_window_m,
        translation_step_m=config.fine_translation_step_m,
        yaw_window_rad=config.fine_yaw_window_rad,
        yaw_step_rad=config.fine_yaw_step_rad,
        max_residual_m=config.max_residual_m,
    )
    corrected = _correct_points(points, sensor_xy_m, *best)
    residuals = field.residuals(corrected, config.max_residual_m)
    inlier_ratio = float(np.mean(residuals <= config.inlier_threshold_m))
    translation_limit = config.translation_window_m + config.fine_translation_window_m
    yaw_limit = config.yaw_window_rad + config.fine_yaw_window_rad
    search_boundary_hit = (
        translation_limit > 0.0
        and max(abs(best[0]), abs(best[1]))
        >= translation_limit - 0.5 * config.fine_translation_step_m
    ) or (
        yaw_limit > 0.0 and abs(best[2]) >= yaw_limit - 0.5 * config.fine_yaw_step_rad
    )
    return ScanMatchResult(
        correction_x_m=best[0],
        correction_y_m=best[1],
        correction_yaw_rad=best[2],
        baseline_mean_residual_m=float(np.mean(baseline)),
        baseline_median_residual_m=float(np.median(baseline)),
        baseline_inlier_ratio=float(
            np.mean(baseline <= config.inlier_threshold_m)
        ),
        best_mean_residual_m=float(np.mean(residuals)),
        best_median_residual_m=float(np.median(residuals)),
        best_p90_residual_m=float(np.percentile(residuals, 90)),
        inlier_ratio=inlier_ratio,
        point_count=len(points),
        search_boundary_hit=search_boundary_hit,
        confident=inlier_ratio >= 0.50 and not search_boundary_hit,
    )


def correction_about_sensor_as_transform(
    *,
    sensor_xy_m: tuple[float, float],
    dx_m: float,
    dy_m: float,
    dyaw_rad: float,
) -> PlanarTransform:
    """Convert a correction about the lidar pivot into a map-frame SE(2) delta."""
    sensor_x, sensor_y = sensor_xy_m
    cos_yaw = math.cos(dyaw_rad)
    sin_yaw = math.sin(dyaw_rad)
    rotated_sensor_x = cos_yaw * sensor_x - sin_yaw * sensor_y
    rotated_sensor_y = sin_yaw * sensor_x + cos_yaw * sensor_y
    return PlanarTransform(
        x=sensor_x + dx_m - rotated_sensor_x,
        y=sensor_y + dy_m - rotated_sensor_y,
        yaw=dyaw_rad,
    )


def _search(
    field: OccupancyDistanceField,
    points: np.ndarray,
    sensor_xy_m: tuple[float, float],
    *,
    center: tuple[float, float, float],
    translation_window_m: float,
    translation_step_m: float,
    yaw_window_rad: float,
    yaw_step_rad: float,
    max_residual_m: float,
) -> tuple[float, float, float]:
    best = center
    best_score = math.inf
    for yaw in _values(center[2], yaw_window_rad, yaw_step_rad):
        rotated = _correct_points(points, sensor_xy_m, 0.0, 0.0, yaw)
        for dx in _values(center[0], translation_window_m, translation_step_m):
            for dy in _values(center[1], translation_window_m, translation_step_m):
                translated = rotated + np.array((dx, dy))
                score = float(np.mean(field.residuals(translated, max_residual_m)))
                if score < best_score:
                    best_score = score
                    best = (float(dx), float(dy), float(yaw))
    return best


def _values(center: float, window: float, step: float) -> np.ndarray:
    if window < 0.0 or step <= 0.0:
        raise ValueError("search window must be non-negative and step must be positive")
    if window == 0.0:
        return np.array((center,))
    count = int(math.floor((2.0 * window) / step + 1e-9))
    return center - window + np.arange(count + 1) * step


def _correct_points(
    points: np.ndarray,
    sensor_xy_m: tuple[float, float],
    dx_m: float,
    dy_m: float,
    dyaw_rad: float,
) -> np.ndarray:
    sensor = np.asarray(sensor_xy_m, dtype=float)
    relative = points - sensor
    cos_yaw = math.cos(dyaw_rad)
    sin_yaw = math.sin(dyaw_rad)
    rotation = np.array(((cos_yaw, -sin_yaw), (sin_yaw, cos_yaw)))
    return relative @ rotation.T + sensor + np.array((dx_m, dy_m))
