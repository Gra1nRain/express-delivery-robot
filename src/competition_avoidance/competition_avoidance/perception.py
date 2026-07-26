"""Pure point-cloud filtering and obstacle clustering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class PerceptionConfig:
    x_min_m: float = -0.50
    x_max_m: float = 5.50
    y_min_m: float = -2.50
    y_max_m: float = 2.50
    z_min_m: float = -0.25
    z_max_m: float = 2.20
    ground_max_z_m: float = -0.20
    self_x_min_m: float = -0.50
    self_x_max_m: float = 0.50
    self_y_min_m: float = -0.35
    self_y_max_m: float = 0.35
    voxel_size_m: float = 0.08
    cluster_tolerance_m: float = 0.30
    min_cluster_points: int = 6
    max_cluster_points: int = 20_000

    def __post_init__(self) -> None:
        if self.x_max_m <= self.x_min_m or self.y_max_m <= self.y_min_m:
            raise ValueError("ROI maximum bounds must exceed minimum bounds")
        if self.z_max_m <= self.z_min_m:
            raise ValueError("z_max_m must exceed z_min_m")
        if self.voxel_size_m <= 0.0 or self.cluster_tolerance_m <= 0.0:
            raise ValueError("voxel and cluster sizes must be positive")
        if self.min_cluster_points < 1:
            raise ValueError("min_cluster_points must be positive")
        if self.max_cluster_points < self.min_cluster_points:
            raise ValueError("max_cluster_points must cover min_cluster_points")


@dataclass(frozen=True)
class ObstacleDetection:
    x: float
    y: float
    z: float
    length_m: float
    width_m: float
    height_m: float
    point_count: int
    classification: str
    confidence: float

    @property
    def radius_m(self) -> float:
        return 0.5 * math.hypot(self.length_m, self.width_m)


def exclude_vehicle_footprint_points(
    points: Iterable[tuple[float, float, float]],
    *,
    x_min_m: float,
    x_max_m: float,
    y_half_width_m: float,
) -> tuple[tuple[float, float, float], ...]:
    """Remove returns inside the vehicle footprint without masking the stop zone.

    The longitudinal upper bound is half-open so a point exactly at the
    proximity gate's x_min remains available to the safety check.
    """

    if x_max_m <= x_min_m:
        raise ValueError("footprint x_max_m must exceed x_min_m")
    if y_half_width_m <= 0.0:
        raise ValueError("footprint y_half_width_m must be positive")

    filtered: list[tuple[float, float, float]] = []
    for raw_x, raw_y, raw_z in points:
        point = (float(raw_x), float(raw_y), float(raw_z))
        x, y, _ = point
        inside_footprint = (
            x_min_m <= x < x_max_m
            and -y_half_width_m <= y <= y_half_width_m
        )
        if not inside_footprint:
            filtered.append(point)
    return tuple(filtered)


def cluster_points(
    points: Iterable[tuple[float, float, float]],
    config: PerceptionConfig = PerceptionConfig(),
) -> tuple[ObstacleDetection, ...]:
    """Return conservative obstacle detections in the input frame."""

    filtered = _filter_and_voxelize(points, config)
    clusters = _euclidean_clusters(filtered, config.cluster_tolerance_m)
    detections = [
        _detection(cluster)
        for cluster in clusters
        if config.min_cluster_points <= len(cluster) <= config.max_cluster_points
    ]
    return tuple(sorted(detections, key=lambda item: (item.x, item.y)))


def _filter_and_voxelize(
    points: Iterable[tuple[float, float, float]],
    config: PerceptionConfig,
) -> tuple[tuple[float, float, float], ...]:
    voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    for raw_x, raw_y, raw_z in points:
        x = float(raw_x)
        y = float(raw_y)
        z = float(raw_z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if not (
            config.x_min_m <= x <= config.x_max_m
            and config.y_min_m <= y <= config.y_max_m
            and config.z_min_m <= z <= config.z_max_m
        ):
            continue
        if z <= config.ground_max_z_m:
            continue
        if (
            config.self_x_min_m <= x <= config.self_x_max_m
            and config.self_y_min_m <= y <= config.self_y_max_m
        ):
            continue
        key = (
            math.floor(x / config.voxel_size_m),
            math.floor(y / config.voxel_size_m),
            math.floor(z / config.voxel_size_m),
        )
        voxels.setdefault(key, (x, y, z))
    return tuple(voxels.values())


def _euclidean_clusters(
    points: tuple[tuple[float, float, float], ...],
    tolerance_m: float,
) -> tuple[tuple[tuple[float, float, float], ...], ...]:
    if not points:
        return ()
    buckets: dict[tuple[int, int, int], list[int]] = {}
    for index, (x, y, z) in enumerate(points):
        key = (
            math.floor(x / tolerance_m),
            math.floor(y / tolerance_m),
            math.floor(z / tolerance_m),
        )
        buckets.setdefault(key, []).append(index)

    visited: set[int] = set()
    clusters: list[tuple[tuple[float, float, float], ...]] = []
    tolerance_sq = tolerance_m * tolerance_m
    for seed in range(len(points)):
        if seed in visited:
            continue
        visited.add(seed)
        pending = deque([seed])
        members: list[int] = []
        while pending:
            current = pending.popleft()
            members.append(current)
            x, y, z = points[current]
            cell = (
                math.floor(x / tolerance_m),
                math.floor(y / tolerance_m),
                math.floor(z / tolerance_m),
            )
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        for candidate in buckets.get(
                            (cell[0] + dx, cell[1] + dy, cell[2] + dz),
                            (),
                        ):
                            if candidate in visited:
                                continue
                            other_x, other_y, other_z = points[candidate]
                            distance_sq = (
                                (other_x - x) ** 2
                                + (other_y - y) ** 2
                                + (other_z - z) ** 2
                            )
                            if distance_sq <= tolerance_sq:
                                visited.add(candidate)
                                pending.append(candidate)
        clusters.append(tuple(points[index] for index in members))
    return tuple(clusters)


def _detection(
    cluster: tuple[tuple[float, float, float], ...],
) -> ObstacleDetection:
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    zs = [point[2] for point in cluster]
    length = max(xs) - min(xs)
    width = max(ys) - min(ys)
    height = max(zs) - min(zs)
    classification, confidence = _classify(length, width, height)
    return ObstacleDetection(
        x=sum(xs) / len(xs),
        y=sum(ys) / len(ys),
        z=sum(zs) / len(zs),
        length_m=length,
        width_m=width,
        height_m=height,
        point_count=len(cluster),
        classification=classification,
        confidence=confidence,
    )


def _classify(length_m: float, width_m: float, height_m: float) -> tuple[str, float]:
    if (
        0.25 <= height_m <= 0.90
        and length_m <= 0.80
        and width_m <= 0.80
    ):
        return "CONE_CANDIDATE", 0.65
    if (
        0.80 <= height_m <= 2.20
        and length_m <= 1.20
        and width_m <= 1.20
    ):
        return "PERSON_CANDIDATE", 0.55
    return "UNKNOWN", 0.25
