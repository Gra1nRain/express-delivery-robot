#!/usr/bin/env python3
"""Diagnose Day5 local replanning deviation from a recorded ROS 2 bag.

The bag is read directly through rosbag2_py. It is never replayed, so recorded
velocity commands cannot reach a live chassis.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class Transform2D:
    """A planar target-frame-from-source-frame transform."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, x: float, y: float) -> tuple[float, float]:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            self.x + cosine * x - sine * y,
            self.y + sine * x + cosine * y,
        )

    def compose(self, other: "Transform2D") -> "Transform2D":
        x, y = self.apply(other.x, other.y)
        return Transform2D(x=x, y=y, yaw=_wrap_angle(self.yaw + other.yaw))

    def inverse(self) -> "Transform2D":
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return Transform2D(
            x=-cosine * self.x - sine * self.y,
            y=sine * self.x - cosine * self.y,
            yaw=_wrap_angle(-self.yaw),
        )


@dataclass(frozen=True)
class GridSnapshot:
    timestamp_ns: int
    frame_id: str
    resolution_m: float
    width: int
    height: int
    origin: Transform2D
    data: tuple[int, ...]
    occupied_threshold: int = 100

    def cell_for_point(self, x: float, y: float) -> tuple[int, int]:
        grid_x, grid_y = self.origin.inverse().apply(x, y)
        return (
            math.floor(grid_x / self.resolution_m),
            math.floor(grid_y / self.resolution_m),
        )

    def contains(self, cell: tuple[int, int]) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def point_is_blocked(
        self,
        x: float,
        y: float,
        *,
        inflation_radius_m: float,
    ) -> bool:
        col, row = self.cell_for_point(x, y)
        if not self.contains((col, row)):
            return False
        radius_cells = math.ceil(max(0.0, inflation_radius_m) / self.resolution_m)
        radius_squared = radius_cells * radius_cells
        for delta_y in range(-radius_cells, radius_cells + 1):
            for delta_x in range(-radius_cells, radius_cells + 1):
                if delta_x * delta_x + delta_y * delta_y > radius_squared:
                    continue
                candidate = (col + delta_x, row + delta_y)
                if not self.contains(candidate):
                    continue
                candidate_col, candidate_row = candidate
                value = self.data[candidate_row * self.width + candidate_col]
                if value >= self.occupied_threshold:
                    return True
        return False


@dataclass(frozen=True)
class PathSnapshot:
    timestamp_ns: int
    frame_id: str
    points: tuple[tuple[float, float], ...]
    yaws: tuple[float, ...] = ()


@dataclass(frozen=True)
class ReplanInput:
    timestamp_ns: int
    elapsed_s: float
    payload: dict[str, object]
    global_path: PathSnapshot | None
    local_path: PathSnapshot | None
    dynamic_grid: GridSnapshot | None
    static_grid: GridSnapshot | None
    transforms: dict[tuple[str, str], Transform2D]


@dataclass(frozen=True)
class ReplanEventMetrics:
    timestamp_ns: int
    elapsed_s: float
    status: str
    reference_start_index: int
    rejoin_index: int
    dynamic_obstacle_count: int
    local_path_point_count: int
    max_local_to_global_deviation_m: float | None
    p95_local_to_global_deviation_m: float | None
    reference_point_count: int
    dynamic_blocked_reference_points: int
    static_blocked_reference_points: int
    combined_blocked_reference_points: int
    tf_available: bool = True
    current_pose_x_m: float | None = None
    current_pose_y_m: float | None = None
    current_pose_yaw_rad: float | None = None
    current_to_reference_start_distance_m: float | None = None
    current_to_reference_start_heading_deg: float | None = None

    @property
    def combined_blocked_reference_ratio(self) -> float | None:
        if self.reference_point_count <= 0:
            return None
        return self.combined_blocked_reference_points / self.reference_point_count


@dataclass(frozen=True)
class ReplanningReport:
    passed: bool
    failed_checks: tuple[str, ...]
    analyzed_replanned_events: int
    tf_unavailable_events: int
    high_deviation_events: int
    high_deviation_clear_reference_events: int
    high_deviation_blocked_reference_events: int
    max_local_deviation_limit_m: float
    peak_event: ReplanEventMetrics | None
    tf_edges: tuple[tuple[str, str], ...]
    events: tuple[ReplanEventMetrics, ...]

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        if self.peak_event is not None:
            result["peak_event"]["combined_blocked_reference_ratio"] = (
                self.peak_event.combined_blocked_reference_ratio
            )
        for event, event_dict in zip(self.events, result["events"]):
            event_dict["combined_blocked_reference_ratio"] = (
                event.combined_blocked_reference_ratio
            )
        return result


def lookup_transform(
    transforms: Mapping[tuple[str, str], Transform2D],
    target_frame: str,
    source_frame: str,
) -> Transform2D:
    """Return target-from-source by composing the current TF graph."""

    target = _normalize_frame(target_frame)
    source = _normalize_frame(source_frame)
    if target == source:
        return Transform2D()

    neighbors: dict[str, list[tuple[str, Transform2D]]] = {}
    for (parent, child), parent_from_child in transforms.items():
        parent = _normalize_frame(parent)
        child = _normalize_frame(child)
        neighbors.setdefault(child, []).append((parent, parent_from_child))
        neighbors.setdefault(parent, []).append((child, parent_from_child.inverse()))

    queue = deque([(source, Transform2D())])
    visited = {source}
    while queue:
        frame, frame_from_source = queue.popleft()
        for neighbor, neighbor_from_frame in neighbors.get(frame, ()):
            if neighbor in visited:
                continue
            neighbor_from_source = neighbor_from_frame.compose(frame_from_source)
            if neighbor == target:
                return neighbor_from_source
            visited.add(neighbor)
            queue.append((neighbor, neighbor_from_source))
    raise KeyError(f"no transform from {source!r} to {target!r}")


def analyze_replan_input(
    event: ReplanInput,
    *,
    inflation_radius_m: float,
) -> ReplanEventMetrics | None:
    status = str(event.payload.get("status") or "")
    if status != "REPLANNED":
        return None
    if event.global_path is None or event.local_path is None:
        return None

    start_index = int(event.payload.get("reference_start_index") or 0)
    rejoin_index = int(event.payload.get("rejoin_index") or start_index)
    global_points = event.global_path.points
    start_index = max(0, min(start_index, len(global_points)))
    rejoin_index = max(start_index, min(rejoin_index, len(global_points) - 1))
    reference_points = global_points[start_index : rejoin_index + 1]

    deviations = [
        min(math.hypot(x - gx, y - gy) for gx, gy in global_points)
        for x, y in event.local_path.points
    ]
    dynamic_blocked = 0
    static_blocked = 0
    combined_blocked = 0
    tf_available = True
    current_pose_x: float | None = None
    current_pose_y: float | None = None
    current_pose_yaw: float | None = None
    start_distance: float | None = None
    start_heading_deg: float | None = None

    if event.local_path.points:
        current_pose_x, current_pose_y = event.local_path.points[0]
        if event.local_path.yaws:
            current_pose_yaw = event.local_path.yaws[0]
        if reference_points:
            start_distance = math.hypot(
                current_pose_x - reference_points[0][0],
                current_pose_y - reference_points[0][1],
            )
            if (
                current_pose_yaw is not None
                and start_index < len(event.global_path.yaws)
            ):
                start_heading_deg = math.degrees(
                    abs(
                        _wrap_angle(
                            current_pose_yaw - event.global_path.yaws[start_index]
                        )
                    )
                )

    for x, y in reference_points:
        dynamic_hit = False
        static_hit = False
        if event.dynamic_grid is not None:
            try:
                dynamic_from_global = lookup_transform(
                    event.transforms,
                    event.dynamic_grid.frame_id,
                    event.global_path.frame_id,
                )
                dynamic_x, dynamic_y = dynamic_from_global.apply(x, y)
                dynamic_hit = event.dynamic_grid.point_is_blocked(
                    dynamic_x,
                    dynamic_y,
                    inflation_radius_m=inflation_radius_m,
                )
            except KeyError:
                tf_available = False
        if event.static_grid is not None:
            try:
                static_from_global = lookup_transform(
                    event.transforms,
                    event.static_grid.frame_id,
                    event.global_path.frame_id,
                )
                static_x, static_y = static_from_global.apply(x, y)
                static_hit = event.static_grid.point_is_blocked(
                    static_x,
                    static_y,
                    inflation_radius_m=inflation_radius_m,
                )
            except KeyError:
                tf_available = False
        dynamic_blocked += int(dynamic_hit)
        static_blocked += int(static_hit)
        combined_blocked += int(dynamic_hit or static_hit)

    return ReplanEventMetrics(
        timestamp_ns=event.timestamp_ns,
        elapsed_s=event.elapsed_s,
        status=status,
        reference_start_index=start_index,
        rejoin_index=rejoin_index,
        dynamic_obstacle_count=int(event.payload.get("dynamic_obstacle_count") or 0),
        local_path_point_count=len(event.local_path.points),
        max_local_to_global_deviation_m=max(deviations) if deviations else None,
        p95_local_to_global_deviation_m=_percentile(deviations, 0.95),
        reference_point_count=len(reference_points),
        dynamic_blocked_reference_points=dynamic_blocked,
        static_blocked_reference_points=static_blocked,
        combined_blocked_reference_points=combined_blocked,
        tf_available=tf_available,
        current_pose_x_m=current_pose_x,
        current_pose_y_m=current_pose_y,
        current_pose_yaw_rad=current_pose_yaw,
        current_to_reference_start_distance_m=start_distance,
        current_to_reference_start_heading_deg=start_heading_deg,
    )


def build_report(
    events: Iterable[ReplanEventMetrics],
    *,
    max_local_deviation_m: float,
    tf_edges: Iterable[tuple[str, str]],
) -> ReplanningReport:
    event_tuple = tuple(events)
    measurable = tuple(
        event
        for event in event_tuple
        if event.max_local_to_global_deviation_m is not None
    )
    peak = max(
        measurable,
        key=lambda event: float(event.max_local_to_global_deviation_m),
        default=None,
    )
    high_deviation = tuple(
        event
        for event in measurable
        if float(event.max_local_to_global_deviation_m) > max_local_deviation_m
    )
    failed_checks: list[str] = []
    if not measurable:
        failed_checks.append("no measurable REPLANNED events")
    elif peak is not None and float(peak.max_local_to_global_deviation_m) > max_local_deviation_m:
        failed_checks.append(
            "max_local_to_global_deviation_m "
            f"{peak.max_local_to_global_deviation_m:.3f} > "
            f"{max_local_deviation_m:.3f}"
        )
    return ReplanningReport(
        passed=not failed_checks,
        failed_checks=tuple(failed_checks),
        analyzed_replanned_events=len(event_tuple),
        tf_unavailable_events=sum(not event.tf_available for event in event_tuple),
        high_deviation_events=len(high_deviation),
        high_deviation_clear_reference_events=sum(
            event.combined_blocked_reference_points == 0
            for event in high_deviation
        ),
        high_deviation_blocked_reference_events=sum(
            event.combined_blocked_reference_points > 0
            for event in high_deviation
        ),
        max_local_deviation_limit_m=max_local_deviation_m,
        peak_event=peak,
        tf_edges=tuple(sorted(set(tf_edges))),
        events=event_tuple,
    )


def read_bag(
    bag_path: Path,
    *,
    inflation_radius_m: float,
) -> ReplanningReport:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages are required; run this script in the car ROS environment"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    wanted_topics = {
        "/avoidance/local_costmap",
        "/map",
        "/planning/local_replan_status",
        "/planning/local_trajectory",
        "/planning/optimized_trajectory",
        "/tf",
        "/tf_static",
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in wanted_topics
    }

    bag_start_ns: int | None = None
    global_path: PathSnapshot | None = None
    local_path: PathSnapshot | None = None
    dynamic_grid: GridSnapshot | None = None
    static_grid: GridSnapshot | None = None
    transforms: dict[tuple[str, str], Transform2D] = {}
    all_tf_edges: set[tuple[str, str]] = set()
    metrics: list[ReplanEventMetrics] = []

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        if topic == "/planning/optimized_trajectory":
            global_path = _path_snapshot(message, timestamp_ns)
        elif topic == "/planning/local_trajectory":
            local_path = _path_snapshot(message, timestamp_ns)
        elif topic == "/avoidance/local_costmap":
            dynamic_grid = _grid_snapshot(message, timestamp_ns, occupied_threshold=100)
        elif topic == "/map":
            static_grid = _grid_snapshot(message, timestamp_ns, occupied_threshold=65)
        elif topic in ("/tf", "/tf_static"):
            for item in message.transforms:
                parent = _normalize_frame(item.header.frame_id)
                child = _normalize_frame(item.child_frame_id)
                if not parent or not child:
                    continue
                transform = Transform2D(
                    x=float(item.transform.translation.x),
                    y=float(item.transform.translation.y),
                    yaw=_yaw_from_quaternion(item.transform.rotation),
                )
                transforms[(parent, child)] = transform
                all_tf_edges.add((parent, child))
        elif topic == "/planning/local_replan_status":
            try:
                payload = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                continue
            event = ReplanInput(
                timestamp_ns=timestamp_ns,
                elapsed_s=(timestamp_ns - bag_start_ns) / 1e9,
                payload=payload,
                global_path=global_path,
                local_path=local_path,
                dynamic_grid=dynamic_grid,
                static_grid=static_grid,
                transforms=dict(transforms),
            )
            event_metrics = analyze_replan_input(
                event,
                inflation_radius_m=inflation_radius_m,
            )
            if event_metrics is not None:
                metrics.append(event_metrics)

    return build_report(
        metrics,
        max_local_deviation_m=0.5,
        tf_edges=all_tf_edges,
    )


def _path_snapshot(message, timestamp_ns: int) -> PathSnapshot:
    return PathSnapshot(
        timestamp_ns=timestamp_ns,
        frame_id=_normalize_frame(message.header.frame_id),
        points=tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ),
        yaws=tuple(_yaw_from_quaternion(pose.pose.orientation) for pose in message.poses),
    )


def _grid_snapshot(
    message,
    timestamp_ns: int,
    *,
    occupied_threshold: int,
) -> GridSnapshot:
    return GridSnapshot(
        timestamp_ns=timestamp_ns,
        frame_id=_normalize_frame(message.header.frame_id),
        resolution_m=float(message.info.resolution),
        width=int(message.info.width),
        height=int(message.info.height),
        origin=Transform2D(
            x=float(message.info.origin.position.x),
            y=float(message.info.origin.position.y),
            yaw=_yaw_from_quaternion(message.info.origin.orientation),
        ),
        data=tuple(int(value) for value in message.data),
        occupied_threshold=occupied_threshold,
    )


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).strip().lstrip("/")


def _percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--inflation-radius-m", type=float, default=0.45)
    parser.add_argument("--max-local-deviation-m", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    report = read_bag(
        args.bag,
        inflation_radius_m=args.inflation_radius_m,
    )
    if args.max_local_deviation_m != report.max_local_deviation_limit_m:
        report = build_report(
            report.events,
            max_local_deviation_m=args.max_local_deviation_m,
            tf_edges=report.tf_edges,
        )
    payload = report.to_dict()
    rendered = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")

    peak = report.peak_event
    print(
        f"status={'PASS' if report.passed else 'FAIL'} "
        f"events={report.analyzed_replanned_events} "
        f"high_deviation={report.high_deviation_events} "
        f"high_clear={report.high_deviation_clear_reference_events} "
        f"high_blocked={report.high_deviation_blocked_reference_events} "
        "peak_m="
        f"{None if peak is None else peak.max_local_to_global_deviation_m}"
    )
    if peak is not None:
        print(
            f"peak_t_s={peak.elapsed_s:.3f} "
            f"reference={peak.reference_start_index}:{peak.rejoin_index} "
            f"dynamic_obstacles={peak.dynamic_obstacle_count} "
            f"blocked_reference="
            f"{peak.combined_blocked_reference_points}/{peak.reference_point_count}"
        )
    for check in report.failed_checks:
        print(f"- {check}")
    return 0 if report.passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
