#!/usr/bin/env python3
"""Extract costmap candidates around the largest local-replanning status gap.

The bag is read directly and never replayed, so recorded velocity commands
cannot reach a live chassis.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping, Sequence

from analyze_day5_local_replanning_bag import (
    Transform2D,
    _grid_snapshot,
    _normalize_frame,
    _yaw_from_quaternion,
    lookup_transform,
)


@dataclass(frozen=True)
class StatusRecord:
    timestamp_ns: int
    payload: Mapping[str, object]


def largest_status_gap(
    records: Sequence[StatusRecord],
) -> tuple[StatusRecord, StatusRecord]:
    if len(records) < 2:
        raise ValueError("at least two status records are required")
    return max(
        zip(records, records[1:]),
        key=lambda pair: pair[1].timestamp_ns - pair[0].timestamp_ns,
    )


def _reader(path: Path):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def _message_types(reader, topics: set[str]):
    from rosidl_runtime_py.utilities import get_message

    return {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
        if item.name in topics
    }


def _read_statuses(path: Path) -> tuple[int, tuple[StatusRecord, ...]]:
    from rclpy.serialization import deserialize_message

    reader = _reader(path)
    message_types = _message_types(reader, {"/planning/local_replan_status"})
    bag_start_ns: int | None = None
    records: list[StatusRecord] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            continue
        records.append(StatusRecord(timestamp_ns, payload))
    if bag_start_ns is None:
        raise ValueError("bag contains no messages")
    return bag_start_ns, tuple(records)


def _occupied_points_map(grid, map_from_grid: Transform2D) -> list[list[float]]:
    points: list[list[float]] = []
    for index, value in enumerate(grid.data):
        if value < 100:
            continue
        row, col = divmod(index, grid.width)
        grid_x = (col + 0.5) * grid.resolution_m
        grid_y = (row + 0.5) * grid.resolution_m
        frame_x, frame_y = grid.origin.apply(grid_x, grid_y)
        map_x, map_y = map_from_grid.apply(frame_x, frame_y)
        points.append([map_x, map_y])
    return points


def extract_gap_fixture(
    bag_path: Path,
    *,
    capture_window_s: float = 1.5,
    capture_before_s: float = 1.5,
    base_frame: str = "chassis_center",
) -> dict[str, object]:
    from rclpy.serialization import deserialize_message

    if capture_window_s <= 0.0:
        raise ValueError("capture_window_s must be positive")
    if capture_before_s < 0.0:
        raise ValueError("capture_before_s must be non-negative")
    bag_start_ns, statuses = _read_statuses(bag_path)
    before, after = largest_status_gap(statuses)
    capture_start_ns = before.timestamp_ns - round(capture_before_s * 1e9)
    capture_end_ns = before.timestamp_ns + round(capture_window_s * 1e9)

    wanted_topics = {"/avoidance/local_costmap", "/tf", "/tf_static"}
    reader = _reader(bag_path)
    message_types = _message_types(reader, wanted_topics)
    transforms: dict[tuple[str, str], Transform2D] = {}
    candidates: list[dict[str, object]] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if timestamp_ns > capture_end_ns:
            break
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        if topic in ("/tf", "/tf_static"):
            for item in message.transforms:
                parent = _normalize_frame(item.header.frame_id)
                child = _normalize_frame(item.child_frame_id)
                if not parent or not child:
                    continue
                transforms[(parent, child)] = Transform2D(
                    x=float(item.transform.translation.x),
                    y=float(item.transform.translation.y),
                    yaw=_yaw_from_quaternion(item.transform.rotation),
                )
            continue
        if timestamp_ns < capture_start_ns:
            continue

        grid = _grid_snapshot(message, timestamp_ns, occupied_threshold=100)
        try:
            map_from_grid = lookup_transform(
                transforms,
                "map",
                grid.frame_id,
            )
        except KeyError:
            continue
        try:
            map_from_base = lookup_transform(transforms, "map", base_frame)
            current_pose = {
                "x": map_from_base.x,
                "y": map_from_base.y,
                "yaw": map_from_base.yaw,
            }
        except KeyError:
            current_pose = None
        candidates.append(
            {
                "elapsed_s": (timestamp_ns - bag_start_ns) / 1e9,
                "frame_id": grid.frame_id,
                "current_pose": current_pose,
                "occupied_points_map": _occupied_points_map(grid, map_from_grid),
            }
        )

    return {
        "bag": str(bag_path),
        "gap": {
            "start_elapsed_s": (before.timestamp_ns - bag_start_ns) / 1e9,
            "end_elapsed_s": (after.timestamp_ns - bag_start_ns) / 1e9,
            "duration_s": (after.timestamp_ns - before.timestamp_ns) / 1e9,
            "before": dict(before.payload),
            "after": dict(after.payload),
        },
        "capture_before_s": capture_before_s,
        "capture_window_s": capture_window_s,
        "base_frame": base_frame,
        "candidates": candidates,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--capture-before-s", type=float, default=1.5)
    parser.add_argument("--capture-window-s", type=float, default=1.5)
    parser.add_argument("--base-frame", default="chassis_center")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)

    fixture = extract_gap_fixture(
        args.bag,
        capture_before_s=args.capture_before_s,
        capture_window_s=args.capture_window_s,
        base_frame=args.base_frame,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(fixture, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"gap_s={fixture['gap']['duration_s']:.3f} "
        f"candidates={len(fixture['candidates'])} "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
