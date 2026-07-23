#!/usr/bin/env python3
"""Capture body-frame point-cloud points that would trigger Day5 proximity stop.

This is a read-only diagnostic helper. It subscribes to the configured point-cloud
topic, applies the same stop-box geometry used by competition_safety, and writes
compact JSON evidence. It never publishes cmd_vel, initialpose, or stop requests.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class StopBoxConfig:
    x_min_m: float = 0.25
    stop_distance_m: float = 0.85
    front_half_angle_rad: float = 0.4363
    lateral_half_width_m: float = 0.45
    z_min_m: float = -0.25
    z_max_m: float = 0.80


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _point_is_in_stop_box(
    x: float,
    y: float,
    z: float,
    config: StopBoxConfig,
) -> bool:
    if not (
        math.isfinite(x)
        and math.isfinite(y)
        and math.isfinite(z)
        and config.z_min_m <= z <= config.z_max_m
    ):
        return False
    range_xy = math.hypot(x, y)
    in_front_sector = (
        config.x_min_m <= range_xy <= config.stop_distance_m
        and x > 0.0
        and abs(math.atan2(y, x)) <= config.front_half_angle_rad
    )
    in_body_corridor = (
        config.x_min_m <= x <= config.stop_distance_m
        and abs(y) <= config.lateral_half_width_m
    )
    return in_front_sector or in_body_corridor


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None, "mean": None}
    return {
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def _round_nested(value: Any, digits: int = 4) -> Any:
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, list):
        return [_round_nested(item, digits) for item in value]
    if isinstance(value, dict):
        return {key: _round_nested(item, digits) for key, item in value.items()}
    return value


def _summarize_stop_points(
    *,
    stamp_s: float,
    now_s: float,
    frame_id: str,
    cloud_width: int,
    cloud_height: int,
    total_points: int,
    stop_points: list[tuple[float, float, float]],
    sample_limit: int,
) -> dict[str, Any]:
    ranges = [math.hypot(x, y) for x, y, _ in stop_points]
    closest = sorted(
        (
            {
                "x": x,
                "y": y,
                "z": z,
                "range": math.hypot(x, y),
                "angle_deg": math.degrees(math.atan2(y, x)),
            }
            for x, y, z in stop_points
        ),
        key=lambda item: item["range"],
    )[:sample_limit]
    return _round_nested(
        {
            "stamp_s": stamp_s,
            "cloud_age_s": now_s - stamp_s if stamp_s > 0.0 else None,
            "frame_id": frame_id,
            "cloud_width": cloud_width,
            "cloud_height": cloud_height,
            "total_points": total_points,
            "stop_point_count": len(stop_points),
            "range_stats_m": _stats(ranges),
            "x_stats_m": _stats([x for x, _, _ in stop_points]),
            "y_stats_m": _stats([y for _, y, _ in stop_points]),
            "z_stats_m": _stats([z for _, _, z in stop_points]),
            "closest_stop_points": closest,
        }
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", default="/cloud_registered_body")
    parser.add_argument("--sample-count", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--expected-frame-id", default="body")
    parser.add_argument("--x-min-m", type=float, default=0.25)
    parser.add_argument("--stop-distance-m", type=float, default=0.85)
    parser.add_argument("--front-half-angle-rad", type=float, default=0.4363)
    parser.add_argument("--lateral-half-width-m", type=float, default=0.45)
    parser.add_argument("--z-min-m", type=float, default=-0.25)
    parser.add_argument("--z-max-m", type=float, default=0.80)
    parser.add_argument("--sample-limit", type=int, default=12)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.sample_count < 1:
        raise ValueError("--sample-count must be at least 1")
    if args.sample_limit < 1:
        raise ValueError("--sample-limit must be at least 1")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    config = StopBoxConfig(
        x_min_m=args.x_min_m,
        stop_distance_m=args.stop_distance_m,
        front_half_angle_rad=args.front_half_angle_rad,
        lateral_half_width_m=args.lateral_half_width_m,
        z_min_m=args.z_min_m,
        z_max_m=args.z_max_m,
    )
    samples: list[dict[str, Any]] = []

    class SnapshotNode(Node):
        def __init__(self) -> None:
            super().__init__("day5_proximity_snapshot")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(PointCloud2, args.cloud_topic, self._cb, qos)

        def _cb(self, message: PointCloud2) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            stamp_s = _stamp_to_seconds(message.header.stamp)
            frame_id = message.header.frame_id
            stop_points: list[tuple[float, float, float]] = []
            total_points = 0
            if args.expected_frame_id and frame_id != args.expected_frame_id:
                samples.append(
                    {
                        "stamp_s": stamp_s,
                        "cloud_age_s": now_s - stamp_s if stamp_s > 0.0 else None,
                        "frame_id": frame_id,
                        "frame_mismatch": True,
                        "stop_point_count": 0,
                    }
                )
                return
            for point in point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                total_points += 1
                if _point_is_in_stop_box(x, y, z, config):
                    stop_points.append((x, y, z))
            samples.append(
                _summarize_stop_points(
                    stamp_s=stamp_s,
                    now_s=now_s,
                    frame_id=frame_id,
                    cloud_width=message.width,
                    cloud_height=message.height,
                    total_points=total_points,
                    stop_points=stop_points,
                    sample_limit=args.sample_limit,
                )
            )

    rclpy.init()
    node = SnapshotNode()
    deadline_s = time.monotonic() + args.timeout_s
    try:
        while len(samples) < args.sample_count and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    counts = [int(sample.get("stop_point_count", 0)) for sample in samples]
    payload = {
        "cloud_topic": args.cloud_topic,
        "sample_count": len(samples),
        "stop_box_config": config.__dict__,
        "stop_point_count_min": min(counts) if counts else None,
        "stop_point_count_max": max(counts) if counts else None,
        "samples": samples,
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0 if samples else 2


if __name__ == "__main__":
    raise SystemExit(main())
