#!/usr/bin/env python3
"""Check Day5 FAST-LIO output freshness directly from a ROS 2 bag."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


HEADER_TOPICS = {
    "/cloud_registered_body": "sensor_msgs/msg/PointCloud2",
    "/Odometry": "nav_msgs/msg/Odometry",
}
PROXIMITY_TOPIC = "/avoidance/proximity_status"


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[max(index, 0)]


def _read_bag(bag_path: Path, warmup_s: float):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected_topics = set(HEADER_TOPICS) | {PROXIMITY_TOPIC}
    missing = sorted(selected_topics - set(topic_types))
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")

    message_types = {
        topic: get_message(topic_types[topic]) for topic in selected_topics
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected_topics)))
    header_samples: dict[str, list[tuple[float, float]]] = {
        topic: [] for topic in HEADER_TOPICS
    }
    proximity_samples: list[tuple[float, dict[str, object]]] = []
    first_record_s: float | None = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        record_s = timestamp_ns / 1e9
        if first_record_s is None:
            first_record_s = record_s
        elapsed_s = record_s - first_record_s
        if elapsed_s < warmup_s:
            continue

        message = deserialize_message(data, message_types[topic])
        if topic in HEADER_TOPICS:
            stamp_s = _stamp_seconds(message.header.stamp)
            header_samples[topic].append((elapsed_s, record_s - stamp_s))
        else:
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                payload = {"reason": "invalid_json"}
            proximity_samples.append((elapsed_s, payload))

    return header_samples, proximity_samples


def _summarize_topic(
    topic: str,
    samples: list[tuple[float, float]],
    threshold_s: float,
    max_violation_fraction: float,
) -> tuple[bool, str]:
    if not samples:
        return False, f"{topic}: FAIL no samples after warmup"
    ages = [age for _, age in samples]
    violations = [(elapsed, age) for elapsed, age in samples if age > threshold_s]
    p50 = statistics.median(ages)
    p95 = _percentile(ages, 0.95)
    first_violation = f"{violations[0][0]:.3f}s" if violations else "none"
    violation_fraction = len(violations) / len(ages)
    passed = violation_fraction <= max_violation_fraction
    line = (
        f"{topic}: {'PASS' if passed else 'FAIL'} count={len(ages)} "
        f"age_min={min(ages):.3f}s age_p50={p50:.3f}s "
        f"age_p95={p95:.3f}s age_max={max(ages):.3f}s "
        f"over_{threshold_s:.3f}s={len(violations)} "
        f"({violation_fraction:.2%}, allowed={max_violation_fraction:.2%}) "
        f"first_violation={first_violation}"
    )
    return passed, line


def analyze(
    bag_path: Path,
    *,
    warmup_s: float,
    cloud_max_age_s: float,
    odometry_max_age_s: float,
    max_violation_fraction: float,
) -> bool:
    header_samples, proximity_samples = _read_bag(bag_path, warmup_s)
    checks = []
    for topic, threshold_s in (
        ("/cloud_registered_body", cloud_max_age_s),
        ("/Odometry", odometry_max_age_s),
    ):
        passed, line = _summarize_topic(
            topic,
            header_samples[topic],
            threshold_s,
            max_violation_fraction,
        )
        print(line)
        checks.append(passed)

    reasons = Counter(
        str(payload.get("reason", "missing_reason"))
        for _, payload in proximity_samples
    )
    stale_samples = [
        (elapsed, payload)
        for elapsed, payload in proximity_samples
        if payload.get("reason") == "stale_cloud"
    ]
    stale_fraction = (
        len(stale_samples) / len(proximity_samples) if proximity_samples else 1.0
    )
    proximity_passed = stale_fraction <= max_violation_fraction
    first_stale = f"{stale_samples[0][0]:.3f}s" if stale_samples else "none"
    print(
        f"{PROXIMITY_TOPIC}: {'PASS' if proximity_passed else 'FAIL'} "
        f"count={len(proximity_samples)} reasons={dict(sorted(reasons.items()))} "
        f"stale_fraction={stale_fraction:.2%} "
        f"allowed={max_violation_fraction:.2%} "
        f"first_stale={first_stale}"
    )
    checks.append(proximity_passed)

    passed = all(checks)
    print(f"VERDICT: {'PASS' if passed else 'FAIL'}")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--warmup-s", type=float, default=5.0)
    parser.add_argument("--cloud-max-age-s", type=float, default=0.50)
    parser.add_argument("--odometry-max-age-s", type=float, default=0.20)
    parser.add_argument("--max-violation-fraction", type=float, default=0.01)
    args = parser.parse_args()
    if args.warmup_s < 0.0:
        parser.error("--warmup-s must be non-negative")
    if args.cloud_max_age_s <= 0.0 or args.odometry_max_age_s <= 0.0:
        parser.error("age thresholds must be positive")
    if not 0.0 <= args.max_violation_fraction <= 1.0:
        parser.error("--max-violation-fraction must be in [0, 1]")

    try:
        passed = analyze(
            args.bag,
            warmup_s=args.warmup_s,
            cloud_max_age_s=args.cloud_max_age_s,
            odometry_max_age_s=args.odometry_max_age_s,
            max_violation_fraction=args.max_violation_fraction,
        )
    except RuntimeError as exc:
        print(f"VERDICT: ERROR {exc}")
        return 2
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
