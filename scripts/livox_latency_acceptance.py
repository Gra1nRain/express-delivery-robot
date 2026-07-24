#!/usr/bin/env python3
"""Measure Livox and FAST-LIO timestamp freshness for a fixed duration."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def summarize(samples: list[float], elapsed_s: float) -> dict[str, float | int]:
    if not samples:
        return {"count": 0, "callback_rate_hz": 0.0}
    ordered = sorted(samples)
    return {
        "count": len(ordered),
        "callback_rate_hz": round(len(ordered) / elapsed_s, 3),
        "mean_s": round(sum(ordered) / len(ordered), 6),
        "p50_s": round(percentile(ordered, 0.50), 6),
        "p95_s": round(percentile(ordered, 0.95), 6),
        "p99_s": round(percentile(ordered, 0.99), 6),
        "max_s": round(ordered[-1], 6),
    }


class LatencyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("livox_latency_acceptance")
        self.samples: dict[str, list[float]] = {"livox": [], "cloud": []}
        self.invalid_samples: dict[str, int] = {"livox": 0, "cloud": 0}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import PointCloud2

        self.create_subscription(
            CustomMsg, "/livox/lidar", self._livox_callback, qos
        )
        self.create_subscription(
            PointCloud2,
            "/cloud_registered_body",
            self._cloud_callback,
            qos,
        )

    def _append_age(self, source: str, stamp_ns: int) -> None:
        age_s = (time.time_ns() - stamp_ns) * 1e-9
        if math.isfinite(age_s) and age_s >= 0.0:
            self.samples[source].append(age_s)
        else:
            self.invalid_samples[source] += 1

    def _livox_callback(self, message) -> None:
        stamp_ns = int(message.timebase)
        if message.points:
            stamp_ns += int(message.points[-1].offset_time)
        self._append_age("livox", stamp_ns)

    def _cloud_callback(self, message) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(message.header.stamp.nanosec)
        self._append_age("cloud", stamp_ns)


def report(node: LatencyMonitor, elapsed_s: float, status: str) -> dict:
    return {
        "status": status,
        "elapsed_s": round(elapsed_s, 3),
        "livox": summarize(node.samples["livox"], elapsed_s),
        "cloud": summarize(node.samples["cloud"], elapsed_s),
        "invalid_samples": node.invalid_samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--report-every-s", type=float, default=60.0)
    parser.add_argument("--max-p95-s", type=float, default=0.30)
    parser.add_argument("--max-age-s", type=float, default=0.50)
    parser.add_argument("--minimum-samples", type=int, default=100)
    args = parser.parse_args()
    if min(
        args.duration_s,
        args.report_every_s,
        args.max_p95_s,
        args.max_age_s,
        args.minimum_samples,
    ) <= 0:
        parser.error("all limits must be positive")

    rclpy.init()
    node = LatencyMonitor()
    started_s = time.monotonic()
    next_report_s = args.report_every_s
    try:
        while True:
            elapsed_s = time.monotonic() - started_s
            if elapsed_s >= args.duration_s:
                break
            rclpy.spin_once(node, timeout_sec=min(0.1, args.duration_s - elapsed_s))
            elapsed_s = time.monotonic() - started_s
            if elapsed_s >= next_report_s and elapsed_s < args.duration_s:
                print(json.dumps(report(node, elapsed_s, "progress")), flush=True)
                next_report_s += args.report_every_s

        final = report(node, time.monotonic() - started_s, "complete")
        passed = all(
            final[source].get("count", 0) >= args.minimum_samples
            and final[source].get("p95_s", math.inf) < args.max_p95_s
            and final[source].get("max_s", math.inf) < args.max_age_s
            for source in ("livox", "cloud")
        )
        final["result"] = "pass" if passed else "fail"
        final["limits"] = {
            "minimum_samples": args.minimum_samples,
            "max_p95_s": args.max_p95_s,
            "max_age_s": args.max_age_s,
        }
        print(json.dumps(final), flush=True)
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
