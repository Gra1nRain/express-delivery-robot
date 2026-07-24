#!/usr/bin/env python3
"""Wait for a latest-only Livox or FAST-LIO freshness window."""

from __future__ import annotations

import argparse
from collections import deque
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


def _percentile(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(percentile * len(ordered)) - 1)
    return ordered[index]


class FreshnessGate(Node):
    def __init__(self, mode: str, sample_count: int) -> None:
        super().__init__(f"day5_{mode}_freshness_gate")
        self.ages_s: deque[float] = deque(maxlen=sample_count)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if mode == "livox":
            from livox_ros_driver2.msg import CustomMsg

            self.create_subscription(
                CustomMsg,
                "/livox/lidar",
                self._livox_callback,
                qos,
            )
        else:
            from sensor_msgs.msg import PointCloud2

            self.create_subscription(
                PointCloud2,
                "/cloud_registered_body",
                self._cloud_callback,
                qos,
            )

    def _append_age(self, stamp_ns: int) -> None:
        age_s = (time.time_ns() - stamp_ns) * 1e-9
        if math.isfinite(age_s) and age_s >= 0.0:
            self.ages_s.append(age_s)

    def _livox_callback(self, message) -> None:
        stamp_ns = int(message.timebase)
        if message.points:
            stamp_ns += int(message.points[-1].offset_time)
        self._append_age(stamp_ns)

    def _cloud_callback(self, message) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(message.header.stamp.nanosec)
        self._append_age(stamp_ns)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("livox", "cloud"), required=True)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--max-p95-age-s", type=float, required=True)
    parser.add_argument("--sample-count", type=int, default=20)
    args = parser.parse_args()
    if args.timeout_s <= 0.0 or args.max_p95_age_s <= 0.0:
        parser.error("timeouts and age limits must be positive")
    if args.sample_count < 2:
        parser.error("--sample-count must be at least 2")

    rclpy.init()
    node = FreshnessGate(args.mode, args.sample_count)
    started_s = time.monotonic()
    last_report_s = started_s
    try:
        while time.monotonic() - started_s < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            now_s = time.monotonic()
            if len(node.ages_s) == args.sample_count:
                p95_age_s = _percentile(list(node.ages_s), 0.95)
                if p95_age_s <= args.max_p95_age_s:
                    print(
                        json.dumps(
                            {
                                "status": "ready",
                                "mode": args.mode,
                                "samples": args.sample_count,
                                "p95_age_s": round(p95_age_s, 6),
                                "max_p95_age_s": args.max_p95_age_s,
                            }
                        ),
                        flush=True,
                    )
                    return 0
            if now_s - last_report_s >= 2.0:
                last_report_s = now_s
                p95_age_s = (
                    _percentile(list(node.ages_s), 0.95) if node.ages_s else None
                )
                print(
                    json.dumps(
                        {
                            "status": "waiting",
                            "mode": args.mode,
                            "samples": len(node.ages_s),
                            "p95_age_s": p95_age_s,
                        }
                    ),
                    flush=True,
                )
        print(
            json.dumps(
                {
                    "status": "timeout",
                    "mode": args.mode,
                    "samples": len(node.ages_s),
                    "p95_age_s": (
                        _percentile(list(node.ages_s), 0.95)
                        if node.ages_s
                        else None
                    ),
                }
            ),
            flush=True,
        )
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
