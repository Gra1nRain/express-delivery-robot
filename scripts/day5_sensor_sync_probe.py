#!/usr/bin/env python3
"""Measure Livox/IMU/FAST-LIO timing without queueing old samples."""

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


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SensorSyncProbe(Node):
    def __init__(self) -> None:
        super().__init__("day5_sensor_sync_probe")
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import Imu, PointCloud2

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.lidar_end_ns: int | None = None
        self.lidar_span_s: float | None = None
        self.lidar_points = 0
        self.imu_ns: int | None = None
        self.cloud_ns: int | None = None
        self.counts = {"lidar": 0, "imu": 0, "cloud": 0}
        self.create_subscription(CustomMsg, "/livox/lidar", self._lidar_cb, qos)
        self.create_subscription(Imu, "/livox/imu", self._imu_cb, qos)
        self.create_subscription(
            PointCloud2,
            "/cloud_registered_body",
            self._cloud_cb,
            qos,
        )

    def _lidar_cb(self, message) -> None:
        offset_ns = int(message.points[-1].offset_time) if message.points else 0
        self.lidar_end_ns = int(message.timebase) + offset_ns
        self.lidar_span_s = offset_ns * 1e-9
        self.lidar_points = int(message.point_num)
        self.counts["lidar"] += 1

    def _imu_cb(self, message) -> None:
        self.imu_ns = _stamp_ns(message.header.stamp)
        self.counts["imu"] += 1

    def _cloud_cb(self, message) -> None:
        self.cloud_ns = _stamp_ns(message.header.stamp)
        self.counts["cloud"] += 1

    def snapshot(self, elapsed_s: float) -> dict[str, object]:
        now_ns = time.time_ns()

        def age_s(stamp_ns: int | None) -> float | None:
            if stamp_ns is None:
                return None
            age = (now_ns - stamp_ns) * 1e-9
            return round(age, 6) if math.isfinite(age) else None

        lidar_minus_imu_s = None
        if self.lidar_end_ns is not None and self.imu_ns is not None:
            lidar_minus_imu_s = round(
                (self.lidar_end_ns - self.imu_ns) * 1e-9,
                6,
            )
        rates_hz = {
            name: round(count / elapsed_s, 3)
            for name, count in self.counts.items()
        }
        return {
            "elapsed_s": round(elapsed_s, 3),
            "rates_hz": rates_hz,
            "lidar_age_s": age_s(self.lidar_end_ns),
            "imu_age_s": age_s(self.imu_ns),
            "cloud_age_s": age_s(self.cloud_ns),
            "lidar_end_minus_imu_s": lidar_minus_imu_s,
            "lidar_scan_span_s": (
                round(self.lidar_span_s, 6)
                if self.lidar_span_s is not None
                else None
            ),
            "lidar_points": self.lidar_points,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--report-period-s", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration_s <= 0.0 or args.report_period_s <= 0.0:
        parser.error("durations must be positive")

    rclpy.init()
    node = SensorSyncProbe()
    started_s = time.monotonic()
    next_report_s = started_s + args.report_period_s
    try:
        while time.monotonic() - started_s < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            now_s = time.monotonic()
            if now_s >= next_report_s:
                print(
                    json.dumps(node.snapshot(now_s - started_s)),
                    flush=True,
                )
                next_report_s += args.report_period_s
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
