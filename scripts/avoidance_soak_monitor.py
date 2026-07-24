#!/usr/bin/env python3
"""Low-overhead avoidance freshness and mode soak monitor."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AvoidanceSoakMonitor(Node):
    def __init__(self) -> None:
        super().__init__("avoidance_soak_monitor")
        self.samples = 0
        self.maximum_cloud_age_s = 0.0
        self.reasons: dict[str, int] = {}
        self.modes: dict[str, int] = {}
        self.create_subscription(String, "/avoidance/status", self._callback, 10)

    def _callback(self, message: String) -> None:
        payload = json.loads(message.data)
        self.samples += 1
        reason = str(payload.get("reason", ""))
        mode = str(payload.get("mode", ""))
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        self.modes[mode] = self.modes.get(mode, 0) + 1
        if "cloud_age_s" in payload:
            self.maximum_cloud_age_s = max(
                self.maximum_cloud_age_s,
                float(payload["cloud_age_s"]),
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=600.0)
    parser.add_argument("--report-every-s", type=float, default=60.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.duration_s <= 0.0 or args.report_every_s <= 0.0:
        raise ValueError("monitor durations must be positive")

    rclpy.init()
    node = AvoidanceSoakMonitor()
    started = time.monotonic()
    deadline = started + args.duration_s
    next_report = started + args.report_every_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now >= next_report:
                print(
                    json.dumps(
                        {
                            "status": "progress",
                            "elapsed_s": round(now - started, 1),
                            "samples": node.samples,
                            "max_cloud_age_s": round(
                                node.maximum_cloud_age_s,
                                3,
                            ),
                            "stale_count": node.reasons.get(
                                "stale_cloud_timestamp",
                                0,
                            ),
                            "modes": node.modes,
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                next_report += args.report_every_s
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = (
        "pass"
        if node.reasons.get("stale_cloud_timestamp", 0) == 0
        and node.maximum_cloud_age_s < 0.50
        else "fail"
    )
    print(
        json.dumps(
            {
                "status": "complete",
                "elapsed_s": round(time.monotonic() - started, 1),
                "samples": node.samples,
                "max_cloud_age_s": round(node.maximum_cloud_age_s, 3),
                "reasons": node.reasons,
                "modes": node.modes,
                "result": result,
            },
            separators=(",", ":"),
        ),
        flush=True,
    )
    return 0 if result == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
