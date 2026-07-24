#!/usr/bin/env python3
"""Drop queued Livox frames and expose only the freshest frame to FAST-LIO."""

from __future__ import annotations

import json
import math

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from .livox_latest_frame_gate import (
    LatestFrameGate,
    header_stamp_seconds_from_cdr,
)


class LivoxLatestFrameAdapterNode(Node):
    """Republish at a bounded rate from a best-effort, depth-one input."""

    def __init__(self) -> None:
        super().__init__("livox_latest_frame_adapter")
        input_topic = str(
            self.declare_parameter("input_topic", "/livox/lidar").value
        )
        output_topic = str(
            self.declare_parameter(
                "output_topic", "/avoidance/livox_latest"
            ).value
        )
        publish_frequency_hz = float(
            self.declare_parameter("publish_frequency_hz", 10.0).value
        )
        maximum_input_age_s = float(
            self.declare_parameter("maximum_input_age_s", 0.40).value
        )
        if input_topic == output_topic:
            raise ValueError("Livox adapter input and output must differ")
        if (
            not math.isfinite(publish_frequency_hz)
            or publish_frequency_hz <= 0.0
        ):
            raise ValueError("publish_frequency_hz must be finite and positive")

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            CustomMsg, output_topic, output_qos
        )
        self._status_publisher = self.create_publisher(
            String, "/avoidance/livox_adapter_status", 1
        )
        self.create_subscription(
            CustomMsg, input_topic, self._receive, input_qos, raw=True
        )

        self._gate = LatestFrameGate(maximum_input_age_s)
        self._latest_serialized_message: bytes | None = None
        self._latest_sequence = 0
        self._received_count = 0
        self._published_count = 0
        self._stale_count = 0
        self._latest_age_s: float | None = None
        self.create_timer(1.0 / publish_frequency_hz, self._publish_latest)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "Livox latest-frame adapter ready: "
            f"{input_topic} -> {output_topic}, "
            f"rate<={publish_frequency_hz:.1f}Hz, "
            f"age<={maximum_input_age_s:.2f}s"
        )

    def _receive(self, serialized_message: bytes) -> None:
        self._latest_serialized_message = serialized_message
        self._latest_sequence += 1
        self._received_count += 1

    def _message_age_s(self, serialized_message: bytes) -> float:
        stamp_s = header_stamp_seconds_from_cdr(serialized_message)
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        age_s = now_s - stamp_s
        if -0.05 <= age_s < 0.0:
            return 0.0
        return age_s

    def _publish_latest(self) -> None:
        serialized_message = self._latest_serialized_message
        if serialized_message is None:
            return
        sequence = self._latest_sequence
        try:
            age_s = self._message_age_s(serialized_message)
        except ValueError as error:
            self._stale_count += 1
            self.get_logger().error(
                f"Rejecting malformed serialized Livox frame: {error}"
            )
            return
        self._latest_age_s = age_s
        if not self._gate.should_publish(sequence, age_s):
            if age_s > self._gate.maximum_age_s or age_s < 0.0:
                self._stale_count += 1
            return
        self._publisher.publish(serialized_message)
        self._gate.mark_published(sequence)
        self._published_count += 1

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps(
            {
                "received": self._received_count,
                "published": self._published_count,
                "stale_rejected": self._stale_count,
                "latest_age_s": self._latest_age_s,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._status_publisher.publish(status)


def main() -> None:
    rclpy.init()
    node = LivoxLatestFrameAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
