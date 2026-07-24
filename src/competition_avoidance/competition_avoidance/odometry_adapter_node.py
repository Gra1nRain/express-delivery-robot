#!/usr/bin/env python3
"""Bridge FAST-LIO /Odometry to the frozen navigation /odom interface."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryAdapterNode(Node):
    """Republish odometry unchanged without owning any motion topic."""

    def __init__(self) -> None:
        super().__init__("odometry_adapter")
        input_topic = str(
            self.declare_parameter("input_topic", "/Odometry").value
        )
        output_topic = str(
            self.declare_parameter("output_topic", "/odom").value
        )
        if input_topic == output_topic:
            raise ValueError("odometry adapter input and output must differ")

        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self.create_subscription(Odometry, input_topic, self._relay, 20)
        self.get_logger().info(
            f"Odometry adapter ready: {input_topic} -> {output_topic}"
        )

    def _relay(self, message: Odometry) -> None:
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = OdometryAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
