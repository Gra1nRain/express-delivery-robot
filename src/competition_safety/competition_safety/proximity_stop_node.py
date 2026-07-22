#!/usr/bin/env python3
"""Publish a conservative stop request from a body-frame point cloud."""

from __future__ import annotations

import json

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
from std_msgs.msg import Bool, String

from competition_safety.proximity_stop import (
    ProximityStopConfig,
    should_stop_for_points,
)


class ProximityStopNode(Node):
    def __init__(self) -> None:
        super().__init__("proximity_stop")
        self._cloud_topic = str(
            self.declare_parameter("cloud_topic", "/cloud_registered_body").value
        )
        cloud_qos_reliability = str(
            self.declare_parameter("cloud_qos_reliability", "best_effort").value
        ).lower()
        cloud_qos_depth = int(self.declare_parameter("cloud_qos_depth", 1).value)
        if cloud_qos_reliability not in {"best_effort", "reliable"}:
            raise ValueError(
                "cloud_qos_reliability must be 'best_effort' or 'reliable'"
            )
        if cloud_qos_depth < 1:
            raise ValueError("cloud_qos_depth must be at least 1")
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=cloud_qos_depth,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if cloud_qos_reliability == "best_effort"
                else ReliabilityPolicy.RELIABLE
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self._expected_frame_id = str(
            self.declare_parameter("expected_frame_id", "body").value
        )
        self._max_cloud_age_s = float(
            self.declare_parameter("max_cloud_age_s", 0.50).value
        )
        self._config = ProximityStopConfig(
            x_min_m=float(self.declare_parameter("x_min_m", 0.25).value),
            stop_distance_m=float(
                self.declare_parameter("stop_distance_m", 0.55).value
            ),
            front_half_angle_rad=float(
                self.declare_parameter("front_half_angle_rad", 0.4363).value
            ),
            lateral_half_width_m=float(
                self.declare_parameter("lateral_half_width_m", 0.45).value
            ),
            z_min_m=float(self.declare_parameter("z_min_m", -0.25).value),
            z_max_m=float(self.declare_parameter("z_max_m", 0.80).value),
            min_points=int(self.declare_parameter("min_points", 3).value),
        )
        self._stop_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "stop_request_topic",
                    "/avoidance/stop_request",
                ).value
            ),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic",
                    "/avoidance/proximity_status",
                ).value
            ),
            10,
        )
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            cloud_qos,
        )
        self.get_logger().info(
            "Proximity stop ready: "
            f"cloud_topic={self._cloud_topic}, range=[{self._config.x_min_m:.2f}, "
            f"{self._config.stop_distance_m:.2f}], "
            f"half_angle={self._config.front_half_angle_rad:.3f}rad, "
            f"lateral_half_width={self._config.lateral_half_width_m:.2f}, "
            f"z=[{self._config.z_min_m:.2f}, {self._config.z_max_m:.2f}], "
            f"min_points={self._config.min_points}, "
            f"qos={cloud_qos_reliability}/keep_last_{cloud_qos_depth}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cloud_callback(self, message: PointCloud2) -> None:
        frame_id = message.header.frame_id
        if self._expected_frame_id and frame_id != self._expected_frame_id:
            self._publish(True, "frame_mismatch", 0, frame_id, None)
            return

        cloud_stamp_s = _stamp_to_seconds(message.header.stamp)
        age_s = self._now_s() - cloud_stamp_s if cloud_stamp_s > 0.0 else None
        if age_s is not None and age_s > self._max_cloud_age_s:
            self._publish(True, "stale_cloud", 0, frame_id, age_s)
            return

        points = (
            (float(point[0]), float(point[1]), float(point[2]))
            for point in point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        )
        stop, count = should_stop_for_points(points, self._config)
        self._publish(stop, "obstacle_in_stop_box" if stop else "clear", count, frame_id, age_s)

    def _publish(
        self,
        stop: bool,
        reason: str,
        point_count: int,
        frame_id: str,
        cloud_age_s: float | None,
    ) -> None:
        self._stop_publisher.publish(Bool(data=stop))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "stop": stop,
                        "reason": reason,
                        "point_count": point_count,
                        "frame_id": frame_id,
                        "cloud_age_s": cloud_age_s,
                    },
                    separators=(",", ":"),
                )
            )
        )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = ProximityStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
