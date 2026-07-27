#!/usr/bin/env python3
"""Project the 10 Hz Livox CustomMsg stream into a body-frame LaserScan."""

from __future__ import annotations

import math

from livox_ros_driver2.msg import CustomMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan

from competition_safety.livox_scan_projection import (
    ScanProjectionConfig,
    project_points_to_scan_ranges,
)


class LivoxCustomToScanNode(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_scan")
        self._input_topic = str(
            self.declare_parameter("input_topic", "/livox/lidar").value
        )
        self._output_topic = str(
            self.declare_parameter("output_topic", "/scan").value
        )
        self._expected_input_frame = str(
            self.declare_parameter("expected_input_frame", "livox_frame").value
        )
        self._output_frame = str(
            self.declare_parameter("output_frame", "body").value
        )
        self._max_input_age_s = float(
            self.declare_parameter("max_input_age_s", 0.25).value
        )
        self._point_stride = int(self.declare_parameter("point_stride", 1).value)
        self._scan_time_s = float(
            self.declare_parameter("scan_time_s", 0.10).value
        )
        if self._max_input_age_s <= 0.0:
            raise ValueError("max_input_age_s must be positive")
        if self._point_stride < 1:
            raise ValueError("point_stride must be at least 1")
        if self._scan_time_s <= 0.0:
            raise ValueError("scan_time_s must be positive")

        sensor_to_body_xyz = list(
            self.declare_parameter(
                "sensor_to_body_xyz_m",
                [-0.011, -0.02329, 0.04412],
            ).value
        )
        if len(sensor_to_body_xyz) != 3:
            raise ValueError("sensor_to_body_xyz_m must contain three values")
        self._config = ScanProjectionConfig(
            min_height_m=float(
                self.declare_parameter("min_height_m", -0.25).value
            ),
            max_height_m=float(
                self.declare_parameter("max_height_m", 0.80).value
            ),
            angle_min_rad=float(
                self.declare_parameter("angle_min_rad", -math.pi).value
            ),
            angle_max_rad=float(
                self.declare_parameter("angle_max_rad", math.pi).value
            ),
            angle_increment_rad=float(
                self.declare_parameter(
                    "angle_increment_rad",
                    math.radians(0.5),
                ).value
            ),
            range_min_m=float(
                self.declare_parameter("range_min_m", 0.10).value
            ),
            range_max_m=float(
                self.declare_parameter("range_max_m", 6.00).value
            ),
            sensor_to_body_x_m=float(sensor_to_body_xyz[0]),
            sensor_to_body_y_m=float(sensor_to_body_xyz[1]),
            sensor_to_body_z_m=float(sensor_to_body_xyz[2]),
            sensor_to_body_yaw_rad=float(
                self.declare_parameter("sensor_to_body_yaw_rad", 0.0).value
            ),
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            LaserScan,
            self._output_topic,
            qos,
        )
        self.create_subscription(
            CustomMsg,
            self._input_topic,
            self._callback,
            qos,
        )
        self.get_logger().info(
            "Livox 2D projection ready: "
            f"{self._input_topic} ({self._expected_input_frame}) -> "
            f"{self._output_topic} ({self._output_frame}), "
            f"bins={self._config.bin_count}, stride={self._point_stride}"
        )

    def _callback(self, message: CustomMsg) -> None:
        if (
            self._expected_input_frame
            and message.header.frame_id != self._expected_input_frame
        ):
            self.get_logger().error(
                "Ignoring Livox frame mismatch: "
                f"{message.header.frame_id!r} != {self._expected_input_frame!r}",
                throttle_duration_sec=2.0,
            )
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        stamp_s = _stamp_to_seconds(message.header.stamp)
        if stamp_s > 0.0 and now_s - stamp_s > self._max_input_age_s:
            self.get_logger().warning(
                f"Ignoring stale Livox message: age={now_s - stamp_s:.3f}s",
                throttle_duration_sec=2.0,
            )
            return

        ranges = project_points_to_scan_ranges(
            (
                (float(point.x), float(point.y), float(point.z))
                for index, point in enumerate(message.points)
                if index % self._point_stride == 0
            ),
            self._config,
        )
        scan = LaserScan()
        scan.header.stamp = message.header.stamp
        scan.header.frame_id = self._output_frame
        scan.angle_min = self._config.angle_min_rad
        scan.angle_max = (
            self._config.angle_min_rad
            + (len(ranges) - 1) * self._config.angle_increment_rad
        )
        scan.angle_increment = self._config.angle_increment_rad
        scan.scan_time = self._scan_time_s
        scan.time_increment = (
            self._scan_time_s / len(ranges) if ranges else 0.0
        )
        scan.range_min = self._config.range_min_m
        scan.range_max = self._config.range_max_m
        scan.ranges = list(ranges)
        self._publisher.publish(scan)


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = LivoxCustomToScanNode()
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
