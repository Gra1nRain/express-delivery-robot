#!/usr/bin/env python3
"""Rebase Livox lidar/IMU timestamps into the host ROS clock domain."""

from __future__ import annotations

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from competition_localization.livox_clock import ClockOffsetEstimator


_NANOSECONDS_PER_SECOND = 1_000_000_000


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * _NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def _set_stamp_from_nanoseconds(stamp, value_ns: int) -> None:
    stamp.sec = int(value_ns // _NANOSECONDS_PER_SECOND)
    stamp.nanosec = int(value_ns % _NANOSECONDS_PER_SECOND)


class LivoxTimestampRebaser(Node):
    """Apply one frozen lidar-to-host clock offset to lidar and IMU messages."""

    def __init__(self) -> None:
        super().__init__("livox_timestamp_rebaser")
        calibration_samples = int(
            self.declare_parameter("calibration_samples", 20).value
        )
        self._clock_offset = ClockOffsetEstimator(calibration_samples)
        self._last_output_ns: dict[str, int] = {}
        self._dropped_lidar_before_calibration = 0

        input_lidar_topic = str(
            self.declare_parameter("input_lidar_topic", "/livox/lidar_raw").value
        )
        input_imu_topic = str(
            self.declare_parameter("input_imu_topic", "/livox/imu_raw").value
        )
        output_lidar_topic = str(
            self.declare_parameter("output_lidar_topic", "/livox/lidar").value
        )
        output_imu_topic = str(
            self.declare_parameter("output_imu_topic", "/livox/imu").value
        )

        qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
        self._lidar_publisher = self.create_publisher(
            CustomMsg, output_lidar_topic, qos
        )
        self._imu_publisher = self.create_publisher(Imu, output_imu_topic, qos)
        self.create_subscription(CustomMsg, input_lidar_topic, self._on_lidar, qos)
        self.create_subscription(Imu, input_imu_topic, self._on_imu, qos)
        self.get_logger().info(
            "Livox timestamp rebaser waiting for "
            f"{calibration_samples} IMU clock samples"
        )

    def _on_imu(self, message: Imu) -> None:
        source_ns = _stamp_to_nanoseconds(message.header.stamp)
        if not self._clock_offset.ready:
            became_ready = self._clock_offset.observe(
                source_stamp_ns=source_ns,
                receipt_stamp_ns=self.get_clock().now().nanoseconds,
            )
            if not became_ready:
                return
            self.get_logger().info(
                "Livox clock calibrated: offset="
                f"{self._clock_offset.offset_ns / 1e9:.6f}s, "
                f"dropped_lidar_frames={self._dropped_lidar_before_calibration}"
            )

        rebased_ns = self._clock_offset.rebase(source_ns)
        if not self._accept_monotonic("imu", rebased_ns):
            return
        _set_stamp_from_nanoseconds(message.header.stamp, rebased_ns)
        self._imu_publisher.publish(message)

    def _on_lidar(self, message: CustomMsg) -> None:
        if not self._clock_offset.ready:
            self._dropped_lidar_before_calibration += 1
            return

        source_ns = _stamp_to_nanoseconds(message.header.stamp)
        rebased_ns = self._clock_offset.rebase(source_ns)
        if not self._accept_monotonic("lidar", rebased_ns):
            return
        _set_stamp_from_nanoseconds(message.header.stamp, rebased_ns)
        message.timebase = self._clock_offset.rebase(int(message.timebase))
        self._lidar_publisher.publish(message)

    def _accept_monotonic(self, stream: str, stamp_ns: int) -> bool:
        previous_ns = self._last_output_ns.get(stream)
        if previous_ns is not None and stamp_ns < previous_ns:
            self.get_logger().error(
                f"Dropping reversed {stream} timestamp: "
                f"{stamp_ns} < {previous_ns}"
            )
            return False
        self._last_output_ns[stream] = stamp_ns
        return True


def main() -> None:
    rclpy.init()
    node = LivoxTimestampRebaser()
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
