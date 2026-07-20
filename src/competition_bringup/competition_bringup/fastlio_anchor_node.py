#!/usr/bin/env python3
"""Anchor FAST-LIO's local frame to the configured map after /initialpose.

FAST-LIO publishes ``camera_init -> body`` as a local odometry transform.  A
known initial pose is enough to derive the fixed ``map -> camera_init``
transform without introducing a second pose estimator.  The anchor is
deliberately opt-in; AMCL and this node must not publish the same TF chain at
the same time.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _compose(first: tuple[float, float, float], second: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose planar transforms first * second."""

    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    cos_yaw = math.cos(yaw1)
    sin_yaw = math.sin(yaw1)
    return (
        x1 + cos_yaw * x2 - sin_yaw * y2,
        y1 + sin_yaw * x2 + cos_yaw * y2,
        _wrap_angle(yaw1 + yaw2),
    )


def _inverse(transform: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, yaw = transform
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        _wrap_angle(-yaw),
    )


def _pose_to_planar(message: PoseWithCovarianceStamped) -> tuple[float, float, float]:
    pose = message.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        _yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


class FastLioAnchor(Node):
    """Publish one fixed map-to-local-frame anchor from an initial pose."""

    def __init__(self) -> None:
        super().__init__("fastlio_anchor")
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "camera_init").value)
        self.base_frame = str(self.declare_parameter("base_frame", "body").value)
        publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._target_map_base: tuple[float, float, float] | None = None
        self._map_to_odom: tuple[float, float, float] | None = None
        self._warned_waiting_tf = False

        self._initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_anchor)
        self.get_logger().info(
            f"Waiting for /initialpose; will anchor {self.map_frame} -> {self.odom_frame} "
            f"from {self.odom_frame} -> {self.base_frame}"
        )

    def _initialpose_callback(self, message: PoseWithCovarianceStamped) -> None:
        frame_id = message.header.frame_id or self.map_frame
        if frame_id != self.map_frame:
            self.get_logger().warning(
                f"Ignoring /initialpose in {frame_id}; expected {self.map_frame}"
            )
            return

        target = _pose_to_planar(message)
        if not _finite(*target):
            self.get_logger().warning("Ignoring non-finite /initialpose")
            return

        self._target_map_base = target
        self._map_to_odom = None
        self._warned_waiting_tf = False
        self.get_logger().info(
            "Received initial pose: "
            f"x={target[0]:.3f}, y={target[1]:.3f}, yaw={target[2]:.3f}"
        )

    def _lookup_odom_to_base(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            if not self._warned_waiting_tf:
                self.get_logger().warning(
                    f"Waiting for {self.odom_frame} -> {self.base_frame}"
                )
                self._warned_waiting_tf = True
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        result = (
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        if not _finite(*result):
            self.get_logger().warning("Ignoring non-finite FAST-LIO transform")
            return None
        return result

    def _publish_anchor(self) -> None:
        if self._target_map_base is not None and self._map_to_odom is None:
            odom_to_base = self._lookup_odom_to_base()
            if odom_to_base is None:
                return
            self._map_to_odom = _compose(self._target_map_base, _inverse(odom_to_base))
            self.get_logger().info(
                "Anchored map to FAST-LIO: "
                f"x={self._map_to_odom[0]:.3f}, y={self._map_to_odom[1]:.3f}, "
                f"yaw={self._map_to_odom[2]:.3f}"
            )

        if self._map_to_odom is None:
            return

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.odom_frame
        message.transform.translation.x = self._map_to_odom[0]
        message.transform.translation.y = self._map_to_odom[1]
        message.transform.translation.z = 0.0
        message.transform.rotation.z = math.sin(self._map_to_odom[2] / 2.0)
        message.transform.rotation.w = math.cos(self._map_to_odom[2] / 2.0)
        self._tf_broadcaster.sendTransform(message)


def main() -> None:
    rclpy.init()
    node = FastLioAnchor()
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
