#!/usr/bin/env python3
"""Anchor FAST-LIO's local frame to the configured map after /initialpose.

FAST-LIO publishes ``camera_init -> body`` as a local odometry transform. A
known initial pose is enough to derive the fixed ``map -> camera_init``
transform without introducing a second pose estimator. The anchor is
deliberately opt-in; AMCL and this node must not publish the same TF chain at
the same time.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from competition_localization.anchor_manager import (
    AnchorCorrectionMode,
    AnchorManager,
    AnchorSafetyState,
)
from competition_localization.planar_transform import PlanarTransform, yaw_from_quaternion


def _pose_to_planar(message: PoseWithCovarianceStamped) -> PlanarTransform:
    pose = message.pose.pose
    return PlanarTransform(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(
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
        self._anchor_manager = AnchorManager()
        self._target_map_base: PlanarTransform | None = None
        self._coarse_anchor_pending = False
        self._warned_waiting_tf = False
        self._route_enabled = False
        self._checkpoint_hold = False
        self._vehicle_velocity: tuple[float, float] | None = None
        self._vehicle_odom_stamp_s: float | None = None
        self._stationary_linear_speed_mps = float(
            self.declare_parameter("stationary_linear_speed_mps", 0.01).value
        )
        self._stationary_yaw_rate_radps = float(
            self.declare_parameter("stationary_yaw_rate_radps", 0.01).value
        )
        self._max_vehicle_odom_age_s = float(
            self.declare_parameter("max_vehicle_odom_age_s", 0.50).value
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "anchor_status_topic", "/localization/anchor_status"
                ).value
            ),
            status_qos,
        )

        self._initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.declare_parameter("vehicle_odom_topic", "/odom").value),
            self._vehicle_odom_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(
                self.declare_parameter(
                    "route_enable_topic", "/mission/route_enable"
                ).value
            ),
            self._route_enable_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "alignment_status_topic", "/localization/alignment_status"
                ).value
            ),
            self._alignment_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.declare_parameter(
                    "anchor_update_request_topic",
                    "/localization/anchor_update_request",
                ).value
            ),
            self._anchor_update_request_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_anchor)
        self.get_logger().info(
            f"Waiting for /initialpose; will anchor {self.map_frame} -> {self.odom_frame} "
            f"from {self.odom_frame} -> {self.base_frame}"
        )

    def _initialpose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if self._route_enabled:
            self.get_logger().warning("Ignoring /initialpose while route is enabled")
            return
        frame_id = message.header.frame_id or self.map_frame
        if frame_id != self.map_frame:
            self.get_logger().warning(
                f"Ignoring /initialpose in {frame_id}; expected {self.map_frame}"
            )
            return

        target = _pose_to_planar(message)
        if not target.is_finite():
            self.get_logger().warning("Ignoring non-finite /initialpose")
            return

        self._target_map_base = target
        self._coarse_anchor_pending = True
        self._warned_waiting_tf = False
        self.get_logger().info(
            "Received initial pose: "
            f"x={target.x:.3f}, y={target.y:.3f}, yaw={target.yaw:.3f}"
        )

    def _vehicle_odom_callback(self, message: Odometry) -> None:
        twist = message.twist.twist
        self._vehicle_velocity = (
            math.hypot(float(twist.linear.x), float(twist.linear.y)),
            abs(float(twist.angular.z)),
        )
        self._vehicle_odom_stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )

    def _route_enable_callback(self, message: Bool) -> None:
        self._route_enabled = bool(message.data)

    def _alignment_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(status, dict):
            self._checkpoint_hold = bool(status.get("checkpoint_hold"))

    def _anchor_update_request_callback(self, message: String) -> None:
        try:
            request = json.loads(message.data)
            request_id = str(request["request_id"])
            operation = str(request["operation"])
            expected_revision = int(request["expected_revision"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid anchor update request: {exc}")
            return
        safety = AnchorSafetyState(
            stationary=self._is_stationary(),
            route_enabled=self._route_enabled,
            checkpoint_hold=self._checkpoint_hold,
        )
        if operation == "apply":
            try:
                correction = PlanarTransform(
                    x=float(request["correction_x_m"]),
                    y=float(request["correction_y_m"]),
                    yaw=float(request["correction_yaw_rad"]),
                )
                displacement_correction = PlanarTransform(
                    x=float(request["displacement_x_m"]),
                    y=float(request["displacement_y_m"]),
                    yaw=float(request["displacement_yaw_rad"]),
                )
                mode = AnchorCorrectionMode(str(request["mode"]))
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f"Ignoring invalid anchor correction request: {exc}"
                )
                return
            update = self._anchor_manager.apply_correction(
                correction=correction,
                displacement_correction=displacement_correction,
                expected_revision=expected_revision,
                mode=mode,
                safety=safety,
            )
        elif operation == "rollback":
            update = self._anchor_manager.rollback(
                expected_revision=expected_revision,
                safety=safety,
            )
        else:
            self.get_logger().warning(
                f"Ignoring unsupported anchor operation: {operation}"
            )
            return
        self._publish_status(
            source=operation,
            request_id=request_id,
            applied=update.applied,
            reason=update.reason,
        )

    def _lookup_odom_to_base(self) -> PlanarTransform | None:
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
        result = PlanarTransform(
            x=float(translation.x),
            y=float(translation.y),
            yaw=yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        if not result.is_finite():
            self.get_logger().warning("Ignoring non-finite FAST-LIO transform")
            return None
        return result

    def _publish_anchor(self) -> None:
        if self._target_map_base is not None and self._coarse_anchor_pending:
            odom_to_base = self._lookup_odom_to_base()
            if odom_to_base is None:
                return
            update = self._anchor_manager.set_coarse_anchor(
                target_map_base=self._target_map_base,
                odom_to_base=odom_to_base,
                stationary=self._is_stationary(),
            )
            if not update.applied:
                return
            self._coarse_anchor_pending = False
            self.get_logger().info(
                "Anchored map to FAST-LIO: "
                f"x={update.transform.x:.3f}, y={update.transform.y:.3f}, "
                f"yaw={update.transform.yaw:.3f}"
            )
            self._publish_status(
                source="coarse",
                request_id=None,
                applied=True,
                reason=update.reason,
            )

        map_to_odom = self._anchor_manager.transform
        if map_to_odom is None:
            return

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.odom_frame
        message.transform.translation.x = map_to_odom.x
        message.transform.translation.y = map_to_odom.y
        message.transform.translation.z = 0.0
        message.transform.rotation.z = math.sin(map_to_odom.yaw / 2.0)
        message.transform.rotation.w = math.cos(map_to_odom.yaw / 2.0)
        self._tf_broadcaster.sendTransform(message)

    def _is_stationary(self) -> bool:
        if self._vehicle_velocity is None or self._vehicle_odom_stamp_s is None:
            return False
        now_s = self.get_clock().now().nanoseconds * 1e-9
        linear_speed, yaw_rate = self._vehicle_velocity
        return (
            now_s - self._vehicle_odom_stamp_s <= self._max_vehicle_odom_age_s
            and linear_speed <= self._stationary_linear_speed_mps
            and yaw_rate <= self._stationary_yaw_rate_radps
        )

    def _publish_status(
        self,
        *,
        source: str,
        request_id: str | None,
        applied: bool,
        reason: str,
    ) -> None:
        transform = self._anchor_manager.transform
        status = {
            "source": source,
            "request_id": request_id,
            "applied": applied,
            "reason": reason,
            "revision": self._anchor_manager.revision,
            "stationary": self._is_stationary(),
            "route_enabled": self._route_enabled,
            "checkpoint_hold": self._checkpoint_hold,
            "x_m": transform.x if transform is not None else None,
            "y_m": transform.y if transform is not None else None,
            "yaw_rad": transform.yaw if transform is not None else None,
        }
        self._status_publisher.publish(
            String(data=json.dumps(status, separators=(",", ":"), allow_nan=False))
        )


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
