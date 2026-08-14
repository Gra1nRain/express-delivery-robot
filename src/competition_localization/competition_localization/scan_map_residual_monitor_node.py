#!/usr/bin/env python3
"""Observe scan-to-static-map residuals without changing localization or motion."""

from __future__ import annotations

from collections import deque
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from competition_localization.planar_transform import yaw_from_quaternion
from competition_localization.scan_map_residual import (
    OccupancyDistanceField,
    ScanMatchConfig,
    StationaryResidualSample,
    classify_stationary_residuals,
    correction_about_sensor_as_transform,
    laser_scan_points,
    match_scan_to_map,
    velocity_is_stationary,
)


def _stamp_seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ScanMapResidualMonitor(Node):
    """Publish metric scan-to-map alignment diagnostics; never publish a TF or command."""

    def __init__(self) -> None:
        super().__init__("scan_map_residual_monitor")
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._max_scan_age_s = float(
            self.declare_parameter("max_scan_age_s", 0.50).value
        )
        self._update_period_s = 1.0 / float(
            self.declare_parameter("update_rate_hz", 0.5).value
        )
        self._max_scan_points = int(
            self.declare_parameter("max_scan_points", 360).value
        )
        self._stationary_linear_speed_mps = float(
            self.declare_parameter("stationary_linear_speed_mps", 0.01).value
        )
        self._stationary_yaw_rate_radps = float(
            self.declare_parameter("stationary_yaw_rate_radps", 0.01).value
        )
        self._max_vehicle_odom_age_s = float(
            self.declare_parameter("max_vehicle_odom_age_s", 0.50).value
        )
        self._stationary_min_duration_s = float(
            self.declare_parameter("stationary_min_duration_s", 300.0).value
        )
        self._config = ScanMatchConfig(
            translation_window_m=float(
                self.declare_parameter("translation_window_m", 0.40).value
            ),
            translation_step_m=float(
                self.declare_parameter("translation_step_m", 0.05).value
            ),
            yaw_window_rad=math.radians(
                float(self.declare_parameter("yaw_window_deg", 10.0).value)
            ),
            yaw_step_rad=math.radians(
                float(self.declare_parameter("yaw_step_deg", 1.0).value)
            ),
            fine_translation_window_m=float(
                self.declare_parameter("fine_translation_window_m", 0.05).value
            ),
            fine_translation_step_m=float(
                self.declare_parameter("fine_translation_step_m", 0.01).value
            ),
            fine_yaw_window_rad=math.radians(
                float(self.declare_parameter("fine_yaw_window_deg", 1.0).value)
            ),
            fine_yaw_step_rad=math.radians(
                float(self.declare_parameter("fine_yaw_step_deg", 0.25).value)
            ),
            max_residual_m=float(self.declare_parameter("max_residual_m", 0.50).value),
            inlier_threshold_m=float(
                self.declare_parameter("inlier_threshold_m", 0.10).value
            ),
            min_points=int(self.declare_parameter("min_points", 60).value),
        )
        self._distance_field: OccupancyDistanceField | None = None
        self._latest_scan: LaserScan | None = None
        self._last_scan_stamp_s: float | None = None
        self._vehicle_velocity: tuple[float, float] | None = None
        self._vehicle_odom_stamp_s: float | None = None
        self._fastlio_odom_stamp_s: float | None = None
        self._stationary_samples: deque[StationaryResidualSample] = deque(maxlen=1200)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.declare_parameter("map_topic", "/map").value),
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.declare_parameter("scan_topic", "/scan").value),
            self._scan_callback,
            scan_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.declare_parameter("vehicle_odom_topic", "/odom").value),
            self._vehicle_odom_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.declare_parameter("fastlio_odom_topic", "/Odometry").value),
            self._fastlio_odom_callback,
            20,
        )
        self._publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic", "/localization/scan_map_residual"
                ).value
            ),
            10,
        )
        self._timer = self.create_timer(
            self._update_period_s,
            self._process_latest_scan,
        )
        self.get_logger().info(
            "Scan-map residual monitor ready (observation only; no TF or motion output)"
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        self._distance_field = OccupancyDistanceField.from_occupancy(
            message.data,
            width=int(message.info.width),
            height=int(message.info.height),
            resolution_m=float(message.info.resolution),
            origin_x_m=float(origin.position.x),
            origin_y_m=float(origin.position.y),
            origin_yaw_rad=yaw_from_quaternion(
                float(origin.orientation.x),
                float(origin.orientation.y),
                float(origin.orientation.z),
                float(origin.orientation.w),
            ),
        )
        self.get_logger().info(
            f"Loaded static map {message.info.width}x{message.info.height}"
        )

    def _vehicle_odom_callback(self, message: Odometry) -> None:
        twist = message.twist.twist
        self._vehicle_velocity = (
            math.hypot(float(twist.linear.x), float(twist.linear.y)),
            abs(float(twist.angular.z)),
        )
        self._vehicle_odom_stamp_s = _stamp_seconds(message.header.stamp)

    def _fastlio_odom_callback(self, message: Odometry) -> None:
        self._fastlio_odom_stamp_s = _stamp_seconds(message.header.stamp)

    def _scan_callback(self, message: LaserScan) -> None:
        self._latest_scan = message

    def _process_latest_scan(self) -> None:
        message = self._latest_scan
        if message is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        scan_stamp_s = _stamp_seconds(message.header.stamp)
        scan_age_s = now_s - scan_stamp_s
        if self._last_scan_stamp_s == scan_stamp_s:
            return
        if self._distance_field is None or scan_age_s > self._max_scan_age_s:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as exc:
            self.get_logger().warning(f"Scan-map TF unavailable: {exc}")
            return
        self._last_scan_stamp_s = scan_stamp_s
        points_scan = laser_scan_points(
            message.ranges,
            angle_min_rad=float(message.angle_min),
            angle_increment_rad=float(message.angle_increment),
            range_min_m=float(message.range_min),
            range_max_m=float(message.range_max),
            max_points=self._max_scan_points,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sensor_xy = (float(translation.x), float(translation.y))
        yaw = yaw_from_quaternion(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_rotation = np.array(((cos_yaw, -sin_yaw), (sin_yaw, cos_yaw)))
        points_map = points_scan @ map_rotation.T + np.asarray(sensor_xy)
        try:
            result = match_scan_to_map(
                self._distance_field,
                points_map,
                sensor_xy_m=sensor_xy,
                config=self._config,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Scan-map residual unavailable: {exc}")
            return
        stationary = self._is_stationary(now_s)
        if stationary and result.confident:
            self._stationary_samples.append(
                StationaryResidualSample(
                    now_s,
                    result.correction_x_m,
                    result.correction_y_m,
                    result.correction_yaw_rad,
                )
            )
        elif not stationary:
            self._stationary_samples.clear()
        assessment = classify_stationary_residuals(
            tuple(self._stationary_samples),
            min_duration_s=self._stationary_min_duration_s,
        )
        classification = assessment.classification
        if not result.confident:
            classification = "low_confidence"
        elif not stationary:
            classification = "moving_observation"
        anchor_correction = correction_about_sensor_as_transform(
            sensor_xy_m=sensor_xy,
            dx_m=result.correction_x_m,
            dy_m=result.correction_y_m,
            dyaw_rad=result.correction_yaw_rad,
        )
        status = {
            "classification": classification,
            "stationary": stationary,
            "confident": result.confident,
            "stamp_s": scan_stamp_s,
            "scan_age_s": scan_age_s,
            "tf_age_s": now_s - _stamp_seconds(transform.header.stamp),
            "vehicle_odom_age_s": self._age(now_s, self._vehicle_odom_stamp_s),
            "fastlio_odom_age_s": self._age(now_s, self._fastlio_odom_stamp_s),
            "correction_x_m": result.correction_x_m,
            "correction_y_m": result.correction_y_m,
            "correction_yaw_rad": result.correction_yaw_rad,
            "correction_yaw_deg": math.degrees(result.correction_yaw_rad),
            "anchor_correction_x_m": anchor_correction.x,
            "anchor_correction_y_m": anchor_correction.y,
            "anchor_correction_yaw_rad": anchor_correction.yaw,
            "baseline_mean_residual_m": result.baseline_mean_residual_m,
            "baseline_median_residual_m": result.baseline_median_residual_m,
            "baseline_inlier_ratio": result.baseline_inlier_ratio,
            "best_mean_residual_m": result.best_mean_residual_m,
            "best_median_residual_m": result.best_median_residual_m,
            "best_p90_residual_m": result.best_p90_residual_m,
            "inlier_ratio": result.inlier_ratio,
            "point_count": result.point_count,
            "search_boundary_hit": result.search_boundary_hit,
            "stationary_sample_count": assessment.sample_count,
            "stationary_duration_s": assessment.duration_s,
            "stationary_translation_change_m": assessment.translation_change_m,
            "stationary_yaw_change_deg": math.degrees(assessment.yaw_change_rad),
            "stationary_translation_rate_mps": assessment.translation_rate_mps,
            "stationary_yaw_rate_degps": math.degrees(assessment.yaw_rate_radps),
        }
        encoded = json.dumps(status, separators=(",", ":"), allow_nan=False)
        self._publisher.publish(String(data=encoded))
        self.get_logger().info(encoded)

    def _is_stationary(self, now_s: float) -> bool:
        if self._vehicle_velocity is None or self._vehicle_odom_stamp_s is None:
            return False
        linear_speed, yaw_rate = self._vehicle_velocity
        return velocity_is_stationary(
            linear_speed_mps=linear_speed,
            yaw_rate_radps=yaw_rate,
            odom_age_s=now_s - self._vehicle_odom_stamp_s,
            max_odom_age_s=self._max_vehicle_odom_age_s,
            linear_threshold_mps=self._stationary_linear_speed_mps,
            yaw_rate_threshold_radps=self._stationary_yaw_rate_radps,
        )

    @staticmethod
    def _age(now_s: float, stamp_s: float | None) -> float | None:
        return None if stamp_s is None else now_s - stamp_s


def main() -> None:
    rclpy.init()
    node = ScanMapResidualMonitor()
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
