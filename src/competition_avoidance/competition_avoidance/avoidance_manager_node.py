#!/usr/bin/env python3
"""ROS adapter for additive static and dynamic avoidance."""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from competition_avoidance.costmap_frame import (
    transform_grid_origin,
    yaw_quaternion,
)
from competition_avoidance.engine import AvoidanceDecision, AvoidanceEngine
from competition_avoidance.perception import (
    ObstacleDetection,
    PerceptionConfig,
    cluster_points,
)
from competition_avoidance.rate_gate import LatestSampleRateGate
from competition_avoidance.risk import EgoState, RiskConfig
from competition_avoidance.tracker import TrackerConfig
from competition_safety.proximity_stop import (
    LocalGridConfig,
    ProximityStopConfig,
    evaluate_local_clearance,
)


class AvoidanceManagerNode(Node):
    """Publish the canonical avoidance topics without owning any motion topic."""

    def __init__(self) -> None:
        super().__init__("avoidance_manager")
        self._dry_run = bool(self.declare_parameter("dry_run", True).value)
        self._enable_chassis_output = bool(
            self.declare_parameter("enable_chassis_output", False).value
        )
        self._operation_mode = str(
            self.declare_parameter("operation_mode", "dry_run").value
        )
        if (
            not self._dry_run
            or self._enable_chassis_output
            or self._operation_mode != "dry_run"
        ):
            raise RuntimeError("avoidance_manager is currently authorized for dry-run only")

        self._cloud_topic = str(
            self.declare_parameter("cloud_topic", "/cloud_registered_body").value
        )
        self._odometry_topic = str(
            self.declare_parameter("odometry_topic", "/odom").value
        )
        self._expected_odometry_frame = str(
            self.declare_parameter("expected_odometry_frame", "camera_init").value
        )
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._expected_cloud_frame = str(
            self.declare_parameter("expected_cloud_frame", "body").value
        )
        self._maximum_cloud_age_s = float(
            self.declare_parameter("maximum_cloud_age_s", 0.50).value
        )
        self._perception_timeout_s = float(
            self.declare_parameter("perception_timeout_s", 0.30).value
        )
        self._odometry_timeout_s = float(
            self.declare_parameter("odometry_timeout_s", 0.30).value
        )
        self._transform_timeout_s = float(
            self.declare_parameter("transform_timeout_s", 0.05).value
        )
        processing_frequency_hz = float(
            self.declare_parameter(
                "maximum_processing_frequency_hz",
                10.0,
            ).value
        )
        self._processing_rate_gate = LatestSampleRateGate(processing_frequency_hz)
        self._scan_time_s = 1.0 / processing_frequency_hz
        stop_frequency_hz = float(
            self.declare_parameter("stop_publish_frequency_hz", 20.0).value
        )
        self._corridor_ttl_s = float(
            self.declare_parameter("corridor_ttl_s", 0.50).value
        )
        if min(
            self._maximum_cloud_age_s,
            self._perception_timeout_s,
            self._odometry_timeout_s,
            self._transform_timeout_s,
            stop_frequency_hz,
            self._corridor_ttl_s,
        ) <= 0.0:
            raise ValueError("avoidance timing parameters must be positive")

        planning_radius = float(
            self.declare_parameter("planning_min_turning_radius_m", 0.81).value
        )
        maximum_speed = float(
            self.declare_parameter("maximum_speed_mps", 0.15).value
        )
        maximum_acceleration = float(
            self.declare_parameter("maximum_acceleration_mps2", 0.20).value
        )
        maximum_deceleration = float(
            self.declare_parameter("maximum_deceleration_mps2", 0.30).value
        )
        vehicle_length = float(
            self.declare_parameter("vehicle_length_m", 0.72).value
        )
        vehicle_width = float(
            self.declare_parameter("vehicle_width_m", 0.50).value
        )
        if planning_radius < 0.81 - 1e-9:
            raise ValueError("avoidance planning radius may not undercut global 0.81 m")
        if maximum_speed > 0.15 + 1e-9:
            raise ValueError("avoidance maximum speed may not exceed 0.15 m/s")
        if maximum_acceleration > 0.20 + 1e-9:
            raise ValueError("avoidance acceleration may not exceed 0.20 m/s^2")
        if vehicle_length <= 0.0 or vehicle_width <= 0.0:
            raise ValueError("vehicle dimensions must be positive")

        proximity_config = ProximityStopConfig(
            x_min_m=float(self.declare_parameter("proximity_x_min_m", 0.25).value),
            stop_distance_m=float(
                self.declare_parameter("proximity_stop_distance_m", 0.85).value
            ),
            front_half_angle_rad=float(
                self.declare_parameter(
                    "proximity_front_half_angle_rad",
                    0.4363,
                ).value
            ),
            lateral_half_width_m=float(
                self.declare_parameter(
                    "proximity_lateral_half_width_m",
                    0.45,
                ).value
            ),
            z_min_m=float(
                self.declare_parameter("proximity_z_min_m", -0.25).value
            ),
            z_max_m=float(
                self.declare_parameter("proximity_z_max_m", 0.80).value
            ),
            min_points=int(
                self.declare_parameter("proximity_min_points", 3).value
            ),
        )
        grid_config = LocalGridConfig(
            resolution_m=float(
                self.declare_parameter("grid_resolution_m", 0.05).value
            ),
            x_min_m=float(self.declare_parameter("grid_x_min_m", -0.50).value),
            x_max_m=float(self.declare_parameter("grid_x_max_m", 5.50).value),
            y_min_m=float(self.declare_parameter("grid_y_min_m", -2.50).value),
            y_max_m=float(self.declare_parameter("grid_y_max_m", 2.50).value),
            inflation_radius_m=float(
                self.declare_parameter("grid_inflation_radius_m", 0.20).value
            ),
            scan_bin_count=int(
                self.declare_parameter("scan_bin_count", 360).value
            ),
            scan_range_min_m=float(
                self.declare_parameter("scan_range_min_m", 0.10).value
            ),
            scan_range_max_m=float(
                self.declare_parameter("scan_range_max_m", 6.00).value
            ),
        )
        self._proximity_config = proximity_config
        self._grid_config = grid_config
        self._perception_config = PerceptionConfig(
            x_min_m=grid_config.x_min_m,
            x_max_m=grid_config.x_max_m,
            y_min_m=grid_config.y_min_m,
            y_max_m=grid_config.y_max_m,
            z_min_m=proximity_config.z_min_m,
            z_max_m=float(
                self.declare_parameter("perception_z_max_m", 2.20).value
            ),
            ground_max_z_m=float(
                self.declare_parameter("ground_max_z_m", -0.20).value
            ),
            voxel_size_m=float(
                self.declare_parameter("voxel_size_m", 0.08).value
            ),
            cluster_tolerance_m=float(
                self.declare_parameter("cluster_tolerance_m", 0.30).value
            ),
            min_cluster_points=int(
                self.declare_parameter("minimum_cluster_points", 6).value
            ),
        )
        tracker_config = TrackerConfig(
            association_gate_m=float(
                self.declare_parameter("association_gate_m", 0.80).value
            ),
            track_timeout_s=float(
                self.declare_parameter("track_timeout_s", 0.80).value
            ),
            minimum_confirmed_hits=int(
                self.declare_parameter("minimum_confirmed_hits", 2).value
            ),
            moving_speed_mps=float(
                self.declare_parameter("moving_speed_mps", 0.20).value
            ),
            static_speed_mps=float(
                self.declare_parameter("static_speed_mps", 0.08).value
            ),
            moving_confirmation_count=int(
                self.declare_parameter("moving_confirmation_count", 2).value
            ),
            static_confirmation_count=int(
                self.declare_parameter("static_confirmation_count", 3).value
            ),
            maximum_unknown_dynamic_radius_m=float(
                self.declare_parameter(
                    "maximum_unknown_dynamic_radius_m",
                    0.80,
                ).value
            ),
        )
        risk_config = RiskConfig(
            prediction_horizon_s=float(
                self.declare_parameter("prediction_horizon_s", 3.0).value
            ),
            emergency_distance_m=float(
                self.declare_parameter(
                    "dynamic_emergency_distance_m",
                    0.85,
                ).value
            ),
            collision_distance_m=float(
                self.declare_parameter(
                    "dynamic_collision_distance_m",
                    1.00,
                ).value
            ),
            slowdown_distance_m=float(
                self.declare_parameter(
                    "dynamic_slowdown_distance_m",
                    1.50,
                ).value
            ),
            reaction_time_s=float(
                self.declare_parameter("dynamic_reaction_time_s", 0.35).value
            ),
            max_deceleration_mps2=maximum_deceleration,
            safety_margin_m=float(
                self.declare_parameter("dynamic_safety_margin_m", 0.40).value
            ),
        )
        self._engine = AvoidanceEngine(
            tracker_config=tracker_config,
            risk_config=risk_config,
        )

        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic",
                    "/avoidance/status",
                ).value
            ),
            10,
        )
        self._objects_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "objects_topic",
                    "/avoidance/objects",
                ).value
            ),
            10,
        )
        self._corridor_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "corridor_update_topic",
                    "/avoidance/corridor_update",
                ).value
            ),
            10,
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
        self._costmap_publisher = self.create_publisher(
            OccupancyGrid,
            str(
                self.declare_parameter(
                    "local_costmap_topic",
                    "/avoidance/local_costmap",
                ).value
            ),
            10,
        )

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._scan_publisher = self.create_publisher(
            LaserScan,
            str(
                self.declare_parameter(
                    "scan_topic",
                    "/avoidance/scan",
                ).value
            ),
            cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            cloud_qos,
        )
        self.create_subscription(
            Odometry,
            self._odometry_topic,
            self._odometry_callback,
            20,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_odometry: Odometry | None = None
        self._latest_odometry_error = ""
        self._latest_odometry_received_s = 0.0
        self._last_cloud_received_s = 0.0
        self._stop_required = True
        self._stop_reason = "waiting_for_perception"
        self.create_timer(1.0 / stop_frequency_hz, self._stop_cycle)
        self.get_logger().info(
            "Additive avoidance manager ready in dry-run mode: "
            f"cloud={self._cloud_topic}, odometry={self._odometry_topic}, "
            f"planning_radius={planning_radius:.2f}m, max_speed={maximum_speed:.2f}m/s"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _odometry_callback(self, message: Odometry) -> None:
        if (
            self._expected_odometry_frame
            and message.header.frame_id != self._expected_odometry_frame
        ):
            self._latest_odometry = None
            self._latest_odometry_error = (
                f"{message.header.frame_id}->{self._expected_odometry_frame}"
            )
            self._latest_odometry_received_s = self._now_s()
            return
        self._latest_odometry = message
        self._latest_odometry_error = ""
        self._latest_odometry_received_s = self._now_s()

    def _cloud_callback(self, message: PointCloud2) -> None:
        started_at = time.perf_counter()
        now_s = self._now_s()
        self._last_cloud_received_s = now_s
        if not self._processing_rate_gate.allow(now_s):
            return
        frame_id = message.header.frame_id
        if frame_id != self._expected_cloud_frame:
            self._fail_closed(
                "cloud_frame_mismatch",
                frame_id=frame_id,
            )
            return
        stamp_s = _stamp_to_seconds(message.header.stamp)
        cloud_age_s = now_s - stamp_s
        if stamp_s <= 0.0 or cloud_age_s < -0.05:
            self._fail_closed("invalid_cloud_timestamp", cloud_age_s=cloud_age_s)
            return
        if cloud_age_s > self._maximum_cloud_age_s:
            self._fail_closed("stale_cloud_timestamp", cloud_age_s=cloud_age_s)
            return

        points = tuple(
            (float(point[0]), float(point[1]), float(point[2]))
            for point in point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        )
        clearance = evaluate_local_clearance(
            points,
            self._proximity_config,
            self._grid_config,
        )
        self._scan_publisher.publish(
            _scan_message(message, clearance, self._grid_config, self._scan_time_s)
        )

        if self._latest_odometry_error:
            self._fail_closed(
                "odometry_frame_mismatch",
                detail=self._latest_odometry_error,
                cloud_age_s=cloud_age_s,
            )
            return
        if (
            self._latest_odometry is None
            or now_s - self._latest_odometry_received_s > self._odometry_timeout_s
        ):
            self._fail_closed("odometry_unavailable", cloud_age_s=cloud_age_s)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                frame_id,
                rclpy.time.Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._transform_timeout_s),
            )
        except TransformException as exc:
            self._fail_closed("cloud_transform_unavailable", detail=str(exc))
            return

        yaw = _yaw_from_quaternion(transform.transform.rotation)
        self._costmap_publisher.publish(
            _costmap_message(
                message,
                clearance.costmap,
                map_frame=self._map_frame,
                transform=transform,
                yaw=yaw,
            )
        )
        body_detections = cluster_points(points, self._perception_config)
        map_detections = tuple(
            _transform_detection(detection, transform, yaw)
            for detection in body_detections
        )
        velocity = self._latest_odometry.twist.twist.linear
        ego = EgoState(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            vx_mps=math.cos(yaw) * float(velocity.x)
            - math.sin(yaw) * float(velocity.y),
            vy_mps=math.sin(yaw) * float(velocity.x)
            + math.cos(yaw) * float(velocity.y),
        )
        try:
            decision = self._engine.update(
                map_detections,
                timestamp_s=stamp_s,
                ego=ego,
                proximity_stop=clearance.stop,
            )
        except ValueError as exc:
            self._fail_closed("tracker_timestamp_error", detail=str(exc))
            return
        self._stop_required = decision.stop_required
        self._stop_reason = decision.reason
        self._publish_decision(
            decision,
            frame_id=self._map_frame,
            stamp_s=stamp_s,
            cloud_age_s=cloud_age_s,
            processing_time_ms=(time.perf_counter() - started_at) * 1000.0,
        )

    def _stop_cycle(self) -> None:
        now_s = self._now_s()
        if (
            self._last_cloud_received_s <= 0.0
            or now_s - self._last_cloud_received_s > self._perception_timeout_s
        ):
            self._stop_required = True
            self._stop_reason = "perception_timeout"
        self._stop_publisher.publish(Bool(data=self._stop_required))

    def _fail_closed(self, reason: str, **fields: object) -> None:
        self._stop_required = True
        self._stop_reason = reason
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "mode": "SAFE_HOLD",
                        "stop_required": True,
                        "reason": reason,
                        **fields,
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _publish_decision(
        self,
        decision: AvoidanceDecision,
        *,
        frame_id: str,
        stamp_s: float,
        cloud_age_s: float,
        processing_time_ms: float,
    ) -> None:
        self._objects_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "frame_id": frame_id,
                        "stamp_s": stamp_s,
                        "tracks": [
                            {
                                "track_id": track.track_id,
                                "x": track.x,
                                "y": track.y,
                                "vx_mps": track.vx_mps,
                                "vy_mps": track.vy_mps,
                                "radius_m": track.radius_m,
                                "classification": track.classification,
                                "confidence": track.confidence,
                                "motion_state": track.motion_state,
                                "confirmed": track.confirmed,
                            }
                            for track in decision.tracks
                        ],
                    },
                    separators=(",", ":"),
                )
            )
        )
        self._corridor_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "mode": decision.mode,
                        "valid_until_s": self._now_s() + self._corridor_ttl_s,
                        "source": "competition_avoidance",
                        "reason": decision.reason,
                    },
                    separators=(",", ":"),
                )
            )
        )
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "mode": decision.mode,
                        "stop_required": decision.stop_required,
                        "reason": decision.reason,
                        "static_track_count": decision.static_track_count,
                        "dynamic_track_count": decision.dynamic_track_count,
                        "risk_level": decision.risk.level,
                        "risk_track_id": decision.risk.track_id,
                        "time_to_cpa_s": decision.risk.time_to_cpa_s,
                        "distance_at_cpa_m": decision.risk.distance_at_cpa_m,
                        "cloud_age_s": cloud_age_s,
                        "processing_time_ms": processing_time_ms,
                        "operation_mode": self._operation_mode,
                    },
                    separators=(",", ":"),
                )
            )
        )


def _costmap_message(
    cloud: PointCloud2,
    local_costmap,
    *,
    map_frame: str,
    transform,
    yaw: float,
) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.stamp = cloud.header.stamp
    message.header.frame_id = map_frame
    message.info.resolution = local_costmap.resolution_m
    message.info.width = local_costmap.width
    message.info.height = local_costmap.height
    origin_x, origin_y = transform_grid_origin(
        origin_x_m=local_costmap.origin_x_m,
        origin_y_m=local_costmap.origin_y_m,
        translation_x_m=float(transform.transform.translation.x),
        translation_y_m=float(transform.transform.translation.y),
        yaw_rad=yaw,
    )
    message.info.origin.position.x = origin_x
    message.info.origin.position.y = origin_y
    quaternion = yaw_quaternion(yaw)
    message.info.origin.orientation.x = quaternion[0]
    message.info.origin.orientation.y = quaternion[1]
    message.info.origin.orientation.z = quaternion[2]
    message.info.origin.orientation.w = quaternion[3]
    message.data = list(local_costmap.data)
    return message


def _scan_message(cloud, clearance, grid_config, scan_time_s: float) -> LaserScan:
    message = LaserScan()
    message.header = cloud.header
    message.angle_min = clearance.scan_angle_min_rad
    message.angle_increment = clearance.scan_angle_increment_rad
    message.angle_max = message.angle_min + message.angle_increment * (
        len(clearance.scan_ranges_m) - 1
    )
    message.scan_time = scan_time_s
    message.range_min = grid_config.scan_range_min_m
    message.range_max = grid_config.scan_range_max_m
    message.ranges = list(clearance.scan_ranges_m)
    return message


def _transform_detection(detection, transform, yaw: float) -> ObstacleDetection:
    translation = transform.transform.translation
    return ObstacleDetection(
        x=float(translation.x)
        + math.cos(yaw) * detection.x
        - math.sin(yaw) * detection.y,
        y=float(translation.y)
        + math.sin(yaw) * detection.x
        + math.cos(yaw) * detection.y,
        z=float(translation.z) + detection.z,
        length_m=detection.length_m,
        width_m=detection.width_m,
        height_m=detection.height_m,
        point_count=detection.point_count,
        classification=detection.classification,
        confidence=detection.confidence,
    )


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = AvoidanceManagerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
