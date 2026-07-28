#!/usr/bin/env python3
"""ROS adapter from an inflated 2D costmap to a DWA local reference path."""

from __future__ import annotations

import json
import math
import time

from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from competition_planning.dwa_local_planner import (
    DWAConfig,
    DWALocalPlanner,
    DWAPlanningError,
    DWAVelocity,
    occupied_grid_cell_centers,
)
from competition_planning.local_replanner_node import (
    _load_reference_path,
    _path_message,
    _yaw_from_quaternion,
)
from competition_planning.semantic_planner import PathPoint


class DWALocalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("dwa_local_planner")
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body").value)
        trajectory_file = str(self.declare_parameter("trajectory_file", "").value)
        if not trajectory_file:
            raise ValueError("trajectory_file parameter is required")
        self._reference_path = _load_reference_path(
            trajectory_file,
            self._map_frame,
        )

        frequency_hz = float(self.declare_parameter("frequency_hz", 1.0).value)
        if frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        self._planner = DWALocalPlanner(
            DWAConfig(
                min_speed_mps=float(
                    self.declare_parameter("min_speed_mps", 0.05).value
                ),
                max_speed_mps=float(
                    self.declare_parameter("max_speed_mps", 0.20).value
                ),
                max_acceleration_mps2=float(
                    self.declare_parameter("max_acceleration_mps2", 0.20).value
                ),
                max_deceleration_mps2=float(
                    self.declare_parameter("max_deceleration_mps2", 0.30).value
                ),
                max_yaw_rate_radps=float(
                    self.declare_parameter("max_yaw_rate_radps", 0.30).value
                ),
                max_yaw_acceleration_radps2=float(
                    self.declare_parameter(
                        "max_yaw_acceleration_radps2",
                        0.60,
                    ).value
                ),
                min_turning_radius_m=float(
                    self.declare_parameter("min_turning_radius_m", 0.81).value
                ),
                control_interval_s=1.0 / frequency_hz,
                prediction_horizon_s=float(
                    self.declare_parameter("prediction_horizon_s", 6.0).value
                ),
                simulation_step_s=float(
                    self.declare_parameter("simulation_step_s", 0.20).value
                ),
                speed_sample_count=int(
                    self.declare_parameter("speed_sample_count", 4).value
                ),
                yaw_rate_sample_count=int(
                    self.declare_parameter("yaw_rate_sample_count", 11).value
                ),
                obstacle_clearance_m=float(
                    self.declare_parameter("obstacle_clearance_m", 0.55).value
                ),
                reference_lookahead_m=float(
                    self.declare_parameter("reference_lookahead_m", 1.50).value
                ),
                max_reference_deviation_m=float(
                    self.declare_parameter(
                        "max_reference_deviation_m",
                        1.20,
                    ).value
                ),
                reference_search_window_points=int(
                    self.declare_parameter(
                        "reference_search_window_points",
                        160,
                    ).value
                ),
                progress_weight=float(
                    self.declare_parameter("progress_weight", 6.0).value
                ),
                path_distance_weight=float(
                    self.declare_parameter("path_distance_weight", 2.0).value
                ),
                goal_distance_weight=float(
                    self.declare_parameter("goal_distance_weight", 1.0).value
                ),
                heading_weight=float(
                    self.declare_parameter("heading_weight", 0.3).value
                ),
                clearance_weight=float(
                    self.declare_parameter("clearance_weight", 0.8).value
                ),
                speed_weight=float(
                    self.declare_parameter("speed_weight", 1.0).value
                ),
                yaw_rate_weight=float(
                    self.declare_parameter("yaw_rate_weight", 0.15).value
                ),
            )
        )
        obstacle_source = str(
            self.declare_parameter("obstacle_source", "costmap").value
        ).lower()
        if obstacle_source != "costmap":
            raise ValueError("obstacle_source must be 'costmap'")
        self._expected_obstacle_frame = str(
            self.declare_parameter("expected_obstacle_frame", "body").value
        )
        self._max_obstacle_age_s = float(
            self.declare_parameter("max_obstacle_age_s", 0.50).value
        )
        self._max_odom_age_s = float(
            self.declare_parameter("max_odom_age_s", 0.50).value
        )
        self._obstacle_x_min_m = float(
            self.declare_parameter("obstacle_x_min_m", 0.05).value
        )
        self._obstacle_x_max_m = float(
            self.declare_parameter("obstacle_x_max_m", 4.0).value
        )
        self._obstacle_y_half_width_m = float(
            self.declare_parameter("obstacle_y_half_width_m", 2.5).value
        )
        self._costmap_occupancy_threshold = int(
            self.declare_parameter("costmap_occupancy_threshold", 50).value
        )
        self._max_obstacle_points = int(
            self.declare_parameter("max_obstacle_points", 2000).value
        )
        if self._max_obstacle_age_s <= 0.0 or self._max_odom_age_s <= 0.0:
            raise ValueError("sensor age limits must be positive")
        if (
            self._obstacle_x_max_m <= self._obstacle_x_min_m
            or self._obstacle_y_half_width_m <= 0.0
        ):
            raise ValueError("obstacle bounds are invalid")
        if not 0 <= self._costmap_occupancy_threshold <= 100:
            raise ValueError("costmap_occupancy_threshold must be in [0, 100]")
        if self._max_obstacle_points < 1:
            raise ValueError("max_obstacle_points must be positive")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._obstacle_points: tuple[tuple[float, float], ...] | None = None
        self._obstacle_frame = ""
        self._obstacle_received_s = 0.0
        self._obstacle_header_stamp_s = 0.0
        self._velocity: DWAVelocity | None = None
        self._odom_received_s = 0.0
        self._previous_reference_index = 0

        self._path_publisher = self.create_publisher(
            Path,
            str(
                self.declare_parameter(
                    "local_trajectory_topic",
                    "/planning/local_trajectory",
                ).value
            ),
            1,
        )
        self._stop_publisher = self.create_publisher(
            Bool,
            str(
                self.declare_parameter(
                    "local_stop_request_topic",
                    "/planning/local_stop_request",
                ).value
            ),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(
                self.declare_parameter(
                    "status_topic",
                    "/planning/local_replan_status",
                ).value
            ),
            10,
        )
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        costmap_topic = str(
            self.declare_parameter(
                "costmap_topic",
                "/avoidance/local_costmap",
            ).value
        )
        self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self._costmap_callback,
            costmap_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.declare_parameter("odom_topic", "/odom").value),
            self._odom_callback,
            20,
        )
        self.create_timer(1.0 / frequency_hz, self._planning_cycle)
        self._stop_publisher.publish(Bool(data=True))
        self.get_logger().info(
            "DWA local planner ready: "
            f"inflated_costmap={costmap_topic}, reference={trajectory_file}, "
            f"frequency={frequency_hz:.2f}Hz"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        received_s = self._now_s()
        if (
            self._expected_obstacle_frame
            and message.header.frame_id != self._expected_obstacle_frame
        ):
            self._obstacle_points = None
            self._publish_stop(
                "COSTMAP_FRAME_MISMATCH",
                detail=(
                    f"{message.header.frame_id}->{self._expected_obstacle_frame}"
                ),
            )
            return
        origin_yaw = _yaw_from_quaternion(message.info.origin.orientation)
        if abs(origin_yaw) > 1e-6:
            self._obstacle_points = None
            self._publish_stop(
                "COSTMAP_ORIGIN_ROTATED",
                detail=f"yaw={origin_yaw:.6f}",
            )
            return
        self._obstacle_points = occupied_grid_cell_centers(
            message.data,
            width=int(message.info.width),
            height=int(message.info.height),
            resolution_m=float(message.info.resolution),
            origin_x_m=float(message.info.origin.position.x),
            origin_y_m=float(message.info.origin.position.y),
            occupancy_threshold=self._costmap_occupancy_threshold,
            x_min_m=self._obstacle_x_min_m,
            x_max_m=self._obstacle_x_max_m,
            y_half_width_m=self._obstacle_y_half_width_m,
            max_points=self._max_obstacle_points,
        )
        self._obstacle_frame = message.header.frame_id
        self._obstacle_received_s = received_s
        self._obstacle_header_stamp_s = _stamp_to_seconds(message.header.stamp)

    def _odom_callback(self, message: Odometry) -> None:
        self._velocity = DWAVelocity(
            linear_mps=float(message.twist.twist.linear.x),
            yaw_rate_radps=float(message.twist.twist.angular.z),
        )
        self._odom_received_s = self._now_s()

    def _planning_cycle(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        if self._obstacle_points is None or self._obstacle_received_s <= 0.0:
            self._publish_stop("WAITING_FOR_COSTMAP")
            return
        obstacle_age_s = now_s - self._obstacle_received_s
        if obstacle_age_s > self._max_obstacle_age_s:
            self._publish_stop(
                "COSTMAP_STALE",
                obstacle_age_s=obstacle_age_s,
            )
            return
        if self._velocity is None or self._odom_received_s <= 0.0:
            self._publish_stop(
                "WAITING_FOR_ODOMETRY",
                obstacle_age_s=obstacle_age_s,
            )
            return
        odom_age_s = now_s - self._odom_received_s
        if odom_age_s > self._max_odom_age_s:
            self._publish_stop(
                "ODOMETRY_STALE",
                obstacle_age_s=obstacle_age_s,
                odom_age_s=odom_age_s,
            )
            return

        try:
            map_from_base = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            base_from_obstacle = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._obstacle_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            current_pose = PathPoint(
                x=float(map_from_base.transform.translation.x),
                y=float(map_from_base.transform.translation.y),
                yaw=_yaw_from_quaternion(map_from_base.transform.rotation),
            )
            obstacle_points = self._obstacles_in_base(base_from_obstacle)
            started_at = time.perf_counter()
            result = self._planner.plan(
                reference_path=self._reference_path,
                current_pose=current_pose,
                current_velocity=self._velocity,
                obstacle_points_body=obstacle_points,
                previous_reference_index=self._previous_reference_index,
            )
            planning_time_ms = (time.perf_counter() - started_at) * 1000.0
        except (TransformException, DWAPlanningError, ValueError) as exc:
            self._publish_stop(
                "DWA_NO_FEASIBLE_PATH",
                detail=str(exc),
                obstacle_age_s=obstacle_age_s,
                odom_age_s=odom_age_s,
            )
            return

        self._previous_reference_index = result.reference_start_index
        publish_now = self.get_clock().now()
        self._path_publisher.publish(
            _path_message(result.path, self._map_frame, publish_now.to_msg())
        )
        self._stop_publisher.publish(Bool(data=False))
        self._publish_status(
            result.status,
            stop_requested=False,
            obstacle_age_s=obstacle_age_s,
            obstacle_header_age_s=(
                now_s - self._obstacle_header_stamp_s
                if self._obstacle_header_stamp_s > 0.0
                else None
            ),
            odom_age_s=odom_age_s,
            planning_time_ms=planning_time_ms,
            obstacle_count=result.obstacle_count,
            path_point_count=len(result.path),
            reference_start_index=result.reference_start_index,
            reference_target_index=result.reference_target_index,
            selected_speed_mps=result.selected_velocity.linear_mps,
            selected_yaw_rate_radps=result.selected_velocity.yaw_rate_radps,
            minimum_clearance_m=result.minimum_clearance_m,
        )

    def _obstacles_in_base(self, transform) -> tuple[tuple[float, float], ...]:
        translation = transform.transform.translation
        yaw = _yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        points: list[tuple[float, float]] = []
        for obstacle_x, obstacle_y in self._obstacle_points or ():
            base_x = (
                float(translation.x)
                + cos_yaw * obstacle_x
                - sin_yaw * obstacle_y
            )
            base_y = (
                float(translation.y)
                + sin_yaw * obstacle_x
                + cos_yaw * obstacle_y
            )
            if not (
                self._obstacle_x_min_m <= base_x <= self._obstacle_x_max_m
                and abs(base_y) <= self._obstacle_y_half_width_m
            ):
                continue
            points.append((base_x, base_y))
        return tuple(points)

    def _publish_stop(self, status: str, **fields: object) -> None:
        self._stop_publisher.publish(Bool(data=True))
        self._publish_status(status, stop_requested=True, **fields)

    def _publish_status(self, status: str, **fields: object) -> None:
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {"status": status, **fields},
                    separators=(",", ":"),
                )
            )
        )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = DWALocalPlannerNode()
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
