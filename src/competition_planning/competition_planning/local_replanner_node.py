#!/usr/bin/env python3
"""ROS adapter from the live body costmap to a reference-aware local path."""

from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
)
from competition_planning.occupancy_grid_planner import (
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


class LocalReplannerNode(Node):
    def __init__(self) -> None:
        super().__init__("local_replanner")
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._base_frame = str(self.declare_parameter("base_frame", "body").value)
        trajectory_file = str(self.declare_parameter("trajectory_file", "").value)
        map_file = str(self.declare_parameter("map_file", "").value)
        if not trajectory_file or not map_file:
            raise ValueError("trajectory_file and map_file parameters are required")
        self._reference_path = _load_reference_path(trajectory_file, self._map_frame)
        static_map = OccupancyGridMap.from_yaml(map_file)
        self._config = LocalReplanConfig(
            lookahead_distance_m=float(
                self.declare_parameter("lookahead_distance_m", 5.0).value
            ),
            inflation_radius_m=float(
                self.declare_parameter("inflation_radius_m", 0.45).value
            ),
            search_padding_m=float(
                self.declare_parameter("search_padding_m", 3.0).value
            ),
            sample_spacing_m=float(
                self.declare_parameter("sample_spacing_m", 0.10).value
            ),
            min_turning_radius_m=float(
                self.declare_parameter("min_turning_radius_m", 0.81).value
            ),
            step_length_m=float(
                self.declare_parameter("step_length_m", 0.20).value
            ),
            curvature_bins=int(self.declare_parameter("curvature_bins", 9).value),
            heading_bins=int(self.declare_parameter("heading_bins", 72).value),
            goal_position_tolerance_m=float(
                self.declare_parameter("goal_position_tolerance_m", 0.25).value
            ),
            goal_heading_tolerance_rad=math.radians(
                float(
                    self.declare_parameter("goal_heading_tolerance_deg", 15.0).value
                )
            ),
            reference_deviation_weight=float(
                self.declare_parameter("reference_deviation_weight", 2.0).value
            ),
            max_expansions=int(
                self.declare_parameter("max_expansions", 250_000).value
            ),
            reference_search_window_points=int(
                self.declare_parameter("reference_search_window_points", 120).value
            ),
        )
        self._planner = LocalTrajectoryPlanner(static_map, self._config)
        self._max_costmap_age_s = float(
            self.declare_parameter("max_costmap_age_s", 0.75).value
        )
        if self._max_costmap_age_s <= 0.0:
            raise ValueError("max_costmap_age_s must be positive")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._obstacle_points_map: tuple[tuple[float, float], ...] | None = None
        self._costmap_stamp_s = 0.0
        self._costmap_frame = ""
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
        self.create_subscription(
            OccupancyGrid,
            str(
                self.declare_parameter(
                    "costmap_topic",
                    "/avoidance/local_costmap",
                ).value
            ),
            self._costmap_callback,
            1,
        )
        frequency_hz = float(self.declare_parameter("frequency_hz", 2.0).value)
        if frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        self.create_timer(1.0 / frequency_hz, self._planning_cycle)
        self.get_logger().info(
            "Local replanner ready: "
            f"lookahead={self._config.lookahead_distance_m:.2f}m, "
            f"inflation={self._config.inflation_radius_m:.2f}m, "
            f"reference_weight={self._config.reference_deviation_weight:.2f}"
        )

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        if not message.header.frame_id:
            self._publish_status("INVALID_COSTMAP_FRAME")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                message.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self._publish_status("COSTMAP_TF_UNAVAILABLE", detail=str(exc))
            return

        local_yaw = _yaw_from_quaternion(message.info.origin.orientation)
        transform_yaw = _yaw_from_quaternion(transform.transform.rotation)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        resolution = float(message.info.resolution)
        width = int(message.info.width)
        points: list[tuple[float, float]] = []
        for index, value in enumerate(message.data):
            if value < 100:
                continue
            row, col = divmod(index, width)
            grid_x = (col + 0.5) * resolution
            grid_y = (row + 0.5) * resolution
            frame_x = origin_x + math.cos(local_yaw) * grid_x - math.sin(local_yaw) * grid_y
            frame_y = origin_y + math.sin(local_yaw) * grid_x + math.cos(local_yaw) * grid_y
            map_x = (
                float(transform.transform.translation.x)
                + math.cos(transform_yaw) * frame_x
                - math.sin(transform_yaw) * frame_y
            )
            map_y = (
                float(transform.transform.translation.y)
                + math.sin(transform_yaw) * frame_x
                + math.cos(transform_yaw) * frame_y
            )
            points.append((map_x, map_y))
        self._obstacle_points_map = tuple(points)
        self._costmap_stamp_s = _stamp_to_seconds(message.header.stamp)
        self._costmap_frame = message.header.frame_id

    def _planning_cycle(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        if self._obstacle_points_map is None or self._costmap_stamp_s <= 0.0:
            self._publish_status("WAITING_FOR_COSTMAP")
            return
        age_s = now_s - self._costmap_stamp_s
        if age_s > self._max_costmap_age_s:
            self._publish_status("COSTMAP_STALE", costmap_age_s=age_s)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            current_pose = PathPoint(
                x=float(transform.transform.translation.x),
                y=float(transform.transform.translation.y),
                yaw=_yaw_from_quaternion(transform.transform.rotation),
            )
            started_at = time.perf_counter()
            result = self._planner.plan(
                reference_path=self._reference_path,
                current_pose=current_pose,
                dynamic_obstacle_points=self._obstacle_points_map,
                previous_reference_index=self._previous_reference_index,
            )
            planning_time_ms = (time.perf_counter() - started_at) * 1000.0
        except (TransformException, GridPlanningError, ValueError) as exc:
            self._publish_status("PLAN_FAILED", detail=str(exc), costmap_age_s=age_s)
            return

        self._previous_reference_index = result.reference_start_index
        publish_now = self.get_clock().now()
        self._path_publisher.publish(
            _path_message(result.path, self._map_frame, publish_now.to_msg())
        )
        self._publish_status(
            result.status,
            costmap_age_s=age_s,
            planning_time_ms=planning_time_ms,
            reference_start_index=result.reference_start_index,
            rejoin_index=result.rejoin_index,
            dynamic_obstacle_count=result.dynamic_obstacle_count,
            path_point_count=len(result.path),
            planning_grid_cell_count=result.planning_grid_cell_count,
        )

    def _publish_status(self, status: str, **fields: object) -> None:
        payload = {
            "status": status,
            "costmap_frame": self._costmap_frame,
            **fields,
        }
        self._status_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


def _load_reference_path(path: str, expected_frame: str) -> tuple[PathPoint, ...]:
    with FilePath(path).open("r", encoding="utf-8") as stream:
        artifact = yaml.safe_load(stream)
    if not isinstance(artifact, dict) or not artifact.get("ok", False):
        raise ValueError(f"trajectory_file is not a successful artifact: {path}")
    frame_id = str(artifact.get("frame_id", "map"))
    if frame_id != expected_frame:
        raise ValueError(
            f"trajectory frame_id={frame_id} does not match map_frame={expected_frame}"
        )
    points = tuple(
        PathPoint(
            x=float(point["x"]),
            y=float(point["y"]),
            yaw=float(point["yaw"]),
            ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
        )
        for point in artifact.get("points", [])
    )
    if len(points) < 2:
        raise ValueError("trajectory artifact requires at least two points")
    return points


def _path_message(
    points: tuple[PathPoint, ...],
    frame_id: str,
    stamp,
) -> Path:
    message = Path()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    for point in points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(point.yaw * 0.5)
        pose.pose.orientation.w = math.cos(point.yaw * 0.5)
        message.poses.append(pose)
    return message


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z) + float(quaternion.x) * float(quaternion.y)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2),
    )


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = LocalReplannerNode()
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
