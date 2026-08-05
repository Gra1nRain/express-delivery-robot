#!/usr/bin/env python3
"""Record a compact, passive Day5 control timeline without sensor frames."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_OUTPUT = "/tmp/day5_trip_trace.jsonl"


class Day5TripDiagnostics(Node):
    """Subscribe to planning/control/safety topics and emit only compact metrics."""

    def __init__(self, output_path: str, sample_rate_hz: float) -> None:
        super().__init__("day5_trip_diagnostics")
        if sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")

        self._output_path = Path(output_path)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self._output_path.open("w", encoding="utf-8", buffering=1)
        self._start_monotonic_s = time.monotonic()
        self._event_counts: dict[str, int] = {}
        self._latest: dict[str, Any] = {}
        self._received_at: dict[str, float] = {}
        self._transition_signatures: dict[str, str] = {}
        self._occupied_centers_body: tuple[tuple[float, float], ...] = ()
        self._costmap_resolution_m: float | None = None
        self._costmap_frame = ""
        self._last_path_signature: str | None = None
        self._last_path_start: tuple[float, float] | None = None
        self._last_path_end: tuple[float, float] | None = None

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        default_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            String,
            "/planning/local_replan_status",
            lambda message: self._json_status_callback(
                "local_replan_status", message, always_log=True
            ),
            default_qos,
        )
        self.create_subscription(
            Bool,
            "/planning/local_stop_request",
            lambda message: self._bool_callback("local_stop_request", message),
            default_qos,
        )
        self.create_subscription(
            NavPath,
            "/planning/local_trajectory",
            self._path_callback,
            default_qos,
        )
        self.create_subscription(
            String,
            "/control/status",
            lambda message: self._json_status_callback("control_status", message),
            default_qos,
        )
        self.create_subscription(
            TwistStamped,
            "/control/body_cmd",
            lambda message: self._twist_callback(
                "body_cmd", message.twist.linear.x, message.twist.angular.z
            ),
            default_qos,
        )
        self.create_subscription(
            Vector3Stamped,
            "/control/tracking_error",
            self._tracking_error_callback,
            default_qos,
        )
        self.create_subscription(
            Bool,
            "/control/state_valid",
            lambda message: self._bool_callback("control_state_valid", message),
            default_qos,
        )
        self.create_subscription(
            String,
            "/safety/event",
            lambda message: self._json_status_callback("safety_event", message),
            default_qos,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            lambda message: self._twist_callback(
                "cmd_vel_safe", message.linear.x, message.angular.z
            ),
            default_qos,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda message: self._twist_callback(
                "cmd_vel", message.linear.x, message.angular.z
            ),
            default_qos,
        )
        self.create_subscription(
            String,
            "/avoidance/proximity_status",
            lambda message: self._json_status_callback("proximity_status", message),
            default_qos,
        )
        self.create_subscription(
            Bool,
            "/avoidance/stop_request",
            lambda message: self._bool_callback("avoidance_stop_request", message),
            default_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/avoidance/local_costmap",
            self._costmap_callback,
            default_qos,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_callback, scan_qos)
        self.create_subscription(Odometry, "/odom", self._odom_callback, default_qos)
        self.create_timer(1.0 / sample_rate_hz, self._sample_callback)

        self._write(
            "session_start",
            {
                "output": str(self._output_path),
                "sample_rate_hz": sample_rate_hz,
                "raw_sensor_frames_saved": False,
            },
        )
        self.get_logger().info(
            f"Passive Day5 trace started: {self._output_path}; "
            "no publishers and no raw scan/costmap frames"
        )

    def close(self) -> None:
        if self._stream.closed:
            return
        self._write("session_end", {"event_counts": self._event_counts})
        self._stream.close()

    def _now_monotonic_s(self) -> float:
        return time.monotonic()

    def _write(self, event: str, payload: dict[str, Any]) -> None:
        self._event_counts[event] = self._event_counts.get(event, 0) + 1
        record = {
            "t_s": round(self._now_monotonic_s() - self._start_monotonic_s, 4),
            "ros_time_s": round(self.get_clock().now().nanoseconds / 1e9, 6),
            "event": event,
            **payload,
        }
        self._stream.write(json.dumps(record, separators=(",", ":")) + "\n")

    def _remember(self, key: str, value: Any) -> None:
        self._latest[key] = value
        self._received_at[key] = self._now_monotonic_s()

    def _json_status_callback(
        self,
        key: str,
        message: String,
        *,
        always_log: bool = False,
    ) -> None:
        try:
            payload: Any = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            payload = {"raw_text": str(message.data)[:300]}
        if not isinstance(payload, dict):
            payload = {"value": payload}
        self._remember(key, payload)

        signature_payload = _transition_fields(key, payload)
        signature = json.dumps(signature_payload, sort_keys=True, separators=(",", ":"))
        if always_log or self._transition_signatures.get(key) != signature:
            self._transition_signatures[key] = signature
            self._write(key, {"data": _compact_status(payload)})

    def _bool_callback(self, key: str, message: Bool) -> None:
        value = bool(message.data)
        self._remember(key, value)
        signature = str(value)
        if self._transition_signatures.get(key) != signature:
            self._transition_signatures[key] = signature
            self._write(key, {"value": value})

    def _twist_callback(self, key: str, linear_x: float, angular_z: float) -> None:
        self._remember(
            key,
            {
                "linear_x_mps": round(float(linear_x), 5),
                "angular_z_radps": round(float(angular_z), 5),
            },
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._remember(
            "odom",
            {
                "linear_x_mps": round(float(message.twist.twist.linear.x), 5),
                "angular_z_radps": round(float(message.twist.twist.angular.z), 5),
            },
        )

    def _tracking_error_callback(self, message: Vector3Stamped) -> None:
        self._remember(
            "tracking_error",
            {
                "lateral_error_m": round(float(message.vector.x), 5),
                "heading_error_rad": round(float(message.vector.y), 5),
                "target_index": int(round(float(message.vector.z))),
            },
        )

    def _scan_callback(self, message: LaserScan) -> None:
        valid_count = 0
        footprint_hit_count = 0
        minimum_range_m = math.inf
        minimum_footprint_clearance_m = math.inf
        angle = float(message.angle_min)
        for range_value in getattr(message, "ranges", ()):
            distance_m = float(range_value)
            if (
                math.isfinite(distance_m)
                and float(message.range_min) <= distance_m <= float(message.range_max)
            ):
                x_m = distance_m * math.cos(angle)
                y_m = distance_m * math.sin(angle)
                clearance_m = _signed_rectangle_clearance(
                    x_m,
                    y_m,
                    half_length_m=0.36,
                    half_width_m=0.25,
                )
                valid_count += 1
                minimum_range_m = min(minimum_range_m, distance_m)
                minimum_footprint_clearance_m = min(
                    minimum_footprint_clearance_m,
                    clearance_m,
                )
                if clearance_m <= 0.0:
                    footprint_hit_count += 1
            angle += float(message.angle_increment)
        self._remember(
            "scan",
            {
                "frame_id": message.header.frame_id,
                "valid_beam_count": valid_count,
                "minimum_range_m": _finite_or_none(minimum_range_m),
                "minimum_footprint_clearance_m": _finite_or_none(
                    minimum_footprint_clearance_m
                ),
                "footprint_hit_count": footprint_hit_count,
            },
        )

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        width = int(message.info.width)
        resolution_m = float(message.info.resolution)
        if width <= 0 or resolution_m <= 0.0:
            self._remember(
                "costmap",
                {"frame_id": message.header.frame_id, "valid": False},
            )
            return
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        occupied: list[tuple[float, float]] = []
        for index, occupancy in enumerate(getattr(message, "data", ())):
            if int(occupancy) < 50:
                continue
            row, column = divmod(index, width)
            occupied.append(
                (
                    origin_x + (column + 0.5) * resolution_m,
                    origin_y + (row + 0.5) * resolution_m,
                )
            )
        self._occupied_centers_body = tuple(occupied)
        self._costmap_resolution_m = resolution_m
        self._costmap_frame = message.header.frame_id
        self._remember(
            "costmap",
            {
                "frame_id": message.header.frame_id,
                "occupied_cell_count": len(occupied),
                "width": width,
                "height": int(message.info.height),
                "resolution_m": resolution_m,
            },
        )

    def _path_callback(self, message: NavPath) -> None:
        map_points = tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        )
        quantized = ";".join(f"{x:.3f},{y:.3f}" for x, y in map_points)
        signature = hashlib.sha1(quantized.encode("ascii")).hexdigest()[:12]
        start = map_points[0] if map_points else None
        end = map_points[-1] if map_points else None
        summary: dict[str, Any] = {
            "frame_id": message.header.frame_id,
            "point_count": len(map_points),
            "path_hash": signature,
            "same_hash": signature == self._last_path_signature,
            "start_jump_m": _point_distance(start, self._last_path_start),
            "endpoint_jump_m": _point_distance(end, self._last_path_end),
        }
        body_points = self._transform_points_to_costmap(
            message.header.frame_id,
            map_points,
        )
        if body_points is not None and self._occupied_centers_body:
            minimum_clearance_m = min(
                math.hypot(path_x - obstacle_x, path_y - obstacle_y)
                for path_x, path_y in body_points
                for obstacle_x, obstacle_y in self._occupied_centers_body
            )
            summary["min_path_to_inflated_cell_center_m"] = round(
                minimum_clearance_m, 4
            )
            if self._costmap_resolution_m is not None:
                summary["path_intersects_inflated_grid"] = (
                    minimum_clearance_m <= self._costmap_resolution_m * 0.75
                )
        self._last_path_signature = signature
        self._last_path_start = start
        self._last_path_end = end
        self._remember("local_trajectory", summary)
        self._write("local_trajectory", summary)

    def _transform_points_to_costmap(
        self,
        source_frame: str,
        points: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[float, float], ...] | None:
        if not points or not self._costmap_frame:
            return None
        if source_frame == self._costmap_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._costmap_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0
            * (
                float(rotation.w) * float(rotation.z)
                + float(rotation.x) * float(rotation.y)
            ),
            1.0 - 2.0 * (float(rotation.y) ** 2 + float(rotation.z) ** 2),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return tuple(
            (
                float(translation.x) + cosine * x_m - sine * y_m,
                float(translation.y) + sine * x_m + cosine * y_m,
            )
            for x_m, y_m in points
        )

    def _sample_callback(self) -> None:
        now_s = self._now_monotonic_s()
        ages = {
            key: round(now_s - received_s, 4)
            for key, received_s in self._received_at.items()
        }
        self._write(
            "sample",
            {
                "body_cmd": self._latest.get("body_cmd"),
                "cmd_vel_safe": self._latest.get("cmd_vel_safe"),
                "cmd_vel": self._latest.get("cmd_vel"),
                "odom": self._latest.get("odom"),
                "tracking_error": self._latest.get("tracking_error"),
                "control_state_valid": self._latest.get("control_state_valid"),
                "control": _sample_status(self._latest.get("control_status")),
                "local_replan": _sample_status(
                    self._latest.get("local_replan_status")
                ),
                "safety": _sample_status(self._latest.get("safety_event")),
                "proximity": _sample_status(self._latest.get("proximity_status")),
                "local_stop": self._latest.get("local_stop_request"),
                "avoidance_stop": self._latest.get("avoidance_stop_request"),
                "scan": self._latest.get("scan"),
                "costmap": self._latest.get("costmap"),
                "local_trajectory": self._latest.get("local_trajectory"),
                "ages_s": ages,
            },
        )


def _transition_fields(key: str, payload: dict[str, Any]) -> dict[str, Any]:
    common = {
        name: payload.get(name)
        for name in (
            "status",
            "state",
            "stop_requested",
            "state_reasons",
            "reasons",
            "local_plan_error",
            "local_plan_update_mode",
        )
        if name in payload
    }
    return common or {"key": key, "value": payload}


def _compact_status(payload: dict[str, Any]) -> dict[str, Any]:
    excluded = {"trajectory", "path", "points", "ranges", "data"}
    return {
        key: value
        for key, value in payload.items()
        if key not in excluded and _json_size(value) <= 1000
    }


def _sample_status(payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    names = (
        "status",
        "state",
        "stop_requested",
        "target_index",
        "state_reasons",
        "reasons",
        "pose_delay_s",
        "local_plan_age_s",
        "local_plan_error",
        "local_plan_update_mode",
        "local_plan_reuse_count",
        "local_plan_replace_count",
        "planning_time_ms",
        "obstacle_count",
    )
    return {name: payload.get(name) for name in names if name in payload}


def _signed_rectangle_clearance(
    x_m: float,
    y_m: float,
    *,
    half_length_m: float,
    half_width_m: float,
) -> float:
    dx = abs(x_m) - half_length_m
    dy = abs(y_m) - half_width_m
    if dx <= 0.0 and dy <= 0.0:
        return -min(-dx, -dy)
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def _finite_or_none(value: float) -> float | None:
    return round(value, 4) if math.isfinite(value) else None


def _point_distance(
    first: tuple[float, float] | None,
    second: tuple[float, float] | None,
) -> float | None:
    if first is None or second is None:
        return None
    return round(math.hypot(first[0] - second[0], first[1] - second[1]), 4)


def _json_size(value: Any) -> int:
    try:
        return len(json.dumps(value, separators=(",", ":")))
    except TypeError:
        return 1001


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--sample-rate-hz", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = Day5TripDiagnostics(args.output, args.sample_rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"Trace saved to {args.output}")


if __name__ == "__main__":
    main()
