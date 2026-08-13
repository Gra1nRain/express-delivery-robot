#!/usr/bin/env python3
"""Run Day5 field validation through a supervised /cmd_vel_safe relay.

The launch file is started with ``command_output_topic:=/cmd_vel_safe`` so the
controller and safety node can settle without moving the chassis.  This helper
publishes the initial pose, waits for a live local plan, then relays
``/cmd_vel_safe`` to the real ``/cmd_vel`` until the global route finishes or a
safety condition asks it to stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String

from competition_control.ranger_twist_adapter import (
    RangerMiniV3Geometry,
    adapt_yaw_rate_for_ranger_driver,
)
from day5_run_policy import (
    control_status_requires_stop,
    load_route_metadata,
    local_planner_status_is_ready,
    resolve_watchdog_timeout_s,
    scan_stop_reason,
)

try:
    from ranger_msgs.msg import SystemState
except Exception:  # pragma: no cover - local unit environments may not have it.
    SystemState = None


def _shell(command: str, timeout_s: float = 4.0) -> str:
    try:
        return subprocess.run(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
        ).stdout.strip()
    except Exception as exc:  # pragma: no cover - best-effort shutdown evidence.
        return f"shell_error: {exc}"


def _pgrep(pattern: str) -> list[int]:
    try:
        completed = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return []
    pids: list[int] = []
    own_pid = os.getpid()
    for line in completed.stdout.splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        if pid != own_pid:
            pids.append(pid)
    return sorted(set(pids))


def _child_pids(parent_pid: int) -> list[int]:
    try:
        completed = subprocess.run(
            ["pgrep", "-P", str(parent_pid)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return []
    children: list[int] = []
    for line in completed.stdout.splitlines():
        try:
            children.append(int(line.strip()))
        except ValueError:
            continue
    return sorted(set(children))


def _descendant_pids(root_pid: int) -> list[int]:
    descendants: list[int] = []
    stack = [root_pid]
    while stack:
        current = stack.pop()
        children = _child_pids(current)
        descendants.extend(children)
        stack.extend(children)
    return sorted(set(descendants))


def _signal_pids(pids: list[int], sig: signal.Signals) -> str:
    signaled: list[int | str] = []
    for pid in sorted(set(pids), reverse=True):
        try:
            os.kill(pid, sig)
            signaled.append(pid)
        except ProcessLookupError:
            continue
        except PermissionError as exc:
            signaled.append(f"{pid}:permission_error:{exc}")
    return f"{sig.name}:{signaled}"

def _parse_json_string(value: str | None) -> dict[str, Any] | None:
    if not value:
        return None
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError:
        return None
    return parsed if isinstance(parsed, dict) else None


class RelayMonitor(Node):
    def __init__(self, args: argparse.Namespace, route_point_count: int | None) -> None:
        super().__init__("day5_full_route_relay_monitor")
        self._args = args
        self._route_point_count = route_point_count
        self.data: dict[str, Any] = {}
        self.wall_stamp: dict[str, float] = {}
        self.header_stamp: dict[str, float] = {}
        self.latest_safe_cmd = Twist()
        self.relay_active = False

        default_qos = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.route_enable_pub = (
            self.create_publisher(Bool, "/mission/route_enable", 10)
            if args.enable_segmented_route
            else None
        )

        self.create_subscription(Twist, "/cmd_vel_safe", self._safe_cmd_cb, default_qos)
        self.create_subscription(
            TwistStamped, "/control/body_cmd", self._body_cmd_cb, default_qos
        )
        self.create_subscription(
            PointCloud2, "/cloud_registered_body", self._cloud_cb, best_effort_qos
        )
        self.create_subscription(
            OccupancyGrid, "/avoidance/local_costmap", self._costmap_cb, default_qos
        )
        self.create_subscription(LaserScan, "/avoidance/scan", self._scan_cb, best_effort_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, best_effort_qos)
        self.create_subscription(
            String, "/avoidance/proximity_status", lambda msg: self._string_cb("proximity", msg), default_qos
        )
        self.create_subscription(
            Bool, "/avoidance/stop_request", lambda msg: self._bool_cb("stop_request", msg), default_qos
        )
        self.create_subscription(
            String, "/planning/local_replan_status", lambda msg: self._string_cb("local_status", msg), default_qos
        )
        self.create_subscription(
            String, "/control/status", lambda msg: self._string_cb("control_status", msg), default_qos
        )
        self.create_subscription(
            Vector3Stamped, "/control/tracking_error", self._tracking_error_cb, default_qos
        )
        self.create_subscription(
            NavPath, "/control/executed_path", self._executed_path_cb, default_qos
        )
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self._system_cb, default_qos)

    def _mark(self, topic: str, msg: Any | None = None) -> None:
        self.wall_stamp[topic] = time.time()
        if msg is not None:
            try:
                self.header_stamp[topic] = (
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                )
            except Exception:
                pass

    def age(self, topic: str) -> float | None:
        if topic not in self.wall_stamp:
            return None
        return max(0.0, time.time() - self.wall_stamp[topic])

    def header_age(self, topic: str) -> float | None:
        if topic not in self.header_stamp:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, now_s - self.header_stamp[topic])

    def _safe_cmd_cb(self, msg: Twist) -> None:
        self._mark("cmd_vel_safe")
        self.latest_safe_cmd = msg
        self.data["safe_cmd_x"] = float(msg.linear.x)
        self.data["safe_cmd_z"] = float(msg.angular.z)

    def _body_cmd_cb(self, msg: TwistStamped) -> None:
        self._mark("body_cmd", msg)
        self.data["body_cmd_x"] = float(msg.twist.linear.x)
        self.data["body_cmd_z"] = float(msg.twist.angular.z)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._mark("cloud", msg)
        self.data["cloud_points_width"] = int(msg.width)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self._mark("costmap", msg)
        self.data["costmap_width"] = int(msg.info.width)
        self.data["costmap_height"] = int(msg.info.height)
        self.data["costmap_resolution"] = float(msg.info.resolution)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._mark("scan", msg)
        finite = [value for value in msg.ranges if math.isfinite(value)]
        self.data["scan_min"] = float(min(finite)) if finite else None
        self.data["scan_count"] = len(msg.ranges)

    def _odom_cb(self, msg: Odometry) -> None:
        self._mark("odom", msg)
        self.data["odom_x"] = float(msg.pose.pose.position.x)
        self.data["odom_y"] = float(msg.pose.pose.position.y)
        self.data["odom_vx"] = float(msg.twist.twist.linear.x)
        self.data["odom_wz"] = float(msg.twist.twist.angular.z)

    def _string_cb(self, key: str, msg: String) -> None:
        self._mark(key)
        self.data[key] = str(msg.data)
        parsed = _parse_json_string(msg.data)
        if key == "control_status" and parsed is not None:
            self.data["control_status_value"] = parsed.get("status")
            self.data["control_target_index"] = parsed.get("target_index")
            self.data["control_state_reasons"] = parsed.get("state_reasons")
            self.data["pose_delay_s"] = parsed.get("pose_delay_s")
            self.data["local_plan_age_s"] = parsed.get("local_plan_age_s")
            self.data["active_checkpoint_index"] = parsed.get(
                "active_checkpoint_index"
            )
            self.data["active_checkpoint_ref"] = parsed.get(
                "active_checkpoint_ref"
            )
            self.data["precision_phase"] = parsed.get("precision_phase")
        elif key == "local_status" and parsed is not None:
            self.data["local_status_value"] = parsed.get("status")
            self.data["local_reference_start_index"] = parsed.get("reference_start_index")
            self.data["local_rejoin_index"] = parsed.get("rejoin_index")
            self.data["local_dynamic_obstacle_count"] = parsed.get("dynamic_obstacle_count")
            self.data["local_path_point_count"] = parsed.get("path_point_count")
        elif key == "proximity" and parsed is not None:
            self.data["proximity_stop"] = parsed.get("stop")
            self.data["proximity_reason"] = parsed.get("reason")
            self.data["nearest_obstacle_distance_m"] = parsed.get(
                "nearest_obstacle_distance_m"
            )

    def _bool_cb(self, key: str, msg: Bool) -> None:
        self._mark(key)
        self.data[key] = bool(msg.data)

    def _tracking_error_cb(self, msg: Vector3Stamped) -> None:
        self._mark("tracking", msg)
        self.data["lateral_error_m"] = float(msg.vector.x)
        self.data["heading_error_rad"] = float(msg.vector.y)
        self.data["heading_error_deg"] = float(math.degrees(msg.vector.y))
        self.data["tracking_target_index"] = int(round(float(msg.vector.z)))

    def _executed_path_cb(self, msg: NavPath) -> None:
        self._mark("executed_path", msg)
        self.data["exec_points"] = len(msg.poses)
        if msg.poses:
            pose = msg.poses[-1].pose.position
            self.data["exec_last_x"] = float(pose.x)
            self.data["exec_last_y"] = float(pose.y)

    def _system_cb(self, msg: Any) -> None:
        self._mark("system_state")
        for field in ("error_code", "base_state", "vehicle_state", "control_mode"):
            if hasattr(msg, field):
                value = getattr(msg, field)
                try:
                    value = int(value)
                except Exception:
                    value = str(value)
                self.data[f"system_{field}"] = value

    def relay_or_zero(self) -> None:
        if self.relay_active and self.age("cmd_vel_safe") is not None and self.age("cmd_vel_safe") < 0.4:
            command = self._relay_command(self.latest_safe_cmd)
            self.cmd_pub.publish(command)
            self.data["relay_cmd_x"] = float(command.linear.x)
            self.data["relay_cmd_z"] = float(command.angular.z)
        else:
            self.cmd_pub.publish(Twist())
            self.data["relay_cmd_x"] = 0.0
            self.data["relay_cmd_z"] = 0.0

    def _relay_command(self, safe_command: Twist) -> Twist:
        command = Twist()
        command.linear.x = safe_command.linear.x
        command.linear.y = safe_command.linear.y
        command.linear.z = safe_command.linear.z
        command.angular.x = safe_command.angular.x
        command.angular.y = safe_command.angular.y
        command.angular.z = safe_command.angular.z
        if self._args.adapt_ranger_twist:
            command.angular.z = adapt_yaw_rate_for_ranger_driver(
                linear_x_mps=float(safe_command.linear.x),
                desired_yaw_rate_radps=float(safe_command.angular.z),
                geometry=RangerMiniV3Geometry(
                    wheelbase_m=self._args.ranger_wheelbase_m,
                    track_width_m=self._args.ranger_track_width_m,
                    driver_min_turn_radius_m=self._args.ranger_driver_min_turn_radius_m,
                ),
            )
        return command

    def publish_zero_for(self, seconds: float) -> None:
        self.relay_active = False
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def publish_route_enable(self, enabled: bool) -> None:
        if self.route_enable_pub is None:
            return
        self.relay_active = False
        message = Bool(data=enabled)
        for _ in range(6):
            self.route_enable_pub.publish(message)
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.08)
        self.data["segmented_route_enabled"] = enabled

    def publish_initial_pose(self) -> None:
        if (
            self._args.initial_x is None
            or self._args.initial_y is None
            or self._args.initial_yaw is None
        ):
            raise ValueError("initial pose values are required when publishing initialpose")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self._args.initial_x
        msg.pose.pose.position.y = self._args.initial_y
        msg.pose.pose.orientation.z = math.sin(self._args.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._args.initial_yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.04
        covariance[7] = 0.04
        covariance[35] = 0.01
        msg.pose.covariance = covariance
        for _ in range(6):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.08)

    def prepose_ready(self) -> bool:
        initialpose_ready = (
            bool(self._args.skip_initialpose)
            or self.initial_pose_pub.get_subscription_count() >= 1
        )
        return (
            initialpose_ready
            and self.age("cloud") is not None
            and self.age("cloud") < 0.8
            and self.age("odom") is not None
            and self.age("odom") < 0.8
            and self.age("cmd_vel_safe") is not None
            and self.age("cmd_vel_safe") < 0.8
            and not bool(self.data.get("stop_request", False))
        )

    def local_ready(self) -> bool:
        trajectory_ready = (
            bool(self._args.global_tracking_mode)
            or local_planner_status_is_ready(self.data.get("local_status_value"))
        )
        return (
            trajectory_ready
            and self.data.get("control_status_value") in ("TRACKING", "GOAL_REACHED")
            and self.age("costmap") is not None
            and self.age("costmap") < 0.9
            and self.age("scan") is not None
            and self.age("scan") < 0.9
            and self.age("cmd_vel_safe") is not None
            and self.age("cmd_vel_safe") < 0.4
            and not bool(self.data.get("stop_request", False))
            and not bool(self.data.get("proximity_stop", False))
        )

    def snapshot(self, phase: str, elapsed_s: float | None = None) -> dict[str, Any]:
        snapshot = {
            "t_wall": time.time(),
            "phase": phase,
            "elapsed_s": None if elapsed_s is None else round(float(elapsed_s), 3),
            "init_subscribers": self.initial_pose_pub.get_subscription_count(),
            "relay_active": self.relay_active,
            "rx_age": {
                key: round(value, 3)
                for key in sorted(self.wall_stamp)
                if (value := self.age(key)) is not None
            },
            "header_age": {
                key: round(value, 3)
                for key in sorted(self.header_stamp)
                if (value := self.header_age(key)) is not None
            },
        }
        snapshot.update(self.data)
        return snapshot


def _sustained(
    bad_since: dict[str, float],
    key: str,
    bad: bool,
    now_s: float,
    duration_s: float,
) -> bool:
    if bad:
        bad_since.setdefault(key, now_s)
        return now_s - bad_since[key] >= duration_s
    bad_since.pop(key, None)
    return False


def _route_complete(
    node: RelayMonitor,
    route_point_count: int | None,
    finish_xy: tuple[float, float] | None,
) -> bool:
    status = node.data.get("control_status_value")
    start_index = node.data.get("local_reference_start_index")
    rejoin_index = node.data.get("local_rejoin_index")
    near_final_rejoin = (
        route_point_count is not None
        and isinstance(rejoin_index, int)
        and rejoin_index >= route_point_count - 2
    )
    near_final_start = (
        route_point_count is not None
        and isinstance(start_index, int)
        and start_index >= route_point_count - 2
    )
    near_finish_pose = False
    if finish_xy is not None and "exec_last_x" in node.data and "exec_last_y" in node.data:
        near_finish_pose = (
            math.hypot(
                float(node.data["exec_last_x"]) - finish_xy[0],
                float(node.data["exec_last_y"]) - finish_xy[1],
            )
            <= 0.55
        )
    return bool(
        (status == "GOAL_REACHED" and (near_final_rejoin or near_finish_pose))
        or near_final_start
    )


def _write_status(path: Path, snapshot: dict[str, Any], stop_reason: str | None = None) -> None:
    status = dict(snapshot)
    if stop_reason is not None:
        status["stop_reason"] = stop_reason
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(status, ensure_ascii=False, separators=(",", ":")) + "\n")
    tmp.replace(path)


def _safe_shutdown(node: RelayMonitor, launch_pid: int) -> list[str]:
    logs: list[str] = []
    day5_node_patterns = [
        "/competition_control/lib/competition_control/mppi_control_node",
        "/competition_safety/lib/competition_safety/safety_node",
        "/competition_planning/lib/competition_planning/local_replanner_node",
        "/competition_safety/lib/competition_safety/proximity_stop_node",
        "/competition_localization/lib/competition_localization/fastlio_anchor_node",
        "/fast_lio/lib/fast_lio/fastlio_mapping",
        "/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node",
        "/ranger_base/lib/ranger_base/ranger_base_node",
        "__node:=day5_map_server",
        "__node:=day5_map_lifecycle_manager",
    ]
    node.publish_zero_for(1.2)
    launch_tree = [launch_pid] + _descendant_pids(launch_pid)
    logs.append(f"launch_tree_before={launch_tree}")
    logs.append(_signal_pids(_pgrep(day5_node_patterns[0]), signal.SIGINT))
    node.publish_zero_for(0.8)
    logs.append(_signal_pids(_pgrep(day5_node_patterns[1]), signal.SIGINT))
    node.publish_zero_for(0.8)
    logs.append(_signal_pids(launch_tree, signal.SIGINT))
    time.sleep(1.5)
    remaining = [launch_pid] + _descendant_pids(launch_pid)
    for pattern in day5_node_patterns:
        remaining.extend(_pgrep(pattern))
    logs.append(_signal_pids(remaining, signal.SIGTERM))
    time.sleep(0.8)
    still_running: list[int] = []
    for pattern in day5_node_patterns:
        still_running.extend(_pgrep(pattern))
    logs.append(f"day5_nodes_after={sorted(set(still_running))}")
    node.publish_zero_for(0.5)
    return logs


def _hold_after_stop(
    node: RelayMonitor,
    *,
    seconds: float,
    status_path: Path,
    stop_reason: str,
) -> list[str]:
    if seconds <= 0.0:
        return []
    hold_log = [f"post_stop_hold_begin_s={seconds:.1f}"]
    node.relay_active = False
    hold_start_s = time.time()
    while time.time() - hold_start_s < seconds:
        node.relay_or_zero()
        rclpy.spin_once(node, timeout_sec=0.05)
        _write_status(
            status_path,
            node.snapshot("post_stop_hold", time.time() - hold_start_s),
            stop_reason,
        )
        time.sleep(0.05)
    node.publish_zero_for(0.5)
    hold_log.append("post_stop_hold_end")
    return hold_log


def run(args: argparse.Namespace) -> int:
    route_metadata = load_route_metadata(Path(args.route_file))
    route_point_count = route_metadata.point_count
    finish_xy = route_metadata.finish_xy
    watchdog_timeout_s = resolve_watchdog_timeout_s(
        args.watchdog_timeout_s,
        route_metadata.duration_s,
        duration_scale=args.watchdog_duration_scale,
        margin_s=args.watchdog_margin_s,
    )
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = log_dir / f"{args.label}.jsonl"
    summary_path = log_dir / f"{args.label}_summary.txt"
    status_path = log_dir / f"{args.label}_status.json"

    rclpy.init()
    node = RelayMonitor(args, route_point_count)
    stop_reason = "not_started"
    shutdown_log: list[str] = []
    bad_since: dict[str, float] = {}
    final_snapshot: dict[str, Any] | None = None
    samples = 0
    max_safe_cmd = 0.0
    max_body_cmd = 0.0
    max_relay_cmd = 0.0
    max_odom_vx = 0.0
    max_abs_lateral_error = 0.0
    max_abs_heading_error = 0.0
    min_scan: float | None = None
    first_odom: tuple[float, float] | None = None
    last_odom: tuple[float, float] | None = None

    try:
        with jsonl_path.open("w", encoding="utf-8") as jsonl:
            jsonl.write(
                json.dumps(
                    {
                        "event": "script_start",
                        "label": args.label,
                        "launch_pid": args.launch_pid,
                        "initial_pose": None
                        if args.skip_initialpose
                        else [args.initial_x, args.initial_y, args.initial_yaw],
                        "skip_initialpose": bool(args.skip_initialpose),
                        "enable_segmented_route": bool(args.enable_segmented_route),
                        "route_file": args.route_file,
                        "route_point_count": route_point_count,
                        "finish_xy": finish_xy,
                        "planned_duration_s": route_metadata.duration_s,
                        "watchdog_timeout_s": watchdog_timeout_s,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )

            gate_start = time.time()
            while time.time() - gate_start < args.prepose_timeout_s:
                node.relay_active = False
                node.relay_or_zero()
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.prepose_ready():
                    break
                _write_status(status_path, node.snapshot("prepose_gate", time.time() - gate_start))
            else:
                stop_reason = "prepose_gate_timeout"
                jsonl.write(json.dumps({"event": "stop", "reason": stop_reason}, ensure_ascii=False) + "\n")
                return 2

            if args.skip_initialpose:
                jsonl.write(
                    json.dumps(
                        {
                            "event": "initialpose_skipped",
                            "reason": "using_existing_map_tf",
                            "snapshot": node.snapshot("initialpose_skipped"),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
            else:
                node.publish_initial_pose()
                jsonl.write(
                    json.dumps(
                        {
                            "event": "initialpose_published",
                            "snapshot": node.snapshot("initialpose"),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )

            if args.enable_segmented_route:
                node.publish_route_enable(True)
                jsonl.write(
                    json.dumps(
                        {
                            "event": "segmented_route_enabled",
                            "snapshot": node.snapshot("segmented_route_enabled"),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )

            ready_start = time.time()
            ready_count = 0
            while time.time() - ready_start < args.local_ready_timeout_s:
                node.relay_active = False
                node.relay_or_zero()
                rclpy.spin_once(node, timeout_sec=0.05)
                ready_count = ready_count + 1 if node.local_ready() else 0
                snapshot = node.snapshot("local_ready_gate", time.time() - ready_start)
                _write_status(status_path, snapshot)
                if ready_count >= 3:
                    break
            else:
                stop_reason = "local_ready_timeout_no_motion"
                jsonl.write(json.dumps({"event": "stop", "reason": stop_reason}, ensure_ascii=False) + "\n")
                return 3

            node.relay_active = True
            run_start = time.time()
            last_status_write_s = 0.0
            last_progress_s = run_start
            last_progress_checkpoint: int | None = None
            last_progress_xy: tuple[float, float] | None = None
            jsonl.write(json.dumps({"event": "relay_start", "snapshot": node.snapshot("relay_start", 0.0)}, ensure_ascii=False) + "\n")

            while True:
                rclpy.spin_once(node, timeout_sec=0.03)
                node.relay_or_zero()
                now_s = time.time()
                elapsed_s = now_s - run_start
                checkpoint_index = node.data.get("active_checkpoint_index")
                if (
                    isinstance(checkpoint_index, int)
                    and checkpoint_index != last_progress_checkpoint
                ):
                    last_progress_checkpoint = checkpoint_index
                    last_progress_s = now_s
                if "odom_x" in node.data and "odom_y" in node.data:
                    current_progress_xy = (
                        float(node.data["odom_x"]),
                        float(node.data["odom_y"]),
                    )
                    if last_progress_xy is None:
                        last_progress_xy = current_progress_xy
                        last_progress_s = now_s
                    elif (
                        math.dist(current_progress_xy, last_progress_xy)
                        >= args.progress_distance_m
                    ):
                        last_progress_xy = current_progress_xy
                        last_progress_s = now_s
                node.data["route_progress_age_s"] = max(
                    0.0, now_s - last_progress_s
                )
                snapshot = node.snapshot("run", elapsed_s)
                jsonl.write(json.dumps(snapshot, ensure_ascii=False) + "\n")
                samples += 1

                max_safe_cmd = max(max_safe_cmd, abs(float(node.data.get("safe_cmd_x", 0.0))))
                max_body_cmd = max(max_body_cmd, abs(float(node.data.get("body_cmd_x", 0.0))))
                max_relay_cmd = max(max_relay_cmd, abs(float(node.data.get("relay_cmd_x", 0.0))))
                max_odom_vx = max(max_odom_vx, abs(float(node.data.get("odom_vx", 0.0))))
                max_abs_lateral_error = max(
                    max_abs_lateral_error,
                    abs(float(node.data.get("lateral_error_m", 0.0))),
                )
                max_abs_heading_error = max(
                    max_abs_heading_error,
                    abs(float(node.data.get("heading_error_deg", 0.0))),
                )
                if node.data.get("scan_min") is not None:
                    scan_min = float(node.data["scan_min"])
                    min_scan = scan_min if min_scan is None else min(min_scan, scan_min)
                if "odom_x" in node.data and "odom_y" in node.data:
                    current_odom = (
                        float(node.data["odom_x"]),
                        float(node.data["odom_y"]),
                    )
                    first_odom = first_odom or current_odom
                    last_odom = current_odom

                if elapsed_s - last_status_write_s >= 1.0:
                    last_status_write_s = elapsed_s
                    _write_status(status_path, snapshot)
                    jsonl.flush()

                local_status = str(node.data.get("local_status_value", ""))
                control_status = str(node.data.get("control_status_value", ""))
                control_reasons = node.data.get("control_state_reasons") or []
                if _route_complete(node, route_point_count, finish_xy):
                    stop_reason = "route_complete"
                    break
                proximity_blocked = bool(
                    node.data.get("stop_request", False)
                ) or bool(node.data.get("proximity_stop", False))
                if _sustained(
                    bad_since,
                    "proximity_stop",
                    proximity_blocked,
                    now_s,
                    args.sustained_error_s,
                ):
                    stop_reason = "proximity_stop_request"
                    break
                scan_reason = scan_stop_reason(
                    node.data.get("scan_min"),
                    args.scan_stop_m,
                )
                if scan_reason is not None:
                    stop_reason = scan_reason
                    break
                if abs(float(node.data.get("safe_cmd_x", 0.0))) > args.max_command_mps:
                    stop_reason = "safe_command_over_limit"
                    break
                if abs(float(node.data.get("relay_cmd_x", 0.0))) > args.max_command_mps:
                    stop_reason = "relay_command_over_limit"
                    break
                if abs(float(node.data.get("odom_vx", 0.0))) > args.max_odom_mps:
                    stop_reason = "odom_velocity_over_limit"
                    break
                cmd_vel_safe_stale = (
                    node.age("cmd_vel_safe") is None
                    or node.age("cmd_vel_safe") > 0.6
                )
                if _sustained(
                    bad_since,
                    "cmd_vel_safe_stale",
                    cmd_vel_safe_stale,
                    now_s,
                    args.sustained_error_s,
                ):
                    stop_reason = "cmd_vel_safe_stale"
                    break
                local_bad = any(
                    token in local_status
                    for token in ("UNAVAILABLE", "STALE", "FAILED", "ERROR")
                )
                if elapsed_s > 6.0 and _sustained(
                    bad_since, "local_bad", local_bad, now_s, args.sustained_error_s
                ):
                    stop_reason = f"local_status_sustained_{node.data.get('local_status')}"
                    break
                control_bad = control_status_requires_stop(
                    control_status,
                    control_reasons,
                )
                if elapsed_s > 6.0 and _sustained(
                    bad_since, "control_bad", control_bad, now_s, args.sustained_error_s
                ):
                    stop_reason = f"control_status_sustained_{node.data.get('control_status')}"
                    break
                if elapsed_s > 6.0 and abs(float(node.data.get("lateral_error_m", 0.0))) > args.max_lateral_error_m:
                    stop_reason = "tracking_lateral_error_over_limit"
                    break
                if (
                    node.data["route_progress_age_s"]
                    >= args.no_progress_timeout_s
                ):
                    stop_reason = "no_route_progress_timeout"
                    break
                if (
                    watchdog_timeout_s is not None
                    and elapsed_s >= watchdog_timeout_s
                ):
                    stop_reason = "watchdog_timeout_route_not_complete"
                    break

            jsonl.write(
                json.dumps(
                    {"event": "stop_begin", "reason": stop_reason, "snapshot": node.snapshot("stop_begin", time.time() - run_start)},
                    ensure_ascii=False,
                )
                + "\n"
            )
            hold_log = _hold_after_stop(
                node,
                seconds=args.post_stop_hold_s,
                status_path=status_path,
                stop_reason=stop_reason,
            )
            for item in hold_log:
                jsonl.write(
                    json.dumps({"event": "post_stop_hold", "detail": item}, ensure_ascii=False)
                    + "\n"
                )
            shutdown_log = _safe_shutdown(node, args.launch_pid)
    finally:
        try:
            node.publish_route_enable(False)
        except Exception:
            pass
        try:
            final_snapshot = node.snapshot("final")
        except Exception as exc:
            final_snapshot = {"phase": "final_snapshot_failed", "error": str(exc)}
        try:
            rclpy.shutdown()
        except Exception:
            pass

    odom_dx = None
    odom_dy = None
    if first_odom is not None and last_odom is not None:
        odom_dx = last_odom[0] - first_odom[0]
        odom_dy = last_odom[1] - first_odom[1]
    summary_lines = [
        f"label={args.label}",
        f"stop_reason={stop_reason}",
        f"route_point_count={route_point_count}",
        f"planned_duration_s={route_metadata.duration_s}",
        "watchdog_timeout_s=disabled"
        if watchdog_timeout_s is None
        else f"watchdog_timeout_s={watchdog_timeout_s:.3f}",
        f"no_progress_timeout_s={args.no_progress_timeout_s:.3f}",
        f"progress_distance_m={args.progress_distance_m:.3f}",
        f"finish_xy={finish_xy}",
        f"samples={samples}",
        "initialpose=skipped_existing_map_tf"
        if args.skip_initialpose
        else f"initialpose=x={args.initial_x:.3f}, y={args.initial_y:.3f}, yaw={args.initial_yaw:.3f}",
        "relay=/cmd_vel_safe -> /cmd_vel",
        f"max_abs_body_cmd_x_mps={max_body_cmd:.4f}",
        f"max_abs_safe_cmd_x_mps={max_safe_cmd:.4f}",
        f"max_abs_relay_cmd_x_mps={max_relay_cmd:.4f}",
        f"max_abs_odom_vx_mps={max_odom_vx:.4f}",
        f"odom_delta_x_m={None if odom_dx is None else round(odom_dx, 4)}",
        f"odom_delta_y_m={None if odom_dy is None else round(odom_dy, 4)}",
        f"max_abs_lateral_error_m={max_abs_lateral_error:.4f}",
        f"max_abs_heading_error_deg={max_abs_heading_error:.3f}",
        f"min_scan_m={None if min_scan is None else round(min_scan, 4)}",
        f"last_local_status={node.data.get('local_status')}",
        f"last_control_status={node.data.get('control_status')}",
        f"last_proximity_status={node.data.get('proximity')}",
        f"last_stop_request={node.data.get('stop_request')}",
        f"last_exec_points={node.data.get('exec_points')}",
        f"last_exec_pose=({node.data.get('exec_last_x')}, {node.data.get('exec_last_y')})",
        f"jsonl={jsonl_path}",
        f"status={status_path}",
        f"shutdown_log={shutdown_log}",
    ]
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    _write_status(
        summary_path.with_name(f"{args.label}_final_status.json"),
        final_snapshot or {"phase": "final_snapshot_missing"},
        stop_reason,
    )
    return 0 if stop_reason in ("route_complete", "watchdog_timeout_route_not_complete") else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--launch-pid", type=int, required=True)
    parser.add_argument("--initial-x", type=float)
    parser.add_argument("--initial-y", type=float)
    parser.add_argument("--initial-yaw", type=float)
    parser.add_argument(
        "--skip-initialpose",
        action="store_true",
        help="Use an existing map TF from RViz/manual initialpose instead of publishing one.",
    )
    parser.add_argument("--route-file", required=True)
    parser.add_argument(
        "--global-tracking-mode",
        action="store_true",
        help="Do not require /planning/local_replan_status before enabling relay.",
    )
    parser.add_argument(
        "--enable-segmented-route",
        action="store_true",
        help=(
            "Arm /mission/route_enable while the chassis relay is still held at "
            "zero, then disarm it on exit."
        ),
    )
    parser.add_argument("--log-dir", default="/home/agilex/competition_ws/log")
    parser.add_argument("--prepose-timeout-s", type=float, default=18.0)
    parser.add_argument("--local-ready-timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--watchdog-timeout-s",
        type=float,
        default=None,
        help=(
            "Explicit wall-clock watchdog timeout; 0 disables it. By default "
            "use trajectory duration * watchdog-duration-scale + "
            "watchdog-margin-s."
        ),
    )
    parser.add_argument("--watchdog-duration-scale", type=float, default=2.5)
    parser.add_argument("--watchdog-margin-s", type=float, default=60.0)
    parser.add_argument(
        "--no-progress-timeout-s",
        type=float,
        default=120.0,
        help=(
            "Stop if neither the mission checkpoint nor odometry position makes "
            "meaningful progress for this duration."
        ),
    )
    parser.add_argument(
        "--progress-distance-m",
        type=float,
        default=0.05,
        help="Odometry displacement counted as route progress.",
    )
    parser.add_argument("--sustained-error-s", type=float, default=4.0)
    parser.add_argument(
        "--scan-stop-m",
        type=float,
        default=0.0,
        help="Raw-scan stop threshold; 0 disables distance-based termination.",
    )
    parser.add_argument("--max-command-mps", type=float, default=0.23)
    parser.add_argument("--max-odom-mps", type=float, default=0.28)
    parser.add_argument("--max-lateral-error-m", type=float, default=0.45)
    parser.add_argument(
        "--adapt-ranger-twist",
        action="store_true",
        help="Adapt body yaw-rate to the Ranger driver's dual-Ackermann Twist semantics.",
    )
    parser.add_argument("--ranger-wheelbase-m", type=float, default=0.494)
    parser.add_argument("--ranger-track-width-m", type=float, default=0.364)
    parser.add_argument(
        "--ranger-driver-min-turn-radius-m",
        type=float,
        default=0.47644,
    )
    parser.add_argument(
        "--post-stop-hold-s",
        type=float,
        default=0.0,
        help=(
            "After a stop condition, keep publishing zero /cmd_vel for this many "
            "seconds before shutting down the launch tree so RViz costmap/scan "
            "displays remain inspectable."
        ),
    )
    args = parser.parse_args()
    if args.no_progress_timeout_s <= 0.0:
        parser.error("--no-progress-timeout-s must be positive")
    if args.progress_distance_m <= 0.0:
        parser.error("--progress-distance-m must be positive")
    if not args.skip_initialpose and (
        args.initial_x is None or args.initial_y is None or args.initial_yaw is None
    ):
        parser.error(
            "--initial-x, --initial-y, and --initial-yaw are required unless "
            "--skip-initialpose is set"
        )
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
