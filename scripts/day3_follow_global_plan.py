#!/usr/bin/env python3
"""Follow the Day 3 debug global plan with a conservative Twist controller.

This is a supervised field-test helper.  It reads the offline global-plan YAML,
concatenates the planned route steps, tracks the path from map->base_link TF,
and publishes low-speed /cmd_vel commands.  It is intentionally separate from
the unfinished competition_control package so the test path is explicit.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import sys
import time
from typing import Any

import yaml


DEFAULT_STEPS = (
    "go_traffic_light_1",
    "random_obstacle_1",
    "cone_lane_change_1",
    "return_to_pickup_area",
    "cone_lane_change_2",
    "finish_park",
)


class FrameValidationError(RuntimeError):
    """Raised when the configured TF chain is present but unsafe for 2D tracking."""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", required=True, help="debug_global_plan*.yaml file")
    parser.add_argument("--step", action="append", help="Step id to follow; default is full route")
    parser.add_argument("--check-only", action="store_true", help="Parse the path and exit")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--body-frame", dest="base_frame", help=argparse.SUPPRESS)
    parser.add_argument("--max-pose-z-m", type=float, default=0.30)
    parser.add_argument("--tf-timeout-s", type=float, default=0.10)
    parser.add_argument("--max-tf-dropout-s", type=float, default=0.75)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--lookahead-m", type=float, default=0.45)
    parser.add_argument("--max-speed-mps", type=float, default=0.18)
    parser.add_argument("--min-speed-mps", type=float, default=0.05)
    parser.add_argument("--max-angular-rps", type=float, default=0.45)
    parser.add_argument("--heading-gain", type=float, default=1.3)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.22)
    parser.add_argument("--start-tolerance-m", type=float, default=0.50)
    parser.add_argument("--max-path-error-m", type=float, default=0.85)
    args = parser.parse_args()

    steps = tuple(args.step) if args.step else DEFAULT_STEPS
    path = load_path(Path(args.plan), steps)
    print(
        f"loaded path: points={len(path)} length_m={path_length(path):.3f} "
        f"steps={','.join(steps)}"
    )
    print(f"first=({path[0][0]:.3f},{path[0][1]:.3f}) last=({path[-1][0]:.3f},{path[-1][1]:.3f})")
    if args.check_only:
        return 0

    return run_tracker(args, path)


def load_path(plan_path: Path, steps: tuple[str, ...]) -> list[tuple[float, float]]:
    data = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data.get("ok", False):
        raise SystemExit(f"plan is not ok: {plan_path}")
    by_step = {str(item.get("step_id")): item for item in data.get("plans", [])}
    result: list[tuple[float, float]] = []
    for step in steps:
        item = by_step.get(step)
        if item is None:
            raise SystemExit(f"missing plan step: {step}")
        points = item.get("points", [])
        if len(points) < 2:
            raise SystemExit(f"plan step has too few points: {step}")
        for raw in points:
            point = (float(raw["x"]), float(raw["y"]))
            if result and math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) < 1e-4:
                continue
            result.append(point)
    if len(result) < 2:
        raise SystemExit("concatenated path has too few points")
    return result


def path_length(path: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(path, path[1:])
    )


def run_tracker(args: argparse.Namespace, path: list[tuple[float, float]]) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = Node("day3_debug_global_plan_tracker")
    publisher = node.create_publisher(Twist, args.cmd_topic, 10)
    tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
    TransformListener(tf_buffer, node)
    stop_requested = {"value": False}

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def publish_zero(repeat: int = 6) -> None:
        zero = Twist()
        for _ in range(repeat):
            publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)

    period = 1.0 / max(args.rate_hz, 1.0)
    current_index = 0
    last_log = 0.0
    tf_lost_since: float | None = None

    try:
        wait_for_subscriber(node, publisher, args.cmd_topic)
        pose = wait_for_pose(
            node,
            tf_buffer,
            args.map_frame,
            args.base_frame,
            args.max_pose_z_m,
            args.tf_timeout_s,
        )
        start_error = math.hypot(pose[0] - path[0][0], pose[1] - path[0][1])
        if start_error > args.start_tolerance_m:
            print(
                f"ABORT start_error={start_error:.3f}m exceeds "
                f"{args.start_tolerance_m:.3f}m"
            )
            publish_zero()
            return 3
        print(
            f"START frame={args.map_frame}->{args.base_frame} "
            f"pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) "
            f"start_error={start_error:.3f}m"
        )

        while rclpy.ok() and not stop_requested["value"]:
            loop_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            try:
                pose = lookup_pose(
                    tf_buffer,
                    args.map_frame,
                    args.base_frame,
                    args.max_pose_z_m,
                    args.tf_timeout_s,
                )
            except (TransformException, FrameValidationError) as exc:
                publish_zero(1)
                now = time.monotonic()
                if tf_lost_since is None:
                    tf_lost_since = now
                    print(f"WARN lost_tf: {exc}", flush=True)
                if now - tf_lost_since > args.max_tf_dropout_s:
                    print(
                        f"ABORT lost_tf_for={now - tf_lost_since:.3f}s: {exc}",
                        flush=True,
                    )
                    publish_zero()
                    return 4
                time.sleep(period)
                continue
            tf_lost_since = None

            nearest_index, nearest_error = find_nearest_index(path, pose, current_index)
            current_index = nearest_index
            if nearest_error > args.max_path_error_m:
                print(
                    f"ABORT path_error={nearest_error:.3f}m exceeds "
                    f"{args.max_path_error_m:.3f}m at index={current_index}"
                )
                publish_zero()
                return 5

            goal_distance = math.hypot(pose[0] - path[-1][0], pose[1] - path[-1][1])
            if current_index >= len(path) - 2 and goal_distance <= args.goal_tolerance_m:
                print(f"GOAL reached goal_distance={goal_distance:.3f}m")
                publish_zero(12)
                return 0

            target_index = find_lookahead_index(path, pose, current_index, args.lookahead_m)
            target = path[target_index]
            heading_error = target_heading_error(pose, target)
            command = Twist()
            if abs(heading_error) > 1.0:
                command.linear.x = 0.0
            else:
                scale = max(0.25, math.cos(abs(heading_error)))
                command.linear.x = max(args.min_speed_mps, args.max_speed_mps * scale)
            command.angular.z = clamp(
                args.heading_gain * heading_error,
                -args.max_angular_rps,
                args.max_angular_rps,
            )
            publisher.publish(command)

            now = time.monotonic()
            if now - last_log >= 1.0:
                print(
                    "TRACK "
                    f"idx={current_index}/{len(path)-1} target={target_index} "
                    f"pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) "
                    f"err={nearest_error:.2f} goal={goal_distance:.2f} "
                    f"cmd=({command.linear.x:.2f},{command.angular.z:.2f})",
                    flush=True,
                )
                last_log = now

            elapsed = time.monotonic() - loop_started
            time.sleep(max(0.0, period - elapsed))

        print("STOP requested")
        publish_zero(12)
        return 130
    finally:
        publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def wait_for_subscriber(node: Any, publisher: Any, topic: str) -> None:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return
        print(f"waiting for subscriber on {topic}", flush=True)
        time.sleep(0.2)
    raise SystemExit(f"no subscriber on {topic}")


def wait_for_pose(
    node: Any,
    tf_buffer: Any,
    map_frame: str,
    base_frame: str,
    max_pose_z_m: float,
    tf_timeout_s: float,
) -> tuple[float, float, float]:
    deadline = time.monotonic() + 5.0
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        rclpy = sys.modules["rclpy"]
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            return lookup_pose(tf_buffer, map_frame, base_frame, max_pose_z_m, tf_timeout_s)
        except Exception as exc:  # tf2_ros exception type is imported inside run_tracker.
            last_error = exc
            time.sleep(0.05)
    raise SystemExit(f"no valid TF {map_frame}->{base_frame}: {last_error}")


def lookup_pose(
    tf_buffer: Any,
    map_frame: str,
    base_frame: str,
    max_pose_z_m: float,
    tf_timeout_s: float,
) -> tuple[float, float, float]:
    import rclpy
    from rclpy.duration import Duration

    transform = tf_buffer.lookup_transform(
        map_frame,
        base_frame,
        rclpy.time.Time(),
        timeout=Duration(seconds=max(0.0, tf_timeout_s)),
    )
    translation = transform.transform.translation
    z = float(translation.z)
    if abs(z) > max_pose_z_m:
        raise FrameValidationError(
            f"TF {map_frame}->{base_frame} z={z:.3f}m exceeds {max_pose_z_m:.3f}m; "
            "check that the tracker is using the chassis frame, not FAST-LIO body"
        )
    rotation = transform.transform.rotation
    yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
    return float(translation.x), float(translation.y), yaw


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def find_nearest_index(
    path: list[tuple[float, float]],
    pose: tuple[float, float, float],
    start_index: int,
) -> tuple[int, float]:
    end = min(len(path), start_index + 80)
    best_index = start_index
    best_distance = float("inf")
    for index in range(start_index, end):
        distance = math.hypot(path[index][0] - pose[0], path[index][1] - pose[1])
        if distance < best_distance:
            best_index = index
            best_distance = distance
    return best_index, best_distance


def find_lookahead_index(
    path: list[tuple[float, float]],
    pose: tuple[float, float, float],
    start_index: int,
    lookahead_m: float,
) -> int:
    for index in range(start_index, len(path)):
        if math.hypot(path[index][0] - pose[0], path[index][1] - pose[1]) >= lookahead_m:
            return index
    return len(path) - 1


def target_heading_error(
    pose: tuple[float, float, float],
    target: tuple[float, float],
) -> float:
    heading = math.atan2(target[1] - pose[1], target[0] - pose[0])
    return normalize_angle(heading - pose[2])


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


if __name__ == "__main__":
    sys.exit(main())
