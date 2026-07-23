#!/usr/bin/env python3
"""Evaluate a Day5 field-run JSONL log against motion-control acceptance limits."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable, Sequence

import yaml


DEFAULT_REQUIRED_BAG_TOPICS = (
    "/tf",
    "/tf_static",
    "/odom",
    "/control/body_cmd",
    "/control/tracking_error",
    "/cmd_vel_safe",
)
REQUIRED_JSONL_FIELDS = (
    "body_cmd_x",
    "safe_cmd_x",
    "relay_cmd_x",
    "odom_vx",
    "lateral_error_m",
    "heading_error_deg",
    "control_status_value",
)
FRESH_TOPICS = ("body_cmd", "cmd_vel_safe", "odom", "tracking")


@dataclass(frozen=True)
class AcceptanceLimits:
    max_lateral_error_m: float = 0.15
    max_heading_error_deg: float = 5.0
    max_odom_speed_mps: float = 0.23
    max_command_speed_mps: float = 0.23
    min_samples: int = 20
    min_odom_distance_m: float = 0.20
    max_topic_age_s: float = 0.60
    acceptable_stop_reasons: tuple[str, ...] = ("route_complete",)
    require_bag: bool = False
    required_bag_topics: tuple[str, ...] = DEFAULT_REQUIRED_BAG_TOPICS


@dataclass(frozen=True)
class FieldMotionReport:
    label: str | None
    jsonl_path: str
    passed: bool
    failed_checks: list[str]
    stop_reason: str | None
    samples: int
    run_samples: int
    tracking_samples: int
    duration_s: float | None
    odom_distance_m: float | None
    max_abs_lateral_error_m: float | None
    max_abs_heading_error_deg: float | None
    max_abs_odom_speed_mps: float | None
    max_abs_body_cmd_x_mps: float | None
    max_abs_safe_cmd_x_mps: float | None
    max_abs_relay_cmd_x_mps: float | None
    control_status_counts: dict[str, int]
    missing_jsonl_fields: list[str]
    fresh_topic_sample_counts: dict[str, int]
    bag_metadata_path: str | None
    missing_bag_topics: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def analyze_jsonl(
    jsonl_path: Path,
    limits: AcceptanceLimits,
    *,
    bag_metadata_path: Path | None = None,
) -> FieldMotionReport:
    label, stop_reason, snapshots = _load_run(jsonl_path)
    run_samples = [sample for sample in snapshots if sample.get("phase") == "run"]
    relay_samples = [sample for sample in run_samples if sample.get("relay_active") is True]
    metric_samples = relay_samples or run_samples

    missing_fields = [
        field
        for field in REQUIRED_JSONL_FIELDS
        if not any(field in sample for sample in metric_samples)
    ]
    control_counts = Counter(
        str(sample.get("control_status_value"))
        for sample in metric_samples
        if sample.get("control_status_value") is not None
    )
    fresh_counts = {
        topic: sum(
            1
            for sample in metric_samples
            if _topic_age(sample, topic) is not None
            and _topic_age(sample, topic) <= limits.max_topic_age_s
        )
        for topic in FRESH_TOPICS
    }

    odom_distance = _odom_distance(metric_samples)
    max_lateral = _max_abs(metric_samples, "lateral_error_m")
    max_heading = _max_abs(metric_samples, "heading_error_deg")
    max_odom_speed = _max_abs(metric_samples, "odom_vx")
    max_body_cmd = _max_abs(metric_samples, "body_cmd_x")
    max_safe_cmd = _max_abs(metric_samples, "safe_cmd_x")
    max_relay_cmd = _max_abs(metric_samples, "relay_cmd_x")
    duration = _duration(metric_samples)
    missing_bag_topics = _missing_bag_topics(
        bag_metadata_path, limits.required_bag_topics
    )

    failed_checks = _failed_checks(
        limits=limits,
        stop_reason=stop_reason,
        metric_sample_count=len(metric_samples),
        tracking_sample_count=sum(
            control_counts.get(status, 0) for status in ("TRACKING", "GOAL_REACHED")
        ),
        max_lateral=max_lateral,
        max_heading=max_heading,
        max_odom_speed=max_odom_speed,
        max_body_cmd=max_body_cmd,
        max_safe_cmd=max_safe_cmd,
        max_relay_cmd=max_relay_cmd,
        odom_distance=odom_distance,
        control_counts=control_counts,
        missing_fields=missing_fields,
        fresh_counts=fresh_counts,
        bag_metadata_path=bag_metadata_path,
        missing_bag_topics=missing_bag_topics,
    )

    return FieldMotionReport(
        label=label,
        jsonl_path=str(jsonl_path),
        passed=not failed_checks,
        failed_checks=failed_checks,
        stop_reason=stop_reason,
        samples=len(snapshots),
        run_samples=len(metric_samples),
        tracking_samples=sum(
            control_counts.get(status, 0) for status in ("TRACKING", "GOAL_REACHED")
        ),
        duration_s=duration,
        odom_distance_m=odom_distance,
        max_abs_lateral_error_m=max_lateral,
        max_abs_heading_error_deg=max_heading,
        max_abs_odom_speed_mps=max_odom_speed,
        max_abs_body_cmd_x_mps=max_body_cmd,
        max_abs_safe_cmd_x_mps=max_safe_cmd,
        max_abs_relay_cmd_x_mps=max_relay_cmd,
        control_status_counts=dict(control_counts),
        missing_jsonl_fields=missing_fields,
        fresh_topic_sample_counts=fresh_counts,
        bag_metadata_path=None if bag_metadata_path is None else str(bag_metadata_path),
        missing_bag_topics=missing_bag_topics,
    )


def _load_run(jsonl_path: Path) -> tuple[str | None, str | None, list[dict[str, Any]]]:
    label: str | None = None
    stop_reason: str | None = None
    snapshots: list[dict[str, Any]] = []
    with jsonl_path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                record = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{jsonl_path}:{line_no}: invalid JSON: {exc}") from exc
            if record.get("event") == "script_start":
                label = record.get("label")
            if record.get("event") in ("stop", "stop_begin"):
                stop_reason = record.get("reason")
            snapshot = record.get("snapshot")
            if isinstance(snapshot, dict):
                snapshots.append(snapshot)
            elif isinstance(record, dict) and "phase" in record:
                snapshots.append(record)
    return label, stop_reason, snapshots


def _topic_age(sample: dict[str, Any], topic: str) -> float | None:
    age = (sample.get("rx_age") or {}).get(topic)
    return _as_float(age)


def _max_abs(samples: Iterable[dict[str, Any]], field: str) -> float | None:
    values = [_as_float(sample.get(field)) for sample in samples]
    finite = [abs(value) for value in values if value is not None]
    return max(finite) if finite else None


def _duration(samples: list[dict[str, Any]]) -> float | None:
    elapsed = [_as_float(sample.get("elapsed_s")) for sample in samples]
    finite = [value for value in elapsed if value is not None]
    if not finite:
        return None
    return max(finite) - min(finite)


def _odom_distance(samples: list[dict[str, Any]]) -> float | None:
    poses = [
        (float(sample["odom_x"]), float(sample["odom_y"]))
        for sample in samples
        if _as_float(sample.get("odom_x")) is not None
        and _as_float(sample.get("odom_y")) is not None
    ]
    if len(poses) < 2:
        return None
    first = poses[0]
    last = poses[-1]
    return math.hypot(last[0] - first[0], last[1] - first[1])


def _as_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _missing_bag_topics(
    bag_metadata_path: Path | None,
    required_topics: Sequence[str],
) -> list[str]:
    if bag_metadata_path is None:
        return []
    metadata = yaml.safe_load(bag_metadata_path.read_text(encoding="utf-8")) or {}
    info = metadata.get("rosbag2_bagfile_information", metadata)
    present: set[str] = set()
    for topic_info in info.get("topics_with_message_count", []) or []:
        topic_metadata = topic_info.get("topic_metadata") or {}
        name = topic_metadata.get("name")
        count = int(topic_info.get("message_count") or 0)
        if name and count > 0:
            present.add(str(name))
    return [topic for topic in required_topics if topic not in present]


def _failed_checks(
    *,
    limits: AcceptanceLimits,
    stop_reason: str | None,
    metric_sample_count: int,
    tracking_sample_count: int,
    max_lateral: float | None,
    max_heading: float | None,
    max_odom_speed: float | None,
    max_body_cmd: float | None,
    max_safe_cmd: float | None,
    max_relay_cmd: float | None,
    odom_distance: float | None,
    control_counts: Counter[str],
    missing_fields: list[str],
    fresh_counts: dict[str, int],
    bag_metadata_path: Path | None,
    missing_bag_topics: list[str],
) -> list[str]:
    failed: list[str] = []
    if metric_sample_count < limits.min_samples:
        failed.append(
            f"run_samples {metric_sample_count} < required {limits.min_samples}"
        )
    if tracking_sample_count <= 0:
        failed.append("no TRACKING/GOAL_REACHED samples")
    if stop_reason not in limits.acceptable_stop_reasons:
        failed.append(
            f"stop_reason {stop_reason!r} not in {limits.acceptable_stop_reasons}"
        )
    if odom_distance is None or odom_distance < limits.min_odom_distance_m:
        failed.append(
            f"odom_distance {odom_distance} < required {limits.min_odom_distance_m:.3f}m"
        )
    if max_lateral is None or max_lateral > limits.max_lateral_error_m:
        failed.append(
            f"max_abs_lateral_error {max_lateral} > {limits.max_lateral_error_m:.3f}m"
        )
    if max_heading is None or max_heading > limits.max_heading_error_deg:
        failed.append(
            f"max_abs_heading_error {max_heading} > {limits.max_heading_error_deg:.3f}deg"
        )
    if max_odom_speed is None or max_odom_speed > limits.max_odom_speed_mps:
        failed.append(
            f"max_abs_odom_speed {max_odom_speed} > {limits.max_odom_speed_mps:.3f}m/s"
        )
    for name, value in (
        ("body_cmd", max_body_cmd),
        ("safe_cmd", max_safe_cmd),
        ("relay_cmd", max_relay_cmd),
    ):
        if value is None or value > limits.max_command_speed_mps:
            failed.append(
                f"max_abs_{name}_x {value} > {limits.max_command_speed_mps:.3f}m/s"
            )
    if missing_fields:
        failed.append(f"missing_jsonl_fields={','.join(missing_fields)}")
    stale_topics = [
        topic for topic, count in fresh_counts.items() if count <= 0
    ]
    if stale_topics:
        failed.append(f"no_fresh_samples_for={','.join(stale_topics)}")
    bad_statuses = {
        status: count
        for status, count in control_counts.items()
        if status not in ("TRACKING", "GOAL_REACHED")
    }
    if bad_statuses:
        failed.append(f"unexpected_control_statuses={dict(bad_statuses)}")
    if limits.require_bag and bag_metadata_path is None:
        failed.append("bag_metadata_required_but_not_provided")
    if missing_bag_topics:
        failed.append(f"missing_bag_topics={','.join(missing_bag_topics)}")
    return failed


def _write_markdown(report: FieldMotionReport, path: Path) -> None:
    status = "PASS" if report.passed else "FAIL"
    lines = [
        "# Day 5 field motion acceptance report",
        "",
        f"- status: {status}",
        f"- label: {report.label}",
        f"- stop_reason: {report.stop_reason}",
        f"- run_samples: {report.run_samples}",
        f"- tracking_samples: {report.tracking_samples}",
        f"- duration_s: {report.duration_s}",
        f"- odom_distance_m: {report.odom_distance_m}",
        f"- max_abs_lateral_error_m: {report.max_abs_lateral_error_m}",
        f"- max_abs_heading_error_deg: {report.max_abs_heading_error_deg}",
        f"- max_abs_odom_speed_mps: {report.max_abs_odom_speed_mps}",
        f"- missing_jsonl_fields: {report.missing_jsonl_fields}",
        f"- missing_bag_topics: {report.missing_bag_topics}",
        "",
        "## Failed checks",
        "",
        *[f"- {item}" for item in report.failed_checks],
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--jsonl", required=True, type=Path)
    parser.add_argument("--bag-metadata", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--max-lateral-error-m", type=float, default=0.15)
    parser.add_argument("--max-heading-error-deg", type=float, default=5.0)
    parser.add_argument("--max-odom-speed-mps", type=float, default=0.23)
    parser.add_argument("--max-command-speed-mps", type=float, default=0.23)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--min-odom-distance-m", type=float, default=0.20)
    parser.add_argument("--max-topic-age-s", type=float, default=0.60)
    parser.add_argument(
        "--acceptable-stop-reason",
        action="append",
        default=[],
        help="May be passed more than once. Default is route_complete.",
    )
    parser.add_argument("--require-bag", action="store_true")
    args = parser.parse_args(argv)

    limits = AcceptanceLimits(
        max_lateral_error_m=args.max_lateral_error_m,
        max_heading_error_deg=args.max_heading_error_deg,
        max_odom_speed_mps=args.max_odom_speed_mps,
        max_command_speed_mps=args.max_command_speed_mps,
        min_samples=args.min_samples,
        min_odom_distance_m=args.min_odom_distance_m,
        max_topic_age_s=args.max_topic_age_s,
        acceptable_stop_reasons=tuple(args.acceptable_stop_reason)
        or ("route_complete",),
        require_bag=bool(args.require_bag),
    )
    report = analyze_jsonl(args.jsonl, limits, bag_metadata_path=args.bag_metadata)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            yaml.safe_dump(report.to_dict(), sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
    if args.report:
        _write_markdown(report, args.report)

    print(
        f"status={'PASS' if report.passed else 'FAIL'} "
        f"label={report.label} stop_reason={report.stop_reason} "
        f"run_samples={report.run_samples} "
        f"max_lateral_m={report.max_abs_lateral_error_m} "
        f"max_heading_deg={report.max_abs_heading_error_deg}"
    )
    if report.failed_checks:
        print("failed_checks:")
        for item in report.failed_checks:
            print(f"- {item}")
    return 0 if report.passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
