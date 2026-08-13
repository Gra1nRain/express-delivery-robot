#!/usr/bin/env python3
"""Pure helpers for Day5 supervised-run timing and trajectory metadata."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml


DEFAULT_WATCHDOG_FALLBACK_S = 420.0
DEFAULT_WATCHDOG_DURATION_SCALE = 2.5
DEFAULT_WATCHDOG_MARGIN_S = 60.0
DEFAULT_WATCHDOG_MINIMUM_S = 120.0
LOCAL_PLANNER_READY_STATUSES = frozenset(
    {
        "REPLANNED",
        "REFERENCE_CLEAR",
        "RELAXED_REPLANNED",
        "RELAXED_HOLD",
        "DWA_TRACKING",
        "DWA_AVOIDING",
    }
)


@dataclass(frozen=True)
class RouteMetadata:
    point_count: int | None
    finish_xy: tuple[float, float] | None
    duration_s: float | None


def local_planner_status_is_ready(status: str | None) -> bool:
    return status in LOCAL_PLANNER_READY_STATUSES


def control_status_requires_stop(
    status: str | None,
    reasons: Sequence[str],
) -> bool:
    if status == "FAULT_HOLD":
        return True
    return status in ("INVALID_STATE", "ERROR", "RECOVERY_REQUIRED") and bool(
        reasons
    )


def load_route_metadata(path: Path) -> RouteMetadata:
    if not path.exists():
        return RouteMetadata(None, None, None)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    points = data.get("points") or []
    finish_xy = None
    if points:
        finish = points[-1]
        finish_xy = (float(finish["x"]), float(finish["y"]))
    duration = data.get("duration_s")
    duration_s = float(duration) if duration is not None else None
    if duration_s is not None and duration_s <= 0.0:
        duration_s = None
    return RouteMetadata(len(points) if points else None, finish_xy, duration_s)


def resolve_watchdog_timeout_s(
    explicit_timeout_s: float | None,
    planned_duration_s: float | None,
    *,
    duration_scale: float = DEFAULT_WATCHDOG_DURATION_SCALE,
    margin_s: float = DEFAULT_WATCHDOG_MARGIN_S,
    minimum_s: float = DEFAULT_WATCHDOG_MINIMUM_S,
    fallback_s: float = DEFAULT_WATCHDOG_FALLBACK_S,
) -> float | None:
    if explicit_timeout_s is not None:
        if explicit_timeout_s < 0.0:
            raise ValueError("explicit watchdog timeout must be non-negative")
        if explicit_timeout_s == 0.0:
            return None
        return explicit_timeout_s
    if planned_duration_s is None:
        return fallback_s
    if duration_scale <= 0.0 or margin_s < 0.0 or minimum_s <= 0.0:
        raise ValueError("invalid adaptive watchdog policy")
    return max(minimum_s, planned_duration_s * duration_scale + margin_s)


def scan_stop_reason(
    scan_min_m: float | None,
    stop_threshold_m: float,
) -> str | None:
    if stop_threshold_m <= 0.0 or scan_min_m is None:
        return None
    if scan_min_m < stop_threshold_m:
        return f"scan_min_under_{stop_threshold_m:.2f}m"
    return None
