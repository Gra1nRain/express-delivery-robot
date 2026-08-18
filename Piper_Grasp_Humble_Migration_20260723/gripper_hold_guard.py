"""Thread-safe software interlock for retaining a grasped object."""

from __future__ import annotations

import math
import statistics
from threading import Lock


def choose_bottle_hold_position(
    commanded_closed_m: float,
    measured_opening_m: float,
    preload_m: float,
) -> float:
    """Keep contact without repeatedly driving an obstructed gripper to zero."""
    commanded_closed_m = float(commanded_closed_m)
    measured_opening_m = float(measured_opening_m)
    preload_m = float(preload_m)
    values = (commanded_closed_m, measured_opening_m, preload_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gripper hold inputs must be finite")
    if commanded_closed_m < 0.0 or measured_opening_m < 0.0 or preload_m < 0.0:
        raise ValueError("gripper hold inputs must be non-negative")
    return max(commanded_closed_m, measured_opening_m - preload_m)


def evaluate_contact_stability(
    openings_m,
    *,
    min_samples: int,
    max_span_m: float,
    min_contact_opening_m: float,
) -> dict[str, float | int | bool]:
    """Summarize whether recent gripper feedback is a stable object contact."""
    min_samples = int(min_samples)
    max_span_m = float(max_span_m)
    min_contact_opening_m = float(min_contact_opening_m)
    if min_samples < 1:
        raise ValueError("min_samples must be positive")
    if max_span_m < 0.0 or min_contact_opening_m < 0.0:
        raise ValueError("contact thresholds must be non-negative")

    values = [float(value) for value in openings_m]
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("gripper openings must be finite and non-negative")

    window = values[-min_samples:]
    median_opening_m = (
        float(statistics.median(window)) if window else 0.0
    )
    span_m = float(max(window) - min(window)) if window else math.inf
    stable = (
        len(window) >= min_samples
        and median_opening_m >= min_contact_opening_m
        and span_m <= max_span_m
    )
    return {
        "stable": stable,
        "sample_count": len(window),
        "median_opening_m": median_opening_m,
        "span_m": span_m,
    }


class GripperHoldGuard:
    """Clamp premature open commands until placement authorizes release."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._active = False
        self._closed_m = 0.0
        self._target_type = ""
        self._release_authorized = False

    def activate(self, closed_m: float, target_type: str) -> None:
        closed_m = float(closed_m)
        if not math.isfinite(closed_m) or closed_m < 0.0:
            raise ValueError("closed gripper position must be finite and non-negative")
        with self._lock:
            self._active = True
            self._closed_m = closed_m
            self._target_type = str(target_type).strip()
            self._release_authorized = False

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def apply(self, requested_m: float) -> tuple[float, bool]:
        requested_m = float(requested_m)
        with self._lock:
            if (
                not self._active
                or self._release_authorized
                or requested_m <= self._closed_m + 1e-6
            ):
                return requested_m, False
            return self._closed_m, True

    def authorize_release(self, reason: str) -> None:
        if not str(reason).strip():
            raise ValueError("release authorization requires a reason")
        with self._lock:
            if not self._active:
                raise RuntimeError("cannot authorize release without an active hold")
            self._release_authorized = True

    def cancel_release(self) -> None:
        with self._lock:
            self._release_authorized = False

    def complete_release(self) -> None:
        with self._lock:
            self._active = False
            self._closed_m = 0.0
            self._target_type = ""
            self._release_authorized = False

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "active": self._active,
                "closed_m": self._closed_m,
                "target_type": self._target_type,
                "release_authorized": self._release_authorized,
            }
