"""Thread-safe software interlock for retaining a grasped object."""

from __future__ import annotations

import math
from threading import Lock


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
