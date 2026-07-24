"""Small deterministic rate gate for latest-sample processing."""

from __future__ import annotations

import math


class LatestSampleRateGate:
    """Allow at most one expensive processing cycle per configured interval."""

    def __init__(self, frequency_hz: float) -> None:
        frequency = float(frequency_hz)
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError("frequency_hz must be finite and positive")
        self._minimum_interval_s = 1.0 / frequency
        self._last_allowed_s: float | None = None

    def allow(self, timestamp_s: float) -> bool:
        timestamp = float(timestamp_s)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp_s must be finite")
        if (
            self._last_allowed_s is None
            or timestamp - self._last_allowed_s
            >= self._minimum_interval_s - 1e-12
        ):
            self._last_allowed_s = timestamp
            return True
        return False
