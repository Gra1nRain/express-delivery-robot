"""Pure state gate for publishing only fresh, previously unseen LiDAR frames."""

from __future__ import annotations

import math


class LatestFrameGate:
    """Track the last published input sequence and enforce a maximum age."""

    def __init__(self, maximum_age_s: float) -> None:
        if not math.isfinite(maximum_age_s) or maximum_age_s <= 0.0:
            raise ValueError("maximum_age_s must be finite and positive")
        self.maximum_age_s = float(maximum_age_s)
        self._published_sequence = 0

    def should_publish(self, sequence: int, age_s: float) -> bool:
        """Return true when a frame is new and still safe to consume."""
        if sequence <= self._published_sequence:
            return False
        if not math.isfinite(age_s) or age_s < 0.0:
            return False
        return age_s <= self.maximum_age_s

    def mark_published(self, sequence: int) -> None:
        """Record a successful publish."""
        if sequence <= self._published_sequence:
            raise ValueError("published sequence must advance")
        self._published_sequence = sequence
