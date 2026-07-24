"""Clock-domain adapter for Livox messages stamped by the lidar clock."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ClockOffsetEstimator:
    """Estimate and freeze the lidar-to-ROS clock offset.

    Transport delay is non-negative, so the minimum observed
    ``receipt - source`` delta is the least biased offset estimate.  The
    estimate is frozen after a short IMU-only calibration window to avoid
    introducing time jumps while FAST-LIO is running.
    """

    calibration_samples: int = 20
    _observed_offsets_ns: list[int] = field(default_factory=list, init=False)
    _offset_ns: int | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.calibration_samples < 1:
            raise ValueError("calibration_samples must be positive")

    @property
    def ready(self) -> bool:
        return self._offset_ns is not None

    @property
    def offset_ns(self) -> int:
        if self._offset_ns is None:
            raise RuntimeError("clock offset is not calibrated")
        return self._offset_ns

    def observe(self, *, source_stamp_ns: int, receipt_stamp_ns: int) -> bool:
        """Add one calibration sample and return whether calibration is ready."""

        if self.ready:
            return True
        if source_stamp_ns < 0 or receipt_stamp_ns < 0:
            raise ValueError("timestamps must be non-negative")

        self._observed_offsets_ns.append(receipt_stamp_ns - source_stamp_ns)
        if len(self._observed_offsets_ns) >= self.calibration_samples:
            self._offset_ns = min(self._observed_offsets_ns)
            self._observed_offsets_ns.clear()
        return self.ready

    def rebase(self, source_stamp_ns: int) -> int:
        """Translate one lidar-clock timestamp into the ROS clock domain."""

        if source_stamp_ns < 0:
            raise ValueError("source_stamp_ns must be non-negative")
        return max(0, source_stamp_ns + self.offset_ns)
