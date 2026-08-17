"""Pure state machine for supervised execution of stop-bounded route segments."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


_RECOVERABLE_FRESHNESS_FAILURES = frozenset({"stale_pose", "stale_velocity"})


def state_failure_requires_rearm(reasons: tuple[str, ...]) -> bool:
    """Return whether an invalid estimate must latch the route in FAULT_HOLD."""
    return not reasons or not set(reasons).issubset(
        _RECOVERABLE_FRESHNESS_FAILURES
    )


class SegmentedRoutePhase(str, Enum):
    DISARMED = "MISSION_DISARMED"
    TRACKING = "SEGMENT_TRACKING"
    DOCK_HOLD = "DOCK_HOLD"
    WAIT_RELEASE = "WAIT_RELEASE"
    SAFETY_HOLD = "SAFETY_HOLD"
    OVERSHOOT_HOLD = "DOCK_OVERSHOOT_HOLD"
    FAULT_HOLD = "FAULT_HOLD"
    COMPLETED = "ROUTE_COMPLETED"


@dataclass(frozen=True)
class SegmentedRouteConfig:
    goal_position_tolerance_m: float = 0.10
    goal_heading_tolerance_rad: float = math.radians(4.0)
    goal_overshoot_tolerance_m: float = 0.02
    stop_speed_tolerance_mps: float = 0.03
    dock_hold_s: float = 2.0

    def __post_init__(self) -> None:
        if self.goal_position_tolerance_m <= 0.0:
            raise ValueError("goal_position_tolerance_m must be positive")
        if self.goal_heading_tolerance_rad < 0.0:
            raise ValueError("goal_heading_tolerance_rad must be non-negative")
        if self.goal_overshoot_tolerance_m < 0.0:
            raise ValueError("goal_overshoot_tolerance_m must be non-negative")
        if self.stop_speed_tolerance_mps < 0.0:
            raise ValueError("stop_speed_tolerance_mps must be non-negative")
        if self.dock_hold_s < 0.0:
            raise ValueError("dock_hold_s must be non-negative")


@dataclass(frozen=True)
class SegmentedRouteObservation:
    now_s: float
    enabled: bool
    state_valid: bool
    stop_requested: bool
    position_error_m: float
    heading_error_rad: float
    speed_mps: float
    longitudinal_error_m: float = math.nan
    release_segment_index: int | None = None


@dataclass(frozen=True)
class SegmentedRouteDecision:
    phase: SegmentedRoutePhase
    active_segment_index: int
    allow_tracking: bool
    segment_changed: bool = False


class SegmentedRouteStateMachine:
    """Hide all segment switching and hold semantics behind one update method."""

    def __init__(
        self,
        segment_count: int,
        config: SegmentedRouteConfig,
    ) -> None:
        if segment_count < 1:
            raise ValueError("segment_count must be positive")
        self._segment_count = segment_count
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._phase = SegmentedRoutePhase.DISARMED
        self._active_segment_index = 0
        self._dock_hold_started_s: float | None = None

    def update(
        self,
        observation: SegmentedRouteObservation,
    ) -> SegmentedRouteDecision:
        if not math.isfinite(observation.now_s):
            raise ValueError("now_s must be finite")

        if not observation.enabled:
            if self._phase != SegmentedRoutePhase.COMPLETED:
                self._phase = SegmentedRoutePhase.DISARMED
                self._dock_hold_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase == SegmentedRoutePhase.COMPLETED:
            return self._decision(allow_tracking=False)
        if self._phase in (
            SegmentedRoutePhase.OVERSHOOT_HOLD,
            SegmentedRoutePhase.FAULT_HOLD,
        ):
            return self._decision(allow_tracking=False)

        if not observation.state_valid:
            self._phase = SegmentedRoutePhase.FAULT_HOLD
            self._dock_hold_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase == SegmentedRoutePhase.WAIT_RELEASE:
            if observation.stop_requested:
                return self._decision(allow_tracking=False)
            release_index = observation.release_segment_index
            if release_index is None:
                return self._decision(allow_tracking=False)
            if not (
                self._active_segment_index < release_index < self._segment_count
            ):
                return self._decision(allow_tracking=False)
            self._active_segment_index = release_index
            self._phase = SegmentedRoutePhase.TRACKING
            return self._decision(
                allow_tracking=False,
                segment_changed=True,
            )

        if observation.stop_requested:
            self._phase = SegmentedRoutePhase.SAFETY_HOLD
            self._dock_hold_started_s = None
            return self._decision(allow_tracking=False)

        if self._phase in (
            SegmentedRoutePhase.DISARMED,
            SegmentedRoutePhase.SAFETY_HOLD,
        ):
            self._phase = SegmentedRoutePhase.TRACKING

        pose_at_goal = (
            math.isfinite(observation.position_error_m)
            and math.isfinite(observation.heading_error_rad)
            and observation.position_error_m
            <= self._config.goal_position_tolerance_m
            and abs(observation.heading_error_rad)
            <= self._config.goal_heading_tolerance_rad
        )
        stopped = (
            math.isfinite(observation.speed_mps)
            and abs(observation.speed_mps)
            <= self._config.stop_speed_tolerance_mps
        )
        overshot = (
            math.isfinite(observation.longitudinal_error_m)
            and observation.longitudinal_error_m
            > self._config.goal_overshoot_tolerance_m
        )

        if self._phase == SegmentedRoutePhase.TRACKING:
            if pose_at_goal:
                self._phase = SegmentedRoutePhase.DOCK_HOLD
                self._dock_hold_started_s = observation.now_s if stopped else None
                return self._decision(allow_tracking=False)
            if overshot:
                self._phase = SegmentedRoutePhase.OVERSHOOT_HOLD
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=False)
            return self._decision(allow_tracking=True)

        if self._phase == SegmentedRoutePhase.DOCK_HOLD:
            if not pose_at_goal:
                if overshot:
                    self._phase = SegmentedRoutePhase.OVERSHOOT_HOLD
                    self._dock_hold_started_s = None
                    return self._decision(allow_tracking=False)
                self._phase = SegmentedRoutePhase.TRACKING
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=True)
            if not stopped:
                self._dock_hold_started_s = None
                return self._decision(allow_tracking=False)
            if self._dock_hold_started_s is None:
                self._dock_hold_started_s = observation.now_s
                return self._decision(allow_tracking=False)
            if (
                observation.now_s - self._dock_hold_started_s
                < self._config.dock_hold_s
            ):
                return self._decision(allow_tracking=False)
            self._dock_hold_started_s = None
            if self._active_segment_index >= self._segment_count - 1:
                self._phase = SegmentedRoutePhase.COMPLETED
                return self._decision(allow_tracking=False)
            self._phase = SegmentedRoutePhase.WAIT_RELEASE
            return self._decision(allow_tracking=False)

        raise RuntimeError(f"unsupported segmented route phase: {self._phase}")

    def _decision(
        self,
        *,
        allow_tracking: bool,
        segment_changed: bool = False,
    ) -> SegmentedRouteDecision:
        return SegmentedRouteDecision(
            phase=self._phase,
            active_segment_index=self._active_segment_index,
            allow_tracking=allow_tracking,
            segment_changed=segment_changed,
        )
