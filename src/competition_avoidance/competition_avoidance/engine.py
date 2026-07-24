"""High-level static and dynamic avoidance decision interface."""

from __future__ import annotations

from dataclasses import dataclass

from competition_avoidance.perception import ObstacleDetection
from competition_avoidance.risk import (
    EgoState,
    RiskAssessment,
    RiskConfig,
    evaluate_dynamic_risk,
)
from competition_avoidance.tracker import (
    ObstacleTracker,
    TrackedObstacle,
    TrackerConfig,
)


@dataclass(frozen=True)
class AvoidanceDecision:
    mode: str
    stop_required: bool
    reason: str
    tracks: tuple[TrackedObstacle, ...]
    static_track_count: int
    dynamic_track_count: int
    risk: RiskAssessment


class AvoidanceEngine:
    """Own tracking and risk policy behind one deterministic interface."""

    def __init__(
        self,
        *,
        tracker_config: TrackerConfig = TrackerConfig(),
        risk_config: RiskConfig = RiskConfig(),
    ) -> None:
        self._tracker = ObstacleTracker(tracker_config)
        self._risk_config = risk_config

    def update(
        self,
        detections: tuple[ObstacleDetection, ...],
        *,
        timestamp_s: float,
        ego: EgoState,
        proximity_stop: bool,
    ) -> AvoidanceDecision:
        tracks = self._tracker.update(detections, timestamp_s=timestamp_s)
        static_count = sum(
            track.confirmed and track.motion_state == "STATIC" for track in tracks
        )
        dynamic_count = sum(
            track.confirmed and track.motion_state == "DYNAMIC" for track in tracks
        )
        risk = evaluate_dynamic_risk(tracks, ego, self._risk_config)

        if proximity_stop:
            mode = "EMERGENCY_HOLD"
            stop_required = True
            reason = "proximity_stop"
        elif risk.level in {"STOP", "EMERGENCY"}:
            mode = "DYNAMIC_STOP"
            stop_required = True
            reason = risk.reason
        elif risk.level == "SLOWDOWN":
            mode = "DYNAMIC_SLOWDOWN"
            stop_required = False
            reason = risk.reason
        elif static_count:
            mode = "STATIC_REPLAN"
            stop_required = False
            reason = "confirmed_static_obstacle"
        else:
            mode = "CLEAR"
            stop_required = False
            reason = risk.reason
        return AvoidanceDecision(
            mode=mode,
            stop_required=stop_required,
            reason=reason,
            tracks=tracks,
            static_track_count=static_count,
            dynamic_track_count=dynamic_count,
            risk=risk,
        )
