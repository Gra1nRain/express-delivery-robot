"""Two-dimensional dynamic-obstacle risk assessment."""

from __future__ import annotations

from dataclasses import dataclass
import math

from competition_avoidance.tracker import TrackedObstacle


@dataclass(frozen=True)
class EgoState:
    x: float
    y: float
    vx_mps: float
    vy_mps: float

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.vx_mps, self.vy_mps)


@dataclass(frozen=True)
class RiskConfig:
    prediction_horizon_s: float = 3.0
    emergency_distance_m: float = 0.85
    collision_distance_m: float = 1.00
    slowdown_distance_m: float = 1.50
    reaction_time_s: float = 0.35
    max_deceleration_mps2: float = 0.30
    safety_margin_m: float = 0.40

    def __post_init__(self) -> None:
        if self.prediction_horizon_s <= 0.0:
            raise ValueError("prediction_horizon_s must be positive")
        if not (
            0.0
            < self.emergency_distance_m
            <= self.collision_distance_m
            <= self.slowdown_distance_m
        ):
            raise ValueError("risk distances must be positive and ordered")
        stopping_distance_m(
            speed_mps=0.0,
            reaction_time_s=self.reaction_time_s,
            max_deceleration_mps2=self.max_deceleration_mps2,
            safety_margin_m=self.safety_margin_m,
        )


@dataclass(frozen=True)
class RiskAssessment:
    level: str
    reason: str
    track_id: int | None = None
    current_distance_m: float | None = None
    time_to_cpa_s: float | None = None
    distance_at_cpa_m: float | None = None
    stopping_distance_m: float = 0.0

    @property
    def stop_required(self) -> bool:
        return self.level in {"STOP", "EMERGENCY"}


def stopping_distance_m(
    *,
    speed_mps: float,
    reaction_time_s: float,
    max_deceleration_mps2: float,
    safety_margin_m: float,
) -> float:
    speed = abs(float(speed_mps))
    if reaction_time_s < 0.0:
        raise ValueError("reaction_time_s must be non-negative")
    if max_deceleration_mps2 <= 0.0:
        raise ValueError("max_deceleration_mps2 must be positive")
    if safety_margin_m < 0.0:
        raise ValueError("safety_margin_m must be non-negative")
    return (
        speed * reaction_time_s
        + speed * speed / (2.0 * max_deceleration_mps2)
        + safety_margin_m
    )


def evaluate_dynamic_risk(
    tracks: tuple[TrackedObstacle, ...],
    ego: EgoState,
    config: RiskConfig = RiskConfig(),
) -> RiskAssessment:
    """Return the highest conservative CPA risk from confirmed dynamic tracks."""

    stopping_distance = stopping_distance_m(
        speed_mps=ego.speed_mps,
        reaction_time_s=config.reaction_time_s,
        max_deceleration_mps2=config.max_deceleration_mps2,
        safety_margin_m=config.safety_margin_m,
    )
    assessments: list[RiskAssessment] = []
    for track in tracks:
        if not track.confirmed or track.motion_state != "DYNAMIC":
            continue
        relative_x = track.x - ego.x
        relative_y = track.y - ego.y
        relative_vx = track.vx_mps - ego.vx_mps
        relative_vy = track.vy_mps - ego.vy_mps
        current_distance = math.hypot(relative_x, relative_y)
        relative_speed_sq = relative_vx * relative_vx + relative_vy * relative_vy
        if relative_speed_sq <= 1e-9:
            time_to_cpa = 0.0
        else:
            time_to_cpa = -(
                relative_x * relative_vx + relative_y * relative_vy
            ) / relative_speed_sq
            time_to_cpa = min(config.prediction_horizon_s, max(0.0, time_to_cpa))
        distance_at_cpa = math.hypot(
            relative_x + relative_vx * time_to_cpa,
            relative_y + relative_vy * time_to_cpa,
        )

        if current_distance <= config.emergency_distance_m:
            level = "EMERGENCY"
            reason = "dynamic_obstacle_inside_emergency_distance"
        elif (
            current_distance <= stopping_distance
            or distance_at_cpa <= config.collision_distance_m
        ):
            level = "STOP"
            reason = "predicted_dynamic_collision"
        elif distance_at_cpa <= config.slowdown_distance_m:
            level = "SLOWDOWN"
            reason = "dynamic_obstacle_near_predicted_path"
        else:
            level = "CLEAR"
            reason = "dynamic_tracks_clear"
        assessments.append(
            RiskAssessment(
                level=level,
                reason=reason,
                track_id=track.track_id,
                current_distance_m=current_distance,
                time_to_cpa_s=time_to_cpa,
                distance_at_cpa_m=distance_at_cpa,
                stopping_distance_m=stopping_distance,
            )
        )

    if not assessments:
        return RiskAssessment(
            level="CLEAR",
            reason="no_confirmed_dynamic_tracks",
            stopping_distance_m=stopping_distance,
        )
    priority = {"CLEAR": 0, "SLOWDOWN": 1, "STOP": 2, "EMERGENCY": 3}
    return max(
        assessments,
        key=lambda item: (
            priority[item.level],
            -(item.time_to_cpa_s or 0.0),
        ),
    )
