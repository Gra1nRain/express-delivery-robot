"""Field-independent precision docking policy for semantic checkpoints."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
from typing import Any

from competition_control.mission_checkpoints import MissionCheckpoint
from competition_control.mppi_controller import (
    ControlTrajectory,
    ControlTrajectoryPoint,
    VehicleState,
)


class RequestedMotionMode(str, Enum):
    DUAL_ACKERMANN = "dual_ackermann"
    SPIN = "spin"
    PARALLEL = "parallel"


class PrecisionDockingPhase(str, Enum):
    NORMAL_NAV = "NORMAL_NAV"
    PRECISION_APPROACH = "PRECISION_APPROACH"
    STOP_SETTLE = "STOP_SETTLE"
    HEADING_TRIM = "HEADING_TRIM"
    POSITION_TRIM = "POSITION_TRIM"
    DOCK_READY = "DOCK_READY"
    ALIGNMENT_HOLD = "ALIGNMENT_HOLD"


@dataclass(frozen=True)
class PrecisionDockingConfig:
    activation_distance_m: float
    trim_entry_distance_m: float
    final_position_tolerance_m: float
    heading_realign_tolerance_rad: float
    heading_trim_target_rad: float
    settle_time_s: float = 0.30
    stable_time_s: float = 0.50
    stopped_speed_tolerance_mps: float = 0.02
    stopped_yaw_rate_tolerance_radps: float = 0.03
    max_spin_correction_rad: float = math.radians(10.0)
    max_parallel_correction_m: float = 0.08
    spin_gain: float = 1.0
    spin_min_yaw_rate_radps: float = 0.06
    spin_max_yaw_rate_radps: float = 0.15
    parallel_gain: float = 0.8
    parallel_min_speed_mps: float = 0.04
    parallel_max_speed_mps: float = 0.06

    def __post_init__(self) -> None:
        if not (
            self.activation_distance_m
            > self.trim_entry_distance_m
            > self.final_position_tolerance_m
            > 0.0
        ):
            raise ValueError(
                "precision distances must satisfy activation > trim entry > final > 0"
            )
        if not (
            0.0
            < self.heading_trim_target_rad
            < self.heading_realign_tolerance_rad
            <= self.max_spin_correction_rad
        ):
            raise ValueError(
                "heading thresholds must satisfy target < realign <= max correction"
            )
        if self.settle_time_s < 0.0 or self.stable_time_s < 0.0:
            raise ValueError("precision settle times must be non-negative")
        if self.max_parallel_correction_m <= self.final_position_tolerance_m:
            raise ValueError("max parallel correction must exceed final tolerance")
        if not (0.0 < self.spin_min_yaw_rate_radps <= self.spin_max_yaw_rate_radps):
            raise ValueError("spin rates must satisfy 0 < min <= max")
        if not (0.0 < self.parallel_min_speed_mps <= self.parallel_max_speed_mps):
            raise ValueError("parallel speeds must satisfy 0 < min <= max")


@dataclass(frozen=True)
class PrecisionDockingDecision:
    phase: PrecisionDockingPhase
    motion_mode: RequestedMotionMode
    use_dynamic_replanning: bool
    use_fixed_reference: bool
    linear_x_mps: float = 0.0
    linear_y_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    pose_ready: bool = False
    position_error_m: float = math.inf
    heading_error_rad: float = math.inf


def precision_docking_configs_from_dict(
    route: dict[str, Any],
    dock_params: dict[str, Any],
) -> dict[str, PrecisionDockingConfig]:
    """Resolve semantic checkpoint assignments without embedding field geometry."""

    route_policy = route.get("precision_docking", {})
    if route_policy is None:
        return {}
    if not isinstance(route_policy, dict):
        raise ValueError("precision_docking route configuration must be a mapping")
    assignments = route_policy.get("checkpoints", {})
    profiles = dock_params.get("precision_profiles", {})
    if not isinstance(assignments, dict) or not isinstance(profiles, dict):
        raise ValueError(
            "precision checkpoint assignments and profiles must be mappings"
        )

    resolved: dict[str, PrecisionDockingConfig] = {}
    for raw_ref, raw_profile_name in assignments.items():
        ref = str(raw_ref).strip()
        profile_name = str(raw_profile_name).strip()
        if not ref or not profile_name:
            raise ValueError(
                "precision checkpoint refs and profile names must be non-empty"
            )
        raw = profiles.get(profile_name)
        if not isinstance(raw, dict):
            raise ValueError(
                f"unknown precision profile {profile_name!r} for checkpoint {ref!r}"
            )
        try:
            resolved[ref] = PrecisionDockingConfig(
                activation_distance_m=float(raw["activation_distance_m"]),
                trim_entry_distance_m=float(raw["trim_entry_distance_m"]),
                final_position_tolerance_m=float(raw["final_position_tolerance_m"]),
                heading_realign_tolerance_rad=math.radians(
                    float(raw["heading_realign_tolerance_deg"])
                ),
                heading_trim_target_rad=math.radians(
                    float(raw["heading_trim_target_deg"])
                ),
                settle_time_s=float(raw.get("settle_time_s", 0.30)),
                stable_time_s=float(raw.get("stable_time_s", 0.50)),
                stopped_speed_tolerance_mps=float(
                    raw.get("stopped_speed_tolerance_mps", 0.02)
                ),
                stopped_yaw_rate_tolerance_radps=float(
                    raw.get("stopped_yaw_rate_tolerance_radps", 0.03)
                ),
                max_spin_correction_rad=math.radians(
                    float(raw.get("max_spin_correction_deg", 10.0))
                ),
                max_parallel_correction_m=float(
                    raw.get("max_parallel_correction_m", 0.08)
                ),
                spin_gain=float(raw.get("spin_gain", 1.0)),
                spin_min_yaw_rate_radps=float(raw.get("spin_min_yaw_rate_radps", 0.06)),
                spin_max_yaw_rate_radps=float(raw.get("spin_max_yaw_rate_radps", 0.15)),
                parallel_gain=float(raw.get("parallel_gain", 0.8)),
                parallel_min_speed_mps=float(raw.get("parallel_min_speed_mps", 0.04)),
                parallel_max_speed_mps=float(raw.get("parallel_max_speed_mps", 0.06)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, ValueError) and str(exc).startswith("precision"):
                raise
            raise ValueError(
                f"invalid precision profile {profile_name!r} for checkpoint {ref!r}"
            ) from exc
    return resolved


def fixed_reference_to_checkpoint(
    trajectory: ControlTrajectory,
    state: VehicleState,
    checkpoint_ref: str,
    *,
    backtrack_points: int = 2,
) -> ControlTrajectory:
    """Slice the frozen global route from the current pose to one semantic stop."""

    if backtrack_points < 0:
        raise ValueError("backtrack_points must be non-negative")
    end_index = next(
        (
            index
            for index, point in enumerate(trajectory.points)
            if point.ref_id == checkpoint_ref
        ),
        None,
    )
    if end_index is None:
        raise ValueError(
            f"checkpoint ref not found in global trajectory: {checkpoint_ref}"
        )
    if end_index < 1:
        raise ValueError("precision checkpoint must not be the first trajectory point")
    nearest_index = min(
        range(end_index + 1),
        key=lambda index: (
            (trajectory.points[index].x - state.x) ** 2
            + (trajectory.points[index].y - state.y) ** 2
        ),
    )
    start_index = min(
        end_index - 1,
        max(0, nearest_index - backtrack_points),
    )
    selected = trajectory.points[start_index : end_index + 1]
    base_s = selected[0].s
    base_t = selected[0].t
    points = tuple(
        replace(point, s=point.s - base_s, t=point.t - base_t) for point in selected
    )
    return ControlTrajectory(
        frame_id=trajectory.frame_id,
        route_name=f"precision_to_{checkpoint_ref}",
        points=points,
    )


class PrecisionDockingController:
    """Select fixed-path approach, spin trim, and two-axis parallel trim."""

    def __init__(self, config: PrecisionDockingConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._phase = PrecisionDockingPhase.NORMAL_NAV
        self._settle_started_s: float | None = None
        self._stable_started_s: float | None = None

    def update(
        self,
        *,
        now_s: float,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        yaw_rate_radps: float,
    ) -> PrecisionDockingDecision:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        dx = checkpoint.x - state.x
        dy = checkpoint.y - state.y
        position_error = math.hypot(dx, dy)
        heading_error = _wrap_angle(checkpoint.yaw - state.yaw)

        if position_error > self._config.activation_distance_m:
            self.reset()
            return self._decision(
                PrecisionDockingPhase.NORMAL_NAV,
                position_error,
                heading_error,
            )
        if position_error > self._config.trim_entry_distance_m:
            self._phase = PrecisionDockingPhase.PRECISION_APPROACH
            self._settle_started_s = None
            self._stable_started_s = None
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                use_fixed_reference=True,
            )

        stopped = (
            abs(state.linear_speed_mps) <= self._config.stopped_speed_tolerance_mps
            and abs(yaw_rate_radps) <= self._config.stopped_yaw_rate_tolerance_radps
        )
        if self._phase in (
            PrecisionDockingPhase.NORMAL_NAV,
            PrecisionDockingPhase.PRECISION_APPROACH,
        ):
            return self._settle(now_s, stopped, position_error, heading_error)
        if self._phase == PrecisionDockingPhase.STOP_SETTLE:
            if not stopped:
                self._settle_started_s = None
                return self._decision(
                    self._phase,
                    position_error,
                    heading_error,
                    use_fixed_reference=True,
                )
            if self._settle_started_s is None:
                self._settle_started_s = now_s
            if now_s - self._settle_started_s < self._config.settle_time_s:
                return self._decision(
                    self._phase,
                    position_error,
                    heading_error,
                    use_fixed_reference=True,
                )
            return self._select_trim(
                now_s, state, checkpoint, position_error, heading_error
            )
        if self._phase == PrecisionDockingPhase.HEADING_TRIM:
            if abs(heading_error) <= self._config.heading_trim_target_rad:
                return self._settle(now_s, stopped, position_error, heading_error)
            if abs(heading_error) > self._config.max_spin_correction_rad:
                return self._alignment_hold(position_error, heading_error)
            yaw_rate = _bounded_signed(
                self._config.spin_gain * heading_error,
                self._config.spin_min_yaw_rate_radps,
                self._config.spin_max_yaw_rate_radps,
            )
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                mode=RequestedMotionMode.SPIN,
                yaw_rate_radps=yaw_rate,
                use_fixed_reference=True,
            )
        if self._phase == PrecisionDockingPhase.POSITION_TRIM:
            if abs(heading_error) > self._config.heading_realign_tolerance_rad:
                return self._settle(now_s, stopped, position_error, heading_error)
            if position_error <= self._config.final_position_tolerance_m:
                return self._settle(now_s, stopped, position_error, heading_error)
            if position_error > self._config.max_parallel_correction_m:
                return self._alignment_hold(position_error, heading_error)
            return self._parallel_command(
                state, checkpoint, position_error, heading_error
            )
        if self._phase == PrecisionDockingPhase.DOCK_READY:
            if (
                position_error > self._config.final_position_tolerance_m
                or abs(heading_error) > self._config.heading_realign_tolerance_rad
                or not stopped
            ):
                self._stable_started_s = None
                return self._settle(now_s, stopped, position_error, heading_error)
            if self._stable_started_s is None:
                self._stable_started_s = now_s
            return self._decision(
                self._phase,
                position_error,
                heading_error,
                use_fixed_reference=True,
                pose_ready=(
                    now_s - self._stable_started_s >= self._config.stable_time_s
                ),
            )
        if self._phase == PrecisionDockingPhase.ALIGNMENT_HOLD:
            if (
                abs(heading_error) <= self._config.max_spin_correction_rad
                and position_error <= self._config.max_parallel_correction_m
            ):
                return self._settle(now_s, stopped, position_error, heading_error)
            return self._alignment_hold(position_error, heading_error)
        return self._alignment_hold(position_error, heading_error)

    def _settle(
        self,
        now_s: float,
        stopped: bool,
        position_error: float,
        heading_error: float,
    ) -> PrecisionDockingDecision:
        self._phase = PrecisionDockingPhase.STOP_SETTLE
        self._stable_started_s = None
        self._settle_started_s = now_s if stopped else None
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
        )

    def _select_trim(
        self,
        now_s: float,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        position_error: float,
        heading_error: float,
    ) -> PrecisionDockingDecision:
        self._settle_started_s = None
        if abs(heading_error) > self._config.max_spin_correction_rad:
            return self._alignment_hold(position_error, heading_error)
        if abs(heading_error) > self._config.heading_realign_tolerance_rad:
            self._phase = PrecisionDockingPhase.HEADING_TRIM
            return self.update(
                now_s=now_s,
                state=state,
                checkpoint=checkpoint,
                yaw_rate_radps=0.0,
            )
        if position_error > self._config.max_parallel_correction_m:
            return self._alignment_hold(position_error, heading_error)
        if position_error > self._config.final_position_tolerance_m:
            self._phase = PrecisionDockingPhase.POSITION_TRIM
            return self._parallel_command(
                state, checkpoint, position_error, heading_error
            )
        self._phase = PrecisionDockingPhase.DOCK_READY
        self._stable_started_s = now_s
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
        )

    def _parallel_command(
        self,
        state: VehicleState,
        checkpoint: MissionCheckpoint,
        position_error: float,
        heading_error: float,
    ) -> PrecisionDockingDecision:
        dx = checkpoint.x - state.x
        dy = checkpoint.y - state.y
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        raw_x = self._config.parallel_gain * body_x
        raw_y = self._config.parallel_gain * body_y
        magnitude = math.hypot(raw_x, raw_y)
        if magnitude <= 1e-12:
            return self._alignment_hold(position_error, heading_error)
        commanded = min(
            self._config.parallel_max_speed_mps,
            max(self._config.parallel_min_speed_mps, magnitude),
        )
        scale = commanded / magnitude
        return self._decision(
            PrecisionDockingPhase.POSITION_TRIM,
            position_error,
            heading_error,
            mode=RequestedMotionMode.PARALLEL,
            linear_x_mps=raw_x * scale,
            linear_y_mps=raw_y * scale,
            use_fixed_reference=True,
        )

    def _alignment_hold(
        self,
        position_error: float,
        heading_error: float,
    ) -> PrecisionDockingDecision:
        self._phase = PrecisionDockingPhase.ALIGNMENT_HOLD
        return self._decision(
            self._phase,
            position_error,
            heading_error,
            use_fixed_reference=True,
        )

    @staticmethod
    def _decision(
        phase: PrecisionDockingPhase,
        position_error: float,
        heading_error: float,
        *,
        mode: RequestedMotionMode = RequestedMotionMode.DUAL_ACKERMANN,
        linear_x_mps: float = 0.0,
        linear_y_mps: float = 0.0,
        yaw_rate_radps: float = 0.0,
        use_fixed_reference: bool = False,
        pose_ready: bool = False,
    ) -> PrecisionDockingDecision:
        return PrecisionDockingDecision(
            phase=phase,
            motion_mode=mode,
            use_dynamic_replanning=not use_fixed_reference,
            use_fixed_reference=use_fixed_reference,
            linear_x_mps=linear_x_mps,
            linear_y_mps=linear_y_mps,
            yaw_rate_radps=yaw_rate_radps,
            pose_ready=pose_ready,
            position_error_m=position_error,
            heading_error_rad=heading_error,
        )


def _bounded_signed(value: float, minimum: float, maximum: float) -> float:
    if abs(value) <= 1e-12:
        return 0.0
    return math.copysign(min(maximum, max(minimum, abs(value))), value)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
