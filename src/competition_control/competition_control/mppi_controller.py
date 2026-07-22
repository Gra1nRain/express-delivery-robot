"""Sampling-based MPPI trajectory tracker for the Ranger four-wheel chassis."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class ControlTrajectoryPoint:
    x: float
    y: float
    yaw: float
    s: float
    curvature: float
    v: float
    t: float
    ref_id: str | None = None


@dataclass(frozen=True)
class ControlTrajectory:
    frame_id: str
    route_name: str
    points: tuple[ControlTrajectoryPoint, ...]

    @classmethod
    def from_dict(cls, artifact: dict[str, Any]) -> "ControlTrajectory":
        raw_points = artifact.get("points", [])
        if not isinstance(raw_points, list) or len(raw_points) < 2:
            raise ValueError("continuous trajectory artifact requires at least two points")
        points = tuple(
            ControlTrajectoryPoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point["yaw"]),
                s=float(point["s"]),
                curvature=float(point["curvature"]),
                v=float(point["v"]),
                t=float(point["t"]),
                ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
            )
            for point in raw_points
        )
        return cls(
            frame_id=str(artifact.get("frame_id", "map")),
            route_name=str(artifact.get("route_name", "")),
            points=points,
        )

@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    yaw: float
    linear_speed_mps: float


@dataclass(frozen=True)
class BodyCommand:
    linear_x_mps: float
    yaw_rate_radps: float
    curvature_1pm: float
    target_index: int
    lateral_error_m: float
    heading_error_rad: float
    status: str

    @classmethod
    def hold(
        cls,
        *,
        target_index: int,
        lateral_error_m: float,
        heading_error_rad: float,
        status: str,
    ) -> "BodyCommand":
        return cls(
            linear_x_mps=0.0,
            yaw_rate_radps=0.0,
            curvature_1pm=0.0,
            target_index=target_index,
            lateral_error_m=lateral_error_m,
            heading_error_rad=heading_error_rad,
            status=status,
        )


@dataclass(frozen=True)
class MPPIParams:
    control_dt_s: float = 0.05
    horizon_steps: int = 30
    rollout_count: int = 768
    iterations: int = 2
    temperature: float = 0.35
    speed_noise_std_mps: float = 0.05
    curvature_noise_std_1pm: float = 0.25
    max_speed_mps: float = 0.20
    max_acceleration_mps2: float = 0.20
    max_deceleration_mps2: float = 0.30
    min_turning_radius_m: float = 0.81
    max_curvature_rate_1pmps: float = 0.80
    goal_position_tolerance_m: float = 0.10
    recovery_lateral_error_m: float = 0.30
    recovery_heading_error_rad: float = math.radians(65.0)
    position_weight: float = 45.0
    heading_weight: float = 12.0
    speed_weight: float = 4.0
    curvature_weight: float = 0.8
    control_smoothness_weight: float = 2.0
    terminal_weight: float = 80.0
    progress_search_window_points: int = 40
    max_progress_advance_points: int = 3
    lateral_feedback_gain_1pm_per_m: float = 1.5
    heading_feedback_gain_1pm_per_rad: float = 1.0
    feedback_blend: float = 0.35


class MPPIController:
    """Optimize bounded speed/curvature sequences through one small interface."""

    def __init__(
        self,
        trajectory: ControlTrajectory,
        params: MPPIParams,
        *,
        random_seed: int | None = None,
    ) -> None:
        if len(trajectory.points) < 2:
            raise ValueError("MPPI requires at least two trajectory points")
        if params.min_turning_radius_m <= 0.0:
            raise ValueError("min_turning_radius_m must be positive")
        if params.control_dt_s <= 0.0:
            raise ValueError("control_dt_s must be positive")
        self._trajectory = trajectory
        self._params = params
        self._rng = np.random.default_rng(random_seed)
        self._xy = np.array([(point.x, point.y) for point in trajectory.points], dtype=float)
        self._yaw = np.array([point.yaw for point in trajectory.points], dtype=float)
        self._s = np.array([point.s for point in trajectory.points], dtype=float)
        self._time = np.array([point.t for point in trajectory.points], dtype=float)
        self._speed = np.array([point.v for point in trajectory.points], dtype=float)
        self._curvature = np.array(
            [point.curvature for point in trajectory.points],
            dtype=float,
        )
        if np.any(np.abs(self._curvature) > 1.0 / params.min_turning_radius_m + 1e-6):
            raise ValueError("trajectory exceeds the runtime turning-radius envelope")
        if np.any(np.diff(self._s) <= 0.0) or np.any(np.diff(self._time) <= 0.0):
            raise ValueError("trajectory distance and time must be strictly increasing")
        self._nominal = np.zeros((params.horizon_steps, 2), dtype=float)
        self._progress_index = 0
        self._last_curvature = 0.0

    def reset(self) -> None:
        self._nominal.fill(0.0)
        self._progress_index = 0
        self._last_curvature = 0.0

    def compute_command(self, state: VehicleState) -> BodyCommand:
        nearest = self._nearest_index(state)
        reference_indices = self._reference_indices(nearest)
        lateral_error, heading_error = self._tracking_errors(state, nearest)
        goal_distance = math.hypot(
            state.x - self._xy[-1, 0],
            state.y - self._xy[-1, 1],
        )
        if nearest >= len(self._trajectory.points) - 2 and goal_distance <= self._params.goal_position_tolerance_m:
            return BodyCommand.hold(
                target_index=len(self._trajectory.points) - 1,
                lateral_error_m=lateral_error,
                heading_error_rad=heading_error,
                status="GOAL_REACHED",
            )
        if (
            abs(lateral_error) > self._params.recovery_lateral_error_m
            or abs(heading_error) > self._params.recovery_heading_error_rad
        ):
            return BodyCommand.hold(
                target_index=nearest,
                lateral_error_m=lateral_error,
                heading_error_rad=heading_error,
                status="RECOVERY_REQUIRED",
            )

        self._warm_start(reference_indices, lateral_error, heading_error)
        for _ in range(max(1, self._params.iterations)):
            noise = self._sample_noise()
            candidates = self._nominal[np.newaxis, :, :] + noise
            candidates = self._enforce_controls(candidates, state.linear_speed_mps)
            costs = self._rollout_costs(state, candidates, reference_indices)
            normalized = costs - np.min(costs)
            weights = np.exp(-normalized / max(self._params.temperature, 1e-6))
            weight_sum = float(np.sum(weights))
            if not math.isfinite(weight_sum) or weight_sum <= 1e-12:
                break
            weights /= weight_sum
            effective_noise = candidates - self._nominal[np.newaxis, :, :]
            self._nominal += np.sum(weights[:, np.newaxis, np.newaxis] * effective_noise, axis=0)
            self._nominal = self._enforce_controls(
                self._nominal[np.newaxis, :, :],
                state.linear_speed_mps,
            )[0]

        speed = float(self._nominal[0, 0])
        feedback_curvature = self._feedback_curvature(lateral_error, heading_error)
        blend = self._feedback_blend()
        stabilized_curvature = (
            float(self._curvature[int(reference_indices[0])]) + feedback_curvature
        )
        curvature = (
            (1.0 - blend) * float(self._nominal[0, 1])
            + blend * stabilized_curvature
        )
        max_curvature = 1.0 / self._params.min_turning_radius_m
        max_curvature_step = (
            self._params.max_curvature_rate_1pmps * self._params.control_dt_s
        )
        curvature = min(
            max_curvature,
            self._last_curvature + max_curvature_step,
            max(
                -max_curvature,
                self._last_curvature - max_curvature_step,
                curvature,
            ),
        )
        self._nominal[0, 1] = curvature
        yaw_rate = speed * curvature
        self._last_curvature = curvature
        self._nominal[:-1] = self._nominal[1:]
        self._nominal[-1] = self._nominal[-2]
        return BodyCommand(
            linear_x_mps=speed,
            yaw_rate_radps=yaw_rate,
            curvature_1pm=curvature,
            target_index=int(reference_indices[0]),
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            status="TRACKING",
        )

    def _nearest_index(self, state: VehicleState) -> int:
        start = max(0, self._progress_index - 3)
        end = min(
            len(self._xy),
            self._progress_index + max(1, self._params.progress_search_window_points) + 1,
        )
        offsets = self._xy[start:end] - np.array([state.x, state.y])
        nearest = start + int(np.argmin(np.sum(offsets * offsets, axis=1)))
        nearest = min(
            nearest,
            self._progress_index + max(1, self._params.max_progress_advance_points),
        )
        self._progress_index = max(self._progress_index, nearest)
        return self._progress_index

    def _reference_indices(self, nearest: int) -> np.ndarray:
        timestamps = self._time[nearest] + self._params.control_dt_s * (
            np.arange(self._params.horizon_steps) + 1
        )
        return np.clip(
            np.searchsorted(self._time, timestamps, side="left"),
            0,
            len(self._time) - 1,
        )

    def _tracking_errors(self, state: VehicleState, index: int) -> tuple[float, float]:
        dx = state.x - self._xy[index, 0]
        dy = state.y - self._xy[index, 1]
        reference_yaw = self._yaw[index]
        lateral = -math.sin(reference_yaw) * dx + math.cos(reference_yaw) * dy
        heading = _wrap_angle(state.yaw - reference_yaw)
        return float(lateral), float(heading)

    def _warm_start(
        self,
        reference_indices: np.ndarray,
        lateral_error: float,
        heading_error: float,
    ) -> None:
        if not np.any(self._nominal):
            self._nominal[:, 0] = self._speed[reference_indices]
            self._nominal[:, 1] = self._curvature[reference_indices]
        feedback_curvature = self._feedback_curvature(lateral_error, heading_error)
        max_curvature = 1.0 / self._params.min_turning_radius_m
        desired_curvature = np.clip(
            self._curvature[reference_indices] + feedback_curvature,
            -max_curvature,
            max_curvature,
        )
        blend = self._feedback_blend()
        self._nominal[:, 1] = (
            (1.0 - blend) * self._nominal[:, 1] + blend * desired_curvature
        )

    def _feedback_curvature(
        self,
        lateral_error: float,
        heading_error: float,
    ) -> float:
        return -(
            self._params.lateral_feedback_gain_1pm_per_m * lateral_error
            + self._params.heading_feedback_gain_1pm_per_rad * heading_error
        )

    def _feedback_blend(self) -> float:
        return min(1.0, max(0.0, self._params.feedback_blend))

    def _sample_noise(self) -> np.ndarray:
        noise = self._rng.normal(
            size=(self._params.rollout_count, self._params.horizon_steps, 2)
        )
        noise[:, :, 0] *= self._params.speed_noise_std_mps
        noise[:, :, 1] *= self._params.curvature_noise_std_1pm
        noise[0, :, :] = 0.0
        return noise

    def _enforce_controls(self, controls: np.ndarray, current_speed: float) -> np.ndarray:
        bounded = np.array(controls, copy=True)
        max_curvature = 1.0 / self._params.min_turning_radius_m
        bounded[:, :, 0] = np.clip(bounded[:, :, 0], 0.0, self._params.max_speed_mps)
        bounded[:, :, 1] = np.clip(bounded[:, :, 1], -max_curvature, max_curvature)
        previous_speed = np.full(bounded.shape[0], max(0.0, current_speed))
        previous_curvature = np.full(bounded.shape[0], self._last_curvature)
        accel_step = self._params.max_acceleration_mps2 * self._params.control_dt_s
        decel_step = self._params.max_deceleration_mps2 * self._params.control_dt_s
        curvature_step = self._params.max_curvature_rate_1pmps * self._params.control_dt_s
        for index in range(self._params.horizon_steps):
            bounded[:, index, 0] = np.minimum(
                previous_speed + accel_step,
                np.maximum(previous_speed - decel_step, bounded[:, index, 0]),
            )
            bounded[:, index, 1] = np.clip(
                bounded[:, index, 1],
                previous_curvature - curvature_step,
                previous_curvature + curvature_step,
            )
            previous_speed = bounded[:, index, 0]
            previous_curvature = bounded[:, index, 1]
        return bounded

    def _rollout_costs(
        self,
        state: VehicleState,
        controls: np.ndarray,
        reference_indices: np.ndarray,
    ) -> np.ndarray:
        count = controls.shape[0]
        x = np.full(count, state.x)
        y = np.full(count, state.y)
        yaw = np.full(count, state.yaw)
        costs = np.zeros(count)
        previous_controls = np.column_stack(
            [
                np.full(count, max(0.0, state.linear_speed_mps)),
                np.full(count, self._last_curvature),
            ]
        )
        dt = self._params.control_dt_s
        for step, reference_index in enumerate(reference_indices):
            speed = controls[:, step, 0]
            curvature = controls[:, step, 1]
            yaw_mid = yaw + 0.5 * speed * curvature * dt
            x += speed * np.cos(yaw_mid) * dt
            y += speed * np.sin(yaw_mid) * dt
            yaw = _wrap_angle_array(yaw + speed * curvature * dt)

            dx = x - self._xy[reference_index, 0]
            dy = y - self._xy[reference_index, 1]
            heading_error = _wrap_angle_array(yaw - self._yaw[reference_index])
            speed_error = speed - self._speed[reference_index]
            curvature_error = curvature - self._curvature[reference_index]
            delta_controls = controls[:, step, :] - previous_controls
            costs += (
                self._params.position_weight * (dx * dx + dy * dy)
                + self._params.heading_weight * heading_error * heading_error
                + self._params.speed_weight * speed_error * speed_error
                + self._params.curvature_weight * curvature_error * curvature_error
                + self._params.control_smoothness_weight
                * np.sum(delta_controls * delta_controls, axis=1)
            )
            previous_controls = controls[:, step, :]

        terminal_index = int(reference_indices[-1])
        terminal_dx = x - self._xy[terminal_index, 0]
        terminal_dy = y - self._xy[terminal_index, 1]
        costs += self._params.terminal_weight * (
            terminal_dx * terminal_dx + terminal_dy * terminal_dy
        )
        return costs


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_angle_array(values: np.ndarray) -> np.ndarray:
    return (values + math.pi) % (2.0 * math.pi) - math.pi
