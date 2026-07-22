#!/usr/bin/env python3
"""Deterministic no-hardware validation of the complete motion chain."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import sys
from typing import Any, Sequence

import yaml

from competition_control.mppi_controller import (
    ControlTrajectory,
    MPPIController,
    MPPIParams,
    VehicleState,
)
from competition_localization.state_estimator import (
    Pose2D,
    StateEstimator,
    StateEstimatorLimits,
    StateObservation,
    Velocity2D,
)
from competition_safety.supervisor import (
    SafetyContext,
    SafetyLimits,
    SafetySupervisor,
)


@dataclass(frozen=True)
class SimulationConfig:
    dt_s: float = 0.05
    max_steps: int = 20_000
    random_seed: int = 7


@dataclass(frozen=True)
class MotionValidationReport:
    route_name: str
    completed: bool
    steps: int
    simulated_duration_s: float
    max_speed_mps: float
    max_abs_yaw_rate_radps: float
    max_abs_lateral_error_m: float
    max_abs_heading_error_rad: float
    final_position_error_m: float
    final_speed_mps: float
    safe_hold_count: int
    safe_limited_count: int
    failure_reason: str | None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def simulate_motion_chain(
    trajectory: ControlTrajectory,
    controller_params: MPPIParams,
    safety_limits: SafetyLimits,
    config: SimulationConfig,
) -> MotionValidationReport:
    """Exercise estimator, MPPI, safety, and Ranger curvature kinematics."""

    if config.dt_s <= 0.0:
        raise ValueError("simulation dt_s must be positive")
    controller = MPPIController(
        trajectory,
        controller_params,
        random_seed=config.random_seed,
    )
    safety = SafetySupervisor(safety_limits)
    estimator = StateEstimator(StateEstimatorLimits())
    first = trajectory.points[0]
    x = first.x
    y = first.y
    yaw = first.yaw
    speed = 0.0
    yaw_rate = 0.0
    max_speed = 0.0
    max_yaw_rate = 0.0
    max_lateral = 0.0
    max_heading = 0.0
    hold_count = 0
    limited_count = 0
    completed = False
    failure_reason: str | None = None
    executed_steps = 0

    for step in range(config.max_steps):
        executed_steps = step + 1
        now_s = 1.0 + step * config.dt_s
        estimate = estimator.update(
            StateObservation(
                pose=Pose2D(x, y, yaw),
                velocity=Velocity2D(speed, yaw_rate),
                pose_stamp_s=now_s,
                velocity_stamp_s=now_s,
            ),
            now_s=now_s,
        )
        command = controller.compute_command(
            VehicleState(
                x=estimate.pose.x,
                y=estimate.pose.y,
                yaw=estimate.pose.yaw,
                linear_speed_mps=estimate.velocity.linear_x_mps,
            )
        )
        safe = safety.filter_command(
            command,
            SafetyContext(
                now_s=now_s,
                command_stamp_s=now_s,
                state_stamp_s=now_s,
                measured_speed_mps=speed,
                estop_ready=True,
                remote_ready=True,
                state_valid=estimate.valid,
                avoidance_stop=False,
                chassis_fault=False,
                system_ready=True,
                ackermann_mode=True,
            ),
        )
        max_lateral = max(max_lateral, abs(command.lateral_error_m))
        max_heading = max(max_heading, abs(command.heading_error_rad))
        if safe.status == "SAFE_HOLD":
            hold_count += 1
            failure_reason = ",".join(safe.reasons) or "safe_hold"
            speed = 0.0
            yaw_rate = 0.0
            break
        if safe.status == "SAFE_LIMITED":
            limited_count += 1
        speed = safe.linear_x_mps
        yaw_rate = safe.yaw_rate_radps
        max_speed = max(max_speed, abs(speed))
        max_yaw_rate = max(max_yaw_rate, abs(yaw_rate))
        if safe.status == "SAFE_STOP":
            completed = True
            speed = 0.0
            yaw_rate = 0.0
            break
        yaw_mid = yaw + 0.5 * yaw_rate * config.dt_s
        x += speed * math.cos(yaw_mid) * config.dt_s
        y += speed * math.sin(yaw_mid) * config.dt_s
        yaw = _wrap_angle(yaw + yaw_rate * config.dt_s)
    else:
        failure_reason = "simulation_step_limit"

    final = trajectory.points[-1]
    return MotionValidationReport(
        route_name=trajectory.route_name,
        completed=completed,
        steps=executed_steps,
        simulated_duration_s=executed_steps * config.dt_s,
        max_speed_mps=max_speed,
        max_abs_yaw_rate_radps=max_yaw_rate,
        max_abs_lateral_error_m=max_lateral,
        max_abs_heading_error_rad=max_heading,
        final_position_error_m=math.hypot(x - final.x, y - final.y),
        final_speed_mps=speed,
        safe_hold_count=hold_count,
        safe_limited_count=limited_count,
        failure_reason=failure_reason,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--control-params", required=True)
    parser.add_argument("--safety-params", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report")
    args = parser.parse_args(argv)

    trajectory = ControlTrajectory.from_dict(_load_yaml(args.trajectory))
    control = _load_yaml(args.control_params).get("trajectory_tracker", {})
    mppi = control.get("mppi", {})
    motion = _load_yaml(args.control_params).get("motion", {})
    safety_config = _load_yaml(args.safety_params).get("safety", {})
    dt_s = 1.0 / float(control.get("frequency_hz", 20.0))
    report = simulate_motion_chain(
        trajectory,
        MPPIParams(
            control_dt_s=dt_s,
            horizon_steps=int(mppi.get("horizon_steps", 30)),
            rollout_count=int(mppi.get("rollout_count", 768)),
            iterations=int(mppi.get("iterations", 2)),
            temperature=float(mppi.get("temperature", 0.35)),
            speed_noise_std_mps=float(mppi.get("speed_noise_std_mps", 0.05)),
            curvature_noise_std_1pm=float(
                mppi.get("curvature_noise_std_1pm", 0.25)
            ),
            max_speed_mps=float(motion.get("max_speed_mps", 0.20)),
            max_acceleration_mps2=float(motion.get("max_acceleration_mps2", 0.20)),
            max_deceleration_mps2=float(motion.get("max_deceleration_mps2", 0.30)),
            min_turning_radius_m=float(motion.get("min_turning_radius_m", 0.81)),
            max_curvature_rate_1pmps=float(
                motion.get("max_curvature_rate_1pmps", 0.80)
            ),
            progress_search_window_points=int(
                mppi.get("progress_search_window_points", 40)
            ),
            max_progress_advance_points=int(
                mppi.get("max_progress_advance_points", 3)
            ),
            lateral_feedback_gain_1pm_per_m=float(
                mppi.get("lateral_feedback_gain_1pm_per_m", 1.5)
            ),
            heading_feedback_gain_1pm_per_rad=float(
                mppi.get("heading_feedback_gain_1pm_per_rad", 1.0)
            ),
            feedback_blend=float(mppi.get("feedback_blend", 0.35)),
        ),
        SafetyLimits(
            command_timeout_s=float(safety_config.get("command_timeout_s", 0.15)),
            state_timeout_s=float(safety_config.get("state_timeout_s", 0.15)),
            max_speed_mps=float(safety_config.get("max_speed_mps", 0.20)),
            max_acceleration_mps2=float(
                safety_config.get("max_acceleration_mps2", 0.20)
            ),
            max_deceleration_mps2=float(
                safety_config.get("max_deceleration_mps2", 0.30)
            ),
            min_turning_radius_m=float(
                safety_config.get("min_turning_radius_m", 0.81)
            ),
            max_lateral_error_m=float(
                safety_config.get("max_lateral_error_m", 0.15)
            ),
            max_heading_error_rad=math.radians(
                float(safety_config.get("max_heading_error_deg", 20.0))
            ),
            nominal_period_s=dt_s,
        ),
        SimulationConfig(dt_s=dt_s),
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report.to_dict(), stream, sort_keys=False, allow_unicode=True)
    if args.report:
        _write_report(report, Path(args.report))
    print(
        f"route={report.route_name} completed={report.completed} steps={report.steps} "
        f"max_lateral_error_m={report.max_abs_lateral_error_m:.3f} "
        f"safe_holds={report.safe_hold_count}"
    )
    return 0 if report.completed and report.safe_hold_count == 0 else 2


def _load_yaml(path: str) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def _write_report(report: MotionValidationReport, path: Path) -> None:
    lines = [
        "# Day 5 离线运动闭环报告",
        "",
        "## 事实",
        "",
        *[f"- {key}: {value}" for key, value in report.to_dict().items()],
        "",
        "## 未验证",
        "",
        "- 该结果来自确定性运动学仿真，不包含实车执行延迟、轮胎侧偏和地面扰动。",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    sys.exit(main())
