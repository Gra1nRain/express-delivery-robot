"""Safety policy shared by the ROS adapter and log-replay regression tests."""

from collections.abc import Mapping
from dataclasses import dataclass


@dataclass(frozen=True)
class AlignmentGateDecision:
    hold_requested: bool
    completion_allowed: bool
    request_checkpoint: bool
    reason: str | None


def alignment_gate_decision(
    *,
    required: bool,
    route_enabled: bool,
    status_age_s: float | None,
    status_timeout_s: float,
    startup_ready: bool,
    checkpoint_hold: bool,
    checkpoint_ready_ref: str | None,
    active_checkpoint_ref: str | None,
    dock_hold_reached: bool,
) -> AlignmentGateDecision:
    if not required or not route_enabled:
        return AlignmentGateDecision(False, True, False, None)
    if (
        status_age_s is None
        or status_age_s < 0.0
        or status_age_s > status_timeout_s
    ):
        return AlignmentGateDecision(True, False, False, "alignment_status_stale")
    if not startup_ready:
        return AlignmentGateDecision(True, False, False, "startup_alignment_not_ready")
    if checkpoint_hold:
        return AlignmentGateDecision(True, False, False, "checkpoint_alignment_hold")
    checkpoint_ready = (
        active_checkpoint_ref is not None
        and checkpoint_ready_ref == active_checkpoint_ref
    )
    return AlignmentGateDecision(
        hold_requested=False,
        completion_allowed=checkpoint_ready,
        request_checkpoint=dock_hold_reached and not checkpoint_ready,
        reason=None,
    )


def update_local_hard_stop_latch(
    current: bool,
    status: Mapping[str, object],
) -> bool:
    if status.get("stop_requested") is False:
        return False
    if current:
        return True
    detail = status.get("detail")
    return (
        status.get("stop_requested") is True
        and status.get("status") == "HYBRID_ASTAR_NO_FEASIBLE_PATH"
        and isinstance(detail, str)
        and detail.startswith("start pose (")
        and detail.endswith(") is blocked")
    )


def segmented_safety_stop_requested(
    *,
    replanning_enabled: bool,
    precision_active: bool,
    local_stop_requested: bool,
    local_hard_stop_requested: bool,
    local_plan_stale: bool,
    avoidance_stop_requested: bool,
) -> bool:
    return (
        avoidance_stop_requested
        or (replanning_enabled and local_hard_stop_requested)
        or (
            replanning_enabled
            and not precision_active
            and (local_stop_requested or local_plan_stale)
        )
    )
