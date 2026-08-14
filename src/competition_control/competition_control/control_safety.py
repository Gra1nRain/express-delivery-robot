"""Safety policy shared by the ROS adapter and log-replay regression tests."""

from collections.abc import Mapping


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
