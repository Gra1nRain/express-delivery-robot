"""Task-stop checkpoints resolved independently from motion segmentation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from competition_control.mppi_controller import ControlTrajectory


@dataclass(frozen=True)
class MissionCheckpoint:
    ref_id: str
    x: float
    y: float
    yaw: float


def mission_checkpoints_from_route(
    route: dict[str, Any],
    semantic_map: dict[str, Any],
    trajectory: ControlTrajectory,
) -> tuple[MissionCheckpoint, ...]:
    """Resolve ordered task stops while verifying that the path visits them."""

    raw_refs = route.get("mission_checkpoints", [])
    if raw_refs is None:
        return ()
    if not isinstance(raw_refs, list):
        raise ValueError("mission_checkpoints must be a list")
    points = semantic_map.get("points", {})
    if not isinstance(points, dict):
        raise ValueError("semantic map points must be a mapping")

    checkpoints: list[MissionCheckpoint] = []
    trajectory_index = 0
    for raw_ref in raw_refs:
        if not isinstance(raw_ref, str) or not raw_ref:
            raise ValueError("mission checkpoint refs must be non-empty strings")
        matching_index = next(
            (
                index
                for index in range(trajectory_index, len(trajectory.points))
                if trajectory.points[index].ref_id == raw_ref
            ),
            None,
        )
        if matching_index is None:
            raise ValueError(
                f"continuous trajectory does not visit mission checkpoint {raw_ref}"
            )
        raw_point = points.get(raw_ref)
        if not isinstance(raw_point, dict):
            raise ValueError(f"semantic map has no point for checkpoint {raw_ref}")
        try:
            values = (
                float(raw_point["x"]),
                float(raw_point["y"]),
                float(raw_point.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid semantic checkpoint {raw_ref}") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"semantic checkpoint {raw_ref} is non-finite")
        checkpoints.append(
            MissionCheckpoint(
                ref_id=raw_ref,
                x=values[0],
                y=values[1],
                yaw=values[2],
            )
        )
        trajectory_index = matching_index + 1
    return tuple(checkpoints)
