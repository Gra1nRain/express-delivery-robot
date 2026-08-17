"""Load the mission state machine's semantic and timing configuration."""

from __future__ import annotations

from typing import Any

from competition_mission.mission_state_machine import MissionConfig


def mission_config_from_dict(document: dict[str, Any]) -> MissionConfig:
    mission = _mapping(document.get("mission", document), "mission")
    refs = _mapping(mission.get("refs", {}), "mission.refs")
    traffic = _mapping(mission.get("traffic", {}), "mission.traffic")
    arm = _mapping(mission.get("arm", {}), "mission.arm")
    pickup = _mapping(arm.get("pickup", {}), "mission.arm.pickup")
    drop = _mapping(arm.get("drop", {}), "mission.arm.drop")
    defaults = MissionConfig()
    return MissionConfig(
        traffic_marker_ref=str(
            refs.get("traffic_marker", defaults.traffic_marker_ref)
        ),
        traffic_checkpoint_ref=str(
            refs.get("traffic_stop", defaults.traffic_checkpoint_ref)
        ),
        pickup_front_ref=str(
            refs.get("pickup_front", defaults.pickup_front_ref)
        ),
        pickup_rear_ref=str(refs.get("pickup_rear", defaults.pickup_rear_ref)),
        drop_front_ref=str(refs.get("drop_front", defaults.drop_front_ref)),
        drop_rear_ref=str(refs.get("drop_rear", defaults.drop_rear_ref)),
        finish_ref=str(refs.get("finish", defaults.finish_ref)),
        traffic_no_result_timeout_s=float(
            traffic.get(
                "no_result_timeout_s",
                defaults.traffic_no_result_timeout_s,
            )
        ),
        pickup_timeout_s=float(
            pickup.get("total_timeout_s", defaults.pickup_timeout_s)
        ),
        drop_timeout_s=float(drop.get("total_timeout_s", defaults.drop_timeout_s)),
        pickup_max_attempts=int(
            pickup.get("max_attempts", defaults.pickup_max_attempts)
        ),
        drop_max_attempts=int(drop.get("max_attempts", defaults.drop_max_attempts)),
    )


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value
