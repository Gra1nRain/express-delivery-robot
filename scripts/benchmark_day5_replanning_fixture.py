#!/usr/bin/env python3
"""Benchmark recorded local-replanning inputs without replaying ROS topics."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

import yaml

from competition_planning.local_trajectory_planner import (
    LocalReplanConfig,
    LocalTrajectoryPlanner,
    _crop_planning_map,
    _lookahead_index,
    _nearest_reference_index,
    _overlay_obstacles,
    _select_navigable_rejoin_index,
)
from competition_planning.occupancy_grid_planner import (
    GridAStarPlanner,
    GridPlanningError,
    OccupancyGridMap,
)
from competition_planning.semantic_planner import PathPoint


def config_from_mapping(
    replanning: Mapping[str, Any],
    *,
    max_expansions: int | None = None,
) -> LocalReplanConfig:
    return LocalReplanConfig(
        lookahead_distance_m=float(replanning.get("lookahead_distance_m", 5.0)),
        inflation_radius_m=float(replanning.get("inflation_radius_m", 0.45)),
        search_padding_m=float(replanning.get("search_padding_m", 3.0)),
        sample_spacing_m=float(replanning.get("sample_spacing_m", 0.10)),
        min_turning_radius_m=float(
            replanning.get("min_turning_radius_m", 0.81)
        ),
        step_length_m=float(replanning.get("step_length_m", 0.20)),
        curvature_bins=int(replanning.get("curvature_bins", 9)),
        heading_bins=int(replanning.get("heading_bins", 72)),
        goal_position_tolerance_m=float(
            replanning.get("goal_position_tolerance_m", 0.25)
        ),
        goal_heading_tolerance_rad=math.radians(
            float(replanning.get("goal_heading_tolerance_deg", 15.0))
        ),
        reference_deviation_weight=float(
            replanning.get("reference_deviation_weight", 2.0)
        ),
        max_expansions=(
            int(replanning.get("max_expansions", 250_000))
            if max_expansions is None
            else max_expansions
        ),
        reference_search_window_points=int(
            replanning.get("reference_search_window_points", 120)
        ),
    )


def _reference_path(path: Path) -> tuple[PathPoint, ...]:
    artifact = yaml.safe_load(path.read_text(encoding="utf-8"))
    return tuple(
        PathPoint(
            x=float(point["x"]),
            y=float(point["y"]),
            yaw=float(point["yaw"]),
            ref_id=str(point["ref_id"]) if point.get("ref_id") else None,
        )
        for point in artifact["points"]
    )


def grid_connectivity_check(
    static_map: OccupancyGridMap,
    config: LocalReplanConfig,
    reference_path: Sequence[PathPoint],
    current_pose: PathPoint,
    dynamic_obstacle_points: Sequence[tuple[float, float]],
    *,
    previous_reference_index: int,
) -> dict[str, object]:
    start_index = _nearest_reference_index(
        reference_path,
        current_pose,
        previous_reference_index=previous_reference_index,
        search_window_points=config.reference_search_window_points,
    )
    rejoin_index = _lookahead_index(
        reference_path,
        start_index,
        config.lookahead_distance_m,
    )
    local_reference = tuple(reference_path[start_index : rejoin_index + 1])
    planning_map = _crop_planning_map(
        static_map,
        current_pose,
        local_reference,
        margin_m=(
            config.search_padding_m
            + config.inflation_radius_m
            + config.step_length_m
        ),
    )
    live_map, _ = _overlay_obstacles(
        planning_map,
        dynamic_obstacle_points,
    )
    local_planner = LocalTrajectoryPlanner(static_map, config)
    hybrid_planner = local_planner._planner(live_map, local_reference)
    rejoin_index = _select_navigable_rejoin_index(
        reference_path,
        hybrid_planner,
        start_index=start_index,
        preferred_rejoin_index=rejoin_index,
    )
    goal = reference_path[rejoin_index]
    grid_planner = GridAStarPlanner(
        live_map,
        inflation_radius_m=config.inflation_radius_m,
        search_padding_m=config.search_padding_m,
        sample_spacing_m=config.sample_spacing_m,
        simplify_path=False,
    )
    started_at = time.perf_counter()
    try:
        grid_path = grid_planner.plan((current_pose, goal))
        result: dict[str, object] = {
            "status": "CONNECTED",
            "path_point_count": len(grid_path),
        }
    except GridPlanningError as exc:
        result = {
            "status": "DISCONNECTED",
            "detail": str(exc),
        }
    result["planning_time_ms"] = (time.perf_counter() - started_at) * 1000.0
    result["goal"] = {"x": goal.x, "y": goal.y, "yaw": goal.yaw}
    return result


def benchmark_fixture(
    fixture_path: Path,
    *,
    map_path: Path,
    trajectory_path: Path,
    planning_params_path: Path,
    previous_reference_index: int,
    current_pose: PathPoint | None = None,
    max_expansions: int | None = None,
) -> dict[str, object]:
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    params = yaml.safe_load(planning_params_path.read_text(encoding="utf-8"))
    config = config_from_mapping(
        params["replanning"],
        max_expansions=max_expansions,
    )
    static_map = OccupancyGridMap.from_yaml(map_path)
    reference_path = _reference_path(trajectory_path)
    results: list[dict[str, object]] = []

    for candidate in fixture["candidates"]:
        raw_pose = candidate.get("current_pose")
        pose = current_pose
        if pose is None and isinstance(raw_pose, dict):
            pose = PathPoint(
                x=float(raw_pose["x"]),
                y=float(raw_pose["y"]),
                yaw=float(raw_pose["yaw"]),
            )
        if pose is None:
            raise ValueError(
                "candidate has no current_pose; provide --current-pose"
            )

        obstacle_points = tuple(
            (float(point[0]), float(point[1]))
            for point in candidate["occupied_points_map"]
        )
        connectivity = grid_connectivity_check(
            static_map,
            config,
            reference_path,
            pose,
            obstacle_points,
            previous_reference_index=previous_reference_index,
        )
        planner = LocalTrajectoryPlanner(static_map, config)
        started_at = time.perf_counter()
        try:
            plan = planner.plan(
                reference_path=reference_path,
                current_pose=pose,
                dynamic_obstacle_points=obstacle_points,
                previous_reference_index=previous_reference_index,
            )
            result: dict[str, object] = {
                "status": plan.status,
                "reference_start_index": plan.reference_start_index,
                "rejoin_index": plan.rejoin_index,
                "dynamic_obstacle_count": plan.dynamic_obstacle_count,
                "path_point_count": len(plan.path),
            }
        except (GridPlanningError, ValueError) as exc:
            result = {
                "status": "PLAN_FAILED",
                "detail": str(exc),
            }
        result["planning_time_ms"] = (
            time.perf_counter() - started_at
        ) * 1000.0
        result["elapsed_s"] = float(candidate["elapsed_s"])
        result["occupied_point_count"] = len(
            candidate["occupied_points_map"]
        )
        result["grid_connectivity"] = connectivity
        results.append(result)

    return {
        "fixture": str(fixture_path),
        "max_expansions": config.max_expansions,
        "previous_reference_index": previous_reference_index,
        "results": results,
    }


def _pose(value: str) -> PathPoint:
    fields = value.split(",")
    if len(fields) != 3:
        raise argparse.ArgumentTypeError("pose must be x,y,yaw")
    try:
        return PathPoint(*(float(field) for field in fields))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("pose must contain numbers") from exc


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--planning-params", required=True, type=Path)
    parser.add_argument("--previous-reference-index", required=True, type=int)
    parser.add_argument("--current-pose", type=_pose)
    parser.add_argument("--max-expansions", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    report = benchmark_fixture(
        args.fixture,
        map_path=args.map,
        trajectory_path=args.trajectory,
        planning_params_path=args.planning_params,
        previous_reference_index=args.previous_reference_index,
        current_pose=args.current_pose,
        max_expansions=args.max_expansions,
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    for result in report["results"]:
        print(
            f"elapsed_s={result['elapsed_s']:.3f} "
            f"status={result['status']} "
            f"planning_ms={result['planning_time_ms']:.3f} "
            f"grid={result['grid_connectivity']['status']} "
            f"grid_ms={result['grid_connectivity']['planning_time_ms']:.3f} "
            f"detail={result.get('detail', '')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
