#!/usr/bin/env python3
import math

import numpy as np


def _wrap_degrees(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _rotation_from_rpy_xyz(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def robust_point_cloud_box_center(
    points,
    lower_quantile=0.05,
    upper_quantile=0.95,
):
    points_array = np.asarray(points, dtype=np.float64)
    if points_array.ndim != 2 or points_array.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if len(points_array) == 0:
        raise ValueError("points must not be empty")

    bounds_min = np.quantile(
        points_array,
        float(lower_quantile),
        axis=0,
    )
    bounds_max = np.quantile(
        points_array,
        float(upper_quantile),
        axis=0,
    )
    center = (bounds_min + bounds_max) * 0.5
    return center, bounds_min, bounds_max


def build_block_rpy_candidates(
    roll_deg,
    fallback_yaw_deg,
    object_yaw_deg,
    pitch_candidates_deg,
    yaw_offset_candidates_deg=(0.0,),
):
    primary_yaw = (
        float(fallback_yaw_deg)
        if object_yaw_deg is None
        else float(object_yaw_deg)
    )
    candidates = []
    for pitch_deg in pitch_candidates_deg:
        for yaw_offset_deg in yaw_offset_candidates_deg:
            offset_yaw = primary_yaw + float(yaw_offset_deg)
            yaw_candidates = (
                _wrap_degrees(offset_yaw),
                _wrap_degrees(offset_yaw + 180.0),
            )
            for yaw_deg in yaw_candidates:
                candidates.append(
                    {
                        "name": (
                            f"pitch_{float(pitch_deg):.1f}_"
                            f"yaw_{float(yaw_deg):.1f}"
                        ),
                        "roll_deg": float(roll_deg),
                        "pitch_deg": float(pitch_deg),
                        "yaw_deg": float(yaw_deg),
                        "yaw_offset_deg": float(yaw_offset_deg),
                        "yaw_source": (
                            "object_axis"
                            if object_yaw_deg is not None
                            else "calibrated_fallback"
                        ),
                    }
                )
    return candidates


def build_tool_axis_pregrasp_pose(grasp_pose, backoff_m):
    result = dict(grasp_pose)
    tool_z_base = _rotation_from_rpy_xyz(
        float(grasp_pose["roll"]),
        float(grasp_pose["pitch"]),
        float(grasp_pose["yaw"]),
    )[:, 2]
    grasp_xyz = np.array(
        [
            float(grasp_pose["x"]),
            float(grasp_pose["y"]),
            float(grasp_pose["z"]),
        ],
        dtype=np.float64,
    )
    pregrasp_xyz = grasp_xyz - tool_z_base * float(backoff_m)
    result.update(
        {
            "x": float(pregrasp_xyz[0]),
            "y": float(pregrasp_xyz[1]),
            "z": float(pregrasp_xyz[2]),
        }
    )
    return result


def build_world_yz_pregrasp_pose(
    grasp_pose,
    backoff_y_m,
    lift_z_m,
):
    """Build a fixed-RPY pregrasp that enters along world -Y and -Z."""
    result = dict(grasp_pose)
    result["y"] = float(grasp_pose["y"]) + float(backoff_y_m)
    result["z"] = float(grasp_pose["z"]) + float(lift_z_m)
    return result


def choose_reachable_block_candidate(
    evaluations,
    minimum_joint_margin_rad,
):
    accepted = [
        evaluation
        for evaluation in evaluations
        if float(evaluation["minimum_joint_margin_rad"])
        >= float(minimum_joint_margin_rad)
    ]
    if not accepted:
        best_margin = max(
            (
                float(evaluation["minimum_joint_margin_rad"])
                for evaluation in evaluations
            ),
            default=float("-inf"),
        )
        raise RuntimeError(
            "方块抓取候选均未达到最小关节余量: "
            f"best_margin={best_margin:.3f}rad < "
            f"required={float(minimum_joint_margin_rad):.3f}rad"
        )

    return min(
        accepted,
        key=lambda evaluation: (
            abs(float(evaluation.get("yaw_offset_deg", 0.0))),
            abs(float(evaluation["pitch_deg"])),
            -float(evaluation["minimum_joint_margin_rad"]),
            float(evaluation["max_joint_step_rad"]),
            float(evaluation["max_joint_travel_rad"]),
        ),
    )
