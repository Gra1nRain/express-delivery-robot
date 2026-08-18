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


def project_yolo_bbox_center_with_robust_depth(
    object_points_cam,
    object_pixels,
    target_bbox,
    camera_matrix,
    center_fraction=0.35,
    min_depth_points=12,
):
    """Back-project the YOLO box center using robust central object depth."""
    points = np.asarray(object_points_cam, dtype=np.float64)
    pixels = np.asarray(object_pixels, dtype=np.float64)
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    bbox = np.asarray(target_bbox, dtype=np.float64).reshape(-1)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("object_points_cam must have shape (N, 3)")
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError("object_pixels must have shape (N, 2)")
    if len(points) == 0 or len(points) != len(pixels):
        raise ValueError("object points and pixels must be non-empty and aligned")
    if intrinsics.shape != (3, 3):
        raise ValueError("camera_matrix must have shape (3, 3)")
    if bbox.size != 4:
        raise ValueError("target_bbox must contain x0, y0, x1, y1")

    x0, y0, x1, y1 = bbox
    if x1 <= x0 or y1 <= y0:
        raise ValueError("target_bbox must have positive width and height")
    fraction = float(center_fraction)
    if not 0.0 < fraction <= 1.0:
        raise ValueError("center_fraction must be in (0, 1]")

    center_u = float((x0 + x1) * 0.5)
    center_v = float((y0 + y1) * 0.5)
    half_width = max(2.0, float(x1 - x0) * fraction * 0.5)
    half_height = max(2.0, float(y1 - y0) * fraction * 0.5)
    valid_depth = np.isfinite(points[:, 2]) & (points[:, 2] > 0.0)
    central_mask = (
        valid_depth
        & (np.abs(pixels[:, 0] - center_u) <= half_width)
        & (np.abs(pixels[:, 1] - center_v) <= half_height)
    )
    central_indices = np.flatnonzero(central_mask)

    required = max(1, int(min_depth_points))
    if len(central_indices) >= required:
        selected_indices = central_indices
        depth_source = "central_object_points"
    else:
        valid_indices = np.flatnonzero(valid_depth)
        if len(valid_indices) == 0:
            raise ValueError("object_points_cam has no valid positive depth")
        normalized_distance = (
            ((pixels[valid_indices, 0] - center_u) / half_width) ** 2
            + ((pixels[valid_indices, 1] - center_v) / half_height) ** 2
        )
        nearest_count = min(required, len(valid_indices))
        nearest_order = np.argsort(normalized_distance)[:nearest_count]
        selected_indices = valid_indices[nearest_order]
        depth_source = "nearest_object_points"

    depth_m = float(np.median(points[selected_indices, 2]))
    fx = float(intrinsics[0, 0])
    fy = float(intrinsics[1, 1])
    cx = float(intrinsics[0, 2])
    cy = float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    point_cam = np.array(
        [
            (center_u - cx) * depth_m / fx,
            (center_v - cy) * depth_m / fy,
            depth_m,
        ],
        dtype=np.float64,
    )
    diagnostics = {
        "pixel_center": [center_u, center_v],
        "depth_m": depth_m,
        "depth_points": int(len(selected_indices)),
        "central_depth_points": int(len(central_indices)),
        "depth_source": depth_source,
        "center_fraction": fraction,
    }
    return point_cam, diagnostics


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
