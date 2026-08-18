#!/usr/bin/env python3
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np


PLACE_AFTER_GRASP_ENABLED = os.getenv(
    "WRIST_PLACE_AFTER_GRASP_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_FORWARD_M = float(
    os.getenv("WRIST_PLACE_FORWARD_FROM_SHEET_M", "0.300")
)
PLACE_LEFT_M = float(
    os.getenv("WRIST_PLACE_LEFT_FROM_SHEET_M", "0.000")
)
PLACE_SAFE_Z_M = float(os.getenv("WRIST_PLACE_SAFE_Z_M", "0.330"))
PLACE_RELEASE_ABOVE_SHEET_Z_M = float(
    os.getenv("WRIST_PLACE_RELEASE_ABOVE_SHEET_Z_M", "0.110")
)
PLACE_EXTRA_DESCENT_M = float(
    os.getenv("WRIST_PLACE_EXTRA_DESCENT_M", "0.100")
)
PLACE_MIN_RELEASE_Z_M = float(
    os.getenv("WRIST_PLACE_MIN_RELEASE_Z_M", "0.060")
)
PLACE_MAX_RELEASE_Z_M = float(
    os.getenv("WRIST_PLACE_MAX_RELEASE_Z_M", "0.360")
)
PLACE_POST_LIFT_M = float(
    os.getenv("WRIST_PLACE_POST_RELEASE_LIFT_M", "0.100")
)
PLACE_DETECT_TIMEOUT_S = float(
    os.getenv("WRIST_PLACE_DETECT_TIMEOUT_S", "8.0")
)
PLACE_DETECT_REQUIRED_FRAMES = int(
    os.getenv("WRIST_PLACE_DETECT_REQUIRED_FRAMES", "2")
)
PLACE_CONFIDENCE = float(
    os.getenv("WRIST_PLACE_CONFIDENCE", "0.450")
)
PLACE_DEPTH_MIN_M = float(
    os.getenv("WRIST_PLACE_DEPTH_MIN_M", "0.120")
)
PLACE_DEPTH_MAX_M = float(
    os.getenv("WRIST_PLACE_DEPTH_MAX_M", "2.250")
)
PLACE_PRE_DETECT_ENTER_ENABLED = os.getenv(
    "WRIST_PLACE_PRE_DETECT_ENTER_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_MOVE_OBSERVE_BEFORE_DETECT = os.getenv(
    "WRIST_PLACE_MOVE_OBSERVE_BEFORE_DETECT",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_USE_MOVE_J_FOR_XY = os.getenv(
    "WRIST_PLACE_USE_MOVE_J_FOR_XY",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_MAX_JOINT_TRAVEL_RAD = float(
    os.getenv("WRIST_PLACE_MAX_JOINT_TRAVEL_RAD", "2.250")
)
PLACE_SCAN_ENABLED = os.getenv(
    "WRIST_PLACE_SCAN_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_SCAN_DETECT_TIMEOUT_S = float(
    os.getenv("WRIST_PLACE_SCAN_DETECT_TIMEOUT_S", "3.0")
)
PLACE_SCAN_OFFSETS_DEG_TEXT = os.getenv(
    "WRIST_PLACE_SCAN_OFFSETS_DEG",
    "10,-10",
)
PLACE_RETURN_CENTER_BEFORE_PLACE = os.getenv(
    "WRIST_PLACE_RETURN_CENTER_BEFORE_PLACE",
    "1",
).lower() in {"1", "true", "yes", "on"}
PLACE_RETURN_OBSERVATION_AFTER_SUCCESS = os.getenv(
    "WRIST_PLACE_RETURN_OBSERVATION_AFTER_SUCCESS",
    "1",
).lower() in {"1", "true", "yes", "on"}


def _parse_float_list(text, default_values):
    values = []
    for part in str(text or "").replace(";", ",").split(","):
        part = part.strip()
        if part:
            values.append(float(part))
    return values or list(default_values)


PLACE_SCAN_OFFSETS_DEG = _parse_float_list(
    PLACE_SCAN_OFFSETS_DEG_TEXT,
    [10.0, -10.0],
)


def _homogeneous(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def _normalize_class_name(value):
    text = str(value or "").strip().lower()
    text = text.replace("-", "_").replace(" ", "_")
    while "__" in text:
        text = text.replace("__", "_")
    return text


def _bbox_depth_point_base(
    bbox,
    depth,
    camera_matrix,
    transform_cam_to_base,
):
    height, width = depth.shape[:2]
    x0, y0, x1, y1 = [float(value) for value in bbox]

    # 使用框中心 50% 区域，避免边缘背景深度污染。
    cx_box = 0.5 * (x0 + x1)
    cy_box = 0.5 * (y0 + y1)
    half_w = max(3.0, 0.25 * (x1 - x0))
    half_h = max(3.0, 0.25 * (y1 - y0))
    ix0 = max(0, int(round(cx_box - half_w)))
    ix1 = min(width - 1, int(round(cx_box + half_w)))
    iy0 = max(0, int(round(cy_box - half_h)))
    iy1 = min(height - 1, int(round(cy_box + half_h)))

    roi = depth[iy0 : iy1 + 1, ix0 : ix1 + 1]
    valid = roi[
        np.isfinite(roi)
        & (roi > PLACE_DEPTH_MIN_M)
        & (roi < PLACE_DEPTH_MAX_M)
    ]
    if len(valid) < 20:
        raise RuntimeError(
            "放置图纸检测框内有效深度太少: "
            f"valid={len(valid)}, bbox={[round(float(v), 1) for v in bbox]}"
        )

    z = float(np.median(valid))
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    point_cam = np.array(
        [
            (cx_box - cx) / fx * z,
            (cy_box - cy) / fy * z,
            z,
            1.0,
        ],
        dtype=np.float64,
    )
    return (transform_cam_to_base @ point_cam)[:3], z


def _select_target_sheet_candidate(
    controller,
    target_class_name,
    timeout_s=None,
    view_label="当前观察位",
):
    timeout = (
        PLACE_DETECT_TIMEOUT_S
        if timeout_s is None
        else float(timeout_s)
    )
    deadline = time.monotonic() + max(1.0, timeout)
    stable_hits = 0
    best_match = None
    last_result = None

    while time.monotonic() < deadline:
        snapshot = controller.snapshot_rgbd()
        if snapshot[0] is None:
            time.sleep(0.10)
            continue
        bgr, depth, camera_matrix = snapshot
        result = controller.instruction_detector.detect(bgr.copy())
        last_result = (result, bgr, depth, camera_matrix)

        matches = [
            candidate
            for candidate in result["candidates"]
            if (
                _normalize_class_name(candidate.get("class_name"))
                == target_class_name
                and float(candidate.get("confidence", 0.0))
                >= PLACE_CONFIDENCE
            )
        ]
        if not matches:
            stable_hits = 0
            time.sleep(0.15)
            continue

        best_match = max(
            matches,
            key=lambda item: float(item.get("confidence", 0.0)),
        )
        stable_hits += 1
        controller.get_logger().info(
            f"放置阶段图纸 YOLO({view_label}): "
            f"class={best_match['class_name']}, "
            f"confidence={float(best_match['confidence']):.4f}, "
            f"confirm={stable_hits}/{PLACE_DETECT_REQUIRED_FRAMES}"
        )
        if stable_hits >= max(1, PLACE_DETECT_REQUIRED_FRAMES):
            return best_match, bgr, depth, camera_matrix, result

        time.sleep(0.15)

    raw = []
    if last_result is not None:
        raw = [
            (
                candidate.get("class_name"),
                round(float(candidate.get("confidence", 0.0)), 3),
            )
            for candidate in last_result[0].get("candidates", [])
        ]
    raise RuntimeError(
        f"放置阶段{view_label}未识别到对应图片: "
        f"target={target_class_name}, raw={raw}"
    )


def _select_target_sheet_candidate_with_scan(
    controller,
    target_class_name,
    observation_joints_rad,
    closed_gripper,
):
    if not PLACE_SCAN_ENABLED or observation_joints_rad is None:
        candidate = _select_target_sheet_candidate(
            controller,
            target_class_name,
        )
        return candidate + ("当前观察位", 0.0)

    base_joints = np.asarray(
        observation_joints_rad,
        dtype=np.float64,
    ).reshape(6)
    scan_views = [("中心观察位", 0.0)]
    for offset_deg in PLACE_SCAN_OFFSETS_DEG:
        side = "左侧观察" if float(offset_deg) > 0.0 else "右侧观察"
        scan_views.append((side, float(offset_deg)))

    errors = []
    for label, offset_deg in scan_views:
        if abs(offset_deg) > 1e-6:
            target_joints = base_joints.copy()
            target_joints[0] += math.radians(float(offset_deg))
            controller.get_logger().info(
                "放置牌子识别扫描: "
                f"{label}, J1_offset={offset_deg:+.1f}deg, "
                f"target_j1={math.degrees(float(target_joints[0])):.1f}deg"
            )
            if controller.move_to_joint_pose(
                target_joints,
                label=(
                    "放置牌子识别扫描 "
                    f"{label}: J1_offset={offset_deg:+.1f}deg"
                ),
                gripper_m=closed_gripper,
                timeout_s=10.0,
            ) is False:
                errors.append(f"{label}: MOVE_J 未到达")
                continue

        try:
            candidate = _select_target_sheet_candidate(
                controller,
                target_class_name,
                timeout_s=PLACE_SCAN_DETECT_TIMEOUT_S,
                view_label=label,
            )
            controller.get_logger().info(
                "放置牌子识别扫描成功: "
                f"{label}, J1_offset={offset_deg:+.1f}deg；"
                "不再继续扫描其它方向。"
            )
            return candidate + (label, offset_deg)
        except RuntimeError as exc:
            errors.append(f"{label}: {exc}")

    if controller.move_to_joint_pose(
        base_joints,
        label="放置牌子识别失败后回中心观察位",
        gripper_m=closed_gripper,
        timeout_s=10.0,
    ) is False:
        errors.append("回中心观察位失败")

    raise RuntimeError(
        "放置阶段中心/左/右扫描均未识别到对应图片: "
        + " | ".join(errors)
    )


def execute_place_after_grasp(
    controller,
    r_cam_to_gripper,
    t_cam_to_gripper,
    observation_joints_rad=None,
    place_ik_solver=None,
):
    if not PLACE_AFTER_GRASP_ENABLED:
        controller.get_logger().info(
            "抓取后放置模块已关闭，跳过放置。"
        )
        return False

    if controller.instruction_detector is None:
        raise RuntimeError("放置阶段需要图纸 YOLO 检测器。")
    if not controller.instruction_detector.is_loaded:
        raise RuntimeError("放置阶段图纸 YOLO 模型尚未加载完成。")

    target_class_name = _normalize_class_name(
        controller.target_model_class_name
    )
    if not target_class_name:
        raise RuntimeError(
            "放置阶段没有可复用的图纸目标类别。"
        )

    closed_gripper = float(
        controller.get_grasp_config(
            controller.target_class_id
        )["gripper_closed"]
    )
    open_gripper = float(
        controller.get_grasp_config(
            controller.target_class_id
        )["gripper_open"]
    )

    if (
        PLACE_MOVE_OBSERVE_BEFORE_DETECT
        and observation_joints_rad is not None
    ):
        if controller.move_to_joint_pose(
            observation_joints_rad,
            label="放置前回观察位（夹爪保持闭合）",
            gripper_m=closed_gripper,
            timeout_s=20.0,
        ) is False:
            raise RuntimeError(
                "放置前回观察位失败，已停止放置流程。"
            )

    if PLACE_PRE_DETECT_ENTER_ENABLED:
        controller.wait_for_enter_confirmation(
            "抓取已完成，机械臂已夹持物体回到观察位。"
            "请将对应物体牌子放到腕部相机可见位置，"
            "确认后按 Enter 开始识别物体牌子并执行放置。",
            "开始识别物体牌子",
            target_class_name,
            "press Enter to detect placement sign> ",
        )

    (
        candidate,
        bgr,
        depth,
        camera_matrix,
        result,
        scan_view_label,
        scan_offset_deg,
    ) = (
        _select_target_sheet_candidate_with_scan(
            controller,
            target_class_name,
            observation_joints_rad,
            closed_gripper,
        )
    )

    (
        detection_rotation_gripper_to_base,
        detection_translation_gripper_to_base,
        _,
    ) = controller.get_cached_end_pose(max_age_s=1.0)
    detection_transform_cam_to_base = (
        _homogeneous(
            detection_rotation_gripper_to_base,
            detection_translation_gripper_to_base,
        )
        @ _homogeneous(r_cam_to_gripper, t_cam_to_gripper)
    )
    sheet_point_base, sheet_depth = _bbox_depth_point_base(
        candidate["bbox"],
        depth,
        camera_matrix,
        detection_transform_cam_to_base,
    )

    if (
        PLACE_RETURN_CENTER_BEFORE_PLACE
        and observation_joints_rad is not None
        and abs(float(scan_offset_deg)) > 1e-6
    ):
        if controller.move_to_joint_pose(
            observation_joints_rad,
            label="放置牌子识别完成后回中心观察位（夹爪保持闭合）",
            gripper_m=closed_gripper,
            timeout_s=10.0,
        ) is False:
            raise RuntimeError(
                "放置牌子识别后回中心观察位失败，已停止放置流程。"
            )

    (
        rotation_gripper_to_base,
        translation_gripper_to_base,
        current_rpy_deg,
    ) = controller.get_cached_end_pose(max_age_s=1.0)

    picture_xy = sheet_point_base[:2].astype(np.float64)
    camera_xy = np.asarray(
        translation_gripper_to_base[:2],
        dtype=np.float64,
    )
    front_dir = camera_xy - picture_xy
    norm = float(np.linalg.norm(front_dir))
    if norm < 1e-6:
        raise RuntimeError(
            "放置方向计算失败：相机与图片 XY 几乎重合。"
        )
    front_dir /= norm

    # 左侧按“从图片看向相机/机械臂”的左手方向定义。
    left_dir = np.array(
        [-front_dir[1], front_dir[0]],
        dtype=np.float64,
    )
    place_xy = (
        picture_xy
        + front_dir * float(PLACE_FORWARD_M)
        + left_dir * float(PLACE_LEFT_M)
    )

    release_z = float(
        np.clip(
            float(sheet_point_base[2])
            + PLACE_RELEASE_ABOVE_SHEET_Z_M
            - PLACE_EXTRA_DESCENT_M,
            PLACE_MIN_RELEASE_Z_M,
            PLACE_MAX_RELEASE_Z_M,
        )
    )
    safe_z = max(
        float(translation_gripper_to_base[2]),
        release_z + max(0.03, PLACE_POST_LIFT_M),
        PLACE_SAFE_Z_M,
    )

    overlay = (
        result["overlay"].copy()
        if result.get("overlay") is not None
        else bgr.copy()
    )
    x0, y0, x1, y1 = [
        int(round(value))
        for value in candidate["bbox"]
    ]
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 0), 2)
    cv2.putText(
        overlay,
        (
            f"PLACE {target_class_name}: "
            f"front={PLACE_FORWARD_M:.2f}m left={PLACE_LEFT_M:.2f}m"
        ),
        (10, 84),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    preview_path = os.path.join(
        controller.output_dir,
        (
            "place_sheet_target_"
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        ),
    )
    cv2.imwrite(preview_path, overlay)

    current_pose = {
        "x": float(translation_gripper_to_base[0]),
        "y": float(translation_gripper_to_base[1]),
        "z": float(translation_gripper_to_base[2]),
        "roll": math.radians(float(current_rpy_deg[0])),
        "pitch": math.radians(float(current_rpy_deg[1])),
        "yaw": math.radians(float(current_rpy_deg[2])),
        "gripper": closed_gripper,
    }
    safe_pose = dict(current_pose)
    safe_pose["z"] = safe_z
    place_safe_pose = dict(safe_pose)
    place_safe_pose["x"] = float(place_xy[0])
    place_safe_pose["y"] = float(place_xy[1])

    release_pose = dict(place_safe_pose)
    release_pose["z"] = release_z
    release_pose["gripper"] = current_pose["gripper"]

    controller.get_logger().warn(
        "抓取后放置目标: "
        f"class={target_class_name}, "
            f"sheet_point={np.round(sheet_point_base, 4).tolist()}, "
            f"sheet_depth={sheet_depth:.3f}m, "
            f"scan_view={scan_view_label}, "
            f"scan_offset_deg={float(scan_offset_deg):+.1f}, "
            f"front_dir={np.round(front_dir, 4).tolist()}, "
            f"left_dir={np.round(left_dir, 4).tolist()}, "
            f"place_xy={np.round(place_xy, 4).tolist()}, "
            f"release_above_sheet={PLACE_RELEASE_ABOVE_SHEET_Z_M:.3f}, "
            f"extra_descent={PLACE_EXTRA_DESCENT_M:.3f}, "
            f"release_z={release_z:.4f}, safe_z={safe_z:.4f}, "
            f"preview={preview_path}"
        )

    if abs(float(current_pose["z"]) - safe_z) > 0.004:
        if controller.publish_strict_vertical_path(
            current_pose,
            target_z=safe_z,
            duration=1.0,
            gripper=current_pose["gripper"],
            label="放置步骤1：抓取后垂直抬到放置安全高度",
        ) is False:
            raise RuntimeError("放置步骤1安全抬升失败。")

    aligned_pose = None
    if (
        PLACE_USE_MOVE_J_FOR_XY
        and place_ik_solver is not None
        and observation_joints_rad is not None
    ):
        target_joints = place_ik_solver(
            place_safe_pose,
            seeds=[observation_joints_rad],
        )
        if target_joints is None:
            raise RuntimeError(
                "放置步骤2高位目标无离线 IK 解，"
                "已拒绝发送放置运动。"
            )
        target_joints = np.asarray(target_joints, dtype=np.float64).reshape(6)
        seed_joints = np.asarray(
            observation_joints_rad,
            dtype=np.float64,
        ).reshape(6)
        max_joint_travel = float(
            np.max(np.abs(target_joints - seed_joints))
        )
        if max_joint_travel > PLACE_MAX_JOINT_TRAVEL_RAD:
            raise RuntimeError(
                "放置步骤2高位目标关节跨度过大，"
                "已拒绝发送放置 MOVE_J: "
                f"max_joint_travel={max_joint_travel:.3f}rad > "
                f"limit={PLACE_MAX_JOINT_TRAVEL_RAD:.3f}rad, "
                f"target_joints={np.round(target_joints, 4).tolist()}"
            )
        if controller.move_to_joint_pose(
            target_joints,
            label=(
                "放置步骤2：MOVE_J 到图片正前方高位"
                f"(joint_travel={max_joint_travel:.3f}rad)"
            ),
            gripper_m=current_pose["gripper"],
            timeout_s=20.0,
        ) is False:
            raise RuntimeError("放置步骤2 MOVE_J 高位移动失败。")
        _, actual_xyz, actual_rpy_deg = controller.get_cached_end_pose(
            max_age_s=1.0
        )
        aligned_pose = {
            "x": float(actual_xyz[0]),
            "y": float(actual_xyz[1]),
            "z": float(actual_xyz[2]),
            "roll": math.radians(float(actual_rpy_deg[0])),
            "pitch": math.radians(float(actual_rpy_deg[1])),
            "yaw": math.radians(float(actual_rpy_deg[2])),
            "gripper": current_pose["gripper"],
        }
        release_pose["roll"] = aligned_pose["roll"]
        release_pose["pitch"] = aligned_pose["pitch"]
        release_pose["yaw"] = aligned_pose["yaw"]
        controller.get_logger().info(
            "放置步骤2已使用离线 IK + MOVE_J 完成高位移动: "
            f"actual_xyz={np.round(actual_xyz, 4).tolist()}, "
            f"actual_rpy_deg={np.round(actual_rpy_deg, 2).tolist()}"
        )
    else:
        aligned_pose = controller.publish_xy_only_path(
            safe_pose,
            target_x=float(place_xy[0]),
            target_y=float(place_xy[1]),
            duration=1.5,
            gripper=current_pose["gripper"],
            label="放置步骤2：高位水平移动到图片正前方",
        )
        if aligned_pose is False:
            raise RuntimeError("放置步骤2高位水平移动失败。")

    if controller.publish_strict_vertical_path(
        aligned_pose,
        target_z=release_z,
        duration=1.0,
        gripper=current_pose["gripper"],
        label="放置步骤3：垂直下降到释放高度",
    ) is False:
        raise RuntimeError("放置步骤3下降失败。")

    if controller.publish_pose_for(
        release_pose,
        duration=0.25,
    ) is False:
        raise RuntimeError("放置步骤3释放前保持失败。")

    release_pose["gripper"] = open_gripper
    controller.authorize_gripper_release(
        "放置步骤4：到达释放高度后打开夹爪"
    )
    try:
        if controller.publish_pose_for(
            release_pose,
            duration=0.80,
        ) is False:
            raise RuntimeError("放置步骤4打开夹爪失败。")
    except Exception:
        controller.cancel_gripper_release()
        raise
    controller.complete_gripper_release()

    if controller.publish_strict_vertical_path(
        release_pose,
        target_z=release_z + PLACE_POST_LIFT_M,
        duration=1.0,
        gripper=open_gripper,
        label="放置步骤5：释放后垂直抬升",
    ) is False:
        raise RuntimeError("放置步骤5释放后抬升失败。")

    if (
        PLACE_RETURN_OBSERVATION_AFTER_SUCCESS
        and observation_joints_rad is not None
    ):
        if controller.move_to_joint_pose(
            observation_joints_rad,
            label="放置成功后回观察位（夹爪保持打开）",
            gripper_m=open_gripper,
            timeout_s=20.0,
        ) is False:
            raise RuntimeError("放置成功后回观察位失败。")

    controller.get_logger().info(
        "抓取后放置序列完成，夹爪已打开，机械臂保持使能。"
    )
    return True
