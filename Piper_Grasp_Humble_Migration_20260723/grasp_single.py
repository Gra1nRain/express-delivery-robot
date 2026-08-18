#!/usr/bin/env python3
import sys
import numpy as np

ROS_PYTHON_VERSIONS = {
    "humble": (3, 10),
    "jazzy": (3, 12),
}
ros_distro = __import__("os").environ.get("ROS_DISTRO", "").lower()
expected_python = ROS_PYTHON_VERSIONS.get(ros_distro)
if expected_python and sys.version_info[:2] != expected_python:
    raise SystemExit(
        f"ROS {ros_distro} 需要 Python "
        f"{expected_python[0]}.{expected_python[1]}，"
        f"当前是 {sys.version_info.major}.{sys.version_info.minor}。\n"
        "请通过项目的 ./run_grasp_single.sh 启动。"
    )

# Prefer Ubuntu's ROS-compatible OpenCV over a pip/Conda wheel.  Humble's
# cv_bridge imports cv2 lazily (when CvBridge is constructed), so cv2 must be
# imported explicitly while the system dist-packages directory has priority.
# Otherwise the wheel's bundled Qt/OpenBLAS libraries can terminate the process
# with SIGFPE when ROS, RealSense and the OpenCV GUI start concurrently.
system_dist_packages = "/usr/lib/python3/dist-packages"
use_system_opencv = __import__("os").environ.get(
    "WRIST_USE_SYSTEM_OPENCV",
    "1",
).lower() in {"1", "true", "yes", "on"}
original_system_dist_packages_index = None
prioritize_system_dist_packages = (
    use_system_opencv
    and __import__("os").path.isdir(system_dist_packages)
)
if prioritize_system_dist_packages:
    if system_dist_packages in sys.path:
        original_system_dist_packages_index = sys.path.index(system_dist_packages)
        sys.path.remove(system_dist_packages)
    sys.path.insert(0, system_dist_packages)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from piper_msgs.msg import PosCmd, PiperStatusMsg
from piper_msgs.srv import Enable
from enum import Enum, auto
import subprocess
import time
import math
from threading import Event, Lock, Thread
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

if prioritize_system_dist_packages:
    sys.path.remove(system_dist_packages)
    if original_system_dist_packages_index is not None:
        sys.path.insert(
            min(original_system_dist_packages_index, len(sys.path)),
            system_dist_packages,
        )

from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as ScipyRotation
import os
import sys
import json
import signal
import importlib
import site
import torch
from PIL import Image as PILImage
import warnings
import queue
from datetime import datetime
from place_after_grasp import execute_place_after_grasp
from block_grasp_planner import (
    build_block_rpy_candidates,
    build_world_yz_pregrasp_pose,
    choose_reachable_block_candidate,
    project_yolo_bbox_center_with_robust_depth,
    robust_point_cloud_box_center,
)
from target_detection_gate import (
    bbox_iou,
    evaluate_bbox_visibility,
    localization_detection_policy,
)
from gripper_hold_guard import (
    GripperHoldGuard,
    choose_bottle_hold_position,
)
from yolo_runtime import warm_up_yolo_model

# 设置环境变量
os.environ["TRANSFORMERS_OFFLINE"] = "1"
os.environ["HF_DATASETS_OFFLINE"] = "1"
os.environ.setdefault("YOLO_OFFLINE", "true")
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
warnings.filterwarnings("ignore")

# 添加模块路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
workspace_dir = current_dir


def parse_env_float_list(name, default_values):
    text = os.getenv(name, "").strip()
    if not text:
        return list(default_values)

    values = []
    for part in text.replace(";", ",").split(","):
        part = part.strip()
        if part:
            values.append(float(part))

    return values or list(default_values)


LEFT_CAMERA_SERIAL_DEFAULT = "151222079131"


def detect_realsense_serial():
    """
    固定返回左侧 RealSense 序列号。

    不依赖 pyrealsense2，也不在 Python 启动阶段调用
    rs-enumerate-devices。实际设备匹配交给 realsense2_camera
    的 serial_no 参数完成。

    可通过环境变量覆盖：
        export LEFT_CAMERA_SERIAL=xxxxxxxxxxxx
    """
    serial = os.getenv(
        "LEFT_CAMERA_SERIAL",
        LEFT_CAMERA_SERIAL_DEFAULT,
    ).strip().lstrip("_")

    if not serial:
        raise RuntimeError(
            "左侧相机序列号为空，请设置 LEFT_CAMERA_SERIAL。"
        )

    print(f"固定使用左侧 RealSense serial={serial}")
    return serial


CAMERA_SERIAL_RAW = detect_realsense_serial()

# realsense2_camera 对纯数字序列号使用前导下划线，避免被解释为整数。
CAMERA1_SERIAL = f"_{CAMERA_SERIAL_RAW}"

# 左侧相机独立命名空间，防止误复用右侧相机或旧相机节点。
CAMERA_NAMESPACE = "left_wrist_camera"
CAMERA_NAME = "camera"
COLOR_TOPIC = "/left_wrist_camera/camera/color/image_raw"
DEPTH_TOPIC = "/left_wrist_camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/left_wrist_camera/camera/color/camera_info"

CAN_NAME = os.getenv("CAN_NAME", os.getenv("PIPER_CAN_NAME", "can2"))
if (
    CAN_NAME == "can_left"
    and not os.path.exists("/sys/class/net/can_left")
    and os.path.exists("/sys/class/net/can0")
):
    CAN_NAME = "can0"


# 当前 Piper 上通过关节示教并已验证的观察位。旧的
# 笛卡尔观察姿态会被主控回报 REACH_TARGET_POS_FAILED，所以
# 自动流程使用 MOVE_J 直接到达示教关节位。
OBSERVATION_JOINTS_RAD = np.asarray(
    parse_env_float_list(
        "WRIST_OBSERVATION_JOINTS_RAD",
        [
            -1.544267,
            0.593639,
            -0.717610,
            -0.072065,
            0.786428,
            0.000000,
        ],
    ),
    dtype=np.float64,
)
if OBSERVATION_JOINTS_RAD.shape != (6,):
    raise RuntimeError(
        "WRIST_OBSERVATION_JOINTS_RAD 必须恰好包含 6 个关节角(rad)"
    )

# 放置阶段单独使用的观察位，默认来自现场示教。
# 与抓取观察位分开，避免为了识别物体牌子而改变取物视角。
PLACE_OBSERVATION_JOINTS_RAD = np.asarray(
    parse_env_float_list(
        "WRIST_PLACE_OBSERVATION_JOINTS_RAD",
        [
            -1.556258,
            0.853972,
            -1.132928,
            -0.137270,
            0.703263,
            0.112521,
        ],
    ),
    dtype=np.float64,
)
if PLACE_OBSERVATION_JOINTS_RAD.shape != (6,):
    raise RuntimeError(
        "WRIST_PLACE_OBSERVATION_JOINTS_RAD 必须恰好包含 6 个关节角(rad)"
    )

BLOCK_TOP_DOWN_JOINTS_RAD = np.asarray(
    parse_env_float_list(
        "WRIST_BLOCK_TOP_DOWN_JOINTS_RAD",
        [
            -1.598023,
            1.633942,
            -1.521962,
            0.000000,
            1.100000,
            -0.028868,
        ],
    ),
    dtype=np.float64,
)
if BLOCK_TOP_DOWN_JOINTS_RAD.shape != (6,):
    raise RuntimeError(
        "WRIST_BLOCK_TOP_DOWN_JOINTS_RAD 必须恰好包含 6 个关节角(rad)"
    )

PIPER_JOINT_LIMITS_RAD = np.asarray(
    [
        [-2.617994, 2.617994],
        [0.0, 3.141593],
        [-2.967060, 0.0],
        [-1.745330, 1.745330],
        [-1.221730, 1.221730],
        [-2.094396, 2.094396],
    ],
    dtype=np.float64,
)

# Piper S-V1.8-2 的修正 DH 参数，用于发送动作前做离线可达性
# 检查。真正的关节命令仍由 piper_ros 发送。
PIPER_MDH = (
    (0.123, 0.0, 0.0, 0.0),
    (0.0, 0.0, -math.pi / 2.0, -3.0058060377846343),
    (0.0, 0.28503, 0.0, -1.793849405199772),
    (0.25075, -0.02198, math.pi / 2.0, 0.0),
    (0.0, 0.0, -math.pi / 2.0, 0.0),
    (0.091, 0.0, math.pi / 2.0, 0.0),
)


GRIPPER_CENTER_OFFSET_FILE = os.path.join(
    workspace_dir,
    "src",
    "real",
    "gripper_center_offset.json",
)

# ================= 本次相机/手眼标定结果 =================
# 标定板内部角点：长度方向 10，宽度方向 7；单格边长 0.02 m。
CALIBRATION_BOARD_CORNERS = (10, 7)
CALIBRATION_SQUARE_SIZE_M = 0.012

# 本次标定得到的左侧 D435 彩色相机内参，标定分辨率为 640 x 480。
CALIBRATED_CAMERA_MATRIX = np.array(
    [
        [609.95968135, 0.0, 338.11086625],
        [0.0, 609.43632008, 233.19505887],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# OpenCV 顺序：[k1, k2, p1, p2, k3]
CALIBRATED_DIST_COEFFS = np.array(
    [[
        0.03023475,
        1.01184202,
        -0.00841295,
        0.00633068,
        -4.22891351,
    ]],
    dtype=np.float64,
)

# 默认使用本次标定内参做深度反投影。
# 如需改回 ROS camera_info，可运行：
# export USE_CALIBRATED_INTRINSICS=0
USE_CALIBRATED_INTRINSICS = os.getenv(
    "USE_CALIBRATED_INTRINSICS",
    "1",
).lower() in {"1", "true", "yes", "on"}

# Tsai 手眼结果: camera -> gripper/end-effector
R_CAM_TO_GRIPPER = np.array(
    [
        [0.01098421, 0.94903696, 0.31497332],
        [-0.99980896, 0.01551666, -0.01188600],
        [-0.01616758, -0.31478258, 0.94902610],
    ],
    dtype=np.float64,
)
T_CAM_TO_GRIPPER = np.array(
    [-0.07677144, 0.04391610, 0.03689397],
    dtype=np.float64,
)

# 默认值只做兜底；优先读取 src/real/gripper_center_offset.json。
DEFAULT_GRIPPER_CENTER_OFFSET = np.array(
    [0.00216058, 0.04197832, 0.13830479],
    dtype=np.float64,
)

# ================= 简单、非倾斜的顶部抓取姿态 =================
# roll=180°, pitch=0°：工具 +Z 轴竖直朝向 base -Z。
# pitch=0° 避开欧拉角 pitch=±90° 的表示奇异。
# yaw 只决定夹爪两指在水平面内的方向。
SIMPLE_GRASP_ROLL_DEG = float(
    os.getenv("WRIST_SIMPLE_GRASP_ROLL_DEG", "180.0")
)
SIMPLE_GRASP_PITCH_DEG = float(
    os.getenv("WRIST_SIMPLE_GRASP_PITCH_DEG", "0.0")
)
SIMPLE_GRASP_PRIMARY_YAW_DEG = float(
    os.getenv("WRIST_SIMPLE_GRASP_YAW_DEG", "90.0")
)
SIMPLE_GRASP_ALLOW_FLIPPED_YAW = os.getenv(
    "WRIST_SIMPLE_GRASP_ALLOW_FLIPPED_YAW",
    "1",
).lower() in {"1", "true", "yes", "on"}

# 在这个高度以上才允许改变抓取姿态，避免低处扭腕。
SAFE_SIMPLE_POSE_Z_M = float(
    os.getenv("WRIST_SAFE_SIMPLE_POSE_Z_M", "0.320")
)

# 保留旧变量名供日志和兼容代码使用；实际抓取时会在 yaw 和 yaw±180°
# 中选择与当前末端姿态旋转距离更短的一组。
GRASP_RPY_DEG = np.array(
    [
        SIMPLE_GRASP_ROLL_DEG,
        SIMPLE_GRASP_PITCH_DEG,
        SIMPLE_GRASP_PRIMARY_YAW_DEG,
    ],
    dtype=np.float64,
)

OPEN_GRIPPER_M = 0.080
MAX_GRIPPER_M = 0.100
CLOSE_GRIPPER_M = 0.0

# True: 让真实夹爪中心对准物体；False: 让法兰原点对准物体。
APPLY_GRIPPER_CENTER_OFFSET_XY = True
APPLY_GRIPPER_CENTER_OFFSET_Z = True

GRASP_FINE_TUNE_BASE_M = np.array(
    [
        float(os.getenv("WRIST_GRASP_FINE_TUNE_X_M", "0.0")),
        float(os.getenv("WRIST_GRASP_FINE_TUNE_Y_M", "0.0")),
        float(os.getenv("WRIST_GRASP_FINE_TUNE_Z_M", "0.0")),
    ],
    dtype=np.float64,
)

# 正值表示让目标点往“夹爪右侧”偏。默认不偏移，让夹爪中心对准点云中心；
# 现场确认存在系统误差时再通过环境变量微调。
GRASP_RIGHT_BIAS_M = float(
    os.getenv("WRIST_GRASP_RIGHT_BIAS_M", "0.0")
)

# 向左补偿量：正值表示沿夹爪局部 +Y 方向移动。默认不补偿。
GRASP_LEFT_COMPENSATION_M = float(
    os.getenv("WRIST_GRASP_LEFT_COMPENSATION_M", "0.0")
)

# 深度点云估计物体高度后，夹爪中心默认对准物体高度的中部。
GRASP_HEIGHT_FRACTION = float(
    os.getenv("WRIST_GRASP_HEIGHT_FRACTION", "0.50")
)
GRASP_MIN_ABOVE_BOTTOM_M = float(
    os.getenv("WRIST_GRASP_MIN_ABOVE_BOTTOM_M", "0.018")
)
GRASP_MAX_BELOW_TOP_M = float(
    os.getenv("WRIST_GRASP_MAX_BELOW_TOP_M", "0.010")
)

BOTTLE_UPRIGHT_SIDE_GRASP = os.getenv(
    "WRIST_BOTTLE_UPRIGHT_SIDE_GRASP",
    "1",
).lower() in {"1", "true", "yes", "on"}
BOTTLE_MOTION_MODE = os.getenv(
    "WRIST_BOTTLE_MOTION_MODE",
    "locked_direct",
).strip().lower()
BOTTLE_TOP_DOWN_PATH = os.getenv(
    "WRIST_BOTTLE_TOP_DOWN_PATH",
    "1" if BOTTLE_MOTION_MODE == "top_down" else "0",
).lower() in {"1", "true", "yes", "on"}
BOTTLE_GRASP_HEIGHT_FRACTION = float(
    os.getenv("WRIST_BOTTLE_GRASP_HEIGHT_FRACTION", "0.52")
)
BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M = float(
    os.getenv("WRIST_BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M", "0.040")
)
BOTTLE_GRASP_MAX_BELOW_TOP_M = float(
    os.getenv("WRIST_BOTTLE_GRASP_MAX_BELOW_TOP_M", "0.040")
)
BOTTLE_MIN_ESTIMATED_HEIGHT_M = float(
    os.getenv("WRIST_BOTTLE_MIN_ESTIMATED_HEIGHT_M", "0.040")
)
BOTTLE_MIN_CLUSTER_POINTS = int(
    os.getenv("WRIST_BOTTLE_MIN_CLUSTER_POINTS", "300")
)
BOTTLE_GRASP_DEPTH_BAND_FRACTION = float(
    os.getenv("WRIST_BOTTLE_GRASP_DEPTH_BAND_FRACTION", "0.16")
)
BOTTLE_KEEP_OBSERVATION_RPY = os.getenv(
    "WRIST_BOTTLE_KEEP_OBSERVATION_RPY",
    "1",
).lower() in {"1", "true", "yes", "on"}
BOTTLE_RPY_ADJUST_THRESHOLD_DEG = float(
    os.getenv("WRIST_BOTTLE_RPY_ADJUST_THRESHOLD_DEG", "3.0")
)
BOTTLE_SLANT_ROLL_DEG = float(
    os.getenv("WRIST_BOTTLE_SLANT_ROLL_DEG", "180.0")
)
BOTTLE_SLANT_PITCH_DEG = float(
    os.getenv("WRIST_BOTTLE_SLANT_PITCH_DEG", "63.0")
)
BOTTLE_SIDE_APPROACH_BACKOFF_M = float(
    os.getenv("WRIST_BOTTLE_SIDE_APPROACH_BACKOFF_M", "0.060")
)
BOTTLE_GRASP_LEFT_SHIFT_M = float(
    os.getenv("WRIST_BOTTLE_GRASP_LEFT_SHIFT_M", "0.035")
)
BOTTLE_FORWARD_EXTRA_M = float(
    os.getenv("WRIST_BOTTLE_FORWARD_EXTRA_M", "0.040")
)
BLOCK_GRASP_LEFT_SHIFT_M = float(
    os.getenv("WRIST_BLOCK_GRASP_LEFT_SHIFT_M", "0.035")
)
BLOCK_FORWARD_EXTRA_M = float(
    os.getenv("WRIST_BLOCK_FORWARD_EXTRA_M", "0.040")
)

# 直立瓶侧抓时，先保持 J2~J6 不变，只转 J1 让工具 +Z
#（夹爪正前方）指向瓶子。之后再向前下方伸展到预抓位。
BOTTLE_BASE_AIM_ENABLED = os.getenv(
    "WRIST_BOTTLE_BASE_AIM_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
BOTTLE_BASE_AIM_MAX_DELTA_DEG = float(
    os.getenv("WRIST_BOTTLE_BASE_AIM_MAX_DELTA_DEG", "60.0")
)
BOTTLE_BASE_AIM_STEP_DEG = float(
    os.getenv("WRIST_BOTTLE_BASE_AIM_STEP_DEG", "0.25")
)
BOTTLE_BASE_AIM_MAX_ERROR_DEG = float(
    os.getenv("WRIST_BOTTLE_BASE_AIM_MAX_ERROR_DEG", "1.0")
)
BOTTLE_FORWARD_DOWN_MAX_LATERAL_M = float(
    os.getenv("WRIST_BOTTLE_FORWARD_DOWN_MAX_LATERAL_M", "0.020")
)
BOTTLE_FORWARD_DOWN_MAX_UPWARD_M = float(
    os.getenv("WRIST_BOTTLE_FORWARD_DOWN_MAX_UPWARD_M", "0.040")
)
BOTTLE_FORWARD_DOWN_MAX_RPY_DEVIATION_DEG = float(
    os.getenv(
        "WRIST_BOTTLE_FORWARD_DOWN_MAX_RPY_DEVIATION_DEG",
        "18.0",
    )
)
BOTTLE_FORWARD_DOWN_MAX_JOINT_TRAVEL_RAD = float(
    os.getenv(
        "WRIST_BOTTLE_FORWARD_DOWN_MAX_JOINT_TRAVEL_RAD",
        "2.40",
    )
)
BOTTLE_LOCKED_PATH_MAX_JOINT_STEP_RAD = float(
    os.getenv(
        "WRIST_BOTTLE_LOCKED_PATH_MAX_JOINT_STEP_RAD",
        "0.45",
    )
)
BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS = os.getenv(
    "WRIST_BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS",
    "0",
).lower() in {"1", "true", "yes", "on"}

# 深度自适应高度之后的 Z 微调量；正值更高，负值更低。
GRASP_CENTER_EXTRA_Z_M = float(
    os.getenv("WRIST_GRASP_CENTER_EXTRA_Z_M", "-0.02")
)

# 额外下探量；正值表示最终闭合点再向下。默认下探 2.5cm。
GRASP_EXTRA_DESCENT_M = float(
    os.getenv("WRIST_GRASP_EXTRA_DESCENT_M", "0.025")
)

# 每次抓取必须先让夹爪中心到达物体最高点上方，再垂直下降。
ABOVE_TOP_CLEARANCE_M = float(
    os.getenv(
        "WRIST_GRASP_ABOVE_TOP_CLEARANCE_M",
        os.getenv("WRIST_GRASP_APPROACH_HEIGHT_M", "0.060"),
    )
)
LIFT_HEIGHT_M = float(
    os.getenv("WRIST_GRASP_LIFT_HEIGHT_M", "0.090")
)
MIN_FLANGE_Z_M = float(
    os.getenv("WRIST_GRASP_MIN_FLANGE_Z_M", "0.105")
)
MIN_FLANGE_Y_M = float(
    os.getenv("WRIST_GRASP_MIN_FLANGE_Y_M", "-0.550")
)
BLOCK_OUTER_KEEP_CURRENT_ROLL_PITCH = os.getenv(
    "WRIST_BLOCK_OUTER_KEEP_CURRENT_ROLL_PITCH",
    "1",
).lower() in {"1", "true", "yes", "on"}
BLOCK_OUTER_RPY_Y_THRESHOLD_M = float(
    os.getenv("WRIST_BLOCK_OUTER_RPY_Y_M", "-0.380")
)
BLOCK_TOP_DOWN_GRASP = os.getenv(
    "WRIST_BLOCK_TOP_DOWN_GRASP",
    "1",
).lower() in {"1", "true", "yes", "on"}
BLOCK_TOP_DOWN_RPY_ENV = os.getenv(
    "WRIST_BLOCK_TOP_DOWN_RPY_DEG",
    "",
).strip()
BLOCK_TOP_DOWN_REQUIRE_CALIBRATED_RPY = os.getenv(
    "WRIST_BLOCK_TOP_DOWN_REQUIRE_CALIBRATED_RPY",
    "1",
).lower() in {"1", "true", "yes", "on"}
BLOCK_TOP_DOWN_USE_MOVE_J = os.getenv(
    "WRIST_BLOCK_TOP_DOWN_USE_MOVE_J",
    "1",
).lower() in {"1", "true", "yes", "on"}
BLOCK_TOP_DOWN_MAX_XY_MOVE_M = float(
    os.getenv("WRIST_BLOCK_TOP_DOWN_MAX_XY_MOVE_M", "0.210")
)
BLOCK_TOP_DOWN_MIN_FLANGE_Y_M = float(
    os.getenv("WRIST_BLOCK_TOP_DOWN_MIN_FLANGE_Y_M", "-0.380")
)
BLOCK_TERMINAL_PITCH_CANDIDATES_DEG = tuple(
    parse_env_float_list(
        "WRIST_BLOCK_TERMINAL_PITCH_CANDIDATES_DEG",
        [30.0, 35.0, 40.0],
    )
)
if not BLOCK_TERMINAL_PITCH_CANDIDATES_DEG:
    raise RuntimeError(
        "WRIST_BLOCK_TERMINAL_PITCH_CANDIDATES_DEG 不能为空"
    )
BLOCK_TERMINAL_YAW_OFFSETS_DEG = tuple(
    parse_env_float_list(
        "WRIST_BLOCK_TERMINAL_YAW_OFFSETS_DEG",
        [0.0, -20.0, 20.0],
    )
)
if not BLOCK_TERMINAL_YAW_OFFSETS_DEG:
    raise RuntimeError(
        "WRIST_BLOCK_TERMINAL_YAW_OFFSETS_DEG 不能为空"
    )
BLOCK_FINAL_APPROACH_Y_M = float(
    os.getenv("WRIST_BLOCK_FINAL_APPROACH_Y_M", "0.030")
)
BLOCK_FINAL_APPROACH_Z_M = float(
    os.getenv("WRIST_BLOCK_FINAL_APPROACH_Z_M", "0.050")
)
BLOCK_MIN_JOINT_MARGIN_RAD = float(
    os.getenv("WRIST_BLOCK_MIN_JOINT_MARGIN_RAD", "0.150")
)
BLOCK_PREGRASP_MIN_FLANGE_Z_M = float(
    os.getenv("WRIST_BLOCK_PREGRASP_MIN_FLANGE_Z_M", "0.150")
)
BLOCK_PREGRASP_MAX_JOINT_TRAVEL_RAD = float(
    os.getenv("WRIST_BLOCK_PREGRASP_MAX_JOINT_TRAVEL_RAD", "2.000")
)
BLOCK_LOCKED_PATH_SAMPLES = int(
    os.getenv("WRIST_BLOCK_LOCKED_PATH_SAMPLES", "24")
)
BLOCK_LOCKED_PATH_MAX_JOINT_STEP_RAD = float(
    os.getenv("WRIST_BLOCK_LOCKED_PATH_MAX_JOINT_STEP_RAD", "0.150")
)
BLOCK_TOP_DOWN_RPY_DEG = None
if BLOCK_TOP_DOWN_RPY_ENV:
    BLOCK_TOP_DOWN_RPY_DEG = np.asarray(
        parse_env_float_list(
            "WRIST_BLOCK_TOP_DOWN_RPY_DEG",
            [],
        ),
        dtype=np.float64,
    )
    if BLOCK_TOP_DOWN_RPY_DEG.size != 3:
        raise RuntimeError(
            "WRIST_BLOCK_TOP_DOWN_RPY_DEG 必须是 3 个角度: "
            "roll,pitch,yaw，单位 deg。"
        )
    BLOCK_TOP_DOWN_RPY_DEG = BLOCK_TOP_DOWN_RPY_DEG.reshape(3)
BLOCK_KEEP_OBSERVATION_RPY = os.getenv(
    "WRIST_BLOCK_KEEP_OBSERVATION_RPY",
    "0",
).lower() in {"1", "true", "yes", "on"}
MAX_GRASP_YAW_DELTA_DEG = float(
    os.getenv("WRIST_GRASP_MAX_YAW_DELTA_DEG", "45.0")
)
OVERHEAD_DWELL_S = float(
    os.getenv("WRIST_GRASP_OVERHEAD_DWELL_S", "0.15")
)
SAFE_LIFT_DURATION_S = float(
    os.getenv("WRIST_GRASP_SAFE_LIFT_DURATION_S", "0.8")
)
OVERHEAD_MOVE_DURATION_S = float(
    os.getenv("WRIST_GRASP_OVERHEAD_MOVE_DURATION_S", "1.4")
)
APPROACH_DESCENT_DURATION_S = float(
    os.getenv("WRIST_GRASP_APPROACH_DESCENT_DURATION_S", "0.8")
)
FINAL_DESCENT_DURATION_S = float(
    os.getenv("WRIST_GRASP_FINAL_DESCENT_DURATION_S", "1.2")
)
GRASP_CLOSE_DWELL_S = float(
    os.getenv("WRIST_GRASP_CLOSE_DWELL_S", "0.65")
)
GRASP_PRE_CLOSE_DWELL_S = float(
    os.getenv("WRIST_GRASP_PRE_CLOSE_DWELL_S", "0.35")
)
GRASP_PRE_CLOSE_WAIT_TIMEOUT_S = float(
    os.getenv("WRIST_GRASP_PRE_CLOSE_WAIT_TIMEOUT_S", "1.8")
)
GRASP_PRE_CLOSE_POSITION_TOL_M = float(
    os.getenv("WRIST_GRASP_PRE_CLOSE_POSITION_TOL_M", "0.015")
)
GRASP_PRE_CLOSE_Z_TOL_M = float(
    os.getenv("WRIST_GRASP_PRE_CLOSE_Z_TOL_M", "0.018")
)
GRASP_PRE_CLOSE_RPY_TOL_DEG = float(
    os.getenv("WRIST_GRASP_PRE_CLOSE_RPY_TOL_DEG", "10.0")
)
BLOCK_GRASP_PRE_CLOSE_POSITION_TOL_M = float(
    os.getenv("WRIST_BLOCK_PRE_CLOSE_POSITION_TOL_M", "0.007")
)
BLOCK_GRASP_PRE_CLOSE_Z_TOL_M = float(
    os.getenv("WRIST_BLOCK_PRE_CLOSE_Z_TOL_M", "0.006")
)
BLOCK_GRIPPER_OPEN_TOLERANCE_M = float(
    os.getenv("WRIST_BLOCK_GRIPPER_OPEN_TOLERANCE_M", "0.005")
)
BLOCK_GRIPPER_OPEN_WAIT_TIMEOUT_S = float(
    os.getenv("WRIST_BLOCK_GRIPPER_OPEN_WAIT_TIMEOUT_S", "2.5")
)
BOTTLE_HOLD_PRELOAD_M = float(
    os.getenv("WRIST_BOTTLE_HOLD_PRELOAD_M", "0.002")
)
BLOCK_POST_CLOSE_DWELL_S = float(
    os.getenv("WRIST_BLOCK_POST_CLOSE_DWELL_S", "0.45")
)
BLOCK_INITIAL_LIFT_DURATION_S = float(
    os.getenv("WRIST_BLOCK_INITIAL_LIFT_DURATION_S", "1.8")
)
GRIPPER_FEEDBACK_MAX_AGE_S = float(
    os.getenv("WRIST_GRIPPER_FEEDBACK_MAX_AGE_S", "1.0")
)
GRASP_RETURN_HOME_DURATION_S = float(
    os.getenv("WRIST_GRASP_RETURN_HOME_DURATION_S", "1.4")
)
CAN_CONTROL_LOSS_GRACE_S = float(
    os.getenv("WRIST_CAN_CONTROL_LOSS_GRACE_S", "2.0")
)
OBSERVATION_SCAN_ENABLED = os.getenv(
    "WRIST_OBSERVATION_SCAN_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
OBSERVATION_SCAN_MODE = os.getenv(
    "WRIST_OBSERVATION_SCAN_MODE",
    "joint1",
).strip().lower()
if OBSERVATION_SCAN_MODE not in {"joint1", "base_joint"}:
    raise RuntimeError(
        "WRIST_OBSERVATION_SCAN_MODE 只允许 joint1，"
        "已禁用旧笛卡尔平移扫描。"
    )
OBSERVATION_SCAN_MODE = "joint1"
OBSERVATION_SCAN_OFFSETS_DEG = parse_env_float_list(
    "WRIST_OBSERVATION_SCAN_OFFSETS_DEG",
    [20.0, -20.0],
)
if (
    not any(value > 0.0 for value in OBSERVATION_SCAN_OFFSETS_DEG)
    or not any(value < 0.0 for value in OBSERVATION_SCAN_OFFSETS_DEG)
    or any(abs(value) > 20.0 for value in OBSERVATION_SCAN_OFFSETS_DEG)
):
    raise RuntimeError(
        "WRIST_OBSERVATION_SCAN_OFFSETS_DEG 必须同时包含"
        "左右偏移，且绝对值不得超过 20°。"
    )
OBSERVATION_SCAN_SETTLE_S = float(
    os.getenv("WRIST_OBSERVATION_SCAN_SETTLE_S", "0.50")
)
OBSERVATION_SCAN_DETECTION_ATTEMPTS = int(
    os.getenv("WRIST_OBSERVATION_SCAN_DETECTION_ATTEMPTS", "3")
)

STOP_BEFORE_CLOSE = os.getenv(
    "WRIST_GRASP_STOP_BEFORE_CLOSE",
    "0",
).lower() in {"1", "true", "yes", "on"}

# ================= 二维码 + 自定义 YOLO 目标抓取 =================
# grasp class_id 表示动作类型（0=瓶子，1=方块）；YOLO 数据集
# class 单独保存。图纸/实物 YOLO 的类别名用于区分六个具体目标。
QR_TO_CLASS = {
    "OBJ_01": 0,
    "OBJ_02": 1,
}

TARGET_CLASS_PROMPTS = {
    0: os.getenv("WRIST_TARGET_0_PROMPT", "bottle"),
    1: os.getenv("WRIST_TARGET_1_PROMPT", "block,cube"),
}

CUSTOM_YOLO_GRASP_CLASSES = {
    "green_bottle": {
        "dataset_class_id": 0,
        "grasp_class_id": 0,
        "color": "green",
        "display_name_zh": "绿色瓶子",
    },
    "orange_bottle": {
        "dataset_class_id": 1,
        "grasp_class_id": 0,
        "color": "orange",
        "display_name_zh": "橙色瓶子",
    },
    "purple_bottle": {
        "dataset_class_id": 2,
        "grasp_class_id": 0,
        "color": "purple",
        "display_name_zh": "紫色瓶子",
    },
    "yellow_block": {
        "dataset_class_id": 4,
        "grasp_class_id": 1,
        "color": "yellow",
        "display_name_zh": "黄色物块",
    },
    "blue_block": {
        "dataset_class_id": 5,
        "grasp_class_id": 1,
        "color": "blue",
        "display_name_zh": "蓝色物块",
    },
    "red_block": {
        "dataset_class_id": 3,
        "grasp_class_id": 1,
        "color": "red",
        "display_name_zh": "红色物块",
    },
}
CUSTOM_YOLO_MODEL_CLASS_ALIASES = {
    "red_cube": "red_block",
    "yellow_cube": "yellow_block",
    "blue_cube": "blue_block",
}
CUSTOM_YOLO_DATASET_ID_TO_NAME = {
    int(spec["dataset_class_id"]): name
    for name, spec in CUSTOM_YOLO_GRASP_CLASSES.items()
}
CUSTOM_YOLO_NON_GRASPABLE_CLASSES = {}


def normalize_custom_yolo_class_name(class_name):
    lowered = (
        str(class_name or "")
        .strip()
        .lower()
        .replace("-", "_")
        .replace(" ", "_")
    )
    while "__" in lowered:
        lowered = lowered.replace("__", "_")
    return CUSTOM_YOLO_MODEL_CLASS_ALIASES.get(lowered, lowered)


def get_custom_target_display_name(class_name):
    normalized = normalize_custom_yolo_class_name(class_name)
    spec = CUSTOM_YOLO_GRASP_CLASSES.get(normalized)
    if spec is None:
        return str(class_name or "unknown")
    return str(spec.get("display_name_zh") or normalized)


def is_bottle_grasp_target(class_id=None, model_class_name=None, prompt=None):
    if class_id is not None:
        try:
            if int(class_id) == 0:
                return True
        except (TypeError, ValueError):
            pass

    for text in (model_class_name, prompt):
        lowered = str(text or "").lower()
        if "bottle" in lowered or "瓶" in lowered:
            return True

    return False


def is_block_grasp_target(class_id=None, model_class_name=None, prompt=None):
    if class_id is not None:
        try:
            if int(class_id) == 1:
                return True
        except (TypeError, ValueError):
            pass

    for text in (model_class_name, prompt):
        lowered = str(text or "").lower()
        if "block" in lowered or "方块" in lowered or "物块" in lowered:
            return True

    return False

QR_CONFIRM_FRAMES = int(os.getenv("WRIST_QR_CONFIRM_FRAMES", "3"))
MOVING_DETECTION_WINDOW = int(os.getenv("WRIST_MOVING_DETECTION_WINDOW", "5"))
MOVING_DETECTION_MIN_HITS = int(os.getenv("WRIST_MOVING_DETECTION_MIN_HITS", "3"))
MOVING_DETECTION_CONFIDENCE = float(os.getenv("WRIST_MOVING_DETECTION_CONFIDENCE", "0.50"))
STILL_DETECTION_CONFIRM_FRAMES = int(os.getenv("WRIST_STILL_DETECTION_CONFIRM_FRAMES", "3"))
STILL_DETECTION_CONFIDENCE = float(os.getenv("WRIST_STILL_DETECTION_CONFIDENCE", "0.50"))
LOCALIZATION_DETECTION_CONFIDENCE = float(
    os.getenv("WRIST_LOCALIZATION_DETECTION_CONFIDENCE", "0.50")
)
BLOCK_COMPLETE_DETECTION_CONFIDENCE = float(
    os.getenv("WRIST_BLOCK_COMPLETE_DETECTION_CONFIDENCE", "0.50")
)
BLOCK_DETECTION_CONFIRM_FRAMES = int(
    os.getenv("WRIST_BLOCK_DETECTION_CONFIRM_FRAMES", "2")
)
BLOCK_DETECTION_CONFIRM_MIN_IOU = float(
    os.getenv("WRIST_BLOCK_DETECTION_CONFIRM_MIN_IOU", "0.50")
)
TARGET_BBOX_EDGE_MARGIN_PX = int(
    os.getenv("WRIST_TARGET_BBOX_EDGE_MARGIN_PX", "12")
)
SECOND_LOCALIZATION_SAMPLES = int(os.getenv("WRIST_SECOND_LOCALIZATION_SAMPLES", "2"))
SECOND_LOCALIZATION_MAX_XY_M = float(os.getenv("WRIST_SECOND_LOCALIZATION_MAX_XY_M", "0.030"))
SECOND_LOCALIZATION_MAX_Z_M = float(os.getenv("WRIST_SECOND_LOCALIZATION_MAX_Z_M", "0.020"))
SECOND_LOCALIZATION_SETTLE_S = float(os.getenv("WRIST_SECOND_LOCALIZATION_SETTLE_S", "0.20"))
SECOND_LOCALIZATION_REQUIRED = os.getenv(
    "WRIST_SECOND_LOCALIZATION_REQUIRED",
    "0",
).lower() in {"1", "true", "yes", "on"}

USE_TARGET_BBOX_FOR_DEPTH = os.getenv(
    "WRIST_USE_TARGET_BBOX_FOR_DEPTH",
    "1",
).lower() in {"1", "true", "yes", "on"}
REQUIRE_TARGET_BBOX_FOR_DEPTH = os.getenv(
    "WRIST_REQUIRE_TARGET_BBOX_FOR_DEPTH",
    "1",
).lower() in {"1", "true", "yes", "on"}
TARGET_BBOX_MARGIN_PX = int(os.getenv("WRIST_TARGET_BBOX_MARGIN_PX", "14"))
MIN_TARGET_DEPTH_POINTS = int(os.getenv("WRIST_MIN_TARGET_DEPTH_POINTS", "70"))

USE_OBJECT_AXIS_YAW = os.getenv(
    "WRIST_USE_OBJECT_AXIS_YAW",
    "1",
).lower() in {"1", "true", "yes", "on"}
# 抓取 yaw 按物体短轴计算。实机观察中 yaw 跟随短轴时，夹爪本体
# 与短轴垂直，也就是与物体长轴平行；这样两指从短轴两侧居中夹紧。
# 如果现场夹爪安装方向相差 90deg，可设置 WRIST_GRASP_AXIS_YAW_OFFSET_DEG=90。
GRASP_CLOSING_AXIS = os.getenv("WRIST_GRASP_CLOSING_AXIS", "short").lower()
GRASP_AXIS_YAW_OFFSET_DEG = float(os.getenv("WRIST_GRASP_AXIS_YAW_OFFSET_DEG", "0.0"))
MIN_OBJECT_AXIS_RATIO = float(os.getenv("WRIST_MIN_OBJECT_AXIS_RATIO", "1.03"))
REQUIRE_OBJECT_AXIS_YAW = os.getenv(
    "WRIST_REQUIRE_OBJECT_AXIS_YAW",
    "1",
).lower() in {"1", "true", "yes", "on"}

USE_COLOR_SHAPE_FALLBACK = os.getenv(
    "WRIST_USE_COLOR_SHAPE_FALLBACK",
    "1",
).lower() in {"1", "true", "yes", "on"}
MIN_COLOR_OBJECT_AREA_PX = int(os.getenv("WRIST_MIN_COLOR_OBJECT_AREA_PX", "900"))

COLOR_HSV_RANGES = {
    "red": [((0, 70, 50), (10, 255, 255)), ((170, 70, 50), (180, 255, 255))],
    "orange": [((11, 70, 50), (24, 255, 255))],
    "yellow": [((25, 70, 50), (34, 255, 255))],
    "green": [((35, 45, 40), (85, 255, 255))],
    "blue": [((86, 45, 40), (130, 255, 255))],
    "purple": [((131, 45, 40), (169, 255, 255))],
    "white": [((0, 0, 185), (180, 55, 255))],
    "black": [((0, 0, 0), (180, 255, 55))],
}

GRASP_CONFIG = {
    0: {
        "height_offset": float(os.getenv("WRIST_BOTTLE_HEIGHT_OFFSET_M", "0.000")),
        "approach_height": float(os.getenv("WRIST_BOTTLE_APPROACH_HEIGHT_M", "0.100")),
        "gripper_open": float(os.getenv("WRIST_BOTTLE_GRIPPER_OPEN_M", f"{OPEN_GRIPPER_M:.3f}")),
        "gripper_closed": float(os.getenv("WRIST_BOTTLE_GRIPPER_CLOSED_M", f"{CLOSE_GRIPPER_M:.3f}")),
        "yaw_deg": os.getenv("WRIST_BOTTLE_YAW_DEG", ""),
        "lift_height": float(os.getenv("WRIST_BOTTLE_LIFT_HEIGHT_M", f"{LIFT_HEIGHT_M:.3f}")),
    },
    1: {
        "height_offset": float(os.getenv("WRIST_BLOCK_HEIGHT_OFFSET_M", "0.010")),
        "approach_height": float(os.getenv("WRIST_BLOCK_APPROACH_HEIGHT_M", "0.120")),
        "gripper_open": float(os.getenv("WRIST_BLOCK_GRIPPER_OPEN_M", f"{MAX_GRIPPER_M:.3f}")),
        "gripper_closed": float(os.getenv("WRIST_BLOCK_GRIPPER_CLOSED_M", f"{CLOSE_GRIPPER_M:.3f}")),
        "yaw_deg": os.getenv("WRIST_BLOCK_YAW_DEG", ""),
        "lift_height": float(os.getenv("WRIST_BLOCK_LIFT_HEIGHT_M", f"{LIFT_HEIGHT_M:.3f}")),
    },
}

for _config in GRASP_CONFIG.values():
    _yaw_text = str(_config.get("yaw_deg", "")).strip()
    _config["yaw_deg"] = None if not _yaw_text else float(_yaw_text)

# 尝试添加 Grounded-Segment-Anything 路径
possible_gsa_paths = [
    os.path.join(workspace_dir, "src", "Grounded-Segment-Anything-main"),
    os.path.join(workspace_dir, "src", "real"),
    os.path.join(parent_dir, "Grounded-Segment-Anything"),
    os.path.join(os.path.expanduser("~"), "Grounded-Segment-Anything"),
    os.path.join(
        os.path.expanduser("~"),
        "piper",
        "Grounded-Segment-Anything",
    ),
    "/home/yy/Grounded-Segment-Anything",
]

gsa_dir = None
for path in possible_gsa_paths:
    if os.path.exists(path):
        gsa_dir = path
        print(f"找到 Grounded-Segment-Anything 目录: {gsa_dir}")
        break

VISION_AVAILABLE = False
if gsa_dir is not None:
    sys.path.insert(0, gsa_dir)
    sys.path.insert(0, os.path.join(gsa_dir, "GroundingDINO"))
    sys.path.insert(0, os.path.join(gsa_dir, "segment_anything"))
    VISION_AVAILABLE = True
    print("✓ 发现视觉检测模块路径（已准备好延迟加载）")

if VISION_AVAILABLE:
    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    BOX_THRESHOLD = 0.3
    TEXT_THRESHOLD = 0.25

    CONFIG_CANDIDATES = [
        os.path.join(gsa_dir, "config_full.py"),
        os.path.join(
            gsa_dir,
            "GroundingDINO",
            "groundingdino",
            "config",
            "GroundingDINO_SwinT_OGC.py",
        ),
        os.path.join(
            workspace_dir,
            "src",
            "real",
            "GroundingDINO",
            "groundingdino",
            "config",
            "GroundingDINO_SwinT_OGC.py",
        ),
    ]

    GROUNDED_CHECKPOINT_CANDIDATES = [
        os.path.join(
            gsa_dir,
            "weights",
            "groundingdino_swint_ogc.pth",
        ),
        os.path.join(gsa_dir, "groundingdino_swint_ogc.pth"),
        os.path.join(
            workspace_dir,
            "src",
            "real",
            "weights",
            "groundingdino_swint_ogc.pth",
        ),
    ]

    SAM_CHECKPOINT_CANDIDATES = [
        os.path.join(gsa_dir, "weights", "sam_vit_h_4b8939.pth"),
        os.path.join(gsa_dir, "sam_vit_h_4b8939.pth"),
        os.path.join(
            workspace_dir,
            "src",
            "real",
            "weights",
            "sam_vit_h_4b8939.pth",
        ),
    ]

    CONFIG_PATH = next(
        (
            path
            for path in CONFIG_CANDIDATES
            if os.path.exists(path)
        ),
        CONFIG_CANDIDATES[0],
    )
    GROUNDED_CHECKPOINT = next(
        (
            path
            for path in GROUNDED_CHECKPOINT_CANDIDATES
            if os.path.exists(path)
        ),
        GROUNDED_CHECKPOINT_CANDIDATES[0],
    )
    SAM_CHECKPOINT = next(
        (
            path
            for path in SAM_CHECKPOINT_CANDIDATES
            if os.path.exists(path)
        ),
        SAM_CHECKPOINT_CANDIDATES[0],
    )

    OUTPUT_DIR = os.path.join(current_dir, "detection_results")
    print(f"GroundingDINO config: {CONFIG_PATH}")
    print(f"GroundingDINO checkpoint: {GROUNDED_CHECKPOINT}")

DINO_AVAILABLE = VISION_AVAILABLE
CUSTOM_YOLO_DEFAULT_MODEL_PATH = os.path.join(
    current_dir,
    "yolo bottle with qr",
    "runs",
    "detect",
    "runs",
    "train",
    "waterbottle-9",
    "weights",
    "best.pt",
)
YOLO_MODEL_PATH = os.getenv(
    "WRIST_YOLO_MODEL_PATH",
    (
        CUSTOM_YOLO_DEFAULT_MODEL_PATH
        if os.path.exists(CUSTOM_YOLO_DEFAULT_MODEL_PATH)
        else os.path.join(current_dir, "yolov8_assets", "yolov8n.pt")
    ),
)
# 当前自训练权重的验证集最佳综合 F1 位于 confidence≈0.146。
# 旧的 0.25/0.70/0.80 多级阈值会把实机上 0.3~0.4 的正确检测静默丢弃。
YOLO_CONFIDENCE = float(os.getenv("WRIST_YOLO_CONFIDENCE", "0.10"))
YOLO_IMAGE_SIZE = int(os.getenv("WRIST_YOLO_IMAGE_SIZE", "640"))
YOLO_DEVICE = os.getenv("WRIST_YOLO_DEVICE", "").strip()
YOLO_AVAILABLE = os.path.exists(YOLO_MODEL_PATH)
VISION_BACKEND = os.getenv("WRIST_VISION_BACKEND", "yolov8").lower()

# 第一阶段识别“指令图纸上的目标图案”。当前权重可能只包含
# green/orange/purple_bottle；换成六类权重后会自动识别新增物块。
# 第二阶段仍使用上面的 YOLO_MODEL_PATH 识别工作区里的实物。
INSTRUCTION_YOLO_MODEL_PATH = os.getenv(
    "WRIST_INSTRUCTION_YOLO_MODEL_PATH",
    os.path.join(
        current_dir,
        "yolo bottle with qr",
        "armdetect all",
        "weights_first.pt",
    ),
)
INSTRUCTION_YOLO_AVAILABLE = os.path.exists(
    INSTRUCTION_YOLO_MODEL_PATH
)
INSTRUCTION_YOLO_RAW_CONFIDENCE = float(
    os.getenv("WRIST_INSTRUCTION_YOLO_RAW_CONFIDENCE", "0.10")
)
INSTRUCTION_YOLO_ACCEPT_CONFIDENCE = float(
    os.getenv("WRIST_INSTRUCTION_YOLO_ACCEPT_CONFIDENCE", "0.50")
)
INSTRUCTION_YOLO_IMAGE_SIZE = int(
    os.getenv("WRIST_INSTRUCTION_YOLO_IMAGE_SIZE", "640")
)
INSTRUCTION_CONFIRM_FRAMES = int(
    os.getenv("WRIST_INSTRUCTION_CONFIRM_FRAMES", "3")
)
INSTRUCTION_TIMEOUT_S = float(
    os.getenv("WRIST_INSTRUCTION_TIMEOUT_S", "30.0")
)
INSTRUCTION_CLEAR_DELAY_S = float(
    os.getenv("WRIST_INSTRUCTION_CLEAR_DELAY_S", "3.0")
)
PRE_GRASP_INSTRUCTION_ENABLED = os.getenv(
    "WRIST_PRE_GRASP_INSTRUCTION_ENABLED",
    "1",
).lower() in {"1", "true", "yes", "on"}
PRE_INSTRUCTION_ENTER_CONFIRM_ENABLED = os.getenv(
    "WRIST_PRE_INSTRUCTION_ENTER_CONFIRM",
    "1",
).lower() in {"1", "true", "yes", "on"}
PRE_GRASP_ENTER_CONFIRM_ENABLED = os.getenv(
    "WRIST_PRE_GRASP_ENTER_CONFIRM",
    "1",
).lower() in {"1", "true", "yes", "on"}
RETRY_GRASP_ON_TARGET_FAILURE_ENABLED = os.getenv(
    "WRIST_RETRY_GRASP_ON_TARGET_FAILURE",
    "1",
).lower() in {"1", "true", "yes", "on"}

if VISION_BACKEND in {"yolo", "yolov8"}:
    VISION_BACKEND = "yolov8"
    VISION_AVAILABLE = YOLO_AVAILABLE
elif VISION_BACKEND in {"dino", "groundingdino"}:
    VISION_BACKEND = "dino"
    VISION_AVAILABLE = DINO_AVAILABLE
else:
    print(
        f"未知视觉后端 {VISION_BACKEND}，默认使用 yolov8。"
    )
    VISION_BACKEND = "yolov8"
    VISION_AVAILABLE = YOLO_AVAILABLE

if VISION_BACKEND == "yolov8":
    if YOLO_AVAILABLE:
        print(f"YOLOv8 模型: {YOLO_MODEL_PATH}")
    else:
        print(f"未找到 YOLOv8 模型: {YOLO_MODEL_PATH}")

if INSTRUCTION_YOLO_AVAILABLE:
    print(f"图纸 YOLO 模型: {INSTRUCTION_YOLO_MODEL_PATH}")
else:
    print(f"未找到图纸 YOLO 模型: {INSTRUCTION_YOLO_MODEL_PATH}")


def rotation_from_rpy_xyz(roll, pitch, yaw):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cx, -sx],
            [0.0, sx, cx],
        ],
        dtype=np.float64,
    )
    ry = np.array(
        [
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ],
        dtype=np.float64,
    )
    rz = np.array(
        [
            [cz, -sz, 0.0],
            [sz, cz, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return rz @ ry @ rx


def wrap_angle_rad(angle):
    """把角度归一化到 [-pi, pi)。"""
    return math.atan2(
        math.sin(float(angle)),
        math.cos(float(angle)),
    )


def wrap_angle_deg(angle):
    return math.degrees(
        wrap_angle_rad(math.radians(float(angle)))
    )


def choose_minimal_equivalent_yaw_deg(target_yaw_deg, current_yaw_deg):
    """
    夹爪两指对称时 yaw 和 yaw±180° 等效。

    在所有等效 yaw 中选择相对当前末端 yaw 旋转最小的一个，
    避免为了同一个抓取方向多转半圈。
    """
    target = float(target_yaw_deg)
    current = float(current_yaw_deg)
    candidates = [
        target + 180.0 * offset
        for offset in range(-2, 3)
    ]
    best = min(
        candidates,
        key=lambda candidate: abs(
            wrap_angle_deg(candidate - current)
        ),
    )
    return wrap_angle_deg(best)


def rotation_distance_deg(rotation_a, rotation_b):
    """计算两个旋转矩阵之间的最短旋转角，单位为度。"""
    delta = (
        np.asarray(rotation_a, dtype=np.float64).T
        @ np.asarray(rotation_b, dtype=np.float64)
    )
    cosine = float(
        np.clip(
            (np.trace(delta) - 1.0) * 0.5,
            -1.0,
            1.0,
        )
    )
    return math.degrees(math.acos(cosine))


def choose_simple_grasp_rpy_deg(current_rpy_deg):
    """
    选择简单顶部抓取姿态。

    roll/pitch 固定为 180°/0°。由于两指夹爪绕工具 Z 轴旋转 180°
    后闭合轴等价，因此可在 primary_yaw 和 primary_yaw+180° 中
    选择与当前末端姿态旋转距离更短的一组，减少腕部绕转。
    """
    current_rpy_deg = np.asarray(
        current_rpy_deg,
        dtype=np.float64,
    ).reshape(3)
    current_rotation = rotation_from_rpy_xyz(
        *np.deg2rad(current_rpy_deg)
    )

    primary_yaw = float(
        SIMPLE_GRASP_PRIMARY_YAW_DEG
    )
    yaw_candidates = [primary_yaw]

    if SIMPLE_GRASP_ALLOW_FLIPPED_YAW:
        yaw_candidates.append(
            primary_yaw + 180.0
        )

    candidates = []
    for yaw_deg in yaw_candidates:
        normalized_yaw = math.degrees(
            wrap_angle_rad(
                math.radians(yaw_deg)
            )
        )
        rpy_deg = np.array(
            [
                SIMPLE_GRASP_ROLL_DEG,
                SIMPLE_GRASP_PITCH_DEG,
                normalized_yaw,
            ],
            dtype=np.float64,
        )
        rotation = rotation_from_rpy_xyz(
            *np.deg2rad(rpy_deg)
        )

        # 工具 +Z 必须明显向下。
        tool_z_base = rotation[:, 2]
        if float(tool_z_base[2]) > -0.98:
            continue

        distance_deg = rotation_distance_deg(
            current_rotation,
            rotation,
        )
        candidates.append(
            (distance_deg, rpy_deg)
        )

    if not candidates:
        raise RuntimeError(
            "没有有效的简单向下抓取姿态。"
        )

    _, selected_rpy_deg = min(
        candidates,
        key=lambda item: item[0],
    )
    return selected_rpy_deg.copy()


def homogeneous(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(
        translation,
        dtype=np.float64,
    ).reshape(3)
    return transform


def piper_forward_kinematics(joints_rad):
    """Return T_flange_in_base for six Piper joint angles."""
    joints = np.asarray(joints_rad, dtype=np.float64).reshape(6)
    transform = np.eye(4, dtype=np.float64)

    for joint, (d, a, alpha, theta_offset) in zip(
        joints,
        PIPER_MDH,
    ):
        theta = float(joint) + theta_offset
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        link = np.array(
            [
                [ct, -st, 0.0, a],
                [ca * st, ca * ct, -sa, -sa * d],
                [sa * st, sa * ct, ca, ca * d],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        transform = transform @ link

    return transform


def solve_piper_ik_pose(position_dict, seeds):
    """Solve a bounded Piper IK target and reject inaccurate solutions."""
    target = homogeneous(
        rotation_from_rpy_xyz(
            float(position_dict["roll"]),
            float(position_dict["pitch"]),
            float(position_dict["yaw"]),
        ),
        [
            float(position_dict["x"]),
            float(position_dict["y"]),
            float(position_dict["z"]),
        ],
    )
    margin_rad = 0.02
    lower = PIPER_JOINT_LIMITS_RAD[:, 0] + margin_rad
    upper = PIPER_JOINT_LIMITS_RAD[:, 1] - margin_rad
    best = None

    for raw_seed in seeds:
        seed = np.clip(
            np.asarray(raw_seed, dtype=np.float64).reshape(6),
            lower,
            upper,
        )

        def residual(joints):
            actual = piper_forward_kinematics(joints)
            position_error = actual[:3, 3] - target[:3, 3]
            rotation_error = ScipyRotation.from_matrix(
                target[:3, :3].T @ actual[:3, :3]
            ).as_rotvec()
            regularization = 0.002 * (joints - seed)
            return np.concatenate(
                (
                    position_error,
                    0.20 * rotation_error,
                    regularization,
                )
            )

        result = least_squares(
            residual,
            seed,
            bounds=(lower, upper),
            max_nfev=2500,
            xtol=1e-11,
            ftol=1e-11,
            gtol=1e-11,
        )
        solution = result.x
        actual = piper_forward_kinematics(solution)
        position_error_m = float(
            np.linalg.norm(actual[:3, 3] - target[:3, 3])
        )
        orientation_error_rad = float(
            np.linalg.norm(
                ScipyRotation.from_matrix(
                    target[:3, :3].T @ actual[:3, :3]
                ).as_rotvec()
            )
        )
        score = (
            position_error_m
            + 0.10 * orientation_error_rad
            + 0.001 * float(np.linalg.norm(solution - seed))
        )
        if best is None or score < best[0]:
            best = (
                score,
                solution.copy(),
                position_error_m,
                orientation_error_rad,
            )

        if (
            position_error_m <= 0.004
            and orientation_error_rad <= math.radians(2.0)
        ):
            return solution

    if best is None:
        raise RuntimeError("IK 没有可用初值。")

    raise RuntimeError(
        "IK 无可用解: "
        f"position_error={best[2] * 1000.0:.1f}mm, "
        f"orientation_error={math.degrees(best[3]):.2f}deg"
    )


def validate_piper_joint_path(
    start_joints_rad,
    target_joints_rad,
    minimum_flange_z_m,
    maximum_joint_travel_rad=2.0,
):
    """Conservatively validate the joint interpolation used by MOVE_J."""
    start = np.asarray(start_joints_rad, dtype=np.float64).reshape(6)
    target = np.asarray(target_joints_rad, dtype=np.float64).reshape(6)
    max_joint_travel = float(np.max(np.abs(target - start)))
    maximum_joint_travel = float(maximum_joint_travel_rad)
    if max_joint_travel > maximum_joint_travel:
        raise RuntimeError(
            "MOVE_J 关节跨度过大: "
            f"max_joint_travel={max_joint_travel:.3f}rad > "
            f"limit={maximum_joint_travel:.3f}rad"
        )

    minimum_z = float("inf")
    for ratio in np.linspace(0.0, 1.0, 61):
        joints = start + float(ratio) * (target - start)
        flange_z = float(piper_forward_kinematics(joints)[2, 3])
        minimum_z = min(minimum_z, flange_z)

    if minimum_z < float(minimum_flange_z_m):
        raise RuntimeError(
            "MOVE_J 离线路径高度不安全: "
            f"minimum_z={minimum_z:.3f}m < "
            f"limit={float(minimum_flange_z_m):.3f}m"
        )

    return max_joint_travel, minimum_z


def plan_piper_base_aim_at_target(
    start_joints_rad,
    target_xyz_m,
    minimum_flange_z_m=0.20,
):
    """Keep J2~J6 fixed and find the J1 angle that aims tool +Z at target."""
    start = np.asarray(start_joints_rad, dtype=np.float64).reshape(6)
    target = np.asarray(target_xyz_m, dtype=np.float64).reshape(3)
    max_delta = math.radians(
        float(np.clip(BOTTLE_BASE_AIM_MAX_DELTA_DEG, 1.0, 90.0))
    )
    lower = max(
        float(PIPER_JOINT_LIMITS_RAD[0, 0] + 0.02),
        float(start[0] - max_delta),
    )
    upper = min(
        float(PIPER_JOINT_LIMITS_RAD[0, 1] - 0.02),
        float(start[0] + max_delta),
    )
    step = math.radians(
        float(np.clip(BOTTLE_BASE_AIM_STEP_DEG, 0.10, 2.0))
    )
    sample_count = max(2, int(math.ceil((upper - lower) / step)) + 1)
    best = None

    for joint1 in np.linspace(lower, upper, sample_count):
        candidate = start.copy()
        candidate[0] = float(joint1)
        transform = piper_forward_kinematics(candidate)
        tool_forward_xy = transform[:2, 2]
        target_vector_xy = target[:2] - transform[:2, 3]
        if (
            np.linalg.norm(tool_forward_xy) < 0.10
            or np.linalg.norm(target_vector_xy) < 0.05
        ):
            continue

        tool_bearing = math.atan2(
            float(tool_forward_xy[1]),
            float(tool_forward_xy[0]),
        )
        target_bearing = math.atan2(
            float(target_vector_xy[1]),
            float(target_vector_xy[0]),
        )
        aim_error = abs(wrap_angle_rad(tool_bearing - target_bearing))
        score = aim_error + 1e-6 * abs(float(joint1 - start[0]))
        if best is None or score < best[0]:
            best = (
                score,
                candidate.copy(),
                transform.copy(),
                aim_error,
                target_bearing,
                tool_bearing,
            )

    if best is None:
        raise RuntimeError("没有可用的基座对准角。")

    aim_error_deg = math.degrees(best[3])
    if aim_error_deg > max(0.1, BOTTLE_BASE_AIM_MAX_ERROR_DEG):
        raise RuntimeError(
            "只转 J1 后夹爪前方仍未对准瓶子: "
            f"aim_error={aim_error_deg:.2f}deg"
        )

    max_travel, minimum_z = validate_piper_joint_path(
        start,
        best[1],
        minimum_flange_z_m=minimum_flange_z_m,
    )
    aligned_rpy_deg = ScipyRotation.from_matrix(
        best[2][:3, :3]
    ).as_euler("xyz", degrees=True)
    return {
        "alignment_joints": best[1],
        "aligned_transform": best[2],
        "aligned_rpy_deg": aligned_rpy_deg,
        "joint1_delta_deg": math.degrees(float(best[1][0] - start[0])),
        "aim_error_deg": aim_error_deg,
        "target_bearing_deg": math.degrees(best[4]),
        "tool_forward_bearing_deg": math.degrees(best[5]),
        "max_joint_travel_rad": max_travel,
        "minimum_z_m": minimum_z,
    }


def validate_piper_forward_down_joint_path(
    start_joints_rad,
    target_joints_rad,
    minimum_flange_z_m,
    sample_count=81,
):
    """Validate that a MOVE_J extension stays nearly forward and pose-stable."""
    start = np.asarray(start_joints_rad, dtype=np.float64).reshape(6)
    target = np.asarray(target_joints_rad, dtype=np.float64).reshape(6)
    max_joint_travel, minimum_z = validate_piper_joint_path(
        start,
        target,
        minimum_flange_z_m=minimum_flange_z_m,
        maximum_joint_travel_rad=(
            BOTTLE_FORWARD_DOWN_MAX_JOINT_TRAVEL_RAD
        ),
    )
    transforms = [
        piper_forward_kinematics(
            start + float(ratio) * (target - start)
        )
        for ratio in np.linspace(0.0, 1.0, max(3, int(sample_count)))
    ]
    positions = np.asarray(
        [transform[:3, 3] for transform in transforms],
        dtype=np.float64,
    )
    xy_direction = positions[-1, :2] - positions[0, :2]
    xy_length = float(np.linalg.norm(xy_direction))
    if xy_length < 0.05:
        raise RuntimeError("前下方轨迹的水平伸展距离过小。")
    xy_normal = np.array(
        [-xy_direction[1], xy_direction[0]],
        dtype=np.float64,
    ) / xy_length
    lateral = np.abs(
        (positions[:, :2] - positions[0, :2]) @ xy_normal
    )
    max_lateral = float(np.max(lateral))
    max_upward = max(
        0.0,
        float(np.max(positions[:, 2]) - positions[0, 2]),
    )
    start_rotation = ScipyRotation.from_matrix(transforms[0][:3, :3])
    orientation_deviations = [
        np.linalg.norm(
            (
                start_rotation.inv()
                * ScipyRotation.from_matrix(transform[:3, :3])
            ).as_rotvec()
        )
        for transform in transforms
    ]
    max_orientation_deg = math.degrees(
        float(np.max(orientation_deviations))
    )

    if (
        BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS
        and max_lateral > BOTTLE_FORWARD_DOWN_MAX_LATERAL_M
    ):
        raise RuntimeError(
            "前下方 MOVE_J 横向弯曲过大: "
            f"{max_lateral * 100.0:.1f}cm"
        )
    if (
        BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS
        and max_upward > BOTTLE_FORWARD_DOWN_MAX_UPWARD_M
    ):
        raise RuntimeError(
            "前下方 MOVE_J 上拱过大: "
            f"{max_upward * 100.0:.1f}cm"
        )
    if (
        BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS
        and max_orientation_deg
        > BOTTLE_FORWARD_DOWN_MAX_RPY_DEVIATION_DEG
    ):
        raise RuntimeError(
            "前下方 MOVE_J 中间姿态变化过大: "
            f"{max_orientation_deg:.1f}deg"
        )

    return {
        "max_joint_travel_rad": max_joint_travel,
        "minimum_z_m": minimum_z,
        "max_lateral_m": max_lateral,
        "max_upward_m": max_upward,
        "max_orientation_deviation_deg": max_orientation_deg,
    }


def validate_piper_locked_cartesian_path(
    start_pose,
    target_pose,
    start_joints_rad,
    sample_count=24,
    maximum_joint_step_rad=None,
    minimum_joint_margin_rad=0.0,
    return_metrics=False,
):
    """Verify every sample of a locked-RPY Cartesian segment has a nearby IK."""
    for key in ("roll", "pitch", "yaw"):
        if not math.isclose(
            float(start_pose[key]),
            float(target_pose[key]),
            rel_tol=0.0,
            abs_tol=1e-10,
        ):
            raise RuntimeError(
                f"锁姿态路径的 {key} 不一致。"
            )

    samples = max(2, int(sample_count))
    previous = np.asarray(
        start_joints_rad,
        dtype=np.float64,
    ).reshape(6)
    initial = previous.copy()
    max_joint_step = 0.0
    minimum_z = float("inf")
    minimum_joint_margin = float("inf")
    maximum_joint_step = (
        BOTTLE_LOCKED_PATH_MAX_JOINT_STEP_RAD
        if maximum_joint_step_rad is None
        else float(maximum_joint_step_rad)
    )

    for index in range(1, samples + 1):
        ratio = float(index) / float(samples)
        waypoint = dict(start_pose)
        for key in ("x", "y", "z"):
            waypoint[key] = (
                float(start_pose[key])
                + ratio
                * (float(target_pose[key]) - float(start_pose[key]))
            )

        solution = solve_piper_ik_pose(
            waypoint,
            seeds=[previous, initial, OBSERVATION_JOINTS_RAD],
        )
        joint_step = float(np.max(np.abs(solution - previous)))
        if joint_step > maximum_joint_step:
            raise RuntimeError(
                "锁姿态直线路径的 IK 解不连续: "
                f"sample={index}/{samples}, "
                f"joint_step={joint_step:.3f}rad > "
                f"limit={maximum_joint_step:.3f}rad"
            )
        max_joint_step = max(max_joint_step, joint_step)
        joint_margin = np.minimum(
            solution - PIPER_JOINT_LIMITS_RAD[:, 0],
            PIPER_JOINT_LIMITS_RAD[:, 1] - solution,
        )
        minimum_joint_margin = min(
            minimum_joint_margin,
            float(np.min(joint_margin)),
        )
        minimum_z = min(
            minimum_z,
            float(piper_forward_kinematics(solution)[2, 3]),
        )
        previous = solution

    if minimum_joint_margin < float(minimum_joint_margin_rad):
        raise RuntimeError(
            "锁姿态直线路径的关节余量不足: "
            f"margin={minimum_joint_margin:.3f}rad < "
            f"required={float(minimum_joint_margin_rad):.3f}rad"
        )

    if return_metrics:
        return {
            "max_joint_step_rad": max_joint_step,
            "minimum_z_m": minimum_z,
            "minimum_joint_margin_rad": minimum_joint_margin,
            "final_joints_rad": previous.copy(),
        }

    return max_joint_step, minimum_z


def load_gripper_center_offset():
    if not os.path.exists(GRIPPER_CENTER_OFFSET_FILE):
        return DEFAULT_GRIPPER_CENTER_OFFSET.copy(), "default"

    try:
        with open(
            GRIPPER_CENTER_OFFSET_FILE,
            "r",
            encoding="utf-8",
        ) as file:
            data = json.load(file)

        if data.get("enabled", True) is False:
            return (
                np.zeros(3, dtype=np.float64),
                GRIPPER_CENTER_OFFSET_FILE,
            )

        values = data.get(
            "offset_xyz_m",
            DEFAULT_GRIPPER_CENTER_OFFSET.tolist(),
        )
        return (
            np.array(values, dtype=np.float64),
            GRIPPER_CENTER_OFFSET_FILE,
        )

    except Exception:
        return DEFAULT_GRIPPER_CENTER_OFFSET.copy(), "default"


def load_piper_interface_class():
    """
    从真正安装的 piper_sdk 中加载接口类。

    旧代码把 /workspace/piper_ws/src/piper_SDK 插到 sys.path
    最前面，该目录可能只是一个不完整的命名空间包，因此会遮蔽
    ~/.local/lib/python3.10/site-packages 中实际安装的 piper_sdk。
    """
    workspace_sdk_path = os.path.abspath(
        os.path.join(workspace_dir, "src", "piper_SDK")
    )

    # 删除会遮蔽正式安装包的源码路径。
    cleaned_sys_path = []
    for path in sys.path:
        try:
            absolute_path = os.path.abspath(path)
        except Exception:
            absolute_path = path

        if absolute_path == workspace_sdk_path:
            continue

        cleaned_sys_path.append(path)

    sys.path[:] = cleaned_sys_path

    # PYTHONNOUSERSITE=1 只是不自动加入用户 site-packages；
    # 这里显式加入，不会影响 NumPy/OpenCV 的版本选择。
    python_version = (
        f"{sys.version_info.major}.{sys.version_info.minor}"
    )
    user_site_candidates = [
        site.getusersitepackages(),
        os.path.expanduser(
            f"~/.local/lib/python{python_version}/site-packages"
        ),
    ]

    for user_site in reversed(user_site_candidates):
        if (
            user_site
            and os.path.isdir(user_site)
            and user_site not in sys.path
        ):
            sys.path.insert(0, user_site)

    # 清除此前失败导入留下的不完整 namespace package。
    for module_name in list(sys.modules):
        if (
            module_name == "piper_sdk"
            or module_name.startswith("piper_sdk.")
        ):
            module = sys.modules.get(module_name)
            module_file = getattr(module, "__file__", None)
            module_path = str(getattr(module, "__path__", ""))

            if (
                module_file is None
                or workspace_sdk_path in str(module_file)
                or workspace_sdk_path in module_path
            ):
                sys.modules.pop(module_name, None)

    importlib.invalidate_caches()

    errors = []

    try:
        module_v2 = importlib.import_module(
            "piper_sdk.interface.piper_interface_v2"
        )
        interface_v2 = getattr(
            module_v2,
            "C_PiperInterface_V2",
        )
        return interface_v2, (
            "piper_sdk.interface.piper_interface_v2."
            "C_PiperInterface_V2"
        )
    except Exception as exc:
        errors.append(f"V2: {exc}")

    try:
        module_v1 = importlib.import_module(
            "piper_sdk.interface.piper_interface"
        )
        interface_v1 = getattr(
            module_v1,
            "C_PiperInterface",
        )
        return interface_v1, (
            "piper_sdk.interface.piper_interface."
            "C_PiperInterface"
        )
    except Exception as exc:
        errors.append(f"V1: {exc}")

    # 最后尝试正式包顶层导出。
    try:
        package = importlib.import_module("piper_sdk")

        if hasattr(package, "C_PiperInterface_V2"):
            return (
                getattr(package, "C_PiperInterface_V2"),
                "piper_sdk.C_PiperInterface_V2",
            )

        if hasattr(package, "C_PiperInterface"):
            return (
                getattr(package, "C_PiperInterface"),
                "piper_sdk.C_PiperInterface",
            )

        package_file = getattr(package, "__file__", None)
        package_path = getattr(package, "__path__", None)
        errors.append(
            "top-level: 类未导出；"
            f"file={package_file}, path={package_path}"
        )

    except Exception as exc:
        errors.append(f"top-level: {exc}")

    raise ImportError(
        "无法加载 Piper SDK 接口。"
        + " | ".join(errors)
        + f" | sys.path前5项={sys.path[:5]}"
    )


def read_sdk_end_pose():
    """
    使用额外的只读 SocketCAN 连接读取末端位姿。

    读取结束后主动 DisconnectPort，避免每次按 o 都残留 SDK
    后台线程或 CAN socket。
    """
    Piper, interface_source = load_piper_interface_class()
    piper = None

    try:
        piper = Piper(CAN_NAME)
        piper.ConnectPort()
        time.sleep(0.35)

        end_pose_message = piper.GetArmEndPoseMsgs()
        pose = end_pose_message.end_pose

        xyz = np.array(
            [
                int(pose.X_axis) / 1_000_000.0,
                int(pose.Y_axis) / 1_000_000.0,
                int(pose.Z_axis) / 1_000_000.0,
            ],
            dtype=np.float64,
        )
        rpy_deg = np.array(
            [
                int(pose.RX_axis) / 1000.0,
                int(pose.RY_axis) / 1000.0,
                int(pose.RZ_axis) / 1000.0,
            ],
            dtype=np.float64,
        )

        if not np.all(np.isfinite(xyz)):
            raise RuntimeError(
                f"SDK 返回了无效末端坐标: {xyz.tolist()}"
            )

        return (
            rotation_from_rpy_xyz(
                *np.deg2rad(rpy_deg)
            ),
            xyz,
            rpy_deg,
        )

    except Exception as exc:
        raise RuntimeError(
            f"读取 Piper 末端位姿失败，"
            f"接口={interface_source}: {exc}"
        ) from exc

    finally:
        if piper is not None:
            disconnect = getattr(
                piper,
                "DisconnectPort",
                None,
            )
            if callable(disconnect):
                try:
                    disconnect()
                except Exception:
                    pass


def request_sdk_can_control_mode(speed_percent=20):
    """Exit drag-teach and request CAN/MOVE_J without sending a target."""
    Piper, interface_source = load_piper_interface_class()
    piper = None

    try:
        piper = Piper(CAN_NAME)
        piper.ConnectPort()
        time.sleep(0.30)

        # CAN 0x150 byte2=0x02: end teaching record / exit drag-teach.
        piper.MotionCtrl_1(0x00, 0x00, 0x02)
        time.sleep(0.20)

        # CAN 0x151: enter CAN command control in MOVE_J mode.  This frame
        # contains no joint or Cartesian target and therefore cannot move the
        # arm by itself.
        speed = int(np.clip(round(float(speed_percent)), 1, 100))
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        time.sleep(0.30)

        status = piper.GetArmStatus().arm_status
        return {
            "ctrl_mode": int(status.ctrl_mode),
            "teach_status": int(status.teach_status),
            "arm_status": int(status.arm_status),
            "err_code": int(status.err_code),
            "interface": interface_source,
        }

    except Exception as exc:
        raise RuntimeError(
            "退出示教/请求 CAN 控制模式失败，"
            f"接口={interface_source}: {exc}"
        ) from exc

    finally:
        if piper is not None:
            disconnect = getattr(piper, "DisconnectPort", None)
            if callable(disconnect):
                try:
                    disconnect()
                except Exception:
                    pass


class InstructionSheetDetector:
    """第一阶段：用独立 YOLO 权重识别图纸上的目标瓶子照片。"""

    def __init__(self, model_path):
        self.model_path = str(model_path)
        self.model = None
        self.model_class_names = {}
        self.is_loaded = False
        self.device = YOLO_DEVICE or (
            "0" if torch.cuda.is_available() else "cpu"
        )
        self.predict_lock = Lock()

    def load_model(self):
        if not os.path.exists(self.model_path):
            raise RuntimeError(
                f"图纸 YOLO 权重不存在: {self.model_path}"
            )

        from ultralytics import YOLO

        print(f"加载图纸 YOLO 模型: {self.model_path}")
        self.model = YOLO(self.model_path)
        raw_names = self.model.names
        if isinstance(raw_names, dict):
            self.model_class_names = {
                int(class_id): normalize_custom_yolo_class_name(name)
                for class_id, name in raw_names.items()
            }
        else:
            self.model_class_names = {
                int(class_id): normalize_custom_yolo_class_name(name)
                for class_id, name in enumerate(raw_names)
            }

        required_names = set(CUSTOM_YOLO_GRASP_CLASSES)
        available_names = set(self.model_class_names.values())
        missing_names = sorted(required_names - available_names)
        supported_names = sorted(required_names & available_names)
        if not supported_names:
            raise RuntimeError(
                "图纸 YOLO 权重没有任何可抓取目标类别；"
                f"期望类别={sorted(required_names)}；"
                f"模型类别={self.model_class_names}"
            )
        if missing_names:
            print(
                "⚠ 图纸 YOLO 权重暂未包含全部六类目标，"
                f"缺少={missing_names}；"
                f"当前可用={supported_names}。"
                "换成六类权重后会自动识别新增类别。"
            )

        warmup_elapsed_s = warm_up_yolo_model(
            self.model,
            image_size=INSTRUCTION_YOLO_IMAGE_SIZE,
            confidence=INSTRUCTION_YOLO_RAW_CONFIDENCE,
            device=self.device,
        )
        self.is_loaded = True
        print(
            "✓ 图纸 YOLO 模型加载成功，classes="
            f"{self.model_class_names}, "
            f"warmup={warmup_elapsed_s:.2f}s"
        )
        return True

    def detect(self, frame):
        if frame is None or not self.is_loaded or self.model is None:
            return {
                "candidates": [],
                "overlay": None,
                "elapsed_s": 0.0,
            }

        start_time = time.time()
        with self.predict_lock:
            results = self.model.predict(
                frame,
                imgsz=INSTRUCTION_YOLO_IMAGE_SIZE,
                conf=INSTRUCTION_YOLO_RAW_CONFIDENCE,
                device=self.device,
                verbose=False,
            )

        candidates = []
        result = results[0]
        names = result.names
        if result.boxes is not None:
            for item in result.boxes:
                class_id = int(item.cls[0])
                class_name = str(
                    names.get(class_id, class_id)
                    if isinstance(names, dict)
                    else names[class_id]
                ).lower()
                raw_class_name = class_name
                class_name = normalize_custom_yolo_class_name(class_name)
                if class_name not in CUSTOM_YOLO_GRASP_CLASSES:
                    continue
                candidates.append(
                    {
                        "class_id": class_id,
                        "raw_class_name": raw_class_name,
                        "class_name": class_name,
                        "confidence": float(item.conf[0]),
                        "bbox": (
                            item.xyxy[0]
                            .detach()
                            .cpu()
                            .numpy()
                            .astype(np.float64)
                        ),
                    }
                )

        candidates.sort(
            key=lambda candidate: candidate["confidence"],
            reverse=True,
        )
        overlay = frame.copy()
        height, width = overlay.shape[:2]
        for candidate in candidates:
            x0, y0, x1, y1 = [
                int(round(value))
                for value in candidate["bbox"]
            ]
            x0 = max(0, min(width - 1, x0))
            x1 = max(0, min(width - 1, x1))
            y0 = max(0, min(height - 1, y0))
            y1 = max(0, min(height - 1, y1))
            accepted = (
                candidate["confidence"]
                >= INSTRUCTION_YOLO_ACCEPT_CONFIDENCE
            )
            color = (0, 255, 255) if accepted else (0, 128, 255)
            cv2.rectangle(
                overlay,
                (x0, y0),
                (x1, y1),
                color,
                2,
            )
            cv2.putText(
                overlay,
                (
                    f"SHEET {candidate['class_name']} "
                    f"{candidate['confidence']:.3f}"
                ),
                (x0, max(18, y0 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                2,
            )

        cv2.putText(
            overlay,
            (
                "Instruction sheet target "
                f"(threshold={INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:.2f})"
            ),
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )
        return {
            "candidates": candidates,
            "overlay": overlay,
            "elapsed_s": time.time() - start_time,
        }


class SimpleVisionDetector:
    """简化版视觉检测器。"""

    def __init__(self, output_dir):
        self.backend = VISION_BACKEND
        self.output_dir = output_dir
        self.model = None
        self.model_class_names = {}
        self.predictor = None
        self.is_loaded = False
        self.detection_target = "person"
        self.detection_running = False

        if not VISION_AVAILABLE:
            return

        if self.backend == "yolov8":
            self.model_path = YOLO_MODEL_PATH
            self.device = YOLO_DEVICE or (
                "0" if torch.cuda.is_available() else "cpu"
            )
        else:
            self.config_path = CONFIG_PATH
            self.dino_checkpoint = GROUNDED_CHECKPOINT
            self.sam_checkpoint = SAM_CHECKPOINT
            self.device = DEVICE

    def set_detection_target(self, target):
        self.detection_target = target
        print(f"检测目标已设置为: {target}")

    def check_model_files(self):
        if self.backend == "yolov8":
            if not os.path.exists(self.model_path):
                print(f"✗ YOLOv8模型不存在: {self.model_path}")
                return False
            return True

        files_to_check = [
            ("配置文件", self.config_path),
            ("GroundingDINO模型", self.dino_checkpoint),
        ]
        all_exist = True

        for file_name, file_path in files_to_check:
            if not os.path.exists(file_path):
                print(f"✗ {file_name}不存在: {file_path}")
                all_exist = False

        return all_exist

    def load_models_simple(self):
        if not VISION_AVAILABLE:
            return False

        if self.backend == "yolov8":
            try:
                from ultralytics import YOLO

                if not self.check_model_files():
                    return False

                print(f"加载 YOLOv8 模型: {self.model_path}")
                self.model = YOLO(self.model_path)
                raw_names = self.model.names
                if isinstance(raw_names, dict):
                    self.model_class_names = {
                        int(class_id): normalize_custom_yolo_class_name(name)
                        for class_id, name in raw_names.items()
                    }
                else:
                    self.model_class_names = {
                        int(class_id): normalize_custom_yolo_class_name(name)
                        for class_id, name in enumerate(raw_names)
                    }
                if (
                    self.detection_target in CUSTOM_YOLO_GRASP_CLASSES
                    and self.detection_target
                    not in set(self.model_class_names.values())
                ):
                    raise RuntimeError(
                        "当前 YOLO 权重不包含指定数据集类别 "
                        f"{self.detection_target}；"
                        f"模型类别={self.model_class_names}"
                    )
                warmup_elapsed_s = warm_up_yolo_model(
                    self.model,
                    image_size=YOLO_IMAGE_SIZE,
                    confidence=YOLO_CONFIDENCE,
                    device=self.device,
                )
                self.is_loaded = True
                print(
                    "✓ YOLOv8 模型加载成功，classes="
                    f"{self.model_class_names}, "
                    f"warmup={warmup_elapsed_s:.2f}s"
                )
                return True

            except Exception as exc:
                print(f"加载 YOLOv8 模型失败: {exc}")
                return False

        try:
            import GroundingDINO.groundingdino.datasets.transforms as T
            from GroundingDINO.groundingdino.models import build_model
            from GroundingDINO.groundingdino.util.slconfig import SLConfig
            from GroundingDINO.groundingdino.util.utils import clean_state_dict

            if not self.check_model_files():
                return False

            print("加载 Grounding DINO 模型...")
            args = SLConfig.fromfile(self.config_path)
            args.device = self.device

            required_params = {
                "hidden_dim": 256,
                "position_embedding": "sine",
                "pe_temperatureH": 20,
                "pe_temperatureW": 20,
                "return_interm_indices": [1, 2, 3],
                "backbone_freeze_keywords": None,
                "enc_layers": 6,
                "dec_layers": 6,
                "pre_norm": False,
                "dim_feedforward": 2048,
                "dropout": 0.0,
                "nheads": 8,
                "num_queries": 900,
                "query_dim": 4,
                "num_patterns": 0,
                "num_feature_levels": 4,
                "enc_n_points": 4,
                "dec_n_points": 4,
                "two_stage_type": "standard",
                "two_stage_bbox_embed_share": False,
                "two_stage_class_embed_share": False,
                "transformer_activation": "relu",
                "masks": False,
                "aux_loss": True,
                "set_cost_class": 2.0,
                "set_cost_bbox": 5.0,
                "set_cost_giou": 2.0,
                "cls_loss_coef": 1.0,
                "bbox_loss_coef": 5.0,
                "giou_loss_coef": 2.0,
                "focal_alpha": 0.25,
                "dn_labelbook_size": 2000,
                "max_text_len": 256,
                "tokenizer_type": "bert-base-uncased",
            }

            for key, value in required_params.items():
                if not hasattr(args, key):
                    setattr(args, key, value)

            self.model = build_model(args)
            checkpoint = torch.load(
                self.dino_checkpoint,
                map_location="cpu",
            )
            self.model.load_state_dict(
                clean_state_dict(checkpoint["model"]),
                strict=False,
            )
            self.model.eval()
            self.model.to(self.device)

            print("✓ Grounding DINO 模型加载成功")
            self.predictor = None
            self.is_loaded = True
            return True

        except Exception as exc:
            print(f"加载视觉模型失败: {exc}")
            return False

    def load_image_from_cv2_simple(self, image_cv):
        try:
            import GroundingDINO.groundingdino.datasets.transforms as T

            if image_cv is None or image_cv.size == 0:
                return None, None

            height, width = image_cv.shape[:2]
            if width > 640 or height > 480:
                scale = min(640 / width, 480 / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                image_cv = cv2.resize(
                    image_cv,
                    (new_width, new_height),
                )

            image_cv_rgb = cv2.cvtColor(
                image_cv,
                cv2.COLOR_BGR2RGB,
            )
            image_pil = PILImage.fromarray(image_cv_rgb)

            transform = T.Compose(
                [
                    T.RandomResize([512], max_size=512),
                    T.ToTensor(),
                    T.Normalize(
                        [0.485, 0.456, 0.406],
                        [0.229, 0.224, 0.225],
                    ),
                ]
            )

            image_tensor, _ = transform(image_pil, None)
            return image_pil, image_tensor

        except Exception as exc:
            print(f"加载图片失败: {exc}")
            return None, None

    def get_grounding_output_simple(self, image_tensor, text_prompt):
        caption = text_prompt.lower().strip()
        if not caption.endswith("."):
            caption += "."

        self.model = self.model.to(self.device)
        image_tensor = image_tensor.to(self.device)

        with torch.no_grad():
            try:
                outputs = self.model(
                    image_tensor[None],
                    captions=[caption],
                )
            except Exception as exc:
                print(f"Grounding DINO 推理错误: {exc}")
                return torch.empty(0, 4), []

        logits = outputs["pred_logits"].cpu().sigmoid()[0]
        boxes = outputs["pred_boxes"].cpu()[0]

        filt_mask = logits.max(dim=1)[0] > BOX_THRESHOLD
        logits_filt = logits[filt_mask]
        boxes_filt = boxes[filt_mask]

        if len(boxes_filt) == 0:
            return boxes_filt, []

        pred_phrases = [
            f"object({logit.max().item():.2f})"
            for logit in logits_filt
        ]
        return boxes_filt, pred_phrases

    def draw_simple_detection(
        self,
        frame,
        boxes,
        labels,
        confidences=None,
        text_prompt="",
    ):
        if frame is None:
            return None

        result_image = frame.copy()
        height, width = result_image.shape[:2]

        for index, (box, label) in enumerate(zip(boxes, labels)):
            x0, y0, x1, y1 = [int(value) for value in box]
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(width - 1, x1)
            y1 = min(height - 1, y1)

            color = (0, 255, 0)
            cv2.rectangle(
                result_image,
                (x0, y0),
                (x1, y1),
                color,
                2,
            )

            if confidences:
                label_text = f"{label} ({confidences[index]:.2f})"
            else:
                label_text = label

            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.5
            thickness = 1
            (text_width, text_height), _ = cv2.getTextSize(
                label_text,
                font,
                scale,
                thickness,
            )

            cv2.rectangle(
                result_image,
                (x0, y0 - text_height - 5),
                (x0 + text_width, y0),
                color,
                -1,
            )
            cv2.putText(
                result_image,
                label_text,
                (x0, y0 - 5),
                font,
                scale,
                (255, 255, 255),
                thickness,
            )

        if len(boxes) > 0:
            info_text = f"Objects: {len(boxes)}"
            info_color = (0, 255, 0)
        else:
            info_text = f"No: {text_prompt}"
            info_color = (0, 0, 255)

        cv2.putText(
            result_image,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            info_color,
            2,
        )
        cv2.putText(
            result_image,
            f"Target: {text_prompt}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        return result_image

    def estimate_roi_color(self, frame, box):
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = [int(round(value)) for value in box]
        x0 = max(0, min(width - 1, x0))
        x1 = max(0, min(width, x1))
        y0 = max(0, min(height - 1, y0))
        y1 = max(0, min(height, y1))

        if x1 <= x0 or y1 <= y0:
            return "unknown"

        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return "unknown"

        margin_x = max(1, int(roi.shape[1] * 0.2))
        margin_y = max(1, int(roi.shape[0] * 0.2))
        core = roi[
            margin_y:max(margin_y + 1, roi.shape[0] - margin_y),
            margin_x:max(margin_x + 1, roi.shape[1] - margin_x),
        ]
        if core.size == 0:
            core = roi

        hsv = cv2.cvtColor(core, cv2.COLOR_BGR2HSV)
        best_name = "unknown"
        best_count = 0

        for color_name, ranges in COLOR_HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                )
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_name = color_name
                best_count = count

        if best_count < max(20, int(hsv.shape[0] * hsv.shape[1] * 0.08)):
            return "unknown"

        return best_name

    def detect_colored_shape_frame(self, frame, text_prompt=None, start_time=None):
        if frame is None:
            return None

        if start_time is None:
            start_time = time.time()

        target_text = str(text_prompt or "").lower().strip()
        target_terms = [
            term.strip()
            for term in target_text.replace(",", ".").split(".")
            if term.strip()
        ]
        wants_block = any(term in {"block", "cube", "box"} for term in target_terms)
        wants_bottle = any(term == "bottle" for term in target_terms)
        accept_all = target_text in {"", "*", "all", "object"}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        segmentation_colors = (
            "red",
            "orange",
            "yellow",
            "green",
            "blue",
            "purple",
        )
        for color_name in segmentation_colors:
            for lower, upper in COLOR_HSV_RANGES[color_name]:
                color_mask |= cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                )

        kernel = np.ones((5, 5), dtype=np.uint8)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            color_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        boxes = []
        labels = []
        confidences = []
        height, width = frame.shape[:2]

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < MIN_COLOR_OBJECT_AREA_PX:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            if x <= 2 or y <= 2 or x + w >= width - 2 or y + h >= height - 2:
                continue

            rect_area = float(w * h)
            extent = area / max(rect_area, 1.0)
            aspect = max(w, h) / max(1.0, min(w, h))
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.035 * peri, True)
            vertex_count = len(approx)

            is_block = (
                0.45 <= extent <= 1.05
                and aspect <= 1.8
                and 4 <= vertex_count <= 10
            )
            is_bottle = aspect > 1.35 and extent < 0.85

            if wants_block and not is_block:
                continue
            if wants_bottle and not is_bottle:
                continue
            if not accept_all and not wants_block and not wants_bottle:
                continue

            label = "block" if is_block or wants_block else "bottle"
            box = np.array([x, y, x + w, y + h], dtype=np.float64)
            color_name = self.estimate_roi_color(frame, box)
            if color_name != "unknown":
                label = f"{color_name} {label}"

            # Keep this high enough to satisfy still/moving confirmation, but
            # below a real trained detector's typical confidence.
            confidence = min(0.88, 0.60 + area / float(width * height))
            boxes.append(box)
            labels.append(label)
            confidences.append(float(confidence))

        if not boxes:
            return None

        order = np.argsort(confidences)[::-1]
        boxes_array = np.vstack([boxes[index] for index in order])
        labels = [labels[index] for index in order]
        confidences = [confidences[index] for index in order]

        return {
            "boxes": boxes_array,
            "labels": labels,
            "confidences": confidences,
            "class_ids": [None] * len(labels),
            "class_names": [None] * len(labels),
            "text_prompt": text_prompt,
            "elapsed_s": time.time() - start_time,
        }

    def detect_frame_simple(self, frame, text_prompt=None):
        if text_prompt is None:
            text_prompt = self.detection_target

        if not self.is_loaded or self.model is None:
            return None

        if self.backend == "yolov8":
            try:
                start_time = time.time()
                results = self.model.predict(
                    frame,
                    imgsz=YOLO_IMAGE_SIZE,
                    conf=YOLO_CONFIDENCE,
                    device=self.device,
                    verbose=False,
                )

                result = results[0]
                names = result.names
                boxes = []
                labels = []
                confidences = []
                class_ids = []
                class_names = []
                target_text = str(text_prompt or "").lower().strip()
                exact_custom_target = (
                    target_text in CUSTOM_YOLO_GRASP_CLASSES
                )
                accept_all = target_text in {"", "*", "all", "object"}
                target_terms = [
                    term.strip()
                    for term in target_text.replace(",", ".").split(".")
                    if term.strip()
                ]

                if result.boxes is not None:
                    for item in result.boxes:
                        class_id = int(item.cls[0])
                        label = str(names.get(class_id, class_id))
                        raw_label_lower = label.lower()
                        label_lower = normalize_custom_yolo_class_name(
                            raw_label_lower
                        )

                        if (
                            exact_custom_target
                            and label_lower != target_text
                        ):
                            continue

                        if (
                            not exact_custom_target
                            and not accept_all
                            and target_terms
                        ):
                            searchable_label = " ".join(
                                {
                                    raw_label_lower,
                                    label_lower,
                                    raw_label_lower.replace("_", " "),
                                    label_lower.replace("_", " "),
                                }
                            )
                            if not any(term in searchable_label for term in target_terms):
                                continue

                        box_xyxy = (
                            item.xyxy[0]
                            .detach()
                            .cpu()
                            .numpy()
                            .astype(np.float64)
                        )
                        color_name = self.estimate_roi_color(
                            frame,
                            box_xyxy,
                        )
                        boxes.append(box_xyxy)
                        class_ids.append(class_id)
                        class_names.append(label_lower)
                        if label_lower in CUSTOM_YOLO_GRASP_CLASSES:
                            labels.append(label_lower)
                        elif color_name != "unknown":
                            labels.append(f"{color_name} {label}")
                        else:
                            labels.append(label)
                        confidences.append(float(item.conf[0]))

                if boxes:
                    boxes_array = np.vstack(boxes)
                else:
                    if USE_COLOR_SHAPE_FALLBACK and not exact_custom_target:
                        fallback = self.detect_colored_shape_frame(
                            frame,
                            text_prompt,
                            start_time=start_time,
                        )
                        if fallback is not None:
                            return fallback
                    boxes_array = np.empty((0, 4), dtype=np.float64)

                return {
                    "boxes": boxes_array,
                    "labels": labels,
                    "confidences": confidences,
                    "class_ids": class_ids,
                    "class_names": class_names,
                    "text_prompt": text_prompt,
                    "elapsed_s": time.time() - start_time,
                }

            except Exception as exc:
                print(f"YOLOv8检测失败: {exc}")
                return None

        try:
            start_time = time.time()
            image_pil, image_tensor = self.load_image_from_cv2_simple(
                frame
            )

            if image_pil is None or image_tensor is None:
                return None

            boxes_filt, pred_phrases = (
                self.get_grounding_output_simple(
                    image_tensor,
                    text_prompt,
                )
            )

            height, width = frame.shape[:2]
            labels = []
            confidences = []

            if len(boxes_filt) == 0:
                boxes = np.empty((0, 4), dtype=np.float64)
            else:
                boxes_filt_original = boxes_filt.clone()

                for index in range(boxes_filt.size(0)):
                    boxes_filt_original[index] = (
                        boxes_filt_original[index]
                        * torch.Tensor([width, height, width, height])
                    )
                    boxes_filt_original[index][:2] -= (
                        boxes_filt_original[index][2:] / 2
                    )
                    boxes_filt_original[index][2:] += (
                        boxes_filt_original[index][:2]
                    )

                boxes = boxes_filt_original.cpu().numpy()

                for phrase in pred_phrases:
                    if "(" in phrase and ")" in phrase:
                        _, confidence_part = phrase.rsplit("(", 1)
                        labels.append("object")
                        try:
                            confidences.append(
                                float(confidence_part.rstrip(")"))
                            )
                        except Exception:
                            confidences.append(0.5)
                    else:
                        labels.append(phrase)
                        confidences.append(0.5)

            return {
                "boxes": boxes,
                "labels": labels,
                "confidences": confidences,
                "text_prompt": text_prompt,
                "elapsed_s": time.time() - start_time,
            }

        except Exception as exc:
            print(f"结构化检测失败: {exc}")
            return None

    def process_single_frame_fast(self, frame, text_prompt=None):
        detection = self.detect_frame_simple(frame, text_prompt)

        if detection is None:
            return None

        try:
            result_image = self.draw_simple_detection(
                frame,
                detection["boxes"],
                detection["labels"],
                detection["confidences"],
                detection["text_prompt"],
            )

            cv2.putText(
                result_image,
                f"Time: {detection['elapsed_s']:.2f}s",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            return result_image

        except Exception as exc:
            print(f"快速检测失败: {exc}")
            return None


COLOR_ALIASES = {
    "red": "red",
    "红": "red",
    "红色": "red",
    "orange": "orange",
    "橙": "orange",
    "橙色": "orange",
    "yellow": "yellow",
    "黄": "yellow",
    "黄色": "yellow",
    "green": "green",
    "绿": "green",
    "绿色": "green",
    "blue": "blue",
    "蓝": "blue",
    "蓝色": "blue",
    "purple": "purple",
    "紫": "purple",
    "紫色": "purple",
    "white": "white",
    "白": "white",
    "白色": "white",
    "black": "black",
    "黑": "black",
    "黑色": "black",
}

TARGET_CLASS_ALIASES = {
    "0": 0,
    "1": 0,
    "obj_01": 0,
    "obj01": 0,
    "bottle": 0,
    "瓶子": 0,
    "2": 1,
    "obj_02": 1,
    "obj02": 1,
    "block": 1,
    "cube": 1,
    "box": 1,
    "方块": 1,
    "物块": 1,
    "正方体": 1,
    "积木": 1,
}


def normalize_target_class(target):
    parsed = parse_target_spec(target)
    return None if parsed is None else parsed["class_id"]


def parse_target_spec(target, color=None):
    if target is None and color is None:
        return None

    words = []
    if color is not None:
        words.append(str(color))
    if target is not None:
        words.append(str(target))

    raw_text = " ".join(words).strip().lower()
    if not raw_text:
        return None

    compact_target = (
        raw_text.replace("-", "_").replace(" ", "_")
    )
    compact_target = normalize_custom_yolo_class_name(compact_target)
    if compact_target in {"qr", "qrcode", "qr_code", "二维码"}:
        raise ValueError(
            "qr_code 只用于识别/索引，不是可抓取物体类别。"
        )

    normalized = (
        raw_text
        .replace("-", " ")
        .replace("_", " ")
        .replace(",", " ")
        .replace("的", " ")
    )
    tokens = [token for token in normalized.split() if token]

    target_color = None
    class_id = None

    for token in tokens:
        if token in COLOR_ALIASES:
            target_color = COLOR_ALIASES[token]
            continue

        alias_token = token
        if alias_token in TARGET_CLASS_ALIASES:
            class_id = TARGET_CLASS_ALIASES[alias_token]
            continue

        # Chinese phrases often arrive without spaces, e.g. 红色瓶子.
        for color_alias in sorted(COLOR_ALIASES, key=len, reverse=True):
            canonical = COLOR_ALIASES[color_alias]
            if color_alias and color_alias in alias_token:
                target_color = canonical
                alias_token = alias_token.replace(color_alias, "")
                break

        if alias_token in TARGET_CLASS_ALIASES:
            class_id = TARGET_CLASS_ALIASES[alias_token]
            continue

        for target_alias, target_id in TARGET_CLASS_ALIASES.items():
            if target_alias and target_alias in alias_token:
                class_id = target_id
                break
        if class_id is not None:
            continue

    custom_name = None
    for name, spec in CUSTOM_YOLO_GRASP_CLASSES.items():
        if compact_target == name:
            custom_name = name
            class_id = int(spec["grasp_class_id"])
            target_color = str(spec["color"])
            break

    if custom_name is None and target_color is not None:
        candidates = [
            (name, spec)
            for name, spec in CUSTOM_YOLO_GRASP_CLASSES.items()
            if (
                str(spec["color"]) == target_color
                and (
                    class_id is None
                    or int(spec["grasp_class_id"]) == int(class_id)
                )
            )
        ]
        if len(candidates) == 1:
            custom_name, spec = candidates[0]
            class_id = int(spec["grasp_class_id"])

    if custom_name is None and class_id == 0:
        candidate_name = f"{target_color}_bottle"
        if candidate_name in CUSTOM_YOLO_GRASP_CLASSES:
            custom_name = candidate_name
    if custom_name is None and class_id == 1:
        candidate_name = f"{target_color}_block"
        if candidate_name in CUSTOM_YOLO_GRASP_CLASSES:
            custom_name = candidate_name

    if class_id is None:
        raise ValueError(
            "目标应为 green_bottle / orange_bottle / purple_bottle / "
            "yellow_block / blue_block / red_block "
            "（也可写颜色+瓶子/物块），"
            f"收到: {target}"
        )

    return {
        "class_id": class_id,
        "color": target_color,
        "model_class_name": custom_name,
        "dataset_class_id": (
            None
            if custom_name is None
            else int(
                CUSTOM_YOLO_GRASP_CLASSES[custom_name][
                    "dataset_class_id"
                ]
            )
        ),
    }


def parse_dataset_target_spec(target):
    text = str(target).strip().lower()
    if not text:
        raise ValueError("--dataset-class 后必须跟类别 ID 或名称。")

    if text.isdigit():
        dataset_class_id = int(text)
        class_name = CUSTOM_YOLO_DATASET_ID_TO_NAME.get(dataset_class_id)
        if class_name is None:
            if dataset_class_id in CUSTOM_YOLO_NON_GRASPABLE_CLASSES:
                raise ValueError(
                    f"数据集 class {dataset_class_id}="
                    f"{CUSTOM_YOLO_NON_GRASPABLE_CLASSES[dataset_class_id]} "
                    "不允许进入抓取路径。"
                )
            raise ValueError(
                "可抓取的数据集 class ID 只有 0/1/2/3/4/5，"
                f"收到: {target}"
            )
        spec = CUSTOM_YOLO_GRASP_CLASSES[class_name]
        return {
            "class_id": int(spec["grasp_class_id"]),
            "color": str(spec["color"]),
            "model_class_name": class_name,
            "dataset_class_id": dataset_class_id,
        }

    parsed = parse_target_spec(text)
    if parsed["model_class_name"] is None:
        raise ValueError(
            "--dataset-class 必须是 0/1/2/3/4/5 或 "
            "green_bottle/orange_bottle/purple_bottle/"
            "yellow_block/blue_block/red_block。"
        )
    return parsed


def build_target_prompt(class_id, model_class_name=None):
    if model_class_name is not None:
        return str(model_class_name)
    return TARGET_CLASS_PROMPTS[int(class_id)]


def parse_runtime_args(argv):
    runtime = {
        "target_class_id": None,
        "target_color": None,
        "target_model_class_name": None,
        "target_dataset_class_id": None,
        "auto_grasp": os.getenv(
            "WRIST_AUTO_GRASP",
            "0",
        ).lower() in {"1", "true", "yes", "on"},
        "auto_preview": os.getenv(
            "WRIST_AUTO_PREVIEW",
            "0",
        ).lower() in {"1", "true", "yes", "on"},
        "auto_instruction_target": os.getenv(
            "WRIST_AUTO_INSTRUCTION_TARGET",
            "1",
        ).lower() in {"1", "true", "yes", "on"},
        "auto_delay_s": float(os.getenv("WRIST_AUTO_DELAY_S", "0.3")),
        "auto_zero_observe": os.getenv(
            "WRIST_AUTO_ZERO_OBSERVE",
            "0",
        ).lower() in {"1", "true", "yes", "on"},
        "auto_move_observe": os.getenv(
            "WRIST_AUTO_MOVE_OBSERVE",
            "1",
        ).lower() in {"1", "true", "yes", "on"},
        "auto_zero_duration_s": float(
            os.getenv("WRIST_AUTO_ZERO_DURATION_S", "1.4")
        ),
        "auto_observe_duration_s": float(
            os.getenv("WRIST_AUTO_OBSERVE_DURATION_S", "1.4")
        ),
    }

    env_target = os.getenv("WRIST_AUTO_TARGET", "").strip()
    env_color = os.getenv("WRIST_AUTO_COLOR", "").strip()
    if env_target or env_color:
        parsed = parse_target_spec(env_target, env_color or None)
        runtime["target_class_id"] = parsed["class_id"]
        runtime["target_color"] = parsed["color"]
        runtime["target_model_class_name"] = parsed["model_class_name"]
        runtime["target_dataset_class_id"] = parsed["dataset_class_id"]

    pending_color = None
    ros_args = []
    index = 0
    while index < len(argv):
        arg = argv[index]

        if arg == "--color":
            if index + 1 >= len(argv):
                raise ValueError("--color 后必须跟颜色")
            pending_color = argv[index + 1]
            if runtime["target_class_id"] is not None:
                parsed = parse_target_spec(
                    TARGET_CLASS_PROMPTS[runtime["target_class_id"]],
                    pending_color,
                )
                runtime["target_color"] = parsed["color"]
                runtime["target_model_class_name"] = parsed[
                    "model_class_name"
                ]
                runtime["target_dataset_class_id"] = parsed[
                    "dataset_class_id"
                ]
            index += 2
            continue

        if arg.startswith("--color="):
            pending_color = arg.split("=", 1)[1]
            if runtime["target_class_id"] is not None:
                parsed = parse_target_spec(
                    TARGET_CLASS_PROMPTS[runtime["target_class_id"]],
                    pending_color,
                )
                runtime["target_color"] = parsed["color"]
                runtime["target_model_class_name"] = parsed[
                    "model_class_name"
                ]
                runtime["target_dataset_class_id"] = parsed[
                    "dataset_class_id"
                ]
            index += 1
            continue

        if arg in {"--dataset-class", "--yolo-class"}:
            if index + 1 >= len(argv):
                raise ValueError(f"{arg} 后必须跟类别 ID 或名称")
            parsed = parse_dataset_target_spec(argv[index + 1])
            runtime["target_class_id"] = parsed["class_id"]
            runtime["target_color"] = parsed["color"]
            runtime["target_model_class_name"] = parsed[
                "model_class_name"
            ]
            runtime["target_dataset_class_id"] = parsed[
                "dataset_class_id"
            ]
            index += 2
            continue

        if arg.startswith("--dataset-class=") or arg.startswith(
            "--yolo-class="
        ):
            parsed = parse_dataset_target_spec(arg.split("=", 1)[1])
            runtime["target_class_id"] = parsed["class_id"]
            runtime["target_color"] = parsed["color"]
            runtime["target_model_class_name"] = parsed[
                "model_class_name"
            ]
            runtime["target_dataset_class_id"] = parsed[
                "dataset_class_id"
            ]
            index += 1
            continue

        if arg in {"--target", "--object"}:
            if index + 1 >= len(argv):
                raise ValueError(f"{arg} 后必须跟目标名称")
            parsed = parse_target_spec(argv[index + 1], pending_color)
            runtime["target_class_id"] = parsed["class_id"]
            runtime["target_color"] = parsed["color"]
            runtime["target_model_class_name"] = parsed[
                "model_class_name"
            ]
            runtime["target_dataset_class_id"] = parsed[
                "dataset_class_id"
            ]
            index += 2
            continue

        if arg.startswith("--target="):
            parsed = parse_target_spec(
                arg.split("=", 1)[1],
                pending_color,
            )
            runtime["target_class_id"] = parsed["class_id"]
            runtime["target_color"] = parsed["color"]
            runtime["target_model_class_name"] = parsed[
                "model_class_name"
            ]
            runtime["target_dataset_class_id"] = parsed[
                "dataset_class_id"
            ]
            index += 1
            continue

        if arg in {"--auto-grasp", "--auto-grab"}:
            runtime["auto_grasp"] = True
            index += 1
            continue

        if arg == "--auto-preview":
            runtime["auto_preview"] = True
            index += 1
            continue

        if arg in {"--instruction-sheet", "--auto-instruction-target"}:
            runtime["auto_instruction_target"] = True
            index += 1
            continue

        if arg in {
            "--no-instruction-sheet",
            "--manual-target-only",
        }:
            runtime["auto_instruction_target"] = False
            index += 1
            continue

        if arg == "--auto-delay":
            if index + 1 >= len(argv):
                raise ValueError("--auto-delay 后必须跟秒数")
            runtime["auto_delay_s"] = float(argv[index + 1])
            index += 2
            continue

        if arg.startswith("--auto-delay="):
            runtime["auto_delay_s"] = float(arg.split("=", 1)[1])
            index += 1
            continue

        if arg == "--zero-duration":
            if index + 1 >= len(argv):
                raise ValueError("--zero-duration 后必须跟秒数")
            runtime["auto_zero_duration_s"] = float(argv[index + 1])
            index += 2
            continue

        if arg.startswith("--zero-duration="):
            runtime["auto_zero_duration_s"] = float(arg.split("=", 1)[1])
            index += 1
            continue

        if arg == "--observe-duration":
            if index + 1 >= len(argv):
                raise ValueError("--observe-duration 后必须跟秒数")
            runtime["auto_observe_duration_s"] = float(argv[index + 1])
            index += 2
            continue

        if arg.startswith("--observe-duration="):
            runtime["auto_observe_duration_s"] = float(arg.split("=", 1)[1])
            index += 1
            continue

        if arg == "--skip-zero-observe":
            runtime["auto_zero_observe"] = False
            index += 1
            continue

        if arg == "--skip-observe":
            runtime["auto_move_observe"] = False
            index += 1
            continue

        ros_args.append(arg)
        index += 1

    return runtime, ros_args


class PiperState(Enum):
    INITIALIZE_CAN = auto()
    WAIT_FOR_CAN = auto()
    READ_QR = auto()
    SET_TARGET_CLASS = auto()
    MOVE_TO_SCAN_POSE = auto()
    SCAN_NEXT_POINT = auto()
    DETECT_TARGET = auto()
    FINISH_CURRENT_SHORT_MOVE = auto()
    WAIT_ARM_STILL = auto()
    CONFIRM_TARGET = auto()
    COMPUTE_TARGET_3D = auto()
    MOVE_J_TO_PREGRASP = auto()
    SECOND_VISUAL_LOCALIZATION = auto()
    CORRECT_TARGET_POSITION = auto()
    MOVE_L_DOWN = auto()
    CLOSE_GRIPPER = auto()
    VERIFY_GRASP = auto()
    MOVE_L_UP = auto()
    MOVE_J_TO_PLACE = auto()
    FINISH = auto()
    INITIALIZE_ARM = auto()
    WAIT_FOR_SERVICE = auto()
    ENABLE_ARM = auto()
    LOAD_VISION_MODELS = auto()
    MOVE_TO_HOME = auto()
    MOVE_TO_TARGET = auto()
    IDLE = auto()
    ERROR = auto()
    SKIP_ARM = auto()


class PiperController(Node):
    def __init__(self, runtime_options=None):
        super().__init__("piper_controller")
        self.runtime_options = runtime_options or {}

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        # 默认使用本次 640x480 标定内参；camera_info 回调仍会保存
        # RealSense 驱动发布的实时内参，便于比较和必要时切换。
        self.camera_matrix = (
            CALIBRATED_CAMERA_MATRIX.copy()
            if USE_CALIBRATED_INTRINSICS
            else None
        )
        self.camera_dist_coeffs = CALIBRATED_DIST_COEFFS.copy()
        self.ros_camera_matrix = None
        self.point_cloud = None

        self._logged_color_ready = False
        self._logged_depth_ready = False
        self._logged_camera_info_ready = False

        self.output_dir = os.path.join(
            current_dir,
            "detection_results",
        )
        os.makedirs(self.output_dir, exist_ok=True)

        self.vision_detector = (
            SimpleVisionDetector(self.output_dir)
            if VISION_AVAILABLE
            else None
        )
        self.instruction_detector = (
            InstructionSheetDetector(
                INSTRUCTION_YOLO_MODEL_PATH
            )
            if INSTRUCTION_YOLO_AVAILABLE
            else None
        )

        # 多线程通信队列
        self.vision_queue_in = queue.Queue(maxsize=1)
        self.vision_queue_out = queue.Queue(maxsize=1)

        self.detection_active = False
        self.qr_detector = cv2.QRCodeDetector()
        self.qr_history = []
        self.target_class_id = None
        self.target_color = None
        self.target_model_class_name = None
        self.target_dataset_class_id = None
        self.target_qr_code = None
        self.detection_target = os.getenv(
            "WRIST_DEFAULT_TARGET_PROMPT",
            TARGET_CLASS_PROMPTS[0],
        )
        self.detection_history = []
        self.last_detection_filter_log_time = 0.0
        self.last_detection_result = None
        self.last_wrist_preview = None
        self.last_wrist_preview_path = None
        self.last_instruction_preview = None
        self.last_instruction_preview_path = None

        # 状态控制
        self.state = PiperState.INITIALIZE_CAN
        self.prev_state = None
        self.state_lock = Lock()

        self.node_process = None
        self.camera_process = None

        self.enable_response_received = False
        self.enable_request_pending = False
        self.arm_enabled = False
        self.arm_faulted = False
        self.last_arm_status_code = None
        self.last_arm_error_code = None
        self.last_ctrl_mode = None
        self.last_mode_feedback = None
        self.last_motion_status = None
        self.can_control_stable_since = None
        self.last_logged_control_feedback = None
        self.last_control_feedback_log_time = 0.0
        self.last_joint_positions = None
        self.joint_feedback_lock = Lock()
        self.last_gripper_position_m = None
        self.last_gripper_effort_nm = None
        self.last_gripper_feedback_at = None
        self.last_end_pose_rotation = None
        self.last_end_pose_xyz = None
        self.last_end_pose_rpy_deg = None
        self.last_end_pose_received_at = None
        self.end_pose_feedback_lock = Lock()
        self.gripper_hold_guard = GripperHoldGuard()
        self.last_gripper_hold_clamp_log_at = 0.0

        self.state_entry_time = time.time()
        self.skip_arm_control = False

        # 相机与 OpenCV 窗口状态
        self.camera_start_time = time.time()
        self.camera_timeout_logged = False
        self.ui_initialized = False
        self.ui_error_logged = False
        self.last_camera_save_time = 0.0

        self.ui_available = bool(
            os.environ.get("DISPLAY")
            or os.environ.get("WAYLAND_DISPLAY")
        )

        # CAN 异步配置标志位
        self.can_setup_done = False
        self.can_setup_success = False
        self.vision_model_loading = False
        self.vision_model_load_done = not VISION_AVAILABLE
        self.vision_model_load_success = False
        self.vision_model_load_error = None
        self.vision_model_lock = Lock()

        self.grasp_running = False
        self.auto_grasp_requested = bool(
            self.runtime_options.get("auto_grasp", False)
        )
        self.auto_preview_requested = bool(
            self.runtime_options.get("auto_preview", False)
        )
        self.auto_instruction_target = bool(
            self.runtime_options.get(
                "auto_instruction_target",
                True,
            )
        )
        self.auto_action_done = False
        self.auto_action_requested_at = None
        self.auto_action_delay_s = float(
            self.runtime_options.get("auto_delay_s", 0.8)
        )
        self.startup_move_observe = os.getenv(
            "WRIST_STARTUP_MOVE_OBSERVE",
            "1",
        ).lower() in {"1", "true", "yes", "on"}
        self.startup_observe_done = False
        self.startup_observe_running = False
        self.auto_zero_observe = bool(
            self.runtime_options.get("auto_zero_observe", False)
        )
        self.auto_move_observe = bool(
            self.runtime_options.get("auto_move_observe", True)
        )
        self.auto_zero_duration_s = float(
            self.runtime_options.get("auto_zero_duration_s", 1.4)
        )
        self.auto_observe_duration_s = float(
            self.runtime_options.get("auto_observe_duration_s", 1.4)
        )
        self.observation_joint_speed_percent = float(
            os.getenv("WRIST_OBSERVATION_JOINT_SPEED_PERCENT", "20")
        )
        self.observation_joint_timeout_s = float(
            os.getenv("WRIST_OBSERVATION_JOINT_TIMEOUT_S", "20")
        )
        self.observation_joint_tolerance_rad = float(
            os.getenv("WRIST_OBSERVATION_JOINT_TOLERANCE_RAD", "0.035")
        )
        self.can_control_wait_timeout_s = float(
            os.getenv("WRIST_CAN_CONTROL_WAIT_TIMEOUT_S", "30")
        )
        self.auto_sequence_running = False
        self.pending_terminal_target = None
        self.terminal_target_lock = Lock()
        self.terminal_input_active = False
        self.pre_grasp_enter_event = Event()
        self.pre_grasp_enter_lock = Lock()
        self.pre_grasp_enter_pending = False
        self.pre_grasp_enter_target_text = None
        self.pre_grasp_enter_action_text = None

        # 防止 OpenCV 按键连发在短时间内创建多个识别线程。
        self.last_preview_trigger_time = 0.0
        self.preview_key_debounce_s = 1.0

        (
            self.gripper_center_offset,
            self.gripper_offset_source,
        ) = load_gripper_center_offset()

        self.home_position = {
            "x": -0.001868,
            "y": -0.104469,
            "z": 0.337680,
            "roll": math.radians(-172.601),
            "pitch": math.radians(56.854),
            "yaw": math.radians(96.857),
            "gripper": OPEN_GRIPPER_M,
        }

        self.target_position = {
            "x": 0.055831,
            "y": 0.001209,
            "z": 0.329083,
            "roll": math.radians(180.000),
            "pitch": math.radians(63.623),
            "yaw": math.radians(-178.758),
            "gripper": 0.08,
        }

        self.grasp_pose = {
            "roll": math.radians(float(GRASP_RPY_DEG[0])),
            "pitch": math.radians(float(GRASP_RPY_DEG[1])),
            "yaw": math.radians(float(GRASP_RPY_DEG[2])),
        }

        runtime_target = self.runtime_options.get("target_class_id")
        if runtime_target is not None:
            self.set_target_class(
                runtime_target,
                color=self.runtime_options.get("target_color"),
                model_class_name=self.runtime_options.get(
                    "target_model_class_name"
                ),
            )

        self.setup_ros_components()
        self.display_welcome_message()
        self.initialize_ui()

        self.get_logger().info("开始启动左侧相机节点...")
        self.start_camera_node()

        # 启动后台视觉推理线程
        if VISION_AVAILABLE:
            self.inference_thread = Thread(
                target=self.vision_inference_worker,
                daemon=True,
            )
            self.inference_thread.start()
            self.start_vision_model_loading()

        self.terminal_input_thread = Thread(
            target=self.terminal_input_worker,
            daemon=True,
        )
        self.terminal_input_thread.start()

    def setup_ros_components(self):
        self.enable_client = self.create_client(
            Enable,
            "/enable_srv",
        )
        self.pos_pub = self.create_publisher(
            PosCmd,
            "/pos_cmd",
            10,
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            "/joint_ctrl_single",
            10,
        )
        self.prepare_move_j_pub = self.create_publisher(
            Bool,
            "/prepare_move_j",
            10,
        )
        self.status_sub = self.create_subscription(
            PiperStatusMsg,
            "/arm_status",
            self.status_callback,
            10,
        )
        self.joint_feedback_sub = self.create_subscription(
            JointState,
            "/joint_states_single",
            self.joint_feedback_callback,
            qos_profile_sensor_data,
        )
        self.end_pose_feedback_sub = self.create_subscription(
            Pose,
            "/end_pose",
            self.end_pose_feedback_callback,
            qos_profile_sensor_data,
        )
        self.color_image_sub = self.create_subscription(
            Image,
            COLOR_TOPIC,
            self.color_image_callback,
            qos_profile_sensor_data,
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        # 主循环定时器 30 Hz
        self.main_timer = self.create_timer(
            0.033,
            self.main_update_loop,
        )

    def display_welcome_message(self):
        print("=" * 60)
        print(
            "Piper 机械臂控制系统 "
            "(异步防卡死视觉版 - 含 CAN 自动配置)"
        )
        print("=" * 60)
        print(
            "控制说明: h/t: 旧固定点位已禁用 | "
            "r: 读取二维码 | 1-2: 手动选目标 | "
            "o: 腕部相机识别预览 | g: 图纸确认后腕部相机抓取 | "
            "d: DINO检测开关 | c: 修改DINO目标 | "
            "k: 跳过机械臂 | q: 退出"
        )
        print(
            "双模型自动模式: ./run_grasp_single.sh --auto-grasp；"
            "自动流程: 先到观察位 -> 按 Enter 开始图纸YOLO -> "
            "识别六类目标 -> 再按 Enter 开始实物YOLO定位/抓取。"
            "--dataset-class 0/1/2/3/4/5 仅保留用于调试。"
        )

        serial_text = CAMERA1_SERIAL
        print(
            f"腕部相机: serial={serial_text}, "
            f"color={COLOR_TOPIC}, "
            f"aligned_depth={DEPTH_TOPIC}"
        )
        print(
            "手眼矩阵: 使用本次 Tsai camera->gripper；"
            f"夹爪中心offset: "
            f"{self.gripper_center_offset.tolist()} "
            f"({self.gripper_offset_source})"
        )
        print(
            f"标定板: corners={CALIBRATION_BOARD_CORNERS}, "
            f"square={CALIBRATION_SQUARE_SIZE_M:.3f}m"
        )
        print(
            "标定内参模式: "
            f"{'固定标定内参' if USE_CALIBRATED_INTRINSICS else 'ROS camera_info'}"
        )
        print(
            "标定内参 K: "
            f"fx={CALIBRATED_CAMERA_MATRIX[0, 0]:.8f}, "
            f"fy={CALIBRATED_CAMERA_MATRIX[1, 1]:.8f}, "
            f"cx={CALIBRATED_CAMERA_MATRIX[0, 2]:.8f}, "
            f"cy={CALIBRATED_CAMERA_MATRIX[1, 2]:.8f}"
        )
        print(
            f"视觉后端: {VISION_BACKEND}, "
            f"实物YOLO={YOLO_MODEL_PATH}, "
            f"YOLO可用={YOLO_AVAILABLE}"
        )
        print(
            "图纸照片YOLO: "
            f"model={INSTRUCTION_YOLO_MODEL_PATH}, "
            f"available={INSTRUCTION_YOLO_AVAILABLE}, "
            f"threshold={INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:.2f}, "
            f"confirm_frames={max(1, INSTRUCTION_CONFIRM_FRAMES)}"
        )
        print(
            f"OpenCV: version={cv2.__version__}, "
            f"path={getattr(cv2, '__file__', 'unknown')}"
        )
        print(
            "简单抓取姿态: "
            f"roll={SIMPLE_GRASP_ROLL_DEG:.1f}°, "
            f"pitch={SIMPLE_GRASP_PITCH_DEG:.1f}°, "
            f"primary_yaw={SIMPLE_GRASP_PRIMARY_YAW_DEG:.1f}°, "
            f"allow_flipped_yaw={SIMPLE_GRASP_ALLOW_FLIPPED_YAW}, "
            f"safe_rotate_z={SAFE_SIMPLE_POSE_Z_M:.3f}m"
        )
        print(
            f"抓取微调: "
            f"fine_tune_base={GRASP_FINE_TUNE_BASE_M.tolist()}, "
            f"right_bias={GRASP_RIGHT_BIAS_M:.3f}m, "
            f"left_compensation={GRASP_LEFT_COMPENSATION_M:.3f}m, "
            f"block_left_shift={BLOCK_GRASP_LEFT_SHIFT_M:.3f}m, "
            f"block_forward_extra={BLOCK_FORWARD_EXTRA_M:.3f}m, "
            f"height_fraction={GRASP_HEIGHT_FRACTION:.2f}, "
            f"z_extra={GRASP_CENTER_EXTRA_Z_M:.3f}m, "
            f"min_flange_z={MIN_FLANGE_Z_M:.3f}m, "
            f"above_top_clearance={ABOVE_TOP_CLEARANCE_M:.3f}m, "
            f"stop_before_close={STOP_BEFORE_CLOSE}"
        )

    def initialize_ui(self):
        """初始化 OpenCV 图形窗口。"""
        if not self.ui_available:
            self.get_logger().error(
                "当前没有检测到 DISPLAY 或 WAYLAND_DISPLAY，"
                "OpenCV 无法显示相机窗口。"
            )
            return False

        try:
            cv2.namedWindow(
                "Camera View",
                cv2.WINDOW_NORMAL,
            )
            cv2.resizeWindow(
                "Camera View",
                640,
                480,
            )
            cv2.moveWindow(
                "Camera View",
                40,
                40,
            )

            self.ui_initialized = True
            self.get_logger().info(
                "OpenCV 相机窗口已创建，"
                f"DISPLAY={os.environ.get('DISPLAY', '')}"
            )
            return True

        except cv2.error as exc:
            self.ui_initialized = False
            self.ui_error_logged = True
            self.get_logger().error(
                f"OpenCV 窗口初始化失败: {exc}。"
                "可能安装了 opencv-python-headless，"
                "或者当前终端没有图形显示权限。"
            )
            return False

    def status_callback(self, msg):
        arm_status_code = int(msg.arm_status)
        error_code = int(msg.err_code)
        ctrl_mode = int(msg.ctrl_mode)
        mode_feedback = int(msg.mode_feedback)
        motion_status = int(msg.motion_status)
        faulted = arm_status_code != 0 or error_code != 0

        now = time.monotonic()
        previous_ctrl_mode = self.last_ctrl_mode
        self.last_ctrl_mode = ctrl_mode
        self.last_mode_feedback = mode_feedback
        self.last_motion_status = motion_status

        if ctrl_mode == 1:
            if previous_ctrl_mode != 1:
                self.can_control_stable_since = now
        else:
            self.can_control_stable_since = None

        control_feedback = (ctrl_mode, mode_feedback, motion_status)
        if (
            control_feedback != self.last_logged_control_feedback
            and now - self.last_control_feedback_log_time >= 1.0
        ):
            self.last_logged_control_feedback = control_feedback
            self.last_control_feedback_log_time = now
            self.get_logger().info(
                "Piper 控制反馈: "
                f"ctrl_mode={ctrl_mode}, "
                f"mode_feedback={mode_feedback}, "
                f"motion_status={motion_status}"
            )

        if (
            arm_status_code != self.last_arm_status_code
            or error_code != self.last_arm_error_code
        ):
            self.last_arm_status_code = arm_status_code
            self.last_arm_error_code = error_code

            if faulted:
                status_names = {
                    1: "急停",
                    2: "无解",
                    3: "奇异点",
                    4: "目标角度超限",
                    5: "关节通信异常",
                    6: "关节抱闸未打开",
                    7: "碰撞",
                    8: "拖动示教超速",
                    9: "关节状态异常",
                    10: "其它异常",
                    14: "主控过温",
                    15: "释放电阻过温",
                }
                # Use the decoded PiperStatusMsg booleans directly.  The raw
                # err_code byte order differs between older documentation and
                # SDK revisions, so shifting the integer here can swap the two
                # fault groups in logs.
                angle_limit_joints = [
                    index
                    for index in range(1, 7)
                    if bool(
                        getattr(msg, f"joint_{index}_angle_limit")
                    )
                ]
                communication_joints = [
                    index
                    for index in range(1, 7)
                    if bool(
                        getattr(
                            msg,
                            f"communication_status_joint_{index}",
                        )
                    )
                ]
                self.get_logger().error(
                    "机械臂保护状态: "
                    f"arm_status={arm_status_code}"
                    f"({status_names.get(arm_status_code, '未知')}), "
                    f"err_code={error_code}, "
                    f"角度超限关节={angle_limit_joints}, "
                    f"通信异常关节={communication_joints}"
                )
            elif self.arm_faulted:
                self.get_logger().info(
                    "机械臂保护状态已恢复正常。"
                )

        self.arm_faulted = faulted
        if faulted:
            # Stop all subsequent publishing from worker threads immediately.
            self.arm_enabled = False

    def joint_feedback_callback(self, msg):
        if len(msg.position) < 6:
            return

        joints = np.asarray(msg.position[:6], dtype=np.float64)
        if not np.all(np.isfinite(joints)):
            return
        # 驱动进程退出时可能收到一帧拼接不完整的关节
        # 反馈；不让这种帧覆盖最后一帧有效状态。
        feedback_margin_rad = 0.20
        if np.any(
            joints < PIPER_JOINT_LIMITS_RAD[:, 0] - feedback_margin_rad
        ) or np.any(
            joints > PIPER_JOINT_LIMITS_RAD[:, 1] + feedback_margin_rad
        ):
            return

        with self.joint_feedback_lock:
            self.last_joint_positions = joints.copy()
            if len(msg.position) >= 7:
                gripper_position = float(msg.position[6])
                if math.isfinite(gripper_position):
                    self.last_gripper_position_m = gripper_position
                    gripper_effort = (
                        float(msg.effort[6])
                        if len(msg.effort) >= 7
                        else math.nan
                    )
                    self.last_gripper_effort_nm = (
                        gripper_effort
                        if math.isfinite(gripper_effort)
                        else None
                    )
                    self.last_gripper_feedback_at = time.monotonic()

    def end_pose_feedback_callback(self, msg):
        quaternion = np.array(
            [
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ],
            dtype=np.float64,
        )
        quaternion_norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(quaternion_norm) or quaternion_norm < 1e-8:
            return

        xyz = np.array(
            [
                float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(xyz)):
            return

        rotation = ScipyRotation.from_quat(
            quaternion / quaternion_norm
        )
        with self.end_pose_feedback_lock:
            self.last_end_pose_rotation = rotation.as_matrix()
            self.last_end_pose_xyz = xyz
            self.last_end_pose_rpy_deg = rotation.as_euler(
                "xyz",
                degrees=True,
            )
            self.last_end_pose_received_at = time.monotonic()

    def get_cached_end_pose(self, max_age_s=0.5):
        with self.end_pose_feedback_lock:
            if (
                self.last_end_pose_rotation is None
                or self.last_end_pose_xyz is None
                or self.last_end_pose_rpy_deg is None
                or self.last_end_pose_received_at is None
            ):
                raise RuntimeError("Piper ROS 尚未发布末端位姿。")
            age_s = time.monotonic() - self.last_end_pose_received_at
            rotation = self.last_end_pose_rotation.copy()
            xyz = self.last_end_pose_xyz.copy()
            rpy_deg = self.last_end_pose_rpy_deg.copy()

        if age_s > max(0.05, float(max_age_s)):
            raise RuntimeError(
                "Piper ROS 末端位姿已过期: "
                f"age={age_s:.3f}s"
            )
        return rotation, xyz, rpy_deg

    def wait_for_stable_can_control(self):
        """Require a stable CAN control mode before any observation motion."""
        stable_required_s = 0.5
        deadline = time.monotonic() + max(
            1.0,
            self.can_control_wait_timeout_s,
        )
        if self.last_ctrl_mode != 1:
            self.get_logger().info(
                "通过已有 piper_ros CAN 连接退出拖动示教并"
                "请求 MOVE_J；此步不发送位置目标。"
            )
            request = Bool()
            request.data = True
            self.prepare_move_j_pub.publish(request)

        self.get_logger().info(
            "等待连续 0.5 秒 ctrl_mode=1 后再允许观察位运动。"
        )

        while rclpy.ok() and self.arm_enabled and not self.arm_faulted:
            now = time.monotonic()
            if (
                self.last_ctrl_mode == 1
                and self.can_control_stable_since is not None
                and now - self.can_control_stable_since >= stable_required_s
            ):
                self.get_logger().info(
                    "Piper 已稳定进入 CAN 指令控制模式。"
                )
                return True
            if now >= deadline:
                break
            time.sleep(0.05)

        self.get_logger().error(
            "未检测到稳定 CAN 控制模式，已禁止观察位和抓取动作: "
            f"ctrl_mode={self.last_ctrl_mode}, "
            f"mode_feedback={self.last_mode_feedback}, "
            f"motion_status={self.last_motion_status}"
        )
        return False

    def color_image_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8",
            )
            self.camera_timeout_logged = False

            if not self._logged_color_ready:
                self.get_logger().info(
                    f"已收到腕部相机 RGB: {COLOR_TOPIC}, "
                    f"shape={self.color_image.shape}"
                )
                self._logged_color_ready = True

        except Exception as exc:
            self.get_logger().error(
                f"彩色图像转换错误: {exc}"
            )

    def depth_image_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001
            else:
                depth = depth.astype(np.float32)

            self.depth_image = depth

            if not self._logged_depth_ready:
                valid = int(
                    np.count_nonzero(
                        (self.depth_image > 0.05)
                        & (self.depth_image < 2.5)
                    )
                )
                self.get_logger().info(
                    f"已收到腕部相机 depth: {DEPTH_TOPIC}, "
                    f"shape={self.depth_image.shape}, "
                    f"valid={valid}"
                )
                self._logged_depth_ready = True

        except Exception as exc:
            self.get_logger().error(
                f"深度图像转换错误: {exc}"
            )

    def camera_info_callback(self, msg):
        ros_camera_matrix = np.array(
            msg.k,
            dtype=np.float64,
        ).reshape(3, 3)
        self.ros_camera_matrix = ros_camera_matrix

        if USE_CALIBRATED_INTRINSICS:
            self.camera_matrix = CALIBRATED_CAMERA_MATRIX.copy()
        else:
            self.camera_matrix = ros_camera_matrix.copy()

        if not self._logged_camera_info_ready:
            delta = (
                ros_camera_matrix
                - CALIBRATED_CAMERA_MATRIX
            )
            max_abs_delta = float(
                np.max(np.abs(delta))
            )

            self.get_logger().info(
                "已收到腕部相机 camera_info: "
                f"{CAMERA_INFO_TOPIC}"
            )
            self.get_logger().info(
                "当前深度反投影使用: "
                f"{'固定标定内参' if USE_CALIBRATED_INTRINSICS else 'ROS camera_info'}；"
                f"ROS K 与标定 K 最大绝对差={max_abs_delta:.6f}"
            )
            self._logged_camera_info_ready = True

    def transition_to_state(self, new_state):
        with self.state_lock:
            self.prev_state = self.state
            self.state = new_state
            self.state_entry_time = time.time()
            self.get_logger().info(
                f"状态转换: "
                f"{self.prev_state.name} -> {self.state.name}"
            )

    def get_grasp_config(self, class_id=None):
        if class_id is None:
            class_id = self.target_class_id

        if class_id in GRASP_CONFIG:
            return GRASP_CONFIG[class_id]

        return {
            "height_offset": 0.0,
            "approach_height": ABOVE_TOP_CLEARANCE_M,
            "gripper_open": OPEN_GRIPPER_M,
            "gripper_closed": CLOSE_GRIPPER_M,
            "yaw_deg": None,
            "lift_height": LIFT_HEIGHT_M,
        }

    def set_target_class(
        self,
        class_id,
        qr_code=None,
        color=None,
        model_class_name=None,
        source=None,
    ):
        class_id = int(class_id)

        if class_id not in TARGET_CLASS_PROMPTS:
            raise ValueError(
                f"目标类别必须在 0-1 之间，收到: {class_id}"
            )

        if model_class_name is None and color is not None:
            suffix = "bottle" if class_id == 0 else "block"
            candidate_name = f"{str(color).lower()}_{suffix}"
            if candidate_name in CUSTOM_YOLO_GRASP_CLASSES:
                model_class_name = candidate_name
        if (
            model_class_name is not None
            and model_class_name not in CUSTOM_YOLO_GRASP_CLASSES
        ):
            raise ValueError(
                f"不可抓取或未知的 YOLO 类别: {model_class_name}"
            )

        self.target_class_id = class_id
        self.target_color = color
        self.target_model_class_name = model_class_name
        self.target_dataset_class_id = (
            None
            if model_class_name is None
            else int(
                CUSTOM_YOLO_GRASP_CLASSES[model_class_name][
                    "dataset_class_id"
                ]
            )
        )
        self.target_qr_code = qr_code
        self.detection_target = build_target_prompt(
            class_id,
            model_class_name=model_class_name,
        )
        self.detection_history.clear()

        if self.vision_detector:
            self.vision_detector.set_detection_target(
                self.detection_target
            )

        if source is None:
            source = f"二维码 {qr_code}" if qr_code else "手动选择"
        color_text = self.target_color or "any"
        display_text = (
            get_custom_target_display_name(self.target_model_class_name)
            if self.target_model_class_name is not None
            else "generic"
        )
        self.get_logger().info(
            f"目标类别已设置: class={class_id}, "
            f"color={color_text}, "
            f"dataset_class={self.target_dataset_class_id}, "
            f"model_class={self.target_model_class_name or 'generic'}, "
            f"display={display_text}, "
            f"prompt='{self.detection_target}', source={source}"
        )

    def update_qr_target_from_current_frame(self):
        if self.color_image is None:
            self.get_logger().warn(
                "还没有相机图像，无法读取二维码。"
            )
            return None

        decoded, _, _ = self.qr_detector.detectAndDecode(
            self.color_image
        )
        decoded = decoded.strip() if decoded else ""

        if not decoded:
            self.get_logger().warn("当前帧未识别到二维码。")
            return None

        if decoded not in QR_TO_CLASS:
            self.get_logger().warn(
                f"二维码 '{decoded}' 不在目标映射表中。"
            )
            return None

        self.qr_history.append(decoded)
        self.qr_history = self.qr_history[-max(QR_CONFIRM_FRAMES, 1):]

        if self.qr_history.count(decoded) >= QR_CONFIRM_FRAMES:
            self.set_target_class(
                QR_TO_CLASS[decoded],
                qr_code=decoded,
            )
            return self.target_class_id

        self.get_logger().info(
            f"二维码候选: {decoded}，"
            f"确认进度 {self.qr_history.count(decoded)}/"
            f"{QR_CONFIRM_FRAMES}。"
        )
        return None

    def detect_target_once(
        self,
        min_confidence,
        require_complete_bbox=False,
    ):
        if self.color_image is None:
            return None

        if (
            self.vision_detector is None
            or not self.vision_detector.is_loaded
        ):
            return None

        detector_prompt = (
            "*"
            if self.vision_detector.backend == "yolov8"
            else self.detection_target
        )
        frame = self.color_image.copy()
        detection = self.vision_detector.detect_frame_simple(
            frame,
            detector_prompt,
        )

        if detection is None:
            self.get_logger().warn(
                "YOLO 本次检测失败：推理后端没有返回结果。"
            )
            return None

        if len(detection["boxes"]) == 0:
            self.get_logger().info(
                "YOLO 本次检测: 无原始候选框，"
                f"model_threshold={YOLO_CONFIDENCE:.3f}, "
                f"target={self.detection_target}"
            )
            return None

        candidates = []
        raw_candidates = []
        incomplete_candidates = []
        image_height, image_width = frame.shape[:2]
        for index, confidence in enumerate(detection["confidences"]):
            label = ""
            if index < len(detection.get("labels", [])):
                label = str(detection["labels"][index]).lower()

            model_class_name = None
            if index < len(detection.get("class_names", [])):
                raw_name = detection["class_names"][index]
                if raw_name is not None:
                    model_class_name = normalize_custom_yolo_class_name(
                        raw_name
                    )

            dataset_class_id = None
            if index < len(detection.get("class_ids", [])):
                raw_class_id = detection["class_ids"][index]
                if raw_class_id is not None:
                    dataset_class_id = int(raw_class_id)

            raw_candidates.append(
                (
                    model_class_name or label or "unknown",
                    float(confidence),
                    dataset_class_id,
                )
            )

            if self.target_model_class_name is not None:
                if model_class_name != self.target_model_class_name:
                    continue
            elif self.target_color and self.target_color not in label.split():
                continue
            elif self.detection_target:
                target_terms = [
                    term.strip()
                    for term in str(self.detection_target)
                    .lower()
                    .replace(",", ".")
                    .split(".")
                    if term.strip()
                ]
                searchable_name = " ".join(
                    part
                    for part in (model_class_name, label)
                    if part
                )
                if target_terms and not any(
                    term in searchable_name
                    for term in target_terms
                ):
                    continue

            bbox = detection["boxes"][index]
            if require_complete_bbox:
                visibility = evaluate_bbox_visibility(
                    bbox,
                    image_width=image_width,
                    image_height=image_height,
                    edge_margin_px=TARGET_BBOX_EDGE_MARGIN_PX,
                )
                if not visibility["complete"]:
                    incomplete_candidates.append(
                        {
                            "class_name": model_class_name or label,
                            "confidence": float(confidence),
                            "bbox": [float(value) for value in bbox],
                            "reason": visibility["reason"],
                            "clipped_edges": visibility["clipped_edges"],
                        }
                    )
                    continue

            candidates.append(
                (
                    index,
                    float(confidence),
                    label,
                    model_class_name,
                    dataset_class_id,
                )
            )

        if not candidates:
            if incomplete_candidates:
                rejected_text = "; ".join(
                    f"{item['class_name']}:{item['confidence']:.3f}, "
                    f"bbox={[round(value, 1) for value in item['bbox']]}, "
                    f"edges={item['clipped_edges']}"
                    for item in incomplete_candidates
                )
                self.get_logger().warn(
                    "YOLO 目标框不完整，已拒绝三维定位和抓取: "
                    f"edge_margin={TARGET_BBOX_EDGE_MARGIN_PX}px, "
                    f"rejected=[{rejected_text}]"
                )
                return None
            raw_text = ", ".join(
                f"{name}:{confidence:.3f}"
                for name, confidence, _ in sorted(
                    raw_candidates,
                    key=lambda item: item[1],
                    reverse=True,
                )
            )
            self.get_logger().info(
                "YOLO 本次检测: 存在原始候选，但均与目标不匹配，"
                f"target={self.target_model_class_name or self.detection_target}, "
                f"raw=[{raw_text}]"
            )
            return None

        (
            best_index,
            best_confidence,
            best_label,
            best_model_class_name,
            best_dataset_class_id,
        ) = max(
            candidates,
            key=lambda item: item[1],
        )

        accepted = best_confidence >= float(min_confidence)
        inference_elapsed_s = float(detection.get("elapsed_s", 0.0))
        self.get_logger().info(
            "YOLO 本次检测: "
            f"class={best_model_class_name or best_label}, "
            f"confidence={best_confidence:.4f} "
            f"({best_confidence * 100.0:.2f}%), "
            f"threshold={float(min_confidence):.3f}, "
            f"accepted={accepted}, "
            f"inference={inference_elapsed_s:.3f}s"
        )

        if not accepted:
            return None

        return {
            "class_id": self.target_class_id,
            "color": self.target_color,
            "confidence": best_confidence,
            "bbox": detection["boxes"][best_index],
            "bbox_complete": bool(require_complete_bbox),
            "image_size": [int(image_width), int(image_height)],
            "label": best_label,
            "model_class_name": best_model_class_name,
            "dataset_class_id": best_dataset_class_id,
            "prompt": self.detection_target,
            "timestamp": time.time(),
        }

    def detect_target_for_localization(self, policy):
        required_frames = max(1, int(policy["confirm_frames"]))
        require_complete_bbox = bool(policy["require_complete_bbox"])
        min_confidence = float(policy["confidence"])
        previous_bbox = None
        confirmed = None

        for frame_index in range(1, required_frames + 1):
            hit = self.detect_target_once(
                min_confidence,
                require_complete_bbox=require_complete_bbox,
            )
            if hit is None:
                if required_frames > 1:
                    self.get_logger().warn(
                        "方块完整检测连续帧确认失败: "
                        f"frame={frame_index}/{required_frames}"
                    )
                return None

            current_bbox = np.asarray(hit["bbox"], dtype=np.float64)
            if previous_bbox is not None:
                overlap = bbox_iou(previous_bbox, current_bbox)
                if overlap < BLOCK_DETECTION_CONFIRM_MIN_IOU:
                    self.get_logger().warn(
                        "方块完整检测框不稳定，已拒绝定位和抓取: "
                        f"iou={overlap:.3f} < "
                        f"{BLOCK_DETECTION_CONFIRM_MIN_IOU:.3f}"
                    )
                    return None

            previous_bbox = current_bbox
            confirmed = hit
            if frame_index < required_frames:
                time.sleep(0.08)

        if confirmed is not None and required_frames > 1:
            confirmed["confirmation_frames"] = required_frames
            confirmed["bbox_complete"] = True
            self.get_logger().info(
                "方块完整检测确认通过: "
                f"frames={required_frames}, "
                f"confidence={float(confirmed['confidence']):.4f}, "
                "bbox 未接触画面边缘。"
            )
        return confirmed

    def update_moving_detection_window(self):
        hit = self.detect_target_once(
            MOVING_DETECTION_CONFIDENCE
        )
        self.detection_history.append(hit is not None)
        self.detection_history = self.detection_history[
            -MOVING_DETECTION_WINDOW:
        ]

        return (
            len(self.detection_history)
            >= MOVING_DETECTION_WINDOW
            and sum(self.detection_history)
            >= MOVING_DETECTION_MIN_HITS
        )

    def confirm_target_still(self):
        hits = 0
        previous_bbox = None

        for _ in range(max(1, STILL_DETECTION_CONFIRM_FRAMES)):
            hit = self.detect_target_once(
                STILL_DETECTION_CONFIDENCE
            )

            if hit is None:
                return False

            bbox = np.asarray(hit["bbox"], dtype=np.float64)
            if previous_bbox is not None:
                center_delta = np.linalg.norm(
                    (bbox[:2] + bbox[2:]) * 0.5
                    - (previous_bbox[:2] + previous_bbox[2:]) * 0.5
                )
                if center_delta > 20.0:
                    return False

            previous_bbox = bbox
            hits += 1
            time.sleep(0.08)

        return hits >= STILL_DETECTION_CONFIRM_FRAMES

    # ================= CAN 异步配置核心 =================

    def can_setup_worker(self):
        """检查并配置真实 CAN，禁止回退到虚拟 CAN。"""
        self.get_logger().info(
            f"CAN 后台配置线程已启动，目标接口: {CAN_NAME}"
        )

        try:
            for module_name in ("can", "can_raw", "can_dev"):
                if os.path.isdir(f"/sys/module/{module_name}"):
                    continue
                subprocess.run(
                    ["sudo", "-n", "modprobe", module_name],
                    check=False,
                )

            exists_result = subprocess.run(
                ["ip", "link", "show", CAN_NAME],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )

            if exists_result.returncode != 0:
                self.can_setup_success = False
                self.get_logger().error(
                    f"没有找到真实 CAN 接口 {CAN_NAME}。"
                    "请检查 USB-CAN 是否连接，以及多 CAN "
                    "激活脚本是否已正确执行。"
                )
                return

            brief_result = subprocess.run(
                ["ip", "-br", "link", "show", CAN_NAME],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )

            already_up = (
                brief_result.returncode == 0
                and "UP" in brief_result.stdout.strip().split()
            )

            if not already_up:
                subprocess.run(
                    ["sudo", "-n", "ip", "link", "set", CAN_NAME, "down"],
                    check=False,
                )
                subprocess.run(
                    [
                        "sudo",
                        "-n",
                        "ip",
                        "link",
                        "set",
                        CAN_NAME,
                        "type",
                        "can",
                        "bitrate",
                        "1000000",
                    ],
                    check=False,
                )
                subprocess.run(
                    [
                        "sudo",
                        "-n",
                        "ip",
                        "link",
                        "set",
                        CAN_NAME,
                        "txqueuelen",
                        "1000",
                    ],
                    check=False,
                )
                subprocess.run(
                    [
                        "sudo",
                        "-n",
                        "ip",
                        "link",
                        "set",
                        CAN_NAME,
                        "up",
                    ],
                    check=False,
                )
                time.sleep(0.5)

            if self.check_can_interface():
                self.can_setup_success = True
                self.get_logger().info(
                    f"✓ 真实 CAN 接口已就绪: {CAN_NAME}"
                )
            else:
                self.can_setup_success = False
                self.get_logger().error(
                    f"{CAN_NAME} 存在，但没有成功进入 UP 状态。"
                )

        except Exception as exc:
            self.can_setup_success = False
            self.get_logger().error(
                f"CAN 配置线程出错: {exc}"
            )

        finally:
            self.can_setup_done = True

    def check_can_interface(self):
        """确认真实 CAN 存在且处于 UP 状态。"""
        try:
            result = subprocess.run(
                ["ip", "-br", "link", "show", CAN_NAME],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )

            if result.returncode != 0:
                return False

            fields = result.stdout.strip().split()
            return "UP" in fields

        except Exception:
            return False

    def start_piper_node(self):
        """只允许使用真实 CAN 启动 Piper 控制节点。"""
        if (
            self.node_process is not None
            and self.node_process.poll() is None
        ):
            return True

        if not self.check_can_interface():
            self.get_logger().error(
                f"{CAN_NAME} 不存在或未启动，"
                "拒绝启动机械臂控制节点。"
            )
            return False

        command = [
            "ros2",
            "run",
            "piper",
            "piper_single_ctrl",
            "--ros-args",
            "-p",
            f"can_port:={CAN_NAME}",
            "-p",
            "auto_enable:=false",
        ]

        try:
            self.get_logger().info(
                f"启动 Piper 节点，CAN 接口: {CAN_NAME}"
            )

            # 子进程使用独立进程组，避免 Ctrl+C 同时打断父子两个 rclpy context。
            # 主程序继续使用 PYTHONNOUSERSITE=1，避免 NumPy/OpenCV 串环境。
            # 但 ROS 安装生成的 piper_single_ctrl 使用系统 Python，
            # 而 piper_sdk 可能位于用户 site-packages 或 PYTHONPATH。
            # 因此只对子进程移除 PYTHONNOUSERSITE，不影响本主程序。
            child_env = os.environ.copy()
            child_env.pop("PYTHONNOUSERSITE", None)

            self.node_process = subprocess.Popen(
                command,
                start_new_session=True,
                env=child_env,
            )
            return True

        except Exception as exc:
            self.node_process = None
            self.get_logger().error(
                f"启动 Piper 节点失败: {exc}"
            )
            return False

    def topic_has_publishers(self, topic_name):
        """
        检查 ROS 话题是否有真实发布者，
        防止只根据残留的话题名称误判相机在线。
        """
        try:
            return self.count_publishers(topic_name) > 0

        except Exception:
            return False

    def start_camera_node(self):
        """
        启动左侧 RealSense。

        只有目标话题存在真实发布者时才复用现有节点，
        防止误复用其他相机或残留 ROS 话题。
        """
        if self.camera_process is not None:
            if self.camera_process.poll() is None:
                return True
            self.camera_process = None

        color_online = self.topic_has_publishers(COLOR_TOPIC)
        depth_online = self.topic_has_publishers(DEPTH_TOPIC)
        info_online = self.topic_has_publishers(CAMERA_INFO_TOPIC)

        if color_online and depth_online and info_online:
            self.get_logger().info(
                "检测到左侧 RGBD 相机已有真实发布者，"
                "复用现有节点。"
            )
            return True

        command = [
            "ros2",
            "launch",
            "realsense2_camera",
            "rs_launch.py",
            f"camera_namespace:={CAMERA_NAMESPACE}",
            f"camera_name:={CAMERA_NAME}",
            "initial_reset:=true",
            "enable_color:=true",
            "enable_depth:=true",
            "enable_infra:=false",
            "enable_infra1:=false",
            "enable_infra2:=false",
            "enable_gyro:=false",
            "enable_accel:=false",
            "enable_motion:=false",
            "enable_sync:=true",
            "align_depth.enable:=true",
            "spatial_filter.enable:=true",
            "temporal_filter.enable:=true",
            "hole_filling_filter.enable:=true",
            "pointcloud.enable:=false",
            "publish_tf:=false",
            "diagnostics_period:=0.0",
            "rgb_camera.color_profile:=640,480,15",
            "depth_module.depth_profile:=640,480,15",
            "log_level:=warn",
        ]

        # 始终绑定左侧相机序列号，禁止自动选择其他 RealSense。
        command.insert(
            6,
            f"serial_no:={CAMERA1_SERIAL}",
        )

        try:
            serial_text = CAMERA1_SERIAL
            self.get_logger().info(
                f"启动左侧 RealSense: serial={serial_text}, "
                f"namespace=/{CAMERA_NAMESPACE}/{CAMERA_NAME}"
            )

            self.camera_process = subprocess.Popen(
                command,
                start_new_session=True,
            )

            self.camera_start_time = time.time()
            self.camera_timeout_logged = False
            return True

        except Exception as exc:
            self.camera_process = None
            self.get_logger().error(
                f"启动左侧 RealSense 失败: {exc}"
            )
            return False

    def activate_gripper_hold(self, gripper_m, target_type):
        self.gripper_hold_guard.activate(gripper_m, target_type)
        self.get_logger().info(
            "夹爪持物锁已启用: "
            f"target={target_type}, closed={float(gripper_m):.4f}m；"
            "放置释放步骤前禁止张开。"
        )

    def activate_gripper_hold_for_current_target(self, gripper_m):
        target_type = (
            self.target_model_class_name
            or self.detection_target
            or f"class_{self.target_class_id}"
        )
        self.activate_gripper_hold(gripper_m, target_type)

    def is_gripper_hold_active(self):
        return self.gripper_hold_guard.is_active()

    def get_gripper_hold_position(self):
        state = self.gripper_hold_guard.snapshot()
        if not state["active"]:
            return None
        return float(state["closed_m"])

    def get_fresh_gripper_feedback(self, received_after=None):
        with self.joint_feedback_lock:
            position = self.last_gripper_position_m
            effort = self.last_gripper_effort_nm
            received_at = self.last_gripper_feedback_at
        if position is None or received_at is None:
            return None
        now = time.monotonic()
        if now - float(received_at) > GRIPPER_FEEDBACK_MAX_AGE_S:
            return None
        if received_after is not None and float(received_at) < float(received_after):
            return None
        position = float(position)
        if not math.isfinite(position) or position < 0.0:
            return None
        return position, effort, float(received_at)

    def wait_until_block_gripper_open(self, commanded_open_m):
        required_open_m = max(
            0.0,
            float(commanded_open_m) - BLOCK_GRIPPER_OPEN_TOLERANCE_M,
        )
        deadline = time.monotonic() + max(
            0.1,
            BLOCK_GRIPPER_OPEN_WAIT_TIMEOUT_S,
        )
        last_position = None
        while rclpy.ok() and time.monotonic() <= deadline:
            feedback = self.get_fresh_gripper_feedback()
            if feedback is not None:
                last_position = float(feedback[0])
                if last_position >= required_open_m:
                    self.get_logger().info(
                        "方块抓取前夹爪开度确认通过: "
                        f"actual={last_position:.4f}m, "
                        f"required={required_open_m:.4f}m"
                    )
                    return True
            time.sleep(0.05)
        raise RuntimeError(
            "方块抓取前夹爪未张开到位，禁止下探: "
            f"actual={last_position}, required={required_open_m:.4f}m"
        )

    def resolve_post_close_hold_gripper(self, commanded_closed_m, close_started_at):
        target_type = (
            self.target_model_class_name
            or self.detection_target
            or f"class_{self.target_class_id}"
        )
        if not is_bottle_grasp_target(
            self.target_class_id,
            self.target_model_class_name,
            target_type,
        ):
            return float(commanded_closed_m)
        feedback = self.get_fresh_gripper_feedback(
            received_after=close_started_at,
        )
        if feedback is None:
            self.get_logger().warn(
                "瓶子闭合后没有新鲜夹爪反馈，继续使用原闭合值。"
            )
            return float(commanded_closed_m)
        measured_opening_m = float(feedback[0])
        hold_gripper_m = choose_bottle_hold_position(
            commanded_closed_m,
            measured_opening_m,
            BOTTLE_HOLD_PRELOAD_M,
        )
        self.get_logger().info(
            "瓶子接触保持开度: "
            f"measured={measured_opening_m:.4f}m, "
            f"preload={BOTTLE_HOLD_PRELOAD_M:.4f}m, "
            f"hold={hold_gripper_m:.4f}m"
        )
        return hold_gripper_m

    def authorize_gripper_release(self, reason):
        self.gripper_hold_guard.authorize_release(reason)
        self.get_logger().info(f"夹爪持物锁已授权释放: {reason}")

    def cancel_gripper_release(self):
        self.gripper_hold_guard.cancel_release()
        self.get_logger().warn(
            "夹爪释放未完成，已恢复持物锁闭合保护。"
        )

    def complete_gripper_release(self):
        self.gripper_hold_guard.complete_release()
        self.get_logger().info("物体已在放置步骤释放，夹爪持物锁已解除。")

    def apply_gripper_hold(self, requested_m, context):
        effective_m, clamped = self.gripper_hold_guard.apply(requested_m)
        if clamped:
            now = time.monotonic()
            if now - self.last_gripper_hold_clamp_log_at >= 1.0:
                state = self.gripper_hold_guard.snapshot()
                self.get_logger().warn(
                    "夹爪持物锁拦截提前张开命令: "
                    f"context={context}, requested={float(requested_m):.4f}m, "
                    f"held={float(effective_m):.4f}m, "
                    f"target={state['target_type']}"
                )
                self.last_gripper_hold_clamp_log_at = now
        return effective_m

    def send_pos_command(self, position_dict):
        if not self.arm_enabled or self.arm_faulted:
            return False

        msg = PosCmd()
        msg.x = float(position_dict["x"])
        msg.y = float(position_dict["y"])
        msg.z = float(position_dict["z"])
        msg.roll = float(position_dict["roll"])
        msg.pitch = float(position_dict["pitch"])
        msg.yaw = float(position_dict["yaw"])
        msg.gripper = self.apply_gripper_hold(
            position_dict["gripper"],
            "Cartesian pose command",
        )
        msg.mode1 = 0x01
        msg.mode2 = 0x00
        self.pos_pub.publish(msg)
        return True

    def move_to_joint_pose(
        self,
        target_joints_rad,
        label,
        gripper_m,
        timeout_s=None,
    ):
        """Move to a joint target with MOVE_J and verify joint feedback."""
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                f"机械臂未使能或处于保护状态，未发送{label}关节指令。"
            )
            return False

        target = np.asarray(
            target_joints_rad,
            dtype=np.float64,
        ).reshape(6)
        if np.any(target < PIPER_JOINT_LIMITS_RAD[:, 0]) or np.any(
            target > PIPER_JOINT_LIMITS_RAD[:, 1]
        ):
            self.get_logger().error(
                f"{label}超出 Piper 关节限位: {target.tolist()}"
            )
            return False

        if self.last_ctrl_mode != 1:
            self.get_logger().info(
                f"{label}运动前未处于稳定 CAN 控制模式，"
                "已请求 MOVE_J 控制并随目标刷新等待主控接管。"
            )
            request = Bool()
            request.data = True
            self.prepare_move_j_pub.publish(request)

        speed = float(
            np.clip(self.observation_joint_speed_percent, 1.0, 100.0)
        )
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]
        effective_gripper_m = self.apply_gripper_hold(
            gripper_m,
            f"MOVE_J {label}",
        )
        command.position = target.tolist() + [effective_gripper_m]
        # piper_ros 通过第 7 个 velocity 值设置全轴速度百分比。
        command.velocity = [0.0] * 6 + [speed]
        command.effort = [0.0] * 6 + [1.0]

        self.get_logger().info(
            f"MOVE_J {label}: "
            f"joints_rad={np.round(target, 4).tolist()}, "
            f"speed={speed:.0f}%"
        )

        deadline = time.monotonic() + max(
            1.0,
            self.observation_joint_timeout_s
            if timeout_s is None
            else float(timeout_s),
        )
        stable_count = 0
        last_progress_log = 0.0
        control_unstable_since = None

        while (
            rclpy.ok()
            and self.arm_enabled
            and not self.arm_faulted
            and time.monotonic() < deadline
        ):
            # 刷新 piper_ros 的命令看门狗；驱动内部会把最新
            # 目标以 200 Hz 发到 S-V1.8-2 主控。
            command.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(command)

            now = time.monotonic()
            control_stable = (
                self.last_ctrl_mode == 1
                and self.can_control_stable_since is not None
                and now - self.can_control_stable_since >= 0.10
            )
            if control_stable:
                control_unstable_since = None
            elif control_unstable_since is None:
                control_unstable_since = now
            elif now - control_unstable_since >= CAN_CONTROL_LOSS_GRACE_S:
                self.get_logger().error(
                    f"{label}运动期间 CAN 控制模式丢失，"
                    f"超过 {CAN_CONTROL_LOSS_GRACE_S:.1f}s，"
                    "已停止刷新目标。"
                )
                return False

            with self.joint_feedback_lock:
                current = (
                    None
                    if self.last_joint_positions is None
                    else self.last_joint_positions.copy()
                )

            if current is not None:
                max_error = float(np.max(np.abs(current - target)))
                stable_count = (
                    stable_count + 1
                    if max_error <= self.observation_joint_tolerance_rad
                    else 0
                )
                if stable_count >= 3:
                    self.get_logger().info(
                        f"{label}已到达: "
                        f"actual={np.round(current, 4).tolist()}, "
                        f"max_error={max_error:.4f}rad"
                    )
                    return True

                if now - last_progress_log >= 2.0:
                    self.get_logger().info(
                        f"{label}运动中: "
                        f"max_joint_error={max_error:.4f}rad"
                    )
                    last_progress_log = now

            time.sleep(0.05)

        with self.joint_feedback_lock:
            current = (
                None
                if self.last_joint_positions is None
                else self.last_joint_positions.copy()
            )
        self.get_logger().error(
            f"{label} MOVE_J 未到达，已中止后续识别/抓取: "
            f"actual={None if current is None else np.round(current, 4).tolist()}, "
            f"target={np.round(target, 4).tolist()}, "
            f"ctrl_mode={self.last_ctrl_mode}, "
            f"mode_feedback={self.last_mode_feedback}, "
            f"motion_status={self.last_motion_status}"
        )
        return False

    def move_to_observation_joint_pose(self):
        """Move to the taught observation point with MOVE_J and verify it."""
        return self.move_to_joint_pose(
            OBSERVATION_JOINTS_RAD,
            label="观察位",
            gripper_m=OPEN_GRIPPER_M,
        )

    def return_to_observation_after_grasp_failure(self):
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "抓取失败后未自动回观察位："
                "机械臂未使能或处于保护状态，不能继续发送运动指令。"
            )
            return False

        self.get_logger().warn(
            "抓取失败，自动返回初始观察位。"
        )
        if self.move_to_joint_pose(
            OBSERVATION_JOINTS_RAD,
            label="抓取失败恢复观察位",
            gripper_m=OPEN_GRIPPER_M,
            timeout_s=max(
                20.0,
                self.observation_joint_timeout_s,
            ),
        ) is False:
            self.get_logger().error(
                "抓取失败后自动回观察位未到达。"
            )
            return False

        self.get_logger().info(
            "抓取失败后已回到初始观察位。"
        )
        return True

    def is_retryable_target_grasp_failure(self, exc):
        if not RETRY_GRASP_ON_TARGET_FAILURE_ENABLED:
            return False

        text = str(exc)
        retry_markers = (
            "未找到目标",
            "没有可靠目标检测框",
            "没有从目标检测框内分割出足够物体点",
            "没有从桌面上分割出足够物体点",
            "最大物体簇点数太少",
            "抓取目标超出安全工作区",
            "超出工作空间",
            "超出安全工作区",
        )
        return any(marker in text for marker in retry_markers)

    def wait_for_retry_grasp_enter_confirmation(self, reason):
        self.wait_for_enter_confirmation(
            "本次实物识别/抓取未执行成功，"
            f"原因: {reason}。"
            "机械臂已回到观察位。请重新放置图纸，"
            "确认后按 Enter 从图纸 YOLO 识别阶段重新开始；"
            "本次不会沿用上一次图纸目标。",
            "重新开始图纸 YOLO 识别/抓取流程",
            "instruction_sheet",
            "press Enter to restart sheet YOLO> ",
        )

    def publish_pose_for(
        self,
        position_dict,
        duration=1.5,
        rate_hz=30.0,
    ):
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "机械臂未使能或处于保护状态，未发布抓取点位。"
            )
            return False

        msg = PosCmd()
        msg.x = float(position_dict["x"])
        msg.y = float(position_dict["y"])
        msg.z = float(position_dict["z"])
        msg.roll = float(position_dict["roll"])
        msg.pitch = float(position_dict["pitch"])
        msg.yaw = float(position_dict["yaw"])
        msg.gripper = self.apply_gripper_hold(
            position_dict["gripper"],
            "Cartesian pose hold",
        )
        msg.mode1 = 0x01
        msg.mode2 = 0x00

        self.get_logger().info(
            f"抓取点位: "
            f"x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, "
            f"rpy=("
            f"{math.degrees(msg.roll):.1f}, "
            f"{math.degrees(msg.pitch):.1f}, "
            f"{math.degrees(msg.yaw):.1f}), "
            f"gripper={msg.gripper:.3f}"
        )

        sleep_s = 1.0 / rate_hz
        deadline = time.time() + float(duration)

        while (
            rclpy.ok()
            and self.arm_enabled
            and not self.arm_faulted
            and time.time() < deadline
        ):
            self.pos_pub.publish(msg)
            time.sleep(sleep_s)

        return self.arm_enabled and not self.arm_faulted

    def publish_linear_pose_path(
        self,
        start_pose,
        end_pose,
        duration=1.5,
        rate_hz=30.0,
        label="线性路径",
    ):
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "机械臂未使能，未发布线性路径。"
            )
            return False

        steps = max(
            2,
            int(float(duration) * float(rate_hz)),
        )
        sleep_s = 1.0 / rate_hz

        self.get_logger().info(
            f"{label}: "
            f"from=("
            f"{start_pose['x']:.3f}, "
            f"{start_pose['y']:.3f}, "
            f"{start_pose['z']:.3f}) "
            f"to=("
            f"{end_pose['x']:.3f}, "
            f"{end_pose['y']:.3f}, "
            f"{end_pose['z']:.3f}), "
            f"duration={duration:.2f}s"
        )

        keys = (
            "x",
            "y",
            "z",
            "roll",
            "pitch",
            "yaw",
            "gripper",
        )

        for index in range(steps):
            alpha = index / float(steps - 1)
            pose = {}

            for key in keys:
                pose[key] = (
                    (1.0 - alpha) * float(start_pose[key])
                    + alpha * float(end_pose[key])
                )

            self.send_pos_command(pose)
            time.sleep(sleep_s)

        return True

    def publish_locked_rpy_linear_path(
        self,
        start_pose,
        end_pose,
        duration=1.5,
        rate_hz=30.0,
        gripper=None,
        label="锁姿态直线路径",
    ):
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "机械臂未使能，未发布锁姿态直线路径。"
            )
            return False

        locked_roll = float(start_pose["roll"])
        locked_pitch = float(start_pose["pitch"])
        locked_yaw = float(start_pose["yaw"])
        locked_gripper = (
            float(start_pose["gripper"])
            if gripper is None
            else float(gripper)
        )

        start_xyz = np.array(
            [
                float(start_pose["x"]),
                float(start_pose["y"]),
                float(start_pose["z"]),
            ],
            dtype=np.float64,
        )
        end_xyz = np.array(
            [
                float(end_pose["x"]),
                float(end_pose["y"]),
                float(end_pose["z"]),
            ],
            dtype=np.float64,
        )

        steps = max(
            2,
            int(float(duration) * float(rate_hz)),
        )
        sleep_s = 1.0 / float(rate_hz)

        self.get_logger().info(
            f"{label}: XYZ=({start_xyz[0]:.4f}, "
            f"{start_xyz[1]:.4f}, {start_xyz[2]:.4f}) -> "
            f"({end_xyz[0]:.4f}, {end_xyz[1]:.4f}, "
            f"{end_xyz[2]:.4f}), RPY锁定=("
            f"{math.degrees(locked_roll):.2f}, "
            f"{math.degrees(locked_pitch):.2f}, "
            f"{math.degrees(locked_yaw):.2f})"
        )

        for index in range(steps):
            alpha = index / float(steps - 1)
            smooth_alpha = (
                alpha * alpha * (3.0 - 2.0 * alpha)
            )
            xyz = (
                (1.0 - smooth_alpha) * start_xyz
                + smooth_alpha * end_xyz
            )
            pose = {
                "x": float(xyz[0]),
                "y": float(xyz[1]),
                "z": float(xyz[2]),
                "roll": locked_roll,
                "pitch": locked_pitch,
                "yaw": locked_yaw,
                "gripper": locked_gripper,
            }
            self.send_pos_command(pose)
            time.sleep(sleep_s)

        final_pose = {
            "x": float(end_xyz[0]),
            "y": float(end_xyz[1]),
            "z": float(end_xyz[2]),
            "roll": locked_roll,
            "pitch": locked_pitch,
            "yaw": locked_yaw,
            "gripper": locked_gripper,
        }
        for _ in range(3):
            self.send_pos_command(final_pose)
            time.sleep(sleep_s)

        return final_pose

    def publish_rpy_only_path(
        self,
        start_pose,
        target_rpy,
        duration=1.5,
        rate_hz=30.0,
        gripper=None,
        label="安全高处纯姿态调整",
    ):
        """
        XYZ 完全固定，只改变 roll/pitch/yaw。

        每个欧拉角都按 [-pi, pi] 内的最短角度插值，避免例如
        -170° 到 180° 被错误插值成 350° 大绕转。
        """
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "机械臂未使能，未发布纯姿态轨迹。"
            )
            return False

        target_rpy = np.asarray(
            target_rpy,
            dtype=np.float64,
        ).reshape(3)

        start_rpy = np.array(
            [
                float(start_pose["roll"]),
                float(start_pose["pitch"]),
                float(start_pose["yaw"]),
            ],
            dtype=np.float64,
        )
        deltas = np.array(
            [
                wrap_angle_rad(
                    target_rpy[index]
                    - start_rpy[index]
                )
                for index in range(3)
            ],
            dtype=np.float64,
        )

        locked_x = float(start_pose["x"])
        locked_y = float(start_pose["y"])
        locked_z = float(start_pose["z"])
        locked_gripper = (
            float(start_pose["gripper"])
            if gripper is None
            else float(gripper)
        )

        steps = max(
            2,
            int(float(duration) * float(rate_hz)),
        )
        sleep_s = 1.0 / float(rate_hz)

        self.get_logger().info(
            f"{label}: XYZ=({locked_x:.4f}, "
            f"{locked_y:.4f}, {locked_z:.4f}) 固定，"
            f"RPY=("
            f"{math.degrees(start_rpy[0]):.1f}, "
            f"{math.degrees(start_rpy[1]):.1f}, "
            f"{math.degrees(start_rpy[2]):.1f}) -> ("
            f"{math.degrees(target_rpy[0]):.1f}, "
            f"{math.degrees(target_rpy[1]):.1f}, "
            f"{math.degrees(target_rpy[2]):.1f})"
        )

        for index in range(steps):
            if not self.arm_enabled or self.arm_faulted:
                self.get_logger().warn(
                    f"{label} 中止：机械臂未使能或处于保护状态。"
                )
                return False
            alpha = index / float(steps - 1)
            smooth_alpha = (
                alpha * alpha * (3.0 - 2.0 * alpha)
            )
            current_rpy = (
                start_rpy
                + smooth_alpha * deltas
            )
            pose = {
                "x": locked_x,
                "y": locked_y,
                "z": locked_z,
                "roll": float(current_rpy[0]),
                "pitch": float(current_rpy[1]),
                "yaw": float(current_rpy[2]),
                "gripper": locked_gripper,
            }
            self.send_pos_command(pose)
            time.sleep(sleep_s)

        final_pose = {
            "x": locked_x,
            "y": locked_y,
            "z": locked_z,
            "roll": float(target_rpy[0]),
            "pitch": float(target_rpy[1]),
            "yaw": float(target_rpy[2]),
            "gripper": locked_gripper,
        }
        for _ in range(3):
            if not self.arm_enabled or self.arm_faulted:
                self.get_logger().warn(
                    f"{label} 末端保持中止："
                    "机械臂未使能或处于保护状态。"
                )
                return False
            self.send_pos_command(final_pose)
            time.sleep(sleep_s)

        return final_pose

    def publish_xy_only_path(
        self,
        start_pose,
        target_x,
        target_y,
        duration=1.5,
        rate_hz=30.0,
        gripper=None,
        label="固定姿态高位水平移动",
    ):
        """
        Z 和 RPY 完全固定，只改变 X/Y。
        """
        if not self.arm_enabled or self.arm_faulted:
            self.get_logger().warn(
                "机械臂未使能，未发布水平轨迹。"
            )
            return False

        start_x = float(start_pose["x"])
        start_y = float(start_pose["y"])
        target_x = float(target_x)
        target_y = float(target_y)

        locked_z = float(start_pose["z"])
        locked_roll = float(start_pose["roll"])
        locked_pitch = float(start_pose["pitch"])
        locked_yaw = float(start_pose["yaw"])
        locked_gripper = (
            float(start_pose["gripper"])
            if gripper is None
            else float(gripper)
        )

        steps = max(
            2,
            int(float(duration) * float(rate_hz)),
        )
        sleep_s = 1.0 / float(rate_hz)

        self.get_logger().info(
            f"{label}: Z={locked_z:.4f} 固定，"
            f"RPY=("
            f"{math.degrees(locked_roll):.1f}, "
            f"{math.degrees(locked_pitch):.1f}, "
            f"{math.degrees(locked_yaw):.1f}) 固定，"
            f"XY=({start_x:.4f}, {start_y:.4f}) -> "
            f"({target_x:.4f}, {target_y:.4f})"
        )

        for index in range(steps):
            if not self.arm_enabled or self.arm_faulted:
                self.get_logger().warn(
                    f"{label} 中止：机械臂未使能或处于保护状态。"
                )
                return False
            alpha = index / float(steps - 1)
            smooth_alpha = (
                alpha * alpha * (3.0 - 2.0 * alpha)
            )
            pose = {
                "x": (
                    (1.0 - smooth_alpha) * start_x
                    + smooth_alpha * target_x
                ),
                "y": (
                    (1.0 - smooth_alpha) * start_y
                    + smooth_alpha * target_y
                ),
                "z": locked_z,
                "roll": locked_roll,
                "pitch": locked_pitch,
                "yaw": locked_yaw,
                "gripper": locked_gripper,
            }
            self.send_pos_command(pose)
            time.sleep(sleep_s)

        final_pose = {
            "x": target_x,
            "y": target_y,
            "z": locked_z,
            "roll": locked_roll,
            "pitch": locked_pitch,
            "yaw": locked_yaw,
            "gripper": locked_gripper,
        }
        for _ in range(3):
            if not self.arm_enabled or self.arm_faulted:
                self.get_logger().warn(
                    f"{label} 末端保持中止："
                    "机械臂未使能或处于保护状态。"
                )
                return False
            self.send_pos_command(final_pose)
            time.sleep(sleep_s)

        return final_pose

    def publish_strict_vertical_path(
        self,
        locked_pose,
        target_z,
        duration=1.5,
        rate_hz=30.0,
        gripper=None,
        label="严格垂直路径",
    ):
        """
        严格垂直运动。

        整段轨迹只允许 z 改变；以下量从起点锁定并保持不变：
            x, y, roll, pitch, yaw

        该函数不使用 end_pose 中的横向坐标或姿态，避免通用插值
        在下降阶段产生横移、旋转或姿态二次调整。
        """
        if not self.arm_enabled:
            self.get_logger().warn(
                "机械臂未使能，未发布严格垂直路径。"
            )
            return False

        required_keys = (
            "x",
            "y",
            "z",
            "roll",
            "pitch",
            "yaw",
            "gripper",
        )
        missing = [
            key
            for key in required_keys
            if key not in locked_pose
        ]
        if missing:
            raise ValueError(
                f"垂直路径起点缺少字段: {missing}"
            )

        start_z = float(locked_pose["z"])
        end_z = float(target_z)
        locked_x = float(locked_pose["x"])
        locked_y = float(locked_pose["y"])
        locked_roll = float(locked_pose["roll"])
        locked_pitch = float(locked_pose["pitch"])
        locked_yaw = float(locked_pose["yaw"])
        locked_gripper = (
            float(locked_pose["gripper"])
            if gripper is None
            else float(gripper)
        )

        steps = max(
            2,
            int(float(duration) * float(rate_hz)),
        )
        sleep_s = 1.0 / float(rate_hz)

        direction = (
            "下降"
            if end_z < start_z
            else "上升"
        )

        self.get_logger().info(
            f"{label}: {direction}，"
            f"x={locked_x:.4f}, y={locked_y:.4f} 固定，"
            f"rpy=("
            f"{math.degrees(locked_roll):.2f}, "
            f"{math.degrees(locked_pitch):.2f}, "
            f"{math.degrees(locked_yaw):.2f}) 固定，"
            f"z={start_z:.4f}->{end_z:.4f}, "
            f"duration={duration:.2f}s"
        )

        for index in range(steps):
            alpha = index / float(steps - 1)

            # 平滑起停，仍然只改变 z。
            smooth_alpha = (
                alpha * alpha * (3.0 - 2.0 * alpha)
            )
            current_z = (
                (1.0 - smooth_alpha) * start_z
                + smooth_alpha * end_z
            )

            pose = {
                "x": locked_x,
                "y": locked_y,
                "z": current_z,
                "roll": locked_roll,
                "pitch": locked_pitch,
                "yaw": locked_yaw,
                "gripper": locked_gripper,
            }
            self.send_pos_command(pose)
            time.sleep(sleep_s)

        # 最终点额外保持几帧，确保控制器收到精确目标。
        final_pose = {
            "x": locked_x,
            "y": locked_y,
            "z": end_z,
            "roll": locked_roll,
            "pitch": locked_pitch,
            "yaw": locked_yaw,
            "gripper": locked_gripper,
        }
        for _ in range(3):
            self.send_pos_command(final_pose)
            time.sleep(sleep_s)

        return True

    def snapshot_rgbd(self):
        if (
            self.color_image is None
            or self.depth_image is None
            or self.camera_matrix is None
        ):
            return None, None, None

        return (
            self.color_image.copy(),
            self.depth_image.copy(),
            self.camera_matrix.copy(),
        )

    def backproject_depth(self, depth, camera_matrix, stride=2):
        height, width = depth.shape[:2]
        ys, xs = np.mgrid[
            0:height:stride,
            0:width:stride,
        ]
        z = depth[
            0:height:stride,
            0:width:stride,
        ]

        valid = (
            (z > 0.12)
            & (z < 1.5)
            & np.isfinite(z)
        )

        xs_valid = xs[valid].astype(np.float64)
        ys_valid = ys[valid].astype(np.float64)
        z_valid = z[valid].astype(np.float64)

        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])

        points = np.column_stack(
            (
                (xs_valid - cx) / fx * z_valid,
                (ys_valid - cy) / fy * z_valid,
                z_valid,
            )
        )
        pixels = np.column_stack(
            (xs_valid, ys_valid)
        ).astype(np.int32)

        return points, pixels

    def segment_table_object(
        self,
        points_cam,
        pixels,
        camera_matrix,
        roi_bbox=None,
    ):
        try:
            import open3d as o3d
        except Exception as exc:
            raise RuntimeError(
                "需要 open3d 才能做桌面点云分割: "
                f"{exc}"
            )

        if len(points_cam) < 2000:
            raise RuntimeError(
                f"深度点太少: {len(points_cam)}"
            )

        pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(points_cam)
        )
        plane, inliers = pcd.segment_plane(
            distance_threshold=0.008,
            ransac_n=3,
            num_iterations=1200,
        )
        a, b, c, d = [float(value) for value in plane]

        rays = np.column_stack(
            (
                (
                    pixels[:, 0].astype(np.float64)
                    - float(camera_matrix[0, 2])
                )
                / float(camera_matrix[0, 0]),
                (
                    pixels[:, 1].astype(np.float64)
                    - float(camera_matrix[1, 2])
                )
                / float(camera_matrix[1, 1]),
                np.ones(len(pixels), dtype=np.float64),
            )
        )

        denom = rays @ np.array(
            [a, b, c],
            dtype=np.float64,
        )
        z_plane = np.full(
            len(points_cam),
            np.nan,
            dtype=np.float64,
        )
        ok = np.abs(denom) > 1e-8
        z_plane[ok] = -d / denom[ok]

        height_toward_camera = (
            z_plane - points_cam[:, 2]
        )

        image_margin = (
            (pixels[:, 0] > 20)
            & (pixels[:, 0] < 620)
            & (pixels[:, 1] > 20)
            & (pixels[:, 1] < 460)
        )

        object_mask = (
            np.isfinite(height_toward_camera)
            & image_margin
            & (height_toward_camera > 0.012)
            & (height_toward_camera < 0.25)
        )

        roi_applied = False
        if roi_bbox is not None:
            x0, y0, x1, y1 = [float(value) for value in roi_bbox]
            x0 -= TARGET_BBOX_MARGIN_PX
            y0 -= TARGET_BBOX_MARGIN_PX
            x1 += TARGET_BBOX_MARGIN_PX
            y1 += TARGET_BBOX_MARGIN_PX
            roi_mask = (
                (pixels[:, 0] >= x0)
                & (pixels[:, 0] <= x1)
                & (pixels[:, 1] >= y0)
                & (pixels[:, 1] <= y1)
            )
            object_mask &= roi_mask
            roi_applied = True

        obj_points = points_cam[object_mask]
        obj_pixels = pixels[object_mask]

        if len(obj_points) < MIN_TARGET_DEPTH_POINTS:
            source = "目标检测框内" if roi_applied else "桌面上"
            raise RuntimeError(
                f"没有从{source}分割出足够物体点: "
                f"{len(obj_points)}"
            )

        obj_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(obj_points)
        )
        labels = np.asarray(
            obj_pcd.cluster_dbscan(
                eps=0.025,
                min_points=18,
                print_progress=False,
            )
        )

        valid_labels = labels[labels >= 0]

        if len(valid_labels) == 0:
            cluster_mask = np.ones(
                len(obj_points),
                dtype=bool,
            )
        else:
            label_ids, counts = np.unique(
                valid_labels,
                return_counts=True,
            )
            cluster_mask = labels == int(
                label_ids[np.argmax(counts)]
            )

        cluster_count = int(
            np.count_nonzero(cluster_mask)
        )
        if cluster_count < 60:
            raise RuntimeError(
                f"最大物体簇点数太少: {cluster_count}"
            )

        diagnostics = {
            "plane": [a, b, c, d],
            "plane_inliers": int(len(inliers)),
            "cluster_points": cluster_count,
            "roi_applied": roi_applied,
            "roi_bbox": (
                None
                if roi_bbox is None
                else [float(value) for value in roi_bbox]
            ),
        }

        return (
            obj_points[cluster_mask],
            obj_pixels[cluster_mask],
            diagnostics,
        )

    def estimate_wrist_object_in_base(self):
        snapshot = self.snapshot_rgbd()

        if snapshot[0] is None:
            raise RuntimeError(
                "还没有收到左侧相机的 RGBD 和 camera_info。"
            )

        bgr, depth, camera_matrix = snapshot

        (
            rotation_gripper_to_base,
            translation_gripper_to_base,
            rpy_deg,
        ) = self.get_cached_end_pose()

        transform_cam_to_base = (
            homogeneous(
                rotation_gripper_to_base,
                translation_gripper_to_base,
            )
            @ homogeneous(
                R_CAM_TO_GRIPPER,
                T_CAM_TO_GRIPPER,
            )
        )

        target_detection = None
        target_bbox = None
        if USE_TARGET_BBOX_FOR_DEPTH:
            block_target = is_block_grasp_target(
                self.target_class_id,
                self.target_model_class_name,
                self.detection_target,
            )
            detection_policy = localization_detection_policy(
                is_block_target=block_target,
                yolo_confidence=YOLO_CONFIDENCE,
                regular_confidence=LOCALIZATION_DETECTION_CONFIDENCE,
                complete_block_confidence=(
                    BLOCK_COMPLETE_DETECTION_CONFIDENCE
                ),
                block_confirm_frames=BLOCK_DETECTION_CONFIRM_FRAMES,
            )
            detection_confidence_threshold = float(
                detection_policy["confidence"]
            )
            target_detection = self.detect_target_for_localization(
                detection_policy
            )
            if target_detection is not None:
                target_bbox = target_detection["bbox"]
                self.get_logger().info(
                    "YOLO 目标检测: "
                    f"class={target_detection.get('model_class_name')}, "
                    f"confidence={float(target_detection['confidence']):.4f} "
                    f"({float(target_detection['confidence']) * 100.0:.2f}%), "
                    f"threshold={detection_confidence_threshold:.2f}, "
                    f"complete_bbox="
                    f"{bool(target_detection.get('bbox_complete', False))}, "
                    f"confirm_frames="
                    f"{int(target_detection.get('confirmation_frames', 1))}, "
                    "bbox="
                    f"{[round(float(value), 1) for value in target_bbox]}"
                )
            elif REQUIRE_TARGET_BBOX_FOR_DEPTH:
                raise RuntimeError(
                    "没有可靠目标检测框，已拒绝继续三维定位。"
                )

        points_cam, pixels = self.backproject_depth(
            depth,
            camera_matrix,
            stride=2,
        )

        (
            object_points_cam,
            object_pixels,
            diagnostics,
        ) = self.segment_table_object(
            points_cam,
            pixels,
            camera_matrix,
            roi_bbox=target_bbox,
        )
        if target_detection is not None:
            diagnostics["target_detection"] = {
                "confidence": float(target_detection["confidence"]),
                "prompt": target_detection["prompt"],
                "model_class_name": target_detection.get(
                    "model_class_name"
                ),
                "dataset_class_id": target_detection.get(
                    "dataset_class_id"
                ),
                "bbox": [float(value) for value in target_bbox],
            }

        object_points_h = np.column_stack(
            (
                object_points_cam,
                np.ones(
                    len(object_points_cam),
                    dtype=np.float64,
                ),
            )
        )
        object_points_base = (
            transform_cam_to_base
            @ object_points_h.T
        ).T[:, :3]

        median_center = np.median(
            object_points_base,
            axis=0,
        )
        geometric_center, aabb_min, aabb_max = (
            robust_point_cloud_box_center(
                object_points_base,
                lower_quantile=0.05,
                upper_quantile=0.95,
            )
        )
        try:
            is_block_target = int(self.target_class_id) == 1
        except (TypeError, ValueError):
            is_block_target = False
        center = (
            geometric_center.copy()
            if is_block_target
            else median_center.copy()
        )
        yolo_center_diagnostics = None
        if is_block_target and target_bbox is not None:
            (
                yolo_center_cam,
                yolo_center_diagnostics,
            ) = project_yolo_bbox_center_with_robust_depth(
                object_points_cam=object_points_cam,
                object_pixels=object_pixels,
                target_bbox=target_bbox,
                camera_matrix=camera_matrix,
            )
            yolo_center_base = (
                transform_cam_to_base
                @ np.append(yolo_center_cam, 1.0)
            )[:3]
            center[:2] = yolo_center_base[:2]
            yolo_center_diagnostics["camera_point"] = (
                yolo_center_cam.tolist()
            )
            yolo_center_diagnostics["base_point"] = (
                yolo_center_base.tolist()
            )
        diagnostics["grasp_center"] = {
            "source": (
                "yolo_bbox_center_with_robust_depth"
                if yolo_center_diagnostics is not None
                else (
                    "robust_point_cloud_box_center"
                    if is_block_target
                    else "point_cloud_median"
                )
            ),
            "median_center": median_center.tolist(),
            "geometric_center": geometric_center.tolist(),
            "selected_center": center.tolist(),
            "yolo_center": yolo_center_diagnostics,
        }

        axis_info = None
        grasp_yaw_deg = None
        if (
            USE_OBJECT_AXIS_YAW
            and not is_block_target
            and len(object_points_base) >= 20
        ):
            xy = object_points_base[:, :2]
            xy_centered = xy - np.median(xy, axis=0)
            cov = np.cov(xy_centered.T)
            eigvals, eigvecs = np.linalg.eigh(cov)
            order = np.argsort(eigvals)
            short_vec = eigvecs[:, order[0]]
            long_vec = eigvecs[:, order[-1]]
            short_len = math.sqrt(max(float(eigvals[order[0]]), 1e-12))
            long_len = math.sqrt(max(float(eigvals[order[-1]]), 1e-12))
            axis_ratio = long_len / max(short_len, 1e-9)
            short_angle_deg = math.degrees(
                math.atan2(float(short_vec[1]), float(short_vec[0]))
            )
            long_angle_deg = math.degrees(
                math.atan2(float(long_vec[1]), float(long_vec[0]))
            )
            reference_axis_deg = (
                short_angle_deg
                if GRASP_CLOSING_AXIS == "short"
                else long_angle_deg
            )
            if axis_ratio >= MIN_OBJECT_AXIS_RATIO:
                # 实机上 yaw=短轴角度时，夹爪/机械臂本体垂直短轴。
                # offset 保留给现场微调，避免再改代码。
                grasp_yaw_deg = (
                    (reference_axis_deg + GRASP_AXIS_YAW_OFFSET_DEG + 180.0)
                    % 360.0
                ) - 180.0
            axis_info = {
                "short_axis_deg": float(short_angle_deg),
                "long_axis_deg": float(long_angle_deg),
                "axis_ratio": float(axis_ratio),
                "reference_axis": GRASP_CLOSING_AXIS,
                "yaw_offset_deg": float(GRASP_AXIS_YAW_OFFSET_DEG),
                "min_axis_ratio": float(MIN_OBJECT_AXIS_RATIO),
                "grasp_yaw_deg": (
                    None if grasp_yaw_deg is None else float(grasp_yaw_deg)
                ),
            }
            diagnostics["object_axes"] = axis_info

        object_bottom_z = float(aabb_min[2])
        object_top_z = float(aabb_max[2])
        object_height_m = max(
            0.0,
            object_top_z - object_bottom_z,
        )

        top_center = np.array(
            [
                center[0],
                center[1],
                aabb_max[2],
            ],
            dtype=np.float64,
        )
        bottom_center = np.array(
            [
                center[0],
                center[1],
                object_bottom_z,
            ],
            dtype=np.float64,
        )

        upright_bottle_grasp = (
            BOTTLE_UPRIGHT_SIDE_GRASP
            and is_bottle_grasp_target(
                self.target_class_id,
                self.target_model_class_name,
                self.detection_target,
            )
        )
        if upright_bottle_grasp:
            cluster_points = int(
                diagnostics.get("cluster_points", 0)
            )
            invalid_geometry = []
            if object_height_m < BOTTLE_MIN_ESTIMATED_HEIGHT_M:
                invalid_geometry.append(
                    "height="
                    f"{object_height_m:.3f}m < "
                    f"{BOTTLE_MIN_ESTIMATED_HEIGHT_M:.3f}m"
                )
            if cluster_points < BOTTLE_MIN_CLUSTER_POINTS:
                invalid_geometry.append(
                    f"cluster={cluster_points} < "
                    f"{BOTTLE_MIN_CLUSTER_POINTS}"
                )
            if invalid_geometry:
                raise RuntimeError(
                    "瓶子三维几何不可靠，拒绝该帧并继续左右扫描: "
                    + ", ".join(invalid_geometry)
                )
        grasp_height_fraction = (
            BOTTLE_GRASP_HEIGHT_FRACTION
            if upright_bottle_grasp
            else GRASP_HEIGHT_FRACTION
        )
        min_above_bottom = (
            BOTTLE_GRASP_MIN_ABOVE_BOTTOM_M
            if upright_bottle_grasp
            else GRASP_MIN_ABOVE_BOTTOM_M
        )
        max_below_top = (
            BOTTLE_GRASP_MAX_BELOW_TOP_M
            if upright_bottle_grasp
            else GRASP_MAX_BELOW_TOP_M
        )

        if object_height_m > 1e-4:
            lower_z = object_bottom_z + min(
                min_above_bottom,
                object_height_m * 0.45,
            )
            upper_z = object_top_z - min(
                max_below_top,
                object_height_m * 0.45,
            )
            nominal_z = (
                object_bottom_z
                + object_height_m
                * float(
                    np.clip(
                        grasp_height_fraction,
                        0.0,
                        1.0,
                    )
                )
            )

            if lower_z <= upper_z:
                grasp_z = float(
                    np.clip(
                        nominal_z,
                        lower_z,
                        upper_z,
                    )
                )
            else:
                grasp_z = (
                    object_bottom_z
                    + object_height_m * 0.5
                )
        else:
            grasp_z = float(center[2])

        grasp_xy = center[:2].copy()
        if upright_bottle_grasp and object_height_m > 1e-4:
            band_half = max(
                0.010,
                object_height_m
                * float(BOTTLE_GRASP_DEPTH_BAND_FRACTION)
                * 0.5,
            )
            mid_band_mask = (
                (object_points_base[:, 2] >= grasp_z - band_half)
                & (object_points_base[:, 2] <= grasp_z + band_half)
            )
            mid_band_points = object_points_base[mid_band_mask]
            if len(mid_band_points) >= max(12, MIN_TARGET_DEPTH_POINTS // 4):
                grasp_xy = np.median(
                    mid_band_points[:, :2],
                    axis=0,
                )
            diagnostics["upright_bottle_depth"] = {
                "grasp_height_fraction": float(grasp_height_fraction),
                "grasp_z": float(grasp_z),
                "band_half_m": float(band_half),
                "band_points": int(len(mid_band_points)),
                "xy_source": (
                    "middle_depth_band"
                    if len(mid_band_points)
                    >= max(12, MIN_TARGET_DEPTH_POINTS // 4)
                    else "full_object_median"
                ),
            }

        depth_grasp_center = np.array(
            [
                grasp_xy[0],
                grasp_xy[1],
                grasp_z,
            ],
            dtype=np.float64,
        )

        overlay = bgr.copy()

        sample_step = max(
            1,
            len(object_pixels) // 3000,
        )
        for u, v in object_pixels[::sample_step]:
            cv2.circle(
                overlay,
                (int(u), int(v)),
                1,
                (0, 0, 255),
                -1,
            )

        x0, y0 = np.min(
            object_pixels,
            axis=0,
        )
        x1, y1 = np.max(
            object_pixels,
            axis=0,
        )

        cv2.rectangle(
            overlay,
            (int(x0), int(y0)),
            (int(x1), int(y1)),
            (0, 255, 0),
            2,
        )
        if target_bbox is not None:
            tx0, ty0, tx1, ty1 = [int(round(value)) for value in target_bbox]
            cv2.rectangle(
                overlay,
                (tx0, ty0),
                (tx1, ty1),
                (255, 255, 0),
                2,
            )
            if is_block_target and yolo_center_diagnostics is not None:
                center_u, center_v = [
                    int(round(value))
                    for value in yolo_center_diagnostics["pixel_center"]
                ]
                cv2.line(
                    overlay,
                    (center_u - 10, center_v),
                    (center_u + 10, center_v),
                    (255, 0, 255),
                    2,
                )
                cv2.line(
                    overlay,
                    (center_u, center_v - 10),
                    (center_u, center_v + 10),
                    (255, 0, 255),
                    2,
                )
        if axis_info is not None:
            cv2.putText(
                overlay,
                (
                    f"axis ratio={axis_info['axis_ratio']:.2f} "
                    f"yaw={axis_info['grasp_yaw_deg']}"
                ),
                (10, 86),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

        return {
            "center": center,
            "top_center": top_center,
            "bottom_center": bottom_center,
            "depth_grasp_center": depth_grasp_center,
            "aabb_min": aabb_min,
            "aabb_max": aabb_max,
            "object_height_m": object_height_m,
            "diagnostics": diagnostics,
            "robot_xyz_m": translation_gripper_to_base,
            "robot_rpy_deg": rpy_deg,
            "T_cam_to_base": transform_cam_to_base,
            "overlay": overlay,
            "target_class_id": self.target_class_id,
            "target_model_class_name": self.target_model_class_name,
            "target_dataset_class_id": self.target_dataset_class_id,
            "target_prompt": self.detection_target,
            "target_detection": target_detection,
            "object_axes": axis_info,
            "grasp_yaw_deg": grasp_yaw_deg,
        }

    def build_grasp_waypoints(
        self,
        object_result,
        bottle_grasp_rpy_override_deg=None,
        block_grasp_rpy_override_deg=None,
    ):
        object_grasp_center = np.asarray(
            object_result["depth_grasp_center"],
            dtype=np.float64,
        ).copy()
        object_top = np.asarray(
            object_result["top_center"],
            dtype=np.float64,
        ).copy()

        target_class_id = object_result.get("target_class_id")
        grasp_config = self.get_grasp_config(
            target_class_id
        )
        upright_bottle_grasp = (
            BOTTLE_UPRIGHT_SIDE_GRASP
            and is_bottle_grasp_target(
                target_class_id,
                object_result.get("target_model_class_name"),
                object_result.get("target_prompt"),
            )
        )
        height_offset = float(grasp_config["height_offset"])
        approach_height = float(grasp_config["approach_height"])
        open_gripper = float(grasp_config["gripper_open"])
        closed_gripper = float(grasp_config["gripper_closed"])
        lift_height = float(grasp_config["lift_height"])

        selected_grasp_rpy_deg = (
            choose_simple_grasp_rpy_deg(
                object_result["robot_rpy_deg"]
            )
        ).copy()
        yaw_source = "simple"
        raw_target_yaw_deg = float(selected_grasp_rpy_deg[2])
        current_rpy_deg = np.asarray(
            object_result["robot_rpy_deg"],
            dtype=np.float64,
        ).reshape(3)
        current_yaw_deg = float(
            current_rpy_deg[2]
        )
        try:
            is_block_target = int(target_class_id) == 1
        except (TypeError, ValueError):
            is_block_target = False
        if is_block_target:
            open_gripper = float(grasp_config["gripper_open"])

        yaw_locked_by_calibration = False
        if upright_bottle_grasp:
            if bottle_grasp_rpy_override_deg is None:
                selected_grasp_rpy_deg = current_rpy_deg.copy()
                yaw_source = "locked_observation_rpy_upright_bottle"
                raw_target_yaw_deg = current_yaw_deg
            else:
                selected_grasp_rpy_deg = np.asarray(
                    bottle_grasp_rpy_override_deg,
                    dtype=np.float64,
                ).reshape(3).copy()
                yaw_source = "base_joint_aimed_upright_bottle"
                raw_target_yaw_deg = float(selected_grasp_rpy_deg[2])
        elif BLOCK_TOP_DOWN_GRASP and is_block_target:
            if (
                block_grasp_rpy_override_deg is None
                and
                BLOCK_TOP_DOWN_RPY_DEG is None
                and BLOCK_TOP_DOWN_REQUIRE_CALIBRATED_RPY
            ):
                raise RuntimeError(
                    "方块顶部抓取姿态未标定，拒绝执行。"
                    "请先手动把夹爪摆到真正从上方夹方块的姿态，"
                    "读取 read_piper_state_sdk.py 输出的 end_rpy_deg，"
                    "再设置 WRIST_BLOCK_TOP_DOWN_RPY_DEG="
                    "\"roll,pitch,yaw\"。"
                )
            if block_grasp_rpy_override_deg is not None:
                selected_grasp_rpy_deg = np.asarray(
                    block_grasp_rpy_override_deg,
                    dtype=np.float64,
                ).reshape(3).copy()
            elif BLOCK_TOP_DOWN_RPY_DEG is not None:
                selected_grasp_rpy_deg = np.asarray(
                    BLOCK_TOP_DOWN_RPY_DEG,
                    dtype=np.float64,
                ).reshape(3).copy()
            else:
                selected_grasp_rpy_deg = np.array(
                    [
                        SIMPLE_GRASP_ROLL_DEG,
                        SIMPLE_GRASP_PITCH_DEG,
                        current_yaw_deg,
                    ],
                    dtype=np.float64,
                )
            yaw_source = (
                "adaptive_block_terminal_rpy"
                if block_grasp_rpy_override_deg is not None
                else (
                    "calibrated_top_down_block_rpy"
                    if BLOCK_TOP_DOWN_RPY_DEG is not None
                    else "top_down_block_current_yaw"
                )
            )
            raw_target_yaw_deg = float(selected_grasp_rpy_deg[2])
            yaw_locked_by_calibration = (
                block_grasp_rpy_override_deg is not None
                or BLOCK_TOP_DOWN_RPY_DEG is not None
            )
            object_result["block_top_down_grasp"] = {
                "enabled": True,
                "calibrated": (
                    block_grasp_rpy_override_deg is not None
                    or BLOCK_TOP_DOWN_RPY_DEG is not None
                ),
                "adaptive_terminal": (
                    block_grasp_rpy_override_deg is not None
                ),
                "roll_deg": float(selected_grasp_rpy_deg[0]),
                "pitch_deg": float(selected_grasp_rpy_deg[1]),
                "yaw_deg": float(selected_grasp_rpy_deg[2]),
                "rpy_switch_stage": "safe_height_before_xy",
            }
        elif BLOCK_KEEP_OBSERVATION_RPY and is_block_target:
            selected_grasp_rpy_deg = current_rpy_deg.copy()
            yaw_source = "locked_observation_rpy_block"
            raw_target_yaw_deg = current_yaw_deg
            object_result["block_observation_rpy_override"] = {
                "enabled": True,
                "kept_roll_deg": float(current_rpy_deg[0]),
                "kept_pitch_deg": float(current_rpy_deg[1]),
                "kept_yaw_deg": float(current_rpy_deg[2]),
            }
        elif (
            USE_OBJECT_AXIS_YAW
            and object_result.get("grasp_yaw_deg") is not None
        ):
            raw_target_yaw_deg = float(object_result["grasp_yaw_deg"])
            yaw_source = "object_axis"
        elif REQUIRE_OBJECT_AXIS_YAW:
            raise RuntimeError(
                "未获得可靠短轴方向，禁止退回固定 yaw 抓取，"
                f"object_axes={object_result.get('object_axes')}"
            )
        elif grasp_config.get("yaw_deg") is not None:
            raw_target_yaw_deg = float(grasp_config["yaw_deg"])
            yaw_source = "config"

        selected_yaw_deg = (
            float(raw_target_yaw_deg)
            if yaw_locked_by_calibration
            else choose_minimal_equivalent_yaw_deg(
                raw_target_yaw_deg,
                current_yaw_deg,
            )
        )
        selected_grasp_rpy_deg[2] = selected_yaw_deg
        object_result["selected_grasp_yaw_deg"] = selected_yaw_deg
        object_result["selected_grasp_yaw_source"] = yaw_source
        object_result["selected_grasp_yaw_delta_deg"] = wrap_angle_deg(
            selected_yaw_deg - current_yaw_deg
        )

        if (
            BLOCK_OUTER_KEEP_CURRENT_ROLL_PITCH
            and not upright_bottle_grasp
            and not object_result.get("block_observation_rpy_override")
        ):
            if (
                is_block_target
                and float(object_grasp_center[1])
                < BLOCK_OUTER_RPY_Y_THRESHOLD_M
            ):
                selected_grasp_rpy_deg[0] = float(
                    current_rpy_deg[0]
                )
                selected_grasp_rpy_deg[1] = float(
                    current_rpy_deg[1]
                )
                object_result[
                    "block_outer_roll_pitch_override"
                ] = {
                    "enabled": True,
                    "object_y_m": float(object_grasp_center[1]),
                    "threshold_y_m": float(
                        BLOCK_OUTER_RPY_Y_THRESHOLD_M
                    ),
                    "kept_roll_deg": float(current_rpy_deg[0]),
                    "kept_pitch_deg": float(current_rpy_deg[1]),
                    "selected_yaw_deg": float(
                        selected_grasp_rpy_deg[2]
                    ),
                }

        rotation_target = rotation_from_rpy_xyz(
            *np.deg2rad(
                selected_grasp_rpy_deg
            )
        )

        right_bias_m = (
            0.0
            if upright_bottle_grasp
            else GRASP_RIGHT_BIAS_M
        )
        right_bias_base = rotation_target @ np.array(
            [
                0.0,
                -right_bias_m,
                0.0,
            ],
            dtype=np.float64,
        )
        right_bias_base[2] = 0.0

        if upright_bottle_grasp:
            left_compensation_m = BOTTLE_GRASP_LEFT_SHIFT_M
        elif is_block_target:
            left_compensation_m = BLOCK_GRASP_LEFT_SHIFT_M
        else:
            left_compensation_m = GRASP_LEFT_COMPENSATION_M
        left_compensation_base = (
            rotation_target
            @ np.array(
                [
                    0.0,
                    left_compensation_m,
                    0.0,
                ],
                dtype=np.float64,
            )
        )
        left_compensation_base[2] = 0.0
        forward_extra_base = np.zeros(3, dtype=np.float64)
        if upright_bottle_grasp:
            forward_extra_base = (
                rotation_target[:, 2]
                * float(BOTTLE_FORWARD_EXTRA_M)
            )
        elif is_block_target and abs(BLOCK_FORWARD_EXTRA_M) > 1e-9:
            forward_direction_base = rotation_target[:, 2].copy()
            forward_direction_base[2] = 0.0
            forward_norm = float(
                np.linalg.norm(forward_direction_base)
            )
            if forward_norm > 1e-9:
                forward_extra_base = (
                    forward_direction_base
                    / forward_norm
                    * float(BLOCK_FORWARD_EXTRA_M)
                )
                object_result["block_forward_extra"] = {
                    "extra_m": float(BLOCK_FORWARD_EXTRA_M),
                    "direction_base": (
                        forward_direction_base / forward_norm
                    ).tolist(),
                    "extra_base": forward_extra_base.tolist(),
                }
            else:
                object_result["block_forward_extra"] = {
                    "extra_m": float(BLOCK_FORWARD_EXTRA_M),
                    "direction_base": None,
                    "extra_base": forward_extra_base.tolist(),
                    "skipped_reason": "tool_forward_horizontal_norm_zero",
                }
        if is_block_target:
            object_result["block_grasp_compensation"] = {
                "left_shift_m": float(BLOCK_GRASP_LEFT_SHIFT_M),
                "left_shift_base": left_compensation_base.tolist(),
                "forward_extra_m": float(BLOCK_FORWARD_EXTRA_M),
                "forward_extra_base": forward_extra_base.tolist(),
            }

        fine_tune_xy = GRASP_FINE_TUNE_BASE_M.copy()
        fine_tune_xy[2] = 0.0

        # 先计算最终夹爪中心目标。
        target_center = object_grasp_center.copy()
        if upright_bottle_grasp:
            target_center[2] += height_offset
        elif not is_block_target:
            target_center[2] += (
                GRASP_CENTER_EXTRA_Z_M
                + height_offset
                - GRASP_EXTRA_DESCENT_M
            )
        target_center += right_bias_base
        target_center += left_compensation_base
        target_center += GRASP_FINE_TUNE_BASE_M
        target_center += forward_extra_base

        # 高位中心的 X/Y 必须与最终抓取中心完全一致。
        # 只把 Z 设置到物体最高点上方。
        above_top_center = target_center.copy()
        above_top_center[2] = max(
            float(
                object_top[2]
                + approach_height
                + GRASP_FINE_TUNE_BASE_M[2]
            ),
            float(target_center[2] + 0.020),
        )

        offset_base = (
            rotation_target
            @ self.gripper_center_offset
        )
        applied_offset = offset_base.copy()

        if not APPLY_GRIPPER_CENTER_OFFSET_XY:
            applied_offset[0] = 0.0
            applied_offset[1] = 0.0

        if not APPLY_GRIPPER_CENTER_OFFSET_Z:
            applied_offset[2] = 0.0

        flange_grasp_unclamped = (
            target_center - applied_offset
        )
        flange_grasp = flange_grasp_unclamped.copy()
        flange_grasp[2] = max(
            float(flange_grasp[2]),
            MIN_FLANGE_Z_M,
        )

        min_z_clamped = bool(
            flange_grasp[2]
            > flange_grasp_unclamped[2] + 1e-9
        )

        # 严格垂直下降的关键：
        # approach 与 grasp 使用完全相同的 flange X/Y，
        # 后续轨迹只改变 Z。
        if upright_bottle_grasp:
            tool_z_base = rotation_target[:, 2].copy()
            if BOTTLE_TOP_DOWN_PATH:
                flange_approach = flange_grasp.copy()
                flange_approach[2] = max(
                    float(
                        above_top_center[2]
                        - applied_offset[2]
                    ),
                    float(flange_grasp[2] + 0.030),
                    MIN_FLANGE_Z_M,
                )
                tcp_approach_center = (
                    flange_approach + applied_offset
                )
            else:
                tcp_approach_center = (
                    target_center
                    - tool_z_base
                    * float(
                        BOTTLE_SIDE_APPROACH_BACKOFF_M
                        + BOTTLE_FORWARD_EXTRA_M
                    )
                )
                flange_approach = (
                    tcp_approach_center - applied_offset
                )
                flange_approach[2] = max(
                    float(flange_approach[2]),
                    float(flange_grasp[2] + 0.020),
                    MIN_FLANGE_Z_M,
                )
            object_result["upright_bottle_grasp"] = {
                "enabled": True,
                "top_down_path": bool(BOTTLE_TOP_DOWN_PATH),
                "tool_z_base": tool_z_base.tolist(),
                "side_approach_backoff_m": float(
                    BOTTLE_SIDE_APPROACH_BACKOFF_M
                ),
                "left_shift_m": float(BOTTLE_GRASP_LEFT_SHIFT_M),
                "forward_extra_m": float(BOTTLE_FORWARD_EXTRA_M),
                "forward_extra_base": forward_extra_base.tolist(),
                "tcp_approach_center": (
                    tcp_approach_center.tolist()
                ),
            }
        else:
            flange_approach = flange_grasp.copy()
            flange_approach[2] = max(
                float(
                    above_top_center[2]
                    - applied_offset[2]
                ),
                float(flange_grasp[2] + 0.020),
                MIN_FLANGE_Z_M,
            )

        # 抬升也保持同一个 X/Y。
        flange_lift = flange_grasp.copy()
        flange_lift[2] = (
            float(flange_grasp[2])
            + lift_height
        )

        base_pose = {
            "roll": math.radians(
                float(selected_grasp_rpy_deg[0])
            ),
            "pitch": math.radians(
                float(selected_grasp_rpy_deg[1])
            ),
            "yaw": math.radians(
                float(selected_grasp_rpy_deg[2])
            ),
        }

        approach = {
            **base_pose,
            "x": float(flange_approach[0]),
            "y": float(flange_approach[1]),
            "z": float(flange_approach[2]),
            "gripper": open_gripper,
        }

        grasp_open = {
            **base_pose,
            "x": float(flange_grasp[0]),
            "y": float(flange_grasp[1]),
            "z": float(flange_grasp[2]),
            "gripper": open_gripper,
        }

        grasp_closed = {
            **grasp_open,
            "gripper": closed_gripper,
        }

        lift = {
            **grasp_closed,
            "z": float(flange_lift[2]),
        }

        # 硬检查：垂直段禁止存在任何 XY/姿态变化。
        locked_keys = (
            "x",
            "y",
            "roll",
            "pitch",
            "yaw",
        )
        if (not upright_bottle_grasp) or BOTTLE_TOP_DOWN_PATH:
            for key in locked_keys:
                if not math.isclose(
                    float(approach[key]),
                    float(grasp_open[key]),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                ):
                    raise RuntimeError(
                        f"垂直下降锁定失败: "
                        f"{key} 在高位和抓取点不一致"
                    )

        return (
            approach,
            grasp_open,
            grasp_closed,
            lift,
            target_center,
            above_top_center,
            right_bias_base,
            left_compensation_base,
            applied_offset,
            flange_grasp,
            flange_grasp_unclamped,
            min_z_clamped,
            selected_grasp_rpy_deg,
            grasp_config,
        )

    def execute_wrist_grasp(self, skip_instruction_confirmation=False):
        if self.grasp_running:
            self.get_logger().warn(
                "抓取/识别线程已经在运行，本次抓取请求已忽略。"
            )
            return

        # 在线程创建前锁定，避免按键连发创建多个抓取线程。
        self.grasp_running = True

        try:
            Thread(
                target=self._execute_wrist_grasp_worker,
                args=(skip_instruction_confirmation,),
                daemon=True,
            ).start()
        except Exception:
            self.grasp_running = False
            raise

    def preview_wrist_object_detection(self):
        if self.grasp_running:
            self.get_logger().warn(
                "抓取/识别线程已经在运行，本次请求已忽略。"
            )
            return

        # 必须在线程启动前设置，避免按键连发造成竞态。
        self.grasp_running = True

        try:
            Thread(
                target=self._preview_wrist_object_detection_worker,
                daemon=True,
            ).start()
        except Exception:
            self.grasp_running = False
            raise

    def _preview_wrist_object_detection_worker(self):
        try:
            result = self.estimate_wrist_object_with_observation_scan()
            overlay = result["overlay"].copy()

            cv2.putText(
                overlay,
                (
                    f"center: "
                    f"{result['center'][0]:.3f}, "
                    f"{result['center'][1]:.3f}, "
                    f"{result['center'][2]:.3f}"
                ),
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

            preview_path = os.path.join(
                self.output_dir,
                (
                    "wrist_preview_"
                    f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    ".png"
                ),
            )
            cv2.imwrite(preview_path, overlay)

            self.last_wrist_preview = overlay
            self.last_wrist_preview_path = preview_path

            self.get_logger().info(
                "腕部相机识别结果: "
                f"center={result['center'].tolist()}, "
                f"top={result['top_center'].tolist()}, "
                f"bottom={result['bottom_center'].tolist()}, "
                f"height={result['object_height_m']:.3f}m, "
                "depth_grasp_center="
                f"{result['depth_grasp_center'].tolist()}, "
                f"cluster="
                f"{result['diagnostics']['cluster_points']}。"
            )
            self.get_logger().info(
                f"识别预览已保存: {preview_path}。"
                "此操作不执行抓取命令。"
            )

        except Exception as exc:
            snapshot = self.snapshot_rgbd()

            if snapshot[0] is not None:
                bgr, depth, _ = snapshot
                debug = bgr.copy()

                valid_depth = int(
                    np.count_nonzero(
                        (depth > 0.05)
                        & (depth < 1.5)
                    )
                )

                cv2.putText(
                    debug,
                    f"Wrist detection failed: {exc}",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 255),
                    2,
                )
                cv2.putText(
                    debug,
                    f"valid depth pixels: {valid_depth}",
                    (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

                preview_path = os.path.join(
                    self.output_dir,
                    (
                        "wrist_preview_failed_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                        ".png"
                    ),
                )
                cv2.imwrite(preview_path, debug)

                self.last_wrist_preview = debug
                self.last_wrist_preview_path = preview_path

                self.get_logger().warn(
                    f"失败诊断图已保存: {preview_path}"
                )

            self.get_logger().error(
                f"腕部相机识别失败: {exc}"
            )

        finally:
            self.grasp_running = False

    def iter_observation_scan_offsets(self):
        for offset_deg in OBSERVATION_SCAN_OFFSETS_DEG:
            yield float(offset_deg), "deg"

    def describe_observation_scan_offset(self, offset_value, unit):
        if unit == "deg":
            direction = "右侧" if float(offset_value) < 0.0 else "左侧"
            if math.isclose(float(offset_value), 0.0, abs_tol=1e-9):
                direction = "中心"
            return (
                f"{direction}观察: "
                f"J1_offset={float(offset_value):+.1f}deg"
            )
        raise RuntimeError(f"未知扫描单位: {unit}")

    def estimate_wrist_object_with_observation_scan(self):
        try:
            return self.estimate_wrist_object_in_base()
        except Exception as first_exc:
            if "open3d" in str(first_exc).lower():
                raise RuntimeError(
                    "三维定位依赖缺失，未执行左右扫描。"
                    f"请安装 open3d: {first_exc}"
                ) from first_exc
            if not OBSERVATION_SCAN_ENABLED:
                raise

            last_error = first_exc
            self.get_logger().warn(
                "当前观察位未找到目标，开始移动相机左右观察: "
                f"{first_exc}"
            )

        for offset_value, unit in self.iter_observation_scan_offsets():
            offset_text = self.describe_observation_scan_offset(
                offset_value,
                unit,
            )
            scan_joints = OBSERVATION_JOINTS_RAD.copy()
            scan_joints[0] += math.radians(float(offset_value))
            with self.joint_feedback_lock:
                current_joints = (
                    None
                    if self.last_joint_positions is None
                    else self.last_joint_positions.copy()
                )
            if current_joints is None:
                last_error = RuntimeError(
                    "无关节反馈，禁止执行左右扫描。"
                )
                break

            try:
                max_travel, minimum_z = validate_piper_joint_path(
                    current_joints,
                    scan_joints,
                    minimum_flange_z_m=0.20,
                )
            except Exception as exc:
                last_error = exc
                self.get_logger().error(
                    f"观察扫描路径检查失败: {offset_text}, {exc}"
                )
                break

            self.get_logger().info(
                "观察扫描: "
                f"mode=joint1, {offset_text}, "
                f"target_j1={math.degrees(float(scan_joints[0])):.1f}deg, "
                f"max_joint_travel={max_travel:.3f}rad, "
                f"minimum_flange_z={minimum_z:.3f}m"
            )
            if self.move_to_joint_pose(
                scan_joints,
                label=offset_text,
                gripper_m=OPEN_GRIPPER_M,
            ) is False:
                last_error = RuntimeError(
                    f"{offset_text} MOVE_J 未到达。"
                )
                break

            time.sleep(max(0.0, OBSERVATION_SCAN_SETTLE_S))

            attempts = max(1, OBSERVATION_SCAN_DETECTION_ATTEMPTS)
            for attempt in range(1, attempts + 1):
                try:
                    result = self.estimate_wrist_object_in_base()
                    result["observation_scan_mode"] = OBSERVATION_SCAN_MODE
                    result["observation_scan_axis"] = "joint1"
                    result["observation_scan_offset"] = float(offset_value)
                    result["observation_scan_offset_unit"] = unit
                    self.get_logger().info(
                        "观察扫描找到目标，立即停止扫描并进入抓取: "
                        f"{offset_text}, attempt={attempt}/{attempts}"
                    )
                    return result
                except Exception as exc:
                    if "open3d" in str(exc).lower():
                        raise RuntimeError(
                            "三维定位依赖缺失，已停止观察扫描。"
                            f"请安装 open3d: {exc}"
                        ) from exc
                    last_error = exc
                    if attempt < attempts:
                        time.sleep(0.10)

            self.get_logger().warn(
                "该观察位置未找到目标: "
                f"mode={OBSERVATION_SCAN_MODE}, {offset_text}, "
                f"attempts={attempts}, reason={last_error}"
            )

        if self.arm_enabled and not self.arm_faulted:
            self.get_logger().info(
                "左右扫描均未找到目标，返回中心观察位。"
            )
            if self.move_to_observation_joint_pose() is False:
                last_error = RuntimeError(
                    f"扫描失败后回中心观察位也未到达: {last_error}"
                )

        raise RuntimeError(
            "中心观察位及 J1 左右±10°均未找到目标: "
            f"{last_error}"
        )

    def refine_grasp_with_second_localization(
        self,
        approach,
        grasp_open,
        grasp_closed,
        lift,
    ):
        time.sleep(SECOND_LOCALIZATION_SETTLE_S)

        latest_result = None
        last_error = None
        samples = max(1, SECOND_LOCALIZATION_SAMPLES)

        for _ in range(samples):
            try:
                latest_result = self.estimate_wrist_object_in_base()
            except Exception as exc:
                last_error = exc
            time.sleep(0.08)

        if latest_result is None:
            message = (
                "二次视觉定位失败，沿用第一次定位结果继续抓取: "
                f"{last_error}"
            )
            if SECOND_LOCALIZATION_REQUIRED:
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return (
                approach,
                grasp_open,
                grasp_closed,
                lift,
            )

        (
            corrected_approach,
            corrected_grasp_open,
            corrected_grasp_closed,
            corrected_lift,
            *_unused,
        ) = self.build_grasp_waypoints(latest_result)

        dx = float(corrected_approach["x"] - approach["x"])
        dy = float(corrected_approach["y"] - approach["y"])
        dz = float(corrected_grasp_open["z"] - grasp_open["z"])
        xy_error = math.hypot(dx, dy)

        self.get_logger().info(
            "二次视觉定位修正: "
            f"dx={dx:.4f}m, dy={dy:.4f}m, "
            f"dz={dz:.4f}m, xy={xy_error:.4f}m"
        )

        if (
            corrected_grasp_open.get("y") is not None
            and latest_result.get("block_top_down_grasp")
            and float(corrected_grasp_open["y"])
            < BLOCK_TOP_DOWN_MIN_FLANGE_Y_M
        ):
            message = (
                "方块顶部二次定位 XY 修正会越过顶部抓取可达边界，"
                "保留第一次定位结果继续抓取: "
                f"corrected_y={float(corrected_grasp_open['y']):.4f}m < "
                f"limit={BLOCK_TOP_DOWN_MIN_FLANGE_Y_M:.4f}m, "
                f"original_y={float(grasp_open['y']):.4f}m"
            )
            if float(grasp_open["y"]) < BLOCK_TOP_DOWN_MIN_FLANGE_Y_M:
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return (
                approach,
                grasp_open,
                grasp_closed,
                lift,
            )

        if xy_error > SECOND_LOCALIZATION_MAX_XY_M:
            message = (
                "二次定位 XY 修正超限，沿用第一次定位结果继续抓取: "
                f"{xy_error:.4f}m > "
                f"{SECOND_LOCALIZATION_MAX_XY_M:.4f}m"
            )
            if SECOND_LOCALIZATION_REQUIRED:
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return (
                approach,
                grasp_open,
                grasp_closed,
                lift,
            )

        if abs(dz) > SECOND_LOCALIZATION_MAX_Z_M:
            action_text = (
                "已停止抓取"
                if SECOND_LOCALIZATION_REQUIRED
                else "沿用第一次定位结果继续抓取"
            )
            message = (
                f"二次定位 Z 修正超限，{action_text}: "
                f"{dz:.4f}m > "
                f"{SECOND_LOCALIZATION_MAX_Z_M:.4f}m"
            )
            if SECOND_LOCALIZATION_REQUIRED:
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return (
                approach,
                grasp_open,
                grasp_closed,
                lift,
            )

        corrected_current = dict(approach)

        if xy_error > 0.002:
            moved_pose = self.publish_xy_only_path(
                approach,
                target_x=float(corrected_approach["x"]),
                target_y=float(corrected_approach["y"]),
                duration=0.8,
                gripper=float(corrected_approach["gripper"]),
                label="二次视觉校准：只修正 X/Y",
            )
            if moved_pose is False:
                raise RuntimeError("二次视觉 XY 修正失败。")
            corrected_current = moved_pose

        approach_z_delta = float(
            corrected_approach["z"] - corrected_current["z"]
        )
        if abs(approach_z_delta) > 0.004:
            moved = self.publish_strict_vertical_path(
                corrected_current,
                target_z=float(corrected_approach["z"]),
                duration=0.6,
                gripper=float(corrected_approach["gripper"]),
                label="二次视觉校准：必要 Z 修正",
            )
            if moved is False:
                raise RuntimeError("二次视觉 Z 修正失败。")

        return (
            corrected_approach,
            corrected_grasp_open,
            corrected_grasp_closed,
            corrected_lift,
        )

    def wait_until_grasp_pose_reached_before_close(self, grasp_open):
        target_xyz = np.array(
            [
                float(grasp_open["x"]),
                float(grasp_open["y"]),
                float(grasp_open["z"]),
            ],
            dtype=np.float64,
        )
        target_rpy_deg = np.array(
            [
                math.degrees(float(grasp_open["roll"])),
                math.degrees(float(grasp_open["pitch"])),
                math.degrees(float(grasp_open["yaw"])),
            ],
            dtype=np.float64,
        )

        is_block_target = is_block_grasp_target(
            self.target_class_id,
            self.target_model_class_name,
            self.detection_target,
        )
        position_tolerance_m = (
            BLOCK_GRASP_PRE_CLOSE_POSITION_TOL_M
            if is_block_target
            else GRASP_PRE_CLOSE_POSITION_TOL_M
        )
        z_tolerance_m = (
            BLOCK_GRASP_PRE_CLOSE_Z_TOL_M
            if is_block_target
            else GRASP_PRE_CLOSE_Z_TOL_M
        )
        deadline = time.time() + max(0.1, GRASP_PRE_CLOSE_WAIT_TIMEOUT_S)
        last_error = None

        while rclpy.ok():
            try:
                _, current_xyz, current_rpy_deg = self.get_cached_end_pose()
                xyz_error = float(np.linalg.norm(current_xyz - target_xyz))
                z_error = float(abs(current_xyz[2] - target_xyz[2]))
                rpy_error = max(
                    abs(
                        wrap_angle_deg(
                            float(current_rpy_deg[index])
                            - float(target_rpy_deg[index])
                        )
                    )
                    for index in range(3)
                )

                self.get_logger().info(
                    "闭合前到位检查: "
                    f"xyz_error={xyz_error:.4f}m, "
                    f"z_error={z_error:.4f}m, "
                    f"rpy_error={rpy_error:.2f}deg"
                )

                if (
                    xyz_error <= position_tolerance_m
                    and z_error <= z_tolerance_m
                    and rpy_error <= GRASP_PRE_CLOSE_RPY_TOL_DEG
                ):
                    return True

                last_error = (
                    f"xyz={xyz_error:.4f}m, "
                    f"z={z_error:.4f}m, "
                    f"rpy={rpy_error:.2f}deg"
                )
            except Exception as exc:
                last_error = str(exc)
                self.get_logger().warn(
                    f"闭合前到位检查读取失败: {exc}"
                )

            remaining = deadline - time.time()
            if remaining <= 0.0:
                break

            hold_until = time.time() + min(0.20, remaining)
            while rclpy.ok() and time.time() < hold_until:
                self.send_pos_command(grasp_open)
                time.sleep(1.0 / 30.0)

        raise RuntimeError(
            "末端未确认到达夹取高度，禁止闭合夹爪: "
            f"{last_error}"
        )

    def plan_adaptive_block_grasp_path(
        self,
        object_result,
        start_joints_rad,
    ):
        start_joints = np.asarray(
            start_joints_rad,
            dtype=np.float64,
        ).reshape(6)
        calibrated_rpy = (
            np.asarray(BLOCK_TOP_DOWN_RPY_DEG, dtype=np.float64)
            if BLOCK_TOP_DOWN_RPY_DEG is not None
            else np.array(
                [
                    SIMPLE_GRASP_ROLL_DEG,
                    SIMPLE_GRASP_PITCH_DEG,
                    SIMPLE_GRASP_PRIMARY_YAW_DEG,
                ],
                dtype=np.float64,
            )
        )
        candidate_specs = build_block_rpy_candidates(
            roll_deg=float(calibrated_rpy[0]),
            fallback_yaw_deg=float(calibrated_rpy[2]),
            object_yaw_deg=None,
            pitch_candidates_deg=(
                BLOCK_TERMINAL_PITCH_CANDIDATES_DEG
            ),
            yaw_offset_candidates_deg=(
                BLOCK_TERMINAL_YAW_OFFSETS_DEG
            ),
        )
        evaluations = []
        failures = []

        for candidate_spec in candidate_specs:
            candidate_result = dict(object_result)
            candidate_rpy_deg = np.array(
                [
                    candidate_spec["roll_deg"],
                    candidate_spec["pitch_deg"],
                    candidate_spec["yaw_deg"],
                ],
                dtype=np.float64,
            )
            try:
                waypoints = self.build_grasp_waypoints(
                    candidate_result,
                    block_grasp_rpy_override_deg=(
                        candidate_rpy_deg
                    ),
                )
                (
                    _approach,
                    grasp_open,
                    grasp_closed,
                    lift,
                    *_waypoint_diagnostics,
                ) = waypoints

                if float(grasp_open["y"]) < MIN_FLANGE_Y_M:
                    raise RuntimeError(
                        "最终抓取点超出机械臂工作区: "
                        f"y={float(grasp_open['y']):.3f}m < "
                        f"limit={MIN_FLANGE_Y_M:.3f}m"
                    )

                pregrasp = build_world_yz_pregrasp_pose(
                    grasp_open,
                    backoff_y_m=BLOCK_FINAL_APPROACH_Y_M,
                    lift_z_m=BLOCK_FINAL_APPROACH_Z_M,
                )
                if (
                    float(pregrasp["z"])
                    < BLOCK_PREGRASP_MIN_FLANGE_Z_M
                ):
                    raise RuntimeError(
                        "方块预抓位高度不足: "
                        f"z={float(pregrasp['z']):.3f}m < "
                        "limit="
                        f"{BLOCK_PREGRASP_MIN_FLANGE_Z_M:.3f}m"
                    )

                pregrasp_joints = solve_piper_ik_pose(
                    pregrasp,
                    seeds=[
                        start_joints,
                        OBSERVATION_JOINTS_RAD,
                        BLOCK_TOP_DOWN_JOINTS_RAD,
                    ],
                )
                max_joint_travel, move_j_minimum_z = (
                    validate_piper_joint_path(
                        start_joints,
                        pregrasp_joints,
                        minimum_flange_z_m=(
                            BLOCK_PREGRASP_MIN_FLANGE_Z_M
                        ),
                        maximum_joint_travel_rad=(
                            BLOCK_PREGRASP_MAX_JOINT_TRAVEL_RAD
                        ),
                    )
                )
                locked_metrics = (
                    validate_piper_locked_cartesian_path(
                        pregrasp,
                        grasp_open,
                        pregrasp_joints,
                        sample_count=BLOCK_LOCKED_PATH_SAMPLES,
                        maximum_joint_step_rad=(
                            BLOCK_LOCKED_PATH_MAX_JOINT_STEP_RAD
                        ),
                        return_metrics=True,
                    )
                )
                if (
                    locked_metrics["minimum_z_m"]
                    < MIN_FLANGE_Z_M - 0.003
                ):
                    raise RuntimeError(
                        "方块最终进给路径低于法兰硬限位: "
                        f"minimum_z="
                        f"{locked_metrics['minimum_z_m']:.3f}m < "
                        f"limit={MIN_FLANGE_Z_M:.3f}m"
                    )

                pregrasp_margin = np.minimum(
                    pregrasp_joints
                    - PIPER_JOINT_LIMITS_RAD[:, 0],
                    PIPER_JOINT_LIMITS_RAD[:, 1]
                    - pregrasp_joints,
                )
                minimum_joint_margin = min(
                    float(np.min(pregrasp_margin)),
                    float(
                        locked_metrics[
                            "minimum_joint_margin_rad"
                        ]
                    ),
                )
                evaluations.append(
                    {
                        **candidate_spec,
                        "minimum_joint_margin_rad": (
                            minimum_joint_margin
                        ),
                        "max_joint_step_rad": float(
                            locked_metrics["max_joint_step_rad"]
                        ),
                        "max_joint_travel_rad": float(
                            max_joint_travel
                        ),
                        "move_j_minimum_z_m": float(
                            move_j_minimum_z
                        ),
                        "locked_path_minimum_z_m": float(
                            locked_metrics["minimum_z_m"]
                        ),
                        "pregrasp": pregrasp,
                        "pregrasp_joints": pregrasp_joints,
                        "grasp_open": grasp_open,
                        "grasp_closed": grasp_closed,
                        "lift": lift,
                        "object_result": candidate_result,
                        "waypoints": waypoints,
                    }
                )
            except Exception as exc:
                failures.append(
                    f"{candidate_spec['name']}: {exc}"
                )

        if not evaluations:
            raise RuntimeError(
                "方块所有近垂直终态候选均不可达: "
                + "; ".join(failures)
            )

        selected = choose_reachable_block_candidate(
            evaluations,
            minimum_joint_margin_rad=(
                BLOCK_MIN_JOINT_MARGIN_RAD
            ),
        )
        self.get_logger().info(
            "方块自适应终态规划通过: "
            f"candidate={selected['name']}, "
            f"rpy_deg=({selected['roll_deg']:.1f}, "
            f"{selected['pitch_deg']:.1f}, "
            f"{selected['yaw_deg']:.1f}), "
            f"yaw_source={selected['yaw_source']}, "
            f"yaw_offset={selected['yaw_offset_deg']:.1f}deg, "
            f"joint_margin="
            f"{selected['minimum_joint_margin_rad']:.3f}rad, "
            f"move_j_travel="
            f"{selected['max_joint_travel_rad']:.3f}rad, "
            f"locked_step="
            f"{selected['max_joint_step_rad']:.3f}rad, "
            f"move_j_min_z="
            f"{selected['move_j_minimum_z_m']:.3f}m, "
            f"locked_min_z="
            f"{selected['locked_path_minimum_z_m']:.3f}m"
        )
        if failures:
            self.get_logger().info(
                "其余方块姿态候选被拒绝: "
                + "; ".join(failures)
            )
        return selected

    def execute_adaptive_block_grasp_path(
        self,
        object_result,
        start_joints_rad,
    ):
        plan = self.plan_adaptive_block_grasp_path(
            object_result,
            start_joints_rad,
        )
        pregrasp = dict(plan["pregrasp"])
        grasp_open = dict(plan["grasp_open"])
        grasp_closed = dict(plan["grasp_closed"])
        open_gripper = float(grasp_open["gripper"])
        closed_gripper = float(grasp_closed["gripper"])

        if self.move_to_joint_pose(
            plan["pregrasp_joints"],
            label="方块自由路径预抓位",
            gripper_m=open_gripper,
            timeout_s=max(
                30.0,
                self.observation_joint_timeout_s,
            ),
        ) is False:
            raise RuntimeError("方块预抓位 MOVE_J 未到达。")
        self.wait_until_block_gripper_open(open_gripper)

        if self.publish_locked_rpy_linear_path(
            pregrasp,
            grasp_open,
            duration=FINAL_DESCENT_DURATION_S,
            gripper=open_gripper,
            label=(
                "方块最终短距离进给：锁定终态姿态，"
                "锁定世界 X，沿世界 -Y/-Z 前下方靠近方块中心"
            ),
        ) is False:
            raise RuntimeError("方块最终锁姿态进给失败。")

        if self.publish_pose_for(
            grasp_open,
            duration=GRASP_PRE_CLOSE_DWELL_S,
        ) is False:
            raise RuntimeError("方块闭合前姿态保持失败。")
        self.wait_until_grasp_pose_reached_before_close(
            grasp_open
        )

        if STOP_BEFORE_CLOSE:
            self.get_logger().warn(
                "方块测试已停在闭合前，未执行夹取或放置。"
            )
            return False

        if self.publish_pose_for(
            grasp_closed,
            duration=GRASP_CLOSE_DWELL_S,
        ) is False:
            raise RuntimeError("方块夹爪闭合失败。")
        self.activate_gripper_hold_for_current_target(closed_gripper)
        self.get_logger().info(
            "方块夹爪强制闭合保持: "
            f"target={closed_gripper:.4f}m；"
            "放置释放前不根据反馈增大开度。"
        )
        if self.publish_pose_for(
            grasp_closed,
            duration=BLOCK_POST_CLOSE_DWELL_S,
        ) is False:
            raise RuntimeError("方块夹爪接触保持失败。")

        retreat = dict(pregrasp)
        retreat["gripper"] = closed_gripper
        if self.publish_locked_rpy_linear_path(
            grasp_closed,
            retreat,
            duration=max(
                BLOCK_INITIAL_LIFT_DURATION_S,
                FINAL_DESCENT_DURATION_S,
            ),
            gripper=closed_gripper,
            label=(
                "方块夹紧后锁定终态姿态，"
                "沿原进给方向反向抽回"
            ),
        ) is False:
            raise RuntimeError("方块夹紧后沿原路抽回失败。")

        if self.move_to_joint_pose(
            OBSERVATION_JOINTS_RAD,
            label="携带方块返回观察运输位",
            gripper_m=closed_gripper,
            timeout_s=max(
                30.0,
                self.observation_joint_timeout_s,
            ),
        ) is False:
            raise RuntimeError("方块携取后返回运输位失败。")
        return True

    def execute_upright_bottle_grasp_path(
        self,
        current_pose,
        approach,
        grasp_open,
        grasp_closed,
        selected_grasp_rpy_deg,
        open_gripper,
        closed_gripper,
        base_alignment_plan=None,
    ):
        self.get_logger().info(
            "直立瓶子抓取路径: "
            "先进行离线可达性检查，再执行预抓、前探、"
            "到位确认、闭合和返回。"
        )

        safe_z = max(
            float(current_pose["z"]),
            float(approach["z"]),
            float(SAFE_SIMPLE_POSE_Z_M),
        )

        if abs(safe_z - float(current_pose["z"])) > 0.004:
            current_safe = dict(current_pose)
            current_safe["z"] = safe_z
            if self.publish_strict_vertical_path(
                current_pose,
                target_z=safe_z,
                duration=SAFE_LIFT_DURATION_S,
                gripper=open_gripper,
                label="瓶子斜抓步骤1：保持当前姿态升到安全高度",
            ) is False:
                raise RuntimeError(
                    "瓶子斜抓安全升高失败。"
                )
        else:
            current_safe = dict(current_pose)
            current_safe["z"] = safe_z

        if not BOTTLE_TOP_DOWN_PATH:
            self.get_logger().info(
                "直立瓶子 locked_direct 路径: "
                "先只转 J1 使夹爪正前方对准瓶子，"
                "再用 MOVE_J 向前下方伸展到预抓位，"
                "再锁定夹爪姿态短距离前探；"
                "抽回后沿原路返回观察位。"
            )

            with self.joint_feedback_lock:
                current_joints = (
                    None
                    if self.last_joint_positions is None
                    else self.last_joint_positions.copy()
                )
            if current_joints is None:
                raise RuntimeError(
                    "无关节反馈，禁止计算预抓位 MOVE_J。"
                )

            path_start_joints = current_joints
            alignment_joints = None
            if base_alignment_plan is not None:
                alignment_joints = np.asarray(
                    base_alignment_plan["alignment_joints"],
                    dtype=np.float64,
                ).reshape(6)
                validate_piper_joint_path(
                    current_joints,
                    alignment_joints,
                    minimum_flange_z_m=0.20,
                )
                path_start_joints = alignment_joints

            approach_joints = solve_piper_ik_pose(
                approach,
                seeds=[
                    path_start_joints,
                    current_joints,
                    OBSERVATION_JOINTS_RAD,
                    [0.1, 1.2, -1.0, 0.0, 0.5, 0.0],
                    [0.2, 1.5, -0.8, 0.5, -0.4, -0.4],
                ],
            )
            minimum_path_z = max(
                float(MIN_FLANGE_Z_M),
                min(
                    float(current_pose["z"]),
                    float(approach["z"]),
                )
                - 0.03,
            )
            forward_down_metrics = validate_piper_forward_down_joint_path(
                path_start_joints,
                approach_joints,
                minimum_flange_z_m=minimum_path_z,
            )
            max_cartesian_joint_step, cartesian_minimum_z = (
                validate_piper_locked_cartesian_path(
                    approach,
                    grasp_open,
                    approach_joints,
                )
            )
            if (
                cartesian_minimum_z
                < float(MIN_FLANGE_Z_M) - 0.003
            ):
                raise RuntimeError(
                    "瓶子直线前探低于法兰硬限位: "
                    f"minimum_z={cartesian_minimum_z:.3f}m < "
                    f"limit={MIN_FLANGE_Z_M:.3f}m"
            )
            self.get_logger().info(
                "预抓位离线规划通过: "
                f"joints_rad={np.round(approach_joints, 4).tolist()}, "
                f"max_joint_travel="
                f"{forward_down_metrics['max_joint_travel_rad']:.3f}rad, "
                f"minimum_flange_z="
                f"{forward_down_metrics['minimum_z_m']:.3f}m, "
                f"lateral_curve="
                f"{forward_down_metrics['max_lateral_m'] * 100.0:.1f}cm, "
                f"upward_clearance="
                f"{forward_down_metrics['max_upward_m'] * 100.0:.1f}cm, "
                f"pose_deviation="
                f"{forward_down_metrics['max_orientation_deviation_deg']:.1f}deg, "
                "locked_path_max_joint_step="
                f"{max_cartesian_joint_step:.3f}rad, "
                "locked_path_minimum_z="
                f"{cartesian_minimum_z:.3f}m"
            )
            if not BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS:
                self.get_logger().warn(
                    "前下方轨迹外形限位已关闭："
                    "横向弯曲/上拱/中间姿态只记录不拒绝；"
                    "关节物理限位、最低高度和 IK 连续性仍生效。"
                )

            if alignment_joints is not None:
                self.get_logger().info(
                    "瓶子基座对准: "
                    f"只转 J1="
                    f"{base_alignment_plan['joint1_delta_deg']:+.1f}deg, "
                    f"aim_error="
                    f"{base_alignment_plan['aim_error_deg']:.2f}deg, "
                    "J2~J6 保持观察位。"
                )
                if self.move_to_joint_pose(
                    alignment_joints,
                    label="瓶子基座对准位（只转J1）",
                    gripper_m=open_gripper,
                    timeout_s=max(
                        20.0,
                        self.observation_joint_timeout_s,
                    ),
                ) is False:
                    raise RuntimeError(
                        "瓶子基座对准位 MOVE_J 未到达。"
                    )

            if self.move_to_joint_pose(
                approach_joints,
                label="瓶子前下方预抓位",
                gripper_m=open_gripper,
                timeout_s=max(
                    30.0,
                    self.observation_joint_timeout_s,
                ),
            ) is False:
                raise RuntimeError(
                    "瓶子预抓位 MOVE_J 未到达。"
                )

            time.sleep(max(0.0, OVERHEAD_DWELL_S))

            for key in ("roll", "pitch", "yaw"):
                if not math.isclose(
                    float(approach[key]),
                    float(grasp_open[key]),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                ):
                    raise RuntimeError(
                        f"瓶子前探姿态锁定失败: {key} 不一致"
                    )

            if self.publish_locked_rpy_linear_path(
                approach,
                grasp_open,
                duration=FINAL_DESCENT_DURATION_S,
                gripper=open_gripper,
                label="瓶子抓取步骤3：先到预抓取点后，夹爪平行前探到瓶身中部",
            ) is False:
                raise RuntimeError(
                    "瓶子 locked_direct 靠近瓶身中部失败。"
                )

            self.publish_pose_for(
                grasp_open,
                duration=GRASP_PRE_CLOSE_DWELL_S,
            )
            self.wait_until_grasp_pose_reached_before_close(
                grasp_open
            )

            if STOP_BEFORE_CLOSE:
                self.get_logger().warn(
                    "已停在闭合夹爪前："
                    "夹爪保持张开，未闭合，"
                    "机械臂保持使能。"
                )
                return

            close_started_at = time.monotonic()
            if self.publish_pose_for(
                grasp_closed,
                duration=GRASP_CLOSE_DWELL_S,
            ) is False:
                raise RuntimeError("瓶子夹爪闭合失败。")
            closed_gripper = self.resolve_post_close_hold_gripper(
                closed_gripper,
                close_started_at,
            )
            grasp_closed["gripper"] = closed_gripper
            self.activate_gripper_hold_for_current_target(closed_gripper)

            retreat = dict(approach)
            retreat["gripper"] = closed_gripper
            if self.publish_locked_rpy_linear_path(
                grasp_closed,
                retreat,
                duration=max(0.8, FINAL_DESCENT_DURATION_S),
                gripper=closed_gripper,
                label="瓶子抓取步骤4：夹紧后夹爪平行按原路径抽回",
            ) is False:
                raise RuntimeError(
                    "瓶子 locked_direct 退出失败。"
                )

            if alignment_joints is not None:
                if self.move_to_joint_pose(
                    alignment_joints,
                    label="携瓶沿原路收回到基座对准位",
                    gripper_m=closed_gripper,
                    timeout_s=max(
                        30.0,
                        self.observation_joint_timeout_s,
                    ),
                ) is False:
                    raise RuntimeError(
                        "瓶子携取后沿原路收回失败。"
                    )

            if self.move_to_joint_pose(
                OBSERVATION_JOINTS_RAD,
                label="携瓶返回观察位",
                gripper_m=closed_gripper,
                timeout_s=max(
                    30.0,
                    self.observation_joint_timeout_s,
                ),
            ) is False:
                raise RuntimeError(
                    "瓶子携取后 MOVE_J 回观察位失败。"
                )
            return

        locked_safe = dict(current_safe)
        locked_safe["gripper"] = open_gripper
        overhead = dict(approach)
        overhead["z"] = float(locked_safe["z"])

        overhead_safe = self.publish_xy_only_path(
            locked_safe,
            target_x=float(overhead["x"]),
            target_y=float(overhead["y"]),
            duration=OVERHEAD_MOVE_DURATION_S,
            gripper=open_gripper,
            label="瓶子抓取步骤2：锁定观察姿态，在安全高度平面移动到抓取点上方",
        )
        if overhead_safe is False:
            raise RuntimeError(
                "瓶子安全高度平面移动失败。"
            )

        if abs(float(overhead_safe["z"]) - float(approach["z"])) > 0.004:
            if self.publish_strict_vertical_path(
                overhead_safe,
                target_z=float(approach["z"]),
                duration=APPROACH_DESCENT_DURATION_S,
                gripper=open_gripper,
                label="瓶子抓取步骤3：锁定观察姿态，垂直到抓取点上方高度",
            ) is False:
                raise RuntimeError(
                    "瓶子垂直到上方预抓高度失败。"
                )

        self.publish_pose_for(
            approach,
            duration=OVERHEAD_DWELL_S,
        )

        if self.publish_strict_vertical_path(
            approach,
            target_z=float(grasp_open["z"]),
            duration=FINAL_DESCENT_DURATION_S,
            gripper=open_gripper,
            label="瓶子抓取步骤4：锁定观察姿态，夹爪张开垂直下降到瓶身中部",
        ) is False:
            raise RuntimeError(
                "瓶子垂直下降到瓶身中部失败。"
            )

        self.publish_pose_for(
            grasp_open,
            duration=GRASP_PRE_CLOSE_DWELL_S,
        )
        self.wait_until_grasp_pose_reached_before_close(
            grasp_open
        )

        if STOP_BEFORE_CLOSE:
            self.get_logger().warn(
                "已停在闭合夹爪前："
                "夹爪保持张开，未闭合，"
                "机械臂保持使能。"
            )
            return

        close_started_at = time.monotonic()
        if self.publish_pose_for(
            grasp_closed,
            duration=GRASP_CLOSE_DWELL_S,
        ) is False:
            raise RuntimeError("瓶子夹爪闭合失败。")
        closed_gripper = self.resolve_post_close_hold_gripper(
            closed_gripper,
            close_started_at,
        )
        grasp_closed["gripper"] = closed_gripper
        self.activate_gripper_hold_for_current_target(closed_gripper)

        retreat = dict(approach)
        retreat["gripper"] = closed_gripper
        if self.publish_strict_vertical_path(
            grasp_closed,
            target_z=float(retreat["z"]),
            duration=max(0.8, FINAL_DESCENT_DURATION_S),
            gripper=closed_gripper,
            label="瓶子抓取步骤5：夹紧后严格垂直上升",
        ) is False:
            raise RuntimeError(
                "瓶子夹紧后垂直上升失败。"
            )

        return_pose = dict(current_pose)
        return_pose["gripper"] = closed_gripper
        return_overhead = dict(return_pose)
        return_overhead["z"] = float(retreat["z"])
        returned_overhead = self.publish_xy_only_path(
            retreat,
            target_x=float(return_overhead["x"]),
            target_y=float(return_overhead["y"]),
            duration=GRASP_RETURN_HOME_DURATION_S,
            gripper=closed_gripper,
            label="瓶子抓取步骤6：夹紧后锁定观察姿态，同高度平面回到原位上方",
        )
        if returned_overhead is False:
            raise RuntimeError(
                "瓶子夹紧后平面回到原位上方失败。"
            )
        if abs(float(returned_overhead["z"]) - float(return_pose["z"])) > 0.004:
            if self.publish_strict_vertical_path(
                returned_overhead,
                target_z=float(return_pose["z"]),
                duration=APPROACH_DESCENT_DURATION_S,
                gripper=closed_gripper,
                label="瓶子抓取步骤7：夹紧后垂直回到原位高度",
            ) is False:
                raise RuntimeError(
                    "瓶子夹紧后垂直回到原位高度失败。"
                )

    def _execute_wrist_grasp_worker(
        self,
        skip_instruction_confirmation=False,
    ):
        retry_after_failure = False
        try:
            if not self.arm_enabled:
                self.get_logger().warn(
                    "机械臂未使能，请等待初始化完成，"
                    "或先使能后再按 g。"
                )
                return

            if not skip_instruction_confirmation:
                self.ensure_instruction_target_confirmed_before_grasp()
            else:
                self.get_logger().warn(
                    "二次抓取：沿用上次图纸识别目标，"
                    "跳过图纸 YOLO 阶段。"
                )

            result = self.estimate_wrist_object_with_observation_scan()

            target_class_id = result.get("target_class_id")
            upright_bottle_grasp = (
                BOTTLE_UPRIGHT_SIDE_GRASP
                and is_bottle_grasp_target(
                    target_class_id,
                    result.get("target_model_class_name"),
                    result.get("target_prompt"),
                )
            )
            base_alignment_plan = None
            bottle_rpy_override_deg = None
            if (
                upright_bottle_grasp
                and not BOTTLE_TOP_DOWN_PATH
                and BOTTLE_BASE_AIM_ENABLED
            ):
                with self.joint_feedback_lock:
                    detection_joints = (
                        None
                        if self.last_joint_positions is None
                        else self.last_joint_positions.copy()
                    )
                if detection_joints is None:
                    raise RuntimeError(
                        "无关节反馈，禁止计算瓶子基座对准角。"
                    )
                base_alignment_plan = plan_piper_base_aim_at_target(
                    detection_joints,
                    result["depth_grasp_center"],
                    minimum_flange_z_m=0.20,
                )
                bottle_rpy_override_deg = base_alignment_plan[
                    "aligned_rpy_deg"
                ]

            (
                approach,
                grasp_open,
                grasp_closed,
                lift,
                center_target,
                above_top_center,
                right_bias_base,
                left_compensation_base,
                applied_offset,
                flange_grasp,
                flange_unclamped,
                min_z_clamped,
                selected_grasp_rpy_deg,
                grasp_config,
            ) = self.build_grasp_waypoints(
                result,
                bottle_grasp_rpy_override_deg=bottle_rpy_override_deg,
            )

            open_gripper = float(approach["gripper"])
            closed_gripper = float(grasp_closed["gripper"])

            if float(flange_grasp[1]) < MIN_FLANGE_Y_M:
                raise RuntimeError(
                    "抓取目标超出安全工作区，已拒绝发送抓取运动: "
                    f"flange_grasp_y={float(flange_grasp[1]):.3f}m < "
                    f"limit={MIN_FLANGE_Y_M:.3f}m, "
                    f"flange_grasp_target={flange_grasp.tolist()}, "
                    f"object_center={result['center'].tolist()}"
                )

            if (
                result.get("block_top_down_grasp")
                and float(flange_grasp[1])
                < BLOCK_TOP_DOWN_MIN_FLANGE_Y_M
            ):
                raise RuntimeError(
                    "方块顶部抓取目标超出当前顶部姿态可达边界，"
                    "已拒绝发送抓取运动: "
                    f"flange_grasp_y={float(flange_grasp[1]):.3f}m < "
                    f"limit={BLOCK_TOP_DOWN_MIN_FLANGE_Y_M:.3f}m, "
                    f"flange_grasp_target={flange_grasp.tolist()}, "
                    f"object_center={result['center'].tolist()}"
                )

            yaw_delta_deg = result.get("selected_grasp_yaw_delta_deg")
            if (
                yaw_delta_deg is not None
                and abs(float(yaw_delta_deg)) > MAX_GRASP_YAW_DELTA_DEG
            ):
                raise RuntimeError(
                    "抓取姿态 yaw 调整过大，已拒绝发送抓取运动: "
                    f"yaw_delta={float(yaw_delta_deg):.1f}deg > "
                    f"limit={MAX_GRASP_YAW_DELTA_DEG:.1f}deg, "
                    f"selected_yaw={result.get('selected_grasp_yaw_deg')}, "
                    f"yaw_source={result.get('selected_grasp_yaw_source')}, "
                    f"object_axes={result.get('object_axes')}"
                )

            self.get_logger().info(
                "腕部相机定位: "
                f"object_top={result['top_center'].tolist()}, "
                f"object_bottom={result['bottom_center'].tolist()}, "
                f"object_height={result['object_height_m']:.3f}, "
                "depth_grasp_center="
                f"{result['depth_grasp_center'].tolist()}, "
                f"above_top_center={above_top_center.tolist()}, "
                f"grasp_center_target={center_target.tolist()}, "
                f"above_top_clearance={ABOVE_TOP_CLEARANCE_M:.3f}, "
                f"fine_tune_base={GRASP_FINE_TUNE_BASE_M.tolist()}, "
                f"right_bias={GRASP_RIGHT_BIAS_M:.3f}, "
                "left_compensation="
                f"{float(np.linalg.norm(left_compensation_base)):.3f}, "
                f"right_bias_base={right_bias_base.tolist()}, "
                f"left_compensation_base={left_compensation_base.tolist()}, "
                f"height_fraction={GRASP_HEIGHT_FRACTION:.2f}, "
                f"bottle_keep_observation_rpy={BOTTLE_KEEP_OBSERVATION_RPY}, "
                f"bottle_rpy_adjust_threshold={BOTTLE_RPY_ADJUST_THRESHOLD_DEG:.1f}°, "
                f"z_extra={GRASP_CENTER_EXTRA_Z_M:.3f}, "
                f"extra_descent={GRASP_EXTRA_DESCENT_M:.3f}, "
                f"pre_close_dwell={GRASP_PRE_CLOSE_DWELL_S:.3f}, "
                f"pre_close_wait={GRASP_PRE_CLOSE_WAIT_TIMEOUT_S:.3f}, "
                f"grasp_config={grasp_config}, "
                "applied_tcp_offset_base="
                f"{applied_offset.tolist()}, "
                "flange_grasp_unclamped="
                f"{flange_unclamped.tolist()}, "
                f"flange_grasp_target={flange_grasp.tolist()}, "
                f"min_z_clamped={min_z_clamped}, "
                f"simple_grasp_rpy_deg="
                f"{selected_grasp_rpy_deg.tolist()}, "
                f"yaw_source={result.get('selected_grasp_yaw_source')}, "
                f"selected_yaw_deg={result.get('selected_grasp_yaw_deg')}, "
                f"yaw_delta_deg={result.get('selected_grasp_yaw_delta_deg')}, "
                f"require_axis_yaw={REQUIRE_OBJECT_AXIS_YAW}, "
                f"object_axes={result.get('object_axes')}, "
                f"block_top_down_grasp={result.get('block_top_down_grasp')}, "
                "block_forward_extra="
                f"{result.get('block_forward_extra')}, "
                "block_grasp_compensation="
                f"{result.get('block_grasp_compensation')}, "
                "block_observation_rpy_override="
                f"{result.get('block_observation_rpy_override')}, "
                "block_outer_roll_pitch_override="
                f"{result.get('block_outer_roll_pitch_override')}, "
                f"upright_bottle_depth={result['diagnostics'].get('upright_bottle_depth')}, "
                f"upright_bottle_grasp={result.get('upright_bottle_grasp')}, "
                f"roi_applied={result['diagnostics'].get('roi_applied')}, "
                f"cluster="
                f"{result['diagnostics']['cluster_points']}"
            )

            current_xyz = np.asarray(
                result["robot_xyz_m"],
                dtype=np.float64,
            ).reshape(3)
            current_rpy_deg = np.asarray(
                result["robot_rpy_deg"],
                dtype=np.float64,
            ).reshape(3)

            current_pose = {
                **approach,
                "x": float(current_xyz[0]),
                "y": float(current_xyz[1]),
                "z": float(current_xyz[2]),
                "roll": math.radians(
                    float(current_rpy_deg[0])
                ),
                "pitch": math.radians(
                    float(current_rpy_deg[1])
                ),
                "yaw": math.radians(
                    float(current_rpy_deg[2])
                ),
                "gripper": open_gripper,
            }

            if (
                target_class_id is not None
                and int(target_class_id) == 1
                and BLOCK_TOP_DOWN_GRASP
            ):
                with self.joint_feedback_lock:
                    block_start_joints = (
                        None
                        if self.last_joint_positions is None
                        else self.last_joint_positions.copy()
                    )
                if block_start_joints is None:
                    raise RuntimeError(
                        "无关节反馈，禁止规划方块自适应抓取路径。"
                    )
                self.get_logger().info(
                    "已识别为方块，使用自适应终态抓取："
                    "自由 MOVE_J 到预抓位，最后短距离锁姿态进给，"
                    "夹紧后沿原路抽回；不执行世界坐标垂直下探。"
                )
                completed = self.execute_adaptive_block_grasp_path(
                    result,
                    block_start_joints,
                )
                if not completed:
                    return
                try:
                    execute_place_after_grasp(
                        self,
                        R_CAM_TO_GRIPPER,
                        T_CAM_TO_GRIPPER,
                        PLACE_OBSERVATION_JOINTS_RAD,
                        solve_piper_ik_pose,
                    )
                except Exception as place_exc:
                    self.get_logger().error(
                        "方块抓取成功，但放置阶段失败；"
                        "不执行抓取失败恢复，避免误开夹爪: "
                        f"{place_exc}"
                    )
                self.get_logger().info(
                    "方块自适应抓取序列完成，机械臂保持使能。"
                )
                return

            if (
                target_class_id is not None
                and int(target_class_id) == 0
                and BOTTLE_UPRIGHT_SIDE_GRASP
            ):
                self.get_logger().info(
                    "已识别为直立瓶子，使用中部斜抓，"
                    "不执行垂直下探路径。"
                )
                self.execute_upright_bottle_grasp_path(
                    current_pose,
                    approach,
                    grasp_open,
                    grasp_closed,
                    selected_grasp_rpy_deg,
                    open_gripper,
                    closed_gripper,
                    base_alignment_plan=base_alignment_plan,
                )
                try:
                    execute_place_after_grasp(
                        self,
                        R_CAM_TO_GRIPPER,
                        T_CAM_TO_GRIPPER,
                        PLACE_OBSERVATION_JOINTS_RAD,
                        solve_piper_ik_pose,
                    )
                except Exception as place_exc:
                    self.get_logger().error(
                        "直立瓶子抓取成功，但放置阶段失败；"
                        "不执行抓取失败恢复，避免误开夹爪: "
                        f"{place_exc}"
                    )
                self.get_logger().info(
                    "直立瓶子斜抓序列完成，机械臂保持使能。"
                )
                return

            safe_z = max(
                float(current_xyz[2]),
                float(approach["z"]),
                float(SAFE_SIMPLE_POSE_Z_M),
            )

            self.get_logger().info(
                "非瓶子抓取执行路径: "
                "先保持当前姿态只升高 -> "
                "必要时在安全高度切到抓取姿态 -> "
                "锁定姿态高位水平对准 -> "
                "锁定抓取 RPY 严格垂直下降。"
            )

            # 步骤1：只沿 Z 上升，保持当前真实姿态和 XY。
            if (
                abs(
                    safe_z
                    - float(current_pose["z"])
                )
                > 0.004
            ):
                current_safe = dict(current_pose)
                current_safe["z"] = safe_z
                if self.publish_strict_vertical_path(
                    current_pose,
                    target_z=safe_z,
                    duration=SAFE_LIFT_DURATION_S,
                    gripper=open_gripper,
                    label=(
                        "步骤1：保持当前姿态只升到安全高度"
                    ),
                ) is False:
                    raise RuntimeError(
                        "步骤1安全高度上升失败。"
                    )
            else:
                current_safe = dict(current_pose)
                current_safe["z"] = safe_z

            block_top_down_grasp = bool(
                result.get("block_top_down_grasp")
            )
            block_lock_observation_rpy = bool(
                result.get("block_observation_rpy_override")
            )

            if block_top_down_grasp:
                if BLOCK_TOP_DOWN_USE_MOVE_J:
                    if self.move_to_joint_pose(
                        BLOCK_TOP_DOWN_JOINTS_RAD,
                        label="方块顶部抓取标定位",
                        gripper_m=open_gripper,
                    ) is False:
                        raise RuntimeError(
                            "方块顶部抓取 MOVE_J 到标定位失败。"
                        )
                    _, top_down_xyz, top_down_rpy_deg = (
                        self.get_cached_end_pose(max_age_s=1.0)
                    )
                    actual_top_down_rpy = {
                        "roll": math.radians(
                            float(top_down_rpy_deg[0])
                        ),
                        "pitch": math.radians(
                            float(top_down_rpy_deg[1])
                        ),
                        "yaw": math.radians(
                            float(top_down_rpy_deg[2])
                        ),
                    }
                    for pose in (
                        approach,
                        grasp_open,
                        grasp_closed,
                        lift,
                    ):
                        pose.update(actual_top_down_rpy)
                    current_safe = {
                        **actual_top_down_rpy,
                        "x": float(top_down_xyz[0]),
                        "y": float(top_down_xyz[1]),
                        "z": float(top_down_xyz[2]),
                        "gripper": open_gripper,
                    }
                    self.get_logger().info(
                        "方块顶部垂直抓取："
                        "已 MOVE_J 到标定位并锁定实际顶部姿态，"
                        f"xyz={top_down_xyz.tolist()}, "
                        f"rpy_deg={top_down_rpy_deg.tolist()}"
                    )
                else:
                    top_down_rpy_rad = np.array(
                        [
                            float(approach["roll"]),
                            float(approach["pitch"]),
                            float(approach["yaw"]),
                        ],
                        dtype=np.float64,
                    )
                    current_safe_rpy_rad = np.array(
                        [
                            float(current_safe["roll"]),
                            float(current_safe["pitch"]),
                            float(current_safe["yaw"]),
                        ],
                        dtype=np.float64,
                    )
                    high_rpy_delta_deg = rotation_distance_deg(
                        rotation_from_rpy_xyz(*current_safe_rpy_rad),
                        rotation_from_rpy_xyz(*top_down_rpy_rad),
                    )
                    if high_rpy_delta_deg > 1.0:
                        top_down_safe = self.publish_rpy_only_path(
                            current_safe,
                            target_rpy=top_down_rpy_rad,
                            duration=max(
                                1.0,
                                OVERHEAD_MOVE_DURATION_S * 0.60,
                            ),
                            gripper=open_gripper,
                            label=(
                                "方块顶部抓取步骤2："
                                "在安全高度切换到垂直向下姿态"
                            ),
                        )
                        if top_down_safe is False:
                            raise RuntimeError(
                                "方块顶部抓取高处姿态切换失败。"
                            )
                        current_safe = top_down_safe
                    else:
                        current_safe = dict(current_safe)
                        current_safe["roll"] = float(approach["roll"])
                        current_safe["pitch"] = float(approach["pitch"])
                        current_safe["yaw"] = float(approach["yaw"])
                        self.get_logger().info(
                            "方块顶部抓取姿态与当前安全高度姿态基本一致，"
                            "跳过高处调姿: "
                            f"rpy_delta={high_rpy_delta_deg:.2f}deg"
                        )

                observation_approach = dict(approach)
                top_down_xy_delta = math.hypot(
                    float(observation_approach["x"])
                    - float(current_safe["x"]),
                    float(observation_approach["y"])
                    - float(current_safe["y"]),
                )
                if (
                    top_down_xy_delta
                    > BLOCK_TOP_DOWN_MAX_XY_MOVE_M
                ):
                    raise RuntimeError(
                        "方块顶部抓取目标超出当前顶部标定位的"
                        "安全水平可达范围，已拒绝发送高位 XY 平移: "
                        f"xy_delta={top_down_xy_delta:.3f}m > "
                        f"limit={BLOCK_TOP_DOWN_MAX_XY_MOVE_M:.3f}m, "
                        "请将方块移动到顶部标定位正下方附近，"
                        "或重新示教 WRIST_BLOCK_TOP_DOWN_JOINTS_RAD "
                        "到当前取物区域上方。"
                    )
                self.get_logger().info(
                    "方块顶部垂直抓取："
                    "姿态已在安全高度完成切换，"
                    "后续高处 XY 对准和下探均锁定顶部姿态。"
                )
            else:
                observation_approach = dict(approach)
                observation_approach["roll"] = float(
                    current_pose["roll"]
                )
                observation_approach["pitch"] = float(
                    current_pose["pitch"]
                )
                observation_approach["yaw"] = float(
                    current_pose["yaw"]
                )

            if block_lock_observation_rpy and not block_top_down_grasp:
                locked_rpy = {
                    "roll": float(observation_approach["roll"]),
                    "pitch": float(observation_approach["pitch"]),
                    "yaw": float(observation_approach["yaw"]),
                }
                for pose in (
                    approach,
                    grasp_open,
                    grasp_closed,
                    lift,
                ):
                    pose.update(locked_rpy)
                self.get_logger().info(
                    "方块抓取锁定观察位 RPY，"
                    "禁止靠近目标后的腕部调姿: "
                    f"rpy_rad={locked_rpy}"
                )

            # 步骤2：保持当前锁定姿态，只平移到目标正上方。
            xy_align_label = (
                "方块顶部抓取步骤3：锁定顶部姿态高位水平对准"
                if block_top_down_grasp
                else "步骤2：保持观察姿态高位水平对准"
            )
            overhead_safe = self.publish_xy_only_path(
                current_safe,
                target_x=float(observation_approach["x"]),
                target_y=float(observation_approach["y"]),
                duration=OVERHEAD_MOVE_DURATION_S,
                gripper=open_gripper,
                label=xy_align_label,
            )
            if overhead_safe is False:
                raise RuntimeError(
                    "高位水平对准失败。"
                )

            # 步骤3：锁定 X/Y/RPY，只下降到预抓取高度。
            pregrasp_descent_label = (
                "方块顶部抓取步骤4：锁定顶部姿态垂直到预抓高度"
                if block_top_down_grasp
                else "步骤3：保持观察姿态垂直下降到预抓取高度"
            )
            if (
                abs(
                    float(overhead_safe["z"])
                    - float(observation_approach["z"])
                )
                > 0.004
            ):
                if self.publish_strict_vertical_path(
                    overhead_safe,
                    target_z=float(observation_approach["z"]),
                    duration=APPROACH_DESCENT_DURATION_S,
                    gripper=open_gripper,
                    label=pregrasp_descent_label,
                ) is False:
                    raise RuntimeError(
                        "步骤3垂直到预抓高度失败。"
                    )

            if self.publish_pose_for(
                observation_approach,
                duration=OVERHEAD_DWELL_S,
            ) is False:
                raise RuntimeError(
                    "预抓观察姿态保持失败。"
                )

            # 步骤4：进入抓取阶段，允许在预抓取点原地调整到抓取姿态。
            if block_top_down_grasp:
                self.get_logger().info(
                    "方块顶部姿态已在安全高度完成，"
                    "跳过步骤4原地调姿。"
                )
                rotated_approach = dict(observation_approach)
            elif block_lock_observation_rpy:
                self.get_logger().info(
                    "方块抓取已锁定观察姿态，"
                    "跳过步骤4原地调姿，避免腕部扫碰目标。"
                )
                rotated_approach = dict(observation_approach)
            else:
                target_rpy_rad = np.deg2rad(
                    selected_grasp_rpy_deg
                )
                observation_rpy_rad = np.array(
                    [
                        float(observation_approach["roll"]),
                        float(observation_approach["pitch"]),
                        float(observation_approach["yaw"]),
                    ],
                    dtype=np.float64,
                )
                rpy_adjust_distance_deg = rotation_distance_deg(
                    rotation_from_rpy_xyz(*observation_rpy_rad),
                    rotation_from_rpy_xyz(*target_rpy_rad),
                )
                if rpy_adjust_distance_deg > 1.0:
                    rotated_approach = self.publish_rpy_only_path(
                        observation_approach,
                        target_rpy=target_rpy_rad,
                        duration=max(
                            1.0,
                            OVERHEAD_MOVE_DURATION_S * 0.60,
                        ),
                        gripper=open_gripper,
                        label=(
                            "步骤4：抓取阶段原地调整到抓取姿态"
                        ),
                    )
                    if rotated_approach is False:
                        raise RuntimeError(
                            "抓取阶段姿态调整失败。"
                        )
                else:
                    self.get_logger().info(
                        "抓取姿态与观察姿态基本一致，"
                        "跳过步骤4原地调姿: "
                        f"rpy_delta={rpy_adjust_distance_deg:.2f}deg"
                    )
                    rotated_approach = dict(observation_approach)

            if block_top_down_grasp:
                self.get_logger().info(
                    "方块顶部抓取跳过靠近目标后的额外姿态保持指令。"
                )
            elif block_lock_observation_rpy:
                self.get_logger().info(
                    "方块抓取跳过靠近目标后的额外姿态保持指令。"
                )
            else:
                if self.publish_pose_for(
                    approach,
                    duration=OVERHEAD_DWELL_S,
                ) is False:
                    raise RuntimeError(
                        "抓取姿态保持失败。"
                    )

            self.get_logger().info(
                "已到达预抓观察点，执行二次腕部视觉定位，"
                "用当前正对视角修正最终抓取点。"
            )
            (
                approach,
                grasp_open,
                grasp_closed,
                lift,
            ) = self.refine_grasp_with_second_localization(
                approach,
                grasp_open,
                grasp_closed,
                lift,
            )
            open_gripper = float(approach["gripper"])
            closed_gripper = float(grasp_closed["gripper"])

            if float(grasp_open["y"]) < MIN_FLANGE_Y_M:
                raise RuntimeError(
                    "二次定位后的抓取目标超出安全工作区，"
                    "已拒绝发送最终下探: "
                    f"flange_grasp_y={float(grasp_open['y']):.3f}m < "
                    f"limit={MIN_FLANGE_Y_M:.3f}m, "
                    f"flange_grasp_target="
                    f"[{float(grasp_open['x']):.6f}, "
                    f"{float(grasp_open['y']):.6f}, "
                    f"{float(grasp_open['z']):.6f}]"
                )

            if self.publish_strict_vertical_path(
                approach,
                target_z=float(grasp_open["z"]),
                duration=FINAL_DESCENT_DURATION_S,
                gripper=open_gripper,
                label=(
                    "严格垂直下探到夹取高度，夹爪保持张开"
                ),
            ) is False:
                raise RuntimeError(
                    "严格垂直下探到夹取高度失败。"
                )

            if self.publish_pose_for(
                grasp_open,
                duration=GRASP_PRE_CLOSE_DWELL_S,
            ) is False:
                raise RuntimeError(
                    "夹取高度保持失败。"
                )
            self.wait_until_grasp_pose_reached_before_close(grasp_open)

            if STOP_BEFORE_CLOSE:
                self.get_logger().warn(
                    "已停在闭合夹爪前："
                    "夹爪保持张开，未闭合，"
                    "机械臂保持使能。"
                )
                return

            close_started_at = time.monotonic()
            if self.publish_pose_for(
                grasp_closed,
                duration=GRASP_CLOSE_DWELL_S,
            ) is False:
                raise RuntimeError("夹爪闭合失败。")
            closed_gripper = self.resolve_post_close_hold_gripper(
                closed_gripper,
                close_started_at,
            )
            grasp_closed["gripper"] = closed_gripper
            self.activate_gripper_hold_for_current_target(closed_gripper)

            self.get_logger().info(
                "抓取完成，夹爪保持闭合并严格垂直抬升，"
                "不再返回旧机器的固定零位。"
            )
            if self.publish_strict_vertical_path(
                grasp_closed,
                target_z=float(lift["z"]),
                duration=max(0.8, FINAL_DESCENT_DURATION_S),
                gripper=closed_gripper,
                label="抓取完成后垂直抬升",
            ) is False:
                raise RuntimeError(
                    "抓取后垂直抬升失败。"
                )

            try:
                execute_place_after_grasp(
                    self,
                    R_CAM_TO_GRIPPER,
                    T_CAM_TO_GRIPPER,
                    PLACE_OBSERVATION_JOINTS_RAD,
                    solve_piper_ik_pose,
                )
            except Exception as place_exc:
                self.get_logger().error(
                    "抓取成功，但放置阶段失败；"
                    "不执行抓取失败恢复，避免误开夹爪: "
                    f"{place_exc}"
                )

            self.get_logger().info(
                "腕部相机抓取序列完成，"
                "机械臂保持使能。"
            )

        except Exception as exc:
            snapshot = self.snapshot_rgbd()
            if snapshot[0] is not None:
                failed_frame = snapshot[0].copy()
                failed_path = os.path.join(
                    self.output_dir,
                    (
                        "grasp_detection_failed_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                        ".png"
                    ),
                )
                cv2.imwrite(failed_path, failed_frame)
                self.get_logger().warn(
                    f"抓取失败时的腕部画面已保存: {failed_path}"
                )
            self.get_logger().error(
                f"腕部相机抓取失败: {exc}"
            )
            recovered_to_observation = (
                self.return_to_observation_after_grasp_failure()
            )
            if (
                recovered_to_observation
                and rclpy.ok()
            ):
                try:
                    self.wait_for_retry_grasp_enter_confirmation(exc)
                    retry_after_failure = True
                except Exception as retry_exc:
                    self.get_logger().warn(
                        "等待重新开始图纸识别/抓取确认失败，"
                        f"本次不自动重启流程: {retry_exc}"
                    )

        finally:
            self.grasp_running = False
            if retry_after_failure and rclpy.ok():
                self.execute_wrist_grasp(
                    skip_instruction_confirmation=False
                )

    def enable_arm(self, enable=True):
        if self.enable_client is None:
            return None

        if self.enable_request_pending:
            return None

        self.enable_request_pending = True

        request = Enable.Request()
        request.enable_request = bool(enable)

        future = self.enable_client.call_async(request)
        future.add_done_callback(self.enable_callback)
        return future

    def enable_callback(self, future):
        self.enable_request_pending = False

        try:
            response = future.result()
            self.enable_response_received = True
            self.arm_enabled = bool(
                response.enable_response
            )

            self.get_logger().info(
                f"机械臂使能服务返回: "
                f"{self.arm_enabled}"
            )

        except Exception as exc:
            self.enable_response_received = False
            self.arm_enabled = False
            self.get_logger().error(
                f"机械臂使能服务调用失败: {exc}"
            )

    # ================= 异步视觉推理线程 =================

    def start_vision_model_loading(self):
        if not VISION_AVAILABLE or self.vision_detector is None:
            self.vision_model_load_done = True
            self.vision_model_load_success = False
            return False

        with self.vision_model_lock:
            if (
                self.vision_model_loading
                or self.vision_model_load_done
                or self.vision_detector.is_loaded
            ):
                return True

            self.vision_model_loading = True

        Thread(
            target=self.vision_model_worker,
            daemon=True,
        ).start()
        return True

    def vision_model_worker(self):
        try:
            self.get_logger().info(
                "后台加载图纸 YOLO 和实物瓶子 YOLO，"
                "与机械臂使能并行进行..."
            )
            bottle_success = bool(
                self.vision_detector.load_models_simple()
            )
            instruction_success = False
            instruction_error = None
            if self.instruction_detector is not None:
                try:
                    instruction_success = bool(
                        self.instruction_detector.load_model()
                    )
                except Exception as exc:
                    instruction_error = str(exc)
                    self.get_logger().error(
                        f"图纸 YOLO 模型加载失败: {exc}"
                    )

            with self.vision_model_lock:
                self.vision_model_load_success = bottle_success
                self.vision_model_load_error = None
                self.vision_model_load_done = True
                self.vision_model_loading = False

            if bottle_success:
                self.get_logger().info(
                    "实物瓶子 YOLO 模型后台加载完成。"
                )
            else:
                self.get_logger().error(
                    "实物瓶子 YOLO 模型后台加载失败。"
                )

            if instruction_success:
                self.get_logger().info(
                    "图纸 YOLO 模型后台加载完成。"
                )
            elif self.instruction_detector is None:
                self.get_logger().error(
                    "图纸 YOLO 权重不可用；无类别参数的"
                    "自动流程将不会移动机械臂。"
                )
            elif instruction_error is None:
                self.get_logger().error(
                    "图纸 YOLO 模型后台加载失败。"
                )

        except Exception as exc:
            with self.vision_model_lock:
                self.vision_model_load_success = False
                self.vision_model_load_error = str(exc)
                self.vision_model_load_done = True
                self.vision_model_loading = False

            self.get_logger().error(
                f"视觉模型后台加载异常: {exc}"
            )

    def vision_inference_worker(self):
        while True:
            try:
                frame, target = self.vision_queue_in.get(
                    timeout=0.5
                )

                if (
                    self.vision_detector
                    and self.vision_detector.is_loaded
                ):
                    result_image = (
                        self.vision_detector
                        .process_single_frame_fast(
                            frame,
                            target,
                        )
                    )

                    if result_image is not None:
                        if self.vision_queue_out.full():
                            try:
                                self.vision_queue_out.get_nowait()
                            except queue.Empty:
                                pass

                        self.vision_queue_out.put(
                            result_image
                        )

            except queue.Empty:
                pass

            except Exception as exc:
                self.get_logger().error(
                    f"后台推理线程错误: {exc}"
                )

    def terminal_input_worker(self):
        if not sys.stdin or not sys.stdin.isatty():
            return

        if self.auto_grasp_requested or self.auto_preview_requested:
            if self.target_class_id is None:
                print(
                    "图纸自动选目标模式：无需在终端输入 "
                    "目标类别；请按提示把图纸放到相机前。"
                )
            else:
                print(
                    "已通过启动参数指定调试目标，无需再次输入类别。"
                )
            return

        print(
            "终端目标输入: 输入 green_bottle / orange_bottle / "
            "purple_bottle / yellow_block / blue_block / red_block "
            "后回车开始下一轮；"
            "输入 q 退出。"
        )

        self.terminal_input_active = True
        try:
            while rclpy.ok():
                try:
                    text = input("target> ").strip()
                except (EOFError, KeyboardInterrupt):
                    return

                if text.lower() in {"q", "quit", "exit"}:
                    rclpy.try_shutdown()
                    return

                with self.pre_grasp_enter_lock:
                    waiting_for_enter = self.pre_grasp_enter_pending

                if waiting_for_enter:
                    if not text:
                        self.confirm_pre_grasp_enter("terminal")
                    else:
                        with self.pre_grasp_enter_lock:
                            action_text = (
                                self.pre_grasp_enter_action_text
                                or "继续"
                            )
                        self.get_logger().warn(
                            "当前正在等待 Enter 确认。"
                            f"请直接按空回车以{action_text}；"
                            f"本次输入已忽略: {text}"
                        )
                    continue

                if not text:
                    continue

                try:
                    parsed = parse_target_spec(text)
                except Exception as exc:
                    self.get_logger().error(
                        f"终端目标解析失败: {exc}"
                    )
                    continue

                with self.terminal_target_lock:
                    self.pending_terminal_target = parsed

                self.get_logger().info(
                    "已收到终端新目标，空闲后开始下一轮: "
                    f"class={parsed['class_id']}, "
                    f"color={parsed['color'] or 'any'}, "
                    f"model_class="
                    f"{parsed.get('model_class_name') or 'generic'}"
                )
        finally:
            self.terminal_input_active = False

    def apply_pending_terminal_target(self):
        with self.terminal_target_lock:
            pending = self.pending_terminal_target
            self.pending_terminal_target = None

        if pending is None:
            return

        self.set_target_class(
            pending["class_id"],
            color=pending["color"],
            model_class_name=pending.get("model_class_name"),
        )
        self.auto_grasp_requested = True
        self.auto_preview_requested = False
        self.auto_action_done = False
        self.get_logger().info(
            "终端目标已生效，准备执行自动抓取。"
        )

    def confirm_pre_grasp_enter(self, source):
        with self.pre_grasp_enter_lock:
            if not self.pre_grasp_enter_pending:
                return False
            target_text = self.pre_grasp_enter_target_text
            action_text = self.pre_grasp_enter_action_text or "继续"
            self.pre_grasp_enter_event.set()

        self.get_logger().info(
            "已收到 Enter 确认，"
            f"{action_text}: "
            f"target={target_text}, source={source}"
        )
        return True

    def wait_for_enter_confirmation(
        self,
        prompt,
        action_text,
        target_text,
        input_prompt,
    ):
        self.get_logger().warn(prompt)

        if self.terminal_input_active:
            with self.pre_grasp_enter_lock:
                self.pre_grasp_enter_target_text = target_text
                self.pre_grasp_enter_action_text = action_text
                self.pre_grasp_enter_pending = True
                self.pre_grasp_enter_event.clear()

            print("等待 Enter 确认：如果看到 target> 提示，直接空回车。")
            try:
                while rclpy.ok():
                    if self.pre_grasp_enter_event.wait(0.10):
                        return
                raise RuntimeError(
                    "等待 Enter 确认时 ROS 已退出，已取消抓取。"
                )
            finally:
                with self.pre_grasp_enter_lock:
                    self.pre_grasp_enter_pending = False
                    self.pre_grasp_enter_target_text = None
                    self.pre_grasp_enter_action_text = None
                self.pre_grasp_enter_event.clear()

        if not sys.stdin or not sys.stdin.isatty():
            raise RuntimeError(
                f"需要 Enter 确认后才能{action_text}，"
                "但当前没有可交互终端。"
            )

        try:
            input(input_prompt)
        except (EOFError, KeyboardInterrupt) as exc:
            raise RuntimeError(
                f"等待 Enter 确认被中断，已取消{action_text}。"
            ) from exc

    def wait_for_instruction_detection_enter_confirmation(self):
        if not PRE_INSTRUCTION_ENTER_CONFIRM_ENABLED:
            return

        self.wait_for_enter_confirmation(
            "已准备进入图纸 YOLO 识别。"
            "请把图纸放到腕部相机前，确认画面稳定后按 Enter；"
            "按下前不会运行图纸 YOLO 推理。",
            "开始图纸 YOLO 识别",
            "instruction_sheet",
            "press Enter to start sheet YOLO> ",
        )

    def wait_for_pre_grasp_enter_confirmation(
        self,
        class_name,
        confidence,
    ):
        if not PRE_GRASP_ENTER_CONFIRM_ENABLED:
            return

        display_name = get_custom_target_display_name(class_name)
        target_text = f"{display_name} ({class_name})"
        self.wait_for_enter_confirmation(
            "图纸目标已识别为 "
            f"{target_text}, confidence={float(confidence):.4f}。"
            "请移走图纸、放好对应实物并确认工作区安全，"
            "然后按 Enter 开始实物识别和抓取。",
            "开始执行实物识别/抓取",
            target_text,
            "press Enter to grasp> ",
        )

    def ensure_instruction_target_confirmed_before_grasp(self):
        if not PRE_GRASP_INSTRUCTION_ENABLED:
            return None
        if not self.auto_instruction_target:
            return None

        self.wait_for_instruction_detection_enter_confirmation()
        candidate = self.select_target_from_instruction_sheet()
        class_name = candidate["class_name"]
        confidence = candidate["confidence"]
        display_name = get_custom_target_display_name(class_name)
        self.get_logger().warn(
            "抓取前图纸识别结果: "
            f"{display_name} ({class_name}), "
            f"dataset_class={candidate.get('dataset_class_id')}, "
            f"confidence={float(confidence):.4f}"
        )
        self.wait_for_pre_grasp_enter_confirmation(class_name, confidence)
        return candidate

    def select_target_from_instruction_sheet(self):
        """Use the first YOLO model to choose the object shown on paper."""
        if self.instruction_detector is None:
            raise RuntimeError(
                "图纸 YOLO 权重不可用: "
                f"{INSTRUCTION_YOLO_MODEL_PATH}"
            )
        if not self.instruction_detector.is_loaded:
            raise RuntimeError(
                "图纸 YOLO 模型尚未加载完成。"
            )

        required_hits = max(1, INSTRUCTION_CONFIRM_FRAMES)
        deadline = time.monotonic() + max(
            1.0,
            INSTRUCTION_TIMEOUT_S,
        )
        stable_class_name = None
        stable_hits = 0
        best_confirmed = None

        self.get_logger().warn(
            "阶段1/2：请将只包含一个目标物体图案的图纸"
            "放到腕部相机前；此阶段不会发送机械臂运动命令。"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            if self.color_image is None:
                time.sleep(0.10)
                continue

            result = self.instruction_detector.detect(
                self.color_image.copy()
            )
            if result["overlay"] is not None:
                self.last_instruction_preview = result["overlay"]

            candidates = result["candidates"]
            accepted = [
                candidate
                for candidate in candidates
                if candidate["confidence"]
                >= INSTRUCTION_YOLO_ACCEPT_CONFIDENCE
            ]
            accepted_names = {
                candidate["class_name"]
                for candidate in accepted
            }

            if not accepted:
                stable_class_name = None
                stable_hits = 0
                if candidates:
                    top = candidates[0]
                    self.get_logger().info(
                        "图纸 YOLO 本次检测: "
                        f"class={top['class_name']}, "
                        f"confidence={top['confidence']:.4f} "
                        f"({top['confidence'] * 100.0:.2f}%), "
                        "threshold="
                        f"{INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:.3f}, "
                        "accepted=False"
                    )
                else:
                    self.get_logger().info(
                        "图纸 YOLO 本次检测: 无候选框，"
                        "model_threshold="
                        f"{INSTRUCTION_YOLO_RAW_CONFIDENCE:.3f}"
                    )
                time.sleep(0.15)
                continue

            if len(accepted_names) != 1:
                stable_class_name = None
                stable_hits = 0
                candidate_text = ", ".join(
                    (
                        f"{candidate['class_name']}:"
                        f"{candidate['confidence']:.3f}"
                    )
                    for candidate in accepted
                )
                self.get_logger().warn(
                    "图纸 YOLO 同时识别到多个目标类别，"
                    f"本帧不确认: [{candidate_text}]"
                )
                time.sleep(0.15)
                continue

            class_name = next(iter(accepted_names))
            best = max(
                (
                    candidate
                    for candidate in accepted
                    if candidate["class_name"] == class_name
                ),
                key=lambda candidate: candidate["confidence"],
            )
            if class_name == stable_class_name:
                stable_hits += 1
            else:
                stable_class_name = class_name
                stable_hits = 1
            best_confirmed = best

            self.get_logger().info(
                "图纸 YOLO 本次检测: "
                f"class={class_name}, "
                f"confidence={best['confidence']:.4f} "
                f"({best['confidence'] * 100.0:.2f}%), "
                "threshold="
                f"{INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:.3f}, "
                "accepted=True, "
                f"confirm={stable_hits}/{required_hits}"
            )

            if stable_hits >= required_hits:
                spec = CUSTOM_YOLO_GRASP_CLASSES[class_name]
                best_confirmed["dataset_class_id"] = int(
                    spec["dataset_class_id"]
                )
                best_confirmed["display_name_zh"] = (
                    get_custom_target_display_name(class_name)
                )
                overlay = result["overlay"].copy()
                cv2.putText(
                    overlay,
                    (
                        f"CONFIRMED: {class_name} "
                        f"{best['confidence']:.3f}"
                    ),
                    (10, 54),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.70,
                    (0, 255, 0),
                    2,
                )
                preview_path = os.path.join(
                    self.output_dir,
                    (
                        "instruction_sheet_target_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                        ".png"
                    ),
                )
                cv2.imwrite(preview_path, overlay)
                self.last_instruction_preview = overlay
                self.last_instruction_preview_path = preview_path

                self.set_target_class(
                    int(spec["grasp_class_id"]),
                    color=str(spec["color"]),
                    model_class_name=class_name,
                    source=(
                        "图纸YOLO "
                        f"confidence={best['confidence']:.4f}"
                    ),
                )
                self.get_logger().info(
                    "图纸目标确认完成: "
                    f"display={best_confirmed['display_name_zh']}, "
                    f"dataset_class={spec['dataset_class_id']}, "
                    f"model_class={class_name}, "
                    f"confidence={best['confidence']:.4f} "
                    f"({best['confidence'] * 100.0:.2f}%), "
                    f"preview={preview_path}"
                )
                return best_confirmed

            time.sleep(0.15)

        raise RuntimeError(
            "图纸目标识别超时：没有连续 "
            f"{required_hits} 帧得到唯一且置信度不低于 "
            f"{INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:.2f} 的目标类别。"
        )

    def run_auto_sequence_worker(self):
        try:
            if self.target_class_id is None:
                if not self.auto_instruction_target:
                    raise RuntimeError(
                        "未指定目标类别，且图纸自动选目标已关闭。"
                    )
                self.get_logger().warn(
                    "机械臂将先移动到观察位。运动期间请勿将"
                    "图纸或手伸入工作区；到达后程序会提示放图纸。"
                )

            if self.auto_zero_observe:
                self.get_logger().info(
                    "自动流程：移动到零位，夹爪最大打开。"
                )
                if self.publish_pose_for(
                    self.home_position,
                    duration=self.auto_zero_duration_s,
                ) is False:
                    return
            else:
                self.get_logger().warn(
                    "已跳过会触发关节超限的旧零位。"
                )

            if self.auto_move_observe:
                self.get_logger().info(
                    "自动流程：通过示教关节角移动到观察位。"
                )
                if self.move_to_observation_joint_pose() is False:
                    return
            else:
                self.get_logger().warn(
                    "已按 --skip-observe 跳过观察位，"
                    "将从当前腕部相机位姿识别。"
                )

            self.get_logger().info(
                "自动流程：等待画面稳定后识别。"
            )
            time.sleep(max(0.0, self.auto_action_delay_s))

            if self.auto_preview_requested and not self.auto_grasp_requested:
                self.get_logger().info(
                    f"自动预览目标: class={self.target_class_id}, "
                    f"color={self.target_color or 'any'}, "
                    f"model_class="
                    f"{self.target_model_class_name or 'generic'}, "
                    f"prompt={self.detection_target}"
                )
                self.preview_wrist_object_detection()
                return

            self.get_logger().warn(
                f"自动抓取目标: class={self.target_class_id}, "
                f"color={self.target_color or 'any'}, "
                f"model_class="
                f"{self.target_model_class_name or 'generic'}, "
                f"prompt={self.detection_target}。"
                "请确认工作空间内没有人员或障碍物。"
            )
            self.execute_wrist_grasp()

        except Exception as exc:
            self.get_logger().error(
                f"自动流程失败: {exc}"
            )

        finally:
            self.auto_sequence_running = False

    def run_startup_observe_worker(self):
        try:
            self.get_logger().info(
                "启动后自动进入 home 观察关节位。"
            )
            if self.move_to_observation_joint_pose() is False:
                return
            self.get_logger().info(
                "启动 home 观察位已到达。"
            )
        except Exception as exc:
            self.get_logger().error(
                f"启动 home 观察位失败: {exc}"
            )
        finally:
            self.startup_observe_running = False

    def maybe_move_startup_observe(self):
        if not self.startup_move_observe:
            return
        if self.startup_observe_done or self.startup_observe_running:
            return
        if self.auto_grasp_requested or self.auto_preview_requested:
            return
        if self.auto_sequence_running or self.grasp_running:
            return
        if self.state != PiperState.IDLE or not self.arm_enabled:
            return

        self.startup_observe_done = True
        self.startup_observe_running = True
        Thread(
            target=self.run_startup_observe_worker,
            daemon=True,
        ).start()

    def maybe_run_auto_action(self):
        if self.auto_sequence_running or self.grasp_running:
            return

        self.apply_pending_terminal_target()

        if self.auto_action_done:
            return

        if not (self.auto_grasp_requested or self.auto_preview_requested):
            return

        if self.state != PiperState.IDLE:
            return

        if self.grasp_running:
            return

        requires_instruction_target = (
            PRE_GRASP_INSTRUCTION_ENABLED
            and self.auto_instruction_target
        )
        if self.target_class_id is None or requires_instruction_target:
            if not self.auto_instruction_target:
                self.get_logger().error(
                    "未指定目标类别，且图纸自动识别已关闭。"
                )
                self.auto_action_done = True
                return
            if self.instruction_detector is None:
                self.get_logger().error(
                    "未找到图纸 YOLO 权重，已禁止机械臂运动: "
                    f"{INSTRUCTION_YOLO_MODEL_PATH}"
                )
                self.auto_action_done = True
                return
            if not self.instruction_detector.is_loaded:
                if self.vision_model_load_done:
                    self.get_logger().error(
                        "图纸 YOLO 模型加载失败，已禁止机械臂运动。"
                    )
                    self.auto_action_done = True
                return

        if not self.arm_enabled:
            return

        if (
            self.color_image is None
            or self.depth_image is None
            or self.camera_matrix is None
        ):
            return

        if (
            self.vision_detector is None
            or not self.vision_detector.is_loaded
        ):
            return

        self.auto_action_done = True
        self.auto_sequence_running = True
        Thread(
            target=self.run_auto_sequence_worker,
            daemon=True,
        ).start()


    def main_update_loop(self):
        """30 Hz 主循环，统一处理状态机和 UI 更新。"""
        self.state_machine()
        self.maybe_move_startup_observe()
        self.maybe_run_auto_action()
        self.update_vision_and_ui()

    def update_vision_and_ui(self):
        """处理视觉队列、相机显示和窗口键盘事件。"""

        # 启动一段时间后仍未收到图像，打印明确诊断。
        camera_wait_time = (
            time.time() - self.camera_start_time
        )

        if (
            self.color_image is None
            and not self.camera_timeout_logged
            and camera_wait_time > 8.0
        ):
            self.camera_timeout_logged = True

            color_publisher = self.topic_has_publishers(
                COLOR_TOPIC
            )
            depth_publisher = self.topic_has_publishers(
                DEPTH_TOPIC
            )
            info_publisher = self.topic_has_publishers(
                CAMERA_INFO_TOPIC
            )

            self.get_logger().error(
                "启动超过 8 秒仍未收到腕部相机 RGB 图像。"
                f"发布者状态: color={color_publisher}, "
                f"depth={depth_publisher}, "
                f"info={info_publisher}; "
                f"目标话题={COLOR_TOPIC}"
            )

        # 1. DINO 推理队列
        if (
            self.detection_active
            and self.color_image is not None
        ):
            if not self.vision_queue_in.full():
                self.vision_queue_in.put(
                    (
                        self.color_image.copy(),
                        self.detection_target,
                    )
                )

        try:
            self.last_detection_result = (
                self.vision_queue_out.get_nowait()
            )
        except queue.Empty:
            pass

        # 2. 准备主显示画面
        if self.color_image is not None:
            display_image = self.color_image.copy()

            cv2.putText(
                display_image,
                f"Status: {self.state.name}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            arm_status = (
                "Enabled"
                if self.arm_enabled
                else "Disabled"
            )

            cv2.putText(
                display_image,
                f"Arm: {arm_status}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (
                    (0, 255, 0)
                    if self.arm_enabled
                    else (0, 0, 255)
                ),
                2,
            )

            cv2.putText(
                display_image,
                f"Topic: {COLOR_TOPIC}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                1,
            )

            target_text = (
                "Target: "
                f"{self.target_model_class_name or self.detection_target}"
                if self.target_class_id is not None
                else f"Target: {self.detection_target}"
            )
            cv2.putText(
                display_image,
                target_text,
                (10, 118),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

        else:
            display_image = np.zeros(
                (480, 640, 3),
                dtype=np.uint8,
            )

            cv2.putText(
                display_image,
                "Waiting for left RealSense...",
                (55, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (0, 255, 255),
                2,
            )

            serial_text = (
                CAMERA_SERIAL_RAW
                if CAMERA_SERIAL_RAW
                else "auto"
            )

            cv2.putText(
                display_image,
                f"Serial: {serial_text}",
                (55, 255),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )

            cv2.putText(
                display_image,
                f"Status: {self.state.name}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                display_image,
                (
                    "h:home t:target r:qr 1-2:target "
                    "o:preview g:grasp q:quit"
                ),
                (30, 300),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )

        # 3. 显示窗口
        if (
            not self.ui_initialized
            and self.ui_available
        ):
            self.initialize_ui()

        if self.ui_initialized:
            try:
                cv2.imshow(
                    "Camera View",
                    display_image,
                )

                if (
                    self.last_detection_result
                    is not None
                    and self.detection_active
                ):
                    cv2.imshow(
                        "Detection Result",
                        self.last_detection_result,
                    )

                if self.last_wrist_preview is not None:
                    cv2.imshow(
                        "Wrist Object Preview",
                        self.last_wrist_preview,
                    )

                if self.last_instruction_preview is not None:
                    cv2.imshow(
                        "Instruction Sheet Target",
                        self.last_instruction_preview,
                    )

            except cv2.error as exc:
                self.ui_initialized = False

                if not self.ui_error_logged:
                    self.ui_error_logged = True
                    self.get_logger().error(
                        f"OpenCV 显示窗口失败: {exc}。"
                        "请检查 DISPLAY，并确认没有安装 "
                        "opencv-python-headless 覆盖普通 OpenCV。"
                    )

        # GUI 不可用时，每隔约 3 秒保存一张图。
        elif self.color_image is not None:
            now = time.time()

            if (
                now - self.last_camera_save_time
                > 3.0
            ):
                self.last_camera_save_time = now
                fallback_path = os.path.join(
                    self.output_dir,
                    "latest_camera_frame.png",
                )
                cv2.imwrite(
                    fallback_path,
                    self.color_image,
                )

        # 4. 窗口按键
        key = -1

        if self.ui_initialized:
            try:
                key = cv2.waitKey(1) & 0xFF
            except cv2.error:
                key = -1

        if key in {10, 13}:
            if self.confirm_pre_grasp_enter("opencv_window"):
                return

        if (
            key == ord("h")
            and self.arm_enabled
        ):
            if self.auto_sequence_running or self.grasp_running:
                self.get_logger().warn(
                    "当前已有运动/抓取任务，忽略 h 回 home。"
                )
            else:
                Thread(
                    target=self.move_to_observation_joint_pose,
                    daemon=True,
                ).start()

        elif (
            key == ord("t")
            and self.arm_enabled
        ):
            self.get_logger().warn(
                "t 对应的旧观察位已禁用，未发送运动指令。"
            )

        elif key == ord("r"):
            self.update_qr_target_from_current_frame()

        elif key in [ord(str(index)) for index in range(1, 3)]:
            self.set_target_class(key - ord("1"))

        elif key == ord("o"):
            now = time.time()

            if (
                now - self.last_preview_trigger_time
                >= self.preview_key_debounce_s
            ):
                self.last_preview_trigger_time = now
                self.preview_wrist_object_detection()

        elif key == ord("g"):
            self.execute_wrist_grasp()

        elif key == ord("d"):
            if not VISION_AVAILABLE:
                self.get_logger().warn(
                    "GroundingDINO 路径不可用；"
                    "请按 o 使用腕部 RGBD 识别。"
                )
                return

            if (
                self.vision_detector is None
                or not self.vision_detector.is_loaded
            ):
                self.get_logger().warn(
                    "GroundingDINO 模型尚未加载成功；"
                    "请查看启动日志。"
                )
                return

            self.detection_active = (
                not self.detection_active
            )

            self.get_logger().info(
                f"检测状态切换: "
                f"{self.detection_active}"
            )

            if not self.detection_active:
                try:
                    cv2.destroyWindow(
                        "Detection Result"
                    )
                except cv2.error:
                    pass

        elif key == ord("c"):
            if not VISION_AVAILABLE:
                self.get_logger().warn(
                    "GroundingDINO 路径不可用，"
                    "无法修改文本目标。"
                )
                return

            new_target = input(
                "\n请输入新目标: "
            ).strip()

            if new_target:
                self.target_class_id = None
                self.target_color = None
                self.target_model_class_name = None
                self.target_dataset_class_id = None
                self.target_qr_code = None
                self.detection_target = new_target
                self.detection_history.clear()

                if self.vision_detector:
                    self.vision_detector.set_detection_target(
                        new_target
                    )

                self.get_logger().info(
                    f"DINO 检测目标已手动修改为: "
                    f"{new_target}"
                )

        elif key == ord("k"):
            self.skip_arm_control = True

            if self.state in [
                PiperState.INITIALIZE_CAN,
                PiperState.WAIT_FOR_CAN,
                PiperState.WAIT_FOR_SERVICE,
            ]:
                self.transition_to_state(
                    PiperState.SKIP_ARM
                )

        elif key == ord("q"):
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

            if rclpy.ok():
                rclpy.try_shutdown()

    def state_machine(self):
        with self.state_lock:
            current_state = self.state

        time_in_state = (
            time.time() - self.state_entry_time
        )

        if current_state == PiperState.INITIALIZE_CAN:
            can_thread = Thread(
                target=self.can_setup_worker,
                daemon=True,
            )
            can_thread.start()
            self.transition_to_state(
                PiperState.WAIT_FOR_CAN
            )

        elif current_state == PiperState.WAIT_FOR_CAN:
            if self.skip_arm_control:
                self.transition_to_state(
                    PiperState.SKIP_ARM
                )

            elif self.can_setup_done:
                if self.can_setup_success:
                    self.transition_to_state(
                        PiperState.INITIALIZE_ARM
                    )
                else:
                    self.get_logger().warn(
                        "CAN 配置失败，已自动跳过"
                        "机械臂控制（仅启用视觉）。"
                    )
                    self.skip_arm_control = True
                    self.transition_to_state(
                        PiperState.SKIP_ARM
                    )

        elif current_state == PiperState.INITIALIZE_ARM:
            if self.start_piper_node():
                self.transition_to_state(
                    PiperState.WAIT_FOR_SERVICE
                )

        elif current_state == PiperState.WAIT_FOR_SERVICE:
            if self.skip_arm_control:
                self.transition_to_state(
                    PiperState.SKIP_ARM
                )

            elif (
                self.enable_client
                and self.enable_client.service_is_ready()
            ):
                self.transition_to_state(
                    PiperState.ENABLE_ARM
                )

        elif current_state == PiperState.ENABLE_ARM:
            if (
                not self.enable_response_received
                and not self.enable_request_pending
            ):
                self.enable_arm(True)

            elif self.arm_enabled:
                self.transition_to_state(
                    PiperState.IDLE
                )

        elif current_state == PiperState.LOAD_VISION_MODELS:
            if (
                VISION_AVAILABLE
                and self.vision_detector
            ):
                self.start_vision_model_loading()

            self.transition_to_state(
                PiperState.IDLE
            )

        elif current_state == PiperState.SKIP_ARM:
            self.start_vision_model_loading()

            self.transition_to_state(
                PiperState.IDLE
            )

        elif current_state in [
            PiperState.MOVE_TO_HOME,
            PiperState.MOVE_TO_TARGET,
        ]:
            if self.arm_enabled:
                if (
                    current_state
                    == PiperState.MOVE_TO_HOME
                ):
                    position = self.home_position
                else:
                    position = self.target_position

                self.send_pos_command(position)

                if time_in_state > 2.0:
                    self.transition_to_state(
                        PiperState.IDLE
                    )
            else:
                self.transition_to_state(
                    PiperState.IDLE
                )

    def cleanup(self):
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

        processes = [
            ("Piper", self.node_process),
            ("RealSense", self.camera_process),
        ]

        for process_name, process in processes:
            if process is None:
                continue

            if process.poll() is not None:
                continue

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGTERM,
                )
                process.wait(timeout=3.0)

            except subprocess.TimeoutExpired:
                try:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGKILL,
                    )
                except Exception:
                    process.kill()

            except ProcessLookupError:
                pass

            except Exception as exc:
                self.get_logger().warn(
                    f"结束 {process_name} "
                    f"子进程失败: {exc}"
                )


def main(args=None):
    raw_args = list(sys.argv[1:] if args is None else args)
    try:
        runtime_options, ros_args = parse_runtime_args(raw_args)
    except Exception as exc:
        raise SystemExit(f"启动参数错误: {exc}") from exc

    rclpy.init(args=[sys.argv[0], *ros_args])
    controller = PiperController(runtime_options)

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info(
            "收到 Ctrl+C，正在安全退出。"
        )

    finally:
        controller.cleanup()
        controller.destroy_node()

        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
