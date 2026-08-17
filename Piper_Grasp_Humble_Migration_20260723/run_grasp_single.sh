#!/usr/bin/env bash
set -eo pipefail
set +u

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PIPER_WS="${PIPER_WS:-$PROJECT_DIR/drivers/piper_ros}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -z "${PIPER_SETUP:-}" ]]; then
    PIPER_SETUP="$PIPER_WS/install/setup.bash"
fi

cd "$PROJECT_DIR"

exec 9>/tmp/piper_grasp_single.lock
if ! flock -n 9; then
    echo "另一个 grasp_single 任务正在运行；请先 Ctrl+C 停掉旧任务，或等待其结束。"
    exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup not found: $ROS_SETUP"
    exit 1
fi

if [[ ! -f "$PIPER_SETUP" ]]; then
    echo "Piper ROS workspace has not been built: $PIPER_SETUP"
    echo "Run: \"$PROJECT_DIR/build_piper_humble.sh\""
    exit 1
fi

# This program uses the piper_ros PosCmd/Enable API.  Remove an inherited
# agx_arm_ws overlay so both drivers cannot accidentally share the same CAN
# interface in one process environment.
remove_agx_overlay_entries() {
    local variable_name="$1"
    local original_value="${!variable_name:-}"
    local filtered_value=""
    local entry

    IFS=':' read -r -a entries <<< "$original_value"
    for entry in "${entries[@]}"; do
        [[ -z "$entry" ]] && continue
        [[ "$entry" == *"/agx_arm_ws/"* ]] && continue
        [[ "$entry" == */agx_arm_ws/install ]] && continue
        if [[ -z "$filtered_value" ]]; then
            filtered_value="$entry"
        else
            filtered_value="$filtered_value:$entry"
        fi
    done

    printf -v "$variable_name" '%s' "$filtered_value"
    export "$variable_name"
}

for path_variable in \
    AMENT_PREFIX_PATH \
    COLCON_PREFIX_PATH \
    CMAKE_PREFIX_PATH \
    LD_LIBRARY_PATH \
    PYTHONPATH
do
    remove_agx_overlay_entries "$path_variable"
done

source "$ROS_SETUP"
source "$PIPER_SETUP"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "This project requires ROS 2 Humble, but ROS_DISTRO=${ROS_DISTRO:-unset}."
    exit 1
fi

# setup 文件可能把启动终端继承的 AGX overlay 再次加入环境；
# source 完成后再过滤一次，确保只加载 Piper 消息和驱动。
for path_variable in \
    AMENT_PREFIX_PATH \
    COLCON_PREFIX_PATH \
    CMAKE_PREFIX_PATH \
    LD_LIBRARY_PATH \
    PYTHONPATH
do
    remove_agx_overlay_entries "$path_variable"
done

# piper_sdk is normally installed with pip. A source checkout can be selected
# explicitly without hard-coding a path from another machine.
if [[ -n "${PIPER_SDK_ROOT:-}" ]]; then
    if [[ ! -d "$PIPER_SDK_ROOT/piper_sdk" ]]; then
        echo "Piper SDK source not found: $PIPER_SDK_ROOT/piper_sdk"
        exit 1
    fi
    export PYTHONPATH="$PIPER_SDK_ROOT${PYTHONPATH:+:$PYTHONPATH}"
fi

# CPU is the conservative default for the mixed ROS/Conda process.  Set this
# to 0 explicitly after the complete CUDA inference path has been validated.
export WRIST_YOLO_DEVICE="${WRIST_YOLO_DEVICE:-cpu}"
export WRIST_YOLO_CONFIDENCE="${WRIST_YOLO_CONFIDENCE:-0.10}"
export WRIST_MOVING_DETECTION_CONFIDENCE="${WRIST_MOVING_DETECTION_CONFIDENCE:-0.50}"
export WRIST_STILL_DETECTION_CONFIDENCE="${WRIST_STILL_DETECTION_CONFIDENCE:-0.50}"
export WRIST_LOCALIZATION_DETECTION_CONFIDENCE="${WRIST_LOCALIZATION_DETECTION_CONFIDENCE:-0.50}"
export WRIST_INSTRUCTION_YOLO_RAW_CONFIDENCE="${WRIST_INSTRUCTION_YOLO_RAW_CONFIDENCE:-0.10}"
export WRIST_INSTRUCTION_YOLO_ACCEPT_CONFIDENCE="${WRIST_INSTRUCTION_YOLO_ACCEPT_CONFIDENCE:-0.50}"
export WRIST_INSTRUCTION_CONFIRM_FRAMES="${WRIST_INSTRUCTION_CONFIRM_FRAMES:-3}"
export WRIST_INSTRUCTION_TIMEOUT_S="${WRIST_INSTRUCTION_TIMEOUT_S:-30}"
export WRIST_INSTRUCTION_CLEAR_DELAY_S="${WRIST_INSTRUCTION_CLEAR_DELAY_S:-3}"
export WRIST_AUTO_INSTRUCTION_TARGET="${WRIST_AUTO_INSTRUCTION_TARGET:-1}"
export WRIST_PRE_INSTRUCTION_ENTER_CONFIRM="${WRIST_PRE_INSTRUCTION_ENTER_CONFIRM:-1}"
export WRIST_PRE_GRASP_ENTER_CONFIRM="${WRIST_PRE_GRASP_ENTER_CONFIRM:-1}"
export WRIST_RETRY_GRASP_ON_TARGET_FAILURE="${WRIST_RETRY_GRASP_ON_TARGET_FAILURE:-1}"
# 裸 bash run_grasp_single.sh 就按比赛抓取完整流程执行：
# 观察位 -> Enter 开始图纸 YOLO -> 识别结果 -> Enter 开始抓取。
# 找不到实物或实物超出安全工作区时，回观察位后再按 Enter 二次识别/抓取。
# 如需只启动界面/手动按键调试，可临时设置 WRIST_AUTO_GRASP=0。
export WRIST_AUTO_GRASP="${WRIST_AUTO_GRASP:-1}"
export WRIST_USE_SYSTEM_OPENCV="${WRIST_USE_SYSTEM_OPENCV:-1}"
export YOLO_OFFLINE="${YOLO_OFFLINE:-true}"
export YOLO_AUTOINSTALL="${YOLO_AUTOINSTALL:-false}"
export CAN_NAME="${CAN_NAME:-can2}"
export USE_CALIBRATED_INTRINSICS="${USE_CALIBRATED_INTRINSICS:-1}"
# The old zero pose triggers joint-limit protection on this arm.  Skip that
# pose, but retain the calibrated observation pose before detection.
export WRIST_AUTO_ZERO_OBSERVE="${WRIST_AUTO_ZERO_OBSERVE:-0}"
export WRIST_AUTO_MOVE_OBSERVE="${WRIST_AUTO_MOVE_OBSERVE:-1}"
export WRIST_OBSERVATION_JOINTS_RAD="${WRIST_OBSERVATION_JOINTS_RAD:--1.544267,0.593639,-0.717610,-0.072065,0.786428,0.000000}"
# 放置阶段识别物体牌子的独立观察位；不要复用抓取观察位。
export WRIST_PLACE_OBSERVATION_JOINTS_RAD="${WRIST_PLACE_OBSERVATION_JOINTS_RAD:--1.556258,0.853972,-1.132928,-0.137270,0.703263,0.112521}"
# 中心观察位找不到目标时，用 J1 以示教位为中心左右各扫描 20°。
export WRIST_OBSERVATION_SCAN_ENABLED="${WRIST_OBSERVATION_SCAN_ENABLED:-1}"
export WRIST_OBSERVATION_SCAN_MODE="${WRIST_OBSERVATION_SCAN_MODE:-joint1}"
export WRIST_OBSERVATION_SCAN_OFFSETS_DEG="${WRIST_OBSERVATION_SCAN_OFFSETS_DEG:-20,-20}"
# 找到直立瓶后，先保持 J2~J6 不变，只转 J1 让夹爪正前方对准瓶子。
export WRIST_BOTTLE_BASE_AIM_ENABLED="${WRIST_BOTTLE_BASE_AIM_ENABLED:-1}"
# 抓取几何偏移/补偿默认值；如需现场微调，可在外部显式 export 覆盖。
export WRIST_GRASP_RIGHT_BIAS_M="${WRIST_GRASP_RIGHT_BIAS_M:-0.010}"
export WRIST_GRASP_LEFT_COMPENSATION_M="${WRIST_GRASP_LEFT_COMPENSATION_M:-0.050}"
export WRIST_BOTTLE_SIDE_APPROACH_BACKOFF_M="${WRIST_BOTTLE_SIDE_APPROACH_BACKOFF_M:-0.030}"
export WRIST_BOTTLE_GRASP_LEFT_SHIFT_M="${WRIST_BOTTLE_GRASP_LEFT_SHIFT_M:-0.035}"
export WRIST_BOTTLE_FORWARD_EXTRA_M="${WRIST_BOTTLE_FORWARD_EXTRA_M:-0.040}"
export WRIST_BOTTLE_HEIGHT_OFFSET_M="${WRIST_BOTTLE_HEIGHT_OFFSET_M:-0.020}"
export WRIST_GRASP_CENTER_EXTRA_Z_M="${WRIST_GRASP_CENTER_EXTRA_Z_M:--0.020}"
export WRIST_GRASP_EXTRA_DESCENT_M="${WRIST_GRASP_EXTRA_DESCENT_M:-0.025}"
export WRIST_BLOCK_FORWARD_EXTRA_M="${WRIST_BLOCK_FORWARD_EXTRA_M:-0.020}"
export WRIST_BLOCK_HEIGHT_OFFSET_M="${WRIST_BLOCK_HEIGHT_OFFSET_M:-0.010}"
export WRIST_BLOCK_APPROACH_HEIGHT_M="${WRIST_BLOCK_APPROACH_HEIGHT_M:-0.120}"
export WRIST_GRASP_MIN_FLANGE_Y_M="${WRIST_GRASP_MIN_FLANGE_Y_M:--0.550}"
export WRIST_BLOCK_TOP_DOWN_GRASP="${WRIST_BLOCK_TOP_DOWN_GRASP:-1}"
export WRIST_BLOCK_TOP_DOWN_REQUIRE_CALIBRATED_RPY="${WRIST_BLOCK_TOP_DOWN_REQUIRE_CALIBRATED_RPY:-1}"
export WRIST_BLOCK_TOP_DOWN_RPY_DEG="${WRIST_BLOCK_TOP_DOWN_RPY_DEG:--179.449,25.000,90.183}"
export WRIST_BLOCK_TOP_DOWN_USE_MOVE_J="${WRIST_BLOCK_TOP_DOWN_USE_MOVE_J:-1}"
export WRIST_BLOCK_TOP_DOWN_JOINTS_RAD="${WRIST_BLOCK_TOP_DOWN_JOINTS_RAD:--1.598023,1.633942,-1.521962,0.000000,1.100000,-0.028868}"
export WRIST_BLOCK_TOP_DOWN_MAX_XY_MOVE_M="${WRIST_BLOCK_TOP_DOWN_MAX_XY_MOVE_M:-0.210}"
export WRIST_BLOCK_TOP_DOWN_MIN_FLANGE_Y_M="${WRIST_BLOCK_TOP_DOWN_MIN_FLANGE_Y_M:--0.380}"
export WRIST_BLOCK_KEEP_OBSERVATION_RPY="${WRIST_BLOCK_KEEP_OBSERVATION_RPY:-0}"
export WRIST_BLOCK_OUTER_KEEP_CURRENT_ROLL_PITCH="${WRIST_BLOCK_OUTER_KEEP_CURRENT_ROLL_PITCH:-0}"
export WRIST_BLOCK_OUTER_RPY_Y_M="${WRIST_BLOCK_OUTER_RPY_Y_M:--0.380}"
export WRIST_SECOND_LOCALIZATION_REQUIRED="${WRIST_SECOND_LOCALIZATION_REQUIRED:-1}"
export WRIST_SECOND_LOCALIZATION_MAX_XY_M="${WRIST_SECOND_LOCALIZATION_MAX_XY_M:-0.080}"
export WRIST_SECOND_LOCALIZATION_MAX_Z_M="${WRIST_SECOND_LOCALIZATION_MAX_Z_M:-0.050}"
export WRIST_PLACE_AFTER_GRASP_ENABLED="${WRIST_PLACE_AFTER_GRASP_ENABLED:-1}"
export WRIST_PLACE_FORWARD_FROM_SHEET_M="${WRIST_PLACE_FORWARD_FROM_SHEET_M:-0.300}"
export WRIST_PLACE_LEFT_FROM_SHEET_M="${WRIST_PLACE_LEFT_FROM_SHEET_M:-0.000}"
export WRIST_PLACE_SAFE_Z_M="${WRIST_PLACE_SAFE_Z_M:-0.330}"
export WRIST_PLACE_RELEASE_ABOVE_SHEET_Z_M="${WRIST_PLACE_RELEASE_ABOVE_SHEET_Z_M:-0.110}"
export WRIST_PLACE_EXTRA_DESCENT_M="${WRIST_PLACE_EXTRA_DESCENT_M:-0.100}"
export WRIST_PLACE_MIN_RELEASE_Z_M="${WRIST_PLACE_MIN_RELEASE_Z_M:-0.060}"
export WRIST_PLACE_MAX_RELEASE_Z_M="${WRIST_PLACE_MAX_RELEASE_Z_M:-0.360}"
export WRIST_PLACE_DEPTH_MAX_M="${WRIST_PLACE_DEPTH_MAX_M:-2.250}"
export WRIST_PLACE_PRE_DETECT_ENTER_ENABLED="${WRIST_PLACE_PRE_DETECT_ENTER_ENABLED:-1}"
export WRIST_PLACE_MOVE_OBSERVE_BEFORE_DETECT="${WRIST_PLACE_MOVE_OBSERVE_BEFORE_DETECT:-1}"
export WRIST_PLACE_USE_MOVE_J_FOR_XY="${WRIST_PLACE_USE_MOVE_J_FOR_XY:-1}"
export WRIST_PLACE_MAX_JOINT_TRAVEL_RAD="${WRIST_PLACE_MAX_JOINT_TRAVEL_RAD:-2.250}"
export WRIST_PLACE_SCAN_ENABLED="${WRIST_PLACE_SCAN_ENABLED:-1}"
export WRIST_PLACE_SCAN_OFFSETS_DEG="${WRIST_PLACE_SCAN_OFFSETS_DEG:-10,-10}"
export WRIST_PLACE_SCAN_DETECT_TIMEOUT_S="${WRIST_PLACE_SCAN_DETECT_TIMEOUT_S:-3.0}"
export WRIST_PLACE_RETURN_CENTER_BEFORE_PLACE="${WRIST_PLACE_RETURN_CENTER_BEFORE_PLACE:-1}"
export WRIST_PLACE_RETURN_OBSERVATION_AFTER_SUCCESS="${WRIST_PLACE_RETURN_OBSERVATION_AFTER_SUCCESS:-1}"
export WRIST_GRASP_MAX_YAW_DELTA_DEG="${WRIST_GRASP_MAX_YAW_DELTA_DEG:-45.0}"
export WRIST_GRASP_PRE_CLOSE_Z_TOL_M="${WRIST_GRASP_PRE_CLOSE_Z_TOL_M:-0.018}"
# 前下方 MOVE_J 中间允许最多 18° 工具姿态偏离；末端仍回到对准姿态。
export WRIST_BOTTLE_FORWARD_DOWN_MAX_RPY_DEVIATION_DEG="${WRIST_BOTTLE_FORWARD_DOWN_MAX_RPY_DEVIATION_DEG:-18}"
# 只放宽瓶子“对准后向前下方伸展”段；其他 MOVE_J 仍使用 2.0rad。
export WRIST_BOTTLE_FORWARD_DOWN_MAX_JOINT_TRAVEL_RAD="${WRIST_BOTTLE_FORWARD_DOWN_MAX_JOINT_TRAVEL_RAD:-2.40}"
export WRIST_BOTTLE_LOCKED_PATH_MAX_JOINT_STEP_RAD="${WRIST_BOTTLE_LOCKED_PATH_MAX_JOINT_STEP_RAD:-0.45}"
# 透明/反光瓶的深度点可能只剩桌面薄片；这种结果改为左右扫描重试。
export WRIST_BOTTLE_MIN_ESTIMATED_HEIGHT_M="${WRIST_BOTTLE_MIN_ESTIMATED_HEIGHT_M:-0.040}"
export WRIST_BOTTLE_MIN_CLUSTER_POINTS="${WRIST_BOTTLE_MIN_CLUSTER_POINTS:-300}"
# 轨迹外形指标只记录，不因轻微横向弯曲、上拱或姿态变化中止。
export WRIST_BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS="${WRIST_BOTTLE_FORWARD_DOWN_ENFORCE_SHAPE_LIMITS:-0}"

export LEFT_CAMERA_USB_PORT="${LEFT_CAMERA_USB_PORT:-2-3.3.2}"

list_realsense_serials() {
    if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
        return 0
    fi

    {
        rs-enumerate-devices -s 2>/dev/null \
            | awk '/Intel RealSense/ {print $(NF-1)}' \
            || true
        rs-enumerate-devices 2>/dev/null \
            | awk '
                /^[[:space:]]*Serial Number[[:space:]]*:/ {
                    sub(/.*Serial Number[[:space:]]*:[[:space:]]*/, "", $0)
                    gsub(/[[:space:]]+/, "", $0)
                    print $0
                }
            ' \
            || true
    } | awk 'NF && !seen[$0]++'
}

detect_realsense_serial_by_usb_port() {
    local target_port="$1"
    if [[ -z "$target_port" ]] || ! command -v rs-enumerate-devices >/dev/null 2>&1; then
        return 0
    fi

    rs-enumerate-devices 2>/dev/null \
        | awk -v target="$target_port" '
            /^[[:space:]]*Serial Number[[:space:]]*:/ {
                serial = $0
                sub(/.*Serial Number[[:space:]]*:[[:space:]]*/, "", serial)
                gsub(/[[:space:]]+/, "", serial)
            }
            /Physical Port[[:space:]]*:/ {
                if (index($0, target) > 0 && serial != "") {
                    print serial
                    exit
                }
            }
        ' \
        || true
}

if [[ -z "${LEFT_CAMERA_SERIAL:-}" ]] && command -v rs-enumerate-devices >/dev/null 2>&1; then
    detected_serial="$(detect_realsense_serial_by_usb_port "$LEFT_CAMERA_USB_PORT")"
    if [[ -z "$detected_serial" ]]; then
        detected_serial="$(list_realsense_serials | awk 'NF {print; exit}')"
    fi
    if [[ -n "$detected_serial" ]]; then
        export LEFT_CAMERA_SERIAL="$detected_serial"
    fi
fi
export LEFT_CAMERA_SERIAL="${LEFT_CAMERA_SERIAL:-151222079131}"

if command -v rs-enumerate-devices >/dev/null 2>&1; then
    if ! list_realsense_serials | grep -Fxq "$LEFT_CAMERA_SERIAL"; then
        echo "Requested wrist RealSense serial is not present: $LEFT_CAMERA_SERIAL"
        echo "Set LEFT_CAMERA_SERIAL to the current wrist camera serial, or set LEFT_CAMERA_USB_PORT."
        echo "Detected RealSense serials:"
        list_realsense_serials | sed 's/^/  /'
        exit 1
    fi
fi

if [[ -z "${WRIST_YOLO_MODEL_PATH:-}" ]]; then
    for model_candidate in \
        "$PROJECT_DIR/yolo bottle with qr/armdetect all/runs/detect/runs/train/waterbottle_6cls-2/weights/best.pt" \
        "$PROJECT_DIR/yolo bottle with qr/runs/detect/runs/train/waterbottle-9/weights/best.pt" \
        "$PROJECT_DIR/yolo bottle with qr/runs/train/waterbottle3/weights/best.pt" \
        "$PROJECT_DIR/yolov8_assets/yolov8n.pt" \
        "$PROJECT_DIR/../yolov8n-seg.pt" \
        "$PROJECT_DIR/../piper_grasp/yolov8n-seg.pt"
    do
        if [[ -f "$model_candidate" ]]; then
            export WRIST_YOLO_MODEL_PATH="$model_candidate"
            break
        fi
    done
fi

if [[ -z "${WRIST_YOLO_MODEL_PATH:-}" ]]; then
    echo "YOLO model not found. Set WRIST_YOLO_MODEL_PATH."
    exit 1
fi

if [[ -z "${WRIST_INSTRUCTION_YOLO_MODEL_PATH:-}" ]]; then
    for instruction_model_candidate in \
        "$PROJECT_DIR/yolo bottle with qr/armdetect all/runs/detect/runs/train/waterbottle_6cls-2/weights/best.pt" \
        "$PROJECT_DIR/yolo bottle with qr/armdetect all/weights_first.pt" \
        "$PROJECT_DIR/yolo bottle with qr/weights_first.pt"
    do
        if [[ -f "$instruction_model_candidate" ]]; then
            export WRIST_INSTRUCTION_YOLO_MODEL_PATH="$instruction_model_candidate"
            break
        fi
    done
fi

if [[ -z "${WRIST_INSTRUCTION_YOLO_MODEL_PATH:-}" ]]; then
    echo "Instruction-sheet YOLO model not found."
    echo "Expected: $PROJECT_DIR/yolo bottle with qr/armdetect all/runs/detect/runs/train/waterbottle_6cls-2/weights/best.pt"
    exit 1
fi

setup_can_before_launch() {
    local can_name
    can_name="${CAN_NAME:-${PIPER_CAN_NAME:-can_left}}"

    if [[ "$can_name" == "can_left" && ! -e /sys/class/net/can_left && -e /sys/class/net/can0 ]]; then
        can_name="can0"
        export CAN_NAME="can0"
    fi

    if [[ ! -e "/sys/class/net/$can_name" ]]; then
        echo "CAN interface not found: $can_name"
        return 1
    fi

    if ip -br link show "$can_name" | awk '{print $2}' | grep -qw UP; then
        echo "CAN already UP: $can_name"
        return 0
    fi

    echo "Configuring CAN interface: $can_name"
    sudo -v
    sudo modprobe can || true
    sudo modprobe can_raw || true
    sudo modprobe can_dev || true
    sudo ip link set "$can_name" down || true
    sudo ip link set "$can_name" type can bitrate 1000000
    sudo ip link set "$can_name" txqueuelen 1000
    sudo ip link set "$can_name" up
    ip -details link show "$can_name"
}

choose_conda_env() {
    if [[ -n "${PIPER_CONDA_ENV:-}" ]]; then
        echo "$PIPER_CONDA_ENV"
    else
        echo ""
    fi
}

run_python() {
    local env_name
    local python_bin
    env_name="$(choose_conda_env)"

    if [[ -n "$env_name" ]]; then
        if ! command -v conda >/dev/null 2>&1; then
            echo "PIPER_CONDA_ENV is set, but conda is not available."
            return 1
        fi
        echo "Using conda env: $env_name"
        conda run --no-capture-output -n "$env_name" python "$@"
    else
        python_bin="${PIPER_PYTHON:-/usr/bin/python3}"
        if [[ ! -x "$python_bin" ]]; then
            echo "Python interpreter not found: $python_bin"
            return 1
        fi
        echo "Using Humble Python: $python_bin"
        "$python_bin" "$@"
    fi
}

if [[ "${1:-}" == "--check-import-only" ]]; then
    run_python -c "import grasp_single, open3d, piper_sdk; from piper_msgs.msg import PosCmd, PiperStatusMsg; from piper_msgs.srv import Enable; print(f\"startup import ok; open3d={open3d.__version__}; opencv={grasp_single.cv2.__version__}; cv2_path={grasp_single.cv2.__file__}\")"
    exit 0
fi

setup_can_before_launch

wrist_camera_topics_ready() {
    local topic
    local info
    for topic in \
        /left_wrist_camera/camera/color/image_raw \
        /left_wrist_camera/camera/aligned_depth_to_color/image_raw \
        /left_wrist_camera/camera/color/camera_info
    do
        info="$(timeout 2s ros2 topic info "$topic" 2>/dev/null || true)"
        if ! grep -Eq 'Publisher count:[[:space:]]*[1-9]' <<< "$info"; then
            return 1
        fi
    done
    return 0
}

cleanup_stale_processes() {
    if [[ "${WRIST_CLEAN_STALE_PIPER:-1}" != "1" ]]; then
        return 0
    fi

    local patterns=(
        "$PROJECT_DIR/grasp_single.py"
        "$PIPER_WS/install/piper/lib/piper/piper_single_ctrl"
        'ros2 run piper piper_single_ctrl'
    )

    if wrist_camera_topics_ready; then
        echo "Healthy wrist RGBD camera is already online; preserving it for reuse."
    else
        patterns+=(
            'ros2 launch realsense2_camera'
            '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node'
        )
    fi

    local pids=""
    local pattern
    for pattern in "${patterns[@]}"; do
        found=$(pgrep -u "$USER" -f "$pattern" || true)
        if [[ -n "$found" ]]; then
            pids="$pids $found"
        fi
    done

    pids=$(echo "$pids" | tr ' ' '
' | awk 'NF && !seen[$0]++')
    if [[ -z "$pids" ]]; then
        return 0
    fi

    echo "Cleaning stale piper/camera/grasp processes:"
    echo "$pids"
    kill $pids || true
    sleep 1.0

    local left=""
    for pattern in "${patterns[@]}"; do
        found=$(pgrep -u "$USER" -f "$pattern" || true)
        if [[ -n "$found" ]]; then
            left="$left $found"
        fi
    done
    left=$(echo "$left" | tr ' ' '
' | awk 'NF && !seen[$0]++')
    if [[ -n "$left" ]]; then
        echo "Force cleaning stale processes:"
        echo "$left"
        kill -9 $left || true
        sleep 0.5
    fi
}

cleanup_stale_processes

# Python cannot run its normal cleanup after a native SIGFPE.  Keep the shell
# alive long enough to terminate any RealSense/Piper children it orphaned.
trap cleanup_stale_processes EXIT

# ROS 2 Humble on Ubuntu 22.04 uses Python 3.10. PIPER_CONDA_ENV is supported
# only when that environment also uses Python 3.10 and imports Humble packages.
run_python "$PROJECT_DIR/grasp_single.py" "$@"
