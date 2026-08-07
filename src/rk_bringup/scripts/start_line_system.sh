#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
RUNTIME_DIR="${RK_LINE_RUNTIME_DIR:-$HOME/rk_line_runtime}"
LOG_DIR="${RK_LINE_LOG_DIR:-$HOME/rk_line_logs}"
PID_FILE="${RUNTIME_DIR}/pids"

SDK_SERVER="${RK_SDK_SERVER:-}"
START_SDK_SERVER="${RK_START_SDK_SERVER:-true}"
START_ECONOMIC_GAIT="${RK_START_ECONOMIC_GAIT:-true}"
SDK_INTERFACE="${RK_SDK_INTERFACE:-eth0}"
SDK_UDP_HOST="${RK_SDK_UDP_HOST:-127.0.0.1}"
SDK_UDP_PORT="${RK_SDK_UDP_PORT:-15001}"

ENABLE_DEPTH="${RK_REALSENSE_ENABLE_DEPTH:-false}"
RGB_PROFILE="${RK_REALSENSE_RGB_PROFILE:-424x240x15}"
DEPTH_PROFILE="${RK_REALSENSE_DEPTH_PROFILE:-424x240x15}"
IMAGE_TOPIC="${RK_IMAGE_TOPIC:-/camera/color/image_raw}"

LINE_DEBUG_IMAGE="${RK_LINE_DEBUG_IMAGE:-true}"
LINE_DEBUG_LOG="${RK_LINE_DEBUG_LOG:-true}"
LINE_MIN_SPEED="${RK_LINE_MIN_SPEED:-0.27}"
LINE_BASE_SPEED="${RK_LINE_BASE_SPEED:-0.27}"
LINE_MID_SPEED="${RK_LINE_MID_SPEED:-0.27}"
LINE_SLOW_SPEED="${RK_LINE_SLOW_SPEED:-0.27}"
SHORT_LOST_LINEAR_SPEED="${RK_SHORT_LOST_LINEAR_SPEED:-0.0}"
SEARCH_LINEAR_SPEED="${RK_SEARCH_LINEAR_SPEED:-0.0}"
BRIDGE_MAX_LINEAR_X="${RK_GO2_BRIDGE_MAX_LINEAR_X:-$LINE_BASE_SPEED}"
BRIDGE_MAX_ANGULAR_Z="${RK_GO2_BRIDGE_MAX_ANGULAR_Z:-1.30}"

# 冷启动门禁的数值只能取自现场记录，绝不以固定 sleep 替代控制面证据。
ROBOT_IP="${RK_ROBOT_IP:-192.168.123.161}"
CONTROL_PLANE_NETWORK_TIMEOUT_SEC="${RK_CONTROL_PLANE_NETWORK_TIMEOUT_SEC:-}"
CONTROL_PLANE_PING_COUNT="${RK_CONTROL_PLANE_PING_COUNT:-}"
CONTROL_PLANE_PING_POLL_SEC="${RK_CONTROL_PLANE_PING_POLL_SEC:-}"
CONTROL_PLANE_DDS_TIMEOUT_SEC="${RK_CONTROL_PLANE_DDS_TIMEOUT_SEC:-}"
CONTROL_PLANE_REQUIRED_FRAMES="${RK_CONTROL_PLANE_REQUIRED_FRAMES:-}"
CONTROL_PLANE_MAX_FRAME_GAP_MS="${RK_CONTROL_PLANE_MAX_FRAME_GAP_MS:-}"
SDK_LISTEN_TIMEOUT_SEC="${RK_SDK_LISTEN_TIMEOUT_SEC:-10}"

select_ros_setup() {
    local distro

    if [ -n "${ROS_DISTRO:-}" ]; then
        local active_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
        if [ -f "$active_setup" ]; then
            printf "%s\n" "$active_setup"
            return 0
        fi
    fi

    for distro in foxy humble; do
        local setup_file="/opt/ros/${distro}/setup.bash"
        if [ -f "$setup_file" ]; then
            printf "%s\n" "$setup_file"
            return 0
        fi
    done

    echo "ERROR: no supported ROS2 setup.bash found under /opt/ros." >&2
    echo "Checked active ROS_DISTRO, then foxy, then humble." >&2
    return 1
}

source_base_ros() {
    local ros_setup
    ros_setup="$(select_ros_setup)"
    source "$ros_setup"
}

source_workspace_if_present() {
    if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
        source "${WORKSPACE_DIR}/install/setup.bash"
    fi
}

workspace_packages_ready() {
    ros2 pkg prefix rk_bringup >/dev/null 2>&1 \
        && ros2 pkg prefix rk_perception >/dev/null 2>&1 \
        && ros2 pkg prefix rk_navigation >/dev/null 2>&1 \
        && ros2 pkg prefix rk_mission >/dev/null 2>&1 \
        && ros2 pkg prefix rk_safety >/dev/null 2>&1 \
        && ros2 pkg prefix rk_go2_sdk_bridge >/dev/null 2>&1 \
        && ros2 pkg prefix rk_tools >/dev/null 2>&1
}

run_clean_colcon_build() {
    local ros_setup
    ros_setup="$(select_ros_setup)"

    env \
        -u AMENT_PREFIX_PATH \
        -u CMAKE_PREFIX_PATH \
        -u COLCON_PREFIX_PATH \
        bash -lc \
        "source \"${ros_setup}\" && cd \"${WORKSPACE_DIR}\" && colcon build --symlink-install --packages-up-to rk_bringup"
}

build_workspace_if_needed() {
    source_base_ros
    source_workspace_if_present

    if workspace_packages_ready; then
        return 0
    fi

    echo "ROS packages are not ready in install/. Building required packages..."
    run_clean_colcon_build
}

resolve_env_script() {
    local source_script="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
    local install_script="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/ros_clean_env.sh"

    if [ -f "$source_script" ]; then
        printf "%s\n" "$source_script"
        return 0
    fi

    if [ -f "$install_script" ]; then
        printf "%s\n" "$install_script"
        return 0
    fi

    echo "ERROR: ros_clean_env.sh not found in source or install tree." >&2
    echo "Checked:" >&2
    echo "  $source_script" >&2
    echo "  $install_script" >&2
    return 1
}

resolve_line_config() {
    local source_config="${WORKSPACE_DIR}/src/rk_bringup/config/line_nav_params.yaml"
    local install_config="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/config/line_nav_params.yaml"

    if [ -f "$source_config" ]; then
        printf "%s\n" "$source_config"
        return 0
    fi

    if [ -f "$install_config" ]; then
        printf "%s\n" "$install_config"
        return 0
    fi

    echo "ERROR: line_nav_params.yaml not found in source or install tree." >&2
    echo "Checked:" >&2
    echo "  $source_config" >&2
    echo "  $install_config" >&2
    return 1
}

stop_existing_processes() {
    set +e
    if [ -f "$PID_FILE" ]; then
        while IFS='|' read -r name pid log_file; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                echo "Stopping old ${name} pid=${pid}"
                kill "$pid" 2>/dev/null
            fi
        done < "$PID_FILE"
        sleep 1
        while IFS='|' read -r name pid log_file; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                echo "Force stopping old ${name} pid=${pid}"
                kill -9 "$pid" 2>/dev/null
            fi
        done < "$PID_FILE"
    fi

    pkill -f "competition_line_nav.launch.py" || true
    pkill -f "go2_sdk_udp_server" || true
    pkill -f "cmd_vel_udp_forwarder.py" || true
    pkill -f "command_mux_node" || true
    pkill -f "real_line_tracker_node" || true
    pkill -f "line_follower_node" || true
    pkill -f "line_course_mission_node" || true
    pkill -f "realsense2_camera_node" || true
    pkill -f "realsense2_camera" || true
    set -e
}

start_background() {
    local name="$1"
    local command="$2"
    local log_file="${LOG_DIR}/${name}.log"

    echo "Starting ${name}; log=${log_file}"
    nohup bash -lc "source \"${ENV_SCRIPT}\" && ${command}" \
        > "$log_file" 2>&1 < /dev/null &
    local pid="$!"
    echo "${name}|${pid}|${log_file}" >> "$PID_FILE"

    sleep 0.5
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: ${name} exited during startup. Last log lines:" >&2
        tail -n 40 "$log_file" >&2 || true
        exit 1
    fi
}

run_step() {
    local name="$1"
    local command="$2"
    local log_file="${LOG_DIR}/${name}.log"

    echo "Running ${name}; log=${log_file}"
    if ! bash -lc "source \"${ENV_SCRIPT}\" && ${command}" \
        > "$log_file" 2>&1; then
        echo "WARN: ${name} failed. Last log lines:" >&2
        tail -n 40 "$log_file" >&2 || true
        return 1
    fi
}

topic_has_publisher() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Publisher count: [1-9][0-9]*"
}

topic_has_subscription() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Subscription count: [1-9][0-9]*"
}

topic_has_single_publisher() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Publisher count: 1$"
}

wait_for_topic_publisher() {
    local topic_name="$1"
    local timeout_sec="$2"
    local start_time
    start_time="$(date +%s)"

    while true; do
        if topic_has_publisher "$topic_name"; then
            echo "Topic ready: ${topic_name}"
            return 0
        fi

        if [ $(( $(date +%s) - start_time )) -ge "$timeout_sec" ]; then
            echo "WARN: timeout waiting for publisher on ${topic_name}" >&2
            return 1
        fi
        sleep 1
    done
}

wait_for_topic_subscription() {
    local topic_name="$1"
    local timeout_sec="$2"
    local start_time
    start_time="$(date +%s)"

    while true; do
        if topic_has_subscription "$topic_name"; then
            echo "Topic subscriber ready: ${topic_name}"
            return 0
        fi

        if [ $(( $(date +%s) - start_time )) -ge "$timeout_sec" ]; then
            echo "ERROR: timeout waiting for subscriber on ${topic_name}" >&2
            return 1
        fi
        sleep 1
    done
}

ENV_SCRIPT="$(resolve_env_script)"
LINE_CONFIG="$(resolve_line_config)"

mkdir -p "$RUNTIME_DIR" "$LOG_DIR"
rm -f "$PID_FILE"
touch "$PID_FILE"

build_workspace_if_needed
source "$ENV_SCRIPT"

SDK_BRIDGE_PREFIX="$(ros2 pkg prefix rk_go2_sdk_bridge)"
SDK_RUNTIME_WRAPPER="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py"
CONTROL_PLANE_GATE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_control_plane_gate.py"
CONTROL_PLANE_PROBE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_sport_state_monitor"
if [ -n "$SDK_SERVER" ]; then
    SDK_SERVER_BINARY="$SDK_SERVER"
else
    SDK_SERVER_BINARY="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_udp_server"
fi
for required_file in "$SDK_RUNTIME_WRAPPER" "$CONTROL_PLANE_GATE" \
    "$CONTROL_PLANE_PROBE" "$SDK_SERVER_BINARY"; do
    if [ ! -x "$required_file" ]; then
        echo "ERROR: required staged-start executable is missing: ${required_file}" >&2
        exit 1
    fi
done

if [ "$START_SDK_SERVER" = "true" ]; then
    for measured_value in \
        "$CONTROL_PLANE_NETWORK_TIMEOUT_SEC" "$CONTROL_PLANE_PING_COUNT" \
        "$CONTROL_PLANE_PING_POLL_SEC" "$CONTROL_PLANE_DDS_TIMEOUT_SEC" \
        "$CONTROL_PLANE_REQUIRED_FRAMES" "$CONTROL_PLANE_MAX_FRAME_GAP_MS"; do
        if [ -z "$measured_value" ]; then
            echo "ERROR: cold-start control-plane thresholds are not configured." >&2
            echo "Set RK_CONTROL_PLANE_{NETWORK_TIMEOUT_SEC,PING_COUNT,PING_POLL_SEC,DDS_TIMEOUT_SEC,REQUIRED_FRAMES,MAX_FRAME_GAP_MS} from a recorded cold-boot measurement." >&2
            exit 1
        fi
    done
fi

echo "Stopping old RK line system processes..."
stop_existing_processes
rm -f "$PID_FILE"
touch "$PID_FILE"

if [ "$START_SDK_SERVER" = "true" ]; then
    # 阶段 A：网络连续稳定 + 只读 SportModeState；失败时尚未启动 UDP 输入口。
    run_step "control_plane_gate" \
        "exec \"${CONTROL_PLANE_GATE}\" --interface \"${SDK_INTERFACE}\" --robot-ip \"${ROBOT_IP}\" --runtime-wrapper \"${SDK_RUNTIME_WRAPPER}\" --probe \"${CONTROL_PLANE_PROBE}\" --network-timeout-sec \"${CONTROL_PLANE_NETWORK_TIMEOUT_SEC}\" --ping-count \"${CONTROL_PLANE_PING_COUNT}\" --ping-poll-sec \"${CONTROL_PLANE_PING_POLL_SEC}\" --dds-timeout-sec \"${CONTROL_PLANE_DDS_TIMEOUT_SEC}\" --required-frames \"${CONTROL_PLANE_REQUIRED_FRAMES}\" --max-frame-gap-ms \"${CONTROL_PLANE_MAX_FRAME_GAP_MS}\""
    # 阶段 B：只有 startup StopMove 成功并打印 listening 后，才允许相机和 ROS 图启动。
    start_background "sdk_server" \
        "exec \"${SDK_RUNTIME_WRAPPER}\" \"${SDK_SERVER_BINARY}\" --interface \"${SDK_INTERFACE}\" --listen-ip \"${SDK_UDP_HOST}\" --port \"${SDK_UDP_PORT}\""
    sdk_listen_deadline=$(( $(date +%s) + SDK_LISTEN_TIMEOUT_SEC ))
    while [ "$(date +%s)" -lt "$sdk_listen_deadline" ]; do
        if grep -Fq "UDP server listening on" "${LOG_DIR}/sdk_server.log"; then
            break
        fi
        if ! kill -0 "$(awk -F'|' '$1 == \"sdk_server\" { print $2; exit }' "$PID_FILE")" 2>/dev/null; then
            echo "ERROR: SDK_STARTUP_DIAG classification=SDK_RUNTIME_LIBRARY_ERROR" >&2
            tail -n 80 "${LOG_DIR}/sdk_server.log" >&2 || true
            exit 1
        fi
        sleep 0.1
    done
    if ! grep -Fq "UDP server listening on" "${LOG_DIR}/sdk_server.log"; then
        echo "ERROR: SDK_STARTUP_DIAG classification=ROBOT_CONTROL_PLANE_NOT_READY" >&2
        tail -n 80 "${LOG_DIR}/sdk_server.log" >&2 || true
        exit 1
    fi
    echo "CONTROL_PLANE_DIAG event=SDK_UDP_LISTENING"
fi

if [ "$START_ECONOMIC_GAIT" = "true" ]; then
    run_step "economic_gait" \
        "ros2 run rk_go2_sdk_bridge go2_sdk_motion_action ${SDK_INTERFACE} economic_gait 1.0" \
        || true
fi

start_background "cmd_vel_udp_forwarder" \
    "exec ros2 run rk_go2_sdk_bridge cmd_vel_udp_forwarder.py --ros-args -p cmd_vel_topic:=/navigation/cmd_vel -p udp_host:=${SDK_UDP_HOST} -p udp_port:=${SDK_UDP_PORT} -p max_vx:=${BRIDGE_MAX_LINEAR_X} -p max_yaw:=${BRIDGE_MAX_ANGULAR_Z}"

wait_for_topic_subscription "/navigation/cmd_vel" 10 || {
    echo "ERROR: /navigation/cmd_vel has no subscriber." >&2
    echo "cmd_vel_udp_forwarder is not connected, so the robot will not move." >&2
    tail -n 40 "${LOG_DIR}/cmd_vel_udp_forwarder.log" >&2 || true
    exit 1
}

start_background "command_mux" \
    "exec ros2 run rk_safety command_mux_node --ros-args -p mission_cmd_topic:=/control/mission_cmd -p estop_topic:=/safety/estop -p enable_estop_service:=true -p estop_service_name:=/safety/estop -p output_cmd_topic:=/navigation/cmd_vel"

start_background "realsense_camera" \
    "exec ros2 launch rk_bringup realsense_low_bandwidth.launch.py enable_color:=true enable_depth:=${ENABLE_DEPTH} rgb_camera.profile:=${RGB_PROFILE} depth_module.profile:=${DEPTH_PROFILE}"

wait_for_topic_publisher "/camera/color/image_raw" 15 || true

start_background "real_line_tracker" \
    "exec ros2 run rk_perception real_line_tracker_node --ros-args --params-file \"${LINE_CONFIG}\" -p image_topic:=${IMAGE_TOPIC} -p enable_debug_image:=${LINE_DEBUG_IMAGE} -p debug_log:=${LINE_DEBUG_LOG}"

start_background "line_follower" \
    "exec ros2 run rk_navigation line_follower_node --ros-args --params-file \"${LINE_CONFIG}\" -p debug_log:=${LINE_DEBUG_LOG} -p min_driving_speed:=${LINE_MIN_SPEED} -p base_speed:=${LINE_BASE_SPEED} -p mid_speed:=${LINE_MID_SPEED} -p slow_speed:=${LINE_SLOW_SPEED} -p short_lost_linear_speed:=${SHORT_LOST_LINEAR_SPEED} -p search_linear_speed:=${SEARCH_LINEAR_SPEED}"

start_background "line_course_mission" \
    "exec ros2 run rk_mission line_course_mission_node --ros-args --params-file \"${LINE_CONFIG}\" -p cmd_vel_topic:=/control/mission_cmd -p sdk_network_interface:=${SDK_INTERFACE}"

wait_for_topic_publisher "/perception/line_track" 15 || true
wait_for_topic_publisher "/navigation/line_follow_cmd_suggested" 10 || true
wait_for_topic_publisher "/control/mission_cmd" 10 || true
wait_for_topic_publisher "/navigation/cmd_vel" 10 || true
if ! topic_has_single_publisher "/navigation/cmd_vel"; then
    echo "ERROR: /navigation/cmd_vel must have exactly one publisher." >&2
    ros2 topic info -v "/navigation/cmd_vel" >&2 || true
    exit 1
fi
wait_for_topic_subscription "/navigation/cmd_vel" 5 || {
    echo "ERROR: /navigation/cmd_vel subscriber disappeared after startup." >&2
    tail -n 40 "${LOG_DIR}/cmd_vel_udp_forwarder.log" >&2 || true
    exit 1
}

cat <<EOF
RK line system started in background.
Runtime dir: ${RUNTIME_DIR}
Logs dir: ${LOG_DIR}
PID file: ${PID_FILE}

Started processes:
$(cat "$PID_FILE")

Check:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/check_line_system.sh
View image debug:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/view_line_debug.sh
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/stream_line_debug_web.sh
Logs:
  tail -f ${LOG_DIR}/real_line_tracker.log
  tail -f ${LOG_DIR}/line_follower.log
  tail -f ${LOG_DIR}/line_course_mission.log
  tail -f ${LOG_DIR}/command_mux.log
  tail -f ${LOG_DIR}/cmd_vel_udp_forwarder.log

Motion is NOT started yet.
Start line following:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/mission_start.sh
Stop everything:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/stop_line_system.sh
EOF
