#!/bin/bash
# 正式非机械臂比赛链一键启动；本脚本永不自动发布 /mission/start。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
RUNTIME_DIR="${RK_COMPETITION_RUNTIME_DIR:-$HOME/rk_non_arm_competition_runtime}"
LOG_DIR="${RK_COMPETITION_LOG_DIR:-$HOME/rk_non_arm_competition_logs}"
SESSION="${RK_COMPETITION_TMUX_SESSION:-rk_non_arm_competition}"
HARDWARE_MODE="${RK_COMPETITION_HARDWARE_MODE:-true}"
SOFTWARE_SMOKE_MODE="${RK_COMPETITION_SOFTWARE_SMOKE_MODE:-false}"
START_LINE_CAMERA="${RK_COMPETITION_START_LINE_CAMERA:-true}"
START_SDK_SERVER="${RK_COMPETITION_START_SDK_SERVER:-true}"
START_UDP_FORWARDER="${RK_COMPETITION_START_UDP_FORWARDER:-true}"
ENABLE_DEBUG_IMAGE="${RK_COMPETITION_ENABLE_DEBUG_IMAGE:-false}"
SDK_NETWORK_INTERFACE="${RK_COMPETITION_SDK_NETWORK_INTERFACE:-eth1}"
# 正式 Go2 控制网段的本机固定地址；本脚本只验证，不修改任何网络配置。
SDK_NETWORK_ADDRESS_CIDR="192.168.123.18/24"
LINE_IMAGE_TOPIC="${RK_COMPETITION_LINE_IMAGE_TOPIC:-/line_camera/image_raw}"
LINE_CAMERA_DEVICE="${RK_COMPETITION_LINE_CAMERA_DEVICE:-/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0}"
LINE_CAMERA_WIDTH="${RK_COMPETITION_LINE_CAMERA_WIDTH:-640}"
LINE_CAMERA_HEIGHT="${RK_COMPETITION_LINE_CAMERA_HEIGHT:-480}"
LINE_CAMERA_FPS="${RK_COMPETITION_LINE_CAMERA_FPS:-15.0}"
SDK_SERVER="${RK_COMPETITION_SDK_SERVER:-}"
SDK_UDP_HOST="${RK_COMPETITION_SDK_UDP_HOST:-127.0.0.1}"
SDK_UDP_PORT="${RK_COMPETITION_SDK_UDP_PORT:-15001}"
# 正式速度合同由这组三个变量统一注入阶段 B 的 SDK server 和阶段 C 的
# launch/forwarder；禁止其中任一端回退到 SDK 二进制自身的保守默认值。
MOTION_MAX_VX="${RK_COMPETITION_MOTION_MAX_VX:-0.30}"
MOTION_MAX_VY="${RK_COMPETITION_MOTION_MAX_VY:-0.05}"
MOTION_MAX_YAW="${RK_COMPETITION_MOTION_MAX_YAW:-0.80}"
STARTUP_TIMEOUT_SEC="${RK_COMPETITION_STARTUP_TIMEOUT_SEC:-25}"
# 下列门禁数值必须由本机冷启动实测填写。留空不是“使用方便的默认值”，
# 而是明确拒绝启动，避免把 ping 成功误当作 Sport 控制面就绪。
ROBOT_IP="${RK_COMPETITION_ROBOT_IP:-192.168.123.161}"
CONTROL_PLANE_NETWORK_TIMEOUT_SEC="${RK_COMPETITION_CONTROL_PLANE_NETWORK_TIMEOUT_SEC:-}"
CONTROL_PLANE_PING_COUNT="${RK_COMPETITION_CONTROL_PLANE_PING_COUNT:-}"
CONTROL_PLANE_PING_POLL_SEC="${RK_COMPETITION_CONTROL_PLANE_PING_POLL_SEC:-}"
CONTROL_PLANE_DDS_TIMEOUT_SEC="${RK_COMPETITION_CONTROL_PLANE_DDS_TIMEOUT_SEC:-}"
CONTROL_PLANE_REQUIRED_FRAMES="${RK_COMPETITION_CONTROL_PLANE_REQUIRED_FRAMES:-}"
CONTROL_PLANE_MAX_FRAME_GAP_MS="${RK_COMPETITION_CONTROL_PLANE_MAX_FRAME_GAP_MS:-}"
SDK_LISTEN_TIMEOUT_SEC="${RK_COMPETITION_SDK_LISTEN_TIMEOUT_SEC:-10}"

resolve_workspace_dir() {
    local candidate

    if [ -n "${RK_INSPECTION_WS:-}" ]; then
        cd -- "$RK_INSPECTION_WS" && pwd -P
        return
    fi

    # 源码和实体安装均可直接调用本脚本，不依赖 $HOME/rk_inspection_ws。
    for candidate in "$SCRIPT_DIR/../../.." "$SCRIPT_DIR/../../../../.."; do
        candidate="$(cd -- "$candidate" 2>/dev/null && pwd -P || true)"
        if [ -n "$candidate" ] \
            && { [ -d "$candidate/src/rk_bringup" ] \
                || [ -d "$candidate/install/rk_bringup" ]; }; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done

    echo "ERROR: cannot infer workspace from ${SCRIPT_PATH}; set RK_INSPECTION_WS." >&2
    return 1
}

WORKSPACE_DIR="$(resolve_workspace_dir)" || exit 1
export RK_INSPECTION_WS="$WORKSPACE_DIR"

resolve_env_script() {
    local colocated_script="${SCRIPT_DIR}/ros_clean_env.sh"
    local source_script="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
    local install_script="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/ros_clean_env.sh"

    if [ -f "$colocated_script" ]; then
        printf '%s\n' "$colocated_script"
        return 0
    fi
    if [ -f "$source_script" ]; then
        printf '%s\n' "$source_script"
        return 0
    fi
    if [ -f "$install_script" ]; then
        printf '%s\n' "$install_script"
        return 0
    fi
    echo "ERROR: ros_clean_env.sh not found in source or install tree." >&2
    return 1
}

link_node_log() {
    local label="$1"
    local pattern="$2"
    local candidate

    # SDK server 在阶段 B 由本脚本直接监管，不是 ros2 launch 子进程；
    # 保留它的原始诊断日志，不能用一个空 ROS 日志链接覆盖。
    if [ "$label" = "sdk_server" ] && [ -f "${LOG_DIR}/sdk_server.log" ]; then
        return 0
    fi

    candidate="$(find "${LOG_DIR}/ros" -type f -name "*${pattern}*.log" \
        -print 2>/dev/null | head -n 1 || true)"
    if [ -n "$candidate" ]; then
        ln -sfn "$candidate" "${LOG_DIR}/${label}.log"
    else
        # 进程刚启动时 ROS 文件可能稍后才出现；预建入口便于运维查看。
        : > "${LOG_DIR}/${label}.log"
    fi
}

create_log_aliases() {
    link_node_log line_camera line_camera_node
    link_node_log tracker real_line_tracker_node
    link_node_log sign_detector real_sign_detector_node
    link_node_log line_follower line_follower_node
    link_node_log line_course line_course_mission_node
    link_node_log white_stage_publisher white_bar_stage_command_publisher
    link_node_log white_action_executor white_bar_action_executor
    link_node_log inspection_executor inspection_action_executor
    link_node_log gait_control gait_control_node
    link_node_log command_mux command_mux_node
    link_node_log udp_forwarder cmd_vel_udp_forwarder
    link_node_log sdk_server go2_sdk_udp_server
}

readiness_passes() {
    local response
    response="$(timeout 6s ros2 service call /competition/check_readiness \
        std_srvs/srv/Trigger '{}' 2>&1)" || return 1
    printf '%s\n' "$response"
    printf '%s\n' "$response" | grep -Eq 'success[=:][[:space:]]*(true|True)'
}

readonly_graph_check() {
    local topic_info

    topic_info="$(timeout 5s ros2 topic info -v /navigation/cmd_vel 2>&1)" || {
        printf '%s\n' "$topic_info" >&2
        return 1
    }
    printf '%s\n' "$topic_info"
    if ! printf '%s\n' "$topic_info" | grep -Eq 'Publisher count: 1'; then
        echo "ERROR: /navigation/cmd_vel does not have exactly one publisher." >&2
        return 1
    fi
    if ! printf '%s\n' "$topic_info" | grep -Eq 'Node name: command_mux_node'; then
        echo "ERROR: command_mux_node is not the final cmd_vel publisher." >&2
        return 1
    fi
    timeout 4s ros2 topic info /competition/readiness_status >/dev/null
    readiness_passes
}

validate_sdk_network_interface() {
    # 在启动任何 Go2 SDK/DDS 进程前 fail-closed，避免错误网卡上的偶发路由
    # 让正式控制链带着错误拓扑继续启动。所有命令均为只读查询。
    local link_state
    local address_state
    local carrier

    if ! command -v ip >/dev/null 2>&1; then
        echo "ERROR: ip command is required for Go2 network readiness check." >&2
        return 1
    fi
    if [[ ! "$SDK_NETWORK_INTERFACE" =~ ^[[:alnum:]_.:-]+$ ]]; then
        echo "ERROR: invalid Go2 SDK interface name: ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    if ! link_state="$(ip -o link show dev "$SDK_NETWORK_INTERFACE" 2>&1)"; then
        echo "ERROR: Go2 SDK interface does not exist: ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    if [[ "$link_state" != *,UP,* || "$link_state" != *LOWER_UP* ]]; then
        echo "ERROR: Go2 SDK interface must be UP + LOWER_UP: ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    if ! address_state="$(ip -4 -o addr show dev "$SDK_NETWORK_INTERFACE" 2>&1)"; then
        echo "ERROR: cannot read IPv4 addresses for ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    if ! printf '%s\n' "$address_state" | grep -Fq "$SDK_NETWORK_ADDRESS_CIDR"; then
        echo "ERROR: ${SDK_NETWORK_INTERFACE} must have ${SDK_NETWORK_ADDRESS_CIDR} for Go2 control." >&2
        return 1
    fi
    if [ ! -r "/sys/class/net/${SDK_NETWORK_INTERFACE}/carrier" ]; then
        echo "ERROR: cannot read carrier state for ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    carrier="$(cat "/sys/class/net/${SDK_NETWORK_INTERFACE}/carrier")"
    if [ "$carrier" != "1" ]; then
        echo "ERROR: Go2 SDK interface carrier is not present: ${SDK_NETWORK_INTERFACE}" >&2
        return 1
    fi
    echo "Go2 network readiness passed: ${SDK_NETWORK_INTERFACE} ${SDK_NETWORK_ADDRESS_CIDR} (LOWER_UP, carrier=yes)"
}

if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is required for the formal competition session." >&2
    exit 1
fi
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "ERROR: tmux session already exists: ${SESSION}" >&2
    echo "Run stop_line_system.sh first; no second control graph was started." >&2
    exit 1
fi

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"
if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ]; then
    validate_sdk_network_interface || exit 1
fi
if ! ros2 pkg prefix rk_bringup >/dev/null 2>&1; then
    echo "ERROR: rk_bringup is not built/sourced. Build the workspace first." >&2
    exit 1
fi

SDK_BRIDGE_PREFIX="$(ros2 pkg prefix rk_go2_sdk_bridge)"
SDK_RUNTIME_WRAPPER="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py"
CONTROL_PLANE_GATE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_control_plane_gate.py"
CONTROL_PLANE_PROBE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_sport_state_monitor"
if [ -n "${SDK_SERVER}" ]; then
    SDK_SERVER_BINARY="${SDK_SERVER}"
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

if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ] \
        && [ "$START_SDK_SERVER" = "true" ]; then
    for measured_value in \
        "$CONTROL_PLANE_NETWORK_TIMEOUT_SEC" "$CONTROL_PLANE_PING_COUNT" \
        "$CONTROL_PLANE_PING_POLL_SEC" "$CONTROL_PLANE_DDS_TIMEOUT_SEC" \
        "$CONTROL_PLANE_REQUIRED_FRAMES" "$CONTROL_PLANE_MAX_FRAME_GAP_MS"; do
        if [ -z "$measured_value" ]; then
            echo "ERROR: cold-start control-plane thresholds are not configured." >&2
            echo "Set RK_COMPETITION_CONTROL_PLANE_{NETWORK_TIMEOUT_SEC,PING_COUNT,PING_POLL_SEC,DDS_TIMEOUT_SEC,REQUIRED_FRAMES,MAX_FRAME_GAP_MS} from a recorded cold-boot measurement." >&2
            exit 1
        fi
    done
fi

mkdir -p "$RUNTIME_DIR" "$LOG_DIR/ros"
rm -f "${RUNTIME_DIR}/pids"
touch "${RUNTIME_DIR}/pids"

LAUNCH_ARGS=(
    "hardware_mode:=${HARDWARE_MODE}"
    "software_smoke_mode:=${SOFTWARE_SMOKE_MODE}"
    "start_line_camera:=${START_LINE_CAMERA}"
    # SDK server 由阶段 B 单独启动并确认 UDP listening；launch 仅负责阶段 C。
    "start_sdk_server:=false"
    "start_udp_forwarder:=${START_UDP_FORWARDER}"
    "enable_debug_image:=${ENABLE_DEBUG_IMAGE}"
    "sdk_network_interface:=${SDK_NETWORK_INTERFACE}"
    "line_image_topic:=${LINE_IMAGE_TOPIC}"
    "line_camera_device:=${LINE_CAMERA_DEVICE}"
    "line_camera_width:=${LINE_CAMERA_WIDTH}"
    "line_camera_height:=${LINE_CAMERA_HEIGHT}"
    "line_camera_fps:=${LINE_CAMERA_FPS}"
    "sdk_udp_host:=${SDK_UDP_HOST}"
    "sdk_udp_port:=${SDK_UDP_PORT}"
    "motion_max_vx:=${MOTION_MAX_VX}"
    "motion_max_vy:=${MOTION_MAX_VY}"
    "motion_max_yaw:=${MOTION_MAX_YAW}"
)
# sdk_server 仅在显式指定时覆盖 launch 文件中 FindPackagePrefix 默认值。
if [ -n "${SDK_SERVER}" ]; then
    LAUNCH_ARGS+=("sdk_server:=${SDK_SERVER}")
fi
QUOTED_ARGS="$(printf ' %q' "${LAUNCH_ARGS[@]}")"
LAUNCH_COMMAND="source $(printf '%q' "$ENV_SCRIPT") && export ROS_LOG_DIR=$(printf '%q' "${LOG_DIR}/ros") && exec ros2 launch rk_bringup competition_non_arm.launch.py${QUOTED_ARGS}"

if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ] \
        && [ "$START_SDK_SERVER" = "true" ]; then
    GATE_COMMAND="$(printf '%q ' "$CONTROL_PLANE_GATE" \
        --interface "$SDK_NETWORK_INTERFACE" --robot-ip "$ROBOT_IP" \
        --runtime-wrapper "$SDK_RUNTIME_WRAPPER" --probe "$CONTROL_PLANE_PROBE" \
        --network-timeout-sec "$CONTROL_PLANE_NETWORK_TIMEOUT_SEC" \
        --ping-count "$CONTROL_PLANE_PING_COUNT" \
        --ping-poll-sec "$CONTROL_PLANE_PING_POLL_SEC" \
        --dds-timeout-sec "$CONTROL_PLANE_DDS_TIMEOUT_SEC" \
        --required-frames "$CONTROL_PLANE_REQUIRED_FRAMES" \
        --max-frame-gap-ms "$CONTROL_PLANE_MAX_FRAME_GAP_MS")"
    SERVER_COMMAND="$(printf '%q ' "$SDK_RUNTIME_WRAPPER" "$SDK_SERVER_BINARY" \
        --interface "$SDK_NETWORK_INTERFACE" --listen-ip "$SDK_UDP_HOST" \
        --port "$SDK_UDP_PORT" --max-vx "$MOTION_MAX_VX" \
        --max-vy "$MOTION_MAX_VY" --max-yaw "$MOTION_MAX_YAW")"
    # 阶段 A 成功后才启动阶段 B；以 server 的明确 listening 日志作为阶段 C
    # 放行条件。轮询只是观察状态，绝非用固定 sleep 猜测 DDS 是否完成发现。
    LAUNCH_COMMAND="source $(printf '%q' "$ENV_SCRIPT"); set -e; ${GATE_COMMAND}; ${SERVER_COMMAND} > $(printf '%q' "${LOG_DIR}/sdk_server.log") 2>&1 & sdk_pid=\$!; deadline=\$(( \$(date +%s) + $(printf '%q' "$SDK_LISTEN_TIMEOUT_SEC") )); while [ \$(date +%s) -lt \$deadline ]; do if grep -Fq 'UDP server listening on' $(printf '%q' "${LOG_DIR}/sdk_server.log"); then break; fi; if ! kill -0 \$sdk_pid 2>/dev/null; then echo 'SDK_STARTUP_DIAG classification=SDK_RUNTIME_LIBRARY_ERROR'; cat $(printf '%q' "${LOG_DIR}/sdk_server.log"); exit 1; fi; sleep 0.1; done; if ! grep -Fq 'UDP server listening on' $(printf '%q' "${LOG_DIR}/sdk_server.log"); then echo 'SDK_STARTUP_DIAG classification=ROBOT_CONTROL_PLANE_NOT_READY'; kill \$sdk_pid 2>/dev/null || true; exit 1; fi; echo 'CONTROL_PLANE_DIAG event=SDK_UDP_LISTENING'; ${LAUNCH_COMMAND}"
fi

tmux new-session -d -s "$SESSION" "bash -lc $(printf '%q' "$LAUNCH_COMMAND")"
tmux pipe-pane -o -t "$SESSION" "cat >> $(printf '%q' "${LOG_DIR}/launch.log")"
PANE_PID="$(tmux display-message -p -t "$SESSION" '#{pane_pid}')"
printf 'competition_launch|%s|%s\n' "$PANE_PID" "${LOG_DIR}/launch.log" \
    >> "${RUNTIME_DIR}/pids"

deadline=$(( $(date +%s) + STARTUP_TIMEOUT_SEC ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    # readiness 成功后仍要核对 ROS 图中的最终速度所有者，不能只凭服务
    # 返回值宣布可起跑。
    if readonly_graph_check >/dev/null 2>&1; then
        create_log_aliases
        echo "Formal non-arm competition chain is ready in tmux session: ${SESSION}"
        echo "Logs: ${LOG_DIR}"
        echo "Readiness: ros2 service call /competition/check_readiness std_srvs/srv/Trigger '{}'"
        echo "Mission remains stopped. Start separately: ${WORKSPACE_DIR}/src/rk_bringup/scripts/mission_start.sh"
        exit 0
    fi
    sleep 1
done

create_log_aliases
echo "ERROR: read-only ROS graph/readiness check failed; no mission start was sent." >&2
tmux send-keys -t "$SESSION" C-c 2>/dev/null || true
sleep 1
tmux kill-session -t "$SESSION" 2>/dev/null || true
exit 1
