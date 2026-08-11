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
# Sonix SN0001 的正式验收请求；驱动协商到 320x240@15 YUYV 属于已验证
# 行为。此处与 competition_non_arm launch 默认值保持同一比赛 profile。
LINE_CAMERA_WIDTH="${RK_COMPETITION_LINE_CAMERA_WIDTH:-424}"
LINE_CAMERA_HEIGHT="${RK_COMPETITION_LINE_CAMERA_HEIGHT:-240}"
LINE_CAMERA_FPS="${RK_COMPETITION_LINE_CAMERA_FPS:-15.0}"
# 默认仍是正式比赛唯一入口；dynamic preflight 才显式覆盖为私有 topic，
# 且该参数只传给 launch 内的 line_follower_node。
LINE_FOLLOWER_START_TOPIC="${RK_COMPETITION_LINE_FOLLOWER_START_TOPIC:-/mission/start}"
# 生产默认保留完整 mission；单次动态 validation 显式关闭 mission 候选，
# 防止 WAIT_START 的安全零候选按 mux 优先级覆盖 follower adapter。
START_MISSION_NODES="${RK_COMPETITION_START_MISSION_NODES:-true}"
READINESS_PROFILE="${RK_COMPETITION_READINESS_PROFILE:-production}"
SDK_SERVER="${RK_COMPETITION_SDK_SERVER:-}"
SDK_UDP_HOST="${RK_COMPETITION_SDK_UDP_HOST:-127.0.0.1}"
SDK_UDP_PORT="${RK_COMPETITION_SDK_UDP_PORT:-15001}"
SDK_STATUS_IP="${RK_COMPETITION_SDK_STATUS_IP:-127.0.0.1}"
SDK_STATUS_PORT="${RK_COMPETITION_SDK_STATUS_PORT:-15002}"
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
# 默认先走显式正常 DDS teardown；确认底层析构断言后，现场只能显式选
# controlled。该例外仍要求同实例 fsync evidence，绝不由 shell 放宽 RC。
CONTROL_PLANE_PROBE_TERMINAL_SUCCESS_MODE="${RK_COMPETITION_CONTROL_PLANE_PROBE_TERMINAL_SUCCESS_MODE:-normal}"
SDK_LISTEN_TIMEOUT_SEC="${RK_COMPETITION_SDK_LISTEN_TIMEOUT_SEC:-10}"
STATUS_GATE_TIMEOUT_SEC="${RK_COMPETITION_STATUS_GATE_TIMEOUT_SEC:-12}"
# 控制面 DDS 样本连续并不等于 Sport RPC 已完成服务切换。默认 0 保持正式
# 启动行为不变；零运动验收可显式给出稳定窗口，期间不创建 SDK client、更不发命令。
SDK_STARTUP_SETTLE_SEC="${RK_COMPETITION_SDK_STARTUP_SETTLE_SEC:-0}"
# 当前人工经典状态由操作者显式确认；禁止用 SportModeState error_code 反推步态。
MANUAL_CLASSIC_CONFIRMED="${RK_COMPETITION_MANUAL_CLASSIC_CONFIRMED:-false}"

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
    if { [ "$label" = "sdk_server" ] || [ "$label" = "udp_forwarder" ]; } \
            && [ -f "${LOG_DIR}/${label}.log" ]; then
        return 0
    fi

    candidate="$(find "${LOG_DIR}/ros" -type f -name "*${pattern}*.log" \
        -print 2>/dev/null | head -n 1 || true)"
    if [ -z "$candidate" ]; then
        # Foxy launch 的 Python 节点日志常命名为 python3_PID_timestamp.log；
        # 此时按结构化 logger 名匹配内容，不能生成一个空 alias 掩盖证据。
        while IFS= read -r ros_log; do
            if grep -Fqm1 "[${pattern}]" "$ros_log"; then
                candidate="$ros_log"
                break
            fi
        done < <(find "${LOG_DIR}/ros" -type f -name '*.log' -print \
            2>/dev/null | sort)
    fi
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
    link_node_log global_gait_owner global_gait_owner
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

validate_clean_ros_environment() {
    # 正式父进程必须使用 Foxy/Domain10，且不继承 Unitree DDS 选择。
    local graph_output
    local forbidden_path

    if [ "${ROS_DISTRO:-}" != "foxy" ] || [ "${ROS_DOMAIN_ID:-}" != "10" ]; then
        echo "ERROR: formal ROS environment must be Foxy / Domain 10." >&2
        return 1
    fi
    for variable_name in RMW_IMPLEMENTATION CYCLONEDDS_URI CYCLONEDDS_HOME; do
        if [ -n "${!variable_name+x}" ]; then
            echo "ERROR: ${variable_name} must be unset in normal ROS environment." >&2
            return 1
        fi
    done
    for forbidden_path in \
        /usr/local/cyclonedds/lib \
        /home/unitree/cyclonedds_ws/install/cyclonedds/lib; do
        case ":${LD_LIBRARY_PATH:-}:" in
            *:"${forbidden_path}":*)
                echo "ERROR: forbidden ROS DDS path remains: ${forbidden_path}" >&2
                return 1
                ;;
        esac
    done
    if ! graph_output="$(timeout 8s ros2 node list 2>&1)"; then
        printf '%s\n' "$graph_output" >&2
        return 1
    fi
    if printf '%s\n' "$graph_output" | grep -Fq 'std::bad_alloc'; then
        printf '%s\n' "$graph_output" >&2
        return 1
    fi
    echo "ROS environment gate passed: Foxy Domain10, graph query healthy."
}

validate_profile_graph_contract() {
    # shell 启动入口必须在任何 SDK/ROS 进程之前拒绝混搭；节点内仍保留独立
    # fail-closed 检查，形成启动器与运行图的双重证据。
    case "${READINESS_PROFILE}:${START_MISSION_NODES}" in
        production:true|isolated_line_validation:false)
            return 0
            ;;
        *)
            echo "ERROR: readiness profile/start_mission_nodes mismatch: ${READINESS_PROFILE}:${START_MISSION_NODES}" >&2
            return 1
            ;;
    esac
}

udp_listener_count() {
    local port="$1"
    # `ss -lun` 的本地监听地址位于第 4 列；第 5 列是 peer 地址。若检查
    # peer 列，会把已经 bind 的 receiver 误判为未就绪并提前清理。
    ss -H -lun | awk -v port="$port" '$4 ~ (":" port "$") {count += 1} END {print count + 0}'
}

wait_for_udp_listener_count() {
    local port="$1"
    local expected="$2"
    local timeout_sec="$3"
    local deadline=$(( $(date +%s) + timeout_sec ))

    while [ "$(date +%s)" -lt "$deadline" ]; do
        if [ "$(udp_listener_count "$port")" -eq "$expected" ]; then
            return 0
        fi
        sleep 0.1
    done
    echo "ERROR: UDP port ${port} listener count is $(udp_listener_count "$port"), expected ${expected}." >&2
    return 1
}

wait_for_global_gait_owner_status_subscriber() {
    # UDP receiver ready 只证明 datagram 不会丢；server 启动前还要确认正式
    # GlobalGaitOwner 已订阅 ROS status，避免把 late-joiner replay 当作主路径。
    local timeout_sec="$1"
    local deadline=$(( $(date +%s) + timeout_sec ))
    local topic_info

    while [ "$(date +%s)" -lt "$deadline" ]; do
        topic_info="$(timeout 4s ros2 topic info -v /go2/sdk_motion_status 2>&1 || true)"
        if printf '%s\n' "$topic_info" | grep -Fq 'Node name: global_gait_owner'; then
            echo "GlobalGaitOwner status subscriber ready."
            return 0
        fi
        sleep 0.2
    done
    echo "ERROR: global_gait_owner did not subscribe to /go2/sdk_motion_status." >&2
    return 1
}

record_tmux_pane() {
    local label="$1"
    local target="$2"
    local log_file="$3"
    local pane_pid
    pane_pid="$(tmux display-message -p -t "$target" '#{pane_pid}')"
    printf '%s|%s|%s\n' "$label" "$pane_pid" "$log_file" \
        >> "${RUNTIME_DIR}/pids"
}

cleanup_failed_start() {
    # 优先向每个受管 pane 发送 SIGINT，使 server 的 signal_exit StopMove 和
    # forwarder 的全零 shutdown 路径有机会正常完成；随后再收走 tmux session。
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        local window
        for window in ros_graph sdk_status sdk_server; do
            tmux send-keys -t "${SESSION}:${window}" C-c 2>/dev/null || true
        done
        sleep 2
        tmux kill-session -t "$SESSION" 2>/dev/null || true
    fi
    rm -f "${RUNTIME_DIR}/pids"
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

validate_profile_graph_contract || exit 1

ENV_SCRIPT="$(resolve_env_script)"
# 正式入口不依赖调用者 shell；软件 smoke 请使用独立 acceptance 的隔离域。
export RK_ROS_DOMAIN_ID=10
source "$ENV_SCRIPT"
validate_clean_ros_environment || exit 1
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
SDK_STATUS_GATE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/sdk_motion_status_gate.py"
SDK_SERVER_START_GATE="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/sdk_server_start_gate.py"
MOTION_CONTROL_PREARM="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_motion_control_prearm"
UDP_FORWARDER="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py"
if [ -n "${SDK_SERVER}" ]; then
    SDK_SERVER_BINARY="${SDK_SERVER}"
else
    SDK_SERVER_BINARY="${SDK_BRIDGE_PREFIX}/lib/rk_go2_sdk_bridge/go2_sdk_udp_server"
fi

for required_file in "$SDK_RUNTIME_WRAPPER" "$CONTROL_PLANE_GATE" \
    "$CONTROL_PLANE_PROBE" "$SDK_STATUS_GATE" "$SDK_SERVER_START_GATE" \
    "$MOTION_CONTROL_PREARM" \
    "$UDP_FORWARDER" \
    "$SDK_SERVER_BINARY"; do
    if [ ! -x "$required_file" ]; then
        echo "ERROR: required staged-start executable is missing: ${required_file}" >&2
        exit 1
    fi
done

if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ] \
        && [ "$START_SDK_SERVER" = "true" ]; then
    case "$CONTROL_PLANE_PROBE_TERMINAL_SUCCESS_MODE" in
        normal|controlled) ;;
        *)
            echo "ERROR: invalid control-plane probe terminal-success mode." >&2
            exit 1
            ;;
    esac
    if ! [[ "$SDK_STARTUP_SETTLE_SEC" =~ ^[0-9]+$ ]]; then
        echo "ERROR: RK_COMPETITION_SDK_STARTUP_SETTLE_SEC must be a non-negative integer." >&2
        exit 1
    fi
    if [ "$MANUAL_CLASSIC_CONFIRMED" != "true" ] \
            && [ "$MANUAL_CLASSIC_CONFIRMED" != "false" ]; then
        echo "ERROR: RK_COMPETITION_MANUAL_CLASSIC_CONFIRMED must be true or false." >&2
        exit 1
    fi
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

if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ]; then
    if [ "$START_SDK_SERVER" != "true" ] || [ "$START_UDP_FORWARDER" != "true" ]; then
        echo "ERROR: formal hardware mode requires one staged SDK server and forwarder." >&2
        exit 1
    fi
    if [ "$(udp_listener_count "$SDK_UDP_PORT")" -ne 0 ] \
            || [ "$(udp_listener_count "$SDK_STATUS_PORT")" -ne 0 ]; then
        echo "ERROR: stale command/status UDP listener exists; run cleanup first." >&2
        exit 1
    fi
fi

mkdir -p "$RUNTIME_DIR" "$LOG_DIR/ros"
rm -f "${RUNTIME_DIR}/pids"
touch "${RUNTIME_DIR}/pids"
# cleanup 必须读取启动时冻结的真实图合同，不能依赖另一个 shell 的默认值。
printf '%s\n' "$READINESS_PROFILE" > "${RUNTIME_DIR}/readiness_profile"
printf '%s\n' "$START_MISSION_NODES" > "${RUNTIME_DIR}/start_mission_nodes"

SERVER_INSTANCE_ID="software-smoke"
if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ]; then
    SERVER_INSTANCE_ID="$(tr -d '\r\n' < /proc/sys/kernel/random/uuid)"
fi
printf '%s\n' "$SERVER_INSTANCE_ID" > "${RUNTIME_DIR}/server_instance_id"

LAUNCH_ARGS=(
    "hardware_mode:=${HARDWARE_MODE}"
    "software_smoke_mode:=${SOFTWARE_SMOKE_MODE}"
    "start_line_camera:=${START_LINE_CAMERA}"
    # 阶段 C 严禁重复创建阶段 B0/B1 已监管的进程。
    "start_sdk_server:=false"
    "start_udp_forwarder:=false"
    "enable_debug_image:=${ENABLE_DEBUG_IMAGE}"
    "sdk_network_interface:=${SDK_NETWORK_INTERFACE}"
    "line_image_topic:=${LINE_IMAGE_TOPIC}"
    "line_camera_device:=${LINE_CAMERA_DEVICE}"
    "line_camera_width:=${LINE_CAMERA_WIDTH}"
    "line_camera_height:=${LINE_CAMERA_HEIGHT}"
    "line_camera_fps:=${LINE_CAMERA_FPS}"
    "line_follower_start_topic:=${LINE_FOLLOWER_START_TOPIC}"
    "start_mission_nodes:=${START_MISSION_NODES}"
    "readiness_profile:=${READINESS_PROFILE}"
    "sdk_udp_host:=${SDK_UDP_HOST}"
    "sdk_udp_port:=${SDK_UDP_PORT}"
    "sdk_status_ip:=${SDK_STATUS_IP}"
    "sdk_status_port:=${SDK_STATUS_PORT}"
    "sdk_server_instance_id:=${SERVER_INSTANCE_ID}"
    "motion_max_vx:=${MOTION_MAX_VX}"
    "motion_max_vy:=${MOTION_MAX_VY}"
    "motion_max_yaw:=${MOTION_MAX_YAW}"
)
if [ -n "${SDK_SERVER}" ]; then
    LAUNCH_ARGS+=("sdk_server:=${SDK_SERVER}")
fi
QUOTED_ARGS="$(printf ' %q' "${LAUNCH_ARGS[@]}")"
LAUNCH_COMMAND="source $(printf '%q' "$ENV_SCRIPT") && export RK_ROS_DOMAIN_ID=10 ROS_LOG_DIR=$(printf '%q' "${LOG_DIR}/ros") && exec ros2 launch rk_bringup competition_non_arm.launch.py${QUOTED_ARGS}"

if [ "$HARDWARE_MODE" = "true" ] && [ "$SOFTWARE_SMOKE_MODE" != "true" ]; then
    CONTROL_GATE_COMMAND=(
        "$CONTROL_PLANE_GATE"
        --interface "$SDK_NETWORK_INTERFACE"
        --robot-ip "$ROBOT_IP"
        --runtime-wrapper "$SDK_RUNTIME_WRAPPER"
        --probe "$CONTROL_PLANE_PROBE"
        --network-timeout-sec "$CONTROL_PLANE_NETWORK_TIMEOUT_SEC"
        --ping-count "$CONTROL_PLANE_PING_COUNT"
        --ping-poll-sec "$CONTROL_PLANE_PING_POLL_SEC"
        --dds-timeout-sec "$CONTROL_PLANE_DDS_TIMEOUT_SEC"
        --required-frames "$CONTROL_PLANE_REQUIRED_FRAMES"
        --max-frame-gap-ms "$CONTROL_PLANE_MAX_FRAME_GAP_MS"
        --evidence-dir "${RUNTIME_DIR}/control_plane_probe_evidence"
        --terminal-success-mode "$CONTROL_PLANE_PROBE_TERMINAL_SUCCESS_MODE"
    )
    if ! "${CONTROL_GATE_COMMAND[@]}" 2>&1 \
            | tee "${LOG_DIR}/control_plane_gate.log"; then
        cleanup_failed_start
        exit 1
    fi
    # 正式 SDK server 之前只读核验 MotionSwitcher 与 Sport responder。
    # mcf 是经典静止时的已观测状态，不能在此处触发任何模式释放或服务切换。
    echo "MOTION_CONTROL_PREARM mutation_count=0"
    if ! "$SDK_RUNTIME_WRAPPER" "$MOTION_CONTROL_PREARM" \
            --interface "$SDK_NETWORK_INTERFACE" \
            --version-timeout-sec 3 2>&1 \
            | tee "${LOG_DIR}/motion_control_prearm.log"; then
        cleanup_failed_start
        exit 1
    fi
    if [ "$SDK_STARTUP_SETTLE_SEC" -gt 0 ]; then
        # 只被 dynamic preflight 显式启用的被动等待，避免 DDS state stream
        # 刚稳定时立即创建 SportClient 所造成的启动 RPC 瞬态失败。
        echo "CONTROL_PLANE_DIAG event=POST_DDS_PASSIVE_SETTLE seconds=${SDK_STARTUP_SETTLE_SEC}"
        sleep "$SDK_STARTUP_SETTLE_SEC"
    fi

    FORWARDER_ARGS=(
        "$UDP_FORWARDER" --ros-args
        -p "cmd_vel_topic:=/navigation/cmd_vel"
        -p "udp_host:=${SDK_UDP_HOST}"
        -p "udp_port:=${SDK_UDP_PORT}"
        -p "status_ip:=${SDK_STATUS_IP}"
        -p "status_port:=${SDK_STATUS_PORT}"
        -p "expected_server_instance_id:=${SERVER_INSTANCE_ID}"
        -p "max_vx:=${MOTION_MAX_VX}"
        -p "max_vy:=${MOTION_MAX_VY}"
        -p "max_yaw:=${MOTION_MAX_YAW}"
    )
    FORWARDER_COMMAND="source $(printf '%q' "$ENV_SCRIPT") && export RK_ROS_DOMAIN_ID=10 && exec $(printf '%q ' "${FORWARDER_ARGS[@]}")"
    if ! tmux new-session -d -s "$SESSION" -n sdk_status \
            "bash -lc $(printf '%q' "$FORWARDER_COMMAND")"; then
        cleanup_failed_start
        exit 1
    fi
    tmux pipe-pane -o -t "${SESSION}:sdk_status" \
        "cat >> $(printf '%q' "${LOG_DIR}/udp_forwarder.log")"
    record_tmux_pane udp_forwarder "${SESSION}:sdk_status" \
        "${LOG_DIR}/udp_forwarder.log"

    if ! "$SDK_STATUS_GATE" --mode receiver \
            --expected-server-instance-id "$SERVER_INSTANCE_ID" \
            --status-ip "$SDK_STATUS_IP" --status-port "$SDK_STATUS_PORT" \
            --timeout-sec "$STATUS_GATE_TIMEOUT_SEC" 2>&1 \
            | tee "${LOG_DIR}/status_receiver_gate.log"; then
        cleanup_failed_start
        exit 1
    fi
    if ! wait_for_udp_listener_count \
            "$SDK_STATUS_PORT" 1 "$STATUS_GATE_TIMEOUT_SEC"; then
        cleanup_failed_start
        exit 1
    fi

    # 先创建完整 ROS 图：GlobalGaitOwner 必须已订阅 status，server 的启动
    # CLASSIC 事件才可同时走实时 subscriber 与现有 replay 双路径。
    if ! tmux new-window -d -t "$SESSION" -n ros_graph \
            "bash -lc $(printf '%q' "$LAUNCH_COMMAND")"; then
        cleanup_failed_start
        exit 1
    fi
    tmux pipe-pane -o -t "${SESSION}:ros_graph" \
        "cat >> $(printf '%q' "${LOG_DIR}/launch.log")"
    record_tmux_pane competition_launch "${SESSION}:ros_graph" \
        "${LOG_DIR}/launch.log"
    if ! wait_for_global_gait_owner_status_subscriber \
            "$STATUS_GATE_TIMEOUT_SEC"; then
        cleanup_failed_start
        exit 1
    fi

    STATUS_MIN_RECEIVE_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
    SERVER_ARGS=(
        "$SDK_SERVER_START_GATE"
        --runtime-wrapper "$SDK_RUNTIME_WRAPPER"
        --sdk-server "$SDK_SERVER_BINARY"
        --interface "$SDK_NETWORK_INTERFACE"
        --listen-ip "$SDK_UDP_HOST"
        --port "$SDK_UDP_PORT"
        --status-ip "$SDK_STATUS_IP"
        --status-port "$SDK_STATUS_PORT"
        --server-instance-id "$SERVER_INSTANCE_ID"
        --max-vx "$MOTION_MAX_VX"
        --max-vy "$MOTION_MAX_VY"
        --max-yaw "$MOTION_MAX_YAW"
        --manual-classic-confirmed "$MANUAL_CLASSIC_CONFIRMED"
        --receiver-ready-timeout-sec "$STATUS_GATE_TIMEOUT_SEC"
    )
    SERVER_COMMAND="exec $(printf '%q ' "${SERVER_ARGS[@]}")"
    if ! tmux new-window -d -t "$SESSION" -n sdk_server \
            "bash -lc $(printf '%q' "$SERVER_COMMAND")"; then
        cleanup_failed_start
        exit 1
    fi
    tmux pipe-pane -o -t "${SESSION}:sdk_server" \
        "cat >> $(printf '%q' "${LOG_DIR}/sdk_server.log")"
    record_tmux_pane sdk_server "${SESSION}:sdk_server" \
        "${LOG_DIR}/sdk_server.log"

    # 唯一 SDK server 已完成 ClassicWalk 调用序列后才发布当前实例 ACK；
    # 不读取 error_code 来推断步态，也不把 UDP bind 当作经典步态成功。
    if ! "$SDK_STATUS_GATE" --mode status --event CLASSIC_VERIFIED \
            --required-ret 0 --reject-move \
            --expected-server-instance-id "$SERVER_INSTANCE_ID" \
            --min-receive-monotonic-ns "$STATUS_MIN_RECEIVE_NS" \
            --status-ip "$SDK_STATUS_IP" --status-port "$SDK_STATUS_PORT" \
            --timeout-sec "$STATUS_GATE_TIMEOUT_SEC" 2>&1 \
            | tee "${LOG_DIR}/classic_verified_status_gate.log"; then
        cleanup_failed_start
        exit 1
    fi
    if ! "$SDK_STATUS_GATE" --mode status --event STARTUP_STOP \
            --required-ret 0 --reject-move \
            --expected-server-instance-id "$SERVER_INSTANCE_ID" \
            --min-receive-monotonic-ns "$STATUS_MIN_RECEIVE_NS" \
            --status-ip "$SDK_STATUS_IP" --status-port "$SDK_STATUS_PORT" \
            --timeout-sec "$STATUS_GATE_TIMEOUT_SEC" 2>&1 \
            | tee "${LOG_DIR}/startup_status_gate.log"; then
        cleanup_failed_start
        exit 1
    fi
    if ! wait_for_udp_listener_count \
            "$SDK_UDP_PORT" 1 "$SDK_LISTEN_TIMEOUT_SEC"; then
        cleanup_failed_start
        exit 1
    fi
    if [ "$(tmux display-message -p -t "${SESSION}:sdk_server" '#{pane_dead}')" != "0" ]; then
        echo "ERROR: SDK server exited after startup ACK." >&2
        cleanup_failed_start
        exit 1
    fi

else
    if ! tmux new-session -d -s "$SESSION" -n ros_graph \
            "bash -lc $(printf '%q' "$LAUNCH_COMMAND")"; then
        cleanup_failed_start
        exit 1
    fi
fi

if [ "$HARDWARE_MODE" != "true" ] || [ "$SOFTWARE_SMOKE_MODE" = "true" ]; then
    tmux pipe-pane -o -t "${SESSION}:ros_graph" \
        "cat >> $(printf '%q' "${LOG_DIR}/launch.log")"
    record_tmux_pane competition_launch "${SESSION}:ros_graph" \
        "${LOG_DIR}/launch.log"
fi

deadline=$(( $(date +%s) + STARTUP_TIMEOUT_SEC ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    # readiness 成功后仍要核对 ROS 图中的最终速度所有者，不能只凭服务
    # 返回值宣布可起跑。每轮覆盖保存完整只读证据，超时时不能只留下
    # 一个笼统错误而丢失实际未通过的 readiness 条目。
    if readonly_graph_check > "${LOG_DIR}/readiness_gate.log" 2>&1; then
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
if [ -s "${LOG_DIR}/readiness_gate.log" ]; then
    sed -n '1,240p' "${LOG_DIR}/readiness_gate.log" >&2
fi
cleanup_failed_start
exit 1
