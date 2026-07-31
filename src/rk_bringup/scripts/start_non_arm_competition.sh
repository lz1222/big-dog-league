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
START_REALSENSE="${RK_COMPETITION_START_REALSENSE:-true}"
START_SDK_SERVER="${RK_COMPETITION_START_SDK_SERVER:-true}"
START_UDP_FORWARDER="${RK_COMPETITION_START_UDP_FORWARDER:-true}"
ENABLE_DEBUG_IMAGE="${RK_COMPETITION_ENABLE_DEBUG_IMAGE:-false}"
SDK_NETWORK_INTERFACE="${RK_COMPETITION_SDK_NETWORK_INTERFACE:-eth0}"
IMAGE_TOPIC="${RK_COMPETITION_IMAGE_TOPIC:-/camera/camera/color/image_raw}"
SDK_SERVER="${RK_COMPETITION_SDK_SERVER:-/home/unitree/unitree_go2_sdk_test/build/go2_sdk_udp_server}"
SDK_UDP_HOST="${RK_COMPETITION_SDK_UDP_HOST:-127.0.0.1}"
SDK_UDP_PORT="${RK_COMPETITION_SDK_UDP_PORT:-15001}"
STARTUP_TIMEOUT_SEC="${RK_COMPETITION_STARTUP_TIMEOUT_SEC:-25}"

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
    link_node_log camera realsense
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
if ! ros2 pkg prefix rk_bringup >/dev/null 2>&1; then
    echo "ERROR: rk_bringup is not built/sourced. Build the workspace first." >&2
    exit 1
fi

mkdir -p "$RUNTIME_DIR" "$LOG_DIR/ros"
rm -f "${RUNTIME_DIR}/pids"
touch "${RUNTIME_DIR}/pids"

LAUNCH_ARGS=(
    "hardware_mode:=${HARDWARE_MODE}"
    "software_smoke_mode:=${SOFTWARE_SMOKE_MODE}"
    "start_realsense:=${START_REALSENSE}"
    "start_sdk_server:=${START_SDK_SERVER}"
    "start_udp_forwarder:=${START_UDP_FORWARDER}"
    "enable_debug_image:=${ENABLE_DEBUG_IMAGE}"
    "sdk_network_interface:=${SDK_NETWORK_INTERFACE}"
    "image_topic:=${IMAGE_TOPIC}"
    "sdk_server:=${SDK_SERVER}"
    "sdk_udp_host:=${SDK_UDP_HOST}"
    "sdk_udp_port:=${SDK_UDP_PORT}"
)
QUOTED_ARGS="$(printf ' %q' "${LAUNCH_ARGS[@]}")"
LAUNCH_COMMAND="source $(printf '%q' "$ENV_SCRIPT") && export ROS_LOG_DIR=$(printf '%q' "${LOG_DIR}/ros") && exec ros2 launch rk_bringup competition_non_arm.launch.py${QUOTED_ARGS}"

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
