#!/bin/bash
set -e

SESSION="rk_line"
WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"

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

send_window_command() {
    local window_name="$1"
    local command="$2"

    tmux send-keys -t "${SESSION}:${window_name}" "$command" C-m
}

ENV_SCRIPT="$(resolve_env_script)"
BRIDGE_TYPE="${RK_GO2_BRIDGE_TYPE:-sdk_udp}"
BRIDGE_BACKEND="${RK_GO2_BACKEND:-mock}"
LINE_DEBUG="${RK_LINE_DEBUG:-false}"
START_ECONOMIC_GAIT="${RK_START_ECONOMIC_GAIT:-true}"
SDK_INTERFACE="${RK_SDK_INTERFACE:-eth0}"
LINE_MIN_SPEED="${RK_LINE_MIN_SPEED:-0.27}"
LINE_BASE_SPEED="${RK_LINE_BASE_SPEED:-0.30}"
LINE_MID_SPEED="${RK_LINE_MID_SPEED:-0.28}"
LINE_SLOW_SPEED="${RK_LINE_SLOW_SPEED:-0.27}"
SHORT_LOST_LINEAR_SPEED="${RK_SHORT_LOST_LINEAR_SPEED:-0.27}"
SEARCH_LINEAR_SPEED="${RK_SEARCH_LINEAR_SPEED:-0.27}"
BRIDGE_MAX_LINEAR_X="${RK_GO2_BRIDGE_MAX_LINEAR_X:-$LINE_BASE_SPEED}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n line_nav
send_window_command "line_nav" \
    "source \"${ENV_SCRIPT}\" && ros2 launch rk_bringup competition_line_nav.launch.py image_topic:=/camera/color/image_raw debug:=${LINE_DEBUG} bridge_type:=${BRIDGE_TYPE} backend:=${BRIDGE_BACKEND} start_realsense:=true start_economic_gait:=${START_ECONOMIC_GAIT} sdk_interface:=${SDK_INTERFACE} line_min_speed:=${LINE_MIN_SPEED} line_base_speed:=${LINE_BASE_SPEED} line_mid_speed:=${LINE_MID_SPEED} line_slow_speed:=${LINE_SLOW_SPEED} short_lost_linear_speed:=${SHORT_LOST_LINEAR_SPEED} search_linear_speed:=${SEARCH_LINEAR_SPEED} bridge_max_linear_x:=${BRIDGE_MAX_LINEAR_X}"

tmux new-window -t "$SESSION" -n line_track
send_window_command "line_track" \
    "source \"${ENV_SCRIPT}\" && ros2 topic echo /perception/line_track"

tmux new-window -t "$SESSION" -n cmd_vel
send_window_command "cmd_vel" \
    "source \"${ENV_SCRIPT}\" && ros2 topic echo /navigation/cmd_vel"

tmux new-window -t "$SESSION" -n system_check
send_window_command "system_check" \
    "source \"${ENV_SCRIPT}\" && watch -n 1 'ros2 topic list | grep -E \"camera|line_track|cmd_vel|mission|perception\"'"

tmux select-window -t "${SESSION}:line_nav"

echo "RK line system tmux session started: ${SESSION}"
echo "Bridge type: ${BRIDGE_TYPE}"
echo "Unitree driver backend, only used with bridge_type=unitree_driver: ${BRIDGE_BACKEND}"
echo "Line debug images/logs: ${LINE_DEBUG}"
echo "Start economic gait: ${START_ECONOMIC_GAIT}, sdk_interface=${SDK_INTERFACE}"
echo "Line min nonzero speed: ${LINE_MIN_SPEED}"
echo "Line speeds: base=${LINE_BASE_SPEED}, mid=${LINE_MID_SPEED}, slow=${LINE_SLOW_SPEED}"
echo "Recovery speeds: short_lost=${SHORT_LOST_LINEAR_SPEED}, search=${SEARCH_LINEAR_SPEED}"
echo "Bridge max linear.x: ${BRIDGE_MAX_LINEAR_X}"
echo "Attach: tmux attach -t ${SESSION}"
echo "Confirm line_visible=true before running mission_start.sh."
echo "Emergency stop: stop_line_system.sh"
