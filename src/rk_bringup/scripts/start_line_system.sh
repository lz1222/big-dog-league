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
BRIDGE_BACKEND="${RK_GO2_BACKEND:-unitree_ros2}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n line_nav
send_window_command "line_nav" \
    "source \"${ENV_SCRIPT}\" && ros2 launch rk_bringup competition_line_nav.launch.py image_topic:=/camera/color/image_raw debug:=true backend:=${BRIDGE_BACKEND} start_realsense:=true"

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
echo "Bridge backend: ${BRIDGE_BACKEND}"
echo "Attach: tmux attach -t ${SESSION}"
echo "Confirm line_visible=true before running mission_start.sh."
echo "Emergency stop: stop_line_system.sh"
