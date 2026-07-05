#!/bin/bash
set +e

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

ENV_SCRIPT="$(resolve_env_script)"
if [ -n "$ENV_SCRIPT" ]; then
    source "$ENV_SCRIPT" || echo "WARN: failed to source clean ROS environment; continuing stop actions." >&2
fi
set +e

timeout 3s ros2 topic pub --once /mission/stop std_msgs/msg/Bool "{data: true}" || true

pkill -f "ros2 topic pub" || true
pkill -f "go2_sdk_udp_server" || true
pkill -f "cmd_vel_udp_forwarder.py" || true
pkill -f "vision_nav_debug.launch.py" || true
pkill -f "real_line_tracker_node" || true
pkill -f "line_follower_node" || true
pkill -f "realsense2_camera" || true
pkill -f "rqt_image_view" || true
pkill -f "image_view" || true

tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "RK line system stopped."
exit 0
