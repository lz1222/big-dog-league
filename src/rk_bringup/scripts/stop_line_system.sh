#!/bin/bash
set +e

SESSION="rk_line"
WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
RUNTIME_DIR="${RK_LINE_RUNTIME_DIR:-$HOME/rk_line_runtime}"
PID_FILE="${RUNTIME_DIR}/pids"

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
timeout 3s ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true

if [ -f "$PID_FILE" ]; then
    while IFS='|' read -r name pid log_file; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "Stopping ${name} pid=${pid}"
            kill "$pid" 2>/dev/null
        fi
    done < "$PID_FILE"
    sleep 1
    while IFS='|' read -r name pid log_file; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "Force stopping ${name} pid=${pid}"
            kill -9 "$pid" 2>/dev/null
        fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
fi

pkill -f "ros2 topic pub" || true
pkill -f "competition_line_nav.launch.py" || true
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
