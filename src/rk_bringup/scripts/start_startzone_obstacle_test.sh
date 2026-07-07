#!/bin/bash
set -e

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
source "$ENV_SCRIPT"

SDK_INTERFACE="${RK_SDK_INTERFACE:-eth0}"
LINE_DEBUG="${RK_LINE_DEBUG:-false}"
BRIDGE_MAX_LINEAR_X="${RK_GO2_BRIDGE_MAX_LINEAR_X:-0.60}"
BRIDGE_MAX_ANGULAR_Z="${RK_GO2_BRIDGE_MAX_ANGULAR_Z:-1.00}"

echo "Starting start-zone line-follow + obstacle direct route test"
echo "workspace=${WORKSPACE_DIR}"
echo "sdk_interface=${SDK_INTERFACE}"
echo "line_debug=${LINE_DEBUG}"
echo "bridge_max_linear_x=${BRIDGE_MAX_LINEAR_X}"
echo "bridge_max_angular_z=${BRIDGE_MAX_ANGULAR_Z}"
echo "Ctrl+C will trigger route-node emergency stop."

exec ros2 launch rk_bringup obstacle_direct_open_loop.launch.py \
    sdk_network_interface:="${SDK_INTERFACE}" \
    bridge_max_linear_x:="${BRIDGE_MAX_LINEAR_X}" \
    bridge_max_angular_z:="${BRIDGE_MAX_ANGULAR_Z}" \
    start_realsense:=true \
    start_line_nodes:=true \
    debug:="${LINE_DEBUG}" \
    run_without_sdk_actions:=false \
    allow_ros_topic_sdk_actions:=false
