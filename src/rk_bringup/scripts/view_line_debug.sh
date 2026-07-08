#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
DEFAULT_TOPIC="${RK_LINE_DEBUG_TOPIC:-/perception/debug/line_overlay}"

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

topic_has_publisher() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Publisher count: [1-9][0-9]*"
}

wait_for_debug_topic() {
    local topic_name="$1"
    local timeout_sec="$2"
    local start_time
    start_time="$(date +%s)"

    while true; do
        if topic_has_publisher "$topic_name"; then
            return 0
        fi

        if [ $(( $(date +%s) - start_time )) -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 1
    done
}

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: view_line_debug.sh must run in a VNC graphical desktop terminal." >&2
    echo "Pure SSH without a graphical display cannot open rqt_image_view." >&2
    exit 1
fi

if ! wait_for_debug_topic "$DEFAULT_TOPIC" 5; then
    echo "WARN: ${DEFAULT_TOPIC} has no publisher yet." >&2
    echo "Check tracker debug params:" >&2
    echo "  ros2 param get /real_line_tracker_node enable_debug_image" >&2
    echo "  ros2 param set /real_line_tracker_node enable_debug_image true" >&2
fi

echo "Opening rqt_image_view. Prefer this topic:"
echo "  ${DEFAULT_TOPIC}"
echo "Other useful topics:"
echo "  /camera/color/image_raw"
echo "  /perception/debug/line_overlay"
echo "  /perception/debug/line_mask"
echo "If the GUI stays blank, run:"
echo "  ${WORKSPACE_DIR}/src/rk_bringup/scripts/save_line_debug_frame.sh ${DEFAULT_TOPIC}"
echo "  ${WORKSPACE_DIR}/src/rk_bringup/scripts/stream_line_debug_web.sh ${DEFAULT_TOPIC}"

ros2 run rqt_image_view rqt_image_view "$DEFAULT_TOPIC" \
    || ros2 run rqt_image_view rqt_image_view
