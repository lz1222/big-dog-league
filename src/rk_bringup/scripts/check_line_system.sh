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

print_topic_info() {
    local topic_name="$1"

    echo
    echo "==== ${topic_name} ===="
    if ! ros2 topic info "$topic_name" -v; then
        echo "WARN: unable to read topic info for ${topic_name}."
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

validate_line_system() {
    local failed=0

    if ! topic_has_publisher "/camera/color/image_raw"; then
        echo "ERROR: /camera/color/image_raw has no publisher." >&2
        failed=1
    fi

    if ! topic_has_publisher "/perception/line_track"; then
        echo "ERROR: /perception/line_track has no publisher." >&2
        failed=1
    fi

    if ! topic_has_subscription "/perception/line_track"; then
        echo "ERROR: /perception/line_track has no subscriber." >&2
        failed=1
    fi

    if ! topic_has_publisher "/navigation/cmd_vel"; then
        echo "ERROR: /navigation/cmd_vel has no publisher." >&2
        failed=1
    fi

    if ! topic_has_subscription "/navigation/cmd_vel"; then
        echo "ERROR: /navigation/cmd_vel has no subscriber." >&2
        echo "cmd_vel bridge/forwarder is not connected, so the robot will not move." >&2
        failed=1
    fi

    return "$failed"
}

validate_debug_topics() {
    local failed=0

    if ! topic_has_publisher "/perception/debug/line_overlay"; then
        echo "WARN: /perception/debug/line_overlay has no publisher." >&2
        failed=1
    fi

    if ! topic_has_publisher "/perception/debug/line_mask"; then
        echo "WARN: /perception/debug/line_mask has no publisher." >&2
        failed=1
    fi

    return "$failed"
}

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"

print_topic_info "/camera/color/image_raw"
print_topic_info "/perception/line_track"
print_topic_info "/navigation/cmd_vel"
print_topic_info "/mission/start"
print_topic_info "/mission/stop"
print_topic_info "/perception/debug/line_overlay"
print_topic_info "/perception/debug/line_mask"

cat <<'EOF'

正常链路应该是：
/camera/color/image_raw:
  Publisher: camera
  Subscriber: real_line_tracker_node

/perception/line_track:
  Publisher: real_line_tracker_node
  Subscriber: line_follower_node

/navigation/cmd_vel:
  Publisher count: 1
  Publisher node: line_follower_node
  Subscriber: cmd_vel_bridge_node or cmd_vel_udp_forwarder

/mission/start:
  Subscriber: line_follower_node
EOF

echo
if validate_line_system; then
    echo "OK: line system ROS topic chain is connected."
else
    echo "ERROR: line system ROS topic chain is incomplete." >&2
    exit 1
fi

if validate_debug_topics; then
    echo "OK: line debug image topics are being published."
else
    echo "WARN: debug image topics are incomplete. Restart with RK_LINE_DEBUG_IMAGE=true or set /real_line_tracker_node enable_debug_image true." >&2
fi
