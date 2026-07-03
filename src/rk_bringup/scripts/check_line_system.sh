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

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"

print_topic_info "/camera/color/image_raw"
print_topic_info "/perception/line_track"
print_topic_info "/navigation/cmd_vel"
print_topic_info "/mission/start"
print_topic_info "/mission/stop"

cat <<'EOF'

正常链路应该是：
/camera/color/image_raw:
  Publisher: camera
  Subscriber: real_line_tracker_node

/perception/line_track:
  Publisher: real_line_tracker_node
  Subscriber: line_follower_node

/navigation/cmd_vel:
  Publisher: line_follower_node
  Subscriber: cmd_vel_udp_forwarder

/mission/start:
  Subscriber: line_follower_node
EOF
