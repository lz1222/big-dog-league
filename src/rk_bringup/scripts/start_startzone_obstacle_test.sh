#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"

select_ros_setup() {
    local distro

    if [ -n "${ROS_DISTRO:-}" ]; then
        local active_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
        if [ -f "$active_setup" ]; then
            printf "%s\n" "$active_setup"
            return 0
        fi
    fi

    for distro in foxy humble; do
        local setup_file="/opt/ros/${distro}/setup.bash"
        if [ -f "$setup_file" ]; then
            printf "%s\n" "$setup_file"
            return 0
        fi
    done

    echo "ERROR: no supported ROS2 setup.bash found under /opt/ros." >&2
    return 1
}

source_base_ros() {
    local ros_setup
    ros_setup="$(select_ros_setup)"
    source "$ros_setup"
}

source_workspace_if_present() {
    if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
        source "${WORKSPACE_DIR}/install/setup.bash"
    fi
}

workspace_packages_ready() {
    ros2 pkg prefix rk_bringup >/dev/null 2>&1 \
        && ros2 pkg prefix rk_config >/dev/null 2>&1 \
        && ros2 pkg prefix rk_unitree_driver >/dev/null 2>&1 \
        && ros2 pkg prefix rk_locomotion >/dev/null 2>&1 \
        && ros2 pkg prefix rk_mission >/dev/null 2>&1 \
        && ros2 pkg prefix rk_tools >/dev/null 2>&1 \
        && ros2 pkg prefix rk_perception >/dev/null 2>&1 \
        && ros2 pkg prefix rk_navigation >/dev/null 2>&1 \
        && ros2 pkg prefix rk_go2_sdk_bridge >/dev/null 2>&1
}

run_clean_colcon_build() {
    local ros_setup
    ros_setup="$(select_ros_setup)"

    env \
        -u AMENT_PREFIX_PATH \
        -u CMAKE_PREFIX_PATH \
        -u COLCON_PREFIX_PATH \
        bash -lc \
        "source \"${ros_setup}\" && cd \"${WORKSPACE_DIR}\" && colcon build --symlink-install --packages-select rk_interfaces rk_config rk_unitree_driver rk_locomotion rk_mission rk_go2_sdk_bridge rk_navigation rk_perception rk_tools rk_bringup"
}

build_workspace_if_needed() {
    source_base_ros
    source_workspace_if_present

    if [ "${RK_SKIP_BUILD:-false}" != "true" ]; then
        echo "Building start-zone obstacle test packages..."
        run_clean_colcon_build
        return 0
    fi

    if workspace_packages_ready; then
        return 0
    fi

    echo "ROS packages are not ready in install/. Building required packages..."
    run_clean_colcon_build
}

stop_existing_processes() {
    set +e
    pkill -f "obstacle_direct_open_loop.launch.py" || true
    pkill -f "obstacle_direct_route_node.py" || true
    pkill -f "real_line_tracker_node" || true
    pkill -f "line_follower_node" || true
    pkill -f "realsense2_camera_node" || true
    pkill -f "realsense2_camera" || true
    pkill -f "cmd_vel_udp_forwarder.py" || true
    pkill -f "go2_sdk_udp_server" || true
    sleep 1
    set -e
}

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
stop_existing_processes
build_workspace_if_needed
source "$ENV_SCRIPT"

SDK_INTERFACE="${RK_SDK_INTERFACE:-eth0}"
LINE_DEBUG="${RK_LINE_DEBUG:-false}"
BRIDGE_MAX_LINEAR_X="${RK_GO2_BRIDGE_MAX_LINEAR_X:-0.60}"
BRIDGE_MAX_ANGULAR_Z="${RK_GO2_BRIDGE_MAX_ANGULAR_Z:-1.00}"
LINE_LOST_SWITCH_SEC="${RK_LINE_LOST_SWITCH_SEC:-0.6}"
LINE_TRACK_STALE_SEC="${RK_LINE_TRACK_STALE_SEC:-0.8}"
WHITE_LINE_DETECTION_ENABLED="${RK_WHITE_LINE_DETECTION_ENABLED:-true}"
WHITE_LINE_MIN_WIDTH_FRACTION="${RK_WHITE_LINE_MIN_WIDTH_FRACTION:-0.35}"
WHITE_LINE_MIN_VALUE="${RK_WHITE_LINE_MIN_VALUE:-185}"
WHITE_LINE_MAX_SATURATION="${RK_WHITE_LINE_MAX_SATURATION:-95}"

echo "Starting start-zone line-follow + obstacle direct route test"
echo "workspace=${WORKSPACE_DIR}"
echo "sdk_interface=${SDK_INTERFACE}"
echo "line_debug=${LINE_DEBUG}"
echo "bridge_max_linear_x=${BRIDGE_MAX_LINEAR_X}"
echo "bridge_max_angular_z=${BRIDGE_MAX_ANGULAR_Z}"
echo "line_lost_switch_sec=${LINE_LOST_SWITCH_SEC}"
echo "line_track_stale_sec=${LINE_TRACK_STALE_SEC}"
echo "white_line_detection_enabled=${WHITE_LINE_DETECTION_ENABLED}"
echo "white_line_min_width_fraction=${WHITE_LINE_MIN_WIDTH_FRACTION}"
echo "white_line_min_value=${WHITE_LINE_MIN_VALUE}"
echo "white_line_max_saturation=${WHITE_LINE_MAX_SATURATION}"
echo "Ctrl+C will trigger route-node emergency stop."

exec ros2 launch rk_bringup obstacle_direct_open_loop.launch.py \
    sdk_network_interface:="${SDK_INTERFACE}" \
    bridge_max_linear_x:="${BRIDGE_MAX_LINEAR_X}" \
    bridge_max_angular_z:="${BRIDGE_MAX_ANGULAR_Z}" \
    start_realsense:=true \
    start_line_nodes:=true \
    debug:="${LINE_DEBUG}" \
    line_lost_switch_sec:="${LINE_LOST_SWITCH_SEC}" \
    line_track_stale_sec:="${LINE_TRACK_STALE_SEC}" \
    white_line_detection_enabled:="${WHITE_LINE_DETECTION_ENABLED}" \
    white_line_min_width_fraction:="${WHITE_LINE_MIN_WIDTH_FRACTION}" \
    white_line_min_value:="${WHITE_LINE_MIN_VALUE}" \
    white_line_max_saturation:="${WHITE_LINE_MAX_SATURATION}" \
    run_without_sdk_actions:=false \
    allow_ros_topic_sdk_actions:=false
