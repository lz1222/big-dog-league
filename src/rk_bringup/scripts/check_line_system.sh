#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
RUNTIME_DIR="${RK_LINE_RUNTIME_DIR:-$HOME/rk_line_runtime}"
LOG_DIR="${RK_LINE_LOG_DIR:-$HOME/rk_line_logs}"
PID_FILE="${RUNTIME_DIR}/pids"
SDK_UDP_PORT="${RK_SDK_UDP_PORT:-15001}"

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

topic_has_single_publisher() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Publisher count: 1$"
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

    if ! topic_has_publisher "/navigation/line_follow_cmd_suggested"; then
        echo "ERROR: line follower has no suggested command publisher." >&2
        failed=1
    fi

    if ! topic_has_subscription "/navigation/line_follow_cmd_suggested"; then
        echo "ERROR: mission is not subscribed to suggested commands." >&2
        failed=1
    fi

    if ! topic_has_publisher "/control/mission_cmd"; then
        echo "ERROR: line-course mission has no candidate command publisher." >&2
        failed=1
    fi

    if ! topic_has_subscription "/control/mission_cmd"; then
        echo "ERROR: command mux is not subscribed to mission commands." >&2
        failed=1
    fi

    if ! topic_has_publisher "/control/cmd_mux_status"; then
        echo "ERROR: command mux has no status publisher." >&2
        failed=1
    fi

    if ! topic_has_single_publisher "/navigation/cmd_vel"; then
        echo "ERROR: /navigation/cmd_vel must have exactly one publisher." >&2
        failed=1
    fi

    if ! topic_has_subscription "/navigation/cmd_vel"; then
        echo "ERROR: /navigation/cmd_vel has no subscriber." >&2
        echo "cmd_vel bridge/forwarder is not connected, so the robot will not move." >&2
        failed=1
    fi

    if ! topic_has_subscription "/mission/start"; then
        echo "ERROR: /mission/start has no subscriber." >&2
        echo "line follower/mission is not listening for the start command." >&2
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

print_pid_status() {
    echo
    echo "==== RK line process status ===="
    if [ ! -f "$PID_FILE" ]; then
        echo "WARN: PID file not found: ${PID_FILE}"
        return 0
    fi

    local failed=0
    while IFS='|' read -r name pid log_file; do
        if [ -z "$name" ] || [ -z "$pid" ]; then
            continue
        fi

        if kill -0 "$pid" 2>/dev/null; then
            echo "OK: ${name} pid=${pid} is running; log=${log_file}"
        else
            echo "ERROR: ${name} pid=${pid} is not running; log=${log_file}" >&2
            failed=1
        fi
    done < "$PID_FILE"

    return "$failed"
}

print_log_matches() {
    local title="$1"
    local log_file="$2"
    local pattern="$3"
    local line_count="${4:-8}"
    local matches

    echo
    echo "==== ${title} ===="
    if [ ! -f "$log_file" ]; then
        echo "WARN: log file not found: ${log_file}"
        return 0
    fi

    matches="$(grep -E "$pattern" "$log_file" || true)"
    if [ -n "$matches" ]; then
        echo "$matches" | tail -n "$line_count"
    else
        echo "WARN: no matching log lines in ${log_file}"
    fi
}

sample_line_track() {
    local sample

    echo
    echo "==== /perception/line_track sample ===="
    sample="$(timeout 4s ros2 topic echo --once /perception/line_track 2>/dev/null || true)"
    if [ -z "$sample" ]; then
        echo "WARN: no /perception/line_track sample within 4s."
        return 0
    fi

    echo "$sample" | grep -E "lateral_error:|heading_error:|confidence:|line_visible:" || true
    if echo "$sample" | grep -Eq "line_visible: true"; then
        echo "OK: line_visible=true."
    else
        echo "WARN: line_visible=false; line_follower will not drive forward until the line is accepted." >&2
    fi
}

sample_cmd_vel() {
    local sample
    local linear_x
    local angular_z

    echo
    echo "==== /navigation/cmd_vel sample ===="
    sample="$(timeout 4s ros2 topic echo --once /navigation/cmd_vel 2>/dev/null || true)"
    if [ -z "$sample" ]; then
        echo "WARN: no /navigation/cmd_vel sample within 4s."
        echo "      Usually this means /mission/start has not been accepted yet,"
        echo "      or line_follower_node is waiting for a valid line."
        return 0
    fi

    linear_x="$(echo "$sample" | awk '
        /linear:/ {section="linear"}
        /angular:/ {section="angular"}
        section == "linear" && $1 == "x:" {print $2; exit}
    ')"
    angular_z="$(echo "$sample" | awk '
        /linear:/ {section="linear"}
        /angular:/ {section="angular"}
        section == "angular" && $1 == "z:" {print $2; exit}
    ')"

    echo "linear.x=${linear_x:-unknown}"
    echo "angular.z=${angular_z:-unknown}"
    if awk -v vx="${linear_x:-0}" -v yaw="${angular_z:-0}" \
        'BEGIN { exit !((vx + 0) != 0 || (yaw + 0) != 0) }'; then
        echo "OK: cmd_vel is nonzero."
    else
        echo "WARN: cmd_vel sample is zero; the robot will not move from this command." >&2
    fi
}

check_udp_bridge_runtime() {
    echo
    echo "==== SDK UDP bridge runtime ===="

    if pgrep -af "go2_sdk_udp_server" >/dev/null 2>&1; then
        echo "OK: go2_sdk_udp_server process exists:"
        pgrep -af "go2_sdk_udp_server" || true
    else
        echo "WARN: no go2_sdk_udp_server process found." >&2
        echo "      If using start_line_system.sh defaults, the robot cannot move without this SDK receiver." >&2
    fi

    if command -v ss >/dev/null 2>&1; then
        if ss -lunp 2>/dev/null | grep -Eq "[:.]${SDK_UDP_PORT}[[:space:]]"; then
            echo "OK: UDP port ${SDK_UDP_PORT} appears to be listening."
        else
            echo "WARN: no UDP listener found on port ${SDK_UDP_PORT} by ss." >&2
        fi
    else
        echo "WARN: ss command not found; skipping UDP listener check."
    fi
}

print_runtime_diagnosis() {
    print_pid_status || true
    sample_line_track
    sample_cmd_vel
    check_udp_bridge_runtime

    print_log_matches \
        "recent tracker decisions" \
        "${LOG_DIR}/real_line_tracker.log" \
        "line_visible|reason=|reject_reason|too_dark|bottom_missing|not_enough_bands|confidence_low|track_jump_rejected|reacquire_jump_rejected" \
        12
    print_log_matches \
        "recent line_follower decisions" \
        "${LOG_DIR}/line_follower.log" \
        "Mission start received|navigation debug:|suggested_cmd.linear.x|stop_reason" \
        12
    print_log_matches \
        "recent mission decisions" \
        "${LOG_DIR}/line_course_mission.log" \
        "\\[LINE_COURSE\\]|final_cmd_vel|SDK action" \
        12
    print_log_matches \
        "recent UDP forwarder sends" \
        "${LOG_DIR}/cmd_vel_udp_forwarder.log" \
        "\\[UDP\\] send|cmd_vel_udp_forwarder started" \
        12
    print_log_matches \
        "recent SDK server log" \
        "${LOG_DIR}/sdk_server.log" \
        "." \
        20

    cat <<'EOF'

说明：
- /mission/start 的 Publisher count: 0 是正常的；mission_start.sh 是一次性发布，发布完进程就退出。
- /navigation/cmd_vel 有 subscriber 只说明 ROS forwarder 在听；机器狗是否会动，还要看 cmd_vel 是否非零、UDP forwarder 是否发送、SDK server 是否在运行。
EOF
}

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"

print_topic_info "/camera/color/image_raw"
print_topic_info "/perception/line_track"
print_topic_info "/perception/red_circle_detection"
print_topic_info "/perception/stop_zone_detection"
print_topic_info "/perception/white_bar_detection"
print_topic_info "/perception/corner_candidate"
print_topic_info "/navigation/line_follow_cmd_suggested"
print_topic_info "/control/mission_cmd"
print_topic_info "/navigation/cmd_vel"
print_topic_info "/control/cmd_mux_status"
print_topic_info "/mission/line_course_state"
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
  Subscriber: line_follower_node and line_course_mission_node

/navigation/line_follow_cmd_suggested:
  Publisher: line_follower_node
  Subscriber: line_course_mission_node

/control/mission_cmd:
  Publisher: line_course_mission_node
  Subscriber: command_mux_node

/navigation/cmd_vel:
  Publisher count: 1
  Publisher node: command_mux_node
  Subscriber: cmd_vel_udp_forwarder

/mission/start:
  Publisher count: 0 is normal before/after the one-shot start command
  Subscriber: line_follower_node and line_course_mission_node

说明：
  只运行 start_line_system.sh 只会启动节点，不会让机器狗开始走。
  真正开始巡线需要再运行：
    ~/rk_inspection_ws/src/rk_bringup/scripts/mission_start.sh

  /perception/debug/line_overlay 和 /perception/debug/line_mask 是调试图像输出。
  没有打开 rqt/view_line_debug 时，Subscription count: 0 是正常的，不影响机器狗运动。
EOF

echo
line_system_failed=0
if validate_line_system; then
    echo "OK: line system ROS topic chain is connected."
else
    echo "ERROR: line system ROS topic chain is incomplete." >&2
    line_system_failed=1
fi

if validate_debug_topics; then
    echo "OK: line debug image topics are being published."
else
    echo "WARN: debug image topics are incomplete. Restart with RK_LINE_DEBUG_IMAGE=true or set /real_line_tracker_node enable_debug_image true." >&2
fi

print_runtime_diagnosis

if [ "$line_system_failed" -ne 0 ]; then
    exit 1
fi
