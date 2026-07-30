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

topic_has_subscription() {
    local topic_name="$1"

    ros2 topic info "$topic_name" 2>/dev/null \
        | grep -Eq "Subscription count: [1-9][0-9]*"
}

wait_for_topic_subscription() {
    local topic_name="$1"
    local timeout_sec="$2"
    local start_time
    start_time="$(date +%s)"

    while true; do
        if topic_has_subscription "$topic_name"; then
            return 0
        fi

        if [ $(( $(date +%s) - start_time )) -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 1
    done
}

if ! wait_for_topic_subscription "/mission/start" 8; then
    echo "ERROR: /mission/start has no subscriber." >&2
    echo "line course mission is not ready; run start_line_system.sh and check line_course_mission.log." >&2
    exit 1
fi

for _ in 1 2 3; do
    ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
    sleep 0.2
done

echo "Published /mission/start true. Check /navigation/cmd_vel and line_course_mission.log if the robot does not move."
