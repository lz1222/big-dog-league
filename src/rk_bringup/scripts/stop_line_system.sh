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

ESTOP_SERVICE="/safety/estop"
ESTOP_SERVICE_TYPE="std_srvs/srv/SetBool"

is_zero_twist_sample() {
    awk '
        BEGIN {
            section = ""
            values_seen = 0
            all_zero = 1
        }
        /^linear:/ {
            section = "linear"
            next
        }
        /^angular:/ {
            section = "angular"
            next
        }
        /^[[:space:]]+[xyz]:/ {
            if (section != "linear" && section != "angular") {
                next
            }
            value = $2
            values_seen += 1
            if (value !~ /^[-+]?[0-9]+([.][0-9]*)?([eE][-+]?[0-9]+)?$/) {
                all_zero = 0
            } else if ((value + 0.0) != 0.0) {
                all_zero = 0
            }
        }
        END {
            exit !(values_seen == 6 && all_zero == 1)
        }
    '
}

wait_for_mux_zero() {
    local attempt
    local sample

    for attempt in 1 2 3 4 5; do
        sample="$(timeout 1s ros2 topic echo --once /navigation/cmd_vel 2>/dev/null)"
        if [ -n "$sample" ] && printf "%s\n" "$sample" | is_zero_twist_sample; then
            echo "Verified command_mux zero output on /navigation/cmd_vel."
            return 0
        fi
    done

    return 1
}

emergency_cmd_vel_fallback() {
    echo "!!!!!!!!!!!!!!!! EMERGENCY FALLBACK !!!!!!!!!!!!!!!!" >&2
    echo "The SetBool estop service is unavailable or its call failed." >&2
    echo "Publishing one zero Twist directly to /navigation/cmd_vel." >&2
    echo "This bypasses the normal single-publisher command_mux architecture." >&2
    echo "!!!!!!!!!!!!!!!! EMERGENCY FALLBACK !!!!!!!!!!!!!!!!" >&2
    timeout 3s ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
}

estop_service_type="$(timeout 3s ros2 service type "$ESTOP_SERVICE" 2>/dev/null)"
if [ "$estop_service_type" != "$ESTOP_SERVICE_TYPE" ]; then
    echo "WARN: ${ESTOP_SERVICE} is not available as ${ESTOP_SERVICE_TYPE}." >&2
    emergency_cmd_vel_fallback
else
    estop_response="$(
        timeout 5s ros2 service call "$ESTOP_SERVICE" "$ESTOP_SERVICE_TYPE" \
            "{data: true}" 2>&1
    )"
    estop_call_status=$?
    printf "%s\n" "$estop_response"

    if [ "$estop_call_status" -ne 0 ] \
        || ! printf "%s\n" "$estop_response" \
            | grep -Eq 'success[=:][[:space:]]*(true|True)'; then
        echo "WARN: ${ESTOP_SERVICE} call did not return success=true." >&2
        emergency_cmd_vel_fallback
    elif ! wait_for_mux_zero; then
        echo "ERROR: estop service succeeded, but command_mux zero output was not observed." >&2
        echo "Nodes remain running so the estop owner is not removed before zero output is verified." >&2
        exit 1
    fi
fi

timeout 3s ros2 topic pub --once /mission/stop std_msgs/msg/Bool "{data: true}" || true

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
pkill -f "command_mux_node" || true
pkill -f "vision_nav_debug.launch.py" || true
pkill -f "real_line_tracker_node" || true
pkill -f "line_follower_node" || true
pkill -f "line_course_mission_node" || true
pkill -f "realsense2_camera" || true
pkill -f "rqt_image_view" || true
pkill -f "image_view" || true

tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "RK line system stopped."
exit 0
