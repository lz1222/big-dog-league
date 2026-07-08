#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
ROUTE_FILE="${RK_KEYBOARD_ROUTE_FILE:-$HOME/rk_keyboard_routes/latest_route.json}"

MOTION_SPEED="${RK_KEYBOARD_SPEED:-0.30}"
FORWARD_SPEED="${RK_KEYBOARD_FORWARD_SPEED:-$MOTION_SPEED}"
BACKWARD_SPEED="${RK_KEYBOARD_BACKWARD_SPEED:-$MOTION_SPEED}"
TURN_SPEED="${RK_KEYBOARD_TURN_SPEED:-0.75}"
ACTION_SEC="${RK_KEYBOARD_ACTION_SEC:-1.0}"
MOTION_BACKEND="${RK_KEYBOARD_MOTION_BACKEND:-sdk_direct}"
SDK_VELOCITY_RATE_HZ="${RK_KEYBOARD_SDK_RATE_HZ:-20.0}"
SDK_VELOCITY_STOP_MODE="${RK_KEYBOARD_SDK_STOP_MODE:-move_zero}"
SDK_VELOCITY_STOP_SEC="${RK_KEYBOARD_SDK_STOP_SEC:-0.10}"
FRONT_JUMP_WAIT_SEC="${RK_KEYBOARD_FRONT_JUMP_WAIT_SEC:-2.0}"
FRONT_JUMP_PRE_STOP_SEC="${RK_KEYBOARD_FRONT_JUMP_PRE_STOP_SEC:-2.0}"
RECORD_GAIT_ACTIONS="${RK_KEYBOARD_RECORD_GAIT_ACTIONS:-true}"
RECORD_IDLE_GAPS="${RK_KEYBOARD_RECORD_IDLE_GAPS:-true}"
MIN_IDLE_SEC="${RK_KEYBOARD_MIN_IDLE_SEC:-0.20}"
LINE_FOLLOW_SEC="${RK_KEYBOARD_LINE_FOLLOW_SEC:-3.0}"
LINE_UNTIL_LOST_MAX_SEC="${RK_KEYBOARD_LINE_UNTIL_LOST_MAX_SEC:-30.0}"
SDK_INTERFACE="${RK_SDK_INTERFACE:-eth0}"

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

cat <<EOF
Starting keyboard route recorder.

Before this, start the bridge/line stack in another terminal:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/start_line_system.sh

Controls:
  w forward, s backward, a turn left, d turn right
  each motion key runs once for ${ACTION_SEC}s
  x economic_gait, c normal gait, j front jump, space stop
  l timed line-follow stage (${LINE_FOLLOW_SEC}s)
  u line-follow until lost (max ${LINE_UNTIL_LOST_MAX_SEC}s)
  q finish and save route

route_file=${ROUTE_FILE}
linear_speed=${MOTION_SPEED}
turn_speed=${TURN_SPEED}
action_sec=${ACTION_SEC}
motion_backend=${MOTION_BACKEND}
sdk_velocity_stop_mode=${SDK_VELOCITY_STOP_MODE}
sdk_velocity_stop_sec=${SDK_VELOCITY_STOP_SEC}
front_jump_wait_sec=${FRONT_JUMP_WAIT_SEC}
front_jump_pre_stop_sec=${FRONT_JUMP_PRE_STOP_SEC}
record_gait_actions=${RECORD_GAIT_ACTIONS}
record_idle_gaps=${RECORD_IDLE_GAPS}
min_idle_sec=${MIN_IDLE_SEC}
sdk_interface=${SDK_INTERFACE}
EOF

exec ros2 run rk_tools keyboard_route_recorder --ros-args \
    -p route_file:="${ROUTE_FILE}" \
    -p forward_speed:="${FORWARD_SPEED}" \
    -p backward_speed:="${BACKWARD_SPEED}" \
    -p turn_speed:="${TURN_SPEED}" \
    -p key_action_duration_sec:="${ACTION_SEC}" \
    -p motion_backend:="${MOTION_BACKEND}" \
    -p sdk_velocity_rate_hz:="${SDK_VELOCITY_RATE_HZ}" \
    -p sdk_velocity_stop_mode:="${SDK_VELOCITY_STOP_MODE}" \
    -p sdk_velocity_stop_sec:="${SDK_VELOCITY_STOP_SEC}" \
    -p front_jump_wait_sec:="${FRONT_JUMP_WAIT_SEC}" \
    -p front_jump_pre_stop_sec:="${FRONT_JUMP_PRE_STOP_SEC}" \
    -p record_gait_actions:="${RECORD_GAIT_ACTIONS}" \
    -p record_idle_gaps:="${RECORD_IDLE_GAPS}" \
    -p record_idle_min_duration_sec:="${MIN_IDLE_SEC}" \
    -p line_insert_duration_sec:="${LINE_FOLLOW_SEC}" \
    -p line_until_lost_max_sec:="${LINE_UNTIL_LOST_MAX_SEC}" \
    -p sdk_network_interface:="${SDK_INTERFACE}"
