#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"
ROUTE_FILE="${RK_KEYBOARD_ROUTE_FILE:-$HOME/rk_keyboard_routes/latest_route.json}"

SPEED_SCALE="${RK_KEYBOARD_REPLAY_SPEED_SCALE:-1.0}"
DURATION_SCALE="${RK_KEYBOARD_REPLAY_DURATION_SCALE:-1.0}"
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

if [ ! -f "$ROUTE_FILE" ]; then
    echo "ERROR: route file not found: ${ROUTE_FILE}" >&2
    exit 1
fi

cat <<EOF
Replaying keyboard route.

Before this, start the bridge/line stack in another terminal:
  ${WORKSPACE_DIR}/src/rk_bringup/scripts/start_line_system.sh

route_file=${ROUTE_FILE}
speed_scale=${SPEED_SCALE}
duration_scale=${DURATION_SCALE}
sdk_interface=${SDK_INTERFACE}
EOF

exec ros2 run rk_tools keyboard_route_replay --ros-args \
    -p route_file:="${ROUTE_FILE}" \
    -p speed_scale:="${SPEED_SCALE}" \
    -p duration_scale:="${DURATION_SCALE}" \
    -p sdk_network_interface:="${SDK_INTERFACE}"
