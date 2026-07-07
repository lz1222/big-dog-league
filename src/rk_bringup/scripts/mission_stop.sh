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
set +e

STOP_STATUS=0
for _ in 1 2 3; do
    timeout 3s ros2 topic pub --once /mission/stop std_msgs/msg/Bool "{data: true}"
    CURRENT_STATUS=$?
    if [ "$CURRENT_STATUS" -ne 0 ]; then
        STOP_STATUS="$CURRENT_STATUS"
    fi
    sleep 0.2
done

if [ "$STOP_STATUS" -ne 0 ]; then
    echo "WARN: failed to publish /mission/stop." >&2
    exit 1
fi

exit 0
