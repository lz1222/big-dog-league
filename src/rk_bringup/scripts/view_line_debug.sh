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

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: view_line_debug.sh must run in a VNC graphical desktop terminal." >&2
    echo "Pure SSH without a graphical display cannot open rqt_image_view." >&2
    exit 1
fi

echo "Opening rqt_image_view. Select one of these topics in the GUI:"
echo "  /camera/color/image_raw"
echo "  /perception/debug/line_overlay"
echo "  /perception/debug/line_mask"

ros2 run rqt_image_view rqt_image_view
