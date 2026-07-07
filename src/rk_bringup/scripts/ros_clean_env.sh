#!/bin/bash
_RK_CLEAN_ENV_ERREXIT_SET=0
case "$-" in
    *e*) _RK_CLEAN_ENV_ERREXIT_SET=1 ;;
esac
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
    echo "Checked active ROS_DISTRO, then foxy, then humble." >&2
    return 1
}

remove_ld_path_entry() {
    local remove_path="$1"
    local new_path=""
    local entry
    local old_ifs="$IFS"

    IFS=":"
    for entry in ${LD_LIBRARY_PATH:-}; do
        if [ -n "$entry" ] && [ "$entry" != "$remove_path" ]; then
            if [ -z "$new_path" ]; then
                new_path="$entry"
            else
                new_path="${new_path}:${entry}"
            fi
        fi
    done
    IFS="$old_ifs"

    export LD_LIBRARY_PATH="$new_path"
}

cd "$WORKSPACE_DIR"
ROS_SETUP="$(select_ros_setup)"
source "$ROS_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"

export ROS_DOMAIN_ID=10

remove_ld_path_entry "/usr/local/lib"
remove_ld_path_entry "/home/unitree/cyclonedds_ws/install/cyclonedds/lib"

if [ "$_RK_CLEAN_ENV_ERREXIT_SET" -eq 0 ]; then
    set +e
fi
unset ROS_SETUP
unset _RK_CLEAN_ENV_ERREXIT_SET
