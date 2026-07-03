#!/bin/bash
set -e

WORKSPACE_DIR="${RK_INSPECTION_WS:-$HOME/rk_inspection_ws}"

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
source /opt/ros/foxy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export ROS_DOMAIN_ID=10

remove_ld_path_entry "/usr/local/lib"
remove_ld_path_entry "/home/unitree/cyclonedds_ws/install/cyclonedds/lib"
