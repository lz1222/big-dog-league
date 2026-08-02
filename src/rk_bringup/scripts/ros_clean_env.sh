#!/bin/bash
# 此脚本可从源码树或 install 树被 source；工作区必须由脚本位置确定，
# 不能假定开发者的 HOME 下存在固定英文目录。
_RK_CLEAN_ENV_ERREXIT_SET=0
case "$-" in
    *e*) _RK_CLEAN_ENV_ERREXIT_SET=1 ;;
esac
_RK_CLEAN_ENV_NOUNSET_SET=0
case "$-" in
    *u*) _RK_CLEAN_ENV_NOUNSET_SET=1 ;;
esac
set -e
# ROS Foxy/Humble setup.bash 会读取可选环境变量，不能在 nounset 下直接 source。
# 记录并在返回调用脚本前恢复其原始 shell 选项。
set +u

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"

resolve_workspace_dir() {
    local candidate

    if [ -n "${RK_INSPECTION_WS:-}" ]; then
        cd -- "$RK_INSPECTION_WS" && pwd -P
        return
    fi

    # 源码路径为 <ws>/src/rk_bringup/scripts；实体安装路径为
    # <ws>/install/rk_bringup/share/rk_bringup/scripts。
    for candidate in "$SCRIPT_DIR/../../.." "$SCRIPT_DIR/../../../../.."; do
        candidate="$(cd -- "$candidate" 2>/dev/null && pwd -P || true)"
        if [ -n "$candidate" ] \
            && { [ -d "$candidate/src/rk_bringup" ] \
                || [ -d "$candidate/install/rk_bringup" ]; }; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done

    echo "ERROR: cannot infer workspace from ${SCRIPT_PATH}; set RK_INSPECTION_WS." >&2
    return 1
}

WORKSPACE_DIR="$(resolve_workspace_dir)" || exit 1
export RK_INSPECTION_WS="$WORKSPACE_DIR"

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
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "WARN: workspace overlay not found: $WORKSPACE_DIR/install/setup.bash" >&2
    echo "WARN: run colcon build --symlink-install in $WORKSPACE_DIR before starting RK nodes." >&2
fi

# 验收可指定独立 install overlay。必须在工作区默认 overlay 之后 source，
# 让当前测试代码优先于旧 install；未显式指定时绝不猜测临时目录。
if [ -n "${RK_ROS_OVERLAY_SETUP:-}" ]; then
    if [ ! -f "$RK_ROS_OVERLAY_SETUP" ]; then
        echo "ERROR: RK_ROS_OVERLAY_SETUP is missing: ${RK_ROS_OVERLAY_SETUP}" >&2
        return 1 2>/dev/null || exit 1
    fi
    source "$RK_ROS_OVERLAY_SETUP"
fi

# 默认保持机器人既有 Domain 10；软件验收可显式设 RK_ROS_DOMAIN_ID 隔离
# DDS 图，防止测试 publisher 与现场机器人进程互相可见。
export ROS_DOMAIN_ID="${RK_ROS_DOMAIN_ID:-10}"

# 添加 SDK helper/server 到 PATH，让 resolve_sdk_executable 的 which 查找生效。
_RK_SDK_BIN_DIR="${WORKSPACE_DIR}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge"
if [ -d "$_RK_SDK_BIN_DIR" ]; then
    case ":$PATH:" in
        *:"$_RK_SDK_BIN_DIR":*) ;;
        *) export PATH="$_RK_SDK_BIN_DIR:$PATH" ;;
    esac
fi
unset _RK_SDK_BIN_DIR

remove_ld_path_entry "/usr/local/lib"
remove_ld_path_entry "/home/unitree/cyclonedds_ws/install/cyclonedds/lib"

if [ "$_RK_CLEAN_ENV_ERREXIT_SET" -eq 0 ]; then
    set +e
fi
if [ "$_RK_CLEAN_ENV_NOUNSET_SET" -eq 1 ]; then
    set -u
else
    set +u
fi
unset ROS_SETUP
unset _RK_CLEAN_ENV_ERREXIT_SET
unset _RK_CLEAN_ENV_NOUNSET_SET
