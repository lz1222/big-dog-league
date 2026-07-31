#!/bin/bash
# 停止正式巡线/非机械臂比赛链：先任务取消和 mux 归零，再终止进程。
set +e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
LINE_RUNTIME_DIR="${RK_LINE_RUNTIME_DIR:-$HOME/rk_line_runtime}"
COMPETITION_RUNTIME_DIR="${RK_COMPETITION_RUNTIME_DIR:-$HOME/rk_non_arm_competition_runtime}"
LINE_SESSION="${RK_LINE_TMUX_SESSION:-rk_line}"
COMPETITION_SESSION="${RK_COMPETITION_TMUX_SESSION:-rk_non_arm_competition}"
ESTOP_SERVICE="${RK_COMPETITION_ESTOP_SERVICE:-/safety/estop}"
FALLBACK_USED=0

resolve_workspace_dir() {
    local candidate

    if [ -n "${RK_INSPECTION_WS:-}" ]; then
        cd -- "$RK_INSPECTION_WS" && pwd -P
        return
    fi

    # 停止入口也必须可从源码或实体 install 树调用，避免清理时路径失配。
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

resolve_env_script() {
    local colocated_script="${SCRIPT_DIR}/ros_clean_env.sh"
    local source_script="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
    local install_script="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/ros_clean_env.sh"

    if [ -f "$colocated_script" ]; then
        printf '%s\n' "$colocated_script"
        return 0
    fi
    if [ -f "$source_script" ]; then
        printf '%s\n' "$source_script"
        return 0
    fi
    if [ -f "$install_script" ]; then
        printf '%s\n' "$install_script"
        return 0
    fi
    return 1
}

resolve_companion_script() {
    local filename="$1"
    local candidate

    for candidate in \
        "${SCRIPT_DIR}/${filename}" \
        "${WORKSPACE_DIR}/src/rk_bringup/scripts/${filename}" \
        "${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/${filename}"; do
        if [ -f "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

is_zero_twist_sample() {
    awk '
        BEGIN { section=""; seen=0; ok=1 }
        /^linear:/ { section="linear"; next }
        /^angular:/ { section="angular"; next }
        /^[[:space:]]+[xyz]:/ {
            if (section != "linear" && section != "angular") next
            seen += 1
            value = $2
            if (value !~ /^[-+]?[0-9]+([.][0-9]*)?([eE][-+]?[0-9]+)?$/ \
                || (value + 0.0) != 0.0) ok = 0
        }
        END { exit !(seen == 6 && ok == 1) }
    '
}

wait_for_mux_zero() {
    local count=0
    local sample
    local attempt

    for attempt in 1 2 3 4 5 6 7 8 9; do
        sample="$(timeout 2s ros2 topic echo --once /navigation/cmd_vel \
            2>/dev/null)"
        if [ -n "$sample" ] && printf '%s\n' "$sample" | is_zero_twist_sample; then
            count=$((count + 1))
            if [ "$count" -ge 3 ]; then
                echo "Verified three consecutive command_mux zero outputs."
                return 0
            fi
        else
            count=0
        fi
    done
    return 1
}

call_mux_estop() {
    local service_type
    local response

    service_type="$(timeout 3s ros2 service type "$ESTOP_SERVICE" 2>/dev/null)"
    if [ "$service_type" != "std_srvs/srv/SetBool" ]; then
        return 1
    fi
    response="$(timeout 5s ros2 service call "$ESTOP_SERVICE" \
        std_srvs/srv/SetBool '{data: true}' 2>&1)" || return 1
    printf '%s\n' "$response"
    printf '%s\n' "$response" | grep -Eq 'success[=:][[:space:]]*(true|True)'
}

emergency_cmd_vel_fallback() {
    echo "!!!!!!!!!!!!!!!! EMERGENCY FALLBACK !!!!!!!!!!!!!!!!" >&2
    echo "Normal command_mux estop/zero verification failed." >&2
    echo "Publishing ONE direct zero Twist; this bypasses normal ownership." >&2
    echo "This fallback is NOT a software or hardware acceptance pass." >&2
    echo "!!!!!!!!!!!!!!!! EMERGENCY FALLBACK !!!!!!!!!!!!!!!!" >&2
    FALLBACK_USED=1
    timeout 3s ros2 topic pub --once /navigation/cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' || true
}

stop_pid_file() {
    local pid_file="$1"
    local name
    local pid
    local log_file

    [ -f "$pid_file" ] || return 0
    while IFS='|' read -r name pid log_file; do
        if [ -n "$pid" ] && [[ "$pid" =~ ^[0-9]+$ ]] \
            && kill -0 "$pid" 2>/dev/null; then
            echo "Stopping ${name} pid=${pid}"
            kill "$pid" 2>/dev/null || true
        fi
    done < "$pid_file"
    sleep 1
    while IFS='|' read -r name pid log_file; do
        if [ -n "$pid" ] && [[ "$pid" =~ ^[0-9]+$ ]] \
            && kill -0 "$pid" 2>/dev/null; then
            echo "Force stopping ${name} pid=${pid}"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done < "$pid_file"
    rm -f "$pid_file"
}

ENV_SCRIPT="$(resolve_env_script)"
if [ -n "$ENV_SCRIPT" ]; then
    source "$ENV_SCRIPT" || echo "WARN: ROS environment source failed." >&2
else
    echo "WARN: ROS environment script is unavailable." >&2
fi

# 顺序要求 1--3：mission_stop 内部先发 stop，再等待白横线/检查动作终止。
MISSION_STOP_SCRIPT="$(resolve_companion_script mission_stop.sh || true)"
if [ -x "$MISSION_STOP_SCRIPT" ]; then
    "$MISSION_STOP_SCRIPT" || echo "WARN: mission_stop reported incomplete cleanup." >&2
else
    timeout 3s ros2 topic pub --once /mission/stop std_msgs/msg/Bool \
        '{data: true}' || true
fi

# 顺序要求 4--5：即使 mission_stop 已调用，也重复幂等 estop 并复核 mux 输出。
if ! call_mux_estop || ! wait_for_mux_zero; then
    emergency_cmd_vel_fallback
fi

# 仅在最终命令已由 mux 归零（或明确标记 emergency fallback）后终止进程。
stop_pid_file "${COMPETITION_RUNTIME_DIR}/pids"
stop_pid_file "${LINE_RUNTIME_DIR}/pids"

# 仅清理本启动入口创建的固定 session；禁止按名称 pkill，以免影响其他 ROS 图。
tmux kill-session -t "$COMPETITION_SESSION" 2>/dev/null || true
tmux kill-session -t "$LINE_SESSION" 2>/dev/null || true

if [ "$FALLBACK_USED" -ne 0 ]; then
    echo "RK system stopped via emergency fallback; acceptance remains FAILED." >&2
    exit 1
fi
echo "RK line/non-arm competition system stopped through normal mux ownership."
exit 0
