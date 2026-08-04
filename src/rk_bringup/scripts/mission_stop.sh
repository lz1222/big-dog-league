#!/bin/bash
# 非机械臂任务停止：先取消任务，再由 command_mux 的 estop 归零。
set -u

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
ESTOP_SERVICE="${RK_COMPETITION_ESTOP_SERVICE:-/safety/estop}"
source "${SCRIPT_DIR}/stop_safety_common.sh"
ACTION_CANCEL_TIMEOUT_SEC="${RK_COMPETITION_ACTION_CANCEL_TIMEOUT_SEC:-12}"
# 仅用于 CLI 新订阅读取状态/最终零速度，避免慢机误把状态缺失当作安全完成。
TOPIC_SAMPLE_TIMEOUT_SEC="${RK_COMPETITION_TOPIC_SAMPLE_TIMEOUT_SEC:-6}"

resolve_workspace_dir() {
    local candidate

    if [ -n "${RK_INSPECTION_WS:-}" ]; then
        cd -- "$RK_INSPECTION_WS" && pwd -P
        return
    fi

    # 兼容源码树和实体 install 树；symlink-install 会解析到源码树。
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
    echo "ERROR: ros_clean_env.sh not found in source or install tree." >&2
    return 1
}

json_action_is_terminal() {
    python3 -c '
import ast
import json
import sys

rows = [row.strip() for row in sys.stdin.read().splitlines() if row.strip()]
if not rows:
    raise SystemExit(2)
payload = None
for text in reversed(rows):
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        try:
            value = json.loads(ast.literal_eval(text))
        except (SyntaxError, TypeError, ValueError):
            continue
    if isinstance(value, dict):
        payload = value
        break
if payload is None:
    raise SystemExit(2)
state = str(payload.get("state", payload.get("status", ""))).upper()
active = {
    "ARMED", "WAIT_SERVER", "WAIT_SIGN", "COMMAND_READY", "WAIT_ZERO",
    "GOAL_SENT", "RUNNING", "CANCELING", "CLEANUP_PENDING"
}
if state in active:
    raise SystemExit(1)
if state:
    raise SystemExit(0)
raise SystemExit(2)
'
}

wait_for_action_terminal() {
    local topic_name="$1"
    local label="$2"
    local deadline=$(( $(date +%s) + ACTION_CANCEL_TIMEOUT_SEC ))
    local sample
    local state_result

    while [ "$(date +%s)" -lt "$deadline" ]; do
        sample="$(timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" ros2 topic echo --once "$topic_name" \
            --field data 2>/dev/null || true)"
        if [ -z "$sample" ]; then
            # mux 仍运行时，状态缺失不能证明没有底层 Action；继续等待直到
            # 取消总超时并让正常停止路径失败，而不是过早停止进程。
            echo "WARN: ${label} status unavailable; waiting for proof." >&2
            sleep 0.2
            continue
        fi
        printf '%s' "$sample" | json_action_is_terminal
        state_result=$?
        if [ "$state_result" -eq 0 ]; then
            echo "INFO: ${label} reached terminal state."
            return 0
        fi
        if [ "$state_result" -eq 2 ]; then
            echo "WARN: ${label} status malformed; waiting until timeout." >&2
        fi
        sleep 0.2
    done
    echo "ERROR: ${label} did not reach a terminal state after mission stop." >&2
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

wait_for_continuous_mux_zero() {
    local zero_samples=0
    local attempts=0
    local sample

    while [ "$attempts" -lt 12 ]; do
        attempts=$((attempts + 1))
        sample="$(timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" ros2 topic echo --once /navigation/cmd_vel \
            2>/dev/null || true)"
        if [ -n "$sample" ] && printf '%s\n' "$sample" | is_zero_twist_sample; then
            zero_samples=$((zero_samples + 1))
            if [ "$zero_samples" -ge 3 ]; then
                echo "Verified three consecutive command_mux zero outputs."
                return 0
            fi
        else
            zero_samples=0
        fi
    done
    echo "ERROR: command_mux zero output was not continuously observed." >&2
    return 1
}

ENV_SCRIPT="$(resolve_env_script)" || exit 1
source "$ENV_SCRIPT" || exit 1

# stop 必须先送达所有任务状态机；即使 topic 当前无订阅者也保持幂等返回。
timeout 4s ros2 topic pub --once /mission/stop std_msgs/msg/Bool \
    '{data: true}' || echo "WARN: /mission/stop publish did not confirm a subscriber." >&2

# 没有 mux 表明比赛图尚未启动或已完整退出；此时不存在可由本脚本等待的
# 最终速度链，停止保持幂等 no-op，避免无节点场景被状态采样超时误判为失败。
if ! ros2 node list 2>/dev/null | grep -Eq '(^|/)command_mux_node$'; then
    echo "INFO: command_mux_node is absent; stop is a safe idempotent no-op."
    exit 0
fi

WHITE_OK=0
INSPECTION_OK=0
wait_for_action_terminal /mission/white_bar_action_status "white-bar Action" || WHITE_OK=1
wait_for_action_terminal /mission/inspection_action_status "inspection Action" || INSPECTION_OK=1

if ! rk_call_mux_estop mission_stop_primary; then
    echo "ERROR: normal stop could not enable command_mux estop." >&2
    exit 1
fi
if ! wait_for_continuous_mux_zero; then
    exit 1
fi
if [ "$WHITE_OK" -ne 0 ] || [ "$INSPECTION_OK" -ne 0 ]; then
    exit 1
fi

echo "Mission stopped: actions canceled/terminal, mux estop enabled, final command zero."
exit 0
