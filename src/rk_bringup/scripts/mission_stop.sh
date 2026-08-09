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
COMPETITION_RUNTIME_DIR="${RK_COMPETITION_RUNTIME_DIR:-$HOME/rk_non_arm_competition_runtime}"

load_action_status_contract() {
    # cleanup 以 start 脚本冻结的 profile/图文件为准；缺失或非法时回退生产
    # 严格合同，绝不能因调用者临时环境变量放宽 action terminal 证明。
    local profile=production
    local start_mission_nodes=true
    if [ -r "${COMPETITION_RUNTIME_DIR}/readiness_profile" ]; then
        profile="$(tr -d '\r\n' < "${COMPETITION_RUNTIME_DIR}/readiness_profile")"
    fi
    if [ -r "${COMPETITION_RUNTIME_DIR}/start_mission_nodes" ]; then
        start_mission_nodes="$(tr -d '\r\n' < "${COMPETITION_RUNTIME_DIR}/start_mission_nodes")"
    fi
    if [ "$profile" = isolated_line_validation ] \
            && [ "$start_mission_nodes" = false ]; then
        ACTION_STATUS_REQUIRED=0
        ACTION_STATUS_SKIP_REASON="SKIPPED_BY_PROFILE:isolated_line_validation"
        return 0
    fi
    ACTION_STATUS_REQUIRED=1
    ACTION_STATUS_SKIP_REASON=""
    if [ "$profile" != production ] || [ "$start_mission_nodes" != true ]; then
        echo "WARN: invalid cleanup profile contract; requiring production action proof." >&2
    fi
}

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
TOPIC_OBSERVER="${RK_COMPETITION_TOPIC_OBSERVER:-${WORKSPACE_DIR}/src/rk_bringup/scripts/non_arm_smoke_observer.py}"
if [ ! -x "$TOPIC_OBSERVER" ]; then
    echo "ERROR: native read-only topic observer is unavailable: ${TOPIC_OBSERVER}" >&2
    exit 1
fi

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
        sample="$(timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" python3 "$TOPIC_OBSERVER" \
            "$topic_name" --once --dump \
            --timeout-sec "$TOPIC_SAMPLE_TIMEOUT_SEC" 2>/dev/null || true)"
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
    # 单一 rclpy 订阅跨越三个样本，避免每次启动 ros2 CLI 都重新等待发现。
    if timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" python3 "$TOPIC_OBSERVER" \
            /navigation/cmd_vel --twist --consecutive-zero-count 3 \
            --timeout-sec "$TOPIC_SAMPLE_TIMEOUT_SEC" >/dev/null 2>&1; then
        echo "Verified three consecutive command_mux zero outputs."
        return 0
    fi
    echo "ERROR: command_mux zero output was not continuously observed." >&2
    return 1
}

ENV_SCRIPT="$(resolve_env_script)" || exit 1
source "$ENV_SCRIPT" || exit 1
load_action_status_contract

# stop 必须先送达所有任务状态机；即使 topic 当前无订阅者也保持幂等返回。
timeout 4s ros2 topic pub --once /mission/stop std_msgs/msg/Bool \
    '{data: true}' || echo "WARN: /mission/stop publish did not confirm a subscriber." >&2

# 没有 mux 表明比赛图尚未启动或已完整退出；此时不存在可由本脚本等待的
# 最终速度链，停止保持幂等 no-op，避免无节点场景被状态采样超时误判为失败。
if ! ros2 node list 2>/dev/null | grep -Eq '(^|/)command_mux_node$'; then
    echo "INFO: command_mux_node is absent; stop is a safe idempotent no-op."
    exit 0
fi

# 急停和最终零速必须先于 action status 观测。ROS 图正在衰退时，等待一个
# 已经停止发布的 IDLE 状态不能延迟真正的停车，也不能把安全资源归零误判失败。
if ! rk_call_mux_estop mission_stop_primary; then
    echo "ERROR: normal stop could not enable command_mux estop." >&2
    exit 1
fi
if ! wait_for_continuous_mux_zero; then
    exit 1
fi

# action 状态只用于诊断。mission/estop 已送达且 mux 已连续归零时，图关闭
# 造成的末条 IDLE 缺失不能逆转停车成功；stop_line_system 会继续验证进程/端口。
if [ "$ACTION_STATUS_REQUIRED" -eq 1 ]; then
    wait_for_action_terminal /mission/white_bar_action_status "white-bar Action" \
        || echo "WARN: white-bar Action terminal proof unavailable after safe stop." >&2
    wait_for_action_terminal /mission/inspection_action_status "inspection Action" \
        || echo "WARN: inspection Action terminal proof unavailable after safe stop." >&2
else
    echo "INFO: white-bar Action status ${ACTION_STATUS_SKIP_REASON}."
    echo "INFO: inspection Action status ${ACTION_STATUS_SKIP_REASON}."
fi

echo "Mission stopped: mux estop enabled and final command zero."
exit 0
