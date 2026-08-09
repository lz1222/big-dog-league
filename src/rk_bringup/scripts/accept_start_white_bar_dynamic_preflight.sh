#!/bin/bash
# START 白横杆动态验收的零运动 preflight：验证所有链路，绝不发布 /control/line_cmd。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
WORKSPACE_DIR="$(cd -- "${RK_INSPECTION_WS:-${SCRIPT_DIR}/../../..}" && pwd -P)"
START_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/start_non_arm_competition.sh"
STOP_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/stop_line_system.sh"
ENV_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
OBSERVER="${WORKSPACE_DIR}/src/rk_bringup/scripts/dynamic_preflight_observer.py"
FOLLOWER_START="${WORKSPACE_DIR}/src/rk_bringup/scripts/validation_line_follow_start.py"
ZERO_OBSERVER="${WORKSPACE_DIR}/src/rk_bringup/scripts/non_arm_smoke_observer.py"
CYCLES="${RK_DYNAMIC_PREFLIGHT_CYCLES:-3}"
IDLE_SEC="${RK_DYNAMIC_PREFLIGHT_IDLE_SEC:-35}"
RUN_ROOT="${RK_DYNAMIC_PREFLIGHT_RUN_ROOT:-/home/unitree/rk_validation_data/start_white_bar_dynamic_preflight_$(date +%Y%m%d_%H%M%S)}"
ACTIVE_CYCLE=0

sdk_server_process_count() {
    pgrep -fc '^/[^ ]*/go2_sdk_udp_server( |$)' 2>/dev/null || true
}

udp_forwarder_process_count() {
    pgrep -fc '^([^ ]*/)?python3 [^ ]*/cmd_vel_udp_forwarder[.]py( |$)' \
        2>/dev/null || true
}

udp_listener_count() {
    local port="$1"
    ss -H -lun | awk -v port="$port" \
        '$4 ~ (":" port "$") {count += 1} END {print count + 0}'
}

cleanup_complete() {
    [ "$(sdk_server_process_count)" -eq 0 ] \
        && [ "$(udp_forwarder_process_count)" -eq 0 ] \
        && [ "$(udp_listener_count 15001)" -eq 0 ] \
        && [ "$(udp_listener_count 15002)" -eq 0 ] \
        && ! tmux has-session -t "$RK_COMPETITION_TMUX_SESSION" 2>/dev/null
}

cleanup_cycle() {
    local saved_status=$?
    if [ "$ACTIVE_CYCLE" -ne 0 ]; then
        set +e
        "$STOP_SCRIPT"
        local stop_status=$?
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            cleanup_complete && break
            sleep 1
        done
        if [ "$stop_status" -ne 0 ] || ! cleanup_complete; then
            saved_status=1
        fi
        ACTIVE_CYCLE=0
        set -e
    fi
    return "$saved_status"
}

on_exit() {
    local saved_status=$?
    cleanup_cycle || saved_status=1
    exit "$saved_status"
}
trap on_exit EXIT INT TERM

readiness_passes() {
    local output_file="$1"
    timeout 10s ros2 service call /competition/check_readiness \
        std_srvs/srv/Trigger '{}' > "$output_file" 2>&1
    grep -Eq 'success[=:][[:space:]]*(true|True)' "$output_file"
    grep -Fq 'SDK_MOTION_BACKEND_READY' "$output_file"
}

validation_json_passes() {
    local file="$1"
    python3 - "$file" <<'PY'
import json
import sys

text = open(sys.argv[1], encoding='utf-8').read().strip()
payload_text = text.split(' ', 1)[1] if ' ' in text else ''
payload = json.loads(payload_text)
raise SystemExit(0 if payload.get('success') is True else 1)
PY
}

control_inhibit_gate() {
    local line_info
    local mission_info
    line_info="$(timeout 8s ros2 topic info /control/line_cmd)"
    mission_info="$(timeout 8s ros2 topic info /mission/start)"
    # 动态 preflight 没有 adapter，正式 mission start 也没有 publisher；这是
    # follower 产生 suggested candidate 时仍无法抵达机器人控制面的硬隔离证据。
    grep -Eq 'Publisher count:[[:space:]]*0' <<<"$line_info"
    grep -Eq 'Publisher count:[[:space:]]*0' <<<"$mission_info"
    printf '%s\n%s\n' "$line_info" "$mission_info"
}

if ! [[ "$CYCLES" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: RK_DYNAMIC_PREFLIGHT_CYCLES must be a positive integer." >&2
    exit 1
fi
for required in "$START_SCRIPT" "$STOP_SCRIPT" "$ENV_SCRIPT" "$OBSERVER" \
        "$FOLLOWER_START" "$ZERO_OBSERVER"; do
    [ -x "$required" ] || {
        echo "ERROR: required preflight executable is missing: $required" >&2
        exit 1
    }
done

mkdir -p "$RUN_ROOT"
RUN_ROOT="$(cd -- "$RUN_ROOT" && pwd -P)"
export RK_INSPECTION_WS="$WORKSPACE_DIR"
export RK_ROS_DOMAIN_ID=10
source "$ENV_SCRIPT"

# 这些值是冻结的 zero-motion 3/3 验收记录，不向正式启动脚本引入便利默认值。
export RK_COMPETITION_CONTROL_PLANE_NETWORK_TIMEOUT_SEC=10
export RK_COMPETITION_CONTROL_PLANE_PING_COUNT=3
export RK_COMPETITION_CONTROL_PLANE_PING_POLL_SEC=0.2
export RK_COMPETITION_CONTROL_PLANE_DDS_TIMEOUT_SEC=10
    export RK_COMPETITION_CONTROL_PLANE_REQUIRED_FRAMES=200
export RK_COMPETITION_CONTROL_PLANE_MAX_FRAME_GAP_MS=20
# 真机 preflight 已由现场确认静止/支撑，显式允许 prearm 仅在需要时恢复控制权。
export RK_COMPETITION_ENABLE_MOTION_CONTROL_RECOVERY=true
    # 仅在 preflight 显式加入的被动窗口；不构造 SDK client，也不发布任何命令。
    export RK_COMPETITION_SDK_STARTUP_SETTLE_SEC=3

for cycle in $(seq 1 "$CYCLES"); do
    CYCLE_ROOT="${RUN_ROOT}/cycle_${cycle}"
    mkdir -p "$CYCLE_ROOT"
    export RK_COMPETITION_RUNTIME_DIR="${CYCLE_ROOT}/runtime"
    export RK_COMPETITION_LOG_DIR="${CYCLE_ROOT}/logs"
    export RK_COMPETITION_TMUX_SESSION="rk_dynamic_preflight_${$}_${cycle}"
    export RK_LINE_RUNTIME_DIR="${CYCLE_ROOT}/line_unused"
    export RK_LINE_TMUX_SESSION="rk_dynamic_preflight_line_${$}_${cycle}"
    export RK_COMPETITION_HARDWARE_MODE=true
    export RK_COMPETITION_SOFTWARE_SMOKE_MODE=false
    export RK_COMPETITION_START_SDK_SERVER=true
    export RK_COMPETITION_START_UDP_FORWARDER=true
    export RK_COMPETITION_START_LINE_CAMERA=true
    export RK_COMPETITION_LINE_FOLLOWER_START_TOPIC=/validation/line_follow/start
    ACTIVE_CYCLE=1

    "$START_SCRIPT" 2>&1 | tee "${CYCLE_ROOT}/formal_start.txt"
    INSTANCE_ID="$(tr -d '\r\n' < "${RK_COMPETITION_RUNTIME_DIR}/server_instance_id")"
    grep -Fq 'MOTION_CONTROL_PREARM event=RESULT classification=PASS ready=true' \
        "${RK_COMPETITION_LOG_DIR}/motion_control_prearm.log"
    grep -Fq '"event":"STARTUP_STOP"' \
        "${RK_COMPETITION_LOG_DIR}/startup_status_gate.log"
    grep -Fq '"ret":0' "${RK_COMPETITION_LOG_DIR}/startup_status_gate.log"
    readiness_passes "${CYCLE_ROOT}/readiness_startup.txt"

    # 规格要求连续 10 Hz；0.8 s 仅排除明显中断，同时容忍 USB/UVC
    # 单帧调度抖动。每轮仍完整记录最大间隔，绝不据此掩盖频率不足。
    "$OBSERVER" --duration-sec 5 --discovery-timeout-sec 12 \
        --minimum-rate-hz 10 --maximum-gap-sec 0.8 \
        > "${CYCLE_ROOT}/perception_static.jsonl"
    validation_json_passes "${CYCLE_ROOT}/perception_static.jsonl"

    sleep "$IDLE_SEC"
    readiness_passes "${CYCLE_ROOT}/readiness_idle_${IDLE_SEC}s.txt"
    "$OBSERVER" --duration-sec 5 --discovery-timeout-sec 12 \
        --minimum-rate-hz 10 --maximum-gap-sec 0.8 \
        > "${CYCLE_ROOT}/perception_idle.jsonl"
    validation_json_passes "${CYCLE_ROOT}/perception_idle.jsonl"

    "$OBSERVER" --duration-sec 5 --discovery-timeout-sec 12 \
        --minimum-rate-hz 10 --maximum-gap-sec 0.8 \
        > "${CYCLE_ROOT}/perception_follower_active.jsonl" &
    OBSERVER_PID=$!
    "$FOLLOWER_START" --topic /validation/line_follow/start \
        --discovery-timeout-sec 10 --observe-sec 5 \
        > "${CYCLE_ROOT}/follower_activation.jsonl"
    wait "$OBSERVER_PID"
    validation_json_passes "${CYCLE_ROOT}/follower_activation.jsonl"
    validation_json_passes "${CYCLE_ROOT}/perception_follower_active.jsonl"
    readiness_passes "${CYCLE_ROOT}/readiness_follower_active.txt"
    control_inhibit_gate > "${CYCLE_ROOT}/control_inhibit_gate.txt"
    timeout 10s python3 "$ZERO_OBSERVER" /navigation/cmd_vel --twist \
        --consecutive-zero-count 3 --timeout-sec 8 \
        > "${CYCLE_ROOT}/final_cmd_zero.txt"
    if ! rg -Fq '"sdk_move_count":0' "${CYCLE_ROOT}/follower_activation.jsonl" \
            || ! rg -Fq '"sdk_error_count":0' "${CYCLE_ROOT}/follower_activation.jsonl"; then
        echo "ERROR: preflight observed MOVE or SDK_ERROR." >&2
        exit 1
    fi
    if [ "$(sdk_server_process_count)" -ne 1 ] \
            || [ "$(udp_forwarder_process_count)" -ne 1 ] \
            || [ "$(udp_listener_count 15001)" -ne 1 ] \
            || [ "$(udp_listener_count 15002)" -ne 1 ]; then
        echo "ERROR: preflight process or UDP endpoint gate failed." >&2
        exit 1
    fi

    "$STOP_SCRIPT" 2>&1 | tee "${CYCLE_ROOT}/formal_stop.txt"
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        cleanup_complete && break
        sleep 1
    done
    cleanup_complete
    ACTIVE_CYCLE=0
    printf 'DYNAMIC_PREFLIGHT_CYCLE cycle=%s result=PASS instance=%s\n' \
        "$cycle" "$INSTANCE_ID" | tee "${CYCLE_ROOT}/result.txt"
done

trap - EXIT INT TERM
printf 'DYNAMIC_ACCEPTANCE_PREFLIGHT result=PASS cycles=%s run_root=%s\n' \
    "$CYCLES" "$RUN_ROOT"
