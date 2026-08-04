#!/bin/bash
# 非机械臂正式任务只允许在 readiness 通过后发布一次 start。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
READINESS_SERVICE="${RK_COMPETITION_READINESS_SERVICE:-/competition/check_readiness}"
READINESS_TIMEOUT_SEC="${RK_COMPETITION_READINESS_TIMEOUT_SEC:-30}"
START_CONFIRM_TIMEOUT_SEC="${RK_COMPETITION_START_CONFIRM_TIMEOUT_SEC:-20}"
# 双 ACK 交付在路线和循线器都确认前，最多发送有限条可靠 VOLATILE 消息；
# 这是同一个逻辑请求的底层传输补偿，绝不创建第二轮任务。
START_PUBLISH_TIMEOUT_SEC="${RK_COMPETITION_START_PUBLISH_TIMEOUT_SEC:-20}"
START_MIN_SUBSCRIBERS="${RK_COMPETITION_START_MIN_SUBSCRIBERS:-2}"
START_MAX_TRANSPORT_PUBLISHES="${RK_COMPETITION_START_MAX_TRANSPORT_PUBLISHES:-3}"
START_RETRANSMIT_INTERVAL_SEC="${RK_COMPETITION_START_RETRANSMIT_INTERVAL_SEC:-0.35}"

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

safe_stop_after_start_failure() {
    local stop_script

    stop_script="$(resolve_companion_script mission_stop.sh || true)"

    echo "ERROR: start confirmation failed; requesting safe mission stop." >&2
    if [ -x "$stop_script" ]; then
        timeout 20s "$stop_script" || true
        return 0
    fi
    timeout 3s ros2 topic pub --once /mission/stop std_msgs/msg/Bool \
        '{data: true}' || true
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

readiness_passes() {
    local response

    # mission_start 是正式唯一 start 入口，必须先等待真实 Trigger 服务回应。
    # 短生命周期 ros2 CLI 在高负载 VM 上可能刚完成图发现就销毁上下文；此处
    # 使用同一服务类型的有界 rclpy 客户端，只提高发现可靠性，不降低门控条件。
    if ! response="$(timeout "${READINESS_TIMEOUT_SEC}s" python3 - \
        "$READINESS_SERVICE" "$READINESS_TIMEOUT_SEC" <<'PY'
import json
import os
import sys
import time

import rclpy
from std_srvs.srv import Trigger

service_name = sys.argv[1]
timeout_sec = float(sys.argv[2])
rclpy.init()
node = rclpy.create_node('competition_start_readiness_{}'.format(os.getpid()))
try:
    client = node.create_client(Trigger, service_name)
    deadline = time.monotonic() + max(0.1, timeout_sec - 1.0)
    while not client.wait_for_service(timeout_sec=0.25):
        if time.monotonic() >= deadline:
            raise RuntimeError('readiness_service_unavailable')
    future = client.call_async(Trigger.Request())
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise RuntimeError('readiness_service_response_timeout')
    result = future.result()
    if result is None:
        raise RuntimeError('readiness_service_empty_response')
    payload = json.loads(result.message)
    print(json.dumps(payload, ensure_ascii=True, separators=(',', ':')))
    raise SystemExit(0 if result.success and payload.get('success') is True else 1)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
)"; then
        [ -n "$response" ] && printf '%s\n' "$response"
        echo "ERROR: readiness service call timed out, failed, or rejected start." >&2
        return 1
    fi
    printf '%s\n' "$response"
    printf '%s\n' "$response" | grep -Eq '"success":true'
}

deliver_formal_start_with_dual_ack() {
    # 不用固定 sleep 猜测 DDS 是否送达：交付器订阅路线和循线状态，只有双 ACK
    # 同时成立才成功。输出 JSON 供验收记录逻辑请求数、传输数和唯一 run_id。
    PYTHONPATH="${WORKSPACE_DIR}/src/rk_bringup${PYTHONPATH:+:${PYTHONPATH}}" \
        timeout "${START_CONFIRM_TIMEOUT_SEC}s" python3 -m \
        rk_bringup.mission_start_delivery \
        --timeout-sec "$START_CONFIRM_TIMEOUT_SEC" \
        --min-subscribers "$START_MIN_SUBSCRIBERS" \
        --max-transport-publishes "$START_MAX_TRANSPORT_PUBLISHES" \
        --retransmit-interval-sec "$START_RETRANSMIT_INTERVAL_SEC"
}

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"

if ! readiness_passes; then
    # readiness 失败时没有发布 start，因此不能把此路径报告成任务已启动。
    exit 1
fi

if ! deliver_formal_start_with_dual_ack; then
    echo "ERROR: mission start dual-ACK delivery failed; no unconfirmed task start is accepted." >&2
    # 已发送但未获得双 ACK 时不能假定两个消费者状态一致，主动请求安全停止。
    safe_stop_after_start_failure
    exit 1
fi
echo "Mission start accepted once; route and follower dual ACK established."
