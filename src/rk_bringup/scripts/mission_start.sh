#!/bin/bash
# 非机械臂正式任务只允许在 readiness 通过后发布一次 start。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
READINESS_SERVICE="${RK_COMPETITION_READINESS_SERVICE:-/competition/check_readiness}"
READINESS_TIMEOUT_SEC="${RK_COMPETITION_READINESS_TIMEOUT_SEC:-30}"
START_CONFIRM_TIMEOUT_SEC="${RK_COMPETITION_START_CONFIRM_TIMEOUT_SEC:-20}"
# 一次性 publisher 必须等到关键任务订阅者完成 DDS 发现；VM 冷启动下
# 5 秒不足会造成“未发送 start”的假失败。此上限只约束发布准备，不增加消息次数。
START_PUBLISH_TIMEOUT_SEC="${RK_COMPETITION_START_PUBLISH_TIMEOUT_SEC:-20}"
START_MIN_SUBSCRIBERS="${RK_COMPETITION_START_MIN_SUBSCRIBERS:-2}"
# 冷启动的 ROS CLI 订阅建立可能慢于一个控制周期；只延长只读状态采样。
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

publish_formal_start_once() {
    # 只在至少两个核心消费者（路线和循线）已发现时发布一条 start；发布后
    # 保持 publisher 一秒以完成 DDS 发送。发现不足即失败，不重发第二条命令。
    timeout "${START_PUBLISH_TIMEOUT_SEC}s" python3 - \
        "$START_PUBLISH_TIMEOUT_SEC" "$START_MIN_SUBSCRIBERS" <<'PY'
import os
import sys
import time

import rclpy
from std_msgs.msg import Bool

timeout_sec = float(sys.argv[1])
required_subscribers = int(sys.argv[2])
if required_subscribers < 1:
    raise ValueError('START_MIN_SUBSCRIBERS must be at least one')

rclpy.init()
node = rclpy.create_node('competition_start_publisher_{}'.format(os.getpid()))
try:
    publisher = node.create_publisher(Bool, '/mission/start', 10)
    deadline = time.monotonic() + max(0.1, timeout_sec - 1.1)
    while publisher.get_subscription_count() < required_subscribers:
        if time.monotonic() >= deadline:
            raise RuntimeError(
                'mission_start_subscriber_discovery_timeout_{}'.format(
                    publisher.get_subscription_count()
                )
            )
        rclpy.spin_once(node, timeout_sec=0.1)

    message = Bool(data=True)
    publisher.publish(message)
    # 与旧 CLI 的 keep-alive 等价，但不会额外 publish；确保一次消息有传输窗口。
    drain_deadline = time.monotonic() + 1.0
    while rclpy.ok() and time.monotonic() < drain_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(
        'MISSION_START_PUBLISHED subscriber_count={}'.format(
            publisher.get_subscription_count()
        )
    )
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
}

start_state_confirmed() {
    local sample

    sample="$(timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" ros2 topic echo --once /mission/line_course_state \
        --field data 2>/dev/null || true)"
    if [ -z "$sample" ]; then
        return 1
    fi
    printf '%s' "$sample" | python3 -c '
import ast
import json
import sys

rows = [row.strip() for row in sys.stdin.read().splitlines() if row.strip()]
if not rows:
    raise SystemExit(1)
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
    raise SystemExit(1)
route_phase = str(payload.get("route_phase", "")).strip()
accepted = (
    payload.get("mission_started") is True
    and bool(str(payload.get("run_id", "")).strip())
    # 若视觉输入已就绪，路线可在同一状态采样周期离开 START_STAGE；确认
    # start 只要求 run 已建立且未回到 WAIT_START/故障，不把正常推进误判失败。
    and route_phase not in (
        "",
        "WAIT_START",
        "EMERGENCY_STOP",
        "FAULTED",
    )
)
if accepted:
    print(
        "MISSION_START_STATE run_id={} state={} route_phase={}".format(
            str(payload.get("run_id", "")).strip(),
            str(payload.get("state", "")).strip(),
            route_phase,
        )
    )
raise SystemExit(not accepted)
'
}

ENV_SCRIPT="$(resolve_env_script)"
source "$ENV_SCRIPT"

if ! readiness_passes; then
    # readiness 失败时没有发布 start，因此不能把此路径报告成任务已启动。
    exit 1
fi

if ! publish_formal_start_once; then
    echo "ERROR: failed to publish one /mission/start message." >&2
    exit 1
fi

deadline=$(( $(date +%s) + START_CONFIRM_TIMEOUT_SEC ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    if start_state_confirmed; then
        echo "Mission start accepted once; run_id established and route left WAIT_START."
        exit 0
    fi
    sleep 0.2
done

safe_stop_after_start_failure
exit 1
