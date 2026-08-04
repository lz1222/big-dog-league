#!/bin/bash
# 非机械臂正式链的软件 smoke 验收：全程隔离 DDS，不接触机器人或网络 SDK。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
# 包含隔离 build 与完整路线 smoke；可由 CI/操作者按机器性能覆盖。
# 发现、三路只读观测和整条路线各自均有边界，在冷启动/高负载 VM 上它们的
# 合理上界已超过 480 秒；900 秒仍是总进程组的硬上限，避免中途误杀已完成路线。
TOTAL_TIMEOUT_SEC="${RK_ACCEPT_TOTAL_TIMEOUT_SEC:-900}"
# Humble VM 在冷启动时 ros2 CLI 建立 DDS 订阅可能超过 2 秒；这是只读采样
# 上限，不改变任何控制超时，避免把正常状态心跳误判为缺失。
TOPIC_SAMPLE_TIMEOUT_SEC="${RK_ACCEPT_TOPIC_SAMPLE_TIMEOUT_SEC:-6}"
# 观测器必须覆盖 readiness、正式 start 和完整路线；退出仍由 cleanup 主动回收。
TOPIC_OBSERVER_TIMEOUT_SEC="${RK_ACCEPT_TOPIC_OBSERVER_TIMEOUT_SEC:-180}"
if [ "${RK_ACCEPT_WRAPPED:-0}" != "1" ]; then
    exec env RK_ACCEPT_WRAPPED=1 timeout --foreground --kill-after=10s \
        "${TOTAL_TIMEOUT_SEC}s" "$SCRIPT_PATH" "$@"
fi

WORKSPACE_DIR="$(cd -- "${RK_INSPECTION_WS:-${SCRIPT_DIR}/..}" && pwd -P)"
if [ -n "${RK_NON_ARM_ACCEPT_DIR:-}" ]; then
    CHECK_DIR="${RK_NON_ARM_ACCEPT_DIR}"
    CHECK_DIR_CREATED=false
else
    CHECK_DIR="$(mktemp -d /tmp/rk_non_arm_acceptance_XXXXXX)"
    CHECK_DIR_CREATED=true
fi
mkdir -p "$CHECK_DIR"
CHECK_DIR="$(cd -- "$CHECK_DIR" && pwd -P)"
KEEP_CHECK_DIR="${RK_KEEP_NON_ARM_ACCEPT_DIR:-false}"
REQUESTED_OVERLAY_SETUP="${RK_ROS_OVERLAY_SETUP:-}"
export RK_ROS_DOMAIN_ID="${RK_ACCEPT_ROS_DOMAIN_ID:-110}"
export RK_INSPECTION_WS="$WORKSPACE_DIR"
export ROS_LOG_DIR="${CHECK_DIR}/ros_logs"

LAUNCH_PID=""
WATCHDOG_PID=""
MISSION_START_PID=""
TOPIC_OBSERVER_PIDS=()
FAKE_HELPER="${CHECK_DIR}/fake_sdk_motion_helper"
# cleanup guard 的父目录必须独立于可能由调用方以 0775 创建的验收根目录；
# PersistentCleanupGuard 会严格拒绝不安全父目录，smoke 不得放宽该生产规则。
SMOKE_GUARD_DIR="${CHECK_DIR}/cleanup_guard_private"
SMOKE_GUARD="${SMOKE_GUARD_DIR}/front_jump_cleanup_guard.json"
PROCESS_GUARD_FAILURE="${CHECK_DIR}/process_guard_failure.txt"
PROCESS_GUARD_DETAIL="${CHECK_DIR}/process_guard_detail.txt"
FAKE_HELPER_SEEN_FILE="${CHECK_DIR}/fake_helper_seen.txt"
FAKE_HELPER_IDENTITY_FILE="${CHECK_DIR}/fake_helper_proc_identity.txt"
OVERLAY_SETUP=""
OVERLAY_PREFIX_ROOT=""

# 正式 launch 在 smoke 下仍解析这些包；隔离 overlay 必须覆盖全部运行依赖。
FORMAL_BUILD_PACKAGES=(
    rk_interfaces
    rk_perception
    rk_navigation
    rk_mission
    rk_locomotion
    rk_safety
    rk_go2_sdk_bridge
    rk_bringup
)

cleanup() {
    local status=$?

    set +e
    if [ -n "$WATCHDOG_PID" ] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
        kill "$WATCHDOG_PID" 2>/dev/null || true
        wait "$WATCHDOG_PID" 2>/dev/null || true
    fi
    for observer_pid in "${TOPIC_OBSERVER_PIDS[@]}"; do
        if kill -0 "$observer_pid" 2>/dev/null; then
            kill "$observer_pid" 2>/dev/null || true
            wait "$observer_pid" 2>/dev/null || true
        fi
    done
    if [ -n "$MISSION_START_PID" ] \
        && kill -0 "$MISSION_START_PID" 2>/dev/null; then
        kill "$MISSION_START_PID" 2>/dev/null || true
        wait "$MISSION_START_PID" 2>/dev/null || true
    fi
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        timeout 18s "${WORKSPACE_DIR}/src/rk_bringup/scripts/mission_stop.sh" \
            >/dev/null 2>&1 || true
        # launch 在独立 session 中运行，因此只向本次 smoke 的进程组发信号；
        # timeout 中断时也不会遗留同一 DDS 域中的 ROS 子节点。
        kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
        fi
        for _ in 1 2 3; do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
        fi
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    if [ "$KEEP_CHECK_DIR" != "true" ] && [ "$CHECK_DIR_CREATED" = "true" ]; then
        rm -rf "$CHECK_DIR"
    elif [ "$KEEP_CHECK_DIR" = "true" ]; then
        echo "Kept software smoke artifacts: ${CHECK_DIR}" >&2
    fi
    exit "$status"
}
trap cleanup EXIT INT TERM

resolve_env_script() {
    local source_script="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
    local install_script="${WORKSPACE_DIR}/install/rk_bringup/share/rk_bringup/scripts/ros_clean_env.sh"

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

prepare_smoke_guard_dir() {
    # 在 launch 之前一次完成路径、owner、权限和 symlink 校验。gait、
    # readiness 与 acceptance 共享同一个绝对 guard 文件路径，任何异常均
    # fail-closed，不能启动可能锁存 cleanup_guard_fault 的 ROS 图。
    local owner_uid
    local actual_uid
    local actual_mode

    case "$CHECK_DIR" in
        /*) ;;
        *) echo "ERROR: smoke CHECK_DIR is not absolute: ${CHECK_DIR}" >&2; return 1 ;;
    esac
    case "$SMOKE_GUARD_DIR" in
        "${CHECK_DIR}"/*) ;;
        *) echo "ERROR: private guard directory escapes CHECK_DIR." >&2; return 1 ;;
    esac
    if [ -e "$SMOKE_GUARD_DIR" ] || [ -L "$SMOKE_GUARD_DIR" ]; then
        echo "ERROR: private guard directory already exists or is a symlink." >&2
        return 1
    fi
    if ! mkdir "$SMOKE_GUARD_DIR"; then
        echo "ERROR: failed to create private guard directory." >&2
        return 1
    fi
    if ! chmod 0700 "$SMOKE_GUARD_DIR"; then
        echo "ERROR: failed to set private guard directory mode 0700." >&2
        return 1
    fi
    owner_uid="$(id -u)"
    actual_uid="$(stat -c '%u' "$SMOKE_GUARD_DIR" 2>/dev/null || true)"
    actual_mode="$(stat -c '%a' "$SMOKE_GUARD_DIR" 2>/dev/null || true)"
    if [ ! -d "$SMOKE_GUARD_DIR" ] || [ -L "$SMOKE_GUARD_DIR" ] \
        || [ "$actual_uid" != "$owner_uid" ] || [ "$actual_mode" != "700" ]; then
        echo "ERROR: private guard directory verification failed: uid=${actual_uid:-missing} mode=${actual_mode:-missing}." >&2
        return 1
    fi
    if [ -e "$SMOKE_GUARD" ] || [ -L "$SMOKE_GUARD" ]; then
        echo "ERROR: smoke guard file already exists or is a symlink." >&2
        return 1
    fi
    printf '%s\n' "$SMOKE_GUARD" > "${CHECK_DIR}/smoke_cleanup_guard_path.txt"
}

source_selected_overlay() {
    if [ ! -f "$OVERLAY_SETUP" ]; then
        echo "ERROR: selected isolated overlay is missing: ${OVERLAY_SETUP}" >&2
        return 1
    fi
    export RK_ROS_OVERLAY_SETUP="$OVERLAY_SETUP"
    # ros_clean_env 先 source 基础工作区，再 source 该路径，保证正式脚本
    # 与当前 smoke launch 使用同一份隔离 install，而不是陈旧 install。
    source "$ENV_SCRIPT"
}

prepare_isolated_overlay() {
    local package_prefix

    if [ -n "$REQUESTED_OVERLAY_SETUP" ]; then
        if [ ! -f "$REQUESTED_OVERLAY_SETUP" ]; then
            echo "ERROR: requested RK_ROS_OVERLAY_SETUP is missing: " >&2
            echo "  ${REQUESTED_OVERLAY_SETUP}" >&2
            return 1
        fi
        OVERLAY_SETUP="$(readlink -f "$REQUESTED_OVERLAY_SETUP")"
        echo "Using caller-provided isolated overlay: ${OVERLAY_SETUP}"
    else
        if ! command -v colcon >/dev/null 2>&1; then
            echo "ERROR: colcon is required to build the isolated overlay." >&2
            return 1
        fi
        echo "Building isolated formal overlay under ${CHECK_DIR}/install ..."
        colcon --log-base "${CHECK_DIR}/colcon_log" build --symlink-install \
            --build-base "${CHECK_DIR}/build" \
            --install-base "${CHECK_DIR}/install" \
            --packages-select "${FORMAL_BUILD_PACKAGES[@]}"
        OVERLAY_SETUP="${CHECK_DIR}/install/setup.bash"
    fi

    if [ ! -f "$OVERLAY_SETUP" ]; then
        echo "ERROR: isolated overlay setup was not produced: ${OVERLAY_SETUP}" >&2
        return 1
    fi
    OVERLAY_PREFIX_ROOT="$(cd -- "$(dirname -- "$OVERLAY_SETUP")" && pwd -P)"
    source_selected_overlay
    package_prefix="$(ros2 pkg prefix rk_bringup 2>/dev/null || true)"
    case "$package_prefix" in
        "$OVERLAY_PREFIX_ROOT"/*)
            ;;
        *)
            echo "ERROR: rk_bringup did not resolve from isolated overlay." >&2
            echo "  expected under: ${OVERLAY_PREFIX_ROOT}" >&2
            echo "  resolved: ${package_prefix:-missing}" >&2
            return 1
            ;;
    esac
}

proc_cmdline() {
    local process_dir="$1"

    tr '\0' ' ' < "${process_dir}/cmdline" 2>/dev/null || true
}

proc_argv0() {
    local process_dir="$1"
    local argv0=""

    # 仅 argv[0] 等于 fake helper 才是实际 helper 进程；launch 命令本身也会
    # 携带 fake_sdk_action_executable 参数，不能据此误判为 exe 伪装。
    IFS= read -r -d '' argv0 < "${process_dir}/cmdline" || true
    printf '%s' "$argv0"
}

# 只审计本次 ros2 launch 的后代。主机上无关进程即使名称相同也不能造成
# 软件验收误报；反之，任何由本 smoke 启动的真实 SDK/UDP 子进程都会被拦截。
process_is_launch_descendant() {
    local process_dir="$1"
    local process_id="${process_dir##*/}"
    local stat_line
    local stat_tail
    local parent_id
    local hop_count=0

    while [ "$process_id" != "1" ] && [ "$hop_count" -lt 128 ]; do
        if [ "$process_id" = "$LAUNCH_PID" ]; then
            return 0
        fi
        [ -r "/proc/${process_id}/stat" ] || return 1
        stat_line="$(< "/proc/${process_id}/stat")"
        # comm 可包含空格或右括号，截取最后一个 ") " 后才解析 state/ppid。
        stat_tail="${stat_line##*) }"
        set -- $stat_tail
        parent_id="${2:-}"
        [[ "$parent_id" =~ ^[0-9]+$ ]] || return 1
        process_id="$parent_id"
        hop_count=$((hop_count + 1))
    done
    return 1
}

go2_sdk_server_process_running() {
    local process_dir
    local command_line

    for process_dir in /proc/[0-9]*; do
        [ -r "${process_dir}/cmdline" ] || continue
        process_is_launch_descendant "$process_dir" || continue
        command_line="$(proc_cmdline "$process_dir")"
        if [[ "$command_line" == *go2_sdk_udp_server* ]]; then
            printf '%s %s\n' "${process_dir##*/}" "$command_line"
            return 0
        fi
    done
    return 1
}

check_smoke_processes_once() {
    local process_dir
    local command_line
    local executable
    local argv0

    if go2_sdk_server_process_running > "${CHECK_DIR}/go2_sdk_server_seen.txt"; then
        echo "go2_sdk_udp_server process observed during software smoke" >&2
        return 1
    fi
    for process_dir in /proc/[0-9]*; do
        [ -r "${process_dir}/cmdline" ] || continue
        process_is_launch_descendant "$process_dir" || continue
        command_line="$(proc_cmdline "$process_dir")"
        executable="$(readlink -f "${process_dir}/exe" 2>/dev/null || true)"
        argv0="$(proc_argv0 "$process_dir")"
        # 真实 SDK helper 一旦出现即判定 smoke 边界失效；路径相同的假 ELF
        # 则必须能由 /proc/<pid>/exe 精确识别，防止 argv 伪装。
        if [[ "$command_line" == *go2_sdk_motion_action* ]]; then
            echo "real go2_sdk_motion_action process observed: ${process_dir##*/}" >&2
            return 1
        fi
        if [ "$argv0" = "$FAKE_HELPER" ] \
            || [ "$executable" = "$FAKE_HELPER" ]; then
            if [ "$executable" != "$FAKE_HELPER" ]; then
                echo "fake helper argv/exe identity mismatch: ${process_dir##*/}" >&2
                return 1
            fi
            printf '%s %s\n' "${process_dir##*/}" "$executable" \
                > "$FAKE_HELPER_SEEN_FILE"
        fi
    done
    return 0
}

start_smoke_process_watchdog() {
    (
        while kill -0 "$LAUNCH_PID" 2>/dev/null; do
            if ! check_smoke_processes_once > "$PROCESS_GUARD_DETAIL" 2>&1; then
                {
                    echo "software smoke /proc identity guard failed"
                    cat "$PROCESS_GUARD_DETAIL"
                } > "$PROCESS_GUARD_FAILURE"
                exit 1
            fi
            sleep 0.05
        done
    ) &
    WATCHDOG_PID="$!"
}

assert_smoke_process_guard() {
    if [ -f "$PROCESS_GUARD_FAILURE" ]; then
        cat "$PROCESS_GUARD_FAILURE" >&2
        return 1
    fi
    if ! check_smoke_processes_once; then
        return 1
    fi
    # Action helper 生命周期短于安全心跳窗口，不能为了轮询而延长到改变
    # FrontJump 行为。前置 /proc 身份校验验证 ELF，真实 Action 日志再证明
    # 同一路径确实被调用，两者缺一不可。
    if [ ! -s "$FAKE_HELPER_IDENTITY_FILE" ]; then
        echo "ERROR: marked fake helper did not pass /proc exe identity check." >&2
        return 1
    fi
    if ! grep -F '"event": "helper_started"' "${CHECK_DIR}/launch.log" \
        | grep -Fq "$FAKE_HELPER"; then
        echo "ERROR: real gait Action did not start the marked fake helper." >&2
        return 1
    fi
    return 0
}

verify_fake_helper_proc_identity() {
    # 先以最小、无网络的本地进程验证 /proc/<pid>/exe，拒绝 shebang/argv
    # 伪装；正式链路随后仍通过同一绝对路径驱动真实 gait Action。
    local helper_pid
    local observed_exe=""
    local attempt

    RK_FAKE_SDK_SLEEP_MS=500 "$FAKE_HELPER" >/dev/null 2>&1 &
    helper_pid="$!"
    for attempt in $(seq 1 20); do
        observed_exe="$(readlink -f "/proc/${helper_pid}/exe" 2>/dev/null || true)"
        if [ "$observed_exe" = "$FAKE_HELPER" ]; then
            printf '%s %s\n' "$helper_pid" "$observed_exe" \
                > "$FAKE_HELPER_IDENTITY_FILE"
            break
        fi
        sleep 0.02
    done
    wait "$helper_pid" || true
    [ -s "$FAKE_HELPER_IDENTITY_FILE" ]
}

run_fault_injection_matrix() {
    # 执行不依赖硬件的故障矩阵，覆盖取消、超时、错 ID 与阶段越序。
    local source_pythonpath

    source_pythonpath="${WORKSPACE_DIR}/src/rk_bringup:${WORKSPACE_DIR}/src/rk_mission:${WORKSPACE_DIR}/src/rk_navigation:${WORKSPACE_DIR}/src/rk_locomotion"
    if [ -n "${PYTHONPATH:-}" ]; then
        source_pythonpath="${source_pythonpath}:${PYTHONPATH}"
    fi
    echo "Running non-arm software fault-injection unit matrix..."
    PYTHONPATH="$source_pythonpath" python3 -m pytest -q \
        "${WORKSPACE_DIR}/src/rk_bringup/test/test_non_arm_competition_contract.py" \
        "${WORKSPACE_DIR}/src/rk_navigation/test/test_start_ready_core.py" \
        "${WORKSPACE_DIR}/src/rk_mission/test/test_non_arm_route_phase_core.py" \
        "${WORKSPACE_DIR}/src/rk_mission/test/test_white_bar_action_core.py" \
        "${WORKSPACE_DIR}/src/rk_mission/test/test_white_bar_stage_command_core.py" \
        "${WORKSPACE_DIR}/src/rk_mission/test/test_inspection_action_core.py" \
        "${WORKSPACE_DIR}/src/rk_locomotion/test/test_front_jump_supervisor.py"
}

topic_json_matches() {
    # Foxy 兼容：使用原生 rclpy observer 替代不支持的 ros2 topic echo --field
    local topic_name="$1"
    local key="$2"
    local expected="$3"

    timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" \
        python3 "$WORKSPACE_DIR/src/rk_bringup/scripts/non_arm_smoke_observer.py" \
        "$topic_name" \
        --once --match-key "$key" --match-value "$expected" \
        --timeout-sec "$TOPIC_SAMPLE_TIMEOUT_SEC" \
        >/dev/null 2>&1
}

topic_json_value() {
    # Foxy 兼容：使用原生 rclpy observer 替代不支持的 ros2 topic echo --field
    local topic_name="$1"
    local key="$2"

    timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" \
        python3 "$WORKSPACE_DIR/src/rk_bringup/scripts/non_arm_smoke_observer.py" \
        "$topic_name" \
        --once --value-key "$key" \
        --timeout-sec "$TOPIC_SAMPLE_TIMEOUT_SEC" \
        2>/dev/null
}

wait_until() {
    local timeout_sec="$1"
    shift
    local deadline=$(( $(date +%s) + timeout_sec ))

    while [ "$(date +%s)" -lt "$deadline" ]; do
        if "$@"; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

start_topic_observer() {
    # 先建立长期订阅再发布 start，避免冷启动 CLI 因 DDS 发现错过短暂阶段。
    # Foxy 的 ros2 topic echo 不支持 --field，因此使用原生 rclpy observer
    # 作为替代。Twist observer（无 field）仍保留 ros2 CLI。
    local topic_name="$1"
    local output_file="$2"
    local field_name="${3:-}"

    if [ -n "$field_name" ]; then
        # Foxy 兼容：使用原生 rclpy String observer，每条 msg.data 一行
        env PYTHONUNBUFFERED=1 timeout "${TOPIC_OBSERVER_TIMEOUT_SEC}s" \
            python3 "$WORKSPACE_DIR/src/rk_bringup/scripts/non_arm_smoke_observer.py" \
            "$topic_name" \
            --timeout-sec "$TOPIC_OBSERVER_TIMEOUT_SEC" \
            > "$output_file" 2>&1 &
        TOPIC_OBSERVER_PIDS+=("$!")
    else
        # Twist observer：ros2 topic echo 在 Foxy 正常工作
        env PYTHONUNBUFFERED=1 timeout "${TOPIC_OBSERVER_TIMEOUT_SEC}s" \
            ros2 topic echo "$topic_name" \
            > "$output_file" 2>&1 &
        TOPIC_OBSERVER_PIDS+=("$!")
    fi
}

topic_stream_json_matches() {
    local stream_file="$1"
    local key="$2"
    local expected="$3"

    python3 - "$stream_file" "$key" "$expected" <<'PY'
import ast
import json
from pathlib import Path
import sys

stream_path, key, expected = sys.argv[1:]
try:
    rows = Path(stream_path).read_text(encoding='utf-8', errors='replace').splitlines()
except OSError:
    raise SystemExit(1)

for raw in rows:
    candidate = raw.strip()
    if not candidate or candidate == '---':
        continue
    try:
        value = json.loads(candidate)
    except (TypeError, ValueError):
        try:
            value = ast.literal_eval(candidate)
        except (SyntaxError, TypeError, ValueError):
            continue
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (TypeError, ValueError):
            continue
    if not isinstance(value, dict):
        continue
    actual = value.get(key)
    if expected == '__true__' and actual is True:
        raise SystemExit(0)
    if expected == '__false__' and actual is False:
        raise SystemExit(0)
    if expected not in ('__true__', '__false__') and str(actual) == expected:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

twist_stream_tail_is_all_zero() {
    local stream_file="$1"
    local first_new_byte="$2"

    tail -c "+${first_new_byte}" "$stream_file" 2>/dev/null | awk '
        BEGIN { section=""; fields=0; samples=0; bad=0 }
        /^linear:/ { section="linear"; next }
        /^angular:/ { section="angular"; next }
        /^[[:space:]]+[xyz]:/ {
            if (section != "linear" && section != "angular") next
            value = $2
            if (value !~ /^[-+]?[0-9]+([.][0-9]*)?([eE][-+]?[0-9]+)?$/ \
                || (value + 0.0) != 0.0) bad = 1
            fields += 1
            if (fields == 6) {
                samples += 1
                fields = 0
                section = ""
            }
        }
        END { exit !(samples > 0 && bad == 0) }
    '
}

readiness_passes() {
    local response
    # 短生命周期 ros2 CLI 在高负载 VM 上可能在 DDS 发现完成前退出。这里使用
    # 一个有界的 rclpy 客户端调用同一 Trigger 服务，不绕过 readiness 合约。
    if ! response="$(timeout 15s python3 - <<'PY'
import json
import os
import time

import rclpy
from std_srvs.srv import Trigger

rclpy.init()
node = rclpy.create_node('non_arm_readiness_probe_{}'.format(os.getpid()))
try:
    client = node.create_client(Trigger, '/competition/check_readiness')
    deadline = time.monotonic() + 12.0
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
        return 1
    fi
    printf '%s\n' "$response"
    printf '%s\n' "$response" | grep -Eq '"success":true'
}

publish_duplicate_start_for_contract_check() {
    # 这不是正式启动入口：正式 start 仍只由 mission_start.sh 执行一次。
    # 本函数仅在启动后验证重复请求不会改写 run_id。避免高负载 VM 上一次性
    # ros2 CLI 在 DDS 上下文销毁瞬间报错，改用同样有界的原生 rclpy 发布器。
    timeout 12s python3 - <<'PY'
import os
import time

import rclpy
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('non_arm_duplicate_start_probe_{}'.format(os.getpid()))
try:
    publisher = node.create_publisher(Bool, '/mission/start', 10)
    deadline = time.monotonic() + 4.0
    message = Bool(data=True)
    while time.monotonic() < deadline:
        # 重复 start 是幂等合约验证；小窗口内重发仅补偿 DDS 发现，不改变控制权。
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.2)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
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

final_cmd_is_zero() {
    # Foxy 兼容：使用有界 rclpy 订阅器替代不支持的 ros2 topic echo --once
    timeout "${TOPIC_SAMPLE_TIMEOUT_SEC}s" python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('final_cmd_zero_probe')
result = {'vx': None, 'wz': None}

def _on_twist(msg):
    result['vx'] = msg.linear.x
    result['wz'] = msg.angular.z

node.create_subscription(Twist, '/navigation/cmd_vel', _on_twist, 10)
deadline = time.monotonic() + 5.0
while rclpy.ok() and (result['vx'] is None or result['wz'] is None) \
        and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()

vx = result['vx']
wz = result['wz']
if vx is None or wz is None:
    raise SystemExit(1)
if abs(vx) < 1e-9 and abs(wz) < 1e-9:
    raise SystemExit(0)
raise SystemExit(1)
PY
}

smoke_hardware_is_suppressed() {
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    if printf '%s\n' "$nodes" | grep -Eqi \
        '(cmd_vel_udp_forwarder|realsense|/camera$)'; then
        printf '%s\n' "$nodes" >&2
        return 1
    fi
    return 0
}

smoke_route_completed() {
    # FINAL_STOP 是短暂状态后仍可能继续发布 WAIT_START；Foxy 的临时订阅
    # 会错过已发生的终态。只读预热流保留本轮完整历史，且不改变路线超时。
    topic_stream_json_matches "$LINE_COURSE_STREAM" route_phase FINAL_STOP \
        && topic_stream_json_matches "$LINE_COURSE_STREAM" final_zone_armed __true__ \
        && topic_stream_json_matches "$LINE_COURSE_STREAM" start_jump_completed __true__ \
        && topic_stream_json_matches "$LINE_COURSE_STREAM" inspection_completed __true__ \
        && topic_stream_json_matches "$LINE_COURSE_STREAM" finish_jump_completed __true__ \
        && topic_stream_json_matches \
            "$WHITE_STAGE_PUBLISHER_STREAM" sequence 2
}

capture_line_course_state() {
    # 失败证据保存在验收目录，避免 timeout cleanup 吞掉 ROS CLI 的末尾输出。
    # Foxy 兼容：使用原生 rclpy observer 替代不支持的 ros2 topic echo --once --field
    timeout 5s python3 "$WORKSPACE_DIR/src/rk_bringup/scripts/non_arm_smoke_observer.py" \
        /mission/line_course_state --once --dump --timeout-sec 4 \
        > "${CHECK_DIR}/line_course_state_failure_sample.txt" 2>&1 || true
}

cd "$WORKSPACE_DIR"
mkdir -p "$CHECK_DIR" "$ROS_LOG_DIR"
if ! prepare_smoke_guard_dir; then
    echo "ERROR: smoke cleanup guard preflight failed before launch." >&2
    exit 1
fi
ENV_SCRIPT="$(resolve_env_script)" || {
    echo "ERROR: clean ROS environment script is unavailable." >&2
    exit 1
}
source "$ENV_SCRIPT"
if ! prepare_isolated_overlay; then
    echo "ERROR: isolated formal overlay preparation failed." >&2
    exit 1
fi

COMPILER="${CC:-cc}"
if ! command -v "$COMPILER" >/dev/null 2>&1; then
    echo "ERROR: C compiler is required to create the fake ELF helper." >&2
    exit 1
fi
"$COMPILER" -std=c11 -O2 -Wall -Wextra \
    src/rk_bringup/test_support/fake_sdk_motion_helper.c \
    -o "$FAKE_HELPER"
if ! file "$FAKE_HELPER" | grep -q 'ELF'; then
    echo "ERROR: fake helper is not an ELF executable." >&2
    exit 1
fi
chmod 700 "$FAKE_HELPER"
if ! python3 - "$FAKE_HELPER" <<'PY'
from pathlib import Path
import sys

marker = b'RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1'
contents = Path(sys.argv[1]).read_bytes()
if not contents.startswith(b'\x7fELF') or marker not in contents:
    raise SystemExit(1)
PY
then
    echo "ERROR: fake helper is missing the required test-only ELF marker." >&2
    exit 1
fi
if ! verify_fake_helper_proc_identity; then
    echo "ERROR: fake helper /proc executable identity check failed." >&2
    exit 1
fi

if ! run_fault_injection_matrix; then
    echo "ERROR: non-arm software fault-injection unit matrix failed." >&2
    exit 1
fi

echo "SOFTWARE_SMOKE_MODE: isolated ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; no robot I/O."
# 只验证 returncode=0 的软件链路，helper 不应占用 estop 新鲜度窗口；其 ELF
# 身份已在前置步骤校验，真实 Action 调用另由 launch 日志证明。
export RK_FAKE_SDK_SLEEP_MS=0
if ! command -v setsid >/dev/null 2>&1; then
    echo "ERROR: setsid is required to isolate smoke launch cleanup." >&2
    exit 1
fi
setsid ros2 launch rk_bringup competition_non_arm.launch.py \
    hardware_mode:=false \
    software_smoke_mode:=true \
    start_realsense:=false \
    start_sdk_server:=false \
    start_udp_forwarder:=false \
    fake_sdk_action_executable:="$FAKE_HELPER" \
    smoke_cleanup_guard_path:="$SMOKE_GUARD" \
    smoke_scenario:=full \
    smoke_auto_start:=false \
    > "${CHECK_DIR}/launch.log" 2>&1 &
LAUNCH_PID="$!"
start_smoke_process_watchdog

# 该 gate 会连续检查真正的 Action server 与 gait 心跳；高负载 VM 冷启动时
# DDS discovery 与 SDK bridge 装载可跨越 60 秒。120 秒仅延长只读等待，
# 仍受总 900 秒上限约束，不改变任何动作或安全超时。
if ! wait_until 120 readiness_passes; then
    echo "ERROR: software smoke readiness did not pass." >&2
    tail -n 120 "${CHECK_DIR}/launch.log" >&2 || true
    exit 1
fi
if ! smoke_hardware_is_suppressed; then
    echo "ERROR: software smoke detected a prohibited hardware ROS node." >&2
    exit 1
fi
# 三个只读 observer 在正式 start 前预热：readiness 已用同一快照确认初始
# WAIT_START/mission_started=false；这里持久观察状态流再确认 WAIT_START 和
# 空 run_id，避免一次性 ros2 topic echo 因发现延迟误判 auto-start。后续状态流
# 还证明实际经过 START_READY 和
# START_STAGE，cmd_vel 流则只检查 start 后的新增样本，避免把 pre-start 零速
# 当成门控证据。该做法不依赖一次性 ros2 CLI 的偶发发现时序。
LINE_FOLLOW_STREAM="${CHECK_DIR}/line_follow_status_stream.log"
LINE_COURSE_STREAM="${CHECK_DIR}/line_course_state_stream.log"
FINAL_CMD_STREAM="${CHECK_DIR}/final_cmd_stream.log"
WHITE_STAGE_PUBLISHER_STREAM="${CHECK_DIR}/white_stage_publisher_status_stream.log"
start_topic_observer /navigation/line_follow_status "$LINE_FOLLOW_STREAM" data
start_topic_observer /mission/line_course_state "$LINE_COURSE_STREAM" data
start_topic_observer /navigation/cmd_vel "$FINAL_CMD_STREAM"
start_topic_observer /mission/white_bar_stage_command_publisher_status \
    "$WHITE_STAGE_PUBLISHER_STREAM" data
if ! wait_until 45 topic_stream_json_matches "$LINE_FOLLOW_STREAM" \
    nav_state WAIT_START; then
    echo "ERROR: line follower observer did not warm up in WAIT_START." >&2
    exit 1
fi
if ! wait_until 45 topic_stream_json_matches "$LINE_COURSE_STREAM" \
    route_phase WAIT_START; then
    echo "ERROR: line course observer did not warm up in WAIT_START." >&2
    exit 1
fi
if ! wait_until 45 topic_stream_json_matches "$LINE_COURSE_STREAM" \
    run_id ''; then
    echo "ERROR: WAIT_START observer saw a nonempty run_id before start." >&2
    exit 1
fi
if ! wait_until 45 twist_stream_tail_is_all_zero "$FINAL_CMD_STREAM" 1; then
    echo "ERROR: final cmd_vel observer did not warm up with zero output." >&2
    exit 1
fi
FINAL_CMD_START_BYTE=$(( $(wc -c < "$FINAL_CMD_STREAM") + 1 ))

# 使用正式 start 脚本，验证 readiness gate 后只发布一次任务命令。它在后台
# 运行仅为了让验收在 START_READY 真实存在时立刻检查零速窗口；start 本身仍
# 只由这个脚本执行，随后必须等到其确认结果，不能绕过正式 gate。
MISSION_START_LOG="${CHECK_DIR}/mission_start.log"
RK_COMPETITION_READINESS_TIMEOUT_SEC=30 \
    RK_COMPETITION_START_CONFIRM_TIMEOUT_SEC=60 \
    RK_COMPETITION_START_PUBLISH_TIMEOUT_SEC=20 \
    "${WORKSPACE_DIR}/src/rk_bringup/scripts/mission_start.sh" \
    > "$MISSION_START_LOG" 2>&1 &
MISSION_START_PID="$!"

# 合成输入器在 start 后先给出低置信新 LineTrack；真实 follower 必须先公开
# START_READY，并在此期间让 command_mux 输出维持零，不能立即盲走。读取已
# 预热的状态流，既检查真实状态，又避免单次 CLI 发现速度干扰验收。
if ! wait_until 45 topic_stream_json_matches "$LINE_FOLLOW_STREAM" \
    nav_state START_READY; then
    cat "$MISSION_START_LOG" >&2 || true
    echo "ERROR: line follower did not enter START_READY after mission start." >&2
    exit 1
fi
if ! wait_until 45 topic_stream_json_matches "$LINE_COURSE_STREAM" \
    route_phase START_STAGE; then
    echo "ERROR: route did not enter START_STAGE after mission start." >&2
    exit 1
fi
# smoke 仅在 START_READY 保持低置信新帧 3 秒；等待 1 秒可保证以下截取只
# 包含 start 后真实输出，且仍严格处于门控窗口内。
sleep 1
if ! twist_stream_tail_is_all_zero "$FINAL_CMD_STREAM" "$FINAL_CMD_START_BYTE"; then
    echo "ERROR: final cmd_vel was nonzero during START_READY." >&2
    exit 1
fi
if ! wait "$MISSION_START_PID"; then
    cat "$MISSION_START_LOG" >&2 || true
    echo "ERROR: formal mission_start.sh failed in software smoke." >&2
    exit 1
fi
MISSION_START_PID=""
cat "$MISSION_START_LOG"
if ! wait_until 30 topic_stream_json_matches "$LINE_FOLLOW_STREAM" \
    ready __true__; then
    echo "ERROR: stable post-start LineTrack did not satisfy START_READY." >&2
    exit 1
fi
# START 命令可能在本机迟滞时已经完成到 FINISH；验证预热 observer 捕获到过
# sequence=1，不用一次性 CLI 错把当前 sequence=2 当作缺少 START 命令。
if ! wait_until 30 topic_stream_json_matches \
    "$WHITE_STAGE_PUBLISHER_STREAM" sequence 1; then
    echo "ERROR: START white-bar stage sequence=1 was not published." >&2
    exit 1
fi
RUN_ID="$(topic_json_value /mission/line_course_state run_id)"
if [ -z "$RUN_ID" ]; then
    echo "ERROR: mission start did not create run_id." >&2
    exit 1
fi

# 重复 start 只能被节点忽略，不能替换 run_id 或清空阶段。
if ! publish_duplicate_start_for_contract_check; then
    echo "ERROR: duplicate start contract probe could not publish." >&2
    exit 1
fi
sleep 0.4
if [ "$(topic_json_value /mission/line_course_state run_id)" != "$RUN_ID" ]; then
    echo "ERROR: duplicate start changed run_id." >&2
    exit 1
fi

if ! wait_until 100 smoke_route_completed; then
    echo "ERROR: full software smoke did not reach FINAL_STOP." >&2
    tail -n 180 "${CHECK_DIR}/launch.log" >&2 || true
    exit 1
fi

if ! wait_until 8 final_cmd_is_zero; then
    echo "ERROR: final cmd_vel was not zero after FINAL_STOP." >&2
    exit 1
fi

CMD_INFO="$(timeout 12s ros2 topic info -v /navigation/cmd_vel 2>&1 || true)"
if [ -z "$CMD_INFO" ]; then
    echo "ERROR: final cmd_vel topic info is unavailable." >&2
    exit 1
fi
printf '%s\n' "$CMD_INFO"
if ! printf '%s\n' "$CMD_INFO" | grep -Eq 'Publisher count: 1'; then
    echo "ERROR: final cmd_vel publisher count is not one." >&2
    exit 1
fi
if ! printf '%s\n' "$CMD_INFO" | grep -Eq 'Node name: command_mux_node'; then
    echo "ERROR: command_mux_node is not final cmd_vel owner." >&2
    exit 1
fi
if ! smoke_hardware_is_suppressed; then
    echo "ERROR: prohibited hardware node appeared during smoke." >&2
    exit 1
fi
if ! assert_smoke_process_guard; then
    echo "ERROR: software smoke process isolation check failed." >&2
    exit 1
fi

echo "ACCEPT_NON_ARM_COMPETITION_SOFTWARE_SMOKE_PASS"
echo "No RealSense, UDP server, UDP forwarder, Unitree SDK, or physical motion was used."
