#!/bin/bash
# 正式非机械臂真机零运动闭环：只允许既有 startup/zero/shutdown StopMove。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd -P)"
WORKSPACE_DIR="$(cd -- "${RK_INSPECTION_WS:-${SCRIPT_DIR}/../../..}" && pwd -P)"
CYCLES="${RK_ZERO_ACCEPT_CYCLES:-3}"
OBSERVE_SEC="${RK_ZERO_ACCEPT_TOPIC_OBSERVE_SEC:-6}"
RUN_ROOT="${RK_ZERO_ACCEPT_RUN_ROOT:-${HOME}/rk_non_arm_zero_motion_acceptance_$(date +%Y%m%d_%H%M%S)}"
START_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/start_non_arm_competition.sh"
STOP_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/stop_line_system.sh"
ENV_SCRIPT="${WORKSPACE_DIR}/src/rk_bringup/scripts/ros_clean_env.sh"
TOPIC_OBSERVER="${WORKSPACE_DIR}/src/rk_bringup/scripts/non_arm_smoke_observer.py"
ROBOT_IP="192.168.123.161"
INTERFACE="eth1"
EXPECTED_MAC="48:b0:2d:f9:01:24"
COMMAND_PORT="${RK_COMPETITION_SDK_UDP_PORT:-15001}"
STATUS_PORT="${RK_COMPETITION_SDK_STATUS_PORT:-15002}"
ACTIVE_CYCLE=0
STOP_TIMEOUT_SEC="${RK_ZERO_ACCEPT_STOP_TIMEOUT_SEC:-90}"

require_control_plane_thresholds() {
    local variable_name
    for variable_name in \
        RK_COMPETITION_CONTROL_PLANE_NETWORK_TIMEOUT_SEC \
        RK_COMPETITION_CONTROL_PLANE_PING_COUNT \
        RK_COMPETITION_CONTROL_PLANE_PING_POLL_SEC \
        RK_COMPETITION_CONTROL_PLANE_DDS_TIMEOUT_SEC \
        RK_COMPETITION_CONTROL_PLANE_REQUIRED_FRAMES \
        RK_COMPETITION_CONTROL_PLANE_MAX_FRAME_GAP_MS; do
        if [ -z "${!variable_name:-}" ]; then
            echo "ERROR: measured threshold is required: ${variable_name}" >&2
            return 1
        fi
    done
}

udp_listener_count() {
    local port="$1"
    # `ss -lun` 第 4 列才是本地 bind 地址；验收与正式启动必须使用同一
    # 解析合同，避免 ready receiver 被第 5 列的 peer 地址误判为缺失。
    ss -H -lun | awk -v port="$port" '$4 ~ (":" port "$") {count += 1} END {print count + 0}'
}

sdk_server_process_count() {
    # 只匹配真实 ELF argv[0]；tmux server 的保存命令行不能算第二个 server。
    pgrep -fc '^/[^ ]*/go2_sdk_udp_server( |$)' 2>/dev/null || true
}

udp_forwarder_process_count() {
    # Python 入口必须是 argv[1]，拒绝把 tmux pane 命令字符串误计为进程。
    pgrep -fc '^([^ ]*/)?python3 [^ ]*/cmd_vel_udp_forwarder[.]py( |$)' \
        2>/dev/null || true
}

cleanup_is_complete() {
    [ "$(sdk_server_process_count)" -eq 0 ] \
        && [ "$(udp_forwarder_process_count)" -eq 0 ] \
        && [ "$(udp_listener_count "$COMMAND_PORT")" -eq 0 ] \
        && [ "$(udp_listener_count "$STATUS_PORT")" -eq 0 ] \
        && ! tmux has-session -t "$RK_COMPETITION_TMUX_SESSION" 2>/dev/null
}

cleanup_current_cycle() {
    local status=$?
    if [ "$ACTIVE_CYCLE" -ne 0 ]; then
        set +e
        timeout "${STOP_TIMEOUT_SEC}s" "$STOP_SCRIPT"
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            cleanup_is_complete && break
            sleep 1
        done
        if ! cleanup_is_complete; then
            echo "ERROR: acceptance cleanup left a managed process or UDP port." >&2
            status=1
        fi
        ACTIVE_CYCLE=0
        set -e
    fi
    return "$status"
}

on_exit() {
    local status=$?
    cleanup_current_cycle || status=1
    exit "$status"
}
trap on_exit EXIT INT TERM

validate_environment() {
    export RK_ROS_DOMAIN_ID=10
    source "$ENV_SCRIPT"
    [ "${ROS_DISTRO:-}" = "foxy" ]
    [ "${ROS_DOMAIN_ID:-}" = "10" ]
    [ -z "${RMW_IMPLEMENTATION+x}" ]
    [ -z "${CYCLONEDDS_URI+x}" ]
    [ -z "${CYCLONEDDS_HOME+x}" ]
    case ":${LD_LIBRARY_PATH:-}:" in
        *:/usr/local/cyclonedds/lib:*|*:/home/unitree/cyclonedds_ws/install/cyclonedds/lib:*)
            echo "ERROR: CycloneDDS pollution remains in normal ROS environment." >&2
            return 1
            ;;
    esac
    local graph_output
    graph_output="$(timeout 8s ros2 node list 2>&1)"
    ! printf '%s\n' "$graph_output" | grep -Fq 'std::bad_alloc'
}

validate_network() {
    local link
    link="$(ip -o link show dev "$INTERFACE")"
    [[ "$link" == *,UP,* && "$link" == *LOWER_UP* ]]
    [ "$(cat "/sys/class/net/${INTERFACE}/address")" = "$EXPECTED_MAC" ]
    ip -4 -o addr show dev "$INTERFACE" | grep -Fq '192.168.123.18/24'
    ip route get "$ROBOT_IP" | grep -Fq "dev ${INTERFACE}"
    ping -I "$INTERFACE" -c 1 -W 1 "$ROBOT_IP" >/dev/null
}

topic_rate_at_least() {
    local topic="$1"
    local minimum="$2"
    local output_file="$3"
    local rate_type="$4"

    # 使用单一原生订阅完成整个观察窗口，避免 ros2 topic hz 的发现竞态。
    if ! timeout "$((OBSERVE_SEC + 3))s" python3 "$TOPIC_OBSERVER" \
            "$topic" --rate-type "$rate_type" \
            --minimum-rate-hz "$minimum" --required-span-sec 2.0 \
            --timeout-sec "$OBSERVE_SEC" > "$output_file" 2>&1; then
        echo "ERROR: ${topic} native rate gate failed." >&2
        cat "$output_file" >&2
        return 1
    fi
    echo "TOPIC_RATE topic=${topic} $(cat "$output_file")"
}

is_zero_twist_sample() {
    awk '
        BEGIN {section=""; seen=0; ok=1}
        /^linear:/ {section="linear"; next}
        /^angular:/ {section="angular"; next}
        /^[[:space:]]+[xyz]:/ {
            if (section != "linear" && section != "angular") next
            seen += 1
            value = $2
            if (value !~ /^[-+]?[0-9]+([.][0-9]*)?([eE][-+]?[0-9]+)?$/ \
                || (value + 0.0) != 0.0) ok = 0
        }
        END {exit !(seen == 6 && ok == 1)}
    '
}

validate_graph_and_zero() {
    local graph_file="$1"
    local mission_file="$2"
    local twist_file="$3"

    timeout 6s ros2 topic info -v /navigation/cmd_vel > "$graph_file"
    grep -Eq 'Publisher count: 1' "$graph_file"
    grep -Eq 'Node name: command_mux_node' "$graph_file"
    timeout 6s ros2 topic info -v /mission/start > "$mission_file"
    grep -Eq 'Publisher count: 0' "$mission_file"
    timeout 8s python3 "$TOPIC_OBSERVER" /navigation/cmd_vel --twist \
        --consecutive-zero-count 1 --timeout-sec 8 > "$twist_file"
    is_zero_twist_sample < "$twist_file"
}

readiness_passes() {
    local output_file="$1"
    timeout 8s ros2 service call /competition/check_readiness \
        std_srvs/srv/Trigger '{}' > "$output_file" 2>&1
    grep -Eq 'success[=:][[:space:]]*(true|True)' "$output_file"
    grep -Fq 'SDK_MOTION_BACKEND_READY' "$output_file"
}

verify_cycle() {
    local cycle="$1"
    local cycle_root="$2"
    local log_dir="$RK_COMPETITION_LOG_DIR"
    local bridge_prefix
    local status_audit
    local instance_id
    local camera_profile

    bridge_prefix="$(ros2 pkg prefix rk_go2_sdk_bridge)"
    status_audit="${bridge_prefix}/lib/rk_go2_sdk_bridge/sdk_motion_status_audit.py"
    instance_id="$(tr -d '\r\n' < "${RK_COMPETITION_RUNTIME_DIR}/server_instance_id")"

    echo "ZERO_MOTION_GATE cycle=${cycle} gate=CONTROL_PLANE"
    grep -Fq 'classification=ROBOT_CONTROL_PLANE_READY' \
        "${log_dir}/control_plane_gate.log" || return 1
    echo "ZERO_MOTION_GATE cycle=${cycle} gate=RECEIVER_FIRST"
    grep -Fq '"classification":"PASS"' \
        "${log_dir}/status_receiver_gate.log" || return 1
    echo "ZERO_MOTION_GATE cycle=${cycle} gate=STARTUP_ACK"
    grep -Fq '"event":"STARTUP_STOP"' \
        "${log_dir}/startup_status_gate.log" || return 1
    grep -Fq '"ret":0' "${log_dir}/startup_status_gate.log" || return 1

    echo "ZERO_MOTION_GATE cycle=${cycle} gate=UNIQUE_PROCESSES_PORTS"
    [ "$(udp_listener_count "$COMMAND_PORT")" -eq 1 ] || return 1
    [ "$(udp_listener_count "$STATUS_PORT")" -eq 1 ] || return 1
    [ "$(sdk_server_process_count)" -eq 1 ] || return 1
    [ "$(udp_forwarder_process_count)" -eq 1 ] || return 1

    echo "ZERO_MOTION_GATE cycle=${cycle} gate=READINESS"
    readiness_passes "${cycle_root}/readiness.txt" || return 1
    echo "ZERO_MOTION_GATE cycle=${cycle} gate=GRAPH_AND_ZERO"
    validate_graph_and_zero \
        "${cycle_root}/cmd_vel_graph.txt" \
        "${cycle_root}/mission_start_graph.txt" \
        "${cycle_root}/final_zero_twist.txt" || return 1

    echo "ZERO_MOTION_GATE cycle=${cycle} gate=TOPIC_RATES"
    topic_rate_at_least /line_camera/image_raw 10 \
        "${cycle_root}/line_camera_hz.txt" image || return 1
    topic_rate_at_least /perception/line_track 10 \
        "${cycle_root}/line_track_hz.txt" line_track || return 1
    topic_rate_at_least /perception/white_bar_detection 10 \
        "${cycle_root}/white_bar_hz.txt" special_target || return 1

    camera_profile="$(grep -E 'actual=[0-9]+([.][0-9]+)?x[0-9]+([.][0-9]+)?@[0-9.]+.*fourcc=' \
        "${log_dir}/line_camera.log" | tail -n 1 || true)"
    if [ -z "$camera_profile" ]; then
        echo "ERROR: line camera actual profile was not recorded." >&2
        return 1
    fi
    printf '%s\n' "$camera_profile" > "${cycle_root}/line_camera_profile.txt"

    echo "ZERO_MOTION_GATE cycle=${cycle} gate=STATUS_AUDIT"
    "$status_audit" --expected-server-instance-id "$instance_id" \
        --duration-sec 2.0 | tee "${cycle_root}/sdk_status_audit.jsonl" \
        || return 1
    grep -Fq '"success":true' "${cycle_root}/sdk_status_audit.jsonl" \
        || return 1
    grep -Fq '"move_count":0' "${cycle_root}/sdk_status_audit.jsonl" \
        || return 1
    readiness_passes "${cycle_root}/readiness_after_observation.txt" \
        || return 1

    echo "ZERO_MOTION_CYCLE cycle=${cycle} result=PASS instance=${instance_id}"
}

if ! [[ "$CYCLES" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: RK_ZERO_ACCEPT_CYCLES must be a positive integer." >&2
    exit 1
fi
for required_script in "$START_SCRIPT" "$STOP_SCRIPT" "$ENV_SCRIPT" \
        "$TOPIC_OBSERVER"; do
    [ -x "$required_script" ] || {
        echo "ERROR: required executable is missing: ${required_script}" >&2
        exit 1
    }
done
require_control_plane_thresholds
mkdir -p "$RUN_ROOT"
RUN_ROOT="$(cd -- "$RUN_ROOT" && pwd -P)"
export RK_INSPECTION_WS="$WORKSPACE_DIR"
validate_environment
validate_network

if [ "$(sdk_server_process_count)" -ne 0 ] \
        || [ "$(udp_forwarder_process_count)" -ne 0 ] \
        || [ "$(udp_listener_count "$COMMAND_PORT")" -ne 0 ] \
        || [ "$(udp_listener_count "$STATUS_PORT")" -ne 0 ]; then
    echo "ERROR: stale formal SDK process/port exists before acceptance." >&2
    exit 1
fi

for cycle in $(seq 1 "$CYCLES"); do
    CYCLE_ROOT="${RUN_ROOT}/cycle_${cycle}"
    mkdir -p "$CYCLE_ROOT" "${CYCLE_ROOT}/line_unused"
    export RK_COMPETITION_RUNTIME_DIR="${CYCLE_ROOT}/runtime"
    export RK_COMPETITION_LOG_DIR="${CYCLE_ROOT}/logs"
    export RK_COMPETITION_TMUX_SESSION="rk_zero_accept_${$}_${cycle}"
    export RK_LINE_RUNTIME_DIR="${CYCLE_ROOT}/line_unused"
    export RK_LINE_TMUX_SESSION="rk_zero_accept_line_${$}_${cycle}"
    export RK_COMPETITION_HARDWARE_MODE=true
    export RK_COMPETITION_SOFTWARE_SMOKE_MODE=false
    export RK_COMPETITION_START_SDK_SERVER=true
    export RK_COMPETITION_START_UDP_FORWARDER=true
    export RK_COMPETITION_START_LINE_CAMERA=true
    ACTIVE_CYCLE=1

    "$START_SCRIPT" 2>&1 | tee "${CYCLE_ROOT}/formal_start.txt"
    verify_cycle "$cycle" "$CYCLE_ROOT"
    timeout "${STOP_TIMEOUT_SEC}s" "$STOP_SCRIPT" 2>&1 \
        | tee "${CYCLE_ROOT}/formal_stop.txt"
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        cleanup_is_complete && break
        sleep 1
    done
    if ! cleanup_is_complete; then
        echo "ERROR: cleanup failed in cycle ${cycle}." >&2
        exit 1
    fi
    ACTIVE_CYCLE=0
    echo "ZERO_MOTION_CLEANUP cycle=${cycle} result=PASS"
done

trap - EXIT INT TERM
echo "FORMAL_NON_ARM_ZERO_MOTION_ACCEPTANCE result=PASS cycles=${CYCLES} run_root=${RUN_ROOT}"
