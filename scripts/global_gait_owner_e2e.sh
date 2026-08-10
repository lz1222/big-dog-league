#!/bin/bash
# 架空实体 GlobalGaitOwner 端到端验收：只切步态，不发布任何非零速度。
# 所有子进程受外层前台 runner 管理，退出 trap 先向 server 发送 ZERO，再 SIGINT。
set -euo pipefail

if [ "${1:-}" != "--execute-off-ground" ]; then
    echo "Usage: $0 --execute-off-ground" >&2
    exit 2
fi

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID=10
export ROS_LOG_DIR=/tmp/rk_go2_global_owner_e2e_roslog

FORWARDER="${WORKSPACE_DIR}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py"
OWNER="${WORKSPACE_DIR}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/global_gait_owner.py"
RUNTIME="${WORKSPACE_DIR}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py"
SERVER="${WORKSPACE_DIR}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_udp_server"
INSTANCE_ID="owner-e2e-script"
CHILD_PIDS=()

cleanup()
{
    set +e
    # loopback ZERO 由 server 转换为真实 StopMove；即使 server 已退出也不会阻塞。
    python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b"0 0 0",("127.0.0.1",15001)); s.close()'
    for pid in "${CHILD_PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${CHILD_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

# 拒绝与既有运动 writer 并行；匹配仅包含真实命令路径，不包含本检查命令。
if pgrep -f 'go2_sdk_udp_server|cmd_vel_udp_forwarder.py|global_gait_owner.py' \
        >/dev/null; then
    echo "E2E_FAIL reason=existing_motion_or_gait_writer" >&2
    exit 1
fi

"$FORWARDER" --ros-args \
    -p cmd_vel_topic:=/navigation/cmd_vel \
    -p udp_host:=127.0.0.1 -p udp_port:=15001 \
    -p status_ip:=127.0.0.1 -p status_port:=15002 \
    -p expected_server_instance_id:="$INSTANCE_ID" \
    -p max_vx:=0.10 -p max_vy:=0.01 -p max_yaw:=0.15 &
CHILD_PIDS+=("$!")
sleep 0.5

"$OWNER" --ros-args \
    -p udp_host:=127.0.0.1 -p udp_port:=15001 \
    -p final_cmd_topic:=/navigation/cmd_vel \
    -p sdk_status_topic:=/go2/sdk_motion_status \
    -p status_topic:=/gait/mode_status &
CHILD_PIDS+=("$!")
sleep 0.5

"$RUNTIME" "$SERVER" --interface eth1 \
    --listen-ip 127.0.0.1 --port 15001 \
    --status-ip 127.0.0.1 --status-port 15002 \
    --server-instance-id "$INSTANCE_ID" \
    --max-vx 0.10 --max-vy 0.01 --max-yaw 0.15 \
    --watchdog-sec 0.30 --manual-classic-confirmed true &
CHILD_PIDS+=("$!")
sleep 3.0

for pid in "${CHILD_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "E2E_FAIL reason=child_exited pid=${pid}" >&2
        exit 1
    fi
done

# movement lock 后 command_mux 的正式输入应为零；本隔离验收直接持续发布零，
# 只用于满足 owner 对最终 /navigation/cmd_vel 的新鲜连续样本门禁。
# 使用有限计数 publisher，避免 timeout 只终止 wrapper 而遗留 ros2 子进程。
ros2 topic pub -r 10 --times 100 \
    /navigation/cmd_vel geometry_msgs/msg/Twist '{}' &
ZERO_PID="$!"
CHILD_PIDS+=("$ZERO_PID")
sleep 1.0

CLASSIC_OUTPUT="$(timeout 6 ros2 service call \
    /gait/ensure_classic std_srvs/srv/Trigger '{}')"
echo "$CLASSIC_OUTPUT"
case "$CLASSIC_OUTPUT" in
    *"success=True"*) ;;
    *) echo "E2E_FAIL reason=ensure_classic" >&2; exit 1 ;;
esac

FREE_OUTPUT="$(timeout 6 ros2 service call \
    /gait/ensure_free std_srvs/srv/Trigger '{}')"
echo "$FREE_OUTPUT"
case "$FREE_OUTPUT" in
    *"success=True"*) ;;
    *) echo "E2E_FAIL reason=ensure_free" >&2; exit 1 ;;
esac

CLASSIC_BACK_OUTPUT="$(timeout 6 ros2 service call \
    /gait/ensure_classic std_srvs/srv/Trigger '{}')"
echo "$CLASSIC_BACK_OUTPUT"
case "$CLASSIC_BACK_OUTPUT" in
    *"success=True"*) ;;
    *) echo "E2E_FAIL reason=ensure_classic_handback" >&2; exit 1 ;;
esac

echo "E2E_PASS classic_free_classic=true verification_source=command_ack"
# 等待有限零速 publisher 自然退出，确保脚本成功返回前已清理命令源。
wait "$ZERO_PID"
