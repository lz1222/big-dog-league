#!/usr/bin/env bash

# 迷宫人工标定录包管理器。
# 该脚本只启动 B1 感知和 rosbag，不启动 B2、运动桥或任何速度发布器。

set -euo pipefail

SESSION_NAME="maze_manual_recording"
DEFAULT_INTERFACE="eth0"
DEFAULT_DELAY_SEC=5
DEFAULT_PREFIX="maze_manual_full"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_ROOT="${HOME}/maze_bags"

usage() {
  cat <<'EOF'
用法：
  bash scripts/maze_manual_recorder.sh start [选项]
  bash scripts/maze_manual_recorder.sh status
  bash scripts/maze_manual_recorder.sh stop

start 选项：
  --iface NAME       CycloneDDS 网卡，默认 eth0
  --delay SEC        启动 B1 后延迟多少秒开始录包，默认 5
  --prefix NAME      录包目录前缀，默认 maze_manual_full
  --marker TEXT      录包启动后写入的人工检查点，默认 MAZE_START
  -h, --help         显示帮助

示例：
  bash scripts/maze_manual_recorder.sh start \
    --delay 5 \
    --marker "MAZE_START front_cm=107 left_front_cm=19"
EOF
}

fail() {
  echo "错误：$*" >&2
  exit 1
}

session_exists() {
  tmux has-session -t "${SESSION_NAME}" 2>/dev/null
}

session_bag_path() {
  tmux show-environment -t "${SESSION_NAME}" MAZE_BAG_PATH \
    2>/dev/null | sed -n 's/^MAZE_BAG_PATH=//p'
}

setup_ros_environment() {
  local cyclone_install="${HOME}/cyclonedds_ws/install"

  [[ -f /opt/ros/foxy/local_setup.bash ]] || \
    fail "未找到 /opt/ros/foxy/local_setup.bash"
  [[ -d "${cyclone_install}/rmw_cyclonedds_cpp" ]] || \
    fail "未找到真机自编译 rmw_cyclonedds_cpp"
  [[ -d "${cyclone_install}/cyclonedds/lib" ]] || \
    fail "未找到真机自编译 CycloneDDS"

  # Foxy 与 Unitree SDK 的 CycloneDDS 必须使用已验证的真机隔离环境，
  # 避免系统 0.7 与 /usr/local 0.10 动态库混用造成节点段错误。
  # Foxy 的环境脚本会读取若干可选变量，加载期间需暂时关闭 nounset。
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/foxy/local_setup.bash
  set -u
  export AMENT_PREFIX_PATH="${cyclone_install}/rmw_cyclonedds_cpp:${cyclone_install}/cyclonedds:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${cyclone_install}/rmw_cyclonedds_cpp/lib:${cyclone_install}/cyclonedds/lib:${LD_LIBRARY_PATH:-}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${NETWORK_INTERFACE}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
  export ROS_LOG_DIR="${WORKSPACE_DIR}/log/ros"

  mkdir -p "${ROS_LOG_DIR}" "${BAG_ROOT}"
}

wait_for_bag_window_to_close() {
  local attempt
  for attempt in $(seq 1 150); do
    if ! tmux list-windows -t "${SESSION_NAME}" \
      -F '#{window_name}' 2>/dev/null | grep -qx BAG; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

stop_session_safely() {
  local bag_path=""

  session_exists || return 0
  bag_path="$(session_bag_path || true)"

  if tmux list-windows -t "${SESSION_NAME}" \
    -F '#{window_name}' 2>/dev/null | grep -qx BAG; then
    # 必须让 rosbag 接收 SIGINT 并写完 metadata.yaml，不能直接杀 tmux。
    tmux send-keys -t "${SESSION_NAME}:BAG" C-c
    if ! wait_for_bag_window_to_close; then
      echo "录包仍在收尾，请稍后再次执行 stop。" >&2
      return 1
    fi
  fi

  tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
  echo "人工标定录包已停止。"
  [[ -z "${bag_path}" ]] || echo "录包目录：${bag_path}"
}

status_recording() {
  local bag_path=""

  if ! session_exists; then
    echo "状态：未运行"
    return 0
  fi

  bag_path="$(session_bag_path || true)"
  echo "状态：运行中"
  echo "tmux：${SESSION_NAME}"
  [[ -z "${bag_path}" ]] || echo "录包目录：${bag_path}"
  tmux list-windows -t "${SESSION_NAME}" \
    -F '  #{window_index} #{window_name} cmd=#{pane_current_command}'
  if [[ -n "${bag_path}" && -d "${bag_path}" ]]; then
    du -sh "${bag_path}"
  fi
}

start_recording() {
  local delay_sec="${DEFAULT_DELAY_SEC}"
  local marker="MAZE_START"
  local prefix="${DEFAULT_PREFIX}"
  local remaining
  local bag_path
  local cleanup_pending=true

  NETWORK_INTERFACE="${DEFAULT_INTERFACE}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --iface)
        [[ $# -ge 2 ]] || fail "--iface 缺少参数"
        NETWORK_INTERFACE="$2"
        shift 2
        ;;
      --delay)
        [[ $# -ge 2 ]] || fail "--delay 缺少参数"
        delay_sec="$2"
        shift 2
        ;;
      --prefix)
        [[ $# -ge 2 ]] || fail "--prefix 缺少参数"
        prefix="$2"
        shift 2
        ;;
      --marker)
        [[ $# -ge 2 ]] || fail "--marker 缺少参数"
        marker="$2"
        shift 2
        ;;
      -h|--help)
        usage
        return 0
        ;;
      *)
        fail "未知参数：$1"
        ;;
    esac
  done

  [[ "${delay_sec}" =~ ^[0-9]+$ ]] || \
    fail "--delay 必须是非负整数"
  (( delay_sec <= 60 )) || fail "--delay 不能超过 60 秒"
  [[ "${NETWORK_INTERFACE}" =~ ^[[:alnum:]_.:-]+$ ]] || \
    fail "网卡名称包含非法字符"
  [[ "${prefix}" =~ ^[[:alnum:]_.-]+$ ]] || \
    fail "录包前缀包含非法字符"
  [[ -n "${marker}" ]] || fail "人工标记不能为空"

  command -v tmux >/dev/null 2>&1 || fail "未安装 tmux"
  session_exists && fail "${SESSION_NAME} 已运行，请先执行 status 或 stop"
  pgrep -x go2_sdk_udp_server >/dev/null 2>&1 && \
    fail "检测到 go2_sdk_udp_server，人工标定录包拒绝启动"
  pgrep -f '[c]md_vel_udp_forwarder.py' >/dev/null 2>&1 && \
    fail "检测到 cmd_vel_udp_forwarder，人工标定录包拒绝启动"
  pgrep -f '[r]os2 bag record' >/dev/null 2>&1 && \
    fail "检测到其他 rosbag 录制进程"
  pgrep -f '[m]aze_perception_dry_run.py' >/dev/null 2>&1 && \
    fail "检测到其他 B1 感知进程"

  setup_ros_environment
  ip link show dev "${NETWORK_INTERFACE}" >/dev/null 2>&1 || \
    fail "网卡 ${NETWORK_INTERFACE} 不存在"

  bag_path="${BAG_ROOT}/${prefix}_$(date +%Y%m%d_%H%M%S)"

  # 启动失败或倒计时被中断时，只关闭本脚本创建的只读会话。
  cleanup_on_error() {
    local exit_code=$?
    if [[ "${cleanup_pending}" == true ]]; then
      stop_session_safely >/dev/null 2>&1 || true
    fi
    return "${exit_code}"
  }
  trap cleanup_on_error EXIT
  trap 'exit 130' INT TERM

  tmux new-session -d -s "${SESSION_NAME}" -n B1 \
    -c "${WORKSPACE_DIR}" \
    "python3 -u scripts/maze_perception_dry_run.py --ros-args --params-file config/maze_perception_dry_run.yaml"
  tmux set-environment -t "${SESSION_NAME}" \
    MAZE_BAG_PATH "${bag_path}"

  echo "B1 只读感知已启动，${delay_sec} 秒后开始录包。"
  for ((remaining=delay_sec; remaining>0; remaining--)); do
    echo "  倒计时：${remaining}"
    sleep 1
  done

  tmux list-windows -t "${SESSION_NAME}" \
    -F '#{window_name}' | grep -qx B1 || \
    fail "B1 在倒计时期间异常退出，未启动录包"

  tmux new-window -t "${SESSION_NAME}" -n BAG \
    -c "${WORKSPACE_DIR}" \
    "ros2 bag record -o ${bag_path} /utlidar/cloud_base /utlidar/robot_odom /maze/perception/dry_run_status /maze/operator_marker"

  # 等待 rosbag 完成 Topic 发现后再发布起点标记，保证标记进入同一录包。
  sleep 3
  tmux list-windows -t "${SESSION_NAME}" \
    -F '#{window_name}' | grep -qx BAG || \
    fail "rosbag 启动失败"
  ros2 topic pub -t 1 --keep-alive 1.0 \
    /maze/operator_marker std_msgs/msg/String \
    "{data: \"${marker}\"}"

  cleanup_pending=false
  trap - EXIT INT TERM

  echo "录包已启动，可以开始人工行走。"
  echo "录包目录：${bag_path}"
  echo "查看状态：bash scripts/maze_manual_recorder.sh status"
  echo "安全停止：bash scripts/maze_manual_recorder.sh stop"
}

main() {
  local command="${1:-}"
  case "${command}" in
    start)
      shift
      start_recording "$@"
      ;;
    status)
      [[ $# -eq 1 ]] || fail "status 不接受其他参数"
      status_recording
      ;;
    stop)
      [[ $# -eq 1 ]] || fail "stop 不接受其他参数"
      stop_session_safely
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
