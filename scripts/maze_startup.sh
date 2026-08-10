#!/bin/bash
# MAZE_RUNTIME_STABILITY_V1 — 完整迷宫启动流程
# 用法: bash scripts/maze_startup.sh [dry_run|armed]
#   dry_run: 只读传感器，零运动输出 (默认)
#   armed:   真机运动 (危险!)

set -eo pipefail
MODE="${1:-dry_run}"

# ============================================================
# 1. 环境
# ============================================================
source /opt/ros/foxy/setup.bash

export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/thirdparty/lib/aarch64:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

# ============================================================
# 2. 确保 eth1 在线
# ============================================================
# 地址可能仍保留在 DOWN 接口上；同时检查链路状态，避免 DDS 诊断在无网络时
# 被误判为 Python 或 QoS 问题。
if ! ip addr show eth1 | grep -q "192.168.123.18" || \
   ! ip link show dev eth1 | grep -q "state UP"; then
    echo "==> 配置并拉起 eth1..."
    sudo ip link set dev eth1 up
    sudo ip addr add 192.168.123.18/24 dev eth1 2>/dev/null || true
fi

# ============================================================
# 3. 机器人进入 Sport 模式
# ============================================================
echo "==> 检查机器人状态..."
python3 -c "
import sys, time
sys.path.insert(0, '/home/unitree/Documents/NoMachine/unitree_sdk2_python')
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, 'eth1')

# 先看当前模式
mode_val = [0]
def cb(msg): mode_val[0] = msg.mode
sub = ChannelSubscriber('rt/sportmodestate', SportModeState_)
sub.Init(cb, 10)
time.sleep(0.5)

client = SportClient()
client.SetTimeout(5.0)
client.Init()

if mode_val[0] != 0:
    print(f'  机器人已在模式{mode_val[0]}，跳过 StandUp')
    client.StopMove()
else:
    print('  进入 Sport 模式...')
    client.Damp(); time.sleep(0.5)
    client.StandUp(); time.sleep(0.5)
    client.StopMove()
print('  机器人已就绪')
" 2>&1

# ============================================================
# 4. 启动桥接层 (SDK server + forwarder)
# ============================================================
echo "==> 启动桥接层..."
source install/setup.bash 2>/dev/null
ros2 launch rk_go2_sdk_bridge go2_sdk_udp_bridge.launch.py \
    sdk_network_interface:=eth1 &
BRIDGE_PID=$!
sleep 3

# 恢复干净环境给 maze 控制器
unset AMENT_PREFIX_PATH
unset PYTHONPATH
source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/thirdparty/lib/aarch64:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

# ============================================================
# 5. 运行迷宫控制器
# ============================================================
echo "==> 启动迷宫控制器 ($MODE)..."
if [ "$MODE" = "armed" ]; then
    python3 -u scripts/maze_full_auto_v2.py --armed
else
    python3 -u scripts/maze_full_auto_v2.py
fi

# ============================================================
# 6. 清理
# ============================================================
echo "==> 停止桥接层..."
kill $BRIDGE_PID 2>/dev/null || true
wait $BRIDGE_PID 2>/dev/null || true
echo "完成。"
