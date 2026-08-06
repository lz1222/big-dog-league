#!/bin/bash
# 迷宫自主导航一键启动
# 前置: Go2 App已启动SLAM/3D建图
set -euo pipefail

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source ~/rk_inspection_ws/install/setup.bash

echo "============================================"
echo " 迷宫自主导航启动"
echo "============================================"

# 1. SDK UDP Server (隔离CycloneDDS)
echo "[1/3] Starting SDK UDP Server..."
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py \
    /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_udp_server &
SDK_PID=$!
sleep 3

# 2. cmd_vel UDP Forwarder (ROS2 domain 10)
echo "[2/3] Starting cmd_vel forwarder..."
export ROS_DOMAIN_ID=10
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py &
FWD_PID=$!
sleep 2

# 3. Realtime Maze Controller (ROS2 domain 10, dry_run=false!)
echo "[3/3] Starting maze controller..."
export ROS_DOMAIN_ID=10
ros2 run rk_maze realtime_maze_controller --ros-args \
    -p dry_run:=true \
    -p enable_motion:=false \
    -p armed:=false &
CTRL_PID=$!

echo ""
echo "============================================"
echo " 全栈运行中:"
echo "   SDK Server:  PID=$SDK_PID"
echo "   Forwarder:   PID=$FWD_PID"
echo "   Controller:  PID=$CTRL_PID (dry_run)"
echo ""
echo " 观察诊断: ros2 topic echo /maze/realtime/status"
echo " 停止:      kill $SDK_PID $FWD_PID $CTRL_PID"
echo "============================================"

wait
