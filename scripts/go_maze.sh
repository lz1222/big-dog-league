#!/bin/bash
# 迷宫自主 — SDK桥接+感知+状态机 一步到位
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null; sleep 0.5

echo "[1/2] Forwarder (max_vx=0.60)..."
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py --ros-args -p max_vx:=0.60 &
sleep 2

echo "[2/2] 自主迷宫 L-L-R-R-L"
echo "========================================"
timeout 150 python3 /home/unitree/rk_inspection_ws/scripts/maze_full_auto.py 2>&1
echo "========================================"
echo "完成。"
