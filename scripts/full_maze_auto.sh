#!/bin/bash
# 迷宫全自主 — 自包含，不掉环境
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null
sleep 0.5

# 1. Forwarder (SDK server already running)
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py --ros-args -p max_vx:=0.60 &
sleep 2

# 2. Autonomous walker
echo "=== 迷宫全自主 L-L-R-R-L (vx=0.30) ==="
timeout 120 python3 /home/unitree/rk_inspection_ws/scripts/maze_autonomous_walker.py 2>&1

pkill -f forwarder 2>/dev/null
echo "完成"
