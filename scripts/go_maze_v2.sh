#!/bin/bash
# V2 迷宫自主 — 使用 go2_sdk_udp_server_v2 + V2 Forwarder
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null
pkill -f sdk_udp 2>/dev/null
sleep 1

echo "[1/3] V2 SDK Server (50Hz)..."
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_server_runtime.py \
    /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_udp_server_v2 \
    --max-vx 0.60 --rate-hz 50.0 --watchdog-sec 0.30 &
sleep 3

echo "[2/3] V2 Forwarder (50Hz)..."
python3 /home/unitree/rk_inspection_ws/scripts/v2_forwarder.py --ros-args -p max_vx:=0.60 -p publish_rate_hz:=50.0 &
sleep 2

echo "[3/3] 迷宫自主..."
timeout 120 python3 /home/unitree/rk_inspection_ws/scripts/maze_full_auto.py 2>&1

pkill -f forwarder 2>/dev/null
pkill -f sdk_udp 2>/dev/null
echo "完成"
