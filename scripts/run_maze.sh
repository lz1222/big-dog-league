#!/bin/bash
# 迷宫 Dry Run 启动器：必须复用 P0 已验证的 Unitree DDS runtime，避免系统库混载段错误。
source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/unitree_sdk_runtime:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI
cd /home/unitree/rk_inspection_ws
python3 -u scripts/maze_full_auto_v2.py
