#!/usr/bin/env bash
# 迷宫 P0 只读诊断入口：统一 DDS 域与动态库，绝不启动运动桥。

# Foxy 的 setup 脚本会读取未定义的可选变量，故在 source 完成后再开启 nounset。
set -eo pipefail

workspace=/home/unitree/rk_inspection_ws
domain="${1:-0}"

source /opt/ros/foxy/setup.bash
set -u
export LD_LIBRARY_PATH="${workspace}/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/unitree_sdk_runtime:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${domain}"
unset CYCLONEDDS_URI

cd "${workspace}"
exec python3 -u scripts/maze_p0_diagnose.py --domain "${domain}"
