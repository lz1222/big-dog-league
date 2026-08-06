#!/bin/bash
# 迷宫融合导航验证录包
# 用法: bash record_maze_fusion_bag.sh [标签]
#
# 录制话题:
#   /utlidar/cloud_base   - LiDAR点云 (~15Hz)
#   /utlidar/robot_odom   - 里程计    (~149Hz)
#   /utlidar/imu          - IMU       (~250Hz)   ← 新增
#   /lowstate             - 关节状态  (~200Hz)   ← 新增(JointHealthGuard)
#
# 输出: evidence/fusion_bags/<标签>_<时间>/

set -euo pipefail

LABEL="${1:-manual}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="$HOME/rk_inspection_ws/evidence/fusion_bags/${LABEL}_${TIMESTAMP}"

mkdir -p "$BAG_DIR"

TOPICS=(
    /utlidar/cloud_base
    /utlidar/robot_odom
    /utlidar/imu
    /lowstate
)

echo "============================================"
echo " 迷宫融合验证录包"
echo "============================================"
echo " 标签:   $LABEL"
echo " 时间:   $TIMESTAMP"
echo " 输出:   $BAG_DIR"
echo " 话题:"
for t in "${TOPICS[@]}"; do
    echo "    $t"
done
echo ""
echo "按 Ctrl-C 停止录制"
echo "============================================"

source /opt/ros/foxy/setup.bash
ros2 bag record \
    -o "$BAG_DIR" \
    "${TOPICS[@]}"

echo ""
echo "录制完成: $BAG_DIR"
echo "验证: ros2 bag info $BAG_DIR"
