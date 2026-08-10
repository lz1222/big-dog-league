// 置位动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_reset_to_pose_one_action eth1 --execute --power-on
// 本目标从当前七路反馈出发，以受限分段轨迹回到动作一；到位后保持使能。

#define D1_RESET_TO_POSE_ONE_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
