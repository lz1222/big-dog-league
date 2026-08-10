// 一号平台放置动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_platform1_place_action eth1 --execute --power-on
// 本目标复用准备动作安全状态机：先对齐动作一，再移动到动作三并保持使能。

#define D1_PLATFORM1_PLACE_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
