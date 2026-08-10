// 一号平台归位动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_platform1_return_home_action eth1 --execute --power-on
// 本目标复用安全状态机：先对齐动作五的抬升姿态，再由动作五安全归位至动作一并保持使能。

#define D1_PLATFORM1_RETURN_HOME_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
