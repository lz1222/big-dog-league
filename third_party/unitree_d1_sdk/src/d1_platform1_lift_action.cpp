// 一号平台抬升动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_platform1_lift_action eth1 --execute --power-on
// 本目标复用安全状态机：先对齐动作四，再由动作四抬升至动作五并保持使能。

#define D1_PLATFORM1_LIFT_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
