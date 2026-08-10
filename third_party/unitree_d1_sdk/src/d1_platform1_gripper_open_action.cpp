// 一号平台夹爪张开动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_platform1_gripper_open_action eth1 --execute --power-on
// 本目标复用安全状态机：先对齐动作三，再将爪夹从 25° 张开至动作四的 49°。

#define D1_PLATFORM1_GRIPPER_OPEN_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
