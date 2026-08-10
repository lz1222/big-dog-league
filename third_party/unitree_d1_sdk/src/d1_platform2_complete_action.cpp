// 二号平台完整机械臂动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_platform2_complete_action eth1 --execute --power-on
// 顺序执行：动作一到动作六放置后等待 2 秒、动作六到动作七张开后等待 2 秒、动作七到动作八抬升、动作八归位动作一。

#define D1_PLATFORM2_COMPLETE_ACTION 1
#include "d1_grasp_platform_preset_prepare.cpp"
