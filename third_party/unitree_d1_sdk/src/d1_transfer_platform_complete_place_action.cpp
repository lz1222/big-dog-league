// 中转平台完整放置动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_transfer_platform_complete_place_action eth1 --execute --power-on
// 顺序执行：动作一到动作九放置后等待 2 秒、动作九到动作十张开后等待 2 秒、动作十到动作十一抬升、动作十一左移到动作十二。

#define D1_TRANSFER_PLATFORM_COMPLETE_PLACE_ACTION 1
#include "d1_grasp_platform_preset_prepare.cpp"
