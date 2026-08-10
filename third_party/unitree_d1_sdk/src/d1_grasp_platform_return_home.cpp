// 抓取平台预设抓取归位动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_grasp_platform_return_home eth1 --execute --power-on
// 此目标复用准备动作的安全状态机，但只从当前反馈姿态分段回到位置一。

#define D1_RETURN_HOME_ONLY 1
#include "d1_grasp_platform_preset_prepare.cpp"
