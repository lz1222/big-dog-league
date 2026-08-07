#pragma once

#include <string>

namespace rk_go2_sdk_bridge
{

// UDP 运动协议只包含三个速度分量；限制值同时用于输入校验和安全边界。
struct MotionLimits
{
  double max_vx{0.25};
  double max_vy{0.05};
  double max_yaw{0.60};
  double deadband{0.01};
};

struct MotionCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw{0.0};
};

enum class MotionAction
{
  kNone,
  kMove,
  kStop,
};

struct MotionDecision
{
  MotionAction action{MotionAction::kNone};
  MotionCommand command{};
  std::string reason;
};

// 该核心不依赖 ROS 和 Unitree SDK，负责报文校验、状态保持及 watchdog 判定。
class UdpMotionCore
{
public:
  UdpMotionCore(MotionLimits limits, double watchdog_sec);

  // 接收报文只更新目标；零速、非法和越界报文会立即返回停车决策。
  MotionDecision AcceptPacket(const std::string& payload, double now_sec);

  // 服务端以固定频率调用 Tick，持续输出最新目标并检查消息新鲜度。
  MotionDecision Tick(double now_sec);

  // SDK 调用失败或套接字异常时，强制清除目标并生成停车决策。
  MotionDecision ForceStop(const std::string& reason);

  bool active() const;
  const MotionCommand& target() const;

private:
  MotionDecision ParsePacket(const std::string& payload) const;
  bool IsZero(const MotionCommand& command) const;

  MotionLimits limits_;
  double watchdog_sec_;
  bool active_{false};
  double last_packet_sec_{0.0};
  MotionCommand target_{};
};

const char* ToString(MotionAction action);

}  // namespace rk_go2_sdk_bridge
