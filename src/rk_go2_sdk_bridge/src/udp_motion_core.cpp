#include "rk_go2_sdk_bridge/udp_motion_core.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace rk_go2_sdk_bridge
{
namespace
{

double ApplyDeadband(double value, double deadband)
{
  return std::fabs(value) <= deadband ? 0.0 : value;
}

bool IsPositiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

UdpMotionCore::UdpMotionCore(MotionLimits limits, double watchdog_sec)
: limits_(limits), watchdog_sec_(watchdog_sec)
{
  if (!IsPositiveFinite(limits_.max_vx) ||
      !IsPositiveFinite(limits_.max_vy) ||
      !IsPositiveFinite(limits_.max_yaw)) {
    throw std::invalid_argument("motion limits must be positive and finite");
  }
  if (!std::isfinite(limits_.deadband) || limits_.deadband < 0.0) {
    throw std::invalid_argument("deadband must be nonnegative and finite");
  }
  if (!IsPositiveFinite(watchdog_sec_)) {
    throw std::invalid_argument("watchdog_sec must be positive and finite");
  }
}

MotionDecision UdpMotionCore::AcceptPacket(
    const std::string& payload, double now_sec)
{
  if (!std::isfinite(now_sec)) {
    return ForceStop("invalid_receive_time");
  }

  MotionDecision parsed = ParsePacket(payload);
  if (parsed.action == MotionAction::kStop) {
    active_ = false;
    target_ = {};
    return parsed;
  }

  target_ = parsed.command;
  last_packet_sec_ = now_sec;
  active_ = true;
  parsed.action = MotionAction::kNone;
  parsed.reason = "move_target_updated";
  return parsed;
}

MotionDecision UdpMotionCore::Tick(double now_sec)
{
  if (!active_) {
    return {};
  }
  if (!std::isfinite(now_sec) || now_sec < last_packet_sec_) {
    return ForceStop("invalid_watchdog_time");
  }

  const double age_sec = now_sec - last_packet_sec_;
  if (age_sec >= watchdog_sec_) {
    return ForceStop("watchdog_timeout");
  }

  return {MotionAction::kMove, target_, "periodic_move"};
}

MotionDecision UdpMotionCore::ForceStop(const std::string& reason)
{
  active_ = false;
  target_ = {};
  return {MotionAction::kStop, {}, reason};
}

bool UdpMotionCore::active() const
{
  return active_;
}

const MotionCommand& UdpMotionCore::target() const
{
  return target_;
}

MotionDecision UdpMotionCore::ParsePacket(const std::string& payload) const
{
  std::istringstream stream(payload);
  MotionCommand command;
  if (!(stream >> command.vx >> command.vy >> command.yaw)) {
    return {MotionAction::kStop, {}, "invalid_packet"};
  }

  stream >> std::ws;
  if (!stream.eof()) {
    return {MotionAction::kStop, {}, "invalid_packet_extra_fields"};
  }
  if (!std::isfinite(command.vx) ||
      !std::isfinite(command.vy) ||
      !std::isfinite(command.yaw)) {
    return {MotionAction::kStop, {}, "nonfinite_packet"};
  }

  // 越界命令不能静默截断，否则上游控制故障会被掩盖。
  if (std::fabs(command.vx) > limits_.max_vx ||
      std::fabs(command.vy) > limits_.max_vy ||
      std::fabs(command.yaw) > limits_.max_yaw) {
    return {MotionAction::kStop, {}, "out_of_range_packet"};
  }

  command.vx = ApplyDeadband(command.vx, limits_.deadband);
  command.vy = ApplyDeadband(command.vy, limits_.deadband);
  command.yaw = ApplyDeadband(command.yaw, limits_.deadband);

  if (IsZero(command)) {
    return {MotionAction::kStop, {}, "zero_command"};
  }
  return {MotionAction::kNone, command, "valid_packet"};
}

bool UdpMotionCore::IsZero(const MotionCommand& command) const
{
  return command.vx == 0.0 && command.vy == 0.0 && command.yaw == 0.0;
}

const char* ToString(MotionAction action)
{
  switch (action) {
    case MotionAction::kNone:
      return "none";
    case MotionAction::kMove:
      return "move";
    case MotionAction::kStop:
      return "stop";
  }
  return "unknown";
}

}  // namespace rk_go2_sdk_bridge
