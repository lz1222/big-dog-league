#include "rk_arm/d1_command_encoder.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace rk_arm {

std::optional<std::string> D1CommandEncoder::EncodeSingleJointTarget(
    int sequence, int command_id, double target_app_value, int delay_ms, std::string* error) {
  // 形状来自 third_party/unitree_d1_sdk/src/joint_angle_control.cpp：funcode=1,data{id,angle,delay_ms}。
  if (sequence < 0 || command_id < 0 || !std::isfinite(target_app_value) || delay_ms < 0) {
    if (error) *error = "invalid command fields";
    return std::nullopt;
  }
  std::ostringstream output;
  output << "{\"seq\":" << sequence << ",\"address\":1,\"funcode\":1,\"data\":{\"id\":"
         << command_id << ",\"angle\":" << std::fixed << std::setprecision(3)
         << target_app_value << ",\"delay_ms\":" << delay_ms << "}}";
  return output.str();
}

std::optional<std::string> D1CommandEncoder::EncodeStop(std::string* error) {
  // SDK 的 funcode=7 示例名为 arm_zero_control，未证明其为停止；禁止错误复用。
  if (error) *error = "STOP_SCHEMA_UNCONFIRMED";
  return std::nullopt;
}

}  // namespace rk_arm
