#pragma once

#include <array>
#include <cstdint>

namespace rk_arm {

/** 同机 DDS reader 到 ROS 状态节点的固定二进制帧；隔离两套 DDS 运行时。 */
struct FeedbackBridgePacket {
  static constexpr std::uint32_t kMagic = 0x524B4131U;  // "RKA1"
  std::uint32_t magic{kMagic};
  std::uint64_t angle_monotonic_ns{0};
  std::uint64_t servo_monotonic_ns{0};
  std::uint8_t angle_valid{0};
  std::uint8_t servo_valid{0};
  std::array<double, 7> app_values{};
  std::array<double, 7> servo_values{};
  std::int32_t enable_status{-1};
  std::int32_t power_status{-1};
  std::int32_t error_status{-1};
};

}  // namespace rk_arm
