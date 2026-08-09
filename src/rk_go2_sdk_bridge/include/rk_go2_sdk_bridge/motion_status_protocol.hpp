#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace rk_go2_sdk_bridge
{

// SDK status 是独立观测协议：只报告已经返回的真实 SDK 调用，绝不参与
// UdpMotionCore 的命令解析、限幅或停车判定。
struct MotionStatusEvent
{
  std::string server_instance_id;
  std::uint64_t sequence{0};
  std::string event;
  std::int32_t ret{0};
  std::string reason;
  double vx{0.0};
  double vy{0.0};
  double yaw{0.0};
  std::int64_t server_monotonic_ns{0};
};

inline std::string EscapeJsonString(const std::string& value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u00" << std::hex << std::setw(2)
                 << std::setfill('0') << static_cast<int>(character)
                 << std::dec << std::setfill(' ');
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

// JSON 不允许 NaN/Inf；编码失败返回空串，调用方只记录诊断而不影响停车。
inline std::string EncodeMotionStatusJson(const MotionStatusEvent& status)
{
  if (status.server_instance_id.empty() || status.sequence == 0U ||
      status.event.empty() || status.reason.empty() ||
      status.server_monotonic_ns <= 0 || !std::isfinite(status.vx) ||
      !std::isfinite(status.vy) || !std::isfinite(status.yaw)) {
    return std::string();
  }

  std::ostringstream output;
  output << std::setprecision(std::numeric_limits<double>::max_digits10)
         << '{'
         << "\"server_instance_id\":\""
         << EscapeJsonString(status.server_instance_id) << "\","
         << "\"sequence\":" << status.sequence << ','
         << "\"event\":\"" << EscapeJsonString(status.event) << "\","
         << "\"ret\":" << status.ret << ','
         << "\"reason\":\"" << EscapeJsonString(status.reason) << "\","
         << "\"vx\":" << status.vx << ','
         << "\"vy\":" << status.vy << ','
         << "\"yaw\":" << status.yaw << ','
         << "\"server_monotonic_ns\":" << status.server_monotonic_ns
         << '}';
  return output.str();
}

}  // namespace rk_go2_sdk_bridge
