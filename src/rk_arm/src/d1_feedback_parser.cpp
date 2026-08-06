#include "rk_arm/d1_feedback_parser.hpp"

#include <cmath>
#include <regex>

namespace rk_arm {
namespace {
bool FieldNumber(const std::string& text, const std::string& key, double* value) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?(?:0|[1-9][0-9]*)(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  try { *value = std::stod(match[1].str()); } catch (...) { return false; }
  return std::isfinite(*value);
}

bool FieldInt(const std::string& text, const std::string& key, int* value) {
  double parsed = 0.0;
  if (!FieldNumber(text, key, &parsed) || std::floor(parsed) != parsed) return false;
  *value = static_cast<int>(parsed);
  return true;
}
}  // namespace

bool ParseD1Feedback(const std::string& payload, D1FeedbackFrame* frame) {
  if (frame == nullptr) return false;
  *frame = D1FeedbackFrame{};
  // 只接受 JSON 对象外形，避免从任意日志文本误提取字段造成假就绪。
  const auto first = payload.find_first_not_of(" \t\r\n");
  const auto last = payload.find_last_not_of(" \t\r\n");
  if (first == std::string::npos || payload[first] != '{' || payload[last] != '}') {
    frame->error = "payload is not a JSON object"; return false;
  }
  if (!FieldInt(payload, "funcode", &frame->funcode)) { frame->error = "missing or invalid funcode"; return false; }
  if (frame->funcode == 1) {
    std::array<double, 7> values{};
    for (int index = 0; index < 7; ++index) {
      if (!FieldNumber(payload, "angle" + std::to_string(index), &values[index])) {
        frame->error = "missing or invalid angle" + std::to_string(index); return false;
      }
    }
    frame->app_values = values;
    return true;
  }
  if (frame->funcode == 3) {
    int enable = 0, power = 0, error = 0;
    if (!FieldInt(payload, "enable_status", &enable) || !FieldInt(payload, "power_status", &power) || !FieldInt(payload, "error_status", &error)) {
      frame->error = "missing or invalid status field"; return false;
    }
    frame->enable_status = enable; frame->power_status = power; frame->error_status = error;
    return true;
  }
  frame->error = "unobserved funcode";
  return false;
}

}  // namespace rk_arm
