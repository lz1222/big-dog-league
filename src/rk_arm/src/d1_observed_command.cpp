#include "rk_arm/d1_observed_command.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>

namespace rk_arm {
namespace {

bool IsJsonObject(const std::string& payload) {
  const auto first = payload.find_first_not_of(" \t\r\n");
  const auto last = payload.find_last_not_of(" \t\r\n");
  return first != std::string::npos && payload[first] == '{' && payload[last] == '}';
}

bool FieldNumber(const std::string& text, const std::string& key, double* value) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?(?:0|[1-9][0-9]*)(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  try { *value = std::stod(match[1].str()); } catch (...) { return false; }
  return std::isfinite(*value);
}

bool FieldInt(const std::string& text, const std::string& key, int* value) {
  double parsed = 0.0;
  if (!FieldNumber(text, key, &parsed) || std::floor(parsed) != parsed ||
      parsed < static_cast<double>(std::numeric_limits<int>::min()) ||
      parsed > static_cast<double>(std::numeric_limits<int>::max())) return false;
  *value = static_cast<int>(parsed);
  return true;
}

bool FieldUnsigned(const std::string& text, const std::string& key, std::uint64_t* value) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  try { *value = std::stoull(match[1].str()); } catch (...) { return false; }
  return true;
}

bool IsObservedShape(const D1ObservedCommand& command, std::string* error) {
  if (command.funcode != 2 || command.address != 1 || command.mode != 0) {
    if (error) *error = "only observed funcode=2/address=1/mode=0 may be encoded";
    return false;
  }
  for (const double value : command.angles) {
    if (!std::isfinite(value)) {
      if (error) *error = "non-finite angle is rejected";
      return false;
    }
  }
  return true;
}

ShadowCommandPreview PreviewFromFeedback(const FeedbackSnapshot& feedback, int channel,
                                         double target_app_value, std::uint64_t seq,
                                         std::chrono::steady_clock::time_point now,
                                         double feedback_timeout_sec,
                                         double source_tolerance,
                                         bool manual_motion_enabled,
                                         double source_epsilon) {
  ShadowCommandPreview result;
  // 安全原因：影子模式只能在正式手动模式关闭时使用，避免成为旁路控制入口。
  if (manual_motion_enabled) { result.reason = "manual_motion_enabled must remain false"; return result; }
  if (channel < 0 || channel >= 7 || !std::isfinite(target_app_value) ||
      feedback_timeout_sec <= 0.0 || source_tolerance < 0.0 || source_epsilon < 0.0) {
    result.reason = "invalid shadow request"; return result;
  }
  if (seq == std::numeric_limits<std::uint64_t>::max()) {
    result.reason = "SEQ_OVERFLOW_RISK"; return result;
  }
  if (!feedback.dds_ready || !feedback.angle_valid || !feedback.servo_valid) {
    result.reason = "valid dual-source feedback required"; return result;
  }
  const auto angle_age = std::chrono::duration<double>(now - feedback.latest_angle).count();
  const auto servo_age = std::chrono::duration<double>(now - feedback.latest_servo).count();
  if (angle_age < 0.0 || servo_age < 0.0 || angle_age > feedback_timeout_sec ||
      servo_age > feedback_timeout_sec) {
    result.reason = "feedback is stale"; return result;
  }
  for (std::size_t index = 0; index < feedback.app_values.size(); ++index) {
    // epsilon 只用于固定绝对容差的二进制边界，不改变调用方声明的物理容差。
    if (!std::isfinite(feedback.app_values[index]) || !std::isfinite(feedback.servo_values[index]) ||
        std::abs(feedback.app_values[index] - feedback.servo_values[index]) > source_tolerance + source_epsilon) {
      result.reason = "feedback sources differ"; return result;
    }
  }

  D1ObservedCommand command{2, 1, 0, seq, feedback.app_values};
  command.angles[static_cast<std::size_t>(channel)] = target_app_value;
  std::string error;
  auto json = EncodeD1ObservedCommand(command, &error);
  if (!json) { result.reason = error; return result; }
  result.accepted = true;
  result.reason = "DRY_RUN_ONLY / NOT SENT";
  result.command = command;
  result.json = std::move(json);
  return result;
}

}  // namespace

bool ParseD1ObservedCommand(const std::string& payload, D1ObservedCommand* command,
                            std::string* error) {
  if (command == nullptr) return false;
  *command = D1ObservedCommand{};
  if (!IsJsonObject(payload)) { if (error) *error = "payload is not a JSON object"; return false; }
  if (!FieldInt(payload, "funcode", &command->funcode) ||
      !FieldInt(payload, "address", &command->address) ||
      !FieldUnsigned(payload, "seq", &command->seq) ||
      !FieldInt(payload, "mode", &command->mode)) {
    if (error) *error = "missing or invalid command header";
    return false;
  }
  for (int index = 0; index < 7; ++index) {
    if (!FieldNumber(payload, "angle" + std::to_string(index), &command->angles[index])) {
      if (error) *error = "missing or invalid angle" + std::to_string(index);
      return false;
    }
  }
  if (!IsObservedShape(*command, error)) return false;
  return true;
}

std::optional<std::string> EncodeD1ObservedCommand(const D1ObservedCommand& command,
                                                   std::string* error) {
  if (!IsObservedShape(command, error)) return std::nullopt;
  std::ostringstream output;
  output << std::setprecision(17) << "{\"seq\":" << command.seq
         << ",\"address\":" << command.address << ",\"funcode\":" << command.funcode
         << ",\"data\":{\"mode\":" << command.mode;
  for (int index = 0; index < 7; ++index) output << ",\"angle" << index << "\":" << command.angles[index];
  output << "}}";
  return output.str();
}

ShadowCommandPreview D1ShadowCommandGenerator::PreviewJoint(
    const FeedbackSnapshot& feedback, int joint_index, double target_app_value, std::uint64_t seq,
    std::chrono::steady_clock::time_point now, double feedback_timeout_sec,
    double source_tolerance, bool manual_motion_enabled, double source_epsilon) {
  // 关节接口永远不能借用第七路；夹爪必须走独立接口以避免索引混淆。
  if (joint_index < 0 || joint_index > 5) {
    ShadowCommandPreview result; result.reason = "joint index must be in [0,5]"; return result;
  }
  return PreviewFromFeedback(feedback, joint_index, target_app_value, seq, now,
                             feedback_timeout_sec, source_tolerance, manual_motion_enabled, source_epsilon);
}

ShadowCommandPreview D1ShadowCommandGenerator::PreviewGripper(
    const FeedbackSnapshot& feedback, double target_app_value, std::uint64_t seq,
    std::chrono::steady_clock::time_point now, double feedback_timeout_sec,
    double source_tolerance, bool manual_motion_enabled, double source_epsilon) {
  return PreviewFromFeedback(feedback, 6, target_app_value, seq, now,
                             feedback_timeout_sec, source_tolerance, manual_motion_enabled, source_epsilon);
}

}  // namespace rk_arm
