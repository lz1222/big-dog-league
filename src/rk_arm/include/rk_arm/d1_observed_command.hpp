#pragma once

#include "rk_arm/arm_safety_core.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace rk_arm {

/**
 * 官方 App 被动流量中观测到的 funcode=2 完整目标载荷。
 * address=1 与 mode=0 仅是观测值，不是已认证的设备或控制语义。
 */
struct D1ObservedCommand {
  int funcode{-1};
  int address{-1};
  int mode{-1};
  std::uint64_t seq{0};
  std::array<double, 7> angles{};
};

/** 缺少字段、非有限值或不符合已观测外形的 JSON 必须 fail closed。 */
bool ParseD1ObservedCommand(const std::string& payload, D1ObservedCommand* command,
                            std::string* error);

/**
 * 仅重新编码已观测到的 funcode=2/address=1/mode=0 外形。
 * 该函数只是字符串构造器，绝不发送 DDS 数据，也不生成停止或使能命令。
 */
std::optional<std::string> EncodeD1ObservedCommand(const D1ObservedCommand& command,
                                                   std::string* error);

/** 影子生成结果始终是离线预览，禁止将其视为可发送命令。 */
struct ShadowCommandPreview {
  bool accepted{false};
  std::string reason;
  std::string safety_label{"DRY_RUN_ONLY / NOT SENT"};
  std::optional<D1ObservedCommand> command;
  std::optional<std::string> json;
};

/**
 * 从双源一致的最新反馈复制完整目标，仅替换一个已知槽位。
 * 没有停止、使能、失能或发送入口，避免离线编码器扩大为真机控制器。
 */
class D1ShadowCommandGenerator {
 public:
  static ShadowCommandPreview PreviewJoint(const FeedbackSnapshot& feedback, int joint_index,
                                           double target_app_value, std::uint64_t seq,
                                           std::chrono::steady_clock::time_point now,
                                           double feedback_timeout_sec,
                                           double source_tolerance,
                                           bool manual_motion_enabled,
                                           double source_epsilon = 0.0);
  static ShadowCommandPreview PreviewGripper(const FeedbackSnapshot& feedback,
                                             double target_app_value, std::uint64_t seq,
                                             std::chrono::steady_clock::time_point now,
                                             double feedback_timeout_sec,
                                             double source_tolerance,
                                             bool manual_motion_enabled,
                                             double source_epsilon = 0.0);
};

}  // namespace rk_arm
