#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>

namespace rk_arm {

enum class ArmState { kNotReady, kReady, kExecuting, kStopping, kFault, kStale };
enum class ArmError {
  kOk = 0, kManualMotionDisabled, kWriterLockMissing, kDdsUnavailable,
  kFeedbackMissing, kFeedbackStale, kSourcesInconsistent, kStatusAbnormal,
  kBusy, kInvalidRequest, kLimitExceeded, kCommandSchemaUnconfirmed,
  kStopSchemaUnconfirmed, kDdsSendFailed
};

struct AxisLimit { double minimum; double maximum; double max_step; double max_speed; };
struct ArmLimits { std::array<AxisLimit, 7> axis; double max_timeout_sec{0.5}; };
struct FeedbackSnapshot {
  bool angle_valid{false}; bool servo_valid{false}; bool dds_ready{false}; bool writer_lock_held{false};
  std::array<double, 7> app_values{}; std::array<double, 7> servo_values{};
  std::chrono::steady_clock::time_point latest_angle{};
  std::chrono::steady_clock::time_point latest_servo{};
  int enable_status{-1}; int power_status{-1}; int error_status{-1};
};
struct MotionRequest { int axis{-1}; double target{0.0}; double step{0.0}; double speed{0.0}; double timeout_sec{0.0}; };
struct SafetyDecision { bool accepted{false}; ArmError error{ArmError::kFeedbackMissing}; std::string reason; ArmState state{ArmState::kNotReady}; };

/** 集中动作准入：任何不确定条件均拒绝，避免调用方遗漏安全门。 */
class ArmSafetyCore {
 public:
  ArmSafetyCore(ArmLimits limits, double feedback_timeout_sec, double source_tolerance,
                bool manual_motion_enabled);
  SafetyDecision CheckMotion(const FeedbackSnapshot& feedback, const MotionRequest& request,
                             bool command_schema_confirmed, bool command_active,
                             std::chrono::steady_clock::time_point now) const;
  SafetyDecision CheckStop(bool writer_lock_held, bool dds_ready,
                           bool stop_schema_confirmed) const;
  bool SourcesConsistent(const FeedbackSnapshot& feedback) const;
  ArmState CurrentState(const FeedbackSnapshot& feedback,
                        std::chrono::steady_clock::time_point now) const;

 private:
  ArmLimits limits_; double feedback_timeout_sec_; double source_tolerance_; bool manual_motion_enabled_;
};

const char* ToString(ArmError value);
const char* ToString(ArmState value);

}  // namespace rk_arm
