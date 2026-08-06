#include "rk_arm/arm_safety_core.hpp"

#include <cmath>

namespace rk_arm {
ArmSafetyCore::ArmSafetyCore(ArmLimits limits, double feedback_timeout_sec,
                             double source_tolerance, bool manual_motion_enabled)
    : limits_(limits), feedback_timeout_sec_(feedback_timeout_sec),
      source_tolerance_(source_tolerance), manual_motion_enabled_(manual_motion_enabled) {}

bool ArmSafetyCore::SourcesConsistent(const FeedbackSnapshot& feedback) const {
  if (!feedback.angle_valid || !feedback.servo_valid) return false;
  for (std::size_t index = 0; index < feedback.app_values.size(); ++index) {
    if (!std::isfinite(feedback.app_values[index]) || !std::isfinite(feedback.servo_values[index]) ||
        std::abs(feedback.app_values[index] - feedback.servo_values[index]) > source_tolerance_) return false;
  }
  return true;
}

ArmState ArmSafetyCore::CurrentState(const FeedbackSnapshot& feedback,
                                     std::chrono::steady_clock::time_point now) const {
  if (!feedback.dds_ready || !feedback.angle_valid || !feedback.servo_valid) return ArmState::kNotReady;
  const auto angle_age = std::chrono::duration<double>(now - feedback.latest_angle).count();
  const auto servo_age = std::chrono::duration<double>(now - feedback.latest_servo).count();
  if (angle_age > feedback_timeout_sec_ || servo_age > feedback_timeout_sec_) return ArmState::kStale;
  if (!SourcesConsistent(feedback) || feedback.enable_status != 1 || feedback.power_status != 0 || feedback.error_status != 0) return ArmState::kFault;
  return ArmState::kReady;
}

SafetyDecision ArmSafetyCore::CheckMotion(const FeedbackSnapshot& feedback, const MotionRequest& request,
                                          bool command_schema_confirmed, bool command_active,
                                          std::chrono::steady_clock::time_point now) const {
  const auto reject = [](ArmError error, const char* reason, ArmState state) { return SafetyDecision{false, error, reason, state}; };
  const ArmState state = CurrentState(feedback, now);
  if (!manual_motion_enabled_) return reject(ArmError::kManualMotionDisabled, "manual_motion_enabled=false", state);
  if (!feedback.writer_lock_held) return reject(ArmError::kWriterLockMissing, "writer lock is not held", state);
  if (!feedback.dds_ready) return reject(ArmError::kDdsUnavailable, "DDS is unavailable", state);
  if (!feedback.angle_valid || !feedback.servo_valid) return reject(ArmError::kFeedbackMissing, "valid dual-source feedback required", state);
  if (state == ArmState::kStale) return reject(ArmError::kFeedbackStale, "feedback is stale", state);
  if (!SourcesConsistent(feedback)) return reject(ArmError::kSourcesInconsistent, "feedback sources differ", state);
  if (feedback.enable_status != 1 || feedback.power_status != 0 || feedback.error_status != 0) return reject(ArmError::kStatusAbnormal, "funcode=3 status is not observed-normal", state);
  if (command_active) return reject(ArmError::kBusy, "another command is active", ArmState::kExecuting);
  if (!command_schema_confirmed) return reject(ArmError::kCommandSchemaUnconfirmed, "COMMAND_SCHEMA_UNCONFIRMED", state);
  if (request.axis < 0 || request.axis >= static_cast<int>(limits_.axis.size()) || !std::isfinite(request.target) ||
      !std::isfinite(request.step) || !std::isfinite(request.speed) || !std::isfinite(request.timeout_sec)) return reject(ArmError::kInvalidRequest, "non-finite or invalid request", state);
  const AxisLimit& limit = limits_.axis[request.axis];
  if (std::abs(request.step) > limit.max_step || request.speed <= 0.0 || request.speed > limit.max_speed ||
      request.timeout_sec <= 0.0 || request.timeout_sec > limits_.max_timeout_sec ||
      request.target < limit.minimum || request.target > limit.maximum) return reject(ArmError::kLimitExceeded, "request exceeds DEVELOPMENT DEFAULT limits", state);
  return {true, ArmError::kOk, "accepted", ArmState::kReady};
}

SafetyDecision ArmSafetyCore::CheckStop(bool writer_lock_held, bool dds_ready, bool stop_schema_confirmed) const {
  if (!writer_lock_held) return {false, ArmError::kWriterLockMissing, "writer lock is not held", ArmState::kFault};
  if (!dds_ready) return {false, ArmError::kDdsUnavailable, "DDS is unavailable", ArmState::kFault};
  if (!stop_schema_confirmed) return {false, ArmError::kStopSchemaUnconfirmed, "STOP_SCHEMA_UNCONFIRMED", ArmState::kFault};
  return {true, ArmError::kOk, "stop accepted", ArmState::kStopping};
}

const char* ToString(ArmError value) {
  switch (value) { case ArmError::kOk: return "OK"; case ArmError::kManualMotionDisabled: return "MANUAL_MOTION_DISABLED"; case ArmError::kWriterLockMissing: return "WRITER_LOCK_MISSING"; case ArmError::kDdsUnavailable: return "DDS_UNAVAILABLE"; case ArmError::kFeedbackMissing: return "FEEDBACK_MISSING"; case ArmError::kFeedbackStale: return "FEEDBACK_STALE"; case ArmError::kSourcesInconsistent: return "SOURCES_INCONSISTENT"; case ArmError::kStatusAbnormal: return "STATUS_ABNORMAL"; case ArmError::kBusy: return "COMMAND_BUSY"; case ArmError::kInvalidRequest: return "INVALID_REQUEST"; case ArmError::kLimitExceeded: return "LIMIT_EXCEEDED"; case ArmError::kCommandSchemaUnconfirmed: return "COMMAND_SCHEMA_UNCONFIRMED"; case ArmError::kStopSchemaUnconfirmed: return "STOP_SCHEMA_UNCONFIRMED"; case ArmError::kDdsSendFailed: return "DDS_SEND_FAILED"; } return "UNKNOWN";
}
const char* ToString(ArmState value) { switch (value) { case ArmState::kNotReady: return "NOT_READY"; case ArmState::kReady: return "READY"; case ArmState::kExecuting: return "EXECUTING"; case ArmState::kStopping: return "STOPPING"; case ArmState::kFault: return "FAULT"; case ArmState::kStale: return "STALE"; } return "UNKNOWN"; }
}  // namespace rk_arm
