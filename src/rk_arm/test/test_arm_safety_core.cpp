#include "rk_arm/arm_safety_core.hpp"
#include <cassert>
using namespace rk_arm;
int main() {
  ArmLimits limits{}; for (auto& axis : limits.axis) axis = {-1, 1, .1, .1}; limits.max_timeout_sec = .5;
  const auto now = std::chrono::steady_clock::now(); FeedbackSnapshot feedback{}; feedback.dds_ready = feedback.angle_valid = feedback.servo_valid = feedback.writer_lock_held = true; feedback.latest_angle = feedback.latest_servo = now; feedback.enable_status = 1; feedback.power_status = 0; feedback.error_status = 0;
  ArmSafetyCore disabled(limits, .3, .5, false); MotionRequest request{0, .05, .05, .05, .2};
  assert(disabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kManualMotionDisabled);
  ArmSafetyCore enabled(limits, .3, .5, true); assert(enabled.CheckMotion(feedback, request, false, false, now).error == ArmError::kCommandSchemaUnconfirmed);
  feedback.writer_lock_held = false; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kWriterLockMissing); feedback.writer_lock_held = true;
  feedback.servo_values[0] = 1.0; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kSourcesInconsistent);
  feedback.servo_values[0] = 0.0; feedback.latest_angle = now - std::chrono::seconds(1); assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kFeedbackStale);
  feedback.latest_angle = now; request.target = 2.0; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kLimitExceeded);
  request.target = .05; request.step = .2; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kLimitExceeded);
  request.step = .05; request.speed = .2; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kLimitExceeded);
  request.speed = .05; request.timeout_sec = 1.0; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kLimitExceeded);
  request.timeout_sec = .2; request.axis = 7; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kInvalidRequest);
  request.axis = 0; feedback.error_status = 1; assert(enabled.CheckMotion(feedback, request, true, false, now).error == ArmError::kStatusAbnormal);
  assert(enabled.CheckStop(true, true, false).error == ArmError::kStopSchemaUnconfirmed);
}
