#include "rk_arm_feedback_probe/startup_home_validation.hpp"

#include <cassert>

int main() {
  rk_arm_feedback_probe::StartupHomeValidator validator;
  const auto target = rk_arm_feedback_probe::kStartupHomeTargets;
  for (int index = 0; index < 12; ++index) {
    auto angle = target;
    auto servo = target;
    angle[0] += index % 2 == 0 ? 0.05 : -0.05;
    servo[0] += index % 2 == 0 ? 0.05 : -0.05;
    validator.RecordAngles(angle, index * 50000000LL);
    validator.RecordServos(servo, index * 50000000LL + 10000000LL);
    validator.RecordStatus(1, 0, 0);
  }
  const auto summary = validator.Summarize(20.0);
  assert(summary.angle_stats[0].frames == 12);
  assert(summary.angle_stats[0].peak_to_peak == 0.1);
  assert(summary.home_tolerance[0] == 0.2);
  assert(summary.max_source_difference[0] == 0.0);
  assert(summary.source_consistent);
  assert(summary.recommended_confirmation_frames == 10);
  assert(summary.arm_home_confirmed);

  rk_arm_feedback_probe::StartupHomeValidator unstable;
  for (int index = 0; index < 10; ++index) {
    auto angle = target;
    angle[1] += index == 9 ? 1.0 : 0.0;
    unstable.RecordAngles(angle, index * 50000000LL);
    unstable.RecordServos(target, index * 50000000LL);
    unstable.RecordStatus(1, 0, 0);
  }
  const auto unstable_summary = unstable.Summarize(20.0);
  assert(unstable_summary.needs_manual_check[1]);
  assert(unstable_summary.home_tolerance[1] == 0.5);
  assert(!unstable_summary.arm_home_confirmed);
  return 0;
}
