#include "d1_gripper_tx_probe/gripper_tx_core.hpp"
#include "rk_arm/single_writer_guard.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace {
rk_arm::FeedbackSnapshot ValidFeedback(std::chrono::steady_clock::time_point now) {
  rk_arm::FeedbackSnapshot feedback{}; feedback.dds_ready = feedback.angle_valid = feedback.servo_valid = true;
  feedback.latest_angle = feedback.latest_servo = now; feedback.enable_status = 1; feedback.power_status = 0; feedback.error_status = 0;
  feedback.app_values = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -19.8}; feedback.servo_values = feedback.app_values;
  return feedback;
}
d1_gripper_tx_probe::WriterLockInspection Available() { return {d1_gripper_tx_probe::WriterLockState::kAvailable, "no local writer lock"}; }
d1_gripper_tx_probe::GripperPreviewRequest ValidRequest(std::chrono::steady_clock::time_point now) { return {ValidFeedback(now), 60504, 1.0, Available()}; }
}

int main() {
  const auto now = std::chrono::steady_clock::now();
  auto request = ValidRequest(now); auto preview = d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now);
  assert(preview.accepted && preview.command && preview.json && preview.safety_label == "DRY_RUN_ONLY / NOT SENT");
  rk_arm::D1ObservedCommand parsed{}; std::string error;
  assert(rk_arm::ParseD1ObservedCommand(*preview.json, &parsed, &error));
  assert(parsed.seq == *request.seq && parsed.funcode == 2 && parsed.address == 1 && parsed.mode == 0);
  for (int index = 0; index < 6; ++index) assert(preview.command->angles[index] == request.feedback.app_values[index]);
  assert(preview.command->angles[6] == request.feedback.app_values[6] + 1.0);
  request.delta = -1.0; assert(d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request.delta = 1.01; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request.delta = -1.01; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.angle_valid = false; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.latest_servo = now - std::chrono::seconds(1); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.servo_values[0] += 1.0; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.enable_status = 0; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.power_status = 1; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.error_status = 1; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.app_values[6] = std::numeric_limits<double>::infinity(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.seq.reset(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.seq = std::numeric_limits<std::uint64_t>::max(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.writer_lock = {d1_gripper_tx_probe::WriterLockState::kActive, "active local writer PID 1"}; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  assert(!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(false, "SEND_ONE_GRIPPER_TARGET"));
  assert(!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, "wrong"));
  assert(d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, "SEND_ONE_GRIPPER_TARGET"));
  const auto lock_path = std::filesystem::temp_directory_path() / "d1_gripper_tx_probe_test.lock";
  std::filesystem::remove(lock_path);
  { rk_arm::SingleWriterGuard lock(lock_path); std::string error; assert(lock.Acquire(&error)); assert(std::filesystem::exists(lock_path)); }
  assert(!std::filesystem::exists(lock_path));
  { std::ofstream stale(lock_path); stale << "999999\n"; }
  assert(d1_gripper_tx_probe::InspectWriterLock(lock_path).state == d1_gripper_tx_probe::WriterLockState::kStale);
  std::filesystem::remove(lock_path);
  d1_gripper_tx_probe::SingleFrameBudget budget;
  assert(budget.Reserve() && budget.sent_count() == 1U);
  assert(!budget.Reserve() && budget.sent_count() == 1U);
}
