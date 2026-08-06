#include "d1_gripper_tx_probe/gripper_tx_core.hpp"

#include "rk_arm/single_writer_guard.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
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
d1_gripper_tx_probe::FrozenSnapshotPtr FreezeValid(const d1_gripper_tx_probe::GripperPreviewRequest& request,
                                                    std::chrono::steady_clock::time_point now) {
  const auto preview = d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now);
  const auto frozen = d1_gripper_tx_probe::GripperTxCore::FreezePreview(request, preview, now);
  assert(frozen); return *frozen;
}
d1_gripper_tx_probe::GuardedSessionObservation ValidObservation(std::chrono::steady_clock::time_point now) {
  return {ValidFeedback(now), 0U, Available()};
}

/** mock writer 只保存收到的字节，用于证明测试从不触碰 DDS。 */
struct MockWriter {
  bool succeed{true}; int writes{0}; std::string payload;
  bool Write(const std::string& bytes) { ++writes; payload = bytes; return succeed; }
};
bool AttemptMockWrite(const d1_gripper_tx_probe::FrozenSnapshotPtr& snapshot,
                      const d1_gripper_tx_probe::GuardedSessionObservation& observation,
                      std::chrono::steady_clock::time_point now, const std::string& confirmation,
                      MockWriter* writer, d1_gripper_tx_probe::SingleFrameBudget* budget) {
  if (!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, snapshot, confirmation)) return false;
  if (!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted) return false;
  if (!budget->Reserve() || !d1_gripper_tx_probe::GripperTxCore::PayloadBytesMatch(snapshot, snapshot->candidate_json)) return false;
  return writer->Write(snapshot->candidate_json);  // 无失败重试分支。
}
}  // namespace

int main() {
  const auto now = std::chrono::steady_clock::now();
  auto request = ValidRequest(now);
  const auto preview = d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now);
  const auto snapshot = FreezeValid(request, now);
  assert(preview.accepted && preview.command && preview.json && preview.safety_label == "DRY_RUN_ONLY / NOT SENT");
  assert(snapshot->seq == 60504 && snapshot->funcode == 2 && snapshot->address == 1 && snapshot->mode == 0);
  for (int index = 0; index < 6; ++index) assert(snapshot->angles[index] == request.feedback.app_values[index]);
  assert(snapshot->target_angle6 == request.feedback.app_values[6] + 1.0);
  assert(d1_gripper_tx_probe::GripperTxCore::Sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  assert(d1_gripper_tx_probe::GripperTxCore::PayloadBytesMatch(snapshot, *preview.json));
  assert(!d1_gripper_tx_probe::GripperTxCore::PayloadBytesMatch(snapshot, *preview.json + " "));

  // 冻结后即使调用方持有的反馈对象变化，也不能覆盖 payload 或各角度。
  const std::string frozen_payload = snapshot->candidate_json;
  request.feedback.app_values[0] = 99.0; request.feedback.app_values[6] = 88.0;
  assert(snapshot->candidate_json == frozen_payload && snapshot->feedback_angles[0] == 1.0 &&
         snapshot->feedback_servo_values[0] == 1.0 && snapshot->angles[6] == -18.8);
  const auto confirmation = "SEND_ONE_GRIPPER_TARGET " + snapshot->candidate_json_sha256;
  assert(!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(false, snapshot, confirmation));
  assert(!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, snapshot, "SEND_ONE_GRIPPER_TARGET wrong"));
  assert(d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, snapshot, confirmation));

  auto observation = ValidObservation(now);
  const auto initial_decision = d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now);
  if (!initial_decision.accepted) std::cerr << "initial guarded-session rejection: " << initial_decision.reason << '\n';
  assert(initial_decision.accepted);
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(
      snapshot, observation, now + d1_gripper_tx_probe::GripperTxCore::kSnapshotTtl + std::chrono::milliseconds(1)).accepted);
  observation.feedback.latest_servo = now - std::chrono::seconds(1);
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.feedback.servo_values[0] += 1.0;
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.feedback.enable_status = 0;
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.feedback.app_values[0] += 0.21;
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.feedback.app_values[6] += 0.21;
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.command_frames_since_snapshot = 1U;
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);
  observation = ValidObservation(now); observation.writer_lock = {d1_gripper_tx_probe::WriterLockState::kActive, "active local writer PID 1"};
  assert(!d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(snapshot, observation, now).accepted);

  // mock 成功路径：预览字节就是 Write 字节；同一预算拒绝第二帧，也不存在自动回位或停止帧。
  observation = ValidObservation(now); MockWriter writer{}; d1_gripper_tx_probe::SingleFrameBudget budget;
  assert(AttemptMockWrite(snapshot, observation, now, confirmation, &writer, &budget));
  assert(writer.writes == 1 && writer.payload == snapshot->candidate_json && budget.sent_count() == 1U);
  assert(!AttemptMockWrite(snapshot, observation, now, confirmation, &writer, &budget));
  assert(writer.writes == 1 && writer.payload.find("\"funcode\":2") != std::string::npos &&
         writer.payload.find("\"funcode\":5") == std::string::npos && writer.payload.find("\"funcode\":7") == std::string::npos);
  MockWriter failed_writer{false}; d1_gripper_tx_probe::SingleFrameBudget failed_budget;
  assert(!AttemptMockWrite(snapshot, observation, now, confirmation, &failed_writer, &failed_budget));
  assert(failed_writer.writes == 1 && !failed_budget.Reserve());

  request = ValidRequest(now); request.delta = -1.0; assert(d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request.delta = 1.01; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request.delta = -1.01; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.angle_valid = false; assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.feedback.app_values[6] = std::numeric_limits<double>::infinity(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.seq.reset(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);
  request = ValidRequest(now); request.seq = std::numeric_limits<std::uint64_t>::max(); assert(!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now).accepted);

  const auto lock_path = std::filesystem::temp_directory_path() / "d1_gripper_tx_probe_test.lock";
  std::filesystem::remove(lock_path);
  assert(d1_gripper_tx_probe::InspectWriterLock(lock_path).state == d1_gripper_tx_probe::WriterLockState::kAvailable);
  assert(!std::filesystem::exists(lock_path));  // dry-run inspection 不创建 writer lock。
  { rk_arm::SingleWriterGuard lock(lock_path); std::string error; assert(lock.Acquire(&error)); assert(std::filesystem::exists(lock_path)); }
  assert(!std::filesystem::exists(lock_path));
  { std::ofstream stale(lock_path); stale << "999999\n"; }
  assert(d1_gripper_tx_probe::InspectWriterLock(lock_path).state == d1_gripper_tx_probe::WriterLockState::kStale);
  std::filesystem::remove(lock_path);
  std::cout << "MOCK_GUARDED_SESSION PASS preview_length=" << snapshot->candidate_json.size()
            << " write_length=" << writer.payload.size() << " sha256=" << snapshot->candidate_json_sha256
            << " writes=" << writer.writes << "\n";
}
