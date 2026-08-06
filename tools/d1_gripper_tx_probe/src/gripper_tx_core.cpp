#include "d1_gripper_tx_probe/gripper_tx_core.hpp"

#include <cmath>
#include <cerrno>
#include <csignal>
#include <fstream>
#include <limits>
#include <unistd.h>

namespace d1_gripper_tx_probe {

WriterLockInspection InspectWriterLock(const std::filesystem::path& lock_path) {
  std::error_code exists_error;
  if (!std::filesystem::exists(lock_path, exists_error)) {
    return {exists_error ? WriterLockState::kUnreadable : WriterLockState::kAvailable,
            exists_error ? "cannot inspect writer lock" : "no local writer lock"};
  }
  std::ifstream input(lock_path);
  long pid = -1;
  if (!(input >> pid) || pid <= 0) return {WriterLockState::kStale, "writer lock has no live PID"};
  if (::kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM) {
    return {WriterLockState::kActive, "active local writer PID " + std::to_string(pid)};
  }
  return {WriterLockState::kStale, "writer lock PID is confirmed dead"};
}

GripperPreview GripperTxCore::PrepareDryRun(const GripperPreviewRequest& request,
                                            std::chrono::steady_clock::time_point now) {
  GripperPreview result;
  // 硬限制不来自命令行或配置，避免实验工具被扩大为大行程控制器。
  if (!std::isfinite(request.delta) || std::abs(request.delta) > kMaxAbsDelta) {
    result.reason = "delta must be within [-1.0, 1.0] app_display_unit"; return result;
  }
  if (!request.seq) { result.reason = "explicit seq is required"; return result; }
  if (*request.seq == std::numeric_limits<std::uint64_t>::max()) {
    result.reason = "SEQ_OVERFLOW_RISK"; return result;
  }
  if (request.writer_lock.state != WriterLockState::kAvailable) {
    result.reason = request.writer_lock.detail; return result;
  }
  const auto& feedback = request.feedback;
  if (feedback.enable_status != 1 || feedback.power_status != 0 || feedback.error_status != 0) {
    result.reason = "funcode=3 status is not observed-normal"; return result;
  }
  const auto shadow = rk_arm::D1ShadowCommandGenerator::PreviewGripper(
      feedback, feedback.app_values[6] + request.delta, *request.seq, now,
      kFeedbackTimeoutSec, kSourceTolerance, false);
  if (!shadow.accepted || !shadow.command || !shadow.json) { result.reason = shadow.reason; return result; }
  result.accepted = true;
  result.reason = "DRY_RUN_ONLY / NOT SENT";
  result.command = shadow.command;
  result.json = shadow.json;
  return result;
}

bool GripperTxCore::HardwareSendConfirmed(bool hardware_send, const std::string& confirmation) {
  return hardware_send && confirmation == "SEND_ONE_GRIPPER_TARGET";
}

bool SingleFrameBudget::Reserve() {
  if (sent_count_ != 0U) return false;
  ++sent_count_;
  return true;
}

}  // namespace d1_gripper_tx_probe
