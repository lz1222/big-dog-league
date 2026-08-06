#pragma once

#include "rk_arm/arm_safety_core.hpp"
#include "rk_arm/d1_observed_command.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace d1_gripper_tx_probe {

enum class WriterLockState { kAvailable, kActive, kStale, kUnreadable };

struct WriterLockInspection { WriterLockState state{WriterLockState::kUnreadable}; std::string detail; };

/** 仅检查锁，不创建、删除或强占锁；dry run 调用它不会留下文件。 */
WriterLockInspection InspectWriterLock(const std::filesystem::path& lock_path);

struct GripperPreviewRequest {
  rk_arm::FeedbackSnapshot feedback;
  std::optional<std::uint64_t> seq;
  double delta{1.0};
  WriterLockInspection writer_lock;
};

struct GripperPreview {
  bool accepted{false};
  std::string reason;
  std::string safety_label{"DRY_RUN_ONLY / NOT SENT"};
  std::optional<rk_arm::D1ObservedCommand> command;
  std::optional<std::string> json;
};

/** 单帧预算独立于 DDS，确保任何未来硬件路径也无法在单进程内取得第二次发送资格。 */
class SingleFrameBudget {
 public:
  bool Reserve();
  std::uint32_t sent_count() const { return sent_count_; }

 private:
  std::uint32_t sent_count_{0};
};

/**
 * 只允许从当前完整反馈复制前六路并增量修改第七路。
 * 此核心没有 DDS、发送、停止或使能接口，所有成功结果都只是预览。
 */
class GripperTxCore {
 public:
  static constexpr double kMaxAbsDelta = 1.0;
  static constexpr double kFeedbackTimeoutSec = 0.3;
  static constexpr double kSourceTolerance = 0.5;

  static GripperPreview PrepareDryRun(const GripperPreviewRequest& request,
                                      std::chrono::steady_clock::time_point now);
  static bool HardwareSendConfirmed(bool hardware_send, const std::string& confirmation);
};

}  // namespace d1_gripper_tx_probe
