#pragma once

#include "rk_arm/arm_safety_core.hpp"
#include "rk_arm/d1_observed_command.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
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

/**
 * 同一进程内冻结的单帧计划。通过 shared_ptr<const ...> 交给会话代码，
 * 防止确认等待期间被新反馈或命令行数据覆盖；它绝不从磁盘恢复。
 */
struct FrozenGripperSnapshot {
  std::int64_t snapshot_monotonic_ns{0};
  std::int64_t expiry_monotonic_ns{0};
  std::uint64_t seq{0};
  int funcode{0};
  int address{0};
  int mode{0};
  // feedback_angles 是冻结时真实位置；angles 是不可变的 funcode=2 目标字段。
  std::array<double, 7> feedback_angles{};
  std::array<double, 7> feedback_servo_values{};
  std::array<double, 7> angles{};
  double current_angle6{0.0};
  double target_angle6{0.0};
  double delta{0.0};
  int enable_status{-1};
  int power_status{-1};
  int error_status{-1};
  std::chrono::steady_clock::time_point latest_angle{};
  std::chrono::steady_clock::time_point latest_servo{};
  std::string candidate_json;
  std::string candidate_json_sha256;
};

using FrozenSnapshotPtr = std::shared_ptr<const FrozenGripperSnapshot>;

/** 发送前的实时只读观测；command_frames 只计冻结后收到的外部帧。 */
struct GuardedSessionObservation {
  rk_arm::FeedbackSnapshot feedback;
  std::uint64_t command_frames_since_snapshot{0};
  WriterLockInspection writer_lock;
};

struct GuardedSessionDecision {
  bool accepted{false};
  std::string reason;
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
  // 2026-08-07 五秒静止采样的最大通道抖动为 0.2 app_display_unit。
  static constexpr double kSourceTolerance = 0.2;
  // epsilon 只消除二进制浮点边界误判，不扩大 0.2 的物理容差。
  static constexpr double kToleranceEpsilon = 1e-6;
  static constexpr double kStationaryDriftTolerance = 0.2;
  static constexpr std::chrono::seconds kSnapshotTtl{15};

  static bool ExceedsTolerance(double lhs, double rhs, double tolerance,
                               double epsilon = kToleranceEpsilon);
  static GripperPreview PrepareDryRun(const GripperPreviewRequest& request,
                                      std::chrono::steady_clock::time_point now);
  static std::optional<FrozenSnapshotPtr> FreezePreview(const GripperPreviewRequest& request,
                                                         const GripperPreview& preview,
                                                         std::chrono::steady_clock::time_point now);
  static GuardedSessionDecision ValidateGuardedSession(const FrozenSnapshotPtr& snapshot,
                                                       const GuardedSessionObservation& observation,
                                                       std::chrono::steady_clock::time_point now);
  static bool HardwareSendConfirmed(bool hardware_send, const FrozenSnapshotPtr& snapshot,
                                    const std::string& confirmation);
  static bool PayloadBytesMatch(const FrozenSnapshotPtr& snapshot,
                                const std::string& write_payload);
  static std::string Sha256(const std::string& input);
};

}  // namespace d1_gripper_tx_probe
