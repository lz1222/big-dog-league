#include "d1_gripper_tx_probe/gripper_tx_core.hpp"

#include <cmath>
#include <cerrno>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unistd.h>

namespace d1_gripper_tx_probe {
namespace {
constexpr std::uint32_t RotateRight(std::uint32_t value, std::uint32_t bits) {
  return (value >> bits) | (value << (32U - bits));
}

std::int64_t MonotonicNs(std::chrono::steady_clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

std::string DifferenceDetail(const char* prefix, std::size_t channel, double angle, double servo,
                             double difference, double angle_age, double servo_age,
                             std::size_t max_channel, double max_difference) {
  std::ostringstream output;
  output << prefix << " channel=" << channel << " angle=" << std::setprecision(17) << angle
         << " servo=" << servo << " difference=" << difference
         << " tolerance=" << GripperTxCore::kSourceTolerance
         << " epsilon=" << GripperTxCore::kToleranceEpsilon
         << " feedback_age_sec=" << angle_age << ',' << servo_age
         << " max_difference_channel=" << max_channel << " max_difference=" << max_difference;
  return output.str();
}

std::string ValidateFeedback(const rk_arm::FeedbackSnapshot& feedback,
                             std::chrono::steady_clock::time_point now) {
  if (!feedback.dds_ready || !feedback.angle_valid || !feedback.servo_valid) {
    std::ostringstream output;
    output << "FEEDBACK_SOURCE_MISSING dds_ready=" << feedback.dds_ready
           << " angle_valid=" << feedback.angle_valid << " servo_valid=" << feedback.servo_valid;
    return output.str();
  }
  const double angle_age = std::chrono::duration<double>(now - feedback.latest_angle).count();
  const double servo_age = std::chrono::duration<double>(now - feedback.latest_servo).count();
  if (angle_age < 0.0 || servo_age < 0.0 || angle_age > GripperTxCore::kFeedbackTimeoutSec ||
      servo_age > GripperTxCore::kFeedbackTimeoutSec) {
    std::ostringstream output;
    output << "FEEDBACK_STALE feedback_age_sec=" << angle_age << ',' << servo_age
           << " timeout_sec=" << GripperTxCore::kFeedbackTimeoutSec;
    return output.str();
  }
  std::size_t max_channel = 0;
  double max_difference = 0.0;
  for (std::size_t index = 0; index < feedback.app_values.size(); ++index) {
    if (!std::isfinite(feedback.app_values[index]) || !std::isfinite(feedback.servo_values[index])) {
      std::ostringstream output;
      output << "INVALID_NONFINITE_VALUE channel=" << index << " angle=" << feedback.app_values[index]
             << " servo=" << feedback.servo_values[index];
      return output.str();
    }
    const double difference = std::abs(feedback.app_values[index] - feedback.servo_values[index]);
    if (difference > max_difference) { max_difference = difference; max_channel = index; }
    if (GripperTxCore::ExceedsTolerance(feedback.app_values[index], feedback.servo_values[index],
                                        GripperTxCore::kSourceTolerance)) {
      return DifferenceDetail("SOURCE_INCONSISTENT", index, feedback.app_values[index], feedback.servo_values[index],
                              difference, angle_age, servo_age, max_channel, max_difference);
    }
  }
  if (feedback.enable_status != 1 || feedback.power_status != 0 || feedback.error_status != 0) {
    std::ostringstream output;
    output << "STATUS_NOT_READY enable_status=" << feedback.enable_status
           << " power_status=" << feedback.power_status << " error_status=" << feedback.error_status;
    return output.str();
  }
  return {};
}
}  // namespace

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
    result.reason = request.writer_lock.state == WriterLockState::kActive
        ? "WRITER_LOCK_ACTIVE " + request.writer_lock.detail
        : "INTERNAL_PRECONDITION_ERROR " + request.writer_lock.detail;
    return result;
  }
  const auto& feedback = request.feedback;
  if (const std::string reason = ValidateFeedback(feedback, now); !reason.empty()) { result.reason = reason; return result; }
  const auto shadow = rk_arm::D1ShadowCommandGenerator::PreviewGripper(
      feedback, feedback.app_values[6] + request.delta, *request.seq, now,
      kFeedbackTimeoutSec, kSourceTolerance, false, kToleranceEpsilon);
  if (!shadow.accepted || !shadow.command || !shadow.json) { result.reason = shadow.reason; return result; }
  result.accepted = true;
  result.reason = "DRY_RUN_ONLY / NOT SENT";
  result.command = shadow.command;
  result.json = shadow.json;
  return result;
}

bool GripperTxCore::ExceedsTolerance(double lhs, double rhs, double tolerance, double epsilon) {
  return std::abs(lhs - rhs) > tolerance + epsilon;
}

std::optional<FrozenSnapshotPtr> GripperTxCore::FreezePreview(
    const GripperPreviewRequest& request, const GripperPreview& preview,
    std::chrono::steady_clock::time_point now) {
  if (!preview.accepted || !preview.command || !preview.json || !request.seq) return std::nullopt;
  auto mutable_snapshot = std::make_shared<FrozenGripperSnapshot>();
  mutable_snapshot->snapshot_monotonic_ns = MonotonicNs(now);
  mutable_snapshot->expiry_monotonic_ns = MonotonicNs(now + kSnapshotTtl);
  mutable_snapshot->seq = *request.seq;
  mutable_snapshot->funcode = preview.command->funcode;
  mutable_snapshot->address = preview.command->address;
  mutable_snapshot->mode = preview.command->mode;
  mutable_snapshot->feedback_angles = request.feedback.app_values;
  mutable_snapshot->feedback_servo_values = request.feedback.servo_values;
  mutable_snapshot->angles = preview.command->angles;
  mutable_snapshot->current_angle6 = request.feedback.app_values[6];
  mutable_snapshot->target_angle6 = preview.command->angles[6];
  mutable_snapshot->delta = request.delta;
  mutable_snapshot->enable_status = request.feedback.enable_status;
  mutable_snapshot->power_status = request.feedback.power_status;
  mutable_snapshot->error_status = request.feedback.error_status;
  mutable_snapshot->latest_angle = request.feedback.latest_angle;
  mutable_snapshot->latest_servo = request.feedback.latest_servo;
  mutable_snapshot->candidate_json = *preview.json;
  mutable_snapshot->candidate_json_sha256 = Sha256(mutable_snapshot->candidate_json);
  return FrozenSnapshotPtr(mutable_snapshot);
}

GuardedSessionDecision GripperTxCore::ValidateGuardedSession(
    const FrozenSnapshotPtr& snapshot, const GuardedSessionObservation& observation,
    std::chrono::steady_clock::time_point now) {
  if (!snapshot) return {false, "SNAPSHOT_MISSING"};
  if (now > std::chrono::steady_clock::time_point(std::chrono::nanoseconds(snapshot->expiry_monotonic_ns))) {
    return {false, "SNAPSHOT_EXPIRED"};
  }
  if (observation.command_frames_since_snapshot != 0U) return {false, "COMMAND_TOPIC_NOT_SILENT"};
  if (observation.writer_lock.state != WriterLockState::kAvailable) {
    return {false, observation.writer_lock.state == WriterLockState::kActive
        ? "WRITER_LOCK_ACTIVE " + observation.writer_lock.detail
        : "INTERNAL_PRECONDITION_ERROR " + observation.writer_lock.detail};
  }
  if (const std::string reason = ValidateFeedback(observation.feedback, now); !reason.empty()) return {false, reason};
  for (std::size_t index = 0; index < snapshot->feedback_angles.size(); ++index) {
    if (ExceedsTolerance(observation.feedback.app_values[index], snapshot->feedback_angles[index],
                         kStationaryDriftTolerance)) {
      const double difference = std::abs(observation.feedback.app_values[index] - snapshot->feedback_angles[index]);
      std::ostringstream output;
      output << "FEEDBACK_DRIFTED_SINCE_PREVIEW channel=" << index << " current_angle="
             << std::setprecision(17) << observation.feedback.app_values[index] << " frozen_angle="
             << snapshot->feedback_angles[index] << " difference=" << difference
             << " tolerance=" << kStationaryDriftTolerance << " epsilon=" << kToleranceEpsilon;
      return {false, output.str()};
    }
  }
  return {true, "READY_TO_SEND_FROZEN_PAYLOAD"};
}

bool GripperTxCore::HardwareSendConfirmed(bool hardware_send, const FrozenSnapshotPtr& snapshot,
                                          const std::string& confirmation) {
  return hardware_send && snapshot &&
      confirmation == "SEND_ONE_GRIPPER_TARGET " + snapshot->candidate_json_sha256;
}

bool GripperTxCore::PayloadBytesMatch(const FrozenSnapshotPtr& snapshot,
                                      const std::string& write_payload) {
  return snapshot && snapshot->candidate_json.size() == write_payload.size() &&
      snapshot->candidate_json == write_payload &&
      snapshot->candidate_json_sha256 == Sha256(write_payload);
}

std::string GripperTxCore::Sha256(const std::string& input) {
  // 内嵌标准 SHA-256，避免给实验工具增加 OpenSSL/ROS 运行时依赖。
  static constexpr std::array<std::uint32_t, 64> k{ {
      0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
      0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
      0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
      0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
      0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
      0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
      0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
      0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U }};
  std::array<std::uint32_t, 8> hash{{0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U}};
  std::string bytes = input; bytes.push_back(static_cast<char>(0x80));
  while ((bytes.size() % 64U) != 56U) bytes.push_back('\0');
  const std::uint64_t bit_length = static_cast<std::uint64_t>(input.size()) * 8U;
  for (int shift = 56; shift >= 0; shift -= 8) bytes.push_back(static_cast<char>((bit_length >> shift) & 0xffU));
  for (std::size_t offset = 0; offset < bytes.size(); offset += 64U) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index) {
      const auto byte = [&bytes, offset, index](std::size_t part) { return static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset + index * 4U + part])); };
      words[index] = (byte(0) << 24U) | (byte(1) << 16U) | (byte(2) << 8U) | byte(3);
    }
    for (std::size_t index = 16; index < words.size(); ++index) {
      const std::uint32_t s0 = RotateRight(words[index - 15], 7) ^ RotateRight(words[index - 15], 18) ^ (words[index - 15] >> 3U);
      const std::uint32_t s1 = RotateRight(words[index - 2], 17) ^ RotateRight(words[index - 2], 19) ^ (words[index - 2] >> 10U);
      words[index] = words[index - 16] + s0 + words[index - 7] + s1;
    }
    std::uint32_t a=hash[0], b=hash[1], c=hash[2], d=hash[3], e=hash[4], f=hash[5], g=hash[6], h=hash[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t s1 = RotateRight(e, 6) ^ RotateRight(e, 11) ^ RotateRight(e, 25);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary1 = h + s1 + choose + k[index] + words[index];
      const std::uint32_t s0 = RotateRight(a, 2) ^ RotateRight(a, 13) ^ RotateRight(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = s0 + majority;
      h=g; g=f; f=e; e=d+temporary1; d=c; c=b; b=a; a=temporary1+temporary2;
    }
    hash[0]+=a; hash[1]+=b; hash[2]+=c; hash[3]+=d; hash[4]+=e; hash[5]+=f; hash[6]+=g; hash[7]+=h;
  }
  std::ostringstream output;
  for (const auto value : hash) output << std::hex << std::setw(8) << std::setfill('0') << value;
  return output.str();
}

bool SingleFrameBudget::Reserve() {
  if (sent_count_ != 0U) return false;
  ++sent_count_;
  return true;
}

}  // namespace d1_gripper_tx_probe
