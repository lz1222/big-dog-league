#include "d1_gripper_tx_probe/gripper_tx_core.hpp"

#include "rk_arm/d1_feedback_parser.hpp"
#include "rk_arm/single_writer_guard.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <optional>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace {
constexpr const char* kFeedbackTopic = "rt/arm_Feedback";
constexpr const char* kServoTopic = "current_servo_angle";
constexpr const char* kCommandTopic = "rt/arm_Command";
constexpr const char* kLockPath = "/tmp/rk_d1_arm_writer.lock";
constexpr auto kInitialFeedbackWait = std::chrono::milliseconds(1500);
constexpr auto kCommandQuietWindow = std::chrono::seconds(5);
// 真机单帧后仍保持只读十秒，便于操作者判断停止和任何意外联动。
constexpr auto kPostWriteObserveWindow = std::chrono::seconds(10);
std::atomic<bool> g_exit_requested{false};
void HandleSignal(int) { g_exit_requested.store(true); }

struct Arguments {
  std::string interface_name;
  std::optional<std::uint64_t> seq;
  double delta{1.0};
  bool guarded_session{false};
  bool hardware_send{false};
};

bool ParseUnsigned(const std::string& text, std::uint64_t* value) {
  try { std::size_t used = 0; const auto parsed = std::stoull(text, &used); if (used != text.size()) return false; *value = parsed; return true; } catch (...) { return false; }
}
bool ParseDouble(const std::string& text, double* value) {
  try { std::size_t used = 0; const auto parsed = std::stod(text, &used); if (used != text.size() || !std::isfinite(parsed)) return false; *value = parsed; return true; } catch (...) { return false; }
}
bool ParseArguments(int argc, char** argv, Arguments* arguments) {
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--interface" && index + 1 < argc) arguments->interface_name = argv[++index];
    else if (option == "--seq" && index + 1 < argc) { std::uint64_t value = 0; if (!ParseUnsigned(argv[++index], &value)) return false; arguments->seq = value; }
    else if (option == "--delta" && index + 1 < argc) { if (!ParseDouble(argv[++index], &arguments->delta)) return false; }
    else if (option == "--guarded-session") arguments->guarded_session = true;
    else if (option == "--hardware-send") arguments->hardware_send = true;
    else return false;
  }
  return true;
}
void PrintUsage(const char* executable) {
  std::cerr << "EXPERIMENTAL HARDWARE WRITER\nUsage: " << executable
            << " --seq N [--delta VALUE] [--interface IFACE] [--guarded-session --hardware-send]\n"
            << "Default behavior is DRY_RUN_ONLY / NOT SENT. The guarded mode reads a SHA-bound confirmation from stdin.\n";
}
std::array<double, 7> ServoValues(const unitree_arm::msg::dds_::PubServoInfo_& message) {
  return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(), message.servo3_data_(),
          message.servo4_data_(), message.servo5_data_(), message.servo6_data_()};
}
rk_arm::FeedbackSnapshot CopyFeedback(const rk_arm::FeedbackSnapshot& feedback, std::mutex* mutex) {
  std::lock_guard<std::mutex> lock(*mutex); return feedback;
}
void PrintFrozenPreview(const d1_gripper_tx_probe::GripperPreviewRequest& request,
                        const d1_gripper_tx_probe::FrozenSnapshotPtr& snapshot,
                        std::chrono::steady_clock::time_point now,
                        std::uint64_t command_frames) {
  std::cout << "EXPERIMENTAL HARDWARE WRITER\nwriter_lock=" << request.writer_lock.detail << "\n";
  for (int index = 0; index < 7; ++index) {
    std::cout << "feedback_angle" << index << '=' << request.feedback.app_values[index]
              << " servo" << index << '=' << request.feedback.servo_values[index]
              << " difference=" << std::abs(request.feedback.app_values[index] - request.feedback.servo_values[index]) << '\n';
  }
  const double angle_age = std::chrono::duration<double>(now - request.feedback.latest_angle).count();
  const double servo_age = std::chrono::duration<double>(now - request.feedback.latest_servo).count();
  const auto remaining = std::chrono::duration<double>(
      std::chrono::steady_clock::time_point(std::chrono::nanoseconds(snapshot->expiry_monotonic_ns)) - now).count();
  std::cout << "feedback_age_sec=" << angle_age << ',' << servo_age << " status=" << snapshot->enable_status << ','
            << snapshot->power_status << ',' << snapshot->error_status << " command_topic_frames=" << command_frames << '\n';
  std::cout << "snapshot_monotonic_ns=" << snapshot->snapshot_monotonic_ns << " expiry_monotonic_ns="
            << snapshot->expiry_monotonic_ns << " snapshot_ttl_remaining_sec=" << remaining << '\n';
  std::cout << "seq=" << snapshot->seq << " funcode=" << snapshot->funcode << " address=" << snapshot->address
            << " mode=" << snapshot->mode << " delta=" << snapshot->delta << " value_unit=app_display_unit\n";
  std::cout << "current_angle6=" << snapshot->current_angle6 << " target_angle6=" << snapshot->target_angle6 << '\n';
  std::cout << "frozen_json=" << snapshot->candidate_json << "\nfrozen_json_sha256="
            << snapshot->candidate_json_sha256 << "\nresult=DRY_RUN_ONLY / NOT SENT\n";
}

/** 等待阶段只检查最新 reader 状态与静默条件；绝不把新数据写回冻结快照。 */
d1_gripper_tx_probe::GuardedSessionDecision ObserveFrozenSession(
    const d1_gripper_tx_probe::FrozenSnapshotPtr& snapshot, const rk_arm::FeedbackSnapshot& feedback,
    std::mutex* feedback_mutex, const std::atomic<std::uint64_t>& command_frames) {
  d1_gripper_tx_probe::GuardedSessionObservation observation{
      CopyFeedback(feedback, feedback_mutex), command_frames.load(),
      d1_gripper_tx_probe::InspectWriterLock(kLockPath)};
  return d1_gripper_tx_probe::GripperTxCore::ValidateGuardedSession(
      snapshot, observation, std::chrono::steady_clock::now());
}
}  // namespace

int main(int argc, char** argv) {
  Arguments arguments;
  if (!ParseArguments(argc, argv, &arguments) || !arguments.seq ||
      (arguments.hardware_send && !arguments.guarded_session)) { PrintUsage(argv[0]); return 2; }
  rk_arm::FeedbackSnapshot feedback{};
  std::mutex feedback_mutex;
  std::atomic<std::uint64_t> command_frames{0};
  std::signal(SIGINT, HandleSignal); std::signal(SIGTERM, HandleSignal);
  try {
    // 预览和等待期只建立 reader；publisher 只存在于全部门禁通过后的最小作用域内。
    unitree::robot::ChannelFactory::Instance()->Init(0, arguments.interface_name);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> arm_feedback(kFeedbackTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servo_feedback(kServoTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> command_monitor(kCommandTopic);
    arm_feedback.InitChannel([&feedback, &feedback_mutex](const void* raw) {
      rk_arm::D1FeedbackFrame frame{};
      if (!rk_arm::ParseD1Feedback(static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_(), &frame)) return;
      const auto now = std::chrono::steady_clock::now(); std::lock_guard<std::mutex> lock(feedback_mutex);
      if (frame.app_values) { feedback.app_values = *frame.app_values; feedback.angle_valid = true; feedback.latest_angle = now; }
      if (frame.enable_status) { feedback.enable_status = *frame.enable_status; feedback.power_status = *frame.power_status; feedback.error_status = *frame.error_status; }
      feedback.dds_ready = true;
    });
    servo_feedback.InitChannel([&feedback, &feedback_mutex](const void* raw) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      feedback.servo_values = ServoValues(*static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw));
      feedback.servo_valid = true; feedback.latest_servo = std::chrono::steady_clock::now(); feedback.dds_ready = true;
    });
    command_monitor.InitChannel([&command_frames](const void*) { command_frames.fetch_add(1U); });
    const auto cleanup = [&]() { arm_feedback.CloseChannel(); servo_feedback.CloseChannel(); command_monitor.CloseChannel(); unitree::robot::ChannelFactory::Instance()->Release(); };

    const auto initial_deadline = std::chrono::steady_clock::now() + kInitialFeedbackWait;
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < initial_deadline) {
      const auto current = CopyFeedback(feedback, &feedback_mutex);
      if (current.angle_valid && current.servo_valid && current.enable_status >= 0 && current.power_status >= 0 && current.error_status >= 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto quiet_deadline = std::chrono::steady_clock::now() + kCommandQuietWindow;
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < quiet_deadline) {
      const auto current = CopyFeedback(feedback, &feedback_mutex);
      d1_gripper_tx_probe::GripperPreviewRequest guard_request{current, arguments.seq, arguments.delta,
          d1_gripper_tx_probe::InspectWriterLock(kLockPath)};
      if (!d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(guard_request, std::chrono::steady_clock::now()).accepted) {
        std::cerr << "PREVIEW_PRECONDITION_FAILED_DURING_QUIET_WINDOW\n"; cleanup(); return 3;
      }
      if (command_frames.load() != 0U) { std::cerr << "COMMAND_TOPIC_NOT_SILENT\n"; cleanup(); return 3; }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (g_exit_requested.load()) { std::cerr << "SESSION_CANCELLED\n"; cleanup(); return 3; }
    const auto now = std::chrono::steady_clock::now();
    const auto current = CopyFeedback(feedback, &feedback_mutex);
    d1_gripper_tx_probe::GripperPreviewRequest request{current, arguments.seq, arguments.delta,
        d1_gripper_tx_probe::InspectWriterLock(kLockPath)};
    const auto preview = d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now);
    const auto frozen = d1_gripper_tx_probe::GripperTxCore::FreezePreview(request, preview, now);
    if (!frozen) { std::cerr << "SNAPSHOT_FREEZE_FAILED " << preview.reason << '\n'; cleanup(); return 3; }
    PrintFrozenPreview(request, *frozen, now, command_frames.load());
    if (!arguments.guarded_session) {
      std::cout << "DRY_RUN_ONLY / NOT SENT; no DDS writer was created.\n"; cleanup(); return 0;
    }
    if (!arguments.hardware_send) { std::cerr << "GUARDED_SESSION_REQUIRES_HARDWARE_SEND\n"; cleanup(); return 2; }

    std::cout << "Await exact stdin confirmation: SEND_ONE_GRIPPER_TARGET " << (*frozen)->candidate_json_sha256 << '\n';
    std::string confirmation;
    while (!g_exit_requested.load()) {
      const auto safety = ObserveFrozenSession(*frozen, feedback, &feedback_mutex, command_frames);
      if (!safety.accepted) { std::cerr << safety.reason << '\n'; cleanup(); return 4; }
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int poll_result = ::poll(&input, 1, 50);
      if (poll_result < 0) { std::cerr << "CONFIRMATION_INPUT_FAILED\n"; cleanup(); return 4; }
      if ((input.revents & POLLIN) != 0) { std::getline(std::cin, confirmation); break; }
      if ((input.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) { std::cerr << "CONFIRMATION_INPUT_CLOSED\n"; cleanup(); return 4; }
    }
    if (g_exit_requested.load() || !d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(true, *frozen, confirmation)) {
      std::cerr << "SHA_BOUND_CONFIRMATION_REJECTED\n"; cleanup(); return 4;
    }
    const auto final_safety = ObserveFrozenSession(*frozen, feedback, &feedback_mutex, command_frames);
    if (!final_safety.accepted) { std::cerr << final_safety.reason << '\n'; cleanup(); return 4; }

    // 只有冻结 payload、SHA 确认、实时复核和静默检查都成立后，才短暂取得唯一 writer。
    rk_arm::SingleWriterGuard writer_lock(kLockPath); std::string lock_error;
    if (!writer_lock.Acquire(&lock_error)) { std::cerr << "writer lock rejected: " << lock_error << '\n'; cleanup(); return 5; }
    d1_gripper_tx_probe::SingleFrameBudget send_budget;
    if (!send_budget.Reserve()) { std::cerr << "single-frame limit rejected\n"; cleanup(); return 5; }
    unitree_arm::msg::dds_::ArmString_ message{};
    message.data_() = (*frozen)->candidate_json;  // 只复制冻结字符串，不允许二次 JSON 编码。
    const std::string write_payload = message.data_();
    if (!d1_gripper_tx_probe::GripperTxCore::PayloadBytesMatch(*frozen, write_payload)) {
      std::cerr << "PAYLOAD_CHANGED_AFTER_PREVIEW\n"; cleanup(); return 5;
    }
    std::cout << "preview_payload_sha256=" << (*frozen)->candidate_json_sha256 << " write_payload_sha256="
              << d1_gripper_tx_probe::GripperTxCore::Sha256(write_payload) << " preview_payload_length="
              << (*frozen)->candidate_json.size() << " write_payload_length=" << write_payload.size() << '\n';
    const std::uint64_t command_frames_before_write = command_frames.load();
    bool write_return = false;
    { unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_> writer(kCommandTopic);
      writer.InitChannel(); write_return = writer.Write(message); writer.CloseChannel(); }
    // 发送结果无论成功或失败都不重试；后续窗口始终只读，保留故障现场证据。
    std::cout << "write_return=" << (write_return ? "true" : "false") << " actual_write_payload=" << write_payload
              << "\nWRITE_ONCE_COMPLETED; observing feedback for 10 seconds\n";
    std::array<double, 7> max_angle_change{};
    std::array<double, 7> max_servo_change{};
    rk_arm::FeedbackSnapshot final_feedback = CopyFeedback(feedback, &feedback_mutex);
    const auto observe_deadline = std::chrono::steady_clock::now() + kPostWriteObserveWindow;
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < observe_deadline) {
      final_feedback = CopyFeedback(feedback, &feedback_mutex);
      for (std::size_t index = 0; index < max_angle_change.size(); ++index) {
        max_angle_change[index] = std::max(max_angle_change[index], std::abs(final_feedback.app_values[index] - (*frozen)->feedback_angles[index]));
        max_servo_change[index] = std::max(max_servo_change[index], std::abs(final_feedback.servo_values[index] - (*frozen)->feedback_servo_values[index]));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    // 这份数值报告不推断实体运动；实体观察和异常声音仍必须由现场操作者确认。
    std::cout << "command_frames_observed_before_write=" << command_frames_before_write
              << " command_frames_observed_after_write=" << (command_frames.load() - command_frames_before_write) << '\n';
    for (std::size_t index = 0; index < max_angle_change.size(); ++index) {
      std::cout << "post_angle" << index << '=' << final_feedback.app_values[index]
                << " post_servo" << index << '=' << final_feedback.servo_values[index]
                << " angle_max_abs_change=" << max_angle_change[index]
                << " servo_max_abs_change=" << max_servo_change[index] << '\n';
    }
    std::cout << "post_status=" << final_feedback.enable_status << ',' << final_feedback.power_status << ','
              << final_feedback.error_status << '\n';
    cleanup(); return write_return ? 0 : 7;
  } catch (const std::exception& exception) {
    std::cerr << "probe failure: " << exception.what() << '\n'; return 6;
  }
}
