#include "d1_gripper_tx_probe/gripper_tx_core.hpp"

#include "rk_arm/d1_feedback_parser.hpp"
#include "rk_arm/single_writer_guard.hpp"

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
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace {
constexpr const char* kFeedbackTopic = "rt/arm_Feedback";
constexpr const char* kServoTopic = "current_servo_angle";
constexpr const char* kCommandTopic = "rt/arm_Command";
constexpr const char* kLockPath = "/tmp/rk_d1_arm_writer.lock";
constexpr auto kFeedbackWait = std::chrono::milliseconds(1500);
std::atomic<bool> g_exit_requested{false};
void HandleSignal(int) { g_exit_requested.store(true); }

struct Arguments {
  std::string interface_name;
  std::optional<std::uint64_t> seq;
  double delta{1.0};
  bool hardware_send{false};
  std::string confirmation;
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
    else if (option == "--hardware-send") arguments->hardware_send = true;
    else if (option == "--confirm" && index + 1 < argc) arguments->confirmation = argv[++index];
    else return false;
  }
  return true;
}
void PrintUsage(const char* executable) {
  std::cerr << "EXPERIMENTAL HARDWARE WRITER\nUsage: " << executable
            << " --seq N [--delta VALUE] [--interface IFACE] [--hardware-send --confirm SEND_ONE_GRIPPER_TARGET]\n"
            << "Default behavior is DRY_RUN_ONLY / NOT SENT. This tool has no joint, trajectory, stop, or enable options.\n";
}
std::array<double, 7> ServoValues(const unitree_arm::msg::dds_::PubServoInfo_& message) {
  return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(), message.servo3_data_(),
          message.servo4_data_(), message.servo5_data_(), message.servo6_data_()};
}
void PrintPreview(const d1_gripper_tx_probe::GripperPreviewRequest& request,
                  const d1_gripper_tx_probe::GripperPreview& preview,
                  std::chrono::steady_clock::time_point now) {
  std::cout << "EXPERIMENTAL HARDWARE WRITER\n";
  std::cout << "writer_lock=" << request.writer_lock.detail << "\n";
  std::cout << "seq=" << (request.seq ? std::to_string(*request.seq) : "MISSING") << " delta=" << request.delta
            << " value_unit=app_display_unit\n";
  for (int index = 0; index < 7; ++index) std::cout << "feedback_angle" << index << "=" << request.feedback.app_values[index] << " servo" << index << "=" << request.feedback.servo_values[index] << "\n";
  const auto angle_age = std::chrono::duration<double>(now - request.feedback.latest_angle).count();
  const auto servo_age = std::chrono::duration<double>(now - request.feedback.latest_servo).count();
  std::cout << "dual_source_valid=" << (request.feedback.angle_valid && request.feedback.servo_valid ? "true" : "false")
            << " feedback_age_sec=" << angle_age << "," << servo_age << " status=" << request.feedback.enable_status
            << "," << request.feedback.power_status << "," << request.feedback.error_status << "\n";
  if (preview.command) std::cout << "funcode=" << preview.command->funcode << " address=" << preview.command->address
                                 << " mode=" << preview.command->mode << " current_angle6=" << request.feedback.app_values[6]
                                 << " target_angle6=" << preview.command->angles[6] << "\n";
  if (preview.json) std::cout << "candidate_json=" << *preview.json << "\n";
  std::cout << "result=" << preview.reason << "\n";
}
}

int main(int argc, char** argv) {
  Arguments arguments;
  if (!ParseArguments(argc, argv, &arguments) || !arguments.seq) { PrintUsage(argv[0]); return 2; }
    rk_arm::FeedbackSnapshot feedback{};
    std::mutex feedback_mutex;
  std::signal(SIGINT, HandleSignal); std::signal(SIGTERM, HandleSignal);
  try {
    // dry run 与发送前检查均只通过这些 reader 收集当前完整反馈；此处不创建 writer。
    unitree::robot::ChannelFactory::Instance()->Init(0, arguments.interface_name);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> arm_feedback(kFeedbackTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servo_feedback(kServoTopic);
    arm_feedback.InitChannel([&feedback, &feedback_mutex](const void* raw) {
      rk_arm::D1FeedbackFrame frame{};
      if (!rk_arm::ParseD1Feedback(static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_(), &frame)) return;
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(feedback_mutex);
      if (frame.app_values) { feedback.app_values = *frame.app_values; feedback.angle_valid = true; feedback.latest_angle = now; }
      if (frame.enable_status) { feedback.enable_status = *frame.enable_status; feedback.power_status = *frame.power_status; feedback.error_status = *frame.error_status; }
      feedback.dds_ready = true;
    });
    servo_feedback.InitChannel([&feedback, &feedback_mutex](const void* raw) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      feedback.servo_values = ServoValues(*static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw));
      feedback.servo_valid = true; feedback.latest_servo = std::chrono::steady_clock::now(); feedback.dds_ready = true;
    });
    const auto deadline = std::chrono::steady_clock::now() + kFeedbackWait;
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < deadline) {
      { std::lock_guard<std::mutex> lock(feedback_mutex);
        if (feedback.angle_valid && feedback.servo_valid && feedback.enable_status >= 0 &&
            feedback.power_status >= 0 && feedback.error_status >= 0) break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto now = std::chrono::steady_clock::now();
    rk_arm::FeedbackSnapshot snapshot{};
    { std::lock_guard<std::mutex> lock(feedback_mutex); snapshot = feedback; }
    d1_gripper_tx_probe::GripperPreviewRequest request{snapshot, arguments.seq, arguments.delta,
      d1_gripper_tx_probe::InspectWriterLock(kLockPath)};
    const auto preview = d1_gripper_tx_probe::GripperTxCore::PrepareDryRun(request, now);
    PrintPreview(request, preview, now);
    if (!preview.accepted) { arm_feedback.CloseChannel(); servo_feedback.CloseChannel(); unitree::robot::ChannelFactory::Instance()->Release(); return 3; }
    if (!d1_gripper_tx_probe::GripperTxCore::HardwareSendConfirmed(arguments.hardware_send, arguments.confirmation)) {
      std::cout << "DRY_RUN_ONLY / NOT SENT; no DDS writer was created.\n";
      arm_feedback.CloseChannel(); servo_feedback.CloseChannel(); unitree::robot::ChannelFactory::Instance()->Release(); return 0;
    }

    // 未来已获批准的路径：单独取得锁后才创建 writer，计数器强制整个进程仅一帧。
    rk_arm::SingleWriterGuard writer_lock(kLockPath); std::string lock_error;
    if (!writer_lock.Acquire(&lock_error)) { std::cerr << "writer lock rejected: " << lock_error << '\n'; return 4; }
    d1_gripper_tx_probe::SingleFrameBudget send_budget;
    if (!send_budget.Reserve()) { std::cerr << "single-frame limit rejected\n"; return 5; }
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_> writer(kCommandTopic);
    writer.InitChannel(); unitree_arm::msg::dds_::ArmString_ message{}; message.data_() = *preview.json;
    writer.Write(message); writer.CloseChannel();
    arm_feedback.CloseChannel(); servo_feedback.CloseChannel(); unitree::robot::ChannelFactory::Instance()->Release();
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "probe failure: " << exception.what() << '\n'; return 6;
  }
}
