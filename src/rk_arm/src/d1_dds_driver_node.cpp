#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <csignal>
#include <iostream>
#include <mutex>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "rk_arm/d1_feedback_parser.hpp"
#include "rk_arm/feedback_bridge_protocol.hpp"

namespace rk_arm {
namespace {
std::uint64_t MonotonicNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

/** 纯 CycloneDDS 进程：仅接收 D1 反馈并转发到本机 socket，刻意不链接 ROS。 */
class D1FeedbackReader {
 public:
  D1FeedbackReader(std::string interface, std::string arm_topic, std::string servo_topic,
                   std::string socket_path)
      : interface_(std::move(interface)), arm_topic_(std::move(arm_topic)),
        servo_topic_(std::move(servo_topic)), socket_path_(std::move(socket_path)) {}
  ~D1FeedbackReader() { if (arm_subscriber_) arm_subscriber_->CloseChannel(); if (servo_subscriber_) servo_subscriber_->CloseChannel(); if (socket_fd_ >= 0) ::close(socket_fd_); }

  bool Start() {
    socket_fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0 || socket_path_.size() >= sizeof(sockaddr_un::sun_path)) return false;
    std::memset(&destination_, 0, sizeof(destination_)); destination_.sun_family = AF_UNIX;
    std::strncpy(destination_.sun_path, socket_path_.c_str(), sizeof(destination_.sun_path) - 1);
    unitree::robot::ChannelFactory::Instance()->Init(0, interface_);
    arm_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(arm_topic_);
    arm_subscriber_->InitChannel([this](const void* raw) { OnArmFeedback(raw); });
    servo_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(servo_topic_);
    servo_subscriber_->InitChannel([this](const void* raw) { OnServoFeedback(raw); });
    std::cout << "D1 read-only DDS reader started; no rt/arm_Command publisher is created." << std::endl;
    return true;
  }
  void Run() const { while (!stop_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
  void Stop() { stop_.store(true); }

 private:
  void OnArmFeedback(const void* raw) {
    const auto* message = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw); D1FeedbackFrame frame;
    if (!ParseD1Feedback(message->data_(), &frame)) return;
    { std::lock_guard<std::mutex> lock(mutex_); if (frame.app_values) { packet_.app_values = *frame.app_values; packet_.angle_valid = 1; packet_.angle_monotonic_ns = MonotonicNs(); } if (frame.enable_status) { packet_.enable_status = *frame.enable_status; packet_.power_status = *frame.power_status; packet_.error_status = *frame.error_status; } }
    SendLatest();
  }
  void OnServoFeedback(const void* raw) {
    const auto* value = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
    { std::lock_guard<std::mutex> lock(mutex_); packet_.servo_values = {value->servo0_data_(), value->servo1_data_(), value->servo2_data_(), value->servo3_data_(), value->servo4_data_(), value->servo5_data_(), value->servo6_data_()}; packet_.servo_valid = 1; packet_.servo_monotonic_ns = MonotonicNs(); }
    SendLatest();
  }
  void SendLatest() { FeedbackBridgePacket copy; { std::lock_guard<std::mutex> lock(mutex_); copy = packet_; } (void)::sendto(socket_fd_, &copy, sizeof(copy), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_)); }

  std::string interface_, arm_topic_, servo_topic_, socket_path_; int socket_fd_{-1}; sockaddr_un destination_{};
  std::mutex mutex_; FeedbackBridgePacket packet_{}; std::atomic<bool> stop_{false};
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> servo_subscriber_;
};
std::atomic<D1FeedbackReader*> g_reader{nullptr};
void SignalHandler(int) { if (auto* reader = g_reader.load()) reader->Stop(); }
}  // namespace
}  // namespace rk_arm

int main(int argc, char** argv) {
  std::string interface = "eth0", arm_topic = "rt/arm_Feedback", servo_topic = "current_servo_angle", socket_path = "/tmp/rk_d1_arm_feedback.sock";
  for (int index = 1; index + 1 < argc; index += 2) { const std::string key(argv[index]); const std::string value(argv[index + 1]); if (key == "--network-interface") interface = value; else if (key == "--feedback-topic") arm_topic = value; else if (key == "--servo-feedback-topic") servo_topic = value; else if (key == "--state-socket") socket_path = value; else { std::cerr << "Unknown argument: " << key << std::endl; return 2; } }
  rk_arm::D1FeedbackReader reader(interface, arm_topic, servo_topic, socket_path); rk_arm::g_reader.store(&reader);
  std::signal(SIGINT, rk_arm::SignalHandler); std::signal(SIGTERM, rk_arm::SignalHandler);
  if (!reader.Start()) { std::cerr << "D1 read-only DDS initialization failed" << std::endl; return 1; }
  reader.Run(); return 0;
}
