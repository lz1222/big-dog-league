#include "rk_arm_feedback_probe/feedback_recorder.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace {
constexpr const char* kCommandTopic = "rt/arm_Command";
constexpr const char* kFeedbackTopic = "rt/arm_Feedback";
constexpr const char* kServoTopic = "current_servo_angle";
std::atomic<bool> g_exit_requested{false};
void HandleSignal(int) { g_exit_requested.store(true); }

struct Arguments { std::string interface_name; std::string output_dir{"artifacts/d1_command_protocol"}; std::string event_socket{"/tmp/rk_d1_command_probe_events.sock"}; int duration_sec{300}; int stale_after_ms{1000}; std::size_t parser_max_bytes{1024U * 1024U}; };
bool ParsePositiveInt(const char* text, int* value) { try { const int parsed = std::stoi(text); if (parsed <= 0) return false; *value = parsed; return true; } catch (...) { return false; } }
bool ParseArguments(int argc, char** argv, Arguments* arguments) {
  for (int index = 1; index < argc; ++index) { const std::string option(argv[index]);
    if (option == "--interface" && index + 1 < argc) arguments->interface_name = argv[++index];
    else if (option == "--output-dir" && index + 1 < argc) arguments->output_dir = argv[++index];
    else if (option == "--event-socket" && index + 1 < argc) arguments->event_socket = argv[++index];
    else if (option == "--duration-sec" && index + 1 < argc) { if (!ParsePositiveInt(argv[++index], &arguments->duration_sec)) return false; }
    else if (option == "--stale-after-ms" && index + 1 < argc) { if (!ParsePositiveInt(argv[++index], &arguments->stale_after_ms)) return false; }
    else if (option == "--parser-max-bytes" && index + 1 < argc) { int parsed = 0; if (!ParsePositiveInt(argv[++index], &parsed)) return false; arguments->parser_max_bytes = static_cast<std::size_t>(parsed); }
    else return false;
  } return true;
}
std::array<float, 7> CopyServoValues(const unitree_arm::msg::dds_::PubServoInfo_& message) { return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(), message.servo3_data_(), message.servo4_data_(), message.servo5_data_(), message.servo6_data_()}; }
void PrintStats(const char* topic, const rk_arm_feedback_probe::TopicStats& stats) { std::cout << topic << ": received=" << (stats.received ? "true" : "false") << " frames=" << stats.frames << " bad_frames=" << stats.bad_frames << " changed_frames=" << stats.changed_frames << " average_hz=" << stats.average_hz << '\n'; }
/** 本机事件 socket 仅用于人工时间标记，不是 DDS 通道。 */
class EventSocket {
 public:
  explicit EventSocket(const std::string& path) : path_(path) {}
  ~EventSocket() { if (fd_ >= 0) ::close(fd_); if (!path_.empty()) ::unlink(path_.c_str()); }
  bool Open() { if (path_.size() >= sizeof(sockaddr_un::sun_path)) return false; ::unlink(path_.c_str()); fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0); if (fd_ < 0) return false; sockaddr_un address{}; address.sun_family = AF_UNIX; std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1); return ::bind(fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0; }
  void Drain(rk_arm_feedback_probe::FeedbackRecorder* recorder) const { char buffer[128]{}; while (true) { const ssize_t count = ::recv(fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT); if (count <= 0) return; std::string event(buffer, static_cast<std::size_t>(count)); std::string error;
  if (recorder->RecordOperatorEvent(event, &error)) std::cout << "Recorded operator event: " << event << '\n';
  else std::cerr << "Ignored event '" << event << "': " << error << '\n';
  } }
 private: std::string path_; int fd_{-1};
};
}  // namespace

int main(int argc, char** argv) {
  Arguments arguments; if (!ParseArguments(argc, argv, &arguments)) { std::cerr << "Usage: " << argv[0] << " [--interface IFACE] [--output-dir DIR] [--event-socket PATH] [--duration-sec N]\n"; return 2; }
  rk_arm_feedback_probe::ProbeConfig config; config.output_dir = arguments.output_dir; config.stale_after = std::chrono::milliseconds(arguments.stale_after_ms); config.parser_max_payload_bytes = arguments.parser_max_bytes;
  rk_arm_feedback_probe::FeedbackRecorder recorder(config); std::string error; if (!recorder.Open(&error)) { std::cerr << "Recorder setup failed: " << error << '\n'; return 3; }
  EventSocket events(arguments.event_socket); if (!events.Open()) { std::cerr << "Cannot bind local event socket: " << arguments.event_socket << '\n'; return 3; }
  recorder.RegisterTopic(kCommandTopic); recorder.RegisterTopic(kFeedbackTopic); recorder.RegisterTopic(kServoTopic);
  std::signal(SIGINT, HandleSignal); std::signal(SIGTERM, HandleSignal);
  try {
    // 安全边界：本进程只实例化三个 ChannelSubscriber，不含 publisher 或发送调用。
    unitree::robot::ChannelFactory::Instance()->Init(0, arguments.interface_name);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> commands(kCommandTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> feedback(kFeedbackTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servo(kServoTopic);
    commands.InitChannel([&recorder](const void* raw) { recorder.RecordArmCommand(kCommandTopic, static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_()); });
    feedback.InitChannel([&recorder](const void* raw) { recorder.RecordArmFeedback(kFeedbackTopic, static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_()); });
    servo.InitChannel([&recorder](const void* raw) { recorder.RecordServoAngles(kServoTopic, CopyServoValues(*static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw))); });
    std::cout << "Passive D1 command protocol collection started for " << arguments.duration_sec << " seconds. No DDS writer is created. Mark events with d1_command_event --socket " << arguments.event_socket << " EVENT.\n";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(arguments.duration_sec);
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < deadline) { events.Drain(&recorder); recorder.RefreshStaleStates(); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
    recorder.RefreshStaleStates(); commands.CloseChannel(); feedback.CloseChannel(); servo.CloseChannel(); unitree::robot::ChannelFactory::Instance()->Release();
  } catch (const std::exception& exception) { std::cerr << "DDS reader failure: " << exception.what() << '\n'; recorder.Close(); return 4; }
  PrintStats(kCommandTopic, recorder.GetTopicStats(kCommandTopic)); PrintStats(kFeedbackTopic, recorder.GetTopicStats(kFeedbackTopic)); PrintStats(kServoTopic, recorder.GetTopicStats(kServoTopic)); recorder.Close();
  return 0;
}
