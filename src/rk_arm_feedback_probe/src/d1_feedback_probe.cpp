#include "rk_arm_feedback_probe/feedback_recorder.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace {

constexpr const char* kArmFeedbackTopic = "rt/arm_Feedback";
constexpr const char* kServoAngleTopic = "current_servo_angle";
constexpr const char* kRtServoAngleTopic = "rt/current_servo_angle";
std::atomic<bool> g_exit_requested{false};

/** 信号仅请求退出；文件关闭和 DDS reader 销毁都在主线程完成。 */
void HandleSignal(int) { g_exit_requested.store(true); }

struct Arguments {
  std::string interface_name;
  std::string output_dir{"artifacts/d1_feedback_probe"};
  int duration_sec{30};
  int stale_after_ms{1000};
  std::size_t parser_max_bytes{1024U * 1024U};
};

bool ParsePositiveInt(const char* text, int* value) {
  try {
    const int parsed = std::stoi(text);
    if (parsed <= 0) return false;
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseArguments(int argc, char** argv, Arguments* arguments) {
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--interface" && index + 1 < argc) {
      arguments->interface_name = argv[++index];
    } else if (option == "--output-dir" && index + 1 < argc) {
      arguments->output_dir = argv[++index];
    } else if (option == "--duration-sec" && index + 1 < argc) {
      if (!ParsePositiveInt(argv[++index], &arguments->duration_sec)) return false;
    } else if (option == "--stale-after-ms" && index + 1 < argc) {
      if (!ParsePositiveInt(argv[++index], &arguments->stale_after_ms)) return false;
    } else if (option == "--parser-max-bytes" && index + 1 < argc) {
      int parsed = 0;
      if (!ParsePositiveInt(argv[++index], &parsed)) return false;
      arguments->parser_max_bytes = static_cast<std::size_t>(parsed);
    } else {
      return false;
    }
  }
  return true;
}

void PrintUsage(const char* executable) {
  std::cerr << "Usage: " << executable
            << " [--interface IFACE] [--output-dir DIR] [--duration-sec N]"
               " [--stale-after-ms N] [--parser-max-bytes N]\n"
               "This program creates DDS readers only and records passive feedback.\n";
}

std::array<float, 7> CopyServoValues(const unitree_arm::msg::dds_::PubServoInfo_& message) {
  return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(),
          message.servo3_data_(), message.servo4_data_(), message.servo5_data_(),
          message.servo6_data_()};
}

void PrintTopicStats(const char* topic, const rk_arm_feedback_probe::TopicStats& stats) {
  std::cout << topic << ": received=" << (stats.received ? "true" : "false")
            << " frames=" << stats.frames << " bad_frames=" << stats.bad_frames
            << " changed_frames=" << stats.changed_frames
            << " average_hz=" << stats.average_hz
            << " stale=" << (stats.stale ? "true" : "false") << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  Arguments arguments;
  if (!ParseArguments(argc, argv, &arguments)) {
    PrintUsage(argv[0]);
    return 2;
  }

  rk_arm_feedback_probe::ProbeConfig config;
  config.output_dir = arguments.output_dir;
  config.stale_after = std::chrono::milliseconds(arguments.stale_after_ms);
  config.parser_max_payload_bytes = arguments.parser_max_bytes;
  rk_arm_feedback_probe::FeedbackRecorder recorder(config);
  std::string error;
  if (!recorder.Open(&error)) {
    std::cerr << "Recorder setup failed: " << error << '\n';
    return 3;
  }
  // 先登记三种候选 reader；没有数据本身也是需要写入报告的观测结果。
  recorder.RegisterTopic(kArmFeedbackTopic);
  recorder.RegisterTopic(kServoAngleTopic);
  recorder.RegisterTopic(kRtServoAngleTopic);

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  try {
    // SDK 工厂只建立接收通道；此进程没有任何控制接口或命令对象。
    unitree::robot::ChannelFactory::Instance()->Init(0, arguments.interface_name);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> arm_feedback(kArmFeedbackTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servo_angles(kServoAngleTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> rt_servo_angles(kRtServoAngleTopic);
    arm_feedback.InitChannel([&recorder](const void* raw) {
      const auto* message = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw);
      recorder.RecordArmFeedback(kArmFeedbackTopic, message->data_());
    });
    servo_angles.InitChannel([&recorder](const void* raw) {
      const auto* message = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
      recorder.RecordServoAngles(kServoAngleTopic, CopyServoValues(*message));
    });
    rt_servo_angles.InitChannel([&recorder](const void* raw) {
      const auto* message = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
      recorder.RecordServoAngles(kRtServoAngleTopic, CopyServoValues(*message));
    });

    std::cout << "Passive D1 feedback collection started for " << arguments.duration_sec
              << " seconds. No command channel is created.\n";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(arguments.duration_sec);
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < deadline) {
      recorder.RefreshStaleStates();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    recorder.RefreshStaleStates();
    arm_feedback.CloseChannel();
    servo_angles.CloseChannel();
    rt_servo_angles.CloseChannel();
    unitree::robot::ChannelFactory::Instance()->Release();
  } catch (const std::exception& exception) {
    std::cerr << "DDS reader failure: " << exception.what() << '\n';
    recorder.Close();
    return 4;
  }

  PrintTopicStats(kArmFeedbackTopic, recorder.GetTopicStats(kArmFeedbackTopic));
  PrintTopicStats(kServoAngleTopic, recorder.GetTopicStats(kServoAngleTopic));
  PrintTopicStats(kRtServoAngleTopic, recorder.GetTopicStats(kRtServoAngleTopic));
  recorder.Close();
  std::cout << "Raw evidence and protocol_summary.json are in " << arguments.output_dir << '\n';
  return 0;
}
