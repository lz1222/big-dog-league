// Go2 高层 SportModeCmd 的被动观察器。
//
// 该工具只订阅 rt/sportmodecmd，用于复盘 APP/遥控器是否经 DDS 命令面改变
// 步态。它不创建 SportClient、Publisher 或 UDP socket，故不会发出动作命令。
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

constexpr const char* kSportModeCommandTopic = "rt/sportmodecmd";
std::atomic<bool> g_running{true};

void SignalHandler(int)
{
  g_running.store(false);
}

class SportModeCommandObserver
{
public:
  SportModeCommandObserver()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeCmd_>(kSportModeCommandTopic));
    subscriber_->InitChannel(
        std::bind(&SportModeCommandObserver::OnCommand, this,
                  std::placeholders::_1),
        1);
  }

  void PrintSummary() const
  {
    std::cout << "SPORT_MODE_COMMAND_OBSERVER event=SUMMARY"
              << " command_count=" << command_count_.load() << std::endl;
  }

private:
  void OnCommand(const void* raw_message)
  {
    if (raw_message == nullptr) {
      return;
    }
    const auto& command = *static_cast<const unitree_go::msg::dds_::SportModeCmd_*>(
        raw_message);
    const int count = command_count_.fetch_add(1) + 1;
    const auto& position = command.position();
    const auto& euler = command.euler();
    const auto& velocity = command.velocity();
    // 原样记录协议字段和到达顺序；不把 gait 数值预先解释为任何步态名称。
    std::cout << std::fixed << std::setprecision(6)
              << "SPORT_MODE_COMMAND event=RECEIVED"
              << " count=" << count
              << " monotonic_ns=" << std::chrono::duration_cast<
                     std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch()).count()
              << " mode=" << static_cast<int>(command.mode())
              << " gait_type=" << static_cast<int>(command.gait_type())
              << " speed_level=" << static_cast<int>(command.speed_level())
              << " foot_raise_height=" << command.foot_raise_height()
              << " body_height=" << command.body_height()
              << " position=[" << position[0] << "," << position[1] << "]"
              << " euler=[" << euler[0] << "," << euler[1] << "," << euler[2]
              << "]"
              << " velocity=[" << velocity[0] << "," << velocity[1] << "]"
              << std::endl;
  }

  std::atomic<int> command_count_{0};
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeCmd_>
      subscriber_;
};

int RunObserver(const std::string& network_interface, double duration_sec)
{
  if (network_interface.empty() || duration_sec <= 0.0) {
    throw std::runtime_error("network_interface and duration_sec must be positive");
  }
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  std::cout << "SPORT_MODE_COMMAND_OBSERVER event=START"
            << " interface=" << network_interface
            << " domain=0 topic=" << kSportModeCommandTopic
            << " duration_sec=" << duration_sec << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  SportModeCommandObserver observer;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  observer.PrintSummary();
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 3) {
      throw std::runtime_error(
          "Usage: go2_sdk_sport_mode_command_observer <network_interface> <duration_sec>");
    }
    return RunObserver(argv[1], std::stod(argv[2]));
  } catch (const std::exception& error) {
    std::cerr << "SPORT_MODE_COMMAND_OBSERVER event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
