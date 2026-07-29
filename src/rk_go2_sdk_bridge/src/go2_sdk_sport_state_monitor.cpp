// 只读订阅 Go2 高层运动状态，用于在发送 Move 前确认当前步态和反馈速度。
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

constexpr const char* kSportModeStateTopic = "rt/sportmodestate";
constexpr double kDefaultDurationSec = 3.0;
constexpr double kDefaultPrintRateHz = 2.0;

double ParsePositiveDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be finite and positive");
  }
  return value;
}

const char* GaitTypeName(uint8_t gait_type)
{
  // 名称来自 Unitree SDK2 Go2 示例；未知值保留原始数字，避免错误解释固件扩展。
  switch (gait_type) {
    case 0:
      return "idle";
    case 1:
      return "trot";
    case 2:
      return "trot_running";
    case 3:
      return "climb_stair";
    case 4:
      return "trot_obstacle";
    default:
      return "unknown";
  }
}

class SportStateMonitor
{
public:
  SportStateMonitor()
  {
    subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(
            &SportStateMonitor::OnState, this, std::placeholders::_1),
        1);
  }

  bool Snapshot(unitree_go::msg::dds_::SportModeState_& state)
  {
    if (!received_.load()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state = state_;
    return true;
  }

private:
  void OnState(const void* message)
  {
    if (message == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
        message);
    received_.store(true);
  }

  std::mutex mutex_;
  std::atomic<bool> received_{false};
  unitree_go::msg::dds_::SportModeState_ state_;
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::SportModeState_> subscriber_;
};

void PrintState(const unitree_go::msg::dds_::SportModeState_& state)
{
  const auto& position = state.position();
  const auto& velocity = state.velocity();
  const auto& rpy = state.imu_state().rpy();

  std::cout
      << "SPORT_STATE"
      << " mode=" << static_cast<int>(state.mode())
      << " gait_type=" << static_cast<int>(state.gait_type())
      << " gait_name=" << GaitTypeName(state.gait_type())
      << " progress=" << state.progress()
      << " error_code=" << state.error_code()
      << " position=[" << position[0] << "," << position[1] << ","
      << position[2] << "]"
      << " velocity=[" << velocity[0] << "," << velocity[1] << ","
      << velocity[2] << "]"
      << " yaw_speed=" << state.yaw_speed()
      << " rpy=[" << rpy[0] << "," << rpy[1] << "," << rpy[2] << "]"
      << " body_height=" << state.body_height()
      << " foot_raise_height=" << state.foot_raise_height()
      << std::endl;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> [duration_sec] [print_rate_hz]\n\n"
      << "This tool is READ_ONLY and subscribes to "
      << kSportModeStateTopic << ".\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    const double duration_sec =
        argc >= 3 ? ParsePositiveDouble(argv[2], "duration_sec")
                  : kDefaultDurationSec;
    const double print_rate_hz =
        argc >= 4 ? ParsePositiveDouble(argv[3], "print_rate_hz")
                  : kDefaultPrintRateHz;

    std::cout << "READ_ONLY: subscribing to " << kSportModeStateTopic
              << " on interface " << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(
        0, network_interface);
    SportStateMonitor monitor;

    const auto start_time = std::chrono::steady_clock::now();
    const auto end_time = start_time + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(duration_sec));
    const auto print_period = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / print_rate_hz));
    auto next_print = start_time;
    bool printed = false;

    while (std::chrono::steady_clock::now() < end_time) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_print) {
        unitree_go::msg::dds_::SportModeState_ state;
        if (monitor.Snapshot(state)) {
          PrintState(state);
          printed = true;
        }
        next_print = now + print_period;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!printed) {
      std::cerr << "No SportModeState received within "
                << duration_sec << "s" << std::endl;
      return 1;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
