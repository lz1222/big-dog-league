// Go2 遥控器状态的被动观察器。
//
// 该工具只订阅 rt/wirelesscontroller，并把按键位图变化与摇杆原始值写入日志，
// 用于将人工切换动作同 SportModeState 变化对齐。它不创建 Publisher、SportClient
// 或 UDP socket，因此不能向机器人发送控制命令。
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

constexpr const char* kWirelessControllerTopic = "rt/wirelesscontroller";
// 位定义来自 SDK 的 advanced_gamepad.hpp；仅用于可读日志，不推断步态语义。
constexpr std::array<const char*, 16> kKeyNames{
    "R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
    "A", "B", "X", "Y", "up", "right", "down", "left"};
std::atomic<bool> g_running{true};

void SignalHandler(int)
{
  g_running.store(false);
}

std::string PressedKeyNames(uint16_t keys)
{
  std::ostringstream output;
  bool first = true;
  for (size_t index = 0; index < kKeyNames.size(); ++index) {
    if ((keys & (static_cast<uint16_t>(1) << index)) == 0) {
      continue;
    }
    if (!first) {
      output << ',';
    }
    output << kKeyNames[index];
    first = false;
  }
  return first ? "none" : output.str();
}

class WirelessControllerObserver
{
public:
  WirelessControllerObserver()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::WirelessController_>(kWirelessControllerTopic));
    subscriber_->InitChannel(
        std::bind(&WirelessControllerObserver::OnController, this,
                  std::placeholders::_1),
        1);
  }

  void PrintSummary() const
  {
    std::cout << "WIRELESS_CONTROLLER_OBSERVER event=SUMMARY"
              << " message_count=" << message_count_.load()
              << " key_change_count=" << key_change_count_.load() << std::endl;
  }

private:
  void OnController(const void* raw_message)
  {
    if (raw_message == nullptr) {
      return;
    }
    const auto& controller =
        *static_cast<const unitree_go::msg::dds_::WirelessController_*>(raw_message);
    message_count_.fetch_add(1);
    const uint16_t keys = controller.keys();
    // 连续消息中只记录按键位图的边沿，避免空闲 50 Hz 状态淹没人工组合按键。
    if (last_keys_.has_value() && last_keys_.value() == keys) {
      return;
    }
    last_keys_ = keys;
    const int count = key_change_count_.fetch_add(1) + 1;
    std::cout << std::fixed << std::setprecision(6)
              << "WIRELESS_CONTROLLER event=KEYS_CHANGED"
              << " count=" << count
              << " monotonic_ns=" << std::chrono::duration_cast<
                     std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch()).count()
              << " keys_dec=" << keys
              << " keys_hex=0x" << std::hex << std::uppercase << keys << std::dec
              << " pressed=[" << PressedKeyNames(keys) << ']'
              << " lx=" << controller.lx()
              << " ly=" << controller.ly()
              << " rx=" << controller.rx()
              << " ry=" << controller.ry()
              << std::endl;
  }

  std::atomic<int> message_count_{0};
  std::atomic<int> key_change_count_{0};
  std::optional<uint16_t> last_keys_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_>
      subscriber_;
};

int RunObserver(const std::string& network_interface, double duration_sec)
{
  if (network_interface.empty() || duration_sec <= 0.0) {
    throw std::runtime_error("network_interface and duration_sec must be positive");
  }
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  std::cout << "WIRELESS_CONTROLLER_OBSERVER event=START"
            << " interface=" << network_interface
            << " domain=0 topic=" << kWirelessControllerTopic
            << " duration_sec=" << duration_sec << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  WirelessControllerObserver observer;
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
          "Usage: go2_sdk_wireless_controller_observer <network_interface> <duration_sec>");
    }
    return RunObserver(argv[1], std::stod(argv[2]));
  } catch (const std::exception& error) {
    std::cerr << "WIRELESS_CONTROLLER_OBSERVER event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
