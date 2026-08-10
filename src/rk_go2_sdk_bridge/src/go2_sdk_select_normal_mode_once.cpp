// MotionSwitcher 标准 normal mode 的一次性选择步骤。
//
// normal 是编译期固定目标，工具不接受任意 mode 名称，且只调用一次
// SelectMode 后退出。它不包含 ReleaseMode、ServiceSwitch 或任何运动 API。
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace
{

constexpr const char* kNormalMode = "normal";

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

int RunSelect(const std::string& network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  std::cout << "SELECT_NORMAL event=CHANNEL_FACTORY_INIT interface="
            << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::b2::MotionSwitcherClient client;
  client.SetTimeout(2.0F);
  client.Init();
  // 唯一的状态变更调用：目标固定 normal，非零 ret 由外层立即停止。
  const auto started = std::chrono::steady_clock::now();
  const int32_t result = client.SelectMode(kNormalMode);
  std::cout << "SELECT_NORMAL event=SELECT_MODE target=" << kNormalMode
            << " ret=" << result
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(started) << std::endl;
  return result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw std::runtime_error(
          "Usage: go2_sdk_select_normal_mode_once <network_interface>");
    }
    return RunSelect(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "SELECT_NORMAL event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
