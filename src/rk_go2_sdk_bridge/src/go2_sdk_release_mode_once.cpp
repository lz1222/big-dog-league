// MotionSwitcher ReleaseMode 的一次性恢复步骤。
//
// 本工具只调用一次 ReleaseMode 并立即退出，供人工已确认的趴下/机械支撑
// 条件下释放当前模式。它不包含任何姿态、速度或模式选择操作，也没有重试。
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace
{

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

int RunRelease(const std::string& network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  std::cout << "MODE_RECOVERY event=CHANNEL_FACTORY_INIT interface="
            << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::b2::MotionSwitcherClient client;
  client.SetTimeout(2.0F);
  client.Init();
  // 唯一一次状态变更；ret 非零由调用编排立刻 fail-closed。
  const auto started = std::chrono::steady_clock::now();
  const int32_t result = client.ReleaseMode();
  std::cout << "MODE_RECOVERY event=RELEASE_MODE ret=" << result
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(started) << std::endl;
  return result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: go2_sdk_release_mode_once <network_interface>");
    }
    return RunRelease(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "MODE_RECOVERY event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
