// 标准 sport_mode 的单次开关恢复步骤。
//
// 命令行只接受 off/on，服务名固定为 sport_mode；这样诊断不能意外切换
// ai_sport、advanced_sport、mcf 或任意其他服务。每个进程只发出一次请求。
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace
{

constexpr const char* kSportModeService = "sport_mode";

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

int32_t ParseSwitchValue(const std::string& operation)
{
  if (operation == "off") {
    return 0;
  }
  if (operation == "on") {
    return 1;
  }
  throw std::runtime_error("operation must be exactly off or on");
}

int RunSwitch(const std::string& network_interface, const std::string& operation)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  const int32_t switch_value = ParseSwitchValue(operation);
  std::cout << "MODE_RECOVERY event=CHANNEL_FACTORY_INIT interface="
            << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::go2::RobotStateClient client;
  client.SetTimeout(2.0F);
  client.Init();
  int32_t status = 0;
  // 服务名是编译期常量，且唯一一次调用后立刻退出，不存在循环或备用目标。
  const auto started = std::chrono::steady_clock::now();
  const int32_t result = client.ServiceSwitch(kSportModeService, switch_value, status);
  std::cout << "MODE_RECOVERY event=SPORT_MODE_SWITCH"
            << " operation=" << operation
            << " service=" << kSportModeService
            << " ret=" << result
            << " status=" << status
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(started) << std::endl;
  return result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 3) {
      throw std::runtime_error(
          "Usage: go2_sdk_sport_mode_switch_once <network_interface> <off|on>");
    }
    return RunSwitch(argv[1], argv[2]);
  } catch (const std::exception& error) {
    std::cerr << "MODE_RECOVERY event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
