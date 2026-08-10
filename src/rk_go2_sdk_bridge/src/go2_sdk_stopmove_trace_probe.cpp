// StopMove API 1003 的单次追踪 probe。
//
// 本工具只为关联被动 DDS observer 的一次 StopMove 请求而存在：每次进程仅
// 调用一次零速度停车 API，随后立即退出。禁止加入重试、其他动作或模式切换。
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_api.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

int RunProbe(const std::string& network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  std::cout << "STOPMOVE_TRACE event=CHANNEL_FACTORY_INIT interface="
            << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(2.0F);
  const auto init_started = std::chrono::steady_clock::now();
  client.Init();
  std::cout << "STOPMOVE_TRACE event=INIT enable_lease=false elapsed_ms="
            << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(init_started) << std::endl;
  std::cout << "STOPMOVE_TRACE event=CLIENT_API_VERSION value="
            << std::quoted(client.GetApiVersion()) << std::endl;
  const auto version_started = std::chrono::steady_clock::now();
  const std::string server_version = client.GetServerApiVersion();
  std::cout << "STOPMOVE_TRACE event=SERVER_API_VERSION ret=API_NO_RET"
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(version_started)
            << " value=" << std::quoted(server_version) << std::endl;

  // 唯一一次动作调用：零速度 StopMove，不能添加 retry 或任何后续动作。
  const auto stop_started = std::chrono::steady_clock::now();
  const int32_t result = client.StopMove();
  std::cout << "STOPMOVE_TRACE event=STOP_MOVE"
            << " api_id=" << unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE
            << " ret=" << result
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(stop_started) << std::endl;
  return result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: go2_sdk_stopmove_trace_probe <network_interface>");
    }
    return RunProbe(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "STOPMOVE_TRACE event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
