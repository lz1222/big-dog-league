// Go2 Sport API 的只读 AutoRecoverGet 诊断工具。
//
// 2055 是当前 SDK 定义的 AutoRecoverGet API。它只读取自动恢复标志，用于
// 定位 SportClient 通用 Call 路径，绝不调用任何速度、姿态或服务切换接口。
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
  std::cout << "SPORT_READ_PROBE event=CHANNEL_FACTORY_INIT"
            << " interface=" << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  // 显式 false：诊断不申请或继承运动 lease。
  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(2.0F);
  const auto init_started = std::chrono::steady_clock::now();
  client.Init();
  std::cout << std::fixed << std::setprecision(3)
            << "SPORT_READ_PROBE event=INIT enable_lease=false elapsed_ms="
            << ElapsedMilliseconds(init_started) << std::endl;
  std::cout << "SPORT_READ_PROBE event=CLIENT_API_VERSION value="
            << std::quoted(client.GetApiVersion()) << std::endl;

  // SDK 签名只返回 string，空值也必须原样保留，不能猜测为错误码。
  const auto version_started = std::chrono::steady_clock::now();
  const std::string server_version = client.GetServerApiVersion();
  std::cout << "SPORT_READ_PROBE event=SERVER_API_VERSION ret=API_NO_RET"
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(version_started)
            << " value=" << std::quoted(server_version) << std::endl;

  bool auto_recover = false;
  const auto request_started = std::chrono::steady_clock::now();
  const int32_t result = client.AutoRecoverGet(auto_recover);
  std::cout << "SPORT_READ_PROBE event=AUTORECOVER_GET"
            << " api_id="
            << unitree::robot::go2::ROBOT_SPORT_API_ID_AUTORECOVERY_GET
            << " ret=" << result
            << " value=" << (auto_recover ? "true" : "false")
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(request_started) << std::endl;
  return result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw std::runtime_error(
          "Usage: go2_sdk_sport_autorecover_get_probe <network_interface>");
    }
    return RunProbe(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "SPORT_READ_PROBE event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
