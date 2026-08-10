// Go2 SportClient 的只读 service identity / version 诊断工具。
//
// 该工具显式关闭 lease，只调用 Client 基类提供的版本查询，用来判断 Sport
// RPC 是否能建立服务版本握手。源码中绝不调用任何 sport 动作或服务切换接口，
// 因而执行期间不会向机器人发送运动指令或改变服务状态。
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

std::string ParseInterface(int argc, char** argv)
{
  if (argc != 2 || std::string(argv[1]).empty()) {
    throw std::runtime_error(
        "Usage: go2_sdk_sport_identity_probe <network_interface>");
  }
  return argv[1];
}

int RunProbe(const std::string& network_interface)
{
  std::cout << "SPORT_CLIENT_DIAG event=CHANNEL_FACTORY_INIT"
            << " interface=" << network_interface
            << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  // 显式 false 让每次独立 probe 都不申请或继承运动 lease。
  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(2.0F);
  const auto init_started = std::chrono::steady_clock::now();
  client.Init();
  std::cout << std::fixed << std::setprecision(3)
            << "SPORT_CLIENT_DIAG event=INIT"
            << " constructor_enable_lease=false"
            << " init_elapsed_ms=" << ElapsedMilliseconds(init_started)
            << std::endl;

  std::cout << "SPORT_CLIENT_DIAG event=CLIENT_API_VERSION"
            << " value=" << std::quoted(client.GetApiVersion()) << std::endl;

  // 官方接口只返回 string，不存在可记录的 SDK int32 ret；空值原样输出，
  // 由调用方据此分类，避免把它擅自映射为任意错误码。
  const auto server_version_started = std::chrono::steady_clock::now();
  const std::string server_api_version = client.GetServerApiVersion();
  std::cout << "SPORT_CLIENT_DIAG event=SERVER_API_VERSION"
            << " ret=API_NO_RET"
            << " get_server_version_elapsed_ms=" << std::fixed
            << std::setprecision(3)
            << ElapsedMilliseconds(server_version_started)
            << " value=" << std::quoted(server_api_version) << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunProbe(ParseInterface(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "SPORT_CLIENT_DIAG event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
