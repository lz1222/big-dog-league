// Go2 RobotState RPC 只读诊断工具。
//
// 本工具只查询 API 版本与机器人已声明的服务列表，用于区分 DDS 状态订阅
// 正常但 RPC 请求路径异常的情况。它刻意不包含高层运动客户端，也不调用
// 服务状态切换或任何动作接口，因而不会改变机器人服务状态或运动状态。
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace
{

struct ProbeConfig
{
  std::string network_interface;
  float timeout_sec{10.0F};
};

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

ProbeConfig ParseArguments(int argc, char** argv)
{
  if (argc < 2 || argc > 3) {
    throw std::runtime_error(
        "Usage: go2_sdk_robot_state_probe <network_interface> [timeout_sec]");
  }

  ProbeConfig config;
  config.network_interface = argv[1];
  if (argc == 3) {
    config.timeout_sec = std::stof(argv[2]);
  }
  if (config.network_interface.empty() || config.timeout_sec <= 0.0F) {
    throw std::runtime_error("network_interface and timeout_sec must be positive");
  }
  return config;
}

void PrintServices(
    const std::vector<unitree::robot::go2::ServiceState>& services)
{
  // status/protect 是机器人原始协议字段；这里不推断其数值含义。
  for (const auto& service : services) {
    std::cout << "RPC_PROBE event=SERVICE"
              << " name=" << std::quoted(service.name)
              << " status=" << service.status
              << " protect=" << service.protect << std::endl;
  }
}

void PrintRepeatability(
    const std::vector<int32_t>& results,
    const std::vector<double>& elapsed_ms)
{
  std::map<int32_t, int> distribution;
  for (const int32_t result : results) {
    ++distribution[result];
  }
  const int success_count = static_cast<int>(
      std::count(results.begin(), results.end(), static_cast<int32_t>(0)));
  const double min_elapsed = *std::min_element(elapsed_ms.begin(), elapsed_ms.end());
  const double max_elapsed = *std::max_element(elapsed_ms.begin(), elapsed_ms.end());
  const double mean_elapsed = std::accumulate(
      elapsed_ms.begin(), elapsed_ms.end(), 0.0) / elapsed_ms.size();

  std::cout << std::fixed << std::setprecision(3)
            << "RPC_PROBE event=SERVICE_LIST_REPEATABILITY"
            << " attempts=" << results.size()
            << " success=" << success_count
            << " elapsed_min_ms=" << min_elapsed
            << " elapsed_mean_ms=" << mean_elapsed
            << " elapsed_max_ms=" << max_elapsed;
  for (const auto& item : distribution) {
    std::cout << " ret_" << item.first << "_count=" << item.second;
  }
  std::cout << std::endl;
}

int RunProbe(const ProbeConfig& config)
{
  std::cout << "RPC_PROBE event=CHANNEL_FACTORY_INIT"
            << " interface=" << config.network_interface
            << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(
      0, config.network_interface);

  unitree::robot::go2::RobotStateClient client;
  client.SetTimeout(config.timeout_sec);
  const auto init_started = std::chrono::steady_clock::now();
  client.Init();
  std::cout << std::fixed << std::setprecision(3)
            << "RPC_PROBE event=ROBOT_STATE_INIT"
            << " elapsed_ms=" << ElapsedMilliseconds(init_started) << std::endl;

  // GetApiVersion 是本地 SDK 元数据；GetServerApiVersion 的官方签名只返回
  // string，故不能伪造不存在的 int32 ret，仍保留其真实耗时和原始返回值。
  std::cout << "RPC_PROBE event=CLIENT_API_VERSION"
            << " value=" << std::quoted(client.GetApiVersion()) << std::endl;
  const auto server_version_started = std::chrono::steady_clock::now();
  const std::string server_api_version = client.GetServerApiVersion();
  std::cout << "RPC_PROBE event=SERVER_API_VERSION"
            << " ret=API_NO_RET"
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(server_version_started)
            << " value=" << std::quoted(server_api_version) << std::endl;

  std::vector<unitree::robot::go2::ServiceState> services;
  const auto first_list_started = std::chrono::steady_clock::now();
  const int32_t first_result = client.ServiceList(services);
  const double first_elapsed_ms = ElapsedMilliseconds(first_list_started);
  std::cout << "RPC_PROBE event=SERVICE_LIST"
            << " attempt=1"
            << " ret=" << first_result
            << " elapsed_ms=" << std::fixed << std::setprecision(3)
            << first_elapsed_ms
            << " count=" << services.size() << std::endl;
  if (first_result != 0) {
    return 1;
  }
  PrintServices(services);

  // 首次成功后固定再调用九次，总计十次；只用于评估只读 RPC 稳定性。
  std::vector<int32_t> results{first_result};
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(10);
  elapsed_ms.push_back(first_elapsed_ms);
  for (int attempt = 2; attempt <= 10; ++attempt) {
    std::vector<unitree::robot::go2::ServiceState> repeated_services;
    const auto started = std::chrono::steady_clock::now();
    const int32_t result = client.ServiceList(repeated_services);
    const double elapsed = ElapsedMilliseconds(started);
    results.push_back(result);
    elapsed_ms.push_back(elapsed);
    std::cout << "RPC_PROBE event=SERVICE_LIST"
              << " attempt=" << attempt
              << " ret=" << result
              << " elapsed_ms=" << std::fixed << std::setprecision(3) << elapsed
              << " count=" << repeated_services.size() << std::endl;
  }
  PrintRepeatability(results, elapsed_ms);
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunProbe(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "RPC_PROBE event=FATAL message=" << std::quoted(error.what())
              << std::endl;
    return 1;
  }
}
