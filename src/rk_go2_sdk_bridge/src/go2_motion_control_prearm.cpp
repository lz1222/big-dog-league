// 正式运动控制 prearm gate：在创建 UDP SDK server 前只读验证控制面。
//
// mcf 是已实测的经典静止状态，不能据此释放 MotionSwitcher。正式步态进入
// 由 competition_gait_manager 独占；本程序绝不调用 ReleaseMode、服务开关
// 或任何 Sport 动作，避免启动诊断改变实体姿态或控制环境。

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

struct Config
{
  std::string interface;
  double version_timeout_sec{3.0};
};

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

Config ParseArguments(int argc, char** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--interface" && index + 1 < argc) {
      config.interface = argv[++index];
    } else if (option == "--version-timeout-sec" && index + 1 < argc) {
      config.version_timeout_sec = std::stod(argv[++index]);
    } else {
      throw std::runtime_error("invalid prearm argument: " + option);
    }
  }
  if (config.interface.empty() || config.version_timeout_sec <= 0.0) {
    throw std::runtime_error("interface and version timeout must be positive");
  }
  return config;
}

void PrintResult(const std::string& classification, bool ready)
{
  std::cout << "MOTION_CONTROL_PREARM event=RESULT classification="
            << classification << " ready=" << (ready ? "true" : "false")
            << std::endl;
}

void PrintCheckMode(
    const std::string& phase, int32_t result, const std::string& form,
    const std::string& name)
{
  std::cout << "MOTION_CONTROL_PREARM event=CHECK_MODE phase=" << phase
            << " ret=" << result << " form=" << std::quoted(form)
            << " name=" << std::quoted(name) << std::endl;
}

std::string QueryServerVersion(const Config& config, const std::string& phase)
{
  // 每次查询创建新 SportClient(false)，避免 service restart 前的 client 被复用。
  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(static_cast<float>(config.version_timeout_sec));
  const auto init_started = std::chrono::steady_clock::now();
  client.Init();
  const auto query_started = std::chrono::steady_clock::now();
  const std::string client_version = client.GetApiVersion();
  const std::string server_version = client.GetServerApiVersion();
  std::cout << "MOTION_CONTROL_PREARM event=SPORT_VERSION phase=" << phase
            << " enable_lease=false"
            << " init_elapsed_ms=" << std::fixed << std::setprecision(3)
            << ElapsedMilliseconds(init_started)
            << " query_elapsed_ms=" << ElapsedMilliseconds(query_started)
            << " client=" << std::quoted(client_version)
            << " server=" << std::quoted(server_version) << std::endl;
  return server_version;
}

int RunPrearm(const Config& config)
{
  std::cout << "MOTION_CONTROL_PREARM event=CHANNEL_FACTORY_INIT interface="
            << config.interface << " domain=0 mutation_count=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, config.interface);

  unitree::robot::b2::MotionSwitcherClient switcher;
  switcher.SetTimeout(2.0F);
  switcher.Init();
  std::string form;
  std::string name;
  const int32_t check_result = switcher.CheckMode(form, name);
  PrintCheckMode("initial", check_result, form, name);
  if (check_result != 0 || form != "0") {
    PrintResult("MOTION_SWITCHER_CHECK_FAILED", false);
    return 1;
  }

  // 空 name 与 mcf 都是只读可接受的观察值；真实步态建立留给唯一 owner。
  std::cout << "MOTION_CONTROL_PREARM event=MOTION_SWITCHER_OBSERVED name="
            << std::quoted(name) << " mutation_count=0" << std::endl;

  std::string server_version = QueryServerVersion(config, "initial");
  if (server_version.empty()) {
    PrintResult("SPORT_RPC_UNAVAILABLE_NO_MUTATION", false);
    return 1;
  }
  PrintResult("PASS_READ_ONLY", true);
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunPrearm(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "MOTION_CONTROL_PREARM event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
