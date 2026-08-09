// 正式运动控制 prearm gate：在创建 UDP SDK server 前恢复并验证控制权。
//
// 此程序只允许启动阶段一次性执行 ReleaseMode 和 sport_mode OFF/ON；所有
// 分支均 fail-closed，绝不包含 Sport 动作、mode 选择、2055 或重试循环。

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

struct Config
{
  std::string interface;
  bool enable_recovery{false};
  double version_timeout_sec{3.0};
  double recovery_timeout_sec{10.0};
};

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

bool ParseBoolean(const std::string& value)
{
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  throw std::runtime_error("enable-recovery must be true or false");
}

Config ParseArguments(int argc, char** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--interface" && index + 1 < argc) {
      config.interface = argv[++index];
    } else if (option == "--enable-recovery" && index + 1 < argc) {
      config.enable_recovery = ParseBoolean(argv[++index]);
    } else if (option == "--version-timeout-sec" && index + 1 < argc) {
      config.version_timeout_sec = std::stod(argv[++index]);
    } else if (option == "--recovery-timeout-sec" && index + 1 < argc) {
      config.recovery_timeout_sec = std::stod(argv[++index]);
    } else {
      throw std::runtime_error("invalid prearm argument: " + option);
    }
  }
  if (config.interface.empty() || config.version_timeout_sec <= 0.0 ||
      config.recovery_timeout_sec <= 0.0) {
    throw std::runtime_error("interface and timeouts must be positive");
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

bool CheckEmptyMode(
    unitree::robot::b2::MotionSwitcherClient& client, const std::string& phase,
    std::string& form, std::string& name)
{
  form.clear();
  name.clear();
  const int32_t result = client.CheckMode(form, name);
  PrintCheckMode(phase, result, form, name);
  return result == 0 && form == "0" && name.empty();
}

bool ServiceListContainsSportMode(unitree::robot::go2::RobotStateClient& client,
                                  const std::string& phase)
{
  std::vector<unitree::robot::go2::ServiceState> services;
  const int32_t result = client.ServiceList(services);
  std::cout << "MOTION_CONTROL_PREARM event=SERVICE_LIST phase=" << phase
            << " ret=" << result << " count=" << services.size() << std::endl;
  if (result != 0) return false;
  for (const auto& service : services) {
    if (service.name == "sport_mode") {
      // status/protect 仅保留原始值；RPC version 才是最终 responder 证据。
      std::cout << "MOTION_CONTROL_PREARM event=SPORT_MODE_SERVICE phase="
                << phase << " status=" << service.status
                << " protect=" << service.protect << std::endl;
      return true;
    }
  }
  return false;
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
            << config.interface << " domain=0 enable_recovery="
            << (config.enable_recovery ? "true" : "false") << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, config.interface);

  unitree::robot::b2::MotionSwitcherClient switcher;
  switcher.SetTimeout(2.0F);
  switcher.Init();
  std::string form;
  std::string name;
  if (!CheckEmptyMode(switcher, "initial", form, name)) {
    if (form != "0" || name.empty()) {
      PrintResult("MOTION_SWITCHER_CHECK_FAILED", false);
      return 1;
    }
    if (name != "mcf") {
      PrintResult("UNKNOWN_MOTION_MODE", false);
      return 1;
    }
    if (!config.enable_recovery) {
      PrintResult("MOTION_SWITCHER_RECOVERY_DISABLED", false);
      return 1;
    }
    const auto release_started = std::chrono::steady_clock::now();
    const int32_t release_result = switcher.ReleaseMode();
    std::cout << "MOTION_CONTROL_PREARM event=RELEASE_MODE count=1 ret="
              << release_result << " elapsed_ms=" << std::fixed
              << std::setprecision(3) << ElapsedMilliseconds(release_started)
              << std::endl;
    if (release_result != 0) {
      PrintResult("MOTION_SWITCHER_RELEASE_FAILED", false);
      return 1;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(2);
    bool empty = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (CheckEmptyMode(switcher, "post_release", form, name)) {
        empty = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!empty) {
      PrintResult("MOTION_SWITCHER_STILL_OWNED", false);
      return 1;
    }
  }

  std::string server_version = QueryServerVersion(config, "initial");
  if (!server_version.empty()) {
    PrintResult("PASS", true);
    return 0;
  }
  if (!config.enable_recovery) {
    PrintResult("SPORT_RPC_RECOVERY_DISABLED", false);
    return 1;
  }

  unitree::robot::go2::RobotStateClient robot_state;
  robot_state.SetTimeout(2.0F);
  robot_state.Init();
  int32_t returned_status = 0;
  const auto off_started = std::chrono::steady_clock::now();
  const int32_t off_result = robot_state.ServiceSwitch(
      "sport_mode", 0, returned_status);
  std::cout << "MOTION_CONTROL_PREARM event=SPORT_MODE_SWITCH count=1"
            << " operation=off ret=" << off_result
            << " status=" << returned_status << " elapsed_ms=" << std::fixed
            << std::setprecision(3) << ElapsedMilliseconds(off_started)
            << std::endl;
  if (off_result != 0 || !ServiceListContainsSportMode(robot_state, "after_off")) {
    PrintResult("SPORT_RPC_RECOVERY_FAILED", false);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  returned_status = 0;
  const auto on_started = std::chrono::steady_clock::now();
  const int32_t on_result = robot_state.ServiceSwitch(
      "sport_mode", 1, returned_status);
  std::cout << "MOTION_CONTROL_PREARM event=SPORT_MODE_SWITCH count=1"
            << " operation=on ret=" << on_result
            << " status=" << returned_status << " elapsed_ms=" << std::fixed
            << std::setprecision(3) << ElapsedMilliseconds(on_started)
            << std::endl;
  if (on_result != 0) {
    PrintResult("SPORT_RPC_RECOVERY_FAILED", false);
    return 1;
  }

  const auto recovery_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(config.recovery_timeout_sec);
  int attempt = 0;
  while (std::chrono::steady_clock::now() < recovery_deadline) {
    ++attempt;
    server_version = QueryServerVersion(
        config, "post_restart_" + std::to_string(attempt));
    if (!server_version.empty()) {
      PrintResult("PASS", true);
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  PrintResult("SPORT_RPC_RECOVERY_FAILED", false);
  return 1;
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
