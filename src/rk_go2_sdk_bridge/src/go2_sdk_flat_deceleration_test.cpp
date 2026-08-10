// 平地分级减速验证工具。
//
// 本工具只验证经典步态下“较快直行 → 多档减速 → StopMove”的速度指令序列，
// 不包含 T 识别、巡线、台阶动作或步态切换。它不能预测台阶上的实际停车位置，
// 因为台阶接触、足端打滑和步态相位都会改变制动距离。默认 Dry Run；只有
// --execute 才初始化 SDK 并向真机发送 Move 命令。

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

// ===== 平地分级减速参数总表：先在此处看默认值，也可用 --stage 覆盖 =====
constexpr double kDefaultFastSpeedMps = 0.30;      // 第 1 档较快前进速度（m/s）
constexpr double kDefaultFastDurationSec = 1.00;  // 第 1 档持续时间（s）
constexpr double kDefaultMiddleSpeedMps = 0.22;   // 第 2 档减速后速度（m/s）
constexpr double kDefaultMiddleDurationSec = 0.40;// 第 2 档持续时间（s）
constexpr double kDefaultSlowSpeedMps = 0.14;     // 第 3 档低速（m/s）
constexpr double kDefaultSlowDurationSec = 0.40;  // 第 3 档持续时间（s）
constexpr double kDefaultCreepSpeedMps = 0.06;    // 第 4 档极低速（m/s）
constexpr double kDefaultCreepDurationSec = 0.40; // 第 4 档持续时间（s）
constexpr double kCommandRateHz = 20.0;           // 每秒发送 Move 的次数
constexpr double kMaximumSpeedMps = 0.40;          // 平地首次验证限速，避免直接高速试验
constexpr double kMaximumStageDurationSec = 10.0; // 单档最长持续时间，防止误填无限前进

volatile std::sig_atomic_t g_running = 1;

struct SpeedStage
{
  double speed_mps;
  double duration_sec;
};

struct Config
{
  std::string network_interface;
  std::vector<SpeedStage> stages{
      {kDefaultFastSpeedMps, kDefaultFastDurationSec},
      {kDefaultMiddleSpeedMps, kDefaultMiddleDurationSec},
      {kDefaultSlowSpeedMps, kDefaultSlowDurationSec},
      {kDefaultCreepSpeedMps, kDefaultCreepDurationSec},
  };
  bool custom_stages{false};
  bool execute{false};
};

void SignalHandler(int)
{
  // 信号处理器不能调用 SDK；主流程会看到标志并发送最终 StopMove。
  g_running = 0;
}

double ParseFinite(const std::string& raw, const std::string& name)
{
  std::size_t consumed = 0;
  const double value = std::stod(raw, &consumed);
  if (consumed != raw.size() || !std::isfinite(value)) {
    throw std::runtime_error(name + " must be a finite number");
  }
  return value;
}

SpeedStage ParseStage(const std::string& raw)
{
  const std::size_t separator = raw.find(':');
  if (separator == std::string::npos || separator == 0 ||
      separator == raw.size() - 1 || raw.find(':', separator + 1) != std::string::npos) {
    throw std::runtime_error("--stage must use SPEED_MPS:DURATION_SEC");
  }
  return {
      ParseFinite(raw.substr(0, separator), "stage speed"),
      ParseFinite(raw.substr(separator + 1), "stage duration"),
  };
}

void PrintUsage(const char* program)
{
  std::cout
      << "Usage:\n  " << program
      << " <network_interface> [--stage SPEED_MPS:DURATION_SEC]... [--execute]\n\n"
      << "Without --stage the default flat-ground profile is:\n"
      << "  0.30m/s x 1.00s -> 0.22m/s x 0.40s -> 0.14m/s x 0.40s"
      << " -> 0.06m/s x 0.40s -> StopMove\n\n"
      << "Repeat --stage to set every level yourself, for example:\n"
      << "  --stage 0.35:1.20 --stage 0.25:0.50 --stage 0.12:0.50 --stage 0.05:0.30\n"
      << "Safety limits: 0 < speed <= 0.40m/s, 0 < duration <= 10s,"
      << " and every next speed must not increase.\n";
}

void ValidateStages(const std::vector<SpeedStage>& stages)
{
  if (stages.empty()) {
    throw std::runtime_error("at least one --stage is required");
  }
  double previous_speed = kMaximumSpeedMps;
  for (const SpeedStage& stage : stages) {
    if (stage.speed_mps <= 0.0 || stage.speed_mps > kMaximumSpeedMps ||
        stage.duration_sec <= 0.0 || stage.duration_sec > kMaximumStageDurationSec) {
      throw std::runtime_error("stage exceeds flat-ground safety limits");
    }
    // 分级减速测试拒绝后续加速；速度相等允许用来延长某一稳定档。
    if (stage.speed_mps > previous_speed + 1e-9) {
      throw std::runtime_error("each stage speed must not increase");
    }
    previous_speed = stage.speed_mps;
  }
}

Config ParseArguments(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    throw std::runtime_error("network_interface is required");
  }
  Config config;
  config.network_interface = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--execute") {
      config.execute = true;
      continue;
    }
    if (option == "--help" || option == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    if (option != "--stage" || ++index >= argc) {
      throw std::runtime_error("expected --stage SPEED_MPS:DURATION_SEC");
    }
    if (!config.custom_stages) {
      config.stages.clear();
      config.custom_stages = true;
    }
    config.stages.push_back(ParseStage(argv[index]));
  }
  if (config.network_interface.empty()) {
    throw std::runtime_error("network_interface must not be empty");
  }
  ValidateStages(config.stages);
  return config;
}

bool SendStage(unitree::robot::go2::SportClient& client,
               const SpeedStage& stage, std::size_t index)
{
  std::cout << "stage=" << index + 1
            << " vx_mps=" << stage.speed_mps
            << " duration_sec=" << stage.duration_sec << std::endl;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(stage.duration_sec);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    const int32_t result = client.Move(static_cast<float>(stage.speed_mps), 0.0F, 0.0F);
    if (result != 0) {
      std::cerr << "stage=" << index + 1 << " move_failed=" << result << std::endl;
      return false;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(1.0 / kCommandRateHz));
  }
  return g_running != 0;
}

int Run(const Config& config)
{
  std::cout << std::fixed << std::setprecision(3)
            << "FLAT_DECELERATION_TEST interface=" << config.network_interface
            << " command_rate_hz=" << kCommandRateHz
            << " gait_switch=false"
            << " execute=" << (config.execute ? "true" : "false") << std::endl;
  for (std::size_t index = 0; index < config.stages.size(); ++index) {
    std::cout << "planned_stage=" << index + 1
              << " vx_mps=" << config.stages[index].speed_mps
              << " duration_sec=" << config.stages[index].duration_sec << std::endl;
  }
  if (!config.execute) {
    // Dry Run 不建立 DDS 连接，适合先复核时间和逐级速度是否符合现场计划。
    std::cout << "DRY_RUN: no SDK communication and no robot motion." << std::endl;
    return 0;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, config.network_interface);
  unitree::robot::go2::SportClient client;
  client.SetTimeout(10.0F);
  client.Init();

  bool succeeded = true;
  for (std::size_t index = 0; succeeded && index < config.stages.size(); ++index) {
    succeeded = SendStage(client, config.stages[index], index);
  }
  // 无论完成、中断或某档 Move 失败，都只发送一次最终停车，避免残余直行速度。
  const int32_t stop_result = client.StopMove();
  std::cout << "stage=final_stop_move result=" << stop_result << std::endl;
  return (succeeded && g_running && stop_result == 0) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return Run(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
