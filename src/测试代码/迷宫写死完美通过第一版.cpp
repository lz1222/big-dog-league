//迷宫写死完美通过第一版
// 经典步态直行与左半圆弧验证工具：只用于独立验证，不接入迷宫状态机或 ROS 控制链。
//
// 默认不创建 SDK 客户端；只有操作者显式传入 --execute 才会调用真机。执行前
// 先订阅 SportModeState：实测 state code=2010 代表本机固件已处于经典步态，
// 此时跳过重复 ClassicWalk(true)，且结束后保留操作者原有步态。其他状态才
// 请求 ClassicWalk(true)，并在结束或中断时先 StopMove 再撤销本程序的步态切换。

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

constexpr double kClassicWalkVxMps = 0.25;
constexpr double kCommandRateHz = 20.0;
constexpr double kDefaultDurationSec = 8.1;
constexpr double kMaxDurationSec = 15.0;
constexpr double kStateWaitSec = 2.0;
// 左转圆弧参数由操作者实测后细调：当前 vx / wz = 0.25 / 0.694 ≈ 0.360 m，
// wz≈39.76°/s；以当前 4.52 s 计算，转角约 179.8°、弧长约 1.13 m。
constexpr double kLeftArcVxMps = 0.25;
constexpr double kLeftArcWzRadps = 0.694;
constexpr double kLeftArcTargetYawRad = 3.14159265358979323846;
constexpr double kLeftArcDurationSec = 4.52;
// 两个半圆弧之间以入弧前相同直行速度继续 2.1 秒，用于观察左弧出弯后的稳定性。
constexpr double kPostLeftArcStraightDurationSec = 2.1;
// 右弧沿用左弧的线速度、角速度绝对值和持续时间；仅将角速度取负改变转向。
// 这种镜像设置用于验证左、右转曲率一致，而非重新引入另一套未校准参数。
constexpr double kRightArcVxMps = kLeftArcVxMps;
constexpr double kRightArcWzRadps = -kLeftArcWzRadps;
constexpr double kRightArcDurationSec = kLeftArcDurationSec;
// 右弧结束后继续直行 2.3 秒，单独观察最终出弯后的航向和步态收敛。
constexpr double kPostRightArcStraightDurationSec = 2.3;
// 最终左弧采用用户确认的 A 方案：与前段左弧相同速度和角速度，持续 1.89 秒。
// 因 vx / wz = 0.25 / 0.694 ≈ 0.360 m，此段理论半径约为 36 cm，转角约 75.2°。
constexpr double kFinalLeftArcVxMps = kLeftArcVxMps;
constexpr double kFinalLeftArcWzRadps = kLeftArcWzRadps;
constexpr double kFinalLeftArcDurationSec = 1.89;
constexpr const char* kSportModeStateTopic = "rt/sportmodestate";
// 2010 是本机在 App 手动开启 ClassicWalk 后实测到的状态码，不把它外推为
// 所有 Unitree 固件的通用枚举；未知固件应先重新采样再决定是否使用本工具。
constexpr std::uint32_t kObservedClassicWalkStateCode = 2010;

volatile std::sig_atomic_t g_running = 1;

struct ProbeConfig
{
  std::string network_interface;
  double duration_sec{kDefaultDurationSec};
  // 启用左弧时，该时间就是圆弧前直行阶段的时长；0 表示立即入弧。
  double left_arc_start_sec{0.0};
  bool left_arc_enabled{false};
  bool execute{false};
};

class SportStateSnapshot
{
public:
  SportStateSnapshot()
  {
    subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(&SportStateSnapshot::OnState, this, std::placeholders::_1),
        1);
  }

  bool WaitForSnapshot(unitree_go::msg::dds_::SportModeState_& state,
                       double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      if (received_.load()) {
        std::lock_guard<std::mutex> lock(mutex_);
        state = state_;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

private:
  void OnState(const void* message)
  {
    if (message == nullptr) {
      return;
    }
    // 回调只复制最后一帧；步态选择由主线程在收到确定状态后一次性决定。
    const auto& received = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
        message);
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = received;
    received_.store(true);
  }

  std::mutex mutex_;
  std::atomic<bool> received_{false};
  unitree_go::msg::dds_::SportModeState_ state_;
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::SportModeState_> subscriber_;
};

void SignalHandler(int)
{
  // 信号处理器不调用 SDK；主线程随后按固定顺序停车和关闭经典步态。
  g_running = 0;
}

double ParseDuration(const std::string& raw)
{
  std::size_t consumed = 0;
  const double value = std::stod(raw, &consumed);
  if (consumed != raw.size() || !std::isfinite(value) || value <= 0.0 ||
      value > kMaxDurationSec) {
    throw std::runtime_error(
        "duration_sec must be in (0, " + std::to_string(kMaxDurationSec) + "]");
  }
  return value;
}

double ParseLeftArcStart(const std::string& raw)
{
  std::size_t consumed = 0;
  const double value = std::stod(raw, &consumed);
  if (consumed != raw.size() || !std::isfinite(value) || value < 0.0 ||
      value > kMaxDurationSec) {
    throw std::runtime_error(
        "left_arc_start_sec must be in [0, " +
        std::to_string(kMaxDurationSec) + "]");
  }
  return value;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage: " << program
      << " <network_interface> [--duration-sec SEC]"
      << " [--left-arc-start-sec SEC] [--execute]\n"
      << "\n"
      << "Default mode only prints the plan and never initializes the SDK.\n"
      << "Without --left-arc-start-sec, --duration-sec runs straight only.\n"
      << "With --left-arc-start-sec, it runs straight first, then left arc, "
      << "straight 2.1 s, mirrored right arc, straight 2 s, and final "
      << "left arc for 1.89 s.\n"
      << "Current arcs: vx=" << kLeftArcVxMps
      << " m/s, left wz=" << kLeftArcWzRadps
      << " rad/s, right wz=" << kRightArcWzRadps
      << " rad/s, duration=" << kLeftArcDurationSec << " s.\n"
      << "The straight/arc start time is capped at " << kMaxDurationSec
      << " seconds and the program always stops.\n";
}

ProbeConfig ParseArguments(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    std::exit(2);
  }

  ProbeConfig config;
  config.network_interface = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--execute") {
      config.execute = true;
    } else if (option == "--duration-sec") {
      if (++index >= argc) {
        throw std::runtime_error("missing value for --duration-sec");
      }
      config.duration_sec = ParseDuration(argv[index]);
    } else if (option == "--left-arc-start-sec") {
      if (++index >= argc) {
        throw std::runtime_error("missing value for --left-arc-start-sec");
      }
      config.left_arc_start_sec = ParseLeftArcStart(argv[index]);
      config.left_arc_enabled = true;
    } else if (option == "--help" || option == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }

  if (config.network_interface.empty()) {
    throw std::runtime_error("network_interface must not be empty");
  }
  return config;
}

void StopAndRestoreGait(unitree::robot::go2::SportClient& client,
                        const std::string& reason,
                        bool classic_walk_enabled_by_probe)
{
  // 先清除速度；仅撤销本程序主动开启的步态，不能关闭操作者事先设置的经典步态。
  const int32_t stop_result = client.StopMove();
  std::cout << "cleanup reason=" << reason
            << " stop_result=" << stop_result;
  if (classic_walk_enabled_by_probe) {
    const int32_t gait_result = client.ClassicWalk(false);
    std::cout << " classic_walk_off_result=" << gait_result;
  } else {
    std::cout << " classic_walk_off=skipped_preserve_operator_state";
  }
  std::cout << std::endl;
}

int32_t SendVelocityForDuration(unitree::robot::go2::SportClient& client,
                                double vx_mps,
                                double wz_radps,
                                double duration_sec,
                                const std::string& phase)
{
  // 连续发送同一速度，避免控制端因单包超时停车；信号到达或 SDK 报错时立即退出，
  // 统一由调用方 StopMove，保证直行和圆弧阶段共享同一安全收尾路径。
  std::cout << "phase=" << phase
            << " vx=" << vx_mps
            << " wz=" << wz_radps
            << " duration_sec=" << duration_sec << std::endl;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    const int32_t result = client.Move(
        static_cast<float>(vx_mps), 0.0F, static_cast<float>(wz_radps));
    if (result != 0) {
      return result;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(1.0 / kCommandRateHz));
  }
  return 0;
}

int RunProbe(const ProbeConfig& config)
{
  std::cout << "CLASSIC_WALK_PROBE interface=" << config.network_interface
            << " vx=" << kClassicWalkVxMps
            << " vyaw=0"
            << " duration_sec=" << config.duration_sec
            << " left_arc_enabled="
            << (config.left_arc_enabled ? "true" : "false")
            << " left_arc_start_sec=" << config.left_arc_start_sec
            << " execute=" << (config.execute ? "true" : "false")
            << std::endl;
  if (!config.execute) {
    // Dry Run 不初始化 ChannelFactory，保证打印参数不会改变机器人状态。
    std::cout << "DRY_RUN: add --execute to call the Unitree SDK." << std::endl;
    return 0;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, config.network_interface);

  SportStateSnapshot state_monitor;
  unitree_go::msg::dds_::SportModeState_ state;
  if (!state_monitor.WaitForSnapshot(state, kStateWaitSec)) {
    // 无状态帧时无法确认步态归属，拒绝把 Move 发到未知控制面。
    std::cerr << "No SportModeState received; refusing motion." << std::endl;
    return 1;
  }
  const bool classic_walk_already_active =
      state.error_code() == kObservedClassicWalkStateCode;
  std::cout << "sport_state mode=" << static_cast<int>(state.mode())
            << " gait_type=" << static_cast<int>(state.gait_type())
            << " error_code=" << state.error_code()
            << " classic_walk_already_active="
            << (classic_walk_already_active ? "true" : "false") << std::endl;

  unitree::robot::go2::SportClient client;
  client.SetTimeout(10.0F);
  client.Init();

  bool classic_walk_enabled_by_probe = false;
  if (!classic_walk_already_active) {
    const int32_t gait_result = client.ClassicWalk(true);
    std::cout << "classic_walk_on_result=" << gait_result << std::endl;
    if (gait_result != 0) {
      StopAndRestoreGait(client, "classic_walk_on_failed", false);
      return 1;
    }
    classic_walk_enabled_by_probe = true;
  } else {
    std::cout << "classic_walk_on=skipped_existing_operator_state" << std::endl;
  }

  int32_t move_result = 0;
  if (config.left_arc_enabled) {
    // --left-arc-start-sec 是前段直行结束与入弧的协调点；不使用 --duration-sec。
    move_result = SendVelocityForDuration(
        client, kClassicWalkVxMps, 0.0, config.left_arc_start_sec,
        "straight_before_left_arc");
    if (move_result == 0 && g_running) {
      move_result = SendVelocityForDuration(
          client, kLeftArcVxMps, kLeftArcWzRadps, kLeftArcDurationSec,
          "left_half_arc");
    }
    if (move_result == 0 && g_running) {
      // 左弧完整结束后再进入两弧之间的直行；失败或中断时直接进入统一停车流程。
      move_result = SendVelocityForDuration(
          client, kClassicWalkVxMps, 0.0, kPostLeftArcStraightDurationSec,
          "straight_between_arcs");
    }
    if (move_result == 0 && g_running) {
      // 右弧与左弧速度和时间一致、角速度反号，形成镜像 S 形轨迹。
      move_result = SendVelocityForDuration(
          client, kRightArcVxMps, kRightArcWzRadps, kRightArcDurationSec,
          "right_half_arc");
    }
    if (move_result == 0 && g_running) {
      // 最后直行只在右弧完整结束后执行，便于单独观察最终出弯稳定性。
      move_result = SendVelocityForDuration(
          client, kClassicWalkVxMps, 0.0, kPostRightArcStraightDurationSec,
          "straight_after_right_arc");
    }
    if (move_result == 0 && g_running) {
      // 最终左弧在右弧后的 2 秒直行结束后才开始，保持用户确认的 36 cm 半径方案。
      move_result = SendVelocityForDuration(
          client, kFinalLeftArcVxMps, kFinalLeftArcWzRadps,
          kFinalLeftArcDurationSec, "final_left_arc");
    }
  } else {
    // 保留既有直行验证路径，便于与此前的单段测试结果比较。
    move_result = SendVelocityForDuration(
        client, kClassicWalkVxMps, 0.0, config.duration_sec, "straight_only");
  }

  StopAndRestoreGait(
      client, g_running ? "duration_complete" : "signal",
      classic_walk_enabled_by_probe);
  return move_result == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunProbe(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
