// T 型楼梯入口确认后的经典步态三段动作工具。
//
// 动作顺序和默认标定值来自 final_provincial_reference.py 的 Phase 3：
// 上台阶前进 0.55 m/s × 3.9 s → 左转 79°（1.0 rad/s）→ 下台阶前进
// 0.55 m/s × 2.9 s。本工具保留该运动流程，但按当前任务要求始终使用既有
// 经典步态：不调用 FreeWalk、ClassicWalk 或 SwitchGait。默认 Dry Run；只有
// --execute 才会向真机发送速度命令。

#include <algorithm>
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

// ===== 楼梯运动参数总表：修改速度、时间或转弯只改这里 =====
// 识别 T 型标记前：每段前进后由 Python 停车、重新取图，防止持续盲走。
constexpr double kDefaultApproachSpeedMps = 0.25;      // 接近速度（m/s）
constexpr double kDefaultApproachDurationSec = 1.6;    // 单次接近时间（s）
// 识别 T 型标记后：复现参考程序 Phase 3 的上台阶、左转、下台阶参数。
constexpr double kDefaultUpSpeedMps = 0.55;            // 上台阶速度（m/s）
constexpr double kDefaultUpDurationSec = 3.9;          // 上台阶时间（s）
constexpr double kDefaultTurnAngleDeg = 79.0;          // 左转目标角（度）
constexpr double kDefaultTurnWzRadps = 1.0;            // 左转角速度（rad/s）
constexpr double kDefaultDownSpeedMps = 0.55;          // 下台阶速度（m/s）
constexpr double kDefaultDownDurationSec = 2.9;        // 下台阶时间（s）

// 安全边界允许复现旧参数但防止输入意外变成无限时长或极端速度。
constexpr double kMaximumForwardSpeedMps = 0.60;
constexpr double kMaximumTurnSpeedRadps = 1.20;
constexpr double kMaximumSegmentDurationSec = 5.0;
constexpr double kMaximumTurnAngleDeg = 90.0;
constexpr double kCommandRateHz = 20.0;                // 每秒发送 Move 命令次数
constexpr double kStateFreshnessSec = 0.50;
constexpr double kYawWaitTimeoutSec = 3.0;
constexpr double kInterStageStopSec = 0.40;             // 三段动作之间停车稳定时间（s）
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kSportModeStateTopic = "rt/sportmodestate";

volatile std::sig_atomic_t g_running = 1;

struct StairMotionConfig
{
  std::string network_interface;
  // 以下六个值由 Python 在 T 型标记确认后传入，对应旧 Phase 3 的三段动作参数。
  double up_speed_mps{kDefaultUpSpeedMps};
  double up_duration_sec{kDefaultUpDurationSec};
  double turn_angle_deg{kDefaultTurnAngleDeg};
  double turn_wz_radps{kDefaultTurnWzRadps};
  double down_speed_mps{kDefaultDownSpeedMps};
  double down_duration_sec{kDefaultDownDurationSec};
  // 每次 StopMove 后等待该时长，给速度清零与机身稳定留下明确边界。
  double inter_stage_stop_sec{kInterStageStopSec};
  // 接近阶段只在相机尚未确认 T 型标记时使用，完成后会立即 StopMove 并重新识别。
  double approach_speed_mps{kDefaultApproachSpeedMps};
  double approach_duration_sec{kDefaultApproachDurationSec};
  bool approach_only{false};
  bool execute{false};
};

void SignalHandler(int)
{
  // 信号处理器不能调用 SDK；主流程发现该标志后统一 StopMove。
  g_running = 0;
}

double ParseFiniteDouble(const std::string& raw, const std::string& name)
{
  std::size_t consumed = 0;
  const double value = std::stod(raw, &consumed);
  if (consumed != raw.size() || !std::isfinite(value)) {
    throw std::runtime_error(name + " must be finite");
  }
  return value;
}

double NormalizeAngle(double angle_rad)
{
  // 将 IMU 航向角差收敛到 [-pi, pi]，避免跨越 ±180° 时误判转向已完成。
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n  " << program << " <network_interface> [options] [--execute]\n\n"
      << "T-marker Phase-3 defaults: up 0.55 m/s for 3.9 s, turn left "
      << "79 deg at 1.0 rad/s, down 0.55 m/s for 2.9 s.\n"
      << "Options:\n"
      << "  --stairs-up-speed-mps VALUE\n"
      << "  --stairs-up-duration-sec VALUE\n"
      << "  --stairs-turn-angle-deg VALUE\n"
      << "  --stairs-turn-wz-radps VALUE\n"
      << "  --stairs-down-speed-mps VALUE\n"
      << "  --stairs-down-duration-sec VALUE\n"
      << "  --inter-stage-stop-sec VALUE\n"
      << "  --approach-only --approach-speed-mps VALUE"
      << " --approach-duration-sec VALUE\n";
}

StairMotionConfig ParseArguments(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    throw std::runtime_error("network_interface is required");
  }
  StairMotionConfig config;
  config.network_interface = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--execute") {
      config.execute = true;
      continue;
    }
    if (option == "--approach-only") {
      config.approach_only = true;
      continue;
    }
    if (option == "--help" || option == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    if (option != "--stairs-up-speed-mps" &&
        option != "--stairs-up-duration-sec" &&
        option != "--stairs-turn-angle-deg" &&
        option != "--stairs-turn-wz-radps" &&
        option != "--stairs-down-speed-mps" &&
        option != "--stairs-down-duration-sec" &&
        option != "--inter-stage-stop-sec" &&
        option != "--approach-speed-mps" &&
        option != "--approach-duration-sec") {
      throw std::runtime_error("unknown option: " + option);
    }
    if (++index >= argc) {
      throw std::runtime_error("missing value for " + option);
    }
    const double value = ParseFiniteDouble(argv[index], option);
    if (option == "--stairs-up-speed-mps") {
      config.up_speed_mps = value;
    } else if (option == "--stairs-up-duration-sec") {
      config.up_duration_sec = value;
    } else if (option == "--stairs-turn-angle-deg") {
      config.turn_angle_deg = value;
    } else if (option == "--stairs-turn-wz-radps") {
      config.turn_wz_radps = value;
    } else if (option == "--stairs-down-speed-mps") {
      config.down_speed_mps = value;
    } else if (option == "--stairs-down-duration-sec") {
      config.down_duration_sec = value;
    } else if (option == "--inter-stage-stop-sec") {
      config.inter_stage_stop_sec = value;
    } else if (option == "--approach-speed-mps") {
      config.approach_speed_mps = value;
    } else {
      config.approach_duration_sec = value;
    }
  }
  if (config.network_interface.empty()) {
    throw std::runtime_error("network_interface must not be empty");
  }
  if (config.approach_only) {
    if (config.approach_speed_mps <= 0.0 ||
        config.approach_speed_mps > kMaximumForwardSpeedMps ||
        config.approach_duration_sec <= 0.0 ||
        config.approach_duration_sec > kMaximumSegmentDurationSec) {
      throw std::runtime_error("approach parameters exceed allowed limits");
    }
    return config;
  }
  // 前进段禁止倒车；左转使用正角速度，维持旧 Phase 3 的明确运动方向。
  if (config.up_speed_mps < 0.0 ||
      config.up_speed_mps > kMaximumForwardSpeedMps ||
      config.down_speed_mps < 0.0 ||
      config.down_speed_mps > kMaximumForwardSpeedMps ||
      config.up_duration_sec <= 0.0 ||
      config.up_duration_sec > kMaximumSegmentDurationSec ||
      config.down_duration_sec <= 0.0 ||
      config.down_duration_sec > kMaximumSegmentDurationSec ||
      config.turn_angle_deg <= 0.0 ||
      config.turn_angle_deg > kMaximumTurnAngleDeg ||
      config.turn_wz_radps <= 0.0 ||
      config.turn_wz_radps > kMaximumTurnSpeedRadps ||
      config.inter_stage_stop_sec < 0.0 ||
      config.inter_stage_stop_sec > 2.0) {
    throw std::runtime_error("stair Phase-3 parameters exceed allowed limits");
  }
  return config;
}

class YawStateMonitor
{
public:
  YawStateMonitor()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(&YawStateMonitor::OnState, this, std::placeholders::_1), 1);
  }

  bool WaitForFreshYaw(double timeout_sec, double& yaw_rad) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_sec);
    while (g_running && std::chrono::steady_clock::now() < deadline) {
      if (SnapshotFreshYaw(yaw_rad)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool SnapshotFreshYaw(double& yaw_rad) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || std::chrono::duration<double>(
            std::chrono::steady_clock::now() - received_at_).count() >
            kStateFreshnessSec) {
      return false;
    }
    yaw_rad = yaw_rad_;
    return true;
  }

private:
  void OnState(const void* message)
  {
    if (message == nullptr) {
      return;
    }
    const auto& state = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
        message);
    const double yaw_rad = state.imu_state().rpy()[2];
    if (!std::isfinite(yaw_rad)) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    yaw_rad_ = yaw_rad;
    received_at_ = std::chrono::steady_clock::now();
    received_ = true;
  }

  mutable std::mutex mutex_;
  bool received_{false};
  double yaw_rad_{0.0};
  std::chrono::steady_clock::time_point received_at_{};
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::SportModeState_> subscriber_;
};

bool SendTimedVelocity(unitree::robot::go2::SportClient& client,
                       double vx_mps, double wz_radps, double duration_sec,
                       const char* stage_name)
{
  std::cout << "stage=" << stage_name << " vx_mps=" << vx_mps
            << " wz_radps=" << wz_radps << " duration_sec=" << duration_sec
            << std::endl;
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    const int32_t result = client.Move(
        static_cast<float>(vx_mps), 0.0F, static_cast<float>(wz_radps));
    if (result != 0) {
      std::cerr << "stage=" << stage_name << " move_failed=" << result
                << std::endl;
      return false;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(1.0 / kCommandRateHz));
  }
  return g_running != 0;
}

bool StopBetweenStages(unitree::robot::go2::SportClient& client,
                       double stop_duration_sec)
{
  // 与旧 Phase 3 一致，每一段完成后清零并等待标定时长，避免速度指令跨阶段残留。
  const int32_t stop_result = client.StopMove();
  std::cout << "stage=stop_move result=" << stop_result << std::endl;
  if (stop_result != 0) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(stop_duration_sec));
  return g_running != 0;
}

bool TurnByMeasuredYaw(unitree::robot::go2::SportClient& client,
                       const YawStateMonitor& monitor,
                       const StairMotionConfig& config)
{
  double start_yaw_rad = 0.0;
  if (!monitor.WaitForFreshYaw(kYawWaitTimeoutSec, start_yaw_rad)) {
    std::cerr << "stage=turn_left no_fresh_yaw_before_motion" << std::endl;
    return false;
  }
  const double target_rad = config.turn_angle_deg * kPi / 180.0;
  // 超时仅作 DDS 或 IMU 异常时的失效保护；正常完成标准始终是实际 yaw 角度。
  const double timeout_sec = std::max(
      5.0, 2.0 * target_rad / config.turn_wz_radps + 1.0);
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
  std::cout << "stage=turn_left target_deg=" << config.turn_angle_deg
            << " wz_radps=" << config.turn_wz_radps << std::endl;
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    double current_yaw_rad = 0.0;
    if (!monitor.SnapshotFreshYaw(current_yaw_rad)) {
      std::cerr << "stage=turn_left yaw_stream_stale" << std::endl;
      return false;
    }
    const double turned_rad = std::abs(
        NormalizeAngle(current_yaw_rad - start_yaw_rad));
    if (turned_rad >= target_rad) {
      std::cout << "stage=turn_left completed_deg="
                << turned_rad * 180.0 / kPi << std::endl;
      return true;
    }
    const int32_t result = client.Move(
        0.0F, 0.0F, static_cast<float>(config.turn_wz_radps));
    if (result != 0) {
      std::cerr << "stage=turn_left move_failed=" << result << std::endl;
      return false;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(1.0 / kCommandRateHz));
  }
  std::cerr << "stage=turn_left timeout_or_interrupted" << std::endl;
  return false;
}

int RunStairPhase3Action(const StairMotionConfig& config)
{
  std::cout << "STAIR_PHASE3_CLASSIC_ACTION interface=" << config.network_interface
            << " up_speed_mps=" << config.up_speed_mps
            << " up_duration_sec=" << config.up_duration_sec
            << " turn_angle_deg=" << config.turn_angle_deg
            << " turn_wz_radps=" << config.turn_wz_radps
            << " down_speed_mps=" << config.down_speed_mps
            << " down_duration_sec=" << config.down_duration_sec
            << " inter_stage_stop_sec=" << config.inter_stage_stop_sec
            << " command_rate_hz=" << kCommandRateHz
            << " gait_switch=false"
            << " execute=" << (config.execute ? "true" : "false") << std::endl;
  if (!config.execute) {
    // Dry Run 不初始化通信层；允许先核对完整 Phase 3 参数而不改变机器人状态。
    std::cout << "DRY_RUN: no SDK communication and no robot motion." << std::endl;
    return 0;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, config.network_interface);
  unitree::robot::go2::SportClient client;
  client.SetTimeout(10.0F);
  client.Init();

  if (config.approach_only) {
    // 相机尚未看到 T 型标记时只走一个短段；停车后由 Python 重新识别，避免盲走。
    const bool succeeded = SendTimedVelocity(
        client, config.approach_speed_mps, 0.0, config.approach_duration_sec,
        "approach_before_t_marker");
    const int32_t final_stop_result = client.StopMove();
    std::cout << "stage=approach_stop_move result=" << final_stop_result << std::endl;
    return (succeeded && g_running && final_stop_result == 0) ? 0 : 1;
  }

  YawStateMonitor monitor;

  // ===== T 型标记确认后的完整旧 Phase 3 动作顺序（不切换步态） =====
  bool succeeded = SendTimedVelocity(
      client, config.up_speed_mps, 0.0, config.up_duration_sec, "stairs_up");
  if (succeeded) {
    succeeded = StopBetweenStages(client, config.inter_stage_stop_sec);
  }
  if (succeeded) {
    succeeded = TurnByMeasuredYaw(client, monitor, config);
  }
  if (succeeded) {
    succeeded = StopBetweenStages(client, config.inter_stage_stop_sec);
  }
  if (succeeded) {
    succeeded = SendTimedVelocity(
        client, config.down_speed_mps, 0.0, config.down_duration_sec,
        "stairs_down");
  }

  // 无论哪一段失败、DDS 失联或收到 Ctrl-C，均发送最后一条停车命令。
  const int32_t final_stop_result = client.StopMove();
  std::cout << "stage=final_stop_move result=" << final_stop_result << std::endl;
  return (succeeded && g_running && final_stop_result == 0) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunStairPhase3Action(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
