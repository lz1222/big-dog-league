// Go2 SDK 全局步态架空闭环测试器。
//
// 本工具只用于四足完全架空时验证 ClassicWalk / FreeWalk / Move / StopMove。
// 默认仅打印计划；必须显式传入 --execute-off-ground 才创建 SportClient。测试
// 全程检查足端载荷，任一足疑似接地即中止，并由清理守卫发送 ZERO 和 StopMove。
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
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

constexpr const char* kSportModeStateTopic = "rt/sportmodestate";
constexpr double kRateHz = 20.0;
constexpr double kVxMps = 0.10;
constexpr double kYawRadps = 0.15;
constexpr double kLinearDurationSec = 0.80;
constexpr double kYawDurationSec = 0.60;
constexpr double kSettleSec = 0.50;
constexpr double kInterActionStopSec = 1.0;
// 架空时本机实测为 0；留少量传感器噪声余量，但不允许承重状态进入动作。
constexpr int kMaxOffGroundFootForce = 5;

volatile std::sig_atomic_t g_running = 1;

struct Config
{
  std::string interface;
  std::string stage{"full"};
  int cycles{3};
  bool execute{false};
};

void SignalHandler(int)
{
  // SDK 调用不具备异步信号安全性；主循环观察标志后走统一清理路径。
  g_running = 0;
}

void Sleep(double seconds)
{
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

class OffGroundObserver
{
public:
  OffGroundObserver()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(&OffGroundObserver::OnState, this, std::placeholders::_1), 1);
  }

  bool WaitForOffGround(double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_sec);
    while (g_running && std::chrono::steady_clock::now() < deadline) {
      if (HasFreshOffGroundSample()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool HasFreshOffGroundSample() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) return false;
    if (std::chrono::steady_clock::now() - received_at_ >
        std::chrono::milliseconds(200)) {
      return false;
    }
    for (const int force : state_.foot_force()) {
      if (force > kMaxOffGroundFootForce) return false;
    }
    return true;
  }

  void LogSnapshot(const std::string& label) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "GAIT_MATRIX event=STATE label=" << label
              << " received=" << (received_ ? "true" : "false")
              << " error_code=" << state_.error_code()
              << " mode=" << static_cast<unsigned int>(state_.mode())
              << " gait_type=" << static_cast<unsigned int>(state_.gait_type())
              << " body_height=" << state_.body_height()
              << " foot_force=[" << state_.foot_force()[0] << ','
              << state_.foot_force()[1] << ',' << state_.foot_force()[2] << ','
              << state_.foot_force()[3] << "]" << std::endl;
  }

private:
  void OnState(const void* message)
  {
    if (message == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    received_at_ = std::chrono::steady_clock::now();
    received_ = true;
  }

  mutable std::mutex mutex_;
  bool received_{false};
  std::chrono::steady_clock::time_point received_at_{};
  unitree_go::msg::dds_::SportModeState_ state_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      subscriber_;
};

class FinalStopGuard
{
public:
  explicit FinalStopGuard(unitree::robot::go2::SportClient& client)
  : client_(client) {}

  ~FinalStopGuard()
  {
    if (finalized_) return;
    Finalize();
  }

  bool Finalize()
  {
    // 两种零命令都发送：Move ZERO 清目标，StopMove 终止 SDK 连续运动。
    const int32_t zero_ret = client_.Move(0.0F, 0.0F, 0.0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const int32_t stop_ret = client_.StopMove();
    std::cout << "GAIT_MATRIX event=FINAL_ZERO move_zero_ret=" << zero_ret
              << " stop_move_ret=" << stop_ret << std::endl;
    finalized_ = true;
    return zero_ret == 0 && stop_ret == 0;
  }

private:
  unitree::robot::go2::SportClient& client_;
  bool finalized_{false};
};

int32_t LogCall(const std::string& name, const std::function<int32_t()>& call)
{
  const auto started = std::chrono::steady_clock::now();
  const int32_t ret = call();
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  std::cout << std::fixed << std::setprecision(6)
            << "GAIT_MATRIX event=SDK_CALL name=" << name
            << " ret=" << ret << " elapsed_sec=" << elapsed << std::endl;
  return ret;
}

void RequireSuccess(int32_t ret, const std::string& name)
{
  if (ret != 0) {
    throw std::runtime_error(name + " failed ret=" + std::to_string(ret));
  }
}

void RequireOffGround(const OffGroundObserver& observer,
                      const std::string& phase)
{
  if (!observer.HasFreshOffGroundSample()) {
    throw std::runtime_error("off_ground_guard_failed phase=" + phase);
  }
}

void StopAndSettle(unitree::robot::go2::SportClient& client,
                   const OffGroundObserver& observer,
                   const std::string& reason)
{
  RequireSuccess(LogCall(reason + ".move_zero", [&]() {
    return client.Move(0.0F, 0.0F, 0.0F);
  }), "MoveZero");
  RequireSuccess(LogCall(reason + ".stop_move", [&]() {
    return client.StopMove();
  }), "StopMove");
  Sleep(kInterActionStopSec);
  RequireOffGround(observer, reason + ".settled");
}

void EnsureClassic(unitree::robot::go2::SportClient& client,
                   const OffGroundObserver& observer,
                   const std::string& label)
{
  RequireOffGround(observer, label + ".before");
  RequireSuccess(LogCall(label + ".stop_move", [&]() {
    return client.StopMove();
  }), "StopMove");
  Sleep(0.30);
  RequireSuccess(LogCall(label + ".speed_level_1", [&]() {
    return client.SpeedLevel(1);
  }), "SpeedLevel(1)");
  RequireSuccess(LogCall(label + ".classic_walk", [&]() {
    return client.ClassicWalk(true);
  }), "ClassicWalk(true)");
  Sleep(kSettleSec);
  RequireOffGround(observer, label + ".after");
  RequireSuccess(LogCall(label + ".move_zero", [&]() {
    return client.Move(0.0F, 0.0F, 0.0F);
  }), "MoveZero");
}

void SendBoundedMove(unitree::robot::go2::SportClient& client,
                     const OffGroundObserver& observer,
                     float vx, float yaw, double duration_sec,
                     const std::string& label)
{
  constexpr double kFloatComparisonEpsilon = 1.0e-6;
  // 命令最终以 float 进入 SDK；只吸收 float 表示误差，安全上限本身不放宽。
  if (std::fabs(vx) > kVxMps + kFloatComparisonEpsilon ||
      std::fabs(yaw) > kYawRadps + kFloatComparisonEpsilon ||
      duration_sec > 1.0) {
    throw std::runtime_error("bounded move exceeds hard safety limit");
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    RequireOffGround(observer, label);
    RequireSuccess(LogCall(label, [&]() {
      return client.Move(vx, 0.0F, yaw);
    }), "Move");
    std::this_thread::sleep_for(
        std::chrono::duration<double>(1.0 / kRateHz));
  }
  if (!g_running) throw std::runtime_error("interrupted");
  StopAndSettle(client, observer, label + ".cleanup");
}

void RunMotionStages(unitree::robot::go2::SportClient& client,
                     const OffGroundObserver& observer)
{
  SendBoundedMove(client, observer, static_cast<float>(kVxMps), 0.0F,
                  kLinearDurationSec, "C2_FORWARD");
  SendBoundedMove(client, observer, 0.0F, static_cast<float>(kYawRadps),
                  kYawDurationSec, "C3_YAW_POSITIVE");
  SendBoundedMove(client, observer, 0.0F, -static_cast<float>(kYawRadps),
                  kYawDurationSec, "C4_YAW_NEGATIVE");
}

void RunGaitCycles(unitree::robot::go2::SportClient& client,
                   const OffGroundObserver& observer, int cycles)
{
  for (int cycle = 1; cycle <= cycles; ++cycle) {
    StopAndSettle(client, observer,
                  "C5_" + std::to_string(cycle) + ".classic_to_free.zero");
    RequireSuccess(LogCall("C5.classic_to_free.free_walk", [&]() {
      return client.FreeWalk();
    }), "FreeWalk");
    Sleep(kSettleSec);
    RequireOffGround(observer, "C5.free_ready");
    std::cout << "GAIT_MATRIX event=GAIT_READY target=FREE cycle=" << cycle
              << " verification_source=command_ack" << std::endl;

    StopAndSettle(client, observer,
                  "C5_" + std::to_string(cycle) + ".free_to_classic.zero");
    RequireSuccess(LogCall("C5.free_to_classic.classic_walk", [&]() {
      return client.ClassicWalk(true);
    }), "ClassicWalk(true)");
    Sleep(kSettleSec);
    RequireOffGround(observer, "C5.classic_ready");
    std::cout << "GAIT_MATRIX event=GAIT_READY target=CLASSIC cycle=" << cycle
              << " verification_source=command_ack" << std::endl;
  }
}

Config ParseArguments(int argc, char** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--interface" && index + 1 < argc) {
      config.interface = argv[++index];
    } else if (option == "--stage" && index + 1 < argc) {
      config.stage = argv[++index];
    } else if (option == "--cycles" && index + 1 < argc) {
      config.cycles = std::stoi(argv[++index]);
    } else if (option == "--execute-off-ground") {
      config.execute = true;
    } else {
      throw std::runtime_error(
          "Usage: go2_sdk_gait_matrix --interface IFACE "
          "[--stage c0|c1|motion|cycles|restart|full] [--cycles 1..3] "
          "[--execute-off-ground]");
    }
  }
  if (config.interface.empty()) throw std::runtime_error("interface is required");
  if (config.stage != "c0" && config.stage != "c1" &&
      config.stage != "motion" && config.stage != "cycles" &&
      config.stage != "restart" && config.stage != "full") {
    throw std::runtime_error("invalid stage");
  }
  if (config.cycles < 1 || config.cycles > 3) {
    throw std::runtime_error("cycles must be in range 1..3");
  }
  return config;
}

int Run(const Config& config)
{
  std::cout << "GAIT_MATRIX event=PLAN interface=" << config.interface
            << " stage=" << config.stage << " cycles=" << config.cycles
            << " execute=" << (config.execute ? "true" : "false")
            << " select_mode_count=0 stand_up_count=0 release_mode_count=0"
            << std::endl;
  if (!config.execute) return 0;

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, config.interface);
  OffGroundObserver observer;
  if (!observer.WaitForOffGround(3.0)) {
    throw std::runtime_error("off_ground_guard_failed_before_sdk_init");
  }
  observer.LogSnapshot("initial");

  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(2.0F);
  client.Init();
  FinalStopGuard final_stop(client);
  // SportClient::Init 只完成本地初始化；给 DDS responder 一个短而有界的发现窗口，
  // 避免把首次 RPC 的发现瞬态误判为 StopMove 能力故障。
  Sleep(0.50);
  if (config.stage == "c0") {
    std::cout << "GAIT_MATRIX event=STAGE_PASS stage=C0" << std::endl;
    return final_stop.Finalize() ? 0 : 1;
  }

  EnsureClassic(client, observer, "C1");
  std::cout << "GAIT_MATRIX event=STAGE_PASS stage=C1 "
            << "verification_source=command_ack" << std::endl;
  if (config.stage == "c1") return final_stop.Finalize() ? 0 : 1;

  if (config.stage == "motion" || config.stage == "full") {
    RunMotionStages(client, observer);
    std::cout << "GAIT_MATRIX event=STAGE_PASS stage=C2_C4" << std::endl;
  }
  if (config.stage == "restart") {
    SendBoundedMove(client, observer, static_cast<float>(kVxMps), 0.0F,
                    0.50, "RESTART_SHORT_MOVE");
    std::cout << "GAIT_MATRIX event=STAGE_PASS stage=RESTART" << std::endl;
  }
  if (config.stage == "cycles" || config.stage == "full") {
    RunGaitCycles(client, observer, config.cycles);
    std::cout << "GAIT_MATRIX event=STAGE_PASS stage=C5 cycles="
              << config.cycles << std::endl;
  }
  observer.LogSnapshot("final");
  return final_stop.Finalize() ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return Run(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "GAIT_MATRIX event=FAIL reason=" << std::quoted(error.what())
              << std::endl;
    return 1;
  }
}
