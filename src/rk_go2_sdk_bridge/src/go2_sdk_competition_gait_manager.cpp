// 国赛唯一经典步态 backend 的启动零平移入口。
//
// 该程序复现省赛 normal → StandUp(按需) → SpeedLevel(1) → ClassicWalk
// → StopMove 合同。默认 dry-run；只有显式 --execute-zero-translation 才会
// 调用 SDK。它不包含 ReleaseMode、ServiceSwitch、Move 或重试循环。
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

constexpr const char* kSportModeStateTopic = "rt/sportmodestate";
constexpr float kStandingBodyHeightMin = 0.20F;
constexpr std::uint32_t kClassicErrorCode = 2010U;

struct Config
{
  std::string interface;
  bool execute{false};
};

struct CallCounts
{
  int check_mode{0};
  int select_normal{0};
  int stand_up{0};
  int speed_level{0};
  int classic_walk{0};
  int stop_move{0};
};

Config ParseArguments(int argc, char** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--interface" && index + 1 < argc) {
      config.interface = argv[++index];
    } else if (option == "--execute-zero-translation") {
      config.execute = true;
    } else {
      throw std::runtime_error(
          "Usage: go2_sdk_competition_gait_manager --interface IFACE "
          "[--execute-zero-translation]");
    }
  }
  if (config.interface.empty()) {
    throw std::runtime_error("interface is required");
  }
  return config;
}

class SportStateObserver
{
public:
  SportStateObserver()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(&SportStateObserver::OnState, this, std::placeholders::_1), 1);
  }

  bool WaitForFirst(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
                               [this]() { return received_; });
  }

  bool IsStanding() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_ && state_.body_height() >= kStandingBodyHeightMin;
  }

  bool WaitForClassic(int required_samples, double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_sec);
    while (classic_samples_ < required_samples) {
      if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
    return classic_samples_ >= required_samples;
  }

private:
  static bool MatchesClassic(const unitree_go::msg::dds_::SportModeState_& state)
  {
    // 该谓词来自当前静止经典 A/B 采集；动态 A/B 完成前仍只作为启动最小门。
    return state.mode() == 0 && state.gait_type() == 0 &&
        state.progress() == 0.0F && state.error_code() == kClassicErrorCode &&
        state.foot_raise_height() == 0.0F;
  }

  void OnState(const void* message)
  {
    if (message == nullptr) return;
    const auto& received = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
        message);
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = received;
    received_ = true;
    classic_samples_ = MatchesClassic(received) ? classic_samples_ + 1 : 0;
    condition_.notify_all();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool received_{false};
  int classic_samples_{0};
  unitree_go::msg::dds_::SportModeState_ state_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      subscriber_;
};

int Fail(const std::string& reason, const CallCounts& counts)
{
  std::cout << "COMPETITION_GAIT_MANAGER event=RESULT state=FAULT reason="
            << std::quoted(reason) << " ordinary_move_allowed=false"
            << " check_mode_count=" << counts.check_mode
            << " select_normal_count=" << counts.select_normal
            << " stand_up_count=" << counts.stand_up
            << " speed_level_1_count=" << counts.speed_level
            << " classic_walk_true_count=" << counts.classic_walk
            << " stop_move_count=" << counts.stop_move
            << " move_count=0 release_mode_count=0"
            << " sport_mode_service_switch_count=0" << std::endl;
  return 1;
}

int Run(const Config& config)
{
  std::cout << "COMPETITION_GAIT_MANAGER event=START interface="
            << config.interface << " domain=0 execute="
            << (config.execute ? "true" : "false")
            << " release_mode_count=0 sport_mode_service_switch_count=0 move_count=0"
            << std::endl;
  if (!config.execute) {
    std::cout << "COMPETITION_GAIT_MANAGER event=DRY_RUN sequence="
              << "CheckMode,SelectMode(normal),StandUp(if_needed),SpeedLevel(1),"
              << "ClassicWalk(true),StopMove,VerifyClassic" << std::endl;
    return 0;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, config.interface);
  CallCounts counts;
  SportStateObserver observer;
  if (!observer.WaitForFirst(3.0)) {
    return Fail("sportmodestate_timeout_before_startup", counts);
  }

  unitree::robot::go2::SportClient sport(false);
  sport.SetTimeout(5.0F);
  sport.Init();
  bool stop_sent = false;
  auto StopOnce = [&]() {
    if (stop_sent) return int32_t{0};
    stop_sent = true;
    ++counts.stop_move;
    const int32_t ret = sport.StopMove();
    std::cout << "COMPETITION_GAIT_MANAGER event=STOP_MOVE count="
              << counts.stop_move << " ret=" << ret << std::endl;
    return ret;
  };

  unitree::robot::b2::MotionSwitcherClient switcher;
  switcher.SetTimeout(2.0F);
  switcher.Init();
  std::string form;
  std::string name;
  ++counts.check_mode;
  const int32_t check_ret = switcher.CheckMode(form, name);
  std::cout << "COMPETITION_GAIT_MANAGER event=CHECK_MODE ret=" << check_ret
            << " form=" << std::quoted(form) << " name=" << std::quoted(name)
            << std::endl;
  if (check_ret != 0 || form != "0") {
    return Fail("motion_switcher_check_failed", counts);
  }

  ++counts.select_normal;
  const int32_t select_ret = switcher.SelectMode("normal");
  std::cout << "COMPETITION_GAIT_MANAGER event=SELECT_NORMAL count=1 ret="
            << select_ret << std::endl;
  if (select_ret != 0) {
    StopOnce();
    return Fail("select_normal_failed_" + std::to_string(select_ret), counts);
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  if (!observer.IsStanding()) {
    ++counts.stand_up;
    const int32_t stand_ret = sport.StandUp();
    std::cout << "COMPETITION_GAIT_MANAGER event=STAND_UP needed=true ret="
              << stand_ret << std::endl;
    if (stand_ret != 0) {
      StopOnce();
      return Fail("stand_up_failed_" + std::to_string(stand_ret), counts);
    }
    std::this_thread::sleep_for(std::chrono::seconds(4));
  } else {
    std::cout << "COMPETITION_GAIT_MANAGER event=STAND_UP needed=false ret=SKIPPED"
              << std::endl;
  }

  ++counts.speed_level;
  const int32_t speed_ret = sport.SpeedLevel(1);
  std::cout << "COMPETITION_GAIT_MANAGER event=SPEED_LEVEL level=1 ret="
            << speed_ret << std::endl;
  if (speed_ret != 0) {
    StopOnce();
    return Fail("speed_level_failed_" + std::to_string(speed_ret), counts);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ++counts.classic_walk;
  const int32_t classic_ret = sport.ClassicWalk(true);
  std::cout << "COMPETITION_GAIT_MANAGER event=CLASSIC_WALK count=1 ret="
            << classic_ret << std::endl;
  if (classic_ret != 0) {
    StopOnce();
    return Fail("classic_walk_failed_" + std::to_string(classic_ret), counts);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const int32_t stop_ret = StopOnce();
  if (stop_ret != 0) {
    return Fail("stop_move_failed_" + std::to_string(stop_ret), counts);
  }
  if (!observer.WaitForClassic(5, 3.0)) {
    return Fail("classic_signature_not_verified", counts);
  }
  std::cout << "COMPETITION_GAIT_MANAGER event=RESULT state=CLASSIC "
            << "ordinary_move_allowed=true check_mode_count=" << counts.check_mode
            << " select_normal_count=" << counts.select_normal
            << " stand_up_count=" << counts.stand_up
            << " speed_level_1_count=" << counts.speed_level
            << " classic_walk_true_count=" << counts.classic_walk
            << " stop_move_count=" << counts.stop_move
            << " move_count=0 release_mode_count=0"
            << " sport_mode_service_switch_count=0" << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return Run(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "COMPETITION_GAIT_MANAGER event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
