#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

namespace
{

double ParseNonnegativeDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (value < 0.0) {
    throw std::runtime_error(name + " must be nonnegative");
  }
  return value;
}

void SleepSec(double seconds)
{
  if (seconds <= 0.0) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

int32_t BlinkFrontLight(int count, double on_sec, double off_sec,
                        int high_level, int low_level)
{
  unitree::robot::go2::VuiClient vui_client;
  vui_client.SetTimeout(2.0F);
  vui_client.Init();

  int32_t result = 0;
  for (int index = 0; index < count; ++index) {
    const int32_t on_result = vui_client.SetBrightness(high_level);
    if (on_result != 0) {
      result = on_result;
    }
    SleepSec(on_sec);

    const int32_t off_result = vui_client.SetBrightness(low_level);
    if (off_result != 0) {
      result = off_result;
    }
    SleepSec(off_sec);
  }
  return result;
}

int32_t RunAction(unitree::robot::go2::SportClient& sport_client,
                  const std::string& action)
{
  if (action == "balance_stand") {
    return sport_client.BalanceStand();
  }
  if (action == "classic_walk" || action == "classic_walk_on") {
    return sport_client.ClassicWalk(true);
  }
  if (action == "classic_walk_off") {
    return sport_client.ClassicWalk(false);
  }
  if (action == "static_walk") {
    return sport_client.StaticWalk();
  }
  if (action == "trot_run") {
    return sport_client.TrotRun();
  }
  if (action == "free_walk") {
    return sport_client.FreeWalk();
  }
  if (action == "stand_up") {
    return sport_client.StandUp();
  }
  if (action == "economic_gait") {
    return sport_client.EconomicGait();
  }
  if (action == "front_jump") {
    return sport_client.FrontJump();
  }
  if (action == "hello" || action == "wave") {
    return sport_client.Hello();
  }
  if (action == "stretch") {
    return sport_client.Stretch();
  }
  if (action == "blink_front_light_3" || action == "blink_light_3") {
    return BlinkFrontLight(3, 0.35, 0.35, 10, 0);
  }
  if (action == "recovery_stand") {
    return sport_client.RecoveryStand();
  }
  if (action == "stop_move") {
    return sport_client.StopMove();
  }

  throw std::runtime_error("unsupported action: " + action);
}

bool RequiresClassicWalkHandback(const std::string& action)
{
  // 这些入口都会暂时脱离正式巡线步态。helper 是其既有 SportClient owner，
  // 因此在同一进程内完成 handback，避免新增并发 SDK client。
  return action == "balance_stand" || action == "static_walk" ||
         action == "trot_run" || action == "free_walk" ||
         action == "stand_up" || action == "economic_gait" ||
         action == "front_jump" || action == "hello" || action == "wave" ||
         action == "stretch" || action == "recovery_stand" ||
         action == "blink_front_light_3" || action == "blink_light_3";
}

int32_t RestoreClassicWalkForLineFollow(
    unitree::robot::go2::SportClient& sport_client)
{
  // 交接顺序是安全合同的一部分：停止特殊动作残留，再切经典步态，最后
  // StopMove 确认不带入速度。任一失败令 helper 非零退出，上游保持 gait lock。
  int32_t result = sport_client.StopMove();
  std::cout << "ClassicWalk handback StopMove result: " << result << std::endl;
  if (result != 0) {
    return result;
  }
  result = sport_client.ClassicWalk(true);
  std::cout << "ClassicWalk handback enable result: " << result << std::endl;
  if (result != 0) {
    return result;
  }
  result = sport_client.StopMove();
  std::cout << "ClassicWalk handback confirm StopMove result: " << result
            << std::endl;
  return result;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> <action> [wait_sec]\n\n"
      << "Actions:\n"
      << "  stand_up | balance_stand | classic_walk | classic_walk_on | "
         "classic_walk_off | static_walk | trot_run | free_walk | "
         "economic_gait | front_jump | stretch | hello | wave | "
         "blink_front_light_3 | recovery_stand | stop_move\n\n"
      << "Example:\n"
      << "  " << program << " eth0 front_jump 2.5\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    const std::string action = argv[2];
    const double wait_sec =
        argc >= 4 ? ParseNonnegativeDouble(argv[3], "wait_sec") : 0.0;

    std::cout << "Initializing Unitree SDK2 on interface "
              << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0F);
    sport_client.Init();

    std::cout << "Running SDK action: " << action << std::endl;
    int32_t result = RunAction(sport_client, action);
    std::cout << "SDK action result: " << result << std::endl;
    const bool requires_handback = RequiresClassicWalkHandback(action);
    bool classic_walk_handback_failed = false;
    if (result == 0 && requires_handback) {
      result = RestoreClassicWalkForLineFollow(sport_client);
      classic_walk_handback_failed = result != 0;
    }
    SleepSec(wait_sec);

    // 42 是本 helper 的受限内部合同：调用者据此保持 gait lock，不能把
    // ClassicWalk handback 失败误当成普通特殊动作失败后继续巡线。
    return result == 0 ? 0 : (classic_walk_handback_failed ? 42 : 1);
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
