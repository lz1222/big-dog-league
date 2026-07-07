#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

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

int32_t RunAction(unitree::robot::go2::SportClient& sport_client,
                  const std::string& action)
{
  if (action == "balance_stand") {
    return sport_client.BalanceStand();
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
  if (action == "recovery_stand") {
    return sport_client.RecoveryStand();
  }
  if (action == "stop_move") {
    return sport_client.StopMove();
  }

  throw std::runtime_error("unsupported action: " + action);
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> <action> [wait_sec]\n\n"
      << "Actions:\n"
      << "  stand_up | balance_stand | economic_gait | front_jump | "
         "recovery_stand | stop_move\n\n"
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
    const int32_t result = RunAction(sport_client, action);
    std::cout << "SDK action result: " << result << std::endl;
    SleepSec(wait_sec);

    return result == 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
