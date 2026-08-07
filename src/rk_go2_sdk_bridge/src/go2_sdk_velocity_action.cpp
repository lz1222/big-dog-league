#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

constexpr double kDefaultRateHz = 20.0;
constexpr double kDefaultStopSec = 0.10;
constexpr const char* kDefaultStopMode = "move_zero";

double ParseFiniteDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value)) {
    throw std::runtime_error(name + " must be finite");
  }
  return value;
}

double ParsePositiveDouble(const char* raw, const std::string& name)
{
  const double value = ParseFiniteDouble(raw, name);
  if (value <= 0.0) {
    throw std::runtime_error(name + " must be positive");
  }
  return value;
}

double ParseNonnegativeDouble(const char* raw, const std::string& name)
{
  const double value = ParseFiniteDouble(raw, name);
  if (value < 0.0) {
    throw std::runtime_error(name + " must be nonnegative");
  }
  return value;
}

void SleepForRate(double rate_hz)
{
  const auto sleep_time = std::chrono::duration<double>(1.0 / rate_hz);
  std::this_thread::sleep_for(sleep_time);
}

int32_t SendMoveForDuration(unitree::robot::go2::SportClient& sport_client,
                            double vx, double vyaw, double duration_sec,
                            double rate_hz, const std::string& label)
{
  if (duration_sec <= 0.0) {
    return 0;
  }

  std::cout << label << ": Move vx=" << vx << " vyaw=" << vyaw
            << " for " << duration_sec << "s" << std::endl;

  int32_t result = 0;
  const auto end_time = std::chrono::steady_clock::now()
      + std::chrono::duration<double>(duration_sec);
  while (std::chrono::steady_clock::now() < end_time) {
    const int32_t call_result = sport_client.Move(
        static_cast<float>(vx), 0.0F, static_cast<float>(vyaw));
    if (call_result != 0) {
      result = call_result;
    }
    SleepForRate(rate_hz);
  }
  return result;
}

int32_t SendStop(unitree::robot::go2::SportClient& sport_client,
                 const std::string& stop_mode, double stop_sec,
                 double rate_hz)
{
  if (stop_mode == "none" || stop_sec <= 0.0) {
    return 0;
  }

  if (stop_mode == "move_zero") {
    return SendMoveForDuration(
        sport_client, 0.0, 0.0, stop_sec, rate_hz, "Stop");
  }

  if (stop_mode == "stop_move") {
    std::cout << "Stop: StopMove for " << stop_sec << "s" << std::endl;
    int32_t result = 0;
    const auto end_time = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(stop_sec);
    while (std::chrono::steady_clock::now() < end_time) {
      const int32_t call_result = sport_client.StopMove();
      if (call_result != 0) {
        result = call_result;
      }
      SleepForRate(rate_hz);
    }
    return result;
  }

  throw std::runtime_error(
      "stop_mode must be one of: move_zero, stop_move, none");
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> <linear_x> <angular_z> <duration_sec> "
         "[rate_hz] [stop_mode] [stop_sec]\n\n"
      << "stop_mode:\n"
      << "  move_zero | stop_move | none\n\n"
      << "Examples:\n"
      << "  " << program << " eth0 0.30 0.00 1.0\n"
      << "  " << program << " eth0 0.00 0.80 1.0 20 move_zero 0.10\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 5) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    const double linear_x = ParseFiniteDouble(argv[2], "linear_x");
    const double angular_z = ParseFiniteDouble(argv[3], "angular_z");
    const double duration_sec =
        ParseNonnegativeDouble(argv[4], "duration_sec");
    const double rate_hz =
        argc >= 6 ? ParsePositiveDouble(argv[5], "rate_hz")
                  : kDefaultRateHz;
    const std::string stop_mode =
        argc >= 7 ? std::string(argv[6]) : std::string(kDefaultStopMode);
    const double stop_sec =
        argc >= 8 ? ParseNonnegativeDouble(argv[7], "stop_sec")
                  : kDefaultStopSec;

    std::cout << "Initializing Unitree SDK2 on interface "
              << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0F);
    sport_client.Init();

    const int32_t move_result = SendMoveForDuration(
        sport_client, linear_x, angular_z, duration_sec, rate_hz, "Command");
    const int32_t stop_result = SendStop(
        sport_client, stop_mode, stop_sec, rate_hz);

    if (move_result != 0) {
      std::cerr << "Move returned " << move_result << std::endl;
    }
    if (stop_result != 0) {
      std::cerr << "Stop returned " << stop_result << std::endl;
    }
    return move_result == 0 && stop_result == 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
