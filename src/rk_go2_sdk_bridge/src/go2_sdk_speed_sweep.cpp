#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{

constexpr double kDefaultDurationSec = 2.0;
constexpr double kDefaultStopSec = 1.0;
constexpr double kDefaultRateHz = 20.0;

std::vector<double> ParseSpeeds(const std::string& csv)
{
  std::vector<double> speeds;
  std::stringstream stream(csv);
  std::string item;

  while (std::getline(stream, item, ',')) {
    if (item.empty()) {
      continue;
    }

    const double speed = std::stod(item);
    if (!std::isfinite(speed) || speed < 0.0) {
      throw std::runtime_error("speeds_csv must contain nonnegative numbers");
    }
    speeds.push_back(speed);
  }

  if (speeds.empty()) {
    throw std::runtime_error("speeds_csv must contain at least one speed");
  }

  return speeds;
}

double ParsePositiveDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be a positive number");
  }
  return value;
}

double ParseNonnegativeDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(name + " must be a nonnegative number");
  }
  return value;
}

void SleepForRate(double rate_hz)
{
  const auto sleep_time = std::chrono::duration<double>(1.0 / rate_hz);
  std::this_thread::sleep_for(sleep_time);
}

void SendStop(unitree::robot::go2::SportClient& sport_client, double stop_sec,
              double rate_hz, const std::string& label)
{
  std::cout << label << ": StopMove for " << stop_sec << "s" << std::endl;

  const auto end_time =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(stop_sec);
  while (std::chrono::steady_clock::now() < end_time) {
    sport_client.StopMove();
    SleepForRate(rate_hz);
  }
}

void SendMove(unitree::robot::go2::SportClient& sport_client,
              const std::string& mode, double signed_speed,
              double duration_sec, double rate_hz)
{
  float vx = 0.0F;
  float vyaw = 0.0F;

  if (mode == "linear") {
    vx = static_cast<float>(signed_speed);
    std::cout << "TEST Move vx=" << signed_speed << " m/s for "
              << duration_sec << "s" << std::endl;
  } else {
    vyaw = static_cast<float>(signed_speed);
    std::cout << "TEST Move vyaw=" << signed_speed << " rad/s for "
              << duration_sec << "s" << std::endl;
  }

  const auto end_time =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_sec);
  while (std::chrono::steady_clock::now() < end_time) {
    sport_client.Move(vx, 0.0F, vyaw);
    SleepForRate(rate_hz);
  }
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> [linear|angular] [speeds_csv] "
         "[duration_sec] [stop_sec] [rate_hz] [direction]\n\n"
      << "Examples:\n"
      << "  " << program << " eth0 linear 0.20,0.30,0.40,0.50 2.0 1.0\n"
      << "  " << program << " eth0 angular 0.30,0.50,0.80 2.0 1.0\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    const std::string mode = argc >= 3 ? argv[2] : "linear";
    const std::string speeds_csv =
        argc >= 4 ? argv[3] : "0.10,0.20,0.30,0.40,0.50";
    const double duration_sec =
        argc >= 5 ? ParsePositiveDouble(argv[4], "duration_sec")
                  : kDefaultDurationSec;
    const double stop_sec =
        argc >= 6 ? ParseNonnegativeDouble(argv[5], "stop_sec")
                  : kDefaultStopSec;
    const double rate_hz =
        argc >= 7 ? ParsePositiveDouble(argv[6], "rate_hz") : kDefaultRateHz;
    const double direction =
        argc >= 8 ? ParsePositiveDouble(argv[7], "direction") : 1.0;
    const double signed_direction = direction > 0.0 ? 1.0 : -1.0;

    if (mode != "linear" && mode != "angular") {
      throw std::runtime_error("mode must be linear or angular");
    }

    const std::vector<double> speeds = ParseSpeeds(speeds_csv);

    std::cout << "Initializing Unitree SDK2 on interface "
              << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0F);
    sport_client.Init();

    std::cout << "SDK2 speed sweep started. Make sure the robot is standing."
              << std::endl;

    SendStop(sport_client, stop_sec, rate_hz, "Initial stop");
    for (const double speed : speeds) {
      SendMove(sport_client, mode, signed_direction * speed, duration_sec,
               rate_hz);
      SendStop(sport_client, stop_sec, rate_hz, "Step stop");
    }
    SendStop(sport_client, stop_sec, rate_hz, "Final stop");

    std::cout << "SDK2 speed sweep finished." << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }

  return 0;
}
