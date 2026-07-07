#include "inspection/arm_controller.hpp"
#include "inspection/config.hpp"
#include "inspection/logger.hpp"
#include "inspection/mission_fsm.hpp"
#include "inspection/robot_motion.hpp"
#include "inspection/vision_pipeline.hpp"

#include <atomic>
#include <csignal>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

std::atomic_bool g_stop_requested{false};

void HandleSignal(int) {
  g_stop_requested.store(true);
}

void PrintUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [--config config/competition.conf] [--profile default|safe|attack]\n"
      << "\n"
      << "Simulation is the default backend. Press Ctrl+C or create a STOP file in the\n"
      << "working directory to trigger emergency stop.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "config/competition.conf";
  std::string profile = "default";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    }
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if (arg == "--profile" && i + 1 < argc) {
      profile = argv[++i];
      continue;
    }

    inspection::Log(inspection::LogLevel::Error, "Unknown argument: " + arg);
    PrintUsage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  try {
    auto config = inspection::LoadConfig(config_path);
    inspection::ApplyProfile(profile, config);

    auto motion = inspection::CreateSimRobotMotion();
    auto vision = inspection::CreateSimVisionPipeline(config);
    auto arm = inspection::CreateSimArmController();

    inspection::MissionFSM mission(
        config,
        *motion,
        *vision,
        *arm,
        [] {
          return g_stop_requested.load() || std::filesystem::exists("STOP");
        });

    const auto final_state = mission.Run();
    inspection::Log(inspection::LogLevel::Info,
                    "Mission finished with state: " + inspection::ToString(final_state));
    return final_state == inspection::MissionState::Completed ? 0 : 1;
  } catch (const std::exception& ex) {
    inspection::Log(inspection::LogLevel::Error, std::string("Fatal error: ") + ex.what());
    return 1;
  }
}
